//! Windows I/O adapters that bridge the sibling backend crates to the engine's hot-loop
//! traits ([`crate::hot::DeviceSource`] / [`crate::hot::VirtualPad`]).
//!
//! The hot loop is written generically over two engine-owned traits expressed in
//! [`hyperion_core`] value types ([`HotInput`] in / [`OutputState`] out). The concrete backends
//! speak their own vocabulary:
//!
//! * `hid-input`'s [`hid_input::DeviceSource`] fills an [`InputSample`] in place
//!   (`next_sample(&mut InputSample) -> Result<bool, _>`),
//! * `vgamepad-output`'s [`vgamepad_output::VirtualPad`] is a lifecycle trait
//!   (`plugin` / `wait_ready` / `update(&OutputFrame)` / `unplug`).
//!
//! These adapters own one concrete backend each and implement the engine trait, doing the
//! per-report `InputSample → HotInput` translation (carrying the raw DS button word for the
//! mapping engine's structured decode, plus the DS→`PadButtons` decode lowered via the core
//! `pack_xinput` for the all-passthrough fast path), the `OutputState → OutputFrame` projection
//! into the X360 backend (the single i16 round stays in `vgamepad-output`), the KBM injector
//! thread (drains the egress ring → `SendInput`), and the lifecycle wiring (HidHide cloak, ViGEm
//! plug/wait). Everything here is `cfg(windows)` — the whole module is only compiled in via the
//! Windows-gated `supervisor`/`hot` modules.

use std::thread::JoinHandle;
use std::time::Duration;

use hyperion_core::config::{EngineConfig, WaitMode as CfgWaitMode};
use hyperion_core::input::{InputSample, SourceMeta};
use hyperion_core::output::{OutputState, PadTarget};
use hyperion_core::stick::StickSample;

use hid_input::win::hid::WaitMode as HidWaitMode;
use hid_input::{DeviceSource as HidDeviceSource, DualSenseUsbSource, SourceError, TouchEdge};
use platform_win::hidhide::{HidHide, HidHideBackend};
use platform_win::sched::{HotThreadConfig, WaitMode as SchedWaitMode};
// `DynPad` (blueprint §6.3) is the static-dispatch X360-or-DS4 pad; its variant is chosen from the
// active profile's `PadTarget` at (re)plug time via `DynPad::for_target`. It implements the
// `vgamepad_output::VirtualPad` trait (`plugin/wait_ready/update/unplug`, brought into scope as
// `VgVirtualPad`) which dispatches to the X360 / DS4 backend — no `dyn` on the hot path.
use vgamepad_output::{DynPad, OutErr, VirtualPad as VgVirtualPad};

use crate::handoff::KbmRx;
use crate::hot::{DeviceSource, HotInput, VirtualPad};

/// How long the KBM injector sleeps between drains when the ring is empty (no busy-spin off the
/// hot thread). Short enough that key edges feel immediate, long enough to idle near-zero CPU.
const KBM_IDLE_POLL: Duration = Duration::from_millis(1);

/// How long to wait for the OS to enumerate the freshly-plugged virtual pad before giving up.
const VIGEM_READY_TIMEOUT: Duration = Duration::from_secs(2);

/// Build the [`HotThreadConfig`] for `platform-win` from the engine config's thread section.
///
/// Maps the serde [`CfgWaitMode`] (+ spin budget) onto the scheduler's own [`SchedWaitMode`],
/// and forwards core-pinning / MMCSS knobs verbatim. Pure value copy — safe to call off-thread.
pub(crate) fn hot_thread_config(cfg: &EngineConfig) -> HotThreadConfig {
    let t = &cfg.thread;
    let wait_mode = match t.wait_mode {
        // A zero spin budget collapses HybridSpin to a plain blocking wait.
        CfgWaitMode::HybridSpin if t.spin_budget_us > 0 => SchedWaitMode::HybridSpin,
        _ => SchedWaitMode::Blocking,
    };
    HotThreadConfig {
        hot_core: t.hot_core,
        use_mmcss: t.use_mmcss,
        mmcss_task: t.mmcss_task.clone(),
        wait_mode,
    }
}

/// Translate the engine config's [`CfgWaitMode`] (+ spin budget) into the HID reader's
/// [`HidWaitMode`].
fn hid_wait_mode(cfg: &EngineConfig) -> HidWaitMode {
    let t = &cfg.thread;
    match t.wait_mode {
        CfgWaitMode::HybridSpin if t.spin_budget_us > 0 => HidWaitMode::HybridSpin {
            spin_budget_us: t.spin_budget_us,
        },
        _ => HidWaitMode::Blocking,
    }
}

/// The engine-facing device: a [`DualSenseUsbSource`] plus the resident [`InputSample`] it
/// decodes into, the HidHide cloak held for the device's life, and prime tracking.
///
/// Owns the HidHide guard so the physical pad stays cloaked exactly as long as we hold the
/// device open; dropping the device tears the cloak down (the pad reappears for other apps).
pub(crate) struct DualSenseDevice {
    source: DualSenseUsbSource,
    /// Reused decode target — the backend fills this in place, so steady state never allocates.
    sample: InputSample,
    /// HidHide cloak, held for the device's whole life. `None` when cloaking is disabled in
    /// config. Kept alive (never read) purely for its `Drop` (un-cloak on teardown).
    _cloak: Option<HidHide>,
    /// `true` until the first fresh report is delivered, so the hot filter primes once.
    first_report: bool,
}

impl DualSenseDevice {
    /// Enumerate, open, and cloak the primary DualSense USB device.
    ///
    /// Lifecycle (DESIGN §8): enumerate → open the overlapped reader → (if enabled) bring up
    /// HidHide (whitelist *self* first, then blacklist the physical instance path, then cloak
    /// on). Returns `None` when no device is present or any platform step fails — the supervisor
    /// treats that as a clean headless exit (`StopReason::DeviceLost`), not a panic.
    pub(crate) fn open_cloaked(cfg: &EngineConfig) -> Option<Self> {
        let id = DualSenseUsbSource::enumerate().into_iter().next()?;

        // Cloak the physical pad *before* we start reading it, so no other app grabs it first.
        // Whitelisting ourselves is mandatory — otherwise we hide the pad from ourselves too.
        let cloak = if cfg.hidhide.enabled {
            let backend = if cfg.hidhide.use_cli {
                HidHideBackend::Cli {
                    cli_path: cfg.hidhide.cli_path.clone(),
                }
            } else {
                HidHideBackend::Ioctl
            };
            let mut hh = HidHide::open(backend).map_err(log_hidhide_err).ok()?;
            hh.whitelist_self().map_err(log_hidhide_err).ok()?;
            hh.blacklist_device(&id.path)
                .map_err(log_hidhide_err)
                .ok()?;
            hh.set_active(true).map_err(log_hidhide_err).ok()?;
            Some(hh)
        } else {
            None
        };

        let meta = SourceMeta {
            vid: id.vid,
            pid: id.pid,
            name: "DualSense (USB)",
            stick_bits: 8,
            // DualSense Edge (PID 0x0DF2) exposes the Fn/paddle superset; gate its decode.
            is_edge: id.pid == 0x0DF2,
        };
        let source = DualSenseUsbSource::open(id, hid_wait_mode(cfg), meta)
            .map_err(log_source_err)
            .ok()?;

        Some(Self {
            source,
            sample: InputSample::default(),
            _cloak: cloak,
            first_report: true,
        })
    }
}

/// Surface a HidHide setup error to stderr (cold path only).
fn log_hidhide_err(e: std::io::Error) {
    eprintln!("hyperion: HidHide setup failed: {e}");
}

/// Surface a HID open error to stderr (cold path only).
fn log_source_err(e: SourceError) {
    eprintln!("hyperion: HID device open failed: {e}");
}

impl DeviceSource for DualSenseDevice {
    type Error = SourceError;

    fn next(&mut self) -> Result<Option<HotInput>, Self::Error> {
        // The backend re-arms the *other* overlapped buffer and parses the just-completed one,
        // filling `self.sample` in place. `Ok(false)` is a benign timeout (no fresh report).
        if !self.source.next_sample(&mut self.sample)? {
            return Ok(None);
        }
        let is_prime = self.first_report;
        self.first_report = false;
        // M7: pair the stick-only `InputSample` with the touch contacts + Edge bits the backend
        // decoded from the SAME report (the trait surfaces them separately because `InputSample`
        // is the device-agnostic core value). A stick-only backend returns the inert default.
        let touch_edge = self.source.touch_edge();
        Ok(Some(hot_input_from_sample(
            &self.sample,
            &touch_edge,
            is_prime,
        )))
    }
}

/// Convert a decoded [`InputSample`] into the engine's [`HotInput`].
///
/// Sticks/triggers are already canonical (`[-1,1]` / `[0,1]`) in core units; this re-homes them
/// into the engine value type, packs the opaque DS button bytes into the Xbox-360 bitfield for
/// the all-passthrough fast path, **and carries the raw DS button word through verbatim**
/// (`raw_buttons = s.buttons.0`, blueprint §9) so the mapping engine's `ControllerState` decode
/// reads the structured buttons with zero new device-side decode. `dt_us` / `dropped` /
/// `is_duplicate` / `host_qpc_ns` carry through (the backend already folded them).
///
/// **M7 touch / Edge (now wired).** [`HotInput`] carries the two decoded touchpad finger contacts
/// (`touch`) and the Edge button superset (`edge`); `core` decodes both in
/// [`decode_controller_state`](hyperion_core::input::ds_report::decode_controller_state). The
/// `hid-input` `DeviceSource` trait fills the stick-only [`InputSample`] AND surfaces the paired
/// [`TouchEdge`] (`source.touch_edge()`), so this function now copies `te.touch` and the Edge bits
/// straight across instead of leaving them inert. A stick-only backend (XInput / the raw-HID
/// skeleton) returns the inert [`TouchEdge::default`], so its `HotInput` stays byte-identical to M5
/// (untouched pad / all Edge bits `false`). The whole `hot.rs` → `apply()` path already consumes
/// them, so touchpad-as-mouse, the `TouchLeft/Right/Upper/Multi` region controls, and the Edge
/// `Fn/paddle/Mute/Capture/side` controls are live end-to-end.
#[inline]
fn hot_input_from_sample(s: &InputSample, te: &TouchEdge, is_prime: bool) -> HotInput {
    HotInput {
        left: StickSample::new(s.left.x, s.left.y),
        right: StickSample::new(s.right.x, s.right.y),
        lt: s.l2,
        rt: s.r2,
        // The fast-path XInput word is the structured DS→PadButtons decode lowered via the core
        // `pack_xinput` (the single source of truth for the PadButtons→XInput bit layout, §9).
        buttons: hyperion_core::output::pack_xinput(ds_buttons_to_pad(s.buttons.0)),
        raw_buttons: s.buttons.0,
        // M7: the backend-decoded touchpad contacts + Edge superset, plugged straight into the
        // structured `ControllerState` the mapping engine reads (`controller_state_from`). Both
        // default inert, so a non-touch / non-Edge source is byte-identical to M5.
        touch: te.touch,
        edge: edge_buttons_from(&te.edge),
        dt_us: s.dt_us,
        is_prime,
        dropped: s.dropped,
        is_duplicate: s.is_duplicate,
        host_qpc_ns: s.host_qpc_ns,
    }
}

/// Re-home the `hid-input` [`EdgeButtons`](hid_input::EdgeButtons) (the backend's Edge bit bundle)
/// into the engine's [`EdgeButtons`](crate::hot::EdgeButtons) value the [`HotInput`] carries.
///
/// Two structurally-identical flat bundles of bools live in two crates so neither takes a dependency
/// on the other's type; this is the one-place field-for-field copy. Both default to all-`false`
/// (inert non-Edge), so a stick-only / non-Edge source stays byte-identical to M5.
#[inline]
fn edge_buttons_from(e: &hid_input::EdgeButtons) -> crate::hot::EdgeButtons {
    crate::hot::EdgeButtons {
        mute: e.mute,
        capture: e.capture,
        fn_l: e.fn_l,
        fn_r: e.fn_r,
        blp: e.blp,
        brp: e.brp,
        side_l: e.side_l,
        side_r: e.side_r,
    }
}

/// Decode the three raw DualSense button bytes (packed `btn0|btn1<<8|btn2<<16` by the HID
/// backend) into the structured [`PadButtons`](hyperion_core::output::PadButtons) set.
///
/// This is the device-specific half (raw DS bytes → target-agnostic buttons); the
/// target-agnostic half (`PadButtons` → XInput / DS4 wire bits) lives in core
/// ([`pack_xinput`](hyperion_core::output::pack_xinput) / the DS4 lowering), so there is exactly
/// one button-bit-layout authority shared with the mapping engine and the DS4 backend.
///
/// Layout follows the DS4Windows / `DS4Device.cs` ground truth (mirrors core's
/// `decode_controller_state` button map, blueprint §3.5):
/// * `btn0` (byte 5): low nibble = D-pad hat (`0..7`, `8` = released); high nibble = face
///   buttons Square `0x10` / Cross `0x20` / Circle `0x40` / Triangle `0x80`.
/// * `btn1` (byte 6): L1 `0x01`, R1 `0x02`, L2-click `0x04`, R2-click `0x08`, Share `0x10`,
///   Options `0x20`, L3 `0x40`, R3 `0x80`.
/// * `btn2` (byte 7): PS `0x01`, Touchpad-click `0x02` (upper bits are the frame counter).
#[inline]
fn ds_buttons_to_pad(raw: u32) -> hyperion_core::output::PadButtons {
    use hyperion_core::output::PadButtons as P;
    let btn0 = (raw & 0xFF) as u8;
    let btn1 = ((raw >> 8) & 0xFF) as u8;
    let btn2 = ((raw >> 16) & 0xFF) as u8;

    let mut out = P::default();

    // D-pad hat (low nibble of btn0): 0=N,1=NE,2=E,3=SE,4=S,5=SW,6=W,7=NW, 8=released.
    let hat = btn0 & 0x0F;
    let (up, right, down, left) = match hat {
        0 => (true, false, false, false),
        1 => (true, true, false, false),
        2 => (false, true, false, false),
        3 => (false, true, true, false),
        4 => (false, false, true, false),
        5 => (false, false, true, true),
        6 => (false, false, false, true),
        7 => (true, false, false, true),
        _ => (false, false, false, false),
    };
    out.set(P::DPAD_UP, up);
    out.set(P::DPAD_DOWN, down);
    out.set(P::DPAD_LEFT, left);
    out.set(P::DPAD_RIGHT, right);

    // Face buttons (high nibble of btn0), cross layout.
    out.set(P::X, btn0 & 0x10 != 0); // Square
    out.set(P::A, btn0 & 0x20 != 0); // Cross
    out.set(P::B, btn0 & 0x40 != 0); // Circle
    out.set(P::Y, btn0 & 0x80 != 0); // Triangle

    // Shoulders / stick clicks / meta (btn1).
    out.set(P::LB, btn1 & 0x01 != 0); // L1
    out.set(P::RB, btn1 & 0x02 != 0); // R1
    out.set(P::BACK, btn1 & 0x10 != 0); // Share -> Back/View
    out.set(P::START, btn1 & 0x20 != 0); // Options -> Start/Menu
    out.set(P::LS, btn1 & 0x40 != 0); // L3
    out.set(P::RS, btn1 & 0x80 != 0); // R3

    // PS button -> Guide (btn2).
    out.set(P::GUIDE, btn2 & 0x01 != 0);

    out
}

/// Read the active device's [`PadTarget`] (virtual-pad output kind) from a config snapshot.
///
/// Resolves `active_device -> assigned profile -> output_kind`, defaulting to
/// [`PadTarget::X360`](hyperion_core::output::PadTarget) (the M2 default) when the device has no
/// assignment / the profile is absent — so a fresh or mis-configured config plugs the byte-identical
/// X360 pad. Read **only at (re)plug time** (blueprint §6.3 / §7.4), never per report.
pub(crate) fn active_output_kind(cfg: &EngineConfig) -> PadTarget {
    cfg.assignments
        .get(&cfg.active_device)
        .and_then(|pid| cfg.profiles.get(pid))
        .map(|p| p.output_kind)
        .unwrap_or_default()
}

/// The engine-facing virtual pad: a plugged-in [`DynPad`] whose kind (X360 / DS4) was chosen from
/// the active profile's [`PadTarget`](hyperion_core::output::PadTarget) at plug time (blueprint
/// §6.3).
///
/// Construction plugs the target into ViGEmBus and waits for OS enumeration, so by the time the hot
/// loop holds one, `update` is a single bounded IOCTL via static dispatch (no vtable on the hot
/// path — `DynPad` is an enum, not a `dyn` trait object). `Drop` (via `DynPad`) unplugs. A runtime
/// output-kind change is a full unplug → replug of a fresh `DynPad` of the new kind ([`replug`],
/// driven by `HotCommand::ReplugTarget`), since ViGEm cannot morph a plugged target's type.
///
/// [`replug`]: DynPadTarget::replug
pub(crate) struct DynPadTarget {
    pad: DynPad,
    /// The kind currently plugged, so a `ReplugTarget` that does not actually change the kind can
    /// short-circuit (no needless disconnect/reconnect the game would see).
    kind: PadTarget,
}

impl DynPadTarget {
    /// Create the target for `kind`, plug it into ViGEmBus, and wait for the OS to enumerate it.
    ///
    /// Returns `None` if the ViGEmBus driver is unavailable or enumeration times out — the
    /// supervisor maps that to a clean headless exit rather than a panic. The default `kind`
    /// ([`PadTarget::X360`](hyperion_core::output::PadTarget)) reproduces the M2 X360 egress exactly.
    pub(crate) fn plugged(kind: PadTarget) -> Option<Self> {
        let mut pad = DynPad::for_target(kind);
        pad.plugin().map_err(log_out_err).ok()?;
        pad.wait_ready(VIGEM_READY_TIMEOUT)
            .map_err(log_out_err)
            .ok()?;
        Some(Self { pad, kind })
    }
}

impl VirtualPad for DynPadTarget {
    type Error = OutErr;

    /// Submit the structured [`OutputState`] to whichever virtual pad is plugged. `DynPad` performs
    /// the single i16/u8 (X360) or u8 (DS4) round inside the backend (`to_xinput_*` / `to_ds4_axis`),
    /// so the no-mid-chain-quantization invariant is intact and the X360 path stays byte-identical to
    /// M2 (the X360 arm still goes through `OutputState::to_output_frame` → `to_xinput_*`).
    #[inline]
    fn update(&mut self, state: &OutputState) -> Result<(), Self::Error> {
        self.pad.update(state)
    }

    /// Replug as a different output kind (blueprint §6.3): unplug the current `DynPad`, build + plug
    /// a fresh one of `kind`, and wait for enumeration. A no-op (just `Ok`) when `kind` already
    /// matches the plugged kind, so a `ReplugTarget` that does not change the output kind costs the
    /// game no disconnect/reconnect. Runs on the hot thread (the ViGEm handle is thread-affine), but
    /// only on the `ReplugTarget` command edge — never per report.
    fn replug(&mut self, kind: PadTarget) -> Result<(), Self::Error> {
        if kind == self.kind {
            return Ok(());
        }
        // Drop the old target first (ViGEm cannot morph a plugged target's type — the game sees a
        // disconnect), then plug + enumerate the new kind.
        self.pad.unplug();
        let mut pad = DynPad::for_target(kind);
        pad.plugin()?;
        pad.wait_ready(VIGEM_READY_TIMEOUT)?;
        self.pad = pad;
        self.kind = kind;
        Ok(())
    }
}

/// The injector's macro-table feed (M7): the resolved active profile's `Arc<[MacroDef]>`, sent from
/// the supervisor's control-plane drain thread (which receives `ControlPlaneEvent::Macros` off the
/// hot loop) to the KBM injector's `MacroPlayer`. A bounded `crossbeam` channel — a missed publish
/// re-sends on the next generation gate, so the injector's `MacroPlayer` always converges on the
/// active set.
pub(crate) type MacroTable = std::sync::Arc<[hyperion_core::map::MacroDef]>;
/// Receiving end of the [`MacroTable`] feed, owned by the KBM injector thread.
pub(crate) type MacroTableRx = crossbeam_channel::Receiver<MacroTable>;
/// Sending end of the [`MacroTable`] feed, owned by the supervisor's control-plane drain thread.
pub(crate) type MacroTableTx = crossbeam_channel::Sender<MacroTable>;

/// Spawn the **KBM injector** thread (blueprint §7.3): a normal-priority worker that drains the
/// hot loop's [`KbmRx`] egress ring and realizes each [`KbmBatch`](hyperion_core::output::KbmBatch)
/// via one batched `SendInput`. Macro playback timing (the unbounded part) lives here, never on
/// the hot thread.
///
/// **M7 macro wiring.** The thread owns a `MacroPlayer`: it installs the active profile's macro
/// table whenever one arrives on `macro_rx` (sourced from `ControlPlaneEvent::Macros` via the
/// supervisor), routes every drained `KbmEvent::Macro` edge to `MacroPlayer::on_edge`, and ticks
/// the player (`MacroPlayer::tick`) on each wake so in-flight macros advance their step schedule.
/// The player's returned next-deadline bounds the sleep (so a parked macro wakes exactly when due,
/// never busy-spins), clamped to [`KBM_IDLE_POLL`] so the producer/abandon checks still run promptly.
///
/// The thread exits cleanly when the producer (the hot thread's `KbmTx`) is dropped and the ring
/// is drained ([`KbmRx::is_abandoned`]); on shutdown it releases any keys it is holding (including
/// macro-held keys, via `MacroPlayer::stop_all` then `release_all`) so a crash/stop never leaves a
/// key stuck down. A profile swap installs a fresh table, which stops any in-flight macro cleanly.
///
/// The Win32 `SendInput` body lives in the `kbm-output` crate's `SendInputKbm` (`KbmSink`); this
/// is the only place the engine touches it, and it is `cfg(windows)` so the pure-core Linux CI is
/// unaffected.
pub(crate) fn spawn_kbm_injector(
    mut kbm_rx: KbmRx,
    macro_rx: MacroTableRx,
) -> std::io::Result<JoinHandle<()>> {
    use hyperion_core::output::KbmEvent;
    use kbm_output::{KbmSink, MacroPlayer, SendInputKbm};
    use std::time::Instant;

    std::thread::Builder::new()
        .name("hyperion-kbm-injector".to_string())
        .spawn(move || {
            let mut sink = SendInputKbm::new();
            let mut player = MacroPlayer::new();
            loop {
                // (a) Install the latest macro table if the supervisor published one (profile swap
                // / start). Drain the channel so only the most recent table sticks; `set_macros`
                // stops any in-flight macro cleanly against the OLD defs first (no stuck keys).
                let mut latest_table = None;
                while let Ok(table) = macro_rx.try_recv() {
                    latest_table = Some(table);
                }
                if let Some(table) = latest_table {
                    player.set_macros(&mut sink, table.iter().cloned());
                }

                // (b) Drain the egress ring, flushing each batch in one SendInput and routing any
                // `Macro` start/stop edges to the player (the sink itself stages nothing for them).
                let mut drained_any = false;
                while let Some(batch) = kbm_rx.pop() {
                    drained_any = true;
                    for &ev in batch.as_slice() {
                        if let KbmEvent::Macro { id, start } = ev {
                            player.on_edge(&mut sink, id, start);
                        }
                    }
                    if let Err(e) = sink.flush(&batch) {
                        log_kbm_err(&e);
                    }
                }

                // (c) Advance in-flight macros; the returned soonest deadline bounds our sleep.
                let now = Instant::now();
                let next_deadline = player.tick(&mut sink, now);

                // (d) Producer gone and ring drained: stop macros (release their held keys), release
                // everything else, and exit cleanly.
                if kbm_rx.is_abandoned() {
                    player.stop_all(&mut sink);
                    let _ = sink.release_all();
                    return;
                }

                // (e) Idle: if we drained nothing this pass, sleep until the soonest macro deadline
                // (capped at KBM_IDLE_POLL so the abandon/producer checks stay responsive). With no
                // macro running the deadline is `None` -> the full idle poll. A runnable-now macro
                // reports `now` -> a zero sleep, so we re-tick it at once; the player parks every
                // running macro on a FUTURE wait within that one tick, so the next pass sleeps (no
                // busy-spin). The zero-sleep call is skipped so a runnable macro loops immediately.
                if !drained_any {
                    let sleep = match next_deadline {
                        Some(t) => t
                            .saturating_duration_since(Instant::now())
                            .min(KBM_IDLE_POLL),
                        None => KBM_IDLE_POLL,
                    };
                    if !sleep.is_zero() {
                        std::thread::sleep(sleep);
                    }
                }
            }
        })
}

/// Surface a KBM injection error to stderr (cold path only — a `SendInput` failure is rare and
/// non-fatal; the next report's edges retry).
fn log_kbm_err<E: std::fmt::Display>(e: &E) {
    eprintln!("hyperion: KBM injection failed: {e}");
}

/// Surface an [`OutErr`] to stderr (cold setup path only — plug / wait_ready, never the hot
/// loop). Returned from `map_err` so the caller can `.ok()?` the result.
fn log_out_err(e: OutErr) {
    eprintln!("hyperion: virtual pad setup failed: {e}");
}

#[cfg(test)]
mod tests {
    use super::*;
    use hyperion_core::output::{pack_xinput, PadButtons};

    // XInput button bits, for asserting the fast-path word matches the former ds_buttons_to_xinput.
    const XI_DPAD_UP: u16 = 0x0001;
    const XI_BACK: u16 = 0x0020;
    const XI_LTHUMB: u16 = 0x0040;
    const XI_LSHOULDER: u16 = 0x0100;
    const XI_GUIDE: u16 = 0x0400;
    const XI_A: u16 = 0x1000;
    const XI_X: u16 = 0x4000;

    #[test]
    fn ds_buttons_to_pad_decodes_face_and_meta() {
        // btn0: hat=8 (neutral) + Cross (0x20); btn1: L1 (0x01); btn2: PS (0x01).
        let raw = 0x28u32 | (0x01u32 << 8) | (0x01u32 << 16);
        let pad = ds_buttons_to_pad(raw);
        assert!(pad.has(PadButtons::A), "Cross -> A");
        assert!(pad.has(PadButtons::LB), "L1 -> LB");
        assert!(pad.has(PadButtons::GUIDE), "PS -> Guide");
        assert!(!pad.has(PadButtons::B));
        // Lowered to XInput, matches the legacy ds_buttons_to_xinput output.
        assert_eq!(pack_xinput(pad), XI_A | XI_LSHOULDER | XI_GUIDE);
    }

    #[test]
    fn ds_buttons_to_pad_decodes_dpad_hat() {
        // hat=0 -> North (up only).
        let pad = ds_buttons_to_pad(0x00);
        assert!(pad.has(PadButtons::DPAD_UP));
        assert!(!pad.has(PadButtons::DPAD_DOWN));
        assert_eq!(pack_xinput(pad) & XI_DPAD_UP, XI_DPAD_UP);
        // hat=8 -> neutral (no dpad).
        let neutral = ds_buttons_to_pad(0x08);
        assert!(!neutral.has(PadButtons::DPAD_UP));
    }

    #[test]
    fn ds_buttons_to_pad_share_options_and_thumbs() {
        // btn1: Share (0x10) + L3 (0x40); btn0: Square (0x10) + hat neutral (0x08).
        let raw = (0x18u32) | ((0x10u32 | 0x40u32) << 8);
        let pad = ds_buttons_to_pad(raw);
        assert!(pad.has(PadButtons::X), "Square -> X");
        assert!(pad.has(PadButtons::BACK), "Share -> Back");
        assert!(pad.has(PadButtons::LS), "L3 -> LS");
        assert_eq!(pack_xinput(pad), XI_X | XI_BACK | XI_LTHUMB);
    }

    #[test]
    fn hot_input_carries_raw_buttons_and_packed_word() {
        let s = InputSample {
            buttons: hyperion_core::input::Buttons(0x20 | 0x08), // Cross + neutral hat
            ..InputSample::default()
        };
        let hi = hot_input_from_sample(&s, &TouchEdge::default(), false);
        assert_eq!(hi.raw_buttons, 0x20 | 0x08, "raw DS word carried through");
        assert_eq!(
            hi.buttons, XI_A,
            "fast-path XInput word packed from PadButtons"
        );
    }

    #[test]
    fn hot_input_default_touch_edge_is_inert() {
        // A stick-only backend (or any report with no touch / non-Edge source) returns the inert
        // `TouchEdge::default`, which must produce an untouched pad + all-false Edge bits on the
        // `HotInput` — byte-identical to the pre-M7 inert path.
        let hi = hot_input_from_sample(&InputSample::default(), &TouchEdge::default(), false);
        assert_eq!(
            hi.touch,
            [hyperion_core::input::TouchContact::default(); 2],
            "no contacts surface from the inert default"
        );
        assert_eq!(
            hi.edge,
            crate::hot::EdgeButtons::default(),
            "all Edge bits false"
        );
    }

    #[test]
    fn hot_input_carries_touch_contacts_and_edge_bits() {
        // The #1 M7 wire: a backend-surfaced `TouchEdge` (active contact + a couple of Edge bits)
        // must reach `HotInput.touch` / `HotInput.edge` verbatim, so `hot.rs`'s
        // `controller_state_from` (and thus `apply()`'s touch path + Edge controls) see them.
        let te = TouchEdge {
            touch: [
                hyperion_core::input::TouchContact {
                    is_active: true,
                    id: 9,
                    x: 256,
                    y: 64,
                },
                hyperion_core::input::TouchContact::default(),
            ],
            edge: hid_input::EdgeButtons {
                mute: true,
                fn_r: true,
                side_l: true,
                ..hid_input::EdgeButtons::default()
            },
        };
        let hi = hot_input_from_sample(&InputSample::default(), &te, false);
        // Touch contact carried through unchanged.
        assert!(hi.touch[0].is_active && hi.touch[0].id == 9);
        assert_eq!((hi.touch[0].x, hi.touch[0].y), (256, 64));
        assert!(!hi.touch[1].is_active);
        // Edge bits map field-for-field into the engine's EdgeButtons.
        assert!(hi.edge.mute && hi.edge.fn_r && hi.edge.side_l);
        assert!(
            !hi.edge.capture && !hi.edge.fn_l && !hi.edge.blp && !hi.edge.brp && !hi.edge.side_r
        );
        // `hot::controller_state_from` (a sibling-module consumer, covered by hot.rs's own touch
        // test) then reads these into the touch-region + Edge controls — so this `HotInput` is the
        // complete remaining wire.
    }

    // ---------------------------------- M5: output-kind resolution -------------------------------

    use hyperion_core::map::Profile;
    use std::sync::Arc;

    fn cfg_with_kind(kind: PadTarget) -> EngineConfig {
        let mut cfg = EngineConfig {
            active_device: "dev".to_string(),
            ..EngineConfig::default()
        };
        Arc::make_mut(&mut cfg.profiles).insert(
            "p".to_string(),
            Profile {
                name: "p".to_string(),
                output_kind: kind,
                ..Profile::default()
            },
        );
        cfg.assignments.insert("dev".to_string(), "p".to_string());
        cfg
    }

    #[test]
    fn active_output_kind_reads_assigned_profile() {
        assert_eq!(
            active_output_kind(&cfg_with_kind(PadTarget::Ds4)),
            PadTarget::Ds4
        );
        assert_eq!(
            active_output_kind(&cfg_with_kind(PadTarget::X360)),
            PadTarget::X360
        );
    }

    #[test]
    fn active_output_kind_defaults_to_x360_when_unassigned() {
        // No assignment / no profile -> the byte-identical M2 X360 default.
        let cfg = EngineConfig {
            active_device: "dev".to_string(),
            ..EngineConfig::default()
        };
        assert_eq!(active_output_kind(&cfg), PadTarget::X360);
        assert_eq!(
            PadTarget::default(),
            PadTarget::X360,
            "default kind is X360"
        );
    }
}
