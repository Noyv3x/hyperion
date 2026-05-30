//! The hot loop (Windows only). Owns one HID device, runs the `core` stick/trigger pipeline and
//! the mapping engine in place, and submits to the virtual pad inline — zero-alloc, lock-free
//! steady state (`DESIGN.md` §6, blueprint §7.2).
//!
//! # Wiring
//! The **control flow and the types are real**; the actual blocking HID read and the ViGEm
//! IOCTL are driven through the [`DeviceSource`] / [`VirtualPad`] traits below. On Windows the
//! [`crate::win_io`] adapters implement these traits over the `hid-input` `DualSenseUsbSource`
//! and `vgamepad-output` backends; MMCSS / affinity policy binding and HidHide live in the
//! supervisor.
//!
//! The engine is written **generically over two traits** expressed entirely in terms of
//! [`hyperion_core`] value types ([`HotInput`], [`hyperion_core::output::OutputState`]). That
//! keeps the hot loop type-checked on `windows-latest` independently of the sibling crates'
//! concrete struct names, and confines the device-report decode to the backend.
//!
//! Loop order (per §6 / §7.2, zero-alloc steady state):
//! drain gui+sup command queues → blocking read → `cfg.load()` + cheap generation check (resolve
//! the [`ResolvedProfile`] + per-stick/trigger settings ONCE on the gate, NOT per report) →
//! (prime or) `process_stick` / `process_trigger` per axis → build [`ControllerState`] →
//! `map::apply` → project [`OutputState`] to [`OutputFrame`] for the X360 pad → push the
//! [`KbmBatch`] to the injector ring (drop-on-full, never blocks) → triple-buffer telemetry write.
//!
//! ## M3 non-regression
//! With an all-`Passthrough` profile (the default), the **stick-only fast path** builds the
//! `OutputFrame` directly from the filtered sticks + processed triggers + the already-packed
//! XInput button word, byte-identical to the M2 pre-mapper baseline (blueprint §13 conflict 1):
//! `process_stick` with only RC active is bit-exact to the old `RcFilter` step, `process_trigger`
//! with default settings is an analog passthrough, and an all-passthrough profile never queues a
//! KBM edge. The general `map::apply` path runs only when the profile actually remaps something.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use hyperion_core::config::EngineConfig;
use hyperion_core::dt::Dt;
use hyperion_core::input::{Control, ControllerState};
use hyperion_core::map::{apply, BindTarget, MapState, ResolvedProfile};
use hyperion_core::output::OutputState;
use hyperion_core::stick::settings::{StickSettings, StickState};
use hyperion_core::stick::StickSample;
use hyperion_core::trigger::{process_trigger, TriggerSettings, TriggerState};

use crate::clock::DtTracker;
use crate::handoff::{CommandRx, ConfigHandle, HotCommand, KbmTx, TelemetryTx};
use crate::telemetry::{LatencyReservoir, TelemetryFrame};

/// One decoded input report, in the canonical units the pipeline consumes. The `hid-input`
/// backend decodes the raw HID buffer (via `core`'s parser) into this; the engine never touches
/// device bytes. Expressed over plain values so the engine stays decoupled from the backend.
#[derive(Clone, Copy, Debug)]
pub struct HotInput {
    /// Left / right stick, canonical `[-1,1]`.
    pub left: StickSample,
    pub right: StickSample,
    /// Triggers, `[0,1]`.
    pub lt: f64,
    pub rt: f64,
    /// Mapped Xbox-360 button bitfield (the M2 packing, used by the all-passthrough fast path).
    pub buttons: u16,
    /// Raw DualSense button bytes `btn0 | btn1<<8 | btn2<<16` (blueprint §9): the structured
    /// [`ControllerState`] decode the mapping engine reads. Zero new decode — the backend packs
    /// the same three bytes it already produced for `buttons`.
    pub raw_buttons: u32,
    /// Guarded real elapsed time since the previous report (microseconds).
    pub dt_us: f64,
    /// `true` for the first report after enable/reset (pipeline primes, no step).
    pub is_prime: bool,
    /// Reports dropped before this one (seq gap), and whether this report duplicates the last.
    pub dropped: u16,
    pub is_duplicate: bool,
    /// Host monotonic timestamp at read completion (ns), for scope correlation.
    pub host_qpc_ns: u64,
}

impl Default for HotInput {
    /// A neutral, primed report (sticks centered, triggers released). `StickSample` itself has
    /// no `Default`, so this is implemented by hand from [`StickSample::NEUTRAL`].
    fn default() -> Self {
        Self {
            left: StickSample::NEUTRAL,
            right: StickSample::NEUTRAL,
            lt: 0.0,
            rt: 0.0,
            buttons: 0,
            raw_buttons: 0,
            dt_us: 0.0,
            is_prime: true,
            dropped: 0,
            is_duplicate: false,
            host_qpc_ns: 0,
        }
    }
}

/// A blocking source of decoded input reports. Implemented by the `hid-input` backends
/// (`DualSenseUsbSource`, …) on Windows.
pub trait DeviceSource: Send {
    /// Error type for a read.
    type Error;

    /// Block until the next report is available. `Ok(Some(_))` = a fresh decoded report,
    /// `Ok(None)` = a benign timeout (retry), `Err` = device loss.
    fn next(&mut self) -> Result<Option<HotInput>, Self::Error>;
}

/// A virtual gamepad target. Implemented by `vgamepad-output`'s pad on Windows.
///
/// Takes the structured [`OutputState`] (blueprint §6.3): the backend projects it to its wire
/// format (X360 via [`OutputState::to_output_frame`], DS4 via the core lowering) and performs the
/// single i16/u8 round there, so the no-mid-chain-quantization invariant is preserved.
pub trait VirtualPad {
    /// Error type for a submit.
    type Error;

    /// Submit one frame via a single synchronous, bounded IOCTL (must never block the
    /// TIME_CRITICAL hot thread in the driver — §6).
    fn update(&mut self, state: &OutputState) -> Result<(), Self::Error>;
}

/// Why the hot loop stopped.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StopReason {
    /// A [`HotCommand::Shutdown`] was received.
    Shutdown,
    /// The device disconnected (read or submit returned `Err`).
    DeviceLost,
}

/// Everything the hot thread needs, assembled by [`crate::run`] and moved onto the thread.
pub struct HotThread<D, V> {
    device: D,
    target: V,
    config: ConfigHandle,
    /// Shared generation counter; bumped by [`crate::config_store::ConfigStore`].
    config_gen: Arc<AtomicU64>,
    commands: CommandRx,
    telemetry: TelemetryTx,
    /// KBM egress producer to the injector thread (drop-on-full, never blocks — §7.3).
    kbm_tx: KbmTx,
}

/// The resident, alloc-free working set held across reports.
///
/// Resolved ONCE on the generation gate (blueprint §7.2 / verifier latency FIX 1/2/10): the
/// active [`ResolvedProfile`] is held by `Arc` ref (one map get per gate, never per report) and
/// the small `Copy` stick/trigger settings are copied into resident arrays. The per-report path
/// touches ZERO maps / strings / hashes.
struct Resident {
    /// Per-stick pipeline state (`0` = LS, `1` = RS). `Default` is the clean post-reset state.
    stick_state: [StickState; 2],
    /// Per-trigger state (`0` = L2, `1` = R2).
    trig_state: [TriggerState; 2],
    /// Mapping-engine state (turbo/toggle/edge latches). `Default` is the clean post-reset state.
    map_state: MapState,
    /// The active resolved profile (Arc ref resolved on the generation gate).
    resolved: Arc<ResolvedProfile>,
    /// Resident per-stick settings copied off `resolved` on the gate (`0` = LS, `1` = RS).
    stick_settings: [StickSettings; 2],
    /// Resident per-trigger settings copied off `resolved` on the gate (`0` = L2, `1` = R2).
    trig_settings: [TriggerSettings; 2],
    /// Fast-path flag: the resolved profile is the trivial all-`Passthrough` map, so the hot loop
    /// builds the `OutputFrame` directly (byte-identical to M2) and skips `map::apply`.
    passthrough_only: bool,
}

impl Resident {
    /// Resolve the resident set from a fresh config snapshot. Called only on the generation gate.
    fn resolve(cfg: &EngineConfig) -> Self {
        let resolved = resolve_active(cfg);
        let stick_settings = [resolved.ls, resolved.rs];
        let trig_settings = [resolved.l2, resolved.r2];
        let passthrough_only = is_all_passthrough(&resolved);
        Self {
            stick_state: Default::default(),
            trig_state: Default::default(),
            map_state: MapState::default(),
            resolved,
            stick_settings,
            trig_settings,
            passthrough_only,
        }
    }

    /// Re-resolve only the profile-derived fields on a config change, preserving the live filter /
    /// mapping state (a config edit must not snap the sticks or drop a held key). Mirrors how M2
    /// kept the RC state across a generation bump.
    fn reresolve(&mut self, cfg: &EngineConfig) {
        self.resolved = resolve_active(cfg);
        self.stick_settings = [self.resolved.ls, self.resolved.rs];
        self.trig_settings = [self.resolved.l2, self.resolved.r2];
        self.passthrough_only = is_all_passthrough(&self.resolved);
    }

    /// Clear all per-report mutable state to its clean post-reset value (verifier FIX 6). The
    /// [`HotCommand::ResetFilter`] arm: a fresh enable / recalibrate / replug starts clean.
    fn reset(&mut self) {
        self.stick_state = Default::default();
        self.trig_state = Default::default();
        self.map_state = MapState::default();
    }

    /// Per-report prime path (engine `input.is_prime`), distinct from the command-driven full
    /// reset: clear the history that must be re-seeded from the first post-enable report while
    /// leaving the monotonic accumulators running, so the next `process_*` call re-primes.
    fn prime(&mut self) {
        self.stick_state[0].prime_reset();
        self.stick_state[1].prime_reset();
        // Trigger state has no priming history (the chain is stateless modulo edge tracking), but
        // clear the digital edge latch so a held trigger across a replug re-edges cleanly.
        self.trig_state = Default::default();
    }
}

impl<D, V> HotThread<D, V>
where
    D: DeviceSource,
    V: VirtualPad,
{
    /// Bundle the hot-thread inputs. Construction is off the hot path.
    pub fn new(
        device: D,
        target: V,
        config: ConfigHandle,
        config_gen: Arc<AtomicU64>,
        commands: CommandRx,
        telemetry: TelemetryTx,
        kbm_tx: KbmTx,
    ) -> Self {
        Self {
            device,
            target,
            config,
            config_gen,
            commands,
            telemetry,
            kbm_tx,
        }
    }

    /// Run the hot loop until shutdown or device loss. Blocks the calling (hot) thread.
    ///
    /// The MMCSS / affinity / priority RAII guard must be **bound** for the whole call by the
    /// caller (`let _policy = ...; thread.run();`) — see §6 (verifier (c)).
    pub fn run(mut self) -> StopReason {
        let mut clock = DtTracker::new();
        let mut busy = LatencyReservoir::new();
        let mut applied_gen: u64 = 0;
        let mut cfg: EngineConfig = (*self.config.load_full()).clone();
        let mut res = Resident::resolve(&cfg);
        let mut dropped_total: u32 = 0;
        let mut duplicates_total: u32 = 0;
        let mut kbm_dropped_total: u32 = 0;

        loop {
            // Drain BOTH command queues first so a Shutdown is honored promptly.
            while let Some(cmd) = self.commands.try_pop() {
                match cmd {
                    HotCommand::ResetFilter
                    | HotCommand::Recalibrate
                    | HotCommand::ReplugTarget => {
                        res.reset();
                        clock.reset();
                    }
                    HotCommand::Shutdown => return StopReason::Shutdown,
                }
            }

            // Blocking read of the next decoded report.
            let busy_start = clock.now_qpc_ns();
            let input = match self.device.next() {
                Ok(Some(i)) => i,
                Ok(None) => continue, // benign timeout
                Err(_) => return StopReason::DeviceLost,
            };

            // Refresh config only when the generation changed (one atomic load per report). Resolve
            // the profile + settings ONCE here, never per report (§7.2). A config edit re-resolves
            // the profile-derived fields but preserves the live filter / mapping state.
            let cur_gen = self.config_gen.load(Ordering::Acquire);
            if cur_gen != applied_gen {
                cfg = (*self.config.load_full()).clone();
                res.reresolve(&cfg);
                applied_gen = cur_gen;
            }

            let dt = Dt::guarded(input.dt_us);
            dropped_total = dropped_total.wrapping_add(input.dropped as u32);
            if input.is_duplicate {
                duplicates_total = duplicates_total.wrapping_add(1);
            }

            // A fresh post-enable report re-primes the per-stick history (RC, fuzz, snapback,
            // flick) — distinct from the command-driven full reset above.
            if input.is_prime {
                res.prime();
            }

            // --- Stick pipeline (per stick: full DS4Windows chain; RC is stage 0). -------------
            let filt_l = hyperion_core::stick::process_stick(
                input.left,
                &res.stick_settings[0],
                &mut res.stick_state[0],
                dt,
            );
            let filt_r = hyperion_core::stick::process_stick(
                input.right,
                &res.stick_settings[1],
                &mut res.stick_state[1],
                dt,
            );

            // --- Trigger pipeline (per trigger: returns processed analog + digital pressed). ----
            let (lt_analog, _l2_pressed) =
                process_trigger(input.lt, &res.trig_settings[0], &mut res.trig_state[0], dt);
            let (rt_analog, _r2_pressed) =
                process_trigger(input.rt, &res.trig_settings[1], &mut res.trig_state[1], dt);

            // --- Map + submit. -----------------------------------------------------------------
            let out_state = if res.passthrough_only {
                // Fast path (all-Passthrough profile): build the egress directly from the filtered
                // sticks + processed triggers + the already-packed XInput buttons. Byte-identical
                // to the M2 pre-mapper baseline (blueprint §13 conflict 1) — no ControllerState
                // build, no map::apply, no KBM. The fast path emits an `OutputState` carrying the
                // M2 XInput button word so the backend's single round is unchanged.
                OutputState {
                    lx: filt_l.x,
                    ly: filt_l.y,
                    rx: filt_r.x,
                    ry: filt_r.y,
                    lt: lt_analog,
                    rt: rt_analog,
                    buttons: xinput_to_pad_buttons(input.buttons),
                }
            } else {
                // General path: build the decoded ControllerState (filtered sticks + processed
                // triggers + decoded raw buttons) and run the pure mapping engine.
                let state = controller_state_from(&input, filt_l, filt_r, lt_analog, rt_analog);
                // `now_us` reuses the existing clock read (verifier latency FIX 3) — no 2nd QPC.
                let now_us = busy_start / 1000;
                let (out, kbm) = apply(&state, &res.resolved, &mut res.map_state, now_us);
                // Push the KBM batch non-blocking; drop-on-full (never wedge the hot thread).
                if !kbm.is_empty() && !self.kbm_tx.push(kbm) {
                    kbm_dropped_total = kbm_dropped_total.wrapping_add(1);
                }
                out
            };

            // Submit inline (one bounded synchronous IOCTL). The backend performs the single
            // i16/u8 round via `OutputState::to_output_frame` → `to_xinput_*`.
            if self.target.update(&out_state).is_err() {
                return StopReason::DeviceLost;
            }

            // Publish telemetry (never blocks; Copy frame).
            let busy_ns = clock.now_qpc_ns().saturating_sub(busy_start);
            busy.record_ns(busy_ns);
            let frame = out_state.to_output_frame();
            let tf = TelemetryFrame {
                loop_busy_ns: busy_ns,
                dt_us: dt.us() as f32,
                dropped: dropped_total,
                duplicates: duplicates_total,
                in_lx: input.left.x as f32,
                in_ly: input.left.y as f32,
                in_rx: input.right.x as f32,
                in_ry: input.right.y as f32,
                out_lx: frame.lx as f32,
                out_ly: frame.ly as f32,
                out_rx: frame.rx as f32,
                out_ry: frame.ry as f32,
            };
            self.telemetry.0.write(tf);
            // `busy.p99_us()` / `kbm_dropped_total` / `input.host_qpc_ns` feed the M2+ scope.
            let _ = kbm_dropped_total;
        }
    }
}

/// Resolve the active device's [`ResolvedProfile`] from a config snapshot (Arc-ref get, off the
/// per-report path). Falls back to the all-passthrough default when the device has no assigned,
/// resolved profile — so a fresh / mis-configured config drives an identity passthrough rather
/// than failing (matching the M2 "no RC config → passthrough" behavior).
#[inline]
fn resolve_active(cfg: &EngineConfig) -> Arc<ResolvedProfile> {
    cfg.resolved
        .get(&cfg.active_device)
        .cloned()
        .unwrap_or_else(|| Arc::new(ResolvedProfile::default()))
}

/// Whether every control in the resolved profile is the trivial identity `Passthrough` (no shift,
/// no turbo) — the condition for the byte-identical M2 fast path. Computed once on the gate.
#[inline]
fn is_all_passthrough(rp: &ResolvedProfile) -> bool {
    Control::ALL.iter().all(|&c| {
        let s = rp.slot(c);
        matches!(s.bind, BindTarget::Passthrough) && s.shift_trigger.is_none() && s.turbo.is_none()
    })
}

/// Build the decoded [`ControllerState`] the mapping engine reads from a [`HotInput`] plus the
/// already-filtered sticks and processed-analog triggers.
///
/// Sticks/triggers carry the processed values (the chain ran before `apply`, §7.2). `l2_raw` /
/// `r2_raw` reflect the **physical** full-pull (from the raw trigger reading, not the processed
/// analog) so `L2FullPull`/`R2FullPull` digitize against the real `== 255`. The button bools are
/// decoded from the raw DS button word with the same btn0/btn1/btn2 layout as
/// `core::output::pack_xinput` / the former `win_io::ds_buttons_to_xinput` (single source of truth).
#[inline]
fn controller_state_from(
    input: &HotInput,
    filt_l: StickSample,
    filt_r: StickSample,
    lt_analog: f64,
    rt_analog: f64,
) -> ControllerState {
    let raw = input.raw_buttons;
    let btn0 = (raw & 0xFF) as u8;
    let btn1 = ((raw >> 8) & 0xFF) as u8;
    let btn2 = ((raw >> 16) & 0xFF) as u8;

    // D-pad hat (low nibble of btn0): 0=N,1=NE,2=E,3=SE,4=S,5=SW,6=W,7=NW, 8=released.
    let hat = btn0 & 0x0F;
    let (dpad_up, dpad_right, dpad_down, dpad_left) = match hat {
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

    let l2_raw = (input.lt.clamp(0.0, 1.0) * 255.0).round() as u8;
    let r2_raw = (input.rt.clamp(0.0, 1.0) * 255.0).round() as u8;

    ControllerState {
        lx: filt_l.x,
        ly: filt_l.y,
        rx: filt_r.x,
        ry: filt_r.y,
        l2: lt_analog,
        r2: rt_analog,
        l2_raw,
        r2_raw,
        // Face buttons (high nibble of btn0).
        square: btn0 & 0x10 != 0,
        cross: btn0 & 0x20 != 0,
        circle: btn0 & 0x40 != 0,
        triangle: btn0 & 0x80 != 0,
        dpad_up,
        dpad_down,
        dpad_left,
        dpad_right,
        // Shoulders / stick clicks / meta (btn1).
        l1: btn1 & 0x01 != 0,
        r1: btn1 & 0x02 != 0,
        l3: btn1 & 0x40 != 0,
        r3: btn1 & 0x80 != 0,
        share: btn1 & 0x10 != 0,
        options: btn1 & 0x20 != 0,
        // PS / Touchpad click (btn2).
        ps: btn2 & 0x01 != 0,
        touch_button: btn2 & 0x02 != 0,
        // Edge / touch / motion fields are not decoded into the engine's HotInput yet (capability-
        // gated in core; M5/M6). They read their `Default` (`false`/`0`).
        ..ControllerState::default()
    }
}

// XInput (`XINPUT_GAMEPAD_*`) button bits — used to lower the M2 fast-path button word back into
// the structured `PadButtons` set so the fast path and the general path share one backend egress.
const XI_DPAD_UP: u16 = 0x0001;
const XI_DPAD_DOWN: u16 = 0x0002;
const XI_DPAD_LEFT: u16 = 0x0004;
const XI_DPAD_RIGHT: u16 = 0x0008;
const XI_START: u16 = 0x0010;
const XI_BACK: u16 = 0x0020;
const XI_LTHUMB: u16 = 0x0040;
const XI_RTHUMB: u16 = 0x0080;
const XI_LSHOULDER: u16 = 0x0100;
const XI_RSHOULDER: u16 = 0x0200;
const XI_GUIDE: u16 = 0x0400;
const XI_A: u16 = 0x1000;
const XI_B: u16 = 0x2000;
const XI_X: u16 = 0x4000;
const XI_Y: u16 = 0x8000;

/// Lower an XInput button word (the M2 fast-path packing) into the structured
/// [`PadButtons`](hyperion_core::output::PadButtons) set, so the all-passthrough fast path drives
/// the same `OutputState`-based backend as the general `map::apply` path.
///
/// This is the inverse of [`pack_xinput`](hyperion_core::output::pack_xinput): a round-trip
/// `pack_xinput(xinput_to_pad_buttons(b))` is the identity over the X360 button set, so the
/// fast-path egress is provably byte-identical to M2 (which submitted the XInput word directly).
#[inline]
fn xinput_to_pad_buttons(b: u16) -> hyperion_core::output::PadButtons {
    use hyperion_core::output::PadButtons as P;
    let mut out = P::default();
    out.set(P::A, b & XI_A != 0);
    out.set(P::B, b & XI_B != 0);
    out.set(P::X, b & XI_X != 0);
    out.set(P::Y, b & XI_Y != 0);
    out.set(P::LB, b & XI_LSHOULDER != 0);
    out.set(P::RB, b & XI_RSHOULDER != 0);
    out.set(P::BACK, b & XI_BACK != 0);
    out.set(P::START, b & XI_START != 0);
    out.set(P::LS, b & XI_LTHUMB != 0);
    out.set(P::RS, b & XI_RTHUMB != 0);
    out.set(P::GUIDE, b & XI_GUIDE != 0);
    out.set(P::DPAD_UP, b & XI_DPAD_UP != 0);
    out.set(P::DPAD_DOWN, b & XI_DPAD_DOWN != 0);
    out.set(P::DPAD_LEFT, b & XI_DPAD_LEFT != 0);
    out.set(P::DPAD_RIGHT, b & XI_DPAD_RIGHT != 0);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use hyperion_core::map::{BindTarget, KeyKind, PadBtn, Profile};
    use hyperion_core::output::{pack_xinput, KbmEvent, PadButtons};

    /// A resident set resolved from a single-profile config assigned to `"dev"`.
    fn resident_for(profile: Profile) -> (EngineConfig, Resident) {
        let mut cfg = EngineConfig {
            active_device: "dev".to_string(),
            ..EngineConfig::default()
        };
        Arc::make_mut(&mut cfg.profiles).insert("dev_profile".to_string(), profile);
        cfg.assignments
            .insert("dev".to_string(), "dev_profile".to_string());
        let cfg = cfg.clamped(); // rebuilds `resolved` for the assigned device
        let res = Resident::resolve(&cfg);
        (cfg, res)
    }

    #[test]
    fn xinput_pad_buttons_round_trip_is_identity() {
        // Every X360 button word survives lower→pack, so the fast path is byte-identical to M2.
        for bit in [
            XI_A,
            XI_B,
            XI_X,
            XI_Y,
            XI_LSHOULDER,
            XI_RSHOULDER,
            XI_BACK,
            XI_START,
            XI_LTHUMB,
            XI_RTHUMB,
            XI_GUIDE,
            XI_DPAD_UP,
            XI_DPAD_DOWN,
            XI_DPAD_LEFT,
            XI_DPAD_RIGHT,
        ] {
            assert_eq!(
                pack_xinput(xinput_to_pad_buttons(bit)),
                bit,
                "bit {bit:#06x}"
            );
        }
        // A combined word round-trips too.
        let combo = XI_A | XI_DPAD_UP | XI_LSHOULDER | XI_GUIDE;
        assert_eq!(pack_xinput(xinput_to_pad_buttons(combo)), combo);
    }

    #[test]
    fn default_profile_takes_passthrough_fast_path() {
        let (_cfg, res) = resident_for(Profile::default());
        assert!(
            res.passthrough_only,
            "an all-passthrough profile uses the byte-identical fast path"
        );
    }

    #[test]
    fn remapped_profile_leaves_fast_path() {
        let mut p = Profile::default();
        p.bindings.insert(
            Control::Cross,
            hyperion_core::map::BindingSlot::from_bind(BindTarget::GamepadButton(PadBtn::B)),
        );
        let (_cfg, res) = resident_for(p);
        assert!(!res.passthrough_only, "any remap leaves the fast path");
    }

    #[test]
    fn controller_state_decodes_raw_buttons() {
        // btn0: hat=8 (neutral) + Cross (0x20); btn1: L1 (0x01) + Options (0x20); btn2: PS (0x01).
        let raw = 0x28u32 | (0x21u32 << 8) | (0x01u32 << 16);
        let input = HotInput {
            raw_buttons: raw,
            lt: 1.0,
            rt: 0.0,
            ..HotInput::default()
        };
        let st = controller_state_from(
            &input,
            StickSample::new(0.25, -0.5),
            StickSample::new(-0.1, 0.9),
            0.4,
            0.0,
        );
        assert!(st.cross && st.l1 && st.options && st.ps);
        assert!(!st.square && !st.circle && !st.triangle);
        assert!(!st.dpad_up && !st.dpad_down && !st.dpad_left && !st.dpad_right);
        assert_eq!(st.lx, 0.25);
        assert_eq!(st.ry, 0.9);
        assert_eq!(
            st.l2, 0.4,
            "processed analog is carried, not the raw trigger"
        );
        assert_eq!(st.l2_raw, 255, "raw full-pull from the physical trigger");
        assert_eq!(st.r2_raw, 0);
    }

    #[test]
    fn controller_state_decodes_dpad_hat() {
        // hat=2 -> East (right only).
        let input = HotInput {
            raw_buttons: 0x02,
            ..HotInput::default()
        };
        let st =
            controller_state_from(&input, StickSample::NEUTRAL, StickSample::NEUTRAL, 0.0, 0.0);
        assert!(st.dpad_right && !st.dpad_up && !st.dpad_down && !st.dpad_left);
    }

    #[test]
    fn resolve_falls_back_to_passthrough_for_unassigned_device() {
        // A config with no assigned/resolved profile drives an identity passthrough, not a panic.
        let cfg = EngineConfig::default().clamped();
        let res = Resident::resolve(&cfg);
        assert!(res.passthrough_only);
    }

    #[test]
    fn reset_clears_all_state() {
        let (_cfg, mut res) = resident_for(Profile::default());
        // Dirty the mapping state, then reset.
        res.map_state.prev_active[Control::Cross.as_index()] = true;
        res.map_state.toggle.set(0x41, true);
        res.stick_state[0].rc_primed = true;
        res.trig_state[0].last_pressed = true;
        res.reset();
        assert!(!res.map_state.prev_active[Control::Cross.as_index()]);
        assert!(!res.map_state.toggle.get(0x41));
        assert!(!res.stick_state[0].rc_primed);
        assert!(!res.trig_state[0].last_pressed);
    }

    /// Drive one report through the mapping engine the way the hot loop does, returning the
    /// projected frame + any KBM batch. Mirrors the loop body without the trait I/O.
    fn step(
        res: &mut Resident,
        input: &HotInput,
    ) -> (
        hyperion_core::output::OutputFrame,
        Option<hyperion_core::output::KbmBatch>,
    ) {
        let dt = Dt::guarded(input.dt_us);
        if input.is_prime {
            res.prime();
        }
        let filt_l = hyperion_core::stick::process_stick(
            input.left,
            &res.stick_settings[0],
            &mut res.stick_state[0],
            dt,
        );
        let filt_r = hyperion_core::stick::process_stick(
            input.right,
            &res.stick_settings[1],
            &mut res.stick_state[1],
            dt,
        );
        let (lt, _) = process_trigger(input.lt, &res.trig_settings[0], &mut res.trig_state[0], dt);
        let (rt, _) = process_trigger(input.rt, &res.trig_settings[1], &mut res.trig_state[1], dt);
        if res.passthrough_only {
            let out = OutputState {
                lx: filt_l.x,
                ly: filt_l.y,
                rx: filt_r.x,
                ry: filt_r.y,
                lt,
                rt,
                buttons: xinput_to_pad_buttons(input.buttons),
            };
            (out.to_output_frame(), None)
        } else {
            let state = controller_state_from(input, filt_l, filt_r, lt, rt);
            let (out, kbm) = apply(&state, &res.resolved, &mut res.map_state, 0);
            (out.to_output_frame(), Some(kbm))
        }
    }

    #[test]
    fn passthrough_frame_matches_m2_baseline() {
        // The fast path must reproduce the M2 frame: filtered sticks + raw triggers (default
        // trigger settings pass through) + the XInput button word, packed once at egress.
        let (_cfg, mut res) = resident_for(Profile::default());
        let input = HotInput {
            left: StickSample::new(0.25, -0.5),
            right: StickSample::new(-0.1, 0.9),
            lt: 0.4,
            rt: 0.6,
            buttons: XI_A | XI_DPAD_UP,
            raw_buttons: 0,
            is_prime: false,
            ..HotInput::default()
        };
        let (frame, kbm) = step(&mut res, &input);
        assert!(kbm.is_none(), "fast path emits no KBM batch");
        // With default (passthrough) sticks, the filtered sticks equal the input sticks.
        assert_eq!(frame.lx, 0.25);
        assert_eq!(frame.ly, -0.5);
        assert_eq!(frame.rx, -0.1);
        assert_eq!(frame.ry, 0.9);
        assert_eq!(frame.lt, 0.4);
        assert_eq!(frame.rt, 0.6);
        // Buttons survive the lower→pack round-trip identically.
        assert_eq!(frame.buttons, XI_A | XI_DPAD_UP);
    }

    #[test]
    fn button_remap_routes_through_apply() {
        // Cross -> Xbox B: pressing Cross sets B (not A) on the projected frame.
        let mut p = Profile::default();
        p.bindings.insert(
            Control::Cross,
            hyperion_core::map::BindingSlot::from_bind(BindTarget::GamepadButton(PadBtn::B)),
        );
        let (_cfg, mut res) = resident_for(p);
        assert!(!res.passthrough_only);
        // Cross is btn0 0x20.
        let input = HotInput {
            raw_buttons: 0x20 | 0x08, /* hat neutral (8) + cross */
            is_prime: false,
            ..HotInput::default()
        };
        let (frame, kbm) = step(&mut res, &input);
        assert_eq!(frame.buttons & XI_B, XI_B, "B set");
        assert_eq!(
            frame.buttons & XI_A,
            0,
            "A suppressed (identity remapped away)"
        );
        assert!(kbm.unwrap().is_empty(), "a pad remap queues no KBM edge");
    }

    #[test]
    fn key_remap_emits_kbm_edges() {
        // Square -> key 0x41 (hold): KeyDown on press, KeyUp on release.
        let mut p = Profile::default();
        p.bindings.insert(
            Control::Square,
            hyperion_core::map::BindingSlot::from_bind(BindTarget::Key {
                vk: 0x41,
                kind: KeyKind::HOLD,
            }),
        );
        let (_cfg, mut res) = resident_for(p);
        // Square is btn0 0x10; hat neutral = 0x08.
        let down = HotInput {
            raw_buttons: 0x10 | 0x08,
            is_prime: false,
            ..HotInput::default()
        };
        let up = HotInput {
            raw_buttons: 0x08,
            is_prime: false,
            ..HotInput::default()
        };
        let (_f1, b1) = step(&mut res, &down);
        assert_eq!(
            b1.unwrap().as_slice(),
            &[KbmEvent::Key {
                vk: 0x41,
                down: true,
                kind: hyperion_core::output::KeyKind::Virtual
            }]
        );
        // Held: no further edge.
        let (_f2, b2) = step(&mut res, &down);
        assert!(b2.unwrap().is_empty());
        // Release: KeyUp.
        let (_f3, b3) = step(&mut res, &up);
        assert_eq!(
            b3.unwrap().as_slice(),
            &[KbmEvent::Key {
                vk: 0x41,
                down: false,
                kind: hyperion_core::output::KeyKind::Virtual
            }]
        );
    }

    #[test]
    fn rc_filter_path_is_byte_identical_to_m2() {
        // A profile with only RC on (every downstream stage default) must produce the same
        // filtered stick as driving the RcFilter directly — the M2 regression (blueprint §13).
        use hyperion_core::rc::{RcConfig, RcFilter, RcMode, RcStickState};
        use hyperion_core::stick::StickAlgorithm;
        let rc = RcConfig {
            enabled: true,
            mode: RcMode::FireBirdInteger,
            use_dynamic_curve: false,
            period_us: 4000,
            fixed_param: 100,
            ..RcConfig::default()
        };
        let p = Profile {
            ls: StickSettings {
                rc,
                rc_mode_on: true,
                ..StickSettings::default()
            },
            ..Profile::default()
        };
        let (_cfg, mut res) = resident_for(p);
        assert!(
            res.passthrough_only,
            "RC-only is still an all-passthrough map"
        );

        let filter = RcFilter;
        let mut ref_state = RcStickState::default();
        let dt = Dt::guarded(4000.0);
        let inputs = [128.0f64, 255.0, 200.0, 160.0, 128.0, 90.0, 255.0, 128.0];
        let mut first = true;
        for (i, &v) in inputs.iter().enumerate() {
            let sx = hyperion_core::convert::ds4_to_axis(v);
            let input = HotInput {
                left: StickSample::new(sx, 0.0),
                is_prime: i == 0,
                dt_us: 4000.0,
                ..HotInput::default()
            };
            let s = StickSample::new(sx, 0.0);
            let ref_out = if first {
                filter.prime(&rc, &mut ref_state, s);
                first = false;
                s
            } else {
                filter.process(&rc, &mut ref_state, dt, s)
            };
            let (frame, _) = step(&mut res, &input);
            assert!(
                (frame.lx - ref_out.x).abs() < 1e-12,
                "report {i}: hot lx={} ref={}",
                frame.lx,
                ref_out.x
            );
        }
    }

    #[test]
    fn generation_reresolve_swaps_profile_but_keeps_state() {
        // Start all-passthrough, then re-resolve to a remapping profile (simulating a gen bump).
        let (_cfg0, mut res) = resident_for(Profile::default());
        assert!(res.passthrough_only);
        // Dirty some live state; re-resolve must preserve it.
        res.stick_state[0].rc_primed = true;

        let mut p = Profile::default();
        p.bindings.insert(
            Control::Cross,
            hyperion_core::map::BindingSlot::from_bind(BindTarget::GamepadButton(PadBtn::B)),
        );
        let mut cfg = EngineConfig {
            active_device: "dev".to_string(),
            ..EngineConfig::default()
        };
        Arc::make_mut(&mut cfg.profiles).insert("dev_profile".to_string(), p);
        cfg.assignments
            .insert("dev".to_string(), "dev_profile".to_string());
        let cfg = cfg.clamped();

        res.reresolve(&cfg);
        assert!(!res.passthrough_only, "re-resolve picks up the new profile");
        assert!(
            res.stick_state[0].rc_primed,
            "live filter state is preserved"
        );
    }

    #[test]
    fn pad_buttons_packing_sanity() {
        // Guard: PadButtons::A lowers to the XInput A bit (shared with the backend).
        let mut b = PadButtons::default();
        b.set(PadButtons::A, true);
        assert_eq!(pack_xinput(b), XI_A);
    }
}
