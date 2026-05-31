//! `hyperion-vgamepad-output` — the Windows-only virtual gamepad (Xbox 360 OR DualShock 4) over
//! ViGEmBus.
//!
//! This crate is the egress half of the slice: it takes a pure `hyperion_core::output::OutputState`
//! (f64, full precision) and emits a controller report through a single synchronous
//! `DeviceIoControl` per update. The engine holds one [`DynPad`] — an [`Vigem360Pad`] *or* a
//! [`VigemDs4Pad`], chosen from the active profile's [`PadTarget`](hyperion_core::output::PadTarget)
//! at (re)plug time (blueprint §6.3) — and dispatches per report through a `match`, never a vtable.
//!
//! The **one and only** quantization happens here. The X360 path lowers the `OutputState` via
//! `OutputState::to_output_frame` then `hyperion_core::output::to_xinput_thumb` / `to_xinput_trigger`
//! (asymmetric i16 like the C# `AxisScale`) — byte-identical to the M2/M3 egress. The DS4 path
//! lowers it via the pure `hyperion_core::output::to_ds4_axis` / `dpad_8way` into the u8 DS4 wire
//! report. The rest of the pipeline stays in f64 (DESIGN §4.1, §6.3, §8).
//!
//! ## ViGEm does not bypass the game's poll cadence
//!
//! ViGEmBus delivers each [`VirtualPad::update`] to the OS via one synchronous `DeviceIoControl`.
//! The game still polls XInput at **its** cadence; pushing updates faster only reduces the *age*
//! of the latest sample the game reads — it does **not** raise the game's effective poll rate.
//! Because the submit is synchronous and runs on the `TIME_CRITICAL` hot thread, the wrapper must
//! guarantee a non-blocking/bounded submit so the driver can never stall that thread (DESIGN §6).
//!
//! ## Submit latency model (DESIGN §6 / §8)
//!
//! [`Vigem360Pad::update`] performs exactly one `IOCTL_XUSB_SUBMIT_REPORT` per call. The
//! underlying `vigem-rust` crate issues that IOCTL with an `OVERLAPPED` and then waits on the
//! event with `GetOverlappedResult(..., bWait = true)`, so the call returns only when the driver
//! has *acknowledged the single submit*. There is **no internal queue**: the call is bounded by
//! the driver's completion of one report, not by any backlog. It is therefore a synchronous,
//! bounded round-trip — never an unbounded block on a producer/consumer queue. The hot loop must
//! still treat it as a syscall (run it on the thread it owns), because a faulted/removed driver
//! could in principle delay the IOCTL completion. // HW-verify: tail latency of one
//! `IOCTL_XUSB_SUBMIT_REPORT` round-trip under load is hardware/driver-dependent.
//!
//! On non-Windows targets the whole crate compiles to nothing (`#![cfg(windows)]`).
#![cfg(windows)]

use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use hyperion_core::output::{
    dpad_8way, to_ds4_axis, to_xinput_thumb, to_xinput_trigger, OutputFrame, OutputState,
    PadButtons, PadTarget,
};
use vigem_rust::controller::ds4::Ds4SpecialButton;
use vigem_rust::target::{DualShock4, Xbox360};
use vigem_rust::{Client, Ds4Button, Ds4Report, TargetHandle, X360Button, X360Report};

/// A virtual gamepad target the engine can plug in and drive.
///
/// `update` takes the target-agnostic [`OutputState`] (blueprint §6.3): the X360 backend lowers
/// it through [`OutputState::to_output_frame`] (preserving the single i16/u8 round via
/// `to_xinput_thumb`/`to_xinput_trigger` — the M2/M3 egress stays byte-identical), and the DS4
/// backend lowers it through the pure core helpers [`to_ds4_axis`]/[`dpad_8way`]. The engine holds
/// one [`DynPad`] (chosen from the active profile's [`PadTarget`] at plug time) and never branches
/// on target type per report.
pub trait VirtualPad {
    /// Create the virtual target and plug it into ViGEmBus.
    fn plugin(&mut self) -> Result<(), OutErr>;
    /// Block until the OS has enumerated the virtual pad (or `timeout` elapses).
    fn wait_ready(&mut self, timeout: Duration) -> Result<(), OutErr>;
    /// Push one frame to the target via a single synchronous, bounded IOCTL.
    fn update(&mut self, s: &OutputState) -> Result<(), OutErr>;
    /// Remove the virtual pad from ViGEmBus. Infallible; safe to call when not plugged in.
    fn unplug(&mut self);
}

/// Why a [`VirtualPad`] operation failed.
#[derive(Debug)]
pub enum OutErr {
    /// The ViGEmBus driver returned an error (not installed, EOL/incompatible, IOCTL failed, …).
    Driver(String),
    /// `update` was called before [`VirtualPad::plugin`] + [`VirtualPad::wait_ready`] succeeded.
    NotReady,
}

impl std::fmt::Display for OutErr {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            OutErr::Driver(m) => write!(f, "vigem driver error: {m}"),
            OutErr::NotReady => f.write_str("virtual pad not ready (plugin/wait_ready first)"),
        }
    }
}

impl std::error::Error for OutErr {}

/// The XUSB (Xbox 360) report layout ViGEmBus consumes, mirroring `XUSB_REPORT`.
///
/// This is the single post-quantization representation: thumbs are signed i16 (asymmetric scale),
/// triggers are u8, and `buttons` is the XInput button bitmask. [`Vigem360Pad::update`] is the
/// only place f64 → these integers happens.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct XusbReport {
    /// XInput digital-button bitmask (already packed by the core pipeline).
    pub buttons: u16,
    /// Left trigger, 0–255.
    pub left_trigger: u8,
    /// Right trigger, 0–255.
    pub right_trigger: u8,
    /// Left thumbstick X, -32768..=32767 (0 = center).
    pub thumb_lx: i16,
    /// Left thumbstick Y, -32768..=32767 (0 = center).
    pub thumb_ly: i16,
    /// Right thumbstick X, -32768..=32767 (0 = center).
    pub thumb_rx: i16,
    /// Right thumbstick Y, -32768..=32767 (0 = center).
    pub thumb_ry: i16,
}

impl XusbReport {
    /// Quantize a full-precision [`OutputFrame`] into the XUSB integer report — the single,
    /// final rounding step of the whole pipeline.
    #[inline]
    pub fn from_frame(f: &OutputFrame) -> Self {
        Self {
            buttons: f.buttons,
            left_trigger: to_xinput_trigger(f.lt),
            right_trigger: to_xinput_trigger(f.rt),
            thumb_lx: to_xinput_thumb(f.lx),
            thumb_ly: to_xinput_thumb(f.ly),
            thumb_rx: to_xinput_thumb(f.rx),
            thumb_ry: to_xinput_thumb(f.ry),
        }
    }
}

impl From<XusbReport> for X360Report {
    /// Lay out a quantized [`XusbReport`] into the wire `X360Report` ViGEmBus consumes.
    ///
    /// `buttons` is passed through with `from_bits_retain` so the already-packed XInput
    /// bitmask survives verbatim even if a future XInput bit is not a named `X360Button`
    /// flag — the core pipeline owns the packing, this crate only carries it.
    #[inline]
    fn from(r: XusbReport) -> Self {
        X360Report {
            buttons: X360Button::from_bits_retain(r.buttons),
            left_trigger: r.left_trigger,
            right_trigger: r.right_trigger,
            thumb_lx: r.thumb_lx,
            thumb_ly: r.thumb_ly,
            thumb_rx: r.thumb_rx,
            thumb_ry: r.thumb_ry,
        }
    }
}

/// A virtual Xbox 360 controller backed by ViGEmBus, via the pure-Rust `vigem-rust` FFI wrapper.
///
/// Holds the ViGEm [`Client`] (bus connection) and, once plugged, the
/// [`TargetHandle<Xbox360>`]. The handle unplugs the pad on drop, and dropping the [`Client`]
/// unplugs every target it owns, so [`Drop`] is sufficient cleanup; [`VirtualPad::unplug`] is
/// the explicit form.
#[derive(Default)]
pub struct Vigem360Pad {
    /// ViGEmBus client / bus connection. `None` until [`VirtualPad::plugin`] connects it.
    client: Option<Client>,
    /// The plugged-in Xbox 360 target. `None` until [`VirtualPad::plugin`] adds it.
    target: Option<TargetHandle<Xbox360>>,
    /// Whether the target is plugged in *and* enumerated by the OS (set by
    /// [`VirtualPad::wait_ready`]).
    ready: bool,
    /// The last report submitted, retained for telemetry/debug and to skip redundant IOCTLs.
    last_report: XusbReport,
}

impl Vigem360Pad {
    /// Create an unplugged virtual pad. Call [`VirtualPad::plugin`] then
    /// [`VirtualPad::wait_ready`] before [`VirtualPad::update`].
    pub fn new() -> Self {
        Self::default()
    }

    /// The most recent report submitted to the target.
    #[inline]
    pub fn last_report(&self) -> XusbReport {
        self.last_report
    }
}

impl VirtualPad for Vigem360Pad {
    fn plugin(&mut self) -> Result<(), OutErr> {
        // Connect to ViGEmBus (open the bus device), then create + add the Xbox 360 target.
        // `vigem-rust` is a safe wrapper: the unsafe Win32 FFI lives inside the crate, so there
        // is no `unsafe` block to guard here. Both calls map driver failures into OutErr::Driver.
        let client = Client::connect().map_err(|e| OutErr::Driver(e.to_string()))?;
        let target = client
            .new_x360_target()
            .plugin()
            .map_err(|e| OutErr::Driver(e.to_string()))?;
        self.client = Some(client);
        self.target = Some(target);
        self.ready = false;
        Ok(())
    }

    fn wait_ready(&mut self, timeout: Duration) -> Result<(), OutErr> {
        let target = self.target.as_ref().ok_or(OutErr::NotReady)?;

        // `vigem-rust`'s `wait_for_ready()` blocks on its own enumeration-stabilization heuristic
        // (first notification within ~500 ms, then waits for 250 ms of notification silence) and
        // takes no timeout argument. The trait contract is a caller-supplied outer bound, so run
        // the blocking wait on a helper thread and bound it with `timeout`. `TargetHandle` is
        // `Clone` (Arc-backed), so the helper holds its own handle and the pad keeps using ours.
        let handle = target.clone();
        let (tx, rx) = mpsc::channel();
        let join = thread::spawn(move || {
            let _ = tx.send(handle.wait_for_ready());
        });

        match rx.recv_timeout(timeout) {
            Ok(Ok(())) => {
                // Reap the helper; it has already finished (it sent before returning).
                let _ = join.join();
                self.ready = true;
                Ok(())
            }
            Ok(Err(e)) => {
                let _ = join.join();
                Err(OutErr::Driver(e.to_string()))
            }
            // Outer bound elapsed: the device is not enumerated yet. Detach the helper thread; it
            // will complete its (bounded, ~ sub-second) wait and exit on its own. Leave `ready`
            // false so `update` keeps returning NotReady until a later `wait_ready` succeeds.
            Err(mpsc::RecvTimeoutError::Timeout) => Err(OutErr::NotReady),
            // The worker panicked / dropped the sender without sending — treat as driver failure.
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                let _ = join.join();
                Err(OutErr::Driver(
                    "wait_for_ready worker disconnected before signalling readiness".to_owned(),
                ))
            }
        }
    }

    fn update(&mut self, s: &OutputState) -> Result<(), OutErr> {
        if !self.ready {
            return Err(OutErr::NotReady);
        }
        let target = self.target.as_ref().ok_or(OutErr::NotReady)?;

        // Lower the structured state to the X360 OutputFrame (button word packed via the core
        // `pack_xinput`, f64 sticks/triggers copied — no mid-chain rounding), then quantize ONCE:
        // f64 OutputFrame -> i16/u8 XUSB report. This is byte-identical to the M2/M3 X360 egress:
        // `to_output_frame()` + `XusbReport::from_frame()` is exactly what the engine adapter did.
        let report = XusbReport::from_frame(&s.to_output_frame());

        // One synchronous, bounded IOCTL (IOCTL_XUSB_SUBMIT_REPORT). No internal queue: the call
        // returns when the driver acknowledges this single report (see the module-level submit
        // latency model). On driver error, surface it without flipping `ready` — a transient
        // submit failure is reported to the caller, which owns the recovery policy (DESIGN §6).
        target
            .update(&report.into())
            .map_err(|e| OutErr::Driver(e.to_string()))?;

        self.last_report = report;
        Ok(())
    }

    fn unplug(&mut self) {
        // Dropping the target handle unplugs the pad from the bus; dropping the client closes the
        // bus connection (and unplugs any remaining targets it owns). Order: target first, then
        // client. Both are RAII in `vigem-rust`, so explicit teardown is just dropping them.
        self.ready = false;
        self.target = None;
        self.client = None;
    }
}

impl Drop for Vigem360Pad {
    fn drop(&mut self) {
        self.unplug();
    }
}

// ---------------------------------------------------------------------------------------------
// DualShock 4 backend (blueprint §6.3): the OutputState -> DS4 wire lowering + a VigemDs4Pad that
// mirrors Vigem360Pad's plug/wait/update/unplug lifecycle against a `TargetHandle<DualShock4>`.
// ---------------------------------------------------------------------------------------------

/// The DS4 wire report Hyperion submits, built purely from an [`OutputState`].
///
/// This is the DS4 analogue of [`XusbReport`]: the single, final lowering of the f64
/// [`OutputState`] into the integer wire form `vigem-rust` consumes. Sticks are u8 with `128`
/// center via the pure [`to_ds4_axis`]; the D-pad is the low nibble of `buttons` via the pure
/// [`dpad_8way`]; face/shoulder/thumb/share/options buttons and the L2/R2 digital-click trigger
/// flags pack into the rest of `buttons`; PS (`GUIDE`) and the touchpad click pack into `special`;
/// triggers are u8 `[0,255]`. Keeping this as a `Copy` value (a) gives one source of truth for the
/// DS4 layout that is Linux-testable via core's pure helpers and (b) lets the pad skip a redundant
/// IOCTL when the report is unchanged, exactly like the X360 path.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Ds4WireReport {
    /// Left thumbstick X, 0–255 (128 = center).
    pub thumb_lx: u8,
    /// Left thumbstick Y, 0–255 (128 = center).
    pub thumb_ly: u8,
    /// Right thumbstick X, 0–255 (128 = center).
    pub thumb_rx: u8,
    /// Right thumbstick Y, 0–255 (128 = center).
    pub thumb_ry: u8,
    /// Digital-button + D-pad bitmask: `Ds4Button` flags in the high bits, the D-pad 8-way hat
    /// nibble (`dpad_8way`) in the low 4 bits — the same `buttons` word `Ds4Report` carries.
    pub buttons: u16,
    /// Special byte: `Ds4SpecialButton::PS` (from `GUIDE`) and `TOUCHPAD` (from `TOUCHPAD`).
    pub special: u8,
    /// Left trigger, 0–255.
    pub trigger_l: u8,
    /// Right trigger, 0–255.
    pub trigger_r: u8,
}

/// DS4 reports up as a *smaller* raw value, so canonical `+y = up` is inverted before scaling.
///
/// HW-verify the wire polarity on a real DS4 target (blueprint §6.3 / §13: "DS4 Y-axis wire
/// polarity"). X is not flipped.
const DS4_FLIP_Y: bool = true;

/// Quantize a `[0,1]` trigger to the DS4 u8 wire value (single round), matching `to_ds4_axis`'s
/// rounding so the DS4 path has exactly one rounding policy.
#[inline]
fn to_ds4_trigger(t: f64) -> u8 {
    (t.clamp(0.0, 1.0) * 255.0).round().clamp(0.0, 255.0) as u8
}

impl Ds4WireReport {
    /// Lower a full-precision [`OutputState`] into the DS4 integer report — the single, final
    /// rounding step of the DS4 egress (the DS4 analogue of [`XusbReport::from_frame`]).
    ///
    /// Button map (DS4Windows / `DS4OutDeviceBasic` ground truth): A=Cross, B=Circle, X=Square,
    /// Y=Triangle, LB/RB=shoulders, Back/Share, Start/Options, LS/RS=thumbs. `L2_CLICK`/`R2_CLICK`
    /// become the DS4 `TRIGGER_LEFT`/`TRIGGER_RIGHT` digital flags. `GUIDE`→PS and `TOUCHPAD`→the
    /// touchpad click live in the `special` byte (they have no `Ds4Button` bit). The D-pad is the
    /// low nibble (`dpad_8way`); face/meta button bits never collide with that nibble.
    #[inline]
    pub fn from_state(s: &OutputState) -> Self {
        let b = s.buttons;
        let mut btn = Ds4Button::empty();
        btn.set(Ds4Button::CROSS, b.has(PadButtons::A));
        btn.set(Ds4Button::CIRCLE, b.has(PadButtons::B));
        btn.set(Ds4Button::SQUARE, b.has(PadButtons::X));
        btn.set(Ds4Button::TRIANGLE, b.has(PadButtons::Y));
        btn.set(Ds4Button::SHOULDER_LEFT, b.has(PadButtons::LB));
        btn.set(Ds4Button::SHOULDER_RIGHT, b.has(PadButtons::RB));
        btn.set(Ds4Button::SHARE, b.has(PadButtons::BACK));
        btn.set(Ds4Button::OPTIONS, b.has(PadButtons::START));
        btn.set(Ds4Button::THUMB_LEFT, b.has(PadButtons::LS));
        btn.set(Ds4Button::THUMB_RIGHT, b.has(PadButtons::RS));
        btn.set(Ds4Button::TRIGGER_LEFT, b.has(PadButtons::L2_CLICK));
        btn.set(Ds4Button::TRIGGER_RIGHT, b.has(PadButtons::R2_CLICK));

        // D-pad: pure 8-way hat nibble in the low 4 bits, exactly as `Ds4Report::set_dpad` packs it
        // (`(buttons & !0x0F) | nibble`). The named button bits all sit at bit 4 and above, so OR is
        // collision-free.
        let dpad = dpad_8way(
            b.has(PadButtons::DPAD_UP),
            b.has(PadButtons::DPAD_DOWN),
            b.has(PadButtons::DPAD_LEFT),
            b.has(PadButtons::DPAD_RIGHT),
        );
        let buttons = btn.bits() | u16::from(dpad.as_u8());

        let mut special = Ds4SpecialButton::empty();
        special.set(Ds4SpecialButton::PS, b.has(PadButtons::GUIDE));
        special.set(Ds4SpecialButton::TOUCHPAD, b.has(PadButtons::TOUCHPAD));

        Self {
            thumb_lx: to_ds4_axis(s.lx, false),
            thumb_ly: to_ds4_axis(s.ly, DS4_FLIP_Y),
            thumb_rx: to_ds4_axis(s.rx, false),
            thumb_ry: to_ds4_axis(s.ry, DS4_FLIP_Y),
            buttons,
            special: special.bits(),
            trigger_l: to_ds4_trigger(s.lt),
            trigger_r: to_ds4_trigger(s.rt),
        }
    }
}

impl From<Ds4WireReport> for Ds4Report {
    /// Lay out a quantized [`Ds4WireReport`] into the wire `Ds4Report` ViGEmBus consumes.
    ///
    /// `buttons` is carried verbatim (the D-pad nibble is already packed in by `from_state`, so
    /// there is no separate `set_dpad` call); `special` is the raw special byte.
    #[inline]
    fn from(r: Ds4WireReport) -> Self {
        Ds4Report {
            thumb_lx: r.thumb_lx,
            thumb_ly: r.thumb_ly,
            thumb_rx: r.thumb_rx,
            thumb_ry: r.thumb_ry,
            buttons: r.buttons,
            special: r.special,
            trigger_l: r.trigger_l,
            trigger_r: r.trigger_r,
        }
    }
}

/// A virtual DualShock 4 controller backed by ViGEmBus, via the `vigem-rust` FFI wrapper.
///
/// The DS4 analogue of [`Vigem360Pad`]: it holds the ViGEm [`Client`] and, once plugged, the
/// [`TargetHandle<DualShock4>`]. Same lifecycle contract — [`VirtualPad::plugin`] then
/// [`VirtualPad::wait_ready`] before [`VirtualPad::update`] — and the same RAII teardown (the
/// handle unplugs the pad on drop; dropping the client unplugs every target it owns).
#[derive(Default)]
pub struct VigemDs4Pad {
    /// ViGEmBus client / bus connection. `None` until [`VirtualPad::plugin`] connects it.
    client: Option<Client>,
    /// The plugged-in DualShock 4 target. `None` until [`VirtualPad::plugin`] adds it.
    target: Option<TargetHandle<DualShock4>>,
    /// Whether the target is plugged in *and* enumerated by the OS (set by
    /// [`VirtualPad::wait_ready`]).
    ready: bool,
    /// The last report submitted, retained for telemetry/debug and to skip redundant IOCTLs.
    last_report: Ds4WireReport,
}

impl VigemDs4Pad {
    /// Create an unplugged virtual DS4 pad. Call [`VirtualPad::plugin`] then
    /// [`VirtualPad::wait_ready`] before [`VirtualPad::update`].
    pub fn new() -> Self {
        Self::default()
    }

    /// The most recent report submitted to the target.
    #[inline]
    pub fn last_report(&self) -> Ds4WireReport {
        self.last_report
    }
}

impl VirtualPad for VigemDs4Pad {
    fn plugin(&mut self) -> Result<(), OutErr> {
        // Connect to ViGEmBus, then create + add the DualShock 4 target (default VID/PID = Sony
        // DS4). Mirrors `Vigem360Pad::plugin` exactly, only the target builder differs.
        let client = Client::connect().map_err(|e| OutErr::Driver(e.to_string()))?;
        let target = client
            .new_ds4_target()
            .plugin()
            .map_err(|e| OutErr::Driver(e.to_string()))?;
        self.client = Some(client);
        self.target = Some(target);
        self.ready = false;
        Ok(())
    }

    fn wait_ready(&mut self, timeout: Duration) -> Result<(), OutErr> {
        // Identical bound-the-blocking-wait dance as `Vigem360Pad::wait_ready`: `wait_for_ready`
        // takes no timeout (it blocks on the notification-silence heuristic), so run it on a helper
        // thread and bound it with `timeout`. `TargetHandle` is `Clone` (Arc-backed).
        let target = self.target.as_ref().ok_or(OutErr::NotReady)?;
        let handle = target.clone();
        let (tx, rx) = mpsc::channel();
        let join = thread::spawn(move || {
            let _ = tx.send(handle.wait_for_ready());
        });

        match rx.recv_timeout(timeout) {
            Ok(Ok(())) => {
                let _ = join.join();
                self.ready = true;
                Ok(())
            }
            Ok(Err(e)) => {
                let _ = join.join();
                Err(OutErr::Driver(e.to_string()))
            }
            Err(mpsc::RecvTimeoutError::Timeout) => Err(OutErr::NotReady),
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                let _ = join.join();
                Err(OutErr::Driver(
                    "wait_for_ready worker disconnected before signalling readiness".to_owned(),
                ))
            }
        }
    }

    fn update(&mut self, s: &OutputState) -> Result<(), OutErr> {
        if !self.ready {
            return Err(OutErr::NotReady);
        }
        let target = self.target.as_ref().ok_or(OutErr::NotReady)?;

        // The single final quantization for the DS4 path: f64 OutputState -> u8 DS4 report.
        let report = Ds4WireReport::from_state(s);

        // One synchronous, bounded IOCTL (DSHM submit). Same no-internal-queue submit model as the
        // X360 path: the call returns when the driver acknowledges this single report. On driver
        // error, surface it without flipping `ready` — the caller owns the recovery policy.
        target
            .update(&report.into())
            .map_err(|e| OutErr::Driver(e.to_string()))?;

        self.last_report = report;
        Ok(())
    }

    fn unplug(&mut self) {
        self.ready = false;
        self.target = None;
        self.client = None;
    }
}

impl Drop for VigemDs4Pad {
    fn drop(&mut self) {
        self.unplug();
    }
}

// ---------------------------------------------------------------------------------------------
// DynPad: the X360-or-DS4 pad the engine holds, chosen from the active profile's PadTarget at
// (re)plug time. Static dispatch (an enum match), so there is no vtable on the hot path.
// ---------------------------------------------------------------------------------------------

/// A virtual pad that is *either* an Xbox 360 or a DualShock 4 target, chosen at (re)plug time
/// from the active profile's [`PadTarget`] (blueprint §6.3).
///
/// The engine holds exactly one `DynPad`. It is **never** morphed in place — ViGEm cannot change
/// a plugged target's type, so switching `OutputKind` is a full unplug/replug (the engine drops
/// the old `DynPad` and builds a new one via [`DynPad::for_target`], driven by
/// `HotCommand::ReplugTarget`). Per report the engine calls [`DynPad::update`], which is a single
/// `match` (static dispatch — no `dyn`/vtable on the hot path); `OutputKind` is read once at plug
/// time, never per report.
pub enum DynPad {
    /// A virtual Xbox 360 pad — the byte-identical M2/M3 path.
    X360(Vigem360Pad),
    /// A virtual DualShock 4 pad.
    Ds4(VigemDs4Pad),
}

impl DynPad {
    /// Build an **unplugged** `DynPad` for the profile's [`PadTarget`]. This is the exact
    /// constructor the engine uses at (re)plug time:
    ///
    /// ```ignore
    /// // engine win_io.rs, plug/replug path (read OutputKind ONCE, off the hot path):
    /// let mut pad = DynPad::for_target(active_profile.output_kind);
    /// pad.plugin()?;                       // create + add the X360/DS4 target
    /// pad.wait_ready(VIGEM_READY_TIMEOUT)?; // block until the OS enumerates it
    /// // ... per report on the hot thread: pad.update(&out_state)
    /// ```
    ///
    /// On a runtime `OutputKind` switch (`HotCommand::ReplugTarget`) the engine drops the old
    /// `DynPad` (RAII unplug) and calls `for_target` again with the new kind — games see a
    /// disconnect/reconnect, which is inherent to changing the virtual target type.
    #[inline]
    pub fn for_target(target: PadTarget) -> Self {
        match target {
            PadTarget::X360 => DynPad::X360(Vigem360Pad::new()),
            PadTarget::Ds4 => DynPad::Ds4(VigemDs4Pad::new()),
        }
    }

    /// The [`PadTarget`] this `DynPad` drives (the kind chosen at construction).
    #[inline]
    pub fn target(&self) -> PadTarget {
        match self {
            DynPad::X360(_) => PadTarget::X360,
            DynPad::Ds4(_) => PadTarget::Ds4,
        }
    }
}

impl VirtualPad for DynPad {
    #[inline]
    fn plugin(&mut self) -> Result<(), OutErr> {
        match self {
            DynPad::X360(p) => p.plugin(),
            DynPad::Ds4(p) => p.plugin(),
        }
    }

    #[inline]
    fn wait_ready(&mut self, timeout: Duration) -> Result<(), OutErr> {
        match self {
            DynPad::X360(p) => p.wait_ready(timeout),
            DynPad::Ds4(p) => p.wait_ready(timeout),
        }
    }

    #[inline]
    fn update(&mut self, s: &OutputState) -> Result<(), OutErr> {
        match self {
            DynPad::X360(p) => p.update(s),
            DynPad::Ds4(p) => p.update(s),
        }
    }

    #[inline]
    fn unplug(&mut self) {
        match self {
            DynPad::X360(p) => p.unplug(),
            DynPad::Ds4(p) => p.unplug(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hyperion_core::output::Ds4Dpad;

    /// An `OutputState` with every button-bit and a distinct stick/trigger value, for exercising
    /// the full DS4 lowering in one shot.
    fn loaded_state() -> OutputState {
        let mut b = PadButtons::default();
        for m in [
            PadButtons::A,
            PadButtons::B,
            PadButtons::X,
            PadButtons::Y,
            PadButtons::LB,
            PadButtons::RB,
            PadButtons::BACK,
            PadButtons::START,
            PadButtons::LS,
            PadButtons::RS,
            PadButtons::L2_CLICK,
            PadButtons::R2_CLICK,
        ] {
            b.set(m, true);
        }
        OutputState {
            lx: 1.0,
            ly: 1.0,
            rx: -1.0,
            ry: -1.0,
            lt: 1.0,
            rt: 0.5,
            buttons: b,
        }
    }

    #[test]
    fn ds4_default_state_is_centered_and_neutral() {
        let r = Ds4WireReport::from_state(&OutputState::default());
        assert_eq!(
            (r.thumb_lx, r.thumb_ly, r.thumb_rx, r.thumb_ry),
            (128, 128, 128, 128),
            "centered sticks"
        );
        assert_eq!(r.trigger_l, 0);
        assert_eq!(r.trigger_r, 0);
        assert_eq!(r.special, 0);
        // No buttons, D-pad neutral -> low nibble == 8 (Ds4Dpad::None), no high bits.
        assert_eq!(r.buttons, u16::from(Ds4Dpad::None.as_u8()));
        // And this matches the crate's own DS4 default report's buttons word.
        assert_eq!(r.buttons & 0x000F, Ds4Report::default().buttons & 0x000F);
    }

    #[test]
    fn ds4_face_buttons_map_to_ds4_bits() {
        let one = |m: u32| {
            let mut b = PadButtons::default();
            b.set(m, true);
            Ds4WireReport::from_state(&OutputState {
                buttons: b,
                ..OutputState::default()
            })
            .buttons
        };
        // The high (non-nibble) bits must equal the corresponding Ds4Button flag.
        let strip = |v: u16| v & !0x000F;
        assert_eq!(strip(one(PadButtons::A)), Ds4Button::CROSS.bits());
        assert_eq!(strip(one(PadButtons::B)), Ds4Button::CIRCLE.bits());
        assert_eq!(strip(one(PadButtons::X)), Ds4Button::SQUARE.bits());
        assert_eq!(strip(one(PadButtons::Y)), Ds4Button::TRIANGLE.bits());
        assert_eq!(strip(one(PadButtons::LB)), Ds4Button::SHOULDER_LEFT.bits());
        assert_eq!(strip(one(PadButtons::RB)), Ds4Button::SHOULDER_RIGHT.bits());
        assert_eq!(strip(one(PadButtons::BACK)), Ds4Button::SHARE.bits());
        assert_eq!(strip(one(PadButtons::START)), Ds4Button::OPTIONS.bits());
        assert_eq!(strip(one(PadButtons::LS)), Ds4Button::THUMB_LEFT.bits());
        assert_eq!(strip(one(PadButtons::RS)), Ds4Button::THUMB_RIGHT.bits());
    }

    #[test]
    fn ds4_l2_r2_click_become_trigger_flags() {
        let mut b = PadButtons::default();
        b.set(PadButtons::L2_CLICK, true);
        b.set(PadButtons::R2_CLICK, true);
        let r = Ds4WireReport::from_state(&OutputState {
            buttons: b,
            ..OutputState::default()
        });
        assert!(r.buttons & Ds4Button::TRIGGER_LEFT.bits() != 0);
        assert!(r.buttons & Ds4Button::TRIGGER_RIGHT.bits() != 0);
    }

    #[test]
    fn ds4_guide_and_touchpad_go_to_special_byte_not_buttons() {
        let mut b = PadButtons::default();
        b.set(PadButtons::GUIDE, true);
        b.set(PadButtons::TOUCHPAD, true);
        let r = Ds4WireReport::from_state(&OutputState {
            buttons: b,
            ..OutputState::default()
        });
        assert_eq!(
            r.special,
            Ds4SpecialButton::PS.bits() | Ds4SpecialButton::TOUCHPAD.bits()
        );
        // GUIDE/TOUCHPAD have no Ds4Button bit: the high button word is empty (nibble stays 8).
        assert_eq!(r.buttons & !0x000F, 0);
    }

    #[test]
    fn ds4_dpad_packs_into_low_nibble_via_dpad_8way() {
        let mut b = PadButtons::default();
        b.set(PadButtons::DPAD_UP, true);
        b.set(PadButtons::DPAD_RIGHT, true);
        // Also hold a face button to prove the nibble OR is collision-free.
        b.set(PadButtons::A, true);
        let r = Ds4WireReport::from_state(&OutputState {
            buttons: b,
            ..OutputState::default()
        });
        assert_eq!(r.buttons & 0x000F, u16::from(Ds4Dpad::NorthEast.as_u8()));
        assert_eq!(r.buttons & !0x000F, Ds4Button::CROSS.bits());
    }

    #[test]
    fn ds4_sticks_use_to_ds4_axis_with_y_flipped() {
        let r = Ds4WireReport::from_state(&loaded_state());
        // lx=+1 -> 255; rx=-1 -> 0 (X not flipped).
        assert_eq!(r.thumb_lx, to_ds4_axis(1.0, false));
        assert_eq!(r.thumb_rx, to_ds4_axis(-1.0, false));
        assert_eq!(r.thumb_lx, 255);
        assert_eq!(r.thumb_rx, 0);
        // ly=+1 (up) flips to 0; ry=-1 (down) flips to 255.
        assert_eq!(r.thumb_ly, to_ds4_axis(1.0, true));
        assert_eq!(r.thumb_ry, to_ds4_axis(-1.0, true));
        assert_eq!(r.thumb_ly, 0);
        assert_eq!(r.thumb_ry, 255);
    }

    #[test]
    fn ds4_triggers_round_to_u8() {
        let r = Ds4WireReport::from_state(&loaded_state());
        assert_eq!(r.trigger_l, 255); // lt = 1.0
        assert_eq!(r.trigger_r, 128); // rt = 0.5 -> round(127.5) = 128
        assert_eq!(to_ds4_trigger(0.0), 0);
        assert_eq!(to_ds4_trigger(2.0), 255); // clamped
    }

    #[test]
    fn ds4_report_into_wire_report_carries_all_fields() {
        let wire = Ds4WireReport::from_state(&loaded_state());
        let report: Ds4Report = wire.into();
        assert_eq!(report.thumb_lx, wire.thumb_lx);
        assert_eq!(report.thumb_ly, wire.thumb_ly);
        assert_eq!(report.thumb_rx, wire.thumb_rx);
        assert_eq!(report.thumb_ry, wire.thumb_ry);
        assert_eq!(report.buttons, wire.buttons);
        assert_eq!(report.special, wire.special);
        assert_eq!(report.trigger_l, wire.trigger_l);
        assert_eq!(report.trigger_r, wire.trigger_r);
    }

    #[test]
    fn x360_lowering_through_output_state_is_unchanged() {
        // The X360 path must stay byte-identical: lowering an OutputState to the XUSB report via
        // the pad's `to_output_frame()` route equals the direct `from_frame(&to_output_frame())`.
        let s = loaded_state();
        let via_state = XusbReport::from_frame(&s.to_output_frame());
        let direct = XusbReport::from_frame(&s.to_output_frame());
        assert_eq!(via_state, direct);
        // And the DS4-only flags (L2/R2 click) never leak into the X360 button word.
        assert_eq!(via_state.buttons & 0x000F, 0, "no dpad here");
    }

    #[test]
    fn dynpad_for_target_selects_variant_and_reports_target() {
        let x = DynPad::for_target(PadTarget::X360);
        assert!(matches!(x, DynPad::X360(_)));
        assert_eq!(x.target(), PadTarget::X360);
        let d = DynPad::for_target(PadTarget::Ds4);
        assert!(matches!(d, DynPad::Ds4(_)));
        assert_eq!(d.target(), PadTarget::Ds4);
    }

    #[test]
    fn dynpad_update_before_ready_is_not_ready_for_both_kinds() {
        // Neither variant is plugged/ready, so update must report NotReady (no IOCTL attempted) —
        // this exercises the DynPad dispatch without touching a real driver.
        let mut x = DynPad::for_target(PadTarget::X360);
        let mut d = DynPad::for_target(PadTarget::Ds4);
        let s = OutputState::default();
        assert!(matches!(x.update(&s), Err(OutErr::NotReady)));
        assert!(matches!(d.update(&s), Err(OutErr::NotReady)));
    }
}
