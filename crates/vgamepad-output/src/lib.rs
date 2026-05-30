//! `hyperion-vgamepad-output` — the Windows-only virtual Xbox 360 pad over ViGEmBus.
//!
//! This crate is the egress half of the slice: it takes a pure `hyperion_core::output::OutputFrame`
//! (f64, full precision) and emits an Xbox 360 controller report through a single synchronous
//! `DeviceIoControl` per update. The **one and only** quantization happens here, via
//! `hyperion_core::output::to_xinput_thumb` / `to_xinput_trigger` (asymmetric i16 like the C#
//! `AxisScale`); the rest of the pipeline stays in f64 (DESIGN §4.1, §8).
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

use hyperion_core::output::{to_xinput_thumb, to_xinput_trigger, OutputFrame};
use vigem_rust::target::Xbox360;
use vigem_rust::{Client, TargetHandle, X360Button, X360Report};

/// A virtual gamepad target the engine can plug in and drive.
pub trait VirtualPad {
    /// Create the Xbox 360 target and plug it into ViGEmBus.
    fn plugin(&mut self) -> Result<(), OutErr>;
    /// Block until the OS has enumerated the virtual pad (or `timeout` elapses).
    fn wait_ready(&mut self, timeout: Duration) -> Result<(), OutErr>;
    /// Push one frame to the target via a single synchronous, bounded IOCTL.
    fn update(&mut self, f: &OutputFrame) -> Result<(), OutErr>;
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

    fn update(&mut self, f: &OutputFrame) -> Result<(), OutErr> {
        if !self.ready {
            return Err(OutErr::NotReady);
        }
        let target = self.target.as_ref().ok_or(OutErr::NotReady)?;

        // The single final quantization: f64 OutputFrame -> i16/u8 XUSB report.
        let report = XusbReport::from_frame(f);

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
