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
//! On non-Windows targets the whole crate compiles to nothing (`#![cfg(windows)]`). In M1 the
//! ViGEmBus driver calls are bring-up stubs (`TODO(hardware)`), but the [`VirtualPad`] trait, the
//! [`Vigem360Pad`] target, and the f64→i16 mapping into [`XusbReport`] are real and final.
#![cfg(windows)]

use std::time::Duration;

use hyperion_core::output::{to_xinput_thumb, to_xinput_trigger, OutputFrame};

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
    pub buttons: u16,
    pub left_trigger: u8,
    pub right_trigger: u8,
    pub thumb_lx: i16,
    pub thumb_ly: i16,
    pub thumb_rx: i16,
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

/// A virtual Xbox 360 controller backed by ViGEmBus.
///
/// Holds the ViGEm client/bus connection and the Xbox 360 target. The handle fields are stored
/// os-agnostically for M1 (no `vigem`/`windows` crate linked yet — DESIGN §12 M1); the real types
/// are the `vigem` wrapper's `Client` and `Xbox360Target`. `TODO(hardware)`: replace with the
/// typed handles and free them in [`VirtualPad::unplug`] / `Drop`.
#[derive(Default)]
pub struct Vigem360Pad {
    /// ViGEmBus client/bus connection handle (raw, M1 placeholder).
    client: usize,
    /// Xbox 360 target handle (raw, M1 placeholder).
    target: usize,
    /// Whether the target is plugged in and enumerated by the OS.
    ready: bool,
    /// The last report submitted, retained for telemetry/debug and to avoid redundant IOCTLs.
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
        // TODO(hardware): `vigem_alloc()` + `vigem_connect()` for the client, then
        // `vigem_target_x360_alloc()` + `vigem_target_add()` to plug the Xbox 360 target.
        let _ = (&mut self.client, &mut self.target);
        Err(OutErr::Driver(
            "ViGEmBus bring-up pending (no driver linked in M1)".to_owned(),
        ))
    }

    fn wait_ready(&mut self, timeout: Duration) -> Result<(), OutErr> {
        // TODO(hardware): poll target state / wait on the add notification until the OS enumerates
        // the pad or `timeout` elapses; set `self.ready = true` on success.
        let _ = timeout;
        if self.target == 0 {
            return Err(OutErr::NotReady);
        }
        self.ready = true;
        Ok(())
    }

    fn update(&mut self, f: &OutputFrame) -> Result<(), OutErr> {
        if !self.ready {
            return Err(OutErr::NotReady);
        }
        // The single final quantization: f64 OutputFrame -> i16/u8 XUSB report.
        let report = XusbReport::from_frame(f);
        self.last_report = report;
        // TODO(hardware): `vigem_target_x360_update(client, target, &xusb_report)` — one
        // synchronous, bounded `DeviceIoControl`. Must never block the TIME_CRITICAL hot thread.
        Ok(())
    }

    fn unplug(&mut self) {
        // TODO(hardware): `vigem_target_remove()` + free target + `vigem_disconnect()`/free client.
        // No-op safe in M1 (no real handles held).
        self.ready = false;
        self.target = 0;
        self.client = 0;
    }
}

impl Drop for Vigem360Pad {
    fn drop(&mut self) {
        self.unplug();
    }
}
