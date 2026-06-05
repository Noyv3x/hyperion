//! `hyperion-hid-input` — the Windows-only, I/O-only HID shell.
//!
//! This crate is the thin device layer between the operating system's HID stack and the
//! pure [`hyperion_core`] parsers. It **does no numeric work**: it enumerates devices by
//! VID/PID, opens them with overlapped `CreateFileW`, performs double-buffered overlapped
//! `ReadFile`s, and hands the raw `&[u8]` report plus a host QPC timestamp straight to the
//! core decoder. Everything semantic — stick de-quantization, the DS sensor-timestamp
//! `SensorClock`, sequence-counter drop/dupe accounting — lives in `hyperion_core::input`.
//!
//! On non-Windows targets the entire crate compiles to nothing (`#![cfg(windows)]`), so the
//! workspace still builds and `hyperion_core` stays unit-tested on Linux CI. The public
//! surface here (the [`DeviceSource`] trait, [`SourceError`], [`DeviceId`], and the three
//! backends) is final per DESIGN §6/§7. The DualSense USB path (enumeration + overlapped
//! double-buffered `ReadFile`) and the XInput fallback are implemented against the typed
//! `windows` Win32 bindings; the generic raw-HID field decode stays a layout-driven skeleton
//! until a real report descriptor is supplied. Anything that can only be validated against
//! physical hardware is flagged with a `// HW-verify` note.
#![cfg(windows)]

use hyperion_core::input::{InputSample, SourceMeta, TouchContact};

pub mod backends;
pub mod win;

pub use backends::dualsense_usb::DualSenseUsbSource;
pub use backends::raw_hid::RawHidGenericSource;
pub use backends::xinput::XInputSource;

/// A pollable source of normalized input reports.
///
/// One [`DeviceSource`] owns exactly one physical device and is driven by the engine's hot
/// thread. It is `Send` so ownership can be moved onto that dedicated thread, but it is **not**
/// `Sync`: only the hot thread touches it.
pub trait DeviceSource: Send {
    /// Static description of what this source produces (bit depth, expected rate, layout).
    fn meta(&self) -> SourceMeta;

    /// Block until the next report is available and parse it into `out`.
    ///
    /// * `Ok(true)` — a fresh report was decoded into `out`.
    /// * `Ok(false)` — a benign timeout (no new report yet); `out` is left unchanged.
    /// * `Err(_)` — the device was lost or an unrecoverable I/O error occurred.
    fn next_sample(&mut self, out: &mut InputSample) -> Result<bool, SourceError>;

    /// The decoded touchpad contacts + DualSense Edge button bits from the **most recent**
    /// successful [`next_sample`](Self::next_sample) (M7 touch/Edge wiring).
    ///
    /// The stick-only [`InputSample`] cannot carry these (it lives in `hyperion_core` and is
    /// alloc-free / device-agnostic), so the engine reads them through this paired accessor after a
    /// fresh sample. The default impl returns the inert [`TouchEdge::default`] (untouched pad / all
    /// Edge bits `false`), so a stick-only backend (XInput, the generic raw-HID skeleton) is
    /// byte-identical to before; the DualSense backend overrides it to surface the contacts +
    /// Mute/Capture/Fn/paddle/side bits its core decode already produces.
    #[inline]
    fn touch_edge(&self) -> TouchEdge {
        TouchEdge::default()
    }

    /// The VID/PID and OS instance path identifying this device.
    fn device_id(&self) -> DeviceId;
}

/// The decoded touchpad contacts + DualSense Edge button superset a [`DeviceSource`] surfaces
/// alongside the stick-only [`InputSample`] (M7).
///
/// Carried separately from [`InputSample`] because that type is the device-agnostic, alloc-free
/// `hyperion_core` value the stick pipeline consumes; the touch grid + the Edge Fn/paddle/Mute/
/// Capture/side bits are DualSense-specific and only the DualSense backend fills them. `Default`
/// (both contacts inactive, every Edge bit `false`) is the inert non-touch / non-Edge state, so an
/// engine that copies these across is byte-identical to the pre-M7 inert path when the source does
/// not decode them.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TouchEdge {
    /// The two decoded touchpad finger contacts (`core::input::ds_report::decode_touch`).
    pub touch: [TouchContact; 2],
    /// The DualSense / DualSense Edge extended button bits (gated by the source's `meta.is_edge`).
    pub edge: EdgeButtons,
}

/// The DualSense / DualSense Edge extended button superset surfaced on a [`TouchEdge`] (M7).
///
/// Mirrors the capability-gated [`ControllerState`](hyperion_core::input::ControllerState) Edge
/// fields the core decode fills only for an Edge-capable source: Mute, Capture, the two Fn buttons,
/// the back-left/right paddles, and the two side buttons. `Default` (all `false`) is the inert
/// non-Edge behavior.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EdgeButtons {
    /// Mute button (DualSense / Edge).
    pub mute: bool,
    /// Capture / Create-adjacent capture button.
    pub capture: bool,
    /// Edge left function button.
    pub fn_l: bool,
    /// Edge right function button.
    pub fn_r: bool,
    /// Edge back-left paddle.
    pub blp: bool,
    /// Edge back-right paddle.
    pub brp: bool,
    /// Edge left side button.
    pub side_l: bool,
    /// Edge right side button.
    pub side_r: bool,
}

/// Why a [`DeviceSource::next_sample`] call did not yield a fresh report.
#[derive(Debug)]
pub enum SourceError {
    /// The device was unplugged or otherwise removed; the caller should tear the source down.
    Disconnected,
    /// A Win32 I/O error surfaced from the overlapped read path.
    Io(std::io::Error),
    /// The bounded wait elapsed with no report. Benign at the trait level; backends usually
    /// translate a timeout into `Ok(false)` instead and reserve this for hard waits.
    Timeout,
}

impl std::fmt::Display for SourceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SourceError::Disconnected => f.write_str("device disconnected"),
            SourceError::Io(e) => write!(f, "hid i/o error: {e}"),
            SourceError::Timeout => f.write_str("read timed out"),
        }
    }
}

impl std::error::Error for SourceError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            SourceError::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<std::io::Error> for SourceError {
    fn from(e: std::io::Error) -> Self {
        SourceError::Io(e)
    }
}

/// Identifies a physical device: USB vendor/product IDs plus the OS instance path.
///
/// The `path` is the value returned by `SetupDiGetDeviceInterfaceDetail` (the same string
/// HidHide consumes to blacklist the physical pad), so it is the stable key for hot-plug and
/// for telling two identical controllers apart.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DeviceId {
    pub vid: u16,
    pub pid: u16,
    pub path: String,
}

impl DeviceId {
    /// Construct a [`DeviceId`] from its parts.
    pub fn new(vid: u16, pid: u16, path: impl Into<String>) -> Self {
        Self {
            vid,
            pid,
            path: path.into(),
        }
    }
}
