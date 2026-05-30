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

use hyperion_core::input::{InputSample, SourceMeta};

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

    /// The VID/PID and OS instance path identifying this device.
    fn device_id(&self) -> DeviceId;
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
