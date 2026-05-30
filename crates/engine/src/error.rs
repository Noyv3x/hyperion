//! The engine lifecycle error type (cross-platform).
//!
//! [`EngineError`] is returned by both the Windows-only `supervisor::Supervisor` and the
//! cross-platform [`crate::runtime::Runtime::start`], so it lives here (not behind
//! `cfg(windows)`) and is re-exported from `crate::supervisor` for back-compat. (The
//! `supervisor` module is `cfg(windows)`, so those paths are plain text to keep the Linux
//! rustdoc build link-clean.)

/// A supervisor / runtime lifecycle error.
#[derive(Debug)]
pub enum EngineError {
    /// Failed to acquire timer resolution / scheduling policy.
    Platform(String),
    /// Failed to open the physical device.
    DeviceOpen(String),
    /// Failed to create / plug the virtual ViGEm target.
    VirtualPad(String),
    /// The hot thread panicked.
    HotPanic,
}

impl std::fmt::Display for EngineError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EngineError::Platform(m) => write!(f, "platform init failed: {m}"),
            EngineError::DeviceOpen(m) => write!(f, "device open failed: {m}"),
            EngineError::VirtualPad(m) => write!(f, "virtual pad init failed: {m}"),
            EngineError::HotPanic => write!(f, "hot thread panicked"),
        }
    }
}

impl std::error::Error for EngineError {}
