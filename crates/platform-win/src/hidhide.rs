//! HidHide control: hide the physical pad from everything but this process.
//!
//! DESIGN §8 lifecycle:
//! ```text
//! on start: open() -> whitelist_self() (MANDATORY, else we hide the pad from ourselves)
//!           -> blacklist_device(physical instance path) -> set_active(true)
//! on exit:  clear_blacklist() + set_active(false)   (so the pad reappears)
//! ```
//! Primary path is direct `DeviceIoControl` IOCTLs against `\\.\HidHide`; the
//! `CTL_CODE(FILE_DEVICE_UNKNOWN, 0x80x, METHOD_BUFFERED, FILE_READ_DATA)` GET/SET
//! WHITELIST/BLACKLIST/ACTIVE codes are under-documented (HW-verify), so a `HidHideCLI.exe`
//! shell-out is the known-good bring-up fallback behind a config flag. ViGEmBus/HidHide are EOL —
//! isolated here so the backend is swappable.

use std::io;

/// Whether to drive HidHide via the kernel IOCTL interface or the CLI fallback.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub enum HidHideBackend {
    /// Direct `DeviceIoControl` against the HidHide control device.
    #[default]
    Ioctl,
    /// Shell out to `HidHideCLI.exe` at the given path (bring-up / known-good fallback).
    Cli { cli_path: String },
}

/// An open handle to the HidHide control device (or a configured CLI fallback).
///
/// The `handle` is stored os-agnostically for M1 (no `windows` crate linked yet); the real type is
/// a `windows::Win32::Foundation::HANDLE` to `\\.\HidHide`. `TODO(hardware)`: open it in
/// [`HidHide::open`] and close it in `Drop`.
pub struct HidHide {
    /// Raw control-device handle (`0` when using the CLI backend or unopened).
    handle: isize,
    backend: HidHideBackend,
    /// Whether the cloak is currently active (for idempotent teardown).
    active: bool,
}

impl HidHide {
    /// Open the HidHide control device with the given backend.
    ///
    /// `TODO(hardware)`: for [`HidHideBackend::Ioctl`], `CreateFileW("\\\\.\\HidHide", ..)`.
    pub fn open(backend: HidHideBackend) -> io::Result<Self> {
        // TODO(hardware): real CreateFileW for the Ioctl backend; CLI backend needs no handle.
        Ok(Self {
            handle: 0,
            backend,
            active: false,
        })
    }

    /// Whitelist this process so the cloaked device stays visible to **us**. MANDATORY before
    /// activating the cloak, or we hide the pad from ourselves.
    ///
    /// `TODO(hardware)`: SET_WHITELIST IOCTL with this process's image path (or
    /// `HidHideCLI --app-reg <self>`).
    pub fn whitelist_self(&mut self) -> io::Result<()> {
        let _ = (&self.handle, &self.backend);
        // TODO(hardware): resolve current exe path; SET_WHITELIST IOCTL / CLI --app-reg.
        Ok(())
    }

    /// Add a device instance path to the blacklist (the physical pad to hide from other apps).
    ///
    /// `TODO(hardware)`: GET_BLACKLIST → append `instance_path` → SET_BLACKLIST IOCTL (or
    /// `HidHideCLI --dev-hide <instance_path>`).
    pub fn blacklist_device(&mut self, instance_path: &str) -> io::Result<()> {
        let _ = (&self.handle, &self.backend, instance_path);
        Ok(())
    }

    /// Replace the blacklist with the empty set so all hidden devices reappear.
    ///
    /// `TODO(hardware)`: SET_BLACKLIST IOCTL with an empty list (or `HidHideCLI --dev-unhide-all`).
    pub fn clear_blacklist(&mut self) -> io::Result<()> {
        let _ = (&self.handle, &self.backend);
        Ok(())
    }

    /// Activate or deactivate the cloak globally.
    ///
    /// `TODO(hardware)`: SET_ACTIVE IOCTL with `active` (or `HidHideCLI --cloak-on/--cloak-off`).
    pub fn set_active(&mut self, active: bool) -> io::Result<()> {
        let _ = (&self.handle, &self.backend);
        self.active = active;
        Ok(())
    }

    /// Whether the cloak is currently active.
    #[inline]
    pub fn is_active(&self) -> bool {
        self.active
    }
}

impl Drop for HidHide {
    fn drop(&mut self) {
        // Best-effort teardown so the physical pad reappears even on an unclean exit. Errors are
        // intentionally ignored in Drop. No-op safe in M1 (no real handle / no active cloak).
        if self.active {
            let _ = self.clear_blacklist();
            let _ = self.set_active(false);
        }
        // TODO(hardware): CloseHandle(self.handle) for the Ioctl backend.
        let _ = self.handle;
    }
}
