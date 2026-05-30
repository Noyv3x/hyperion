//! HidHide control: hide the physical pad from everything but this process.
//!
//! DESIGN §8 lifecycle:
//! ```text
//! on start: open() -> whitelist_self() (MANDATORY, else we hide the pad from ourselves)
//!           -> blacklist_device(physical instance path) -> set_active(true)
//! on exit:  clear_blacklist() + set_active(false)   (so the pad reappears)
//! ```
//! Primary path for bring-up is the `HidHideCLI.exe` shell-out (known-good, fully documented CLI
//! surface). The direct `DeviceIoControl` IOCTL path against `\\.\HidHide` is framed below behind
//! [`HidHideBackend::Ioctl`] but its control codes are under-documented (HW-verify), so it is a
//! TODO. ViGEmBus/HidHide are EOL — isolated here so the backend is swappable.

use std::io;
use std::process::Command;

use windows::core::PCWSTR;
use windows::Win32::Foundation::{CloseHandle, GENERIC_READ, GENERIC_WRITE, HANDLE};
use windows::Win32::Storage::FileSystem::{
    CreateFileW, FILE_SHARE_READ, FILE_SHARE_WRITE, OPEN_EXISTING,
};

/// NT device path of the HidHide control device.
const HIDHIDE_DEVICE_PATH: &str = r"\\.\HidHide";

// HidHide IOCTL control codes. HW-verify: these mirror the `CTL_CODE(FILE_DEVICE_UNKNOWN, fn,
// METHOD_BUFFERED, FILE_READ_DATA)` layout used by the HidHide driver, but the exact `fn` numbers
// are not officially published and must be confirmed against the installed driver before the
// `Ioctl` backend is enabled.
//
//   CTL_CODE(t, f, m, a) = (t << 16) | (a << 14) | (f << 2) | m
//   FILE_DEVICE_UNKNOWN = 0x22, METHOD_BUFFERED = 0, FILE_READ_DATA (access) = 1
const fn ctl_code(device_type: u32, function: u32, method: u32, access: u32) -> u32 {
    (device_type << 16) | (access << 14) | (function << 2) | method
}
const FILE_DEVICE_UNKNOWN: u32 = 0x0000_0022;
const METHOD_BUFFERED: u32 = 0;
const FILE_READ_DATA: u32 = 0x0001;

/// `IOCTL_GET_WHITELIST` — read the application whitelist. HW-verify function code.
#[allow(dead_code)]
const IOCTL_GET_WHITELIST: u32 =
    ctl_code(FILE_DEVICE_UNKNOWN, 0x801, METHOD_BUFFERED, FILE_READ_DATA);
/// `IOCTL_SET_WHITELIST` — replace the application whitelist. HW-verify function code.
#[allow(dead_code)]
const IOCTL_SET_WHITELIST: u32 =
    ctl_code(FILE_DEVICE_UNKNOWN, 0x802, METHOD_BUFFERED, FILE_READ_DATA);
/// `IOCTL_GET_BLACKLIST` — read the device blacklist. HW-verify function code.
#[allow(dead_code)]
const IOCTL_GET_BLACKLIST: u32 =
    ctl_code(FILE_DEVICE_UNKNOWN, 0x803, METHOD_BUFFERED, FILE_READ_DATA);
/// `IOCTL_SET_BLACKLIST` — replace the device blacklist. HW-verify function code.
#[allow(dead_code)]
const IOCTL_SET_BLACKLIST: u32 =
    ctl_code(FILE_DEVICE_UNKNOWN, 0x804, METHOD_BUFFERED, FILE_READ_DATA);
/// `IOCTL_GET_ACTIVE` — read the cloak-active flag. HW-verify function code.
#[allow(dead_code)]
const IOCTL_GET_ACTIVE: u32 = ctl_code(FILE_DEVICE_UNKNOWN, 0x805, METHOD_BUFFERED, FILE_READ_DATA);
/// `IOCTL_SET_ACTIVE` — set the cloak-active flag. HW-verify function code.
#[allow(dead_code)]
const IOCTL_SET_ACTIVE: u32 = ctl_code(FILE_DEVICE_UNKNOWN, 0x806, METHOD_BUFFERED, FILE_READ_DATA);

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
/// For [`HidHideBackend::Ioctl`] this owns a `\\.\HidHide` control-device handle, closed on `Drop`.
/// For [`HidHideBackend::Cli`] no handle is opened and every operation shells out to `HidHideCLI`.
pub struct HidHide {
    /// Raw control-device handle (`0`/null when using the CLI backend or unopened).
    handle: isize,
    backend: HidHideBackend,
    /// Whether the cloak is currently active (for idempotent teardown).
    active: bool,
}

impl HidHide {
    /// Open the HidHide control device with the given backend.
    ///
    /// For [`HidHideBackend::Ioctl`] this `CreateFileW`s the `\\.\HidHide` control device. For
    /// [`HidHideBackend::Cli`] no handle is opened (operations shell out instead).
    pub fn open(backend: HidHideBackend) -> io::Result<Self> {
        let handle = match &backend {
            HidHideBackend::Cli { .. } => 0,
            HidHideBackend::Ioctl => {
                let path = wide_nul(HIDHIDE_DEVICE_PATH);
                // SAFETY: `path` is a NUL-terminated UTF-16 buffer living for the whole call. We
                // request shared R/W access with no security attributes / template; the returned
                // handle is owned by `self` and closed in `Drop`.
                let raw = unsafe {
                    CreateFileW(
                        PCWSTR(path.as_ptr()),
                        GENERIC_READ.0 | GENERIC_WRITE.0,
                        FILE_SHARE_READ | FILE_SHARE_WRITE,
                        None,
                        OPEN_EXISTING,
                        Default::default(),
                        None,
                    )
                }
                .map_err(|e| io::Error::new(io::ErrorKind::NotFound, e))?;
                raw.0 as isize
            }
        };
        Ok(Self {
            handle,
            backend,
            active: false,
        })
    }

    /// Whitelist this process so the cloaked device stays visible to **us**. MANDATORY before
    /// activating the cloak, or we hide the pad from ourselves.
    ///
    /// Resolves the current executable path and registers it. CLI backend: `--app-reg <self>`.
    /// IOCTL backend: `IOCTL_SET_WHITELIST` (HW-verify) — currently unimplemented.
    pub fn whitelist_self(&mut self) -> io::Result<()> {
        let exe = std::env::current_exe()?;
        let exe = exe.to_string_lossy().into_owned();
        match &self.backend {
            HidHideBackend::Cli { cli_path } => {
                run_cli(cli_path, &["--app-reg", &exe])?;
                Ok(())
            }
            HidHideBackend::Ioctl => {
                // TODO(HW-verify): IOCTL_SET_WHITELIST with this process image path.
                Err(unsupported_ioctl("whitelist_self"))
            }
        }
    }

    /// Add a device instance path to the blacklist (the physical pad to hide from other apps).
    ///
    /// CLI backend: `--dev-hide <instance_path>`. IOCTL backend: GET_BLACKLIST → append →
    /// SET_BLACKLIST (HW-verify) — currently unimplemented.
    pub fn blacklist_device(&mut self, instance_path: &str) -> io::Result<()> {
        match &self.backend {
            HidHideBackend::Cli { cli_path } => {
                run_cli(cli_path, &["--dev-hide", instance_path])?;
                Ok(())
            }
            HidHideBackend::Ioctl => {
                let _ = instance_path;
                // TODO(HW-verify): IOCTL_GET_BLACKLIST, append `instance_path`, IOCTL_SET_BLACKLIST.
                Err(unsupported_ioctl("blacklist_device"))
            }
        }
    }

    /// Replace the blacklist with the empty set so all hidden devices reappear.
    ///
    /// CLI backend: `--dev-unhide-all`. IOCTL backend: `IOCTL_SET_BLACKLIST` with an empty list
    /// (HW-verify) — currently unimplemented.
    pub fn clear_blacklist(&mut self) -> io::Result<()> {
        match &self.backend {
            HidHideBackend::Cli { cli_path } => {
                run_cli(cli_path, &["--dev-unhide-all"])?;
                Ok(())
            }
            HidHideBackend::Ioctl => {
                // TODO(HW-verify): IOCTL_SET_BLACKLIST with an empty payload.
                Err(unsupported_ioctl("clear_blacklist"))
            }
        }
    }

    /// Activate or deactivate the cloak globally.
    ///
    /// CLI backend: `--cloak-on` / `--cloak-off`. IOCTL backend: `IOCTL_SET_ACTIVE` (HW-verify) —
    /// currently unimplemented.
    pub fn set_active(&mut self, active: bool) -> io::Result<()> {
        match &self.backend {
            HidHideBackend::Cli { cli_path } => {
                let arg = if active { "--cloak-on" } else { "--cloak-off" };
                run_cli(cli_path, &[arg])?;
                self.active = active;
                Ok(())
            }
            HidHideBackend::Ioctl => {
                // TODO(HW-verify): IOCTL_SET_ACTIVE with `active as u8`.
                Err(unsupported_ioctl("set_active"))
            }
        }
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
        // intentionally ignored in Drop.
        if self.active {
            let _ = self.clear_blacklist();
            let _ = self.set_active(false);
        }
        if self.handle != 0 {
            // SAFETY: `handle` is a live `\\.\HidHide` handle from `CreateFileW` (Ioctl backend);
            // closed exactly once here. Errors ignored in Drop.
            let _ = unsafe { CloseHandle(HANDLE(self.handle as *mut core::ffi::c_void)) };
            self.handle = 0;
        }
    }
}

/// Encode a Rust `str` as a NUL-terminated UTF-16 buffer suitable for a `PCWSTR` argument.
fn wide_nul(s: &str) -> Vec<u16> {
    s.encode_utf16().chain(std::iter::once(0)).collect()
}

/// Run `HidHideCLI.exe` with `args`, mapping a non-zero exit (or spawn failure) to an `io::Error`.
fn run_cli(cli_path: &str, args: &[&str]) -> io::Result<()> {
    let status = Command::new(cli_path).args(args).status()?;
    if status.success() {
        Ok(())
    } else {
        Err(io::Error::other(format!(
            "HidHideCLI {} exited with {}",
            args.join(" "),
            status
        )))
    }
}

/// Build the "IOCTL backend not yet wired up" error for an operation.
fn unsupported_ioctl(op: &str) -> io::Error {
    io::Error::new(
        io::ErrorKind::Unsupported,
        format!("HidHide Ioctl backend not implemented for {op} (HW-verify control codes); use the Cli backend"),
    )
}
