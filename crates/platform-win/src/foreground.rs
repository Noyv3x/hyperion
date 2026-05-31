//! Foreground-window probe for the auto-profile-switch watcher (DESIGN §7.4, §12 M5).
//!
//! [`foreground_app`] snapshots the current foreground window's owning-process **executable
//! basename** (lower-cased) and **window title**. The engine's `ForegroundWatcher` thread polls
//! this at `auto_switch.poll_hz` (~4 Hz) and feeds the result to
//! [`hyperion_core::autoswitch::match_rules`]. It is **never** called from the hot path — only the
//! resulting `ControlMsg::SetActiveProfile` (and the generation bump it causes) ever reaches the
//! latency-critical loop.
//!
//! ## Engine contract — [`ForegroundApp`]
//!
//! `foreground_app() -> Option<ForegroundApp>` returns:
//! * `Some(ForegroundApp { exe, title })` when a foreground window exists and its process image
//!   path could be read. `exe` is the **lower-cased file-name component** of the process image
//!   path (e.g. `valorant.exe`, never a full path) so it feeds the matcher's case-insensitive
//!   substring test directly; `title` is the window caption verbatim (may be empty for a titleless
//!   window). Both are owned `String`s.
//! * `None` when there is no foreground window (e.g. during a desktop switch / lock screen), or
//!   when the owning process could not be opened/queried (an elevated foreground app cannot be
//!   opened by a non-elevated Hyperion → treat as "unknown", keep the current profile). The matcher
//!   already maps "unknown foreground" (empty/absent fields) to "no rule matched → keep current
//!   profile", so `None` here is the correct, side-effect-free signal.
//!
//! The lower-casing matches the matcher's ASCII-case-insensitive comparison; we still lower-case
//! here so rules authored against, say, `Valorant.exe` and a process reporting `VALORANT.EXE` agree
//! regardless of how the comparison is later refined.

use windows::core::PWSTR;
use windows::Win32::Foundation::{CloseHandle, HANDLE, HWND};
use windows::Win32::System::Threading::{
    OpenProcess, QueryFullProcessImageNameW, PROCESS_NAME_WIN32, PROCESS_QUERY_LIMITED_INFORMATION,
};
use windows::Win32::UI::WindowsAndMessaging::{
    GetForegroundWindow, GetWindowTextLengthW, GetWindowTextW, GetWindowThreadProcessId,
};

/// A snapshot of the foreground window for auto-profile matching (DESIGN §7.4).
///
/// See the module docs for the full contract. `exe` is the **lower-cased basename** of the
/// foreground process's image path; `title` is the window caption (possibly empty).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ForegroundApp {
    /// Lower-cased file-name component of the foreground process image path (e.g. `valorant.exe`).
    pub exe: String,
    /// The foreground window's title/caption text (verbatim; may be empty).
    pub title: String,
}

/// Upper bound on a Win32 path in UTF-16 code units (`MAX_PATH` is `260`, but long paths can be
/// longer; `QueryFullProcessImageNameW` truncates to the buffer, so this is generous headroom).
const IMAGE_PATH_CAP: usize = 1024;

/// Snapshot the current foreground window's process basename (lower-cased) and title.
///
/// Returns `None` when there is no foreground window or the owning process cannot be opened/queried
/// (e.g. an elevated process); see the module docs for why `None` is the correct "keep current
/// profile" signal. Never panics and performs no work on the hot path.
#[must_use]
pub fn foreground_app() -> Option<ForegroundApp> {
    // SAFETY: `GetForegroundWindow` takes no arguments and returns the foreground window handle (or
    // a null `HWND` when there is no foreground window — e.g. during a desktop switch). It is always
    // safe to call.
    let hwnd = unsafe { GetForegroundWindow() };
    if hwnd.is_invalid() {
        return None;
    }

    let pid = foreground_pid(hwnd)?;
    let exe = process_basename_lowercased(pid)?;
    let title = window_title(hwnd);
    Some(ForegroundApp { exe, title })
}

/// Resolve the owning process id of `hwnd`, or `None` if the window has no associated process.
fn foreground_pid(hwnd: HWND) -> Option<u32> {
    let mut pid: u32 = 0;
    // SAFETY: `hwnd` is a non-null window handle from `GetForegroundWindow`. We pass a valid,
    // aligned `*mut u32` out-pointer that lives for the whole call; the callee only writes the
    // owning process id through it. The return value is the owning thread id (0 on failure).
    let thread_id = unsafe { GetWindowThreadProcessId(hwnd, Some(&mut pid)) };
    if thread_id == 0 || pid == 0 {
        return None;
    }
    Some(pid)
}

/// Open `pid` for a limited image-name query and return the **lower-cased basename** of its image
/// path, or `None` if the process cannot be opened (elevated/exited) or queried.
fn process_basename_lowercased(pid: u32) -> Option<String> {
    // SAFETY: `PROCESS_QUERY_LIMITED_INFORMATION` is the minimal access right that lets a
    // non-elevated caller read another process's image name. `binherithandle = false`; `pid` is a
    // plain value. On failure (e.g. the target is elevated and we are not) this returns `Err`, which
    // we map to `None`. On success we own the returned handle and must close it (see below).
    let handle = unsafe { OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, false, pid) }.ok()?;
    if handle.is_invalid() {
        return None;
    }

    let result = query_image_path(handle);

    // SAFETY: `handle` is the live process handle returned by `OpenProcess`, closed exactly once
    // here regardless of the query outcome. Errors are irrelevant for a read-only query handle.
    unsafe {
        let _ = CloseHandle(handle);
    }

    let path = result?;
    Some(basename_lowercased(&path))
}

/// Read the full Win32 image path of an open process handle as a `String`, or `None` on failure.
fn query_image_path(handle: HANDLE) -> Option<String> {
    let mut buf = [0u16; IMAGE_PATH_CAP];
    // In/out: on entry the buffer capacity in code units, on return the number of code units
    // written (excluding the terminating NUL).
    let mut size: u32 = buf.len() as u32;

    // SAFETY: `handle` is a valid process handle opened with `PROCESS_QUERY_LIMITED_INFORMATION`.
    // `PWSTR` wraps a pointer to `buf`, which has `IMAGE_PATH_CAP` code units and outlives the call;
    // `size` is a valid in/out `*mut u32` initialised to that capacity. The callee writes at most
    // `size` code units into `buf` and updates `size` to the written length. `PROCESS_NAME_WIN32`
    // asks for a drive-letter path (not the `\Device\...` native form).
    let ok = unsafe {
        QueryFullProcessImageNameW(
            handle,
            PROCESS_NAME_WIN32,
            PWSTR(buf.as_mut_ptr()),
            &mut size,
        )
    }
    .is_ok();
    if !ok {
        return None;
    }

    let len = (size as usize).min(buf.len());
    if len == 0 {
        return None;
    }
    Some(String::from_utf16_lossy(&buf[..len]))
}

/// Read `hwnd`'s window title. Returns an empty `String` for a titleless window or on failure
/// (an absent title is a valid, non-fatal state — the matcher treats an empty title as "don't
/// care").
fn window_title(hwnd: HWND) -> String {
    // SAFETY: `hwnd` is a non-null foreground window handle. `GetWindowTextLengthW` returns the
    // title length in characters (0 for a titleless window or on error).
    let len = unsafe { GetWindowTextLengthW(hwnd) };
    if len <= 0 {
        return String::new();
    }

    // +1 for the terminating NUL that `GetWindowTextW` always writes.
    let mut buf = vec![0u16; (len as usize) + 1];
    // SAFETY: `hwnd` is valid; `buf` is a mutable slice with room for the title plus its NUL
    // terminator. This high-level binding takes the slice directly and returns the number of
    // characters copied (excluding the NUL); it never writes past `buf`.
    let copied = unsafe { GetWindowTextW(hwnd, &mut buf) };
    if copied <= 0 {
        return String::new();
    }
    String::from_utf16_lossy(&buf[..copied as usize])
}

/// Lower-cased final path component of a Win32 path, handling both `\` and `/` separators.
///
/// Returns the whole input (lower-cased) when there is no separator. The lower-casing matches the
/// auto-switch matcher's ASCII-case-insensitive comparison so rules and the live foreground exe
/// agree on case.
fn basename_lowercased(path: &str) -> String {
    let base = path.rsplit(['\\', '/']).next().unwrap_or(path);
    base.to_lowercase()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basename_handles_backslash_paths() {
        assert_eq!(
            basename_lowercased(r"C:\Riot Games\VALORANT\live\VALORANT.exe"),
            "valorant.exe"
        );
    }

    #[test]
    fn basename_handles_forward_slash_paths() {
        assert_eq!(basename_lowercased("C:/Games/CSGO.EXE"), "csgo.exe");
    }

    #[test]
    fn basename_without_separator_is_lowercased_whole() {
        assert_eq!(basename_lowercased("Notepad.EXE"), "notepad.exe");
    }

    #[test]
    fn basename_empty_is_empty() {
        assert_eq!(basename_lowercased(""), "");
    }

    /// Smoke test: `foreground_app()` must not panic and returns `Some`/`None` either way. We do
    /// NOT assert on content — the CI runner has no stable foreground window (it may be headless or
    /// session-0), so any concrete exe/title would be flaky. This only exercises the unsafe path for
    /// soundness (ASAN/Miri-style "does it run").
    #[test]
    fn foreground_app_does_not_panic() {
        let _ = foreground_app();
    }
}
