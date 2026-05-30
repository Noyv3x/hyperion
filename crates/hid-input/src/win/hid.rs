//! `CreateFileW` + overlapped `ReadFile` + `GetOverlappedResult`, double-buffered.
//!
//! DESIGN §6 read pattern: keep **one** outstanding overlapped read, double-buffered. Complete
//! into `buf[cur]`, flip `cur`, re-arm the read into the *other* buffer, then parse the
//! just-completed buffer. A single shared buffer races the driver's next write against the
//! parse (verifier (a) data-race fix). The wait is either an `INFINITE`
//! `WaitForSingleObject` ([`WaitMode::Blocking`], lowest CPU) or a bounded QPC busy-poll that
//! **always** falls back to a real wait ([`WaitMode::HybridSpin`]) so a stalled report can
//! never spin a `TIME_CRITICAL` thread forever.

use crate::SourceError;

/// How the reader waits for the single outstanding overlapped read to complete.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum WaitMode {
    /// `INFINITE` `WaitForSingleObject` on the overlapped event — lowest CPU, one syscall/report.
    #[default]
    Blocking,
    /// Busy-poll `GetOverlappedResult` against a QPC deadline, then fall back to a blocking wait.
    HybridSpin {
        /// Spin budget in microseconds before falling back to a real wait (`0` ⇒ [`WaitMode::Blocking`]).
        spin_budget_us: u32,
    },
}

/// The fixed report buffer length for the DS4-compatible DualSense USB report (DESIGN §7).
pub const HID_REPORT_LEN: usize = 64;

/// Owns an open HID handle and the double-buffered single-outstanding overlapped read.
///
/// The two `bufs` are the double buffer; `cur` selects which one the *next* read targets. The
/// just-completed buffer is the *other* one, so the parser never reads the buffer the driver is
/// currently writing.
pub struct OverlappedReader {
    /// Win32 `HANDLE` to the opened device, as a raw pointer-sized value.
    ///
    /// Stored os-agnostically for M1 (no `windows` crate linked yet); the real type is
    /// `windows::Win32::Foundation::HANDLE`. `TODO(hardware)`: replace with the typed handle and
    /// close it in `Drop`.
    handle: isize,
    /// Double report buffers. `bufs[cur]` is armed for the next read; the other holds the last
    /// completed report.
    bufs: [[u8; HID_REPORT_LEN]; 2],
    /// Index of the buffer the next/in-flight read targets.
    cur: usize,
    /// Whether a read is currently armed and outstanding.
    armed: bool,
    /// How to wait for completion.
    wait_mode: WaitMode,
}

impl OverlappedReader {
    /// Open `device_path` with `FILE_FLAG_OVERLAPPED` and prepare the double buffer.
    ///
    /// `TODO(hardware)`: `CreateFileW(device_path, GENERIC_READ|GENERIC_WRITE, share, null,
    /// OPEN_EXISTING, FILE_FLAG_OVERLAPPED, null)`; create the per-buffer `OVERLAPPED` events.
    pub fn open(_device_path: &str, wait_mode: WaitMode) -> Result<Self, SourceError> {
        // Constructed-but-unused fields keep the final shape honest under `dead_code` until the
        // real Win32 body lands; reference them so M1 stays clippy-clean.
        let _stub = Self {
            handle: 0,
            bufs: [[0u8; HID_REPORT_LEN]; 2],
            cur: 0,
            armed: false,
            wait_mode,
        };
        // TODO(hardware): real CreateFileW(FILE_FLAG_OVERLAPPED); on success return `_stub`
        // after arming the first read. Until the driver bring-up, report the device as absent.
        Err(SourceError::Disconnected)
    }

    /// The wait strategy this reader was opened with.
    #[inline]
    pub fn wait_mode(&self) -> WaitMode {
        self.wait_mode
    }

    /// Arm a fresh overlapped `ReadFile` into the current buffer if none is outstanding.
    ///
    /// `TODO(hardware)`: `ReadFile(handle, &mut bufs[cur], HID_REPORT_LEN, null, &mut overlapped)`;
    /// expect `ERROR_IO_PENDING`.
    fn arm(&mut self) -> Result<(), SourceError> {
        let _ = &mut self.bufs[self.cur];
        self.armed = true;
        Ok(())
    }

    /// Wait for the outstanding read to complete and return a reference to the **completed**
    /// buffer (the one *not* selected by `cur` after the flip), having re-armed the next read.
    ///
    /// Returns `Ok(Some(&buf))` on a fresh report, `Ok(None)` on a benign timeout, `Err` on loss.
    ///
    /// `TODO(hardware)`: per [`WaitMode`], `WaitForSingleObject`/QPC-spin then
    /// `GetOverlappedResult`; on completion flip `cur`, re-arm into the new `cur`, and return the
    /// previously-completed buffer.
    pub fn read_completed(&mut self) -> Result<Option<&[u8]>, SourceError> {
        if !self.armed {
            self.arm()?;
        }
        // TODO(hardware): real overlapped wait + GetOverlappedResult, then:
        //   let done = self.cur;       // buffer that just completed
        //   self.cur ^= 1;             // flip
        //   self.armed = false; self.arm()?;   // re-arm into the other buffer
        //   return Ok(Some(&self.bufs[done]));
        let _ = &self.handle;
        Ok(None)
    }
}

impl Drop for OverlappedReader {
    fn drop(&mut self) {
        // TODO(hardware): CancelIoEx(handle), CloseHandle on the two OVERLAPPED events and the
        // device handle. No-op safe in M1 (no real handle is held).
        let _ = self.handle;
    }
}
