//! `CreateFileW` + overlapped `ReadFile` + `GetOverlappedResult`, double-buffered.
//!
//! DESIGN §6 read pattern: keep **one** outstanding overlapped read, double-buffered. Complete
//! into `buf[done]`, flip `cur`, re-arm the read into the *other* buffer, then parse the
//! just-completed buffer. A single shared buffer races the driver's next write against the
//! parse (verifier (a) data-race fix). The wait is either an `INFINITE`
//! `WaitForSingleObject` ([`WaitMode::Blocking`], lowest CPU) or a bounded QPC busy-poll that
//! **always** falls back to a real wait ([`WaitMode::HybridSpin`]) so a stalled report can
//! never spin a `TIME_CRITICAL` thread forever.

use windows::core::PCWSTR;
use windows::Win32::Foundation::{
    CloseHandle, ERROR_DEVICE_NOT_CONNECTED, ERROR_IO_INCOMPLETE, ERROR_IO_PENDING, GENERIC_READ,
    GENERIC_WRITE, HANDLE, WAIT_OBJECT_0, WAIT_TIMEOUT,
};
use windows::Win32::Storage::FileSystem::{
    CreateFileW, ReadFile, FILE_FLAG_OVERLAPPED, FILE_SHARE_READ, FILE_SHARE_WRITE, OPEN_EXISTING,
};
use windows::Win32::System::Threading::{CreateEventW, ResetEvent, WaitForSingleObject, INFINITE};
use windows::Win32::System::IO::{
    CancelIoEx, GetOverlappedResult, GetOverlappedResultEx, OVERLAPPED,
};

use crate::win::enumerate::to_wide;
use crate::win::qpc_now_ns;
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
    /// Win32 device handle opened with `FILE_FLAG_OVERLAPPED`.
    handle: HANDLE,
    /// Manual-reset event signaled by the driver when the outstanding read completes; also
    /// stored in `overlapped.hEvent`.
    event: HANDLE,
    /// The single `OVERLAPPED` driving the outstanding read. Boxed so its address is stable
    /// while the kernel holds a pointer to it across the asynchronous read.
    overlapped: Box<OVERLAPPED>,
    /// Double report buffers. `bufs[cur]` is armed for the next read; the other holds the last
    /// completed report.
    bufs: [[u8; HID_REPORT_LEN]; 2],
    /// Index of the buffer the next/in-flight read targets.
    cur: usize,
    /// Whether a read is currently armed and outstanding.
    armed: bool,
    /// Host QPC (ns) captured at the most recent completion, surfaced via [`Self::last_qpc_ns`].
    last_qpc_ns: u64,
    /// How to wait for completion.
    wait_mode: WaitMode,
}

// SAFETY: `OverlappedReader` owns its Win32 handles and is only ever touched by the single hot
// thread the engine moves it onto (the `DeviceSource: Send` contract). The raw-pointer `HANDLE`
// fields make it `!Send` by default, but there is no shared access: ownership is exclusive and
// moves are a transfer, not aliasing. No `Sync` is asserted — concurrent access is never made.
unsafe impl Send for OverlappedReader {}

impl OverlappedReader {
    /// Open `device_path` with `FILE_FLAG_OVERLAPPED`, create the completion event, and prepare
    /// the double buffer. The first read is armed lazily on the first [`Self::read_completed`].
    pub fn open(device_path: &str, wait_mode: WaitMode) -> Result<Self, SourceError> {
        let wide = to_wide(device_path);

        // SAFETY: `wide` is a NUL-terminated wide string kept alive across the call. Shared
        // read/write so we coexist with transient enumeration opens; `FILE_FLAG_OVERLAPPED`
        // makes every `ReadFile` asynchronous. The handle is validated by the `Result`.
        let handle = unsafe {
            CreateFileW(
                PCWSTR::from_raw(wide.as_ptr()),
                GENERIC_READ.0 | GENERIC_WRITE.0,
                FILE_SHARE_READ | FILE_SHARE_WRITE,
                None,
                OPEN_EXISTING,
                FILE_FLAG_OVERLAPPED,
                None,
            )
        }
        .map_err(map_open_err)?;

        // Manual-reset, initially non-signaled: the driver sets it on completion, we reset it
        // before each re-arm so a stale signal can never be mistaken for a fresh completion.
        // SAFETY: no security attributes, no name; the returned handle is validated by `Result`.
        let event = match unsafe { CreateEventW(None, true, false, PCWSTR::null()) } {
            Ok(e) => e,
            Err(e) => {
                // SAFETY: `handle` is the live device handle just opened; close it before
                // bailing so we don't leak it when the event could not be created.
                let _ = unsafe { CloseHandle(handle) };
                return Err(SourceError::Io(e.into()));
            }
        };

        let mut overlapped = Box::new(OVERLAPPED::default());
        overlapped.hEvent = event;

        Ok(Self {
            handle,
            event,
            overlapped,
            bufs: [[0u8; HID_REPORT_LEN]; 2],
            cur: 0,
            armed: false,
            wait_mode,
            last_qpc_ns: 0,
        })
    }

    /// The wait strategy this reader was opened with.
    #[inline]
    pub fn wait_mode(&self) -> WaitMode {
        self.wait_mode
    }

    /// Host QPC timestamp (ns) captured at the most recent completed read.
    ///
    /// The backend forwards this into `InputSample::host_qpc_ns`; it is also the QPC fallback
    /// the core `SensorClock` folds when the device sensor timestamp is unavailable.
    #[inline]
    pub fn last_qpc_ns(&self) -> u64 {
        self.last_qpc_ns
    }

    /// Arm a fresh overlapped `ReadFile` into the current buffer if none is outstanding.
    ///
    /// Resets the completion event, then issues the asynchronous read. `ERROR_IO_PENDING` is the
    /// expected success path (the read is now outstanding); an immediate `Ok` means the report
    /// was already buffered by the driver and the event is already signaled.
    fn arm(&mut self) -> Result<(), SourceError> {
        if self.armed {
            return Ok(());
        }
        // Reset the event so the *next* completion edge is unambiguous, and clear the OVERLAPPED
        // status words before re-issuing. The {Offset, OffsetHigh} union stays zero (HID is a
        // non-seeking device, so every read starts at offset 0).
        // SAFETY: `self.event` is the live manual-reset event we own.
        unsafe {
            let _ = ResetEvent(self.event);
        }
        self.overlapped.Internal = 0;
        self.overlapped.InternalHigh = 0;

        let cur = self.cur;
        // SAFETY: `handle` was opened with FILE_FLAG_OVERLAPPED. `bufs[cur]` lives in `self`,
        // outlives the call, and is not aliased while the read is outstanding (the parser only
        // ever touches the *other* buffer). `overlapped` is boxed so its address is stable for
        // the kernel until completion. We pass no synchronous byte-count out-pointer.
        let result = unsafe {
            ReadFile(
                self.handle,
                Some(&mut self.bufs[cur]),
                None,
                Some(&mut *self.overlapped),
            )
        };

        match result {
            Ok(()) => {
                // Completed synchronously (data already queued); the event is signaled.
                self.armed = true;
                Ok(())
            }
            Err(e) => {
                if win32_eq(&e, ERROR_IO_PENDING.0) {
                    self.armed = true;
                    Ok(())
                } else if win32_eq(&e, ERROR_DEVICE_NOT_CONNECTED.0) {
                    Err(SourceError::Disconnected)
                } else {
                    Err(SourceError::Io(e.into()))
                }
            }
        }
    }

    /// Wait for the outstanding read to complete and return a reference to the **completed**
    /// buffer (the one *not* selected by `cur` after the flip), having re-armed the next read.
    ///
    /// Returns `Ok(Some(&buf))` on a fresh report, `Ok(None)` on a benign timeout (only possible
    /// under [`WaitMode::HybridSpin`], whose bounded wait can elapse), `Err` on device loss.
    pub fn read_completed(&mut self) -> Result<Option<&[u8]>, SourceError> {
        self.arm()?;

        let done = match self.wait_mode {
            WaitMode::Blocking => self.wait_blocking()?,
            WaitMode::HybridSpin { spin_budget_us } => self.wait_hybrid(spin_budget_us)?,
        };

        match done {
            Some(()) => {
                self.last_qpc_ns = qpc_now_ns();
                let completed = self.cur;
                self.cur ^= 1;
                self.armed = false;
                self.arm()?;
                Ok(Some(&self.bufs[completed]))
            }
            None => Ok(None),
        }
    }

    /// `INFINITE` wait on the completion event, then collect the result. Always yields a report
    /// or an error (a blocking wait never benignly times out).
    fn wait_blocking(&mut self) -> Result<Option<()>, SourceError> {
        // SAFETY: `self.event` is the live manual-reset event tied to the outstanding read.
        let wait = unsafe { WaitForSingleObject(self.event, INFINITE) };
        if wait != WAIT_OBJECT_0 {
            // The only documented non-`WAIT_OBJECT_0` result for a single valid INFINITE wait is
            // `WAIT_FAILED`; surface the OS error.
            return Err(SourceError::Io(std::io::Error::last_os_error()));
        }
        self.collect(true)
    }

    /// Bounded QPC busy-poll via `GetOverlappedResultEx`, then a guaranteed fallback to a real
    /// `INFINITE` wait so a stalled report can never spin forever (DESIGN §6).
    fn wait_hybrid(&mut self, spin_budget_us: u32) -> Result<Option<()>, SourceError> {
        if spin_budget_us == 0 {
            return self.wait_blocking();
        }
        let deadline = qpc_now_ns().saturating_add(u64::from(spin_budget_us) * 1_000);
        loop {
            match self.try_collect_nowait()? {
                Some(done) => return Ok(Some(done)),
                None => {
                    if qpc_now_ns() >= deadline {
                        break;
                    }
                    std::hint::spin_loop();
                }
            }
        }
        // Spin budget exhausted: fall back to the lowest-CPU real wait.
        self.wait_blocking()
    }

    /// `GetOverlappedResult(bwait)` translating the byte count into a completion signal.
    fn collect(&mut self, bwait: bool) -> Result<Option<()>, SourceError> {
        let mut transferred: u32 = 0;
        // SAFETY: `handle`/`overlapped` are the live pair for the outstanding read; `transferred`
        // is an owned out-`u32`. With `bwait == true` this blocks until completion.
        let res =
            unsafe { GetOverlappedResult(self.handle, &*self.overlapped, &mut transferred, bwait) };
        match res {
            Ok(()) => Ok(Some(())),
            Err(e) => Err(self.map_io_err(e)),
        }
    }

    /// Non-blocking poll of the outstanding read for the hybrid spin: returns `Some(())` on
    /// completion, `None` while still pending, `Err` on loss.
    fn try_collect_nowait(&mut self) -> Result<Option<()>, SourceError> {
        let mut transferred: u32 = 0;
        // SAFETY: same live pair as `collect`; `dwMilliseconds == 0` makes this a pure poll
        // (no alertable wait) that returns immediately whether or not the read has completed.
        let res = unsafe {
            GetOverlappedResultEx(self.handle, &*self.overlapped, &mut transferred, 0, false)
        };
        match res {
            Ok(()) => Ok(Some(())),
            Err(e) => {
                // Still pending — not an error for a non-blocking poll. A zero-timeout
                // `GetOverlappedResultEx` reports an incomplete read as either
                // `ERROR_IO_INCOMPLETE` or `WAIT_TIMEOUT` depending on the path it takes.
                if win32_eq(&e, ERROR_IO_INCOMPLETE.0) || win32_eq(&e, WAIT_TIMEOUT.0) {
                    Ok(None)
                } else {
                    Err(self.map_io_err(e))
                }
            }
        }
    }

    /// Map a `GetOverlappedResult*` error to a [`SourceError`]: device removal is `Disconnected`,
    /// everything else is a wrapped I/O error.
    fn map_io_err(&self, e: windows::core::Error) -> SourceError {
        if win32_eq(&e, ERROR_DEVICE_NOT_CONNECTED.0) {
            SourceError::Disconnected
        } else {
            SourceError::Io(e.into())
        }
    }
}

impl Drop for OverlappedReader {
    fn drop(&mut self) {
        if self.armed {
            // Cancel the single outstanding read on this handle before tearing the buffers down,
            // so the kernel stops writing into `bufs` once `self` is freed.
            // SAFETY: `handle`/`overlapped` are the live pair; cancelling a not-actually-pending
            // read is harmless (it returns an error we ignore).
            unsafe {
                let _ = CancelIoEx(self.handle, Some(&*self.overlapped));
                // Drain the cancellation so the kernel is done with `*self.overlapped` before
                // the box is dropped.
                let mut transferred: u32 = 0;
                let _ = GetOverlappedResult(self.handle, &*self.overlapped, &mut transferred, true);
            }
        }
        // SAFETY: both handles were created by this reader and are closed exactly once here.
        unsafe {
            let _ = CloseHandle(self.event);
            let _ = CloseHandle(self.handle);
        }
    }
}

/// Map a `CreateFileW` error to a [`SourceError`], treating "device not connected" specially.
fn map_open_err(e: windows::core::Error) -> SourceError {
    if win32_eq(&e, ERROR_DEVICE_NOT_CONNECTED.0) {
        SourceError::Disconnected
    } else {
        SourceError::Io(e.into())
    }
}

/// Does `err` carry the given Win32 error code?
///
/// A Win32 call that fails through the `windows` bindings reports the code as an `HRESULT`
/// produced by `HRESULT::from_win32(GetLastError())`, so we compare against the same mapping.
#[inline]
fn win32_eq(err: &windows::core::Error, win32: u32) -> bool {
    err.code() == windows::core::HRESULT::from_win32(win32)
}
