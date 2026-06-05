//! Hot-thread scheduling policy: MMCSS + affinity + thread priority, as one RAII guard.
//!
//! DESIGN §6 (verifier (c), the BLOCKER): the policy guard MUST be **bound** for the thread's
//! life — `let _policy = sched::apply_hot_thread_policy(..);`. A bare statement drops the guard at
//! the semicolon and reverts MMCSS one line later. The policy is applied **on** the hot thread and
//! reverted **on the same thread** at exit (the Avrt handle is thread-affine; the hot thread is
//! dedicated, never pooled).
//!
//! Policy: MMCSS "Pro Audio" + `AVRT_PRIORITY_CRITICAL` primary; fall back to
//! `SetThreadPriority(TIME_CRITICAL)` under process `HIGH_PRIORITY_CLASS`. Affinity pins to a
//! **physical** core and avoids its SMT sibling (a HybridSpin loop must not saturate the GUI
//! core's sibling).

use windows::core::PCWSTR;
use windows::Win32::Foundation::HANDLE;
use windows::Win32::System::Threading::{
    AvRevertMmThreadCharacteristics, AvSetMmThreadCharacteristicsW, AvSetMmThreadPriority,
    GetCurrentThread, GetThreadPriority, SetThreadAffinityMask, SetThreadPriority,
    AVRT_PRIORITY_CRITICAL, THREAD_PRIORITY, THREAD_PRIORITY_TIME_CRITICAL,
};

/// Sentinel returned by `GetThreadPriority` on failure (`THREAD_PRIORITY_ERROR_RETURN` in the SDK;
/// not surfaced as a typed constant by the `windows` metadata, so we spell the value here).
const THREAD_PRIORITY_ERROR_RETURN: i32 = 0x7FFF_FFFF;

/// Wait strategy of the hot loop (affects whether SMT-sibling avoidance matters).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum WaitMode {
    /// Blocking `WaitForSingleObject` — does not busy-spin a core.
    #[default]
    Blocking,
    /// Bounded busy-poll then block — saturates its core, so SMT-sibling isolation matters.
    HybridSpin,
}

/// Configuration for the hot-thread policy.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HotThreadConfig {
    /// Logical core to pin the hot thread to. `None` ⇒ auto-detect a physical core whose SMT
    /// sibling is left free.
    pub hot_core: Option<usize>,
    /// Whether to register with MMCSS.
    pub use_mmcss: bool,
    /// MMCSS task name (e.g. "Pro Audio").
    pub mmcss_task: String,
    /// The hot loop's wait strategy.
    pub wait_mode: WaitMode,
}

impl Default for HotThreadConfig {
    fn default() -> Self {
        Self {
            hot_core: None,
            use_mmcss: true,
            mmcss_task: "Pro Audio".to_owned(),
            wait_mode: WaitMode::Blocking,
        }
    }
}

/// RAII guard holding the hot-thread MMCSS/affinity/priority policy for the calling thread's life.
///
/// MUST be bound to a named local for the whole hot loop. On `Drop` (same thread) it reverts the
/// thread priority, releases the MMCSS task handle, and restores the prior affinity mask. No-op
/// safe when nothing was applied.
#[must_use = "bind this guard for the hot thread's life; dropping it reverts the policy"]
pub struct HotPolicyGuard {
    /// MMCSS task handle (`AvSetMmThreadCharacteristicsW`), raw; `0` if not registered.
    mmcss_handle: usize,
    /// Previous affinity mask to restore, or `0` if affinity was not changed.
    prev_affinity: usize,
    /// Previous thread priority to restore, when `priority_raised`.
    prev_priority: i32,
    /// Whether the thread priority was raised (and must be reverted).
    priority_raised: bool,
    /// Whether MMCSS registration succeeded (`AvRevertMmThreadCharacteristics` on drop).
    mmcss_active: bool,
}

impl HotPolicyGuard {
    /// Whether MMCSS registration is currently held.
    #[inline]
    pub fn mmcss_active(&self) -> bool {
        self.mmcss_active
    }
}

/// Encode a Rust `str` as a NUL-terminated UTF-16 buffer suitable for a `PCWSTR` argument.
fn wide_nul(s: &str) -> Vec<u16> {
    s.encode_utf16().chain(std::iter::once(0)).collect()
}

/// Apply the hot-thread policy described by `cfg` to the **current** (hot) thread.
///
/// Returns a [`HotPolicyGuard`] that reverts everything on `Drop`. Call it on the hot thread and
/// keep the guard bound for the loop's lifetime.
///
/// Order: affinity first (so MMCSS/priority decisions land on the pinned core), then MMCSS
/// ("Pro Audio" + `AVRT_PRIORITY_CRITICAL`); if MMCSS is disabled or fails, fall back to
/// `SetThreadPriority(TIME_CRITICAL)`.
pub fn apply_hot_thread_policy(cfg: &HotThreadConfig) -> HotPolicyGuard {
    // SAFETY: `GetCurrentThread` returns the current-thread pseudo-handle; always valid, no close.
    let thread: HANDLE = unsafe { GetCurrentThread() };

    let mut guard = HotPolicyGuard {
        mmcss_handle: 0,
        prev_affinity: 0,
        prev_priority: 0,
        priority_raised: false,
        mmcss_active: false,
    };

    // --- Affinity: pin to the configured logical core, else auto-select a physical core. ---
    // `Some(n)` honours an explicit pin exactly as before. `None` asks the topology layer for a
    // physical core whose SMT sibling is free (avoiding logical CPU 0 / the GUI core); if that query
    // yields nothing we leave the thread unpinned — the pre-M7 behaviour for the `None` case.
    let core = cfg.hot_core.or_else(crate::topology::auto_select_core);
    if let Some(core) = core {
        // A mask must fit the affinity word (64 logical CPUs on 64-bit); ignore out-of-range cores.
        if core < usize::BITS as usize {
            let mask: usize = 1usize << core;
            // SAFETY: valid thread pseudo-handle; `mask` is a non-zero affinity mask. Returns the
            // previous mask, or `0` on failure (in which case we leave affinity untouched).
            let prev = unsafe { SetThreadAffinityMask(thread, mask) };
            if prev != 0 {
                guard.prev_affinity = prev;
            }
        }
    }

    // --- MMCSS primary path. ---
    if cfg.use_mmcss {
        let task = wide_nul(&cfg.mmcss_task);
        let mut task_index: u32 = 0;
        // SAFETY: `task` is a NUL-terminated UTF-16 buffer that outlives the call; `task_index` is
        // a valid out-pointer. On success we own the returned handle and must revert it on the
        // same thread in `Drop`.
        let handle =
            unsafe { AvSetMmThreadCharacteristicsW(PCWSTR(task.as_ptr()), &mut task_index) };
        if let Ok(h) = handle {
            if !h.is_invalid() {
                guard.mmcss_handle = h.0 as usize;
                guard.mmcss_active = true;
                // SAFETY: `h` is the live MMCSS handle just returned; `AVRT_PRIORITY_CRITICAL` is a
                // documented Avrt priority. Failure here is non-fatal — MMCSS membership alone
                // already grants the boost.
                let _ = unsafe { AvSetMmThreadPriority(h, AVRT_PRIORITY_CRITICAL) };
            }
        }
    }

    // --- Fallback: raise raw thread priority when MMCSS did not take. ---
    if !guard.mmcss_active {
        // SAFETY: valid thread pseudo-handle. `GetThreadPriority` returns
        // `THREAD_PRIORITY_ERROR_RETURN` on failure.
        let prev = unsafe { GetThreadPriority(thread) };
        if prev != THREAD_PRIORITY_ERROR_RETURN {
            // SAFETY: valid thread pseudo-handle; `THREAD_PRIORITY_TIME_CRITICAL` is in range.
            if unsafe { SetThreadPriority(thread, THREAD_PRIORITY_TIME_CRITICAL) }.is_ok() {
                guard.prev_priority = prev;
                guard.priority_raised = true;
            }
        }
    }

    guard
}

impl Drop for HotPolicyGuard {
    fn drop(&mut self) {
        // SAFETY: current-thread pseudo-handle; the hot thread is dedicated, so `Drop` runs on the
        // same thread that applied the policy (required for the thread-affine Avrt handle).
        let thread = unsafe { GetCurrentThread() };

        // Each revert is independently guarded so a partially-applied policy unwinds cleanly.
        if self.mmcss_active {
            // SAFETY: `mmcss_handle` is the live handle from `AvSetMmThreadCharacteristicsW`,
            // reverted on its owning thread. Errors are ignored in `Drop`.
            let h = HANDLE(self.mmcss_handle as *mut core::ffi::c_void);
            let _ = unsafe { AvRevertMmThreadCharacteristics(h) };
            self.mmcss_active = false;
        }
        if self.priority_raised {
            // SAFETY: valid thread pseudo-handle; `prev_priority` was read back from
            // `GetThreadPriority`. Errors ignored in `Drop`.
            let _ = unsafe { SetThreadPriority(thread, THREAD_PRIORITY(self.prev_priority)) };
            self.priority_raised = false;
        }
        if self.prev_affinity != 0 {
            // SAFETY: valid thread pseudo-handle; `prev_affinity` is the mask previously returned
            // by `SetThreadAffinityMask`. Errors ignored in `Drop`.
            let _ = unsafe { SetThreadAffinityMask(thread, self.prev_affinity) };
            self.prev_affinity = 0;
        }
    }
}
