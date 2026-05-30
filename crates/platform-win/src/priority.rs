//! Process priority class.
//!
//! DESIGN §6: run under `HIGH_PRIORITY_CLASS` (the fallback substrate for the MMCSS hot-thread
//! policy). **Never** `REALTIME_PRIORITY_CLASS` — it starves the OS (including the input stack we
//! depend on). The change is process-wide and reverted on `Drop`.

use windows::Win32::System::Threading::{
    GetCurrentProcess, GetPriorityClass, SetPriorityClass, HIGH_PRIORITY_CLASS,
    PROCESS_CREATION_FLAGS,
};

/// RAII guard that holds `HIGH_PRIORITY_CLASS` for its lifetime and restores the previous class
/// on `Drop`. A no-op-safe `Drop` when the raise never took effect.
#[must_use = "dropping the guard restores the previous process priority class"]
pub struct PriorityClassGuard {
    /// The previous priority class value captured at apply time.
    previous_class: u32,
    /// Whether the change was applied and must be reverted.
    active: bool,
}

impl PriorityClassGuard {
    /// Whether `HIGH_PRIORITY_CLASS` is currently in effect via this guard.
    #[inline]
    pub fn is_active(&self) -> bool {
        self.active
    }
}

/// Raise the current process to `HIGH_PRIORITY_CLASS`, returning a guard that restores the
/// previous class on `Drop`.
///
/// Captures the current class via `GetPriorityClass(GetCurrentProcess())`, then sets
/// `HIGH_PRIORITY_CLASS`. If the capture or the set fails the returned guard is inactive (its
/// `Drop` is a no-op). The priority is **never** raised to `REALTIME_PRIORITY_CLASS`.
pub fn set_high_priority_class() -> PriorityClassGuard {
    // SAFETY: `GetCurrentProcess` returns the pseudo-handle for the current process; it is always
    // valid and needs no close.
    let process = unsafe { GetCurrentProcess() };

    // SAFETY: `process` is the valid current-process pseudo-handle. `GetPriorityClass` returns `0`
    // only on failure, which we treat as "do not raise".
    let previous_class = unsafe { GetPriorityClass(process) };
    if previous_class == 0 {
        return PriorityClassGuard {
            previous_class: 0,
            active: false,
        };
    }

    // Already at or above HIGH? Avoid a redundant set (and a Drop that would lower it). We still
    // capture the original so Drop is a no-op.
    if previous_class == HIGH_PRIORITY_CLASS.0 {
        return PriorityClassGuard {
            previous_class,
            active: false,
        };
    }

    // SAFETY: valid process handle; `HIGH_PRIORITY_CLASS` is a documented, in-range priority class.
    match unsafe { SetPriorityClass(process, HIGH_PRIORITY_CLASS) } {
        Ok(()) => PriorityClassGuard {
            previous_class,
            active: true,
        },
        Err(_) => PriorityClassGuard {
            previous_class,
            active: false,
        },
    }
}

impl Drop for PriorityClassGuard {
    fn drop(&mut self) {
        if !self.active {
            return;
        }
        // SAFETY: current-process pseudo-handle; `previous_class` was read back from
        // `GetPriorityClass`, so it is a valid class value. Errors are ignored in `Drop`.
        unsafe {
            let process = GetCurrentProcess();
            let _ = SetPriorityClass(process, PROCESS_CREATION_FLAGS(self.previous_class));
        }
        self.active = false;
    }
}
