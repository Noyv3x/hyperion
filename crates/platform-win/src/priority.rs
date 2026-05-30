//! Process priority class.
//!
//! DESIGN §6: run under `HIGH_PRIORITY_CLASS` (the fallback substrate for the MMCSS hot-thread
//! policy). **Never** `REALTIME_PRIORITY_CLASS` — it starves the OS (including the input stack we
//! depend on). The change is process-wide and reverted on `Drop`.

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
/// `TODO(hardware)`: `GetPriorityClass(GetCurrentProcess())` to capture `previous_class`, then
/// `SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS)`.
pub fn set_high_priority_class() -> PriorityClassGuard {
    // TODO(hardware): capture + set; mark active=true on success.
    PriorityClassGuard {
        previous_class: 0,
        active: false,
    }
}

impl Drop for PriorityClassGuard {
    fn drop(&mut self) {
        if !self.active {
            return;
        }
        // TODO(hardware): SetPriorityClass(GetCurrentProcess(), self.previous_class).
        let _ = self.previous_class;
        self.active = false;
    }
}
