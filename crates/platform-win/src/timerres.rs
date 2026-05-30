//! System timer resolution, raised for the hot loop and restored on `Drop`.
//!
//! DESIGN §6 (verifier (e)): pick **one** mechanism. Use `NtSetTimerResolution` only; capture the
//! original resolution with `NtQueryTimerResolution` at begin and restore it with `(orig, FALSE)`
//! on `Drop`. Do **not** also call `timeBeginPeriod` (coarser, redundant, and its Drop would leak
//! the Nt request). The guard is owned by the supervisor, set **before** the hot thread spawns and
//! dropped **after** `hot.join()`.

/// RAII guard that holds a raised system timer resolution for its lifetime.
///
/// While alive, the requested resolution (e.g. 0.5 ms) is in effect process-wide. On `Drop` the
/// original resolution captured at [`begin_timer_resolution`] is restored. If the raise never
/// actually took effect (M1 stub, or the Nt call failed), `Drop` is a safe no-op.
#[must_use = "dropping the guard immediately restores the previous timer resolution"]
pub struct TimerResGuard {
    /// Original resolution in 100 ns units, captured via `NtQueryTimerResolution`.
    original_100ns: u32,
    /// The resolution we requested, in 100 ns units (for diagnostics).
    requested_100ns: u32,
    /// Whether the raise was actually applied and must be reverted on `Drop`.
    active: bool,
}

impl TimerResGuard {
    /// The original (pre-raise) timer resolution in microseconds, or `None` if not captured.
    #[inline]
    pub fn original_us(&self) -> Option<f64> {
        if self.active {
            Some(self.original_100ns as f64 / 10.0)
        } else {
            None
        }
    }

    /// The requested resolution in microseconds.
    #[inline]
    pub fn requested_us(&self) -> f64 {
        self.requested_100ns as f64 / 10.0
    }

    /// Whether the raise is currently in effect.
    #[inline]
    pub fn is_active(&self) -> bool {
        self.active
    }
}

/// Raise the system timer resolution to (at most) `target_us` microseconds.
///
/// Returns a [`TimerResGuard`] that restores the previous resolution on `Drop`. In M1 the Nt calls
/// are stubbed, so the returned guard is inactive (its `Drop` is a no-op) but has the final shape.
///
/// `TODO(hardware)`: `NtQueryTimerResolution(&min, &max, &cur)` to capture `cur` into
/// `original_100ns`, then `NtSetTimerResolution(target_100ns, TRUE, &actual)`.
pub fn begin_timer_resolution(target_us: u32) -> TimerResGuard {
    let requested_100ns = target_us.saturating_mul(10);
    // TODO(hardware): capture original via NtQueryTimerResolution and set active=true on success.
    TimerResGuard {
        original_100ns: 0,
        requested_100ns,
        active: false,
    }
}

impl Drop for TimerResGuard {
    fn drop(&mut self) {
        if !self.active {
            return;
        }
        // TODO(hardware): NtSetTimerResolution(self.original_100ns, FALSE, &mut actual) to restore.
        self.active = false;
    }
}
