//! System timer resolution, raised for the hot loop and restored on `Drop`.
//!
//! DESIGN §6 (verifier (e)): pick **one** mechanism. Use `NtSetTimerResolution` only; capture the
//! original resolution with `NtQueryTimerResolution` at begin and restore it with `(orig, FALSE)`
//! on `Drop`. Do **not** also call `timeBeginPeriod` (coarser, redundant, and its Drop would leak
//! the Nt request). The guard is owned by the supervisor, set **before** the hot thread spawns and
//! dropped **after** `hot.join()`.

// The undocumented ntdll timer exports are not part of the `windows` typed surface in a
// stable-feature module, so we declare them ourselves. The prototypes are the long-standing,
// well-known native API shapes (see ntinternals / ntdoc):
//
//   NTSTATUS NtQueryTimerResolution(PULONG Maximum, PULONG Minimum, PULONG Current);
//   NTSTATUS NtSetTimerResolution(ULONG DesiredTime, BOOLEAN SetResolution, PULONG ActualTime);
//
// All three `PULONG` out-params and the in `DesiredTime` are in 100 ns units; `BOOLEAN` is a `u8`
// (1 = TRUE = raise, 0 = FALSE = restore). `NTSTATUS` >= 0 means success.
//
// SAFETY: `ntdll.dll` is always loaded into every Windows process, so these imports always resolve
// at load time; the signatures match the documented native API exactly.
#[link(name = "ntdll")]
extern "system" {
    fn NtQueryTimerResolution(
        maximum_time: *mut u32,
        minimum_time: *mut u32,
        current_time: *mut u32,
    ) -> i32;

    fn NtSetTimerResolution(desired_time: u32, set_resolution: u8, actual_time: *mut u32) -> i32;
}

/// RAII guard that holds a raised system timer resolution for its lifetime.
///
/// While alive, the requested resolution (e.g. 0.5 ms) is in effect process-wide. On `Drop` the
/// original resolution captured at [`begin_timer_resolution`] is restored. If the raise never
/// actually took effect (the Nt call failed), `Drop` is a safe no-op.
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
/// Returns a [`TimerResGuard`] that restores the previous resolution on `Drop`. The original
/// resolution is captured with `NtQueryTimerResolution` and the raise is requested with
/// `NtSetTimerResolution(target_100ns, TRUE, &mut actual)`. If either Nt call fails the returned
/// guard is inactive (its `Drop` is a no-op) and `is_active()` reports `false`.
pub fn begin_timer_resolution(target_us: u32) -> TimerResGuard {
    let requested_100ns = target_us.saturating_mul(10);

    let mut maximum = 0u32;
    let mut minimum = 0u32;
    let mut current = 0u32;

    // SAFETY: out-pointers are valid, aligned, non-null locals living for the whole call; the
    // callee only writes through them. The signature matches the native API.
    let query_status = unsafe { NtQueryTimerResolution(&mut maximum, &mut minimum, &mut current) };
    if query_status < 0 {
        // Could not read the baseline — do not touch resolution, hand back an inactive guard.
        return TimerResGuard {
            original_100ns: 0,
            requested_100ns,
            active: false,
        };
    }

    // The achievable resolution is bounded by `minimum` (the smallest period the platform allows,
    // i.e. the finest resolution). Clamp the request so we never ask for finer than supported.
    // HW-verify: on real hardware `minimum` is typically 5000 (0.5 ms); clamping here keeps the
    // request honest if a platform caps coarser.
    let clamped_100ns = requested_100ns.max(minimum);

    let mut actual = 0u32;
    // SAFETY: `actual` is a valid out-pointer; `clamped_100ns`/`1u8` are plain values. Raising the
    // timer resolution is process-global and reference-counted by the kernel; the matching restore
    // happens in `Drop`.
    let set_status = unsafe { NtSetTimerResolution(clamped_100ns, 1, &mut actual) };
    if set_status < 0 {
        return TimerResGuard {
            original_100ns: 0,
            requested_100ns,
            active: false,
        };
    }

    TimerResGuard {
        original_100ns: current,
        requested_100ns: clamped_100ns,
        active: true,
    }
}

impl Drop for TimerResGuard {
    fn drop(&mut self) {
        if !self.active {
            return;
        }
        let mut actual = 0u32;
        // SAFETY: `actual` is a valid out-pointer; `0u8` (FALSE) releases this process's raise of
        // the resolution back toward `original_100ns`. Errors are ignored in `Drop`.
        let _ = unsafe { NtSetTimerResolution(self.original_100ns, 0, &mut actual) };
        self.active = false;
    }
}
