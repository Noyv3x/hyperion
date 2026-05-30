//! Host clock + dt tracking for the hot loop. Pure and Linux-testable.
//!
//! The dt-compensated Ultimate filter needs a guarded real elapsed time per report. The
//! authoritative source is the DualSense hardware timestamp folded by
//! [`hyperion_core::input::SensorClock`]; the host monotonic clock supplies the QPC fallback
//! (identical hardware stamps) and the first interval before the device stamp is trusted.
//!
//! On Windows the "QPC" timestamp is `QueryPerformanceCounter`; here we use
//! [`std::time::Instant`], which is QPC-backed on Windows and a monotonic clock elsewhere, so
//! [`DtTracker`] builds and tests identically on Linux. The engine captures
//! [`DtTracker::now_qpc_ns`] at read-completion and folds it together with the device stamp.

use std::time::Instant;

use hyperion_core::dt::Dt;
use hyperion_core::input::SensorClock;

/// Tracks per-report elapsed time, folding the device hardware timestamp against a host
/// monotonic clock and clamping the result into the [`Dt`] guard window.
///
/// Construction captures the clock origin; [`now_qpc_ns`](DtTracker::now_qpc_ns) returns
/// nanoseconds since that origin, the host-clock analogue of a raw QPC tick count.
pub struct DtTracker {
    origin: Instant,
    sensor: SensorClock,
    /// Mirrors whether [`SensorClock`] has folded a report yet, so the caller can tell a
    /// prime (seed history, no IIR step) from a real step without reaching into the core
    /// type's private fields.
    primed: bool,
}

impl DtTracker {
    /// Start a fresh tracker with no prior report (the next [`fold`](DtTracker::fold) primes).
    pub fn new() -> Self {
        Self {
            origin: Instant::now(),
            sensor: SensorClock::default(),
            primed: false,
        }
    }

    /// Host monotonic timestamp in nanoseconds since this tracker's origin.
    ///
    /// QPC-equivalent: backed by `QueryPerformanceCounter` on Windows and a monotonic clock
    /// on other platforms. Wall-clock magnitude is irrelevant — only deltas are used.
    #[inline]
    pub fn now_qpc_ns(&self) -> u64 {
        self.origin.elapsed().as_nanos() as u64
    }

    /// Fold a device hardware timestamp (DS `u16` tick) and a host timestamp into a guarded
    /// elapsed time, returning `(dt, is_prime)`.
    ///
    /// On the very first report there is no previous stamp, so `is_prime` is `true` and the
    /// caller must **not** advance the IIR (it emits the input unchanged); subsequent reports
    /// return `is_prime == false` and a guarded [`Dt`].
    #[inline]
    pub fn fold(&mut self, sensor_ts: u16, host_qpc_ns: u64) -> (Dt, bool) {
        let is_prime = !self.primed;
        let dt_us = self.sensor.fold(sensor_ts, host_qpc_ns);
        self.primed = true;
        (Dt::guarded(dt_us), is_prime)
    }

    /// Whether a report has been folded yet (next `fold` would be a step, not a prime).
    #[inline]
    pub fn is_primed(&self) -> bool {
        self.primed
    }

    /// Forget all history so the next [`fold`](DtTracker::fold) primes again (used on
    /// recalibrate / device replug / filter reset).
    pub fn reset(&mut self) {
        self.sensor.reset();
        self.primed = false;
    }
}

impl Default for DtTracker {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_fold_primes_then_steps() {
        let mut t = DtTracker::new();
        assert!(!t.is_primed());

        // Prime: SensorClock returns 0.0 with no prior stamp; caller skips the IIR step.
        let (_dt0, is_prime0) = t.fold(0, 0);
        assert!(is_prime0, "first report must prime");
        assert!(t.is_primed());

        // Second report: a real step (one device tick = 16/3 us, inside the guard window).
        let (dt1, is_prime1) = t.fold(1, 1_000);
        assert!(!is_prime1, "second report must step");
        // 16/3 us is below DT_MIN_US (100), so the guard clamps it up; never zero/negative.
        assert!(dt1.us() >= hyperion_core::dt::DT_MIN_US);
    }

    #[test]
    fn reset_returns_to_prime() {
        let mut t = DtTracker::new();
        t.fold(0, 0);
        t.fold(10, 1_000);
        assert!(t.is_primed());
        t.reset();
        assert!(!t.is_primed(), "reset must require a fresh prime");
        let (_dt, is_prime) = t.fold(0, 0);
        assert!(is_prime);
    }

    #[test]
    fn now_qpc_ns_is_monotonic() {
        let t = DtTracker::new();
        let a = t.now_qpc_ns();
        let b = t.now_qpc_ns();
        assert!(b >= a, "host clock must not go backwards");
    }
}
