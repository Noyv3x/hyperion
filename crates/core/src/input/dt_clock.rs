//! The report time-base: fold the DualSense hardware timestamp into a guarded `dt` (µs).
//!
//! Ground truth is the validated C# path (`DS4Device.cs:1301-1333`): the device reports a
//! **16-bit** timestamp at report bytes `10..12` (little-endian `u16`), advancing one tick
//! every `16/3 µs`. Consecutive reports give `dt = (ts - prev) * 16/3 µs`, with the true
//! 16-bit hardware wrap handled by `u16::wrapping_sub`. When two reports carry the *same*
//! stamp (a hardware duplicate), the device clock cannot resolve the interval, so we fall
//! back to the host QPC delta.
//!
//! Resolved-conflict #1 (vs three design tracks that proposed a `u32` field at bytes 28..31
//! with a `1/3 µs` unit): a `u32` `wrapping_sub` never detects the real ~349 ms 16-bit wrap
//! and would silently saturate `dt` several times a second. The `u16` / `16-3 µs` / bytes
//! 10..11 form here is the one the shipping C# actually reads.

/// Microseconds per hardware timestamp tick (`DS4Device.cs:1305`, `* 16 / 3`).
pub const DSE_TS_UNIT_US: f64 = 16.0 / 3.0;

/// Upper guard on a single folded interval, in microseconds (matches [`crate::dt::DT_MAX_US`]).
const DT_CLAMP_MAX_US: f64 = 20_000.0;

/// Folds the device timestamp (with QPC fallback) into a per-report `dt` in microseconds.
///
/// `prev_qpc_ns` advances on **every** call — including device-timestamp reports — so the
/// first QPC fallback after a run of device-timestamp reports measures only the latest
/// interval, not a huge accumulated span.
#[derive(Clone, Copy, Debug, Default)]
pub struct SensorClock {
    prev_ts: Option<u16>,
    prev_qpc_ns: u64,
}

impl SensorClock {
    /// Fold one report's `(sensor_ts, host_qpc_ns)` into `dt` microseconds.
    ///
    /// * First call after construction/reset primes and returns `0.0` — the caller must
    ///   treat `dt == 0.0` as "do not advance the IIR" (a tiny-`dt` step still moves the
    ///   filter), i.e. emit the input unchanged.
    /// * Otherwise `ticks = sensor_ts.wrapping_sub(prev)`; if `ticks != 0` the interval is
    ///   `ticks * 16/3 µs`; if `ticks == 0` (duplicate stamp) it is the QPC delta in µs.
    ///
    /// The result is clamped to `[0, 20000]` µs; the caller's [`Dt::guarded`](crate::dt::Dt)
    /// applies the lower `100 µs` floor before the filter step.
    pub fn fold(&mut self, sensor_ts: u16, host_qpc_ns: u64) -> f64 {
        let dt_us = match self.prev_ts {
            None => 0.0,
            Some(prev) => {
                let ticks = sensor_ts.wrapping_sub(prev);
                if ticks != 0 {
                    ticks as f64 * DSE_TS_UNIT_US
                } else {
                    host_qpc_ns.saturating_sub(self.prev_qpc_ns) as f64 / 1000.0
                }
            }
        };
        self.prev_ts = Some(sensor_ts);
        self.prev_qpc_ns = host_qpc_ns;
        dt_us.clamp(0.0, DT_CLAMP_MAX_US)
    }

    /// Forget history so the next [`fold`](Self::fold) primes (returns `0.0`).
    pub fn reset(&mut self) {
        self.prev_ts = None;
        self.prev_qpc_ns = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_call_primes_to_zero() {
        let mut c = SensorClock::default();
        assert_eq!(c.fold(1234, 5_000), 0.0);
    }

    #[test]
    fn simple_tick_delta() {
        let mut c = SensorClock::default();
        c.fold(1000, 0);
        // 3 ticks * 16/3 = 16.0 us exactly.
        assert!((c.fold(1003, 1_000_000) - 16.0).abs() < 1e-12);
    }

    #[test]
    fn u16_wrap_65535_to_0() {
        let mut c = SensorClock::default();
        c.fold(65535, 0);
        // wrapping_sub: 0 - 65535 = 1 tick -> 16/3 us.
        let dt = c.fold(0, 1_000_000);
        assert!((dt - 16.0 / 3.0).abs() < 1e-12, "dt={dt}");
    }

    #[test]
    fn u16_wrap_larger_span() {
        let mut c = SensorClock::default();
        c.fold(65530, 0);
        // 0x000A - 0xFFFA = 16 ticks across the wrap.
        let dt = c.fold(10, 1_000_000);
        assert!((dt - 16.0 * 16.0 / 3.0).abs() < 1e-12, "dt={dt}");
    }

    #[test]
    fn identical_stamp_falls_back_to_qpc() {
        let mut c = SensorClock::default();
        c.fold(5000, 1_000_000);
        // same stamp -> use QPC delta: (3_500_000 - 1_000_000) ns = 2500 us.
        let dt = c.fold(5000, 3_500_000);
        assert!((dt - 2500.0).abs() < 1e-9, "dt={dt}");
    }

    #[test]
    fn qpc_advances_even_on_device_ts_path() {
        let mut c = SensorClock::default();
        c.fold(100, 1_000_000);
        // device-ts path; prev_qpc must advance to 9_000_000 here.
        c.fold(103, 9_000_000);
        // now a duplicate stamp -> QPC delta should be measured from 9_000_000, not 1_000_000.
        let dt = c.fold(103, 9_500_000);
        assert!((dt - 500.0).abs() < 1e-9, "dt={dt}");
    }

    #[test]
    fn clamps_to_20ms_ceiling() {
        let mut c = SensorClock::default();
        c.fold(0, 0);
        // huge tick delta -> clamp to 20000 us.
        let dt = c.fold(40000, 1_000_000);
        assert_eq!(dt, DT_CLAMP_MAX_US);
    }

    #[test]
    fn reset_reprimes() {
        let mut c = SensorClock::default();
        c.fold(100, 0);
        c.reset();
        assert_eq!(c.fold(200, 1_000), 0.0);
    }
}
