//! Period-derived RC coefficients (i32-truncating, exactly as the C# ground truth).
//!
//! The RC recurrences (FireBird integer and UltimateLegacy) are bit-exact ports of firmware,
//! so the period-derived constants MUST use **i32 truncating division**, never f64. Computing
//! `base_value` / `lead_base` in floating point would make both legacy paths diverge from the
//! `153.3984375` oracle. These are recomputed only when `period_us` changes (cached upstream).

/// Lower clamp on the configurable filter period, in microseconds.
pub const MIN_PERIOD_US: i32 = 1000;
/// Upper clamp on the configurable filter period, in microseconds.
pub const MAX_PERIOD_US: i32 = 8000;
/// Lower clamp on the RC `param` (curve output / fixed param).
pub const MIN_PARAM: i32 = -500;
/// Upper clamp on the RC `param`.
pub const MAX_PARAM: i32 = 500;
/// Lower clamp on the dynamic-curve speed metric.
pub const MIN_SPEED: i32 = 0;
/// Upper clamp on the dynamic-curve speed metric.
pub const MAX_SPEED: i32 = 128;

/// The period-derived integer coefficients shared by every RC mode.
///
/// All fields are computed with i32 truncating division (see [`PeriodCoeffs::new`]); this is
/// what keeps the FireBird path bit-exact and the UltimateLegacy path matched to the C# golden.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PeriodCoeffs {
    /// The clamped period in microseconds (`[MIN_PERIOD_US, MAX_PERIOD_US]`).
    pub period_us: i32,
    /// `max(period_us / 1000, 1)` — the per-report sample interval in milliseconds (i32 trunc).
    pub sample_ms: i32,
    /// `100 / sample_ms` — the positive low-pass base weight (i32 trunc; e.g. period 4000 → 25).
    pub base_value: i32,
    /// `200000 / period_us` — the negative-branch blend base (i32 trunc; e.g. period 4000 → 50).
    pub lead_base: i32,
}

impl PeriodCoeffs {
    /// Compute the coefficients for a (possibly out-of-range) `period_us`, mirroring
    /// `RcFilter.Process` in C#: clamp the period, then derive every constant by i32 trunc.
    #[inline]
    pub fn new(period_us: i32) -> Self {
        let period_us = period_us.clamp(MIN_PERIOD_US, MAX_PERIOD_US);
        let sample_ms = (period_us / 1000).max(1);
        let base_value = 100 / sample_ms;
        let lead_base = 200_000 / period_us;
        Self {
            period_us,
            sample_ms,
            base_value,
            lead_base,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn period_4000_matches_design_headline() {
        let c = PeriodCoeffs::new(4000);
        assert_eq!(c.period_us, 4000);
        assert_eq!(c.sample_ms, 4);
        assert_eq!(c.base_value, 25);
        assert_eq!(c.lead_base, 50);
    }

    #[test]
    fn period_is_clamped_into_range() {
        assert_eq!(PeriodCoeffs::new(0).period_us, MIN_PERIOD_US);
        assert_eq!(PeriodCoeffs::new(999).period_us, MIN_PERIOD_US);
        assert_eq!(PeriodCoeffs::new(100_000).period_us, MAX_PERIOD_US);
    }

    #[test]
    fn sample_ms_never_zero_and_truncates() {
        // 1000us -> 1ms -> base_value 100.
        let c = PeriodCoeffs::new(1000);
        assert_eq!(c.sample_ms, 1);
        assert_eq!(c.base_value, 100);
        // 1999us still truncates to 1ms (sample_ms quantizes the knob).
        let c = PeriodCoeffs::new(1999);
        assert_eq!(c.sample_ms, 1);
        assert_eq!(c.base_value, 100);
    }

    #[test]
    fn base_value_jumps_at_sample_ms_boundary() {
        // Design note (algo verifier (d)): period 2999 -> 3001 gives 50 -> 33.
        assert_eq!(PeriodCoeffs::new(2999).base_value, 50); // sample_ms 2 -> 100/2
        assert_eq!(PeriodCoeffs::new(3001).base_value, 33); // sample_ms 3 -> 100/3 trunc
    }

    #[test]
    fn lead_base_truncates() {
        assert_eq!(PeriodCoeffs::new(8000).lead_base, 25); // 200000/8000
        assert_eq!(PeriodCoeffs::new(3000).lead_base, 66); // 200000/3000 trunc
    }
}
