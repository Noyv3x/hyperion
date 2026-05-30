//! The guarded per-report elapsed time fed to the dt-compensated filter.
//!
//! The real interval between input reports varies with the device report rate
//! (USB/BT/"fake-4K", ~250–8000 Hz) and with dropped/stalled reports. The dt-compensated
//! Ultimate filter treats `period_us` as a true time constant and rescales its coefficients
//! by `dt/period_us`, so it needs a sane, bounded `dt`. This newtype enforces the guard
//! window once, at the boundary, so the math never sees a zero, negative, or absurd dt.

/// Lower guard on the per-report elapsed time, in microseconds (≈ 10 kHz ceiling).
pub const DT_MIN_US: f64 = 100.0;
/// Upper guard on the per-report elapsed time, in microseconds (dropped reports / BT stalls).
pub const DT_MAX_US: f64 = 20_000.0;

/// A guarded report interval in microseconds, always within `[DT_MIN_US, DT_MAX_US]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Dt(f64);

impl Dt {
    /// Clamp a raw measured interval (microseconds) into the guard window.
    #[inline]
    pub fn guarded(raw_us: f64) -> Self {
        // NaN-safe: a NaN raw collapses to the lower bound.
        let v = if raw_us.is_nan() { DT_MIN_US } else { raw_us };
        Self(v.clamp(DT_MIN_US, DT_MAX_US))
    }

    /// The guarded interval in microseconds.
    #[inline]
    pub fn us(self) -> f64 {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn guards_clamp_the_window() {
        assert_eq!(Dt::guarded(0.0).us(), DT_MIN_US);
        assert_eq!(Dt::guarded(-5.0).us(), DT_MIN_US);
        assert_eq!(Dt::guarded(1_000_000.0).us(), DT_MAX_US);
        assert_eq!(Dt::guarded(4000.0).us(), 4000.0);
        assert_eq!(Dt::guarded(f64::NAN).us(), DT_MIN_US);
    }
}
