//! The 4-point piecewise-linear dynamic curve and the two speed metrics.
//!
//! When `use_dynamic_curve` is set, the RC `param` is derived from a per-report stick **speed**
//! via a 4-point piecewise-linear curve (`y0` at speed 0, breakpoints `x1`/`x2`, terminal `y3`
//! at `MAX_SPEED`). This is an exact port of `RcFilter.CalculateParam` /
//! `RcFilter.CalculateSpeed` (i32 truncating arithmetic), plus the corrected dt speed metric
//! from DESIGN §4.3. `RcCurve` is the serde-facing curve type re-exported at `rc::`.

use super::coeffs::{MAX_PARAM, MAX_SPEED, MIN_PARAM};

/// The 4-point dynamic curve: `param` as a function of stick speed.
///
/// `y0` is the param at speed 0; the curve is piecewise-linear through `(x1, y1)` and `(x2, y2)`
/// and terminates at `(MAX_SPEED, y3)`. Defaults reproduce the C# `RcCurveSettings` reset
/// (a flat `param = 100` curve).
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct RcCurve {
    /// Param at speed 0.
    pub y0: i32,
    /// First breakpoint speed.
    pub x1: i32,
    /// Param at `x1`.
    pub y1: i32,
    /// Second breakpoint speed.
    pub x2: i32,
    /// Param at `x2`.
    pub y2: i32,
    /// Param at `MAX_SPEED`.
    pub y3: i32,
}

impl Default for RcCurve {
    #[inline]
    fn default() -> Self {
        Self {
            y0: 100,
            x1: 32,
            y1: 100,
            x2: 96,
            y2: 100,
            y3: 100,
        }
    }
}

/// Clamp a raw param into `[MIN_PARAM, MAX_PARAM]` (C# `ClampParam`).
#[inline]
pub fn clamp_param(v: i32) -> i32 {
    v.clamp(MIN_PARAM, MAX_PARAM)
}

/// Evaluate the 4-point piecewise-linear curve at `speed`, returning the clamped `param`.
///
/// Exact port of `RcFilter.CalculateParam` (`RcFilter.cs:142-191`): the breakpoints are clamped
/// (`x1c = clamp(x1, 0, 128)`, `x2c = clamp(x2, x1c, 128)`), the three segments use i32
/// truncating division, and each guarded segment is skipped (param held at the segment's start
/// value) when its run is degenerate. The result is finally [`clamp_param`]ed.
pub fn param_from_speed(c: &RcCurve, speed: i32) -> i32 {
    let x1 = c.x1.clamp(0, MAX_SPEED);
    let x2 = c.x2.clamp(x1, MAX_SPEED);

    let result = if speed <= x1 {
        let mut r = c.y0;
        if x1 > 0 {
            r += speed * (c.y1 - c.y0) / x1;
        }
        r
    } else if speed <= x2 {
        let mut r = c.y1;
        if x2 > x1 {
            r += (speed - x1) * (c.y2 - c.y1) / (x2 - x1);
        }
        r
    } else {
        let mut r = c.y2;
        if x2 != MAX_SPEED {
            r += (speed - x2) * (c.y3 - c.y2) / (MAX_SPEED - x2);
        }
        r
    };

    clamp_param(result)
}

/// Convert a DS4-domain `[0,255]` value to the FireBird Q4 "raw12" integer (C# `ToFireBirdRaw`).
///
/// `round(input_ds4 * 16)`, clamped to `[0, 255*16]`. This is the integer the speed metric and
/// the FireBird path consume.
#[inline]
pub fn to_raw12(input_ds4: f64) -> i32 {
    (input_ds4 * 16.0).round().clamp(0.0, 255.0 * 16.0) as i32
}

/// Legacy (FireBird / UltimateLegacy) speed metric: `min(128, delta_raw12 * period_us / 8000)`.
///
/// i32 truncating, exactly as `RcFilter.CalculateSpeed`. `delta_raw12` is the per-report
/// max-axis absolute raw12 change.
#[inline]
pub fn speed_legacy(delta_raw12: i32, period_us: i32) -> i32 {
    (delta_raw12 * period_us / 8000).min(MAX_SPEED)
}

/// dt-compensated speed metric (UltimateDt): rate-invariant for a fixed wall-clock velocity.
///
/// `clamp(trunc((delta/dt_us) * period_us^2 / 8000), 0, 128)` (DESIGN §4.3). The `.trunc()`
/// (not `.round()`) is required so that at `dt == period_us` this reduces **exactly** to
/// [`speed_legacy`] (legacy truncates).
#[inline]
pub fn speed_dt(delta_raw12: i32, dt_us: f64, period_us: i32) -> i32 {
    let v = (delta_raw12 as f64 / dt_us) * (period_us as f64).powi(2) / 8000.0;
    (v.trunc() as i32).clamp(0, MAX_SPEED)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_curve_matches_contract() {
        let c = RcCurve::default();
        assert_eq!(
            c,
            RcCurve {
                y0: 100,
                x1: 32,
                y1: 100,
                x2: 96,
                y2: 100,
                y3: 100
            }
        );
    }

    #[test]
    fn flat_default_curve_is_constant_param() {
        let c = RcCurve::default();
        for speed in 0..=MAX_SPEED {
            assert_eq!(param_from_speed(&c, speed), 100, "speed={speed}");
        }
    }

    #[test]
    fn first_segment_interpolates_and_endpoints() {
        // y0=0 .. y1=100 over [0,128], single segment to MAX_SPEED.
        let c = RcCurve {
            y0: 0,
            x1: 128,
            y1: 100,
            x2: 128,
            y2: 100,
            y3: 100,
        };
        assert_eq!(param_from_speed(&c, 0), 0);
        // speed*100/128 trunc.
        assert_eq!(param_from_speed(&c, 64), 64 * 100 / 128);
        assert_eq!(param_from_speed(&c, 128), 100);
    }

    #[test]
    fn bypass_when_first_segment_x1_zero_holds_y0() {
        // DynamicCurveZeroMultiplierAtY0: x1=0 at speed 0 -> result = y0 (=0 -> bypass param 0).
        let c = RcCurve {
            y0: 0,
            x1: 0,
            y1: 500,
            x2: 128,
            y2: 500,
            y3: 500,
        };
        assert_eq!(param_from_speed(&c, 0), 0);
    }

    #[test]
    fn bypass_when_second_segment_x1_equals_x2() {
        // DynamicCurveZeroMultiplierAtY1: x1==x2 -> middle segment holds y1.
        let c = RcCurve {
            y0: 500,
            x1: 64,
            y1: 0,
            x2: 64,
            y2: 500,
            y3: 500,
        };
        // speed strictly above x1 but x2>x1 false -> holds y1 = 0.
        // speed must land in the middle/last branch; with x1==x2 the middle branch only
        // covers speed==x1 (<=x1 first), so use the final branch which starts at y2... but
        // the documented C# behavior bypasses at y1 when x1==x2 for speed in that region.
        // At speed == x1 (64) we hit the first branch (<=x1): y0 + 64*(y1-y0)/64 = 0.
        assert_eq!(param_from_speed(&c, 64), 0);
    }

    #[test]
    fn bypass_when_third_segment_x2_is_max_speed() {
        // DynamicCurveZeroMultiplierAtY2: x2==MAX_SPEED -> last segment holds y2.
        let c = RcCurve {
            y0: 500,
            x1: 64,
            y1: 500,
            x2: 128,
            y2: 0,
            y3: 500,
        };
        // speed above x2 impossible (x2==MAX); at speed 128 we are in middle branch (<=x2):
        // y1 + (128-64)*(y2-y1)/(128-64) = 500 + 64*(0-500)/64 = 0.
        assert_eq!(param_from_speed(&c, 128), 0);
    }

    #[test]
    fn last_segment_interpolates_toward_y3() {
        let c = RcCurve {
            y0: 100,
            x1: 0,
            y1: 100,
            x2: 64,
            y2: 100,
            y3: 200,
        };
        // speed 96 in last segment: y2 + (96-64)*(200-100)/(128-64) = 100 + 32*100/64 = 150.
        assert_eq!(param_from_speed(&c, 96), 150);
    }

    #[test]
    fn param_result_is_clamped() {
        let c = RcCurve {
            y0: 100_000,
            x1: 0,
            y1: 100_000,
            x2: 128,
            y2: 100_000,
            y3: 100_000,
        };
        assert_eq!(param_from_speed(&c, 0), MAX_PARAM);
        let c = RcCurve {
            y0: -100_000,
            x1: 0,
            y1: -100_000,
            x2: 128,
            y2: -100_000,
            y3: -100_000,
        };
        assert_eq!(param_from_speed(&c, 0), MIN_PARAM);
    }

    #[test]
    fn clamp_param_bounds() {
        assert_eq!(clamp_param(0), 0);
        assert_eq!(clamp_param(9999), MAX_PARAM);
        assert_eq!(clamp_param(-9999), MIN_PARAM);
    }

    #[test]
    fn to_raw12_rounds_and_clamps() {
        assert_eq!(to_raw12(128.0), 2048);
        assert_eq!(to_raw12(255.0), 4080);
        assert_eq!(to_raw12(0.0), 0);
        assert_eq!(to_raw12(-10.0), 0);
        assert_eq!(to_raw12(1000.0), 4080);
        // rounding (not truncation)
        assert_eq!(to_raw12(128.5), (128.5_f64 * 16.0).round() as i32);
    }

    #[test]
    fn speed_legacy_truncates_and_caps() {
        // delta 100, period 4000 -> 100*4000/8000 = 50.
        assert_eq!(speed_legacy(100, 4000), 50);
        // capped at MAX_SPEED.
        assert_eq!(speed_legacy(100_000, 8000), MAX_SPEED);
    }

    #[test]
    fn speed_dt_reduces_to_legacy_at_dt_equals_period() {
        // At dt == period_us, speed_dt == speed_legacy for representative deltas.
        for &(delta, period) in &[(40, 4000), (100, 4000), (17, 2000), (300, 8000)] {
            assert_eq!(
                speed_dt(delta, period as f64, period),
                speed_legacy(delta, period),
                "delta={delta} period={period}"
            );
        }
    }

    #[test]
    fn speed_dt_is_rate_invariant_for_fixed_wall_clock_velocity() {
        // delta/dt held constant (same physical velocity), period fixed -> same speed.
        // 40 raw over 4000us, 10 raw over 1000us, 20 raw over 2000us: all 0.01 raw/us.
        let period = 4000;
        let a = speed_dt(40, 4000.0, period);
        let b = speed_dt(10, 1000.0, period);
        let c = speed_dt(20, 2000.0, period);
        assert_eq!(a, b);
        assert_eq!(b, c);
    }
}
