//! Precision-preserving conversion between the canonical `[-1,1]` axis unit and the
//! DS4-compatible `[0,255]` domain the RC filter computes in.
//!
//! The RC filter is a bit-exact port of firmware that operates on DS4 stick values
//! (neutral 128, range `[0,255]`), so the live pipeline adapts `[-1,1]` ↔ `[0,255]`
//! around it. These conversions are **continuous** (pure f64, no rounding/quantization):
//! `axis_to_ds4` and `ds4_to_axis` are exact inverses, with the same `128`-centered
//! asymmetry as [`crate::axis::RawStickFormat::DS_8BIT`] (128 down, 127 up).

/// Map a canonical axis value `[-1,1]` to the continuous DS4 `[0,255]` domain. Neutral → 128.
#[inline]
pub fn axis_to_ds4(a: f64) -> f64 {
    let a = a.clamp(-1.0, 1.0);
    if a >= 0.0 {
        128.0 + a * 127.0
    } else {
        128.0 + a * 128.0
    }
}

/// Map a continuous DS4 `[0,255]` value back to the canonical axis unit `[-1,1]`. 128 → 0.
#[inline]
pub fn ds4_to_axis(d: f64) -> f64 {
    let d = d.clamp(0.0, 255.0);
    if d >= 128.0 {
        (d - 128.0) / 127.0
    } else {
        (d - 128.0) / 128.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_exactly_at_anchors() {
        assert_eq!(axis_to_ds4(0.0), 128.0);
        assert_eq!(axis_to_ds4(-1.0), 0.0);
        assert_eq!(axis_to_ds4(1.0), 255.0);
        assert_eq!(ds4_to_axis(128.0), 0.0);
        assert_eq!(ds4_to_axis(0.0), -1.0);
        assert_eq!(ds4_to_axis(255.0), 1.0);
    }

    #[test]
    fn round_trips_within_epsilon() {
        for i in 0..=2000 {
            let a = -1.0 + (i as f64) / 1000.0;
            let back = ds4_to_axis(axis_to_ds4(a));
            assert!((a - back).abs() < 1e-12, "a={a} back={back}");
        }
    }
}
