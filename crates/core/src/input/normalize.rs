//! Raw integer → canonical unit maps for the supported source formats.
//!
//! These are thin wrappers over [`RawStickFormat`](crate::axis::RawStickFormat) so the
//! de-quantization rule (asymmetric, neutral → exactly `0.0`, both edges → exactly `±1.0`)
//! lives in one place. Triggers are a plain `u8/255` unit map.

use crate::axis::RawStickFormat;

/// Map a DualSense / DS4 8-bit stick byte (`0x00` low, `0x80` center, `0xFF` high) to `[-1,1]`.
///
/// Neutral `0x80` → exactly `0.0`; the asymmetry (128 below center, 127 above) is handled by
/// [`RawStickFormat::DS_8BIT`]. This maps the *raw* axis; callers negate Y to make `+y == up`.
#[inline]
pub fn u8_stick_to_axis(v: u8) -> f64 {
    RawStickFormat::DS_8BIT.to_axis(v as i32)
}

/// Map an XInput signed-16 thumbstick reading to `[-1,1]` (`0 → 0.0`, lossless 16-bit).
#[inline]
pub fn signed16_to_axis(v: i16) -> f64 {
    RawStickFormat::XINPUT_16.to_axis(v as i32)
}

/// Map an 8-bit trigger reading to `[0,1]` (`0 → 0.0`, `255 → 1.0`).
#[inline]
pub fn u8_trigger(v: u8) -> f64 {
    v as f64 / 255.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stick_endpoints_and_center() {
        assert_eq!(u8_stick_to_axis(0x80), 0.0);
        assert_eq!(u8_stick_to_axis(0x00), -1.0);
        assert_eq!(u8_stick_to_axis(0xFF), 1.0);
    }

    #[test]
    fn signed16_endpoints_and_center() {
        assert_eq!(signed16_to_axis(0), 0.0);
        assert_eq!(signed16_to_axis(i16::MIN), -1.0);
        assert_eq!(signed16_to_axis(i16::MAX), 1.0);
    }

    #[test]
    fn trigger_endpoints() {
        assert_eq!(u8_trigger(0), 0.0);
        assert_eq!(u8_trigger(255), 1.0);
        assert!((u8_trigger(128) - 128.0 / 255.0).abs() < 1e-15);
    }
}
