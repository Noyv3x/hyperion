//! The canonical pipeline stick unit and raw-device de/re-quantization.
//!
//! `Axis` is `f64` in `[-1.0, 1.0]`, neutral `0.0`. Stick math is radial/signed/centered,
//! so this avoids re-biasing at every stage and makes 8-bit and 16-bit sources symmetric.
//! Both edges of the raw range map to exactly `±1.0` and the hardware neutral maps to
//! exactly `0.0` regardless of the (often asymmetric) raw range.

/// Canonical high-precision stick value, semantically `[-1.0, 1.0]`, neutral `0.0`.
pub type Axis = f64;

/// Clamp into the canonical `[-1.0, 1.0]` range.
#[inline]
pub fn clamp_axis(v: f64) -> f64 {
    v.clamp(-1.0, 1.0)
}

/// Describes a raw stick axis range so it can be mapped to/from [`Axis`] without bias.
///
/// The mapping is piecewise-linear with the break at `neutral`, so the negative span
/// (`neutral - min`) and the positive span (`max - neutral`) scale independently. This
/// reproduces the DS4 8-bit asymmetry (`0x80` center, 128 down / 127 up) exactly.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RawStickFormat {
    pub bits: u8,
    pub neutral: i32,
    pub min: i32,
    pub max: i32,
}

impl RawStickFormat {
    /// DualSense / DS4 8-bit stick: `0x00` left, `0x80` center, `0xFF` right.
    pub const DS_8BIT: Self = Self {
        bits: 8,
        neutral: 128,
        min: 0,
        max: 255,
    };
    /// XInput signed 16-bit thumbstick.
    pub const XINPUT_16: Self = Self {
        bits: 16,
        neutral: 0,
        min: -32768,
        max: 32767,
    };

    /// A symmetric signed-integer XInput-style format of `bits` width (used by the
    /// adjustable-resolution controller's higher bit depths).
    pub fn xinput(bits: u8) -> Self {
        let half = 1i64 << (bits - 1);
        Self {
            bits,
            neutral: 0,
            min: -(half as i32),
            max: (half - 1) as i32,
        }
    }

    /// Map a raw integer reading to the canonical `[-1.0, 1.0]` unit. `neutral` → exactly `0.0`.
    #[inline]
    pub fn to_axis(&self, raw: i32) -> Axis {
        let d = raw - self.neutral;
        let a = if d >= 0 {
            d as f64 / (self.max - self.neutral) as f64
        } else {
            d as f64 / (self.neutral - self.min) as f64
        };
        clamp_axis(a)
    }

    /// Map a canonical axis value back to a raw integer reading, rounding exactly once
    /// and clamping into `[min, max]`.
    #[inline]
    pub fn from_axis(&self, a: Axis) -> i32 {
        let a = clamp_axis(a);
        let scaled = if a >= 0.0 {
            a * (self.max - self.neutral) as f64
        } else {
            a * (self.neutral - self.min) as f64
        };
        (scaled.round() as i32 + self.neutral).clamp(self.min, self.max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ds4_endpoints_and_neutral_are_exact() {
        let f = RawStickFormat::DS_8BIT;
        assert_eq!(f.to_axis(128), 0.0);
        assert_eq!(f.to_axis(0), -1.0);
        assert_eq!(f.to_axis(255), 1.0);
        assert_eq!(f.from_axis(0.0), 128);
        assert_eq!(f.from_axis(-1.0), 0);
        assert_eq!(f.from_axis(1.0), 255);
    }

    #[test]
    fn xinput16_endpoints() {
        let f = RawStickFormat::XINPUT_16;
        assert_eq!(f.to_axis(0), 0.0);
        assert_eq!(f.to_axis(-32768), -1.0);
        assert_eq!(f.to_axis(32767), 1.0);
    }
}
