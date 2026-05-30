//! The pure, OS-free virtual-pad output frame and its single final quantization.
//!
//! The whole pipeline runs in `f64` (`[-1,1]` sticks, `[0,1]` triggers) right up to this
//! boundary, where each value is rounded **exactly once** to the XInput integer domain. The
//! mapping is re-used verbatim by the Windows `vgamepad-output` crate so there is one — and
//! only one — quantization point, matching the C# `Xbox360OutDevice` `AxisScale` semantics
//! (asymmetric `i16`: positive scaled by `32767`, negative by `32768`).
//!
//! Submodules:
//! * [`state`] — the structured [`OutputState`] / [`PadButtons`] egress accumulator the mapping
//!   engine fills, plus [`pack_xinput`] and the DS4 lowering ([`to_ds4_axis`], [`dpad_8way`]).
//! * [`kbm`] — the fixed-capacity [`KbmBatch`] keyboard/mouse event accumulator.

pub mod kbm;
pub mod state;

pub use kbm::{KbmBatch, KbmEvent, KeyKind, MouseButton, KBM_BATCH_CAP};
pub use state::{dpad_8way, pack_xinput, to_ds4_axis, Ds4Dpad, OutputState, PadButtons, PadTarget};

/// A fully-processed controller frame ready to be mapped to a virtual Xbox 360 report.
///
/// Sticks are canonical `[-1,1]` (neutral `0.0`, `+y == up`), triggers are `[0,1]`, and
/// `buttons` is the already-packed XInput button bitfield.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct OutputFrame {
    pub lx: f64,
    pub ly: f64,
    pub rx: f64,
    pub ry: f64,
    pub lt: f64,
    pub rt: f64,
    pub buttons: u16,
}

/// Map a canonical `[-1,1]` thumbstick value to the XInput signed-16 domain with a single
/// round.
///
/// The scale is asymmetric — `+1.0 → 32767`, `-1.0 → -32768` — to use the full signed range
/// without clipping either endpoint, mirroring the C# `AxisScale` path. The final
/// `clamp` is a guard against floating rounding nudging a value one ULP past the i16 edge.
#[inline]
pub fn to_xinput_thumb(n: f64) -> i16 {
    let n = n.clamp(-1.0, 1.0);
    let v = if n >= 0.0 { n * 32767.0 } else { n * 32768.0 };
    v.round().clamp(-32768.0, 32767.0) as i16
}

/// Map a canonical `[0,1]` trigger value to the XInput unsigned-8 domain with a single round.
#[inline]
pub fn to_xinput_trigger(t: f64) -> u8 {
    (t.clamp(0.0, 1.0) * 255.0).round() as u8
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn thumb_endpoints_and_neutral() {
        assert_eq!(to_xinput_thumb(0.0), 0);
        assert_eq!(to_xinput_thumb(1.0), 32767);
        assert_eq!(to_xinput_thumb(-1.0), -32768);
        // Asymmetric scale: +0.5 and -0.5 use different denominators.
        assert_eq!(to_xinput_thumb(0.5), 16384); // (0.5*32767).round() = 16384 (16383.5 -> even? no, .round() = 16384)
        assert_eq!(to_xinput_thumb(-0.5), -16384);
    }

    #[test]
    fn thumb_clamps_out_of_range() {
        assert_eq!(to_xinput_thumb(2.0), 32767);
        assert_eq!(to_xinput_thumb(-2.0), -32768);
    }

    #[test]
    fn trigger_endpoints_and_clamp() {
        assert_eq!(to_xinput_trigger(0.0), 0);
        assert_eq!(to_xinput_trigger(1.0), 255);
        assert_eq!(to_xinput_trigger(0.5), 128); // (0.5*255).round() = (127.5).round() = 128
        assert_eq!(to_xinput_trigger(-1.0), 0);
        assert_eq!(to_xinput_trigger(2.0), 255);
    }

    #[test]
    fn default_frame_is_neutral() {
        let f = OutputFrame::default();
        assert_eq!(to_xinput_thumb(f.lx), 0);
        assert_eq!(to_xinput_thumb(f.ly), 0);
        assert_eq!(to_xinput_trigger(f.lt), 0);
        assert_eq!(f.buttons, 0);
    }
}
