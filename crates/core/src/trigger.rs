//! Trigger settings, curves, and the ordered `process_trigger` chain.
//!
//! The trigger pipeline runs `f64` end-to-end (single quantization deferred to the egress),
//! porting the C# `Mapping.SetCurveAndDeadzone` L2/R2 expression. **Pinned divergence (§4):**
//! C# truncates the trigger to a byte between stages; we keep `f64` so our rounded output can
//! differ from the C# byte by `<= 1/255` — intended, not a regression.
//!
//! Stage order (blueprint §4):
//!
//! ```text
//! deadzone -> max-zone -> max-output -> anti-deadzone -> sensitivity -> output curve -> to-button threshold
//! ```
//!
//! Pure, alloc-free, OS-free. The `[0,1]` analog domain mirrors C# `cState.L2 / 255.0`; the
//! to-button threshold compares the raw `[0,255]` reading against `max(button_threshold, dead_zone)`.

use crate::dt::Dt;
use crate::stick::settings::OutputCurve;

/// Trigger output curve.
///
/// Discriminants match the C# integer `l2OutCurveMode`/`r2OutCurveMode` (`Linear = 0` => no curve):
/// `Linear, EnhancedPrecision, Quadratic, Cubic, EaseoutQuad, EaseoutCubic, Bezier,
/// ApexClassicInverse = 7, ApexClassicInverseAxial = 8`. One-dimensional output, so both apex
/// modes use the signed-sqrt axis curve (C# treats `7 || 8` identically for triggers).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum TriggerCurve {
    /// Identity.
    #[default]
    Linear = 0,
    /// Enhanced-precision piecewise (trigger variant: `0.55x / x-0.18 / 1.72x-0.72`).
    EnhancedPrecision = 1,
    /// `x^2`.
    Quadratic = 2,
    /// `x^3`.
    Cubic = 3,
    /// Ease-out quadratic.
    EaseoutQuad = 4,
    /// Ease-out cubic.
    EaseoutCubic = 5,
    /// Custom Bezier (M3 identity stub).
    Bezier = 6,
    /// Apex Classic inverse (signed sqrt).
    ApexClassicInverse = 7,
    /// Apex Classic inverse axial (signed sqrt; identical to radial for a 1-D trigger).
    ApexClassicInverseAxial = 8,
}

impl TriggerCurve {
    /// The C# integer curve mode (`Linear` => 0, no curve applied).
    #[inline]
    pub const fn mode(self) -> u8 {
        self as u8
    }
}

impl From<TriggerCurve> for OutputCurve {
    #[inline]
    fn from(c: TriggerCurve) -> Self {
        match c {
            TriggerCurve::Linear => OutputCurve::Linear,
            TriggerCurve::EnhancedPrecision => OutputCurve::EnhancedPrecision,
            TriggerCurve::Quadratic => OutputCurve::Quadratic,
            TriggerCurve::Cubic => OutputCurve::Cubic,
            TriggerCurve::EaseoutQuad => OutputCurve::EaseoutQuad,
            TriggerCurve::EaseoutCubic => OutputCurve::EaseoutCubic,
            TriggerCurve::Bezier => OutputCurve::Bezier,
            TriggerCurve::ApexClassicInverse => OutputCurve::ApexClassicInverse,
            TriggerCurve::ApexClassicInverseAxial => OutputCurve::ApexClassicInverseAxial,
        }
    }
}

/// Full per-trigger settings (one per L2 / R2), placed inside a `Profile`.
///
/// `Copy` and small so the hot loop copies it into the resident `[TriggerSettings; 2]`. Defaults
/// match the C# `TriggerDeadZoneZInfo.Reset()` clean state: everything off, sensitivity 1.0.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct TriggerSettings {
    /// Deadzone in `[0,255]` raw trigger units (C# "bad old convention").
    pub dead_zone: u8,
    /// Anti-deadzone in `[0,100]` percent.
    pub anti_dead_zone: i32,
    /// Max-zone in `[1,100]` percent.
    pub max_zone: i32,
    /// Max-output in `[0,100]` percent.
    pub max_output: f64,
    /// Output multiplier (`1.0` == off).
    pub sensitivity: f64,
    /// Output curve.
    pub curve: TriggerCurve,
    /// Digital activation threshold in `[0,255]` raw units (trigger-as-button). `0` uses the
    /// kind-default 100/255 at resolve time; the to-button compare uses `max(button_threshold, dead_zone)`.
    pub button_threshold: u8,
}

impl Default for TriggerSettings {
    #[inline]
    fn default() -> Self {
        Self {
            dead_zone: 0,
            anti_dead_zone: 0,
            max_zone: 100,
            max_output: 100.0,
            sensitivity: 1.0,
            curve: TriggerCurve::Linear,
            // 100/255 ≈ 0.3922 — the C# `triggers > 100` digital threshold.
            button_threshold: 100,
        }
    }
}

impl TriggerSettings {
    /// A copy with §4 ranges enforced.
    pub fn clamped(&self) -> TriggerSettings {
        let mut out = *self;
        out.anti_dead_zone = out.anti_dead_zone.clamp(0, 100);
        out.max_zone = out.max_zone.clamp(1, 100);
        out.max_output = out.max_output.clamp(0.0, 100.0);
        out
    }
}

/// Per-trigger mutable state (resident, one per L2 / R2).
///
/// **`Default` is the clean post-reset state.** `last_pressed` tracks digital edges; `elapsed_us`
/// is the monotonic accumulator reserved for the two-stage / hip-fire modes (M6).
#[derive(Default, Clone, Copy, Debug)]
pub struct TriggerState {
    /// Whether the trigger read pressed on the previous report (edge tracking).
    pub last_pressed: bool,
    /// Monotonic microsecond accumulator (reserved for two-stage timing, M6).
    pub elapsed_us: i64,
}

/// Process one trigger report through the ordered chain.
///
/// * `raw` — the trigger reading in `[0,1]` (`cState.L2 / 255.0`).
/// * `cfg` — the per-trigger settings (already `clamped()`).
/// * `st` — the resident per-trigger state.
/// * `dt` — the guarded per-report elapsed time (advances `elapsed_us`).
///
/// Returns `(analog_out, digital_pressed)`: `analog_out` in `[0,1]` (rounded only at egress),
/// `digital_pressed` from the to-button threshold `raw255 >= max(button_threshold, dead_zone)`.
pub fn process_trigger(
    raw: f64,
    cfg: &TriggerSettings,
    st: &mut TriggerState,
    dt: Dt,
) -> (f64, bool) {
    st.elapsed_us = st.elapsed_us.wrapping_add(dt.us() as i64);

    let raw = raw.clamp(0.0, 1.0);
    let raw255 = raw * 255.0;
    let dead_zone = cfg.dead_zone as f64; // [0,255]
    let anti_dead_zone = cfg.anti_dead_zone;
    let max_zone = cfg.max_zone;
    let max_output = cfg.max_output;

    // --- deadzone -> max-zone -> max-output -> anti-deadzone (fused, C# L2 expression) ---
    let mut output;
    let interpret =
        cfg.dead_zone > 0 || anti_dead_zone > 0 || max_zone != 100 || max_output != 100.0;
    if interpret {
        let ratio = max_zone as f64 / 100.0;
        let max_value = 255.0 * ratio;

        if cfg.dead_zone > 0 {
            if raw255 > dead_zone {
                let current = raw255.clamp(0.0, max_value);
                output = (current - dead_zone) / (max_value - dead_zone);
            } else {
                output = 0.0;
            }
        } else {
            let current = raw255.clamp(0.0, max_value);
            output = current / max_value;
        }

        // max-output
        if max_output != 100.0 {
            let max_out_ratio = max_output / 100.0;
            output = output.clamp(0.0, max_out_ratio);
        }

        // anti-deadzone
        let temp_anti_dead = if anti_dead_zone > 0 {
            anti_dead_zone as f64 * 0.01
        } else {
            0.0
        };
        output = if output > 0.0 {
            (1.0 - temp_anti_dead) * output + temp_anti_dead
        } else {
            0.0
        };
    } else {
        output = raw;
    }

    // --- sensitivity ---
    if cfg.sensitivity != 1.0 {
        output = (cfg.sensitivity * output).clamp(0.0, 1.0);
    }

    // --- output curve --- (C# applies only when the post-stage value is non-zero)
    if cfg.curve != TriggerCurve::Linear && output != 0.0 {
        output = trigger_curve(output, cfg.curve);
    }

    let analog_out = output.clamp(0.0, 1.0);

    // --- to-button threshold ---
    // raw255 vs max(button_threshold, dead_zone) (pre-quantization f64 for determinism).
    let threshold = (cfg.button_threshold as f64).max(dead_zone);
    let pressed = raw255 >= threshold && threshold > 0.0;
    st.last_pressed = pressed;

    (analog_out, pressed)
}

/// Trigger output curve (C# `l2OutCurveMode` math), `[0,1] -> [0,1]`.
fn trigger_curve(temp: f64, curve: TriggerCurve) -> f64 {
    match curve {
        TriggerCurve::Linear => temp,
        TriggerCurve::EnhancedPrecision => {
            // C# trigger enhanced: 0.55x / x-0.18 / 1.72x-0.72.
            if temp <= 0.4 {
                0.55 * temp
            } else if temp <= 0.75 {
                temp - 0.18
            } else {
                temp * 1.72 - 0.72
            }
        }
        TriggerCurve::Quadratic => temp * temp,
        TriggerCurve::Cubic => temp * temp * temp,
        TriggerCurve::EaseoutQuad => -(temp * (temp - 2.0)),
        TriggerCurve::EaseoutCubic => {
            let inner = temp.abs() - 1.0;
            -(inner * inner * inner + 1.0)
        }
        TriggerCurve::Bezier => {
            // M3 identity stub (matches a default linear Bezier LUT).
            temp
        }
        TriggerCurve::ApexClassicInverse | TriggerCurve::ApexClassicInverseAxial => {
            // Signed sqrt; for a [0,1] trigger this is sqrt(temp).
            crate::stick::stages::apex_axis_curve(temp)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dt() -> Dt {
        Dt::guarded(4000.0)
    }

    fn approx(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    #[test]
    fn default_passes_through_analog() {
        let cfg = TriggerSettings::default();
        let mut st = TriggerState::default();
        for raw in [0.0, 0.25, 0.5, 1.0] {
            let (a, _) = process_trigger(raw, &cfg, &mut st, dt());
            assert!(approx(a, raw), "raw={raw} a={a}");
        }
    }

    #[test]
    fn default_button_threshold_100_255() {
        let cfg = TriggerSettings::default();
        let mut st = TriggerState::default();
        // raw 99/255 -> below 100 -> not pressed.
        let (_, p0) = process_trigger(99.0 / 255.0, &cfg, &mut st, dt());
        assert!(!p0);
        // raw 100/255 -> >= 100 -> pressed.
        let (_, p1) = process_trigger(100.0 / 255.0, &cfg, &mut st, dt());
        assert!(p1);
    }

    #[test]
    fn deadzone_zeroes_below_then_scales() {
        let cfg = TriggerSettings {
            dead_zone: 50,
            ..TriggerSettings::default()
        };
        let mut st = TriggerState::default();
        // raw 40/255 < dz 50 -> 0 analog.
        let (a0, _) = process_trigger(40.0 / 255.0, &cfg, &mut st, dt());
        assert!(approx(a0, 0.0), "a0={a0}");
        // raw 255 (full) -> (255-50)/(255-50)=1.0.
        let (a1, _) = process_trigger(1.0, &cfg, &mut st, dt());
        assert!(approx(a1, 1.0), "a1={a1}");
        // mid raw 152.5/255 -> (152.5-50)/(255-50)=0.5.
        let (a2, _) = process_trigger(152.5 / 255.0, &cfg, &mut st, dt());
        assert!(approx(a2, 0.5), "a2={a2}");
    }

    #[test]
    fn anti_deadzone_floor() {
        let cfg = TriggerSettings {
            anti_dead_zone: 20,
            ..TriggerSettings::default()
        };
        let mut st = TriggerState::default();
        // small positive raw -> floor lifted by anti-dz: output = (1-0.2)*r + 0.2.
        let r = 0.1;
        let (a, _) = process_trigger(r, &cfg, &mut st, dt());
        // current/maxValue with maxZone=100 -> r; then anti-dz.
        let expected = (1.0 - 0.2) * r + 0.2;
        assert!(approx(a, expected), "a={a} expected={expected}");
    }

    #[test]
    fn max_output_clamps() {
        let cfg = TriggerSettings {
            max_output: 50.0,
            ..TriggerSettings::default()
        };
        let mut st = TriggerState::default();
        let (a, _) = process_trigger(1.0, &cfg, &mut st, dt());
        assert!(approx(a, 0.5), "a={a}");
    }

    #[test]
    fn sensitivity_scales_and_clamps() {
        let cfg = TriggerSettings {
            sensitivity: 2.0,
            ..TriggerSettings::default()
        };
        let mut st = TriggerState::default();
        let (a, _) = process_trigger(0.25, &cfg, &mut st, dt());
        assert!(approx(a, 0.5), "a={a}");
        let (a2, _) = process_trigger(0.9, &cfg, &mut st, dt());
        assert!(approx(a2, 1.0), "clamped a2={a2}");
    }

    #[test]
    fn quadratic_curve() {
        let cfg = TriggerSettings {
            curve: TriggerCurve::Quadratic,
            ..TriggerSettings::default()
        };
        let mut st = TriggerState::default();
        let (a, _) = process_trigger(0.5, &cfg, &mut st, dt());
        assert!(approx(a, 0.25), "a={a}");
    }

    #[test]
    fn apex_curve_sqrt() {
        let cfg = TriggerSettings {
            curve: TriggerCurve::ApexClassicInverse,
            ..TriggerSettings::default()
        };
        let mut st = TriggerState::default();
        let (a, _) = process_trigger(0.25, &cfg, &mut st, dt());
        assert!(approx(a, 0.5), "a={a}");
    }

    #[test]
    fn button_uses_max_of_threshold_and_deadzone() {
        let cfg = TriggerSettings {
            dead_zone: 200,
            button_threshold: 100,
            ..TriggerSettings::default()
        };
        let mut st = TriggerState::default();
        // raw 150/255: above button_threshold 100 but below dead_zone 200 -> not pressed.
        let (_, p0) = process_trigger(150.0 / 255.0, &cfg, &mut st, dt());
        assert!(!p0);
        // raw 210/255 -> pressed.
        let (_, p1) = process_trigger(210.0 / 255.0, &cfg, &mut st, dt());
        assert!(p1);
    }

    #[test]
    fn curve_mode_discriminants() {
        assert_eq!(TriggerCurve::Linear.mode(), 0);
        assert_eq!(TriggerCurve::Bezier.mode(), 6);
        assert_eq!(TriggerCurve::ApexClassicInverse.mode(), 7);
        assert_eq!(TriggerCurve::ApexClassicInverseAxial.mode(), 8);
    }

    #[test]
    fn default_state_is_clean() {
        let st = TriggerState::default();
        assert!(!st.last_pressed);
        assert_eq!(st.elapsed_us, 0);
    }
}
