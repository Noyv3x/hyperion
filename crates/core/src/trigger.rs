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

/// Two-stage / hip-fire trigger mode (M6; ports the C# `TwoStageTriggerMode`,
/// `ProfilePropGroups.cs:1157`). The default [`TriggerMode::Normal`] is byte-identical to M5:
/// a single digital stage at `max(button_threshold, dead_zone)`, soft == full.
///
/// * `TwoStage` — a **soft-pull** digital stage at `soft_threshold` plus a **full-pull** stage at
///   the raw `255` full-pull; both can be active at once (soft stays on as the trigger crosses to
///   full), so a binding can map "light pull" and "hard pull" to different actions.
/// * `HipFire` — time-gated: when the trigger first engages past the soft threshold a window opens;
///   if the **full pull** lands within `hip_fire_us` only the full-pull stage fires (the soft stage
///   is suppressed — a fast "hip fire"); if the window elapses while only the soft pull is held the
///   soft stage fires (an "aim-then-shoot"). Net-new timing uses the resident [`TriggerState`].
///
/// The variant order/names are an append-only persisted-profile contract.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum TriggerMode {
    /// Single digital stage (M5 behavior; soft == full). Default.
    #[default]
    Normal,
    /// Independent soft-pull + full-pull stages.
    TwoStage,
    /// Time-gated soft/full stages (fast full pull suppresses soft).
    HipFire,
    /// Append-only fallback for an unknown persisted mode (degrades to `Normal`).
    #[serde(other)]
    Unknown,
}

/// The staged result of [`process_trigger_staged`]: the analog `[0,1]` output plus the two digital
/// stages. For [`TriggerMode::Normal`] `soft_pull == full_pull` is the single M5 digital view.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TriggerOutput {
    /// Analog output in `[0,1]` (rounded only at egress) — identical across all modes.
    pub analog: f64,
    /// Soft-pull digital stage (the light-pull / first stage).
    pub soft_pull: bool,
    /// Full-pull digital stage (the hard-pull / second stage).
    pub full_pull: bool,
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
    /// Two-stage / hip-fire mode (M6). `Normal` (default) is byte-identical to M5.
    #[serde(default)]
    pub mode: TriggerMode,
    /// Soft-pull digital threshold in `[0,255]` raw units (the first stage in `TwoStage`/`HipFire`).
    /// Ignored in `Normal`. `0` falls back to `button_threshold` at process time.
    #[serde(default)]
    pub soft_threshold: u8,
    /// Hip-fire window in microseconds (the `HipFire` full-vs-soft race; C# `hipFireMS·1000`).
    #[serde(default = "default_hip_fire_us")]
    pub hip_fire_us: u32,
}

/// C# `DEFAULT_HIP_TIME` (100 ms) expressed in microseconds.
const fn default_hip_fire_us() -> u32 {
    100_000
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
            mode: TriggerMode::Normal,
            soft_threshold: 0,
            hip_fire_us: default_hip_fire_us(),
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
        // Hip-fire window floors at 1 ms so the race is always decidable in a bounded number of
        // reports (and clamps to a sane upper bound).
        out.hip_fire_us = out.hip_fire_us.clamp(1_000, 5_000_000);
        out
    }
}

/// Per-trigger mutable state (resident, one per L2 / R2).
///
/// **`Default` is the clean post-reset state.** `last_pressed` tracks digital edges; `elapsed_us`
/// is the monotonic accumulator; the `hip_*` fields drive the M6 hip-fire race (all bounded, no
/// real time / no threads — see [`process_trigger_staged`]).
#[derive(Default, Clone, Copy, Debug)]
pub struct TriggerState {
    /// Whether the trigger read pressed on the previous report (edge tracking).
    pub last_pressed: bool,
    /// Monotonic microsecond accumulator (advanced by the per-report `dt`).
    pub elapsed_us: i64,
    /// Whether the hip-fire window is currently open (set on soft-engage, cleared on release).
    pub hip_active: bool,
    /// `elapsed_us` snapshot when the current soft engage began (the hip-fire window anchor).
    pub hip_start_us: i64,
}

/// Process one trigger report through the ordered chain (the M5 `(analog, digital)` view).
///
/// * `raw` — the trigger reading in `[0,1]` (`cState.L2 / 255.0`).
/// * `cfg` — the per-trigger settings (already `clamped()`).
/// * `st` — the resident per-trigger state.
/// * `dt` — the guarded per-report elapsed time (advances `elapsed_us`).
///
/// Returns `(analog_out, digital_pressed)`: `analog_out` in `[0,1]` (rounded only at egress),
/// `digital_pressed` is the OR of the two staged digital pulls. For the default
/// [`TriggerMode::Normal`] this is byte-identical to M5 (single stage at
/// `max(button_threshold, dead_zone)`); the two-stage / hip-fire detail is in
/// [`process_trigger_staged`].
pub fn process_trigger(
    raw: f64,
    cfg: &TriggerSettings,
    st: &mut TriggerState,
    dt: Dt,
) -> (f64, bool) {
    let out = process_trigger_staged(raw, cfg, st, dt);
    (out.analog, out.soft_pull || out.full_pull)
}

/// Process one trigger report into the full staged [`TriggerOutput`] (M6).
///
/// The analog chain is identical to M5 (and to `process_trigger`); the digital result depends on
/// [`TriggerSettings::mode`]:
/// * `Normal` — `soft_pull == full_pull ==` the M5 to-button compare. Byte-identical to M5.
/// * `TwoStage` — `soft_pull` at `soft_threshold` (or `button_threshold` if 0), `full_pull` at the
///   raw `255` full pull; both independent.
/// * `HipFire` — opens a `hip_fire_us` window on soft engage; a full pull inside the window fires
///   ONLY `full_pull`; if the window elapses with only the soft pull held, `soft_pull` fires.
///
/// Hip-fire timing reads only the resident `st.elapsed_us` accumulator (advanced by the guarded
/// per-report `dt`), so the decision is pure and bounded — no wall clock, no threads.
pub fn process_trigger_staged(
    raw: f64,
    cfg: &TriggerSettings,
    st: &mut TriggerState,
    dt: Dt,
) -> TriggerOutput {
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

    // --- to-button threshold(s) ---
    // Base digital compare: raw255 vs max(button_threshold, dead_zone) (pre-quantization f64).
    let base_threshold = (cfg.button_threshold as f64).max(dead_zone);
    let base_pressed = raw255 >= base_threshold && base_threshold > 0.0;

    let out = match cfg.mode {
        TriggerMode::Normal | TriggerMode::Unknown => TriggerOutput {
            analog: analog_out,
            soft_pull: base_pressed,
            full_pull: base_pressed,
        },
        TriggerMode::TwoStage => {
            // Soft stage at soft_threshold (or the base threshold when unset); full stage at 255.
            let soft_th = if cfg.soft_threshold > 0 {
                (cfg.soft_threshold as f64).max(dead_zone)
            } else {
                base_threshold
            };
            let soft = raw255 >= soft_th && soft_th > 0.0;
            let full = raw255 >= 255.0;
            TriggerOutput {
                analog: analog_out,
                soft_pull: soft,
                full_pull: full,
            }
        }
        TriggerMode::HipFire => {
            hip_fire_stage(raw255, dead_zone, base_threshold, cfg, st, analog_out)
        }
    };

    // Edge tracking reflects "any digital stage engaged".
    st.last_pressed = out.soft_pull || out.full_pull;
    out
}

/// The hip-fire stage machine (M6): a pure, bounded full-vs-soft race off the resident
/// `st.elapsed_us` accumulator.
///
/// On the rising edge of the soft engage the window opens (`hip_active`, anchored to `elapsed_us`).
/// While engaged: a full pull (`raw255 == 255`) inside `hip_fire_us` fires ONLY `full_pull`; once
/// the window elapses with only the soft pull held, `soft_pull` fires. Release (below the soft
/// threshold) closes the window so the next engage re-anchors.
#[inline]
fn hip_fire_stage(
    raw255: f64,
    dead_zone: f64,
    base_threshold: f64,
    cfg: &TriggerSettings,
    st: &mut TriggerState,
    analog_out: f64,
) -> TriggerOutput {
    let soft_th = if cfg.soft_threshold > 0 {
        (cfg.soft_threshold as f64).max(dead_zone)
    } else {
        base_threshold
    };
    let engaged = soft_th > 0.0 && raw255 >= soft_th;
    let full = raw255 >= 255.0;

    if !engaged {
        // Below the soft threshold: window closed, nothing fires.
        st.hip_active = false;
        return TriggerOutput {
            analog: analog_out,
            soft_pull: false,
            full_pull: false,
        };
    }

    // Engaged: open the window on the rising edge.
    if !st.hip_active {
        st.hip_active = true;
        st.hip_start_us = st.elapsed_us;
    }
    let elapsed = st.elapsed_us.wrapping_sub(st.hip_start_us);
    let within_window = elapsed < i64::from(cfg.hip_fire_us);

    let (soft_pull, full_pull) = if full {
        // A full pull suppresses the soft stage (fast hip fire) regardless of the window.
        (false, true)
    } else if within_window {
        // Still racing: hold both stages off until the full pull lands or the window elapses.
        (false, false)
    } else {
        // Window elapsed with only a soft pull held -> the soft (aim) stage fires.
        (true, false)
    };

    TriggerOutput {
        analog: analog_out,
        soft_pull,
        full_pull,
    }
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
        assert!(!st.hip_active);
        assert_eq!(st.hip_start_us, 0);
    }

    // -------------------------------- M6: two-stage / hip-fire modes -----------------------------

    #[test]
    fn default_mode_is_normal_and_byte_identical() {
        // The default settings are Normal: process_trigger_staged's soft == full == the M5 digital,
        // and process_trigger returns exactly the M5 (analog, digital) pair.
        let cfg = TriggerSettings::default();
        assert_eq!(cfg.mode, TriggerMode::Normal);
        let mut st = TriggerState::default();
        let mut st2 = TriggerState::default();
        for raw in [0.0, 99.0 / 255.0, 100.0 / 255.0, 0.5, 1.0] {
            let staged = process_trigger_staged(raw, &cfg, &mut st, dt());
            let (a, p) = process_trigger(raw, &cfg, &mut st2, dt());
            assert_eq!(staged.soft_pull, staged.full_pull, "Normal: soft == full");
            assert!(approx(staged.analog, a));
            assert_eq!(staged.soft_pull || staged.full_pull, p);
        }
    }

    #[test]
    fn two_stage_soft_then_full() {
        let cfg = TriggerSettings {
            mode: TriggerMode::TwoStage,
            soft_threshold: 60,
            ..TriggerSettings::default()
        };
        let mut st = TriggerState::default();
        // Below soft -> neither.
        let o0 = process_trigger_staged(50.0 / 255.0, &cfg, &mut st, dt());
        assert!(!o0.soft_pull && !o0.full_pull);
        // Past soft, not full -> soft only.
        let o1 = process_trigger_staged(150.0 / 255.0, &cfg, &mut st, dt());
        assert!(o1.soft_pull && !o1.full_pull, "soft engaged, full not yet");
        // Full pull -> BOTH stages (soft stays on through to full).
        let o2 = process_trigger_staged(1.0, &cfg, &mut st, dt());
        assert!(
            o2.soft_pull && o2.full_pull,
            "full pull engages both stages"
        );
    }

    #[test]
    fn two_stage_full_requires_raw_255() {
        let cfg = TriggerSettings {
            mode: TriggerMode::TwoStage,
            soft_threshold: 40,
            ..TriggerSettings::default()
        };
        let mut st = TriggerState::default();
        // 254/255 is past soft but not a full pull.
        let o = process_trigger_staged(254.0 / 255.0, &cfg, &mut st, dt());
        assert!(o.soft_pull && !o.full_pull);
    }

    #[test]
    fn hip_fire_fast_full_pull_suppresses_soft() {
        // A full pull on the very first engaged report (within the window) fires only full_pull.
        let cfg = TriggerSettings {
            mode: TriggerMode::HipFire,
            soft_threshold: 60,
            hip_fire_us: 100_000,
            ..TriggerSettings::default()
        }
        .clamped();
        let mut st = TriggerState::default();
        // dt() is 4ms; one report engaged + full immediately -> full only, soft suppressed.
        let o = process_trigger_staged(1.0, &cfg, &mut st, dt());
        assert!(o.full_pull && !o.soft_pull, "fast full pull -> full only");
    }

    #[test]
    fn hip_fire_held_soft_fires_after_window() {
        // Hold only the soft pull. The window opens on the engage report (when elapsed first
        // reaches 4ms, so hip_start == 4ms) and elapses when elapsed - 4ms >= 10ms, i.e. at
        // elapsed == 14ms -> report 4 (16ms). Reports 1..3 race; report 4 fires the soft (aim) stage.
        let cfg = TriggerSettings {
            mode: TriggerMode::HipFire,
            soft_threshold: 60,
            hip_fire_us: 10_000,
            ..TriggerSettings::default()
        }
        .clamped();
        let mut st = TriggerState::default();
        let soft = 150.0 / 255.0; // past soft, not full
        for report in 1..=3 {
            let o = process_trigger_staged(soft, &cfg, &mut st, dt());
            assert!(
                !o.soft_pull && !o.full_pull,
                "still racing at report {report}"
            );
        }
        let o4 = process_trigger_staged(soft, &cfg, &mut st, dt()); // elapsed 16ms, window elapsed
        assert!(o4.soft_pull && !o4.full_pull, "soft fires after the window");
    }

    #[test]
    fn hip_fire_release_recloses_window() {
        let cfg = TriggerSettings {
            mode: TriggerMode::HipFire,
            soft_threshold: 60,
            hip_fire_us: 10_000,
            ..TriggerSettings::default()
        }
        .clamped();
        let mut st = TriggerState::default();
        let soft = 150.0 / 255.0;
        // Engage and let the window elapse so soft fires.
        for _ in 0..4 {
            process_trigger_staged(soft, &cfg, &mut st, dt());
        }
        assert!(st.hip_active);
        // Release below soft -> window closes, nothing fires.
        let rel = process_trigger_staged(0.0, &cfg, &mut st, dt());
        assert!(!rel.soft_pull && !rel.full_pull && !st.hip_active);
        // Re-engage then immediately full -> full only again (window re-anchored).
        let o = process_trigger_staged(1.0, &cfg, &mut st, dt());
        assert!(
            o.full_pull && !o.soft_pull,
            "re-engaged window suppresses soft"
        );
    }

    #[test]
    fn mode_serde_round_trip_and_unknown_fallback() {
        #[derive(serde::Serialize, serde::Deserialize)]
        struct W {
            m: TriggerMode,
        }
        for m in [
            TriggerMode::Normal,
            TriggerMode::TwoStage,
            TriggerMode::HipFire,
        ] {
            let s = toml::to_string(&W { m }).unwrap();
            let back: W = toml::from_str(&s).unwrap();
            assert_eq!(back.m, m);
        }
        let back: W = toml::from_str("m = \"SomeFutureMode\"\n").unwrap();
        assert_eq!(back.m, TriggerMode::Unknown);
    }

    #[test]
    fn clamped_floors_hip_fire_window() {
        let cfg = TriggerSettings {
            hip_fire_us: 0,
            ..TriggerSettings::default()
        }
        .clamped();
        assert_eq!(cfg.hip_fire_us, 1_000, "hip-fire window floors at 1ms");
    }

    #[test]
    fn unknown_mode_behaves_like_normal() {
        let cfg = TriggerSettings {
            mode: TriggerMode::Unknown,
            ..TriggerSettings::default()
        };
        let mut st = TriggerState::default();
        let o = process_trigger_staged(0.5, &cfg, &mut st, dt());
        assert_eq!(
            o.soft_pull, o.full_pull,
            "Unknown degrades to a single stage"
        );
    }
}
