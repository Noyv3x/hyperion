//! Per-stick settings and resident state for the full DS4Windows-class stick pipeline.
//!
//! [`StickSettings`] is the serde-editable, `Copy` parameter bundle placed inside a `Profile`
//! (one per LS / RS). It ports the C# `StickDeadZoneInfo` / `StickAntiSnapbackInfo` /
//! `SquareStickInfo` / output-curve settings into the `[0,255]`-domain pipeline, and folds the
//! existing [`RcConfig`](crate::rc::RcConfig) in as one field (the RC filter is stage 0).
//!
//! [`StickState`] is the resident per-stick mutable state. **Its [`Default`] is the clean
//! post-reset state** (verifier latency FIX 6): `StickState::default()` is exactly what a fresh
//! enable/`ResetFilter` produces, so the engine's reset arm is `*st = Default::default()`.
//!
//! Every settings field is `#[serde(default)]` so a partial TOML table fills in the C# defaults,
//! and [`StickSettings::clamped`] enforces the §4 ranges (dead_zone `0..=127`, anti/max-zone
//! `0..=100`, max-zone `>= 1`, roundness `>= 1`, Bezier points `∈ [0,1]`).

use crate::rc::RcConfig;

/// Fixed capacity of the anti-snapback history ring (replaces the unbounded C# `Queue`).
///
/// Sized to `MAX(report_rate) * max_timeout` so it never overflows at sane rates; at extreme
/// report rates it silently drops the oldest sample (the same as a too-short timeout window).
pub const SNAP_CAP: usize = 256;

/// Which deadzone model a stick runs.
///
/// Mirrors C# `StickDeadZoneInfo.DeadZoneType` (`Radial = 0`, `Axial = 1`); serialized
/// `PascalCase` to match the profile schema.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum DeadZoneType {
    /// Fused radial anti-dz + max-zone + max-output + vertical-scale (the C# default).
    #[default]
    Radial,
    /// Independent per-axis dead/anti/max (axial deadzone).
    Axial,
}

/// Stick output curve.
///
/// Discriminants match C# `StickOutCurve.Curve` **exactly** so persisted profiles and the
/// `getLsOutCurveMode` integer mode round-trip 1:1: `Linear = 0 .. EaseoutCubic = 5`,
/// `Bezier = 6`, `ApexClassicInverse = 7`, `ApexClassicInverseAxial = 8`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum OutputCurve {
    /// Identity (no curve).
    #[default]
    Linear = 0,
    /// DS4Windows enhanced-precision piecewise curve.
    EnhancedPrecision = 1,
    /// `x^2` (sign-preserving).
    Quadratic = 2,
    /// `x^3`.
    Cubic = 3,
    /// Ease-out quadratic.
    EaseoutQuad = 4,
    /// Ease-out cubic.
    EaseoutCubic = 5,
    /// Custom Bezier (cap-aware, ratio LUT). M3 stub evaluates the Bezier control points.
    Bezier = 6,
    /// Apex Classic inverse — radial direction-preserving signed sqrt.
    ApexClassicInverse = 7,
    /// Apex Classic inverse — axial signed sqrt per axis.
    ApexClassicInverseAxial = 8,
}

impl OutputCurve {
    /// The C# integer curve mode (`getLsOutCurveMode`): `Linear` => 0 (no curve applied).
    #[inline]
    pub const fn mode(self) -> u8 {
        self as u8
    }
}

/// Per-axis deadzone parameters for the axial model (C# `AxisDeadZoneInfo`).
///
/// `dead_zone` is in `[0,127]` axis units (the C# "old bad convention"). Defaults match the
/// C# `Reset()` clean state (everything off), NOT the field initializers.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct AxisDeadZone {
    /// Deadzone in `[0,127]` axis units.
    pub dead_zone: i32,
    /// Anti-deadzone in `[0,100]` percent.
    pub anti_dead_zone: i32,
    /// Max-zone in `[1,100]` percent.
    pub max_zone: i32,
    /// Max-output in `[0,100]` percent.
    pub max_output: f64,
}

impl Default for AxisDeadZone {
    #[inline]
    fn default() -> Self {
        Self {
            dead_zone: 0,
            anti_dead_zone: 0,
            max_zone: 100,
            max_output: 100.0,
        }
    }
}

/// One stick's deadzone settings (C# `StickDeadZoneInfo`).
///
/// `dead_zone` is in `[0,127]` axis units. Defaults match the C# `Reset()` clean state, so a
/// fresh stick passes through unchanged.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct StickDeadZone {
    /// Radial vs axial model.
    #[serde(rename = "type")]
    pub dead_zone_type: DeadZoneType,
    /// Radial deadzone in `[0,127]` axis units.
    pub dead_zone: i32,
    /// Radial anti-deadzone in `[0,100]` percent.
    pub anti_dead_zone: i32,
    /// Radial max-zone in `[1,100]` percent.
    pub max_zone: i32,
    /// Radial max-output in `[0,100]` percent.
    pub max_output: f64,
    /// Force the max-output clamp even at `100.0`.
    pub max_output_force: bool,
    /// Input-fuzz delta (`>0` enables fuzz; `delta^2` gate in axis units).
    pub fuzz: i32,
    /// Vertical-scale in percent (`100.0` == off).
    pub vertical_scale: f64,
    /// Axial X-axis parameters (used when `dead_zone_type == Axial`).
    pub x_axis: AxisDeadZone,
    /// Axial Y-axis parameters (used when `dead_zone_type == Axial`).
    pub y_axis: AxisDeadZone,
}

/// C# `StickDeadZoneInfo.DEFAULT_VERTICAL_SCALE`.
pub const DEFAULT_VERTICAL_SCALE: f64 = 100.0;

impl Default for StickDeadZone {
    #[inline]
    fn default() -> Self {
        Self {
            dead_zone_type: DeadZoneType::Radial,
            dead_zone: 0,
            anti_dead_zone: 0,
            max_zone: 100,
            max_output: 100.0,
            max_output_force: false,
            fuzz: 0,
            vertical_scale: DEFAULT_VERTICAL_SCALE,
            x_axis: AxisDeadZone::default(),
            y_axis: AxisDeadZone::default(),
        }
    }
}

/// Stick rotation (C# `getLSRotation` / `rotateLSCoordinates`).
#[derive(Clone, Copy, Debug, PartialEq, Default, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct RotationSettings {
    /// Rotation angle in radians (`0.0` == off).
    pub angle_rad: f64,
}

/// Anti-snapback (C# `StickAntiSnapbackInfo`).
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct AntiSnapback {
    /// Whether anti-snapback is active.
    pub enabled: bool,
    /// Trigger distance in axis units (`DEFAULT_DELTA == 135`).
    pub delta: f64,
    /// Look-back window in milliseconds (`DEFAULT_TIMEOUT == 50`).
    pub timeout_ms: i64,
}

impl Default for AntiSnapback {
    #[inline]
    fn default() -> Self {
        // C# StickAntiSnapbackInfo defaults: disabled, delta 135, timeout 50ms.
        Self {
            enabled: false,
            delta: 135.0,
            timeout_ms: 50,
        }
    }
}

/// Square-stick / circle-to-square (C# `SquareStickInfo`).
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct SquareStick {
    /// Whether circle-to-square is active for this stick.
    pub enabled: bool,
    /// Roundness exponent (`DEFAULT_ROUNDNESS == 5.0`, clamped `>= 1`).
    pub roundness: f64,
}

impl Default for SquareStick {
    #[inline]
    fn default() -> Self {
        // C# SquareStickInfo defaults: off, roundness 5.0.
        Self {
            enabled: false,
            roundness: 5.0,
        }
    }
}

/// Flick-stick settings (terminal stage; the consumer is M5, the data model is M3).
///
/// The reserved tuning interface for the OneEuro mouse contract is carried now so M5 is
/// purely additive (verifier — flick-stick delta units vs the mouse contract are HW-gated).
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct FlickStick {
    /// Whether flick stick is active.
    pub enabled: bool,
    /// Real-world calibration (degrees per full flick), reserved for the M5 mouse path.
    pub real_world_calibration: f64,
    /// OneEuro min-cutoff, reserved for the M5 mouse path.
    pub min_cutoff: f64,
    /// OneEuro beta, reserved for the M5 mouse path.
    pub beta: f64,
}

impl Default for FlickStick {
    #[inline]
    fn default() -> Self {
        Self {
            enabled: false,
            real_world_calibration: 360.0,
            min_cutoff: 1.0,
            beta: 0.7,
        }
    }
}

/// Full per-stick settings (one per LS / RS), placed inside a `Profile`.
///
/// `Copy` and small so the hot loop copies it into the resident `[StickSettings; 2]` on the
/// generation gate. `rc` folds the existing RC filter config in as stage 0; `rc_mode_on` gates
/// it (mirrors the C# `StickAlgorithmMode == RC` selector, distinct from `rc.enabled`).
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct StickSettings {
    /// RC filter config (stage 0). Active only when `rc_mode_on && rc.enabled`.
    pub rc: RcConfig,
    /// Whether the RC stage is selected for this stick (C# `StickAlgorithmMode == RC`).
    pub rc_mode_on: bool,
    /// Stage 1 rotation.
    pub rotation: RotationSettings,
    /// Stage 2 anti-snapback.
    pub anti_snapback: AntiSnapback,
    /// Stage 5 deadzone (radial or axial) + stage 3 fuzz delta (carried on the deadzone struct).
    pub dead_zone: StickDeadZone,
    /// Stage 6 sensitivity (radial-only; axial silently ignores it — C# quirk preserved).
    pub sensitivity: f64,
    /// Stage 7 square-stick.
    pub square: SquareStick,
    /// Stage 8 output curve.
    pub curve: OutputCurve,
    /// Stage 9 flick stick (terminal; M3 stashes the delta, M5 consumes it).
    pub flick: FlickStick,
}

// `StickSettings::default()` must mean "pass-through": sensitivity 1.0, all stages off.
// `#[derive(Default)]` would give sensitivity 0.0, so a hand-written impl is required.
impl Default for StickSettings {
    #[inline]
    fn default() -> Self {
        Self {
            rc: RcConfig::default(),
            rc_mode_on: false,
            rotation: RotationSettings::default(),
            anti_snapback: AntiSnapback::default(),
            dead_zone: StickDeadZone::default(),
            sensitivity: 1.0,
            square: SquareStick::default(),
            curve: OutputCurve::default(),
            flick: FlickStick::default(),
        }
    }
}

impl StickSettings {
    /// A copy with every §4 range enforced. Mirrors the C# clamp funnels so the persisted and
    /// runtime values agree.
    pub fn clamped(&self) -> StickSettings {
        let mut out = *self;
        out.rc = out.rc.clamped();

        // Radial deadzone ranges.
        out.dead_zone.dead_zone = out.dead_zone.dead_zone.clamp(0, 127);
        out.dead_zone.anti_dead_zone = out.dead_zone.anti_dead_zone.clamp(0, 100);
        out.dead_zone.max_zone = out.dead_zone.max_zone.clamp(1, 100);
        out.dead_zone.max_output = out.dead_zone.max_output.clamp(0.0, 100.0);
        out.dead_zone.fuzz = out.dead_zone.fuzz.max(0);
        out.dead_zone.vertical_scale = out.dead_zone.vertical_scale.clamp(0.0, 100.0);

        // Axial per-axis deadzone ranges.
        for axis in [&mut out.dead_zone.x_axis, &mut out.dead_zone.y_axis] {
            axis.dead_zone = axis.dead_zone.clamp(0, 127);
            axis.anti_dead_zone = axis.anti_dead_zone.clamp(0, 100);
            axis.max_zone = axis.max_zone.clamp(1, 100);
            axis.max_output = axis.max_output.clamp(0.0, 100.0);
        }

        // Anti-snapback: non-negative delta + timeout.
        out.anti_snapback.delta = out.anti_snapback.delta.max(0.0);
        out.anti_snapback.timeout_ms = out.anti_snapback.timeout_ms.max(0);

        // Square stick roundness >= 1.
        out.square.roundness = out.square.roundness.max(1.0);

        // Flick stick reserved tuning: keep finite, non-negative cutoff.
        out.flick.real_world_calibration = out.flick.real_world_calibration.max(0.0);
        out.flick.min_cutoff = out.flick.min_cutoff.max(0.0);

        out
    }
}

/// Fixed-capacity (x, y, t_us) ring for anti-snapback history (replaces the C# unbounded
/// `Queue<DS4TimedStickAxisValue>`).
///
/// All samples are stored in the `[0,255]` axis domain with a monotonic `t_us` timestamp drawn
/// from the per-stick `elapsed_us` accumulator (no wall clock — the core stays pure/Linux-testable).
/// Pruning drops samples older than the timeout window; on overflow the oldest sample is dropped.
#[derive(Clone, Copy, Debug)]
pub struct SnapbackRing {
    buf: [(f64, f64, i64); SNAP_CAP],
    head: usize,
    len: usize,
}

impl Default for SnapbackRing {
    #[inline]
    fn default() -> Self {
        Self {
            buf: [(0.0, 0.0, 0); SNAP_CAP],
            head: 0,
            len: 0,
        }
    }
}

impl SnapbackRing {
    /// Number of buffered samples.
    #[inline]
    pub fn len(&self) -> usize {
        self.len
    }

    /// Whether the ring is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Clear the ring.
    #[inline]
    pub fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
    }

    /// Drop samples whose timestamp is older than `cutoff_us` (front of the queue).
    #[inline]
    pub fn prune_older_than(&mut self, cutoff_us: i64) {
        while self.len > 0 {
            let (_, _, t) = self.buf[self.head];
            if t < cutoff_us {
                self.head = (self.head + 1) % SNAP_CAP;
                self.len -= 1;
            } else {
                break;
            }
        }
    }

    /// Push a sample, dropping the oldest if full (bounded; no alloc/panic).
    #[inline]
    pub fn push(&mut self, x: f64, y: f64, t_us: i64) {
        let tail = (self.head + self.len) % SNAP_CAP;
        if self.len == SNAP_CAP {
            // Overwrite the oldest: advance head, keep len at cap.
            self.buf[self.head] = (x, y, t_us);
            self.head = (self.head + 1) % SNAP_CAP;
        } else {
            self.buf[tail] = (x, y, t_us);
            self.len += 1;
        }
    }

    /// Visit each buffered `(x, y, t_us)` sample, oldest first.
    #[inline]
    pub fn for_each(&self, mut f: impl FnMut(f64, f64, i64)) {
        for i in 0..self.len {
            let idx = (self.head + i) % SNAP_CAP;
            let (x, y, t) = self.buf[idx];
            f(x, y, t);
        }
    }
}

/// All per-stick mutable pipeline state (resident, one per LS / RS).
///
/// **`Default` is the clean post-reset state** (verifier latency FIX 6): the engine's reset arm
/// is `*st = StickState::default()`, so any field added here must default to its clean value.
#[derive(Default)]
pub struct StickState {
    /// Existing RC filter state (stage 0), unchanged.
    pub rc: crate::rc::RcStickState,
    /// Whether the RC stage has been primed (moved out of the engine's `primed[]`).
    pub rc_primed: bool,
    /// Monotonic microsecond accumulator (replaces the C# wall clock for anti-snapback timing).
    pub elapsed_us: i64,
    /// Last emitted fuzz X/Y (axis domain) for the `delta^2` gate.
    pub fuzz_last: [f64; 2],
    /// Whether fuzz history has been seeded.
    pub fuzz_primed: bool,
    /// Anti-snapback history ring (fixed-cap; replaces the unbounded C# `Queue`).
    pub snap_hist: SnapbackRing,
    /// Flick: whether a flick is in progress (M5 consumer).
    pub flick_in_progress: bool,
    /// Flick: last stick angle (radians); the anchor the per-report relative turn differences
    /// against. Flick is **single-sweep** in v1 — stage 9 emits the per-report angular delta
    /// (`flick_delta`) directly, so there is no separate in-progress remaining-angle accumulator
    /// (the dead `flick_angle_remaining` field was removed in M7). A whole-flick remaining-angle
    /// model (snap a fixed sweep over N reports) is a future, additive change if HW tuning wants it.
    pub flick_last_angle: f64,
    /// OUT: per-report relative turn for the mouse path (M5 consumer; written by stage 9).
    pub flick_delta: f64,
}

impl StickState {
    /// Per-report prime path (engine `input.is_prime`), distinct from the command-driven full
    /// reset. Clears the history that must be re-seeded from the first post-enable report (RC
    /// prime, fuzz seed, anti-snapback window, flick anchor) while leaving the monotonic
    /// `elapsed_us` accumulator running. The next [`process_stick`](super::pipeline::process_stick)
    /// call re-primes RC and fuzz from the incoming sample.
    #[inline]
    pub fn prime_reset(&mut self) {
        self.rc = crate::rc::RcStickState::default();
        self.rc_primed = false;
        self.fuzz_primed = false;
        self.fuzz_last = [0.0, 0.0];
        self.snap_hist.clear();
        self.flick_in_progress = false;
        self.flick_last_angle = 0.0;
        self.flick_delta = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_settings_are_passthrough() {
        let s = StickSettings::default();
        assert_eq!(s.sensitivity, 1.0);
        assert!(!s.rc_mode_on);
        assert!(!s.rc.enabled);
        assert_eq!(s.dead_zone.dead_zone, 0);
        assert_eq!(s.dead_zone.anti_dead_zone, 0);
        assert_eq!(s.dead_zone.max_zone, 100);
        assert_eq!(s.dead_zone.max_output, 100.0);
        assert_eq!(s.dead_zone.fuzz, 0);
        assert_eq!(s.dead_zone.vertical_scale, 100.0);
        assert_eq!(s.dead_zone.dead_zone_type, DeadZoneType::Radial);
        assert!(!s.anti_snapback.enabled);
        assert!(!s.square.enabled);
        assert_eq!(s.curve, OutputCurve::Linear);
        assert!(!s.flick.enabled);
    }

    #[test]
    fn default_state_is_clean() {
        let st = StickState::default();
        assert!(!st.rc_primed);
        assert_eq!(st.elapsed_us, 0);
        assert!(!st.fuzz_primed);
        assert_eq!(st.fuzz_last, [0.0, 0.0]);
        assert!(st.snap_hist.is_empty());
        assert!(!st.flick_in_progress);
        assert_eq!(st.flick_delta, 0.0);
    }

    #[test]
    fn curve_discriminants_match_csharp() {
        assert_eq!(OutputCurve::Linear.mode(), 0);
        assert_eq!(OutputCurve::EnhancedPrecision.mode(), 1);
        assert_eq!(OutputCurve::Quadratic.mode(), 2);
        assert_eq!(OutputCurve::Cubic.mode(), 3);
        assert_eq!(OutputCurve::EaseoutQuad.mode(), 4);
        assert_eq!(OutputCurve::EaseoutCubic.mode(), 5);
        assert_eq!(OutputCurve::Bezier.mode(), 6);
        assert_eq!(OutputCurve::ApexClassicInverse.mode(), 7);
        assert_eq!(OutputCurve::ApexClassicInverseAxial.mode(), 8);
    }

    #[test]
    fn clamped_enforces_ranges() {
        let s = StickSettings {
            sensitivity: 2.0,
            dead_zone: StickDeadZone {
                dead_zone: 200,
                anti_dead_zone: 150,
                max_zone: 0,
                max_output: 200.0,
                fuzz: -5,
                vertical_scale: -10.0,
                ..StickDeadZone::default()
            },
            anti_snapback: AntiSnapback {
                delta: -1.0,
                timeout_ms: -5,
                ..AntiSnapback::default()
            },
            square: SquareStick {
                roundness: 0.2,
                ..SquareStick::default()
            },
            ..StickSettings::default()
        };
        let c = s.clamped();
        assert_eq!(c.dead_zone.dead_zone, 127);
        assert_eq!(c.dead_zone.anti_dead_zone, 100);
        assert_eq!(c.dead_zone.max_zone, 1);
        assert_eq!(c.dead_zone.max_output, 100.0);
        assert_eq!(c.dead_zone.fuzz, 0);
        assert_eq!(c.dead_zone.vertical_scale, 0.0);
        assert_eq!(c.anti_snapback.delta, 0.0);
        assert_eq!(c.anti_snapback.timeout_ms, 0);
        assert_eq!(c.square.roundness, 1.0);
        // sensitivity is unconstrained (radial multiplier); preserved as-is.
        assert_eq!(c.sensitivity, 2.0);
    }

    #[test]
    fn snapback_ring_prunes_and_caps() {
        let mut r = SnapbackRing::default();
        for i in 0..10i64 {
            r.push(i as f64, 0.0, i);
        }
        assert_eq!(r.len(), 10);
        r.prune_older_than(5);
        // samples with t < 5 dropped: t in {0,1,2,3,4}
        assert_eq!(r.len(), 5);
        let mut first = None;
        r.for_each(|x, _, t| {
            if first.is_none() {
                first = Some((x, t));
            }
        });
        assert_eq!(first, Some((5.0, 5)));

        // Overflow: push more than SNAP_CAP, len stays capped, oldest dropped.
        let mut r2 = SnapbackRing::default();
        for i in 0..(SNAP_CAP as i64 + 50) {
            r2.push(i as f64, 0.0, i);
        }
        assert_eq!(r2.len(), SNAP_CAP);
        let mut min_t = i64::MAX;
        r2.for_each(|_, _, t| min_t = min_t.min(t));
        assert_eq!(min_t, 50);
    }
}
