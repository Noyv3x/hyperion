//! `MouseAccumulator` — the stick→mouse (and gyro→mouse) remainder-carry accumulator.
//!
//! Pure, `Copy`, allocation-free, OS-free (Linux-CI-tested). Ports DS4Windows'
//! `MouseCursor`/`Mapping.calculateFinalMouseMovement` sub-pixel remainder carry to `f64` exactly
//! (blueprint §6.2). The accumulator is **called from [`apply`](crate::map::apply)** (verifier
//! FIX 7): the engine feeds it the per-report stick deflection + elapsed time, it returns the
//! integer `(dx, dy)` to inject, and it carries the fractional remainder in resident `MapState`
//! state so relative-mouse feel stays deterministic and DS4Windows-faithful.
//!
//! ## Ported C# semantics (ground truth `Hyperion-ds4w/.../MouseCursor.cs` + `Mapping.cs`)
//!
//! 1. **Velocity model** (`getMouseMapping`, `ControlType.AxisDir`):
//!    `value = (mouseVelocity − offset_axis)·dt·diff + offset_axis·sign·dt`
//!    where `diff` is the normalized stick deflection past the dead-zone, `mouseVelocity =
//!    sensitivity · MOUSESPEEDFACTOR (48)`, `offset = mouseVelocityOffset · mouseVelocity`, and
//!    `offset_axis = |unit_component| · offset` (the per-axis split of the anti-jitter offset).
//!    `dt` is `timeElapsed · 0.001` (ms→the C# `timeDelta`; here we pass `elapsed_s`-derived ms).
//! 2. **Direction split** via `atan2` (the unit vector `(|cos|, |sin|)`), so a diagonal push splits
//!    the offset between the two axes exactly like DS4Windows.
//! 3. **Sub-pixel cutoff** then **remainder carry** (`calculateFinalMouseMovement`): add the carried
//!    remainder back **only when its sign matches** the new motion (else reset it to 0), truncate to
//!    two decimals via [`remainder_cutoff`] (`x − cutoff(x·100,1)/100`), take the integer part,
//!    store the leftover fraction as the new remainder. **Sign-flip resets the remainder.**
//! 4. **`min_threshold` gate** (the `MouseCursor` `minThreshold` branch): when `min_threshold != 1`,
//!    if the post-cutoff distance² is below `min_threshold²`, **defer** the whole motion (emit 0,
//!    keep the full `xMotion` as the remainder) until it accumulates past the gate.
//! 5. **Invert** flags negate the final integer action; **deadzone** suppresses sub-deadzone
//!    deflection; **acceleration** (optional) raises `diff` to `accel_power`.
//!
//! The `MOUSESPEEDFACTOR`, `MOUSE_OFFSET_DEFAULT`, and the `min_threshold == 1.0` special case are
//! the load-bearing precision behavior — pinned in the unit tests below.

/// DS4Windows `MOUSESPEEDFACTOR` (`Mapping.cs:837`): stick-mouse velocity scale per sensitivity unit.
pub const MOUSE_SPEED_FACTOR: f64 = 48.0;

/// DS4Windows default `mouseVelocityOffset` (anti-jitter start offset, fraction of velocity).
pub const MOUSE_VELOCITY_OFFSET_DEFAULT: f64 = 0.012;

// ----------------------------------- gyro→mouse (M5) constants ----------------------------------
//
// Ground truth: `Hyperion-ds4w/.../MouseCursor.cs::sixaxisMoved`. The gyro path is the same
// remainder-carry/min-threshold/cutoff machinery as the stick path, but with a *velocity model in
// the gyro-rate domain*: `xMotion = coefficient·(delta·tempDouble) + normX·offset·signX`.

/// DS4Windows gyro-mouse time base (`MouseCursor.cs:89`): `tempDouble = elapsed · GYRO_MOUSE_SPEED`
/// scales the per-report rate so the default speed is calibrated on a 5 ms report
/// (`0.005 · 200 == 1.0`). The C# `elapsed` is already in seconds (`arg.sixAxis.elapsed`).
pub const GYRO_MOUSE_SPEED: f64 = 200.0;

/// DS4Windows gyro `mouseCoefficient` baseline (`GyroMouseSens.mouseCoefficient`, the per-device
/// hardware scale folded into the sensitivity coefficient). Bundled into the sensitivity term so a
/// `GyroAccumCfg::sensitivity` of `1.0` reproduces the C# `gyroSensitivity == 100` (`·0.01`) default.
pub const GYRO_MOUSE_COEFFICIENT: f64 = 0.012;

/// DS4Windows gyro anti-jitter offset (`GyroMouseSens.mouseOffset`, `MouseCursor.cs:96`): the
/// direction-split start offset added past the dead-zone so a tiny tilt still nudges the cursor.
pub const GYRO_MOUSE_OFFSET_DEFAULT: f64 = 0.16;

/// DS4Windows default gyro cursor dead-zone in the gyro-rate domain (`GYRO_MOUSE_DEADZONE == 10`,
/// `MouseCursor.cs:61`). Applied as `signX·|normX·deadzone|` subtracted from the rate (NOT a
/// normalized stick dead-zone — the gyro path dead-zones in rate units).
pub const GYRO_MOUSE_DEADZONE_DEFAULT: f64 = 10.0;

/// DS4Windows gyro jitter-compensation threshold (`MouseCursor.cs:149`, `const threshold = 0.26`)
/// and exponent (`Math.Pow(absX/threshold, 1.408)`): below `normX·threshold` the motion is eased
/// with a `^1.408` curve so micro-jitter is suppressed without a hard dead-zone.
pub const GYRO_JITTER_THRESHOLD: f64 = 0.26;
/// Gyro jitter-compensation exponent (see [`GYRO_JITTER_THRESHOLD`]).
pub const GYRO_JITTER_POWER: f64 = 1.408;

/// The C# sub-pixel cutoff: `dividend − divisor · trunc(dividend / divisor)`.
///
/// Ports `Mapping.remainderCutoff` (which uses an `(int)` truncation; we use `f64::trunc`, identical
/// for the in-range magnitudes a per-report mouse delta produces). Used as
/// `x − remainder_cutoff(x·100, 1)/100` to truncate `x` to two decimal places **toward zero**.
#[inline]
#[must_use]
pub fn remainder_cutoff(dividend: f64, divisor: f64) -> f64 {
    dividend - divisor * (dividend / divisor).trunc()
}

/// `Math.Sign`-equivalent: `-1`, `0`, or `+1` (note `f64::signum` returns `±1` for zero, which would
/// mis-sign the gyro dead-zone subtraction — DS4Windows uses `Math.Sign`, which is `0` at `0`).
///
/// `NaN` (which has no ordering) maps to `0.0`, keeping the gyro path total — though the public
/// [`MouseAccumulator::gyro_velocity_step`] already rejects non-finite samples up front.
#[inline]
fn signum_strict(v: f64) -> f64 {
    use core::cmp::Ordering;
    match v.partial_cmp(&0.0) {
        Some(Ordering::Greater) => 1.0,
        Some(Ordering::Less) => -1.0,
        _ => 0.0,
    }
}

/// The gyro jitter-compensation ease (`MouseCursor.cs:146-163`): where `|motion| ≤ norm·threshold`,
/// reshape it as `sign·(|motion|/threshold)^1.408·threshold` so micro-motion is suppressed without a
/// hard dead-zone; larger motions pass through unchanged.
#[inline]
fn jitter_ease(motion: f64, norm: f64) -> f64 {
    let abs = motion.abs();
    if abs <= norm * GYRO_JITTER_THRESHOLD {
        signum_strict(motion)
            * (abs / GYRO_JITTER_THRESHOLD).powf(GYRO_JITTER_POWER)
            * GYRO_JITTER_THRESHOLD
    } else {
        motion
    }
}

/// Tunable mouse-from-stick / gyro settings the accumulator consumes (the resolved, hot-facing
/// form). All `Copy`; derived from the profile's `MouseSettings` off the hot path.
///
/// Field semantics mirror DS4Windows `ButtonMouseInfo` / `GyroMouseInfo`:
/// * `sensitivity` — `activeButtonSensitivity` (stick) scaled by [`MOUSE_SPEED_FACTOR`].
/// * `vertical_scale` — `buttonVerticalScale` applied to the Y velocity only.
/// * `velocity_offset` — `mouseVelocityOffset` anti-jitter start offset.
/// * `deadzone` — normalized stick dead-zone `[0,1)` below which deflection is ignored
///   (DS4Windows uses a hard `3/127` floor when the configured stick dead-zone is 0; expressed here
///   in normalized units).
/// * `min_threshold` — the per-report motion gate; `1.0` is the DS4Windows "no gate, always carry"
///   special case.
/// * `accelerate` / `accel_power` — optional power-curve on the normalized deflection.
/// * `invert_x` / `invert_y` — negate the final integer delta per axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MouseAccumCfg {
    /// Base sensitivity (DS4Windows `activeButtonSensitivity`, typically ~25..100).
    pub sensitivity: f64,
    /// Extra Y-velocity scale (DS4Windows `buttonVerticalScale`).
    pub vertical_scale: f64,
    /// Anti-jitter velocity offset fraction (DS4Windows `mouseVelocityOffset`).
    pub velocity_offset: f64,
    /// Normalized stick dead-zone in `[0,1)`.
    pub deadzone: f64,
    /// Per-report motion gate (`1.0` == always-carry special case).
    pub min_threshold: f64,
    /// Apply the `accel_power` curve to the normalized deflection.
    pub accelerate: bool,
    /// Acceleration exponent applied to `diff` when `accelerate` is set.
    pub accel_power: f64,
    /// Invert the horizontal (X) output.
    pub invert_x: bool,
    /// Invert the vertical (Y) output.
    pub invert_y: bool,
}

impl Default for MouseAccumCfg {
    /// DS4Windows-class defaults: sensitivity 25, no extra vertical scale, default anti-jitter
    /// offset, no dead-zone, the `min_threshold == 1.0` always-carry gate, no acceleration, no
    /// inversion.
    fn default() -> Self {
        Self {
            sensitivity: 25.0,
            vertical_scale: 1.0,
            velocity_offset: MOUSE_VELOCITY_OFFSET_DEFAULT,
            deadzone: 0.0,
            min_threshold: 1.0,
            accelerate: false,
            accel_power: 1.0,
            invert_x: false,
            invert_y: false,
        }
    }
}

/// Tunable gyro→mouse settings the accumulator consumes (the resolved, hot-facing form). All
/// `Copy`; derived from the profile's `GyroSettings` off the hot path via
/// [`GyroSettings::to_accum_cfg`](crate::map::profile::GyroSettings::to_accum_cfg).
///
/// This is the **velocity-model** form of the gyro path (`MouseCursor.cs::sixaxisMoved`), distinct
/// from the stick path's [`MouseAccumCfg`]: the gyro path scales the gyro *rate* by a coefficient ×
/// time base, dead-zones in the **rate domain** (not a normalized stick dead-zone), and applies a
/// jitter-comp ease curve. The remainder-carry / `min_threshold` gate is shared via
/// [`MouseAccumulator::finalize_core`].
///
/// Field semantics mirror DS4Windows `GyroMouseSens` / `GyroMouseInfo`:
/// * `sensitivity` — `gyroSensitivity·0.01` folded with [`GYRO_MOUSE_COEFFICIENT`]; the master
///   speed. `1.0` ≈ the C# `gyroSensitivity == 100` default.
/// * `vertical_scale` — `gyroSensVerticalScale·0.01`, multiplies the Y velocity only.
/// * `velocity_offset` — `mouseOffset`, the direction-split anti-jitter start offset
///   ([`GYRO_MOUSE_OFFSET_DEFAULT`]).
/// * `deadzone` — `gyroCursorDeadZone` in the **gyro-rate domain** ([`GYRO_MOUSE_DEADZONE_DEFAULT`]);
///   the units match `gyro_x`/`gyro_z` as passed to [`gyro_velocity_step`](MouseAccumulator::gyro_velocity_step).
/// * `min_threshold` — the per-report motion gate (`1.0` == always-carry special case).
/// * `jitter_comp` — enable the `^1.408` ease-in below the [`GYRO_JITTER_THRESHOLD`].
/// * `invert_x` / `invert_y` — negate the final integer delta per axis (DS4Windows `gyroInvert`
///   bit `0x02` → X, `0x01` → Y).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GyroAccumCfg {
    /// Master gyro→mouse speed (`gyroSensitivity·0.01 · mouseCoefficient`).
    pub sensitivity: f64,
    /// Extra Y-velocity scale (`gyroSensVerticalScale·0.01`).
    pub vertical_scale: f64,
    /// Direction-split anti-jitter offset (`mouseOffset`).
    pub velocity_offset: f64,
    /// Gyro-rate-domain dead-zone (`gyroCursorDeadZone`).
    pub deadzone: f64,
    /// Per-report motion gate (`1.0` == always-carry special case).
    pub min_threshold: f64,
    /// Apply the `^1.408` jitter-compensation ease curve below the threshold.
    pub jitter_comp: bool,
    /// Invert the horizontal (X) output.
    pub invert_x: bool,
    /// Invert the vertical (Y) output.
    pub invert_y: bool,
}

impl Default for GyroAccumCfg {
    /// DS4Windows-class gyro defaults: unity sensitivity/vertical-scale, the default rate dead-zone
    /// and anti-jitter offset, the always-carry gate, jitter compensation on, no inversion.
    fn default() -> Self {
        Self {
            sensitivity: 1.0,
            vertical_scale: 1.0,
            velocity_offset: GYRO_MOUSE_OFFSET_DEFAULT,
            deadzone: GYRO_MOUSE_DEADZONE_DEFAULT,
            min_threshold: 1.0,
            jitter_comp: true,
            invert_x: false,
            invert_y: false,
        }
    }
}

/// Sub-pixel remainder-carry mouse accumulator (blueprint §6.2).
///
/// Holds only the per-axis fractional remainder; `Default`/[`reset`](Self::reset) is the clean
/// post-reset state. The integer delta is computed entirely inside [`stick_step`](Self::stick_step)
/// / [`gyro_step`](Self::gyro_step) / [`gyro_velocity_step`](Self::gyro_velocity_step); the KBM sink
/// only injects the result.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MouseAccumulator {
    /// Carried horizontal fractional remainder.
    h_remainder: f64,
    /// Carried vertical fractional remainder.
    v_remainder: f64,
}

impl MouseAccumulator {
    /// An empty accumulator (clean post-reset state).
    #[inline]
    #[must_use]
    pub const fn new() -> Self {
        Self {
            h_remainder: 0.0,
            v_remainder: 0.0,
        }
    }

    /// Clear the carried remainder (DS4Windows `mouseRemainderReset` / stick-direction change).
    #[inline]
    pub fn reset(&mut self) {
        self.h_remainder = 0.0;
        self.v_remainder = 0.0;
    }

    /// The carried `(horizontal, vertical)` remainder (test/debug visibility).
    #[inline]
    #[must_use]
    pub fn remainder(&self) -> (f64, f64) {
        (self.h_remainder, self.v_remainder)
    }

    /// Step the accumulator from a signed stick deflection and return the integer `(dx, dy)`.
    ///
    /// `nx`/`ny` are the **signed** stick components in `[-1, 1]` (`+x` = right, `+y` = up, the
    /// core convention). `elapsed_s` is the report interval in seconds. Ports the DS4Windows
    /// `getMouseMapping` AxisDir velocity model + `calculateFinalMouseMovement` carry. `+y` (up) is
    /// converted to screen-space (`dy` positive = down) so the relative-mouse output matches the
    /// physical "push up = look up" feel after the OS Y-down convention.
    pub fn stick_step(
        &mut self,
        nx: f64,
        ny: f64,
        elapsed_s: f64,
        cfg: &MouseAccumCfg,
    ) -> (i32, i32) {
        // Screen space: stick +y (up) maps to mouse −y (up on screen is negative dy).
        let raw_x = Self::axis_velocity(nx, cfg);
        let raw_y = Self::axis_velocity(-ny, cfg);

        // Direction split (atan2 unit vector) for the per-axis anti-jitter offset, matching the C#
        // `Math.Atan2(-deltaY, deltaX)` → `(|cos|, |sin|)` decomposition.
        let dt_ms = elapsed_s * 1000.0;
        let (off_x, off_y) = Self::offset_split(raw_x, raw_y, cfg, dt_ms);

        // C#: value = (velocity − off)·dt·diff + off·sign·dt. With raw == sign·diff, the velocity
        // term is (velocity·dt − off)·raw and the offset term carries the sign explicitly.
        let motion_x = if raw_x != 0.0 {
            (Self::velocity(cfg) * dt_ms - off_x) * raw_x + off_x * raw_x.signum()
        } else {
            0.0
        };
        let motion_y = if raw_y != 0.0 {
            let vel_y = Self::velocity(cfg) * cfg.vertical_scale;
            (vel_y * dt_ms - off_y) * raw_y + off_y * raw_y.signum()
        } else {
            0.0
        };

        self.finalize(motion_x, motion_y, cfg)
    }

    /// Step the accumulator from a **pre-scaled** gyro `(raw_dx, raw_dy)` velocity and return
    /// `(dx, dy)` (the low-level carry primitive; prefer [`gyro_velocity_step`](Self::gyro_velocity_step)
    /// for the full M5 gyro→mouse model).
    ///
    /// This applies the same offset-split + carry + threshold gate as the stick path but takes
    /// already-rate-scaled deltas (the caller owns the gyro→velocity scaling). Kept as the building
    /// block the M4 wiring exercised; `elapsed_s` only feeds the offset split.
    pub fn gyro_step(
        &mut self,
        raw_dx: f64,
        raw_dy: f64,
        elapsed_s: f64,
        cfg: &MouseAccumCfg,
    ) -> (i32, i32) {
        let dt_ms = elapsed_s * 1000.0;
        let (off_x, off_y) = Self::offset_split(raw_dx, raw_dy, cfg, dt_ms);
        let motion_x = if raw_dx != 0.0 {
            raw_dx + off_x * raw_dx.signum()
        } else {
            0.0
        };
        let motion_y = if raw_dy != 0.0 {
            raw_dy + off_y * raw_dy.signum()
        } else {
            0.0
        };
        self.finalize(motion_x, motion_y, cfg)
    }

    /// **M5 gyro→mouse**: step the accumulator from the gyro rates and return the integer `(dx, dy)`,
    /// applying the full DS4Windows `MouseCursor.sixaxisMoved` velocity model.
    ///
    /// `gyro_x` is the horizontal gyro rate (DS4Windows `deltaX` = yaw, or roll when the horizontal
    /// axis is swapped) and `gyro_z` is the vertical rate (DS4Windows `deltaY = -pitch`). The caller
    /// passes them already sign-oriented so that **+`gyro_x` turns the cursor right** and **+`gyro_z`
    /// moves the cursor down** (screen space). `elapsed_s` is the report interval in seconds.
    ///
    /// Ported semantics (`MouseCursor.cs:82-262`):
    /// 1. `tempDouble = elapsed_s · GYRO_MOUSE_SPEED` (default speed calibrated on a 5 ms report).
    /// 2. `coefficient = sensitivity · GYRO_MOUSE_COEFFICIENT`.
    /// 3. Direction split `tempAngle = atan2(-gyro_z, gyro_x)` → `(|cos|, |sin|)` for the per-axis
    ///    offset + dead-zone scaling.
    /// 4. **Rate-domain dead-zone**: subtract `sign·|norm·deadzone|` from each rate (zeroing it when
    ///    inside the dead-zone) — NOT a normalized stick dead-zone.
    /// 5. `xMotion = coefficient·(deltaX·tempDouble) + normX·offset·signX`;
    ///    `yMotion = (coefficient·vertical_scale)·(deltaY·tempDouble) + normY·offset·signY`.
    /// 6. Optional jitter compensation: where `|motion| ≤ norm·threshold`, ease via
    ///    `sign·(|motion|/threshold)^1.408·threshold`.
    /// 7. Shared remainder-carry + `min_threshold` gate + per-axis invert ([`finalize_core`]).
    ///
    /// The OneEuro / weighted-average smoothing branch (`gyroSmooth`) is intentionally **not** ported
    /// here — it is an optional, stateful, off-hot-path filter (blueprint §14: "OneEuro mouse
    /// contract reserved"); the gyro→stick (`MouseJoystick`) output mode is also out of this method's
    /// scope (it is a virtual-pad axis, not a relative-mouse delta).
    pub fn gyro_velocity_step(
        &mut self,
        gyro_x: f64,
        gyro_z: f64,
        elapsed_s: f64,
        cfg: &GyroAccumCfg,
    ) -> (i32, i32) {
        // Non-finite guards keep the pure core panic-free on a bad sample.
        if !gyro_x.is_finite() || !gyro_z.is_finite() || !elapsed_s.is_finite() {
            return (0, 0);
        }
        let temp = elapsed_s * GYRO_MOUSE_SPEED;
        let coefficient = cfg.sensitivity * GYRO_MOUSE_COEFFICIENT;

        let mut delta_x = gyro_x;
        let mut delta_y = gyro_z;

        // Direction split (atan2 unit vector), matching `Math.Atan2(-deltaY, deltaX)`.
        let (norm_x, norm_y) = if delta_x == 0.0 && delta_y == 0.0 {
            (0.0, 0.0)
        } else {
            let angle = (-delta_y).atan2(delta_x);
            (angle.cos().abs(), angle.sin().abs())
        };
        let sign_x = signum_strict(delta_x);
        let sign_y = signum_strict(delta_y);

        // Rate-domain dead-zone: subtract sign·|norm·deadzone|, zeroing inside the dead-zone.
        let dz_x = (norm_x * cfg.deadzone).abs();
        let dz_y = (norm_y * cfg.deadzone).abs();
        if delta_x.abs() > dz_x {
            delta_x -= sign_x * dz_x;
        } else {
            delta_x = 0.0;
        }
        if delta_y.abs() > dz_y {
            delta_y -= sign_y * dz_y;
        } else {
            delta_y = 0.0;
        }

        // Velocity model + per-axis anti-jitter offset.
        let offset = cfg.velocity_offset;
        let mut motion_x = if delta_x != 0.0 {
            coefficient * (delta_x * temp) + norm_x * (offset * sign_x)
        } else {
            0.0
        };
        let mut motion_y = if delta_y != 0.0 {
            (coefficient * cfg.vertical_scale) * (delta_y * temp) + norm_y * (offset * sign_y)
        } else {
            0.0
        };

        // Jitter compensation: ease the near-zero region with a `^1.408` curve (per-axis, scaled by
        // the direction-split norm so a diagonal eases proportionally).
        if cfg.jitter_comp {
            motion_x = jitter_ease(motion_x, norm_x);
            motion_y = jitter_ease(motion_y, norm_y);
        }

        self.finalize_core(
            motion_x,
            motion_y,
            cfg.min_threshold,
            cfg.invert_x,
            cfg.invert_y,
        )
    }

    /// The base mouse velocity (`sensitivity · MOUSESPEEDFACTOR`).
    #[inline]
    fn velocity(cfg: &MouseAccumCfg) -> f64 {
        cfg.sensitivity * MOUSE_SPEED_FACTOR
    }

    /// Normalized signed deflection past the dead-zone (the C# `diff`), with optional acceleration.
    ///
    /// Returns a value in `[-1, 1]`: 0 inside the dead-zone, otherwise the deflection rescaled so the
    /// dead-zone edge maps to 0 and full deflection maps to ±1. The vertical scale (a per-axis C#
    /// branch difference) is applied later in [`stick_step`](Self::stick_step), not here.
    #[inline]
    fn axis_velocity(component: f64, cfg: &MouseAccumCfg) -> f64 {
        let mag = component.abs();
        if mag <= cfg.deadzone {
            return 0.0;
        }
        let span = 1.0 - cfg.deadzone;
        let mut diff = if span > 0.0 {
            (mag - cfg.deadzone) / span
        } else {
            0.0
        };
        diff = diff.clamp(0.0, 1.0);
        if cfg.accelerate {
            diff = diff.powf(cfg.accel_power);
        }
        component.signum() * diff
    }

    /// Per-axis anti-jitter offset, split by the motion direction's unit vector (`atan2`).
    #[inline]
    fn offset_split(raw_x: f64, raw_y: f64, cfg: &MouseAccumCfg, dt_ms: f64) -> (f64, f64) {
        let offset = cfg.velocity_offset * Self::velocity(cfg) * dt_ms;
        if raw_x == 0.0 && raw_y == 0.0 {
            return (0.0, 0.0);
        }
        // C#: tempAngle = atan2(-deltaY, deltaX); normX = |cos|, normY = |sin|.
        let angle = (-raw_y).atan2(raw_x);
        let norm_x = angle.cos().abs();
        let norm_y = angle.sin().abs();
        (norm_x * offset, norm_y * offset)
    }

    /// The shared `calculateFinalMouseMovement` carry + `min_threshold` gate + invert.
    #[inline]
    fn finalize(&mut self, motion_x: f64, motion_y: f64, cfg: &MouseAccumCfg) -> (i32, i32) {
        self.finalize_core(
            motion_x,
            motion_y,
            cfg.min_threshold,
            cfg.invert_x,
            cfg.invert_y,
        )
    }

    /// The shared sub-pixel carry + `min_threshold` gate + per-axis invert, parameterized so both
    /// the stick path ([`MouseAccumCfg`]) and the gyro path ([`GyroAccumCfg`]) feed it the same
    /// remainder-carry semantics (ports `MouseCursor.calculateFinalMouseMovement` /
    /// `sixaxisMoved` lines 220-256). `min_threshold == 1.0` is the always-carry special case.
    #[inline]
    fn finalize_core(
        &mut self,
        motion_x: f64,
        motion_y: f64,
        min_threshold: f64,
        invert_x: bool,
        invert_y: bool,
    ) -> (i32, i32) {
        let mut mx = motion_x;
        let mut my = motion_y;

        // Add the carried remainder back only when its sign matches the new motion; else reset it.
        if (mx > 0.0 && self.h_remainder > 0.0) || (mx < 0.0 && self.h_remainder < 0.0) {
            mx += self.h_remainder;
        } else {
            self.h_remainder = 0.0;
        }
        if (my > 0.0 && self.v_remainder > 0.0) || (my < 0.0 && self.v_remainder < 0.0) {
            my += self.v_remainder;
        } else {
            self.v_remainder = 0.0;
        }

        // Truncate to two decimals toward zero (the C# remainderCutoff sub-pixel step).
        let cut_x = mx - remainder_cutoff(mx * 100.0, 1.0) / 100.0;
        let cut_y = my - remainder_cutoff(my * 100.0, 1.0) / 100.0;

        let dist_sq = cut_x * cut_x + cut_y * cut_y;
        let mut action_x = cut_x.trunc() as i32;
        let mut action_y = cut_y.trunc() as i32;

        if (min_threshold - 1.0).abs() < f64::EPSILON {
            // min_threshold == 1.0: always carry the fractional remainder (no gate).
            self.h_remainder = cut_x - f64::from(action_x);
            self.v_remainder = cut_y - f64::from(action_y);
        } else if dist_sq >= min_threshold * min_threshold {
            self.h_remainder = cut_x - f64::from(action_x);
            self.v_remainder = cut_y - f64::from(action_y);
        } else {
            // Below the gate: defer the whole motion, emit nothing this report.
            self.h_remainder = cut_x;
            self.v_remainder = cut_y;
            action_x = 0;
            action_y = 0;
        }

        if invert_x {
            action_x = -action_x;
        }
        if invert_y {
            action_y = -action_y;
        }
        (action_x, action_y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A config with sensitivity/offset chosen so each report contributes ≈0.4 px, so three
    /// reports accumulate to one pixel — the canonical 0.4×3 → 0,0,1 (rem 0.2) carry test.
    fn cfg_carry() -> MouseAccumCfg {
        MouseAccumCfg {
            sensitivity: 1.0,
            vertical_scale: 1.0,
            velocity_offset: 0.0,
            deadzone: 0.0,
            min_threshold: 1.0, // always carry
            accelerate: false,
            accel_power: 1.0,
            invert_x: false,
            invert_y: false,
        }
    }

    #[test]
    fn remainder_cutoff_exact_values_incl_negative() {
        // 0.4 px expressed as x*100 = 40.0: cutoff(40,1) = 40 - 1*40 = 0 -> truncates to 0.40.
        assert_eq!(remainder_cutoff(40.0, 1.0), 0.0);
        // 1.236 -> *100 = 123.6 -> cutoff = 123.6 - 123 = 0.6 -> x - 0.6/100 = 1.236 - 0.006 = 1.23.
        let x = 1.236;
        let cut = x - remainder_cutoff(x * 100.0, 1.0) / 100.0;
        assert!((cut - 1.23).abs() < 1e-9, "cut={cut}");
        // Negative truncates toward zero: -1.236 -> -1.23.
        let xn = -1.236;
        let cutn = xn - remainder_cutoff(xn * 100.0, 1.0) / 100.0;
        assert!((cutn + 1.23).abs() < 1e-9, "cutn={cutn}");
    }

    #[test]
    fn carry_accumulates_point_four_times_three() {
        // Feed three reports each producing motion_x = 0.4. Expect dx = 0, 0, 1 with rem ≈ 0.2.
        let cfg = cfg_carry();
        let mut acc = MouseAccumulator::new();

        // Drive raw motion directly via gyro_step (raw_dx = 0.4, no offset): cleanest control.
        let nudge = MouseAccumCfg {
            velocity_offset: 0.0,
            ..cfg
        };
        let (dx1, _) = acc.gyro_step(0.4, 0.0, 0.0, &nudge);
        assert_eq!(dx1, 0, "first 0.4 -> 0 px");
        let (dx2, _) = acc.gyro_step(0.4, 0.0, 0.0, &nudge);
        assert_eq!(dx2, 0, "second 0.4 (0.8 total) -> 0 px");
        let (dx3, _) = acc.gyro_step(0.4, 0.0, 0.0, &nudge);
        assert_eq!(dx3, 1, "third 0.4 (1.2 total) -> 1 px");
        let (h, _) = acc.remainder();
        assert!(
            (h - 0.2).abs() < 1e-9,
            "remainder after carry ≈ 0.2, got {h}"
        );
    }

    #[test]
    fn min_threshold_defers_then_emits() {
        // With a gate of 3.0, a motion of 2.0 px is below the 3px gate (dist²=4 < 9) -> deferred,
        // emitting 0 but keeping the full 2.0 as remainder. The next 2.0 accumulates to 4.0 (dist²=16
        // >= 9) -> emits 4 px.
        let cfg = MouseAccumCfg {
            velocity_offset: 0.0,
            min_threshold: 3.0,
            ..cfg_carry()
        };
        let mut acc = MouseAccumulator::new();
        let (dx1, _) = acc.gyro_step(2.0, 0.0, 0.0, &cfg);
        assert_eq!(dx1, 0, "2px below 3px gate -> deferred");
        let (h, _) = acc.remainder();
        assert!(
            (h - 2.0).abs() < 1e-9,
            "full motion held as remainder, got {h}"
        );
        let (dx2, _) = acc.gyro_step(2.0, 0.0, 0.0, &cfg);
        assert_eq!(dx2, 4, "accumulated 4px now past the gate -> emits");
    }

    #[test]
    fn sign_flip_resets_remainder() {
        // Build up a positive remainder, then push negative: the positive remainder must NOT be
        // added back (it is reset), so a sign flip starts the carry fresh.
        let cfg = MouseAccumCfg {
            velocity_offset: 0.0,
            min_threshold: 1.0,
            ..cfg_carry()
        };
        let mut acc = MouseAccumulator::new();
        let _ = acc.gyro_step(0.6, 0.0, 0.0, &cfg); // dx 0, rem 0.6
        let (h0, _) = acc.remainder();
        assert!((h0 - 0.6).abs() < 1e-9);
        // Now a negative motion of -0.3: sign differs from +0.6 remainder, so remainder resets,
        // motion stays -0.3 -> dx 0, new remainder -0.3 (not -0.3+0.6).
        let (dxn, _) = acc.gyro_step(-0.3, 0.0, 0.0, &cfg);
        assert_eq!(dxn, 0);
        let (hn, _) = acc.remainder();
        assert!(
            (hn + 0.3).abs() < 1e-9,
            "sign flip reset the carry, got {hn}"
        );
    }

    #[test]
    fn deadzone_suppresses_sub_threshold_deflection() {
        let cfg = MouseAccumCfg {
            sensitivity: 100.0,
            velocity_offset: 0.0,
            deadzone: 0.5,
            min_threshold: 1.0,
            ..cfg_carry()
        };
        let mut acc = MouseAccumulator::new();
        // Deflection 0.4 < deadzone 0.5 -> zero velocity -> zero output, zero remainder.
        let (dx, dy) = acc.stick_step(0.4, 0.0, 0.01, &cfg);
        assert_eq!((dx, dy), (0, 0));
        assert_eq!(acc.remainder(), (0.0, 0.0));
        // Deflection 0.9 > deadzone -> produces motion.
        let (dx2, _) = acc.stick_step(0.9, 0.0, 0.01, &cfg);
        assert!(
            dx2 > 0,
            "past-deadzone deflection moves the mouse, got {dx2}"
        );
    }

    #[test]
    fn invert_flags_negate_output() {
        let base = MouseAccumCfg {
            sensitivity: 100.0,
            velocity_offset: 0.0,
            min_threshold: 1.0,
            ..cfg_carry()
        };
        let mut a = MouseAccumulator::new();
        let (dx, dy) = a.stick_step(1.0, 1.0, 0.01, &base);
        assert!(
            dx > 0 && dy < 0,
            "stick up-right -> mouse right & up(−y): ({dx},{dy})"
        );

        let inv = MouseAccumCfg {
            invert_x: true,
            invert_y: true,
            ..base
        };
        let mut b = MouseAccumulator::new();
        let (idx, idy) = b.stick_step(1.0, 1.0, 0.01, &inv);
        assert_eq!((idx, idy), (-dx, -dy), "invert negates both axes");
    }

    #[test]
    fn reset_clears_remainder() {
        let cfg = cfg_carry();
        let mut acc = MouseAccumulator::new();
        let _ = acc.gyro_step(0.7, 0.7, 0.0, &cfg);
        assert_ne!(acc.remainder(), (0.0, 0.0));
        acc.reset();
        assert_eq!(acc.remainder(), (0.0, 0.0));
        assert_eq!(acc, MouseAccumulator::default());
    }

    #[test]
    fn stick_up_moves_mouse_up_on_screen() {
        // Stick +y (up) must produce a negative dy (up on screen). Pin the Y-axis sign convention.
        let cfg = MouseAccumCfg {
            sensitivity: 100.0,
            velocity_offset: 0.0,
            min_threshold: 1.0,
            ..cfg_carry()
        };
        let mut acc = MouseAccumulator::new();
        let (_, dy) = acc.stick_step(0.0, 1.0, 0.01, &cfg);
        assert!(dy < 0, "push up -> screen up (negative dy), got {dy}");
    }

    #[test]
    fn no_deflection_no_motion_no_remainder() {
        let cfg = cfg_carry();
        let mut acc = MouseAccumulator::new();
        let (dx, dy) = acc.stick_step(0.0, 0.0, 0.01, &cfg);
        assert_eq!((dx, dy), (0, 0));
        assert_eq!(acc.remainder(), (0.0, 0.0));
    }

    #[test]
    fn accumulator_is_copy_and_default_clean() {
        fn assert_copy<T: Copy>() {}
        assert_copy::<MouseAccumulator>();
        let a = MouseAccumulator::default();
        assert_eq!(a.remainder(), (0.0, 0.0));
    }

    // ------------------------------- M5 gyro→mouse velocity model --------------------------------

    /// A gyro config with no dead-zone/offset/jitter so the velocity model is exercised cleanly.
    fn gyro_clean() -> GyroAccumCfg {
        GyroAccumCfg {
            sensitivity: 1.0,
            vertical_scale: 1.0,
            velocity_offset: 0.0,
            deadzone: 0.0,
            min_threshold: 1.0,
            jitter_comp: false,
            invert_x: false,
            invert_y: false,
        }
    }

    #[test]
    fn gyro_default_cfg_matches_constants() {
        let d = GyroAccumCfg::default();
        assert_eq!(d.deadzone, GYRO_MOUSE_DEADZONE_DEFAULT);
        assert_eq!(d.velocity_offset, GYRO_MOUSE_OFFSET_DEFAULT);
        assert_eq!(d.min_threshold, 1.0);
        assert!(d.jitter_comp);
    }

    #[test]
    fn gyro_velocity_sign_right_and_down() {
        // +gyro_x -> cursor right (dx > 0); +gyro_z -> cursor down (dy > 0). High rate so the
        // motion exceeds one pixel in a single report.
        let cfg = GyroAccumCfg {
            sensitivity: 50.0,
            ..gyro_clean()
        };
        let mut acc = MouseAccumulator::new();
        let (dx, dy) = acc.gyro_velocity_step(40.0, 40.0, 0.005, &cfg);
        assert!(dx > 0, "right turn -> dx>0, got {dx}");
        assert!(dy > 0, "down turn -> dy>0, got {dy}");
        // Negative rates flip both signs.
        let mut acc2 = MouseAccumulator::new();
        let (nx, ny) = acc2.gyro_velocity_step(-40.0, -40.0, 0.005, &cfg);
        assert!(
            nx < 0 && ny < 0,
            "negative rate -> negative delta: ({nx},{ny})"
        );
    }

    #[test]
    fn gyro_velocity_remainder_carry() {
        // Choose sensitivity/rate so one report contributes ~0.4 px on X: coefficient =
        // sensitivity*GYRO_MOUSE_COEFFICIENT; motion = coefficient*(rate*elapsed*GYRO_MOUSE_SPEED).
        // With elapsed=0.005 -> temp = 1.0; motion = coefficient*rate. Pick coefficient*rate = 0.4.
        let sens = 0.4 / GYRO_MOUSE_COEFFICIENT; // so coefficient*1.0 = 0.4 at rate 1.0
        let cfg = GyroAccumCfg {
            sensitivity: sens,
            ..gyro_clean()
        };
        let mut acc = MouseAccumulator::new();
        // rate 1.0, elapsed 0.005 -> temp 1.0 -> motion_x = 0.4. Three reports -> 0,0,1 (rem 0.2).
        let (a, _) = acc.gyro_velocity_step(1.0, 0.0, 0.005, &cfg);
        let (b, _) = acc.gyro_velocity_step(1.0, 0.0, 0.005, &cfg);
        let (c, _) = acc.gyro_velocity_step(1.0, 0.0, 0.005, &cfg);
        assert_eq!((a, b, c), (0, 0, 1), "gyro 0.4×3 -> 0,0,1 carry");
        let (h, _) = acc.remainder();
        assert!((h - 0.2).abs() < 1e-9, "carry remainder ≈ 0.2, got {h}");
    }

    #[test]
    fn gyro_rate_domain_deadzone_suppresses_small_rates() {
        // Deadzone is in the gyro-rate domain (default 10). A rate of 5 (< 10) on a pure-X push is
        // fully suppressed (normX==1 so deadzoneX==10); a large rate passes (minus the deadzone).
        let cfg = GyroAccumCfg {
            sensitivity: 50.0,
            deadzone: 10.0,
            ..gyro_clean()
        };
        let mut acc = MouseAccumulator::new();
        let (dx, dy) = acc.gyro_velocity_step(5.0, 0.0, 0.005, &cfg);
        assert_eq!((dx, dy), (0, 0), "sub-deadzone rate -> no motion");
        assert_eq!(acc.remainder(), (0.0, 0.0));
        // A rate well past the deadzone moves.
        let mut acc2 = MouseAccumulator::new();
        let (dx2, _) = acc2.gyro_velocity_step(50.0, 0.0, 0.005, &cfg);
        assert!(dx2 > 0, "past-deadzone rate moves, got {dx2}");
    }

    #[test]
    fn gyro_invert_flags_negate_output() {
        let base = GyroAccumCfg {
            sensitivity: 50.0,
            ..gyro_clean()
        };
        let mut a = MouseAccumulator::new();
        let (dx, dy) = a.gyro_velocity_step(40.0, 40.0, 0.005, &base);
        assert!(dx > 0 && dy > 0);

        let inv = GyroAccumCfg {
            invert_x: true,
            invert_y: true,
            ..base
        };
        let mut b = MouseAccumulator::new();
        let (idx, idy) = b.gyro_velocity_step(40.0, 40.0, 0.005, &inv);
        assert_eq!((idx, idy), (-dx, -dy), "gyro invert negates both axes");
    }

    #[test]
    fn gyro_vertical_scale_scales_y_only() {
        // A pure-Y push with vertical_scale 2.0 produces about twice the dy of vertical_scale 1.0,
        // while X is untouched. Use a rate that yields several pixels so the ratio is observable.
        let base = GyroAccumCfg {
            sensitivity: 50.0,
            vertical_scale: 1.0,
            ..gyro_clean()
        };
        let scaled = GyroAccumCfg {
            vertical_scale: 2.0,
            ..base
        };
        let mut a = MouseAccumulator::new();
        let (_, dy1) = a.gyro_velocity_step(0.0, 30.0, 0.005, &base);
        let mut b = MouseAccumulator::new();
        let (_, dy2) = b.gyro_velocity_step(0.0, 30.0, 0.005, &scaled);
        assert!(
            dy1 > 0 && dy2 > dy1,
            "vertical_scale 2× moves Y more: {dy1} -> {dy2}"
        );
    }

    #[test]
    fn gyro_sensitivity_scales_speed() {
        let lo = GyroAccumCfg {
            sensitivity: 20.0,
            ..gyro_clean()
        };
        let hi = GyroAccumCfg {
            sensitivity: 80.0,
            ..gyro_clean()
        };
        let mut a = MouseAccumulator::new();
        let (dxl, _) = a.gyro_velocity_step(30.0, 0.0, 0.005, &lo);
        let mut b = MouseAccumulator::new();
        let (dxh, _) = b.gyro_velocity_step(30.0, 0.0, 0.005, &hi);
        assert!(
            dxh > dxl,
            "higher sensitivity -> faster cursor: {dxl} -> {dxh}"
        );
    }

    #[test]
    fn gyro_zero_rate_no_motion() {
        let cfg = GyroAccumCfg::default();
        let mut acc = MouseAccumulator::new();
        let (dx, dy) = acc.gyro_velocity_step(0.0, 0.0, 0.005, &cfg);
        assert_eq!((dx, dy), (0, 0));
        assert_eq!(acc.remainder(), (0.0, 0.0));
    }

    #[test]
    fn gyro_jitter_comp_eases_small_motion() {
        // With jitter comp on, a tiny motion (below norm·0.26) is eased smaller than the linear
        // value; with it off, the same input is the linear value. Use a pure-X rate small enough
        // that the velocity motion lands in the jitter region.
        let small_rate = 1.0;
        let elapsed = 0.005;
        // motion_x linear = coefficient * rate * temp = sens*GYRO_MOUSE_COEFFICIENT * 1.0 * 1.0.
        // Pick sens so linear motion ≈ 0.2 (< threshold 0.26, normX==1 for pure X).
        let sens = 0.2 / GYRO_MOUSE_COEFFICIENT;
        let off = GyroAccumCfg {
            sensitivity: sens,
            jitter_comp: false,
            ..gyro_clean()
        };
        let on = GyroAccumCfg {
            jitter_comp: true,
            ..off
        };
        // Drive several reports and compare accumulated remainder magnitude as a proxy for the
        // pre-cutoff motion (no full pixels emitted at this scale).
        let mut a = MouseAccumulator::new();
        let _ = a.gyro_velocity_step(small_rate, 0.0, elapsed, &off);
        let (ho, _) = a.remainder();
        let mut b = MouseAccumulator::new();
        let _ = b.gyro_velocity_step(small_rate, 0.0, elapsed, &on);
        let (hj, _) = b.remainder();
        assert!(ho > 0.0 && hj > 0.0, "both carry a positive remainder");
        assert!(
            hj < ho,
            "jitter comp eases the small motion smaller: linear {ho} vs eased {hj}"
        );
    }

    #[test]
    fn gyro_velocity_step_handles_non_finite() {
        let cfg = GyroAccumCfg::default();
        let mut acc = MouseAccumulator::new();
        assert_eq!(acc.gyro_velocity_step(f64::NAN, 0.0, 0.005, &cfg), (0, 0));
        assert_eq!(
            acc.gyro_velocity_step(1.0, f64::INFINITY, 0.005, &cfg),
            (0, 0)
        );
        assert_eq!(acc.gyro_velocity_step(1.0, 1.0, f64::NAN, &cfg), (0, 0));
        // State untouched on a bad sample.
        assert_eq!(acc.remainder(), (0.0, 0.0));
    }

    #[test]
    fn signum_strict_is_zero_at_zero() {
        assert_eq!(signum_strict(0.0), 0.0);
        assert_eq!(signum_strict(-0.0), 0.0);
        assert_eq!(signum_strict(3.0), 1.0);
        assert_eq!(signum_strict(-3.0), -1.0);
    }
}
