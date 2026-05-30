//! Pure per-stage stick helpers in the DS4 `[0,255]` f64 domain (128 neutral).
//!
//! Each function is a 1:1 port of the corresponding C# `Mapping.SetCurveAndDeadzone` step
//! (`rotateLSCoordinates`, `CalcAntiSnapbackStick`, `CalcStickAxisFuzz`,
//! `ApplyRadialStickDeadZone`, `ApplyAxialStickDeadZone`, `ApplyStickSensitivity`,
//! `ApplySquareStick`/`CircleToSquare`, `ApplyStickOutputCurve`). They take and return the
//! `[0,255]` domain so the C# goldens port with zero algebraic re-derivation and zero mid-chain
//! quantization (no `(byte)` casts — `f64` end-to-end, clamped to `[0,255]` exactly like the C#
//! `ClampAxisValue`).
//!
//! All functions are pure, alloc-free, and have zero OS dependency.

use super::settings::{
    AntiSnapback, AxisDeadZone, DeadZoneType, OutputCurve, SnapbackRing, StickDeadZone,
};
use core::f64::consts::PI;

/// Clamp a value into the DS4 `[0,255]` axis domain (mirrors C# `DS4State.ClampAxisValue`).
#[inline]
pub fn clamp_ds4(v: f64) -> f64 {
    v.clamp(0.0, 255.0)
}

/// Stage 1 — rotation. Ports `DS4State.rotateLSCoordinates`: rotate the `(x-128, y-128)` vector
/// by `angle_rad`, clamp each component to `[-128, 127]`, re-add 128, then clamp to `[0,255]`.
#[inline]
pub fn rotate(x: f64, y: f64, angle_rad: f64) -> (f64, f64) {
    if angle_rad == 0.0 {
        return (x, y);
    }
    let sin = angle_rad.sin();
    let cos = angle_rad.cos();
    let tx = x - 128.0;
    let ty = y - 128.0;
    let nx = (tx * cos - ty * sin).clamp(-128.0, 127.0) + 128.0;
    let ny = (tx * sin + ty * cos).clamp(-128.0, 127.0) + 128.0;
    (clamp_ds4(nx), clamp_ds4(ny))
}

/// Stage 2 — anti-snapback. Ports `CalcAntiSnapbackStick`: if any buffered sample within the
/// timeout window is `>= delta` away **and** the segment to it passes within a 15-unit circle
/// around centre (128, 128), snap the output to neutral; otherwise pass the input through. The
/// current sample is always appended.
///
/// `now_us`/`timeout_ms` replace the C# `DateTimeOffset.Now` + `timeout` with the deterministic
/// `elapsed_us` accumulator (verifier divergence 3). History is the fixed-cap [`SnapbackRing`].
#[inline]
pub fn anti_snapback(
    hist: &mut SnapbackRing,
    cfg: &AntiSnapback,
    x: f64,
    y: f64,
    now_us: i64,
) -> (f64, f64) {
    // Prune samples older than the timeout window (timeout in ms -> us).
    let cutoff = now_us - cfg.timeout_ms.saturating_mul(1000);
    hist.prune_older_than(cutoff);

    let delta_sq = cfg.delta * cfg.delta;
    let mut snap = false;
    hist.for_each(|ox, oy, _| {
        if snap {
            return;
        }
        let dxs = x - ox;
        let dys = y - oy;
        let distance_squared = dxs * dxs + dys * dys;
        if distance_squared >= delta_sq {
            // Closest approach of the segment (current -> old) to centre (128, 128).
            let t = ((128.0 - x) * (ox - x) + (128.0 - y) * (oy - y)) / distance_squared;
            let t = t.clamp(0.0, 1.0);
            let mid_x = 128.0 - (x + t * (ox - x));
            let mid_y = 128.0 - (y + t * (oy - y));
            let distance_to_middle_squared = mid_x * mid_x + mid_y * mid_y;
            if distance_to_middle_squared <= 15.0 * 15.0 {
                snap = true;
            }
        }
    });

    hist.push(x, y, now_us);

    if snap {
        (128.0, 128.0)
    } else {
        (x, y)
    }
}

/// Stage 3 — input fuzz. Ports `CalcStickAxisFuzz`: hold each axis at its last emitted value
/// until the squared motion of the (x, y) pair exceeds `delta^2`, with an endpoint passthrough
/// (`0` or `255` always updates). `last`/`primed` mirror the C# per-stick `lastStickAxisValues`.
#[inline]
pub fn fuzz(last: &mut [f64; 2], primed: &mut bool, delta: i32, x: f64, y: f64) -> (f64, f64) {
    if !*primed {
        // Seed history so the first report passes through (C# starts lastStickAxisValues at 0,
        // which for a real first sample would always trip the endpoint/motion gate; seeding to
        // the input avoids spurious motion on enable and matches the prime contract).
        *last = [x, y];
        *primed = true;
        return (x, y);
    }

    let delta_x = x - last[0];
    let delta_y = y - last[1];
    let mag_squ = delta_x * delta_x + delta_y * delta_y;
    let delta_squ = (delta as f64) * (delta as f64);

    let mut use_x = last[0];
    let mut use_y = last[1];

    if x == 0.0 || x == 255.0 || mag_squ > delta_squ {
        use_x = x;
        last[0] = x;
    }
    if y == 0.0 || y == 255.0 || mag_squ > delta_squ {
        use_y = y;
        last[1] = y;
    }

    (use_x, use_y)
}

/// Stage 5 (radial) — `ApplyRadialStickDeadZone`. Fused radial anti-dz + max-zone + max-output
/// + vertical-scale, ported verbatim from the `[0,255]` C# form.
pub fn radial_dead_zone(x: f64, y: f64, info: &StickDeadZone) -> (f64, f64) {
    let deadzone = info.dead_zone;
    let anti_dead = info.anti_dead_zone;
    let max_zone = info.max_zone;
    let max_output = info.max_output;
    let vertical_scale_setting = info.vertical_scale;
    let interpret = anti_dead > 0
        || max_zone != 100
        || max_output != 100.0
        || info.max_output_force
        || vertical_scale_setting != super::settings::DEFAULT_VERTICAL_SCALE;

    if deadzone <= 0 && !interpret {
        return (x, y);
    }

    let squared = (x - 128.0).powi(2) + (y - 128.0).powi(2);
    let deadzone_squared = (deadzone as f64).powi(2);
    if deadzone > 0 && squared <= deadzone_squared {
        return (128.0, 128.0);
    }

    if (deadzone <= 0 || squared <= deadzone_squared) && !interpret {
        return (x, y);
    }

    let r = (-(y - 128.0)).atan2(x - 128.0);
    let max_x_value = if x >= 128.0 { 127.0 } else { -128.0 };
    let max_y_value = if y >= 128.0 { 127.0 } else { -128.0 };
    let ratio = max_zone as f64 / 100.0;
    let max_out_ratio = max_output / 100.0;
    let vertical_scale = vertical_scale_setting / 100.0;

    let max_zone_x_neg_value = ratio * -128.0 + 128.0;
    let max_zone_x_pos_value = ratio * 127.0 + 128.0;
    let max_zone_y_neg_value = max_zone_x_neg_value;
    let max_zone_y_pos_value = max_zone_x_pos_value;
    let max_zone_x = if x >= 128.0 {
        max_zone_x_pos_value - 128.0
    } else {
        max_zone_x_neg_value - 128.0
    };
    let max_zone_y = if y >= 128.0 {
        max_zone_y_pos_value - 128.0
    } else {
        max_zone_y_neg_value - 128.0
    };

    let mut temp_output_x;
    let mut temp_output_y;
    if deadzone > 0 {
        let temp_x_dead = r.cos().abs() * (deadzone as f64 / 127.0) * max_x_value;
        let temp_y_dead = r.sin().abs() * (deadzone as f64 / 127.0) * max_y_value;
        if squared > deadzone_squared {
            let current_x = x.clamp(max_zone_x_neg_value, max_zone_x_pos_value);
            let current_y = y.clamp(max_zone_y_neg_value, max_zone_y_pos_value);
            temp_output_x = (current_x - 128.0 - temp_x_dead) / (max_zone_x - temp_x_dead);
            temp_output_y = (current_y - 128.0 - temp_y_dead) / (max_zone_y - temp_y_dead);
        } else {
            temp_output_x = 0.0;
            temp_output_y = 0.0;
        }
    } else {
        let current_x = x.clamp(max_zone_x_neg_value, max_zone_x_pos_value);
        let current_y = y.clamp(max_zone_y_neg_value, max_zone_y_pos_value);
        temp_output_x = (current_x - 128.0) / max_zone_x;
        temp_output_y = (current_y - 128.0) / max_zone_y;
    }

    if vertical_scale_setting != super::settings::DEFAULT_VERTICAL_SCALE {
        temp_output_y = (temp_output_y * vertical_scale).clamp(0.0, 1.0);
    }

    if max_output != 100.0 || info.max_output_force {
        let max_out_x_ratio = (r.cos().abs() * max_out_ratio / 0.99).min(1.0);
        let max_out_y_ratio = (r.sin().abs() * max_out_ratio / 0.99).min(1.0);
        temp_output_x = temp_output_x.clamp(0.0, max_out_x_ratio);
        temp_output_y = temp_output_y.clamp(0.0, max_out_y_ratio);
    }

    let mut temp_x_anti_dead_percent = 0.0;
    let mut temp_y_anti_dead_percent = 0.0;
    if anti_dead > 0 {
        temp_x_anti_dead_percent = (anti_dead as f64 * 0.01) * r.cos().abs();
        temp_y_anti_dead_percent = (anti_dead as f64 * 0.01) * r.sin().abs();
    }

    let out_x = if temp_output_x > 0.0 {
        ((1.0 - temp_x_anti_dead_percent) * temp_output_x + temp_x_anti_dead_percent) * max_x_value
            + 128.0
    } else {
        128.0
    };
    let out_y = if temp_output_y > 0.0 {
        ((1.0 - temp_y_anti_dead_percent) * temp_output_y + temp_y_anti_dead_percent) * max_y_value
            + 128.0
    } else {
        128.0
    };
    (clamp_ds4(out_x), clamp_ds4(out_y))
}

/// Stage 5 (axial) — `ApplyAxialStickDeadZone`: independent per-axis dead/anti/max.
pub fn axial_dead_zone(x: f64, y: f64, info: &StickDeadZone) -> (f64, f64) {
    (
        axial_dead_zone_axis(x, &info.x_axis),
        axial_dead_zone_axis(y, &info.y_axis),
    )
}

/// `ApplyAxialStickDeadZoneAxis` — one axis of the axial deadzone.
fn axial_dead_zone_axis(axis: f64, info: &AxisDeadZone) -> f64 {
    if info.dead_zone <= 0
        && info.anti_dead_zone <= 0
        && info.max_zone == 100
        && info.max_output == 100.0
    {
        return axis;
    }

    let dist_val = (axis - 128.0).abs();
    if info.dead_zone > 0 && dist_val <= info.dead_zone as f64 {
        return 128.0;
    }

    let max_axis_value = if axis >= 128.0 { 127.0 } else { -128.0 };
    let ratio = info.max_zone as f64 / 100.0;
    let mut max_out_ratio = info.max_output / 100.0;

    let max_zone_neg_value = ratio * -128.0 + 128.0;
    let max_zone_pos_value = ratio * 127.0 + 128.0;
    let max_zone = if axis >= 128.0 {
        max_zone_pos_value - 128.0
    } else {
        max_zone_neg_value - 128.0
    };

    let temp_dead = if info.dead_zone > 0 {
        (info.dead_zone as f64 / 127.0) * max_axis_value
    } else {
        0.0
    };
    let current_val = axis.clamp(max_zone_neg_value, max_zone_pos_value);
    let mut temp_output = (current_val - 128.0 - temp_dead) / (max_zone - temp_dead);

    if info.max_output != 100.0 {
        max_out_ratio = (max_out_ratio / 0.99).min(1.0);
        temp_output = temp_output.clamp(0.0, max_out_ratio);
    }

    let temp_anti_dead_percent = if info.anti_dead_zone > 0 {
        info.anti_dead_zone as f64 * 0.01
    } else {
        0.0
    };
    let out = if temp_output > 0.0 {
        ((1.0 - temp_anti_dead_percent) * temp_output + temp_anti_dead_percent) * max_axis_value
            + 128.0
    } else {
        128.0
    };
    clamp_ds4(out)
}

/// Stage 5 dispatch on [`DeadZoneType`].
#[inline]
pub fn dead_zone(x: f64, y: f64, info: &StickDeadZone) -> (f64, f64) {
    match info.dead_zone_type {
        DeadZoneType::Radial => radial_dead_zone(x, y, info),
        DeadZoneType::Axial => axial_dead_zone(x, y, info),
    }
}

/// Stage 6 — sensitivity (`ApplyStickSensitivity`). RADIAL-ONLY: the caller must skip this stage
/// for the axial deadzone model (C# quirk preserved). Scales the `(axis-128)` vector by `sens`.
#[inline]
pub fn sensitivity(x: f64, y: f64, sens: f64) -> (f64, f64) {
    if sens == 1.0 {
        return (x, y);
    }
    (
        (sens * (x - 128.0) + 128.0).clamp(0.0, 255.0),
        (sens * (y - 128.0) + 128.0).clamp(0.0, 255.0),
    )
}

/// Stage 7 — square stick (`ApplySquareStick` + `CircleToSquare`) with roundness.
pub fn square_stick(x: f64, y: f64, roundness: f64) -> (f64, f64) {
    if x == 128.0 && y == 128.0 {
        return (x, y);
    }
    let cap_x = if x >= 128.0 { 127.0 } else { 128.0 };
    let cap_y = if y >= 128.0 { 127.0 } else { 128.0 };
    let temp_x = (x - 128.0) / cap_x;
    let temp_y = (y - 128.0) / cap_y;
    let (cur_x, cur_y) = circle_to_square(temp_x, temp_y, roundness);
    let temp_x = cur_x.clamp(-1.0, 1.0);
    let temp_y = cur_y.clamp(-1.0, 1.0);
    (temp_x * cap_x + 128.0, temp_y * cap_y + 128.0)
}

/// `DS4SquareStick.CircleToSquare` ported verbatim (operates on a unit `[-1,1]` vector).
fn circle_to_square(cur_x: f64, cur_y: f64, roundness: f64) -> (f64, f64) {
    const PI_OVER_FOUR: f64 = PI / 4.0;
    let mut angle = cur_y.atan2(-cur_x);
    angle += PI;
    let cos_ang = angle.cos();

    let (squared_x, squared_y) = if angle <= PI_OVER_FOUR || angle > 7.0 * PI_OVER_FOUR {
        // X+ wall
        let temp_val = 1.0 / cos_ang;
        (cur_x * temp_val, cur_y * temp_val)
    } else if angle > PI_OVER_FOUR && angle <= 3.0 * PI_OVER_FOUR {
        // Y+ wall
        let temp_val = 1.0 / angle.sin();
        (cur_x * temp_val, cur_y * temp_val)
    } else if angle > 3.0 * PI_OVER_FOUR && angle <= 5.0 * PI_OVER_FOUR {
        // X- wall
        let temp_val = -1.0 / cos_ang;
        (cur_x * temp_val, cur_y * temp_val)
    } else if angle > 5.0 * PI_OVER_FOUR && angle <= 7.0 * PI_OVER_FOUR {
        // Y- wall
        let temp_val = -1.0 / angle.sin();
        (cur_x * temp_val, cur_y * temp_val)
    } else {
        return (cur_x, cur_y);
    };

    let length = cur_x / cos_ang;
    let factor = length.powf(roundness);
    let out_x = cur_x + (squared_x - cur_x) * factor;
    let out_y = cur_y + (squared_y - cur_y) * factor;
    (out_x, out_y)
}

/// Apex Classic inverse — axial signed sqrt (`StickOutCurve.ApplyApexClassicInverseAxisCurve`).
#[inline]
pub fn apex_axis_curve(value: f64) -> f64 {
    let sign = if value < 0.0 { -1.0 } else { 1.0 };
    value.abs().sqrt() * sign
}

/// Apex Classic inverse — radial direction-preserving
/// (`StickOutCurve.ApplyApexClassicInverseRadialCurve`).
#[inline]
fn apex_radial_curve(x_ratio: f64, y_ratio: f64) -> (f64, f64) {
    let travel_ratio = x_ratio.abs().max(y_ratio.abs());
    if travel_ratio <= 0.0 {
        return (0.0, 0.0);
    }
    let scale = travel_ratio.sqrt() / travel_ratio;
    (x_ratio * scale, y_ratio * scale)
}

/// `ApplyEnhancedPrecisionCurve` — the DS4Windows enhanced-precision piecewise curve.
#[inline]
fn enhanced_precision_curve(value: f64) -> f64 {
    let abs = value.abs();
    if abs <= 0.4 {
        0.8 * abs
    } else if abs <= 0.75 {
        abs - 0.08
    } else {
        abs * 1.32 - 0.32
    }
}

/// Cubic-Bezier easing ratio for a single `[-1,1]` ratio, mirroring the C# custom Bezier LUT
/// shape. The four control values are `(0, p1, p2, 1)`; the cap-aware caller folds in `capX/capY`.
///
/// M3 carries this so the `Bezier` curve variant is non-panicking and shape-correct; the exact
/// LUT/`real_world` calibration is HW-tuned later. For unit `(0,0,1,1)` control points it reduces
/// to the identity, so a default Bezier profile is a pass-through.
#[inline]
fn bezier_curve(ratio: f64, p1: f64, p2: f64) -> f64 {
    let sign = if ratio >= 0.0 { 1.0 } else { -1.0 };
    let t = ratio.abs().clamp(0.0, 1.0);
    let inv = 1.0 - t;
    // Cubic Bezier value with endpoints 0 and 1: 3(1-t)^2 t p1 + 3(1-t) t^2 p2 + t^3.
    let val = 3.0 * inv * inv * t * p1 + 3.0 * inv * t * t * p2 + t * t * t;
    sign * val
}

/// Stage 8 — output curve (`ApplyStickOutputCurve`, cap-aware form). Operates in the `[0,255]`
/// domain; the radial-vs-axial ratio derivation matches the C# `deadzoneType` branch. `bezier`
/// carries the two interior Bezier control points (used only for [`OutputCurve::Bezier`]).
pub fn output_curve(
    x: f64,
    y: f64,
    curve: OutputCurve,
    dead_zone_type: DeadZoneType,
    bezier: (f64, f64),
) -> (f64, f64) {
    if curve == OutputCurve::Linear || (x == 128.0 && y == 128.0) {
        return (x, y);
    }

    let (temp_ratio_x, temp_ratio_y, cap_x, cap_y);
    match dead_zone_type {
        DeadZoneType::Radial => {
            let r = (-(y - 128.0)).atan2(x - 128.0);
            let max_out_x_ratio = r.cos().abs();
            let max_out_y_ratio = r.sin().abs();
            let side_x = x - 128.0;
            let side_y = y - 128.0;
            let mut cx = if x >= 128.0 {
                max_out_x_ratio * 127.0
            } else {
                max_out_x_ratio * 128.0
            };
            let mut cy = if y >= 128.0 {
                max_out_y_ratio * 127.0
            } else {
                max_out_y_ratio * 128.0
            };
            let abs_side_x = side_x.abs();
            let abs_side_y = side_y.abs();
            if abs_side_x > cx {
                cx = abs_side_x;
            }
            if abs_side_y > cy {
                cy = abs_side_y;
            }
            cap_x = cx;
            cap_y = cy;
            temp_ratio_x = if cx > 0.0 { (x - 128.0) / cx } else { 0.0 };
            temp_ratio_y = if cy > 0.0 { (y - 128.0) / cy } else { 0.0 };
        }
        DeadZoneType::Axial => {
            cap_x = if x >= 128.0 { 127.0 } else { 128.0 };
            cap_y = if y >= 128.0 { 127.0 } else { 128.0 };
            temp_ratio_x = (x - 128.0) / cap_x;
            temp_ratio_y = (y - 128.0) / cap_y;
        }
    }

    let sign_x = if temp_ratio_x >= 0.0 { 1.0 } else { -1.0 };
    let sign_y = if temp_ratio_y >= 0.0 { 1.0 } else { -1.0 };

    let (out_x, out_y) = match curve {
        // Already handled above, but the exhaustive match keeps clippy happy.
        OutputCurve::Linear => (x, y),
        OutputCurve::EnhancedPrecision => (
            enhanced_precision_curve(temp_ratio_x) * sign_x * cap_x + 128.0,
            enhanced_precision_curve(temp_ratio_y) * sign_y * cap_y + 128.0,
        ),
        OutputCurve::Quadratic => (
            temp_ratio_x * temp_ratio_x * sign_x * cap_x + 128.0,
            temp_ratio_y * temp_ratio_y * sign_y * cap_y + 128.0,
        ),
        OutputCurve::Cubic => (
            temp_ratio_x * temp_ratio_x * temp_ratio_x * cap_x + 128.0,
            temp_ratio_y * temp_ratio_y * temp_ratio_y * cap_y + 128.0,
        ),
        OutputCurve::EaseoutQuad => {
            let output_x = temp_ratio_x.abs() * (temp_ratio_x.abs() - 2.0);
            let output_y = temp_ratio_y.abs() * (temp_ratio_y.abs() - 2.0);
            (
                -(output_x * sign_x * cap_x) + 128.0,
                -(output_y * sign_y * cap_y) + 128.0,
            )
        }
        OutputCurve::EaseoutCubic => {
            let inner_x = temp_ratio_x.abs() - 1.0;
            let inner_y = temp_ratio_y.abs() - 1.0;
            (
                (inner_x * inner_x * inner_x + 1.0) * sign_x * cap_x + 128.0,
                (inner_y * inner_y * inner_y + 1.0) * sign_y * cap_y + 128.0,
            )
        }
        OutputCurve::Bezier => (
            bezier_curve(temp_ratio_x, bezier.0, bezier.1) * cap_x + 128.0,
            bezier_curve(temp_ratio_y, bezier.0, bezier.1) * cap_y + 128.0,
        ),
        OutputCurve::ApexClassicInverse => {
            let (cx, cy) = apex_radial_curve(temp_ratio_x, temp_ratio_y);
            (cx * cap_x + 128.0, cy * cap_y + 128.0)
        }
        OutputCurve::ApexClassicInverseAxial => {
            let axis_cap_x = if x >= 128.0 { 127.0 } else { 128.0 };
            let axis_cap_y = if y >= 128.0 { 127.0 } else { 128.0 };
            let axis_ratio_x = (x - 128.0) / axis_cap_x;
            let axis_ratio_y = (y - 128.0) / axis_cap_y;
            (
                apex_axis_curve(axis_ratio_x) * axis_cap_x + 128.0,
                apex_axis_curve(axis_ratio_y) * axis_cap_y + 128.0,
            )
        }
    };

    (clamp_ds4(out_x), clamp_ds4(out_y))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::stick::settings::StickDeadZone;

    const EPS: f64 = 1e-9;

    fn approx(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-6
    }

    #[test]
    fn rotation_identity_when_zero() {
        assert_eq!(rotate(200.0, 100.0, 0.0), (200.0, 100.0));
    }

    #[test]
    fn rotation_90_degrees() {
        // +X (full right, 255 ~ +127) rotates to... rotateLSCoordinates uses screen coords.
        // tx=127, ty=0; angle=pi/2: nx = -0 -> 128, ny = 127 -> 255.
        let (nx, ny) = rotate(255.0, 128.0, PI / 2.0);
        assert!(approx(nx, 128.0), "nx={nx}");
        assert!(approx(ny, 255.0), "ny={ny}");
    }

    #[test]
    fn sensitivity_radial_2x_at_160_is_192() {
        // sens=2.0 @ 160 -> 2*(160-128)+128 = 192.
        let (nx, _) = sensitivity(160.0, 128.0, 2.0);
        assert!(approx(nx, 192.0), "nx={nx}");
    }

    #[test]
    fn sensitivity_identity_at_one() {
        assert_eq!(sensitivity(160.0, 200.0, 1.0), (160.0, 200.0));
    }

    #[test]
    fn enhanced_precision_golden() {
        // Enhanced 0.4 -> 0.32, 0.75 -> 0.67.
        assert!(approx(enhanced_precision_curve(0.4), 0.32));
        assert!(approx(enhanced_precision_curve(0.75), 0.67));
    }

    #[test]
    fn enhanced_precision_curve_axis_radial() {
        // Pure +X at 0.4 ratio: x=128+0.4*127=178.8, y=128. cap_x = 127 (cos r =1).
        // tempRatioX = (178.8-128)/127 = 0.4. Enhanced(0.4)=0.32 -> 0.32*127+128 = 168.64.
        let x = 128.0 + 0.4 * 127.0;
        let (nx, ny) = output_curve(
            x,
            128.0,
            OutputCurve::EnhancedPrecision,
            DeadZoneType::Radial,
            (0.0, 1.0),
        );
        assert!(approx(nx, 0.32 * 127.0 + 128.0), "nx={nx}");
        assert!(approx(ny, 128.0), "ny={ny}");
    }

    #[test]
    fn quadratic_golden_axial() {
        // Quadratic 0.5 -> 0.25 (axial, pure X). x = 128 + 0.5*127 = 191.5; cap=127.
        // ratio=0.5; out = 0.5^2 * 127 + 128 = 0.25*127+128.
        let x = 128.0 + 0.5 * 127.0;
        let (nx, _) = output_curve(
            x,
            128.0,
            OutputCurve::Quadratic,
            DeadZoneType::Axial,
            (0.0, 1.0),
        );
        assert!(approx(nx, 0.25 * 127.0 + 128.0), "nx={nx}");
    }

    #[test]
    fn cubic_golden_axial() {
        // Cubic 0.5 -> 0.125.
        let x = 128.0 + 0.5 * 127.0;
        let (nx, _) = output_curve(
            x,
            128.0,
            OutputCurve::Cubic,
            DeadZoneType::Axial,
            (0.0, 1.0),
        );
        assert!(approx(nx, 0.125 * 127.0 + 128.0), "nx={nx}");
    }

    #[test]
    fn easeout_quad_golden_axial() {
        // EaseoutQuad 0.5 -> 0.75. output = abs(0.5)*(0.5-2) = -0.75; *-1 = 0.75.
        let x = 128.0 + 0.5 * 127.0;
        let (nx, _) = output_curve(
            x,
            128.0,
            OutputCurve::EaseoutQuad,
            DeadZoneType::Axial,
            (0.0, 1.0),
        );
        assert!(approx(nx, 0.75 * 127.0 + 128.0), "nx={nx}");
    }

    #[test]
    fn apex_axis_golden() {
        // ApexAxis 0.25 -> 0.5 (sqrt(0.25)=0.5).
        assert!(approx(apex_axis_curve(0.25), 0.5));
        let x = 128.0 + 0.25 * 127.0;
        let (nx, _) = output_curve(
            x,
            128.0,
            OutputCurve::ApexClassicInverseAxial,
            DeadZoneType::Axial,
            (0.0, 1.0),
        );
        assert!(approx(nx, 0.5 * 127.0 + 128.0), "nx={nx}");
    }

    #[test]
    fn linear_curve_is_passthrough() {
        let (nx, ny) = output_curve(
            200.0,
            90.0,
            OutputCurve::Linear,
            DeadZoneType::Radial,
            (0.0, 1.0),
        );
        assert_eq!((nx, ny), (200.0, 90.0));
    }

    #[test]
    fn bezier_unit_control_points_is_identity_ratio() {
        // With control points (0,1) the cubic reduces to t (identity); axial pure X passes through.
        let x = 128.0 + 0.5 * 127.0;
        let (nx, _) = output_curve(
            x,
            128.0,
            OutputCurve::Bezier,
            DeadZoneType::Axial,
            (1.0 / 3.0, 2.0 / 3.0),
        );
        // (0,1/3,2/3,1) bezier is the identity line -> ratio 0.5 -> 0.5.
        assert!(approx(nx, 0.5 * 127.0 + 128.0), "nx={nx}");
    }

    #[test]
    fn radial_deadzone_neutral_passthrough_when_off() {
        let info = StickDeadZone::default();
        assert_eq!(dead_zone(128.0, 128.0, &info), (128.0, 128.0));
        // Off (dz=0, no interpret) -> passthrough.
        assert_eq!(dead_zone(200.0, 100.0, &info), (200.0, 100.0));
    }

    #[test]
    fn radial_deadzone_inside_snaps_to_neutral() {
        let info = StickDeadZone {
            dead_zone: 20,
            ..StickDeadZone::default()
        };
        // 10 units off-centre, dz=20 -> inside -> neutral.
        let (nx, ny) = dead_zone(138.0, 128.0, &info);
        assert_eq!((nx, ny), (128.0, 128.0));
    }

    #[test]
    fn radial_deadzone_endpoint_reaches_full() {
        let info = StickDeadZone {
            dead_zone: 10,
            ..StickDeadZone::default()
        };
        // Full right (255): with only a deadzone, the endpoint should map to ~255.
        let (nx, ny) = dead_zone(255.0, 128.0, &info);
        assert!(approx(nx, 255.0), "nx={nx}");
        assert!(approx(ny, 128.0), "ny={ny}");
    }

    #[test]
    fn axial_deadzone_per_axis() {
        let mut info = StickDeadZone {
            dead_zone_type: DeadZoneType::Axial,
            ..StickDeadZone::default()
        };
        info.x_axis.dead_zone = 30;
        // x within 30 of centre -> neutral; y untouched (its axis is off).
        let (nx, ny) = dead_zone(150.0, 200.0, &info);
        assert_eq!(nx, 128.0);
        assert_eq!(ny, 200.0);
    }

    #[test]
    fn fuzz_holds_then_releases() {
        let mut last = [0.0; 2];
        let mut primed = false;
        // prime
        let (x0, y0) = fuzz(&mut last, &mut primed, 5, 150.0, 150.0);
        assert_eq!((x0, y0), (150.0, 150.0));
        // small move within delta^2 (delta=5 -> 25): move by (3,0) -> mag 9 < 25 -> held.
        let (x1, y1) = fuzz(&mut last, &mut primed, 5, 153.0, 150.0);
        assert_eq!((x1, y1), (150.0, 150.0));
        // bigger move -> released.
        let (x2, _) = fuzz(&mut last, &mut primed, 5, 160.0, 150.0);
        assert_eq!(x2, 160.0);
    }

    #[test]
    fn fuzz_endpoint_passthrough() {
        let mut last = [128.0; 2];
        let mut primed = true;
        // endpoint 0 always passes even if motion is small.
        let (x, _) = fuzz(&mut last, &mut primed, 100, 0.0, 128.0);
        assert_eq!(x, 0.0);
    }

    #[test]
    fn anti_snapback_snaps_on_crossing() {
        let mut hist = SnapbackRing::default();
        let cfg = AntiSnapback {
            enabled: true,
            delta: 50.0,
            timeout_ms: 100,
        };
        // Push a far-right sample, then a return toward centre that crosses the middle.
        let _ = anti_snapback(&mut hist, &cfg, 240.0, 128.0, 0);
        // Move back across centre to the left: segment passes through middle -> snap to 128.
        let (nx, ny) = anti_snapback(&mut hist, &cfg, 20.0, 128.0, 1000);
        assert_eq!((nx, ny), (128.0, 128.0));
    }

    #[test]
    fn anti_snapback_passes_normal_motion() {
        let mut hist = SnapbackRing::default();
        let cfg = AntiSnapback {
            enabled: true,
            delta: 50.0,
            timeout_ms: 100,
        };
        let _ = anti_snapback(&mut hist, &cfg, 130.0, 128.0, 0);
        // small motion below delta -> passes through unchanged.
        let (nx, _) = anti_snapback(&mut hist, &cfg, 135.0, 128.0, 1000);
        assert_eq!(nx, 135.0);
    }

    #[test]
    fn square_stick_neutral_passthrough() {
        assert_eq!(square_stick(128.0, 128.0, 5.0), (128.0, 128.0));
    }

    #[test]
    fn square_stick_corner_pushes_out() {
        // A diagonal inside the circle should move outward toward the square corner.
        let (nx, ny) = square_stick(200.0, 200.0, 1.0);
        assert!(nx >= 200.0 - EPS || ny >= 200.0 - EPS);
    }
}
