//! `process_stick` — the full ordered DS4Windows-class stick chain.
//!
//! The whole chain after the RC stage runs in the DS4 `[0,255]` f64 domain (128 neutral),
//! entered/exited **once** via [`axis_to_ds4`]/[`ds4_to_axis`] so the C# goldens port 1:1 with
//! zero mid-chain quantization. Stage order (blueprint §4):
//!
//! ```text
//! stage 0  RC filter        (canonical [-1,1] domain, BEFORE entering [0,255])
//! --- enter [0,255] ONCE via axis_to_ds4 ---
//! stage 1  rotation
//! stage 2  anti-snapback
//! stage 3  fuzz
//! stage 4  calibration      (identity hook; omitted v1)
//! stage 5  deadzone         (radial OR axial)
//! stage 6  sensitivity      (RADIAL-ONLY — C# quirk)
//! stage 7  square stick
//! stage 8  output curve
//! --- exit [0,255] ONCE via ds4_to_axis ---
//! stage 9  flick stick      (terminal; M3 stashes st.flick_delta, returns the abs stick unchanged)
//! ```
//!
//! Pure, alloc-free, OS-free. The per-stick mutable state lives in [`StickState`]; the RC sub-step
//! reuses the existing bit-exact [`RcFilter`](crate::rc::RcFilter) so its goldens stay byte-identical.

use crate::convert::{axis_to_ds4, ds4_to_axis};
use crate::dt::Dt;
use crate::rc::RcFilter;
use crate::stick::settings::{DeadZoneType, StickSettings, StickState};
use crate::stick::stages;
use crate::stick::{StickAlgorithm, StickSample};

/// Collapse a non-finite axis value to neutral (0.0); pass finite values through unchanged.
///
/// The pure-core safety net for the stick egress (M7): NaN/Inf must never reach the virtual pad.
/// Mirrors the mouse accumulator's non-finite guards so the whole core has one "non-finite →
/// neutral" rule. `#[inline]` + branch-free-friendly; a finite chain is a no-op (goldens unchanged).
#[inline]
fn finite_or_zero(v: f64) -> f64 {
    if v.is_finite() {
        v
    } else {
        0.0
    }
}

/// Process one stick report through the full DS4Windows-class chain.
///
/// * `raw` — the device's stick sample in the canonical `[-1,1]` unit.
/// * `cfg` — the per-stick settings (already `clamped()` by the config funnel).
/// * `st` — the resident per-stick state (RC, fuzz/anti-snapback history, flick).
/// * `dt` — the guarded per-report elapsed time (drives RC dt-compensation + anti-snapback timing).
///
/// Returns the processed stick in the canonical `[-1,1]` unit.
pub fn process_stick(
    raw: StickSample,
    cfg: &StickSettings,
    st: &mut StickState,
    dt: Dt,
) -> StickSample {
    // Advance the monotonic accumulator once per report (anti-snapback timing; replaces wall clock).
    st.elapsed_us = st.elapsed_us.wrapping_add(dt.us() as i64);

    // --- stage 0: RC filter (canonical [-1,1] domain) ---
    let rc_active = cfg.rc_mode_on && cfg.rc.enabled;
    let after_rc = if rc_active {
        let filter = RcFilter;
        if !st.rc_primed {
            filter.prime(&cfg.rc, &mut st.rc, raw);
            st.rc_primed = true;
            // First report after enable takes no RC step (the prime contract).
            raw
        } else {
            filter.process(&cfg.rc, &mut st.rc, dt, raw)
        }
    } else {
        // RC not selected: keep the state clean so a later enable re-primes (mirrors ResetIfActive).
        if st.rc_primed {
            st.rc_primed = false;
            st.rc = crate::rc::RcStickState::default();
        }
        raw
    };

    // --- enter [0,255] ONCE ---
    let mut x = axis_to_ds4(after_rc.x);
    let mut y = axis_to_ds4(after_rc.y);

    // --- stage 1: rotation ---
    if cfg.rotation.angle_rad != 0.0 {
        let (nx, ny) = stages::rotate(x, y, cfg.rotation.angle_rad);
        x = nx;
        y = ny;
    }

    // --- stage 2: anti-snapback ---
    if cfg.anti_snapback.enabled {
        let (nx, ny) =
            stages::anti_snapback(&mut st.snap_hist, &cfg.anti_snapback, x, y, st.elapsed_us);
        x = nx;
        y = ny;
    }

    // --- stage 3: fuzz ---
    if cfg.dead_zone.fuzz > 0 {
        let (nx, ny) = stages::fuzz(
            &mut st.fuzz_last,
            &mut st.fuzz_primed,
            cfg.dead_zone.fuzz,
            x,
            y,
        );
        x = nx;
        y = ny;
    }

    // --- stage 4: calibration (identity hook; omitted v1) ---

    // --- stage 5: deadzone (radial OR axial) ---
    let (nx, ny) = stages::dead_zone(x, y, &cfg.dead_zone);
    x = nx;
    y = ny;

    // --- stage 6: sensitivity (RADIAL-ONLY; axial silently ignores it) ---
    if cfg.dead_zone.dead_zone_type == DeadZoneType::Radial {
        let (nx, ny) = stages::sensitivity(x, y, cfg.sensitivity);
        x = nx;
        y = ny;
    }

    // --- stage 7: square stick ---
    if cfg.square.enabled {
        let (nx, ny) = stages::square_stick(x, y, cfg.square.roundness);
        x = nx;
        y = ny;
    }

    // --- stage 8: output curve ---
    let (nx, ny) = stages::output_curve(
        x,
        y,
        cfg.curve,
        cfg.dead_zone.dead_zone_type,
        // Bezier control points: M3 uses the identity (0,1/3,2/3,1) so a default Bezier
        // profile passes through; the editable control points land with the GUI editor (M4+).
        (1.0 / 3.0, 2.0 / 3.0),
    );
    x = nx;
    y = ny;

    // --- exit [0,255] ONCE ---
    // NaN/Inf guard (M7): an aberrant input or an atan2/sqrt/pow singularity anywhere in the chain
    // (rotation, square-stick, output curve) could in principle surface a non-finite component. The
    // virtual pad must NEVER see NaN/Inf, so clamp any non-finite axis to neutral (0.0) — the same
    // "non-finite collapses to neutral" contract the mouse accumulator guards apply. A finite chain
    // (every default/golden path) is untouched, so M1-M6 goldens stay byte-identical.
    let out = StickSample {
        x: finite_or_zero(ds4_to_axis(x)),
        y: finite_or_zero(ds4_to_axis(y)),
    };

    // --- stage 9: flick stick (terminal; M3 stashes the delta, returns abs stick unchanged) ---
    if cfg.flick.enabled {
        flick_stash(out, cfg, st);
    } else {
        // Keep the stashed delta clean when flick is off.
        st.flick_delta = 0.0;
    }

    out
}

/// Stage 9: compute the per-report relative turn and stash it in `st.flick_delta` for the mouse
/// path, WITHOUT folding it into the returned absolute stick. Flick is **single-sweep** in v1 — the
/// bookkeeping (`flick_in_progress` rising-edge anchor + `flick_last_angle`) yields the per-report
/// angular delta directly; there is no remaining-angle accumulator (the dead `flick_angle_remaining`
/// field was removed in M7). A whole-flick remaining-angle model is a future additive change.
#[inline]
fn flick_stash(out: StickSample, _cfg: &StickSettings, st: &mut StickState) {
    let magnitude = (out.x * out.x + out.y * out.y).sqrt();
    // Flick-stick engages above a small radius; below it the stick is "centred".
    const FLICK_DEADZONE: f64 = 0.9;
    if magnitude >= FLICK_DEADZONE {
        let angle = out.x.atan2(out.y); // 0 == forward (+y up)
        if !st.flick_in_progress {
            // Rising edge: a fresh flick begins; M5 owns the actual sweep. M3 records the anchor.
            st.flick_in_progress = true;
            st.flick_last_angle = angle;
            st.flick_delta = 0.0;
        } else {
            // Continued aim: the per-report angular delta is the relative turn.
            let mut delta = angle - st.flick_last_angle;
            // Wrap to [-PI, PI].
            if delta > core::f64::consts::PI {
                delta -= 2.0 * core::f64::consts::PI;
            } else if delta < -core::f64::consts::PI {
                delta += 2.0 * core::f64::consts::PI;
            }
            st.flick_delta = delta;
            st.flick_last_angle = angle;
        }
    } else {
        st.flick_in_progress = false;
        st.flick_delta = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::convert::{axis_to_ds4, ds4_to_axis};
    use crate::rc::{RcConfig, RcMode, RcStickState};

    fn ds4_sample(x_ds4: f64, y_ds4: f64) -> StickSample {
        StickSample {
            x: ds4_to_axis(x_ds4),
            y: ds4_to_axis(y_ds4),
        }
    }

    fn out_ds4(s: StickSample) -> (f64, f64) {
        (axis_to_ds4(s.x), axis_to_ds4(s.y))
    }

    #[test]
    fn default_settings_pass_through() {
        let cfg = StickSettings::default();
        let mut st = StickState::default();
        let dt = Dt::guarded(4000.0);
        // No stages active -> output equals input within fp epsilon.
        for &(dx, dy) in &[(128.0, 128.0), (255.0, 128.0), (40.0, 200.0), (0.0, 255.0)] {
            let s = ds4_sample(dx, dy);
            let out = process_stick(s, &cfg, &mut st, dt);
            let (ox, oy) = out_ds4(out);
            assert!((ox - dx).abs() < 1e-9, "x in={dx} out={ox}");
            assert!((oy - dy).abs() < 1e-9, "y in={dy} out={oy}");
        }
    }

    #[test]
    fn only_rc_on_matches_rc_filter_directly() {
        // process_stick with ONLY RC on, all downstream stages default, must be byte-identical
        // to driving RcFilter directly.
        let rc = RcConfig {
            enabled: true,
            mode: RcMode::FireBirdInteger,
            use_dynamic_curve: false,
            period_us: 4000,
            fixed_param: 100,
            ..RcConfig::default()
        };
        let cfg = StickSettings {
            rc,
            rc_mode_on: true,
            ..StickSettings::default()
        };

        let dt = Dt::guarded(4000.0);
        let filter = RcFilter;

        // Direct RcFilter reference path (prime then process).
        let mut ref_state = RcStickState::default();
        // process_stick path.
        let mut st = StickState::default();

        let inputs = [128.0, 255.0, 200.0, 160.0, 128.0, 90.0, 255.0, 128.0];

        // Prime the reference filter exactly as process_stick does on its first call.
        let mut first = true;
        for &inp in &inputs {
            let s = ds4_sample(inp, 128.0);
            let ref_out = if first {
                filter.prime(&rc, &mut ref_state, s);
                first = false;
                s
            } else {
                filter.process(&rc, &mut ref_state, dt, s)
            };
            let proc_out = process_stick(s, &cfg, &mut st, dt);
            let (rx, ry) = out_ds4(ref_out);
            let (px, py) = out_ds4(proc_out);
            assert!((rx - px).abs() < 1e-12, "inp={inp} ref={rx} proc={px}");
            assert!((ry - py).abs() < 1e-12, "inp={inp} ref={ry} proc={py}");
        }
    }

    #[test]
    fn sensitivity_radial_only_through_pipeline() {
        // sens 2.0 @ 160 -> 192 in [0,255] (radial path).
        let cfg = StickSettings {
            sensitivity: 2.0,
            ..StickSettings::default()
        };
        let mut st = StickState::default();
        let dt = Dt::guarded(4000.0);
        let out = process_stick(ds4_sample(160.0, 128.0), &cfg, &mut st, dt);
        let (ox, _) = out_ds4(out);
        assert!((ox - 192.0).abs() < 1e-9, "ox={ox}");
    }

    #[test]
    fn sensitivity_ignored_for_axial() {
        // Axial deadzone model silently ignores sensitivity (C# quirk).
        let cfg = StickSettings {
            sensitivity: 2.0,
            dead_zone: crate::stick::settings::StickDeadZone {
                dead_zone_type: DeadZoneType::Axial,
                ..crate::stick::settings::StickDeadZone::default()
            },
            ..StickSettings::default()
        };
        let mut st = StickState::default();
        let dt = Dt::guarded(4000.0);
        let out = process_stick(ds4_sample(160.0, 128.0), &cfg, &mut st, dt);
        let (ox, _) = out_ds4(out);
        // axial dz off + sens ignored -> passthrough 160.
        assert!((ox - 160.0).abs() < 1e-9, "ox={ox}");
    }

    #[test]
    fn flick_stashes_delta_but_returns_abs_unchanged() {
        let cfg = StickSettings {
            flick: crate::stick::settings::FlickStick {
                enabled: true,
                ..crate::stick::settings::FlickStick::default()
            },
            ..StickSettings::default()
        };
        let mut st = StickState::default();
        let dt = Dt::guarded(4000.0);
        // Full right (255) -> magnitude 1.0, above flick deadzone.
        let s = ds4_sample(255.0, 128.0);
        let out = process_stick(s, &cfg, &mut st, dt);
        let (ox, _) = out_ds4(out);
        // Absolute stick unchanged (no fold into x/y).
        assert!((ox - 255.0).abs() < 1e-9, "ox={ox}");
        assert!(st.flick_in_progress);
    }

    #[test]
    fn output_is_finite_for_nan_and_extreme_inputs() {
        // M7 NaN guard: aberrant inputs (non-finite or extreme magnitudes) must never surface a
        // NaN/Inf to the virtual pad — the egress clamps non-finite axes to neutral (0.0). Exercise
        // both a pass-through chain and a fully-loaded chain (rotation/square/curve/sensitivity) that
        // stresses the atan2/sqrt/pow stages where a singularity could arise.
        let loaded = StickSettings {
            sensitivity: 3.0,
            rotation: crate::stick::settings::RotationSettings { angle_rad: 0.5 },
            square: crate::stick::settings::SquareStick {
                enabled: true,
                roundness: 5.0,
            },
            curve: crate::stick::settings::OutputCurve::Cubic,
            dead_zone: crate::stick::settings::StickDeadZone {
                dead_zone: 10,
                anti_dead_zone: 15,
                ..crate::stick::settings::StickDeadZone::default()
            },
            ..StickSettings::default()
        };
        let dt = Dt::guarded(4000.0);
        for cfg in [&StickSettings::default(), &loaded] {
            for &(rx, ry) in &[
                (f64::NAN, 0.0),
                (0.0, f64::NAN),
                (f64::INFINITY, f64::NEG_INFINITY),
                (f64::NAN, f64::NAN),
                (1e308, -1e308),
                (-1e300, 1e300),
            ] {
                let mut st = StickState::default();
                let out = process_stick(StickSample { x: rx, y: ry }, cfg, &mut st, dt);
                assert!(
                    out.x.is_finite() && out.y.is_finite(),
                    "non-finite input ({rx},{ry}) must yield a finite output, got ({},{})",
                    out.x,
                    out.y
                );
            }
        }
    }

    #[test]
    fn finite_or_zero_collapses_non_finite() {
        assert_eq!(super::finite_or_zero(0.5), 0.5);
        assert_eq!(super::finite_or_zero(-1.0), -1.0);
        assert_eq!(super::finite_or_zero(f64::NAN), 0.0);
        assert_eq!(super::finite_or_zero(f64::INFINITY), 0.0);
        assert_eq!(super::finite_or_zero(f64::NEG_INFINITY), 0.0);
    }

    #[test]
    fn zero_alloc_over_many_calls() {
        // Smoke determinism: many calls do not panic and stay finite (alloc-freedom is asserted by
        // the crate-level counting-allocator test; here we ensure the hot path is branch-stable).
        let cfg = StickSettings {
            sensitivity: 1.3,
            dead_zone: crate::stick::settings::StickDeadZone {
                dead_zone: 10,
                anti_dead_zone: 15,
                ..crate::stick::settings::StickDeadZone::default()
            },
            anti_snapback: crate::stick::settings::AntiSnapback {
                enabled: true,
                ..crate::stick::settings::AntiSnapback::default()
            },
            ..StickSettings::default()
        };
        let mut st = StickState::default();
        let dt = Dt::guarded(4000.0);
        for i in 0..10_000u32 {
            let v = 128.0 + 100.0 * ((i as f64) * 0.01).sin();
            let out = process_stick(ds4_sample(v, 128.0), &cfg, &mut st, dt);
            assert!(out.x.is_finite() && out.y.is_finite());
        }
    }
}
