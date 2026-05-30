//! The RC stick filter: config, per-stick state, and the [`RcFilter`] [`StickAlgorithm`].
//!
//! Three modes share the same period-derived coefficients ([`coeffs`]), dynamic-curve param
//! ([`curve`]), and priming/dispatch scaffolding, differing only in the per-axis recurrence:
//! [`RcMode::FireBirdInteger`] (bit-exact i32 Q4, [`firebird`]), [`RcMode::UltimateLegacy`]
//! (f64, rate-coupled, [`ultimate::step_legacy`]), and [`RcMode::UltimateDt`] (the corrected
//! report-rate-invariant form, [`ultimate::step_dt`]).
//!
//! The filter computes in the DS4-compatible `[0,255]` domain (so the C# goldens port 1:1):
//! [`StickAlgorithm`] entry/exit adapts `[-1,1] ↔ [0,255]` via [`crate::convert`]. X and Y share
//! one `param` per report but keep fully independent per-axis state.

pub mod coeffs;
pub mod curve;
pub mod firebird;
pub mod ultimate;

pub use coeffs::*;
pub use curve::*;

use crate::convert::{axis_to_ds4, ds4_to_axis};
use crate::dt::Dt;
use crate::stick::{StickAlgorithm, StickSample};
use firebird::FbAxisState;
use ultimate::UltAxisState;

/// Which RC recurrence a stick runs. Serialized in `PascalCase` to match the config schema.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum RcMode {
    /// Bit-exact i32 Q4 FireBird integer oracle (rate-coupled, frozen).
    FireBirdInteger,
    /// f64 Ultimate, rate-coupled (legacy feel, converges exactly to a held input).
    UltimateLegacy,
    /// f64 Ultimate, report-rate invariant (dt-compensated, corrected lead). The default.
    UltimateDt,
}

impl Default for RcMode {
    #[inline]
    fn default() -> Self {
        Self::UltimateDt
    }
}

/// Per-stick RC configuration (one of these per LS / RS).
///
/// Every field is `#[serde(default)]` so a partial TOML table fills in the C# `Reset()` defaults.
#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct RcConfig {
    /// Whether the filter is active. When `false`, [`RcFilter::process`] is a pass-through.
    #[serde(default)]
    pub enabled: bool,
    /// Which recurrence to run.
    #[serde(default)]
    pub mode: RcMode,
    /// When set, derive `param` from stick speed via [`curve`]; otherwise use `fixed_param`.
    #[serde(default)]
    pub use_dynamic_curve: bool,
    /// Filter period in microseconds; clamped to `[MIN_PERIOD_US, MAX_PERIOD_US]`.
    #[serde(default)]
    pub period_us: i32,
    /// Fixed `param` used when `use_dynamic_curve` is `false`; clamped to `[MIN_PARAM, MAX_PARAM]`.
    #[serde(default)]
    pub fixed_param: i32,
    /// The dynamic-curve breakpoints.
    #[serde(default)]
    pub curve: RcCurve,
}

impl Default for RcConfig {
    #[inline]
    fn default() -> Self {
        Self {
            enabled: false,
            mode: RcMode::UltimateDt,
            use_dynamic_curve: false,
            period_us: 4000,
            fixed_param: 100,
            curve: RcCurve::default(),
        }
    }
}

impl RcConfig {
    /// A copy with `period_us`/`fixed_param` clamped and `curve.x2 >= x1` enforced (C# F12).
    pub fn clamped(&self) -> RcConfig {
        let mut out = *self;
        out.period_us = out.period_us.clamp(MIN_PERIOD_US, MAX_PERIOD_US);
        out.fixed_param = clamp_param(out.fixed_param);
        if out.curve.x2 < out.curve.x1 {
            out.curve.x2 = out.curve.x1;
        }
        out
    }
}

/// All per-stick mutable filter state: both branch states per axis, the speed history, and the
/// cached period coefficients (recomputed only when `period_us` changes).
#[derive(Default)]
pub struct RcStickState {
    /// FireBird integer per-axis state (`[x, y]`).
    pub fb: [FbAxisState; 2],
    /// Ultimate (f64) per-axis state (`[x, y]`).
    pub ult: [UltAxisState; 2],
    /// Previous raw12 per axis, for the dynamic-curve speed delta.
    pub prev_raw12: [i32; 2],
    /// Cached period coefficients; `None` until first use / after a period change.
    pub coeffs: Option<PeriodCoeffs>,
}

impl RcStickState {
    /// Ensure `coeffs` matches `period_us`, recomputing only on change.
    #[inline]
    fn ensure_coeffs(&mut self, period_us: i32) -> PeriodCoeffs {
        let clamped = period_us.clamp(MIN_PERIOD_US, MAX_PERIOD_US);
        match self.coeffs {
            Some(c) if c.period_us == clamped => c,
            _ => {
                let c = PeriodCoeffs::new(clamped);
                self.coeffs = Some(c);
                c
            }
        }
    }
}

/// The RC stick filter — a zero-sized [`StickAlgorithm`] dispatching on [`RcMode`].
pub struct RcFilter;

impl StickAlgorithm for RcFilter {
    type Config = RcConfig;
    type State = RcStickState;

    fn prime(&self, cfg: &Self::Config, st: &mut Self::State, s: StickSample) {
        // Seed BOTH branch states (and the speed history) to the input in the ds4 domain so the
        // first real report takes no step regardless of which branch param selects. The lazy
        // per-branch priming inside step() is the safety net if param flips sign later.
        let dx = axis_to_ds4(s.x);
        let dy = axis_to_ds4(s.y);
        let ds = [dx, dy];
        let raw = [curve::to_raw12(dx), curve::to_raw12(dy)];

        for (((fb, ult), &r), &d) in st
            .fb
            .iter_mut()
            .zip(st.ult.iter_mut())
            .zip(raw.iter())
            .zip(ds.iter())
        {
            self_seed_fb(fb, r << 4);
            self_seed_ult(ult, d, cfg.mode);
        }
        st.prev_raw12 = raw;
        st.coeffs = Some(PeriodCoeffs::new(
            cfg.period_us.clamp(MIN_PERIOD_US, MAX_PERIOD_US),
        ));
    }

    fn process(
        &self,
        cfg: &Self::Config,
        st: &mut Self::State,
        dt: Dt,
        s: StickSample,
    ) -> StickSample {
        if !cfg.enabled {
            return s;
        }

        let c = st.ensure_coeffs(cfg.period_us);
        let period = c.period_us;

        let dx = axis_to_ds4(s.x);
        let dy = axis_to_ds4(s.y);
        let rx = curve::to_raw12(dx);
        let ry = curve::to_raw12(dy);

        // Compute the shared param. The fixed path STILL writes prev_raw12 (mirrors C# so a
        // fixed->dynamic switch without reset sees a correct delta).
        let param = if cfg.use_dynamic_curve {
            let delta = (rx - st.prev_raw12[0])
                .abs()
                .max((ry - st.prev_raw12[1]).abs());
            st.prev_raw12 = [rx, ry];
            let speed = match cfg.mode {
                RcMode::UltimateDt => curve::speed_dt(delta, dt.us(), period),
                _ => curve::speed_legacy(delta, period),
            };
            curve::param_from_speed(&cfg.curve, speed)
        } else {
            st.prev_raw12 = [rx, ry];
            curve::clamp_param(cfg.fixed_param)
        };

        let (out_x, out_y) = match cfg.mode {
            RcMode::FireBirdInteger => (
                firebird::step(&mut st.fb[0], rx, param, c.lead_base, c.base_value),
                firebird::step(&mut st.fb[1], ry, param, c.lead_base, c.base_value),
            ),
            RcMode::UltimateLegacy => (
                ultimate::step_legacy(&mut st.ult[0], dx, param, c.lead_base, c.base_value),
                ultimate::step_legacy(&mut st.ult[1], dy, param, c.lead_base, c.base_value),
            ),
            RcMode::UltimateDt => (
                ultimate::step_dt(
                    &mut st.ult[0],
                    dx,
                    param,
                    c.lead_base,
                    c.base_value,
                    dt.us(),
                    period,
                ),
                ultimate::step_dt(
                    &mut st.ult[1],
                    dy,
                    param,
                    c.lead_base,
                    c.base_value,
                    dt.us(),
                    period,
                ),
            ),
        };

        StickSample {
            x: ds4_to_axis(out_x),
            y: ds4_to_axis(out_y),
        }
    }
}

/// Seed a FireBird axis state to a primed-at-input condition (both branches).
#[inline]
fn self_seed_fb(st: &mut FbAxisState, input_q4: i32) {
    st.pos_primed = true;
    st.pos_q4 = input_q4;
    st.neg_primed = true;
    st.neg_q4 = input_q4;
    st.neg_prev_q4 = input_q4;
}

/// Seed an Ultimate axis state to a primed-at-input condition (both branches).
///
/// The legacy path keeps state in the `*256` Q-domain; the dt path keeps it in `[0,255]`.
#[inline]
fn self_seed_ult(st: &mut UltAxisState, input_ds4: f64, mode: RcMode) {
    let seed = match mode {
        RcMode::UltimateLegacy => input_ds4 * 256.0,
        _ => input_ds4,
    };
    st.pos_primed = true;
    st.pos = seed;
    st.neg_primed = true;
    st.neg = seed;
    st.neg_prev = seed;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ds4_sample(x_ds4: f64, y_ds4: f64) -> StickSample {
        StickSample {
            x: ds4_to_axis(x_ds4),
            y: ds4_to_axis(y_ds4),
        }
    }

    fn out_ds4(s: StickSample) -> (f64, f64) {
        (axis_to_ds4(s.x), axis_to_ds4(s.y))
    }

    fn cfg_fb(fixed_param: i32) -> RcConfig {
        RcConfig {
            enabled: true,
            mode: RcMode::FireBirdInteger,
            use_dynamic_curve: false,
            period_us: 4000,
            fixed_param,
            curve: RcCurve::default(),
        }
    }

    #[test]
    fn default_mode_is_ultimate_dt() {
        assert_eq!(RcMode::default(), RcMode::UltimateDt);
        assert_eq!(RcConfig::default().mode, RcMode::UltimateDt);
        assert!(!RcConfig::default().enabled);
        assert_eq!(RcConfig::default().period_us, 4000);
        assert_eq!(RcConfig::default().fixed_param, 100);
    }

    #[test]
    fn clamped_enforces_ranges_and_x2() {
        let cfg = RcConfig {
            period_us: 999,
            fixed_param: 9999,
            curve: RcCurve {
                x1: 64,
                x2: 10,
                ..RcCurve::default()
            },
            ..RcConfig::default()
        };
        let c = cfg.clamped();
        assert_eq!(c.period_us, MIN_PERIOD_US);
        assert_eq!(c.fixed_param, MAX_PARAM);
        assert_eq!(c.curve.x2, 64);
    }

    #[test]
    fn disabled_is_pass_through() {
        let f = RcFilter;
        let cfg = RcConfig {
            enabled: false,
            ..cfg_fb(100)
        };
        let mut st = RcStickState::default();
        let s = ds4_sample(0.0, 255.0);
        let out = f.process(&cfg, &mut st, Dt::guarded(4000.0), s);
        assert_eq!(out, s);
    }

    #[test]
    fn firebird_headline_through_filter() {
        // The 153.3984375 golden, driven through the full StickAlgorithm (ds4 domain in/out).
        let f = RcFilter;
        let cfg = cfg_fb(100);
        let mut st = RcStickState::default();
        let dt = Dt::guarded(4000.0);

        // prime at neutral (128) like the C# first Process call (param>0 first-use primes).
        let p0 = f.process(&cfg, &mut st, dt, ds4_sample(128.0, 128.0));
        let (px, _) = out_ds4(p0);
        assert!((px - 128.0).abs() < 1e-9);

        let p1 = f.process(&cfg, &mut st, dt, ds4_sample(255.0, 128.0));
        let (x1, y1) = out_ds4(p1);
        assert!((x1 - 153.3984375).abs() < 1e-4, "got {x1}");
        assert!((y1 - 128.0).abs() < 1e-4, "got {y1}");
    }

    #[test]
    fn firebird_negative_clamps_to_255() {
        let f = RcFilter;
        let cfg = cfg_fb(-100);
        let mut st = RcStickState::default();
        let dt = Dt::guarded(4000.0);
        let _ = f.process(&cfg, &mut st, dt, ds4_sample(128.0, 128.0));
        let out = f.process(&cfg, &mut st, dt, ds4_sample(160.0, 128.0));
        let (x, _) = out_ds4(out);
        assert!((x - 255.0).abs() < 1e-4, "got {x}");
    }

    #[test]
    fn prime_seeds_so_first_step_takes_no_motion() {
        // After explicit prime(), a process() at the SAME input returns ~input (no step).
        let f = RcFilter;
        let cfg = cfg_fb(100);
        let mut st = RcStickState::default();
        let s = ds4_sample(200.0, 128.0);
        f.prime(&cfg, &mut st, s);
        let out = f.process(&cfg, &mut st, Dt::guarded(4000.0), s);
        let (x, _) = out_ds4(out);
        assert!(
            (x - 200.0).abs() < 1e-4,
            "primed step should not move, got {x}"
        );
    }

    #[test]
    fn ultimate_dt_reduces_to_legacy_through_filter_at_dt_period() {
        // Drive UltimateDt and UltimateLegacy through the full filter; at dt==period they agree.
        let f = RcFilter;
        let mut cfg_dt = RcConfig {
            enabled: true,
            mode: RcMode::UltimateDt,
            period_us: 4000,
            fixed_param: -100,
            ..RcConfig::default()
        };
        let mut st_dt = RcStickState::default();
        let cfg_leg = RcConfig {
            mode: RcMode::UltimateLegacy,
            ..cfg_dt
        };
        let mut st_leg = RcStickState::default();
        let dt = Dt::guarded(4000.0);

        let inputs = [128.0, 160.0, 180.0, 150.0, 120.0, 200.0, 200.0, 130.0];
        for &inp in &inputs {
            let od = f.process(&cfg_dt, &mut st_dt, dt, ds4_sample(inp, 128.0));
            let ol = f.process(&cfg_leg, &mut st_leg, dt, ds4_sample(inp, 128.0));
            let (xd, _) = out_ds4(od);
            let (xl, _) = out_ds4(ol);
            assert!((xd - xl).abs() < 1e-9, "inp={inp} dt={xd} legacy={xl}");
        }
        // also exercise the positive branch.
        cfg_dt.fixed_param = 100;
        let cfg_leg2 = RcConfig {
            mode: RcMode::UltimateLegacy,
            ..cfg_dt
        };
        let mut sd = RcStickState::default();
        let mut sl = RcStickState::default();
        for &inp in &inputs {
            let od = f.process(&cfg_dt, &mut sd, dt, ds4_sample(inp, 128.0));
            let ol = f.process(&cfg_leg2, &mut sl, dt, ds4_sample(inp, 128.0));
            assert!(
                (axis_to_ds4(od.x) - axis_to_ds4(ol.x)).abs() < 1e-9,
                "inp={inp}"
            );
        }
    }

    #[test]
    fn period_change_recomputes_coeffs() {
        let f = RcFilter;
        let mut cfg = cfg_fb(100);
        let mut st = RcStickState::default();
        let dt = Dt::guarded(4000.0);
        let _ = f.process(&cfg, &mut st, dt, ds4_sample(128.0, 128.0));
        assert_eq!(st.coeffs.unwrap().period_us, 4000);
        cfg.period_us = 2000;
        let _ = f.process(&cfg, &mut st, dt, ds4_sample(128.0, 128.0));
        assert_eq!(st.coeffs.unwrap().period_us, 2000);
        assert_eq!(st.coeffs.unwrap().base_value, 50); // 100/2
    }

    #[test]
    fn fixed_mode_writes_prev_raw12() {
        // Mirror C#: fixed-param mode still updates prev_raw12 (for a later dynamic switch).
        let f = RcFilter;
        let cfg = cfg_fb(100);
        let mut st = RcStickState::default();
        let dt = Dt::guarded(4000.0);
        let _ = f.process(&cfg, &mut st, dt, ds4_sample(200.0, 220.0));
        assert_eq!(
            st.prev_raw12,
            [curve::to_raw12(200.0), curve::to_raw12(220.0)]
        );
    }

    #[test]
    fn x_and_y_keep_independent_state() {
        // Different per-axis inputs must produce different per-axis outputs.
        let f = RcFilter;
        let cfg = cfg_fb(100);
        let mut st = RcStickState::default();
        let dt = Dt::guarded(4000.0);
        let _ = f.process(&cfg, &mut st, dt, ds4_sample(128.0, 128.0));
        let out = f.process(&cfg, &mut st, dt, ds4_sample(255.0, 200.0));
        let (x, y) = out_ds4(out);
        assert!((x - 153.3984375).abs() < 1e-4, "x={x}");
        // y from 128 hold -> 200, param 100: 128*16<<4=32768 prime; step to 200.
        let ry = curve::to_raw12(200.0);
        let expected_q4 = 32768 + 25 * ((ry << 4) - 32768) / 125;
        assert!((y - expected_q4 as f64 / 256.0).abs() < 1e-4, "y={y}");
    }

    #[test]
    fn dynamic_curve_bypass_at_high_speed() {
        // DynamicCurveCanBypassAtConfiguredSpeed: high speed -> param 0 -> bypass (255 passes).
        let f = RcFilter;
        let cfg = RcConfig {
            enabled: true,
            mode: RcMode::FireBirdInteger,
            use_dynamic_curve: true,
            period_us: 4000,
            fixed_param: 0,
            curve: RcCurve {
                y0: 100,
                x1: 1,
                y1: 100,
                x2: 128,
                y2: 0,
                y3: 0,
            },
        };
        let mut st = RcStickState::default();
        let dt = Dt::guarded(4000.0);
        let _ = f.process(&cfg, &mut st, dt, ds4_sample(128.0, 128.0));
        let out = f.process(&cfg, &mut st, dt, ds4_sample(255.0, 128.0));
        let (x, _) = out_ds4(out);
        assert!((x - 255.0).abs() < 1e-4, "got {x}");
    }
}
