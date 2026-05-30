//! FireBird-integer golden values, ported from `HyperionTests/RcFilterTests.cs`.
//!
//! These pin the bit-exact integer Q4 oracle. The headline 153.3984375 (and the
//! negative-lead overshoot/clamp, param==0 bypass, dynamic-curve bypass, zero-param
//! no-refresh, and independent positive/negative history) are reproduced here by calling
//! the low-level `rc::firebird` / `rc::curve` / `rc::coeffs` API directly in the ds4 `[0,255]`
//! Q4 domain, exactly as the C# tests do (no `[-1,1]` round-trip).
//!
//! Every numeric expectation is recomputed from first principles in the comments so the
//! truncating-integer arithmetic is auditable.

use hyperion_core::rc::coeffs::PeriodCoeffs;
use hyperion_core::rc::curve::{self, to_raw12};
use hyperion_core::rc::firebird::{self, FbAxisState};
use hyperion_core::rc::{RcCurve, RcMode};

const TOL: f64 = 1e-4;

/// raw12 for an integer ds4 value `v` in `[0,255]`: `round(v*16)`.
fn raw12_of(v: f64) -> i32 {
    to_raw12(v)
}

#[test]
fn firebird_positive_low_pass_128_to_255_param100_period4000() {
    // PeriodCoeffs(4000): sample_ms=4, base_value=100/4=25, lead_base=200000/4000=50.
    let c = PeriodCoeffs::new(4000);
    assert_eq!(c.base_value, 25);
    assert_eq!(c.lead_base, 50);

    let mut st = FbAxisState::default();
    let param = 100;

    // Prime at 128 -> returns 128, seeds pos_q4 = (128*16)<<4 = 32768.
    let out0 = firebird::step(&mut st, raw12_of(128.0), param, c.lead_base, c.base_value);
    assert!(
        (out0 - 128.0).abs() < TOL,
        "prime returns input, got {out0}"
    );
    assert_eq!(st.pos_q4, 32768);

    // Step to 255: input_q4 = (255*16)<<4 = 65280.
    // pos_q4 += 25*(65280-32768)/(25+100) = 25*32512/125 = 812800/125 = 6502 (trunc).
    // pos_q4 = 39270 -> out = 39270/256 = 153.3984375.
    let out1 = firebird::step(&mut st, raw12_of(255.0), param, c.lead_base, c.base_value);
    assert!(
        (out1 - 153.3984375).abs() < TOL,
        "expected 153.3984375, got {out1}"
    );
    assert_eq!(st.pos_q4, 39270);
}

#[test]
fn firebird_negative_lead_overshoots_then_clamps() {
    // param=-100, period 4000: prime at 128, step to 160 -> clamps to 255.
    let c = PeriodCoeffs::new(4000);
    let mut st = FbAxisState::default();
    let param = -100; // p = -param = 100 in the blend/lead math below

    // Prime at 128: neg_q4 = neg_prev_q4 = 32768, returns 128.
    let out0 = firebird::step(&mut st, raw12_of(128.0), param, c.lead_base, c.base_value);
    assert!((out0 - 128.0).abs() < TOL);
    assert_eq!(st.neg_q4, 32768);
    assert_eq!(st.neg_prev_q4, 32768);

    // Step to 160: input_q4 = (160*16)<<4 = 40960.
    // blended = (lead_base*input + p*neg)/(p+lead_base)
    //         = (50*40960 + 100*32768)/(100+50) = (2048000+3276800)/150 = 5324800/150 = 35498 (trunc).
    // lead = ((p+25)*(input-neg_prev))/25 = (125*(40960-32768))/25 = 125*8192/25 = 40960.
    // neg = clamp(35498+40960=76458, 0, 0xfff0=65520) = 65520 -> out = 65520/256 = 255.9375 -> clamp 255.
    let out1 = firebird::step(&mut st, raw12_of(160.0), param, c.lead_base, c.base_value);
    assert!((out1 - 255.0).abs() < TOL, "expected 255.0, got {out1}");
    assert_eq!(st.neg_q4, 0xfff0);
}

#[test]
fn firebird_param_zero_bypasses_without_priming() {
    // param==0 returns from_raw12(raw12) with NO state change (no prime).
    let c = PeriodCoeffs::new(4000);
    let mut st = FbAxisState::default();

    let out = firebird::step(&mut st, raw12_of(132.0), 0, c.lead_base, c.base_value);
    assert!((out - 132.0).abs() < TOL, "param 0 passes input, got {out}");
    assert!(!st.pos_primed, "param 0 must not prime the positive branch");
    assert!(!st.neg_primed, "param 0 must not prime the negative branch");

    // Switching to param 100 on the next call primes (returns input), so 255 -> 255.
    let out2 = firebird::step(&mut st, raw12_of(255.0), 100, c.lead_base, c.base_value);
    assert!(
        (out2 - 255.0).abs() < TOL,
        "first primed step returns input, got {out2}"
    );
}

#[test]
fn firebird_zero_param_does_not_refresh_positive_state() {
    // Mirrors ZeroParamDoesNotRefreshPositiveFilterState: 147.5156 after the gap.
    let c = PeriodCoeffs::new(4000);
    let mut st = FbAxisState::default();

    // prime 128, step 160 (param 100): pos_q4 = 32768 + 25*(40960-32768)/125 = 32768+1638 = 34406.
    firebird::step(&mut st, raw12_of(128.0), 100, c.lead_base, c.base_value);
    firebird::step(&mut st, raw12_of(160.0), 100, c.lead_base, c.base_value);
    assert_eq!(st.pos_q4, 34406);

    // param 0 at 200 -> bypass, pos_q4 UNCHANGED (no refresh).
    let bypass = firebird::step(&mut st, raw12_of(200.0), 0, c.lead_base, c.base_value);
    assert!((bypass - 200.0).abs() < TOL);
    assert_eq!(
        st.pos_q4, 34406,
        "zero-param step must not refresh pos state"
    );

    // back to param 100 at 200: input_q4 = (200*16)<<4 = 51200.
    // pos_q4 += 25*(51200-34406)/125 = 25*16794/125 = 419850/125 = 3358 (trunc) -> 37764.
    // out = 37764/256 = 147.515625.
    let out = firebird::step(&mut st, raw12_of(200.0), 100, c.lead_base, c.base_value);
    assert!((out - 147.5156).abs() < TOL, "expected 147.5156, got {out}");
}

#[test]
fn firebird_zero_param_does_not_refresh_negative_state() {
    // Mirrors ZeroParamDoesNotRefreshNegativeFilterState: 190.5547 after the gap.
    let c = PeriodCoeffs::new(4000);
    let mut st = FbAxisState::default();

    // prime 128 then step 129 at param -100.
    firebird::step(&mut st, raw12_of(128.0), -100, c.lead_base, c.base_value);
    firebird::step(&mut st, raw12_of(129.0), -100, c.lead_base, c.base_value);
    let neg_after = st.neg_q4;
    let neg_prev_after = st.neg_prev_q4;

    // param 0 at 140 -> bypass, negative state UNCHANGED.
    let bypass = firebird::step(&mut st, raw12_of(140.0), 0, c.lead_base, c.base_value);
    assert!((bypass - 140.0).abs() < TOL);
    assert_eq!(
        st.neg_q4, neg_after,
        "zero-param must not refresh neg state"
    );
    assert_eq!(st.neg_prev_q4, neg_prev_after);

    // back to param -100 at 140 -> 190.5547.
    let out = firebird::step(&mut st, raw12_of(140.0), -100, c.lead_base, c.base_value);
    assert!((out - 190.5547).abs() < TOL, "expected 190.5547, got {out}");
}

#[test]
fn firebird_independent_positive_and_negative_history() {
    // Mirrors PositiveAndNegativeBranchesKeepIndependentHistory.
    let c = PeriodCoeffs::new(4000);
    let mut st = FbAxisState::default();

    firebird::step(&mut st, raw12_of(128.0), 100, c.lead_base, c.base_value);
    let a = firebird::step(&mut st, raw12_of(160.0), 100, c.lead_base, c.base_value);
    assert!((a - 134.3984).abs() < TOL, "expected 134.3984, got {a}");

    // First negative use primes -> returns input 160.
    let b = firebird::step(&mut st, raw12_of(160.0), -100, c.lead_base, c.base_value);
    assert!(
        (b - 160.0).abs() < TOL,
        "first neg use primes to input, got {b}"
    );

    // Positive branch resumes from its OWN preserved history, not the negative one.
    let d = firebird::step(&mut st, raw12_of(160.0), 100, c.lead_base, c.base_value);
    assert!((d - 139.5156).abs() < TOL, "expected 139.5156, got {d}");
}

#[test]
fn firebird_positive_branch_does_not_reuse_negative_history_on_first_use() {
    // Mirrors PositiveBranchDoesNotReuseNegativeHistoryOnFirstUse: first pos use returns input.
    let c = PeriodCoeffs::new(4000);
    let mut st = FbAxisState::default();

    firebird::step(&mut st, raw12_of(128.0), -100, c.lead_base, c.base_value);
    firebird::step(&mut st, raw12_of(129.0), -100, c.lead_base, c.base_value);

    // First positive use (param 100) primes the positive branch -> returns input 129.
    let out = firebird::step(&mut st, raw12_of(129.0), 100, c.lead_base, c.base_value);
    assert!(
        (out - 129.0).abs() < TOL,
        "first pos use returns input, got {out}"
    );
}

#[test]
fn dynamic_curve_can_bypass_at_configured_speed() {
    // Port of DynamicCurveCanBypassAtConfiguredSpeed: a curve that returns param 0 at the
    // observed speed makes the filter a pass-through (output == input).
    // Curve y0=100,x1=1,y1=100,x2=128,y2=0,y3=0 ; period 4000.
    let curve = RcCurve {
        y0: 100,
        x1: 1,
        y1: 100,
        x2: 128,
        y2: 0,
        y3: 0,
    };
    let c = PeriodCoeffs::new(4000);

    // Speeds large enough land on the y2/y3=0 segment -> param 0 -> bypass.
    // Use the legacy speed metric for the FireBird mode.
    let rx_prev = raw12_of(128.0);
    let rx_now = raw12_of(255.0);
    let delta = (rx_now - rx_prev).abs();
    let speed = curve::speed_legacy(delta, c.period_us);
    let param = curve::param_from_speed(&curve, speed);
    assert_eq!(param, 0, "high-speed segment of this curve yields param 0");

    let mut st = FbAxisState::default();
    firebird::step(&mut st, rx_prev, param, c.lead_base, c.base_value);
    let out = firebird::step(&mut st, rx_now, param, c.lead_base, c.base_value);
    assert!(
        (out - 255.0).abs() < TOL,
        "param-0 bypass passes input, got {out}"
    );
}

#[test]
fn param_from_speed_piecewise_matches_csharp_segments() {
    // Default curve is flat 100 everywhere -> param 100 at every speed.
    let flat = RcCurve::default();
    for speed in [0, 16, 32, 64, 96, 128] {
        assert_eq!(
            curve::param_from_speed(&flat, speed),
            100,
            "flat curve must yield 100 at speed {speed}"
        );
    }

    // A linear ramp on the first segment: y0=0, x1=128, y1=500 -> at speed s: 0 + s*500/128.
    let ramp = RcCurve {
        y0: 0,
        x1: 128,
        y1: 500,
        x2: 128,
        y2: 500,
        y3: 500,
    };
    // speed 64: 64*500/128 = 250 (trunc).
    assert_eq!(curve::param_from_speed(&ramp, 64), 250);
    // speed 0: y0 = 0.
    assert_eq!(curve::param_from_speed(&ramp, 0), 0);
    // speed >= 128: top of first segment = 500, then clamped to MAX_PARAM 500.
    assert_eq!(curve::param_from_speed(&ramp, 128), 500);
}

#[test]
fn speed_dt_at_dt_equals_period_matches_legacy() {
    // The dt speed metric reduces to legacy at dt == period (DESIGN §4.4).
    let period = 4000;
    for delta in [0, 17, 128, 555, 4096] {
        let legacy = curve::speed_legacy(delta, period);
        let dt = curve::speed_dt(delta, period as f64, period);
        assert_eq!(
            legacy, dt,
            "speed_dt must equal speed_legacy at dt==period (delta={delta})"
        );
    }
}

#[test]
fn rc_mode_default_is_ultimate_dt() {
    assert_eq!(RcMode::default(), RcMode::UltimateDt);
}
