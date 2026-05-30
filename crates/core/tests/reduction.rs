//! The `dt = periodUs` reduction (DESIGN §4.4): the rate-invariant `UltimateDt` recurrence
//! must collapse to the legacy `UltimateLegacy` recurrence when one report spans exactly one
//! period. We drive a ramp-then-hold sequence through both at `dt_us == period_us` and assert
//! per-axis equality within 1e-9 for several params, positive and negative branches.

use hyperion_core::rc::coeffs::PeriodCoeffs;
use hyperion_core::rc::ultimate::{self, UltAxisState};

/// A ramp up, hold, ramp down, hold sequence in the ds4 [0,255] domain.
fn drive_sequence() -> Vec<f64> {
    let mut seq = Vec::new();
    // ramp up 128 -> 230
    let mut v = 128.0;
    while v <= 230.0 {
        seq.push(v);
        v += 7.3;
    }
    // hold
    seq.extend(std::iter::repeat_n(230.0, 50));
    // ramp down 230 -> 90
    let mut v = 230.0;
    while v >= 90.0 {
        seq.push(v);
        v -= 5.1;
    }
    // hold
    seq.extend(std::iter::repeat_n(90.0, 50));
    seq
}

fn assert_branch_reduces(period: i32, param: i32) {
    let c = PeriodCoeffs::new(period);
    let dt_us = period as f64; // dt == period -> ratio 1

    let mut legacy = UltAxisState::default();
    let mut dt = UltAxisState::default();

    for (i, &input) in drive_sequence().iter().enumerate() {
        let out_legacy =
            ultimate::step_legacy(&mut legacy, input, param, c.lead_base, c.base_value);
        let out_dt = ultimate::step_dt(
            &mut dt,
            input,
            param,
            c.lead_base,
            c.base_value,
            dt_us,
            period,
        );
        assert!(
            (out_legacy - out_dt).abs() < 1e-9,
            "step {i}: period={period} param={param} input={input} \
             legacy={out_legacy} dt={out_dt} (diff {})",
            (out_legacy - out_dt).abs()
        );
    }
}

#[test]
fn positive_branch_reduces_to_legacy_at_dt_equals_period() {
    for &period in &[1000, 2000, 4000, 8000] {
        for &param in &[1, 25, 100, 250, 500] {
            assert_branch_reduces(period, param);
        }
    }
}

#[test]
fn negative_branch_reduces_to_legacy_at_dt_equals_period() {
    for &period in &[1000, 2000, 4000, 8000] {
        for &param in &[-1, -25, -100, -250, -500] {
            assert_branch_reduces(period, param);
        }
    }
}

#[test]
fn bypass_param_zero_is_identical_in_both() {
    let c = PeriodCoeffs::new(4000);
    let mut legacy = UltAxisState::default();
    let mut dt = UltAxisState::default();
    for &input in &[0.0, 64.3, 128.0, 200.7, 255.0, 300.0, -5.0] {
        let a = ultimate::step_legacy(&mut legacy, input, 0, c.lead_base, c.base_value);
        let b = ultimate::step_dt(&mut dt, input, 0, c.lead_base, c.base_value, 4000.0, 4000);
        // param==0 is a clamp-to-[0,255] bypass in both modes.
        assert!(
            (a - b).abs() < 1e-12,
            "bypass mismatch input={input}: {a} vs {b}"
        );
        assert!((a - input.clamp(0.0, 255.0)).abs() < 1e-12);
    }
}
