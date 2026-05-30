//! Report-rate invariance of `UltimateDt` (DESIGN §4.4, resolved-conflict #4).
//!
//! The SAME wall-clock signal sampled at different report rates (different `dt` partitions)
//! is pushed through `step_dt`. We assert:
//!   (a) on a HELD segment the positive low-pass converges to **equality** across rates
//!       (<1e-6) — the telescoping product is exact for any partition of the wall-clock
//!       interval, and a constant-from-prime input makes both branches exactly rate-free;
//!   (b) on a genuinely MOVING ramp the cross-rate spread is **bounded** and **shrinks**
//!       as `dt` shrinks (first-order O(dt) convergence) — NOT 1e-6 equality, which ZOH
//!       cannot give on moving input.
//!
//! A couple of proptest cases guard robustness, but the deterministic asserts are primary.

use hyperion_core::rc::coeffs::PeriodCoeffs;
use hyperion_core::rc::ultimate::{self, UltAxisState};
use proptest::prelude::*;

const PERIOD: i32 = 4000;

/// Drive `step_dt` over a wall-clock window `t_total_us`, sampling at `dt_us`, with input
/// given by `input_at(t_us)`. Primes at `input_at(0.0)`. `param`'s sign selects the
/// positive low-pass or the negative blend+lead branch inside `step_dt`.
fn run(dt_us: f64, param: i32, t_total_us: f64, input_at: &dyn Fn(f64) -> f64) -> f64 {
    let c = PeriodCoeffs::new(PERIOD);
    let mut st = UltAxisState::default();
    let mut out = ultimate::step_dt(
        &mut st,
        input_at(0.0),
        param,
        c.lead_base,
        c.base_value,
        dt_us,
        PERIOD,
    );
    let mut t = dt_us;
    while t <= t_total_us + 1e-6 {
        out = ultimate::step_dt(
            &mut st,
            input_at(t),
            param,
            c.lead_base,
            c.base_value,
            dt_us,
            PERIOD,
        );
        t += dt_us;
    }
    out
}

#[test]
fn held_positive_segment_is_rate_invariant_to_1e6() {
    // Prime at 128, then hold 255 for 80 ms at 250/1000/4000 Hz (dt = 4000/1000/250 us).
    let input = |t: f64| if t == 0.0 { 128.0 } else { 255.0 };
    let o_250 = run(4000.0, 100, 80_000.0, &input);
    let o_1000 = run(1000.0, 100, 80_000.0, &input);
    let o_4000 = run(250.0, 100, 80_000.0, &input);
    assert!(
        (o_250 - o_1000).abs() < 1e-6 && (o_1000 - o_4000).abs() < 1e-6,
        "held low-pass must be rate-invariant: {o_250} {o_1000} {o_4000}"
    );
}

#[test]
fn constant_from_prime_is_exactly_rate_invariant_both_branches() {
    // Input never changes from the prime value -> lead is always 0 and the low-pass is
    // already at target, so BOTH branches are bit-stable across any dt partition.
    let hold = |_t: f64| 200.0;
    for &param in &[100, -100] {
        let a = run(4000.0, param, 60_000.0, &hold);
        let b = run(1000.0, param, 60_000.0, &hold);
        let c = run(250.0, param, 60_000.0, &hold);
        assert!((a - 200.0).abs() < 1e-9, "param {param}: {a}");
        assert!((a - b).abs() < 1e-9 && (b - c).abs() < 1e-9);
    }
}

/// Gaps of each level to the finest grid, for a moving ramp. Returns the ordered
/// `(dt, gap_to_finest)` pairs from coarse to fine (finest excluded).
fn ramp_gaps(param: i32, t_total_us: f64, v0: f64, vel_per_us: f64) -> Vec<(f64, f64)> {
    let input = move |t: f64| (v0 + vel_per_us * t).min(255.0);
    let dts = [4000.0, 2000.0, 1000.0, 500.0, 250.0, 125.0];
    let outs: Vec<f64> = dts
        .iter()
        .map(|&dt| run(dt, param, t_total_us, &input))
        .collect();
    let finest = *outs.last().unwrap();
    dts[..dts.len() - 1]
        .iter()
        .zip(&outs[..outs.len() - 1])
        .map(|(&dt, &o)| (dt, (o - finest).abs()))
        .collect()
}

#[test]
fn moving_ramp_positive_spread_bounded_and_shrinking() {
    // Positive low-pass, ramp 128 + 0.001/us over 60 ms (~188 cap).
    let gaps = ramp_gaps(100, 60_000.0, 128.0, 0.001);

    // Bounded: the coarsest grid stays within a couple of units of the finest.
    assert!(gaps[0].1 < 3.0, "positive spread too large: {:?}", gaps);

    // Strictly shrinking AND each halving at least ~halves the gap (first-order, with slack).
    for w in gaps.windows(2) {
        let (coarse, fine) = (w[0].1, w[1].1);
        assert!(
            fine < coarse,
            "gap must shrink: coarse={coarse} fine={fine}"
        );
        assert!(
            fine <= 0.6 * coarse,
            "halving dt must roughly halve the gap: coarse={coarse} fine={fine}"
        );
    }
}

#[test]
fn moving_ramp_negative_spread_bounded_and_shrinking() {
    // Negative blend+lead branch, ramp 128 + 0.0008/us over 50 ms.
    let gaps = ramp_gaps(-100, 50_000.0, 128.0, 0.0008);

    // Bounded (the lead branch has larger absolute spread, still finite and small).
    assert!(gaps[0].1 < 10.0, "negative spread too large: {:?}", gaps);

    for w in gaps.windows(2) {
        let (coarse, fine) = (w[0].1, w[1].1);
        assert!(
            fine < coarse,
            "gap must shrink: coarse={coarse} fine={fine}"
        );
        assert!(
            fine <= 0.6 * coarse,
            "halving dt must roughly halve the gap: coarse={coarse} fine={fine}"
        );
    }
}

#[test]
fn duplicates_contribute_nothing_on_held_input() {
    // A "duplicate" report (same input, tiny real dt) on a held signal must not move the
    // output meaningfully: input-prev=0 (lead 0) and the blend barely advances.
    let c = PeriodCoeffs::new(PERIOD);
    let mut st = UltAxisState::default();
    ultimate::step_dt(
        &mut st,
        128.0,
        -100,
        c.lead_base,
        c.base_value,
        4000.0,
        PERIOD,
    );
    let settled = ultimate::step_dt(
        &mut st,
        200.0,
        -100,
        c.lead_base,
        c.base_value,
        4000.0,
        PERIOD,
    );
    // Now feed several "duplicates" of 200 at the dt-guard floor (100us).
    let mut last = settled;
    for _ in 0..10 {
        last = ultimate::step_dt(
            &mut st,
            200.0,
            -100,
            c.lead_base,
            c.base_value,
            100.0,
            PERIOD,
        );
    }
    // The overshoot decays toward 200 monotonically and never blows up.
    assert!(
        last <= settled + 1e-9,
        "duplicates must not amplify: {last} vs {settled}"
    );
    assert!(last >= 200.0 - 1e-6, "should settle toward the held input");
}

proptest! {
    // Robustness: for any moderate constant velocity, the 4000 Hz output is at least as
    // close to the 8000 Hz reference as the 250 Hz output is (finer dt never worse).
    #[test]
    fn finer_dt_is_never_worse_than_coarser(
        vel_milliunits_per_us in 1i32..15,
        param in prop_oneof![Just(100i32), Just(-100i32)],
    ) {
        let vel = vel_milliunits_per_us as f64 * 0.0001;
        let input = move |t: f64| (128.0 + vel * t).min(255.0);
        let t_total = 40_000.0;
        let o_ref = run(125.0, param, t_total, &input);     // ~8 kHz reference
        let o_fine = run(250.0, param, t_total, &input);    // 4 kHz
        let o_coarse = run(4000.0, param, t_total, &input); // 250 Hz
        prop_assert!(
            (o_fine - o_ref).abs() <= (o_coarse - o_ref).abs() + 1e-9,
            "finer dt should not be farther from the reference: fine={o_fine} coarse={o_coarse} ref={o_ref}"
        );
    }
}
