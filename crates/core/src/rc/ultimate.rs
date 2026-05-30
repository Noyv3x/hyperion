//! The Ultimate RC path: f64 legacy (rate-coupled) and the corrected dt-compensated form.
//!
//! * [`step_legacy`] is a 1:1 port of `RcFilter.ProcessAxisUltimate` (`RcFilter.cs:241-258`),
//!   operating in the `*256` Q-domain so it converges to a held input exactly (no
//!   integer-truncation dead-band) while sharing the FireBird response shape.
//! * [`step_dt`] is the NEW report-rate-invariant form (DESIGN §4.3). It works **directly** in
//!   the `[0,255]` domain; the low-pass and blend retention are `dt`-exponentiated, and the lead
//!   is the **corrected displacement** form (`g·(input-prev)`, no spurious `period/dt` factor).
//!
//! Reduction guarantee (pinned by tests below at 1e-12): at `dt == period_us`, [`step_dt`]
//! reproduces [`step_legacy`] on every branch (`k_dt = k`, `r_dt = r`, lead already dt-free).

/// Per-axis Ultimate state (f64), shared by the legacy and dt paths.
///
/// As with [`super::firebird::FbAxisState`], the positive and negative branches keep independent
/// primed flags and history. `pos`/`neg` are in the `*256` Q-domain for [`step_legacy`] and in
/// the direct `[0,255]` domain for [`step_dt`]; a given filter instance only ever drives one
/// path (selected by [`super::RcMode`]), so the domains never mix on the same state.
#[derive(Default, Clone, Copy, Debug)]
pub struct UltAxisState {
    /// Whether the positive low-pass branch has been primed.
    pub pos_primed: bool,
    /// Positive low-pass state.
    pub pos: f64,
    /// Whether the negative blend/lead branch has been primed.
    pub neg_primed: bool,
    /// Negative branch state.
    pub neg: f64,
    /// Previous input to the negative branch (drives the lead term).
    pub neg_prev: f64,
}

/// Advance the legacy (rate-coupled) Ultimate recurrence; returns DS4-domain `[0,255]`.
///
/// Exact port of `ProcessAxisUltimate`, `*256` Q-domain:
/// * `param == 0` → bypass, `input_ds4.clamp(0,255)` (full precision, no `1/16` snap).
/// * `param > 0` → prime returns input; then `pos += base_value*(scaled-pos)/(base_value+param)`,
///   output `pos/256`.
/// * `param < 0` (`p = -param`) → prime; then
///   `blended = (lead_base*scaled + p*neg)/(p+lead_base)`,
///   `lead = ((p+25)*(scaled-neg_prev))/25.0`,
///   `neg = clamp(blended+lead, 0.0, 65280.0)`, output `neg/256`.
pub fn step_legacy(
    st: &mut UltAxisState,
    input_ds4: f64,
    param: i32,
    lead_base: i32,
    base_value: i32,
) -> f64 {
    if param == 0 {
        return input_ds4.clamp(0.0, 255.0);
    }

    let scaled = input_ds4 * 256.0;

    if param > 0 {
        if !st.pos_primed {
            st.pos_primed = true;
            st.pos = scaled;
            return input_ds4.clamp(0.0, 255.0);
        }
        st.pos += base_value as f64 * (scaled - st.pos) / (base_value as f64 + param as f64);
        return (st.pos / 256.0).clamp(0.0, 255.0);
    }

    // param < 0
    let p = (-param) as f64;
    if !st.neg_primed {
        st.neg_primed = true;
        st.neg = scaled;
        st.neg_prev = scaled;
        return input_ds4.clamp(0.0, 255.0);
    }
    let blended = (lead_base as f64 * scaled + p * st.neg) / (p + lead_base as f64);
    let lead = ((p + 25.0) * (scaled - st.neg_prev)) / 25.0;
    st.neg = (blended + lead).clamp(0.0, 65280.0);
    st.neg_prev = scaled;
    (st.neg / 256.0).clamp(0.0, 255.0)
}

/// Advance the corrected dt-compensated Ultimate recurrence; returns DS4-domain `[0,255]`.
///
/// Works directly in `[0,255]` (DESIGN §4.3). `ratio = dt_us / period_us`.
/// * `param == 0` → bypass, `input`.
/// * `param > 0` → prime (`pos = input`) returns input; then with
///   `k = base_value/(base_value+param)`, `k_dt = 1-(1-k)^ratio`:
///   `pos += k_dt*(input-pos)`, output `pos.clamp(0,255)` (state left unclamped — convex, so safe).
/// * `param < 0` (`p = -param`) → prime (`neg = input`, `neg_prev = input`) returns input; then
///   with `r = p/(p+lead_base)`, `r_dt = r^ratio`:
///   `blended = neg*r_dt + input*(1-r_dt)`,
///   `lead = ((p+25)/25)*(input-neg_prev)` (DISPLACEMENT — no `/dt`, the corrected form),
///   `neg = clamp(blended+lead, 0.0, 255.0)`, `neg_prev = input`, output `neg`.
pub fn step_dt(
    st: &mut UltAxisState,
    input_ds4: f64,
    param: i32,
    lead_base: i32,
    base_value: i32,
    dt_us: f64,
    period_us: i32,
) -> f64 {
    let input = input_ds4.clamp(0.0, 255.0);
    if param == 0 {
        return input;
    }

    let ratio = dt_us / period_us as f64;

    if param > 0 {
        if !st.pos_primed {
            st.pos_primed = true;
            st.pos = input;
            return input;
        }
        let k = base_value as f64 / (base_value as f64 + param as f64);
        let k_dt = 1.0 - (1.0 - k).powf(ratio);
        st.pos += k_dt * (input - st.pos);
        return st.pos.clamp(0.0, 255.0);
    }

    // param < 0
    let p = (-param) as f64;
    if !st.neg_primed {
        st.neg_primed = true;
        st.neg = input;
        st.neg_prev = input;
        return input;
    }
    let r = p / (p + lead_base as f64);
    let r_dt = r.powf(ratio);
    let blended = st.neg * r_dt + input * (1.0 - r_dt);
    // Corrected lead: displacement this step, NO /dt and NO *period factor.
    let lead = ((p + 25.0) / 25.0) * (input - st.neg_prev);
    st.neg = (blended + lead).clamp(0.0, 255.0);
    st.neg_prev = input;
    st.neg
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rc::coeffs::PeriodCoeffs;

    fn coeffs() -> PeriodCoeffs {
        PeriodCoeffs::new(4000) // base_value 25, lead_base 50
    }

    #[test]
    fn legacy_bypass_is_full_precision() {
        let c = coeffs();
        let mut st = UltAxisState::default();
        let out = step_legacy(&mut st, 200.7, 0, c.lead_base, c.base_value);
        assert!((out - 200.7).abs() < 1e-9);
        assert!(!st.pos_primed && !st.neg_primed);
    }

    #[test]
    fn legacy_positive_prime_returns_fractional_input() {
        let c = coeffs();
        let mut st = UltAxisState::default();
        let out = step_legacy(&mut st, 200.7, 100, c.lead_base, c.base_value);
        assert!((out - 200.7).abs() < 1e-9);
        assert!(st.pos_primed);
    }

    #[test]
    fn legacy_positive_converges_to_held_input_exactly() {
        let c = coeffs();
        let mut st = UltAxisState::default();
        let _ = step_legacy(&mut st, 128.0, 100, c.lead_base, c.base_value);
        let mut out = 0.0;
        for _ in 0..2000 {
            out = step_legacy(&mut st, 255.0, 100, c.lead_base, c.base_value);
        }
        assert!((out - 255.0).abs() < 1e-6, "got {out}");
    }

    #[test]
    fn legacy_negative_overshoots_then_settles() {
        let c = coeffs();
        let mut st = UltAxisState::default();
        let _ = step_legacy(&mut st, 128.0, -100, c.lead_base, c.base_value);
        let step_out = step_legacy(&mut st, 160.0, -100, c.lead_base, c.base_value);
        assert!(step_out > 160.0, "should overshoot, got {step_out}");
        let mut out = 0.0;
        for _ in 0..2000 {
            out = step_legacy(&mut st, 160.0, -100, c.lead_base, c.base_value);
        }
        assert!((out - 160.0).abs() < 1e-6, "should settle, got {out}");
        assert!(out < step_out);
    }

    #[test]
    fn dt_bypass_is_dt_independent() {
        let c = coeffs();
        for &dt in &[250.0, 1000.0, 4000.0] {
            let mut st = UltAxisState::default();
            let out = step_dt(
                &mut st,
                200.7,
                0,
                c.lead_base,
                c.base_value,
                dt,
                c.period_us,
            );
            assert!((out - 200.7).abs() < 1e-12, "dt={dt}");
        }
    }

    #[test]
    fn dt_positive_reduces_to_legacy_at_dt_equals_period() {
        let c = coeffs();
        let inputs = [128.0, 160.0, 200.0, 255.0, 200.0, 140.0, 130.0, 255.0];
        let mut sl = UltAxisState::default();
        let mut sd = UltAxisState::default();
        for &inp in &inputs {
            let ol = step_legacy(&mut sl, inp, 100, c.lead_base, c.base_value);
            let od = step_dt(
                &mut sd,
                inp,
                100,
                c.lead_base,
                c.base_value,
                c.period_us as f64,
                c.period_us,
            );
            assert!((ol - od).abs() < 1e-12, "inp={inp} legacy={ol} dt={od}");
        }
    }

    #[test]
    fn dt_negative_reduces_to_legacy_at_dt_equals_period() {
        let c = coeffs();
        // Moving + held inputs exercise both the blend and the lead term.
        let inputs = [128.0, 160.0, 180.0, 180.0, 150.0, 120.0, 200.0, 200.0];
        let mut sl = UltAxisState::default();
        let mut sd = UltAxisState::default();
        for &inp in &inputs {
            let ol = step_legacy(&mut sl, inp, -100, c.lead_base, c.base_value);
            let od = step_dt(
                &mut sd,
                inp,
                -100,
                c.lead_base,
                c.base_value,
                c.period_us as f64,
                c.period_us,
            );
            assert!((ol - od).abs() < 1e-12, "inp={inp} legacy={ol} dt={od}");
        }
    }

    #[test]
    fn dt_positive_exact_on_held_input_any_rate() {
        // Held input: the low-pass retention telescopes, so total decay over a fixed wall-clock
        // is the SAME regardless of how it's partitioned into dt's (exact rate invariance).
        let c = coeffs();
        // Reference: one big step of 8000us (= 2 periods).
        let mut s_ref = UltAxisState::default();
        let _ = step_dt(
            &mut s_ref,
            128.0,
            100,
            c.lead_base,
            c.base_value,
            4000.0,
            c.period_us,
        );
        let out_ref = step_dt(
            &mut s_ref,
            255.0,
            100,
            c.lead_base,
            c.base_value,
            8000.0,
            c.period_us,
        );

        // Same wall-clock 8000us split into 32 steps of 250us, held at 255.
        let mut s_fine = UltAxisState::default();
        let _ = step_dt(
            &mut s_fine,
            128.0,
            100,
            c.lead_base,
            c.base_value,
            4000.0,
            c.period_us,
        );
        let mut out_fine = 0.0;
        for _ in 0..32 {
            out_fine = step_dt(
                &mut s_fine,
                255.0,
                100,
                c.lead_base,
                c.base_value,
                250.0,
                c.period_us,
            );
        }
        assert!(
            (out_ref - out_fine).abs() < 1e-9,
            "held-input rate invariance: ref={out_ref} fine={out_fine}"
        );
    }

    #[test]
    fn dt_negative_lead_is_bounded_across_rates() {
        // The corrected lead does NOT saturate at high rates: a ramp 100->140 over 40ms wall
        // clock stays bounded and convergent (DESIGN §4.3 worked example ~191/181/179).
        let c = coeffs();

        fn ramp(c: &PeriodCoeffs, dt_us: f64, steps: usize) -> f64 {
            let mut st = UltAxisState::default();
            let _ = step_dt(
                &mut st,
                100.0,
                -100,
                c.lead_base,
                c.base_value,
                dt_us,
                c.period_us,
            );
            let mut out = 0.0;
            for i in 1..=steps {
                let frac = i as f64 / steps as f64;
                let inp = 100.0 + 40.0 * frac;
                out = step_dt(
                    &mut st,
                    inp,
                    -100,
                    c.lead_base,
                    c.base_value,
                    dt_us,
                    c.period_us,
                );
            }
            out
        }
        // 40ms ramp at 3 rates: 10 steps@4000us, 40@1000us, 160@250us.
        let r4000 = ramp(&c, 4000.0, 10);
        let r1000 = ramp(&c, 1000.0, 40);
        let r250 = ramp(&c, 250.0, 160);
        // All bounded well under the 255 clamp and within a few units of each other.
        for v in [r4000, r1000, r250] {
            assert!(
                (150.0..=210.0).contains(&v),
                "lead out of expected band: {v}"
            );
        }
        // Higher rate does NOT blow up past the lower rate (no 1/dt saturation).
        assert!(
            (r4000 - r250).abs() < 20.0,
            "spread too large: {r4000} vs {r250}"
        );
    }
}
