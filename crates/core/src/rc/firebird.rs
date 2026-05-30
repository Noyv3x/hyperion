//! The bit-exact FireBird integer RC path (i32 Q4 fixed-point oracle).
//!
//! A 1:1 port of `RcFilter.ProcessAxis` (`RcFilter.cs:193-239`). State lives in Q4 fixed point
//! (`raw12 << 4`, i.e. the DS4 `[0,255]` value scaled by 256), every division **truncates**
//! toward zero like C# integer `/`, and the negative branch clamps the state to
//! `0xfff0 = 65520`. This path is frozen forever as the reference oracle — the headline golden
//! (hold `255` from `128`, param `100`, period `4000` → `153.3984375`) is reproduced here.

/// Per-axis FireBird integer state (positive low-pass + negative lead branch), both Q4.
///
/// Positive and negative branches keep **independent** primed flags and history so switching
/// param sign mid-stream does not cross-contaminate (mirrors the C# separate arrays).
#[derive(Default, Clone, Copy, Debug)]
pub struct FbAxisState {
    /// Whether the positive low-pass branch has been primed (seeded) yet.
    pub pos_primed: bool,
    /// Positive low-pass state, Q4.
    pub pos_q4: i32,
    /// Whether the negative lead branch has been primed yet.
    pub neg_primed: bool,
    /// Negative branch state, Q4.
    pub neg_q4: i32,
    /// Previous input to the negative branch, Q4 (drives the lead/velocity term).
    pub neg_prev_q4: i32,
}

/// Convert a "raw12" integer back to the DS4 `[0,255]` domain (C# `FromFireBirdRaw`).
#[inline]
pub fn from_raw12(raw12: i32) -> f64 {
    (raw12 as f64 / 16.0).clamp(0.0, 255.0)
}

/// Convert a Q4 state back to the DS4 `[0,255]` domain (C# `FromFireBirdStateQ4`).
#[inline]
pub fn from_state_q4(q4: i32) -> f64 {
    (q4 as f64 / 256.0).clamp(0.0, 255.0)
}

/// Advance the FireBird integer recurrence one report; returns the DS4-domain `[0,255]` output.
///
/// Exact port of `ProcessAxis` (i32 truncating division throughout):
/// * `param == 0` → bypass, return `from_raw12(raw12)` (no state touched).
/// * `param > 0` → positive low-pass: on first use prime `pos_q4 = input_q4` and return the raw
///   input; thereafter `pos_q4 += base_value*(input_q4 - pos_q4)/(base_value + param)`.
/// * `param < 0` (`p = -param`) → negative lead branch: on first use prime and return raw; then
///   `blended = (lead_base*input_q4 + p*neg_q4)/(p + lead_base)`,
///   `lead = ((p+25)*(input_q4 - neg_prev_q4))/25`,
///   `neg_q4 = clamp(blended + lead, 0, 0xfff0)`.
pub fn step(st: &mut FbAxisState, raw12: i32, param: i32, lead_base: i32, base_value: i32) -> f64 {
    if param == 0 {
        return from_raw12(raw12);
    }

    let input_q4 = raw12 << 4;

    if param > 0 {
        if !st.pos_primed {
            st.pos_primed = true;
            st.pos_q4 = input_q4;
            return from_raw12(raw12);
        }
        st.pos_q4 += base_value * (input_q4 - st.pos_q4) / (base_value + param);
        return from_state_q4(st.pos_q4);
    }

    // param < 0
    let p = -param;
    if !st.neg_primed {
        st.neg_primed = true;
        st.neg_q4 = input_q4;
        st.neg_prev_q4 = input_q4;
        return from_raw12(raw12);
    }
    let blended = (lead_base * input_q4 + p * st.neg_q4) / (p + lead_base);
    let lead = ((p + 25) * (input_q4 - st.neg_prev_q4)) / 25;
    st.neg_q4 = (blended + lead).clamp(0, 0xfff0);
    st.neg_prev_q4 = input_q4;
    from_state_q4(st.neg_q4)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rc::coeffs::PeriodCoeffs;
    use crate::rc::curve::to_raw12;

    const TOL: f64 = 1e-4;

    fn coeffs() -> PeriodCoeffs {
        PeriodCoeffs::new(4000) // base_value 25, lead_base 50
    }

    #[test]
    fn headline_golden_153_3984375() {
        // From 128 hold 255, param 100, period 4000 -> prime 32768; step to 39270 -> /256.
        let c = coeffs();
        let mut st = FbAxisState::default();
        // prime at 128
        let r0 = to_raw12(128.0);
        let out0 = step(&mut st, r0, 100, c.lead_base, c.base_value);
        assert!((out0 - 128.0).abs() < TOL);
        assert_eq!(st.pos_q4, 32768);
        // step to 255
        let r1 = to_raw12(255.0);
        let out1 = step(&mut st, r1, 100, c.lead_base, c.base_value);
        assert_eq!(st.pos_q4, 39270);
        assert!(
            (out1 - 153.3984375).abs() < TOL,
            "got {out1}, want 153.3984375"
        );
    }

    #[test]
    fn negative_branch_clamps_to_255() {
        // NegativeParamUsesLeadBranchAndClamps: from 128 then 160, param -100 -> 255.0.
        let c = coeffs();
        let mut st = FbAxisState::default();
        let _ = step(&mut st, to_raw12(128.0), -100, c.lead_base, c.base_value);
        let out = step(&mut st, to_raw12(160.0), -100, c.lead_base, c.base_value);
        assert!((out - 255.0).abs() < TOL, "got {out}");
    }

    #[test]
    fn param_zero_bypasses_without_priming() {
        let c = coeffs();
        let mut st = FbAxisState::default();
        let out = step(&mut st, to_raw12(132.0), 0, c.lead_base, c.base_value);
        assert!((out - 132.0).abs() < TOL);
        assert!(!st.pos_primed, "bypass must not prime");
        assert!(!st.neg_primed);
    }

    #[test]
    fn positive_step_from_128_to_160() {
        // PositiveAndNegativeBranchesKeepIndependentHistory first assertion: 134.3984.
        let c = coeffs();
        let mut st = FbAxisState::default();
        let _ = step(&mut st, to_raw12(128.0), 100, c.lead_base, c.base_value);
        let out = step(&mut st, to_raw12(160.0), 100, c.lead_base, c.base_value);
        assert!((out - 134.3984).abs() < TOL, "got {out}");
    }

    #[test]
    fn positive_and_negative_keep_independent_history() {
        // Full sequence from PositiveAndNegativeBranchesKeepIndependentHistory.
        let c = coeffs();
        let mut st = FbAxisState::default();
        let _ = step(&mut st, to_raw12(128.0), 100, c.lead_base, c.base_value);
        let o1 = step(&mut st, to_raw12(160.0), 100, c.lead_base, c.base_value);
        assert!((o1 - 134.3984).abs() < TOL);

        // first negative use primes (returns raw 160).
        let o2 = step(&mut st, to_raw12(160.0), -100, c.lead_base, c.base_value);
        assert!((o2 - 160.0).abs() < TOL, "got {o2}");

        // back to positive: state retained -> 139.5156.
        let o3 = step(&mut st, to_raw12(160.0), 100, c.lead_base, c.base_value);
        assert!((o3 - 139.5156).abs() < TOL, "got {o3}");
    }

    #[test]
    fn zero_param_does_not_refresh_positive_state() {
        // ZeroParamDoesNotRefreshPositiveFilterState -> 147.5156.
        let c = coeffs();
        let mut st = FbAxisState::default();
        let _ = step(&mut st, to_raw12(128.0), 100, c.lead_base, c.base_value);
        let _ = step(&mut st, to_raw12(160.0), 100, c.lead_base, c.base_value);
        // param 0 bypass does not touch pos state.
        let o = step(&mut st, to_raw12(200.0), 0, c.lead_base, c.base_value);
        assert!((o - 200.0).abs() < TOL);
        // resume param 100: continues from the pre-bypass state.
        let o2 = step(&mut st, to_raw12(200.0), 100, c.lead_base, c.base_value);
        assert!((o2 - 147.5156).abs() < TOL, "got {o2}");
    }

    #[test]
    fn zero_param_does_not_refresh_negative_state() {
        // ZeroParamDoesNotRefreshNegativeFilterState -> 190.5547.
        let c = coeffs();
        let mut st = FbAxisState::default();
        let _ = step(&mut st, to_raw12(128.0), -100, c.lead_base, c.base_value);
        let _ = step(&mut st, to_raw12(129.0), -100, c.lead_base, c.base_value);
        let o = step(&mut st, to_raw12(140.0), 0, c.lead_base, c.base_value);
        assert!((o - 140.0).abs() < TOL);
        let o2 = step(&mut st, to_raw12(140.0), -100, c.lead_base, c.base_value);
        assert!((o2 - 190.5547).abs() < TOL, "got {o2}");
    }

    #[test]
    fn positive_branch_does_not_reuse_negative_history_on_first_use() {
        // PositiveBranchDoesNotReuseNegativeHistoryOnFirstUse -> first positive use primes (129).
        let c = coeffs();
        let mut st = FbAxisState::default();
        let _ = step(&mut st, to_raw12(128.0), -100, c.lead_base, c.base_value);
        let _ = step(&mut st, to_raw12(129.0), -100, c.lead_base, c.base_value);
        let o = step(&mut st, to_raw12(129.0), 100, c.lead_base, c.base_value);
        assert!((o - 129.0).abs() < TOL, "got {o}");
    }
}
