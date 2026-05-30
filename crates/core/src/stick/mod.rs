//! The pluggable stick-processing contract and the full DS4Windows-class stick pipeline.
//!
//! A [`StickAlgorithm`] is a pure function of `(sample, dt, config, &mut state)` — no clock,
//! no allocation, no interior mutability, no I/O — so experimental algorithms drop in behind
//! one interface and are trivially testable. The engine owns the per-stick `State` and feeds
//! the guarded real elapsed time each report.
//!
//! On top of that primitive contract this module hosts the full ordered stick chain ported
//! from DS4Windows' `SetCurveAndDeadzone`:
//!
//! * [`settings`] — [`StickSettings`](settings::StickSettings) and the per-stage parameter
//!   structs (deadzone, anti-snapback, fuzz, rotation, square-stick, output curve, flick),
//!   plus the resident per-stick [`StickState`](settings::StickState).
//! * [`stages`] — the pure per-stage helpers in the DS4 `[0,255]` f64 domain.
//! * [`pipeline`] — [`process_stick`](pipeline::process_stick), the fixed-order chain that
//!   reuses the existing bit-exact [`RcFilter`](crate::rc::RcFilter) as stage 0.

pub mod pipeline;
pub mod settings;
pub mod stages;

pub use pipeline::process_stick;
pub use settings::{
    AntiSnapback, DeadZoneType, FlickStick, OutputCurve, RotationSettings, SnapbackRing,
    SquareStick, StickDeadZone, StickSettings, StickState, SNAP_CAP,
};

use crate::axis::Axis;
use crate::dt::Dt;

/// One stick's high-precision X/Y in the canonical `[-1,1]` unit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StickSample {
    pub x: Axis,
    pub y: Axis,
}

impl StickSample {
    pub const NEUTRAL: Self = Self { x: 0.0, y: 0.0 };

    #[inline]
    pub fn new(x: Axis, y: Axis) -> Self {
        Self { x, y }
    }
}

/// A pluggable per-stick processing algorithm.
///
/// Contract:
/// * [`prime`](StickAlgorithm::prime) is called for the first report after enable/reset:
///   it seeds history and returns nothing — the caller emits the input unchanged with **no**
///   filter step (the first report has no meaningful `dt`).
/// * [`process`](StickAlgorithm::process) advances exactly one report by the guarded `dt`.
/// * [`reset`](StickAlgorithm::reset) restores the default (unprimed) state.
pub trait StickAlgorithm {
    type Config;
    type State: Default;

    /// First report after enable/reset: seed history, take no filter step.
    fn prime(&self, cfg: &Self::Config, st: &mut Self::State, s: StickSample);

    /// Advance one report by the guarded real elapsed time `dt`.
    fn process(
        &self,
        cfg: &Self::Config,
        st: &mut Self::State,
        dt: Dt,
        s: StickSample,
    ) -> StickSample;

    /// Clear all state back to unprimed defaults.
    fn reset(&self, st: &mut Self::State) {
        *st = Self::State::default();
    }
}
