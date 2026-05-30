//! `hyperion_core` — the pure, OS-free numeric core of the Hyperion controller engine.
//!
//! Everything here is deterministic, allocation-light, and free of any operating-system
//! dependency, so the entire crate is unit-tested on Linux CI as well as Windows. It owns:
//!
//! * the canonical high-precision stick unit and precision-preserving conversions
//!   ([`axis`], [`convert`], [`output`]),
//! * the [`StickAlgorithm`](stick::StickAlgorithm) pluggable contract ([`stick`]),
//! * the RC stick filter in three modes — bit-exact FireBird integer, Ultimate (f64),
//!   and the report-rate-invariant dt-compensated Ultimate ([`rc`]),
//! * the guarded report time-base ([`dt`]),
//! * device-agnostic input parsing/normalization ([`input`]),
//! * and the serde config tree ([`config`]).
//!
//! No mid-chain quantization: raw int → f64 all the way to a single final i16 egress.

pub mod axis;
pub mod config;
pub mod convert;
pub mod dt;
pub mod input;
pub mod output;
pub mod rc;
pub mod stick;

pub use axis::{clamp_axis, Axis, RawStickFormat};
pub use dt::{Dt, DT_MAX_US, DT_MIN_US};
pub use output::OutputFrame;
pub use rc::{RcConfig, RcCurve, RcFilter, RcMode, RcStickState};
pub use stick::{StickAlgorithm, StickSample};
