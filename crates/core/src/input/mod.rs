//! Device-agnostic input parsing, normalization, and time-base folding.
//!
//! The Windows HID shell does I/O only: it hands a raw `&[u8]` report plus a host QPC
//! timestamp to the pure parsers here, which produce a canonical [`InputSample`] in the
//! `f64` `[-1,1]` / `[0,1]` units the rest of the engine speaks. Everything in this module
//! is OS-free and unit-tested on Linux CI.
//!
//! Submodules:
//! * [`normalize`] — raw 8/16-bit integer → canonical unit maps.
//! * [`ds_report`] — DualSense (DS4-compatible report `0x01`) byte decode.
//! * [`dt_clock`] — the [`SensorClock`] that folds the hardware timestamp into a guarded `dt`.
//! * [`seq`] — the [`SeqTracker`] that derives dropped/duplicate counts from the frame counter.

pub mod ds_report;
pub mod dt_clock;
pub mod normalize;
pub mod seq;

pub use dt_clock::SensorClock;
pub use seq::SeqTracker;

/// One stick's high-precision X/Y in the canonical `[-1,1]` unit (`+y == up`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StickPair {
    pub x: f64,
    pub y: f64,
}

/// A packed device button bitfield (layout is device-specific; carried opaquely).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Buttons(pub u32);

/// A fully-normalized input report the device layer produces for the engine.
///
/// Sticks are canonical `[-1,1]` (`+y == up`), triggers `[0,1]`. `seq` is the device frame
/// counter; `dropped`/`is_duplicate` are derived from it. `dt_us` is the guarded real
/// elapsed time since the previous report (`0.0` on the priming report), and `host_qpc_ns`
/// is the host high-resolution timestamp captured at read completion.
#[derive(Clone, Copy, Debug, Default)]
pub struct InputSample {
    pub left: StickPair,
    pub right: StickPair,
    pub l2: f64,
    pub r2: f64,
    pub buttons: Buttons,
    pub seq: u8,
    pub dropped: u16,
    pub is_duplicate: bool,
    pub dt_us: f64,
    pub host_qpc_ns: u64,
}

/// Static identity of an input source: USB IDs, a human label, and the stick bit depth.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SourceMeta {
    pub vid: u16,
    pub pid: u16,
    pub name: &'static str,
    pub stick_bits: u8,
}
