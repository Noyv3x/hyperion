//! Drop / duplicate detection for the DualSense byte-7 frame counter.
//!
//! This is a thin re-export of [`hyperion_core::input::SeqTracker`]: the mod-256 delta logic
//! is pure numerics and already lives in `core`. The module is kept (rather than folded away)
//! for topology parity with `DESIGN.md` §6 and so the hot loop can refer to `seq::SeqTracker`
//! alongside its sibling `clock::DtTracker`.

#[doc(inline)]
pub use hyperion_core::input::SeqTracker;
