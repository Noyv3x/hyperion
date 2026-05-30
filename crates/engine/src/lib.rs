//! `engine` — the threading architecture and lock-free handoff around the pure
//! [`hyperion_core`] numerics.
//!
//! The engine owns the hot loop, dt measurement, telemetry, the single-writer config store,
//! and (on Windows) the supervisor lifecycle. Per `DESIGN.md` §6 the platform-independent
//! pieces — [`clock`], [`seq`], [`handoff`], [`telemetry`], [`config_store`] — are pure and
//! unit-tested on Linux CI. The Windows-only pieces (`hot`, `supervisor`, `run`) wire the real
//! HID/ViGEm/HidHide I/O and are type-checked on `windows-latest`.
//!
//! Concurrency primitives (see §6 "Lock-free hot-swap topology"):
//! * config GUI/supervisor → HOT: [`handoff::ConfigHandle`] (`Arc<ArcSwap<EngineConfig>>`),
//! * telemetry HOT → GUI: [`handoff::TelemetryTx`]/[`handoff::TelemetryRx`] (`triple-buffer`),
//! * commands GUI/supervisor → HOT: **two** SPSC [`handoff::CommandTx`] queues drained by the
//!   hot loop via [`handoff::CommandRx`] (`rtrb`).
//!
//! No `Mutex` lives on the hot path.

pub mod clock;
pub mod config_store;
pub mod control;
pub mod error;
pub mod handoff;
pub mod runtime;
pub mod seq;
pub mod telemetry;

#[doc(inline)]
pub use control::{ControlMsg, Stick};
#[doc(inline)]
pub use error::EngineError;
#[doc(inline)]
pub use runtime::Runtime;

/// Re-export of the pure [`hyperion_core`] config tree so binaries that depend only on
/// `engine` (e.g. the headless `app`) can build / load an [`EngineConfig`] without taking a
/// direct dependency on `hyperion-core`. This is a flat re-export — the types are owned by
/// core and unchanged.
pub mod config {
    pub use hyperion_core::config::{load_toml, to_toml, EngineConfig};
}

#[cfg(windows)]
pub mod hot;
#[cfg(windows)]
pub mod supervisor;
#[cfg(windows)]
mod win_io;

/// Assemble the supervisor + hot thread and run until shutdown (Windows only).
///
/// A thin headless entry point that owns the timer-resolution / HidHide / ViGEm lifecycle
/// through [`supervisor`], spawns the hot thread, and blocks until it joins. The GUI binary
/// (M2) uses [`Runtime`] instead, which returns immediately so egui can own the main thread;
/// this `run()` is kept for headless use and as the simplest possible integration smoke test.
#[cfg(windows)]
pub fn run() -> Result<(), EngineError> {
    supervisor::Supervisor::new()?.run()
}
