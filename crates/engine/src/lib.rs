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
pub mod handoff;
pub mod seq;
pub mod telemetry;

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
/// M1 is a headless vertical slice: no egui yet (that lands in M2). This entry point owns
/// the timer-resolution / HidHide / ViGEm lifecycle through [`supervisor`], spawns the hot
/// thread, and blocks until it joins. The body is an M1 skeleton — the control flow and
/// types are real, the device/driver I/O is filled in during hardware bring-up.
#[cfg(windows)]
pub fn run() -> Result<(), supervisor::EngineError> {
    supervisor::Supervisor::new()?.run()
}
