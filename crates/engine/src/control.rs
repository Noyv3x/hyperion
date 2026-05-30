//! GUI → engine control messages (cross-platform).
//!
//! The egui tuning surface (M2) never mutates the shared config [`ArcSwap`] directly. Instead
//! it sends [`ControlMsg`] values down a `crossbeam_channel` to the engine's **single**
//! config-writer thread, which owns the [`crate::config_store::ConfigStore`], validates/clamps
//! each edit through [`hyperion_core::config`], and publishes one fresh immutable snapshot
//! (`DESIGN.md` §6 "Single writer"). The hot loop only ever does a wait-free `arc-swap` load
//! plus a cheap generation check; it is wholly decoupled from the GUI.
//!
//! These types are platform-independent (the channel + writer thread run on every target) so
//! they live outside the `cfg(windows)` runtime spawn and stay covered by the Linux unit tests.

use hyperion_core::config::{HidHideConfig, StickMode, ThreadConfig};
use hyperion_core::rc::RcConfig;

/// Which analog stick a per-stick [`ControlMsg`] targets.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stick {
    /// Left stick (`DeviceConfig::ls`).
    Left,
    /// Right stick (`DeviceConfig::rs`).
    Right,
}

/// One config edit from the GUI, applied by the engine's single config-writer thread.
///
/// Every variant names the device (or stick) it edits explicitly so the writer can target the
/// right entry in [`hyperion_core::config::EngineConfig::devices`] without the GUI holding any
/// reference into the live snapshot. Unknown / absent devices are a no-op (the writer returns
/// `false` from [`crate::config_store::ConfigStore::apply`] and nothing is republished).
///
/// Not `PartialEq`: the `SetRc` variant carries [`RcConfig`], which deliberately is not
/// `PartialEq` in `core` (the filter compares by serialized form), so neither is this enum.
#[derive(Clone, Debug)]
pub enum ControlMsg {
    /// Set a stick's processing mode (RC on/off) for `device`.
    SetStickMode {
        /// Device id (key into `EngineConfig::devices`).
        device: String,
        /// Which stick to edit.
        stick: Stick,
        /// The new mode.
        mode: StickMode,
    },
    /// Replace a stick's RC parameters for `device` (clamped on apply).
    SetRc {
        /// Device id (key into `EngineConfig::devices`).
        device: String,
        /// Which stick to edit.
        stick: Stick,
        /// The new RC parameters (validated/clamped by the writer).
        rc: RcConfig,
    },
    /// Replace the global threading / scheduling / timing policy.
    SetThread(ThreadConfig),
    /// Replace the global HidHide cloaking policy.
    SetHidHide(HidHideConfig),
    /// Reload the whole config from the store's backing file (if a path is set).
    ReloadFromDisk,
    /// Persist the current snapshot to the store's backing file (if a path is set).
    SaveToDisk,
    /// Switch which device the engine drives (`EngineConfig::active_device`).
    SetActiveDevice(String),
}
