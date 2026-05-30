//! GUI → engine control messages (cross-platform).
//!
//! The egui tuning surface never mutates the shared config [`ArcSwap`] directly. Instead it
//! sends [`ControlMsg`] values down a `crossbeam_channel` to the engine's **single**
//! config-writer thread, which owns the [`crate::config_store::ConfigStore`], validates/clamps
//! each edit through [`hyperion_core::config`], and publishes one fresh immutable snapshot
//! (`DESIGN.md` §6 "Single writer"). The hot loop only ever does a wait-free `arc-swap` load
//! plus a cheap generation check; it is wholly decoupled from the GUI.
//!
//! These types are platform-independent (the channel + writer thread run on every target) so
//! they live outside the `cfg(windows)` runtime spawn and stay covered by the Linux unit tests.
//!
//! # M3 scope
//! M3 wires the **binding / profile / assignment / stick-and-trigger-settings** edits end to
//! end (blueprint §9). The macro / special-action / mouse / gyro / auto-switch variants exist so
//! the GUI and the writer compile against the full surface, but the M3 [`config_store`] arms for
//! them are **accepted-but-no-op** placeholders (their consumers land in M4/M5); they are marked
//! `TODO(M4)` / `TODO(M5)` in [`crate::config_store`].

use hyperion_core::config::{HidHideConfig, StickMode, ThreadConfig};
use hyperion_core::input::Control;
use hyperion_core::map::{BindTarget, MacroDef, ShiftTrigger, SpecialAction};
use hyperion_core::output::PadTarget;
use hyperion_core::rc::RcConfig;
use hyperion_core::stick::settings::StickSettings;
use hyperion_core::trigger::TriggerSettings;

/// Which analog stick a per-stick [`ControlMsg`] targets.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stick {
    /// Left stick (a profile's `ls`).
    Left,
    /// Right stick (a profile's `rs`).
    Right,
}

/// Which trigger a per-trigger [`ControlMsg`] targets.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Trigger {
    /// Left trigger (a profile's `l2`).
    Left,
    /// Right trigger (a profile's `r2`).
    Right,
}

/// One config edit from the GUI (or the auto-switch watcher), applied by the engine's single
/// config-writer thread.
///
/// Every variant names the entry it edits explicitly (device id, profile id, stick) so the
/// writer can target the right entry in [`hyperion_core::config::EngineConfig`] without the GUI
/// holding any reference into the live snapshot. Unknown / absent ids are a no-op (the writer
/// returns `false` from [`crate::config_store::ConfigStore::apply`] and nothing is republished).
///
/// **Not `PartialEq`:** the `SetRc` variant carries [`RcConfig`], which deliberately is not
/// `PartialEq` in `core` (the filter compares by serialized form), so neither is this enum.
#[derive(Clone, Debug)]
pub enum ControlMsg {
    // ---- Existing M2 variants (kept; `SetStickMode`/`SetRc` now target the active profile) ----
    /// Set the active profile's stick processing mode (RC on/off) for `device`'s assigned
    /// profile. Retargeted in M3 from the old `DeviceConfig` stick to the profile's `ls`/`rs`
    /// (blueprint §9): toggles `StickSettings::rc_mode_on`.
    SetStickMode {
        /// Device id whose assigned profile is edited.
        device: String,
        /// Which stick to edit.
        stick: Stick,
        /// The new mode (`Rc` => `rc_mode_on = true`, anything else => `false`).
        mode: StickMode,
    },
    /// Replace the active profile's stick RC parameters for `device` (clamped on apply).
    SetRc {
        /// Device id whose assigned profile is edited.
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

    // ---- Profile lifecycle (blueprint §9) ----
    /// Set the profile assigned to `device` (manual switch + the auto-switch watcher's path).
    /// Routed through `ConfigStore::apply` exactly like [`ControlMsg::SetActiveDevice`]; the hot
    /// loop picks the new profile up through the existing generation gate (§7.4).
    SetActiveProfile {
        /// Device id.
        device: String,
        /// Profile id (key into `EngineConfig::profiles`) to assign.
        name: String,
    },
    /// Create a new empty (all-passthrough) profile under id `name`. No-op if it already exists.
    CreateProfile {
        /// New profile id.
        name: String,
    },
    /// Duplicate the profile `src` under a new id `dst`. No-op if `src` is absent or `dst` exists.
    DuplicateProfile {
        /// Source profile id.
        src: String,
        /// Destination profile id.
        dst: String,
    },
    /// Rename profile `from` to `to`, fixing up any assignments that referenced it. No-op if
    /// `from` is absent or `to` already exists.
    RenameProfile {
        /// Current profile id.
        from: String,
        /// New profile id.
        to: String,
    },
    /// Delete the profile `name` (and drop any assignments that referenced it). No-op if absent.
    DeleteProfile {
        /// Profile id to delete.
        name: String,
    },
    /// Assign `profile` to `device` (the persisted `device -> profile` map). Distinct from
    /// [`ControlMsg::SetActiveProfile`] only in intent; both mutate `EngineConfig::assignments`.
    SetAssignment {
        /// Device id.
        device: String,
        /// Profile id to assign.
        profile: String,
    },
    /// Set a profile's virtual-pad output kind (X360 / DS4). Read at (re)plug time, never per
    /// report (a runtime change triggers a ViGEm replug — M5).
    SetOutputKind {
        /// Profile id.
        profile: String,
        /// The new output target.
        kind: PadTarget,
    },

    // ---- Bindings (blueprint §9; M3 resolves Passthrough/GamepadButton/Key in apply) ----
    /// Set the base binding for `control` in `profile` (inserts the slot if absent).
    SetBinding {
        /// Profile id.
        profile: String,
        /// The control whose base bind is set.
        control: Control,
        /// The new base binding target.
        bind: BindTarget,
    },
    /// Clear `control` back to the default identity passthrough in `profile` (removes the slot).
    ClearBinding {
        /// Profile id.
        profile: String,
        /// The control to clear.
        control: Control,
    },
    /// Set (or clear, with `trigger == None`) the per-control shift trigger + shift bind for
    /// `control` in `profile`. M4 consumer in `apply`, but the data lands now so M4 is additive.
    SetShiftTrigger {
        /// Profile id.
        profile: String,
        /// The control whose shift layer is edited.
        control: Control,
        /// The shift trigger (`None` clears the shift layer).
        trigger: Option<ShiftTrigger>,
        /// The binding applied while the shift trigger is active.
        bind: BindTarget,
    },

    // ---- Stick / trigger settings (blueprint §9) ----
    /// Replace one stick's full settings for `profile` (clamped on apply). Folds the RC config in.
    SetStickSettings {
        /// Profile id.
        profile: String,
        /// Which stick to edit.
        stick: Stick,
        /// The new stick settings (validated/clamped by the writer).
        settings: StickSettings,
    },
    /// Replace one trigger's full settings for `profile` (clamped on apply).
    SetTriggerSettings {
        /// Profile id.
        profile: String,
        /// Which trigger to edit.
        trigger: Trigger,
        /// The new trigger settings (validated/clamped by the writer).
        settings: TriggerSettings,
    },

    // ---- M4/M5 surface: accepted-but-no-op in M3 (consumers land later; §9) ----
    /// Insert or replace a macro definition in `profile`. **M4 consumer** (no-op in M3).
    UpsertMacro {
        /// Profile id.
        profile: String,
        /// The macro definition (its `id` is the key).
        def: MacroDef,
    },
    /// Delete a macro by id from `profile`. **M4 consumer** (no-op in M3).
    DeleteMacro {
        /// Profile id.
        profile: String,
        /// Macro id to delete.
        id: u16,
    },
    /// Insert or replace a special action in `profile`. **M4/M5 consumer** (no-op in M3).
    UpsertSpecialAction {
        /// Profile id.
        profile: String,
        /// The special action (its `id` is the key).
        action: SpecialAction,
    },
    /// Delete a special action by id from `profile`. **M4/M5 consumer** (no-op in M3).
    DeleteSpecialAction {
        /// Profile id.
        profile: String,
        /// Special action id to delete.
        id: u16,
    },
    /// Enable / disable foreground auto-profile-switching. **M5 consumer** (no-op in M3).
    SetAutoSwitchEnabled(bool),
}
