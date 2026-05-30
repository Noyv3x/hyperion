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
//! # Scope (M3 + M4)
//! M3 wired the **binding / profile / assignment / stick-and-trigger-settings** edits end to end
//! (blueprint §9). **M4** makes the macro / special-action / mouse / turbo / shift edits do real
//! work: [`ControlMsg::UpsertMacro`]/[`ControlMsg::DeleteMacro`],
//! [`ControlMsg::UpsertSpecialAction`]/[`ControlMsg::DeleteSpecialAction`],
//! [`ControlMsg::SetMouseSettings`], [`ControlMsg::SetBindingTurbo`], and
//! [`ControlMsg::SetShiftTrigger`] all mutate the active profile in [`crate::config_store`] now.
//! The gyro / auto-switch variants remain forward-compat (their consumers land in M5).

use std::sync::Arc;

use hyperion_core::config::{HidHideConfig, StickMode, ThreadConfig};
use hyperion_core::input::Control;
use hyperion_core::map::{
    BindTarget, MacroDef, MouseSettings, ShiftTrigger, SpecialAction, TurboCfg,
};
use hyperion_core::output::{KbmBatch, KbmEvent, PadTarget};
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
    /// Set (or clear, with `turbo == None`) the per-binding turbo / rapid-fire config for `control`
    /// in `profile` (inserts the slot if absent). **M4 consumer** in `apply` (`turbo_gate`); the
    /// `TurboCfg` is clamped to a sane period/duty by the writer's `clamped()` funnel.
    SetBindingTurbo {
        /// Profile id.
        profile: String,
        /// The control whose turbo is edited.
        control: Control,
        /// The turbo config (`None` clears turbo for the slot).
        turbo: Option<TurboCfg>,
    },

    // ---- Stick / trigger / mouse settings (blueprint §9) ----
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
    /// Replace a profile's mouse-from-stick settings (clamped on apply). **M4 consumer**: the
    /// resolved form feeds `apply`'s [`MouseAccumulator`](hyperion_core::mouse_accum::MouseAccumulator).
    SetMouseSettings {
        /// Profile id.
        profile: String,
        /// The new mouse settings (validated/clamped by the writer).
        settings: MouseSettings,
    },

    // ---- Macros / special actions (M4 consumers; §9) ----
    /// Insert or replace a macro definition in `profile` (its `id` is the key). **M4 consumer**:
    /// the injector thread plays its step list on a `Macro{start}` edge.
    UpsertMacro {
        /// Profile id.
        profile: String,
        /// The macro definition (its `id` is the key).
        def: MacroDef,
    },
    /// Delete a macro by id from `profile`. **M4 consumer**.
    DeleteMacro {
        /// Profile id.
        profile: String,
        /// Macro id to delete.
        id: u16,
    },
    /// Insert or replace a special action in `profile` (its `id` is the key). **M4 consumer**:
    /// referenced by `BindTarget::Special(id)`, fired through the control-plane side channel.
    UpsertSpecialAction {
        /// Profile id.
        profile: String,
        /// The special action (its `id` is the key).
        action: SpecialAction,
    },
    /// Delete a special action by id from `profile`. **M4 consumer**.
    DeleteSpecialAction {
        /// Profile id.
        profile: String,
        /// Special action id to delete.
        id: u16,
    },

    // ---- M5 surface: accepted-but-no-op (consumers land later; §9) ----
    /// Enable / disable foreground auto-profile-switching. **M5 consumer** (no-op until M5).
    SetAutoSwitchEnabled(bool),
}

/// An event the hot loop sends **out** to the control plane (blueprint §5/§12 M4: "Special edges to
/// a control-plane side channel"). Distinct from [`ControlMsg`] (which flows GUI → writer): this is
/// the hot thread → supervisor direction, carrying things that must run **off** the hot path.
///
/// `Copy`-of-`Arc` only (`Special` is a bare `u16`; `Macros` shares the resolved profile's macro
/// table by refcount), so producing one on the hot thread never allocates beyond the single
/// `crossbeam` enqueue.
#[derive(Clone, Debug)]
pub enum ControlPlaneEvent {
    /// A `BindTarget::Special(id)` rising edge fired in `apply()`. The control plane runs the
    /// matching [`SpecialAction`] (profile switch / launch / disconnect) entirely off the hot path;
    /// for M4 a stub handler logs/acks it.
    Special(u16),
    /// The resolved active profile's macro table, republished on start and on every profile change
    /// so the injector's `MacroPlayer` can play a `Macro{start}` edge by id. Arc-shared, so this is
    /// a refcount bump, never a deep copy of the step lists (blueprint §7.1).
    Macros(Arc<[MacroDef]>),
}

/// The hot side of the control-plane side channel: the producer the hot loop pushes
/// [`ControlPlaneEvent`]s into. A bounded `crossbeam` sender; `try_send` is non-blocking so a full
/// channel (a wedged control plane) never stalls the TIME_CRITICAL hot thread — the event is
/// dropped (special actions are idempotent on the next edge; the macro table is re-sent on the next
/// gate).
pub type ControlPlaneTx = crossbeam_channel::Sender<ControlPlaneEvent>;

/// The control-plane side of the channel: the consumer the supervisor drains off the hot path.
pub type ControlPlaneRx = crossbeam_channel::Receiver<ControlPlaneEvent>;

/// Forward every `Special(id)` rising edge in `batch` to the control plane, non-blocking.
///
/// Pure routing helper (Linux-testable): scans the already-produced [`KbmBatch`] for
/// [`KbmEvent::Special`] and `try_send`s each id on `tx`. Returns the number of specials forwarded.
/// A closed/full channel drops the event (the next edge re-sends) — the hot thread never blocks.
/// Called by the hot loop after `apply()`; the rest of the batch (key/mouse/macro edges) still goes
/// to the KBM injector ring (which ignores `Special` defensively).
#[inline]
pub fn forward_specials(batch: &KbmBatch, tx: &ControlPlaneTx) -> usize {
    let mut n = 0;
    for &ev in batch.as_slice() {
        if let KbmEvent::Special { id } = ev {
            // Drop-on-full / disconnected: special actions re-fire on the next rising edge, so a
            // missed one is self-healing and must never wedge the hot thread.
            let _ = tx.try_send(ControlPlaneEvent::Special(id));
            n += 1;
        }
    }
    n
}

#[cfg(test)]
mod tests {
    // `KbmBatch` / `KbmEvent` come in via `super::*` (the file-level import); only `MouseButton`
    // is additionally needed here.
    use super::*;
    use hyperion_core::output::MouseButton;

    #[test]
    fn forward_specials_extracts_only_special_ids() {
        let (tx, rx) = crossbeam_channel::unbounded();
        let mut batch = KbmBatch::new();
        batch.push(KbmEvent::MouseButton {
            btn: MouseButton::Left,
            down: true,
        });
        batch.push(KbmEvent::Special { id: 4 });
        batch.push(KbmEvent::Macro { id: 1, start: true });
        batch.push(KbmEvent::Special { id: 9 });

        let n = forward_specials(&batch, &tx);
        assert_eq!(n, 2, "two Special edges forwarded");

        let mut got = Vec::new();
        while let Ok(ev) = rx.try_recv() {
            match ev {
                ControlPlaneEvent::Special(id) => got.push(id),
                ControlPlaneEvent::Macros(_) => panic!("no macro event expected"),
            }
        }
        assert_eq!(got, vec![4, 9]);
    }

    #[test]
    fn forward_specials_on_empty_batch_sends_nothing() {
        let (tx, rx) = crossbeam_channel::unbounded::<ControlPlaneEvent>();
        let batch = KbmBatch::new();
        assert_eq!(forward_specials(&batch, &tx), 0);
        assert!(rx.try_recv().is_err());
    }

    #[test]
    fn forward_specials_full_channel_drops_without_blocking() {
        // A bounded, full channel must drop the event (never block the hot thread).
        let (tx, _rx) = crossbeam_channel::bounded::<ControlPlaneEvent>(1);
        tx.try_send(ControlPlaneEvent::Special(0)).unwrap(); // fill it
        let mut batch = KbmBatch::new();
        batch.push(KbmEvent::Special { id: 42 });
        // Returns the count it tried to forward; the send itself is dropped silently.
        assert_eq!(forward_specials(&batch, &tx), 1);
    }
}
