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
//!
//! # Scope (M5)
//! **M5** adds the gyro + auto-switch edits: [`ControlMsg::SetGyroSettings`] mutates a profile's
//! `gyro` (consumed by `apply`'s gyro accumulator), and
//! [`ControlMsg::SetAutoSwitchEnabled`]/[`ControlMsg::UpsertAutoSwitchRule`]/
//! [`ControlMsg::DeleteAutoSwitchRule`] mutate `EngineConfig::auto_switch` (consumed by the
//! `cfg(windows)` `ForegroundWatcher`, which sends [`ControlMsg::SetActiveProfile`] on a match).
//!
//! # Scope (M6)
//! **M6** adds the touchpad + profile-import edits: [`ControlMsg::SetTouchpadSettings`] mutates a
//! profile's `touchpad` (consumed by `apply`'s touch accumulator + the touch-region controls), and
//! [`ControlMsg::ImportProfile`] stores a standalone profile TOML under a destination id (via
//! [`hyperion_core::config::import_profile`]). The **two-stage / hip-fire trigger mode** is a field
//! of [`TriggerSettings`](hyperion_core::trigger::TriggerSettings) (`mode`/`soft_threshold`/
//! `hip_fire_us`), so it rides the existing [`ControlMsg::SetTriggerSettings`] arm with no new
//! variant. **Export** is the GUI calling [`hyperion_core::config::export_profile`] on a snapshot
//! profile directly (a pure read — no writer round-trip), so there is no `ExportProfile` message.

use std::sync::Arc;

use hyperion_core::config::{AutoSwitchRule, HidHideConfig, StickMode, ThreadConfig};
use hyperion_core::input::Control;
use hyperion_core::map::profile::TouchpadSettings;
use hyperion_core::map::{
    BindTarget, GyroSettings, MacroDef, MouseSettings, ShiftTrigger, SpecialAction, TurboCfg,
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
    /// Import a standalone profile from a TOML string (M6 profile import/export), storing it under
    /// id `name`. The TOML is parsed with [`hyperion_core::config::import_profile`] (the same
    /// defensive serde as the rest of the tree: missing keys default, unknown enum strings degrade);
    /// a structurally invalid TOML is a silent no-op (the writer's no-change compare returns
    /// `false`). The imported profile's `name` field is normalized to `name` so the on-disk key and
    /// the profile name agree. Overwrites an existing profile with the same id.
    ImportProfile {
        /// Destination profile id (also written into the imported profile's `name`).
        name: String,
        /// The standalone profile TOML (as produced by `export_profile`).
        toml: String,
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
    /// Replace a profile's touchpad→mouse / touch-as-buttons settings (clamped on apply). **M6
    /// consumer**: the resolved form feeds `apply`'s touch
    /// [`MouseAccumulator`](hyperion_core::mouse_accum::MouseAccumulator) via `touch_step` when a
    /// control is bound to `MouseMove(Touchpad)` and `as_mouse` is set, and gates the touch-region
    /// controls (`TouchLeft/Right/Upper/Multi`) when `as_buttons` is set.
    SetTouchpadSettings {
        /// Profile id.
        profile: String,
        /// The new touchpad settings (validated/clamped by the writer).
        settings: TouchpadSettings,
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

    // ---- Gyro settings (M5 consumer; §9) ----
    /// Replace a profile's gyro→mouse settings (clamped on apply). **M5 consumer**: the resolved
    /// form feeds `apply`'s gyro [`MouseAccumulator`](hyperion_core::mouse_accum::MouseAccumulator)
    /// via `gyro_velocity_step` when [`GyroMode`](hyperion_core::map::profile::GyroMode) is active.
    SetGyroSettings {
        /// Profile id.
        profile: String,
        /// The new gyro settings (validated/clamped by the writer).
        settings: GyroSettings,
    },

    // ---- Auto-profile-switch (M5 consumer: the `ForegroundWatcher`; §7.4, §9) ----
    /// Enable / disable foreground auto-profile-switching (`EngineConfig::auto_switch.enabled`).
    /// **M5 consumer**: the `ForegroundWatcher` only polls + matches while this is `true`.
    SetAutoSwitchEnabled(bool),
    /// Insert or replace an auto-switch rule (keyed by its full `(device, exe_substr, title_substr)`
    /// match tuple, so re-pointing an existing rule's `profile` is an in-place update). Appends a new
    /// rule when no rule with that match tuple exists. **M5 consumer**: `match_rules` walks the list
    /// in order (first match wins, §7.4).
    UpsertAutoSwitchRule {
        /// The rule to insert or replace (its match tuple is the logical key).
        rule: AutoSwitchRule,
    },
    /// Delete the auto-switch rule whose `(device, exe_substr, title_substr)` match tuple equals the
    /// given one. No-op if no such rule exists. **M5 consumer**.
    DeleteAutoSwitchRule {
        /// Device id of the rule to delete (empty == "any device").
        device: String,
        /// Exe-substring match key of the rule to delete.
        exe_substr: String,
        /// Title-substring match key of the rule to delete.
        title_substr: String,
    },
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

/// The concrete effect a `BindTarget::Special(id)` edge resolves to (M7), decoded purely from the
/// active profile's [`SpecialAction`] so the OS-side dispatch (in the supervisor's control-plane
/// drain thread) is a thin shell. Kept Linux-testable: the id→effect resolution is the pure
/// [`special_action_effect`]; the side-effecting half (route a `ControlMsg`, spawn a process) lives
/// in the `cfg(windows)` supervisor.
///
/// ## Encoding (no `core` change)
///
/// [`SpecialAction`](hyperion_core::map::SpecialAction) is `{ id, name }` with no structured action
/// field, and `core` is out of scope here, so the action is carried in the `name` string via a
/// documented `kind:payload` prefix — the same lightweight convention a GUI special-action editor
/// would round-trip:
///
/// * `profile:<profile_id>` → [`SpecialEffect::SwitchProfile`] of the active device to `<profile_id>`
///   (only when that profile exists and differs from the current assignment).
/// * `launch:<program> [args…]` → [`SpecialEffect::LaunchProgram`] (whitespace-split; the first
///   token is the program, the rest are arguments).
///
/// Anything else (an unknown prefix, an empty payload, a switch to a missing/already-active profile)
/// resolves to `None`, so an unconfigured / display-only special is inert.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SpecialEffect {
    /// Switch the active device's assigned profile through the single-writer `ControlMsg` path.
    SwitchProfile {
        /// Device id whose assignment is changed (the config's `active_device`).
        device: String,
        /// Profile id to assign.
        name: String,
    },
    /// Launch an external program off the hot path via `std::process::Command`.
    LaunchProgram {
        /// Program / executable path.
        program: String,
        /// Whitespace-split arguments (possibly empty).
        args: Vec<String>,
    },
}

/// Resolve a `BindTarget::Special(id)` edge to its concrete [`SpecialEffect`] against the live
/// config snapshot (M7). **Pure + Linux-testable** — the whole decision the `cfg(windows)`
/// control-plane drain thread makes, factored out of the OS dispatch.
///
/// Looks `id` up in the active device's assigned profile's `specials` (falling back to scanning all
/// profiles' `specials` only if the active profile has none with that id, so a special defined on a
/// shared profile still resolves), then decodes the [`SpecialAction::name`] via the `kind:payload`
/// convention documented on [`SpecialEffect`]. Returns `None` for an unknown id, an unrecognized /
/// empty action, or a profile-switch that would be a no-op (missing profile, or already the active
/// assignment) — so a redundant special never churns the config generation.
#[must_use]
pub fn special_action_effect(
    cfg: &hyperion_core::config::EngineConfig,
    id: u16,
) -> Option<SpecialEffect> {
    let action = find_special(cfg, id)?;
    let (kind, payload) = action.name.split_once(':')?;
    match kind.trim() {
        "profile" => {
            let target = payload.trim();
            if target.is_empty() || !cfg.profiles.contains_key(target) {
                return None;
            }
            let device = cfg.active_device.as_str();
            // Skip a switch to the already-assigned profile (no needless generation bump).
            if cfg.assignments.get(device).map(String::as_str) == Some(target) {
                return None;
            }
            Some(SpecialEffect::SwitchProfile {
                device: device.to_string(),
                name: target.to_string(),
            })
        }
        "launch" => {
            let mut parts = payload.split_whitespace();
            let program = parts.next()?.to_string();
            let args = parts.map(str::to_string).collect();
            Some(SpecialEffect::LaunchProgram { program, args })
        }
        _ => None, // unknown action kind: ignore.
    }
}

/// Find the [`SpecialAction`] with `id`, preferring the active device's assigned profile and
/// falling back to any profile that defines it (so a special on a shared/non-active profile still
/// resolves). Returns the first match.
fn find_special(cfg: &hyperion_core::config::EngineConfig, id: u16) -> Option<&SpecialAction> {
    // Active device's assigned profile first.
    if let Some(active) = cfg
        .assignments
        .get(&cfg.active_device)
        .and_then(|pid| cfg.profiles.get(pid))
    {
        if let Some(a) = active.specials.iter().find(|s| s.id == id) {
            return Some(a);
        }
    }
    // Fall back to any profile defining the id.
    cfg.profiles
        .values()
        .find_map(|p| p.specials.iter().find(|s| s.id == id))
}

/// Decide the auto-profile-switch edit for the current foreground app (blueprint §7.4, §12 M5).
///
/// Pure + Linux-testable: this is the whole decision the `cfg(windows)` `ForegroundWatcher` makes
/// each poll, factored out of the OS read so it is unit-tested without Windows. Given the live
/// config snapshot and the foreground executable path + window title, it:
///
/// 1. does nothing unless `auto_switch.enabled` (the master gate),
/// 2. runs the **pure** [`match_rules_for_device`](hyperion_core::autoswitch::match_rules_for_device)
///    against the active device's rules (first-match-wins, device-scoped, §7.4),
/// 3. returns a [`ControlMsg::SetActiveProfile`] **only** when a rule matches a profile that both
///    exists and differs from the device's current assignment — so an unchanged foreground (or a
///    rule pointing at the already-active profile) sends nothing and the single writer never bumps
///    the generation needlessly.
///
/// The watcher feeds the returned message into the **same** single-writer `ControlMsg` channel the
/// GUI uses (never the hot path); the hot loop picks the switch up through the existing generation
/// gate. A `None` keeps the current profile (the watcher just doesn't switch).
#[must_use]
pub fn auto_switch_decision(
    cfg: &hyperion_core::config::EngineConfig,
    exe: &str,
    title: &str,
) -> Option<ControlMsg> {
    if !cfg.auto_switch.enabled {
        return None;
    }
    let device = cfg.active_device.as_str();
    let matched = hyperion_core::autoswitch::match_rules_for_device(
        &cfg.auto_switch.rules,
        device,
        exe,
        title,
    )?;
    // Only switch to a real profile, and only when it differs from what the device already runs
    // (so a steady foreground / a redundant rule never churns the generation).
    if !cfg.profiles.contains_key(matched) {
        return None;
    }
    if cfg.assignments.get(device).map(String::as_str) == Some(matched) {
        return None;
    }
    Some(ControlMsg::SetActiveProfile {
        device: device.to_string(),
        name: matched.to_string(),
    })
}

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

    // ----------------------------- M5: foreground auto-switch decision ---------------------------

    use hyperion_core::config::{AutoSwitchConfig, AutoSwitchRule, EngineConfig};
    use hyperion_core::map::Profile;

    /// A config with device `"dev"` assigned to `"default"`, plus a second profile `"fps"`, and an
    /// auto-switch rule list. `enabled` toggles the master gate.
    fn cfg_with_rules(enabled: bool, rules: Vec<AutoSwitchRule>) -> EngineConfig {
        let mut cfg = EngineConfig {
            active_device: "dev".to_string(),
            auto_switch: AutoSwitchConfig {
                enabled,
                poll_hz: 4,
                rules,
            },
            ..EngineConfig::default()
        };
        let profiles = Arc::make_mut(&mut cfg.profiles);
        profiles.insert("default".to_string(), Profile::default());
        profiles.insert("fps".to_string(), Profile::default());
        cfg.assignments
            .insert("dev".to_string(), "default".to_string());
        cfg
    }

    fn rule(exe: &str, profile: &str) -> AutoSwitchRule {
        AutoSwitchRule {
            device: String::new(),
            exe_substr: exe.to_string(),
            title_substr: String::new(),
            profile: profile.to_string(),
        }
    }

    #[test]
    fn auto_switch_decision_matches_rule_and_switches() {
        let cfg = cfg_with_rules(true, vec![rule("valorant", "fps")]);
        let msg = auto_switch_decision(&cfg, "C:/Riot/valorant.exe", "VALORANT");
        match msg {
            Some(ControlMsg::SetActiveProfile { device, name }) => {
                assert_eq!(device, "dev");
                assert_eq!(
                    name, "fps",
                    "foreground exe match -> switch to the rule's profile"
                );
            }
            other => panic!("expected SetActiveProfile, got {other:?}"),
        }
    }

    #[test]
    fn auto_switch_decision_disabled_is_none() {
        // The master gate off => no switch even with a matching rule.
        let cfg = cfg_with_rules(false, vec![rule("valorant", "fps")]);
        assert!(auto_switch_decision(&cfg, "valorant.exe", "VALORANT").is_none());
    }

    #[test]
    fn auto_switch_decision_no_match_is_none() {
        let cfg = cfg_with_rules(true, vec![rule("valorant", "fps")]);
        assert!(
            auto_switch_decision(&cfg, "C:/desktop/explorer.exe", "Desktop").is_none(),
            "a non-matching foreground keeps the current profile"
        );
    }

    #[test]
    fn auto_switch_decision_already_active_is_none() {
        // A rule that matches the profile the device ALREADY runs sends nothing (no churn).
        let cfg = cfg_with_rules(true, vec![rule("valorant", "default")]);
        assert!(
            auto_switch_decision(&cfg, "valorant.exe", "x").is_none(),
            "matching the already-assigned profile is a no-op"
        );
    }

    #[test]
    fn auto_switch_decision_unknown_profile_is_none() {
        // A rule pointing at a non-existent profile is ignored (never assign a missing id).
        let cfg = cfg_with_rules(true, vec![rule("valorant", "ghost")]);
        assert!(auto_switch_decision(&cfg, "valorant.exe", "x").is_none());
    }

    #[test]
    fn auto_switch_decision_first_match_wins() {
        // Two matching rules; the first in the list wins (mirrors match_rules ordering).
        let cfg = cfg_with_rules(true, vec![rule("game", "fps"), rule("game", "default")]);
        let msg = auto_switch_decision(&cfg, "mygame.exe", "x");
        assert!(matches!(
            msg,
            Some(ControlMsg::SetActiveProfile { name, .. }) if name == "fps"
        ));
    }

    #[test]
    fn auto_switch_decision_respects_device_scope() {
        // A rule scoped to a DIFFERENT device must not fire for the active device.
        let mut r = rule("game", "fps");
        r.device = "other".to_string();
        let cfg = cfg_with_rules(true, vec![r]);
        assert!(auto_switch_decision(&cfg, "mygame.exe", "x").is_none());
    }

    // ----------------------------- M7: special-action id -> effect mapping ------------------------
    // `SpecialAction` is already in scope via the file-level import (`super::*`).

    /// `cfg_with_rules` (device `"dev"` -> `"default"`, plus `"fps"`) with the given specials added
    /// to the **active** (`"default"`) profile.
    fn cfg_with_specials(specials: Vec<SpecialAction>) -> EngineConfig {
        let mut cfg = cfg_with_rules(false, vec![]);
        let profiles = Arc::make_mut(&mut cfg.profiles);
        profiles.get_mut("default").unwrap().specials = specials;
        cfg
    }

    fn special(id: u16, name: &str) -> SpecialAction {
        SpecialAction {
            id,
            name: name.to_string(),
        }
    }

    #[test]
    fn special_profile_switch_resolves_to_switch_effect() {
        // `profile:fps` on the active profile -> switch the active device to `fps`.
        let cfg = cfg_with_specials(vec![special(1, "profile:fps")]);
        assert_eq!(
            special_action_effect(&cfg, 1),
            Some(SpecialEffect::SwitchProfile {
                device: "dev".to_string(),
                name: "fps".to_string(),
            })
        );
    }

    #[test]
    fn special_profile_switch_to_active_or_missing_is_none() {
        // Switching to the ALREADY-assigned profile is a no-op (no generation churn).
        let cfg = cfg_with_specials(vec![special(1, "profile:default")]);
        assert!(special_action_effect(&cfg, 1).is_none());
        // Switching to a non-existent profile resolves to nothing (never assign a missing id).
        let cfg = cfg_with_specials(vec![special(2, "profile:ghost")]);
        assert!(special_action_effect(&cfg, 2).is_none());
    }

    #[test]
    fn special_launch_resolves_program_and_args() {
        let cfg = cfg_with_specials(vec![special(7, "launch:notepad.exe a.txt b.txt")]);
        assert_eq!(
            special_action_effect(&cfg, 7),
            Some(SpecialEffect::LaunchProgram {
                program: "notepad.exe".to_string(),
                args: vec!["a.txt".to_string(), "b.txt".to_string()],
            })
        );
        // A launch with no args yields an empty arg list.
        let cfg = cfg_with_specials(vec![special(8, "launch:calc.exe")]);
        assert_eq!(
            special_action_effect(&cfg, 8),
            Some(SpecialEffect::LaunchProgram {
                program: "calc.exe".to_string(),
                args: vec![],
            })
        );
    }

    #[test]
    fn special_unknown_id_or_kind_or_empty_is_none() {
        let cfg = cfg_with_specials(vec![special(1, "profile:fps")]);
        // Unknown id: not in the table.
        assert!(special_action_effect(&cfg, 99).is_none());
        // Unknown kind prefix.
        let cfg = cfg_with_specials(vec![special(1, "frobnicate:x")]);
        assert!(special_action_effect(&cfg, 1).is_none());
        // A display-only name with no `kind:payload` separator.
        let cfg = cfg_with_specials(vec![special(1, "My Special")]);
        assert!(special_action_effect(&cfg, 1).is_none());
        // Empty launch payload yields no program.
        let cfg = cfg_with_specials(vec![special(1, "launch:")]);
        assert!(special_action_effect(&cfg, 1).is_none());
        // Empty profile payload.
        let cfg = cfg_with_specials(vec![special(1, "profile:")]);
        assert!(special_action_effect(&cfg, 1).is_none());
    }

    #[test]
    fn special_resolves_from_any_profile_when_active_lacks_it() {
        // A special defined only on a NON-active profile (`fps`) still resolves (fallback scan).
        let mut cfg = cfg_with_rules(false, vec![]);
        let profiles = Arc::make_mut(&mut cfg.profiles);
        profiles.get_mut("fps").unwrap().specials = vec![special(5, "launch:tool.exe")];
        assert_eq!(
            special_action_effect(&cfg, 5),
            Some(SpecialEffect::LaunchProgram {
                program: "tool.exe".to_string(),
                args: vec![],
            })
        );
    }

    #[test]
    fn special_active_profile_takes_precedence_on_id_collision() {
        // Both the active (`default`) and a non-active (`fps`) profile define id 3; the active
        // profile's action wins (so a per-profile special overrides a shared one).
        let mut cfg = cfg_with_rules(false, vec![]);
        let profiles = Arc::make_mut(&mut cfg.profiles);
        profiles.get_mut("default").unwrap().specials = vec![special(3, "profile:fps")];
        profiles.get_mut("fps").unwrap().specials = vec![special(3, "launch:wrong.exe")];
        assert_eq!(
            special_action_effect(&cfg, 3),
            Some(SpecialEffect::SwitchProfile {
                device: "dev".to_string(),
                name: "fps".to_string(),
            }),
            "active profile's special wins on an id collision"
        );
    }
}
