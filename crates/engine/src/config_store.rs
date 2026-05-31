//! The single writer of the config [`ArcSwap`](arc_swap::ArcSwap).
//!
//! GUI edits, the supervisor, and (on Windows) the file-watch all converge here rather than
//! mutating the shared snapshot directly (`DESIGN.md` §6 "Single writer"). [`ConfigStore`]
//! validates/clamps every incoming config through [`hyperion_core::config`], publishes one
//! fresh immutable snapshot via `store()`, and bumps a generation counter so the hot loop can
//! cheaply detect "did config change?" with a single atomic load instead of diffing fields.
//!
//! The core publish/apply path needs no filesystem and is fully Linux-testable; TOML
//! persistence is layered above it via the optional [`config path`](ConfigStore::with_path)
//! the store holds (used only by [`ControlMsg::SaveToDisk`] / [`ControlMsg::ReloadFromDisk`]).
//!
//! In the assembled engine a **single** config-writer thread owns one `ConfigStore` and drains
//! the GUI's `crossbeam_channel::Receiver<ControlMsg>`, calling [`ConfigStore::apply`] per
//! message. That keeps the `ArcSwap` strictly single-writer even though many threads (GUI,
//! supervisor, tray) may *send* edits.

use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use arc_swap::ArcSwap;
use hyperion_core::config::{load_toml, to_toml, AutoSwitchRule, EngineConfig};
use hyperion_core::map::{BindingSlot, Profile};

use crate::control::{ControlMsg, Stick, Trigger};
use crate::handoff::ConfigHandle;

/// A monotonically increasing config version. The hot loop caches the last value it applied
/// and only re-reads the snapshot when this changes.
pub type Generation = u64;

/// Owns the write side of the config [`ConfigHandle`]. Construct **one** of these per engine;
/// every config change (GUI, supervisor, file-watch) routes through [`ConfigStore::apply`].
pub struct ConfigStore {
    handle: ConfigHandle,
    generation: Arc<AtomicU64>,
    /// Optional backing file for [`ControlMsg::SaveToDisk`] / [`ControlMsg::ReloadFromDisk`].
    /// `None` => those messages are no-ops (return `false`).
    config_path: Option<PathBuf>,
}

impl ConfigStore {
    /// Create a store seeded with `initial`, validated/clamped before its first publish, and
    /// generation `1` (the hot loop starts its cache at `0`, so it applies the seed once). No
    /// backing file is attached; use [`ConfigStore::with_path`] to enable disk persistence.
    pub fn new(initial: EngineConfig) -> Self {
        let validated = validate(initial);
        let handle = Arc::new(ArcSwap::from_pointee(validated));
        Self {
            handle,
            generation: Arc::new(AtomicU64::new(1)),
            config_path: None,
        }
    }

    /// Wrap an existing [`ConfigHandle`] (e.g. the one returned by
    /// [`crate::handoff::build_links`]) so the store and the hot loop publish/read the *same*
    /// `ArcSwap`. The handle's current contents are taken as already-validated.
    pub fn from_handle(handle: ConfigHandle) -> Self {
        Self {
            handle,
            generation: Arc::new(AtomicU64::new(1)),
            config_path: None,
        }
    }

    /// Attach (or replace) the optional backing file used by [`ControlMsg::SaveToDisk`] and
    /// [`ControlMsg::ReloadFromDisk`]. Builder-style so it composes with the constructors.
    #[must_use]
    pub fn with_path(mut self, path: Option<PathBuf>) -> Self {
        self.config_path = path;
        self
    }

    /// The backing config file path, if one is attached.
    pub fn config_path(&self) -> Option<&PathBuf> {
        self.config_path.as_ref()
    }

    /// The shared config handle to clone into the hot loop and other readers.
    pub fn handle(&self) -> ConfigHandle {
        self.handle.clone()
    }

    /// The shared generation counter to clone into the hot loop for cheap change detection.
    pub fn generation_counter(&self) -> Arc<AtomicU64> {
        self.generation.clone()
    }

    /// The current generation (the value the hot loop compares against its cache).
    #[inline]
    pub fn generation(&self) -> Generation {
        self.generation.load(Ordering::Acquire)
    }

    /// Load the current immutable snapshot (wait-free `arc-swap` load).
    #[inline]
    pub fn snapshot(&self) -> Arc<EngineConfig> {
        self.handle.load_full()
    }

    /// Validate/clamp `next` through `core`, publish it as the new immutable snapshot, and
    /// bump the generation. Returns the new generation.
    ///
    /// This is a *whole-snapshot* `store()` (wait-free for readers) plus a `Release` increment
    /// of the generation so a hot loop that observes the new generation also observes the new
    /// snapshot. It does **not** diff — callers that need "did anything change?" semantics use
    /// [`ConfigStore::apply`] instead.
    pub fn publish(&self, next: EngineConfig) -> Generation {
        let validated = validate(next);
        self.handle.store(Arc::new(validated));
        // Release so the snapshot store is visible before the generation bump the hot loop
        // keys off of.
        self.generation.fetch_add(1, Ordering::Release) + 1
    }

    /// Read-modify-publish helper: load the current snapshot, let `edit` mutate a clone, then
    /// validate + publish it. Convenience for field-level edits; always republishes.
    pub fn update<F: FnOnce(&mut EngineConfig)>(&self, edit: F) -> Generation {
        let mut next = (*self.handle.load_full()).clone();
        edit(&mut next);
        self.publish(next)
    }

    /// Apply one GUI [`ControlMsg`] as the **sole** writer of the `ArcSwap`.
    ///
    /// Mutates a *clone* of the current snapshot, clamps it through
    /// [`hyperion_core::config`], and — only if the edit actually changed the config —
    /// publishes the new snapshot and bumps the generation. Returns `true` if a new snapshot
    /// was published, `false` for a no-op (unknown device, value already equal, or a
    /// disk message with no backing path).
    ///
    /// Alloc-light: the only heap work is the snapshot clone every config publish already
    /// requires; change detection compares the clamped TOML serialization (off the hot path,
    /// and `EngineConfig` deliberately is not `PartialEq` because `RcConfig` is not).
    pub fn apply(&self, msg: &ControlMsg) -> bool {
        match msg {
            ControlMsg::SaveToDisk => return self.save_to_disk(),
            ControlMsg::ReloadFromDisk => return self.reload_from_disk(),
            _ => {}
        }

        let current = self.handle.load_full();
        let mut next = (*current).clone();
        edit_in_place(&mut next, msg);
        let next = next.clamped();

        // Republish only on a real change. `EngineConfig` is not `PartialEq` (the core
        // contract: `RcConfig` carries no `PartialEq`), so compare the canonical serialization
        // of the already-clamped old/new snapshots. This is the config-edit path, never the
        // hot loop, so the TOML round-trip cost is irrelevant.
        if to_toml(&next) == to_toml(&current) {
            return false;
        }

        self.handle.store(Arc::new(next));
        self.generation.fetch_add(1, Ordering::Release);
        true
    }

    /// Persist the current snapshot to the backing file. No path => `false`.
    fn save_to_disk(&self) -> bool {
        let Some(path) = self.config_path.as_ref() else {
            return false;
        };
        let text = to_toml(&self.handle.load_full());
        std::fs::write(path, text).is_ok()
    }

    /// Replace the snapshot with the (clamped) contents of the backing file. No path, a read
    /// error, or a parse error => `false` and the live snapshot is untouched.
    fn reload_from_disk(&self) -> bool {
        let Some(path) = self.config_path.as_ref() else {
            return false;
        };
        let Ok(text) = std::fs::read_to_string(path) else {
            return false;
        };
        let Ok(loaded) = load_toml(&text) else {
            return false;
        };
        let next = loaded.clamped();
        if to_toml(&next) == to_toml(&self.handle.load_full()) {
            return false;
        }
        self.handle.store(Arc::new(next));
        self.generation.fetch_add(1, Ordering::Release);
        true
    }
}

/// Apply a non-disk [`ControlMsg`] to a mutable snapshot clone. Unknown devices / profiles are
/// silently skipped (the caller's no-change TOML compare then returns `false`), identical to the
/// M2 behavior for an absent device.
///
/// All profile edits mutate `cfg.profiles` through [`Arc::make_mut`] (blueprint §7.1 keeps
/// `profiles` an `Arc<BTreeMap>` so a per-generation `EngineConfig::clone` is a refcount bump,
/// not a deep tree copy — the cost is paid once, here, on the cold config-writer thread).
fn edit_in_place(cfg: &mut EngineConfig, msg: &ControlMsg) {
    match msg {
        // ---- Global / device-level (unchanged topology) ----
        ControlMsg::SetThread(thread) => cfg.thread = thread.clone(),
        ControlMsg::SetHidHide(hidhide) => cfg.hidhide = hidhide.clone(),
        ControlMsg::SetActiveDevice(id) => cfg.active_device = id.clone(),

        // ---- Stick mode / RC now target the device's assigned profile (§9) ----
        ControlMsg::SetStickMode {
            device,
            stick,
            mode,
        } => {
            if let Some(p) = profile_for_device_mut(cfg, device) {
                // `Rc` selects the RC stage; any other mode turns it off (blueprint §9 shim).
                stick_settings_mut(p, *stick).rc_mode_on =
                    *mode == hyperion_core::config::StickMode::Rc;
            }
        }
        ControlMsg::SetRc { device, stick, rc } => {
            if let Some(p) = profile_for_device_mut(cfg, device) {
                stick_settings_mut(p, *stick).rc = *rc;
            }
        }

        // ---- Profile lifecycle ----
        // `SetActiveProfile`/`SetAssignment` both assign `device -> profile`; assign only a
        // profile that exists (an unknown id stays a silent no-op → `false`).
        ControlMsg::SetActiveProfile {
            device,
            name: profile,
        }
        | ControlMsg::SetAssignment { device, profile } => {
            if cfg.profiles.contains_key(profile) {
                cfg.assignments.insert(device.clone(), profile.clone());
            }
        }
        ControlMsg::CreateProfile { name } => {
            let profiles = Arc::make_mut(&mut cfg.profiles);
            profiles.entry(name.clone()).or_insert_with(|| Profile {
                name: name.clone(),
                ..Profile::default()
            });
        }
        ControlMsg::DuplicateProfile { src, dst } => {
            // Only if `src` exists and `dst` is free.
            if let Some(src_profile) = cfg.profiles.get(src).cloned() {
                let profiles = Arc::make_mut(&mut cfg.profiles);
                if !profiles.contains_key(dst) {
                    let mut copy = src_profile;
                    copy.name = dst.clone();
                    profiles.insert(dst.clone(), copy);
                }
            }
        }
        ControlMsg::RenameProfile { from, to } => {
            // Only if `from` exists and `to` is free.
            if cfg.profiles.contains_key(from) && !cfg.profiles.contains_key(to) {
                let profiles = Arc::make_mut(&mut cfg.profiles);
                if let Some(mut p) = profiles.remove(from) {
                    p.name = to.clone();
                    profiles.insert(to.clone(), p);
                }
                // Re-point any assignment that referenced the old id.
                for assigned in cfg.assignments.values_mut() {
                    if assigned == from {
                        *assigned = to.clone();
                    }
                }
            }
        }
        ControlMsg::DeleteProfile { name } => {
            if cfg.profiles.contains_key(name) {
                Arc::make_mut(&mut cfg.profiles).remove(name);
                // Drop any assignment that pointed at the deleted profile.
                cfg.assignments.retain(|_, pid| pid != name);
            }
        }
        ControlMsg::SetOutputKind { profile, kind } => {
            if let Some(p) = profile_mut(cfg, profile) {
                p.output_kind = *kind;
            }
        }

        // ---- Bindings ----
        ControlMsg::SetBinding {
            profile,
            control,
            bind,
        } => {
            if let Some(p) = profile_mut(cfg, profile) {
                p.bindings
                    .entry(*control)
                    .or_insert_with(BindingSlot::default)
                    .bind = *bind;
            }
        }
        ControlMsg::ClearBinding { profile, control } => {
            if let Some(p) = profile_mut(cfg, profile) {
                p.bindings.remove(control);
            }
        }
        ControlMsg::SetShiftTrigger {
            profile,
            control,
            trigger,
            bind,
        } => {
            if let Some(p) = profile_mut(cfg, profile) {
                let slot = p
                    .bindings
                    .entry(*control)
                    .or_insert_with(BindingSlot::default);
                slot.shift_trigger = *trigger;
                slot.shift_bind = *bind;
            }
        }
        ControlMsg::SetBindingTurbo {
            profile,
            control,
            turbo,
        } => {
            if let Some(p) = profile_mut(cfg, profile) {
                p.bindings
                    .entry(*control)
                    .or_insert_with(BindingSlot::default)
                    .turbo = *turbo;
            }
        }

        // ---- Stick / trigger / mouse settings ----
        ControlMsg::SetStickSettings {
            profile,
            stick,
            settings,
        } => {
            if let Some(p) = profile_mut(cfg, profile) {
                *stick_settings_mut(p, *stick) = *settings;
            }
        }
        ControlMsg::SetTriggerSettings {
            profile,
            trigger,
            settings,
        } => {
            if let Some(p) = profile_mut(cfg, profile) {
                match trigger {
                    Trigger::Left => p.l2 = *settings,
                    Trigger::Right => p.r2 = *settings,
                }
            }
        }
        ControlMsg::SetMouseSettings { profile, settings } => {
            if let Some(p) = profile_mut(cfg, profile) {
                p.mouse = *settings;
            }
        }

        // ---- Macros / special actions (M4): mutate the active profile's id-keyed Vecs ----
        // The `id` field is the logical key; an upsert replaces the entry with the matching id (or
        // appends a new one), a delete drops it. Kept as sorted-by-id Vecs so the on-disk form is
        // stable and a re-serialize is deterministic (the no-change TOML compare then works).
        ControlMsg::UpsertMacro { profile, def } => {
            if let Some(p) = profile_mut(cfg, profile) {
                upsert_by_id(&mut p.macros, def.clone(), |m| m.id, def.id);
            }
        }
        ControlMsg::DeleteMacro { profile, id } => {
            if let Some(p) = profile_mut(cfg, profile) {
                p.macros.retain(|m| m.id != *id);
            }
        }
        ControlMsg::UpsertSpecialAction { profile, action } => {
            if let Some(p) = profile_mut(cfg, profile) {
                upsert_by_id(&mut p.specials, action.clone(), |a| a.id, action.id);
            }
        }
        ControlMsg::DeleteSpecialAction { profile, id } => {
            if let Some(p) = profile_mut(cfg, profile) {
                p.specials.retain(|a| a.id != *id);
            }
        }

        // ---- Gyro settings (M5): mutate the active profile's gyro→mouse settings ----
        ControlMsg::SetGyroSettings { profile, settings } => {
            if let Some(p) = profile_mut(cfg, profile) {
                p.gyro = *settings;
            }
        }

        // ---- Auto-profile-switch (M5): mutate `cfg.auto_switch` (off the hot path) ----
        ControlMsg::SetAutoSwitchEnabled(enabled) => {
            cfg.auto_switch.enabled = *enabled;
        }
        ControlMsg::UpsertAutoSwitchRule { rule } => {
            upsert_auto_switch_rule(&mut cfg.auto_switch.rules, rule.clone());
        }
        ControlMsg::DeleteAutoSwitchRule {
            device,
            exe_substr,
            title_substr,
        } => {
            cfg.auto_switch.rules.retain(|r| {
                !(r.device == *device
                    && r.exe_substr == *exe_substr
                    && r.title_substr == *title_substr)
            });
        }

        // Disk messages are handled before this function is reached.
        ControlMsg::SaveToDisk | ControlMsg::ReloadFromDisk => {}
    }
}

/// Insert-or-replace `item` in an id-keyed `Vec`. If an element with `id` already exists it is
/// replaced in place; otherwise the item is inserted at the first position whose id is greater, so
/// a Vec that starts sorted stays sorted (a stable on-disk form). Correct regardless of the input
/// ordering (a linear find, not a binary search — these Vecs are tiny and the path is cold).
///
/// `key` extracts an element's id; `id` is `item`'s id (passed separately so the closure need not
/// borrow `item`, which is moved on insert).
fn upsert_by_id<T, K: Fn(&T) -> u16>(vec: &mut Vec<T>, item: T, key: K, id: u16) {
    if let Some(slot) = vec.iter_mut().find(|e| key(e) == id) {
        *slot = item; // existing id: replace in place.
        return;
    }
    let pos = vec.iter().position(|e| key(e) > id).unwrap_or(vec.len());
    vec.insert(pos, item); // new id: keep a sorted Vec sorted.
}

/// Insert-or-replace an auto-switch rule keyed by its `(device, exe_substr, title_substr)` match
/// tuple. If a rule with that exact tuple exists, its `profile` is updated in place (so re-pointing
/// a rule's target is an edit, not a duplicate); otherwise the rule is appended, preserving the
/// first-match-wins evaluation order (`match_rules`, §7.4). Appending keeps the order the user added
/// rules in, which is the order the watcher evaluates.
fn upsert_auto_switch_rule(rules: &mut Vec<AutoSwitchRule>, rule: AutoSwitchRule) {
    if let Some(existing) = rules.iter_mut().find(|r| {
        r.device == rule.device
            && r.exe_substr == rule.exe_substr
            && r.title_substr == rule.title_substr
    }) {
        existing.profile = rule.profile; // same match tuple: just re-point the target profile.
        return;
    }
    rules.push(rule);
}

/// Mutable borrow of the profile assigned to `device` (via `EngineConfig::assignments`), through
/// [`Arc::make_mut`]. `None` if the device has no assignment or the assigned profile is absent.
#[inline]
fn profile_for_device_mut<'a>(cfg: &'a mut EngineConfig, device: &str) -> Option<&'a mut Profile> {
    let pid = cfg.assignments.get(device)?.clone();
    profile_mut(cfg, &pid)
}

/// Mutable borrow of the profile `id` through [`Arc::make_mut`]. `None` if the id is absent.
#[inline]
fn profile_mut<'a>(cfg: &'a mut EngineConfig, id: &str) -> Option<&'a mut Profile> {
    // Avoid the make_mut clone when the id is absent (keeps an unknown-id edit a true no-op).
    if !cfg.profiles.contains_key(id) {
        return None;
    }
    Arc::make_mut(&mut cfg.profiles).get_mut(id)
}

/// Mutable borrow of the selected stick's [`StickSettings`](hyperion_core::stick::settings::StickSettings)
/// inside a profile.
#[inline]
fn stick_settings_mut(
    p: &mut Profile,
    stick: Stick,
) -> &mut hyperion_core::stick::settings::StickSettings {
    match stick {
        Stick::Left => &mut p.ls,
        Stick::Right => &mut p.rs,
    }
}

/// Validate and clamp a config through `core` before it is ever published.
///
/// All range/ordering invariants (period/param clamps, curve `x2 >= x1`, enum fallbacks)
/// live in [`hyperion_core::config`] so the engine never re-implements them; this funnel
/// keeps that the single point of contact.
#[inline]
fn validate(cfg: EngineConfig) -> EngineConfig {
    cfg.clamped()
}

#[cfg(test)]
mod tests {
    use super::*;
    use hyperion_core::config::StickMode;
    use hyperion_core::input::Control;
    use hyperion_core::map::{BindTarget, KeyKind, PadBtn, Profile};
    use hyperion_core::rc::RcConfig;

    /// A config with one device `"dev"` assigned to a `"default"` profile, so profile edits have
    /// a concrete target reachable both directly (by profile id) and via the device assignment.
    fn store_with_profile() -> ConfigStore {
        let mut cfg = EngineConfig {
            active_device: "dev".to_string(),
            ..EngineConfig::default()
        };
        Arc::make_mut(&mut cfg.profiles).insert(
            "default".to_string(),
            Profile {
                name: "default".to_string(),
                ..Profile::default()
            },
        );
        cfg.assignments
            .insert("dev".to_string(), "default".to_string());
        ConfigStore::new(cfg)
    }

    #[test]
    fn new_store_starts_at_generation_one() {
        let store = ConfigStore::new(EngineConfig::default());
        assert_eq!(store.generation(), 1);
    }

    #[test]
    fn publish_bumps_generation_monotonically() {
        let store = ConfigStore::new(EngineConfig::default());
        let g0 = store.generation();
        let g1 = store.publish(EngineConfig::default());
        let g2 = store.publish(EngineConfig::default());
        assert_eq!(g1, g0 + 1);
        assert_eq!(g2, g1 + 1);
        assert_eq!(store.generation(), g2);
    }

    #[test]
    fn shared_handle_observes_published_snapshot() {
        let store = ConfigStore::new(EngineConfig::default());
        let reader_handle = store.handle();
        let reader_gen = store.generation_counter();

        let before = reader_gen.load(Ordering::Acquire);
        store.publish(EngineConfig::default());
        let after = reader_gen.load(Ordering::Acquire);
        assert!(after > before, "reader sees the generation bump");

        let _snapshot = reader_handle.load();
    }

    #[test]
    fn update_edits_then_publishes() {
        let store = ConfigStore::new(EngineConfig::default());
        let g0 = store.generation();
        let g1 = store.update(|_cfg| {});
        assert_eq!(g1, g0 + 1);
    }

    #[test]
    fn from_handle_shares_the_arcswap() {
        let handle: ConfigHandle = Arc::new(ArcSwap::from_pointee(EngineConfig::default()));
        let store = ConfigStore::from_handle(handle.clone());
        store.publish(EngineConfig::default());
        assert!(Arc::ptr_eq(&handle, &store.handle()));
    }

    #[test]
    fn apply_set_stick_mode_targets_assigned_profile() {
        let store = store_with_profile();
        let g0 = store.generation();
        assert!(
            !store.snapshot().profiles["default"].ls.rc_mode_on,
            "default left-stick RC stage is off"
        );

        let changed = store.apply(&ControlMsg::SetStickMode {
            device: "dev".to_string(),
            stick: Stick::Left,
            mode: StickMode::Rc,
        });

        assert!(changed, "switching the RC stage on is a real change");
        assert!(store.snapshot().profiles["default"].ls.rc_mode_on);
        assert_eq!(store.generation(), g0 + 1, "generation bumped exactly once");
    }

    #[test]
    fn apply_set_rc_clamps_and_targets_assigned_profile() {
        let store = store_with_profile();
        // Wildly out-of-range RC params; the writer must clamp via core before publishing.
        let rc = RcConfig {
            enabled: true,
            period_us: 999,     // below MIN_PERIOD_US
            fixed_param: 9_999, // above MAX_PARAM
            ..RcConfig::default()
        };
        let changed = store.apply(&ControlMsg::SetRc {
            device: "dev".to_string(),
            stick: Stick::Right,
            rc,
        });

        assert!(changed);
        let stored = store.snapshot().profiles["default"].rs.rc;
        let expected = rc.clamped();
        assert_eq!(stored.period_us, expected.period_us, "period clamped up");
        assert_eq!(
            stored.fixed_param, expected.fixed_param,
            "param clamped down"
        );
        assert!(stored.enabled);
    }

    #[test]
    fn apply_set_binding_mutates_and_bumps_generation() {
        let store = store_with_profile();
        let g0 = store.generation();
        let changed = store.apply(&ControlMsg::SetBinding {
            profile: "default".to_string(),
            control: Control::Cross,
            bind: BindTarget::GamepadButton(PadBtn::B),
        });
        assert!(changed, "binding Cross->B is a real change");
        let snap = store.snapshot();
        assert_eq!(
            snap.profiles["default"].bindings[&Control::Cross].bind,
            BindTarget::GamepadButton(PadBtn::B)
        );
        assert_eq!(store.generation(), g0 + 1);

        // Clearing it removes the slot (back to identity passthrough).
        let cleared = store.apply(&ControlMsg::ClearBinding {
            profile: "default".to_string(),
            control: Control::Cross,
        });
        assert!(cleared);
        assert!(!store.snapshot().profiles["default"]
            .bindings
            .contains_key(&Control::Cross));
    }

    #[test]
    fn apply_set_binding_to_key_survives_round_trip() {
        let store = store_with_profile();
        let changed = store.apply(&ControlMsg::SetBinding {
            profile: "default".to_string(),
            control: Control::Square,
            bind: BindTarget::Key {
                vk: 0x41,
                kind: KeyKind::HOLD,
            },
        });
        assert!(changed);
        assert_eq!(
            store.snapshot().profiles["default"].bindings[&Control::Square].bind,
            BindTarget::Key {
                vk: 0x41,
                kind: KeyKind::HOLD
            }
        );
    }

    #[test]
    fn apply_create_then_assign_profile() {
        let store = store_with_profile();
        // Create a second profile and assign it to the device.
        assert!(store.apply(&ControlMsg::CreateProfile {
            name: "fps".to_string(),
        }));
        assert!(store.snapshot().profiles.contains_key("fps"));

        let g_before = store.generation();
        assert!(store.apply(&ControlMsg::SetActiveProfile {
            device: "dev".to_string(),
            name: "fps".to_string(),
        }));
        assert_eq!(store.snapshot().assignments["dev"], "fps");
        assert_eq!(store.generation(), g_before + 1);
    }

    #[test]
    fn apply_assign_unknown_profile_is_noop() {
        let store = store_with_profile();
        let g0 = store.generation();
        let changed = store.apply(&ControlMsg::SetActiveProfile {
            device: "dev".to_string(),
            name: "missing".to_string(),
        });
        assert!(!changed, "assigning a non-existent profile is a no-op");
        assert_eq!(store.generation(), g0);
        assert_eq!(store.snapshot().assignments["dev"], "default");
    }

    #[test]
    fn apply_set_active_profile_to_current_is_noop() {
        let store = store_with_profile();
        let g0 = store.generation();
        let changed = store.apply(&ControlMsg::SetActiveProfile {
            device: "dev".to_string(),
            name: "default".to_string(),
        });
        assert!(
            !changed,
            "re-assigning the same profile must report no change"
        );
        assert_eq!(store.generation(), g0);
    }

    #[test]
    fn apply_delete_profile_drops_assignment() {
        let store = store_with_profile();
        assert!(store.apply(&ControlMsg::DeleteProfile {
            name: "default".to_string(),
        }));
        let snap = store.snapshot();
        assert!(!snap.profiles.contains_key("default"));
        assert!(
            !snap.assignments.contains_key("dev"),
            "deleting a profile drops the assignment that pointed at it"
        );
    }

    #[test]
    fn apply_rename_profile_repoints_assignment() {
        let store = store_with_profile();
        assert!(store.apply(&ControlMsg::RenameProfile {
            from: "default".to_string(),
            to: "renamed".to_string(),
        }));
        let snap = store.snapshot();
        assert!(snap.profiles.contains_key("renamed"));
        assert!(!snap.profiles.contains_key("default"));
        assert_eq!(snap.assignments["dev"], "renamed");
    }

    #[test]
    fn apply_set_stick_settings_clamps() {
        let store = store_with_profile();
        let base = hyperion_core::stick::settings::StickSettings::default();
        let settings = hyperion_core::stick::settings::StickSettings {
            sensitivity: 2.0,
            dead_zone: hyperion_core::stick::settings::StickDeadZone {
                dead_zone: 9_999, // above the 127 clamp
                ..base.dead_zone
            },
            ..base
        };
        let changed = store.apply(&ControlMsg::SetStickSettings {
            profile: "default".to_string(),
            stick: Stick::Left,
            settings,
        });
        assert!(changed);
        let stored = store.snapshot().profiles["default"].ls;
        assert_eq!(stored.dead_zone.dead_zone, 127, "deadzone clamped to 127");
        assert_eq!(stored.sensitivity, 2.0);
    }

    #[test]
    fn apply_set_trigger_settings_clamps() {
        let store = store_with_profile();
        // max_zone clamps to [1,100]; the default is already 100, so use a below-range value
        // (0 -> 1) to produce a genuine change AND exercise the clamp.
        let settings = hyperion_core::trigger::TriggerSettings {
            max_zone: 0, // below the 1 clamp
            ..hyperion_core::trigger::TriggerSettings::default()
        };
        let changed = store.apply(&ControlMsg::SetTriggerSettings {
            profile: "default".to_string(),
            trigger: Trigger::Right,
            settings,
        });
        assert!(changed);
        assert_eq!(store.snapshot().profiles["default"].r2.max_zone, 1);
    }

    #[test]
    fn apply_noop_returns_false_without_bumping_generation() {
        let store = store_with_profile();
        let g0 = store.generation();

        // Setting the active device to its current value is a genuine no-op.
        let changed = store.apply(&ControlMsg::SetActiveDevice("dev".to_string()));
        assert!(!changed, "re-setting the same value must report no change");
        assert_eq!(store.generation(), g0, "no-op must not bump the generation");

        // Editing an unknown profile is also a no-op.
        let unknown = store.apply(&ControlMsg::SetBinding {
            profile: "missing".to_string(),
            control: Control::Cross,
            bind: BindTarget::GamepadButton(PadBtn::A),
        });
        assert!(!unknown, "editing an absent profile changes nothing");
        assert_eq!(store.generation(), g0);
    }

    #[test]
    fn absent_id_edits_are_accepted_noops() {
        let store = store_with_profile();
        let g0 = store.generation();
        // Deleting a macro that does not exist is a no-op (no panic, no bump).
        assert!(!store.apply(&ControlMsg::DeleteMacro {
            profile: "default".to_string(),
            id: 1,
        }));
        // Deleting an auto-switch rule that does not exist is also a no-op.
        assert!(!store.apply(&ControlMsg::DeleteAutoSwitchRule {
            device: String::new(),
            exe_substr: "nope".to_string(),
            title_substr: String::new(),
        }));
        // Re-setting auto-switch to its current (default `false`) value is a no-op.
        assert!(!store.apply(&ControlMsg::SetAutoSwitchEnabled(false)));
        assert_eq!(store.generation(), g0);
    }

    #[test]
    fn apply_upsert_then_delete_macro_mutates_profile() {
        use hyperion_core::map::profile::{MacroDef, MacroStep};
        let store = store_with_profile();
        let g0 = store.generation();

        let def = MacroDef {
            id: 7,
            name: "reload".to_string(),
            repeat: false,
            steps: vec![
                MacroStep::KeyDown {
                    vk: 0x52,
                    scan_code: true,
                },
                MacroStep::Wait { ms: 25 },
                MacroStep::KeyUp {
                    vk: 0x52,
                    scan_code: true,
                },
            ],
        };
        assert!(store.apply(&ControlMsg::UpsertMacro {
            profile: "default".to_string(),
            def: def.clone(),
        }));
        let snap = store.snapshot();
        assert_eq!(snap.profiles["default"].macros.len(), 1);
        assert_eq!(snap.profiles["default"].macros[0], def);
        assert_eq!(store.generation(), g0 + 1);

        // Upserting the same id replaces it (no duplicate appended).
        let def2 = MacroDef {
            name: "reload-fast".to_string(),
            ..def
        };
        assert!(store.apply(&ControlMsg::UpsertMacro {
            profile: "default".to_string(),
            def: def2.clone(),
        }));
        let snap = store.snapshot();
        assert_eq!(
            snap.profiles["default"].macros.len(),
            1,
            "same id replaces, not appends"
        );
        assert_eq!(snap.profiles["default"].macros[0].name, "reload-fast");

        // Delete drops it.
        assert!(store.apply(&ControlMsg::DeleteMacro {
            profile: "default".to_string(),
            id: 7,
        }));
        assert!(store.snapshot().profiles["default"].macros.is_empty());
    }

    #[test]
    fn apply_upsert_macros_stay_sorted_by_id() {
        use hyperion_core::map::profile::MacroDef;
        let store = store_with_profile();
        for id in [9u16, 2, 5] {
            let def = MacroDef {
                id,
                name: format!("m{id}"),
                ..MacroDef::default()
            };
            assert!(store.apply(&ControlMsg::UpsertMacro {
                profile: "default".to_string(),
                def,
            }));
        }
        let snap = store.snapshot();
        let ids: Vec<u16> = snap.profiles["default"]
            .macros
            .iter()
            .map(|m| m.id)
            .collect();
        assert_eq!(
            ids,
            vec![2, 5, 9],
            "macros stay sorted by id for a stable on-disk form"
        );
    }

    #[test]
    fn apply_upsert_then_delete_special_action() {
        use hyperion_core::map::SpecialAction;
        let store = store_with_profile();
        let action = SpecialAction {
            id: 3,
            name: "switch-profile".to_string(),
        };
        assert!(store.apply(&ControlMsg::UpsertSpecialAction {
            profile: "default".to_string(),
            action: action.clone(),
        }));
        assert_eq!(store.snapshot().profiles["default"].specials, vec![action]);
        assert!(store.apply(&ControlMsg::DeleteSpecialAction {
            profile: "default".to_string(),
            id: 3,
        }));
        assert!(store.snapshot().profiles["default"].specials.is_empty());
    }

    #[test]
    fn apply_set_mouse_settings_mutates_and_resolves_clamped() {
        use hyperion_core::map::MouseSettings;
        let store = store_with_profile();
        let settings = MouseSettings {
            sensitivity: 9_999.0, // above the 100 clamp
            invert_y: true,
            ..MouseSettings::default()
        };
        assert!(store.apply(&ControlMsg::SetMouseSettings {
            profile: "default".to_string(),
            settings,
        }));
        // The editable profile carries the user's typed value; the hot-facing resolved form is the
        // clamped one `apply()` consumes (resolve() runs MouseSettings::clamped()).
        let snap = store.snapshot();
        assert!(snap.profiles["default"].mouse.invert_y);
        let resolved = snap.resolved["dev"].mouse;
        assert_eq!(
            resolved.sensitivity, 100.0,
            "resolved mouse sensitivity clamped"
        );
        assert!(resolved.invert_y);
    }

    #[test]
    fn apply_set_binding_turbo_inserts_slot() {
        use hyperion_core::map::TurboCfg;
        let store = store_with_profile();
        let turbo = TurboCfg {
            period_us: 80_000,
            duty_num: 1,
            duty_den: 3,
        };
        assert!(store.apply(&ControlMsg::SetBindingTurbo {
            profile: "default".to_string(),
            control: Control::R2,
            turbo: Some(turbo),
        }));
        assert_eq!(
            store.snapshot().profiles["default"].bindings[&Control::R2].turbo,
            Some(turbo)
        );
        // Clearing turbo sets it back to None.
        assert!(store.apply(&ControlMsg::SetBindingTurbo {
            profile: "default".to_string(),
            control: Control::R2,
            turbo: None,
        }));
        assert_eq!(
            store.snapshot().profiles["default"].bindings[&Control::R2].turbo,
            None
        );
    }

    #[test]
    fn apply_set_shift_trigger_mutates_slot() {
        use hyperion_core::map::ShiftTrigger;
        let store = store_with_profile();
        assert!(store.apply(&ControlMsg::SetShiftTrigger {
            profile: "default".to_string(),
            control: Control::Cross,
            trigger: Some(ShiftTrigger {
                control: Control::L1,
            }),
            bind: BindTarget::GamepadButton(PadBtn::Y),
        }));
        let snap = store.snapshot();
        let slot = &snap.profiles["default"].bindings[&Control::Cross];
        assert_eq!(
            slot.shift_trigger,
            Some(ShiftTrigger {
                control: Control::L1
            })
        );
        assert_eq!(slot.shift_bind, BindTarget::GamepadButton(PadBtn::Y));
    }

    #[test]
    fn apply_set_gyro_settings_mutates_and_resolves_clamped() {
        use hyperion_core::map::profile::{GyroMode, GyroSettings};
        let store = store_with_profile();
        let settings = GyroSettings {
            mode: GyroMode::AlwaysOn,
            sensitivity: 9_999.0, // above the 100 clamp
            invert_x: true,
            ..GyroSettings::default()
        };
        assert!(store.apply(&ControlMsg::SetGyroSettings {
            profile: "default".to_string(),
            settings,
        }));
        let snap = store.snapshot();
        // The editable profile carries the typed value; the resolved (hot-facing) form is clamped.
        assert_eq!(snap.profiles["default"].gyro.mode, GyroMode::AlwaysOn);
        assert!(snap.profiles["default"].gyro.invert_x);
        let resolved = snap.resolved["dev"].gyro;
        assert_eq!(
            resolved.sensitivity, 100.0,
            "resolved gyro sensitivity clamped"
        );
        assert_eq!(resolved.mode, GyroMode::AlwaysOn);
    }

    #[test]
    fn apply_set_auto_switch_enabled_toggles() {
        let store = store_with_profile();
        assert!(!store.snapshot().auto_switch.enabled, "default is disabled");
        assert!(store.apply(&ControlMsg::SetAutoSwitchEnabled(true)));
        assert!(store.snapshot().auto_switch.enabled, "enabled");
        // Re-enabling is a no-op.
        assert!(!store.apply(&ControlMsg::SetAutoSwitchEnabled(true)));
        // Disabling toggles back.
        assert!(store.apply(&ControlMsg::SetAutoSwitchEnabled(false)));
        assert!(!store.snapshot().auto_switch.enabled);
    }

    #[test]
    fn apply_upsert_auto_switch_rule_inserts_then_repoints() {
        let store = store_with_profile();
        let rule = AutoSwitchRule {
            device: String::new(),
            exe_substr: "valorant".to_string(),
            title_substr: String::new(),
            profile: "default".to_string(),
        };
        assert!(store.apply(&ControlMsg::UpsertAutoSwitchRule { rule: rule.clone() }));
        let snap = store.snapshot();
        assert_eq!(snap.auto_switch.rules.len(), 1);
        assert_eq!(snap.auto_switch.rules[0], rule);

        // Upserting the SAME match tuple with a different profile re-points it in place (no append).
        // Create the target profile first so the rule's profile id refers to something real.
        assert!(store.apply(&ControlMsg::CreateProfile {
            name: "fps".to_string(),
        }));
        let repointed = AutoSwitchRule {
            profile: "fps".to_string(),
            ..rule.clone()
        };
        assert!(store.apply(&ControlMsg::UpsertAutoSwitchRule {
            rule: repointed.clone(),
        }));
        let snap = store.snapshot();
        assert_eq!(
            snap.auto_switch.rules.len(),
            1,
            "same tuple replaces, not appends"
        );
        assert_eq!(snap.auto_switch.rules[0].profile, "fps");

        // A DIFFERENT match tuple appends a second rule (order preserved for first-match-wins).
        let other = AutoSwitchRule {
            exe_substr: "csgo".to_string(),
            ..rule
        };
        assert!(store.apply(&ControlMsg::UpsertAutoSwitchRule {
            rule: other.clone(),
        }));
        let snap = store.snapshot();
        assert_eq!(snap.auto_switch.rules.len(), 2);
        assert_eq!(snap.auto_switch.rules[1].exe_substr, "csgo");
    }

    #[test]
    fn apply_delete_auto_switch_rule_removes_by_match_tuple() {
        let store = store_with_profile();
        let rule = AutoSwitchRule {
            device: "dev".to_string(),
            exe_substr: "game".to_string(),
            title_substr: "Ranked".to_string(),
            profile: "default".to_string(),
        };
        assert!(store.apply(&ControlMsg::UpsertAutoSwitchRule { rule: rule.clone() }));
        assert_eq!(store.snapshot().auto_switch.rules.len(), 1);
        // Delete by the exact match tuple.
        assert!(store.apply(&ControlMsg::DeleteAutoSwitchRule {
            device: "dev".to_string(),
            exe_substr: "game".to_string(),
            title_substr: "Ranked".to_string(),
        }));
        assert!(store.snapshot().auto_switch.rules.is_empty());
    }

    #[test]
    fn save_and_reload_without_path_are_noops() {
        let store = store_with_profile();
        let g0 = store.generation();
        assert!(!store.apply(&ControlMsg::SaveToDisk));
        assert!(!store.apply(&ControlMsg::ReloadFromDisk));
        assert_eq!(store.generation(), g0);
    }
}
