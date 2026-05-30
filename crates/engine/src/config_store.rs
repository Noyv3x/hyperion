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
use hyperion_core::config::{load_toml, to_toml, EngineConfig};

use crate::control::{ControlMsg, Stick};
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

/// Apply a non-disk [`ControlMsg`] to a mutable snapshot clone. Unknown devices are silently
/// skipped (the caller's no-change check then returns `false`).
fn edit_in_place(cfg: &mut EngineConfig, msg: &ControlMsg) {
    match msg {
        ControlMsg::SetStickMode {
            device,
            stick,
            mode,
        } => {
            if let Some(dev) = cfg.devices.get_mut(device) {
                stick_mut(dev, *stick).mode = *mode;
            }
        }
        ControlMsg::SetRc { device, stick, rc } => {
            if let Some(dev) = cfg.devices.get_mut(device) {
                stick_mut(dev, *stick).rc = *rc;
            }
        }
        ControlMsg::SetThread(thread) => cfg.thread = thread.clone(),
        ControlMsg::SetHidHide(hidhide) => cfg.hidhide = hidhide.clone(),
        ControlMsg::SetActiveDevice(id) => cfg.active_device = id.clone(),
        // Disk messages are handled before this function is reached.
        ControlMsg::SaveToDisk | ControlMsg::ReloadFromDisk => {}
    }
}

/// Mutable borrow of the selected stick's [`StickConfig`](hyperion_core::config::StickConfig).
#[inline]
fn stick_mut(
    dev: &mut hyperion_core::config::DeviceConfig,
    stick: Stick,
) -> &mut hyperion_core::config::StickConfig {
    match stick {
        Stick::Left => &mut dev.ls,
        Stick::Right => &mut dev.rs,
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
    use hyperion_core::config::{DeviceConfig, StickMode};
    use hyperion_core::rc::RcConfig;

    /// A config with a single device `"dev"` whose left stick runs the RC filter, so edits have
    /// something concrete to target.
    fn store_with_device() -> ConfigStore {
        let mut cfg = EngineConfig {
            active_device: "dev".to_string(),
            ..EngineConfig::default()
        };
        cfg.devices
            .insert("dev".to_string(), DeviceConfig::default());
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
    fn apply_set_stick_mode_changes_snapshot_and_bumps_generation() {
        let store = store_with_device();
        let g0 = store.generation();
        assert_eq!(
            store.snapshot().devices["dev"].ls.mode,
            StickMode::None,
            "default left-stick mode is None"
        );

        let changed = store.apply(&ControlMsg::SetStickMode {
            device: "dev".to_string(),
            stick: Stick::Left,
            mode: StickMode::Rc,
        });

        assert!(changed, "switching the mode is a real change");
        assert_eq!(store.snapshot().devices["dev"].ls.mode, StickMode::Rc);
        assert_eq!(store.generation(), g0 + 1, "generation bumped exactly once");
    }

    #[test]
    fn apply_set_rc_clamps_out_of_range_values() {
        let store = store_with_device();
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
        let stored = store.snapshot().devices["dev"].rs.rc;
        let expected = rc.clamped();
        assert_eq!(
            stored.period_us, expected.period_us,
            "period clamped up to the minimum"
        );
        assert_eq!(
            stored.fixed_param, expected.fixed_param,
            "fixed_param clamped down to the maximum"
        );
        assert!(stored.enabled);
    }

    #[test]
    fn apply_noop_returns_false_without_bumping_generation() {
        let store = store_with_device();
        let g0 = store.generation();

        // Setting the active device to its current value is a genuine no-op.
        let changed = store.apply(&ControlMsg::SetActiveDevice("dev".to_string()));
        assert!(!changed, "re-setting the same value must report no change");
        assert_eq!(store.generation(), g0, "no-op must not bump the generation");

        // Editing an unknown device is also a no-op.
        let unknown = store.apply(&ControlMsg::SetStickMode {
            device: "missing".to_string(),
            stick: Stick::Left,
            mode: StickMode::Rc,
        });
        assert!(!unknown, "editing an absent device changes nothing");
        assert_eq!(store.generation(), g0);
    }

    #[test]
    fn save_and_reload_without_path_are_noops() {
        let store = store_with_device();
        let g0 = store.generation();
        assert!(!store.apply(&ControlMsg::SaveToDisk));
        assert!(!store.apply(&ControlMsg::ReloadFromDisk));
        assert_eq!(store.generation(), g0);
    }
}
