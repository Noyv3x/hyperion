//! The single writer of the config [`ArcSwap`](arc_swap::ArcSwap).
//!
//! GUI edits, the supervisor, and (on Windows) the file-watch all converge here rather than
//! mutating the shared snapshot directly (`DESIGN.md` §6 "Single writer"). [`ConfigStore`]
//! validates/clamps every incoming config through [`hyperion_core::config`], publishes one
//! fresh immutable snapshot via `store()`, and bumps a generation counter so the hot loop can
//! cheaply detect "did config change?" with a single atomic load instead of diffing fields.
//!
//! The core publish/apply path needs no filesystem and is fully Linux-testable; TOML
//! persistence and `notify` file-watch are layered above it and gated out of the Linux unit
//! tests (they live behind the Windows supervisor path in M1).

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use arc_swap::ArcSwap;
use hyperion_core::config::EngineConfig;

use crate::handoff::ConfigHandle;

/// A monotonically increasing config version. The hot loop caches the last value it applied
/// and only re-reads the snapshot when this changes.
pub type Generation = u64;

/// Owns the write side of the config [`ConfigHandle`]. Construct **one** of these per engine;
/// every config change (GUI, supervisor, file-watch) routes through [`ConfigStore::apply`].
pub struct ConfigStore {
    handle: ConfigHandle,
    generation: Arc<AtomicU64>,
}

impl ConfigStore {
    /// Create a store seeded with `initial`, validated/clamped before its first publish, and
    /// generation `1` (the hot loop starts its cache at `0`, so it applies the seed once).
    pub fn new(initial: EngineConfig) -> Self {
        let validated = validate(initial);
        let handle = Arc::new(ArcSwap::from_pointee(validated));
        Self {
            handle,
            generation: Arc::new(AtomicU64::new(1)),
        }
    }

    /// Wrap an existing [`ConfigHandle`] (e.g. the one returned by
    /// [`crate::handoff::build_links`]) so the store and the hot loop publish/read the *same*
    /// `ArcSwap`. The handle's current contents are taken as already-validated.
    pub fn from_handle(handle: ConfigHandle) -> Self {
        Self {
            handle,
            generation: Arc::new(AtomicU64::new(1)),
        }
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

    /// Validate/clamp `next` through `core`, publish it as the new immutable snapshot, and
    /// bump the generation. Returns the new generation.
    ///
    /// This is the *only* path that mutates the shared config. It performs a whole-snapshot
    /// `store()` (wait-free for readers) and a `Release` increment of the generation so a hot
    /// loop that observes the new generation also observes the new snapshot.
    pub fn apply(&self, next: EngineConfig) -> Generation {
        let validated = validate(next);
        self.handle.store(Arc::new(validated));
        // Release so the snapshot store is visible before the generation bump the hot loop
        // keys off of.
        self.generation.fetch_add(1, Ordering::Release) + 1
    }

    /// Read-modify-publish helper: load the current snapshot, let `edit` mutate a clone, then
    /// validate + publish it. Convenience for field-level GUI edits.
    pub fn update<F: FnOnce(&mut EngineConfig)>(&self, edit: F) -> Generation {
        let mut next = (*self.handle.load_full()).clone();
        edit(&mut next);
        self.apply(next)
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

    #[test]
    fn new_store_starts_at_generation_one() {
        let store = ConfigStore::new(EngineConfig::default());
        assert_eq!(store.generation(), 1);
    }

    #[test]
    fn apply_bumps_generation_monotonically() {
        let store = ConfigStore::new(EngineConfig::default());
        let g0 = store.generation();
        let g1 = store.apply(EngineConfig::default());
        let g2 = store.apply(EngineConfig::default());
        assert_eq!(g1, g0 + 1);
        assert_eq!(g2, g1 + 1);
        assert_eq!(store.generation(), g2);
    }

    #[test]
    fn shared_handle_observes_published_snapshot() {
        let store = ConfigStore::new(EngineConfig::default());
        // A hot-loop-style reader holds the same handle + generation counter.
        let reader_handle = store.handle();
        let reader_gen = store.generation_counter();

        let before = reader_gen.load(Ordering::Acquire);
        store.apply(EngineConfig::default());
        let after = reader_gen.load(Ordering::Acquire);
        assert!(after > before, "reader sees the generation bump");

        // The reader can load the latest snapshot wait-free.
        let _snapshot = reader_handle.load();
    }

    #[test]
    fn update_edits_then_publishes() {
        let store = ConfigStore::new(EngineConfig::default());
        let g0 = store.generation();
        // The closure receives a mutable clone; even a no-op edit publishes a new generation.
        let g1 = store.update(|_cfg| {});
        assert_eq!(g1, g0 + 1);
    }

    #[test]
    fn from_handle_shares_the_arcswap() {
        let handle: ConfigHandle = Arc::new(ArcSwap::from_pointee(EngineConfig::default()));
        let store = ConfigStore::from_handle(handle.clone());
        store.apply(EngineConfig::default());
        // Both the external handle and the store's handle point at the same ArcSwap.
        assert!(Arc::ptr_eq(&handle, &store.handle()));
    }
}
