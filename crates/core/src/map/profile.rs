//! Profiles — the editable serde [`Profile`] and the hot-loop-ready [`ResolvedProfile`].
//!
//! A [`Profile`] is the user-facing, string-/map-keyed editable form that lives in
//! `EngineConfig::profiles`. Off the hot path (on the generation gate), [`Profile::resolve`]
//! flattens it into a [`ResolvedProfile`]: a fully immutable, **fixed-array** form keyed by
//! `Control as usize` — **no `HashMap`/`BTreeMap`/`String` on the hot path** (verifier latency
//! FIX 1/2/10). `apply()` indexes the resolved arrays directly.
//!
//! M3 carries the M4/M5 setting groups (mouse/gyro/macros/specials) as forward-compatible
//! placeholders so those milestones are purely additive; `resolve()` simply does not consult them
//! yet.

use std::collections::BTreeMap;

use crate::input::{Control, Thresholds};
use crate::map::binding::BindingSlot;
use crate::output::PadTarget;
use crate::stick::settings::StickSettings;
use crate::trigger::TriggerSettings;

/// The editable, persisted profile (one per named entry in `EngineConfig::profiles`).
///
/// Bindings are a **sparse** `BTreeMap<Control, BindingSlot>`: any control absent from the map
/// resolves to the default (identity passthrough) slot. This keeps the on-disk form compact and
/// stable while [`ResolvedProfile`] is the dense hot-facing form.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Profile {
    /// Human-facing profile name (also the map key in practice; stored for rename robustness).
    pub name: String,
    /// Which virtual pad this profile drives (X360 / DS4). Chosen at (re)plug time.
    pub output_kind: PadTarget,
    /// Sparse per-control bindings; absent => identity passthrough.
    pub bindings: BTreeMap<Control, BindingSlot>,
    /// Left-stick settings (full pipeline incl. the `rc` sub-config).
    pub ls: StickSettings,
    /// Right-stick settings.
    pub rs: StickSettings,
    /// Left-trigger settings.
    pub l2: TriggerSettings,
    /// Right-trigger settings.
    pub r2: TriggerSettings,
    // ---- M4/M5 placeholders (carried so later milestones are additive; not consulted in M3) ----
    /// Mouse-from-stick / wheel settings. M4 consumer.
    #[serde(default)]
    pub mouse: MouseSettings,
    /// Gyro→mouse / gyro→stick settings. M5 consumer.
    #[serde(default)]
    pub gyro: GyroSettings,
    /// Macro definitions referenced by `BindTarget::Macro(id)`. M4 consumer.
    #[serde(default)]
    pub macros: Vec<MacroDef>,
    /// Special actions referenced by `BindTarget::Special(id)`. M4/M5 consumer.
    #[serde(default)]
    pub specials: Vec<SpecialAction>,
}

/// Forward-compat placeholder for mouse-from-stick / wheel settings (M4). Inert in M3.
#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct MouseSettings {}

/// Forward-compat placeholder for gyro settings (M5). Inert in M3.
#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct GyroSettings {}

/// Forward-compat placeholder for a macro definition (M4). Inert in M3.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct MacroDef {
    /// Stable id referenced by `BindTarget::Macro(id)`.
    pub id: u16,
    /// Display name.
    pub name: String,
}

/// Forward-compat placeholder for a special action (M4/M5). Inert in M3.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct SpecialAction {
    /// Stable id referenced by `BindTarget::Special(id)`.
    pub id: u16,
    /// Display name.
    pub name: String,
}

impl Profile {
    /// Resolve this editable profile into the immutable, fixed-array hot-facing form.
    ///
    /// Done **off the hot path** (on the generation gate). Walks every `Control` once and copies
    /// its slot (or the default passthrough slot when absent), clamps stick/trigger settings, and
    /// snapshots the thresholds + output kind. No allocation survives into the returned value
    /// beyond the inline fixed arrays.
    pub fn resolve(&self) -> ResolvedProfile {
        let mut base = [BindingSlot::default(); Control::COUNT];
        for c in Control::ALL {
            if let Some(slot) = self.bindings.get(&c) {
                base[c.as_index()] = *slot;
            }
        }
        ResolvedProfile {
            base,
            ls: self.ls.clamped(),
            rs: self.rs.clamped(),
            l2: self.l2.clamped(),
            r2: self.r2.clamped(),
            // M3: fixed kind-dependent thresholds (axis 55/127, trigger 100/255 — verifier FIX 3);
            // per-profile threshold editing is not exposed yet, so this is the default.
            thresholds: Thresholds::default(),
            output_kind: self.output_kind,
        }
    }
}

/// The hot-loop-ready, immutable resolved profile (blueprint §7.1).
///
/// All fixed-size, all `Copy`-of-`Copy` fields: indexable by `control as usize` with zero map/hash
/// lookups. Rebuilt only on the config generation gate, then read-only per report.
///
/// M3 holds the `base` slot array + clamped stick/trigger settings + thresholds + output kind. The
/// per-control shift table and resolved macro/special/mouse/gyro arrays land additively in M4/M5
/// (they already exist on `BindingSlot`/`Profile`, so adding them here is non-breaking).
///
/// Not `PartialEq`: the resolved `Thresholds` (blueprint §3.1) is `Copy`-only by contract.
#[derive(Clone, Copy, Debug)]
pub struct ResolvedProfile {
    /// Dense per-control base slots, indexed by `Control::as_index`.
    pub base: [BindingSlot; Control::COUNT],
    /// Resolved (clamped) left-stick settings.
    pub ls: StickSettings,
    /// Resolved (clamped) right-stick settings.
    pub rs: StickSettings,
    /// Resolved (clamped) left-trigger settings.
    pub l2: TriggerSettings,
    /// Resolved (clamped) right-trigger settings.
    pub r2: TriggerSettings,
    /// Per-kind digitization thresholds.
    pub thresholds: Thresholds,
    /// Virtual-pad target.
    pub output_kind: PadTarget,
}

impl Default for ResolvedProfile {
    /// The all-passthrough resolved profile (byte-identical egress to the pre-mapper baseline).
    fn default() -> Self {
        Self {
            base: [BindingSlot::default(); Control::COUNT],
            ls: StickSettings::default(),
            rs: StickSettings::default(),
            l2: TriggerSettings::default(),
            r2: TriggerSettings::default(),
            thresholds: Thresholds::default(),
            output_kind: PadTarget::default(),
        }
    }
}

impl ResolvedProfile {
    /// The resolved slot for `c` (dense index, no lookup).
    #[inline]
    pub fn slot(&self, c: Control) -> &BindingSlot {
        &self.base[c.as_index()]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::map::binding::{BindTarget, KeyKind, PadBtn};

    #[test]
    fn resolve_fills_dense_array_from_sparse_map() {
        let mut p = Profile::default();
        p.bindings.insert(
            Control::Cross,
            BindingSlot::from_bind(BindTarget::GamepadButton(PadBtn::B)),
        );
        p.bindings.insert(
            Control::Square,
            BindingSlot::from_bind(BindTarget::Key {
                vk: 0x41,
                kind: KeyKind::HOLD,
            }),
        );
        let rp = p.resolve();
        assert_eq!(
            rp.slot(Control::Cross).bind,
            BindTarget::GamepadButton(PadBtn::B)
        );
        assert_eq!(
            rp.slot(Control::Square).bind,
            BindTarget::Key {
                vk: 0x41,
                kind: KeyKind::HOLD
            }
        );
        // Every unbound control defaults to passthrough.
        assert_eq!(rp.slot(Control::Circle).bind, BindTarget::Passthrough);
        assert_eq!(rp.slot(Control::LxPos).bind, BindTarget::Passthrough);
    }

    #[test]
    fn default_resolved_is_all_passthrough() {
        let rp = ResolvedProfile::default();
        for c in Control::ALL {
            assert_eq!(rp.slot(c).bind, BindTarget::Passthrough);
        }
        assert_eq!(rp.output_kind, PadTarget::default());
    }

    #[test]
    fn profile_serde_roundtrip_omits_default_bindings() {
        let mut p = Profile {
            name: "fps".to_string(),
            ..Default::default()
        };
        p.bindings.insert(
            Control::Triangle,
            BindingSlot::from_bind(BindTarget::GamepadButton(PadBtn::Y)),
        );
        let toml = toml::to_string(&p).unwrap();
        let back: Profile = toml::from_str(&toml).unwrap();
        assert_eq!(back, p);
    }
}
