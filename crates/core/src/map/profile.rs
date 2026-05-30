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
use std::sync::Arc;

use crate::input::{Control, Thresholds};
use crate::map::binding::BindingSlot;
use crate::mouse_accum::{MouseAccumCfg, MOUSE_VELOCITY_OFFSET_DEFAULT};
use crate::output::{MouseButton, PadTarget};
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
    // ---- M4 settings groups (now consumed by `resolve()` + the injector) + M5 placeholders ----
    /// Mouse-from-stick / wheel settings. **M4 consumer** ([`MouseSettings::to_accum_cfg`]).
    #[serde(default)]
    pub mouse: MouseSettings,
    /// Gyro→mouse / gyro→stick settings. M5 consumer.
    #[serde(default)]
    pub gyro: GyroSettings,
    /// Macro definitions referenced by `BindTarget::Macro(id)`. **M4 consumer** (the injector
    /// thread plays them on a `Macro{start}` edge).
    #[serde(default)]
    pub macros: Vec<MacroDef>,
    /// Special actions referenced by `BindTarget::Special(id)`. M4/M5 consumer.
    #[serde(default)]
    pub specials: Vec<SpecialAction>,
}

/// Mouse-from-stick / wheel settings (M4). The editable, persisted form; [`MouseSettings::clamped`]
/// enforces the ranges and [`MouseSettings::to_accum_cfg`] projects it into the hot-facing
/// [`MouseAccumCfg`] the engine's [`apply`](crate::map::apply) feeds the
/// [`MouseAccumulator`](crate::mouse_accum::MouseAccumulator).
///
/// Field semantics mirror DS4Windows `ButtonMouseInfo` (blueprint §6.2): `sensitivity` is the base
/// per-unit velocity scale, `vertical_scale` scales the Y velocity only, `velocity_offset` is the
/// anti-jitter start offset (fraction of velocity), `deadzone` is the normalized stick dead-zone
/// below which deflection is ignored, `min_threshold` is the per-report motion gate (`1.0` is the
/// DS4Windows "always carry, no gate" special case), `accelerate`/`accel_power` apply an optional
/// power curve, and `invert_x`/`invert_y` negate the final integer delta.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct MouseSettings {
    /// Base sensitivity (DS4Windows `activeButtonSensitivity`, typically ~10..100).
    pub sensitivity: f64,
    /// Extra Y-velocity scale (DS4Windows `buttonVerticalScale`).
    pub vertical_scale: f64,
    /// Anti-jitter velocity-offset fraction (DS4Windows `mouseVelocityOffset`).
    pub velocity_offset: f64,
    /// Normalized stick dead-zone in `[0,1)`.
    pub deadzone: f64,
    /// Per-report motion gate (`1.0` == always-carry special case).
    pub min_threshold: f64,
    /// Apply the `accel_power` curve to the normalized deflection.
    pub accelerate: bool,
    /// Acceleration exponent applied to the deflection when `accelerate` is set.
    pub accel_power: f64,
    /// Invert the horizontal (X) output.
    pub invert_x: bool,
    /// Invert the vertical (Y) output.
    pub invert_y: bool,
}

impl Default for MouseSettings {
    /// DS4Windows-class defaults (mirrors [`MouseAccumCfg::default`]): sensitivity 25, unity
    /// vertical scale, the default anti-jitter offset, no dead-zone, the always-carry gate, no
    /// acceleration, no inversion. Equivalent to the M3 inert placeholder for an unconfigured
    /// profile, so adding the fields is non-breaking.
    fn default() -> Self {
        Self {
            sensitivity: 25.0,
            vertical_scale: 1.0,
            velocity_offset: MOUSE_VELOCITY_OFFSET_DEFAULT,
            deadzone: 0.0,
            min_threshold: 1.0,
            accelerate: false,
            accel_power: 1.0,
            invert_x: false,
            invert_y: false,
        }
    }
}

impl MouseSettings {
    /// Clamp every field into its valid range (called off the hot path, in `resolve()` / the
    /// config-store funnel). Sensitivity stays positive, the scales/offset/deadzone are bounded,
    /// the min-threshold floors at 1.0 (the always-carry special case), and the acceleration power
    /// stays in a sane curve range. Non-finite inputs collapse to the default.
    #[inline]
    #[must_use]
    pub fn clamped(self) -> Self {
        let d = Self::default();
        let fin = |v: f64, fallback: f64| if v.is_finite() { v } else { fallback };
        Self {
            sensitivity: fin(self.sensitivity, d.sensitivity).clamp(0.01, 100.0),
            vertical_scale: fin(self.vertical_scale, d.vertical_scale).clamp(0.1, 10.0),
            velocity_offset: fin(self.velocity_offset, d.velocity_offset).clamp(0.0, 1.0),
            deadzone: fin(self.deadzone, d.deadzone).clamp(0.0, 0.99),
            min_threshold: fin(self.min_threshold, d.min_threshold).max(1.0),
            accelerate: self.accelerate,
            accel_power: fin(self.accel_power, d.accel_power).clamp(0.1, 8.0),
            invert_x: self.invert_x,
            invert_y: self.invert_y,
        }
    }

    /// Project into the hot-facing [`MouseAccumCfg`] the engine feeds the mouse accumulator. The
    /// field set is 1:1 (the accumulator is the runtime form of these settings), so this is a flat
    /// copy after `clamped()` has run in `resolve()`.
    #[inline]
    #[must_use]
    pub fn to_accum_cfg(self) -> MouseAccumCfg {
        MouseAccumCfg {
            sensitivity: self.sensitivity,
            vertical_scale: self.vertical_scale,
            velocity_offset: self.velocity_offset,
            deadzone: self.deadzone,
            min_threshold: self.min_threshold,
            accelerate: self.accelerate,
            accel_power: self.accel_power,
            invert_x: self.invert_x,
            invert_y: self.invert_y,
        }
    }
}

/// Forward-compat placeholder for gyro settings (M5). Inert in M3/M4.
#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct GyroSettings {}

/// One mouse-button selector inside a [`MacroStep`] (kept separate from [`MouseButton`] only so the
/// serde form is explicit/append-only). Maps 1:1 to [`MouseButton`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum MacroMouseButton {
    Left,
    Right,
    Middle,
    X1,
    X2,
    /// Append-only fallback for an unknown persisted tag (degrades to `Left`).
    #[serde(other)]
    Unknown,
}

impl MacroMouseButton {
    /// The [`MouseButton`] this selector injects (unknown → `Left`, a harmless default).
    #[inline]
    #[must_use]
    pub fn to_mouse_button(self) -> MouseButton {
        match self {
            Self::Left | Self::Unknown => MouseButton::Left,
            Self::Right => MouseButton::Right,
            Self::Middle => MouseButton::Middle,
            Self::X1 => MouseButton::X1,
            Self::X2 => MouseButton::X2,
        }
    }
}

/// One step of a timed macro (blueprint §8 macros.rs: `KeyDown / KeyUp / MouseDown / MouseUp /
/// Wait`). The injector thread plays the step list with real timing entirely off the hot path
/// (blueprint §7.3); the hot loop only emits a `Macro{start}` edge.
///
/// The variant order/names are an append-only persisted-profile contract.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "kind", content = "val")]
pub enum MacroStep {
    /// Press a keyboard key (virtual-key code). `scan_code` selects hardware-scancode injection.
    KeyDown {
        /// Windows virtual-key code.
        vk: u16,
        /// Inject as a hardware scancode (game-compatible) rather than the virtual key.
        #[serde(default)]
        scan_code: bool,
    },
    /// Release a keyboard key.
    KeyUp {
        /// Windows virtual-key code.
        vk: u16,
        /// Inject as a hardware scancode rather than the virtual key.
        #[serde(default)]
        scan_code: bool,
    },
    /// Press a mouse button.
    MouseDown(MacroMouseButton),
    /// Release a mouse button.
    MouseUp(MacroMouseButton),
    /// Wait `ms` milliseconds before the next step (the injector owns the sleep).
    Wait {
        /// Delay in milliseconds.
        ms: u32,
    },
    /// Append-only fallback for an unknown persisted step (the injector skips it).
    #[serde(other)]
    Unknown,
}

/// A timed macro definition (M4): an id-keyed list of [`MacroStep`]s the injector plays on a
/// `Macro{start}` edge. `repeat` re-runs the whole sequence while the source is held (DS4Windows
/// "Macro" run-while-held mode); when `false` it fires once per press.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct MacroDef {
    /// Stable id referenced by `BindTarget::Macro(id)`.
    pub id: u16,
    /// Display name.
    pub name: String,
    /// Re-run the step list while the source stays held (vs. fire once per press).
    pub repeat: bool,
    /// The ordered step list (`KeyDown`/`KeyUp`/`MouseDown`/`MouseUp`/`Wait`).
    pub steps: Vec<MacroStep>,
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
            // M4: clamped mouse-from-stick settings (consumed by `apply()` via `to_accum_cfg`) and
            // an Arc-shared macro table (consumed by the injector thread, never the hot path).
            mouse: self.mouse.clamped(),
            macros: Arc::from(self.macros.clone()),
        }
    }
}

/// The hot-loop-ready, immutable resolved profile (blueprint §7.1).
///
/// All fixed-size, all `Copy`-of-`Copy` fields: indexable by `control as usize` with zero map/hash
/// lookups. Rebuilt only on the config generation gate, then read-only per report.
///
/// M3 held the `base` slot array + clamped stick/trigger settings + thresholds + output kind. M4
/// adds the resolved `mouse` settings (consumed by `apply()` via [`MouseSettings::to_accum_cfg`])
/// and the `macros` table (consumed off the hot path by the injector thread). The per-control shift
/// table is read from `base` each report; gyro/special resolution lands in M5.
///
/// Not `Copy`: the `macros` table is an `Arc<[MacroDef]>` (cheap to `Clone`, refcount-shared so a
/// per-generation rebuild does not deep-copy the step lists — blueprint §7.1). Everything the hot
/// path reads (`base`/`ls`/`rs`/`l2`/`r2`/`thresholds`/`mouse`) is still flat `Copy`-of-`Copy`, so
/// the per-report cost is unchanged; the `Arc` is only touched on the generation gate.
///
/// Not `PartialEq`: the resolved `Thresholds` (blueprint §3.1) is `Copy`-only by contract.
#[derive(Clone, Debug)]
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
    /// Resolved (clamped) mouse-from-stick settings; `apply()` calls
    /// [`to_accum_cfg`](MouseSettings::to_accum_cfg) to feed the mouse accumulator.
    pub mouse: MouseSettings,
    /// The profile's macro table, Arc-shared so a generation rebuild is a refcount bump (the hot
    /// path never reads this; the injector thread plays a macro on a `Macro{start}` edge).
    pub macros: Arc<[MacroDef]>,
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
            mouse: MouseSettings::default(),
            macros: Arc::from(Vec::new()),
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

    #[test]
    fn mouse_settings_default_matches_accum_default() {
        // The editable default must project to the accumulator's own default (so an unconfigured
        // profile behaves exactly like the M3 `MouseAccumCfg::default()` the engine used before).
        assert_eq!(
            MouseSettings::default().to_accum_cfg(),
            MouseAccumCfg::default()
        );
    }

    #[test]
    fn mouse_settings_clamp_bounds_and_non_finite() {
        let wild = MouseSettings {
            sensitivity: 9_999.0,
            vertical_scale: 0.0,  // below the 0.1 floor
            velocity_offset: 5.0, // above 1.0
            deadzone: 2.0,        // above 0.99
            min_threshold: 0.0,   // below the 1.0 floor
            accel_power: 100.0,   // above 8.0
            accelerate: true,
            invert_x: true,
            invert_y: true,
        };
        let c = wild.clamped();
        assert_eq!(c.sensitivity, 100.0);
        assert_eq!(c.vertical_scale, 0.1);
        assert_eq!(c.velocity_offset, 1.0);
        assert_eq!(c.deadzone, 0.99);
        assert_eq!(
            c.min_threshold, 1.0,
            "min_threshold floors at the always-carry 1.0"
        );
        assert_eq!(c.accel_power, 8.0);
        assert!(c.accelerate && c.invert_x && c.invert_y);

        // Non-finite inputs collapse to the default before clamping.
        let nan = MouseSettings {
            sensitivity: f64::NAN,
            min_threshold: f64::INFINITY,
            ..MouseSettings::default()
        }
        .clamped();
        assert_eq!(nan.sensitivity, MouseSettings::default().sensitivity);
        assert!(nan.min_threshold.is_finite());
    }

    #[test]
    fn resolve_clamps_mouse_and_shares_macros() {
        let mut p = Profile {
            mouse: MouseSettings {
                sensitivity: -3.0, // clamps up to the 0.01 floor
                ..MouseSettings::default()
            },
            ..Profile::default()
        };
        p.macros.push(MacroDef {
            id: 5,
            name: "reload".to_string(),
            repeat: false,
            steps: vec![
                MacroStep::KeyDown {
                    vk: 0x52,
                    scan_code: true,
                },
                MacroStep::Wait { ms: 30 },
                MacroStep::KeyUp {
                    vk: 0x52,
                    scan_code: true,
                },
            ],
        });
        let rp = p.resolve();
        assert_eq!(
            rp.mouse.sensitivity, 0.01,
            "resolve clamps the mouse settings"
        );
        assert_eq!(rp.macros.len(), 1);
        assert_eq!(rp.macros[0].id, 5);
        assert_eq!(rp.macros[0].steps.len(), 3);
    }

    #[test]
    fn default_resolved_has_empty_macros_and_default_mouse() {
        let rp = ResolvedProfile::default();
        assert!(rp.macros.is_empty());
        assert_eq!(rp.mouse, MouseSettings::default());
    }

    #[test]
    fn profile_with_mouse_and_macros_round_trips() {
        let mut p = Profile {
            name: "macro-test".to_string(),
            ..Default::default()
        };
        p.mouse = MouseSettings {
            sensitivity: 40.0,
            invert_y: true,
            ..MouseSettings::default()
        };
        p.macros.push(MacroDef {
            id: 1,
            name: "burst".to_string(),
            repeat: true,
            steps: vec![
                MacroStep::MouseDown(MacroMouseButton::Left),
                MacroStep::Wait { ms: 10 },
                MacroStep::MouseUp(MacroMouseButton::Left),
            ],
        });
        let toml = toml::to_string(&p).unwrap();
        let back: Profile = toml::from_str(&toml).unwrap();
        assert_eq!(back, p);
    }

    #[test]
    fn macro_mouse_button_maps_to_output() {
        assert_eq!(
            MacroMouseButton::Right.to_mouse_button(),
            crate::output::MouseButton::Right
        );
        assert_eq!(
            MacroMouseButton::Unknown.to_mouse_button(),
            crate::output::MouseButton::Left
        );
    }
}
