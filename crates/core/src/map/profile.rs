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
use crate::mouse_accum::{
    GyroAccumCfg, MouseAccumCfg, TouchAccumCfg, GYRO_MOUSE_DEADZONE_DEFAULT,
    GYRO_MOUSE_OFFSET_DEFAULT, MOUSE_VELOCITY_OFFSET_DEFAULT, TOUCH_MOUSE_OFFSET_DEFAULT,
};
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
    /// Gyro→mouse settings. **M5 consumer** ([`GyroSettings::to_accum_cfg`]).
    #[serde(default)]
    pub gyro: GyroSettings,
    /// Touchpad→mouse / as-buttons settings. **M6 consumer** ([`TouchpadSettings::to_accum_cfg`]).
    #[serde(default)]
    pub touchpad: TouchpadSettings,
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

/// When the gyro→mouse output is active (DS4Windows `gyroTriggerBehavior` / activation model).
///
/// `Off` disables gyro→mouse entirely (the default, so an unconfigured profile has inert gyro);
/// `AlwaysOn` runs it every report; `TriggerHeld` runs it only while the gyro activation trigger is
/// held (the engine evaluates the trigger against RAW state and gates [`apply`](crate::map::apply)'s
/// gyro feed). The variant order/names are an append-only persisted-profile contract.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum GyroMode {
    /// Gyro→mouse disabled (default — inert, no behavior change for non-gyro users).
    #[default]
    Off,
    /// Gyro→mouse runs every report (no activation gate).
    AlwaysOn,
    /// Gyro→mouse runs only while the activation trigger is held.
    TriggerHeld,
    /// Append-only fallback for an unknown persisted mode (degrades to `Off`).
    #[serde(other)]
    Unknown,
}

impl GyroMode {
    /// `true` if gyro→mouse should run this report given whether the activation trigger is held.
    ///
    /// The engine owns reading the activation trigger; this folds the mode + trigger state into the
    /// single "feed the gyro accumulator?" decision. `Off`/`Unknown` are always inert.
    #[inline]
    #[must_use]
    pub fn is_active(self, trigger_held: bool) -> bool {
        match self {
            Self::AlwaysOn => true,
            Self::TriggerHeld => trigger_held,
            Self::Off | Self::Unknown => false,
        }
    }
}

/// Gyro→mouse settings (blueprint §12 M5; ground truth `Hyperion-ds4w/.../MouseCursor.cs::sixaxisMoved`).
///
/// The editable, persisted form; [`GyroSettings::clamped`] enforces the ranges and
/// [`GyroSettings::to_accum_cfg`] projects it into the hot-facing
/// [`GyroAccumCfg`](crate::mouse_accum::GyroAccumCfg) that [`apply`](crate::map::apply) feeds the
/// gyro [`MouseAccumulator`](crate::mouse_accum::MouseAccumulator) via `gyro_velocity_step`.
///
/// Field semantics mirror DS4Windows `GyroMouseSens` / `GyroMouseInfo`:
/// * `sensitivity` — master gyro speed (`gyroSensitivity·0.01` folded with the device coefficient);
///   `1.0` ≈ the C# `gyroSensitivity == 100` default.
/// * `vertical_scale` — `gyroSensVerticalScale·0.01`, scales the vertical (pitch) velocity only.
/// * `deadzone` — `gyroCursorDeadZone` in the **gyro-rate domain** (NOT a normalized stick
///   dead-zone); small tilts below it are suppressed.
/// * `velocity_offset` — `mouseOffset`, the direction-split anti-jitter start offset.
/// * `min_threshold` — per-report motion gate (`1.0` == the always-carry special case).
/// * `jitter_comp` — enable the `^1.408` ease-in below the jitter threshold.
/// * `invert_x` / `invert_y` — negate the final delta per axis (DS4Windows `gyroInvert` bits).
/// * `swap_yaw_roll` — use roll instead of yaw for the horizontal axis (DS4Windows
///   `getGyroMouseHorizontalAxis`); the engine selects which `Motion` rate feeds the X channel.
/// * `mode` — when the gyro→mouse output is active ([`GyroMode`]).
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct GyroSettings {
    /// When the gyro→mouse output runs (default `Off` => inert).
    pub mode: GyroMode,
    /// Master gyro→mouse speed (DS4Windows `gyroSensitivity·0.01`).
    pub sensitivity: f64,
    /// Extra vertical (pitch) velocity scale (DS4Windows `gyroSensVerticalScale·0.01`).
    pub vertical_scale: f64,
    /// Gyro-rate-domain dead-zone (DS4Windows `gyroCursorDeadZone`).
    pub deadzone: f64,
    /// Direction-split anti-jitter offset (DS4Windows `mouseOffset`).
    pub velocity_offset: f64,
    /// Per-report motion gate (`1.0` == always-carry special case).
    pub min_threshold: f64,
    /// Enable the `^1.408` jitter-compensation ease curve.
    pub jitter_comp: bool,
    /// Invert the horizontal (X) output.
    pub invert_x: bool,
    /// Invert the vertical (Y) output.
    pub invert_y: bool,
    /// Use roll (instead of yaw) for the horizontal axis (DS4Windows horizontal-axis swap).
    pub swap_yaw_roll: bool,
}

impl Default for GyroSettings {
    /// Inert defaults: gyro→mouse `Off`, DS4Windows-class tunables for when it is enabled. An
    /// unconfigured profile therefore behaves exactly like the M3/M4 placeholder (no gyro output),
    /// so adding these fields is non-breaking.
    fn default() -> Self {
        Self {
            mode: GyroMode::Off,
            sensitivity: 1.0,
            vertical_scale: 1.0,
            deadzone: GYRO_MOUSE_DEADZONE_DEFAULT,
            velocity_offset: GYRO_MOUSE_OFFSET_DEFAULT,
            min_threshold: 1.0,
            jitter_comp: true,
            invert_x: false,
            invert_y: false,
            swap_yaw_roll: false,
        }
    }
}

impl GyroSettings {
    /// Clamp every field into its valid range (called off the hot path, in `resolve()` / the
    /// config-store funnel). Non-finite inputs collapse to the default before clamping. The mode +
    /// the bool flags pass through unchanged; `min_threshold` floors at the always-carry `1.0`.
    #[inline]
    #[must_use]
    pub fn clamped(self) -> Self {
        let d = Self::default();
        let fin = |v: f64, fallback: f64| if v.is_finite() { v } else { fallback };
        Self {
            mode: self.mode,
            sensitivity: fin(self.sensitivity, d.sensitivity).clamp(0.01, 100.0),
            vertical_scale: fin(self.vertical_scale, d.vertical_scale).clamp(0.1, 10.0),
            deadzone: fin(self.deadzone, d.deadzone).clamp(0.0, 1000.0),
            velocity_offset: fin(self.velocity_offset, d.velocity_offset).clamp(0.0, 10.0),
            min_threshold: fin(self.min_threshold, d.min_threshold).max(1.0),
            jitter_comp: self.jitter_comp,
            invert_x: self.invert_x,
            invert_y: self.invert_y,
            swap_yaw_roll: self.swap_yaw_roll,
        }
    }

    /// Project into the hot-facing [`GyroAccumCfg`] the engine feeds `gyro_velocity_step`. Call after
    /// `clamped()` has run in `resolve()`. The `mode`/`swap_yaw_roll` fields are NOT part of the
    /// accumulator config (the engine consumes them to gate + axis-select the feed); only the
    /// velocity-model tunables flow into [`GyroAccumCfg`].
    #[inline]
    #[must_use]
    pub fn to_accum_cfg(self) -> GyroAccumCfg {
        GyroAccumCfg {
            sensitivity: self.sensitivity,
            vertical_scale: self.vertical_scale,
            velocity_offset: self.velocity_offset,
            deadzone: self.deadzone,
            min_threshold: self.min_threshold,
            jitter_comp: self.jitter_comp,
            invert_x: self.invert_x,
            invert_y: self.invert_y,
        }
    }
}

/// Touchpad→mouse / touch-as-buttons settings (blueprint §12 M6; ground truth
/// `Hyperion-ds4w/.../MouseCursor.cs::TouchMoveCursor` + `DS4Device.cs` touch-region split).
///
/// The editable, persisted form; [`TouchpadSettings::clamped`] enforces the ranges and
/// [`TouchpadSettings::to_accum_cfg`] projects the relative-mouse tunables into the hot-facing
/// [`TouchAccumCfg`](crate::mouse_accum::TouchAccumCfg) that [`apply`](crate::map::apply) feeds the
/// touch [`MouseAccumulator`](crate::mouse_accum::MouseAccumulator) via `touch_step` when a control
/// is bound to `MouseMove(MouseMoveSrc::Touchpad)`.
///
/// * `as_mouse` — master enable for touchpad→relative-mouse. When `false` (the default) a
///   `MouseMove(Touchpad)` binding is inert (no touch motion), so an unconfigured profile is
///   byte-identical to M5.
/// * `as_buttons` — enable the touch finger-region controls (`TouchLeft/Right/Upper/Multi`). When
///   `false` those controls always read released even on a contact; the controls themselves are
///   always decoded into [`ControllerState`](crate::input::ControllerState), this flag only gates
///   whether the engine treats them as live. (The hot path reads this on the resolved profile.)
/// * `sensitivity` — `getTouchSensitivity·0.01` master coefficient.
/// * `velocity_offset` — `TOUCHPAD_MOUSE_OFFSET` direction-split anti-jitter offset.
/// * `min_threshold` — per-report motion gate (`1.0` == always-carry special case).
/// * `jitter_comp` — enable the `^1.408` ease below the touch jitter threshold.
/// * `invert_x` / `invert_y` — negate the final delta per axis.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct TouchpadSettings {
    /// Master enable for touchpad→relative-mouse (default `false` => inert).
    pub as_mouse: bool,
    /// Enable the touch finger-region controls (`TouchLeft/Right/Upper/Multi`).
    pub as_buttons: bool,
    /// Master touch→mouse coefficient (`getTouchSensitivity·0.01`).
    pub sensitivity: f64,
    /// Direction-split anti-jitter offset (`TOUCHPAD_MOUSE_OFFSET`).
    pub velocity_offset: f64,
    /// Per-report motion gate (`1.0` == always-carry special case).
    pub min_threshold: f64,
    /// Enable the `^1.408` jitter-compensation ease curve.
    pub jitter_comp: bool,
    /// Invert the horizontal (X) output.
    pub invert_x: bool,
    /// Invert the vertical (Y) output.
    pub invert_y: bool,
}

impl Default for TouchpadSettings {
    /// Inert defaults: touch→mouse `as_mouse = false`, finger-region controls `as_buttons = true`
    /// (decoded but only meaningful when a touch-region control is bound), DS4Windows-class
    /// relative-mouse tunables for when it is enabled. An unconfigured profile therefore produces
    /// no touch mouse motion, so adding these fields is non-breaking.
    fn default() -> Self {
        Self {
            as_mouse: false,
            as_buttons: true,
            sensitivity: 1.0,
            velocity_offset: TOUCH_MOUSE_OFFSET_DEFAULT,
            min_threshold: 1.0,
            jitter_comp: true,
            invert_x: false,
            invert_y: false,
        }
    }
}

impl TouchpadSettings {
    /// Clamp every field into its valid range (called off the hot path, in `resolve()`). Non-finite
    /// inputs collapse to the default before clamping; the bool flags pass through; `min_threshold`
    /// floors at the always-carry `1.0`.
    #[inline]
    #[must_use]
    pub fn clamped(self) -> Self {
        let d = Self::default();
        let fin = |v: f64, fallback: f64| if v.is_finite() { v } else { fallback };
        Self {
            as_mouse: self.as_mouse,
            as_buttons: self.as_buttons,
            sensitivity: fin(self.sensitivity, d.sensitivity).clamp(0.01, 100.0),
            velocity_offset: fin(self.velocity_offset, d.velocity_offset).clamp(0.0, 10.0),
            min_threshold: fin(self.min_threshold, d.min_threshold).max(1.0),
            jitter_comp: self.jitter_comp,
            invert_x: self.invert_x,
            invert_y: self.invert_y,
        }
    }

    /// Project into the hot-facing [`TouchAccumCfg`] the engine feeds `touch_step`. Call after
    /// `clamped()` has run in `resolve()`. The `as_mouse`/`as_buttons` gates are NOT part of the
    /// accumulator config (the engine reads them to gate the feed / the region controls); only the
    /// relative-mouse tunables flow into [`TouchAccumCfg`].
    #[inline]
    #[must_use]
    pub fn to_accum_cfg(self) -> TouchAccumCfg {
        TouchAccumCfg {
            sensitivity: self.sensitivity,
            velocity_offset: self.velocity_offset,
            min_threshold: self.min_threshold,
            jitter_comp: self.jitter_comp,
            invert_x: self.invert_x,
            invert_y: self.invert_y,
        }
    }
}

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
            // M5: clamped gyro→mouse settings (consumed by `apply()` via `to_accum_cfg` when
            // `gyro.mode` is active).
            gyro: self.gyro.clamped(),
            // M6: clamped touchpad→mouse / as-buttons settings (consumed by `apply()` when a
            // `MouseMove(Touchpad)` binding is active and `touchpad.as_mouse` is set).
            touchpad: self.touchpad.clamped(),
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
/// and the `macros` table (consumed off the hot path by the injector thread). M5 adds the resolved
/// `gyro` settings (consumed by `apply()` via [`GyroSettings::to_accum_cfg`] when its mode is
/// active). The per-control shift table is read from `base` each report; special resolution is
/// handled via the `Special` binding edge.
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
    /// Resolved (clamped) gyro→mouse settings (M5); `apply()` calls
    /// [`to_accum_cfg`](GyroSettings::to_accum_cfg) to feed the gyro accumulator when
    /// [`GyroSettings::mode`] is active. `Copy`, read inline on the hot path.
    pub gyro: GyroSettings,
    /// Resolved (clamped) touchpad→mouse / as-buttons settings (M6); `apply()` calls
    /// [`to_accum_cfg`](TouchpadSettings::to_accum_cfg) to feed the touch accumulator when a
    /// `MouseMove(Touchpad)` binding is active and [`TouchpadSettings::as_mouse`] is set. `Copy`,
    /// read inline on the hot path.
    pub touchpad: TouchpadSettings,
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
            gyro: GyroSettings::default(),
            touchpad: TouchpadSettings::default(),
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

    // -------------------------------------- M5 gyro settings -------------------------------------

    #[test]
    fn gyro_settings_default_is_inert_off() {
        // The default gyro mode is Off so an unconfigured profile produces no gyro output (no
        // behavior change for non-gyro users).
        let g = GyroSettings::default();
        assert_eq!(g.mode, GyroMode::Off);
        assert!(
            !g.mode.is_active(true),
            "Off is inert even when trigger held"
        );
        assert!(!g.mode.is_active(false));
    }

    #[test]
    fn gyro_mode_activation() {
        assert!(GyroMode::AlwaysOn.is_active(false));
        assert!(GyroMode::AlwaysOn.is_active(true));
        assert!(!GyroMode::TriggerHeld.is_active(false));
        assert!(GyroMode::TriggerHeld.is_active(true));
        assert!(!GyroMode::Off.is_active(true));
        assert!(!GyroMode::Unknown.is_active(true));
    }

    #[test]
    fn gyro_settings_clamp_bounds_and_non_finite() {
        let wild = GyroSettings {
            mode: GyroMode::AlwaysOn,
            sensitivity: 9_999.0,
            vertical_scale: 0.0,    // below the 0.1 floor
            deadzone: -5.0,         // below 0
            velocity_offset: 999.0, // above 10
            min_threshold: 0.0,     // below the 1.0 floor
            jitter_comp: false,
            invert_x: true,
            invert_y: true,
            swap_yaw_roll: true,
        };
        let c = wild.clamped();
        assert_eq!(c.mode, GyroMode::AlwaysOn, "mode passes through clamp");
        assert_eq!(c.sensitivity, 100.0);
        assert_eq!(c.vertical_scale, 0.1);
        assert_eq!(c.deadzone, 0.0);
        assert_eq!(c.velocity_offset, 10.0);
        assert_eq!(
            c.min_threshold, 1.0,
            "min_threshold floors at the always-carry 1.0"
        );
        assert!(!c.jitter_comp && c.invert_x && c.invert_y && c.swap_yaw_roll);

        // Non-finite inputs collapse to the default before clamping.
        let nan = GyroSettings {
            sensitivity: f64::NAN,
            min_threshold: f64::INFINITY,
            ..GyroSettings::default()
        }
        .clamped();
        assert_eq!(nan.sensitivity, GyroSettings::default().sensitivity);
        assert!(nan.min_threshold.is_finite());
    }

    #[test]
    fn gyro_settings_to_accum_cfg_carries_velocity_tunables() {
        let g = GyroSettings {
            mode: GyroMode::TriggerHeld, // not part of the accum cfg
            sensitivity: 30.0,
            vertical_scale: 1.5,
            deadzone: 8.0,
            velocity_offset: 0.2,
            min_threshold: 2.0,
            jitter_comp: false,
            invert_x: true,
            invert_y: false,
            swap_yaw_roll: true, // not part of the accum cfg
        };
        let cfg = g.to_accum_cfg();
        assert_eq!(cfg.sensitivity, 30.0);
        assert_eq!(cfg.vertical_scale, 1.5);
        assert_eq!(cfg.deadzone, 8.0);
        assert_eq!(cfg.velocity_offset, 0.2);
        assert_eq!(cfg.min_threshold, 2.0);
        assert!(!cfg.jitter_comp);
        assert!(cfg.invert_x && !cfg.invert_y);
    }

    #[test]
    fn resolve_clamps_gyro_and_default_is_off() {
        let p = Profile {
            gyro: GyroSettings {
                mode: GyroMode::AlwaysOn,
                sensitivity: -3.0, // clamps up to the 0.01 floor
                ..GyroSettings::default()
            },
            ..Profile::default()
        };
        let rp = p.resolve();
        assert_eq!(rp.gyro.mode, GyroMode::AlwaysOn);
        assert_eq!(rp.gyro.sensitivity, 0.01, "resolve clamps gyro settings");

        // A default profile resolves to inert (Off) gyro.
        assert_eq!(ResolvedProfile::default().gyro.mode, GyroMode::Off);
    }

    #[test]
    fn profile_with_gyro_round_trips() {
        let p = Profile {
            name: "gyro-test".to_string(),
            gyro: GyroSettings {
                mode: GyroMode::TriggerHeld,
                sensitivity: 45.0,
                invert_y: true,
                swap_yaw_roll: true,
                ..GyroSettings::default()
            },
            ..Default::default()
        };
        let toml = toml::to_string(&p).unwrap();
        let back: Profile = toml::from_str(&toml).unwrap();
        assert_eq!(back, p);
    }

    #[test]
    fn gyro_mode_unknown_serde_fallback() {
        // An unrecognized persisted mode degrades to Unknown (inert), not a load failure.
        #[derive(serde::Deserialize)]
        struct Wrap {
            mode: GyroMode,
        }
        let back: Wrap = toml::from_str("mode = \"SomeFutureMode\"\n").unwrap();
        assert_eq!(back.mode, GyroMode::Unknown);
        assert!(!back.mode.is_active(true));
    }

    // ------------------------------------ M6 touchpad settings -----------------------------------

    #[test]
    fn touchpad_settings_default_is_inert_off() {
        // Default: as_mouse off (no touch motion for an unconfigured profile), as_buttons on.
        let t = TouchpadSettings::default();
        assert!(!t.as_mouse, "touch-as-mouse off by default");
        assert!(t.as_buttons);
        assert_eq!(t.to_accum_cfg(), TouchAccumCfg::default());
    }

    #[test]
    fn touchpad_settings_clamp_bounds_and_non_finite() {
        let wild = TouchpadSettings {
            as_mouse: true,
            as_buttons: false,
            sensitivity: 9_999.0,
            velocity_offset: 999.0, // above 10
            min_threshold: 0.0,     // below the 1.0 floor
            jitter_comp: false,
            invert_x: true,
            invert_y: true,
        };
        let c = wild.clamped();
        assert!(c.as_mouse && !c.as_buttons, "flags pass through clamp");
        assert_eq!(c.sensitivity, 100.0);
        assert_eq!(c.velocity_offset, 10.0);
        assert_eq!(
            c.min_threshold, 1.0,
            "min_threshold floors at always-carry 1.0"
        );
        assert!(!c.jitter_comp && c.invert_x && c.invert_y);

        // Non-finite collapses to default before clamping.
        let nan = TouchpadSettings {
            sensitivity: f64::NAN,
            min_threshold: f64::INFINITY,
            ..TouchpadSettings::default()
        }
        .clamped();
        assert_eq!(nan.sensitivity, TouchpadSettings::default().sensitivity);
        assert!(nan.min_threshold.is_finite());
    }

    #[test]
    fn resolve_clamps_touchpad_and_default_is_off() {
        let p = Profile {
            touchpad: TouchpadSettings {
                as_mouse: true,
                sensitivity: -3.0, // clamps up to the 0.01 floor
                ..TouchpadSettings::default()
            },
            ..Profile::default()
        };
        let rp = p.resolve();
        assert!(rp.touchpad.as_mouse);
        assert_eq!(
            rp.touchpad.sensitivity, 0.01,
            "resolve clamps touchpad settings"
        );
        assert!(!ResolvedProfile::default().touchpad.as_mouse);
    }

    #[test]
    fn profile_with_touchpad_round_trips() {
        let p = Profile {
            name: "touch-test".to_string(),
            touchpad: TouchpadSettings {
                as_mouse: true,
                sensitivity: 60.0,
                invert_y: true,
                ..TouchpadSettings::default()
            },
            ..Default::default()
        };
        let toml = toml::to_string(&p).unwrap();
        let back: Profile = toml::from_str(&toml).unwrap();
        assert_eq!(back, p);
    }
}
