//! Binding data model — the closed, append-only `BindTarget` enum plus the per-control slot
//! ([`BindingSlot`]) that carries the base bind, the per-control shift trigger + shift bind, and
//! the (net-new) turbo config.
//!
//! This is the editable, serde-facing surface (`Profile` stores [`BindingSlot`]s keyed by
//! `Control`; [`crate::map::ResolvedProfile`] flattens them into a fixed array). Every variant is
//! `#[serde(...)]`-tagged and **append-only**: the discriminant order and names are a persisted
//! profile contract, so never renumber or rename. Unknown serde forms fall back via
//! `#[serde(other)]` so an old binary reading a newer profile degrades to `Unbound`/`Default`
//! instead of failing the whole load.
//!
//! M3 note: the full enum exists so M4 (shift/turbo/macro/mouse) is purely additive, but the M3
//! [`apply`](crate::map::apply) only *resolves* `Passthrough`, `GamepadButton`, and `Key` —
//! everything else is treated as a no-op (see `engine.rs`).

use crate::output::MouseButton;

/// How a key event is injected (ported from DS4Windows `DS4KeyType`).
///
/// `Toggle` flips a latched state on each rising edge; `ScanCode` selects hardware scancode
/// injection (game-compatible) over the virtual-key path. The flags compose — a binding can be
/// both `ScanCode` and `Toggle`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct KeyKind {
    /// Inject via hardware scancode (`KEYEVENTF_SCANCODE`) rather than the virtual key.
    #[serde(default)]
    pub scan_code: bool,
    /// Toggle (press once to latch on, again to latch off) rather than hold-while-pressed.
    #[serde(default)]
    pub toggle: bool,
}

impl KeyKind {
    /// A plain hold-while-pressed virtual-key binding.
    pub const HOLD: Self = Self {
        scan_code: false,
        toggle: false,
    };
}

/// Virtual-pad buttons a binding can drive (the `GamepadButton` payload).
///
/// Maps onto the [`crate::output::PadButtons`] bitfield via [`PadBtn::bit`]; kept as a small
/// closed enum (not a raw mask) so it stays serde-stable and append-only.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum PadBtn {
    A,
    B,
    X,
    Y,
    Lb,
    Rb,
    Back,
    Start,
    Ls,
    Rs,
    Guide,
    DpadUp,
    DpadDown,
    DpadLeft,
    DpadRight,
    L2Click,
    R2Click,
    Touchpad,
    /// Append-only fallback so an unknown future button name degrades instead of failing the load.
    #[serde(other)]
    Unknown,
}

impl PadBtn {
    /// The [`crate::output::PadButtons`] bit-mask this button sets, or `0` for `Unknown`.
    #[inline]
    pub const fn bit(self) -> u32 {
        use crate::output::PadButtons as P;
        match self {
            Self::A => P::A,
            Self::B => P::B,
            Self::X => P::X,
            Self::Y => P::Y,
            Self::Lb => P::LB,
            Self::Rb => P::RB,
            Self::Back => P::BACK,
            Self::Start => P::START,
            Self::Ls => P::LS,
            Self::Rs => P::RS,
            Self::Guide => P::GUIDE,
            Self::DpadUp => P::DPAD_UP,
            Self::DpadDown => P::DPAD_DOWN,
            Self::DpadLeft => P::DPAD_LEFT,
            Self::DpadRight => P::DPAD_RIGHT,
            Self::L2Click => P::L2_CLICK,
            Self::R2Click => P::R2_CLICK,
            Self::Touchpad => P::TOUCHPAD,
            Self::Unknown => 0,
        }
    }
}

/// Which virtual-pad analog axis a `GamepadAxis` binding targets.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum GamepadAxis {
    Lx,
    Ly,
    Rx,
    Ry,
    Lt,
    Rt,
    /// Append-only fallback.
    #[serde(other)]
    Unknown,
}

/// Direction half a `GamepadAxis` binding drives (a digital control pushes the axis to one end).
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum AxisDir {
    /// Drive toward the negative end (left / down / release-side).
    Neg,
    /// Drive toward the positive end (right / up).
    Pos,
}

/// Which raw source feeds a mouse-move binding (stick→mouse / gyro→mouse / touchpad→mouse).
/// M4/M5/M6 consumer. **Append-only**: `Touchpad` was added in M6 before the `Unknown` fallback,
/// so the serde tag order/names stay a stable persisted contract.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum MouseMoveSrc {
    LeftStick,
    RightStick,
    Gyro,
    /// Touchpad finger drag → relative mouse (M6).
    Touchpad,
    /// Append-only fallback.
    #[serde(other)]
    Unknown,
}

/// Mouse-wheel direction for a `MouseWheel` binding. M4 consumer.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum WheelDir {
    Up,
    Down,
    Left,
    Right,
    /// Append-only fallback.
    #[serde(other)]
    Unknown,
}

/// A shift-layer trigger: the `Control` whose pressed state activates the per-control shift bind.
///
/// Per the faithful DS4Windows shift model (blueprint §5 step 2 / conflict 4), shift is
/// **per-control** — each [`BindingSlot`] carries its own `shift_trigger` + `shift_bind`, read
/// against RAW digitized state only. M3 stores this but does not yet resolve it (M4).
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct ShiftTrigger {
    /// The control that must read pressed for the shift bind to apply.
    pub control: crate::input::Control,
}

/// Net-new (no DS4Windows reference) per-binding turbo / rapid-fire config (blueprint §5).
///
/// Duty cycle is expressed as a `num/den` fraction with an integer `period_us` so the hot-path
/// gate has no float. M3 carries it but the gate runs in M4.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct TurboCfg {
    /// Full ON+OFF cycle length, microseconds.
    pub period_us: u32,
    /// Duty numerator (ON fraction = `duty_num / duty_den`).
    pub duty_num: u16,
    /// Duty denominator.
    pub duty_den: u16,
}

impl Default for TurboCfg {
    /// A 50% duty, 100 ms cycle — a sane visible rapid-fire default.
    fn default() -> Self {
        Self {
            period_us: 100_000,
            duty_num: 1,
            duty_den: 2,
        }
    }
}

/// The closed, append-only set of things a control can be bound to (blueprint §3).
///
/// Variant order and names are a persisted-profile contract: **append only**, never renumber.
/// `#[serde(other)]` routes any unrecognized tag to [`BindTarget::Unbound`] so a newer profile
/// loaded by an older binary degrades gracefully.
///
/// M3 [`apply`](crate::map::apply) resolves only `Passthrough`, `GamepadButton`, and `Key`; every
/// other variant is a documented no-op until its milestone (`TODO(M4)` / `TODO(M5)` in `engine.rs`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
#[serde(tag = "kind", content = "val")]
pub enum BindTarget {
    /// Identity: copy the physical control through (continuous for axes/triggers, digital for
    /// buttons), subject to half-axis identity suppression. The default for an unconfigured slot.
    #[default]
    Passthrough,
    /// Explicitly unbound: emit nothing AND suppress the identity passthrough (DS4Windows
    /// `ActionType.Default` with an empty action == still mapped, vs a real unbind).
    Unbound,
    /// Drive a virtual-pad button.
    GamepadButton(PadBtn),
    /// Drive a virtual-pad analog axis toward one end. `full` forces a full-deflection push for a
    /// digital source (vs a scaled push). M4 consumer.
    GamepadAxis {
        axis: GamepadAxis,
        dir: AxisDir,
        #[serde(default)]
        full: bool,
    },
    /// Virtual-pad touchpad click (verifier FIX 5).
    TouchpadClick,
    /// Inject a keyboard key (virtual-key code + injection kind). M3 resolved.
    Key { vk: u16, kind: KeyKind },
    /// A key binding explicitly set to "no key" (DS4Windows `DS4KeyType.Unbound`): suppress the
    /// identity, emit nothing (verifier FIX 5). M3 treats it like `Unbound` for suppression.
    KeyUnbound,
    /// Inject a mouse button edge. M4 consumer.
    Mouse(MouseButton),
    /// Feed a mouse-move accumulator (stick/gyro → relative mouse). M4/M5 consumer.
    MouseMove(MouseMoveSrc),
    /// Emit mouse-wheel notches. M4 consumer.
    MouseWheel(WheelDir),
    /// Trigger a macro by id. M4 consumer.
    Macro(u16),
    /// This control acts as a shift trigger only (no direct output). M4 consumer.
    Shift(ShiftTrigger),
    /// A special action (profile switch / launch / disconnect) routed to the control plane. M4/M5.
    Special(u16),
    /// Append-only fallback for an unknown persisted tag — treated exactly like `Unbound`.
    #[serde(other)]
    Unknown,
}

/// Alias kept for the blueprint's `OutputBinding` naming; the binding target type is [`BindTarget`].
pub type OutputBinding = BindTarget;

impl BindTarget {
    /// `true` if this target suppresses the identity passthrough for its control (everything that
    /// is not an actual `Passthrough`). Used to build the half-axis `axis_remapped[4]` pass.
    #[inline]
    pub const fn suppresses_identity(self) -> bool {
        !matches!(self, Self::Passthrough)
    }
}

/// The per-control mapping slot (blueprint §5 step 2): a base bind plus the per-control shift
/// trigger + shift bind plus optional turbo.
///
/// Stored editable in `Profile` (keyed by `Control`) and copied verbatim into the hot-facing
/// [`crate::map::ResolvedProfile`] fixed array. `Copy` so the resolved array is a cheap flat blob.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct BindingSlot {
    /// Base binding (when no shift trigger is active).
    #[serde(default)]
    pub bind: BindTarget,
    /// Per-control shift trigger; when its control reads pressed, `shift_bind` applies. M4 consumer.
    #[serde(default)]
    pub shift_trigger: Option<ShiftTrigger>,
    /// Binding used while `shift_trigger` is active. M4 consumer.
    #[serde(default)]
    pub shift_bind: BindTarget,
    /// Optional per-binding turbo / rapid-fire. M4 consumer.
    #[serde(default)]
    pub turbo: Option<TurboCfg>,
}

impl Default for BindingSlot {
    /// The neutral slot: pure identity passthrough, no shift, no turbo.
    fn default() -> Self {
        Self {
            bind: BindTarget::Passthrough,
            shift_trigger: None,
            shift_bind: BindTarget::Passthrough,
            turbo: None,
        }
    }
}

impl BindingSlot {
    /// A slot whose base bind is `bind`, everything else neutral.
    #[inline]
    pub const fn from_bind(bind: BindTarget) -> Self {
        Self {
            bind,
            shift_trigger: None,
            shift_bind: BindTarget::Passthrough,
            turbo: None,
        }
    }
}

/// Alias the blueprint also refers to as `Binding`.
pub type Binding = BindingSlot;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::output::PadButtons;

    #[test]
    fn padbtn_bit_matches_padbuttons_masks() {
        assert_eq!(PadBtn::A.bit(), PadButtons::A);
        assert_eq!(PadBtn::B.bit(), PadButtons::B);
        assert_eq!(PadBtn::Touchpad.bit(), PadButtons::TOUCHPAD);
        assert_eq!(PadBtn::Unknown.bit(), 0);
    }

    #[test]
    fn default_slot_is_passthrough() {
        let s = BindingSlot::default();
        assert_eq!(s.bind, BindTarget::Passthrough);
        assert!(s.shift_trigger.is_none());
        assert!(s.turbo.is_none());
        assert!(!s.bind.suppresses_identity());
    }

    #[test]
    fn unbound_and_key_suppress_identity() {
        assert!(BindTarget::Unbound.suppresses_identity());
        assert!(BindTarget::KeyUnbound.suppresses_identity());
        assert!(BindTarget::GamepadButton(PadBtn::B).suppresses_identity());
        assert!(BindTarget::Key {
            vk: 0x41,
            kind: KeyKind::HOLD
        }
        .suppresses_identity());
        assert!(!BindTarget::Passthrough.suppresses_identity());
    }

    #[test]
    fn bindtarget_serde_roundtrip_and_unknown_fallback() {
        // Round-trip a representative tagged variant.
        let b = BindTarget::Key {
            vk: 0x41,
            kind: KeyKind {
                scan_code: true,
                toggle: false,
            },
        };
        let toml = toml::to_string(&Wrap { t: b }).unwrap();
        let back: Wrap = toml::from_str(&toml).unwrap();
        assert_eq!(back.t, b);

        // Unknown tag degrades to Unknown (treated as Unbound).
        let back: Wrap = toml::from_str("[t]\nkind = \"SomeFutureKind\"\n").unwrap();
        assert_eq!(back.t, BindTarget::Unknown);
    }

    #[derive(serde::Serialize, serde::Deserialize)]
    struct Wrap {
        t: BindTarget,
    }
}
