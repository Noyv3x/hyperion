//! The pure mapping engine — [`apply`] and its resident [`MapState`].
//!
//! [`apply`] is the single remap entry point: pure, alloc-free, no I/O, deterministic. It reads a
//! decoded [`ControllerState`], resolves every `Control` against an immutable
//! [`ResolvedProfile`](crate::map::ResolvedProfile), composes an [`OutputState`], and queues
//! keyboard/mouse edges into a [`KbmBatch`]. It runs inline on the hot loop exactly where the
//! single `RcFilter` step ran (blueprint §7.2).
//!
//! ## M3 subset
//! M3 implements the spine: per-kind **digitize** (thresholds 55/127 vs 100/255 via
//! [`ControllerState::pressed`]), **half-axis identity suppression** (the `axis_remapped[4]` pass
//! — verifier FIX 1), and per-control resolve for **`Passthrough` + `GamepadButton` + `Key`**
//! (button→button, button→key with an edge/toggle vk-keyed latch — verifier FIX 6). Every other
//! `BindTarget` is a clearly-marked `TODO(M4)`/`TODO(M5)` no-op arm (NO `todo!()`/panic) so M4 is
//! additive. Shift/turbo are read off the resolved slot but not yet applied in M3.

use crate::input::{Control, ControllerState};
use crate::map::binding::{BindTarget, KeyKind};
use crate::map::profile::ResolvedProfile;
use crate::output::{KbmBatch, KbmEvent, KeyKind as InjectKind, OutputState};

/// Fixed-capacity vk→bool latch (verifier FIX 6) — alloc-free, `Copy`.
///
/// DS4Windows keys its `pressedonce[]` / `toggle` state on the **output key value** (vk), so two
/// controls bound to the same vk share one latch. We reproduce that with a tiny open-addressed
/// fixed-cap table: insert is O(cap) worst case but cap is small and the common case is a handful
/// of held keys. On overflow the latch silently no-ops (saturate, never alloc/panic) — at that
/// point far more keys are held than a HID report can carry anyway.
#[derive(Clone, Copy, Debug)]
pub struct VkLatch {
    keys: [u16; Self::CAP],
    vals: [bool; Self::CAP],
    /// Number of live entries.
    len: u8,
}

impl VkLatch {
    /// Distinct vks tracked at once. Sized well above the 6-key HID hold limit + shift churn.
    pub const CAP: usize = 16;

    /// An empty latch (clean post-reset state).
    pub const fn new() -> Self {
        Self {
            keys: [0; Self::CAP],
            vals: [false; Self::CAP],
            len: 0,
        }
    }

    /// Index of `vk` if present.
    #[inline]
    fn find(&self, vk: u16) -> Option<usize> {
        let n = self.len as usize;
        let mut i = 0;
        while i < n {
            if self.keys[i] == vk {
                return Some(i);
            }
            i += 1;
        }
        None
    }

    /// Current latched value for `vk` (default `false` if untracked).
    #[inline]
    pub fn get(&self, vk: u16) -> bool {
        match self.find(vk) {
            Some(i) => self.vals[i],
            None => false,
        }
    }

    /// Set `vk`'s latched value, inserting a slot if needed. Silently no-ops on overflow.
    #[inline]
    pub fn set(&mut self, vk: u16, on: bool) {
        if let Some(i) = self.find(vk) {
            self.vals[i] = on;
            return;
        }
        let n = self.len as usize;
        if n < Self::CAP {
            self.keys[n] = vk;
            self.vals[n] = on;
            self.len = n as u8 + 1;
        }
        // overflow: drop silently (bounded, deterministic, alloc-free).
    }

    /// Flip `vk`'s latched value and return the new value (inserting `true` if untracked).
    #[inline]
    pub fn toggle(&mut self, vk: u16) -> bool {
        let new = !self.get(vk);
        self.set(vk, new);
        new
    }
}

impl Default for VkLatch {
    fn default() -> Self {
        Self::new()
    }
}

/// Per-binding turbo phase state (net-new; gate runs in M4). Verifier FIX 2/6 — `Default` is the
/// clean post-reset state.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TurboState {
    /// Was the source active on the previous report (rising-edge detect).
    pub was_active: bool,
    /// `now_us` of the current cycle's anchor (reset on each rising edge).
    pub cycle_start_us: u64,
}

/// M3 placeholder for the stick/gyro mouse remainder-carry accumulator (real impl lands in
/// `core/src/mouse_accum.rs` for M4/M5 per blueprint §6.2). Carried in [`MapState`] now so the
/// field layout is stable and M4 is additive; inert in M3.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MouseAccumulator {
    h_remainder: f64,
    v_remainder: f64,
}

impl MouseAccumulator {
    /// Clear the carried remainder.
    #[inline]
    pub fn reset(&mut self) {
        self.h_remainder = 0.0;
        self.v_remainder = 0.0;
    }
}

/// Resident mapping state — all `Copy`, all fixed-size arrays, **no heap on the hot path**
/// (blueprint §5). `Default` is the clean post-reset state (verifier FIX 6): the engine's
/// `ResetFilter` command sets `*ms = MapState::default()`.
#[derive(Clone, Copy, Debug)]
pub struct MapState {
    /// Per-control turbo phase (M4 consumer).
    pub turbo: [TurboState; Control::COUNT],
    /// Per-control "was on last report" for edge detection.
    pub prev_active: [bool; Control::COUNT],
    /// Stick→mouse remainder carry (M4 consumer).
    pub stick_mouse: MouseAccumulator,
    /// Gyro→mouse remainder carry (M5 consumer).
    pub gyro_mouse: MouseAccumulator,
    /// Mouse-wheel notch remainder (M4 consumer).
    pub wheel_remainder: f64,
    /// vk→toggle-latched-value (verifier FIX 6).
    pub toggle: VkLatch,
    /// vk→pressed-once edge guard (verifier FIX 6).
    pub pressed_once: VkLatch,
}

impl Default for MapState {
    fn default() -> Self {
        Self {
            turbo: [TurboState::default(); Control::COUNT],
            prev_active: [false; Control::COUNT],
            stick_mouse: MouseAccumulator::default(),
            gyro_mouse: MouseAccumulator::default(),
            wheel_remainder: 0.0,
            toggle: VkLatch::new(),
            pressed_once: VkLatch::new(),
        }
    }
}

/// Write a half-axis identity passthrough into `out` for the given stick `axis`.
///
/// Called only when `!axis_remapped[axis]` (verifier FIX 1 — `ResetToDefaultValue` zeroes BOTH
/// halves of an axis if either is remapped). The full continuous stick value is the signed `state`
/// component; writing it for either half of the pair is idempotent (both halves of a pair map to
/// the same axis value), so the second half's Passthrough arm is a harmless re-write of the same
/// value.
#[inline]
fn passthrough_axis(out: &mut OutputState, state: &ControllerState, axis: usize) {
    match axis {
        0 => out.lx = state.lx,
        1 => out.ly = state.ly,
        2 => out.rx = state.rx,
        _ => out.ry = state.ry,
    }
}

/// Resolve a `Key` binding: edge/toggle latch keyed on the **output vk** (verifier FIX 6), pushing
/// `KbmEvent::Key{down}` edges into `batch`. Hold emits nothing after the first report.
#[inline]
fn resolve_key(
    vk: u16,
    kind: KeyKind,
    on: bool,
    prev_on: bool,
    ms: &mut MapState,
    batch: &mut KbmBatch,
) {
    // The mapping-layer `KeyKind` (scan_code + toggle) selects injection method; the toggle bit is
    // handled below, and the KBM injector only needs ScanCode vs Virtual.
    let inject = if kind.scan_code {
        InjectKind::ScanCode
    } else {
        InjectKind::Virtual
    };
    if kind.toggle {
        // Toggle: flip the latched value on the rising edge only; emit the new state as an edge.
        if on && !ms.pressed_once.get(vk) {
            let now = ms.toggle.toggle(vk);
            ms.pressed_once.set(vk, true);
            batch.push(KbmEvent::Key {
                vk,
                down: now,
                kind: inject,
            });
        } else if !on {
            ms.pressed_once.set(vk, false);
        }
    } else {
        // Hold: KeyDown on rising edge, KeyUp on falling edge, nothing while held.
        if on && !prev_on {
            batch.push(KbmEvent::Key {
                vk,
                down: true,
                kind: inject,
            });
        } else if !on && prev_on {
            batch.push(KbmEvent::Key {
                vk,
                down: false,
                kind: inject,
            });
        }
    }
}

/// Pure, alloc-free, no-I/O remap entry point (blueprint §5).
///
/// Resolves every `Control` against `rp`, composing an [`OutputState`] and queueing KBM edges.
/// `now_us` is the hot loop's monotonic time (reused from `busy_start/1000`).
///
/// M3: digitize + half-axis identity suppression + `Passthrough`/`GamepadButton`/`Key`. Other
/// `BindTarget`s are `TODO` no-ops.
pub fn apply(
    state: &ControllerState,
    rp: &ResolvedProfile,
    ms: &mut MapState,
    now_us: u64,
) -> (OutputState, KbmBatch) {
    let _ = now_us; // turbo (the only now_us consumer) lands in M4.
    let t = &rp.thresholds;

    // --- Step 1: digitize once into a resident bool array (kind-dependent thresholds). -----------
    let mut active_raw = [false; Control::COUNT];
    for c in Control::ALL {
        active_raw[c.as_index()] = state.pressed(c, t);
    }

    // --- Step 2: per-control shift selection is M4; in M3 the effective bind is always the base. --
    // (We still read `slot.shift_trigger`/`shift_bind` off the resolved slot in M4 additively.)

    // --- Step 3: half-axis identity suppression pass (verifier FIX 1). ---------------------------
    // For each stick half-axis whose effective bind is not Passthrough, mark its axis so BOTH
    // halves' Passthrough arms suppress the identity (matching ResetToDefaultValue zeroing the pair).
    let mut axis_remapped = [false; 4];
    for c in Control::ALL {
        if let Some(axis) = c.stick_axis() {
            if rp.slot(c).bind.suppresses_identity() {
                axis_remapped[axis] = true;
            }
        }
    }

    // --- Step 4: per-control resolve. -----------------------------------------------------------
    let mut out = OutputState::default();
    let mut batch = KbmBatch::new();

    for c in Control::ALL {
        let idx = c.as_index();
        let slot = rp.slot(c);
        // M3: effective bind is the base bind (shift resolution is M4).
        let bind = slot.bind;

        // M3: turbo gate is M4; the raw active bit is the effective `on`.
        let on = active_raw[idx];

        match bind {
            BindTarget::Passthrough => {
                resolve_passthrough(&mut out, state, c, on, &axis_remapped);
            }
            BindTarget::GamepadButton(b) => {
                if on {
                    out.buttons.set(b.bit(), true);
                }
            }
            BindTarget::Key { vk, kind } => {
                resolve_key(vk, kind, on, ms.prev_active[idx], ms, &mut batch);
            }
            // Identity-suppressing no-output binds: emit nothing (the axis pass already zeroed the
            // pair; button identity is only ever written in the Passthrough arm, so it's suppressed
            // for free). KeyUnbound == an explicit "no key" (DS4KeyType.Unbound, verifier FIX 5).
            BindTarget::Unbound | BindTarget::KeyUnbound | BindTarget::Unknown => {}

            // ---- TODO(M4): shift/turbo are read above; these output binds land in M4. ----
            BindTarget::GamepadAxis { .. } => { /* TODO(M4): digital→axis push */ }
            BindTarget::Mouse(_) => { /* TODO(M4): mouse-button edge */ }
            BindTarget::MouseMove(_) => { /* TODO(M4): stick/gyro→mouse via MouseAccumulator */ }
            BindTarget::MouseWheel(_) => { /* TODO(M4): wheel notch from wheel_remainder */ }
            BindTarget::Macro(_) => { /* TODO(M4): macro start/stop edge */ }
            BindTarget::Shift(_) => { /* TODO(M4): shift trigger has no direct output */ }
            BindTarget::TouchpadClick => { /* TODO(M4): out.buttons.set(TOUCHPAD, on) */ }
            BindTarget::Special(_) => { /* TODO(M4/M5): control-plane edge */ }
        }

        ms.prev_active[idx] = on;
    }

    (out, batch)
}

/// The `Passthrough` arm: copy the physical control through, with half-axis identity suppression.
#[inline]
fn resolve_passthrough(
    out: &mut OutputState,
    state: &ControllerState,
    c: Control,
    on: bool,
    axis_remapped: &[bool; 4],
) {
    use crate::input::ControlKind;
    match c.kind() {
        ControlKind::AxisDir => {
            if let Some(axis) = c.stick_axis() {
                if !axis_remapped[axis] {
                    passthrough_axis(out, state, axis);
                }
            }
        }
        ControlKind::Trigger => {
            // Continuous trigger passthrough (analog), routed by which trigger this control is.
            let v = state.analog(c);
            match c {
                Control::L2 => out.lt = v,
                Control::R2 => out.rt = v,
                _ => {}
            }
        }
        ControlKind::Button | ControlKind::Touch | ControlKind::GyroDir => {
            // Digital identity: set the corresponding pad bit when pressed. Identity suppression
            // for buttons "falls out for free" — a remapped button never enters this arm.
            if on {
                if let Some(bit) = button_pad_bit(c) {
                    out.buttons.set(bit, true);
                }
            }
        }
    }
}

/// The default identity pad-button bit for a physical control (passthrough identity map).
///
/// Mirrors the existing `win_io::ds_buttons_to_xinput` / `pack_xinput` button list: Cross→A,
/// Circle→B, Square→X, Triangle→Y, Share→Back, Options→Start, PS→Guide, L1/R1→LB/RB,
/// L3/R3→LS/RS, dpad→dpad, touchpad click→TOUCHPAD. Controls without a virtual-pad bit (analog
/// triggers, stick axes, gyro, Edge paddles, touch regions) return `None`.
#[inline]
fn button_pad_bit(c: Control) -> Option<u32> {
    use crate::output::PadButtons as P;
    Some(match c {
        Control::Cross => P::A,
        Control::Circle => P::B,
        Control::Square => P::X,
        Control::Triangle => P::Y,
        Control::L1 => P::LB,
        Control::R1 => P::RB,
        Control::Share => P::BACK,
        Control::Options => P::START,
        Control::Ps => P::GUIDE,
        Control::L3 => P::LS,
        Control::R3 => P::RS,
        Control::DpadUp => P::DPAD_UP,
        Control::DpadDown => P::DPAD_DOWN,
        Control::DpadLeft => P::DPAD_LEFT,
        Control::DpadRight => P::DPAD_RIGHT,
        Control::TouchButton => P::TOUCHPAD,
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::input::Control;
    use crate::map::binding::{BindingSlot, KeyKind, PadBtn};
    use crate::map::profile::Profile;
    use crate::output::PadButtons;

    fn rp_with(c: Control, bind: BindTarget) -> ResolvedProfile {
        let mut p = Profile::default();
        p.bindings.insert(c, BindingSlot::from_bind(bind));
        p.resolve()
    }

    fn pressed_cross() -> ControllerState {
        ControllerState {
            cross: true,
            ..Default::default()
        }
    }

    #[test]
    fn button_to_button_sets_only_target_bit() {
        // Cross -> Xbox B. B set, A NOT set.
        let rp = rp_with(Control::Cross, BindTarget::GamepadButton(PadBtn::B));
        let mut ms = MapState::default();
        let (out, batch) = apply(&pressed_cross(), &rp, &mut ms, 0);
        assert!(out.buttons.has(PadButtons::B), "B should be set");
        assert!(
            !out.buttons.has(PadButtons::A),
            "A must not leak (identity suppressed)"
        );
        assert!(batch.is_empty());
    }

    #[test]
    fn passthrough_button_identity() {
        // No remap: Cross passes through to A.
        let rp = ResolvedProfile::default();
        let mut ms = MapState::default();
        let (out, _) = apply(&pressed_cross(), &rp, &mut ms, 0);
        assert!(out.buttons.has(PadButtons::A));
        assert!(!out.buttons.has(PadButtons::B));
    }

    #[test]
    fn button_to_key_edges() {
        // Square -> key 0x41 (hold). Down on press, nothing on hold, up on release.
        let rp = rp_with(
            Control::Square,
            BindTarget::Key {
                vk: 0x41,
                kind: KeyKind::HOLD,
            },
        );
        let mut ms = MapState::default();

        let down = ControllerState {
            square: true,
            ..Default::default()
        };
        let up = ControllerState::default();

        // Press: one KeyDown.
        let (_, b1) = apply(&down, &rp, &mut ms, 0);
        assert_eq!(
            b1.as_slice(),
            &[KbmEvent::Key {
                vk: 0x41,
                down: true,
                kind: InjectKind::Virtual
            }]
        );

        // Hold: no events.
        let (_, b2) = apply(&down, &rp, &mut ms, 1);
        assert!(b2.is_empty(), "no event while held");

        // Release: one KeyUp.
        let (_, b3) = apply(&up, &rp, &mut ms, 2);
        assert_eq!(
            b3.as_slice(),
            &[KbmEvent::Key {
                vk: 0x41,
                down: false,
                kind: InjectKind::Virtual
            }]
        );
    }

    #[test]
    fn toggle_latched_on_vk() {
        let kind = KeyKind {
            scan_code: false,
            toggle: true,
        };
        let rp = rp_with(Control::Square, BindTarget::Key { vk: 0x42, kind });
        let mut ms = MapState::default();
        let down = ControllerState {
            square: true,
            ..Default::default()
        };
        let up = ControllerState::default();

        // First press latches ON.
        let (_, b1) = apply(&down, &rp, &mut ms, 0);
        assert_eq!(
            b1.as_slice(),
            &[KbmEvent::Key {
                vk: 0x42,
                down: true,
                kind: InjectKind::Virtual
            }]
        );
        assert!(ms.toggle.get(0x42));
        // Hold: no further toggle.
        let (_, b2) = apply(&down, &rp, &mut ms, 1);
        assert!(b2.is_empty());
        // Release: no event, but pressed_once cleared.
        let (_, b3) = apply(&up, &rp, &mut ms, 2);
        assert!(b3.is_empty());
        // Second press latches OFF.
        let (_, b4) = apply(&down, &rp, &mut ms, 3);
        assert_eq!(
            b4.as_slice(),
            &[KbmEvent::Key {
                vk: 0x42,
                down: false,
                kind: InjectKind::Virtual
            }]
        );
        assert!(!ms.toggle.get(0x42));
    }

    #[test]
    fn two_controls_one_vk_share_toggle_latch() {
        // Square and Cross both -> the same vk toggle: a press on either flips the shared latch.
        let kind = KeyKind {
            scan_code: false,
            toggle: true,
        };
        let mut p = Profile::default();
        p.bindings.insert(
            Control::Square,
            BindingSlot::from_bind(BindTarget::Key { vk: 0x55, kind }),
        );
        p.bindings.insert(
            Control::Cross,
            BindingSlot::from_bind(BindTarget::Key { vk: 0x55, kind }),
        );
        let rp = p.resolve();
        let mut ms = MapState::default();

        // Press Square -> latch ON.
        let sq = ControllerState {
            square: true,
            ..Default::default()
        };
        let (_, _) = apply(&sq, &rp, &mut ms, 0);
        assert!(ms.toggle.get(0x55));
        // Release all.
        let (_, _) = apply(&ControllerState::default(), &rp, &mut ms, 1);
        // Press Cross -> shared latch flips OFF.
        let cr = ControllerState {
            cross: true,
            ..Default::default()
        };
        let (_, b) = apply(&cr, &rp, &mut ms, 2);
        assert_eq!(
            b.as_slice(),
            &[KbmEvent::Key {
                vk: 0x55,
                down: false,
                kind: InjectKind::Virtual
            }]
        );
        assert!(!ms.toggle.get(0x55));
    }

    #[test]
    fn vklatch_overflow_saturates_no_panic() {
        let mut l = VkLatch::new();
        for vk in 0..(VkLatch::CAP as u16 + 8) {
            l.set(vk, true);
        }
        // First CAP keys tracked; the rest dropped silently.
        for vk in 0..VkLatch::CAP as u16 {
            assert!(l.get(vk));
        }
        for vk in VkLatch::CAP as u16..(VkLatch::CAP as u16 + 8) {
            assert!(!l.get(vk), "overflow keys are untracked, default false");
        }
    }

    #[test]
    fn half_axis_identity_leak_guard() {
        // Bind ONLY LxPos -> a key; leave LxNeg as Passthrough. Pushing the stick to the negative
        // half must NOT leak into out.lx (both halves of Lx go neutral — verifier FIX 1).
        let rp = rp_with(
            Control::LxPos,
            BindTarget::Key {
                vk: 0x41,
                kind: KeyKind::HOLD,
            },
        );
        let mut ms = MapState::default();

        // Stick hard left (negative Lx).
        let st = ControllerState {
            lx: -1.0,
            ..Default::default()
        };
        let (out, _) = apply(&st, &rp, &mut ms, 0);
        assert_eq!(
            out.lx, 0.0,
            "negative half must not leak when the positive half is remapped"
        );
        assert_eq!(out.ly, 0.0);

        // Other axes still pass through normally.
        let st2 = ControllerState {
            lx: -1.0,
            ry: 0.5,
            ..Default::default()
        };
        let (out2, _) = apply(&st2, &rp, &mut ms, 1);
        assert_eq!(out2.lx, 0.0);
        assert_eq!(out2.ry, 0.5, "an unrelated axis is unaffected");
    }

    #[test]
    fn unremapped_axis_passes_through_both_halves() {
        // All-passthrough: a left push reaches out.lx, a right push reaches out.lx — same field.
        let rp = ResolvedProfile::default();
        let mut ms = MapState::default();
        let (out_l, _) = apply(
            &ControllerState {
                lx: -0.8,
                ..Default::default()
            },
            &rp,
            &mut ms,
            0,
        );
        assert_eq!(out_l.lx, -0.8);
        let (out_r, _) = apply(
            &ControllerState {
                lx: 0.6,
                ..Default::default()
            },
            &rp,
            &mut ms,
            1,
        );
        assert_eq!(out_r.lx, 0.6);
    }

    #[test]
    fn passthrough_precision_is_exact() {
        // A precise stick value flows through unrounded (no mid-chain quantization in apply()).
        let rp = ResolvedProfile::default();
        let mut ms = MapState::default();
        let st = ControllerState {
            lx: 0.123_456_789,
            ry: -0.987_654_321,
            ..Default::default()
        };
        let (out, _) = apply(&st, &rp, &mut ms, 0);
        assert_eq!(out.lx, 0.123_456_789);
        assert_eq!(out.ry, -0.987_654_321);
    }

    #[test]
    fn analog_to_digital_axis_threshold_55_127() {
        // GamepadButton bound to LxPos fires only past the 55/127 stick-dir threshold.
        let rp = rp_with(Control::LxPos, BindTarget::GamepadButton(PadBtn::A));
        let mut ms = MapState::default();
        let below = 54.0 / 127.0;
        let above = 56.0 / 127.0;
        let (lo, _) = apply(
            &ControllerState {
                lx: below,
                ..Default::default()
            },
            &rp,
            &mut ms,
            0,
        );
        assert!(!lo.buttons.has(PadButtons::A), "below 55/127 must not fire");
        let (hi, _) = apply(
            &ControllerState {
                lx: above,
                ..Default::default()
            },
            &rp,
            &mut ms,
            1,
        );
        assert!(hi.buttons.has(PadButtons::A), "above 55/127 must fire");
    }

    #[test]
    fn analog_to_digital_trigger_threshold_100_255() {
        // GamepadButton bound to L2 (analog trigger) fires only past 100/255.
        let rp = rp_with(Control::L2, BindTarget::GamepadButton(PadBtn::Lb));
        let mut ms = MapState::default();
        let below = 99.0 / 255.0;
        let above = 101.0 / 255.0;
        let (lo, _) = apply(
            &ControllerState {
                l2: below,
                ..Default::default()
            },
            &rp,
            &mut ms,
            0,
        );
        assert!(
            !lo.buttons.has(PadButtons::LB),
            "below 100/255 must not fire"
        );
        let (hi, _) = apply(
            &ControllerState {
                l2: above,
                ..Default::default()
            },
            &rp,
            &mut ms,
            1,
        );
        assert!(hi.buttons.has(PadButtons::LB), "above 100/255 must fire");
    }

    #[test]
    fn trigger_passthrough_is_continuous() {
        // Default profile: analog trigger flows to lt continuously (not just digital).
        let rp = ResolvedProfile::default();
        let mut ms = MapState::default();
        let (out, _) = apply(
            &ControllerState {
                l2: 0.42,
                r2: 0.7,
                ..Default::default()
            },
            &rp,
            &mut ms,
            0,
        );
        assert_eq!(out.lt, 0.42);
        assert_eq!(out.rt, 0.7);
    }

    #[test]
    fn kbm_batch_saturates_without_panic() {
        // Bind many distinct controls to distinct keys and press them all in one report. The batch
        // saturates at its cap (no panic, no alloc) and stays a valid slice.
        let mut p = Profile::default();
        // Use face + dpad + shoulder buttons, each to a unique vk, all pressed together.
        let controls = [
            Control::Square,
            Control::Triangle,
            Control::Circle,
            Control::Cross,
            Control::DpadUp,
            Control::DpadDown,
            Control::DpadLeft,
            Control::DpadRight,
            Control::L1,
            Control::R1,
            Control::L3,
            Control::R3,
            Control::Ps,
            Control::Share,
            Control::Options,
        ];
        for (i, &c) in controls.iter().enumerate() {
            p.bindings.insert(
                c,
                BindingSlot::from_bind(BindTarget::Key {
                    vk: 0x41 + i as u16,
                    kind: KeyKind::HOLD,
                }),
            );
        }
        let rp = p.resolve();
        let mut ms = MapState::default();
        let st = ControllerState {
            square: true,
            triangle: true,
            circle: true,
            cross: true,
            dpad_up: true,
            dpad_down: true,
            dpad_left: true,
            dpad_right: true,
            l1: true,
            r1: true,
            l3: true,
            r3: true,
            ps: true,
            share: true,
            options: true,
            ..Default::default()
        };
        let (_, batch) = apply(&st, &rp, &mut ms, 0);
        // 15 distinct key-down edges; well within CAP — all present, no overflow.
        assert_eq!(batch.as_slice().len(), controls.len());
    }

    #[test]
    fn all_passthrough_matches_outputstate_passthrough() {
        // Non-regression invariant (blueprint §7.2): an all-Passthrough profile produces the same
        // OutputState as the identity `OutputState::passthrough(&state)` seed.
        let rp = ResolvedProfile::default();
        let mut ms = MapState::default();
        let st = ControllerState {
            lx: 0.25,
            ly: -0.5,
            rx: -0.1,
            ry: 0.9,
            l2: 0.3,
            r2: 0.8,
            cross: true,
            circle: true,
            l1: true,
            dpad_left: true,
            options: true,
            ..Default::default()
        };
        let (out, batch) = apply(&st, &rp, &mut ms, 0);
        assert_eq!(out, OutputState::passthrough(&st));
        assert!(batch.is_empty(), "passthrough emits no KBM events");
    }

    #[test]
    fn default_mapstate_is_clean() {
        let ms = MapState::default();
        assert!(ms.prev_active.iter().all(|&b| !b));
        assert!(ms.turbo.iter().all(|&t| t == TurboState::default()));
        assert_eq!(ms.wheel_remainder, 0.0);
        assert!(!ms.toggle.get(0x41));
        assert!(!ms.pressed_once.get(0x41));
    }
}
