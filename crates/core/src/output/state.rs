//! `OutputState` + `PadButtons` — the virtual-controller-agnostic egress accumulator.
//!
//! This is the single value the mapping engine fills (blueprint §3.3): processed sticks/
//! triggers in canonical units (`[-1,1]` `+y up`, `[0,1]`) plus a target-agnostic button set.
//! It lowers to either an Xbox-360 frame ([`OutputState::to_output_frame`] → the existing
//! [`OutputFrame`], preserving the single-round i16 egress) or a DS4 wire report (via
//! [`to_ds4_axis`]/[`dpad_8way`], called by the `cfg(windows)` DS4 backend).
//!
//! The X360 button packing ([`pack_xinput`]) and the DS4 lowering are pure and Linux-tested so
//! there is exactly one source of truth for both wire formats, mirroring how
//! [`to_xinput_thumb`](crate::output::to_xinput_thumb) is shared today.

use super::OutputFrame;
use crate::input::ControllerState;

/// A virtual-controller-agnostic button set the mapping engine fills.
///
/// The bit layout is internal (lowered per target by [`pack_xinput`] / the DS4 backend); use
/// the named masks and [`has`](Self::has)/[`set`](Self::set) rather than raw bit math.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PadButtons(pub u32);

// The `1 << N` form documents each button's bit position; `1 << 0` is intentional.
#[allow(clippy::identity_op)]
impl PadButtons {
    /// A (Cross).
    pub const A: u32 = 1 << 0;
    /// B (Circle).
    pub const B: u32 = 1 << 1;
    /// X (Square).
    pub const X: u32 = 1 << 2;
    /// Y (Triangle).
    pub const Y: u32 = 1 << 3;
    /// Left bumper (L1).
    pub const LB: u32 = 1 << 4;
    /// Right bumper (R1).
    pub const RB: u32 = 1 << 5;
    /// Back / View / Share.
    pub const BACK: u32 = 1 << 6;
    /// Start / Menu / Options.
    pub const START: u32 = 1 << 7;
    /// Left stick click (L3).
    pub const LS: u32 = 1 << 8;
    /// Right stick click (R3).
    pub const RS: u32 = 1 << 9;
    /// Guide / PS.
    pub const GUIDE: u32 = 1 << 10;
    /// D-pad up.
    pub const DPAD_UP: u32 = 1 << 11;
    /// D-pad down.
    pub const DPAD_DOWN: u32 = 1 << 12;
    /// D-pad left.
    pub const DPAD_LEFT: u32 = 1 << 13;
    /// D-pad right.
    pub const DPAD_RIGHT: u32 = 1 << 14;
    /// L2 digital click (DS4 trigger flag; no X360 bit).
    pub const L2_CLICK: u32 = 1 << 15;
    /// R2 digital click (DS4 trigger flag; no X360 bit).
    pub const R2_CLICK: u32 = 1 << 16;
    /// Touchpad click (DS4 special byte; no X360 bit) — verifier FIX 5.
    pub const TOUCHPAD: u32 = 1 << 17;

    /// Whether any bit in `mask` is set.
    #[inline]
    pub fn has(self, mask: u32) -> bool {
        self.0 & mask != 0
    }

    /// Set or clear every bit in `mask`.
    #[inline]
    pub fn set(&mut self, mask: u32, on: bool) {
        if on {
            self.0 |= mask;
        } else {
            self.0 &= !mask;
        }
    }
}

/// Full processed virtual-pad state. Sticks `[-1,1]` (`+y up`), triggers `[0,1]`. `Copy`,
/// allocation-free — the single egress value the hot loop builds per report.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct OutputState {
    /// Left-stick X, `[-1,1]`.
    pub lx: f64,
    /// Left-stick Y, `[-1,1]` (`+` = up).
    pub ly: f64,
    /// Right-stick X, `[-1,1]`.
    pub rx: f64,
    /// Right-stick Y, `[-1,1]` (`+` = up).
    pub ry: f64,
    /// Left trigger, `[0,1]`.
    pub lt: f64,
    /// Right trigger, `[0,1]`.
    pub rt: f64,
    /// The target-agnostic button set.
    pub buttons: PadButtons,
}

impl OutputState {
    /// Identity-passthrough seed: copy sticks/triggers and digital buttons straight from the
    /// decoded [`ControllerState`] (no remaps applied).
    ///
    /// With an all-`Passthrough` profile this is exactly what the mapping engine returns, so the
    /// resulting [`OutputFrame`] is byte-identical to the pre-mapper path.
    pub fn passthrough(s: &ControllerState) -> Self {
        let mut buttons = PadButtons::default();
        buttons.set(PadButtons::A, s.cross);
        buttons.set(PadButtons::B, s.circle);
        buttons.set(PadButtons::X, s.square);
        buttons.set(PadButtons::Y, s.triangle);
        buttons.set(PadButtons::LB, s.l1);
        buttons.set(PadButtons::RB, s.r1);
        buttons.set(PadButtons::BACK, s.share);
        buttons.set(PadButtons::START, s.options);
        buttons.set(PadButtons::LS, s.l3);
        buttons.set(PadButtons::RS, s.r3);
        buttons.set(PadButtons::GUIDE, s.ps);
        buttons.set(PadButtons::DPAD_UP, s.dpad_up);
        buttons.set(PadButtons::DPAD_DOWN, s.dpad_down);
        buttons.set(PadButtons::DPAD_LEFT, s.dpad_left);
        buttons.set(PadButtons::DPAD_RIGHT, s.dpad_right);
        // DS4-only flags (no X360 bit): L2/R2 click and touchpad.
        buttons.set(PadButtons::L2_CLICK, s.l2_raw == 255);
        buttons.set(PadButtons::R2_CLICK, s.r2_raw == 255);
        buttons.set(PadButtons::TOUCHPAD, s.touch_button);
        Self {
            lx: s.lx,
            ly: s.ly,
            rx: s.rx,
            ry: s.ry,
            lt: s.l2,
            rt: s.r2,
            buttons,
        }
    }

    /// Project to the Xbox-360 [`OutputFrame`].
    ///
    /// The f64 sticks/triggers carry through unchanged — the single i16/u8 round still happens
    /// ONLY in the backend via [`to_xinput_thumb`](crate::output::to_xinput_thumb) /
    /// [`to_xinput_trigger`](crate::output::to_xinput_trigger). This just packs the button u16.
    pub fn to_output_frame(&self) -> OutputFrame {
        OutputFrame {
            lx: self.lx,
            ly: self.ly,
            rx: self.rx,
            ry: self.ry,
            lt: self.lt,
            rt: self.rt,
            buttons: pack_xinput(self.buttons),
        }
    }
}

// XInput (`XINPUT_GAMEPAD_*`) button bits — the single source of truth promoted from
// `engine/src/win_io.rs` (which this replaces as the packing authority).
const XI_DPAD_UP: u16 = 0x0001;
const XI_DPAD_DOWN: u16 = 0x0002;
const XI_DPAD_LEFT: u16 = 0x0004;
const XI_DPAD_RIGHT: u16 = 0x0008;
const XI_START: u16 = 0x0010;
const XI_BACK: u16 = 0x0020;
const XI_LTHUMB: u16 = 0x0040;
const XI_RTHUMB: u16 = 0x0080;
const XI_LSHOULDER: u16 = 0x0100;
const XI_RSHOULDER: u16 = 0x0200;
const XI_GUIDE: u16 = 0x0400;
const XI_A: u16 = 0x1000;
const XI_B: u16 = 0x2000;
const XI_X: u16 = 0x4000;
const XI_Y: u16 = 0x8000;

/// Lower a [`PadButtons`] set to the XInput button `u16` (the same button list as the C#
/// `Xbox360OutDevice` / the former `win_io::ds_buttons_to_xinput`).
///
/// `A`=Cross, `B`=Circle, `X`=Square, `Y`=Triangle, `Back`=Share, `Start`=Options, `Guide`=PS,
/// `LS`/`RS` thumbs, `LB`/`RB` shoulders, dpad. `L2_CLICK`/`R2_CLICK`/`TOUCHPAD` have no X360
/// bit (they are DS4-only) and are dropped here.
pub fn pack_xinput(b: PadButtons) -> u16 {
    let mut out = 0u16;
    if b.has(PadButtons::A) {
        out |= XI_A;
    }
    if b.has(PadButtons::B) {
        out |= XI_B;
    }
    if b.has(PadButtons::X) {
        out |= XI_X;
    }
    if b.has(PadButtons::Y) {
        out |= XI_Y;
    }
    if b.has(PadButtons::LB) {
        out |= XI_LSHOULDER;
    }
    if b.has(PadButtons::RB) {
        out |= XI_RSHOULDER;
    }
    if b.has(PadButtons::BACK) {
        out |= XI_BACK;
    }
    if b.has(PadButtons::START) {
        out |= XI_START;
    }
    if b.has(PadButtons::LS) {
        out |= XI_LTHUMB;
    }
    if b.has(PadButtons::RS) {
        out |= XI_RTHUMB;
    }
    if b.has(PadButtons::GUIDE) {
        out |= XI_GUIDE;
    }
    if b.has(PadButtons::DPAD_UP) {
        out |= XI_DPAD_UP;
    }
    if b.has(PadButtons::DPAD_DOWN) {
        out |= XI_DPAD_DOWN;
    }
    if b.has(PadButtons::DPAD_LEFT) {
        out |= XI_DPAD_LEFT;
    }
    if b.has(PadButtons::DPAD_RIGHT) {
        out |= XI_DPAD_RIGHT;
    }
    out
}

/// The DS4 wire D-pad hat nibble (`DS4OutDeviceBasic` encoding: `0`=N … `7`=NW, `8`=neutral).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum Ds4Dpad {
    North = 0,
    NorthEast = 1,
    East = 2,
    SouthEast = 3,
    South = 4,
    SouthWest = 5,
    West = 6,
    NorthWest = 7,
    #[default]
    None = 8,
}

impl Ds4Dpad {
    /// The raw DS4 hat nibble byte.
    #[inline]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

/// DS4 wire axis map: canonical `[-1,1]` → unsigned-8 with `128` center (`-1 → 0`, `+1 → 255`).
///
/// `flip_y` inverts before scaling (DS4 reports up as a *smaller* raw value, like the input
/// side — HW-verify the wire polarity on a real DS4 target). The single round happens here.
#[inline]
pub fn to_ds4_axis(n: f64, flip_y: bool) -> u8 {
    let v = if flip_y { -n } else { n };
    (((v.clamp(-1.0, 1.0) + 1.0) * 0.5) * 255.0)
        .round()
        .clamp(0.0, 255.0) as u8
}

/// Resolve four D-pad direction bools to the DS4 8-way hat nibble (`DS4OutDeviceBasic` ladder).
///
/// Diagonals are produced when two adjacent directions are held; opposing presses resolve to
/// the nearest cardinal (`up`/`down` then `left`/`right`), matching the C# if/else ladder.
#[inline]
pub fn dpad_8way(up: bool, down: bool, left: bool, right: bool) -> Ds4Dpad {
    if up && right {
        Ds4Dpad::NorthEast
    } else if up && left {
        Ds4Dpad::NorthWest
    } else if down && right {
        Ds4Dpad::SouthEast
    } else if down && left {
        Ds4Dpad::SouthWest
    } else if up {
        Ds4Dpad::North
    } else if right {
        Ds4Dpad::East
    } else if down {
        Ds4Dpad::South
    } else if left {
        Ds4Dpad::West
    } else {
        Ds4Dpad::None
    }
}

/// Which virtual controller the active profile drives. Chosen at (re)plug time, never per report.
///
/// Carries serde derives (PascalCase) because it lives inside the serde `Profile` as
/// `output_kind` (blueprint §7.1).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum PadTarget {
    /// Virtual Xbox 360 pad (the M2 default).
    #[default]
    X360,
    /// Virtual DualShock 4 pad (M5).
    Ds4,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::output::{to_xinput_thumb, to_xinput_trigger};

    #[test]
    fn pad_buttons_set_has_clear() {
        let mut b = PadButtons::default();
        assert!(!b.has(PadButtons::A));
        b.set(PadButtons::A, true);
        b.set(PadButtons::GUIDE, true);
        assert!(b.has(PadButtons::A));
        assert!(b.has(PadButtons::GUIDE));
        assert!(!b.has(PadButtons::B));
        b.set(PadButtons::A, false);
        assert!(!b.has(PadButtons::A));
        assert!(b.has(PadButtons::GUIDE));
    }

    #[test]
    fn pack_xinput_face_and_meta_mapping() {
        let mut b = PadButtons::default();
        b.set(PadButtons::A, true);
        assert_eq!(pack_xinput(b), 0x1000);
        let mut b = PadButtons::default();
        b.set(PadButtons::B, true);
        assert_eq!(pack_xinput(b), 0x2000);
        let mut b = PadButtons::default();
        b.set(PadButtons::X, true);
        assert_eq!(pack_xinput(b), 0x4000);
        let mut b = PadButtons::default();
        b.set(PadButtons::Y, true);
        assert_eq!(pack_xinput(b), 0x8000);
        let mut b = PadButtons::default();
        b.set(PadButtons::GUIDE, true);
        assert_eq!(pack_xinput(b), 0x0400, "GUIDE -> Guide on X360");
        let mut b = PadButtons::default();
        b.set(PadButtons::BACK, true);
        assert_eq!(pack_xinput(b), 0x0020);
        let mut b = PadButtons::default();
        b.set(PadButtons::START, true);
        assert_eq!(pack_xinput(b), 0x0010);
    }

    #[test]
    fn pack_xinput_drops_ds4_only_flags() {
        let mut b = PadButtons::default();
        b.set(PadButtons::L2_CLICK, true);
        b.set(PadButtons::R2_CLICK, true);
        b.set(PadButtons::TOUCHPAD, true);
        assert_eq!(pack_xinput(b), 0, "DS4-only flags have no X360 bit");
    }

    #[test]
    fn pack_xinput_dpad_and_shoulders() {
        let mut b = PadButtons::default();
        b.set(PadButtons::DPAD_UP, true);
        b.set(PadButtons::DPAD_DOWN, true);
        b.set(PadButtons::DPAD_LEFT, true);
        b.set(PadButtons::DPAD_RIGHT, true);
        b.set(PadButtons::LB, true);
        b.set(PadButtons::RB, true);
        b.set(PadButtons::LS, true);
        b.set(PadButtons::RS, true);
        let v = pack_xinput(b);
        assert_eq!(v & 0x000F, 0x000F, "all dpad");
        assert_eq!(v & 0x0300, 0x0300, "both shoulders");
        assert_eq!(v & 0x00C0, 0x00C0, "both thumbs");
    }

    fn blank() -> ControllerState {
        ControllerState::default()
    }

    #[test]
    fn passthrough_button_packing_matches_legacy() {
        // Cross+Circle+Square+Triangle+L1+R1+Share+Options+PS+all dpad -> the same X360 bits the
        // former ds_buttons_to_xinput produced for those inputs.
        let mut s = blank();
        s.cross = true;
        s.circle = true;
        s.square = true;
        s.triangle = true;
        s.l1 = true;
        s.r1 = true;
        s.share = true;
        s.options = true;
        s.ps = true;
        s.l3 = true;
        s.r3 = true;
        s.dpad_up = true;
        let frame = OutputState::passthrough(&s).to_output_frame();
        let v = frame.buttons;
        assert_eq!(v & XI_A, XI_A);
        assert_eq!(v & XI_B, XI_B);
        assert_eq!(v & XI_X, XI_X);
        assert_eq!(v & XI_Y, XI_Y);
        assert_eq!(v & XI_LSHOULDER, XI_LSHOULDER);
        assert_eq!(v & XI_RSHOULDER, XI_RSHOULDER);
        assert_eq!(v & XI_BACK, XI_BACK);
        assert_eq!(v & XI_START, XI_START);
        assert_eq!(v & XI_GUIDE, XI_GUIDE);
        assert_eq!(v & XI_LTHUMB, XI_LTHUMB);
        assert_eq!(v & XI_RTHUMB, XI_RTHUMB);
        assert_eq!(v & XI_DPAD_UP, XI_DPAD_UP);
    }

    #[test]
    fn passthrough_carries_sticks_and_triggers_bit_identically() {
        let mut s = blank();
        s.lx = 0.123_456_789;
        s.ly = -0.5;
        s.rx = 1.0;
        s.ry = -1.0;
        s.l2 = 0.4;
        s.r2 = 0.6;
        let frame = OutputState::passthrough(&s).to_output_frame();
        assert_eq!(frame.lx, 0.123_456_789);
        assert_eq!(frame.ly, -0.5);
        assert_eq!(frame.rx, 1.0);
        assert_eq!(frame.ry, -1.0);
        assert_eq!(frame.lt, 0.4);
        assert_eq!(frame.rt, 0.6);
        // And the single round still lands where the existing egress maps it.
        assert_eq!(to_xinput_thumb(frame.lx), to_xinput_thumb(0.123_456_789));
        assert_eq!(to_xinput_trigger(frame.lt), to_xinput_trigger(0.4));
    }

    #[test]
    fn default_output_state_lowers_to_default_frame() {
        assert_eq!(
            OutputState::default().to_output_frame(),
            OutputFrame::default()
        );
    }

    #[test]
    fn to_ds4_axis_center_endpoints_and_flip() {
        assert_eq!(to_ds4_axis(0.0, false), 128);
        assert_eq!(to_ds4_axis(1.0, false), 255);
        assert_eq!(to_ds4_axis(-1.0, false), 0);
        // flip_y inverts.
        assert_eq!(to_ds4_axis(1.0, true), 0);
        assert_eq!(to_ds4_axis(-1.0, true), 255);
        assert_eq!(to_ds4_axis(0.0, true), 128);
        // Clamps out of range.
        assert_eq!(to_ds4_axis(2.0, false), 255);
        assert_eq!(to_ds4_axis(-2.0, false), 0);
    }

    #[test]
    fn dpad_8way_all_directions_and_neutral() {
        assert_eq!(dpad_8way(false, false, false, false), Ds4Dpad::None);
        assert_eq!(dpad_8way(true, false, false, false), Ds4Dpad::North);
        assert_eq!(dpad_8way(false, false, false, true), Ds4Dpad::East);
        assert_eq!(dpad_8way(false, true, false, false), Ds4Dpad::South);
        assert_eq!(dpad_8way(false, false, true, false), Ds4Dpad::West);
        // Diagonals.
        assert_eq!(dpad_8way(true, false, false, true), Ds4Dpad::NorthEast);
        assert_eq!(dpad_8way(true, false, true, false), Ds4Dpad::NorthWest);
        assert_eq!(dpad_8way(false, true, false, true), Ds4Dpad::SouthEast);
        assert_eq!(dpad_8way(false, true, true, false), Ds4Dpad::SouthWest);
    }

    #[test]
    fn dpad_8way_resolves_opposing_presses() {
        // up+down with right -> the ladder picks NE (up wins the vertical pair via ordering).
        assert_eq!(dpad_8way(true, true, false, true), Ds4Dpad::NorthEast);
        // up+down only -> North (up checked before down).
        assert_eq!(dpad_8way(true, true, false, false), Ds4Dpad::North);
        // left+right only -> the ladder falls through up/down to East (right before left).
        assert_eq!(dpad_8way(false, false, true, true), Ds4Dpad::East);
    }

    #[test]
    fn ds4_dpad_nibble_values() {
        assert_eq!(Ds4Dpad::North.as_u8(), 0);
        assert_eq!(Ds4Dpad::NorthWest.as_u8(), 7);
        assert_eq!(Ds4Dpad::None.as_u8(), 8);
        assert_eq!(Ds4Dpad::default(), Ds4Dpad::None);
    }

    #[test]
    fn pad_target_serde_round_trips() {
        #[derive(serde::Serialize, serde::Deserialize, PartialEq, Debug)]
        struct W {
            t: PadTarget,
        }
        for t in [PadTarget::X360, PadTarget::Ds4] {
            let s = toml::to_string(&W { t }).unwrap();
            let back: W = toml::from_str(&s).unwrap();
            assert_eq!(back.t, t);
        }
        assert_eq!(PadTarget::default(), PadTarget::X360);
    }
}
