//! DualSense (DS4-compatible report `0x01`, 64-byte USB) byte decode.
//!
//! Offsets are taken from the **ground-truth C#** (`DS4Device.cs:1216-1333`), not the
//! DESIGN §7 prose summary (which is imprecise). The decode here is pure: the Windows HID
//! shell hands us the raw 64-byte buffer and we return the typed fields; conversion to the
//! canonical `[-1,1]` / `[0,1]` units happens in [`ds_report_to_sticks`] /
//! [`crate::input::normalize`].
//!
//! ## Byte map (`buf[0] == 0x01`)
//! | byte | field      | source (`DS4Device.cs`)                       | confidence |
//! |------|------------|-----------------------------------------------|------------|
//! | 0    | report id  | must be `0x01`                                | SOLID      |
//! | 1    | `lx`       | `cState.LX = inputReport[1]` (`:1216`)        | SOLID      |
//! | 2    | `ly`       | `cState.LY = inputReport[2]` (`:1217`)        | SOLID      |
//! | 3    | `rx`       | `cState.RX = inputReport[3]` (`:1218`)        | SOLID      |
//! | 4    | `ry`       | `cState.RY = inputReport[4]` (`:1219`)        | SOLID      |
//! | 5    | `btn0`     | face buttons + dpad nibble (`:1225-1247`)     | HW-verify  |
//! | 6    | `btn1`     | R3/L3/Options/Share/R2/L2 btn/R1/L1 (`:1249`) | HW-verify  |
//! | 7    | `btn2`     | PS/Touch + frame counter `>>2` (`:1259-1264`) | HW-verify  |
//! | 8    | `l2`       | `cState.L2 = inputReport[8]` (`:1220`)        | SOLID      |
//! | 9    | `r2`       | `cState.R2 = inputReport[9]` (`:1221`)        | SOLID      |
//! | 10,11| `sensor_ts`| `(ushort)(inputReport[11]<<8 \| [10])` (`:1301`) | HW-verify |
//!
//! `counter` is the 6-bit frame counter (`inputReport[7] >> 2`, `:1264`) used for
//! drop/duplicate detection. The `btn*`/`sensor_ts` fields are marked **HW-verify**
//! (timestamp tick scale and the DSE paddle/Fn/Mute superset in `btn2` are under-documented
//! for the Edge); the stick bytes 1-4 are **SOLID**.
//!
//! ## Touchpad contacts (M6)
//!
//! The two finger contacts live in the report tail at the C# `DS4_TOUCHPAD_DATA_OFFSET == 35`
//! (`DS4Touchpad.cs:106`). The packing per finger is the "simpler touch storing" path in
//! `DS4Device.cs:1336-1346` (the active per-report path; the alternate `handleTouchpad` history
//! path uses the SAME offsets):
//!
//! | field        | finger 0           | finger 1           | C# expression (`DS4Device.cs`)                |
//! |--------------|--------------------|--------------------|-----------------------------------------------|
//! | id byte      | `buf[35]`          | `buf[39]`          | `Id = b & 0x7f`; `IsActive = (b & 0x80) == 0` |
//! | X (12-bit)   | `buf[36],buf[37]`  | `buf[40],buf[41]`  | `((b_hi & 0x0f) << 8) | b_lo`                 |
//! | Y (12-bit)   | `buf[37],buf[38]`  | `buf[41],buf[42]`  | `(b_y << 4) | ((b_xhi & 0xf0) >> 4)`          |
//!
//! **HW-verify (M6).** Offsets 35..=42 and the **active = high-bit-CLEAR** convention
//! (`(idbyte & 0x80) == 0`) are taken verbatim from the C# ground truth and need a hardware
//! capture to confirm against a real DualSense USB `0x01` report tail. The grid is
//! `x: 0..=1919` / `y: 0..=941` (`RESOLUTION_X_MAX 1920` / `RESOLUTION_Y_MAX 942`,
//! `DS4Touchpad.cs:107-108`, exclusive max). Touch stays `Default` (`is_active == false`,
//! `x/y/id == 0`) for any buffer shorter than the touch tail, so a 10-byte test report or a
//! non-touch source is unchanged.
//!
//! ## DualSense Edge superset (M6, gated by `meta.is_edge`)
//!
//! Mute/Capture and the Edge Fn/paddle/side buttons are NOT present in the DS4-compatible `0x01`
//! report this decoder consumes (the C# fork only ever sets `DS4State.Mute/FnL/...` by copying a
//! prior state — `DS4State.cs:180-187` — it never decodes them from a report byte in the
//! `0x01` path, because they live in the DualSense Edge **extended** report). The bit layout
//! below is the published DualSense-Edge extended-report tail and is **HW-verify (M6)**: it is
//! decoded ONLY when `meta.is_edge` is set, so every non-Edge source (and any Edge source whose
//! extended bytes are zero) reads them as `false` exactly as before.

use super::normalize::{u8_stick_to_axis, u8_trigger};
use super::state::{ControllerState, TouchContact};
use super::{SourceMeta, StickPair};

/// C# `DS4_TOUCHPAD_DATA_OFFSET` (`DS4Touchpad.cs:106`): first touch byte in the report tail.
pub const TOUCH_DATA_OFFSET: usize = 35;
/// Touch grid horizontal span (`RESOLUTION_X_MAX`, exclusive): valid `x` is `0..=1919`.
pub const TOUCH_RES_X: u16 = 1920;
/// Touch grid vertical span (`RESOLUTION_Y_MAX`, exclusive): valid `y` is `0..=941`.
pub const TOUCH_RES_Y: u16 = 942;
/// Highest report byte the touch tail reads (finger-1 Y high byte `buf[42]`).
const TOUCH_TAIL_LAST: usize = TOUCH_DATA_OFFSET + 7;

/// DS4-compatible USB input report id.
pub const DS_USB_REPORT_ID: u8 = 0x01;
/// DS4-compatible USB input report length, in bytes.
pub const DS_USB_REPORT_LEN: usize = 64;

/// The raw, still-integer fields decoded from a DualSense USB report.
///
/// `sensor_ts` is the little-endian `u16` hardware timestamp (bytes 10..11). `counter` is
/// the 6-bit frame counter. `btn0`/`btn1`/`btn2` are the three raw button bytes (5/6/7).
#[derive(Clone, Copy, Debug, Default)]
pub struct DsReport {
    pub lx: u8,
    pub ly: u8,
    pub rx: u8,
    pub ry: u8,
    pub l2: u8,
    pub r2: u8,
    pub counter: u8,
    pub sensor_ts: u16,
    pub btn0: u8,
    pub btn1: u8,
    pub btn2: u8,
}

/// Parse a raw DualSense USB report buffer.
///
/// Returns `None` if the buffer is shorter than [`DS_USB_REPORT_LEN`] or the report id
/// (`buf[0]`) is not [`DS_USB_REPORT_ID`]. All field offsets follow the C# ground truth.
pub fn parse_ds_usb_report(buf: &[u8]) -> Option<DsReport> {
    if buf.len() < DS_USB_REPORT_LEN || buf[0] != DS_USB_REPORT_ID {
        return None;
    }
    Some(DsReport {
        lx: buf[1],
        ly: buf[2],
        rx: buf[3],
        ry: buf[4],
        l2: buf[8],
        r2: buf[9],
        counter: buf[7] >> 2,
        sensor_ts: (u16::from(buf[11]) << 8) | u16::from(buf[10]),
        btn0: buf[5],
        btn1: buf[6],
        btn2: buf[7],
    })
}

/// Convert a decoded report's stick bytes to canonical `(left, right)` pairs in `[-1,1]`.
///
/// **Y convention:** the device reports up as a *smaller* raw value (`0x00` = fully up),
/// so `u8_stick_to_axis(0x00) = -1.0`. We negate Y so that `+y == up` in the canonical
/// frame: `y = -u8_stick_to_axis(raw_y)` (raw `0x00` → axis `-1.0` → `y = +1.0`). X is not
/// negated (`0x00` = left = `-1.0`, which already matches `+x == right`).
pub fn ds_report_to_sticks(r: &DsReport) -> (StickPair, StickPair) {
    let left = StickPair {
        x: u8_stick_to_axis(r.lx),
        y: -u8_stick_to_axis(r.ly),
    };
    let right = StickPair {
        x: u8_stick_to_axis(r.rx),
        y: -u8_stick_to_axis(r.ry),
    };
    (left, right)
}

/// Decode the 4-bit D-pad hat nibble (`btn0 & 0x0F`) into `(up, right, down, left)` bools.
///
/// `0..=7` are the 8 compass directions (`0`=N, `2`=E, `4`=S, `6`=W, odd values diagonals);
/// `8` (and any out-of-range value) is neutral. This is the single source of truth promoted
/// from `engine/src/win_io.rs::ds_buttons_to_xinput`.
#[inline]
pub fn decode_dpad_hat(nibble: u8) -> (bool, bool, bool, bool) {
    match nibble & 0x0F {
        0 => (true, false, false, false),  // N
        1 => (true, true, false, false),   // NE
        2 => (false, true, false, false),  // E
        3 => (false, true, true, false),   // SE
        4 => (false, false, true, false),  // S
        5 => (false, false, true, true),   // SW
        6 => (false, false, false, true),  // W
        7 => (true, false, false, true),   // NW
        _ => (false, false, false, false), // 8 (and 9..=15) neutral
    }
}

/// Decode one touchpad finger contact from its 4-byte slot at `base` (the id byte index).
///
/// Ports the C# "simpler touch storing" path (`DS4Device.cs:1336-1346`): `base` is the id byte
/// (`buf[35]` for finger 0, `buf[39]` for finger 1); the following three bytes carry the 12-bit
/// X (`base+1` low, low nibble of `base+2`) and 12-bit Y (high nibble of `base+2`, `base+3`).
/// **Active is high-bit-CLEAR** (`(idbyte & 0x80) == 0`), matching the C# `IsActive`. `x`/`y` are
/// clamped to the grid (`0..=1919` / `0..=941`) so a malformed report can never index past the
/// region split. Returns the `Default` (inactive) contact when the slot is past the buffer end.
#[inline]
fn decode_touch_contact(buf: &[u8], base: usize) -> TouchContact {
    if base + 3 >= buf.len() {
        return TouchContact::default();
    }
    let id_byte = buf[base];
    let x = (u16::from(buf[base + 2] & 0x0F) << 8) | u16::from(buf[base + 1]);
    let y = (u16::from(buf[base + 3]) << 4) | (u16::from(buf[base + 2] & 0xF0) >> 4);
    TouchContact {
        is_active: id_byte & 0x80 == 0,
        id: id_byte & 0x7F,
        x: x.min(TOUCH_RES_X - 1),
        y: y.min(TOUCH_RES_Y - 1),
    }
}

/// Decode both touchpad contacts from the report tail (HW-verify, M6).
///
/// Returns the two `[TouchContact; 2]` from offsets `35,39` (`DS4_TOUCHPAD_DATA_OFFSET`). A buffer
/// that does not reach the touch tail (`< 43` bytes) yields two `Default` (inactive) contacts, so
/// short test reports and non-touch sources keep their M5 `Default` behavior.
#[inline]
fn decode_touch(buf: &[u8]) -> [TouchContact; 2] {
    if buf.len() <= TOUCH_TAIL_LAST {
        return [TouchContact::default(); 2];
    }
    [
        decode_touch_contact(buf, TOUCH_DATA_OFFSET),
        decode_touch_contact(buf, TOUCH_DATA_OFFSET + 4),
    ]
}

// --- DualSense Edge extended-report superset bit positions (HW-verify, M6) -----------------------
//
// These are decoded ONLY when `meta.is_edge` (so non-Edge output is byte-identical). The Edge
// extended report is LONGER than the 64-byte DS4-compat frame; the Fn/paddle/Mute bits live in a
// tail byte PAST the touch data so they cannot collide with any DS4-compat field (notably `r2` at
// byte 9). Pinned as named constants so a hardware capture only edits one place; offsets are
// HW-verify (the precise Edge tail byte must be confirmed against a real DualSense Edge capture).
/// Edge extended byte carrying Mute + Fn + paddle bits (`buf[EDGE_FN_BYTE]`, past the touch tail).
const EDGE_FN_BYTE: usize = TOUCH_TAIL_LAST + 1;
/// Mute button bit within [`EDGE_FN_BYTE`].
const EDGE_MUTE: u8 = 0x04;
/// Left function button bit.
const EDGE_FN_L: u8 = 0x10;
/// Right function button bit.
const EDGE_FN_R: u8 = 0x20;
/// Back-left paddle bit.
const EDGE_BLP: u8 = 0x40;
/// Back-right paddle bit.
const EDGE_BRP: u8 = 0x80;

/// The decoded DualSense-Edge superset (Mute/Capture + Fn/paddle/side), all `false` unless the
/// source is Edge-capable. HW-verify (M6): see the module-level Edge note.
#[derive(Clone, Copy, Debug, Default)]
struct EdgeBits {
    mute: bool,
    capture: bool,
    fn_l: bool,
    fn_r: bool,
    blp: bool,
    brp: bool,
    side_l: bool,
    side_r: bool,
}

/// Decode the Edge superset from the extended report tail, gated by `is_edge`.
///
/// For a non-Edge source (or a buffer too short for the extended byte) every field is `false`, so
/// the decode is purely additive over M5. Side buttons (`side_l`/`side_r`) have no published stable
/// bit in this tail and stay `false` pending a hardware capture (documented for the maintainer).
#[inline]
fn decode_edge_bits(buf: &[u8], is_edge: bool) -> EdgeBits {
    if !is_edge || buf.len() <= EDGE_FN_BYTE {
        return EdgeBits::default();
    }
    let fb = buf[EDGE_FN_BYTE];
    EdgeBits {
        mute: fb & EDGE_MUTE != 0,
        // HW-verify: the real DualSense Mute/Create-extra ("Capture") bit is unknown. The previous
        // `btn2 & 0x04` decode COLLIDED with the frame-counter LSB (`counter = buf[7] >> 2` uses
        // bits 2..7), so it falsely fired every other report. Kept inert until a hardware capture,
        // exactly like `side_l`/`side_r`.
        capture: false,
        fn_l: fb & EDGE_FN_L != 0,
        fn_r: fb & EDGE_FN_R != 0,
        blp: fb & EDGE_BLP != 0,
        brp: fb & EDGE_BRP != 0,
        // No published stable bit yet — HW-verify follow-up (kept inert, never guessed-on).
        side_l: false,
        side_r: false,
    }
}

/// Decode buttons + triggers + (capability-gated) sensors/touch from an already-parsed
/// [`DsReport`] plus the full report buffer into the structured [`ControllerState`].
///
/// Sticks reuse [`ds_report_to_sticks`] verbatim (no duplicate offsets); triggers carry both
/// the analog `[0,1]` and the raw `u8`. The btn0/btn1/btn2 bit map is the one promoted from
/// `engine/src/win_io.rs::ds_buttons_to_xinput`:
/// * `btn0` (byte 5): low nibble = D-pad hat (decoded via [`decode_dpad_hat`]); high nibble =
///   Square `0x10`, Cross `0x20`, Circle `0x40`, Triangle `0x80`.
/// * `btn1` (byte 6): L1 `0x01`, R1 `0x02`, L2-click `0x04`, R2-click `0x08`, Share `0x10`,
///   Options `0x20`, L3 `0x40`, R3 `0x80`.
/// * `btn2` (byte 7): PS `0x01`, TouchButton `0x02` (upper 6 bits = frame counter, consumed).
///
/// **Capability gate:** Mute/Capture and the DualSense Edge Fn/paddle/side bits live in the
/// extended Edge report; they are decoded only when `meta.is_edge` is set, else read `false`.
/// Touch contacts and motion sensors are HW-verify (M5/M6) — left at their `Default` (`0`)
/// until those decodes land, so the Control variants are valid indices but inert.
pub fn decode_controller_state(r: &DsReport, buf: &[u8], meta: &SourceMeta) -> ControllerState {
    let (left, right) = ds_report_to_sticks(r);
    let (dpad_up, dpad_right, dpad_down, dpad_left) = decode_dpad_hat(r.btn0);

    // The Edge superset (Mute/Capture, Fn/paddle/side) is decoded only for Edge-capable sources
    // from the extended report tail (HW-verify); a non-Edge source reads every gated field `false`,
    // so this is byte-identical to M5 for the common DS4-compat path.
    let edge = decode_edge_bits(buf, meta.is_edge);
    // Touch contacts from the report tail (offsets 35..=42, HW-verify); two inactive `Default`
    // contacts for any buffer that does not reach the tail (M5 behavior preserved).
    let touch = decode_touch(buf);

    ControllerState {
        lx: left.x,
        ly: left.y,
        rx: right.x,
        ry: right.y,
        l2: u8_trigger(r.l2),
        r2: u8_trigger(r.r2),
        l2_raw: r.l2,
        r2_raw: r.r2,
        // Face buttons (high nibble of btn0).
        square: r.btn0 & 0x10 != 0,
        cross: r.btn0 & 0x20 != 0,
        circle: r.btn0 & 0x40 != 0,
        triangle: r.btn0 & 0x80 != 0,
        dpad_up,
        dpad_down,
        dpad_left,
        dpad_right,
        // Shoulders / stick clicks / meta (btn1).
        l1: r.btn1 & 0x01 != 0,
        r1: r.btn1 & 0x02 != 0,
        share: r.btn1 & 0x10 != 0,
        options: r.btn1 & 0x20 != 0,
        l3: r.btn1 & 0x40 != 0,
        r3: r.btn1 & 0x80 != 0,
        // System (btn2).
        ps: r.btn2 & 0x01 != 0,
        touch_button: r.btn2 & 0x02 != 0,
        // Edge superset (HW-verify, M6) — `false` for every non-Edge source via `decode_edge_bits`.
        mute: edge.mute,
        capture: edge.capture,
        fn_l: edge.fn_l,
        fn_r: edge.fn_r,
        blp: edge.blp,
        brp: edge.brp,
        side_l: edge.side_l,
        side_r: edge.side_r,
        // Touch contacts (HW-verify, M6) from the report tail; motion stays Default (M5 HW-verify).
        touch,
        motion: Default::default(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a synthetic 64-byte report with the given stick/trigger/counter/timestamp bytes.
    // Test fixture builder: one parameter per report byte we care about.
    #[allow(clippy::too_many_arguments)]
    fn synth(lx: u8, ly: u8, rx: u8, ry: u8, l2: u8, r2: u8, counter6: u8, ts: u16) -> [u8; 64] {
        let mut b = [0u8; 64];
        b[0] = DS_USB_REPORT_ID;
        b[1] = lx;
        b[2] = ly;
        b[3] = rx;
        b[4] = ry;
        b[7] = counter6 << 2; // frame counter lives in bits 2..8
        b[8] = l2;
        b[9] = r2;
        b[10] = (ts & 0xFF) as u8;
        b[11] = (ts >> 8) as u8;
        // Idle-pad touch tail: high bit SET on both id bytes (== inactive, the C# `IsActive` is
        // high-bit-CLEAR), so an unfilled report decodes two `Default` (inactive) contacts and the
        // M5 touch == Default behavior holds for every test that does not drive the touchpad.
        b[TOUCH_DATA_OFFSET] = 0x80;
        b[TOUCH_DATA_OFFSET + 4] = 0x80;
        b
    }

    /// Place an ACTIVE finger contact into a report buffer at the given finger slot (0 or 1).
    fn set_touch(b: &mut [u8; 64], finger: usize, id: u8, x: u16, y: u16) {
        let base = TOUCH_DATA_OFFSET + finger * 4;
        b[base] = id & 0x7F; // high bit clear => active
        b[base + 1] = (x & 0xFF) as u8;
        b[base + 2] = (((y & 0x0F) << 4) | ((x >> 8) & 0x0F)) as u8;
        b[base + 3] = (y >> 4) as u8;
    }

    #[test]
    fn rejects_short_or_wrong_id() {
        assert!(parse_ds_usb_report(&[0u8; 63]).is_none());
        let mut b = [0u8; 64];
        b[0] = 0x11; // BT report id, not the USB 0x01
        assert!(parse_ds_usb_report(&b).is_none());
    }

    #[test]
    fn parses_offsets() {
        let b = synth(0x10, 0x20, 0x30, 0x40, 0x55, 0x66, 0x2A, 0xBEEF);
        let r = parse_ds_usb_report(&b).expect("valid report");
        assert_eq!(r.lx, 0x10);
        assert_eq!(r.ly, 0x20);
        assert_eq!(r.rx, 0x30);
        assert_eq!(r.ry, 0x40);
        assert_eq!(r.l2, 0x55);
        assert_eq!(r.r2, 0x66);
        assert_eq!(r.counter, 0x2A);
        assert_eq!(r.sensor_ts, 0xBEEF);
    }

    #[test]
    fn timestamp_is_little_endian() {
        let mut b = synth(128, 128, 128, 128, 0, 0, 0, 0);
        b[10] = 0x34;
        b[11] = 0x12;
        let r = parse_ds_usb_report(&b).unwrap();
        assert_eq!(r.sensor_ts, 0x1234);
    }

    #[test]
    fn neutral_sticks_are_zero() {
        let b = synth(0x80, 0x80, 0x80, 0x80, 0, 0, 0, 0);
        let r = parse_ds_usb_report(&b).unwrap();
        let (l, rr) = ds_report_to_sticks(&r);
        assert_eq!(l, StickPair { x: 0.0, y: 0.0 });
        assert_eq!(rr, StickPair { x: 0.0, y: 0.0 });
    }

    #[test]
    fn up_is_positive_y() {
        // raw Y = 0x00 means stick pushed fully up -> canonical y = +1.0.
        let b = synth(0x80, 0x00, 0x80, 0x00, 0, 0, 0, 0);
        let r = parse_ds_usb_report(&b).unwrap();
        let (l, rr) = ds_report_to_sticks(&r);
        assert_eq!(l.y, 1.0);
        assert_eq!(rr.y, 1.0);
        // raw Y = 0xFF means fully down -> canonical y = -1.0.
        let b2 = synth(0x80, 0xFF, 0x80, 0xFF, 0, 0, 0, 0);
        let r2 = parse_ds_usb_report(&b2).unwrap();
        let (l2, _) = ds_report_to_sticks(&r2);
        assert_eq!(l2.y, -1.0);
    }

    #[test]
    fn x_axis_left_is_negative() {
        // raw X = 0x00 is left -> canonical x = -1.0 (not negated).
        let b = synth(0x00, 0x80, 0xFF, 0x80, 0, 0, 0, 0);
        let r = parse_ds_usb_report(&b).unwrap();
        let (l, rr) = ds_report_to_sticks(&r);
        assert_eq!(l.x, -1.0);
        assert_eq!(rr.x, 1.0);
    }

    #[test]
    fn button_bytes_carried_raw() {
        let mut b = synth(128, 128, 128, 128, 0, 0, 0, 0);
        b[5] = 0xA5;
        b[6] = 0x5A;
        b[7] = 0xC3; // counter = 0xC3 >> 2 = 0x30
        let r = parse_ds_usb_report(&b).unwrap();
        assert_eq!(r.btn0, 0xA5);
        assert_eq!(r.btn1, 0x5A);
        assert_eq!(r.btn2, 0xC3);
        assert_eq!(r.counter, 0x30);
    }

    // --- decode_controller_state ---

    use crate::input::control::{Control, Thresholds};

    const META: SourceMeta = SourceMeta {
        vid: 0x054C,
        pid: 0x0CE6,
        name: "test",
        stick_bits: 8,
        is_edge: false,
    };

    /// Build a report with explicit btn bytes (and neutral sticks/triggers).
    fn synth_btn(btn0: u8, btn1: u8, btn2: u8) -> [u8; 64] {
        let mut b = synth(0x80, 0x80, 0x80, 0x80, 0, 0, 0, 0);
        b[5] = btn0;
        b[6] = btn1;
        b[7] = btn2;
        b
    }

    fn decode(b: &[u8]) -> ControllerState {
        let r = parse_ds_usb_report(b).unwrap();
        decode_controller_state(&r, b, &META)
    }

    #[test]
    fn decode_sticks_equal_ds_report_to_sticks() {
        // No duplicate offsets: the state's sticks must equal ds_report_to_sticks exactly.
        let b = synth(0x12, 0x34, 0xCD, 0xEF, 0x55, 0xAA, 0, 0);
        let r = parse_ds_usb_report(&b).unwrap();
        let (l, rr) = ds_report_to_sticks(&r);
        let s = decode_controller_state(&r, &b, &META);
        assert_eq!(s.lx, l.x);
        assert_eq!(s.ly, l.y);
        assert_eq!(s.rx, rr.x);
        assert_eq!(s.ry, rr.y);
        // And triggers match u8_trigger + raw.
        assert_eq!(s.l2, u8_trigger(0x55));
        assert_eq!(s.r2, u8_trigger(0xAA));
        assert_eq!(s.l2_raw, 0x55);
        assert_eq!(s.r2_raw, 0xAA);
    }

    #[test]
    fn every_btn0_high_nibble_bit_flips_the_right_face_button() {
        // hat low nibble = 8 (neutral) so only face bits are exercised.
        assert!(decode(&synth_btn(0x18, 0, 0)).square);
        assert!(decode(&synth_btn(0x28, 0, 0)).cross);
        assert!(decode(&synth_btn(0x48, 0, 0)).circle);
        assert!(decode(&synth_btn(0x88, 0, 0)).triangle);
        // A bit set does not bleed into the others.
        let s = decode(&synth_btn(0x28, 0, 0));
        assert!(s.cross && !s.square && !s.circle && !s.triangle);
    }

    #[test]
    fn every_btn1_bit_flips_the_right_control() {
        assert!(decode(&synth_btn(8, 0x01, 0)).l1);
        assert!(decode(&synth_btn(8, 0x02, 0)).r1);
        assert!(decode(&synth_btn(8, 0x10, 0)).share);
        assert!(decode(&synth_btn(8, 0x20, 0)).options);
        assert!(decode(&synth_btn(8, 0x40, 0)).l3);
        assert!(decode(&synth_btn(8, 0x80, 0)).r3);
        let s = decode(&synth_btn(8, 0x40, 0));
        assert!(s.l3 && !s.r3 && !s.l1);
    }

    #[test]
    fn every_btn2_bit_flips_the_right_control() {
        assert!(decode(&synth_btn(8, 0, 0x01)).ps);
        assert!(decode(&synth_btn(8, 0, 0x02)).touch_button);
        let s = decode(&synth_btn(8, 0, 0x02));
        assert!(s.touch_button && !s.ps);
    }

    #[test]
    fn all_nine_dpad_nibbles_decode() {
        // (nibble, up, right, down, left)
        let cases = [
            (0u8, true, false, false, false),
            (1, true, true, false, false),
            (2, false, true, false, false),
            (3, false, true, true, false),
            (4, false, false, true, false),
            (5, false, false, true, true),
            (6, false, false, false, true),
            (7, true, false, false, true),
            (8, false, false, false, false),
        ];
        for (nib, up, right, down, left) in cases {
            let s = decode(&synth_btn(nib, 0, 0));
            assert_eq!(s.dpad_up, up, "nibble {nib} up");
            assert_eq!(s.dpad_right, right, "nibble {nib} right");
            assert_eq!(s.dpad_down, down, "nibble {nib} down");
            assert_eq!(s.dpad_left, left, "nibble {nib} left");
        }
    }

    #[test]
    fn edge_fields_inert_when_not_edge_capable() {
        let s = decode(&synth_btn(0xF8, 0xFF, 0xFF));
        // Edge superset stays false regardless of which raw bits are set.
        assert!(!s.mute && !s.capture && !s.fn_l && !s.fn_r);
        assert!(!s.blp && !s.brp && !s.side_l && !s.side_r);
        // Touch/motion inert.
        assert_eq!(s.touch, [crate::input::TouchContact::default(); 2]);
        assert_eq!(s.motion, crate::input::Motion::default());
    }

    #[test]
    fn full_pull_digital_view_from_raw_trigger() {
        let t = Thresholds::default();
        let mut b = synth(0x80, 0x80, 0x80, 0x80, 255, 100, 0, 0);
        b[5] = 8; // neutral hat
        let s = decode(&b);
        assert!(s.pressed(Control::L2FullPull, &t));
        assert!(!s.pressed(Control::R2FullPull, &t)); // 100 != 255
                                                      // analog ~ raw/255 within 1e-12.
        assert!((s.analog(Control::R2) - 100.0 / 255.0).abs() < 1e-12);
    }

    // ------------------------------------ M6: touch contacts -------------------------------------

    #[test]
    fn idle_report_decodes_two_inactive_contacts() {
        // The synth fixture writes the idle (high-bit-set) touch tail, so a plain report is two
        // inactive Default contacts (M5 behavior preserved).
        let s = decode(&synth_btn(8, 0, 0));
        assert_eq!(s.touch, [TouchContact::default(); 2]);
    }

    #[test]
    fn active_contact_decodes_id_and_12bit_xy() {
        // Finger 0 active, id 0x2A, x = 0x123 (291), y = 0x0F5 (245).
        let mut b = synth_btn(8, 0, 0);
        set_touch(&mut b, 0, 0x2A, 0x123, 0x0F5);
        let s = decode(&b);
        assert!(s.touch[0].is_active, "high-bit-clear id => active");
        assert_eq!(s.touch[0].id, 0x2A);
        assert_eq!(s.touch[0].x, 0x123);
        assert_eq!(s.touch[0].y, 0x0F5);
        // Finger 1 untouched stays inactive.
        assert!(!s.touch[1].is_active);
    }

    #[test]
    fn both_fingers_decode_independently() {
        let mut b = synth_btn(8, 0, 0);
        set_touch(&mut b, 0, 1, 100, 50);
        set_touch(&mut b, 1, 2, 1900, 900);
        let s = decode(&b);
        assert!(s.touch[0].is_active && s.touch[1].is_active);
        assert_eq!((s.touch[0].id, s.touch[0].x, s.touch[0].y), (1, 100, 50));
        assert_eq!((s.touch[1].id, s.touch[1].x, s.touch[1].y), (2, 1900, 900));
    }

    #[test]
    fn touch_xy_clamped_to_grid() {
        // A bogus all-ones X/Y (0xFFF == 4095) clamps to the grid maxima, never past the region.
        let mut b = synth_btn(8, 0, 0);
        set_touch(&mut b, 0, 0, 0xFFF, 0xFFF);
        let s = decode(&b);
        assert_eq!(s.touch[0].x, TOUCH_RES_X - 1, "x clamps to 1919");
        assert_eq!(s.touch[0].y, TOUCH_RES_Y - 1, "y clamps to 941");
    }

    #[test]
    fn short_buffer_keeps_default_touch() {
        // A buffer that parses (>= 64) but whose tail is the idle encoding yields Default; and the
        // tail-length guard means a hypothetical short slice never panics.
        assert_eq!(decode_touch(&[0u8; 10]), [TouchContact::default(); 2]);
        assert_eq!(decode_touch(&[0u8; 40]), [TouchContact::default(); 2]);
    }

    // ----------------------------- M6: DualSense Edge superset (gated) ----------------------------

    const EDGE_META: SourceMeta = SourceMeta {
        vid: 0x054C,
        pid: 0x0DF2,
        name: "edge",
        stick_bits: 8,
        is_edge: true,
    };

    fn decode_edge(b: &[u8]) -> ControllerState {
        let r = parse_ds_usb_report(b).unwrap();
        decode_controller_state(&r, b, &EDGE_META)
    }

    #[test]
    fn edge_bits_decode_only_when_edge_capable() {
        // Set every Fn/paddle/Mute bit in the extended byte. (Capture is inert pending a hardware
        // capture — its old `btn2 & 0x04` decode collided with the frame-counter LSB, so it is no
        // longer read from btn2.)
        let mut b = synth_btn(8, 0, 0);
        b[EDGE_FN_BYTE] = EDGE_MUTE | EDGE_FN_L | EDGE_FN_R | EDGE_BLP | EDGE_BRP;

        // Non-Edge source: every gated field stays false (byte-identical to M5).
        let non_edge = decode(&b);
        assert!(!non_edge.mute && !non_edge.capture && !non_edge.fn_l && !non_edge.fn_r);
        assert!(!non_edge.blp && !non_edge.brp);

        // Edge source: the extended-byte bits decode; capture/side stay inert (HW-verify).
        let edge = decode_edge(&b);
        assert!(edge.mute && edge.fn_l && edge.fn_r && edge.blp && edge.brp);
        assert!(
            !edge.capture,
            "capture is inert until its real bit is hardware-verified"
        );
        assert!(!edge.side_l && !edge.side_r);
    }

    #[test]
    fn edge_individual_bits_isolated() {
        let only_mute = {
            let mut b = synth_btn(8, 0, 0);
            b[EDGE_FN_BYTE] = EDGE_MUTE;
            decode_edge(&b)
        };
        assert!(only_mute.mute && !only_mute.fn_l && !only_mute.blp && !only_mute.capture);
    }
}
