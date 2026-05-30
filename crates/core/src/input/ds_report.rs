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

use super::normalize::{u8_stick_to_axis, u8_trigger};
use super::state::ControllerState;
use super::{SourceMeta, StickPair};

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
pub fn decode_controller_state(r: &DsReport, _buf: &[u8], meta: &SourceMeta) -> ControllerState {
    let (left, right) = ds_report_to_sticks(r);
    let (dpad_up, dpad_right, dpad_down, dpad_left) = decode_dpad_hat(r.btn0);

    // The Edge superset (Mute/Capture, Fn/paddle/side) is decoded only for Edge-capable
    // sources; the bit positions live in the extended report and land in M6. Until then every
    // gated field reads `false` even on an Edge source, but the gate is wired so M6 is additive.
    let _is_edge = meta.is_edge;

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
        // Edge superset (HW-verify) — inert until the M6 extended-report decode lands.
        mute: false,
        capture: false,
        fn_l: false,
        fn_r: false,
        blp: false,
        brp: false,
        side_l: false,
        side_r: false,
        // Touch contacts + motion: HW-verify (M5/M6), inert at Default.
        touch: Default::default(),
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
        b
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
}
