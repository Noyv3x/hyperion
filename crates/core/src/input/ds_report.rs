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

use super::normalize::u8_stick_to_axis;
use super::StickPair;

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
}
