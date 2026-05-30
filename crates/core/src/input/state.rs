//! `ControllerState` — the fully-decoded physical controller report.
//!
//! This is the structured value the mapping engine reads (blueprint §3.2). It supersedes the
//! opaque `Buttons(u32)` of [`InputSample`](crate::input::InputSample) for remapping, while
//! [`ControllerState::to_input_sample`] still projects back to the stick-only `InputSample` the
//! existing hot loop consumes, with zero behavior change.
//!
//! Conventions match the rest of the core: sticks are canonical `[-1,1]` (`+y == up`),
//! triggers are `[0,1]` analog plus the raw `u8` (so `raw == 255` ⇒ full pull), gyro is rad/s,
//! accel is g. The struct is `Copy` and allocation-free.

use super::control::{Control, ControlKind, Thresholds};
use super::{Buttons, InputSample, ReportMeta, StickPair};

/// A single touchpad finger contact (blueprint §3.2). `x`/`y` span the DS4 touch grid
/// (`x: 0..=1919`, `y: 0..=941`); `id` is the 7-bit contact id; `is_active` is the touch-down
/// flag. Decoded fields are HW-verify (M6) — read inert until the touch decode lands.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TouchContact {
    /// Whether this finger is currently touching the pad.
    pub is_active: bool,
    /// The 7-bit hardware contact id.
    pub id: u8,
    /// Horizontal position, `0..=1919`.
    pub x: u16,
    /// Vertical position, `0..=941`.
    pub y: u16,
}

/// Decoded motion sensors (blueprint §3.2). Calibrated angular rate is rad/s and acceleration
/// is g; the raw `i16` ticks are retained so the full-scale constants can be corrected without
/// re-decoding (HW-verify, M5).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Motion {
    /// Gyro yaw rate, rad/s.
    pub gyro_yaw: f64,
    /// Gyro pitch rate, rad/s.
    pub gyro_pitch: f64,
    /// Gyro roll rate, rad/s.
    pub gyro_roll: f64,
    /// Accelerometer X, g.
    pub accel_x: f64,
    /// Accelerometer Y, g.
    pub accel_y: f64,
    /// Accelerometer Z, g.
    pub accel_z: f64,
    /// Raw gyro ticks `[x, y, z]` (fidelity / post-hoc rescale).
    pub gyro_raw: [i16; 3],
    /// Raw accel ticks `[x, y, z]` (fidelity / post-hoc rescale).
    pub accel_raw: [i16; 3],
}

/// A fully-decoded physical controller report.
///
/// `Copy`, ~200 bytes, allocation-free. Sticks are `[-1,1]` (`+y == up`); triggers are `[0,1]`
/// analog (`l2`/`r2`) plus the raw `u8` (`l2_raw`/`r2_raw`, so `raw == 255` ⇒ full pull).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ControllerState {
    /// Left-stick X, `[-1,1]` (`+` = right).
    pub lx: f64,
    /// Left-stick Y, `[-1,1]` (`+` = up).
    pub ly: f64,
    /// Right-stick X, `[-1,1]` (`+` = right).
    pub rx: f64,
    /// Right-stick Y, `[-1,1]` (`+` = up).
    pub ry: f64,
    /// Left trigger analog, `[0,1]`.
    pub l2: f64,
    /// Right trigger analog, `[0,1]`.
    pub r2: f64,
    /// Left trigger raw byte (`255` ⇒ full pull).
    pub l2_raw: u8,
    /// Right trigger raw byte (`255` ⇒ full pull).
    pub r2_raw: u8,
    /// Square face button.
    pub square: bool,
    /// Triangle face button.
    pub triangle: bool,
    /// Circle face button.
    pub circle: bool,
    /// Cross face button.
    pub cross: bool,
    /// D-pad up.
    pub dpad_up: bool,
    /// D-pad down.
    pub dpad_down: bool,
    /// D-pad left.
    pub dpad_left: bool,
    /// D-pad right.
    pub dpad_right: bool,
    /// L1 shoulder.
    pub l1: bool,
    /// R1 shoulder.
    pub r1: bool,
    /// L3 (left stick click).
    pub l3: bool,
    /// R3 (right stick click).
    pub r3: bool,
    /// PS / Guide button.
    pub ps: bool,
    /// Share / Create button.
    pub share: bool,
    /// Options button.
    pub options: bool,
    /// Mute button (Edge / DualSense; HW-verify).
    pub mute: bool,
    /// Capture button (HW-verify).
    pub capture: bool,
    /// Edge left function button (HW-verify).
    pub fn_l: bool,
    /// Edge right function button (HW-verify).
    pub fn_r: bool,
    /// Edge back-left paddle (HW-verify).
    pub blp: bool,
    /// Edge back-right paddle (HW-verify).
    pub brp: bool,
    /// Edge left side button (HW-verify).
    pub side_l: bool,
    /// Edge right side button (HW-verify).
    pub side_r: bool,
    /// Touchpad click.
    pub touch_button: bool,
    /// The two touchpad finger contacts.
    pub touch: [TouchContact; 2],
    /// Decoded motion sensors.
    pub motion: Motion,
}

impl ControllerState {
    /// Native-unit analog view of a control.
    ///
    /// Sticks are signed `[-1,1]` collapsed to a non-negative half-axis (`LxPos => lx.max(0)`,
    /// `LxNeg => (-lx).max(0)`); triggers/outer-ring are `[0,1]`; gyro is rad/s; plain buttons
    /// read `0.0`/`1.0`. This is the continuous value the `Passthrough` arm consumes.
    pub fn analog(&self, c: Control) -> f64 {
        use Control::*;
        match c {
            None => 0.0,
            // Stick half-axes (non-negative magnitude of the signed component).
            LxNeg => (-self.lx).max(0.0),
            LxPos => self.lx.max(0.0),
            LyNeg => (-self.ly).max(0.0),
            LyPos => self.ly.max(0.0),
            RxNeg => (-self.rx).max(0.0),
            RxPos => self.rx.max(0.0),
            RyNeg => (-self.ry).max(0.0),
            RyPos => self.ry.max(0.0),
            // Analog triggers (the full-pull variants share the same analog value).
            L2 | L2FullPull => self.l2,
            R2 | R2FullPull => self.r2,
            // Stick outer-ring magnitude (radial distance, clamped to 1).
            LsOuter => (self.lx * self.lx + self.ly * self.ly).sqrt().min(1.0),
            RsOuter => (self.rx * self.rx + self.ry * self.ry).sqrt().min(1.0),
            // Gyro directional rates (non-negative half).
            GyroXPos => self.motion.gyro_pitch.max(0.0),
            GyroXNeg => (-self.motion.gyro_pitch).max(0.0),
            GyroZPos => self.motion.gyro_yaw.max(0.0),
            GyroZNeg => (-self.motion.gyro_yaw).max(0.0),
            // Plain digital controls collapse to 0/1.
            _ => {
                if self.button(c) {
                    1.0
                } else {
                    0.0
                }
            }
        }
    }

    /// Digital "pressed" view, applying kind-dependent [`Thresholds`] (verifier FIX 3).
    ///
    /// Buttons read their decoded flag; axis half-directions compare against
    /// `t.stick_dir` (55/127); analog triggers against `t.trigger` (100/255);
    /// `L2FullPull`/`R2FullPull` require the raw byte `== 255`; gyro directions against
    /// `t.gyro_dir`.
    pub fn pressed(&self, c: Control, t: &Thresholds) -> bool {
        use Control::*;
        match c {
            L2FullPull => self.l2_raw == 255,
            R2FullPull => self.r2_raw == 255,
            _ => match c.kind() {
                ControlKind::AxisDir => self.analog(c) >= t.stick_dir,
                ControlKind::Trigger => self.analog(c) >= t.trigger,
                ControlKind::GyroDir => self.analog(c) >= t.gyro_dir,
                ControlKind::Button | ControlKind::Touch => self.button(c),
            },
        }
    }

    /// Raw decoded boolean for a digital control (no threshold). Analog controls read `false`
    /// here — use [`pressed`](Self::pressed) for their digitized view.
    fn button(&self, c: Control) -> bool {
        use Control::*;
        match c {
            Square => self.square,
            Triangle => self.triangle,
            Circle => self.circle,
            Cross => self.cross,
            DpadUp => self.dpad_up,
            DpadDown => self.dpad_down,
            DpadLeft => self.dpad_left,
            DpadRight => self.dpad_right,
            L1 => self.l1,
            R1 => self.r1,
            L3 => self.l3,
            R3 => self.r3,
            Ps => self.ps,
            Share => self.share,
            Options => self.options,
            Mute => self.mute,
            Capture => self.capture,
            FnL => self.fn_l,
            FnR => self.fn_r,
            Blp => self.blp,
            Brp => self.brp,
            SideL => self.side_l,
            SideR => self.side_r,
            TouchButton => self.touch_button,
            _ => false,
        }
    }

    /// Project to the stick-only [`InputSample`] the existing hot loop consumes (no regression).
    ///
    /// Sticks/triggers carry through bit-for-bit; `buttons` re-packs the decoded digital state
    /// into the same `btn0 | btn1<<8 | btn2<<16` layout the HID backend produced (the layout
    /// `win_io::ds_buttons_to_xinput` consumes); seq/dt/host come from `meta`.
    pub fn to_input_sample(&self, m: &ReportMeta) -> InputSample {
        InputSample {
            left: StickPair {
                x: self.lx,
                y: self.ly,
            },
            right: StickPair {
                x: self.rx,
                y: self.ry,
            },
            l2: self.l2,
            r2: self.r2,
            buttons: Buttons(self.pack_ds_buttons()),
            seq: m.seq,
            dropped: m.dropped,
            is_duplicate: m.is_duplicate,
            dt_us: m.dt_us,
            host_qpc_ns: m.host_qpc_ns,
        }
    }

    /// Re-pack the decoded digital state into the raw DS `btn0|btn1<<8|btn2<<16` u32 layout.
    ///
    /// Inverse of [`decode_controller_state`](crate::input::ds_report::decode_controller_state)'s
    /// button decode (the single source of truth in `ds_report`/`win_io`). The btn2 frame-
    /// counter bits are not reconstructed (the downstream X360 packer masks them off), so this
    /// reproduces every bit `ds_buttons_to_xinput` reads.
    fn pack_ds_buttons(&self) -> u32 {
        let hat = encode_dpad_hat(
            self.dpad_up,
            self.dpad_right,
            self.dpad_down,
            self.dpad_left,
        );
        let mut btn0 = hat;
        if self.square {
            btn0 |= 0x10;
        }
        if self.cross {
            btn0 |= 0x20;
        }
        if self.circle {
            btn0 |= 0x40;
        }
        if self.triangle {
            btn0 |= 0x80;
        }

        let mut btn1 = 0u8;
        if self.l1 {
            btn1 |= 0x01;
        }
        if self.r1 {
            btn1 |= 0x02;
        }
        if self.l2_raw == 255 {
            btn1 |= 0x04; // L2 click (full pull); mirrors the C# L2Btn for the digital view.
        }
        if self.r2_raw == 255 {
            btn1 |= 0x08;
        }
        if self.share {
            btn1 |= 0x10;
        }
        if self.options {
            btn1 |= 0x20;
        }
        if self.l3 {
            btn1 |= 0x40;
        }
        if self.r3 {
            btn1 |= 0x80;
        }

        let mut btn2 = 0u8;
        if self.ps {
            btn2 |= 0x01;
        }
        if self.touch_button {
            btn2 |= 0x02;
        }

        u32::from(btn0) | (u32::from(btn1) << 8) | (u32::from(btn2) << 16)
    }
}

/// Encode four d-pad direction bools back into the 4-bit DS hat nibble (`0..=7`, `8` neutral).
/// Inverse of the hat-nibble decode; diagonals are encoded, opposing presses collapse to the
/// nearest cardinal (`up` dominates the vertical pair, `right` the horizontal).
#[inline]
fn encode_dpad_hat(up: bool, right: bool, down: bool, left: bool) -> u8 {
    // Resolve opposing presses the same way the decode never produces them.
    let (u, d) = if up { (true, false) } else { (false, down) };
    let (r, l) = if right { (true, false) } else { (false, left) };
    // (up, right, down, left) -> hat nibble (0=N..7=NW, 8=neutral).
    match (u, r, d, l) {
        (true, false, false, false) => 0,
        (true, true, false, false) => 1,
        (false, true, false, false) => 2,
        (false, true, true, false) => 3,
        (false, false, true, false) => 4,
        (false, false, true, true) => 5,
        (false, false, false, true) => 6,
        (true, false, false, true) => 7,
        _ => 8,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn neutral() -> ControllerState {
        ControllerState::default()
    }

    #[test]
    fn analog_half_axes_split_sign() {
        let mut s = neutral();
        s.lx = 0.6;
        assert_eq!(s.analog(Control::LxPos), 0.6);
        assert_eq!(s.analog(Control::LxNeg), 0.0);
        s.lx = -0.4;
        assert_eq!(s.analog(Control::LxPos), 0.0);
        assert!((s.analog(Control::LxNeg) - 0.4).abs() < 1e-12);
        s.ry = 0.9;
        assert!((s.analog(Control::RyPos) - 0.9).abs() < 1e-12);
        assert_eq!(s.analog(Control::RyNeg), 0.0);
    }

    #[test]
    fn analog_triggers_and_outer_ring() {
        let mut s = neutral();
        s.l2 = 0.75;
        assert_eq!(s.analog(Control::L2), 0.75);
        assert_eq!(s.analog(Control::L2FullPull), 0.75);
        // Outer ring is radial distance, clamped to 1.
        s.lx = 0.6;
        s.ly = 0.8; // |.| = 1.0
        assert!((s.analog(Control::LsOuter) - 1.0).abs() < 1e-12);
        s.lx = 1.0;
        s.ly = 1.0; // sqrt(2) -> clamp 1.0
        assert_eq!(s.analog(Control::LsOuter), 1.0);
    }

    #[test]
    fn pressed_axis_threshold_at_55_over_127() {
        let t = Thresholds::default();
        let mut s = neutral();
        // Just below 55/127 -> not pressed; at/above -> pressed.
        s.lx = 55.0 / 127.0 - 1e-9;
        assert!(!s.pressed(Control::LxPos, &t));
        s.lx = 55.0 / 127.0;
        assert!(s.pressed(Control::LxPos, &t));
        s.lx = 55.0 / 127.0 + 1e-6;
        assert!(s.pressed(Control::LxPos, &t));
    }

    #[test]
    fn pressed_trigger_threshold_at_100_over_255() {
        let t = Thresholds::default();
        let mut s = neutral();
        s.l2 = 100.0 / 255.0 - 1e-9;
        assert!(!s.pressed(Control::L2, &t));
        s.l2 = 100.0 / 255.0;
        assert!(s.pressed(Control::L2, &t));
    }

    #[test]
    fn full_pull_requires_raw_255() {
        let t = Thresholds::default();
        let mut s = neutral();
        s.r2 = 1.0;
        s.r2_raw = 254;
        assert!(!s.pressed(Control::R2FullPull, &t));
        s.r2_raw = 255;
        assert!(s.pressed(Control::R2FullPull, &t));
    }

    #[test]
    fn pressed_reads_digital_buttons() {
        let t = Thresholds::default();
        let mut s = neutral();
        assert!(!s.pressed(Control::Cross, &t));
        s.cross = true;
        assert!(s.pressed(Control::Cross, &t));
        s.dpad_up = true;
        assert!(s.pressed(Control::DpadUp, &t));
    }

    #[test]
    fn to_input_sample_reproduces_sticks_triggers_seq_dt() {
        let mut s = neutral();
        s.lx = 0.123_456_789;
        s.ly = -0.987_654_321;
        s.rx = 0.5;
        s.ry = -0.25;
        s.l2 = 0.4;
        s.r2 = 0.6;
        let m = ReportMeta {
            seq: 42,
            dropped: 3,
            is_duplicate: true,
            dt_us: 16.0 / 3.0,
            host_qpc_ns: 9_999,
        };
        let smp = s.to_input_sample(&m);
        // Sticks carry the canonical (+y up) values through unchanged.
        assert_eq!(smp.left.x, 0.123_456_789);
        assert_eq!(smp.left.y, -0.987_654_321);
        assert_eq!(smp.right.x, 0.5);
        assert_eq!(smp.right.y, -0.25);
        assert_eq!(smp.l2, 0.4);
        assert_eq!(smp.r2, 0.6);
        assert_eq!(smp.seq, 42);
        assert_eq!(smp.dropped, 3);
        assert!(smp.is_duplicate);
        assert_eq!(smp.dt_us, 16.0 / 3.0);
        assert_eq!(smp.host_qpc_ns, 9_999);
    }

    #[test]
    fn to_input_sample_button_packing_matches_ds_layout() {
        let mut s = neutral();
        s.cross = true; // btn0 0x20
        s.l1 = true; // btn1 0x01
        s.options = true; // btn1 0x20
        s.ps = true; // btn2 0x01
        s.touch_button = true; // btn2 0x02
        s.dpad_up = true; // hat 0
        let raw = s.to_input_sample(&ReportMeta::default()).buttons.0;
        let btn0 = (raw & 0xFF) as u8;
        let btn1 = ((raw >> 8) & 0xFF) as u8;
        let btn2 = ((raw >> 16) & 0xFF) as u8;
        assert_eq!(btn0 & 0x0F, 0, "hat nibble = N (0)");
        assert_eq!(btn0 & 0x20, 0x20, "cross bit");
        assert_eq!(btn1 & 0x01, 0x01, "L1 bit");
        assert_eq!(btn1 & 0x20, 0x20, "options bit");
        assert_eq!(btn2 & 0x01, 0x01, "PS bit");
        assert_eq!(btn2 & 0x02, 0x02, "touch button bit");
    }

    #[test]
    fn dpad_hat_encode_decode_round_trips_all_directions() {
        // Encode every cardinal/diagonal and confirm the nibble matches the decode table.
        let cases = [
            (true, false, false, false, 0u8), // N
            (true, true, false, false, 1),    // NE
            (false, true, false, false, 2),   // E
            (false, true, true, false, 3),    // SE
            (false, false, true, false, 4),   // S
            (false, false, true, true, 5),    // SW
            (false, false, false, true, 6),   // W
            (true, false, false, true, 7),    // NW
            (false, false, false, false, 8),  // neutral
        ];
        for (u, r, d, l, want) in cases {
            assert_eq!(encode_dpad_hat(u, r, d, l), want, "({u},{r},{d},{l})");
        }
    }
}
