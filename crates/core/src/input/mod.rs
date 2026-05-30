//! Device-agnostic input parsing, normalization, and time-base folding.
//!
//! The Windows HID shell does I/O only: it hands a raw `&[u8]` report plus a host QPC
//! timestamp to the pure parsers here, which produce a canonical [`InputSample`] in the
//! `f64` `[-1,1]` / `[0,1]` units the rest of the engine speaks. Everything in this module
//! is OS-free and unit-tested on Linux CI.
//!
//! Submodules:
//! * [`normalize`] — raw 8/16-bit integer → canonical unit maps.
//! * [`ds_report`] — DualSense (DS4-compatible report `0x01`) byte decode.
//! * [`control`] — the [`Control`] mapping-table key + [`ControlKind`] + [`Thresholds`].
//! * [`state`] — the structured [`ControllerState`] (decoded physical report) + [`Motion`] +
//!   [`TouchContact`].
//! * [`dt_clock`] — the [`SensorClock`] that folds the hardware timestamp into a guarded `dt`.
//! * [`seq`] — the [`SeqTracker`] that derives dropped/duplicate counts from the frame counter.

pub mod control;
pub mod ds_report;
pub mod dt_clock;
pub mod normalize;
pub mod seq;
pub mod state;

pub use control::{Control, ControlKind, Thresholds};
pub use dt_clock::SensorClock;
pub use seq::SeqTracker;
pub use state::{ControllerState, Motion, TouchContact};

/// One stick's high-precision X/Y in the canonical `[-1,1]` unit (`+y == up`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StickPair {
    pub x: f64,
    pub y: f64,
}

/// A packed device button bitfield (layout is device-specific; carried opaquely).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Buttons(pub u32);

/// A fully-normalized input report the device layer produces for the engine.
///
/// Sticks are canonical `[-1,1]` (`+y == up`), triggers `[0,1]`. `seq` is the device frame
/// counter; `dropped`/`is_duplicate` are derived from it. `dt_us` is the guarded real
/// elapsed time since the previous report (`0.0` on the priming report), and `host_qpc_ns`
/// is the host high-resolution timestamp captured at read completion.
#[derive(Clone, Copy, Debug, Default)]
pub struct InputSample {
    pub left: StickPair,
    pub right: StickPair,
    pub l2: f64,
    pub r2: f64,
    pub buttons: Buttons,
    pub seq: u8,
    pub dropped: u16,
    pub is_duplicate: bool,
    pub dt_us: f64,
    pub host_qpc_ns: u64,
}

/// Static identity of an input source: USB IDs, a human label, the stick bit depth, and the
/// DualSense Edge capability flag.
///
/// `is_edge` gates the Edge-superset decode (Mute/Capture, Fn/paddle/side buttons) in
/// [`decode_controller_state`](ds_report::decode_controller_state): those `Control` variants are
/// always valid table indices, but read `false` unless the source advertises the Edge extended
/// report (blueprint §3.5).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SourceMeta {
    pub vid: u16,
    pub pid: u16,
    pub name: &'static str,
    pub stick_bits: u8,
    /// Whether this source exposes the DualSense Edge extended report (Fn/paddle/Mute/Capture).
    pub is_edge: bool,
}

/// Derived per-report metadata that pairs with a decoded [`ControllerState`].
///
/// `seq` is the device frame counter; `dropped`/`is_duplicate` are derived from it by the
/// [`SeqTracker`]; `dt_us` is the guarded elapsed time from the [`SensorClock`] (`0.0` on the
/// priming report); `host_qpc_ns` is the host timestamp captured at read completion. These are
/// exactly the fields [`ControllerState::to_input_sample`](state::ControllerState::to_input_sample)
/// folds back into an [`InputSample`].
#[derive(Clone, Copy, Debug, Default)]
pub struct ReportMeta {
    pub seq: u8,
    pub dropped: u16,
    pub is_duplicate: bool,
    pub dt_us: f64,
    pub host_qpc_ns: u64,
}

/// Full parse entry: raw report buffer + host time → `(decoded state, derived meta)`.
///
/// Returns `None` on a short/wrong-id buffer (the same guard as
/// [`parse_ds_usb_report`](ds_report::parse_ds_usb_report)). Folds the existing [`SeqTracker`]
/// and [`SensorClock`] unchanged, then decodes the structured [`ControllerState`] via
/// [`decode_controller_state`](ds_report::decode_controller_state). The stick-only hot loop can
/// still obtain its [`InputSample`] from
/// [`ControllerState::to_input_sample`](state::ControllerState::to_input_sample) with the
/// returned [`ReportMeta`], byte-identically to the legacy path.
pub fn parse_controller_state(
    buf: &[u8],
    host_qpc_ns: u64,
    meta: &SourceMeta,
    seq: &mut SeqTracker,
    clock: &mut SensorClock,
) -> Option<(ControllerState, ReportMeta)> {
    let report = ds_report::parse_ds_usb_report(buf)?;
    let dt_us = clock.fold(report.sensor_ts, host_qpc_ns);
    let (dropped, is_duplicate) = seq.update(report.counter);
    let state = ds_report::decode_controller_state(&report, buf, meta);
    let rmeta = ReportMeta {
        seq: report.counter,
        dropped,
        is_duplicate,
        dt_us,
        host_qpc_ns,
    };
    Some((state, rmeta))
}

#[cfg(test)]
mod tests {
    use super::*;

    const META: SourceMeta = SourceMeta {
        vid: 0x054C,
        pid: 0x0CE6,
        name: "test",
        stick_bits: 8,
        is_edge: false,
    };

    fn synth(lx: u8, ly: u8, l2: u8, counter6: u8, ts: u16) -> [u8; 64] {
        let mut b = [0u8; 64];
        b[0] = ds_report::DS_USB_REPORT_ID;
        b[1] = lx;
        b[2] = ly;
        b[3] = 0x80;
        b[4] = 0x80;
        b[7] = counter6 << 2;
        b[8] = l2;
        b[10] = (ts & 0xFF) as u8;
        b[11] = (ts >> 8) as u8;
        b
    }

    #[test]
    fn parse_controller_state_rejects_bad_buffer() {
        let mut seq = SeqTracker::default();
        let mut clock = SensorClock::default();
        assert!(parse_controller_state(&[0u8; 10], 0, &META, &mut seq, &mut clock).is_none());
    }

    #[test]
    fn parse_controller_state_folds_seq_and_dt_like_legacy_path() {
        let mut seq = SeqTracker::default();
        let mut clock = SensorClock::default();
        // First report primes dt to 0 and seq to itself.
        let b0 = synth(0x80, 0x80, 0x40, 5, 1000);
        let (s0, m0) = parse_controller_state(&b0, 1_000, &META, &mut seq, &mut clock).unwrap();
        assert_eq!(m0.seq, 5);
        assert_eq!(m0.dropped, 0);
        assert_eq!(m0.dt_us, 0.0, "first fold primes");
        assert_eq!(s0.l2, super::normalize::u8_trigger(0x40));

        // Second report: 3 ticks later -> 16us, one dropped (counter jumps 5->7).
        let b1 = synth(0x80, 0x80, 0x40, 7, 1003);
        let (_s1, m1) = parse_controller_state(&b1, 2_000, &META, &mut seq, &mut clock).unwrap();
        assert_eq!(m1.seq, 7);
        assert_eq!(m1.dropped, 1);
        assert!((m1.dt_us - 16.0).abs() < 1e-12);

        // And to_input_sample reproduces the sticks/triggers/seq/dt.
        let smp = s0.to_input_sample(&m0);
        assert_eq!(smp.seq, 5);
        assert_eq!(smp.dt_us, 0.0);
        assert_eq!(smp.l2, super::normalize::u8_trigger(0x40));
    }

    #[test]
    fn parse_controller_state_matches_legacy_sticks_and_buttons() {
        // Cross-check against the legacy parse path: same sticks, triggers, and the meaningful
        // (X360-relevant) button bits survive the decode -> re-pack round trip.
        let mut b = synth(0x12, 0x34, 0x55, 0, 0);
        b[5] = 0x28; // hat=8 (neutral) + Cross (0x20)
        b[6] = 0x01; // L1
        let mut seq = SeqTracker::default();
        let mut clock = SensorClock::default();
        let (s, m) = parse_controller_state(&b, 0, &META, &mut seq, &mut clock).unwrap();

        let legacy_report = ds_report::parse_ds_usb_report(&b).unwrap();
        let (l, r) = ds_report::ds_report_to_sticks(&legacy_report);
        assert_eq!(s.lx, l.x);
        assert_eq!(s.ly, l.y);
        assert_eq!(s.rx, r.x);
        assert_eq!(s.ry, r.y);

        let smp = s.to_input_sample(&m);
        let btn0 = (smp.buttons.0 & 0xFF) as u8;
        let btn1 = ((smp.buttons.0 >> 8) & 0xFF) as u8;
        assert_eq!(btn0 & 0x20, 0x20, "cross re-packed");
        assert_eq!(btn1 & 0x01, 0x01, "L1 re-packed");
    }
}
