//! Input-layer parsing and timing: the DS USB report decoder, the corrected u16/16-3µs
//! `SensorClock` (wrap + QPC fallback), and the `SeqTracker` drop/duplicate counter.
//!
//! `SensorClock` and `SeqTracker` are fully specified standalone types (DESIGN §7,
//! resolved-conflict #1), so they are tested directly. The DS report decoder is tested
//! through its documented gates and the SOLID stick offsets (bytes 1-4) plus the
//! up-is-positive Y convention.

use hyperion_core::input::ds_report::{
    ds_report_to_sticks, parse_ds_usb_report, DS_USB_REPORT_ID, DS_USB_REPORT_LEN,
};
use hyperion_core::input::{SensorClock, SeqTracker};

/// A neutral 64-byte DS USB report: report id 0x01, all four stick axes centered at 0x80.
fn neutral_report() -> [u8; DS_USB_REPORT_LEN] {
    let mut buf = [0u8; DS_USB_REPORT_LEN];
    buf[0] = DS_USB_REPORT_ID;
    // SOLID stick offsets (bytes 1..4): LX, LY, RX, RY centered.
    buf[1] = 0x80;
    buf[2] = 0x80;
    buf[3] = 0x80;
    buf[4] = 0x80;
    buf
}

#[test]
fn parse_rejects_short_or_wrong_id() {
    // Too short.
    assert!(parse_ds_usb_report(&[0x01u8; 8]).is_none());
    // Wrong report id.
    let mut buf = neutral_report();
    buf[0] = 0x11;
    assert!(parse_ds_usb_report(&buf).is_none());
    // Exactly the right length and id parses.
    assert!(parse_ds_usb_report(&neutral_report()).is_some());
}

#[test]
fn neutral_report_yields_centered_sticks() {
    let r = parse_ds_usb_report(&neutral_report()).expect("neutral report parses");
    let (left, right) = ds_report_to_sticks(&r);
    // Center 0x80 -> exactly 0.0 on both axes of both sticks.
    assert_eq!(left.x, 0.0);
    assert_eq!(left.y, 0.0);
    assert_eq!(right.x, 0.0);
    assert_eq!(right.y, 0.0);
}

#[test]
fn stick_x_right_positive_and_y_up_positive() {
    // LX raw 0xFF (full right) -> +1 on x. LY raw 0x00 (full up on DS) -> +1 on y
    // (the Y axis is negated so "up" is positive in the canonical unit).
    let mut buf = neutral_report();
    buf[1] = 0xFF; // LX full
    buf[2] = 0x00; // LY at the small-raw end == up
    let r = parse_ds_usb_report(&buf).expect("parses");
    let (left, _right) = ds_report_to_sticks(&r);
    assert!(left.x > 0.99, "raw 0xFF LX should be ~+1, got {}", left.x);
    assert!(
        left.y > 0.99,
        "raw 0x00 LY (up) should be ~+1 after negation, got {}",
        left.y
    );

    // And the opposite ends.
    let mut buf = neutral_report();
    buf[1] = 0x00; // LX full left
    buf[2] = 0xFF; // LY full down
    let r = parse_ds_usb_report(&buf).expect("parses");
    let (left, _right) = ds_report_to_sticks(&r);
    assert!(left.x < -0.99, "raw 0x00 LX should be ~-1, got {}", left.x);
    assert!(
        left.y < -0.99,
        "raw 0xFF LY (down) should be ~-1, got {}",
        left.y
    );
}

// ---- SensorClock: u16 wrap + 16/3 µs/tick + QPC fallback (resolved-conflict #1) ----

#[test]
fn sensor_clock_primes_then_uses_tick_delta() {
    let mut clk = SensorClock::default();
    // First fold primes -> dt 0.0 (caller takes no filter step).
    assert_eq!(clk.fold(1000, 0), 0.0);
    // Next stamp 1003 -> 3 ticks * 16/3 = 16 us.
    let dt = clk.fold(1003, 1_000_000);
    assert!((dt - 16.0).abs() < 1e-9, "3 ticks should be 16us, got {dt}");
}

#[test]
fn sensor_clock_wraps_at_u16_modulus() {
    let mut clk = SensorClock::default();
    clk.fold(65534, 0); // prime near the top
                        // 65534 -> 1 : wrapping_sub = 3 ticks (65535 -> 0 -> 1), i.e. 1u16.wrapping_sub(65534)=3.
    let dt = clk.fold(1, 1_000);
    // ticks = 1 - 65534 mod 65536 = 3 -> 3 * 16/3 = 16 us.
    assert!(
        (dt - 16.0).abs() < 1e-9,
        "u16 wrap should give 16us, got {dt}"
    );
}

#[test]
fn sensor_clock_identical_stamp_falls_back_to_qpc() {
    let mut clk = SensorClock::default();
    clk.fold(5000, 10_000); // prime
                            // Same stamp -> ticks 0 -> use QPC delta (host_qpc_ns - prev)/1000 us.
    let dt = clk.fold(5000, 10_000 + 7_000); // +7000 ns = 7 us
    assert!(
        (dt - 7.0).abs() < 1e-9,
        "identical stamp should use QPC 7us, got {dt}"
    );
}

#[test]
fn sensor_clock_clamps_to_20ms_ceiling() {
    let mut clk = SensorClock::default();
    clk.fold(0, 0);
    // A huge tick delta clamps to the 20_000 us ceiling.
    let dt = clk.fold(60000, 0);
    assert!(
        dt <= 20_000.0 + 1e-9,
        "dt must be clamped to <=20000us, got {dt}"
    );
    assert!(
        dt >= 20_000.0 - 1e-9,
        "this large delta should hit the ceiling, got {dt}"
    );
}

#[test]
fn sensor_clock_advances_qpc_even_on_tick_path() {
    // After a device-tick step, the FIRST QPC-fallback must use the just-advanced prev_qpc,
    // not a stale one (otherwise it returns a huge accumulated dt).
    let mut clk = SensorClock::default();
    clk.fold(100, 1_000_000); // prime
    clk.fold(105, 2_000_000); // tick path; must still advance prev_qpc to 2_000_000
                              // identical stamp now -> QPC delta from 2_000_000, not 1_000_000.
    let dt = clk.fold(105, 2_000_000 + 5_000); // +5000 ns = 5 us
    assert!(
        (dt - 5.0).abs() < 1e-9,
        "prev_qpc must advance on the tick path, got {dt}"
    );
}

// ---- SeqTracker: mod-256 drop counting + duplicate detection ----

#[test]
fn seq_tracker_first_update_has_no_drops() {
    let mut seq = SeqTracker::default();
    let (dropped, dup) = seq.update(10);
    assert_eq!(dropped, 0);
    assert!(!dup);
}

#[test]
fn seq_tracker_counts_gaps_and_duplicates() {
    let mut seq = SeqTracker::default();
    seq.update(10);
    // Consecutive -> 0 dropped.
    let (d, dup) = seq.update(11);
    assert_eq!(d, 0);
    assert!(!dup);
    // Skip 12,13 -> next is 14: dropped = 14-11-1 = 2.
    let (d, dup) = seq.update(14);
    assert_eq!(d, 2);
    assert!(!dup);
    // Duplicate of 14.
    let (d, dup) = seq.update(14);
    assert!(dup, "repeat of the same seq is a duplicate");
    // dropped = 14-14-1 mod 256 = 255 for a duplicate (wrap), per the spec formula.
    assert_eq!(d, 255);
}

#[test]
fn seq_tracker_wraps_mod_256() {
    let mut seq = SeqTracker::default();
    seq.update(254);
    // 254 -> 1: dropped = 1 - 254 - 1 mod 256 = (1u8.wrapping_sub(254).wrapping_sub(1)) = 2.
    let (d, dup) = seq.update(1);
    assert_eq!(d, 2, "mod-256 wrap drop count");
    assert!(!dup);
}

#[test]
fn seq_tracker_reset_clears_history() {
    let mut seq = SeqTracker::default();
    seq.update(50);
    seq.reset();
    // After reset, the next update is treated as a first update (no drops, no dup).
    let (d, dup) = seq.update(200);
    assert_eq!(d, 0);
    assert!(!dup);
}
