//! Precision-conversion invariants: anchors land exactly, the axis<->ds4 round-trip is
//! lossless, the 8-bit raw maps losslessly through axis to 16-bit, and the final XInput
//! egress quantizes exactly once.

use hyperion_core::convert::{axis_to_ds4, ds4_to_axis};
use hyperion_core::output::{to_xinput_thumb, to_xinput_trigger};
use hyperion_core::RawStickFormat;

#[test]
fn axis_ds4_anchors_are_exact() {
    assert_eq!(axis_to_ds4(0.0), 128.0);
    assert_eq!(axis_to_ds4(-1.0), 0.0);
    assert_eq!(axis_to_ds4(1.0), 255.0);
    assert_eq!(ds4_to_axis(128.0), 0.0);
    assert_eq!(ds4_to_axis(0.0), -1.0);
    assert_eq!(ds4_to_axis(255.0), 1.0);
}

#[test]
fn axis_ds4_round_trip_is_lossless() {
    for i in 0..=4000 {
        let a = -1.0 + (i as f64) / 2000.0;
        let back = ds4_to_axis(axis_to_ds4(a));
        assert!((a - back).abs() < 1e-12, "a={a} back={back}");
    }
}

#[test]
fn raw8_to_axis_to_raw16_single_round_is_clean() {
    // DS 8-bit center 128 -> axis 0.0 -> XInput thumb 0.
    let f8 = RawStickFormat::DS_8BIT;
    assert_eq!(f8.to_axis(128), 0.0);
    assert_eq!(to_xinput_thumb(f8.to_axis(128)), 0);

    // 8-bit endpoints map through axis to the 16-bit endpoints.
    assert_eq!(to_xinput_thumb(f8.to_axis(0)), -32768); // -1.0 * 32768
    assert_eq!(to_xinput_thumb(f8.to_axis(255)), 32767); // 1.0 * 32767

    // Single round-trip through the 16-bit format preserves the unit value at every 8-bit code.
    let f16 = RawStickFormat::XINPUT_16;
    for raw8 in 0..=255i32 {
        let a = f8.to_axis(raw8);
        let raw16 = f16.from_axis(a);
        let back = f16.to_axis(raw16);
        // axis -> i16 -> axis is within one 16-bit quantum.
        assert!(
            (a - back).abs() <= 1.0 / 32767.0,
            "raw8={raw8} a={a} back={back}"
        );
    }
}

#[test]
fn xinput_thumb_is_asymmetric_and_rounds_once() {
    // Asymmetric scale like the C# AxisScale: +full uses 32767, -full uses 32768.
    assert_eq!(to_xinput_thumb(1.0), 32767);
    assert_eq!(to_xinput_thumb(-1.0), -32768);
    assert_eq!(to_xinput_thumb(0.0), 0);
    // Clamps out-of-range without panicking.
    assert_eq!(to_xinput_thumb(2.0), 32767);
    assert_eq!(to_xinput_thumb(-2.0), -32768);
    // Rounds to nearest exactly once: 0.5/32767 ~ a tiny positive value rounds to 1? No:
    // 0.5/32767 * 32767 = 0.5 -> rounds to 1 (round half away from zero in f64::round).
    assert_eq!(to_xinput_thumb(0.5 / 32767.0), 1);
}

#[test]
fn xinput_trigger_maps_unit_to_byte() {
    assert_eq!(to_xinput_trigger(0.0), 0);
    assert_eq!(to_xinput_trigger(1.0), 255);
    assert_eq!(to_xinput_trigger(0.5), 128); // 127.5 rounds to 128
                                             // Clamps.
    assert_eq!(to_xinput_trigger(-1.0), 0);
    assert_eq!(to_xinput_trigger(2.0), 255);
}
