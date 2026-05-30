//! Concrete [`crate::DeviceSource`] backends, one per transport.
//!
//! All three implement the same trait and do **I/O only** — every byte they read is handed to
//! `hyperion_core::input` for decoding (DESIGN §7):
//!
//! * [`dualsense_usb`] — the M1 primary path: DualSense / DualSense Edge over USB, DS4-compatible
//!   report 0x01, parsed through the core `parse_into` seam.
//! * [`xinput`] — the `XInputGetState` fallback (~125 Hz, QPC-only dt) for standard XInput pads.
//! * [`raw_hid`] — a generic raw-HID source driven by a user-supplied stick layout, gated on the
//!   user providing a report descriptor for a non-XInput high-poll pad.

pub mod dualsense_usb;
pub mod raw_hid;
pub mod xinput;
