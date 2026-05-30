//! Windows HID I/O primitives: device enumeration and the overlapped-read engine.
//!
//! These modules are the only place allowed to touch Win32. In M1 the bodies are bring-up
//! stubs (no `windows` crate is linked yet — see DESIGN §12 M1), but the types and signatures
//! are final: a [`hid::OverlappedReader`] that owns the double buffer and the single
//! outstanding read, and [`enumerate`] helpers that match VID/PID against the HID class.

pub mod enumerate;
pub mod hid;
