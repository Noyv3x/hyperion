//! Windows HID I/O primitives: device enumeration and the overlapped-read engine.
//!
//! These modules are the only place allowed to touch Win32. A [`hid::OverlappedReader`]
//! owns the double buffer and the single outstanding read; [`enumerate`] walks the HID
//! interface class and matches VID/PID against a [`enumerate::DeviceFilter`].

pub mod enumerate;
pub mod hid;

use windows::Win32::System::Performance::{QueryPerformanceCounter, QueryPerformanceFrequency};

/// Read the host high-resolution timer and convert it to nanoseconds.
///
/// `QueryPerformanceCounter` cannot fail on any OS that runs the runtime (it is documented
/// as always succeeding on Windows XP+), but the binding is fallible; a failure or a zero
/// frequency degrades to `0`, which the core [`hyperion_core::input::SensorClock`] treats as
/// "no host timestamp" and falls back to the device sensor clock.
///
/// The conversion does the `* 1_000_000_000 / freq` in `u128` so the multiply never
/// overflows for the lifetime of a session.
// HW-verify: the QPC→ns scaling factor (and that the frequency is stable across cores) is
// only fully exercised on real hardware; the arithmetic is platform-independent.
#[inline]
#[must_use]
pub fn qpc_now_ns() -> u64 {
    let mut counter: i64 = 0;
    let mut freq: i64 = 0;
    // SAFETY: both calls take a single out-pointer to a stack `i64` we own and keep alive
    // for the duration of the call. The bindings return `Result<()>`; on error we leave the
    // locals at their `0` initial value and fall through to the `freq <= 0` guard below.
    unsafe {
        if QueryPerformanceCounter(&mut counter).is_err() {
            return 0;
        }
        if QueryPerformanceFrequency(&mut freq).is_err() {
            return 0;
        }
    }
    if freq <= 0 || counter < 0 {
        return 0;
    }
    let ns = (counter as u128 * 1_000_000_000u128) / freq as u128;
    ns as u64
}
