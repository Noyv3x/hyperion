//! `DualSenseUsbSource` — the M1 primary input path (DualSense / DualSense Edge over USB).
//!
//! This backend is **I/O only**. It owns an overlapped HID reader and, per report, forwards the
//! raw 64-byte DS4-compatible report (0x01) plus a host QPC timestamp into the pure
//! `hyperion_core::input` decoder via the [`parse_into`] seam. All semantics — stick
//! de-quantization, the DS sensor-timestamp `SensorClock` (u16 wrap, 16/3 µs/tick), and the
//! byte-7 sequence-counter drop/dupe accounting — live in core (DESIGN §7).
//!
//! The overlapped `ReadFile` body is an M1 bring-up stub (`TODO(hardware)`), but [`parse_into`]
//! is the **real, testable I/O→core seam**: a Windows smoke test injects a synthetic report
//! buffer and asserts the core parser populated the [`InputSample`], with no driver involved.

use hyperion_core::input::ds_report::{ds_report_to_sticks, parse_ds_usb_report};
use hyperion_core::input::dt_clock::SensorClock;
use hyperion_core::input::normalize::u8_trigger;
use hyperion_core::input::seq::SeqTracker;
use hyperion_core::input::{Buttons, InputSample, SourceMeta};

use crate::win::enumerate::DeviceFilter;
use crate::win::hid::{OverlappedReader, WaitMode};
use crate::{DeviceId, DeviceSource, SourceError};

/// A DualSense (or DualSense Edge) USB device exposed as a [`DeviceSource`].
pub struct DualSenseUsbSource {
    id: DeviceId,
    reader: OverlappedReader,
    clock: SensorClock,
    seq: SeqTracker,
    meta: SourceMeta,
}

impl DualSenseUsbSource {
    /// Open the device at `device_id.path` with the given wait strategy.
    ///
    /// `meta` describes the bit depth / expected rate / layout the parser produces and is
    /// surfaced to the engine via [`DeviceSource::meta`].
    pub fn open(
        device_id: DeviceId,
        wait_mode: WaitMode,
        meta: SourceMeta,
    ) -> Result<Self, SourceError> {
        let reader = OverlappedReader::open(&device_id.path, wait_mode)?;
        Ok(Self {
            id: device_id,
            reader,
            clock: SensorClock::default(),
            seq: SeqTracker::default(),
            meta,
        })
    }

    /// Enumerate DualSense / DualSense Edge USB devices currently present.
    pub fn enumerate() -> Vec<DeviceId> {
        crate::win::enumerate::enumerate(DeviceFilter::DUALSENSE_ANY)
    }
}

impl DeviceSource for DualSenseUsbSource {
    fn meta(&self) -> SourceMeta {
        self.meta
    }

    fn next_sample(&mut self, out: &mut InputSample) -> Result<bool, SourceError> {
        // Single outstanding overlapped read, double-buffered (DESIGN §6). The just-completed
        // buffer is parsed through the core seam; a benign timeout yields `Ok(false)`.
        //
        // TODO(hardware): supply the real host QPC timestamp captured at completion
        // (`QueryPerformanceCounter` → ns). Stubbed to 0 until the overlapped read lands.
        let qpc_ns: u64 = 0;
        match self.reader.read_completed()? {
            Some(buf) => Ok(parse_into(buf, out, &mut self.clock, &mut self.seq, qpc_ns)),
            None => Ok(false),
        }
    }

    fn device_id(&self) -> DeviceId {
        self.id.clone()
    }
}

/// The real I/O→core seam: decode one DS4-compatible USB report into `out`.
///
/// This is deliberately a free function (not a method) so it can be unit/smoke-tested with an
/// **injected** buffer, bypassing the stubbed overlapped `ReadFile` entirely. It performs no
/// I/O: it forwards the raw bytes, the dt-folding [`SensorClock`], the [`SeqTracker`], and the
/// host QPC timestamp straight to `hyperion_core::input::ds_report::parse_ds_usb_report`, which
/// owns every numeric decision (offsets, u16/`16/3 µs` dt fold, byte-7 seq drop/dupe).
///
/// Returns `true` if a fresh report was decoded into `out`, `false` if `buf` is not a valid
/// report 0x01 (wrong id/length).
#[inline]
pub fn parse_into(
    buf: &[u8],
    out: &mut InputSample,
    clock: &mut SensorClock,
    seq: &mut SeqTracker,
    qpc_ns: u64,
) -> bool {
    let Some(report) = parse_ds_usb_report(buf) else {
        return false;
    };
    let (left, right) = ds_report_to_sticks(&report);
    let dt_us = clock.fold(report.sensor_ts, qpc_ns);
    let (dropped, is_duplicate) = seq.update(report.counter);

    out.left = left;
    out.right = right;
    out.l2 = u8_trigger(report.l2);
    out.r2 = u8_trigger(report.r2);
    // Carry the three raw DS button bytes opaquely (btn0..btn2 -> bits 0..23).
    out.buttons = Buttons(
        u32::from(report.btn0) | (u32::from(report.btn1) << 8) | (u32::from(report.btn2) << 16),
    );
    out.seq = report.counter;
    out.dropped = dropped;
    out.is_duplicate = is_duplicate;
    out.dt_us = dt_us;
    out.host_qpc_ns = qpc_ns;
    true
}
