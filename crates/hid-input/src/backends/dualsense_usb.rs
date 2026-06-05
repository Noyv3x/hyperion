//! `DualSenseUsbSource` — the M1 primary input path (DualSense / DualSense Edge over USB).
//!
//! This backend is **I/O only**. It owns an overlapped HID reader and, per report, forwards the
//! raw 64-byte DS4-compatible report (0x01) plus a host QPC timestamp into the pure
//! `hyperion_core::input` decoder via the [`parse_into`] seam. All semantics — stick
//! de-quantization, the DS sensor-timestamp `SensorClock` (u16 wrap, 16/3 µs/tick), and the
//! byte-7 sequence-counter drop/dupe accounting — live in core (DESIGN §7).
//!
//! The overlapped `ReadFile` body is real (single outstanding double-buffered read via
//! [`OverlappedReader`]); [`parse_into`] is the **real, testable I/O→core seam**: a unit test
//! injects a synthetic report buffer and asserts the core parser populated the [`InputSample`] (and
//! the paired touch/Edge state), with no driver involved.

use hyperion_core::input::ds_report::{
    decode_controller_state, ds_report_to_sticks, parse_ds_usb_report,
};
use hyperion_core::input::dt_clock::SensorClock;
use hyperion_core::input::normalize::u8_trigger;
use hyperion_core::input::seq::SeqTracker;
use hyperion_core::input::{Buttons, InputSample, SourceMeta};

use crate::win::enumerate::DeviceFilter;
use crate::win::hid::{OverlappedReader, WaitMode, HID_REPORT_LEN};
use crate::{DeviceId, DeviceSource, EdgeButtons, SourceError, TouchEdge};

/// A DualSense (or DualSense Edge) USB device exposed as a [`DeviceSource`].
pub struct DualSenseUsbSource {
    id: DeviceId,
    reader: OverlappedReader,
    clock: SensorClock,
    seq: SeqTracker,
    meta: SourceMeta,
    /// Touch contacts + Edge button bits decoded from the most recent report, surfaced to the
    /// engine via [`DeviceSource::touch_edge`] (the stick-only `InputSample` cannot carry them).
    /// `Default` (untouched pad / all Edge bits `false`) until the first report decodes.
    touch_edge: TouchEdge,
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
            touch_edge: TouchEdge::default(),
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
        // buffer is parsed through the core seam; a benign timeout yields `Ok(false)`. The host
        // QPC timestamp is the one captured by the reader at read completion.
        //
        // The completed report is copied into a small stack buffer so the immutable borrow of
        // `self.reader` ends before `parse_into` takes `&mut self.clock`/`&mut self.seq` — the
        // copy is a fixed 64 bytes, off the hot path's allocation budget.
        let (report, qpc_ns) = match self.reader.read_completed()? {
            Some(buf) => {
                let mut report = [0u8; HID_REPORT_LEN];
                let n = buf.len().min(HID_REPORT_LEN);
                report[..n].copy_from_slice(&buf[..n]);
                (report, self.reader.last_qpc_ns())
            }
            None => return Ok(false),
        };
        let ok = parse_into(&report, out, &mut self.clock, &mut self.seq, qpc_ns);
        // On a fresh decode, also surface the touchpad contacts + Edge bits (which the stick-only
        // `InputSample` cannot carry) so the engine can copy them into its `HotInput`. A rejected
        // buffer (wrong id / short) leaves the last good touch/Edge state untouched, matching how
        // `parse_into` leaves `out` unchanged.
        if ok {
            self.touch_edge = touch_edge_from(&report, &self.meta);
        }
        Ok(ok)
    }

    fn touch_edge(&self) -> TouchEdge {
        self.touch_edge
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

/// Decode the touchpad contacts + DualSense Edge button bits a [`DeviceSource`] surfaces alongside
/// the stick-only [`InputSample`] (M7 touch/Edge wiring).
///
/// A free function (like [`parse_into`]) so it is unit-testable with an injected buffer and no
/// device. It runs the SAME pure core decode the engine's `ControllerState` build uses
/// ([`decode_controller_state`]), so the contacts + Edge superset are identical to what the mapping
/// engine would see — no parallel offset table. `meta.is_edge` gates the Edge bits (a non-Edge
/// source reads them all `false`); a buffer too short for the touch tail decodes two inactive
/// contacts. Returns the inert [`TouchEdge::default`] for a buffer that is not a valid report 0x01.
#[inline]
pub fn touch_edge_from(buf: &[u8], meta: &SourceMeta) -> TouchEdge {
    let Some(report) = parse_ds_usb_report(buf) else {
        return TouchEdge::default();
    };
    let state = decode_controller_state(&report, buf, meta);
    TouchEdge {
        touch: state.touch,
        edge: EdgeButtons {
            mute: state.mute,
            capture: state.capture,
            fn_l: state.fn_l,
            fn_r: state.fn_r,
            blp: state.blp,
            brp: state.brp,
            side_l: state.side_l,
            side_r: state.side_r,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hyperion_core::input::ds_report::{DS_USB_REPORT_ID, DS_USB_REPORT_LEN};

    /// Build a minimal valid DS USB report 0x01 with the given stick/trigger/counter/timestamp.
    fn synth_report(lx: u8, ry: u8, l2: u8, counter6: u8, ts: u16) -> [u8; HID_REPORT_LEN] {
        let mut b = [0u8; HID_REPORT_LEN];
        b[0] = DS_USB_REPORT_ID;
        b[1] = lx; // LX
        b[2] = 0x80; // LY centered
        b[3] = 0x80; // RX centered
        b[4] = ry; // RY
        b[8] = l2; // L2 analog
        b[7] = counter6 << 2; // byte-7 high 6 bits are the frame counter
        b[10] = (ts & 0xFF) as u8; // sensor ts low
        b[11] = (ts >> 8) as u8; // sensor ts high
        b
    }

    #[test]
    fn parse_into_decodes_synthetic_report_through_core() {
        // Drives the I/O->core seam with an injected buffer (no device): asserts the core
        // parser populated the sample, exercising offsets, the dt fold, and seq accounting.
        assert_eq!(DS_USB_REPORT_LEN, HID_REPORT_LEN);

        let mut clock = SensorClock::default();
        let mut seq = SeqTracker::default();
        let mut out = InputSample::default();

        // Priming report: dt is 0.0 on the first fold, seq starts the tracker.
        let first = synth_report(0x00, 0x00, 0xFF, 5, 100);
        let ok = parse_into(&first, &mut out, &mut clock, &mut seq, 1_000);
        assert!(ok, "valid report 0x01 must decode");
        assert_eq!(out.seq, 5);
        assert_eq!(out.host_qpc_ns, 1_000);
        assert_eq!(out.dt_us, 0.0, "first fold primes the clock");
        // LX raw 0x00 maps to the canonical left extreme (-1.0); RY raw 0x00 -> +1.0 (up).
        assert!(out.left.x < -0.99);
        assert!(out.right.y > 0.99);
        assert!(out.r2 < 0.01 && out.l2 > 0.99, "triggers map through /255");

        // Second report one counter later: a real positive dt and no drop.
        let second = synth_report(0x80, 0x80, 0x00, 6, 200);
        let ok = parse_into(&second, &mut out, &mut clock, &mut seq, 2_000);
        assert!(ok);
        assert_eq!(out.seq, 6);
        assert_eq!(out.dropped, 0);
        assert!(!out.is_duplicate);
        assert!(out.dt_us > 0.0, "second fold yields a real elapsed dt");
    }

    #[test]
    fn parse_into_rejects_wrong_report_id() {
        let mut clock = SensorClock::default();
        let mut seq = SeqTracker::default();
        let mut out = InputSample::default();

        let mut buf = synth_report(0x40, 0x40, 0x10, 1, 50);
        buf[0] = 0x31; // not the DS4-compatible report 0x01
        assert!(!parse_into(&buf, &mut out, &mut clock, &mut seq, 0));

        let short = [DS_USB_REPORT_ID; 10]; // shorter than DS_USB_REPORT_LEN
        assert!(!parse_into(&short, &mut out, &mut clock, &mut seq, 0));
    }

    // ------------------------------- M7: touch / Edge surfacing seam -----------------------------

    use hyperion_core::input::ds_report::TOUCH_DATA_OFFSET;

    const DS_META: SourceMeta = SourceMeta {
        vid: 0x054C,
        pid: 0x0CE6,
        name: "ds",
        stick_bits: 8,
        is_edge: false,
    };

    const EDGE_META: SourceMeta = SourceMeta {
        vid: 0x054C,
        pid: 0x0DF2,
        name: "edge",
        stick_bits: 8,
        is_edge: true,
    };

    /// Write an ACTIVE finger contact into a report's touch tail (high bit CLEAR == active, the C#
    /// `IsActive` convention); mirrors the core `ds_report` test fixture.
    fn set_touch(b: &mut [u8; HID_REPORT_LEN], finger: usize, id: u8, x: u16, y: u16) {
        let base = TOUCH_DATA_OFFSET + finger * 4;
        b[base] = id & 0x7F;
        b[base + 1] = (x & 0xFF) as u8;
        b[base + 2] = (((y & 0x0F) << 4) | ((x >> 8) & 0x0F)) as u8;
        b[base + 3] = (y >> 4) as u8;
    }

    #[test]
    fn touch_edge_from_surfaces_active_contact_and_edge_bit() {
        // A synthetic report with one ACTIVE touch contact + (Edge source) the Mute bit set in the
        // extended tail reaches `TouchEdge` through the same core decode the engine reads — no
        // device, pure. Drives the #1 wiring gap end-to-end at the backend seam.
        let mut buf = synth_report(0x80, 0x80, 0, 1, 0);
        // Idle the OTHER finger (high bit set == inactive), then activate finger 0.
        buf[TOUCH_DATA_OFFSET + 4] = 0x80;
        set_touch(&mut buf, 0, 0x2A, 300, 120);
        // Edge extended byte is one past the touch tail (core `EDGE_FN_BYTE`); 0x04 == Mute.
        buf[TOUCH_DATA_OFFSET + 8] = 0x04;

        // Edge-capable source: the contact AND the Mute bit surface.
        let te = touch_edge_from(&buf, &EDGE_META);
        assert!(te.touch[0].is_active, "active finger surfaced");
        assert_eq!(te.touch[0].id, 0x2A);
        assert_eq!((te.touch[0].x, te.touch[0].y), (300, 120));
        assert!(!te.touch[1].is_active, "second finger stays inactive");
        assert!(te.edge.mute, "Edge Mute bit surfaced for an Edge source");
        assert!(!te.edge.fn_l && !te.edge.brp, "only Mute set");

        // Non-Edge source: the touch contact still surfaces, but every Edge bit stays false.
        let te_plain = touch_edge_from(&buf, &DS_META);
        assert!(te_plain.touch[0].is_active && te_plain.touch[0].id == 0x2A);
        assert!(!te_plain.edge.mute, "non-Edge source reads Edge bits false");
        assert_eq!(te_plain.edge, EdgeButtons::default());
    }

    #[test]
    fn touch_edge_from_idle_report_is_inert() {
        // A report whose touch tail is idle (no `set_touch`) and whose Edge byte is zero decodes to
        // the inert default — byte-identical to the pre-M7 `Default` the engine used to plug in.
        let mut buf = synth_report(0x80, 0x80, 0, 1, 0);
        buf[TOUCH_DATA_OFFSET] = 0x80; // both fingers high-bit-set == inactive
        buf[TOUCH_DATA_OFFSET + 4] = 0x80;
        assert_eq!(touch_edge_from(&buf, &EDGE_META), TouchEdge::default());

        // A wrong-id buffer also yields the inert default (never a partial decode).
        let mut bad = buf;
        bad[0] = 0x31;
        assert_eq!(touch_edge_from(&bad, &DS_META), TouchEdge::default());
    }
}
