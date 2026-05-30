//! `RawHidGenericSource` — a generic raw-HID source driven by a user-supplied stick layout.
//!
//! For a non-XInput high-poll pad whose report descriptor the user supplies (DESIGN §7 open
//! items), this backend reads raw HID reports and maps fields per a [`StickLayout`] of byte
//! offsets / bit widths / centers. Like the DualSense backend it is **I/O only**: it owns an
//! overlapped reader and forwards raw bytes to a core normalizer. It is unused until a real
//! descriptor exists, so the field-decode body is a `TODO(hardware)` stub.

use hyperion_core::input::{InputSample, SourceMeta};

use crate::win::hid::{OverlappedReader, WaitMode};
use crate::{DeviceId, DeviceSource, SourceError};

/// Byte offsets, bit widths, and centers describing where each axis/trigger lives in a raw HID
/// input report. Supplied by the user from a report-descriptor capture.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StickLayout {
    pub report_len: usize,
    /// `(byte_offset, bits, neutral)` for each of LX, LY, RX, RY.
    pub axes: [FieldSpec; 4],
    /// `(byte_offset, bits)` for each of L2, R2.
    pub triggers: [FieldSpec; 2],
    /// Byte offset of the report-id prefix (or `usize::MAX` if the device sends no id byte).
    pub report_id_offset: usize,
}

/// A single packed field in a raw HID report.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FieldSpec {
    pub byte_offset: usize,
    pub bits: u8,
    pub neutral: i32,
}

/// A generic raw-HID controller decoded via a user-supplied [`StickLayout`].
pub struct RawHidGenericSource {
    id: DeviceId,
    reader: OverlappedReader,
    layout: StickLayout,
    meta: SourceMeta,
}

impl RawHidGenericSource {
    /// Open the device at `device_id.path` and decode reports per `layout`.
    pub fn open(
        device_id: DeviceId,
        layout: StickLayout,
        wait_mode: WaitMode,
        meta: SourceMeta,
    ) -> Result<Self, SourceError> {
        let reader = OverlappedReader::open(&device_id.path, wait_mode)?;
        Ok(Self {
            id: device_id,
            reader,
            layout,
            meta,
        })
    }

    /// The layout this source decodes against.
    #[inline]
    pub fn layout(&self) -> &StickLayout {
        &self.layout
    }
}

impl DeviceSource for RawHidGenericSource {
    fn meta(&self) -> SourceMeta {
        self.meta
    }

    fn next_sample(&mut self, out: &mut InputSample) -> Result<bool, SourceError> {
        // TODO(hardware): on a completed raw report, extract each `FieldSpec` per `self.layout`
        // and hand the values to `hyperion_core::input::normalize` to fill `out`. Returns
        // `Ok(false)` on a benign timeout. No layout is validated against real hardware in M1.
        match self.reader.read_completed()? {
            Some(buf) => {
                let _ = (buf, &self.layout, &mut *out);
                Ok(false)
            }
            None => Ok(false),
        }
    }

    fn device_id(&self) -> DeviceId {
        self.id.clone()
    }
}
