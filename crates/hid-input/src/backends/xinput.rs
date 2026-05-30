//! `XInputSource` — the `XInputGetState` fallback for standard XInput pads.
//!
//! Standard XInput controllers expose **no** raw HID and are firmware-locked to ~125 Hz
//! (~8 ms). This backend polls `XInputGetState`, maps the signed i16 thumbsticks losslessly to
//! the canonical unit, divides the u8 triggers by 255, synthesizes a sequence counter, and uses
//! a QPC-only dt (there is no device timestamp). It exists so *some* pad always works; it is not
//! a latency path (DESIGN §7).

use hyperion_core::input::{InputSample, SourceMeta};

use crate::{DeviceId, DeviceSource, SourceError};

/// An XInput user index in `0..=3`.
pub type XInputUserIndex = u32;

/// A standard XInput controller polled via `XInputGetState`.
pub struct XInputSource {
    user_index: XInputUserIndex,
    /// Monotonic synthesized sequence counter (XInput has no report counter).
    synth_seq: u8,
    /// Host QPC of the previous accepted poll, for QPC-only dt.
    prev_qpc_ns: u64,
    /// `dwPacketNumber` of the last state, to detect "no change" polls.
    last_packet: u32,
    meta: SourceMeta,
}

impl XInputSource {
    /// Bind to the given XInput user index (`0..=3`).
    pub fn open(user_index: XInputUserIndex, meta: SourceMeta) -> Result<Self, SourceError> {
        if user_index > 3 {
            return Err(SourceError::Disconnected);
        }
        Ok(Self {
            user_index,
            synth_seq: 0,
            prev_qpc_ns: 0,
            last_packet: 0,
            meta,
        })
    }
}

impl DeviceSource for XInputSource {
    fn meta(&self) -> SourceMeta {
        self.meta
    }

    fn next_sample(&mut self, out: &mut InputSample) -> Result<bool, SourceError> {
        // TODO(hardware): `XInputGetState(user_index, &mut state)`; on `ERROR_DEVICE_NOT_CONNECTED`
        // return `Err(SourceError::Disconnected)`. If `state.dwPacketNumber == last_packet`,
        // nothing changed → `Ok(false)`. Otherwise advance `synth_seq`, compute QPC-only dt from
        // `prev_qpc_ns`, map i16 thumbs (lossless) + u8 triggers (/255) into `out`, store the
        // packet number, and return `Ok(true)`.
        let _ = (
            self.user_index,
            &mut self.synth_seq,
            &mut self.prev_qpc_ns,
            &mut self.last_packet,
            &mut *out,
        );
        Ok(false)
    }

    fn device_id(&self) -> DeviceId {
        // XInput exposes no instance path; identify by the synthetic user-index slot.
        DeviceId::new(0, 0, format!("xinput://{}", self.user_index))
    }
}
