//! `XInputSource` — the `XInputGetState` fallback for standard XInput pads.
//!
//! Standard XInput controllers expose **no** raw HID and are firmware-locked to ~125 Hz
//! (~8 ms). This backend polls `XInputGetState`, maps the signed i16 thumbsticks losslessly to
//! the canonical unit, divides the u8 triggers by 255, synthesizes a sequence counter, and uses
//! a QPC-only dt (there is no device timestamp). It exists so *some* pad always works; it is not
//! a latency path (DESIGN §7).

use hyperion_core::input::normalize::{signed16_to_axis, u8_trigger};
use hyperion_core::input::{Buttons, InputSample, SourceMeta};
use windows::Win32::Foundation::{ERROR_DEVICE_NOT_CONNECTED, ERROR_SUCCESS};
use windows::Win32::UI::Input::XboxController::{XInputGetState, XINPUT_STATE};

use crate::win::qpc_now_ns;
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
        let mut state = XINPUT_STATE::default();
        // SAFETY: `user_index` is validated to `0..=3` in `open`; `&mut state` is an owned
        // out-struct. `XInputGetState` returns a raw Win32 error code rather than throwing.
        let rc = unsafe { XInputGetState(self.user_index, &mut state) };
        if rc == ERROR_DEVICE_NOT_CONNECTED.0 {
            return Err(SourceError::Disconnected);
        }
        if rc != ERROR_SUCCESS.0 {
            return Err(SourceError::Io(std::io::Error::from_raw_os_error(
                rc as i32,
            )));
        }
        // `dwPacketNumber` is monotonic and only changes when the state actually changes;
        // an unchanged packet is a benign "no new input" poll.
        if state.dwPacketNumber == self.last_packet {
            return Ok(false);
        }

        // QPC-only dt: XInput exposes no device timestamp, so elapsed time is purely host-side.
        // The priming poll (`prev_qpc_ns == 0`) yields `dt_us == 0.0`.
        let now = qpc_now_ns();
        let dt_us = if self.prev_qpc_ns == 0 {
            0.0
        } else {
            now.saturating_sub(self.prev_qpc_ns) as f64 / 1_000.0
        };
        self.prev_qpc_ns = now;
        self.last_packet = state.dwPacketNumber;
        self.synth_seq = self.synth_seq.wrapping_add(1);

        let pad = &state.Gamepad;
        // XInput thumbsticks are already `+y == up`, matching the canonical frame; map losslessly.
        out.left.x = signed16_to_axis(pad.sThumbLX);
        out.left.y = signed16_to_axis(pad.sThumbLY);
        out.right.x = signed16_to_axis(pad.sThumbRX);
        out.right.y = signed16_to_axis(pad.sThumbRY);
        out.l2 = u8_trigger(pad.bLeftTrigger);
        out.r2 = u8_trigger(pad.bRightTrigger);
        // Carry the XInput digital button mask opaquely in the low 16 bits.
        out.buttons = Buttons(u32::from(pad.wButtons.0));
        out.seq = self.synth_seq;
        out.dropped = 0;
        out.is_duplicate = false;
        out.dt_us = dt_us;
        out.host_qpc_ns = now;
        Ok(true)
    }

    fn device_id(&self) -> DeviceId {
        // XInput exposes no instance path; identify by the synthetic user-index slot.
        DeviceId::new(0, 0, format!("xinput://{}", self.user_index))
    }
}
