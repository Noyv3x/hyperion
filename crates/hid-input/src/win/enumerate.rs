//! HID device enumeration: `SetupDiGetClassDevs` + `HidD_GetAttributes` VID/PID match.
//!
//! DESIGN §7: walk the HID interface class, open each candidate, read `HidD_GetAttributes`
//! (VID/PID) and `HidD_GetPreparsedData`/caps (`InputReportByteLength == 64`), and collect the
//! instance paths that match a [`DeviceFilter`]. The returned [`crate::DeviceId`] `path` is the
//! device-interface detail string HidHide also consumes.

use crate::DeviceId;

/// DualSense USB vendor id (Sony Interactive Entertainment).
pub const VID_SONY: u16 = 0x054C;
/// DualSense (CFI-ZCT1) product id.
pub const PID_DUALSENSE: u16 = 0x0CE6;
/// DualSense Edge (CFI-ZCT1) product id.
pub const PID_DUALSENSE_EDGE: u16 = 0x0DF2;

/// A VID/PID predicate plus the required input-report length, used to select devices.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DeviceFilter {
    pub vid: u16,
    /// Product id to match, or `None` to accept any product under `vid`.
    pub pid: Option<u16>,
    /// Required `HIDP_CAPS::InputReportByteLength` (DualSense USB report 0x01 is 64).
    pub input_report_len: usize,
}

impl DeviceFilter {
    /// Filter matching either DualSense or DualSense Edge over USB (64-byte report).
    pub const DUALSENSE_ANY: Self = Self {
        vid: VID_SONY,
        pid: None,
        input_report_len: 64,
    };

    /// Does `attrs` satisfy this filter?
    #[inline]
    pub fn matches(&self, attrs: &HidAttributes) -> bool {
        attrs.vid == self.vid
            && self.pid.map(|p| p == attrs.pid).unwrap_or(true)
            && attrs.input_report_len == self.input_report_len
    }
}

/// The subset of `HIDD_ATTRIBUTES` + caps we match on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HidAttributes {
    pub vid: u16,
    pub pid: u16,
    pub version: u16,
    pub input_report_len: usize,
}

/// Enumerate all HID devices matching `filter`, newest interface first.
///
/// `TODO(hardware)`: `SetupDiGetClassDevs(&GUID_DEVINTERFACE_HID, .., DIGCF_PRESENT |
/// DIGCF_DEVICEINTERFACE)`, iterate `SetupDiEnumDeviceInterfaces`,
/// `SetupDiGetDeviceInterfaceDetailW` for the path, `CreateFileW` + `HidD_GetAttributes` +
/// `HidD_GetPreparsedData`/`HidP_GetCaps`, push matches. Returns empty until bring-up.
pub fn enumerate(filter: DeviceFilter) -> Vec<DeviceId> {
    let _ = filter;
    Vec::new()
}

/// Read VID/PID/version + report length from an already-open device handle.
///
/// `TODO(hardware)`: `HidD_GetAttributes` + caps off the preparsed data for `handle`.
pub fn query_attributes(_handle: isize) -> Option<HidAttributes> {
    None
}
