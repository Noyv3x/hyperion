//! HID device enumeration: `SetupDiGetClassDevs` + `HidD_GetAttributes` VID/PID match.
//!
//! DESIGN §7: walk the HID interface class, open each candidate, read `HidD_GetAttributes`
//! (VID/PID) and `HidD_GetPreparsedData`/caps (`InputReportByteLength == 64`), and collect the
//! instance paths that match a [`DeviceFilter`]. The returned [`crate::DeviceId`] `path` is the
//! device-interface detail string HidHide also consumes.

use windows::core::PCWSTR;
use windows::Win32::Devices::DeviceAndDriverInstallation::{
    SetupDiDestroyDeviceInfoList, SetupDiEnumDeviceInterfaces, SetupDiGetClassDevsW,
    SetupDiGetDeviceInterfaceDetailW, DIGCF_DEVICEINTERFACE, DIGCF_PRESENT, HDEVINFO,
    SETUP_DI_GET_CLASS_DEVS_FLAGS, SP_DEVICE_INTERFACE_DATA, SP_DEVICE_INTERFACE_DETAIL_DATA_W,
};
use windows::Win32::Devices::HumanInterfaceDevice::{
    HidD_FreePreparsedData, HidD_GetAttributes, HidD_GetPreparsedData, HidP_GetCaps,
    GUID_DEVINTERFACE_HID, HIDD_ATTRIBUTES, HIDP_CAPS, HIDP_STATUS_SUCCESS, PHIDP_PREPARSED_DATA,
};
use windows::Win32::Foundation::{CloseHandle, GENERIC_READ, GENERIC_WRITE, HANDLE};
use windows::Win32::Storage::FileSystem::{
    CreateFileW, FILE_FLAGS_AND_ATTRIBUTES, FILE_SHARE_READ, FILE_SHARE_WRITE, OPEN_EXISTING,
};

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

/// Enumerate all HID devices matching `filter`, in `SetupDi` interface order.
///
/// Walks `GUID_DEVINTERFACE_HID` with `DIGCF_PRESENT | DIGCF_DEVICEINTERFACE`, resolves each
/// interface to its device path, opens it transiently to read VID/PID + the input-report
/// length, and collects the matches as [`DeviceId`]s. Devices that cannot be opened (e.g. a
/// mouse/keyboard claimed exclusively, or a pad already blacklisted) are skipped, not errored:
/// enumeration is best-effort and never fails the caller.
pub fn enumerate(filter: DeviceFilter) -> Vec<DeviceId> {
    let mut out = Vec::new();

    // SAFETY: `GUID_DEVINTERFACE_HID` is a valid 'static GUID pointer. We pass no enumerator
    // and no parent window; the returned set is owned by us and freed via
    // `SetupDiDestroyDeviceInfoList` on every exit path below.
    let dev_info: HDEVINFO = match unsafe {
        SetupDiGetClassDevsW(
            Some(&GUID_DEVINTERFACE_HID),
            PCWSTR::null(),
            None,
            SETUP_DI_GET_CLASS_DEVS_FLAGS(DIGCF_PRESENT.0 | DIGCF_DEVICEINTERFACE.0),
        )
    } {
        Ok(h) => h,
        Err(_) => return out,
    };

    let mut index: u32 = 0;
    loop {
        let mut iface = SP_DEVICE_INTERFACE_DATA {
            cbSize: size_of::<SP_DEVICE_INTERFACE_DATA>() as u32,
            ..Default::default()
        };

        // SAFETY: `dev_info` is a live device-info set; `iface` has its `cbSize` initialized as
        // the API requires. An error here (typically `ERROR_NO_MORE_ITEMS`) ends the walk.
        if unsafe {
            SetupDiEnumDeviceInterfaces(dev_info, None, &GUID_DEVINTERFACE_HID, index, &mut iface)
        }
        .is_err()
        {
            break;
        }
        index += 1;

        if let Some(path) = interface_path(dev_info, &iface) {
            if let Some(attrs) = open_and_query(&path) {
                if filter.matches(&attrs) {
                    out.push(DeviceId::new(attrs.vid, attrs.pid, path));
                }
            }
        }
    }

    // SAFETY: `dev_info` came from `SetupDiGetClassDevsW` and has not been freed; this is the
    // single matching destroy. The handle is not used afterward.
    let _ = unsafe { SetupDiDestroyDeviceInfoList(dev_info) };
    out
}

/// Resolve one device-interface entry to its `\\?\HID#...` instance path.
///
/// `SetupDiGetDeviceInterfaceDetailW` is the classic two-call pattern: the first call reports
/// the required byte size (it returns `ERROR_INSUFFICIENT_BUFFER`), the second fills a buffer
/// whose leading `SP_DEVICE_INTERFACE_DETAIL_DATA_W` has `cbSize` set to the *fixed* header
/// size (`8` on x64 — `size_of` of the binding struct), never the total buffer size.
fn interface_path(dev_info: HDEVINFO, iface: &SP_DEVICE_INTERFACE_DATA) -> Option<String> {
    let mut required: u32 = 0;
    // SAFETY: first probe call — null detail buffer with zero size so the API only writes the
    // required size through `required`. The expected error is `ERROR_INSUFFICIENT_BUFFER`.
    let _ = unsafe {
        SetupDiGetDeviceInterfaceDetailW(dev_info, iface, None, 0, Some(&mut required), None)
    };
    let header = size_of::<SP_DEVICE_INTERFACE_DETAIL_DATA_W>() as u32;
    if required < header {
        return None;
    }

    // Back the variable-length detail struct with a `u32`-aligned scratch buffer: the header's
    // leading `cbSize: u32` requires 4-byte alignment, which a `Vec<u32>` guarantees (a
    // `Vec<u16>` would not, making the `cbSize` write potentially-misaligned UB). Size it to
    // `required` bytes rounded up to whole `u32`s.
    let u32_len = (required as usize).div_ceil(4);
    let mut scratch: Vec<u32> = vec![0u32; u32_len];
    let byte_capacity = u32_len * 4;

    // The detail header lives at the front of the buffer; set its `cbSize` to the *fixed* header
    // size the API expects (8 on x64), never `required`.
    // SAFETY: `scratch` is `u32`-aligned and holds `>= header` bytes (guarded above), so the
    // pointer is valid and aligned for `SP_DEVICE_INTERFACE_DETAIL_DATA_W`. We only write the
    // leading `cbSize` field here; the kernel fills the rest.
    let detail = scratch.as_mut_ptr() as *mut SP_DEVICE_INTERFACE_DETAIL_DATA_W;
    unsafe {
        (*detail).cbSize = header;
    }

    // SAFETY: `detail` points at a `byte_capacity >= required` byte buffer with its `cbSize`
    // initialized; `iface` is the live entry just enumerated. On success the device path is
    // written as a NUL-terminated wide string starting at the `DevicePath` field.
    if unsafe {
        SetupDiGetDeviceInterfaceDetailW(
            dev_info,
            iface,
            Some(detail),
            byte_capacity as u32,
            None,
            None,
        )
    }
    .is_err()
    {
        return None;
    }

    // `DevicePath` begins 4 bytes into the struct (just past `cbSize`). Reinterpret the scratch
    // as bytes and read the NUL-terminated wide string from offset 4.
    // SAFETY: `scratch` owns `byte_capacity` initialized bytes; viewing them as `u8` is sound and
    // the slice borrow keeps `scratch` alive for the read.
    let bytes: &[u8] =
        unsafe { core::slice::from_raw_parts(scratch.as_ptr() as *const u8, byte_capacity) };
    let path_bytes = &bytes[4..];
    let words: Vec<u16> = path_bytes
        .chunks_exact(2)
        .map(|c| u16::from_ne_bytes([c[0], c[1]]))
        .take_while(|&w| w != 0)
        .collect();
    Some(String::from_utf16_lossy(&words))
}

/// Open `path` transiently and read its VID/PID/version + input-report length.
///
/// Returns `None` if the device cannot be opened or queried (skipped during enumeration).
fn open_and_query(path: &str) -> Option<HidAttributes> {
    let handle = open_device(path)?;
    let attrs = query_attributes(handle.0 as isize);
    // SAFETY: `handle` is a valid handle from `CreateFileW` that we own and have not closed.
    let _ = unsafe { CloseHandle(handle) };
    attrs
}

/// `CreateFileW(path, GENERIC_READ|GENERIC_WRITE, share-all, OPEN_EXISTING)` with no flags.
///
/// Used by enumeration for a transient query handle; the hot read path opens its own
/// overlapped handle in [`crate::win::hid`].
fn open_device(path: &str) -> Option<HANDLE> {
    let wide = to_wide(path);
    // SAFETY: `wide` is a NUL-terminated wide string kept alive across the call. We request
    // shared read/write so we never block other openers (notably the real overlapped reader).
    // `htemplatefile` is `None`; the returned handle is checked via `Result`.
    let handle = unsafe {
        CreateFileW(
            PCWSTR::from_raw(wide.as_ptr()),
            GENERIC_READ.0 | GENERIC_WRITE.0,
            FILE_SHARE_READ | FILE_SHARE_WRITE,
            None,
            OPEN_EXISTING,
            FILE_FLAGS_AND_ATTRIBUTES(0),
            None,
        )
    }
    .ok()?;
    Some(handle)
}

/// Read VID/PID/version + input-report length from an already-open device handle.
///
/// `handle` is a raw `HANDLE` value (as `isize`) so this can be called both from enumeration
/// and from a backend that already holds a typed handle. Returns `None` if `HidD_GetAttributes`
/// or the caps query fails.
pub fn query_attributes(handle: isize) -> Option<HidAttributes> {
    let hdev = HANDLE(handle as *mut core::ffi::c_void);

    let mut raw = HIDD_ATTRIBUTES {
        Size: size_of::<HIDD_ATTRIBUTES>() as u32,
        ..Default::default()
    };
    // SAFETY: `hdev` is a live HID handle; `raw.Size` is initialized as the API requires and
    // `&mut raw` points at a struct we own for the duration of the call.
    if !unsafe { HidD_GetAttributes(hdev, &mut raw) } {
        return None;
    }

    let mut preparsed = PHIDP_PREPARSED_DATA::default();
    // SAFETY: `hdev` is live; `&mut preparsed` receives an owned preparsed-data handle that we
    // free with `HidD_FreePreparsedData` on every path below.
    if !unsafe { HidD_GetPreparsedData(hdev, &mut preparsed) } {
        return None;
    }

    let mut caps = HIDP_CAPS::default();
    // SAFETY: `preparsed` is the handle just produced by `HidD_GetPreparsedData`; `&mut caps`
    // is an owned out-struct. The status is checked against `HIDP_STATUS_SUCCESS`.
    let status = unsafe { HidP_GetCaps(preparsed, &mut caps) };
    // SAFETY: `preparsed` is the live handle from above and is freed exactly once here.
    let _ = unsafe { HidD_FreePreparsedData(preparsed) };
    if status != HIDP_STATUS_SUCCESS {
        return None;
    }

    Some(HidAttributes {
        vid: raw.VendorID,
        pid: raw.ProductID,
        version: raw.VersionNumber,
        input_report_len: caps.InputReportByteLength as usize,
    })
}

/// Convert a `&str` into a NUL-terminated UTF-16 buffer for the `*W` Win32 APIs.
pub(crate) fn to_wide(s: &str) -> Vec<u16> {
    s.encode_utf16().chain(std::iter::once(0)).collect()
}
