//! Windows I/O adapters that bridge the sibling backend crates to the engine's hot-loop
//! traits ([`crate::hot::DeviceSource`] / [`crate::hot::VirtualPad`]).
//!
//! The hot loop is written generically over two engine-owned traits expressed in
//! [`hyperion_core`] value types ([`HotInput`] / [`OutputFrame`]). The concrete backends speak
//! their own vocabulary:
//!
//! * `hid-input`'s [`hid_input::DeviceSource`] fills an [`InputSample`] in place
//!   (`next_sample(&mut InputSample) -> Result<bool, _>`),
//! * `vgamepad-output`'s [`vgamepad_output::VirtualPad`] is a lifecycle trait
//!   (`plugin` / `wait_ready` / `update` / `unplug`).
//!
//! These adapters own one concrete backend each and implement the engine trait, doing the
//! per-report `InputSample → HotInput` translation (including the DS→Xbox-360 button map, the
//! one mapping core does not own because core carries the raw DS button bytes opaquely) and the
//! lifecycle wiring (HidHide cloak, ViGEm plug/wait). Everything here is `cfg(windows)` — the
//! whole module is only compiled in via the Windows-gated `supervisor`/`hot` modules.

use std::time::Duration;

use hyperion_core::config::{EngineConfig, WaitMode as CfgWaitMode};
use hyperion_core::input::{InputSample, SourceMeta};
use hyperion_core::output::OutputFrame;
use hyperion_core::stick::StickSample;

use hid_input::win::hid::WaitMode as HidWaitMode;
use hid_input::{DeviceSource as HidDeviceSource, DualSenseUsbSource, SourceError};
use platform_win::hidhide::{HidHide, HidHideBackend};
use platform_win::sched::{HotThreadConfig, WaitMode as SchedWaitMode};
use vgamepad_output::{OutErr, Vigem360Pad, VirtualPad as VgVirtualPad};

use crate::hot::{DeviceSource, HotInput, VirtualPad};

/// How long to wait for the OS to enumerate the freshly-plugged virtual pad before giving up.
const VIGEM_READY_TIMEOUT: Duration = Duration::from_secs(2);

/// Build the [`HotThreadConfig`] for `platform-win` from the engine config's thread section.
///
/// Maps the serde [`CfgWaitMode`] (+ spin budget) onto the scheduler's own [`SchedWaitMode`],
/// and forwards core-pinning / MMCSS knobs verbatim. Pure value copy — safe to call off-thread.
pub(crate) fn hot_thread_config(cfg: &EngineConfig) -> HotThreadConfig {
    let t = &cfg.thread;
    let wait_mode = match t.wait_mode {
        // A zero spin budget collapses HybridSpin to a plain blocking wait.
        CfgWaitMode::HybridSpin if t.spin_budget_us > 0 => SchedWaitMode::HybridSpin,
        _ => SchedWaitMode::Blocking,
    };
    HotThreadConfig {
        hot_core: t.hot_core,
        use_mmcss: t.use_mmcss,
        mmcss_task: t.mmcss_task.clone(),
        wait_mode,
    }
}

/// Translate the engine config's [`CfgWaitMode`] (+ spin budget) into the HID reader's
/// [`HidWaitMode`].
fn hid_wait_mode(cfg: &EngineConfig) -> HidWaitMode {
    let t = &cfg.thread;
    match t.wait_mode {
        CfgWaitMode::HybridSpin if t.spin_budget_us > 0 => HidWaitMode::HybridSpin {
            spin_budget_us: t.spin_budget_us,
        },
        _ => HidWaitMode::Blocking,
    }
}

/// The engine-facing device: a [`DualSenseUsbSource`] plus the resident [`InputSample`] it
/// decodes into, the HidHide cloak held for the device's life, and prime tracking.
///
/// Owns the HidHide guard so the physical pad stays cloaked exactly as long as we hold the
/// device open; dropping the device tears the cloak down (the pad reappears for other apps).
pub(crate) struct DualSenseDevice {
    source: DualSenseUsbSource,
    /// Reused decode target — the backend fills this in place, so steady state never allocates.
    sample: InputSample,
    /// HidHide cloak, held for the device's whole life. `None` when cloaking is disabled in
    /// config. Kept alive (never read) purely for its `Drop` (un-cloak on teardown).
    _cloak: Option<HidHide>,
    /// `true` until the first fresh report is delivered, so the hot filter primes once.
    first_report: bool,
}

impl DualSenseDevice {
    /// Enumerate, open, and cloak the primary DualSense USB device.
    ///
    /// Lifecycle (DESIGN §8): enumerate → open the overlapped reader → (if enabled) bring up
    /// HidHide (whitelist *self* first, then blacklist the physical instance path, then cloak
    /// on). Returns `None` when no device is present or any platform step fails — the supervisor
    /// treats that as a clean headless exit (`StopReason::DeviceLost`), not a panic.
    pub(crate) fn open_cloaked(cfg: &EngineConfig) -> Option<Self> {
        let id = DualSenseUsbSource::enumerate().into_iter().next()?;

        // Cloak the physical pad *before* we start reading it, so no other app grabs it first.
        // Whitelisting ourselves is mandatory — otherwise we hide the pad from ourselves too.
        let cloak = if cfg.hidhide.enabled {
            let backend = if cfg.hidhide.use_cli {
                HidHideBackend::Cli {
                    cli_path: cfg.hidhide.cli_path.clone(),
                }
            } else {
                HidHideBackend::Ioctl
            };
            let mut hh = HidHide::open(backend).map_err(log_hidhide_err).ok()?;
            hh.whitelist_self().map_err(log_hidhide_err).ok()?;
            hh.blacklist_device(&id.path)
                .map_err(log_hidhide_err)
                .ok()?;
            hh.set_active(true).map_err(log_hidhide_err).ok()?;
            Some(hh)
        } else {
            None
        };

        let meta = SourceMeta {
            vid: id.vid,
            pid: id.pid,
            name: "DualSense (USB)",
            stick_bits: 8,
        };
        let source = DualSenseUsbSource::open(id, hid_wait_mode(cfg), meta)
            .map_err(log_source_err)
            .ok()?;

        Some(Self {
            source,
            sample: InputSample::default(),
            _cloak: cloak,
            first_report: true,
        })
    }
}

/// Surface a HidHide setup error to stderr (cold path only).
fn log_hidhide_err(e: std::io::Error) {
    eprintln!("hyperion: HidHide setup failed: {e}");
}

/// Surface a HID open error to stderr (cold path only).
fn log_source_err(e: SourceError) {
    eprintln!("hyperion: HID device open failed: {e}");
}

impl DeviceSource for DualSenseDevice {
    type Error = SourceError;

    fn next(&mut self) -> Result<Option<HotInput>, Self::Error> {
        // The backend re-arms the *other* overlapped buffer and parses the just-completed one,
        // filling `self.sample` in place. `Ok(false)` is a benign timeout (no fresh report).
        if !self.source.next_sample(&mut self.sample)? {
            return Ok(None);
        }
        let is_prime = self.first_report;
        self.first_report = false;
        Ok(Some(hot_input_from_sample(&self.sample, is_prime)))
    }
}

/// Convert a decoded [`InputSample`] into the engine's [`HotInput`].
///
/// Sticks/triggers are already canonical (`[-1,1]` / `[0,1]`) in core units; this only re-homes
/// them into the engine value type and maps the opaque DS button bytes into the Xbox-360
/// bitfield. `dt_us` / `dropped` / `is_duplicate` / `host_qpc_ns` are carried through verbatim
/// (the backend already folded dt via the `SensorClock` and derived drop/dupe via `SeqTracker`).
#[inline]
fn hot_input_from_sample(s: &InputSample, is_prime: bool) -> HotInput {
    HotInput {
        left: StickSample::new(s.left.x, s.left.y),
        right: StickSample::new(s.right.x, s.right.y),
        lt: s.l2,
        rt: s.r2,
        buttons: ds_buttons_to_xinput(s.buttons.0),
        dt_us: s.dt_us,
        is_prime,
        dropped: s.dropped,
        is_duplicate: s.is_duplicate,
        host_qpc_ns: s.host_qpc_ns,
    }
}

// XInput (`XINPUT_GAMEPAD_*`) button bits.
const XI_DPAD_UP: u16 = 0x0001;
const XI_DPAD_DOWN: u16 = 0x0002;
const XI_DPAD_LEFT: u16 = 0x0004;
const XI_DPAD_RIGHT: u16 = 0x0008;
const XI_START: u16 = 0x0010;
const XI_BACK: u16 = 0x0020;
const XI_LTHUMB: u16 = 0x0040;
const XI_RTHUMB: u16 = 0x0080;
const XI_LSHOULDER: u16 = 0x0100;
const XI_RSHOULDER: u16 = 0x0200;
const XI_GUIDE: u16 = 0x0400;
const XI_A: u16 = 0x1000;
const XI_B: u16 = 0x2000;
const XI_X: u16 = 0x4000;
const XI_Y: u16 = 0x8000;

/// Map the three raw DualSense button bytes (packed `btn0|btn1<<8|btn2<<16` by the HID
/// backend) into the XInput button bitfield consumed by the virtual Xbox-360 pad.
///
/// Layout follows the DS4Windows / `DS4Device.cs` ground truth:
/// * `btn0` (byte 5): low nibble = D-pad hat (`0..7`, `8` = released); high nibble = face
///   buttons Square `0x10` / Cross `0x20` / Circle `0x40` / Triangle `0x80`.
/// * `btn1` (byte 6): L1 `0x01`, R1 `0x02`, L2-click `0x04`, R2-click `0x08`, Share `0x10`,
///   Options `0x20`, L3 `0x40`, R3 `0x80`.
/// * `btn2` (byte 7): PS `0x01`, Touchpad-click `0x02` (upper 6 bits are the frame counter,
///   already consumed by the core seq tracker).
///
/// Face-button mapping uses the standard cross-layout (Cross→A, Circle→B, Square→X, Triangle→Y).
// HW-verify: the DS button bit positions and the hat-to-DPAD decode are HW-verify in core's
// `ds_report` byte map; this mapping mirrors that ground truth but is validated on hardware.
#[inline]
fn ds_buttons_to_xinput(raw: u32) -> u16 {
    let btn0 = (raw & 0xFF) as u8;
    let btn1 = ((raw >> 8) & 0xFF) as u8;
    let btn2 = ((raw >> 16) & 0xFF) as u8;

    let mut out: u16 = 0;

    // D-pad hat (low nibble of btn0): 0=N,1=NE,2=E,3=SE,4=S,5=SW,6=W,7=NW, 8=released.
    let hat = btn0 & 0x0F;
    let (up, right, down, left) = match hat {
        0 => (true, false, false, false),
        1 => (true, true, false, false),
        2 => (false, true, false, false),
        3 => (false, true, true, false),
        4 => (false, false, true, false),
        5 => (false, false, true, true),
        6 => (false, false, false, true),
        7 => (true, false, false, true),
        _ => (false, false, false, false),
    };
    if up {
        out |= XI_DPAD_UP;
    }
    if down {
        out |= XI_DPAD_DOWN;
    }
    if left {
        out |= XI_DPAD_LEFT;
    }
    if right {
        out |= XI_DPAD_RIGHT;
    }

    // Face buttons (high nibble of btn0), cross layout.
    if btn0 & 0x10 != 0 {
        out |= XI_X; // Square
    }
    if btn0 & 0x20 != 0 {
        out |= XI_A; // Cross
    }
    if btn0 & 0x40 != 0 {
        out |= XI_B; // Circle
    }
    if btn0 & 0x80 != 0 {
        out |= XI_Y; // Triangle
    }

    // Shoulders / stick clicks / meta (btn1).
    if btn1 & 0x01 != 0 {
        out |= XI_LSHOULDER; // L1
    }
    if btn1 & 0x02 != 0 {
        out |= XI_RSHOULDER; // R1
    }
    if btn1 & 0x10 != 0 {
        out |= XI_BACK; // Share -> Back/View
    }
    if btn1 & 0x20 != 0 {
        out |= XI_START; // Options -> Start/Menu
    }
    if btn1 & 0x40 != 0 {
        out |= XI_LTHUMB; // L3
    }
    if btn1 & 0x80 != 0 {
        out |= XI_RTHUMB; // R3
    }

    // PS button -> Guide (btn2).
    if btn2 & 0x01 != 0 {
        out |= XI_GUIDE;
    }

    out
}

/// The engine-facing virtual pad: a plugged-in [`Vigem360Pad`].
///
/// Construction plugs the target into ViGEmBus and waits for OS enumeration, so by the time the
/// hot loop holds one, `update` is a single bounded IOCTL. `Drop` (via `Vigem360Pad`) unplugs.
pub(crate) struct Vigem360Target {
    pad: Vigem360Pad,
}

impl Vigem360Target {
    /// Create the Xbox-360 target, plug it into ViGEmBus, and wait for the OS to enumerate it.
    ///
    /// Returns `None` if the ViGEmBus driver is unavailable or enumeration times out — the
    /// supervisor maps that to a clean headless exit rather than a panic.
    pub(crate) fn plugged() -> Option<Self> {
        let mut pad = Vigem360Pad::new();
        pad.plugin().map_err(log_out_err).ok()?;
        pad.wait_ready(VIGEM_READY_TIMEOUT)
            .map_err(log_out_err)
            .ok()?;
        Some(Self { pad })
    }
}

impl VirtualPad for Vigem360Target {
    type Error = OutErr;

    #[inline]
    fn update(&mut self, frame: &OutputFrame) -> Result<(), Self::Error> {
        self.pad.update(frame)
    }
}

/// Surface an [`OutErr`] to stderr (cold setup path only — plug / wait_ready, never the hot
/// loop). Returned from `map_err` so the caller can `.ok()?` the result.
fn log_out_err(e: OutErr) {
    eprintln!("hyperion: virtual pad setup failed: {e}");
}
