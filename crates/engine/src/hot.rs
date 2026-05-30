//! The hot loop (Windows only). Owns one HID device, runs the `core` RC filter in place, and
//! submits to the virtual pad inline — zero-alloc, lock-free steady state (`DESIGN.md` §6).
//!
//! # Wiring (M1)
//! The **control flow and the types are real**; the actual blocking HID read and the ViGEm
//! IOCTL are driven through the [`DeviceSource`] / [`VirtualPad`] traits below. On Windows the
//! [`crate::win_io`] adapters implement these traits over the `hid-input` `DualSenseUsbSource`
//! and `vgamepad-output` `Vigem360Pad` backends; MMCSS / affinity policy binding and HidHide
//! live in the supervisor. The Win32 bodies inside the backends are validated on hardware.
//!
//! The engine is written **generically over two traits** expressed entirely in terms of
//! [`hyperion_core`] value types ([`HotInput`], [`OutputFrame`]). That keeps the hot loop
//! type-checked on `windows-latest` independently of the sibling crates' concrete struct
//! names, and confines the device-report decode to the backend (where the real
//! double-buffered overlapped read lives).
//!
//! Loop order (per §6, zero-alloc steady state):
//! drain gui+sup command queues → blocking read → `cfg.load()` + cheap generation check →
//! (prime or) `filter.process` per stick → map to [`OutputFrame`] → `target.update()` inline
//! → triple-buffer telemetry write.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use hyperion_core::config::{EngineConfig, StickMode};
use hyperion_core::dt::Dt;
use hyperion_core::output::OutputFrame;
use hyperion_core::rc::{RcConfig, RcFilter, RcStickState};
use hyperion_core::stick::{StickAlgorithm, StickSample};

use crate::clock::DtTracker;
use crate::handoff::{CommandRx, ConfigHandle, HotCommand, TelemetryTx};
use crate::telemetry::{LatencyReservoir, TelemetryFrame};

/// One decoded input report, in the canonical units the filter consumes. The `hid-input`
/// backend decodes the raw HID buffer (via `core`'s parser) into this; the engine never
/// touches device bytes. Expressed over plain values so the engine stays decoupled from the
/// backend's concrete types.
#[derive(Clone, Copy, Debug)]
pub struct HotInput {
    /// Left / right stick, canonical `[-1,1]`.
    pub left: StickSample,
    pub right: StickSample,
    /// Triggers, `[0,1]`.
    pub lt: f64,
    pub rt: f64,
    /// Mapped Xbox-360 button bitfield.
    pub buttons: u16,
    /// Guarded real elapsed time since the previous report (microseconds).
    pub dt_us: f64,
    /// `true` for the first report after enable/reset (filter primes, no step).
    pub is_prime: bool,
    /// Reports dropped before this one (seq gap), and whether this report duplicates the last.
    pub dropped: u16,
    pub is_duplicate: bool,
    /// Host monotonic timestamp at read completion (ns), for scope correlation.
    pub host_qpc_ns: u64,
}

impl Default for HotInput {
    /// A neutral, primed report (sticks centered, triggers released). `StickSample` itself has
    /// no `Default`, so this is implemented by hand from [`StickSample::NEUTRAL`].
    fn default() -> Self {
        Self {
            left: StickSample::NEUTRAL,
            right: StickSample::NEUTRAL,
            lt: 0.0,
            rt: 0.0,
            buttons: 0,
            dt_us: 0.0,
            is_prime: true,
            dropped: 0,
            is_duplicate: false,
            host_qpc_ns: 0,
        }
    }
}

/// A blocking source of decoded input reports. Implemented by the `hid-input` backends
/// (`DualSenseUsbSource`, …) on Windows.
pub trait DeviceSource: Send {
    /// Error type for a read.
    type Error;

    /// Block until the next report is available. `Ok(Some(_))` = a fresh decoded report,
    /// `Ok(None)` = a benign timeout (retry), `Err` = device loss.
    fn next(&mut self) -> Result<Option<HotInput>, Self::Error>;
}

/// A virtual gamepad target. Implemented by `vgamepad-output`'s `Vigem360Pad` on Windows.
pub trait VirtualPad {
    /// Error type for a submit.
    type Error;

    /// Submit one frame via a single synchronous, bounded IOCTL (must never block the
    /// TIME_CRITICAL hot thread in the driver — §6).
    fn update(&mut self, frame: &OutputFrame) -> Result<(), Self::Error>;
}

/// Why the hot loop stopped.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StopReason {
    /// A [`HotCommand::Shutdown`] was received.
    Shutdown,
    /// The device disconnected (read or submit returned `Err`).
    DeviceLost,
}

/// Everything the hot thread needs, assembled by [`crate::run`] and moved onto the thread.
pub struct HotThread<D, V> {
    device: D,
    target: V,
    config: ConfigHandle,
    /// Shared generation counter; bumped by [`crate::config_store::ConfigStore`].
    config_gen: Arc<AtomicU64>,
    commands: CommandRx,
    telemetry: TelemetryTx,
}

impl<D, V> HotThread<D, V>
where
    D: DeviceSource,
    V: VirtualPad,
{
    /// Bundle the hot-thread inputs. Construction is off the hot path.
    pub fn new(
        device: D,
        target: V,
        config: ConfigHandle,
        config_gen: Arc<AtomicU64>,
        commands: CommandRx,
        telemetry: TelemetryTx,
    ) -> Self {
        Self {
            device,
            target,
            config,
            config_gen,
            commands,
            telemetry,
        }
    }

    /// Run the hot loop until shutdown or device loss. Blocks the calling (hot) thread.
    ///
    /// The MMCSS / affinity / priority RAII guard must be **bound** for the whole call by the
    /// caller (`let _policy = ...; thread.run();`) — see §6 (verifier (c)): a bare statement
    /// drops the guard at the semicolon and reverts the policy one line later.
    pub fn run(mut self) -> StopReason {
        // Resident, alloc-free working set (held across reports).
        let filter = RcFilter;
        let mut rc_state: [RcStickState; 2] = Default::default();
        let mut primed = [false; 2];
        let mut clock = DtTracker::new();
        let mut busy = LatencyReservoir::new();
        let mut applied_gen: u64 = 0;
        let mut cfg: EngineConfig = (*self.config.load_full()).clone();
        let mut dropped_total: u32 = 0;
        let mut duplicates_total: u32 = 0;

        loop {
            // Drain BOTH command queues first so a Shutdown is honored promptly.
            while let Some(cmd) = self.commands.try_pop() {
                match cmd {
                    HotCommand::ResetFilter
                    | HotCommand::Recalibrate
                    | HotCommand::ReplugTarget => {
                        filter.reset(&mut rc_state[0]);
                        filter.reset(&mut rc_state[1]);
                        primed = [false; 2];
                        clock.reset();
                    }
                    HotCommand::Shutdown => return StopReason::Shutdown,
                }
            }

            // Blocking read of the next decoded report. (The backend performs the real
            // double-buffered overlapped read: re-arm the *other* buffer, then parse the
            // just-completed one — that lives in `hid-input`.)
            let busy_start = clock.now_qpc_ns();
            let input = match self.device.next() {
                Ok(Some(i)) => i,
                Ok(None) => continue, // benign timeout
                Err(_) => return StopReason::DeviceLost,
            };

            // Refresh config only when the generation changed (one atomic load per report).
            let cur_gen = self.config_gen.load(Ordering::Acquire);
            if cur_gen != applied_gen {
                cfg = (*self.config.load_full()).clone();
                applied_gen = cur_gen;
            }

            let dt = Dt::guarded(input.dt_us);
            dropped_total = dropped_total.wrapping_add(input.dropped as u32);
            if input.is_duplicate {
                duplicates_total = duplicates_total.wrapping_add(1);
            }

            // Run the filter per stick (X,Y share a param, independent state). A prime report
            // seeds history and passes input through with no IIR step; a disabled / non-Rc
            // stick is a pass-through handled inside the filter (and short-circuited here).
            let (lcfg, rcfg) = resolve_rc(&cfg);
            let out_l = step_stick(
                &filter,
                lcfg.as_ref(),
                &mut rc_state[0],
                &mut primed[0],
                dt,
                input.left,
                input.is_prime,
            );
            let out_r = step_stick(
                &filter,
                rcfg.as_ref(),
                &mut rc_state[1],
                &mut primed[1],
                dt,
                input.right,
                input.is_prime,
            );

            // Map to the virtual-pad frame (the single i16 round happens in the output backend
            // via `core::output::to_xinput_*`).
            let frame = OutputFrame {
                lx: out_l.x,
                ly: out_l.y,
                rx: out_r.x,
                ry: out_r.y,
                lt: input.lt,
                rt: input.rt,
                buttons: input.buttons,
            };

            // Submit inline (one bounded synchronous IOCTL).
            if self.target.update(&frame).is_err() {
                return StopReason::DeviceLost;
            }

            // Publish telemetry (never blocks; Copy frame).
            let busy_ns = clock.now_qpc_ns().saturating_sub(busy_start);
            busy.record_ns(busy_ns);
            let tf = TelemetryFrame {
                loop_busy_ns: busy_ns,
                dt_us: dt.us() as f32,
                dropped: dropped_total,
                duplicates: duplicates_total,
                in_lx: input.left.x as f32,
                in_ly: input.left.y as f32,
                in_rx: input.right.x as f32,
                in_ry: input.right.y as f32,
                out_lx: out_l.x as f32,
                out_ly: out_l.y as f32,
                out_rx: out_r.x as f32,
                out_ry: out_r.y as f32,
            };
            self.telemetry.0.write(tf);
            // `busy.p99_us()` / `input.host_qpc_ns` feed the M2 scope + p99 telemetry surface.
        }
    }
}

/// Step one stick: prime on the first post-reset report, else take one filter step. With no
/// resolved RC config (`cfg == None`) the stick passes through unfiltered.
#[inline]
fn step_stick(
    filter: &RcFilter,
    cfg: Option<&RcConfig>,
    state: &mut RcStickState,
    primed: &mut bool,
    dt: Dt,
    s: StickSample,
    is_prime: bool,
) -> StickSample {
    let Some(cfg) = cfg else {
        return s;
    };
    if !*primed || is_prime {
        filter.prime(cfg, state, s);
        *primed = true;
        s
    } else {
        filter.process(cfg, state, dt, s)
    }
}

/// Resolve the `(left, right)` per-stick [`RcConfig`] for the active device from the engine
/// config. A stick whose mode is not [`StickMode::Rc`] (or whose device is absent) resolves to
/// `None` and passes through unfiltered. The returned `RcConfig` is used as-is; the filter
/// itself short-circuits when `RcConfig::enabled` is `false`.
#[inline]
fn resolve_rc(cfg: &EngineConfig) -> (Option<RcConfig>, Option<RcConfig>) {
    match cfg.devices.get(&cfg.active_device) {
        Some(dev) => (
            (dev.ls.mode == StickMode::Rc).then_some(dev.ls.rc),
            (dev.rs.mode == StickMode::Rc).then_some(dev.rs.rc),
        ),
        None => (None, None),
    }
}
