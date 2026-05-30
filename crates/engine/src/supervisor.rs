//! Lifecycle supervisor (Windows only): timer resolution, HidHide, ViGEm target, and hot
//! thread spawn/join (`DESIGN.md` §6 SUPERVISOR / §8).
//!
//! # Skeleton status (M1)
//! The **ordering and ownership are real** — the timer-resolution guard is taken *before* the
//! hot thread spawns and dropped *after* it joins; the policy guard is bound on the hot thread
//! for its whole life (so MMCSS/affinity is not reverted one line later). The actual HID
//! enumeration/open, HidHide IOCTLs, and ViGEm plug/wait/update are filled in during hardware
//! bring-up — they live in the `platform-win`, `hid-input`, and `vgamepad-output` crates.

use std::sync::atomic::AtomicU64;
use std::sync::Arc;
use std::thread::JoinHandle;

use hyperion_core::config::EngineConfig;

use crate::config_store::ConfigStore;
use crate::handoff::{self, CommandTx, ConfigHandle, TelemetryRx};
use crate::hot::StopReason;

/// A supervisor / lifecycle error.
#[derive(Debug)]
pub enum EngineError {
    /// Failed to acquire timer resolution / scheduling policy.
    Platform(String),
    /// Failed to open the physical device.
    DeviceOpen(String),
    /// Failed to create / plug the virtual ViGEm target.
    VirtualPad(String),
    /// The hot thread panicked.
    HotPanic,
}

impl std::fmt::Display for EngineError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EngineError::Platform(m) => write!(f, "platform init failed: {m}"),
            EngineError::DeviceOpen(m) => write!(f, "device open failed: {m}"),
            EngineError::VirtualPad(m) => write!(f, "virtual pad init failed: {m}"),
            EngineError::HotPanic => write!(f, "hot thread panicked"),
        }
    }
}

impl std::error::Error for EngineError {}

/// Owns the engine lifecycle: builds the lock-free links, holds the control-plane ends, and
/// runs the hot thread under the correct resource ordering.
pub struct Supervisor {
    config_store: ConfigStore,
    config: ConfigHandle,
    config_gen: Arc<AtomicU64>,
    /// GUI command producer (handed to the egui thread in M2; held here in the headless slice
    /// so a future GUI can take it).
    gui_tx: Option<CommandTx>,
    /// Supervisor's own command producer (e.g. to push `Shutdown` to the hot loop).
    sup_tx: CommandTx,
    /// Telemetry reader (handed to the GUI in M2; held here for now).
    telemetry_rx: Option<TelemetryRx>,
    /// Built once and moved onto the hot thread at spawn.
    hot_links: Option<HotLinks>,
}

/// The half of the handoff links that the hot thread consumes.
struct HotLinks {
    commands: handoff::CommandRx,
    telemetry: handoff::TelemetryTx,
}

impl Supervisor {
    /// Build the supervisor from the default config (M1 loads defaults; TOML load lands with
    /// the config-store file path in M2).
    pub fn new() -> Result<Self, EngineError> {
        Self::with_config(EngineConfig::default())
    }

    /// Build the supervisor from an explicit config snapshot.
    pub fn with_config(cfg: EngineConfig) -> Result<Self, EngineError> {
        let (config, (telemetry_tx, telemetry_rx), (gui_tx, sup_tx, command_rx)) =
            handoff::build_links(cfg);

        // The store and the hot loop share the same ArcSwap + generation counter.
        let config_store = ConfigStore::from_handle(config.clone());
        let config_gen = config_store.generation_counter();

        Ok(Self {
            config_store,
            config,
            config_gen,
            gui_tx: Some(gui_tx),
            sup_tx,
            telemetry_rx: Some(telemetry_rx),
            hot_links: Some(HotLinks {
                commands: command_rx,
                telemetry: telemetry_tx,
            }),
        })
    }

    /// The single-writer config store (GUI/file-watch route edits here).
    pub fn config_store(&self) -> &ConfigStore {
        &self.config_store
    }

    /// Take the GUI command producer (M2 hands this to the egui thread).
    pub fn take_gui_tx(&mut self) -> Option<CommandTx> {
        self.gui_tx.take()
    }

    /// Take the telemetry reader (M2 hands this to the egui thread).
    pub fn take_telemetry_rx(&mut self) -> Option<TelemetryRx> {
        self.telemetry_rx.take()
    }

    /// Request the hot loop to shut down (used by the headless slice / tray exit).
    pub fn request_shutdown(&mut self) {
        let _ = self.sup_tx.send(crate::handoff::HotCommand::Shutdown);
    }

    /// Assemble platform resources in the correct order, spawn the hot thread, and block until
    /// it exits.
    ///
    /// Ordering (§6, verifier (e)/(c)):
    /// 1. Acquire the timer-resolution guard (`NtSetTimerResolution`, original captured).
    /// 2. Open the physical device through HidHide; create + plug the ViGEm target.
    /// 3. Spawn the hot thread; **inside it** bind the MMCSS/affinity policy guard for the
    ///    thread's whole life (`let _policy = ...; hot.run();`).
    /// 4. `join` the hot thread, then drop the timer-resolution guard.
    pub fn run(mut self) -> Result<(), EngineError> {
        // (1) Timer resolution — held until after join.
        let _timer_res = acquire_timer_resolution()?;

        // (2) Device + virtual target. The real open/plug live in the platform crates; until
        // bring-up wires them, the headless slice has nothing to drive and returns cleanly.
        let hot_links = match self.hot_links.take() {
            Some(links) => links,
            None => return Ok(()),
        };

        // (3) Spawn the hot thread with the policy guard bound inside it.
        let handle = spawn_hot(self.config.clone(), self.config_gen.clone(), hot_links)?;

        // (4) Join, then the timer-resolution guard drops here.
        match handle.join() {
            Ok(StopReason::Shutdown) | Ok(StopReason::DeviceLost) => Ok(()),
            Err(_) => Err(EngineError::HotPanic),
        }
    }
}

/// RAII timer-resolution guard. Real impl lives in `platform-win` (`NtSetTimerResolution`
/// with the original captured via `NtQueryTimerResolution`, restored on `Drop`). M1 skeleton
/// is a no-op placeholder so the ordering is exercised.
struct TimerResGuard;

impl Drop for TimerResGuard {
    fn drop(&mut self) {
        // platform-win restores the original timer resolution here.
    }
}

/// Acquire the 0.5 ms timer resolution before any hot work. Skeleton: never fails.
fn acquire_timer_resolution() -> Result<TimerResGuard, EngineError> {
    Ok(TimerResGuard)
}

/// Spawn the hot thread. The real body constructs the `DualSenseUsbSource` + `Vigem360Pad`,
/// binds the MMCSS/affinity policy guard, and runs [`crate::hot::HotThread::run`]. M1
/// skeleton: spawns a thread that immediately reports `Shutdown` so `run()`'s join path is
/// exercised without real hardware.
fn spawn_hot(
    config: ConfigHandle,
    config_gen: Arc<AtomicU64>,
    links: HotLinks,
) -> Result<JoinHandle<StopReason>, EngineError> {
    let handle = std::thread::Builder::new()
        .name("hyperion-hot".to_string())
        .spawn(move || {
            // Bring-up wires the real hot thread here. The captured resources live on this
            // thread for its whole life:
            //   let _policy = platform_win::sched::apply_hot_thread_policy(...);  // bound for life
            //   let device = hid_input::DualSenseUsbSource::open(...)?;
            //   let target = vgamepad_output::Vigem360Pad::plugged(...)?;
            //   HotThread::new(device, target, config, config_gen,
            //                  links.commands, links.telemetry).run()
            // Destructure so the field moves are exercised exactly as bring-up will consume
            // them (HotThread::run takes `commands` + `telemetry`).
            let HotLinks {
                commands,
                telemetry,
            } = links;
            drop((config, config_gen, commands, telemetry));
            StopReason::Shutdown
        })
        .map_err(|e| EngineError::Platform(e.to_string()))?;
    Ok(handle)
}
