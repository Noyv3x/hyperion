//! Lifecycle supervisor (Windows only): timer resolution, HidHide, ViGEm target, and hot
//! thread spawn/join (`DESIGN.md` §6 SUPERVISOR / §8).
//!
//! The **ordering and ownership are real and wired**: the timer-resolution guard
//! ([`platform_win::begin_timer_resolution`]) is taken *before* the hot thread spawns and
//! dropped *after* it joins; the MMCSS/affinity policy guard is bound on the hot thread for its
//! whole life (so it is not reverted one line later). The HID open + HidHide cloak and the
//! ViGEm plug/wait happen *on the hot thread* (those handles are thread-affine) via the
//! [`crate::win_io`] adapters, which wrap the `hid-input` / `vgamepad-output` / `platform-win`
//! backends. The Win32 bodies inside those backends are validated on hardware by the maintainer.

use std::sync::atomic::AtomicU64;
use std::sync::Arc;
use std::thread::JoinHandle;

use hyperion_core::config::EngineConfig;

use crate::config_store::ConfigStore;
use crate::handoff::{self, CommandTx, ConfigHandle, TelemetryRx};
use crate::hot::{HotThread, StopReason};
use crate::win_io::{DualSenseDevice, Vigem360Target};

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
        // Snapshot the lifecycle parameters once (timer resolution, thread policy, HidHide).
        // The hot loop reads live config through the ArcSwap; these host-policy knobs are taken
        // at spawn and held for the run.
        let cfg = (*self.config.load_full()).clone();

        // (1) Timer resolution — raised before any hot work, restored after join. Real
        // `NtSetTimerResolution` lives in `platform-win`; the guard is owned here so its `Drop`
        // runs strictly after `handle.join()` below (DESIGN §6 verifier (e)).
        let _timer_res = platform_win::begin_timer_resolution(cfg.thread.timer_resolution_us);

        // (2) Device + virtual target are opened *on the hot thread* (the HID handle, the ViGEm
        // client, and the MMCSS/affinity policy are all thread-affine — they must live on the
        // dedicated hot thread, not be created here and moved). If no device is present the hot
        // thread returns `DeviceLost` cleanly, so the headless slice exits without error.
        let hot_links = match self.hot_links.take() {
            Some(links) => links,
            None => return Ok(()),
        };

        // (3) Spawn the hot thread with the policy guard bound inside it.
        let handle = spawn_hot(cfg, self.config.clone(), self.config_gen.clone(), hot_links)?;

        // (4) Join, then the timer-resolution guard drops here (after the hot thread is gone).
        match handle.join() {
            Ok(StopReason::Shutdown) | Ok(StopReason::DeviceLost) => Ok(()),
            Err(_) => Err(EngineError::HotPanic),
        }
    }
}

/// Spawn the hot thread, doing all thread-affine acquisition *inside* the spawned thread.
///
/// Order on the hot thread (DESIGN §6/§8):
/// 1. bind the MMCSS/affinity/priority policy guard for the thread's whole life
///    (`let _policy = ...;` — a bare statement would revert it one line later, verifier (c));
/// 2. open the physical [`DualSenseDevice`] through HidHide (whitelist self → blacklist the
///    physical pad → cloak on); on device-not-found, return `DeviceLost` (headless, clean exit);
/// 3. create + plug the [`Vigem360Target`] (`plugin` then `wait_ready`);
/// 4. run [`HotThread::run`] to steady state.
fn spawn_hot(
    cfg: EngineConfig,
    config: ConfigHandle,
    config_gen: Arc<AtomicU64>,
    links: HotLinks,
) -> Result<JoinHandle<StopReason>, EngineError> {
    let HotLinks {
        commands,
        telemetry,
    } = links;

    let handle = std::thread::Builder::new()
        .name("hyperion-hot".to_string())
        .spawn(move || {
            // (1) Host-thread policy, bound for the thread's whole life (NOT a bare statement).
            let policy = crate::win_io::hot_thread_config(&cfg);
            let _policy = platform_win::apply_hot_thread_policy(&policy);

            // (2) Open the physical device behind the HidHide cloak. A missing device (or a
            // HidHide/driver error) is a clean headless exit, not a panic.
            let Some(device) = DualSenseDevice::open_cloaked(&cfg) else {
                return StopReason::DeviceLost;
            };

            // (3) Create + plug the virtual Xbox 360 target and wait for OS enumeration.
            let Some(target) = Vigem360Target::plugged() else {
                return StopReason::DeviceLost;
            };

            // (4) Run the steady-state loop. The HidHide cloak (held inside `device`), the
            // ViGEm target, and `_policy` all drop on this thread when `run` returns.
            HotThread::new(device, target, config, config_gen, commands, telemetry).run()
        })
        .map_err(|e| EngineError::Platform(e.to_string()))?;
    Ok(handle)
}
