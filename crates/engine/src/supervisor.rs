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

// `EngineError` is shared with the cross-platform `Runtime`, so it lives in `crate::error`;
// re-export it here so `engine::supervisor::EngineError` stays a valid path.
pub use crate::error::EngineError;

/// Owns the engine lifecycle: builds the lock-free links, holds the control-plane ends, and
/// runs the hot thread under the correct resource ordering.
pub struct Supervisor {
    /// The single-writer config store, sharing the hot loop's `ArcSwap` + generation counter.
    /// Taken by [`crate::runtime::Runtime`] to hand to its control-writer thread; held as an
    /// `Option` so ownership can move out without tearing down the supervisor.
    config_store: Option<ConfigStore>,
    config: ConfigHandle,
    config_gen: Arc<AtomicU64>,
    /// GUI command producer (handed to the egui thread in M2; held here in the headless slice
    /// so a future GUI can take it).
    gui_tx: Option<CommandTx>,
    /// Supervisor's own command producer (e.g. to push `Shutdown` to the hot loop). `None`
    /// after [`Supervisor::take_sup_tx`] hands it to the runtime.
    sup_tx: Option<CommandTx>,
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
            config_store: Some(config_store),
            config,
            config_gen,
            gui_tx: Some(gui_tx),
            sup_tx: Some(sup_tx),
            telemetry_rx: Some(telemetry_rx),
            hot_links: Some(HotLinks {
                commands: command_rx,
                telemetry: telemetry_tx,
            }),
        })
    }

    /// The single-writer config store (GUI/file-watch route edits here), while still owned by
    /// the supervisor. `None` after [`Supervisor::take_config_store`].
    pub fn config_store(&self) -> Option<&ConfigStore> {
        self.config_store.as_ref()
    }

    /// Take the single-writer [`ConfigStore`] (sharing the hot loop's `ArcSwap` + generation
    /// counter) so [`crate::runtime::Runtime`] can move it onto its dedicated control-writer
    /// thread. The supervisor still holds the [`ConfigHandle`] + generation counter the hot
    /// thread needs, so the spawn is unaffected.
    pub fn take_config_store(&mut self) -> Option<ConfigStore> {
        self.config_store.take()
    }

    /// Take the GUI command producer (M2 hands this to the egui thread).
    pub fn take_gui_tx(&mut self) -> Option<CommandTx> {
        self.gui_tx.take()
    }

    /// Take the telemetry reader (M2 hands this to the egui thread).
    pub fn take_telemetry_rx(&mut self) -> Option<TelemetryRx> {
        self.telemetry_rx.take()
    }

    /// Take the supervisor's own command producer (the `Shutdown`-pushing end). The
    /// [`crate::runtime::Runtime`] holds this so it can stop the hot loop after [`spawn`] has
    /// consumed the supervisor. Mutually exclusive with [`Supervisor::request_shutdown`].
    pub fn take_sup_tx(&mut self) -> Option<CommandTx> {
        self.sup_tx.take()
    }

    /// Request the hot loop to shut down (used by the headless slice / tray exit). No-op once
    /// the command producer has been taken via [`Supervisor::take_sup_tx`].
    pub fn request_shutdown(&mut self) {
        if let Some(tx) = self.sup_tx.as_mut() {
            let _ = tx.send(crate::handoff::HotCommand::Shutdown);
        }
    }

    /// Assemble platform resources in the correct order and **spawn** the hot thread, returning
    /// a [`RunningSupervisor`] handle immediately (NON-blocking). The caller drives lifetime via
    /// [`RunningSupervisor::join`]; the timer-resolution guard lives inside the handle so it is
    /// dropped strictly *after* the hot thread joins (§6 verifier (e)).
    ///
    /// This is the building block [`crate::runtime::Runtime`] uses to keep the main thread free
    /// for the egui loop; the blocking [`Supervisor::run`] is a thin `spawn().join()` wrapper
    /// for the headless slice.
    ///
    /// Ordering (§6, verifier (e)/(c)):
    /// 1. Acquire the timer-resolution guard (`NtSetTimerResolution`, original captured).
    /// 2. Open the physical device through HidHide; create + plug the ViGEm target.
    /// 3. Spawn the hot thread; **inside it** bind the MMCSS/affinity policy guard for the
    ///    thread's whole life (`let _policy = ...; hot.run();`).
    /// 4. (caller) `join` the hot thread, then drop the timer-resolution guard.
    pub fn spawn(mut self) -> Result<RunningSupervisor, EngineError> {
        // Snapshot the lifecycle parameters once (timer resolution, thread policy, HidHide).
        // The hot loop reads live config through the ArcSwap; these host-policy knobs are taken
        // at spawn and held for the run.
        let cfg = (*self.config.load_full()).clone();

        // (1) Timer resolution — raised before any hot work, restored after join. Real
        // `NtSetTimerResolution` lives in `platform-win`; the guard is moved into the returned
        // handle so its `Drop` runs strictly after `join()` (DESIGN §6 verifier (e)).
        let timer_res = platform_win::begin_timer_resolution(cfg.thread.timer_resolution_us);

        // (2) Device + virtual target are opened *on the hot thread* (the HID handle, the ViGEm
        // client, and the MMCSS/affinity policy are all thread-affine — they must live on the
        // dedicated hot thread, not be created here and moved). If no device is present the hot
        // thread returns `DeviceLost` cleanly, so the headless slice exits without error.
        let hot_links = self.hot_links.take();

        // (3) Spawn the hot thread with the policy guard bound inside it (or `None` if there is
        // nothing to drive, which still joins cleanly).
        let handle = match hot_links {
            Some(links) => Some(spawn_hot(
                cfg,
                self.config.clone(),
                self.config_gen.clone(),
                links,
            )?),
            None => None,
        };

        Ok(RunningSupervisor {
            handle,
            _timer_res: timer_res,
        })
    }

    /// Assemble platform resources, spawn the hot thread, and block until it exits.
    ///
    /// Thin wrapper over [`Supervisor::spawn`] + [`RunningSupervisor::join`] for the headless
    /// slice; the GUI runtime uses `spawn` directly so the main thread stays free for egui.
    pub fn run(self) -> Result<(), EngineError> {
        self.spawn()?.join()
    }
}

/// A spawned, running hot thread plus the timer-resolution guard that must outlive it.
///
/// Holds the [`JoinHandle`] and the [`platform_win::TimerResGuard`]; dropping or
/// [`join`](RunningSupervisor::join)ing this restores the timer resolution strictly *after* the
/// hot thread is gone (§6 verifier (e)). Shutdown is requested through the command queue (the
/// supervisor / runtime owns the producer), not through this handle.
pub struct RunningSupervisor {
    /// `None` when there was nothing to drive (no hot links); `join` is then trivially `Ok`.
    handle: Option<JoinHandle<StopReason>>,
    /// Restored on drop, after `handle` has joined.
    _timer_res: platform_win::TimerResGuard,
}

impl RunningSupervisor {
    /// Block until the hot thread exits, then drop the timer-resolution guard.
    ///
    /// `StopReason::Shutdown` / `DeviceLost` are both clean exits; a panicked hot thread maps to
    /// [`EngineError::HotPanic`].
    pub fn join(self) -> Result<(), EngineError> {
        match self.handle {
            Some(handle) => match handle.join() {
                Ok(StopReason::Shutdown) | Ok(StopReason::DeviceLost) => Ok(()),
                Err(_) => Err(EngineError::HotPanic),
            },
            None => Ok(()),
        }
        // `_timer_res` drops here, after the join.
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
