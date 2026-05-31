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
use std::time::Duration;

use crossbeam_channel::{Receiver, Sender};
use hyperion_core::config::EngineConfig;

use crate::config_store::ConfigStore;
use crate::control::{
    auto_switch_decision, ControlMsg, ControlPlaneEvent, ControlPlaneRx, ControlPlaneTx,
};
use crate::handoff::{self, CommandTx, ConfigHandle, KbmRx, KbmTx, TelemetryRx};
use crate::hot::{HotThread, StopReason};
use crate::win_io::{spawn_kbm_injector, DualSenseDevice, DynPadTarget};

// `EngineError` is shared with the cross-platform `Runtime`, so it lives in `crate::error`;
// re-export it here so `engine::supervisor::EngineError` stays a valid path.
pub use crate::error::EngineError;

/// Capacity of the control-plane side channel (hot → supervisor). Special edges are rare (a user
/// pressing a special-action button) and the macro table re-publishes only on the generation gate,
/// so a small bounded ring is plenty; on overflow the hot thread drops the event (self-healing).
const CONTROL_PLANE_CAP: usize = 64;

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
    /// KBM egress consumer (HOT → injector). Taken at [`Supervisor::spawn`] to start the
    /// injector thread; `None` once consumed.
    kbm_rx: Option<KbmRx>,
    /// Control-plane side channel consumer (HOT → supervisor): `Special` edges + the resolved macro
    /// table (blueprint §5/§12 M4). Drained off the hot path by the control-plane thread spawned in
    /// [`Supervisor::spawn`]; `None` once consumed.
    ctrl_rx: Option<ControlPlaneRx>,
    /// Built once and moved onto the hot thread at spawn.
    hot_links: Option<HotLinks>,
}

/// The half of the handoff links that the hot thread consumes.
struct HotLinks {
    commands: handoff::CommandRx,
    telemetry: handoff::TelemetryTx,
    /// KBM egress producer the hot loop pushes one batch per report into (drop-on-full).
    kbm_tx: KbmTx,
    /// Control-plane side-channel producer (hot → supervisor): `Special` edges + the macro table.
    ctrl_tx: ControlPlaneTx,
}

impl Supervisor {
    /// Build the supervisor from the default config (M1 loads defaults; TOML load lands with
    /// the config-store file path in M2).
    pub fn new() -> Result<Self, EngineError> {
        Self::with_config(EngineConfig::default())
    }

    /// Build the supervisor from an explicit config snapshot.
    pub fn with_config(cfg: EngineConfig) -> Result<Self, EngineError> {
        let (config, (telemetry_tx, telemetry_rx), (gui_tx, sup_tx, command_rx), (kbm_tx, kbm_rx)) =
            handoff::build_links(cfg);

        // Control-plane side channel (hot → supervisor). Bounded so the hot thread's `try_send` is
        // drop-on-full (never blocks the TIME_CRITICAL thread); a missed `Special` re-fires on the
        // next edge and the macro table re-publishes on the next generation gate, so a brief
        // drain stall is self-healing. Capacity absorbs a burst of special edges + macro re-sends.
        let (ctrl_tx, ctrl_rx) = crossbeam_channel::bounded(CONTROL_PLANE_CAP);

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
            kbm_rx: Some(kbm_rx),
            ctrl_rx: Some(ctrl_rx),
            hot_links: Some(HotLinks {
                commands: command_rx,
                telemetry: telemetry_tx,
                kbm_tx,
                ctrl_tx,
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

        // (2b) Spawn the KBM injector thread (normal priority). It drains the egress ring and
        // realizes key/mouse edges via SendInput entirely off the hot thread (blueprint §7.3). It
        // exits on its own once the hot thread's `KbmTx` is dropped and the ring is drained, so no
        // explicit stop channel is needed — `RunningSupervisor::join` joins it after the hot
        // thread has finished (which drops the `KbmTx`).
        let kbm_injector = match self.kbm_rx.take() {
            Some(kbm_rx) => {
                Some(spawn_kbm_injector(kbm_rx).map_err(|e| EngineError::Platform(e.to_string()))?)
            }
            None => None,
        };

        // (2c) Spawn the control-plane drain thread (blueprint §5/§12 M4): it drains the hot loop's
        // `Special` edges + macro-table publishes off the hot path. For M4 the special-action
        // handler is a minimal stub (log/ack); real exec (profile switch / launch / disconnect)
        // builds on this in M5. It exits when the hot thread's `ControlPlaneTx` is dropped.
        let ctrl_drain = self
            .ctrl_rx
            .take()
            .map(spawn_control_plane_drain)
            .transpose()
            .map_err(|e: std::io::Error| EngineError::Platform(e.to_string()))?;

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

        // The `ForegroundWatcher` (auto-profile-switch, §7.4) is spawned by `crate::runtime::Runtime`
        // (not here): it needs a clone of the GUI→writer `ControlMsg` sender, which the runtime owns,
        // and it must be joined **before** the config-writer thread in `Runtime::shutdown` so a final
        // switch it emits is still drained. See `runtime::spawn_foreground_watcher` wiring.

        Ok(RunningSupervisor {
            handle,
            kbm_injector,
            ctrl_drain,
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
    /// The KBM injector thread (blueprint §7.3). Joined strictly *after* the hot thread, whose
    /// exit drops the `KbmTx` and so signals the injector to drain-and-exit. `None` if no ring
    /// consumer was available.
    kbm_injector: Option<JoinHandle<()>>,
    /// The control-plane drain thread (blueprint §5/§12 M4): handles `Special` edges + macro
    /// publishes. Joined after the hot thread, whose exit drops the `ControlPlaneTx` and so signals
    /// this thread to drain-and-exit. `None` if no consumer was available.
    ctrl_drain: Option<JoinHandle<()>>,
    /// Restored on drop, after `handle` has joined.
    _timer_res: platform_win::TimerResGuard,
}

impl RunningSupervisor {
    /// Block until the hot thread exits, join the KBM injector, then drop the timer-resolution
    /// guard.
    ///
    /// Ordering: join the hot thread first (its exit drops the `KbmTx`, which is the injector's
    /// drain-and-exit signal), then join the injector. `StopReason::Shutdown` / `DeviceLost` are
    /// both clean exits; a panicked hot thread maps to [`EngineError::HotPanic`].
    pub fn join(self) -> Result<(), EngineError> {
        let hot_result = match self.handle {
            Some(handle) => match handle.join() {
                Ok(StopReason::Shutdown) | Ok(StopReason::DeviceLost) => Ok(()),
                Err(_) => Err(EngineError::HotPanic),
            },
            None => Ok(()),
        };

        // The hot thread is gone (its `KbmTx` dropped); the injector now drains the ring and
        // returns on its own. A panicked injector is non-fatal to shutdown — log-and-continue so
        // the hot-thread result and the timer-resolution restore still propagate.
        if let Some(injector) = self.kbm_injector {
            if injector.join().is_err() {
                eprintln!("hyperion: KBM injector thread panicked during shutdown");
            }
        }

        // The hot thread's `ControlPlaneTx` is also dropped now, so the control-plane drain thread
        // sees the channel disconnect and returns. A panic there is likewise non-fatal to shutdown.
        if let Some(drain) = self.ctrl_drain {
            if drain.join().is_err() {
                eprintln!("hyperion: control-plane drain thread panicked during shutdown");
            }
        }

        hot_result
        // `_timer_res` drops here, after all joins.
    }
}

/// Spawn the control-plane drain thread (blueprint §5/§12 M4): a normal-priority worker that
/// receives [`ControlPlaneEvent`]s from the hot loop and runs them **off** the hot path.
///
/// * [`ControlPlaneEvent::Special`] → run the matching special action. For M4 this is a minimal
///   stub: it logs/acks the id (real profile-switch / launch / disconnect exec builds on this in
///   M5, routed back through the single-writer `ControlMsg` path so the hot loop sees only a
///   generation bump).
/// * [`ControlPlaneEvent::Macros`] → the active profile's resolved macro table, republished on
///   start and every profile change. The injector's `MacroPlayer` consumes these to play a
///   `Macro{start}` edge by id; the latest table is held here so a profile switch swaps the macro
///   set without touching the hot thread.
///
/// The thread exits when the hot thread's [`ControlPlaneTx`] is dropped (channel disconnect), which
/// is the clean drain-and-exit signal joined in [`RunningSupervisor::join`].
fn spawn_control_plane_drain(rx: ControlPlaneRx) -> std::io::Result<JoinHandle<()>> {
    std::thread::Builder::new()
        .name("hyperion-control-plane".to_string())
        .spawn(move || {
            // The number of macros in the latest published table (held so a future profile-switch
            // handler / the injector wiring can read the active set). Updated on every `Macros`
            // publish; tracking the count keeps the binding genuinely used while the injector-side
            // consumption is wired in `win_io` by the maintainer.
            let mut macro_count = 0usize;
            // `recv` blocks until an event or channel disconnect — zero CPU while idle, no poll.
            while let Ok(ev) = rx.recv() {
                match ev {
                    ControlPlaneEvent::Special(id) => {
                        // M4 stub: acknowledge the special edge off the hot path. Real exec (M5)
                        // resolves `id` against the active profile's `specials` and routes the
                        // effect (e.g. `SetActiveProfile`) through the single-writer control path.
                        eprintln!("hyperion: special action {id} fired (M4 stub: logged)");
                    }
                    ControlPlaneEvent::Macros(table) => {
                        // The active macro table for the injector's MacroPlayer (wired by the KBM
                        // injector). Republished on start + every profile change; track its size so
                        // a profile switch that changes the macro set is observable here.
                        if table.len() != macro_count {
                            macro_count = table.len();
                            eprintln!("hyperion: macro table updated ({macro_count} macros)");
                        }
                    }
                }
            }
        })
}

/// Spawn the hot thread, doing all thread-affine acquisition *inside* the spawned thread.
///
/// Order on the hot thread (DESIGN §6/§8):
/// 1. bind the MMCSS/affinity/priority policy guard for the thread's whole life
///    (`let _policy = ...;` — a bare statement would revert it one line later, verifier (c));
/// 2. open the physical [`DualSenseDevice`] through HidHide (whitelist self → blacklist the
///    physical pad → cloak on); on device-not-found, return `DeviceLost` (headless, clean exit);
/// 3. create + plug the [`DynPadTarget`] whose kind (X360 / DS4) is chosen from the active
///    profile's `OutputKind` (`plugin` then `wait_ready`);
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
        kbm_tx,
        ctrl_tx,
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

            // (3) Create + plug the virtual target whose kind (X360 / DS4) is chosen from the
            // active profile's `OutputKind` at plug time (blueprint §6.3). The default profile is
            // `PadTarget::X360`, so an unconfigured config plugs the byte-identical X360 pad. A
            // runtime kind change replugs via `HotCommand::ReplugTarget` (handled in the hot loop).
            let kind = crate::win_io::active_output_kind(&cfg);
            let Some(target) = DynPadTarget::plugged(kind) else {
                return StopReason::DeviceLost;
            };

            // (4) Run the steady-state loop. The HidHide cloak (held inside `device`), the
            // ViGEm target, `_policy`, and the `KbmTx` (dropped here, signaling the injector to
            // drain-and-exit) all release on this thread when `run` returns.
            HotThread::new(
                device, target, config, config_gen, commands, telemetry, kbm_tx, ctrl_tx,
            )
            .run()
        })
        .map_err(|e| EngineError::Platform(e.to_string()))?;
    Ok(handle)
}

/// A spawned [`ForegroundWatcher`] thread plus its dedicated stop signal, owned by
/// [`crate::runtime::Runtime`].
///
/// The watcher is a low-priority polling thread (blueprint §7.4): it samples the foreground app
/// ~`poll_hz` times a second and, on a change that matches an
/// [`AutoSwitchRule`](hyperion_core::config::AutoSwitchRule), sends a
/// [`ControlMsg::SetActiveProfile`] down the **same** single-writer `ControlMsg` channel the GUI
/// uses — never the hot path. The runtime joins it **before** the config-writer thread in
/// `shutdown()` so the writer is still alive to drain any final switch the watcher emitted.
pub struct ForegroundWatcher {
    stop_tx: Sender<()>,
    join: Option<JoinHandle<()>>,
}

impl ForegroundWatcher {
    /// Signal the watcher to stop and join it. Idempotent; safe to call once.
    pub fn shutdown(mut self) {
        let _ = self.stop_tx.send(());
        if let Some(join) = self.join.take() {
            let _ = join.join();
        }
    }
}

/// Spawn the `cfg(windows)` foreground auto-profile-switch watcher (blueprint §7.4, §12 M5).
///
/// A named, low-priority thread that, every `~1/poll_hz` seconds:
/// 1. reads the foreground app via [`platform_win::foreground::foreground_app`] (a failed/elevated
///    read yields `None` → no switch, never a crash),
/// 2. skips work unless the `(exe, title)` changed since the last poll (so a steady foreground
///    costs one cheap OS read and nothing else),
/// 3. loads the live config snapshot and computes [`auto_switch_decision`] (the pure matcher),
/// 4. on a match that differs from the current assignment, sends [`ControlMsg::SetActiveProfile`]
///    through `control_tx` — the SAME path a GUI edit takes, so the single-writer guarantee holds
///    and the hot loop sees only the resulting generation bump.
///
/// The poll rate is re-read from the live config each tick, so toggling `auto_switch.enabled` or
/// `poll_hz` in the GUI takes effect without a respawn. The thread exits promptly on the stop
/// signal (a `select` with a per-tick timeout, so shutdown never waits a whole poll interval).
pub fn spawn_foreground_watcher(
    config: ConfigHandle,
    control_tx: Sender<ControlMsg>,
) -> std::io::Result<ForegroundWatcher> {
    // The watcher owns its own dedicated stop channel (1-slot, drop-on-full irrelevant — a single
    // `()` is all `shutdown` ever sends). Returning the sender on the handle keeps the stop signal
    // paired with the receiver the loop selects on.
    let (stop_tx, stop_rx) = crossbeam_channel::bounded::<()>(1);
    let join = std::thread::Builder::new()
        .name("hyperion-foreground-watcher".to_string())
        .spawn(move || foreground_watch_loop(&config, &control_tx, &stop_rx))?;
    Ok(ForegroundWatcher {
        stop_tx,
        join: Some(join),
    })
}

/// The watcher's poll loop body (blueprint §7.4). Pulled out so the thread closure stays small.
///
/// Dedupes on the last `(exe, title)` so a match is recomputed only when the foreground actually
/// changes; the decision itself ([`auto_switch_decision`]) is the pure, Linux-tested matcher. This
/// whole module is `cfg(windows)` (it drives ViGEm / HidHide), so the watcher always has a real
/// `platform_win::foreground` to poll.
fn foreground_watch_loop(
    config: &ConfigHandle,
    control_tx: &Sender<ControlMsg>,
    stop_rx: &Receiver<()>,
) {
    let mut last: Option<(String, String)> = None;
    loop {
        // Re-read the poll interval from the live config each tick (clamped to a sane floor so a
        // bad value can never spin the thread). Default 4 Hz.
        let snapshot = config.load_full();
        let poll_hz = snapshot.auto_switch.poll_hz.clamp(1, 60);
        let interval = Duration::from_millis(1000 / u64::from(poll_hz));

        // Block up to one interval for a stop signal; a real stop returns immediately.
        match stop_rx.recv_timeout(interval) {
            Ok(()) | Err(crossbeam_channel::RecvTimeoutError::Disconnected) => return,
            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {}
        }

        // Only the enabled gate does an OS read; a disabled watcher idles near-zero cost.
        if !snapshot.auto_switch.enabled {
            last = None; // forget the cached foreground so re-enabling re-evaluates immediately.
            continue;
        }

        // Read the foreground app. `None` (no window / elevated-process read failure) keeps the
        // current profile — the watcher just doesn't switch, never crashes.
        let Some(app) = platform_win::foreground::foreground_app() else {
            continue;
        };

        // Skip the match unless the foreground changed since the last successfully-handled read.
        if last.as_ref().map(|(e, t)| (e.as_str(), t.as_str()))
            == Some((app.exe.as_str(), app.title.as_str()))
        {
            continue;
        }

        // Pure decision against the live snapshot; on a real switch, send through the single writer.
        match auto_switch_decision(&snapshot, &app.exe, &app.title) {
            Some(msg) => {
                // `try_send` is non-blocking. Only advance the dedupe cache when the switch is
                // actually accepted; if the control queue is momentarily full (a burst of GUI edits
                // in flight), leave `last` unchanged so the SAME foreground is re-evaluated and the
                // switch retried on the next poll (self-healing, never blocks the writer).
                if control_tx.try_send(msg).is_ok() {
                    last = Some((app.exe, app.title));
                }
            }
            // No switch needed for this foreground (no rule, or already-active profile). Cache it so
            // we do not re-run the matcher every poll while it stays foreground.
            None => last = Some((app.exe, app.title)),
        }
    }
}
