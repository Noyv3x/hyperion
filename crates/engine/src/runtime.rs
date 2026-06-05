//! The engine [`Runtime`]: a non-blocking handle the GUI drives from the main thread.
//!
//! `Runtime::start` wires the whole engine and **returns immediately** so the caller can run
//! the egui event loop on the main thread (`DESIGN.md` §6/§10). It owns:
//!
//! * the **control-writer thread** (cross-platform) — the *single* writer of the config
//!   `ArcSwap`. It owns one [`ConfigStore`] and drains a `crossbeam_channel::Receiver<ControlMsg>`,
//!   calling [`ConfigStore::apply`] per message. The GUI only ever *sends* `ControlMsg`s; it
//!   never touches the snapshot, so there is no shared `Mutex` with the hot loop.
//! * on Windows, the **hot thread + supervisor** (`RunningSupervisor`) under the correct
//!   timer-resolution / HidHide / ViGEm ordering.
//!
//! The GUI seeds widget values from [`Runtime::config_snapshot`] (a wait-free `arc-swap` load),
//! reads telemetry from the [`TelemetryRx`] it takes once via [`Runtime::telemetry_reader`], and
//! sends edits through a clone of [`Runtime::control_sender`].

use std::sync::Arc;
use std::thread::JoinHandle;

use crossbeam_channel::{Receiver, Sender};
use hyperion_core::config::EngineConfig;

use crate::config_store::ConfigStore;
use crate::control::ControlMsg;
use crate::error::EngineError;
use crate::handoff::{ConfigHandle, TelemetryRx};

/// Capacity of the GUI → control-writer channel. Config edits are user-paced (slider drags at
/// UI frame rate at most), so a small bounded buffer absorbs bursts without unbounded growth;
/// a full channel briefly blocks the GUI thread (never the hot loop), which is acceptable.
const CONTROL_QUEUE_CAP: usize = 256;

/// A running engine, owned by the GUI's main thread.
///
/// Construct with [`Runtime::start`] (non-blocking); tear down with [`Runtime::shutdown`]. All
/// fields are control-plane only — nothing here is touched by the hot loop, which communicates
/// solely through the lock-free handoffs built in [`crate::handoff`].
pub struct Runtime {
    /// GUI → control-writer channel; cloned for every GUI sender via [`Runtime::control_sender`].
    control_tx: Sender<ControlMsg>,
    /// Signals the control-writer thread to stop even while GUI sender clones are still alive.
    writer_stop_tx: Sender<()>,
    /// Join handle for the control-writer thread.
    writer_join: Option<JoinHandle<()>>,
    /// Shared config snapshot (wait-free load for GUI widget seeding).
    config: ConfigHandle,
    /// Telemetry reader handed to the GUI thread exactly once.
    telemetry_rx: Option<TelemetryRx>,
    /// Windows hot-thread lifecycle: the running supervisor handle and its shutdown producer.
    #[cfg(windows)]
    win: Option<WinRuntime>,
}

/// Windows-only hot-thread lifecycle owned by the [`Runtime`].
#[cfg(windows)]
struct WinRuntime {
    running: crate::supervisor::RunningSupervisor,
    /// Producer of [`crate::handoff::HotCommand::Shutdown`] to the hot loop.
    sup_tx: crate::handoff::CommandTx,
    /// The foreground auto-profile-switch watcher (blueprint §7.4). Joined **before** the
    /// config-writer thread in [`Runtime::shutdown`] so a final `SetActiveProfile` it sends is
    /// still drained by the single writer.
    foreground_watcher: crate::supervisor::ForegroundWatcher,
}

impl Runtime {
    /// Wire and start the engine, returning immediately (NON-blocking).
    ///
    /// Always spawns the cross-platform control-writer thread (the sole `ArcSwap` writer). On
    /// Windows it additionally builds the `supervisor::Supervisor` and spawns the hot thread
    /// under the correct resource ordering. `cfg_path`, when `Some`, is the
    /// backing TOML file used by [`ControlMsg::SaveToDisk`] / [`ControlMsg::ReloadFromDisk`].
    pub fn start(
        cfg: EngineConfig,
        cfg_path: Option<std::path::PathBuf>,
    ) -> Result<Runtime, EngineError> {
        Self::start_impl(cfg, cfg_path)
    }

    /// Windows assembly: build the supervisor (which builds the handoff links + store), take the
    /// store onto the control-writer thread, then spawn the hot thread.
    #[cfg(windows)]
    fn start_impl(
        cfg: EngineConfig,
        cfg_path: Option<std::path::PathBuf>,
    ) -> Result<Runtime, EngineError> {
        let mut supervisor = crate::supervisor::Supervisor::with_config(cfg)?;

        // The single-writer store shares the hot loop's ArcSwap + generation counter; attach the
        // optional backing file for Save/Reload.
        let store = supervisor
            .take_config_store()
            .expect("fresh supervisor owns its config store")
            .with_path(cfg_path);
        let config = store.handle();
        let telemetry_rx = supervisor.take_telemetry_rx();
        let sup_tx = supervisor
            .take_sup_tx()
            .expect("fresh supervisor owns its command producer");

        let (control_tx, control_rx) = crossbeam_channel::bounded(CONTROL_QUEUE_CAP);
        let (writer_stop_tx, writer_stop_rx) = crossbeam_channel::bounded(1);
        let writer_join = spawn_control_writer(store, control_rx, writer_stop_rx);

        // Wire the single-writer ControlMsg sender into the supervisor so a profile-switch special
        // action routes through the SAME path the GUI / auto-switch watcher use (M7). Must precede
        // `spawn()` (which consumes the supervisor). Program-launch specials need no sender.
        supervisor.set_control_sender(control_tx.clone());

        // Spawn the foreground auto-profile-switch watcher (blueprint §7.4): a low-priority polling
        // thread that sends `SetActiveProfile` through a clone of the SAME single-writer channel the
        // GUI uses (the single-writer guarantee is intact — it is just another `ControlMsg` sender,
        // never the hot path). It reads the live config snapshot to scope rules + dedupe; the hot
        // loop picks up the resulting switch through the existing generation gate.
        let foreground_watcher =
            crate::supervisor::spawn_foreground_watcher(config.clone(), control_tx.clone())
                .map_err(|e| EngineError::Platform(e.to_string()))?;

        // Spawn the hot thread last (NON-blocking); the running handle owns the timer-resolution
        // guard + join handle.
        let running = supervisor.spawn()?;

        Ok(Runtime {
            control_tx,
            writer_stop_tx,
            writer_join: Some(writer_join),
            config,
            telemetry_rx,
            win: Some(WinRuntime {
                running,
                sup_tx,
                foreground_watcher,
            }),
        })
    }

    /// Non-Windows assembly: there is no hot thread / supervisor (the runtime is Windows-only),
    /// but the control-writer thread + store + telemetry handoff are cross-platform, so the
    /// Runtime still builds and is unit-testable on Linux CI.
    #[cfg(not(windows))]
    fn start_impl(
        cfg: EngineConfig,
        cfg_path: Option<std::path::PathBuf>,
    ) -> Result<Runtime, EngineError> {
        // The KBM egress ring is built cross-platform (it is pure `rtrb<KbmBatch>`); on non-Windows
        // there is no injector thread to drain it, so both ends are dropped immediately here. The
        // hot thread + injector are Windows-only (this `Runtime` is a control-plane-only build off
        // Windows, kept so the cross-platform pieces stay unit-testable on Linux CI).
        let (config, (_telemetry_tx, telemetry_rx), _commands, _kbm) =
            crate::handoff::build_links(cfg);
        let store = ConfigStore::from_handle(config.clone()).with_path(cfg_path);

        let (control_tx, control_rx) = crossbeam_channel::bounded(CONTROL_QUEUE_CAP);
        let (writer_stop_tx, writer_stop_rx) = crossbeam_channel::bounded(1);
        let writer_join = spawn_control_writer(store, control_rx, writer_stop_rx);

        Ok(Runtime {
            control_tx,
            writer_stop_tx,
            writer_join: Some(writer_join),
            config,
            telemetry_rx: Some(telemetry_rx),
        })
    }

    /// A `ControlMsg` sender to clone into the GUI thread. Every clone routes to the single
    /// control-writer thread; the GUI never writes the `ArcSwap` directly.
    pub fn control_sender(&self) -> Sender<ControlMsg> {
        self.control_tx.clone()
    }

    /// Take the telemetry reader to hand to the GUI thread. Returns `Some` exactly once.
    pub fn telemetry_reader(&mut self) -> Option<TelemetryRx> {
        self.telemetry_rx.take()
    }

    /// The current config snapshot (wait-free `arc-swap` load) for seeding GUI widget values.
    pub fn config_snapshot(&self) -> Arc<EngineConfig> {
        self.config.load_full()
    }

    /// Stop the hot loop and the control-writer thread, then join both.
    ///
    /// Ordering: signal the hot loop (`HotCommand::Shutdown`) and join it first so the timer
    /// resolution guard (inside the running supervisor) is restored before anything else; then
    /// signal the control-writer thread (which works even while GUI sender clones survive) and
    /// join it.
    pub fn shutdown(mut self) {
        #[cfg(windows)]
        if let Some(mut win) = self.win.take() {
            // Ask the hot loop to stop, then join it (restores timer resolution on drop).
            let _ = win.sup_tx.send(crate::handoff::HotCommand::Shutdown);
            let _ = win.running.join();
            // Stop the foreground watcher BEFORE the writer below: it must emit no further
            // `SetActiveProfile` once we tear down, and any switch already in flight is still
            // drained because the writer thread is stopped only after this returns.
            win.foreground_watcher.shutdown();
        }

        // Stop the control-writer thread even though `self.control_tx` (and GUI clones) may still
        // be alive: the dedicated stop channel unblocks its `select!`.
        let _ = self.writer_stop_tx.send(());
        if let Some(join) = self.writer_join.take() {
            let _ = join.join();
        }
    }
}

/// Spawn the single config-writer thread: it owns the [`ConfigStore`] and is the *only* writer
/// of the `ArcSwap`. It drains `control_rx`, applying each [`ControlMsg`] via
/// [`ConfigStore::apply`], until either the channel disconnects (all senders dropped) or a stop
/// signal arrives on `stop_rx`.
fn spawn_control_writer(
    store: ConfigStore,
    control_rx: Receiver<ControlMsg>,
    stop_rx: Receiver<()>,
) -> JoinHandle<()> {
    std::thread::Builder::new()
        .name("hyperion-config-writer".to_string())
        .spawn(move || {
            loop {
                crossbeam_channel::select! {
                    recv(control_rx) -> msg => match msg {
                        Ok(msg) => {
                            // The single writer validates/clamps and republishes; `apply`'s
                            // bool result is intentionally ignored here (a no-op simply does
                            // not bump the generation, which the hot loop never observes).
                            let _ = store.apply(&msg);
                        }
                        // All senders dropped: nothing more will arrive, exit cleanly.
                        Err(_) => break,
                    },
                    recv(stop_rx) -> _ => break,
                }
            }
        })
        .expect("spawning the config-writer thread must not fail")
}

#[cfg(test)]
mod tests {
    use super::*;
    use hyperion_core::input::Control;
    use hyperion_core::map::{BindTarget, PadBtn, Profile};

    /// A config with one device assigned to a `"default"` profile so control edits have a target.
    fn cfg_with_profile() -> EngineConfig {
        let mut cfg = EngineConfig {
            active_device: "dev".to_string(),
            ..EngineConfig::default()
        };
        std::sync::Arc::make_mut(&mut cfg.profiles).insert(
            "default".to_string(),
            Profile {
                name: "default".to_string(),
                ..Profile::default()
            },
        );
        cfg.assignments
            .insert("dev".to_string(), "default".to_string());
        cfg
    }

    #[test]
    fn start_then_edit_is_observed_in_the_snapshot() {
        let mut rt = Runtime::start(cfg_with_profile(), None).expect("runtime starts");

        // Seed value from the snapshot: Cross is unbound (identity passthrough).
        assert!(!rt.config_snapshot().profiles["default"]
            .bindings
            .contains_key(&Control::Cross));

        // The GUI sends a binding edit through a cloned sender; the writer thread applies it.
        let tx = rt.control_sender();
        tx.send(ControlMsg::SetBinding {
            profile: "default".to_string(),
            control: Control::Cross,
            bind: BindTarget::GamepadButton(PadBtn::B),
        })
        .expect("send to the control-writer thread");

        // Poll the wait-free snapshot until the single writer thread applies + publishes. Sleep
        // (not `yield_now`) between polls: `yield_now` is only a hint and need not schedule the
        // freshly-spawned writer, so a busy spin can starve it; a short sleep reliably hands off.
        let mut observed = None;
        for _ in 0..200 {
            let snap = rt.config_snapshot();
            if snap.profiles["default"]
                .bindings
                .get(&Control::Cross)
                .map(|s| s.bind)
                == Some(BindTarget::GamepadButton(PadBtn::B))
            {
                observed = Some(());
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        assert!(observed.is_some(), "edit must reach the published snapshot");

        // Telemetry reader is handed out exactly once.
        assert!(rt.telemetry_reader().is_some());
        assert!(rt.telemetry_reader().is_none());

        rt.shutdown();
    }

    #[test]
    fn shutdown_joins_cleanly_with_outstanding_sender() {
        let rt = Runtime::start(cfg_with_profile(), None).expect("runtime starts");
        // Keep a GUI sender clone alive across shutdown: the dedicated stop signal must still
        // unblock and join the writer thread (it does not rely on channel disconnect).
        let _alive = rt.control_sender();
        rt.shutdown();
    }
}
