//! Hyperion application entry point (M2: the live egui tuning GUI).
//!
//! On Windows this binary now owns the **GUI main thread**: it builds an [`EngineConfig`]
//! (loading `hyperion.toml` from the executable's directory if present, else built-in defaults),
//! starts the non-blocking [`engine::Runtime`] (which spawns the cross-platform config-writer
//! thread plus the hot thread + supervisor), hands the telemetry reader + control sender to the
//! egui app, runs the eframe/winit event loop on this thread, and calls
//! [`engine::Runtime::shutdown`] on exit (`DESIGN.md` §6/§10).
//!
//! The GUI never touches the hot loop: it only *reads* a triple-buffered telemetry frame and
//! *sends* [`engine::ControlMsg`] edits to the single config-writer thread, which validates,
//! clamps, and republishes the immutable `ArcSwap` snapshot.
//!
//! The engine runtime is Windows-only (real HID / ViGEm / HidHide I/O and the eframe/winit GUI),
//! so on other platforms this binary is only a compile check for the pure crates: `main` prints
//! a notice and exits 0 without building any GUI or runtime.

// The GUI lives in its own module tree; it is Windows-only so the Linux build stays gtk-free.
#[cfg(windows)]
mod gui;

/// Windows: wire the engine and run the egui + tray event loop on the main thread.
///
/// 1. Load the config (TOML next to the exe, else defaults).
/// 2. `engine::Runtime::start` — non-blocking; spawns the writer + hot threads.
/// 3. Take the telemetry reader, a control sender, and the seed config snapshot.
/// 4. Run eframe on this thread; the close button / tray Quit / Ctrl-C all request exit.
/// 5. On loop exit, `runtime.shutdown()` joins the hot loop (restoring timer resolution) and the
///    config-writer thread.
#[cfg(windows)]
fn main() -> std::process::ExitCode {
    use engine::Runtime;

    let (cfg, cfg_path) = load_config();

    // NON-blocking: returns immediately so we can own the eframe/winit loop on this thread.
    let mut runtime = match Runtime::start(cfg, cfg_path) {
        Ok(rt) => rt,
        Err(e) => {
            eprintln!("hyperion: engine start failed: {e}");
            return std::process::ExitCode::FAILURE;
        }
    };

    // Take the GUI-facing handoffs once, before the runtime is moved into shutdown.
    let control_tx = runtime.control_sender();
    let Some(telemetry_rx) = runtime.telemetry_reader() else {
        eprintln!("hyperion: telemetry reader already taken (internal error)");
        runtime.shutdown();
        return std::process::ExitCode::FAILURE;
    };
    let snapshot = runtime.config_snapshot();

    // Ctrl-C from a console: ask egui to close its viewport, which ends `run_native` and falls
    // through to the shutdown below. A second control sender clone routes the request safely; the
    // GUI thread observes the close via the viewport-close path.
    install_ctrlc(control_tx.clone());

    let app = gui::HyperionApp::new(control_tx, telemetry_rx, snapshot);

    let result = eframe::run_native(
        "Hyperion",
        gui::native_options(),
        Box::new(move |cc| Ok(Box::new(app.with_tray(cc)))),
    );

    // The eframe loop has exited (window closed / tray Quit / Ctrl-C). Tear the engine down:
    // this joins the hot loop first (restoring the timer resolution) then the writer thread.
    runtime.shutdown();

    match result {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("hyperion: GUI exited with error: {e}");
            std::process::ExitCode::FAILURE
        }
    }
}

/// Install a Ctrl-C handler that asks the GUI to quit cleanly.
///
/// The engine runtime lives on (and is torn down from) this main thread, so the signal handler
/// cannot touch it. Instead it flips the process-wide quit flag the GUI polls every frame
/// ([`gui::request_quit`]); the GUI turns that into a `ViewportCommand::Close`, `run_native`
/// returns, and `main` runs the orderly engine shutdown. `keepalive` is a `ControlMsg` sender
/// clone moved into the handler purely to keep the control channel open for the handler's
/// lifetime (it is never sent on).
#[cfg(windows)]
fn install_ctrlc(keepalive: crossbeam_channel::Sender<engine::ControlMsg>) {
    let result = ctrlc::set_handler(move || {
        // Hold the channel open for the handler's lifetime, then request a graceful quit.
        let _keepalive = &keepalive;
        gui::request_quit();
    });
    if let Err(e) = result {
        eprintln!("hyperion: could not install Ctrl-C handler: {e}");
    }
}

/// Load the engine config from `hyperion.toml` next to the executable, falling back to defaults.
///
/// Returns the config plus the resolved path (when one could be determined) so the runtime can
/// honor [`engine::ControlMsg::SaveToDisk`] / [`engine::ControlMsg::ReloadFromDisk`]. A missing
/// file is normal (first run) and silently uses defaults but still reports the intended path so
/// Save can create it. A present-but-malformed file is reported and also falls back to defaults
/// rather than refusing to start — the config store validates/clamps everything downstream.
#[cfg(windows)]
fn load_config() -> (engine::config::EngineConfig, Option<std::path::PathBuf>) {
    use engine::config::{load_toml, EngineConfig};

    let path = std::env::current_exe()
        .ok()
        .and_then(|exe| exe.parent().map(|dir| dir.join("hyperion.toml")));

    let Some(path) = path else {
        return (EngineConfig::default(), None);
    };

    let cfg = match std::fs::read_to_string(&path) {
        Ok(text) => match load_toml(&text) {
            Ok(cfg) => {
                eprintln!("hyperion: loaded config from {}", path.display());
                cfg
            }
            Err(e) => {
                eprintln!(
                    "hyperion: {} is malformed ({e}); using defaults",
                    path.display()
                );
                EngineConfig::default()
            }
        },
        // Not found / unreadable is the common, benign case: use defaults but keep the path so a
        // later Save creates the file there.
        Err(_) => EngineConfig::default(),
    };
    (cfg, Some(path))
}

/// Non-Windows: Hyperion's runtime + GUI are Windows-only; this build exists purely so the pure
/// `core` + `engine` modules compile and test on Linux CI. No eframe / winit / gtk is pulled in.
#[cfg(not(windows))]
fn main() {
    eprintln!("Hyperion is Windows-only; this build is a Linux compile check.");
}
