//! Hyperion application entry point.
//!
//! M1 is a **headless** vertical slice: it assembles the supervisor and hot thread and runs
//! until shutdown. There is no egui GUI yet — the tuning surface and tray land in M2 per the
//! roadmap (`DESIGN.md` §12), so `app` stays minimal to keep M1 CI robust.
//!
//! The engine is Windows-only (real HID / ViGEm / HidHide I/O). On other platforms this binary
//! is only a compile check for the pure crates, so `main` just prints a notice and exits 0.

/// Windows: wire and run the engine (headless for M1).
///
/// Loads `hyperion.toml` from the executable's directory if present (else built-in defaults),
/// builds the [`engine::supervisor::Supervisor`], installs a Ctrl-C handler that asks the hot
/// loop to shut down, then blocks until the hot thread exits.
#[cfg(windows)]
fn main() -> std::process::ExitCode {
    use engine::handoff::HotCommand;
    use engine::supervisor::Supervisor;

    let cfg = load_config();

    let mut supervisor = match Supervisor::with_config(cfg) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("hyperion: supervisor init failed: {e}");
            return std::process::ExitCode::FAILURE;
        }
    };

    // Install the Ctrl-C handler before `run()` (which consumes the supervisor and blocks).
    // The handler owns a command producer to the hot loop; the hot loop drains both the GUI and
    // supervisor queues every report, so a `Shutdown` pushed here is honored promptly.
    if let Some(mut gui_tx) = supervisor.take_gui_tx() {
        let result = ctrlc::set_handler(move || {
            // Best-effort: a full queue means the hot loop is already wedged (other telemetry
            // would surface that); there is nothing useful to do from the signal handler.
            let _ = gui_tx.send(HotCommand::Shutdown);
        });
        if let Err(e) = result {
            eprintln!("hyperion: could not install Ctrl-C handler: {e}");
        }
    }

    match supervisor.run() {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("hyperion: engine exited with error: {e}");
            std::process::ExitCode::FAILURE
        }
    }
}

/// Load the engine config from `hyperion.toml` next to the executable, falling back to defaults.
///
/// A missing file is normal (first run / headless) and silently uses defaults. A present but
/// malformed file is reported and also falls back to defaults rather than refusing to start —
/// the engine still validates/clamps everything downstream through the config store.
#[cfg(windows)]
fn load_config() -> engine::config::EngineConfig {
    use engine::config::{load_toml, EngineConfig};

    let path = std::env::current_exe()
        .ok()
        .and_then(|exe| exe.parent().map(|dir| dir.join("hyperion.toml")));

    let Some(path) = path else {
        return EngineConfig::default();
    };

    match std::fs::read_to_string(&path) {
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
        // Not found / unreadable is the common, benign case: use defaults silently.
        Err(_) => EngineConfig::default(),
    }
}

/// Non-Windows: Hyperion's runtime is Windows-only; this build exists purely so the pure
/// `core` + `engine` modules compile and test on Linux CI.
#[cfg(not(windows))]
fn main() {
    eprintln!("Hyperion is Windows-only; this build is a Linux compile check.");
}
