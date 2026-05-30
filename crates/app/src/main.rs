//! Hyperion application entry point.
//!
//! M1 is a **headless** vertical slice: it assembles the supervisor and hot thread via
//! `engine::run` and runs until shutdown. There is no egui GUI yet — the tuning surface and
//! tray land in M2 per the roadmap (`DESIGN.md` §12), so `app` stays minimal to keep M1 CI
//! robust.
//!
//! The engine is Windows-only (real HID / ViGEm / HidHide I/O). On other platforms this binary
//! is only a compile check for the pure crates, so `main` just prints a notice and exits 0.

/// Windows: wire and run the engine (headless for M1).
#[cfg(windows)]
fn main() -> std::process::ExitCode {
    match engine::run() {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("hyperion: engine exited with error: {e}");
            std::process::ExitCode::FAILURE
        }
    }
}

/// Non-Windows: Hyperion's runtime is Windows-only; this build exists purely so the pure
/// `core` + `engine` modules compile and test on Linux CI.
#[cfg(not(windows))]
fn main() {
    eprintln!("Hyperion is Windows-only; this build is a Linux compile check.");
}
