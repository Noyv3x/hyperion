//! The live egui tuning GUI (Windows-only).
//!
//! [`HyperionApp`] implements [`eframe::App`]. It runs on the eframe/winit **main thread** and is
//! wholly decoupled from the engine's hot loop (`DESIGN.md` §6/§10):
//!
//! * it **reads** the latest [`engine::telemetry::TelemetryFrame`] from a triple-buffer every
//!   frame (wait-free; the hot loop never blocks), and
//! * it **sends** [`engine::ControlMsg`] edits down a `crossbeam_channel` to the engine's single
//!   config-writer thread — it never mutates the shared config `ArcSwap` and never takes a lock
//!   the hot loop uses.
//!
//! Widget state lives in a local editable mirror ([`DeviceMirror`]) seeded once from the
//! snapshot the runtime hands over at construction. Any widget change rebuilds the affected
//! [`hyperion_core::rc::RcConfig`] / [`hyperion_core::config::StickMode`] / thread / hidhide
//! value and sends the corresponding `ControlMsg`; the engine is the sole writer that validates,
//! clamps, and republishes.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use eframe::egui;
use engine::config::EngineConfig;
use engine::handoff::TelemetryRx;
use engine::telemetry::TelemetryFrame;
use engine::{ControlMsg, Stick};
use hyperion_core::config::{DeviceConfig, HidHideConfig, StickConfig, StickMode, ThreadConfig};

mod panels;
mod scope;
mod tray;

use tray::TrayState;

/// Process-wide "please quit" flag. Set by the Ctrl-C handler (which cannot touch the viewport)
/// and by the tray's Quit item; polled by [`HyperionApp::update`], which turns it into a
/// `ViewportCommand::Close` so the eframe loop exits and `main` runs the engine shutdown.
static QUIT_REQUESTED: AtomicBool = AtomicBool::new(false);

/// Request a graceful application quit from any thread (Ctrl-C handler, tray callback).
pub fn request_quit() {
    QUIT_REQUESTED.store(true, Ordering::Relaxed);
}

/// Whether a quit has been requested since the last check.
fn quit_requested() -> bool {
    QUIT_REQUESTED.load(Ordering::Relaxed)
}

/// GUI repaint cadence: ~60 Hz so the live scope stays smooth without busy-spinning. The hot
/// loop is unaffected (this only schedules the next GUI frame).
const REPAINT_INTERVAL: Duration = Duration::from_millis(16);

/// How many recent output points to keep per stick for the scope trail.
const TRAIL_LEN: usize = 48;

/// The eframe application: telemetry reader, control sender, and the editable config mirror.
pub struct HyperionApp {
    /// GUI → engine config-writer channel. The GUI only ever *sends*; the writer is the sole
    /// `ArcSwap` writer.
    control_tx: crossbeam_channel::Sender<ControlMsg>,
    /// Triple-buffered telemetry reader (wait-free `read`); the hot loop is the writer.
    telemetry: TelemetryRx,
    /// Latest telemetry frame read this/last update, used by the scope + readouts.
    latest: TelemetryFrame,
    /// Short trails of recent *output* points per stick, for the scope (oldest first).
    ls_trail: Vec<egui::Vec2>,
    rs_trail: Vec<egui::Vec2>,
    /// The id of the device the GUI is currently editing (the active device at seed time).
    active_device: String,
    /// Local editable mirror of the active device's config (seeded from the snapshot).
    mirror: DeviceMirror,
    /// Editable mirror of the global thread + hidhide policy.
    thread: ThreadConfig,
    hidhide: HidHideConfig,
    /// The system-tray handle + menu ids; built once the eframe/winit loop is live (see
    /// [`HyperionApp::with_tray`]). `None` if tray creation failed (the GUI still works).
    tray: Option<TrayState>,
}

/// A local, editable copy of one device's two stick configs. This is the GUI's source of truth
/// for widget values; edits here are mirrored to the engine via `ControlMsg`.
pub struct DeviceMirror {
    pub ls: StickConfig,
    pub rs: StickConfig,
}

impl DeviceMirror {
    /// Borrow the editable stick config for `stick`.
    pub fn stick(&self, stick: Stick) -> &StickConfig {
        match stick {
            Stick::Left => &self.ls,
            Stick::Right => &self.rs,
        }
    }

    /// Mutably borrow the editable stick config for `stick`.
    pub fn stick_mut(&mut self, stick: Stick) -> &mut StickConfig {
        match stick {
            Stick::Left => &mut self.ls,
            Stick::Right => &mut self.rs,
        }
    }
}

impl HyperionApp {
    /// Build the app from the engine handoffs: a control sender, the telemetry reader, and the
    /// seed config snapshot (used to populate widget values for the active device).
    pub fn new(
        control_tx: crossbeam_channel::Sender<ControlMsg>,
        telemetry: TelemetryRx,
        snapshot: Arc<EngineConfig>,
    ) -> Self {
        let active_device = snapshot.active_device.clone();
        let dev = snapshot
            .devices
            .get(&active_device)
            .copied()
            .unwrap_or_else(DeviceConfig::default);
        let mirror = DeviceMirror {
            ls: dev.ls,
            rs: dev.rs,
        };

        Self {
            control_tx,
            telemetry,
            latest: TelemetryFrame::default(),
            ls_trail: Vec::with_capacity(TRAIL_LEN),
            rs_trail: Vec::with_capacity(TRAIL_LEN),
            active_device,
            mirror,
            thread: snapshot.thread.clone(),
            hidhide: snapshot.hidhide.clone(),
            tray: None,
        }
    }

    /// Build the system tray once the eframe/winit event loop is live and return `self` (called
    /// from the `run_native` creation closure so the tray is created on the winit thread). On
    /// failure the GUI still runs without a tray.
    #[must_use]
    pub fn with_tray(mut self, _cc: &eframe::CreationContext<'_>) -> Self {
        self.tray = TrayState::build();
        self
    }

    /// Send a `ControlMsg` to the engine's config-writer thread. A full/closed channel is
    /// non-fatal (the writer briefly back-pressures, or has shut down during exit); the GUI never
    /// blocks the hot loop, and a dropped edit simply does not republish.
    fn send(&self, msg: ControlMsg) {
        let _ = self.control_tx.try_send(msg);
    }

    /// Pull the freshest telemetry frame and push the new output points onto the scope trails.
    fn pump_telemetry(&mut self) {
        // Wait-free read of the latest complete frame (triple-buffer).
        self.latest = *self.telemetry.0.read();
        push_trail(&mut self.ls_trail, self.latest.out_lx, self.latest.out_ly);
        push_trail(&mut self.rs_trail, self.latest.out_rx, self.latest.out_ry);
    }
}

/// Append a point to a fixed-length trail, dropping the oldest when full.
fn push_trail(trail: &mut Vec<egui::Vec2>, x: f32, y: f32) {
    if trail.len() == TRAIL_LEN {
        trail.remove(0);
    }
    trail.push(egui::vec2(x, y));
}

impl eframe::App for HyperionApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Keep the live scope flowing without spinning the CPU.
        ctx.request_repaint_after(REPAINT_INTERVAL);
        self.pump_telemetry();

        // Tray menu events (Show / Hide / Quit) — handled before drawing so a Quit closes
        // promptly. Forwards window visibility commands through the viewport.
        if let Some(tray) = self.tray.as_ref() {
            tray.handle_events(ctx);
        }

        // A Ctrl-C handler or tray Quit asked us to exit: close the viewport so `run_native`
        // returns and `main` runs the engine shutdown.
        if quit_requested() {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
        // Honor the window close button the same way (so `main`'s shutdown always runs).
        if ctx.input(|i| i.viewport().close_requested()) {
            request_quit();
        }

        egui::TopBottomPanel::top("hyperion-top").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.heading("Hyperion");
                ui.separator();
                ui.label(format!("device: {}", self.active_device));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.button("Save").clicked() {
                        self.send(ControlMsg::SaveToDisk);
                    }
                    if ui.button("Reload").clicked() {
                        self.send(ControlMsg::ReloadFromDisk);
                    }
                });
            });
        });

        egui::TopBottomPanel::bottom("hyperion-readouts").show(ctx, |ui| {
            scope::readouts(ui, &self.latest);
        });

        egui::SidePanel::right("hyperion-scope")
            .resizable(true)
            .default_width(320.0)
            .show(ctx, |ui| {
                ui.heading("Stick scope");
                scope::draw(ui, &self.latest, &self.ls_trail, &self.rs_trail);
            });

        egui::CentralPanel::default().show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                panels::global_panel(ui, self);
                ui.separator();
                panels::stick_panel(ui, self, Stick::Left);
                ui.separator();
                panels::stick_panel(ui, self, Stick::Right);
            });
        });
    }
}

// ----- edit helpers shared by the panels: every mutation funnels through a `ControlMsg` send ---

impl HyperionApp {
    /// Apply a stick-mode change to the mirror and notify the engine.
    pub(crate) fn set_stick_mode(&mut self, stick: Stick, mode: StickMode) {
        self.mirror.stick_mut(stick).mode = mode;
        self.send(ControlMsg::SetStickMode {
            device: self.active_device.clone(),
            stick,
            mode,
        });
    }

    /// Re-send the current mirror RC params for `stick` (after any RC widget edit). The engine
    /// clamps on apply, so the GUI may send freely; the mirror keeps the user's typed value.
    pub(crate) fn push_rc(&mut self, stick: Stick) {
        let rc = self.mirror.stick(stick).rc;
        self.send(ControlMsg::SetRc {
            device: self.active_device.clone(),
            stick,
            rc,
        });
    }

    /// Push the current thread policy mirror to the engine.
    pub(crate) fn push_thread(&mut self) {
        self.send(ControlMsg::SetThread(self.thread.clone()));
    }

    /// Push the current hidhide policy mirror to the engine.
    pub(crate) fn push_hidhide(&mut self) {
        self.send(ControlMsg::SetHidHide(self.hidhide.clone()));
    }

    /// Borrow the editable thread policy mirror.
    pub(crate) fn thread_mut(&mut self) -> &mut ThreadConfig {
        &mut self.thread
    }

    /// Borrow the editable hidhide policy mirror.
    pub(crate) fn hidhide_mut(&mut self) -> &mut HidHideConfig {
        &mut self.hidhide
    }

    /// Borrow the editable device mirror.
    pub(crate) fn mirror_mut(&mut self) -> &mut DeviceMirror {
        &mut self.mirror
    }
}

/// eframe/winit native window options for the tuning GUI.
pub fn native_options() -> eframe::NativeOptions {
    eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("Hyperion")
            .with_inner_size([960.0, 640.0])
            .with_min_inner_size([720.0, 480.0]),
        ..Default::default()
    }
}
