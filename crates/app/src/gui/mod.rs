//! The live egui tuning + remap GUI (Windows-only).
//!
//! [`HyperionApp`] implements [`eframe::App`]. It runs on the eframe/winit **main thread** and is
//! wholly decoupled from the engine's hot loop (`DESIGN.md` §6/§10, `DESIGN-REMAP.md` §8):
//!
//! * it **reads** the latest [`engine::telemetry::TelemetryFrame`] from a triple-buffer every
//!   frame (wait-free; the hot loop never blocks), and
//! * it **sends** [`engine::ControlMsg`] edits down a `crossbeam_channel` to the engine's single
//!   config-writer thread — it never mutates the shared config `ArcSwap` and never takes a lock
//!   the hot loop uses.
//!
//! Widget state lives in a local editable mirror ([`ProfileMirror`]) seeded once from the
//! snapshot the runtime hands over at construction. The snapshot resolves the active **device** to
//! its assigned **profile** (`device → assignments → profiles`), and the mirror clones that
//! profile's stick settings + bindings. Any widget change rebuilds the affected value
//! ([`hyperion_core::stick::settings::StickSettings`] / [`hyperion_core::rc::RcConfig`] /
//! [`hyperion_core::map::BindingSlot`] / thread / hidhide) and sends the corresponding
//! `ControlMsg`; the engine is the sole writer that validates, clamps, and republishes.
//!
//! M4 scope (`DESIGN-REMAP.md` §12): the remap surface now spans **Mapping** (bind any control to
//! any `BindTarget` + an optional shift trigger + turbo), **Sticks** (RC + deadzone / sensitivity /
//! curve), **Mouse** (mouse-from-stick sensitivity / deadzone / accel / invert), **Macros** (timed
//! step-list editor), and **Engine** (thread / HidHide). The Triggers / gyro / profile-manager
//! screens land in later milestones; everything routes through `ControlMsg` (single writer) and
//! never touches the hot loop.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use eframe::egui;
use engine::config::EngineConfig;
use engine::handoff::TelemetryRx;
use engine::telemetry::TelemetryFrame;
use engine::{ControlMsg, Stick};
use hyperion_core::config::{HidHideConfig, ThreadConfig};
use hyperion_core::map::{MacroDef, MouseSettings};
use hyperion_core::stick::settings::StickSettings;

mod bindings;
mod macros;
mod mouse;
mod panels;
mod scope;
mod sticks;
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

/// The top-level tabs (`DESIGN-REMAP.md` §8). M4 ships **Mapping**, **Sticks**, **Mouse**,
/// **Macros**, and **Engine**; the Triggers / Gyro / Profiles screens land additively in later
/// milestones, so they are intentionally absent from this enum rather than stubbed as empty panels.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Tab {
    /// Per-control binding editor: bind any control to any [`BindTarget`] + shift + turbo.
    Mapping,
    /// Per-stick settings: RC sub-section + deadzone / sensitivity / curve.
    Sticks,
    /// Mouse-from-stick settings (sensitivity / deadzone / accel / invert).
    Mouse,
    /// Macro editor (add/edit/delete timed macro step lists).
    Macros,
    /// Global engine policy: thread / scheduling + HidHide.
    Engine,
}

impl Tab {
    /// The tab strip, in display order.
    const ALL: [Tab; 5] = [
        Tab::Mapping,
        Tab::Sticks,
        Tab::Mouse,
        Tab::Macros,
        Tab::Engine,
    ];

    /// Display label.
    fn label(self) -> &'static str {
        match self {
            Tab::Mapping => "Mapping",
            Tab::Sticks => "Sticks",
            Tab::Mouse => "Mouse",
            Tab::Macros => "Macros",
            Tab::Engine => "Engine",
        }
    }
}

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
    /// Local editable mirror of the active device's profile (seeded from the snapshot).
    mirror: ProfileMirror,
    /// Editable mirror of the global thread + hidhide policy.
    thread: ThreadConfig,
    hidhide: HidHideConfig,
    /// The currently selected top-level tab.
    tab: Tab,
    /// Transient binding-editor state (the per-control `Control → BindTarget` + shift + turbo
    /// composer); never persisted.
    binding_editor: bindings::BindingEditor,
    /// The system-tray handle + menu ids; built once the eframe/winit loop is live (see
    /// [`HyperionApp::with_tray`]). `None` if tray creation failed (the GUI still works).
    tray: Option<TrayState>,
}

/// A local, editable copy of the active profile's stick settings and bindings.
///
/// This is the GUI's source of truth for widget values; edits here are mirrored to the engine via
/// `ControlMsg`. It replaces the M2 `DeviceMirror` (which cloned two `StickConfig`s out of the
/// device) — sticks now live on the [`Profile`](hyperion_core::map::Profile) (`DESIGN-REMAP.md`
/// §3.6 / §9), so the mirror is seeded from the resolved active profile instead.
pub struct ProfileMirror {
    /// The id of the profile assigned to the active device (the edit target).
    pub profile: String,
    /// Left-stick settings (RC sub-config + full pipeline params).
    pub ls: StickSettings,
    /// Right-stick settings.
    pub rs: StickSettings,
    /// Mouse-from-stick settings (M4 Mouse tab).
    pub mouse: MouseSettings,
    /// The profile's macro definitions (M4 Macros tab).
    pub macros: Vec<MacroDef>,
}

impl ProfileMirror {
    /// Borrow the editable stick settings for `stick`.
    pub fn stick(&self, stick: Stick) -> &StickSettings {
        match stick {
            Stick::Left => &self.ls,
            Stick::Right => &self.rs,
        }
    }

    /// Mutably borrow the editable stick settings for `stick`.
    pub fn stick_mut(&mut self, stick: Stick) -> &mut StickSettings {
        match stick {
            Stick::Left => &mut self.ls,
            Stick::Right => &mut self.rs,
        }
    }
}

impl HyperionApp {
    /// Build the app from the engine handoffs: a control sender, the telemetry reader, and the
    /// seed config snapshot (used to populate widget values for the active device's profile).
    pub fn new(
        control_tx: crossbeam_channel::Sender<ControlMsg>,
        telemetry: TelemetryRx,
        snapshot: Arc<EngineConfig>,
    ) -> Self {
        let active_device = snapshot.active_device.clone();
        let mirror = ProfileMirror::from_snapshot(&snapshot, &active_device);

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
            tab: Tab::Mapping,
            binding_editor: bindings::BindingEditor::default(),
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

impl ProfileMirror {
    /// Resolve the active device to its assigned profile and clone that profile's stick settings
    /// into a fresh mirror. Falls back to a default profile (all-passthrough sticks) when no
    /// assignment / profile exists yet, so first-run (empty config) still produces a coherent
    /// editing surface rather than panicking.
    ///
    /// The resolution chain is `device → assignments[device] → profiles[id]`
    /// (`DESIGN-REMAP.md` §7.1 / §8): the assignment names the profile id the active device drives.
    fn from_snapshot(snapshot: &EngineConfig, device: &str) -> Self {
        let profile_id = resolve_profile_id(snapshot, device);
        let profile = snapshot
            .profiles
            .get(&profile_id)
            .cloned()
            .unwrap_or_default();
        Self {
            profile: profile_id,
            ls: profile.ls,
            rs: profile.rs,
            mouse: profile.mouse,
            macros: profile.macros,
        }
    }
}

/// Resolve the profile id assigned to `device`, falling back to the literal `"default"` id when no
/// assignment exists (matching the legacy-migration shim, `DESIGN-REMAP.md` §9, which synthesizes a
/// `"default"` profile + assignment for an old-shape config).
fn resolve_profile_id(snapshot: &EngineConfig, device: &str) -> String {
    snapshot
        .assignments
        .get(device)
        .cloned()
        .unwrap_or_else(|| "default".to_string())
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
                ui.separator();
                ui.label(format!("profile: {}", self.mirror.profile));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.button("Save").clicked() {
                        self.send(ControlMsg::SaveToDisk);
                    }
                    if ui.button("Reload").clicked() {
                        self.send(ControlMsg::ReloadFromDisk);
                    }
                });
            });
            // The top-level tab strip (§8).
            ui.horizontal(|ui| {
                for tab in Tab::ALL {
                    ui.selectable_value(&mut self.tab, tab, tab.label());
                }
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
            egui::ScrollArea::vertical().show(ui, |ui| match self.tab {
                Tab::Mapping => bindings::mapping_panel(ui, self),
                Tab::Sticks => {
                    sticks::stick_panel(ui, self, Stick::Left);
                    ui.separator();
                    sticks::stick_panel(ui, self, Stick::Right);
                }
                Tab::Mouse => mouse::mouse_panel(ui, self),
                Tab::Macros => macros::macros_panel(ui, self),
                Tab::Engine => panels::global_panel(ui, self),
            });
        });
    }
}

// ----- edit helpers shared by the panels: every mutation funnels through a `ControlMsg` send ---

impl HyperionApp {
    /// Re-send the active profile's whole [`StickSettings`] for `stick` (after any stick-settings
    /// or RC widget edit). The engine clamps on apply, so the GUI may send freely; the mirror keeps
    /// the user's typed value.
    ///
    /// Targets the mirror's **profile** id directly (`SetStickSettings { profile, stick, settings }`,
    /// `DESIGN-REMAP.md` §9 — sticks moved out of `DeviceConfig` into the `Profile`). The whole
    /// `StickSettings` is sent (RC sub-config included), so the legacy `SetStickMode`/`SetRc`
    /// per-field messages are not needed.
    pub(crate) fn push_stick_settings(&mut self, stick: Stick) {
        let settings = *self.mirror.stick(stick);
        self.send(ControlMsg::SetStickSettings {
            profile: self.mirror.profile.clone(),
            stick,
            settings,
        });
    }

    /// Send a single base-binding edit for `control` on the active profile. A `Passthrough`
    /// bind is the natural "clear" (identity); any other bind is the remap.
    ///
    /// Targets the mirror's **profile** id (`SetBinding { profile, control, bind }`, §9). The
    /// per-control shift / turbo fields of the slot are M4; M3 only sets the base `bind`.
    pub(crate) fn push_binding(
        &mut self,
        control: hyperion_core::input::Control,
        bind: hyperion_core::map::BindTarget,
    ) {
        self.send(ControlMsg::SetBinding {
            profile: self.mirror.profile.clone(),
            control,
            bind,
        });
    }

    /// Set (or clear, with `trigger == None`) the per-control shift trigger + shift bind on the
    /// active profile (`SetShiftTrigger`, blueprint §5 step 2 / §9).
    pub(crate) fn push_shift_trigger(
        &mut self,
        control: hyperion_core::input::Control,
        trigger: Option<hyperion_core::map::ShiftTrigger>,
        bind: hyperion_core::map::BindTarget,
    ) {
        self.send(ControlMsg::SetShiftTrigger {
            profile: self.mirror.profile.clone(),
            control,
            trigger,
            bind,
        });
    }

    /// Set (or clear, with `turbo == None`) the per-binding turbo config on the active profile
    /// (`SetBindingTurbo`, blueprint §5 / §9).
    pub(crate) fn push_binding_turbo(
        &mut self,
        control: hyperion_core::input::Control,
        turbo: Option<hyperion_core::map::TurboCfg>,
    ) {
        self.send(ControlMsg::SetBindingTurbo {
            profile: self.mirror.profile.clone(),
            control,
            turbo,
        });
    }

    /// Re-send the active profile's whole [`MouseSettings`] (after any Mouse-tab widget edit). The
    /// engine clamps on apply; the mirror keeps the user's typed value.
    pub(crate) fn push_mouse_settings(&mut self) {
        self.send(ControlMsg::SetMouseSettings {
            profile: self.mirror.profile.clone(),
            settings: self.mirror.mouse,
        });
    }

    /// Insert or replace a macro definition on the active profile (`UpsertMacro`).
    pub(crate) fn upsert_macro(&mut self, def: MacroDef) {
        self.send(ControlMsg::UpsertMacro {
            profile: self.mirror.profile.clone(),
            def,
        });
    }

    /// Delete a macro by id from the active profile (`DeleteMacro`).
    pub(crate) fn delete_macro(&mut self, id: u16) {
        self.send(ControlMsg::DeleteMacro {
            profile: self.mirror.profile.clone(),
            id,
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

    /// Borrow the editable profile mirror.
    pub(crate) fn mirror_mut(&mut self) -> &mut ProfileMirror {
        &mut self.mirror
    }

    /// Borrow the transient binding-editor state.
    pub(crate) fn binding_editor_mut(&mut self) -> &mut bindings::BindingEditor {
        &mut self.binding_editor
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
