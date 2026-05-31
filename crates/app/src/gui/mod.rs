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

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use eframe::egui;
use engine::config::EngineConfig;
use engine::handoff::TelemetryRx;
use engine::telemetry::TelemetryFrame;
use engine::{ControlMsg, Stick};
use hyperion_core::config::{AutoSwitchConfig, AutoSwitchRule, HidHideConfig, ThreadConfig};
use hyperion_core::map::{GyroSettings, MacroDef, MouseSettings, Profile};
use hyperion_core::output::PadTarget;
use hyperion_core::stick::settings::StickSettings;

mod autoswitch;
mod bindings;
mod gyro;
mod macros;
mod mouse;
mod panels;
mod profiles;
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

/// The top-level tabs (`DESIGN-REMAP.md` §8). M4 shipped **Mapping** / **Sticks** / **Mouse** /
/// **Macros** / **Engine**; M5 adds the **Profiles** manager, the **Auto-switch** rule table, and
/// the **Gyro** → mouse screen. The Mapping..Engine tabs edit the GUI's *active* profile (selected
/// on the Profiles tab); Profiles / Auto-switch are structural; Gyro edits the active profile.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Tab {
    /// Profile manager + device→profile assignment + per-profile output kind (M5).
    Profiles,
    /// Per-control binding editor: bind any control to any [`BindTarget`] + shift + turbo.
    Mapping,
    /// Per-stick settings: RC sub-section + deadzone / sensitivity / curve.
    Sticks,
    /// Mouse-from-stick settings (sensitivity / deadzone / accel / invert).
    Mouse,
    /// Gyro→mouse settings: mode / sensitivity / deadzone / invert / vertical scale (M5).
    Gyro,
    /// Macro editor (add/edit/delete timed macro step lists).
    Macros,
    /// Foreground auto-profile-switch rule table (M5).
    AutoSwitch,
    /// Global engine policy: thread / scheduling + HidHide.
    Engine,
}

impl Tab {
    /// The tab strip, in display order.
    const ALL: [Tab; 8] = [
        Tab::Profiles,
        Tab::Mapping,
        Tab::Sticks,
        Tab::Mouse,
        Tab::Gyro,
        Tab::Macros,
        Tab::AutoSwitch,
        Tab::Engine,
    ];

    /// Display label.
    fn label(self) -> &'static str {
        match self {
            Tab::Profiles => "Profiles",
            Tab::Mapping => "Mapping",
            Tab::Sticks => "Sticks",
            Tab::Mouse => "Mouse",
            Tab::Gyro => "Gyro",
            Tab::Macros => "Macros",
            Tab::AutoSwitch => "Auto-switch",
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
    /// Optimistic mirror of the structural config the M5 screens edit (profile tree, devices,
    /// assignments, auto-switch). Kept in lockstep with the `ControlMsg`s those screens send.
    structure: StructMirror,
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

/// The GUI's optimistic mirror of the **structural** config the M5 screens edit: the full editable
/// profile tree, the device-identity ids, the device→profile assignments, and the auto-switch
/// policy. Seeded once from the runtime's config snapshot and then kept in lockstep with the
/// `ControlMsg`s the Profiles / Auto-switch / Gyro screens send (same optimistic pattern as
/// [`ProfileMirror`] and the macro editor). The engine's single config-writer thread stays
/// authoritative; this mirror exists so the structural screens can render lists and so switching the
/// active profile can re-seed [`ProfileMirror`] without reading the live `ArcSwap`.
pub struct StructMirror {
    /// Editable profile tree (id → full [`Profile`]); the source for re-seeding [`ProfileMirror`].
    pub profiles: BTreeMap<String, Profile>,
    /// Known device ids (hardware identities), sorted, for the assignment + rule tables.
    pub devices: Vec<String>,
    /// Device id → assigned profile id.
    pub assignments: BTreeMap<String, String>,
    /// Auto-profile-switch policy (master enable + the **live** edit buffer of rules; the
    /// Auto-switch table mutates `auto_switch.rules` in place every frame so typed text persists).
    pub auto_switch: AutoSwitchConfig,
    /// The auto-switch rules as the engine's single writer currently holds them (the "committed"
    /// shadow). The table reconciles the live `auto_switch.rules` against this on a commit event,
    /// emitting the minimal tuple-keyed `Upsert`/`Delete` set, then refreshes this shadow. Keeping a
    /// shadow lets the live buffer be edited per keystroke (so in-progress text isn't dropped between
    /// frames) without churning the engine on every character.
    pub auto_switch_committed: Vec<AutoSwitchRule>,
    /// Transient: the "new profile" name buffer on the Profiles screen (never persisted).
    pub new_profile_name: String,
}

impl StructMirror {
    /// Seed the structural mirror from the runtime's config snapshot (a deep clone of the editable
    /// tree; the lists are tiny and this runs once at construction).
    fn from_snapshot(snapshot: &EngineConfig) -> Self {
        Self {
            profiles: (*snapshot.profiles).clone(),
            devices: snapshot.devices.keys().cloned().collect(),
            assignments: snapshot.assignments.clone(),
            auto_switch: snapshot.auto_switch.clone(),
            auto_switch_committed: snapshot.auto_switch.rules.clone(),
            new_profile_name: String::new(),
        }
    }

    /// Profile ids in stable (sorted) order — the `BTreeMap` already iterates sorted.
    fn profile_ids(&self) -> Vec<String> {
        self.profiles.keys().cloned().collect()
    }
}

impl ProfileMirror {
    /// Clone a fresh mirror out of a structural-mirror [`Profile`] (the re-seed path when the active
    /// profile changes). Falls back to an all-passthrough default profile when the id is absent.
    fn from_profile(profile_id: &str, profile: &Profile) -> Self {
        Self {
            profile: profile_id.to_string(),
            ls: profile.ls,
            rs: profile.rs,
            mouse: profile.mouse,
            macros: profile.macros.clone(),
        }
    }
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
        let structure = StructMirror::from_snapshot(&snapshot);

        Self {
            control_tx,
            telemetry,
            latest: TelemetryFrame::default(),
            ls_trail: Vec::with_capacity(TRAIL_LEN),
            rs_trail: Vec::with_capacity(TRAIL_LEN),
            active_device,
            mirror,
            structure,
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
                Tab::Profiles => profiles::profiles_panel(ui, self),
                Tab::Mapping => bindings::mapping_panel(ui, self),
                Tab::Sticks => {
                    sticks::stick_panel(ui, self, Stick::Left);
                    ui.separator();
                    sticks::stick_panel(ui, self, Stick::Right);
                }
                Tab::Mouse => mouse::mouse_panel(ui, self),
                Tab::Gyro => gyro::gyro_panel(ui, self),
                Tab::Macros => macros::macros_panel(ui, self),
                Tab::AutoSwitch => autoswitch::autoswitch_panel(ui, self),
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

// ----- M5 structural edits: the Profiles / Auto-switch / Gyro screens -------------------------
//
// Every mutation updates the optimistic `StructMirror` AND sends a `ControlMsg` to the single
// config-writer thread (never the hot loop). The mirror lets the screens render lists and lets a
// profile switch re-seed `ProfileMirror` (so Mapping/Sticks/Mouse/Macros/Gyro retarget) without
// reading the live `ArcSwap`.

impl HyperionApp {
    // --- Profile tree queries ----------------------------------------------------------------

    /// Profile ids in stable sorted order (for the profile list + the assignment/rule combos).
    pub(crate) fn profile_ids(&self) -> Vec<String> {
        self.structure.profile_ids()
    }

    /// The id of the profile every per-profile tab currently edits (the active edit target).
    pub(crate) fn active_profile(&self) -> &str {
        &self.mirror.profile
    }

    /// Known device ids (for the assignment table + auto-switch device combo).
    pub(crate) fn device_ids(&self) -> Vec<String> {
        self.structure.devices.clone()
    }

    /// The profile id assigned to `device`, if any.
    pub(crate) fn assignment_for(&self, device: &str) -> Option<String> {
        self.structure.assignments.get(device).cloned()
    }

    /// The output kind (X360 / DS4) of profile `id` (defaults to X360 if the id is unknown).
    pub(crate) fn output_kind_for(&self, id: &str) -> PadTarget {
        self.structure
            .profiles
            .get(id)
            .map(|p| p.output_kind)
            .unwrap_or_default()
    }

    /// Borrow the transient "new profile" name buffer (Profiles screen).
    pub(crate) fn profiles_new_name_mut(&mut self) -> &mut String {
        &mut self.structure.new_profile_name
    }

    /// Read the transient "new profile" name buffer.
    pub(crate) fn profiles_new_name(&self) -> &str {
        &self.structure.new_profile_name
    }

    // --- Profile lifecycle edits -------------------------------------------------------------

    /// Create a new all-passthrough profile `id` and send `CreateProfile`. No-op if it exists.
    pub(crate) fn create_profile(&mut self, id: String) {
        if self.structure.profiles.contains_key(&id) {
            return;
        }
        self.structure.profiles.insert(
            id.clone(),
            Profile {
                name: id.clone(),
                ..Profile::default()
            },
        );
        self.send(ControlMsg::CreateProfile { name: id });
    }

    /// Rename profile `from` to `to`, re-pointing any assignments, and send `RenameProfile`. No-op
    /// if `from` is absent or `to` already exists.
    pub(crate) fn rename_profile(&mut self, from: String, to: String) {
        if !self.structure.profiles.contains_key(&from) || self.structure.profiles.contains_key(&to)
        {
            return;
        }
        if let Some(mut p) = self.structure.profiles.remove(&from) {
            p.name = to.clone();
            self.structure.profiles.insert(to.clone(), p);
        }
        for pid in self.structure.assignments.values_mut() {
            if *pid == from {
                *pid = to.clone();
            }
        }
        // Re-point any auto-switch rule that targeted the old id (keeps the mirror coherent; the
        // engine's rename arm does the same on its side).
        for rule in &mut self.structure.auto_switch.rules {
            if rule.profile == from {
                rule.profile = to.clone();
            }
        }
        // If the active edit target was renamed, follow it.
        if self.mirror.profile == from {
            self.mirror.profile = to.clone();
        }
        self.send(ControlMsg::RenameProfile { from, to });
    }

    /// Duplicate profile `src` into a fresh id `dst` and send `DuplicateProfile`. No-op if `src`
    /// is absent or `dst` exists.
    pub(crate) fn duplicate_profile(&mut self, src: String, dst: String) {
        if self.structure.profiles.contains_key(&dst) {
            return;
        }
        if let Some(mut copy) = self.structure.profiles.get(&src).cloned() {
            copy.name = dst.clone();
            self.structure.profiles.insert(dst.clone(), copy);
            self.send(ControlMsg::DuplicateProfile { src, dst });
        }
    }

    /// Delete profile `id` (dropping any assignment that pointed at it) and send `DeleteProfile`.
    /// If the active edit target is deleted, fall back to the first remaining profile.
    pub(crate) fn delete_profile(&mut self, id: String) {
        if self.structure.profiles.remove(&id).is_none() {
            return;
        }
        self.structure.assignments.retain(|_, pid| *pid != id);
        self.send(ControlMsg::DeleteProfile { name: id.clone() });
        // If the active edit target was deleted, fall back to the first remaining profile so the
        // per-profile tabs keep a coherent target.
        let active_deleted = self.mirror.profile == id;
        let next = self.structure.profiles.keys().next().cloned();
        if let (true, Some(next)) = (active_deleted, next) {
            self.select_profile(next);
        }
    }

    /// Set profile `id`'s virtual-pad output kind and send `SetOutputKind`. The engine reads this
    /// at (re)plug time; a runtime change triggers a ViGEm replug.
    pub(crate) fn set_output_kind(&mut self, id: String, kind: PadTarget) {
        if let Some(p) = self.structure.profiles.get_mut(&id) {
            p.output_kind = kind;
        }
        self.send(ControlMsg::SetOutputKind { profile: id, kind });
    }

    /// Assign `profile` to `device` and send `SetAssignment`. If the active device's assignment
    /// changes, retarget the GUI's editable profile so every per-profile tab follows.
    pub(crate) fn set_assignment(&mut self, device: String, profile: String) {
        self.structure
            .assignments
            .insert(device.clone(), profile.clone());
        self.send(ControlMsg::SetAssignment {
            device: device.clone(),
            profile: profile.clone(),
        });
        if device == self.active_device {
            self.select_profile(profile);
        }
    }

    /// Make `id` the GUI's active edit target: re-seed [`ProfileMirror`] from the structural mirror
    /// so the Mapping / Sticks / Mouse / Macros / Gyro tabs all retarget. Local-only (no
    /// `ControlMsg`): which profile the GUI *edits* is a GUI concept; which profile a *device runs*
    /// is the assignment (`set_assignment`).
    pub(crate) fn select_profile(&mut self, id: String) {
        let profile = self
            .structure
            .profiles
            .get(&id)
            .cloned()
            .unwrap_or_default();
        self.mirror = ProfileMirror::from_profile(&id, &profile);
    }

    // --- Gyro settings (active profile) ------------------------------------------------------

    /// The active profile's gyro settings (defaults to inert `Off` if the profile is missing).
    pub(crate) fn gyro_settings(&self) -> GyroSettings {
        self.structure
            .profiles
            .get(&self.mirror.profile)
            .map(|p| p.gyro)
            .unwrap_or_default()
    }

    /// Update the active profile's gyro settings in the structural mirror (call before
    /// [`push_gyro_settings`](Self::push_gyro_settings)).
    pub(crate) fn set_gyro_settings(&mut self, gyro: GyroSettings) {
        if let Some(p) = self.structure.profiles.get_mut(&self.mirror.profile) {
            p.gyro = gyro;
        }
    }

    /// Send the active profile's gyro settings via `SetGyroSettings` (engine clamps on apply).
    pub(crate) fn push_gyro_settings(&mut self) {
        let settings = self.gyro_settings();
        self.send(ControlMsg::SetGyroSettings {
            profile: self.mirror.profile.clone(),
            settings,
        });
    }

    // --- Auto-switch policy ------------------------------------------------------------------

    /// Whether foreground auto-switching is enabled.
    pub(crate) fn autoswitch_enabled(&self) -> bool {
        self.structure.auto_switch.enabled
    }

    /// Toggle foreground auto-switching and send `SetAutoSwitchEnabled`.
    pub(crate) fn set_autoswitch_enabled(&mut self, enabled: bool) {
        self.structure.auto_switch.enabled = enabled;
        self.send(ControlMsg::SetAutoSwitchEnabled(enabled));
    }

    /// Borrow the **live** auto-switch rule edit buffer (the table mutates rows in place every frame
    /// so in-progress typed text persists). Engine sync happens on a commit event via
    /// [`reconcile_autoswitch`](Self::reconcile_autoswitch), not per keystroke.
    pub(crate) fn autoswitch_rules_mut(&mut self) -> &mut Vec<AutoSwitchRule> {
        &mut self.structure.auto_switch.rules
    }

    /// Append a fresh blank rule to the live buffer (the user fills its match keys in, then a
    /// commit reconciles it to the engine). Refuses a second all-blank row, since the engine dedups
    /// by the `("", "", "")` match tuple and two would collapse to one.
    pub(crate) fn add_autoswitch_rule(&mut self) {
        let blank = AutoSwitchRule::default();
        let has_blank =
            self.structure.auto_switch.rules.iter().any(|r| {
                r.device.is_empty() && r.exe_substr.is_empty() && r.title_substr.is_empty()
            });
        if has_blank {
            return;
        }
        self.structure.auto_switch.rules.push(blank);
    }

    /// Remove the rule at `index` from the live buffer and reconcile to the engine immediately (a
    /// delete is an explicit click, not in-progress text, so there is no reason to defer it).
    pub(crate) fn delete_autoswitch_rule(&mut self, index: usize) {
        if index < self.structure.auto_switch.rules.len() {
            self.structure.auto_switch.rules.remove(index);
            self.reconcile_autoswitch();
        }
    }

    /// Reconcile the live auto-switch rule buffer against the engine-committed shadow, emitting the
    /// minimal tuple-keyed message set, then refresh the shadow.
    ///
    /// The engine keys rules by their `(device, exe_substr, title_substr)` match tuple
    /// (`UpsertAutoSwitchRule` re-points an existing tuple's profile or appends a new tuple;
    /// `DeleteAutoSwitchRule` retains-by-tuple — there is no index/order primitive). So, comparing
    /// the **non-blank** live tuples (an all-empty row is an in-progress placeholder the matcher
    /// would treat as inert, so it is never sent) against the committed shadow:
    /// * every committed tuple no longer present live → `DeleteAutoSwitchRule`,
    /// * every live tuple that is new, or whose target profile changed → `UpsertAutoSwitchRule`.
    ///
    /// The live UI buffer is **left intact** (placeholders + in-progress edits persist between
    /// frames); only the committed shadow is rewritten. Called on a row commit (text lost-focus /
    /// combo pick) and on delete.
    pub(crate) fn reconcile_autoswitch(&mut self) {
        let same_tuple = |a: &AutoSwitchRule, b: &AutoSwitchRule| {
            a.device == b.device && a.exe_substr == b.exe_substr && a.title_substr == b.title_substr
        };
        let is_blank = |r: &AutoSwitchRule| {
            r.device.is_empty() && r.exe_substr.is_empty() && r.title_substr.is_empty()
        };

        // The engine-facing view: live rules with a non-blank tuple, deduplicated by tuple (first
        // occurrence wins, matching how the engine's tuple-keyed upsert lands them).
        let mut effective: Vec<AutoSwitchRule> = Vec::new();
        for r in &self.structure.auto_switch.rules {
            if is_blank(r) || effective.iter().any(|e| same_tuple(e, r)) {
                continue;
            }
            effective.push(r.clone());
        }

        // Deletes: committed tuples absent from the effective set.
        let committed = self.structure.auto_switch_committed.clone();
        for old in &committed {
            if !effective.iter().any(|n| same_tuple(n, old)) {
                self.send(ControlMsg::DeleteAutoSwitchRule {
                    device: old.device.clone(),
                    exe_substr: old.exe_substr.clone(),
                    title_substr: old.title_substr.clone(),
                });
            }
        }
        // Upserts: a new tuple, or an existing tuple whose target profile changed.
        for new in &effective {
            let needs_upsert = match committed.iter().find(|o| same_tuple(o, new)) {
                None => true,
                Some(o) => o.profile != new.profile,
            };
            if needs_upsert {
                self.send(ControlMsg::UpsertAutoSwitchRule { rule: new.clone() });
            }
        }

        // The effective set is now exactly what the engine holds; the live buffer is untouched.
        self.structure.auto_switch_committed = effective;
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
