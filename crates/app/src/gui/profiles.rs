//! The Profiles screen (`DESIGN-REMAP.md` §8 profiles.rs, M5): the profile manager + the
//! device→profile assignment table + the per-profile virtual-pad output-kind toggle.
//!
//! Profiles are the named editable [`Profile`](hyperion_core::map::Profile) entries in
//! `EngineConfig::profiles`; this screen creates / renames / duplicates / deletes them and assigns
//! a profile to each known device (`EngineConfig::assignments`). It also picks the **active** profile
//! the rest of the GUI edits — selecting a profile here re-seeds [`super::ProfileMirror`] so the
//! Mapping / Sticks / Mouse / Macros / Gyro panels all retarget the chosen profile.
//!
//! The screen is optimistic: it mutates the GUI's structural mirror in lockstep with the
//! `ControlMsg` it sends (`CreateProfile` / `RenameProfile` / `DuplicateProfile` / `DeleteProfile` /
//! `SetAssignment` / `SetOutputKind`), exactly like the Macros editor mutates the profile's macro
//! `Vec` and sends `UpsertMacro`. The engine's single config-writer thread remains authoritative;
//! an edit that the writer rejects (e.g. a name collision) is a harmless no-op there and the next
//! structural reseed (a manual reload) would reconcile it.

use eframe::egui;
use hyperion_core::output::PadTarget;

use super::HyperionApp;

/// The Profiles screen body.
pub fn profiles_panel(ui: &mut egui::Ui, app: &mut HyperionApp) {
    ui.heading("Profiles");
    ui.label(
        "Create, rename, duplicate, or delete profiles; pick the active profile every other tab \
         edits; assign a profile to each device; and choose each profile's virtual-pad output.",
    );
    ui.separator();

    profile_list_ui(ui, app);
    ui.add_space(8.0);
    ui.separator();
    assignments_ui(ui, app);
    ui.add_space(8.0);
    ui.separator();
    import_export_ui(ui, app);
}

/// The profile import/export section (M6): an Export button dumps the active profile to a TOML
/// text area via `core::config::export_profile`; an Import button parses the text area into a new
/// profile under a chosen id (`ControlMsg::ImportProfile`). A plain text area keeps the dependency
/// surface tiny — no file dialog crate; the user pastes/copies the TOML (or saves the whole config
/// via the header's Save button).
fn import_export_ui(ui: &mut egui::Ui, app: &mut HyperionApp) {
    ui.heading("Import / export");
    ui.label(
        egui::RichText::new(
            "Export copies the active profile to the box below as shareable TOML. To import, \
             paste a profile's TOML, set a destination id, and click Import.",
        )
        .weak()
        .italics(),
    );

    let active = app.active_profile().to_string();
    ui.horizontal(|ui| {
        if ui.button("⬆ Export active").clicked() {
            let toml = app.export_active_profile();
            app.set_import_export_toml(toml);
            // Default the import id to a non-colliding "<active> copy" so a round-trip import does
            // not clobber the source.
            if app.import_name().trim().is_empty() {
                *app.import_name_mut() = format!("{active} copy");
            }
        }
        ui.label(format!("(active profile: {active})"));
    });

    ui.add_space(4.0);
    ui.add(
        egui::TextEdit::multiline(app.import_export_toml_mut())
            .desired_rows(10)
            .desired_width(f32::INFINITY)
            .code_editor()
            .hint_text("# paste a profile's TOML here to import, or click Export to fill it"),
    );

    ui.add_space(4.0);
    ui.horizontal(|ui| {
        ui.label("Import as id:");
        ui.add(
            egui::TextEdit::singleline(app.import_name_mut())
                .hint_text("destination profile id")
                .desired_width(180.0),
        );
        let can_import =
            !app.import_name().trim().is_empty() && !app.import_export_toml().trim().is_empty();
        if ui
            .add_enabled(can_import, egui::Button::new("⬇ Import"))
            .clicked()
        {
            let name = app.import_name().trim().to_string();
            let toml = app.import_export_toml().to_string();
            app.import_profile_toml(name, toml);
        }
    });
    ui.label(
        egui::RichText::new(
            "Import overwrites a profile with the same id. Partial / slightly-stale TOML still \
             loads (missing keys take defaults); only structurally invalid TOML is ignored.",
        )
        .weak()
        .italics(),
    );
}

/// The profile list: a selectable row per profile (active = the GUI's edit target) with a rename
/// field, an output-kind combo, and duplicate / delete actions, plus a "new profile" toolbar.
fn profile_list_ui(ui: &mut egui::Ui, app: &mut HyperionApp) {
    // --- New-profile toolbar -------------------------------------------------------------------
    ui.horizontal(|ui| {
        let name = app.profiles_new_name_mut();
        let resp = ui.add(
            egui::TextEdit::singleline(name)
                .hint_text("new profile id")
                .desired_width(180.0),
        );
        let submit = resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
        let add = ui.button("➕ Create").clicked();
        if (add || submit) && !app.profiles_new_name().trim().is_empty() {
            let id = app.profiles_new_name().trim().to_string();
            app.create_profile(id);
            app.profiles_new_name_mut().clear();
        }
    });
    ui.add_space(6.0);

    // Snapshot the ids + the active id so the per-row UI can borrow `app` mutably without holding
    // an iterator over the structural mirror. Edits are applied immediately (the lists are tiny).
    let ids = app.profile_ids();
    let active = app.active_profile().to_string();

    if ids.is_empty() {
        ui.label(
            egui::RichText::new("No profiles yet — create one above.")
                .weak()
                .italics(),
        );
        return;
    }

    let mut select: Option<String> = None;
    let mut duplicate: Option<String> = None;
    let mut delete: Option<String> = None;
    let only_one = ids.len() == 1;

    egui::Grid::new("profiles-grid")
        .num_columns(4)
        .spacing([10.0, 6.0])
        .striped(true)
        .show(ui, |ui| {
            ui.label(egui::RichText::new("active").strong());
            ui.label(egui::RichText::new("profile").strong());
            ui.label(egui::RichText::new("output").strong());
            ui.label(egui::RichText::new("actions").strong());
            ui.end_row();

            for id in &ids {
                let is_active = *id == active;

                // Active selector (radio-style): clicking a non-active row retargets the GUI.
                if ui.radio(is_active, "").clicked() && !is_active {
                    select = Some(id.clone());
                }

                // Rename field: editing in place renames the profile (and re-points assignments).
                let mut edit = id.clone();
                let resp = ui.add(egui::TextEdit::singleline(&mut edit).desired_width(160.0));
                if resp.lost_focus() && edit.trim() != id.as_str() && !edit.trim().is_empty() {
                    app.rename_profile(id.clone(), edit.trim().to_string());
                }

                // Output-kind combo (X360 / DS4): read at (re)plug time by the engine.
                let mut kind = app.output_kind_for(id);
                let before = kind;
                egui::ComboBox::from_id_salt(("profile-output", id))
                    .selected_text(output_label(kind))
                    .show_ui(ui, |ui| {
                        for k in [PadTarget::X360, PadTarget::Ds4] {
                            ui.selectable_value(&mut kind, k, output_label(k));
                        }
                    });
                if kind != before {
                    app.set_output_kind(id.clone(), kind);
                }

                ui.horizontal(|ui| {
                    if ui.button("Duplicate").clicked() {
                        duplicate = Some(id.clone());
                    }
                    // Never allow deleting the last profile (keeps a coherent edit target).
                    if ui
                        .add_enabled(!only_one, egui::Button::new("🗑 Delete"))
                        .clicked()
                    {
                        delete = Some(id.clone());
                    }
                });
                ui.end_row();
            }
        });

    // Apply the row actions after the loop (only one fires per frame in practice).
    if let Some(id) = select {
        app.select_profile(id);
    }
    if let Some(src) = duplicate {
        let dst = unique_copy_name(&ids, &src);
        app.duplicate_profile(src, dst);
    }
    if let Some(id) = delete {
        app.delete_profile(id);
    }
}

/// The device→profile assignment table: one row per known device with a profile dropdown.
fn assignments_ui(ui: &mut egui::Ui, app: &mut HyperionApp) {
    ui.heading("Device assignments");
    let devices = app.device_ids();
    if devices.is_empty() {
        ui.label(
            egui::RichText::new("No devices configured.")
                .weak()
                .italics(),
        );
        return;
    }
    let ids = app.profile_ids();

    egui::Grid::new("assignments-grid")
        .num_columns(2)
        .spacing([10.0, 6.0])
        .striped(true)
        .show(ui, |ui| {
            ui.label(egui::RichText::new("device").strong());
            ui.label(egui::RichText::new("profile").strong());
            ui.end_row();

            for dev in &devices {
                ui.label(dev);
                let current = app.assignment_for(dev).unwrap_or_default();
                let mut chosen = current.clone();
                egui::ComboBox::from_id_salt(("assign", dev))
                    .selected_text(if chosen.is_empty() {
                        "(unassigned)".to_string()
                    } else {
                        chosen.clone()
                    })
                    .show_ui(ui, |ui| {
                        for pid in &ids {
                            ui.selectable_value(&mut chosen, pid.clone(), pid);
                        }
                    });
                if chosen != current && !chosen.is_empty() {
                    app.set_assignment(dev.clone(), chosen);
                }
                ui.end_row();
            }
        });
}

/// Display label for a [`PadTarget`].
fn output_label(kind: PadTarget) -> &'static str {
    match kind {
        PadTarget::X360 => "Xbox 360",
        PadTarget::Ds4 => "DualShock 4",
    }
}

/// Pick the first free `"<src> copy"`, `"<src> copy 2"`, … id not already present.
fn unique_copy_name(ids: &[String], src: &str) -> String {
    let base = format!("{src} copy");
    if !ids.iter().any(|p| p == &base) {
        return base;
    }
    let mut n = 2u32;
    loop {
        let candidate = format!("{base} {n}");
        if !ids.iter().any(|p| p == &candidate) {
            return candidate;
        }
        n += 1;
    }
}
