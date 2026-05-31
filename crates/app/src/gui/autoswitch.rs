//! The auto-profile-switch rule table (`DESIGN-REMAP.md` §7.4, §8, §12 M5).
//!
//! Edits the per-device foreground→profile rules the Windows `ForegroundWatcher` matches (off the
//! hot path, ~4 Hz) to flip the active profile: a rule matches when its non-empty `exe_substr` /
//! `title_substr` are ASCII-case-insensitive substrings of the foreground window's executable path /
//! title (first match wins; an all-empty rule is inert). The matcher is the pure
//! [`hyperion_core::autoswitch::match_rules`]; this screen only edits the
//! [`AutoSwitchConfig`](hyperion_core::config::AutoSwitchConfig).
//!
//! The screen is optimistic: it edits the GUI's structural-mirror auto-switch buffer in place (so
//! in-progress typed text persists between frames) and reconciles it to the engine on a commit event
//! (a text field losing focus, or a combo pick / delete). The master toggle sends
//! `SetAutoSwitchEnabled`; the rule reconcile ([`super::HyperionApp::reconcile_autoswitch`]) emits
//! the minimal `UpsertAutoSwitchRule` / `DeleteAutoSwitchRule` set. The engine keys each rule by its
//! `(device, exe_substr, title_substr)` **match tuple** (upsert re-points an existing tuple's profile
//! or appends a new tuple; delete retains-by-tuple). There is no index/order mutation primitive in
//! that contract, so the table has no reorder control: rules are evaluated in creation order (first
//! match wins).

use eframe::egui;

use super::HyperionApp;

/// The Auto-switch screen body.
pub fn autoswitch_panel(ui: &mut egui::Ui, app: &mut HyperionApp) {
    ui.heading("Auto profile switch");
    ui.label(
        "Switch the active profile automatically when a game takes the foreground. Rules are \
         evaluated in creation order (first match wins); a blank exe and title is ignored. Matching \
         runs off the hot loop (~4 Hz), never per report.",
    );
    ui.separator();

    // --- Master enable -------------------------------------------------------------------------
    let mut enabled = app.autoswitch_enabled();
    if ui
        .checkbox(&mut enabled, "Enable foreground auto-switching")
        .changed()
    {
        app.set_autoswitch_enabled(enabled);
    }
    ui.add_space(6.0);

    // --- Add-rule toolbar ----------------------------------------------------------------------
    ui.horizontal(|ui| {
        if ui.button("➕ Add rule").clicked() {
            app.add_autoswitch_rule();
        }
        ui.label(
            egui::RichText::new("Fill in a rule's match keys before adding another.")
                .weak()
                .italics(),
        );
    });
    ui.separator();

    rules_table(ui, app, enabled);
}

/// The editable rule table. Each row: device combo (blank = any), exe substring, title substring,
/// target-profile combo, plus a delete action. Disabled (dimmed) when auto-switch is off.
///
/// The table edits the live rule buffer **in place** (so typed text persists between frames) and
/// only syncs to the engine on a commit event — a text field losing focus, or a combo selection —
/// via [`HyperionApp::reconcile_autoswitch`]. A row delete reconciles immediately.
fn rules_table(ui: &mut egui::Ui, app: &mut HyperionApp, enabled: bool) {
    if app.autoswitch_rules_mut().is_empty() {
        ui.label(
            egui::RichText::new("No rules yet — add one above.")
                .weak()
                .italics(),
        );
        return;
    }

    // Snapshot the combo option lists (owned) so the row loop can hold a `&mut` on the rule buffer.
    let device_ids = app.device_ids();
    let profile_ids = app.profile_ids();

    let mut commit = false;
    let mut delete: Option<usize> = None;

    ui.add_enabled_ui(enabled, |ui| {
        let rules = app.autoswitch_rules_mut();
        egui::Grid::new("autoswitch-grid")
            .num_columns(5)
            .spacing([10.0, 6.0])
            .striped(true)
            .show(ui, |ui| {
                ui.label(egui::RichText::new("device").strong());
                ui.label(egui::RichText::new("exe contains").strong());
                ui.label(egui::RichText::new("title contains").strong());
                ui.label(egui::RichText::new("→ profile").strong());
                ui.label("");
                ui.end_row();

                for (i, r) in rules.iter_mut().enumerate() {
                    // Device combo: empty string == "any device" (commits on selection).
                    egui::ComboBox::from_id_salt(("as-device", i))
                        .selected_text(if r.device.is_empty() {
                            "(any)".to_string()
                        } else {
                            r.device.clone()
                        })
                        .show_ui(ui, |ui| {
                            if ui
                                .selectable_value(&mut r.device, String::new(), "(any)")
                                .changed()
                            {
                                commit = true;
                            }
                            for d in &device_ids {
                                if ui.selectable_value(&mut r.device, d.clone(), d).changed() {
                                    commit = true;
                                }
                            }
                        });

                    // Match-key text fields persist into the buffer each frame; they only *commit*
                    // to the engine on lost-focus (a tuple change is a delete+upsert, so committing
                    // per keystroke would churn the engine list).
                    let exe_resp = ui.add(
                        egui::TextEdit::singleline(&mut r.exe_substr)
                            .hint_text("game.exe")
                            .desired_width(140.0),
                    );
                    let title_resp = ui.add(
                        egui::TextEdit::singleline(&mut r.title_substr)
                            .hint_text("(any)")
                            .desired_width(140.0),
                    );
                    if exe_resp.lost_focus() || title_resp.lost_focus() {
                        commit = true;
                    }

                    // Target profile combo (the only payload an existing tuple re-points).
                    egui::ComboBox::from_id_salt(("as-profile", i))
                        .selected_text(if r.profile.is_empty() {
                            "(pick)".to_string()
                        } else {
                            r.profile.clone()
                        })
                        .show_ui(ui, |ui| {
                            for p in &profile_ids {
                                if ui.selectable_value(&mut r.profile, p.clone(), p).changed() {
                                    commit = true;
                                }
                            }
                        });

                    if ui.button("✖").clicked() {
                        delete = Some(i);
                    }
                    ui.end_row();
                }
            });
    });

    // Apply pending mutations after the `&mut` buffer borrow ends. A delete reconciles itself.
    if let Some(i) = delete {
        app.delete_autoswitch_rule(i);
    } else if commit {
        app.reconcile_autoswitch();
    }
}
