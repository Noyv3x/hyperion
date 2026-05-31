//! The Triggers screen (`DESIGN-REMAP.md` §8, M6 two-stage / hip-fire slice).
//!
//! Edits the [`hyperion_core::trigger::TriggerSettings`] for the active profile's L2 and R2: the
//! full §4 analog chain (dead-zone, anti-dead-zone, max-zone, max-output, sensitivity, output
//! curve) plus the digital button threshold and the **M6** two-stage / hip-fire mode
//! ([`TriggerMode`]) with its soft-pull threshold and hip-fire window.
//!
//! * **Normal** (default) — a single digital stage at `max(button_threshold, dead_zone)`, soft ==
//!   full. Byte-identical to M5.
//! * **Two-stage** — a soft-pull stage at the soft threshold plus an independent full-pull stage
//!   at the raw `255` full pull; bind the soft and full pulls (`L2 (analog)`/`L2 full pull`) to
//!   different actions in the Mapping tab.
//! * **Hip-fire** — time-gated: a fast full pull within the window fires only the full stage; a
//!   held soft pull fires the soft stage once the window elapses.
//!
//! Every edit funnels through [`super::HyperionApp::push_trigger_settings`], which re-sends the
//! whole `TriggerSettings` via `ControlMsg::SetTriggerSettings`; the engine clamps on apply
//! (`TriggerSettings::clamped`), so the GUI may send freely and the mirror keeps the typed value.

use eframe::egui;
use engine::Trigger;
use hyperion_core::trigger::{TriggerCurve, TriggerMode};

use super::HyperionApp;

/// The Triggers screen body: an L2 panel and an R2 panel stacked vertically.
pub fn triggers_panel(ui: &mut egui::Ui, app: &mut HyperionApp) {
    ui.heading("Triggers");
    ui.label(format!(
        "Two-stage / hip-fire and the analog chain for profile “{}”. Values are clamped by the \
         engine on apply.",
        app.active_profile()
    ));
    ui.separator();

    trigger_panel(ui, app, Trigger::Left);
    ui.add_space(8.0);
    ui.separator();
    trigger_panel(ui, app, Trigger::Right);
}

/// One trigger's settings panel (L2 or R2).
fn trigger_panel(ui: &mut egui::Ui, app: &mut HyperionApp, trigger: Trigger) {
    let title = match trigger {
        Trigger::Left => "Left trigger (L2)",
        Trigger::Right => "Right trigger (R2)",
    };
    ui.heading(title);

    let mut t = app.trigger_settings(trigger);
    let mut changed = false;

    // --- Mode (M6: Normal / TwoStage / HipFire) ------------------------------------------------
    ui.horizontal(|ui| {
        ui.label("Mode:");
        let before = t.mode;
        egui::ComboBox::from_id_salt(("trigger-mode", title))
            .selected_text(mode_label(t.mode))
            .show_ui(ui, |ui| {
                for m in [
                    TriggerMode::Normal,
                    TriggerMode::TwoStage,
                    TriggerMode::HipFire,
                ] {
                    ui.selectable_value(&mut t.mode, m, mode_label(m));
                }
            });
        changed |= t.mode != before;
    });
    ui.label(egui::RichText::new(mode_hint(t.mode)).weak().italics());

    // Soft-pull threshold + hip-fire window are only meaningful for the staged modes.
    let staged = matches!(t.mode, TriggerMode::TwoStage | TriggerMode::HipFire);
    ui.add_enabled_ui(staged, |ui| {
        egui::Grid::new(("trigger-staged-grid", title))
            .num_columns(2)
            .spacing([12.0, 6.0])
            .show(ui, |ui| {
                changed |= ui
                    .add(
                        egui::Slider::new(&mut t.soft_threshold, 0..=255)
                            .text("soft-pull threshold (0 = button threshold)"),
                    )
                    .changed();
                ui.end_row();
                if t.mode == TriggerMode::HipFire {
                    // Edit the hip-fire window in milliseconds (the persisted unit is µs).
                    let mut hip_ms = (t.hip_fire_us as f32 / 1000.0).round() as u32;
                    let resp = ui
                        .add(egui::Slider::new(&mut hip_ms, 1..=1000).text("hip-fire window (ms)"));
                    if resp.changed() {
                        t.hip_fire_us = hip_ms.saturating_mul(1000).max(1000);
                        changed = true;
                    }
                    ui.end_row();
                }
            });
    });

    // --- Digital button threshold --------------------------------------------------------------
    ui.add_space(4.0);
    egui::Grid::new(("trigger-digital-grid", title))
        .num_columns(2)
        .spacing([12.0, 6.0])
        .show(ui, |ui| {
            changed |= ui
                .add(
                    egui::Slider::new(&mut t.button_threshold, 0..=255)
                        .text("button threshold (trigger-as-button)"),
                )
                .changed();
            ui.end_row();
        });

    // --- Analog chain (§4) ---------------------------------------------------------------------
    ui.add_space(4.0);
    ui.collapsing("Analog chain", |ui| {
        egui::Grid::new(("trigger-analog-grid", title))
            .num_columns(2)
            .spacing([12.0, 6.0])
            .show(ui, |ui| {
                changed |= ui
                    .add(egui::Slider::new(&mut t.dead_zone, 0..=255).text("dead-zone (raw)"))
                    .changed();
                ui.end_row();
                changed |= ui
                    .add(
                        egui::Slider::new(&mut t.anti_dead_zone, 0..=100)
                            .text("anti-dead-zone (%)"),
                    )
                    .changed();
                ui.end_row();
                changed |= ui
                    .add(egui::Slider::new(&mut t.max_zone, 1..=100).text("max-zone (%)"))
                    .changed();
                ui.end_row();
                changed |= ui
                    .add(egui::Slider::new(&mut t.max_output, 0.0..=100.0).text("max-output (%)"))
                    .changed();
                ui.end_row();
                changed |= ui
                    .add(egui::Slider::new(&mut t.sensitivity, 0.1..=4.0).text("sensitivity"))
                    .changed();
                ui.end_row();
            });

        ui.horizontal(|ui| {
            ui.label("Output curve:");
            let before = t.curve;
            egui::ComboBox::from_id_salt(("trigger-curve", title))
                .selected_text(curve_label(t.curve))
                .show_ui(ui, |ui| {
                    for c in CURVES {
                        ui.selectable_value(&mut t.curve, c, curve_label(c));
                    }
                });
            changed |= t.curve != before;
        });
    });

    if changed {
        app.set_trigger_settings(trigger, t);
        app.push_trigger_settings(trigger);
    }
}

/// Display label for a [`TriggerMode`].
fn mode_label(mode: TriggerMode) -> &'static str {
    match mode {
        TriggerMode::Normal => "Normal (single stage)",
        TriggerMode::TwoStage => "Two-stage (soft + full)",
        TriggerMode::HipFire => "Hip-fire (timed soft/full)",
        TriggerMode::Unknown => "(unknown)",
    }
}

/// A one-line hint describing the selected mode.
fn mode_hint(mode: TriggerMode) -> &'static str {
    match mode {
        TriggerMode::Normal => {
            "A single digital stage at the button threshold (soft == full). Identical to M5."
        }
        TriggerMode::TwoStage => {
            "Soft pull fires at the soft threshold; full pull fires at a complete (255) pull. Bind \
             the two stages separately in the Mapping tab."
        }
        TriggerMode::HipFire => {
            "A fast full pull within the window fires only the full stage; a held soft pull fires \
             the soft stage once the window elapses."
        }
        TriggerMode::Unknown => "Unknown mode — the engine treats it as Normal.",
    }
}

/// The output curves offered in the editor (the full [`TriggerCurve`] set).
const CURVES: [TriggerCurve; 9] = [
    TriggerCurve::Linear,
    TriggerCurve::EnhancedPrecision,
    TriggerCurve::Quadratic,
    TriggerCurve::Cubic,
    TriggerCurve::EaseoutQuad,
    TriggerCurve::EaseoutCubic,
    TriggerCurve::Bezier,
    TriggerCurve::ApexClassicInverse,
    TriggerCurve::ApexClassicInverseAxial,
];

/// Display label for a [`TriggerCurve`].
fn curve_label(curve: TriggerCurve) -> &'static str {
    match curve {
        TriggerCurve::Linear => "Linear",
        TriggerCurve::EnhancedPrecision => "Enhanced precision",
        TriggerCurve::Quadratic => "Quadratic",
        TriggerCurve::Cubic => "Cubic",
        TriggerCurve::EaseoutQuad => "Ease-out quadratic",
        TriggerCurve::EaseoutCubic => "Ease-out cubic",
        TriggerCurve::Bezier => "Bezier",
        TriggerCurve::ApexClassicInverse => "Apex classic inverse",
        TriggerCurve::ApexClassicInverseAxial => "Apex classic inverse (axial)",
    }
}
