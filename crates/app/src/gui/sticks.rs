//! The per-stick settings screen (`DESIGN-REMAP.md` §8 sticks.rs, minimal M3 form).
//!
//! Exposes the load-bearing slice of [`hyperion_core::stick::settings::StickSettings`] — radial
//! deadzone / anti-deadzone / max-zone / max-output, sensitivity, and the output-curve combo — plus
//! the existing RC panel reused verbatim as the `rc` sub-section ([`super::panels::rc_controls`]).
//! The full surface (axial deadzone, rotation, fuzz, square-stick, Bezier editor, flick) is fleshed
//! out in later milestones; M3 keeps the panel lean but coherent so the new structure is exercised.
//!
//! Every edit funnels through [`super::HyperionApp::push_stick_settings`], which re-sends the whole
//! `StickSettings` via `ControlMsg::SetStickSettings`; the engine clamps on apply.

use eframe::egui;
use engine::Stick;
use hyperion_core::stick::settings::OutputCurve;

use super::{panels, HyperionApp};

/// One per-stick (Left / Right) settings panel.
pub fn stick_panel(ui: &mut egui::Ui, app: &mut HyperionApp, stick: Stick) {
    let (title, id) = match stick {
        Stick::Left => ("Left stick (LS)", "ls"),
        Stick::Right => ("Right stick (RS)", "rs"),
    };
    ui.heading(title);

    // --- Deadzone / sensitivity / curve (the minimal M3 surface) -------------------------------
    deadzone_and_curve(ui, app, stick, id);

    // --- RC filter sub-section (reused verbatim from the M2 panel) -----------------------------
    egui::CollapsingHeader::new("RC filter")
        .id_salt((id, "rc"))
        .default_open(true)
        .show(ui, |ui| {
            panels::rc_controls(ui, app, stick);
        });
}

/// The radial deadzone + sensitivity + output-curve controls. Any change re-sends the whole
/// [`hyperion_core::stick::settings::StickSettings`].
fn deadzone_and_curve(ui: &mut egui::Ui, app: &mut HyperionApp, stick: Stick, id: &str) {
    let mut s = *app.mirror_mut().stick(stick);
    let mut changed = false;

    ui.label("Deadzone (radial):");
    egui::Grid::new((id, "dz-grid"))
        .num_columns(2)
        .spacing([12.0, 4.0])
        .show(ui, |ui| {
            // dead_zone is in [0,127] axis units (C# convention).
            changed |= ui
                .add(
                    egui::Slider::new(&mut s.dead_zone.dead_zone, 0..=127)
                        .text("dead zone (units)"),
                )
                .changed();
            ui.end_row();
            // anti-deadzone / max-zone / max-output are percentages.
            changed |= ui
                .add(
                    egui::Slider::new(&mut s.dead_zone.anti_dead_zone, 0..=100)
                        .text("anti-deadzone (%)"),
                )
                .changed();
            ui.end_row();
            changed |= ui
                .add(egui::Slider::new(&mut s.dead_zone.max_zone, 1..=100).text("max zone (%)"))
                .changed();
            ui.end_row();
            changed |= ui
                .add(
                    egui::Slider::new(&mut s.dead_zone.max_output, 0.0..=100.0)
                        .text("max output (%)"),
                )
                .changed();
            ui.end_row();
        });

    // Sensitivity is a radial multiplier (C# quirk: radial-only). 1.0 == off.
    changed |= ui
        .add(egui::Slider::new(&mut s.sensitivity, 0.1..=4.0).text("sensitivity"))
        .changed();

    // Output curve combo (every C#-matching discriminant; the heavier Bezier / Apex variants are
    // selectable so a persisted profile round-trips, even though their editors land later).
    let before = s.curve;
    egui::ComboBox::from_id_salt((id, "curve"))
        .selected_text(curve_label(s.curve))
        .show_ui(ui, |ui| {
            for c in [
                OutputCurve::Linear,
                OutputCurve::EnhancedPrecision,
                OutputCurve::Quadratic,
                OutputCurve::Cubic,
                OutputCurve::EaseoutQuad,
                OutputCurve::EaseoutCubic,
                OutputCurve::Bezier,
                OutputCurve::ApexClassicInverse,
                OutputCurve::ApexClassicInverseAxial,
            ] {
                ui.selectable_value(&mut s.curve, c, curve_label(c));
            }
        });
    changed |= s.curve != before;

    if changed {
        *app.mirror_mut().stick_mut(stick) = s;
        app.push_stick_settings(stick);
    }
}

/// Display label for an [`OutputCurve`].
fn curve_label(curve: OutputCurve) -> &'static str {
    match curve {
        OutputCurve::Linear => "Linear",
        OutputCurve::EnhancedPrecision => "Enhanced precision",
        OutputCurve::Quadratic => "Quadratic",
        OutputCurve::Cubic => "Cubic",
        OutputCurve::EaseoutQuad => "Ease-out quadratic",
        OutputCurve::EaseoutCubic => "Ease-out cubic",
        OutputCurve::Bezier => "Bezier (custom)",
        OutputCurve::ApexClassicInverse => "Apex classic inverse",
        OutputCurve::ApexClassicInverseAxial => "Apex classic inverse (axial)",
    }
}
