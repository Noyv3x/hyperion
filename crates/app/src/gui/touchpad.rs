//! The Touchpad settings screen (`DESIGN-REMAP.md` §8, M6 touchpad slice).
//!
//! Exposes the [`hyperion_core::map::profile::TouchpadSettings`] for the **active profile**: the
//! touchpad-as-relative-mouse master enable + its velocity tunables (sensitivity, anti-jitter
//! offset, the per-report motion threshold, the jitter-compensation ease curve, per-axis
//! inversion) and the touch-as-buttons master enable for the finger-region controls
//! (`TouchLeft/Right/Upper/Multi`).
//!
//! Touchpad-as-mouse is consumed by `apply()` when a control is bound to
//! `MouseMove(Touchpad)` in the Mapping tab (the engine feeds the touch
//! [`MouseAccumulator`](hyperion_core::mouse_accum::MouseAccumulator) via `touch_step`); the
//! touch-region **controls** bind like any other control in the Mapping tab, so this screen only
//! carries the per-profile settings — the bindings themselves live on the Mapping screen.
//!
//! Every edit funnels through [`super::HyperionApp::push_touchpad_settings`], which re-sends the
//! whole `TouchpadSettings` via `ControlMsg::SetTouchpadSettings`; the engine clamps on apply
//! (`TouchpadSettings::clamped`), so the GUI may send freely and the mirror keeps the typed value.

use eframe::egui;

use super::HyperionApp;

/// The Touchpad settings panel body. Targets the GUI's active profile (the one the structural
/// mirror currently selects); all per-profile panels share that selection.
pub fn touchpad_panel(ui: &mut egui::Ui, app: &mut HyperionApp) {
    ui.heading("Touchpad");
    ui.label(format!(
        "Touchpad settings for profile “{}”. Bind a control to “Mouse move (from touchpad)” in \
         the Mapping tab to drive the cursor from a finger drag; the touch-region buttons \
         (Touch left / right / upper / multi) bind like any other control there too.",
        app.active_profile()
    ));
    ui.separator();

    let mut t = app.touchpad_settings();
    let mut changed = false;

    // --- Touchpad as mouse ---------------------------------------------------------------------
    ui.group(|ui| {
        ui.label(egui::RichText::new("Touchpad → mouse").strong());
        changed |= ui
            .checkbox(
                &mut t.as_mouse,
                "Drive the mouse from a touchpad finger drag",
            )
            .changed();

        ui.add_enabled_ui(t.as_mouse, |ui| {
            egui::Grid::new("touchpad-grid")
                .num_columns(2)
                .spacing([12.0, 6.0])
                .show(ui, |ui| {
                    changed |= ui
                        .add(egui::Slider::new(&mut t.sensitivity, 0.1..=100.0).text("sensitivity"))
                        .changed();
                    ui.end_row();
                    changed |= ui
                        .add(
                            egui::Slider::new(&mut t.velocity_offset, 0.0..=2.0)
                                .text("anti-jitter offset"),
                        )
                        .changed();
                    ui.end_row();
                    changed |= ui
                        .add(
                            egui::Slider::new(&mut t.min_threshold, 1.0..=10.0)
                                .text("motion threshold (px)"),
                        )
                        .changed();
                    ui.end_row();
                });

            ui.add_space(4.0);
            changed |= ui
                .checkbox(&mut t.jitter_comp, "Jitter compensation (ease-in curve)")
                .changed();

            ui.add_space(4.0);
            ui.horizontal(|ui| {
                changed |= ui.checkbox(&mut t.invert_x, "Invert X").changed();
                changed |= ui.checkbox(&mut t.invert_y, "Invert Y").changed();
            });
        });
    });

    // --- Touchpad as buttons -------------------------------------------------------------------
    ui.add_space(6.0);
    ui.group(|ui| {
        ui.label(egui::RichText::new("Touchpad → buttons").strong());
        changed |= ui
            .checkbox(
                &mut t.as_buttons,
                "Enable the touch-region buttons (left / right / upper / multi)",
            )
            .changed();
        ui.label(
            egui::RichText::new(
                "The touch-region controls are always decoded from the contact position; this \
                 switch gates whether the engine treats them as live. Bind them in the Mapping \
                 tab (under “Touch left / right / upper / multi”).",
            )
            .weak()
            .italics(),
        );
    });

    ui.add_space(4.0);
    ui.label(
        egui::RichText::new(
            "A motion threshold of 1.0 px is the always-carry mode (no sub-pixel gate). Values \
             are clamped by the engine on apply.",
        )
        .weak()
        .italics(),
    );

    if changed {
        app.set_touchpad_settings(t);
        app.push_touchpad_settings();
    }
}
