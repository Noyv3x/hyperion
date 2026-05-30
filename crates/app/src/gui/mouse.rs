//! The Mouse-from-stick settings screen (`DESIGN-REMAP.md` §8 mouse_gyro.rs, M4 mouse slice).
//!
//! Exposes the [`hyperion_core::map::MouseSettings`] tunables a `BindTarget::MouseMove` binding
//! consumes via the resolved profile's `MouseAccumulator`: sensitivity, vertical scale, anti-jitter
//! velocity offset, stick dead-zone, the per-report motion threshold, an optional acceleration
//! curve, and per-axis inversion. Gyro→mouse settings land in M5.
//!
//! Every edit funnels through [`super::HyperionApp::push_mouse_settings`], which re-sends the whole
//! `MouseSettings` via `ControlMsg::SetMouseSettings`; the engine clamps on apply.

use eframe::egui;

use super::HyperionApp;

/// The Mouse settings panel body.
pub fn mouse_panel(ui: &mut egui::Ui, app: &mut HyperionApp) {
    ui.heading("Mouse (from stick)");
    ui.label(
        "Settings for any control bound to “Mouse move (from stick)” in the Mapping tab. \
         Values are clamped by the engine on apply.",
    );
    ui.separator();

    let mut m = app.mirror_mut().mouse;
    let mut changed = false;

    egui::Grid::new("mouse-grid")
        .num_columns(2)
        .spacing([12.0, 6.0])
        .show(ui, |ui| {
            changed |= ui
                .add(egui::Slider::new(&mut m.sensitivity, 1.0..=100.0).text("sensitivity"))
                .changed();
            ui.end_row();
            changed |= ui
                .add(egui::Slider::new(&mut m.vertical_scale, 0.1..=4.0).text("vertical scale"))
                .changed();
            ui.end_row();
            changed |= ui
                .add(
                    egui::Slider::new(&mut m.velocity_offset, 0.0..=0.2).text("anti-jitter offset"),
                )
                .changed();
            ui.end_row();
            changed |= ui
                .add(egui::Slider::new(&mut m.deadzone, 0.0..=0.9).text("stick dead-zone"))
                .changed();
            ui.end_row();
            changed |= ui
                .add(
                    egui::Slider::new(&mut m.min_threshold, 1.0..=10.0)
                        .text("motion threshold (px)"),
                )
                .changed();
            ui.end_row();
        });

    ui.add_space(4.0);
    changed |= ui
        .checkbox(&mut m.accelerate, "Acceleration curve")
        .changed();
    ui.add_enabled_ui(m.accelerate, |ui| {
        changed |= ui
            .add(egui::Slider::new(&mut m.accel_power, 0.5..=4.0).text("accel power"))
            .changed();
    });

    ui.add_space(4.0);
    ui.horizontal(|ui| {
        changed |= ui.checkbox(&mut m.invert_x, "Invert X").changed();
        changed |= ui.checkbox(&mut m.invert_y, "Invert Y").changed();
    });

    ui.add_space(4.0);
    ui.label(
        egui::RichText::new(
            "A motion threshold of 1.0 px is the always-carry mode (no sub-pixel gate). \
             Higher values defer tiny motions until they accumulate past the threshold.",
        )
        .weak()
        .italics(),
    );

    if changed {
        app.mirror_mut().mouse = m;
        app.push_mouse_settings();
    }
}
