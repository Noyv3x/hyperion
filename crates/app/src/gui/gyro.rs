//! The Gyro→mouse settings screen (`DESIGN-REMAP.md` §8 mouse_gyro.rs, M5 gyro slice).
//!
//! Exposes the [`hyperion_core::map::profile::GyroSettings`] tunables the engine's `apply()` feeds
//! the gyro [`MouseAccumulator`](hyperion_core::mouse_accum::MouseAccumulator) via `gyro_velocity_step`
//! when the profile's [`GyroMode`] is active (blueprint §12 M5; ground truth
//! `Hyperion-ds4w/.../MouseCursor.cs::sixaxisMoved`): the activation **mode**, master sensitivity,
//! vertical (pitch) scale, the gyro-rate-domain dead-zone, the anti-jitter velocity offset, the
//! per-report motion threshold, the jitter-compensation ease curve, per-axis inversion, and the
//! yaw/roll horizontal-axis swap.
//!
//! Every edit funnels through [`super::HyperionApp::push_gyro_settings`], which re-sends the whole
//! `GyroSettings` for the **active profile** via `ControlMsg::SetGyroSettings`; the engine clamps on
//! apply (`GyroSettings::clamped`), so the GUI may send freely and the mirror keeps the typed value.

use eframe::egui;
use hyperion_core::map::profile::GyroMode;

use super::HyperionApp;

/// The Gyro settings panel body. Targets the GUI's active profile (the one the structural mirror
/// currently selects); all panels share that selection.
pub fn gyro_panel(ui: &mut egui::Ui, app: &mut HyperionApp) {
    ui.heading("Gyro → mouse");
    ui.label(format!(
        "Gyro aiming for profile “{}”. Bind a control to “Gyro X/Z” in the Mapping tab to feed \
         the gyro; values are clamped by the engine on apply.",
        app.active_profile()
    ));
    ui.separator();

    let mut g = app.gyro_settings();
    let mut changed = false;

    // --- Activation mode -----------------------------------------------------------------------
    ui.horizontal(|ui| {
        ui.label("Activation:");
        let before = g.mode;
        egui::ComboBox::from_id_salt("gyro-mode")
            .selected_text(mode_label(g.mode))
            .show_ui(ui, |ui| {
                for m in [GyroMode::Off, GyroMode::AlwaysOn, GyroMode::TriggerHeld] {
                    ui.selectable_value(&mut g.mode, m, mode_label(m));
                }
            });
        changed |= g.mode != before;
    });
    if g.mode == GyroMode::TriggerHeld {
        ui.label(
            egui::RichText::new(
                "Gyro runs only while its activation trigger is held (configure the trigger in \
                 the Mapping tab).",
            )
            .weak()
            .italics(),
        );
    }

    // The velocity-model controls are only meaningful when gyro→mouse is enabled.
    let active = !matches!(g.mode, GyroMode::Off | GyroMode::Unknown);
    ui.add_enabled_ui(active, |ui| {
        egui::Grid::new("gyro-grid")
            .num_columns(2)
            .spacing([12.0, 6.0])
            .show(ui, |ui| {
                changed |= ui
                    .add(egui::Slider::new(&mut g.sensitivity, 0.1..=100.0).text("sensitivity"))
                    .changed();
                ui.end_row();
                changed |= ui
                    .add(
                        egui::Slider::new(&mut g.vertical_scale, 0.1..=10.0).text("vertical scale"),
                    )
                    .changed();
                ui.end_row();
                changed |= ui
                    .add(
                        egui::Slider::new(&mut g.deadzone, 0.0..=200.0)
                            .text("dead-zone (gyro rate)"),
                    )
                    .changed();
                ui.end_row();
                changed |= ui
                    .add(
                        egui::Slider::new(&mut g.velocity_offset, 0.0..=2.0)
                            .text("anti-jitter offset"),
                    )
                    .changed();
                ui.end_row();
                changed |= ui
                    .add(
                        egui::Slider::new(&mut g.min_threshold, 1.0..=10.0)
                            .text("motion threshold (px)"),
                    )
                    .changed();
                ui.end_row();
            });

        ui.add_space(4.0);
        changed |= ui
            .checkbox(&mut g.jitter_comp, "Jitter compensation (ease-in curve)")
            .changed();

        ui.add_space(4.0);
        ui.horizontal(|ui| {
            changed |= ui.checkbox(&mut g.invert_x, "Invert X").changed();
            changed |= ui.checkbox(&mut g.invert_y, "Invert Y").changed();
        });
        changed |= ui
            .checkbox(
                &mut g.swap_yaw_roll,
                "Use roll (not yaw) for horizontal aim",
            )
            .changed();
    });

    ui.add_space(4.0);
    ui.label(
        egui::RichText::new(
            "The dead-zone is in the gyro rate domain (not a normalized stick dead-zone): small \
             tilts below it are suppressed. A motion threshold of 1.0 px is the always-carry mode.",
        )
        .weak()
        .italics(),
    );

    if changed {
        app.set_gyro_settings(g);
        app.push_gyro_settings();
    }
}

/// Display label for a [`GyroMode`].
fn mode_label(mode: GyroMode) -> &'static str {
    match mode {
        GyroMode::Off => "Off",
        GyroMode::AlwaysOn => "Always on",
        GyroMode::TriggerHeld => "While trigger held",
        GyroMode::Unknown => "(unknown)",
    }
}
