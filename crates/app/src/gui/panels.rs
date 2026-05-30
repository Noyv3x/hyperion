//! The tuning panels: per-stick RC controls, the dynamic-curve editor, and the global
//! thread / HidHide policy.
//!
//! Every widget reads from the app's local mirror and, on change, pushes the rebuilt value to
//! the engine through a `ControlMsg` (`super::HyperionApp::set_stick_mode` / `push_rc` /
//! `push_thread` / `push_hidhide`). Nothing here touches the shared config `ArcSwap` directly —
//! the engine's single writer validates and clamps every edit.

use eframe::egui;
use engine::Stick;
use hyperion_core::config::{DtSource, StickMode, WaitMode};
use hyperion_core::rc::{RcMode, MAX_PARAM, MAX_PERIOD_US, MAX_SPEED, MIN_PARAM, MIN_PERIOD_US};

use super::HyperionApp;

/// One per-stick (Left / Right) tuning panel.
pub fn stick_panel(ui: &mut egui::Ui, app: &mut HyperionApp, stick: Stick) {
    let title = match stick {
        Stick::Left => "Left stick (LS)",
        Stick::Right => "Right stick (RS)",
    };
    ui.heading(title);

    // --- StickMode combo: None | Rc -----------------------------------------------------------
    let mut mode = app.mirror_mut().stick(stick).mode;
    egui::ComboBox::from_id_salt((title, "mode"))
        .selected_text(mode_label(mode))
        .show_ui(ui, |ui| {
            ui.selectable_value(&mut mode, StickMode::None, mode_label(StickMode::None));
            ui.selectable_value(&mut mode, StickMode::Rc, mode_label(StickMode::Rc));
        });
    if mode != app.mirror_mut().stick(stick).mode {
        app.set_stick_mode(stick, mode);
    }

    // The RC parameter controls are only meaningful when the stick runs the filter.
    let editing_rc = mode == StickMode::Rc;
    ui.add_enabled_ui(editing_rc, |ui| {
        rc_controls(ui, app, stick);
    });
}

/// The RC-filter parameter controls for one stick. Any change re-sends the whole `RcConfig`.
fn rc_controls(ui: &mut egui::Ui, app: &mut HyperionApp, stick: Stick) {
    let id = match stick {
        Stick::Left => "ls",
        Stick::Right => "rs",
    };
    // Work on a local copy of the mirror's RC so the borrow of `app` is short; write back +
    // notify only if something changed.
    let mut rc = app.mirror_mut().stick(stick).rc;
    let mut changed = false;

    changed |= ui.checkbox(&mut rc.enabled, "Filter enabled").changed();

    // Algorithm combo: FireBirdInteger | UltimateLegacy | UltimateDt.
    let before = rc.mode;
    egui::ComboBox::from_id_salt((id, "algo"))
        .selected_text(algo_label(rc.mode))
        .show_ui(ui, |ui| {
            for m in [
                RcMode::FireBirdInteger,
                RcMode::UltimateLegacy,
                RcMode::UltimateDt,
            ] {
                ui.selectable_value(&mut rc.mode, m, algo_label(m));
            }
        });
    changed |= rc.mode != before;

    // Fixed / dynamic param source.
    changed |= ui
        .checkbox(&mut rc.use_dynamic_curve, "Dynamic curve (vs. fixed param)")
        .changed();

    // Period slider [MIN_PERIOD_US, MAX_PERIOD_US] microseconds.
    changed |= ui
        .add(
            egui::Slider::new(&mut rc.period_us, MIN_PERIOD_US..=MAX_PERIOD_US).text("period (us)"),
        )
        .changed();

    // Fixed param slider [MIN_PARAM, MAX_PARAM]; disabled in dynamic mode.
    ui.add_enabled_ui(!rc.use_dynamic_curve, |ui| {
        changed |= ui
            .add(egui::Slider::new(&mut rc.fixed_param, MIN_PARAM..=MAX_PARAM).text("fixed param"))
            .changed();
    });

    // The dynamic-curve editor (breakpoints y0 / x1,y1 / x2,y2 / y3).
    ui.add_enabled_ui(rc.use_dynamic_curve, |ui| {
        changed |= curve_editor(ui, id, &mut rc.curve);
    });

    if changed {
        app.mirror_mut().stick_mut(stick).rc = rc;
        app.push_rc(stick);
    }
}

/// The 4-point dynamic curve editor: sliders for `y0`, `x1`, `y1`, `x2`, `y2`, `y3` plus a live
/// preview of the resulting `param`-vs-`speed` curve. Returns whether any breakpoint changed.
fn curve_editor(ui: &mut egui::Ui, id: &str, curve: &mut hyperion_core::rc::RcCurve) -> bool {
    let mut changed = false;
    ui.label("Dynamic curve breakpoints (param vs. speed):");

    egui::Grid::new((id, "curve-grid"))
        .num_columns(2)
        .spacing([12.0, 4.0])
        .show(ui, |ui| {
            changed |= param_slider(ui, "y0 (param @ speed 0)", &mut curve.y0);
            ui.end_row();
            changed |= speed_slider(ui, "x1 (first breakpoint speed)", &mut curve.x1);
            ui.end_row();
            changed |= param_slider(ui, "y1 (param @ x1)", &mut curve.y1);
            ui.end_row();
            changed |= speed_slider(ui, "x2 (second breakpoint speed)", &mut curve.x2);
            ui.end_row();
            changed |= param_slider(ui, "y2 (param @ x2)", &mut curve.y2);
            ui.end_row();
            changed |= param_slider(ui, "y3 (param @ max speed)", &mut curve.y3);
            ui.end_row();
        });

    curve_preview(ui, id, curve);
    changed
}

/// A `param`-range slider returning whether it changed.
fn param_slider(ui: &mut egui::Ui, label: &str, value: &mut i32) -> bool {
    let r = ui.add(egui::Slider::new(value, MIN_PARAM..=MAX_PARAM).text(label));
    r.changed()
}

/// A `speed`-range slider (`[0, MAX_SPEED]`) returning whether it changed.
fn speed_slider(ui: &mut egui::Ui, label: &str, value: &mut i32) -> bool {
    let r = ui.add(egui::Slider::new(value, 0..=MAX_SPEED).text(label));
    r.changed()
}

/// Draw a small read-only preview of the curve `param_from_speed(speed)` across the speed range,
/// so the user sees the shape they are editing. Uses the exact core evaluation.
fn curve_preview(ui: &mut egui::Ui, id: &str, curve: &hyperion_core::rc::RcCurve) {
    let (rect, _resp) =
        ui.allocate_exact_size(egui::vec2(ui.available_width(), 90.0), egui::Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 4.0, ui.visuals().extreme_bg_color);

    let stroke = egui::Stroke::new(1.5, ui.visuals().widgets.active.fg_stroke.color);
    let mut prev: Option<egui::Pos2> = None;
    for speed in 0..=MAX_SPEED {
        let param = hyperion_core::rc::param_from_speed(curve, speed);
        let tx = speed as f32 / MAX_SPEED as f32;
        // Map param [MIN_PARAM, MAX_PARAM] to [bottom, top].
        let ty = (param - MIN_PARAM) as f32 / (MAX_PARAM - MIN_PARAM) as f32;
        let p = egui::pos2(
            rect.left() + tx * rect.width(),
            rect.bottom() - ty * rect.height(),
        );
        if let Some(prev) = prev {
            painter.line_segment([prev, p], stroke);
        }
        prev = Some(p);
    }
    // Zero-param baseline for reference.
    let zero_ty = (-MIN_PARAM) as f32 / (MAX_PARAM - MIN_PARAM) as f32;
    let zero_y = rect.bottom() - zero_ty * rect.height();
    painter.line_segment(
        [
            egui::pos2(rect.left(), zero_y),
            egui::pos2(rect.right(), zero_y),
        ],
        egui::Stroke::new(0.5, ui.visuals().weak_text_color()),
    );
    let _ = id;
}

/// The global threading + HidHide policy panel.
pub fn global_panel(ui: &mut egui::Ui, app: &mut HyperionApp) {
    ui.heading("Engine policy");

    // --- Thread / scheduling -----------------------------------------------------------------
    egui::CollapsingHeader::new("Thread / scheduling")
        .default_open(false)
        .show(ui, |ui| {
            let mut t = app.thread_mut().clone();
            let mut changed = false;

            changed |= ui
                .checkbox(&mut t.use_mmcss, "Register hot thread with MMCSS")
                .changed();
            changed |= ui
                .checkbox(
                    &mut t.skip_duplicate_reports,
                    "Skip filter work on duplicate reports",
                )
                .changed();

            // Wait mode.
            let before = t.wait_mode;
            egui::ComboBox::from_label("wait mode")
                .selected_text(format!("{:?}", t.wait_mode))
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut t.wait_mode, WaitMode::HybridSpin, "HybridSpin");
                    ui.selectable_value(&mut t.wait_mode, WaitMode::Blocking, "Blocking");
                });
            changed |= t.wait_mode != before;

            // dt source.
            let before = t.dt_source;
            egui::ComboBox::from_label("dt source")
                .selected_text(format!("{:?}", t.dt_source))
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut t.dt_source, DtSource::QpcOnly, "QpcOnly");
                    ui.selectable_value(
                        &mut t.dt_source,
                        DtSource::DeviceTimestamp,
                        "DeviceTimestamp",
                    );
                });
            changed |= t.dt_source != before;

            changed |= ui
                .add(
                    egui::Slider::new(&mut t.timer_resolution_us, 250..=2000)
                        .text("timer resolution (us)"),
                )
                .changed();
            changed |= ui
                .add(egui::Slider::new(&mut t.spin_budget_us, 0..=500).text("spin budget (us)"))
                .changed();

            if changed {
                *app.thread_mut() = t;
                app.push_thread();
            }
        });

    // --- HidHide -----------------------------------------------------------------------------
    egui::CollapsingHeader::new("HidHide cloaking")
        .default_open(false)
        .show(ui, |ui| {
            let mut h = app.hidhide_mut().clone();
            let mut changed = false;
            changed |= ui
                .checkbox(&mut h.enabled, "Cloak the physical pad")
                .changed();
            changed |= ui
                .checkbox(&mut h.use_cli, "Use HidHideCLI.exe (fallback)")
                .changed();
            ui.horizontal(|ui| {
                ui.label("CLI path:");
                changed |= ui.text_edit_singleline(&mut h.cli_path).changed();
            });
            if changed {
                *app.hidhide_mut() = h;
                app.push_hidhide();
            }
        });
}

/// Display label for a [`StickMode`].
fn mode_label(mode: StickMode) -> &'static str {
    match mode {
        StickMode::None => "None (pass-through)",
        StickMode::Rc => "RC filter",
    }
}

/// Display label for an [`RcMode`].
fn algo_label(mode: RcMode) -> &'static str {
    match mode {
        RcMode::FireBirdInteger => "FireBird (integer)",
        RcMode::UltimateLegacy => "Ultimate (legacy)",
        RcMode::UltimateDt => "Ultimate (dt-invariant)",
    }
}
