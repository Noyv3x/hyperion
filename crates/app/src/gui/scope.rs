//! The live input-vs-output stick scope and the telemetry readouts.
//!
//! Drawn with a plain [`egui::Painter`] (no extra plotting dep): two unit-square scopes (LS, RS),
//! each showing the **input** dot, the **filtered output** dot, and a short fading trail of recent
//! output points, so the user sees the RC filter's effect while tuning. The data comes purely
//! from the triple-buffered [`TelemetryFrame`]; the GUI never reaches into the hot loop.

use eframe::egui;
use engine::telemetry::TelemetryFrame;

/// Draw both stick scopes stacked vertically into the available width.
pub fn draw(
    ui: &mut egui::Ui,
    frame: &TelemetryFrame,
    ls_trail: &[egui::Vec2],
    rs_trail: &[egui::Vec2],
) {
    let side = (ui.available_width()).min(300.0);
    ui.label("Left stick");
    one_scope(
        ui,
        side,
        egui::vec2(frame.in_lx, frame.in_ly),
        egui::vec2(frame.out_lx, frame.out_ly),
        ls_trail,
    );
    ui.add_space(8.0);
    ui.label("Right stick");
    one_scope(
        ui,
        side,
        egui::vec2(frame.in_rx, frame.in_ry),
        egui::vec2(frame.out_rx, frame.out_ry),
        rs_trail,
    );
}

/// Draw a single square scope of size `side`, with `input`/`output` in canonical `[-1, 1]` axis
/// coordinates and an output `trail`.
fn one_scope(
    ui: &mut egui::Ui,
    side: f32,
    input: egui::Vec2,
    output: egui::Vec2,
    trail: &[egui::Vec2],
) {
    let (rect, _resp) = ui.allocate_exact_size(egui::vec2(side, side), egui::Sense::hover());
    let painter = ui.painter_at(rect);
    let visuals = ui.visuals();

    // Background + border.
    painter.rect_filled(rect, 4.0, visuals.extreme_bg_color);
    painter.rect_stroke(
        rect,
        4.0,
        egui::Stroke::new(1.0, visuals.widgets.noninteractive.fg_stroke.color),
        egui::StrokeKind::Inside,
    );

    // Crosshair at the neutral center.
    let center = rect.center();
    let axis = egui::Stroke::new(0.5, visuals.weak_text_color());
    painter.line_segment(
        [
            egui::pos2(rect.left(), center.y),
            egui::pos2(rect.right(), center.y),
        ],
        axis,
    );
    painter.line_segment(
        [
            egui::pos2(center.x, rect.top()),
            egui::pos2(center.x, rect.bottom()),
        ],
        axis,
    );

    // Map a [-1,1] axis vector to a screen position. Y is inverted (screen y grows downward).
    let half = side * 0.5;
    let map = |v: egui::Vec2| -> egui::Pos2 {
        egui::pos2(
            center.x + v.x.clamp(-1.0, 1.0) * half,
            center.y - v.y.clamp(-1.0, 1.0) * half,
        )
    };

    // Output trail: oldest faint, newest bright.
    let n = trail.len().max(1);
    for (i, &p) in trail.iter().enumerate() {
        let t = (i + 1) as f32 / n as f32;
        let color = visuals
            .widgets
            .active
            .fg_stroke
            .color
            .gamma_multiply(0.15 + 0.55 * t);
        painter.circle_filled(map(p), 1.5 + 1.5 * t, color);
    }

    // Input dot (hollow blue) and output dot (solid amber) for the latest frame.
    let in_pos = map(input);
    let out_pos = map(output);
    painter.circle_stroke(
        in_pos,
        5.0,
        egui::Stroke::new(1.5, egui::Color32::from_rgb(90, 160, 255)),
    );
    painter.circle_filled(out_pos, 4.0, egui::Color32::from_rgb(255, 176, 64));
    // A thin connector shows the filter's lag/lead between input and output.
    painter.line_segment(
        [in_pos, out_pos],
        egui::Stroke::new(
            1.0,
            egui::Color32::from_rgb(255, 176, 64).gamma_multiply(0.6),
        ),
    );
}

/// The telemetry readouts strip: dt, drops, duplicates, and loop-busy timing.
///
/// The [`TelemetryFrame`] carries the *latest* per-report `loop_busy_ns`; the engine's hot loop
/// owns the full p99 reservoir, so here we surface the latest loop-busy in microseconds (a direct,
/// honest readout) alongside the cumulative counters.
pub fn readouts(ui: &mut egui::Ui, frame: &TelemetryFrame) {
    ui.horizontal(|ui| {
        readout(ui, "dt", format!("{:.1} us", frame.dt_us));
        ui.separator();
        readout(
            ui,
            "loop busy",
            format!("{:.1} us", frame.loop_busy_ns as f64 / 1000.0),
        );
        ui.separator();
        readout(ui, "dropped", frame.dropped.to_string());
        ui.separator();
        readout(ui, "duplicates", frame.duplicates.to_string());
        ui.separator();
        let rate = if frame.dt_us > 0.0 {
            1_000_000.0 / frame.dt_us as f64
        } else {
            0.0
        };
        readout(ui, "rate", format!("{rate:.0} Hz"));
    });
}

/// A small labeled readout (`name: value`).
fn readout(ui: &mut egui::Ui, name: &str, value: String) {
    ui.label(egui::RichText::new(format!("{name}:")).weak());
    ui.label(egui::RichText::new(value).strong().monospace());
}
