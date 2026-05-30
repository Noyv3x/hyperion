//! The Macro editor screen (`DESIGN-REMAP.md` §8 macros.rs).
//!
//! Lists the active profile's macros and lets the user add / edit / delete them and their timed
//! step lists (`KeyDown` / `KeyUp` / `MouseDown` / `MouseUp` / `Wait`). The injector thread plays a
//! macro by id on a `Macro{start}` edge (blueprint §7.3), so the editor only mutates the profile's
//! `macros` table and sends `ControlMsg::UpsertMacro` / `DeleteMacro` through the
//! [`super::HyperionApp`] push helpers.
//!
//! The mirror's `macros` `Vec` is the editing source of truth; every mutation re-pushes the changed
//! macro (or a delete) so the engine's single writer stays authoritative.

use eframe::egui;
use hyperion_core::map::profile::{MacroMouseButton, MacroStep};
use hyperion_core::map::MacroDef;

use super::HyperionApp;

/// The Macros screen body.
pub fn macros_panel(ui: &mut egui::Ui, app: &mut HyperionApp) {
    ui.heading("Macros");
    ui.label(
        "Define timed key/mouse sequences. Bind a control to a macro in the Mapping tab; the \
         macro id is the key. Steps play in order on the injector thread, off the hot path.",
    );

    // --- Toolbar: add a new macro --------------------------------------------------------------
    ui.horizontal(|ui| {
        if ui.button("➕ Add macro").clicked() {
            let id = next_free_id(&app.mirror_mut().macros);
            let def = MacroDef {
                id,
                name: format!("macro {id}"),
                repeat: false,
                steps: Vec::new(),
            };
            app.mirror_mut().macros.push(def.clone());
            app.upsert_macro(def);
        }
    });
    ui.separator();

    if app.mirror_mut().macros.is_empty() {
        ui.label(
            egui::RichText::new("No macros yet — add one above.")
                .weak()
                .italics(),
        );
        return;
    }

    // Edit against a local clone of the list so the per-macro UI can borrow it mutably without
    // holding `app`; apply the diffs (upsert changed, delete removed) at the end.
    let mut list = app.mirror_mut().macros.clone();
    let mut to_upsert: Option<MacroDef> = None;
    let mut to_delete: Option<u16> = None;
    let mut deleted_index: Option<usize> = None;

    for (idx, def) in list.iter_mut().enumerate() {
        let mut changed = false;
        egui::CollapsingHeader::new(format!("#{} — {}", def.id, def.name))
            .id_salt(("macro", def.id))
            .default_open(idx == 0)
            .show(ui, |ui| {
                changed |= macro_header_ui(ui, def);
                ui.separator();
                changed |= steps_ui(ui, def);
                ui.separator();
                if ui.button("🗑 Delete this macro").clicked() {
                    to_delete = Some(def.id);
                    deleted_index = Some(idx);
                }
            });
        if changed {
            to_upsert = Some(def.clone());
        }
    }

    // Apply mutations to the mirror + engine. Delete takes precedence for a given frame's row.
    if let (Some(id), Some(i)) = (to_delete, deleted_index) {
        list.remove(i);
        app.mirror_mut().macros = list;
        app.delete_macro(id);
    } else {
        app.mirror_mut().macros = list;
        if let Some(def) = to_upsert {
            app.upsert_macro(def);
        }
    }
}

/// Name + repeat controls for one macro. Returns whether anything changed.
fn macro_header_ui(ui: &mut egui::Ui, def: &mut MacroDef) -> bool {
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.label("Name:");
        changed |= ui.text_edit_singleline(&mut def.name).changed();
    });
    changed |= ui
        .checkbox(
            &mut def.repeat,
            "Repeat while held (vs. fire once per press)",
        )
        .changed();
    changed
}

/// The reorderable step list editor for one macro. Returns whether anything changed.
fn steps_ui(ui: &mut egui::Ui, def: &mut MacroDef) -> bool {
    let mut changed = false;
    ui.label(egui::RichText::new("Steps").strong());

    let mut remove: Option<usize> = None;
    let mut move_up: Option<usize> = None;
    let mut move_down: Option<usize> = None;
    let len = def.steps.len();

    for (i, step) in def.steps.iter_mut().enumerate() {
        ui.horizontal(|ui| {
            ui.label(format!("{}.", i + 1));
            changed |= step_editor(ui, i, step);
            // Reorder / remove controls.
            if ui.add_enabled(i > 0, egui::Button::new("↑")).clicked() {
                move_up = Some(i);
            }
            if ui
                .add_enabled(i + 1 < len, egui::Button::new("↓"))
                .clicked()
            {
                move_down = Some(i);
            }
            if ui.button("✖").clicked() {
                remove = Some(i);
            }
        });
    }

    if let Some(i) = remove {
        def.steps.remove(i);
        changed = true;
    } else if let Some(i) = move_up {
        def.steps.swap(i, i - 1);
        changed = true;
    } else if let Some(i) = move_down {
        def.steps.swap(i, i + 1);
        changed = true;
    }

    // Add-step toolbar.
    ui.horizontal(|ui| {
        if ui.button("+ Key down").clicked() {
            def.steps.push(MacroStep::KeyDown {
                vk: 0x41,
                scan_code: true,
            });
            changed = true;
        }
        if ui.button("+ Key up").clicked() {
            def.steps.push(MacroStep::KeyUp {
                vk: 0x41,
                scan_code: true,
            });
            changed = true;
        }
        if ui.button("+ Mouse down").clicked() {
            def.steps.push(MacroStep::MouseDown(MacroMouseButton::Left));
            changed = true;
        }
        if ui.button("+ Mouse up").clicked() {
            def.steps.push(MacroStep::MouseUp(MacroMouseButton::Left));
            changed = true;
        }
        if ui.button("+ Wait").clicked() {
            def.steps.push(MacroStep::Wait { ms: 20 });
            changed = true;
        }
    });

    changed
}

/// The inline editor for one [`MacroStep`]. Returns whether it changed. Each arm labels itself
/// (the verb is bound per-variant, so no second borrow of `step` is needed).
fn step_editor(ui: &mut egui::Ui, salt: usize, step: &mut MacroStep) -> bool {
    let mut changed = false;
    match step {
        MacroStep::KeyDown { vk, scan_code } => {
            changed |= key_step_ui(ui, "key down", vk, scan_code);
        }
        MacroStep::KeyUp { vk, scan_code } => {
            changed |= key_step_ui(ui, "key up", vk, scan_code);
        }
        MacroStep::MouseDown(btn) => {
            changed |= mouse_step_ui(ui, salt, "mouse down", btn);
        }
        MacroStep::MouseUp(btn) => {
            changed |= mouse_step_ui(ui, salt, "mouse up", btn);
        }
        MacroStep::Wait { ms } => {
            ui.label("wait");
            changed |= ui
                .add(
                    egui::DragValue::new(ms)
                        .suffix(" ms")
                        .range(0u32..=60_000u32),
                )
                .changed();
        }
        MacroStep::Unknown => {
            ui.label(
                egui::RichText::new("(unknown step — from a newer profile)")
                    .weak()
                    .italics(),
            );
        }
    }
    changed
}

/// The shared key-step (down/up) inline editor: a hex VK drag + a scancode toggle.
fn key_step_ui(ui: &mut egui::Ui, verb: &str, vk: &mut u16, scan_code: &mut bool) -> bool {
    let mut changed = false;
    ui.label(verb);
    changed |= ui
        .add(
            egui::DragValue::new(vk)
                .prefix("vk 0x")
                .hexadecimal(2, false, true)
                .range(0u16..=0xFFu16),
        )
        .changed();
    changed |= ui.checkbox(scan_code, "scancode").changed();
    changed
}

/// The shared mouse-step (down/up) inline editor: a button combo.
fn mouse_step_ui(ui: &mut egui::Ui, salt: usize, verb: &str, btn: &mut MacroMouseButton) -> bool {
    let mut changed = false;
    ui.label(verb);
    egui::ComboBox::from_id_salt(("macro-mb", salt))
        .selected_text(mouse_btn_label(*btn))
        .show_ui(ui, |ui| {
            for b in [
                MacroMouseButton::Left,
                MacroMouseButton::Right,
                MacroMouseButton::Middle,
                MacroMouseButton::X1,
                MacroMouseButton::X2,
            ] {
                changed |= ui.selectable_value(btn, b, mouse_btn_label(b)).changed();
            }
        });
    changed
}

/// Display label for a [`MacroMouseButton`].
fn mouse_btn_label(b: MacroMouseButton) -> &'static str {
    match b {
        MacroMouseButton::Left => "Left",
        MacroMouseButton::Right => "Right",
        MacroMouseButton::Middle => "Middle",
        MacroMouseButton::X1 => "X1",
        MacroMouseButton::X2 => "X2",
        MacroMouseButton::Unknown => "(unknown)",
    }
}

/// The smallest macro id not already used (so a new macro gets a stable, unique key).
fn next_free_id(macros: &[MacroDef]) -> u16 {
    let mut id = 0u16;
    while macros.iter().any(|m| m.id == id) {
        id = id.saturating_add(1);
    }
    id
}
