//! The Mapping screen (`DESIGN-REMAP.md` §8 mapping.rs): bind any [`Control`] to any
//! [`BindTarget`], with an optional per-control **shift trigger** + shift bind and an optional
//! **turbo** / rapid-fire (blueprint §5).
//!
//! M4 expands the M3 single-row editor into the full per-control surface the milestone calls for:
//! pick a source control, choose the base bind kind (passthrough / unbound / gamepad-button /
//! gamepad-axis / touchpad-click / key / mouse-button / mouse-move / mouse-wheel / macro / special),
//! then optionally attach a shift layer (a trigger control + the bind used while it is held) and a
//! turbo cycle. The screen is **stateless with respect to the engine**: it never reads the live
//! binding table (that lives in the hot-facing `ResolvedProfile`); it only *emits*
//! `ControlMsg::SetBinding` / `SetShiftTrigger` / `SetBindingTurbo` through the
//! [`super::HyperionApp`] push helpers. The transient selection lives in [`BindingEditor`].

use eframe::egui;
use hyperion_core::input::Control;
use hyperion_core::map::{
    BindTarget, KeyKind, MacroDef, MouseMoveSrc, PadBtn, ShiftTrigger, TurboCfg, WheelDir,
};
use hyperion_core::output::MouseButton;

/// Transient state for the binding editor (never persisted; rebuilt each session).
#[derive(Clone)]
pub struct BindingEditor {
    /// The physical control currently selected as the remap source.
    control: Control,
    /// The base bind composer (kind + payload selections).
    base: BindComposer,
    /// Whether a shift layer is being attached.
    shift_enabled: bool,
    /// The control whose pressed state activates the shift bind.
    shift_trigger: Control,
    /// The bind used while the shift trigger is held.
    shift: BindComposer,
    /// Whether turbo / rapid-fire is attached.
    turbo_enabled: bool,
    /// Turbo full-cycle period, milliseconds (the UI unit; converted to `period_us` on apply).
    turbo_period_ms: u32,
    /// Turbo ON-fraction numerator.
    turbo_duty_num: u16,
    /// Turbo ON-fraction denominator.
    turbo_duty_den: u16,
}

impl Default for BindingEditor {
    fn default() -> Self {
        Self {
            control: Control::Cross,
            base: BindComposer::default(),
            shift_enabled: false,
            shift_trigger: Control::L1,
            shift: BindComposer::default(),
            turbo_enabled: false,
            turbo_period_ms: 100,
            turbo_duty_num: 1,
            turbo_duty_den: 2,
        }
    }
}

/// The composer for one [`BindTarget`] (used for both the base bind and the shift bind).
#[derive(Clone)]
struct BindComposer {
    /// Which binding *kind* is being composed.
    kind: BindKind,
    /// Selected virtual-pad button (when `kind == GamepadButton`).
    pad_btn: PadBtn,
    /// Selected mouse button (when `kind == MouseButton`).
    mouse_btn: UiMouseButton,
    /// Selected mouse-move source (when `kind == MouseMove`).
    mouse_src: UiMouseSrc,
    /// Selected wheel direction (when `kind == MouseWheel`).
    wheel_dir: UiWheelDir,
    /// Selected macro id (when `kind == Macro`).
    macro_id: u16,
    /// Selected special-action id (when `kind == Special`).
    special_id: u16,
    /// Captured keyboard virtual-key code (when `kind == Key`); `None` until a key is pressed.
    captured_vk: Option<u16>,
    /// Human label for the captured key (display only).
    captured_label: String,
    /// Whether the key-capture widget is armed (next key press is recorded).
    capturing: bool,
    /// Inject the captured key as a hardware scancode rather than a virtual key.
    scan_code: bool,
    /// Toggle (latch) rather than hold-while-pressed.
    toggle: bool,
}

impl Default for BindComposer {
    fn default() -> Self {
        Self {
            kind: BindKind::GamepadButton,
            pad_btn: PadBtn::B,
            mouse_btn: UiMouseButton::Left,
            mouse_src: UiMouseSrc::RightStick,
            wheel_dir: UiWheelDir::Up,
            macro_id: 0,
            special_id: 0,
            captured_vk: None,
            captured_label: String::new(),
            capturing: false,
            scan_code: true,
            toggle: false,
        }
    }
}

/// Which binding kind a [`BindComposer`] composes (a UI-only discriminator over [`BindTarget`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum BindKind {
    /// Identity passthrough (the natural "clear" for the base bind).
    Passthrough,
    /// Explicitly unbound (suppress identity, emit nothing).
    Unbound,
    /// Drive a virtual-pad button.
    GamepadButton,
    /// Inject a keyboard key.
    Key,
    /// Inject a mouse button.
    MouseButton,
    /// Feed a mouse-move accumulator from a stick.
    MouseMove,
    /// Emit mouse-wheel notches.
    MouseWheel,
    /// Trigger a macro by id.
    Macro,
    /// Fire a special action by id.
    Special,
}

impl BindKind {
    const ALL: [BindKind; 9] = [
        BindKind::Passthrough,
        BindKind::Unbound,
        BindKind::GamepadButton,
        BindKind::Key,
        BindKind::MouseButton,
        BindKind::MouseMove,
        BindKind::MouseWheel,
        BindKind::Macro,
        BindKind::Special,
    ];

    fn label(self) -> &'static str {
        match self {
            BindKind::Passthrough => "Passthrough (clear)",
            BindKind::Unbound => "Unbound (suppress)",
            BindKind::GamepadButton => "Gamepad button",
            BindKind::Key => "Keyboard key",
            BindKind::MouseButton => "Mouse button",
            BindKind::MouseMove => "Mouse move (from stick)",
            BindKind::MouseWheel => "Mouse wheel",
            BindKind::Macro => "Macro",
            BindKind::Special => "Special action",
        }
    }
}

/// UI mirror of [`MouseButton`] (a closed combo set with stable labels).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum UiMouseButton {
    Left,
    Right,
    Middle,
    X1,
    X2,
}

impl UiMouseButton {
    const ALL: [UiMouseButton; 5] = [
        UiMouseButton::Left,
        UiMouseButton::Right,
        UiMouseButton::Middle,
        UiMouseButton::X1,
        UiMouseButton::X2,
    ];
    fn label(self) -> &'static str {
        match self {
            UiMouseButton::Left => "Left",
            UiMouseButton::Right => "Right",
            UiMouseButton::Middle => "Middle",
            UiMouseButton::X1 => "X1 (back)",
            UiMouseButton::X2 => "X2 (forward)",
        }
    }
    fn to_core(self) -> MouseButton {
        match self {
            UiMouseButton::Left => MouseButton::Left,
            UiMouseButton::Right => MouseButton::Right,
            UiMouseButton::Middle => MouseButton::Middle,
            UiMouseButton::X1 => MouseButton::X1,
            UiMouseButton::X2 => MouseButton::X2,
        }
    }
}

/// UI mirror of [`MouseMoveSrc`] (the stick sources; gyro is M5).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum UiMouseSrc {
    LeftStick,
    RightStick,
}

impl UiMouseSrc {
    const ALL: [UiMouseSrc; 2] = [UiMouseSrc::LeftStick, UiMouseSrc::RightStick];
    fn label(self) -> &'static str {
        match self {
            UiMouseSrc::LeftStick => "Left stick",
            UiMouseSrc::RightStick => "Right stick",
        }
    }
    fn to_core(self) -> MouseMoveSrc {
        match self {
            UiMouseSrc::LeftStick => MouseMoveSrc::LeftStick,
            UiMouseSrc::RightStick => MouseMoveSrc::RightStick,
        }
    }
}

/// UI mirror of [`WheelDir`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum UiWheelDir {
    Up,
    Down,
    Left,
    Right,
}

impl UiWheelDir {
    const ALL: [UiWheelDir; 4] = [
        UiWheelDir::Up,
        UiWheelDir::Down,
        UiWheelDir::Left,
        UiWheelDir::Right,
    ];
    fn label(self) -> &'static str {
        match self {
            UiWheelDir::Up => "Up",
            UiWheelDir::Down => "Down",
            UiWheelDir::Left => "Left",
            UiWheelDir::Right => "Right",
        }
    }
    fn to_core(self) -> WheelDir {
        match self {
            UiWheelDir::Up => WheelDir::Up,
            UiWheelDir::Down => WheelDir::Down,
            UiWheelDir::Left => WheelDir::Left,
            UiWheelDir::Right => WheelDir::Right,
        }
    }
}

/// The Mapping screen body.
pub fn mapping_panel(ui: &mut egui::Ui, app: &mut super::HyperionApp) {
    ui.heading("Mapping");
    ui.label(
        "Pick a control, choose what it does, optionally attach a shift layer and/or turbo, \
         then Apply. The shift trigger and turbo are sent as separate edits so they compose with \
         the base bind.",
    );
    ui.separator();

    // Work on a local copy of the editor + the macro list (the macro combo needs the live ids);
    // write the editor back at the end. Capturing a key needs the egui InputState, read inside.
    let mut ed = app.binding_editor_mut().clone();
    let macros = app.mirror_mut().macros.clone();

    // --- Source control ------------------------------------------------------------------------
    ui.horizontal(|ui| {
        ui.label("Control:");
        egui::ComboBox::from_id_salt("binding-control")
            .selected_text(control_label(ed.control))
            .show_ui(ui, |ui| {
                for c in Control::ALL {
                    if c == Control::None {
                        continue; // the "no control" sentinel is not a bindable source.
                    }
                    ui.selectable_value(&mut ed.control, c, control_label(c));
                }
            });
    });

    // --- Base bind -----------------------------------------------------------------------------
    ui.group(|ui| {
        ui.label(egui::RichText::new("Base binding").strong());
        bind_composer_ui(ui, "base", &mut ed.base, &macros);
    });

    // --- Shift layer ---------------------------------------------------------------------------
    ui.add_space(4.0);
    ui.checkbox(&mut ed.shift_enabled, "Attach a shift layer");
    if ed.shift_enabled {
        ui.group(|ui| {
            ui.horizontal(|ui| {
                ui.label("Shift trigger:");
                egui::ComboBox::from_id_salt("shift-trigger")
                    .selected_text(control_label(ed.shift_trigger))
                    .show_ui(ui, |ui| {
                        for c in Control::ALL {
                            if c == Control::None {
                                continue;
                            }
                            ui.selectable_value(&mut ed.shift_trigger, c, control_label(c));
                        }
                    });
            });
            ui.label("While the trigger is held, this control uses:");
            bind_composer_ui(ui, "shift", &mut ed.shift, &macros);
        });
    }

    // --- Turbo ---------------------------------------------------------------------------------
    ui.add_space(4.0);
    ui.checkbox(&mut ed.turbo_enabled, "Attach turbo / rapid-fire");
    if ed.turbo_enabled {
        ui.group(|ui| {
            ui.add(egui::Slider::new(&mut ed.turbo_period_ms, 10..=1000).text("cycle period (ms)"));
            ui.horizontal(|ui| {
                ui.add(egui::Slider::new(&mut ed.turbo_duty_num, 1..=ed.turbo_duty_den).text("ON"));
                ui.label("/");
                ui.add(egui::Slider::new(&mut ed.turbo_duty_den, 1..=16).text("of"));
            });
            // Keep the numerator <= denominator so the duty stays a valid fraction.
            if ed.turbo_duty_num > ed.turbo_duty_den {
                ed.turbo_duty_num = ed.turbo_duty_den;
            }
            ui.label(
                egui::RichText::new(format!(
                    "≈ {:.0}% ON, {:.1} Hz",
                    100.0 * ed.turbo_duty_num as f32 / ed.turbo_duty_den as f32,
                    1000.0 / ed.turbo_period_ms as f32
                ))
                .weak(),
            );
        });
    }

    ui.separator();

    // --- Apply ---------------------------------------------------------------------------------
    let base_bind = compose_bind(&ed.base);
    let shift_bind = compose_bind(&ed.shift);
    // The row is applicable when the base bind composes; the shift bind only matters when enabled.
    let can_apply = base_bind.is_some() && (!ed.shift_enabled || shift_bind.is_some());
    ui.horizontal(|ui| {
        let apply = ui
            .add_enabled(can_apply, egui::Button::new("Apply binding"))
            .clicked();
        if !can_apply {
            ui.label(
                egui::RichText::new("capture a key for any Key bind first")
                    .weak()
                    .italics(),
            );
        }
        if apply {
            apply_binding(app, &mut ed, base_bind, shift_bind);
        }
    });

    *app.binding_editor_mut() = ed;
}

/// Emit the `SetBinding` + (conditional) `SetShiftTrigger` + `SetBindingTurbo` edits for the
/// composed row. Always sends the base bind; clears or sets the shift layer and turbo to match the
/// editor's checkboxes so toggling them off actually removes the prior value.
fn apply_binding(
    app: &mut super::HyperionApp,
    ed: &mut BindingEditor,
    base_bind: Option<BindTarget>,
    shift_bind: Option<BindTarget>,
) {
    let control = ed.control;
    let Some(base) = base_bind else { return };
    app.push_binding(control, base);

    // Shift: set (trigger + bind) when enabled and the bind composes; otherwise clear it.
    if ed.shift_enabled {
        if let Some(bind) = shift_bind {
            app.push_shift_trigger(
                control,
                Some(ShiftTrigger {
                    control: ed.shift_trigger,
                }),
                bind,
            );
        }
    } else {
        app.push_shift_trigger(control, None, BindTarget::Passthrough);
    }

    // Turbo: set when enabled, otherwise clear.
    if ed.turbo_enabled {
        app.push_binding_turbo(
            control,
            Some(TurboCfg {
                period_us: ed.turbo_period_ms.saturating_mul(1000).max(1),
                duty_num: ed.turbo_duty_num,
                duty_den: ed.turbo_duty_den.max(1),
            }),
        );
    } else {
        app.push_binding_turbo(control, None);
    }

    // Settle the key-capture arming once applied.
    ed.base.capturing = false;
    ed.shift.capturing = false;
}

/// Render the kind selector + the kind-specific payload editor for one [`BindComposer`].
fn bind_composer_ui(ui: &mut egui::Ui, salt: &str, c: &mut BindComposer, macros: &[MacroDef]) {
    ui.horizontal(|ui| {
        ui.label("Bind to:");
        egui::ComboBox::from_id_salt((salt, "kind"))
            .selected_text(c.kind.label())
            .show_ui(ui, |ui| {
                for k in BindKind::ALL {
                    ui.selectable_value(&mut c.kind, k, k.label());
                }
            });
    });

    match c.kind {
        BindKind::Passthrough => {
            ui.label("Passes through unchanged (identity).");
        }
        BindKind::Unbound => {
            ui.label("Emits nothing and suppresses the identity passthrough.");
        }
        BindKind::GamepadButton => {
            ui.horizontal(|ui| {
                ui.label("Button:");
                egui::ComboBox::from_id_salt((salt, "padbtn"))
                    .selected_text(padbtn_label(c.pad_btn))
                    .show_ui(ui, |ui| {
                        for b in PAD_BTNS {
                            ui.selectable_value(&mut c.pad_btn, b, padbtn_label(b));
                        }
                    });
            });
        }
        BindKind::Key => key_capture(ui, salt, c),
        BindKind::MouseButton => {
            ui.horizontal(|ui| {
                ui.label("Mouse button:");
                egui::ComboBox::from_id_salt((salt, "mousebtn"))
                    .selected_text(c.mouse_btn.label())
                    .show_ui(ui, |ui| {
                        for b in UiMouseButton::ALL {
                            ui.selectable_value(&mut c.mouse_btn, b, b.label());
                        }
                    });
            });
        }
        BindKind::MouseMove => {
            ui.horizontal(|ui| {
                ui.label("Source stick:");
                egui::ComboBox::from_id_salt((salt, "mousesrc"))
                    .selected_text(c.mouse_src.label())
                    .show_ui(ui, |ui| {
                        for s in UiMouseSrc::ALL {
                            ui.selectable_value(&mut c.mouse_src, s, s.label());
                        }
                    });
            });
            ui.label(
                egui::RichText::new("Tune sensitivity / deadzone in the Mouse tab.")
                    .weak()
                    .italics(),
            );
        }
        BindKind::MouseWheel => {
            ui.horizontal(|ui| {
                ui.label("Direction:");
                egui::ComboBox::from_id_salt((salt, "wheel"))
                    .selected_text(c.wheel_dir.label())
                    .show_ui(ui, |ui| {
                        for d in UiWheelDir::ALL {
                            ui.selectable_value(&mut c.wheel_dir, d, d.label());
                        }
                    });
            });
        }
        BindKind::Macro => macro_picker(ui, salt, c, macros),
        BindKind::Special => {
            ui.horizontal(|ui| {
                ui.label("Special action id:");
                ui.add(egui::DragValue::new(&mut c.special_id).range(0..=u16::MAX));
            });
            ui.label(
                egui::RichText::new("Fired through the control-plane side channel (M4 stub).")
                    .weak()
                    .italics(),
            );
        }
    }
}

/// The macro-id picker: a combo over the profile's defined macros (or a hint to add one).
fn macro_picker(ui: &mut egui::Ui, salt: &str, c: &mut BindComposer, macros: &[MacroDef]) {
    if macros.is_empty() {
        ui.label(
            egui::RichText::new("No macros defined — add one in the Macros tab first.")
                .weak()
                .italics(),
        );
        return;
    }
    // Default the selection to a real id if the current one is not among the defined macros.
    if !macros.iter().any(|m| m.id == c.macro_id) {
        c.macro_id = macros[0].id;
    }
    ui.horizontal(|ui| {
        ui.label("Macro:");
        egui::ComboBox::from_id_salt((salt, "macro"))
            .selected_text(macro_label(macros, c.macro_id))
            .show_ui(ui, |ui| {
                for m in macros {
                    ui.selectable_value(&mut c.macro_id, m.id, macro_label(macros, m.id));
                }
            });
    });
}

/// Display label for a macro id (its name + id), falling back to just the id.
fn macro_label(macros: &[MacroDef], id: u16) -> String {
    match macros.iter().find(|m| m.id == id) {
        Some(m) if !m.name.is_empty() => format!("{} (#{id})", m.name),
        _ => format!("#{id}"),
    }
}

/// The "click to capture, then press a key" widget for one composer. Reads the egui [`InputState`]
/// for the next key press while armed and records its Windows virtual-key code.
fn key_capture(ui: &mut egui::Ui, salt: &str, c: &mut BindComposer) {
    ui.horizontal(|ui| {
        let btn_text = if c.capturing {
            "press any key…".to_string()
        } else {
            match &c.captured_vk {
                Some(_) => format!("key: {}", c.captured_label),
                None => "click to capture".to_string(),
            }
        };
        if ui.button(btn_text).clicked() {
            c.capturing = !c.capturing;
        }
        if c.captured_vk.is_some() && ui.button("clear").clicked() {
            c.captured_vk = None;
            c.captured_label.clear();
        }
        let _ = salt;
    });

    if c.capturing {
        let captured = ui.input(|i| {
            for key in egui::Key::ALL {
                if i.key_pressed(*key) {
                    return Some(*key);
                }
            }
            None
        });
        if let Some(key) = captured {
            if let Some(vk) = vk_from_egui_key(key) {
                c.captured_vk = Some(vk);
                c.captured_label = format!("{key:?} (vk 0x{vk:02X})");
            }
            c.capturing = false;
        }
    }

    ui.checkbox(&mut c.scan_code, "Inject as scancode (game-compatible)");
    ui.checkbox(&mut c.toggle, "Toggle (latch) instead of hold");
}

/// Compose the [`BindTarget`] a composer currently describes, or `None` when incomplete (a Key bind
/// with no captured key). The engine wraps this base bind into the profile's `BindingSlot` on apply.
fn compose_bind(c: &BindComposer) -> Option<BindTarget> {
    Some(match c.kind {
        BindKind::Passthrough => BindTarget::Passthrough,
        BindKind::Unbound => BindTarget::Unbound,
        BindKind::GamepadButton => BindTarget::GamepadButton(c.pad_btn),
        BindKind::Key => BindTarget::Key {
            vk: c.captured_vk?,
            kind: KeyKind {
                scan_code: c.scan_code,
                toggle: c.toggle,
            },
        },
        BindKind::MouseButton => BindTarget::Mouse(c.mouse_btn.to_core()),
        BindKind::MouseMove => BindTarget::MouseMove(c.mouse_src.to_core()),
        BindKind::MouseWheel => BindTarget::MouseWheel(c.wheel_dir.to_core()),
        BindKind::Macro => BindTarget::Macro(c.macro_id),
        BindKind::Special => BindTarget::Special(c.special_id),
    })
}

/// The virtual-pad buttons offered in the editor (the meaningful X360-mappable subset; the
/// touchpad / L2-R2-click variants exist on [`PadBtn`] but are left to later screens).
const PAD_BTNS: [PadBtn; 15] = [
    PadBtn::A,
    PadBtn::B,
    PadBtn::X,
    PadBtn::Y,
    PadBtn::Lb,
    PadBtn::Rb,
    PadBtn::Back,
    PadBtn::Start,
    PadBtn::Ls,
    PadBtn::Rs,
    PadBtn::Guide,
    PadBtn::DpadUp,
    PadBtn::DpadDown,
    PadBtn::DpadLeft,
    PadBtn::DpadRight,
];

/// Display label for a [`PadBtn`].
fn padbtn_label(b: PadBtn) -> &'static str {
    match b {
        PadBtn::A => "A",
        PadBtn::B => "B",
        PadBtn::X => "X",
        PadBtn::Y => "Y",
        PadBtn::Lb => "LB (L1)",
        PadBtn::Rb => "RB (R1)",
        PadBtn::Back => "Back (Share)",
        PadBtn::Start => "Start (Options)",
        PadBtn::Ls => "LS click",
        PadBtn::Rs => "RS click",
        PadBtn::Guide => "Guide (PS)",
        PadBtn::DpadUp => "D-pad up",
        PadBtn::DpadDown => "D-pad down",
        PadBtn::DpadLeft => "D-pad left",
        PadBtn::DpadRight => "D-pad right",
        PadBtn::L2Click => "L2 click",
        PadBtn::R2Click => "R2 click",
        PadBtn::Touchpad => "Touchpad",
        PadBtn::Unknown => "(unknown)",
    }
}

/// Display label for a [`Control`] (its serde PascalCase name is the stable identity).
fn control_label(c: Control) -> &'static str {
    use Control::*;
    match c {
        None => "None",
        LxNeg => "Left stick left",
        LxPos => "Left stick right",
        LyNeg => "Left stick down",
        LyPos => "Left stick up",
        RxNeg => "Right stick left",
        RxPos => "Right stick right",
        RyNeg => "Right stick down",
        RyPos => "Right stick up",
        L1 => "L1",
        L2 => "L2 (analog)",
        L3 => "L3 (left stick click)",
        R1 => "R1",
        R2 => "R2 (analog)",
        R3 => "R3 (right stick click)",
        Square => "Square",
        Triangle => "Triangle",
        Circle => "Circle",
        Cross => "Cross",
        DpadUp => "D-pad up",
        DpadRight => "D-pad right",
        DpadDown => "D-pad down",
        DpadLeft => "D-pad left",
        Ps => "PS",
        Share => "Share",
        Options => "Options",
        Mute => "Mute",
        Capture => "Capture",
        FnL => "Fn L (Edge)",
        FnR => "Fn R (Edge)",
        Blp => "Back paddle L (Edge)",
        Brp => "Back paddle R (Edge)",
        SideL => "Side L (Edge)",
        SideR => "Side R (Edge)",
        L2FullPull => "L2 full pull",
        R2FullPull => "R2 full pull",
        LsOuter => "LS outer ring",
        RsOuter => "RS outer ring",
        TouchButton => "Touchpad click",
        TouchLeft => "Touch left",
        TouchRight => "Touch right",
        TouchUpper => "Touch upper",
        TouchMulti => "Touch multi",
        GyroXPos => "Gyro X+",
        GyroXNeg => "Gyro X-",
        GyroZPos => "Gyro Z+",
        GyroZNeg => "Gyro Z-",
    }
}

/// Map an [`egui::Key`] to its Windows virtual-key code (`VK_*`), or `None` for keys without a
/// stable single VK (so capture simply ignores them and waits for the next press).
///
/// Letters → `0x41..=0x5A`, digits → `0x30..=0x39`, function keys → `VK_F1 (0x70)..`, plus the
/// common navigation / editing / punctuation keys. This is the inverse of the sink-side
/// `scancodeFromVK`; the captured VK is what `ControlMsg::SetBinding` persists.
fn vk_from_egui_key(key: egui::Key) -> Option<u16> {
    use egui::Key::*;
    Some(match key {
        A => 0x41,
        B => 0x42,
        C => 0x43,
        D => 0x44,
        E => 0x45,
        F => 0x46,
        G => 0x47,
        H => 0x48,
        I => 0x49,
        J => 0x4A,
        K => 0x4B,
        L => 0x4C,
        M => 0x4D,
        N => 0x4E,
        O => 0x4F,
        P => 0x50,
        Q => 0x51,
        R => 0x52,
        S => 0x53,
        T => 0x54,
        U => 0x55,
        V => 0x56,
        W => 0x57,
        X => 0x58,
        Y => 0x59,
        Z => 0x5A,
        Num0 => 0x30,
        Num1 => 0x31,
        Num2 => 0x32,
        Num3 => 0x33,
        Num4 => 0x34,
        Num5 => 0x35,
        Num6 => 0x36,
        Num7 => 0x37,
        Num8 => 0x38,
        Num9 => 0x39,
        F1 => 0x70,
        F2 => 0x71,
        F3 => 0x72,
        F4 => 0x73,
        F5 => 0x74,
        F6 => 0x75,
        F7 => 0x76,
        F8 => 0x77,
        F9 => 0x78,
        F10 => 0x79,
        F11 => 0x7A,
        F12 => 0x7B,
        Escape => 0x1B,
        Tab => 0x09,
        Backspace => 0x08,
        Enter => 0x0D,
        Space => 0x20,
        Insert => 0x2D,
        Delete => 0x2E,
        Home => 0x24,
        End => 0x23,
        PageUp => 0x21,
        PageDown => 0x22,
        ArrowLeft => 0x25,
        ArrowUp => 0x26,
        ArrowRight => 0x27,
        ArrowDown => 0x28,
        Semicolon => 0xBA,
        Plus | Equals => 0xBB,
        Comma => 0xBC,
        Minus => 0xBD,
        Period => 0xBE,
        Slash | Questionmark => 0xBF,
        Backtick => 0xC0,
        OpenBracket | OpenCurlyBracket => 0xDB,
        Backslash | Pipe => 0xDC,
        CloseBracket | CloseCurlyBracket => 0xDD,
        Quote => 0xDE,
        _ => return None,
    })
}
