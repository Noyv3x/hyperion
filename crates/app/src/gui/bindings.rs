//! The minimal M3 Mapping screen: a single `Control → Binding` remap row.
//!
//! `DESIGN-REMAP.md` §8 (mapping.rs) describes the full controller-diagram editor; M3 ships the
//! **one basic remap path** the milestone exit criteria call for (Cross→Xbox-B and Square→key-A):
//! pick a [`Control`], pick a simple [`BindTarget`] — a virtual-pad [`PadBtn`] or a captured
//! keyboard [`Key`](BindTarget::Key) — and apply it. The per-control shift / turbo / macro / mouse
//! editors land in M4/M5; the data model already carries those fields so this is purely additive.
//!
//! The screen is **stateless with respect to the engine**: it never reads the live binding table
//! (that lives in the hot-facing `ResolvedProfile`); it only *emits* `ControlMsg::SetBinding`
//! (a remap) or a `Passthrough` slot (a clear) through [`super::HyperionApp::push_binding`]. The
//! editor's transient selection lives in [`BindingEditor`].

use eframe::egui;
use hyperion_core::input::Control;
use hyperion_core::map::{BindTarget, KeyKind, PadBtn};

/// Transient state for the single binding-editor row (never persisted; rebuilt each session).
#[derive(Clone)]
pub struct BindingEditor {
    /// The physical control currently selected as the remap source.
    control: Control,
    /// Which binding *kind* the row is composing.
    kind: BindKind,
    /// Selected virtual-pad button (when `kind == GamepadButton`).
    pad_btn: PadBtn,
    /// Captured keyboard virtual-key code (when `kind == Key`); `None` until a key is pressed.
    captured_vk: Option<u16>,
    /// Human label for the captured key (display only).
    captured_label: String,
    /// Whether the key-capture widget is armed (next key press is recorded).
    capturing: bool,
    /// Inject the captured key as a hardware scancode (game-compatible) rather than a virtual key.
    scan_code: bool,
    /// Toggle (latch) rather than hold-while-pressed.
    toggle: bool,
}

impl Default for BindingEditor {
    fn default() -> Self {
        Self {
            control: Control::Cross,
            kind: BindKind::GamepadButton,
            pad_btn: PadBtn::B,
            captured_vk: None,
            captured_label: String::new(),
            capturing: false,
            scan_code: true,
            toggle: false,
        }
    }
}

/// Which simple binding kind the M3 row composes (a small UI-only discriminator).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum BindKind {
    /// Identity passthrough (clears any remap on the selected control).
    Passthrough,
    /// Drive a virtual-pad button.
    GamepadButton,
    /// Inject a keyboard key.
    Key,
}

impl BindKind {
    const ALL: [BindKind; 3] = [
        BindKind::Passthrough,
        BindKind::GamepadButton,
        BindKind::Key,
    ];

    fn label(self) -> &'static str {
        match self {
            BindKind::Passthrough => "Passthrough (clear)",
            BindKind::GamepadButton => "Gamepad button",
            BindKind::Key => "Keyboard key",
        }
    }
}

/// The Mapping screen body.
pub fn mapping_panel(ui: &mut egui::Ui, app: &mut super::HyperionApp) {
    ui.heading("Mapping");
    ui.label(
        "Pick a control, choose what it should do, then Apply. \
         M3 supports passthrough, gamepad-button, and keyboard-key bindings.",
    );
    ui.separator();

    // Build the row against a local copy of the editor state, then write it back. Capturing a key
    // needs the egui InputState, so it is read inside this closure.
    let mut ed = app.binding_editor_mut().clone();

    // --- Source control ------------------------------------------------------------------------
    ui.horizontal(|ui| {
        ui.label("Control:");
        egui::ComboBox::from_id_salt("binding-control")
            .selected_text(control_label(ed.control))
            .show_ui(ui, |ui| {
                for c in Control::ALL {
                    // None is the "no control" sentinel — not a bindable source.
                    if c == Control::None {
                        continue;
                    }
                    ui.selectable_value(&mut ed.control, c, control_label(c));
                }
            });
    });

    // --- Binding kind --------------------------------------------------------------------------
    ui.horizontal(|ui| {
        ui.label("Bind to:");
        egui::ComboBox::from_id_salt("binding-kind")
            .selected_text(ed.kind.label())
            .show_ui(ui, |ui| {
                for k in BindKind::ALL {
                    ui.selectable_value(&mut ed.kind, k, k.label());
                }
            });
    });

    // --- Kind-specific editor ------------------------------------------------------------------
    match ed.kind {
        BindKind::Passthrough => {
            ui.label("This control will pass through unchanged (identity).");
        }
        BindKind::GamepadButton => {
            ui.horizontal(|ui| {
                ui.label("Button:");
                egui::ComboBox::from_id_salt("binding-padbtn")
                    .selected_text(padbtn_label(ed.pad_btn))
                    .show_ui(ui, |ui| {
                        for b in PAD_BTNS {
                            ui.selectable_value(&mut ed.pad_btn, b, padbtn_label(b));
                        }
                    });
            });
        }
        BindKind::Key => {
            key_capture(ui, &mut ed);
        }
    }

    ui.separator();

    // --- Apply ---------------------------------------------------------------------------------
    let bind = compose_bind(&ed);
    let can_apply = bind.is_some();
    ui.horizontal(|ui| {
        let apply = ui
            .add_enabled(can_apply, egui::Button::new("Apply binding"))
            .clicked();
        if !can_apply {
            ui.label(egui::RichText::new("capture a key first").weak().italics());
        }
        if apply {
            if let Some(bind) = bind {
                let control = ed.control;
                // Stop capturing once applied so the row settles.
                ed.capturing = false;
                app.push_binding(control, bind);
            }
        }
    });

    *app.binding_editor_mut() = ed;
}

/// The "click to capture, then press a key" widget. Reads the egui [`InputState`] for the next
/// key press while armed and records its Windows virtual-key code.
fn key_capture(ui: &mut egui::Ui, ed: &mut BindingEditor) {
    ui.horizontal(|ui| {
        let btn_text = if ed.capturing {
            "press any key…".to_string()
        } else {
            match &ed.captured_vk {
                Some(_) => format!("key: {}", ed.captured_label),
                None => "click to capture".to_string(),
            }
        };
        if ui.button(btn_text).clicked() {
            ed.capturing = !ed.capturing;
        }
        if ed.captured_vk.is_some() && ui.button("clear").clicked() {
            ed.captured_vk = None;
            ed.captured_label.clear();
        }
    });

    if ed.capturing {
        // Find the first key currently held this frame and record it.
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
                ed.captured_vk = Some(vk);
                ed.captured_label = format!("{key:?} (vk 0x{vk:02X})");
            }
            ed.capturing = false;
        }
    }

    ui.checkbox(&mut ed.scan_code, "Inject as scancode (game-compatible)");
    ui.checkbox(&mut ed.toggle, "Toggle (latch) instead of hold");
}

/// Compose the [`BindTarget`] the row currently describes, or `None` when the row is incomplete
/// (a Key binding with no captured key). The engine wraps this base bind into the profile's
/// `BindingSlot` on apply (`SetBinding { profile, control, bind }`).
fn compose_bind(ed: &BindingEditor) -> Option<BindTarget> {
    Some(match ed.kind {
        BindKind::Passthrough => BindTarget::Passthrough,
        BindKind::GamepadButton => BindTarget::GamepadButton(ed.pad_btn),
        BindKind::Key => BindTarget::Key {
            vk: ed.captured_vk?,
            kind: KeyKind {
                scan_code: ed.scan_code,
                toggle: ed.toggle,
            },
        },
    })
}

/// The virtual-pad buttons offered in the M3 row (the meaningful X360-mappable subset; the
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
        // Letters (VK == ASCII uppercase).
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
        // Digits (top-row; VK == ASCII).
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
        // Function keys (VK_F1 = 0x70).
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
        // Navigation / editing.
        Escape => 0x1B,     // VK_ESCAPE
        Tab => 0x09,        // VK_TAB
        Backspace => 0x08,  // VK_BACK
        Enter => 0x0D,      // VK_RETURN
        Space => 0x20,      // VK_SPACE
        Insert => 0x2D,     // VK_INSERT
        Delete => 0x2E,     // VK_DELETE
        Home => 0x24,       // VK_HOME
        End => 0x23,        // VK_END
        PageUp => 0x21,     // VK_PRIOR
        PageDown => 0x22,   // VK_NEXT
        ArrowLeft => 0x25,  // VK_LEFT
        ArrowUp => 0x26,    // VK_UP
        ArrowRight => 0x27, // VK_RIGHT
        ArrowDown => 0x28,  // VK_DOWN
        // Punctuation (OEM keys, US layout).
        Semicolon => 0xBA,                        // VK_OEM_1
        Plus | Equals => 0xBB,                    // VK_OEM_PLUS
        Comma => 0xBC,                            // VK_OEM_COMMA
        Minus => 0xBD,                            // VK_OEM_MINUS
        Period => 0xBE,                           // VK_OEM_PERIOD
        Slash | Questionmark => 0xBF,             // VK_OEM_2
        Backtick => 0xC0,                         // VK_OEM_3
        OpenBracket | OpenCurlyBracket => 0xDB,   // VK_OEM_4
        Backslash | Pipe => 0xDC,                 // VK_OEM_5
        CloseBracket | CloseCurlyBracket => 0xDD, // VK_OEM_6
        Quote => 0xDE,                            // VK_OEM_7
        // Keys without a single stable US VK (Colon, Exclamationmark, Copy/Cut/Paste, F13+,
        // BrowserBack, …) are ignored so capture waits for the next, mappable, press.
        _ => return None,
    })
}
