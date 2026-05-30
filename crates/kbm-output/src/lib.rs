//! `kbm-output` — the Windows keyboard/mouse injection shell (blueprint §6.1, §7.3).
//!
//! The pure core builds a [`KbmBatch`](hyperion_core::output::kbm::KbmBatch) of edge events per
//! report and hands it to the injector thread over an SPSC ring. This crate is the *only* place
//! that calls `SendInput`: [`SendInputKbm`] turns a batch into a pre-sized, reused `Vec<INPUT>`
//! scratch buffer and submits it in **one** batched `SendInput` per report.
//!
//! Design points pinned by the blueprint:
//! - **No allocation on the hot injector path.** `scratch` is sized to [`KBM_BATCH_CAP`] up front
//!   and only ever `clear()`ed + repushed, never grown (a batch is capacity-bounded, so the
//!   `INPUT` count never exceeds `KBM_BATCH_CAP`).
//! - **Edge-dedupe.** `held_keys`/`held_mouse` track what is currently down, so a duplicate
//!   `Key { down: true }` for an already-held key (and the symmetric release) is dropped rather
//!   than emitting a redundant syscall record.
//! - **Scancode by default.** `KEYEVENTF_SCANCODE` (+ `KEYEVENTF_EXTENDEDKEY` for the extended VK
//!   set) is the game-friendly default, ported bit-for-bit from `SendInputHandler.scancodeFromVK`.
//! - **`release_all()`** lifts every currently-held key/button — called on disable, profile switch,
//!   and shutdown so nothing sticks down.
//!
//! The whole crate is `#![cfg(windows)]`; on other targets it compiles to an empty lib so the
//! Linux CI `cargo check` of the workspace still type-checks the dependency graph.
//!
//! ## M4 additions
//! * **Mouse**: relative-move (`MOUSEEVENTF_MOVE`), wheel (`MOUSEEVENTF_WHEEL`/`HWHEEL` in
//!   `WHEEL_DELTA` units), and full mouse-button edges incl. `X1`/`X2` via `mouseData` — all batched
//!   into the same single-`SendInput`-per-report scratch as keys, and lifted by
//!   [`release_all`](KbmSink::release_all).
//! * **Macro playback** ([`macro_player`]): a [`MacroPlayer`] the injector thread owns. The hot loop
//!   only emits `KbmEvent::Macro` start/stop edges; the player holds the resolved profile's macro
//!   definitions ([`MacroDef`]/[`MacroStep`]) and advances each in-flight macro's step schedule on a
//!   timer, injecting through the same [`SendInputKbm`] so its edge-dedupe state is shared.
#![cfg(windows)]

pub mod macro_player;

pub use macro_player::{MacroDef, MacroMouseButton, MacroPlayer, MacroStep};

use hyperion_core::output::kbm::{KbmBatch, KbmEvent, KeyKind, MouseButton, KBM_BATCH_CAP};

use windows::Win32::UI::Input::KeyboardAndMouse::{
    MapVirtualKeyW, SendInput, INPUT, INPUT_0, INPUT_KEYBOARD, INPUT_MOUSE, KEYBDINPUT,
    KEYBD_EVENT_FLAGS, KEYEVENTF_EXTENDEDKEY, KEYEVENTF_KEYUP, KEYEVENTF_SCANCODE, MAPVK_VK_TO_VSC,
    MOUSEEVENTF_HWHEEL, MOUSEEVENTF_LEFTDOWN, MOUSEEVENTF_LEFTUP, MOUSEEVENTF_MIDDLEDOWN,
    MOUSEEVENTF_MIDDLEUP, MOUSEEVENTF_MOVE, MOUSEEVENTF_RIGHTDOWN, MOUSEEVENTF_RIGHTUP,
    MOUSEEVENTF_WHEEL, MOUSEEVENTF_XDOWN, MOUSEEVENTF_XUP, MOUSEINPUT, MOUSE_EVENT_FLAGS,
    VIRTUAL_KEY,
};

/// X-button selectors packed into `MOUSEINPUT.mouseData` when `MOUSEEVENTF_XDOWN/XUP` is set.
/// Defined locally as the documented Win32 `u32` values so the `mouseData: u32` assignment
/// typechecks regardless of how the `windows` crate types its `XBUTTON1`/`XBUTTON2` constants.
const XBUTTON1: u32 = 0x0001;
/// See [`XBUTTON1`].
const XBUTTON2: u32 = 0x0002;

/// The extended-key VK set from `SendInputHandler.scancodeFromVK` (DS4Windows). These VKs get the
/// `0x100` extended bit OR'd into the scancode, which the sink turns into `KEYEVENTF_EXTENDEDKEY`.
/// Kept as a const array (not a match) so it is auditable against the C# `switch` 1:1.
const EXTENDED_VKS: [u16; 25] = [
    0x25, // VK_LEFT
    0x26, // VK_UP
    0x27, // VK_RIGHT
    0x28, // VK_DOWN
    0x21, // VK_PRIOR (Page Up)
    0x22, // VK_NEXT  (Page Down)
    0x23, // VK_END
    0x24, // VK_HOME
    0x2D, // VK_INSERT
    0x2E, // VK_DELETE
    0x6F, // VK_DIVIDE (numpad /)
    0x90, // VK_NUMLOCK
    0xA3, // VK_RCONTROL
    0xA5, // VK_RMENU (right Alt)
    0x5D, // VK_APPS (context-menu key)
    0xAD, // VK_VOLUME_MUTE
    0xAE, // VK_VOLUME_DOWN
    0xAF, // VK_VOLUME_UP
    0xB0, // VK_MEDIA_NEXT_TRACK
    0xB1, // VK_MEDIA_PREV_TRACK
    0xB5, // VK_LAUNCH_MEDIA_SELECT
    0xAC, // VK_BROWSER_HOME
    0xB4, // VK_LAUNCH_MAIL
    0xB6, // VK_LAUNCH_APP1
    0xB7, // VK_LAUNCH_APP2
];

/// `VK_PAUSE`; `MapVirtualKey` does not yield a usable scancode for it, so it is special-cased to
/// the hardware value `0x45` exactly as DS4Windows does.
const VK_PAUSE: u16 = 0x13;

/// Resolve a virtual-key code to its `(scancode, extended)` pair.
///
/// Ports `SendInputHandler.scancodeFromVK`: `MapVirtualKeyW(vk, MAPVK_VK_TO_VSC)` for the base
/// scancode (with the `VK_PAUSE` special-case), then the extended flag from [`EXTENDED_VKS`]. The
/// C# code folds the extended bit into the scancode (`scancode |= 0x100`) and later tests
/// `(scancode & 0x100) != 0`; here the boolean is returned directly so callers never carry the
/// `0x100` bit into `KEYBDINPUT.wScan` (which is a 16-bit hardware scancode).
fn scancode_from_vk(vk: u16) -> (u16, bool) {
    let scancode = if vk == VK_PAUSE {
        0x45
    } else {
        // SAFETY: `MapVirtualKeyW` reads no caller memory and has no preconditions beyond a valid
        // map type; `MAPVK_VK_TO_VSC` is the documented "VK -> scancode" mode. The result is a
        // 16-bit scancode (0 when the VK has no mapping), narrowed below.
        let sc = unsafe { MapVirtualKeyW(vk as u32, MAPVK_VK_TO_VSC) };
        sc as u16
    };
    let extended = EXTENDED_VKS.contains(&vk);
    (scancode, extended)
}

/// A keyboard/mouse output backend. The engine's injector thread owns one [`SendInputKbm`] and
/// calls [`flush`](Self::flush) once per KBM-producing report, and [`release_all`](Self::release_all)
/// on disable / profile-switch / shutdown.
pub trait KbmSink {
    /// The backend's failure type.
    type Error;
    /// Realize every edge in `batch` in one batched submit. Held-key/button edges that would be
    /// redundant (already-down key pressed again, etc.) are deduped away.
    fn flush(&mut self, batch: &KbmBatch) -> Result<(), Self::Error>;
    /// Release every currently-held key and mouse button (idempotent; safe to call when nothing
    /// is held — then it is a no-op).
    fn release_all(&mut self) -> Result<(), Self::Error>;
}

/// `SendInput`-backed [`KbmSink`] (blueprint §6.1).
///
/// Holds a pre-sized scratch `Vec<INPUT>` (capacity [`KBM_BATCH_CAP`], never reallocated) plus the
/// edge-dedupe bookkeeping. Construct with [`SendInputKbm::new`].
pub struct SendInputKbm {
    /// Reused INPUT staging buffer. Capacity is pinned to [`KBM_BATCH_CAP`]; `flush` only
    /// `clear()`s + repushes, so it never grows on the hot path.
    scratch: Vec<INPUT>,
    /// Currently-down keys, indexed by VK (`0..=255`). Used to dedupe held-key edges and to drive
    /// [`release_all`](Self::release_all).
    held_keys: [bool; 256],
    /// Currently-down mouse buttons as a bitmask over [`MouseButton`] (`1 << btn as u8`).
    held_mouse: u8,
    /// When `true`, re-emit a held key's down edge each report (OS key-repeat, e.g. for chat).
    /// Defaults `false` so a held key costs zero syscalls after the first report.
    fake_key_repeat: bool,
}

impl SendInputKbm {
    /// Create a sink with an empty held-state and a scratch buffer pre-sized to [`KBM_BATCH_CAP`].
    pub fn new() -> Self {
        Self {
            scratch: Vec::with_capacity(KBM_BATCH_CAP),
            held_keys: [false; 256],
            held_mouse: 0,
            fake_key_repeat: false,
        }
    }

    /// Enable/disable OS key-repeat re-emission for held keys (default `false`).
    #[inline]
    pub fn set_fake_key_repeat(&mut self, on: bool) {
        self.fake_key_repeat = on;
    }

    /// Build the `INPUT` records for `batch` into `self.scratch`, updating the held-key/button
    /// edge-dedupe state. Returns the number of records staged (`== self.scratch.len()`).
    ///
    /// Pure w.r.t. the OS *except* the `MapVirtualKeyW` scancode lookup; it issues **no**
    /// `SendInput`, so tests can call it and inspect `self.scratch` directly. `flush` calls this
    /// then submits the staged records in one syscall.
    fn build_inputs(&mut self, batch: &KbmBatch) -> usize {
        self.scratch.clear();
        for &ev in batch.as_slice() {
            match ev {
                KbmEvent::Key { vk, down, kind } => {
                    let idx = vk as usize & 0xFF;
                    // Edge-dedupe: drop a press for an already-held key (unless fake-repeat is on)
                    // and a release for a key that is not held.
                    if down {
                        if self.held_keys[idx] && !self.fake_key_repeat {
                            continue;
                        }
                        self.held_keys[idx] = true;
                    } else {
                        if !self.held_keys[idx] {
                            continue;
                        }
                        self.held_keys[idx] = false;
                    }
                    self.scratch.push(key_input(vk, down, kind));
                }
                KbmEvent::MouseButton { btn, down } => {
                    let bit = 1u8 << (btn as u8);
                    if down {
                        if self.held_mouse & bit != 0 {
                            continue;
                        }
                        self.held_mouse |= bit;
                    } else {
                        if self.held_mouse & bit == 0 {
                            continue;
                        }
                        self.held_mouse &= !bit;
                    }
                    self.scratch.push(mouse_button_input(btn, down));
                }
                KbmEvent::MouseMove { dx, dy } => {
                    if dx != 0 || dy != 0 {
                        self.scratch.push(mouse_move_input(dx, dy));
                    }
                }
                KbmEvent::Wheel {
                    vertical,
                    horizontal,
                } => {
                    if vertical != 0 {
                        self.scratch
                            .push(mouse_wheel_input(MOUSEEVENTF_WHEEL, vertical));
                    }
                    if horizontal != 0 {
                        self.scratch
                            .push(mouse_wheel_input(MOUSEEVENTF_HWHEEL, horizontal));
                    }
                }
                // Macro playback timing is owned by the injector thread's `MacroPlayer`
                // ([`macro_player`]), NOT this per-report mapping step. The injector routes
                // `KbmEvent::Macro` edges to `MacroPlayer::on_edge` before/after `flush`, so a macro
                // edge that reaches `build_inputs` (e.g. mixed into a batch) stages no INPUT here.
                KbmEvent::Macro { .. } => {}
                // Special actions are routed to the control plane upstream and never reach the KBM
                // injector; ignore defensively if one slips into a batch.
                KbmEvent::Special { .. } => {}
            }
        }
        self.scratch.len()
    }

    /// Submit whatever is staged in `self.scratch` in one batched `SendInput`, if non-empty.
    fn submit(&mut self) -> Result<(), Error> {
        if self.scratch.is_empty() {
            return Ok(());
        }
        // SAFETY: `SendInput` reads `scratch.len()` consecutive `INPUT` records from `scratch`;
        // the slice is valid for that length and `cbsize` is exactly `size_of::<INPUT>()`. Every
        // record was fully initialized by the `*_input` constructors (all union/struct fields
        // set). A short return (`< len`) means the OS blocked the injection (e.g. UIPI); we map it
        // to `Error::Blocked` so the caller can surface it.
        let sent = unsafe { SendInput(&self.scratch, core::mem::size_of::<INPUT>() as i32) };
        if sent as usize == self.scratch.len() {
            Ok(())
        } else {
            Err(Error::Blocked {
                sent,
                expected: self.scratch.len() as u32,
            })
        }
    }

    /// Stage up-edges for every currently-held key/button into `self.scratch` and clear the
    /// held-state. Submission is left to the caller. Split out of [`release_all`](Self::release_all)
    /// so the staging can be unit-tested without issuing a real `SendInput`.
    ///
    /// This is the one path that may stage more than [`KBM_BATCH_CAP`] records (up to 256 keys + 5
    /// buttons), so it is allowed to grow `scratch` — it runs only off the per-report hot path
    /// (disable / profile-switch / shutdown).
    fn stage_release_all(&mut self) {
        self.scratch.clear();
        // Release every held key (as a scancode up edge, matching how it was pressed).
        for vk in 0u16..256 {
            if self.held_keys[vk as usize] {
                self.held_keys[vk as usize] = false;
                self.scratch.push(key_input(vk, false, KeyKind::ScanCode));
            }
        }
        // Release every held mouse button.
        for btn in [
            MouseButton::Left,
            MouseButton::Right,
            MouseButton::Middle,
            MouseButton::X1,
            MouseButton::X2,
        ] {
            let bit = 1u8 << (btn as u8);
            if self.held_mouse & bit != 0 {
                self.held_mouse &= !bit;
                self.scratch.push(mouse_button_input(btn, false));
            }
        }
    }
}

impl Default for SendInputKbm {
    fn default() -> Self {
        Self::new()
    }
}

impl KbmSink for SendInputKbm {
    type Error = Error;

    fn flush(&mut self, batch: &KbmBatch) -> Result<(), Error> {
        self.build_inputs(batch);
        self.submit()
    }

    fn release_all(&mut self) -> Result<(), Error> {
        self.stage_release_all();
        self.submit()
    }
}

/// A `SendInput` failure surfaced to the injector thread.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// `SendInput` injected fewer events than requested (typically UIPI / a higher-integrity
    /// foreground window blocking synthetic input).
    Blocked {
        /// How many events the OS accepted.
        sent: u32,
        /// How many were submitted.
        expected: u32,
    },
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Blocked { sent, expected } => {
                write!(
                    f,
                    "SendInput injected {sent} of {expected} events (blocked)"
                )
            }
        }
    }
}

impl std::error::Error for Error {}

// --- INPUT record constructors (each fully initializes every field) ---------------------------

/// Build a keyboard `INPUT` for one key edge, resolving scancode + extended-key flags from `vk`.
fn key_input(vk: u16, down: bool, kind: KeyKind) -> INPUT {
    let (scancode, extended) = scancode_from_vk(vk);
    let mut flags = KEYBD_EVENT_FLAGS(0);
    let (wvk, wscan) = match kind {
        KeyKind::ScanCode => {
            // Scancode injection: VK is left 0, the hardware scancode carries the key.
            flags |= KEYEVENTF_SCANCODE;
            (0u16, scancode)
        }
        KeyKind::Virtual => {
            // VK injection: the OS derives the scancode; we still pass it for fidelity.
            (vk, scancode)
        }
    };
    if extended {
        flags |= KEYEVENTF_EXTENDEDKEY;
    }
    if !down {
        flags |= KEYEVENTF_KEYUP;
    }
    INPUT {
        r#type: INPUT_KEYBOARD,
        Anonymous: INPUT_0 {
            ki: KEYBDINPUT {
                wVk: VIRTUAL_KEY(wvk),
                wScan: wscan,
                dwFlags: flags,
                time: 0,
                dwExtraInfo: 0,
            },
        },
    }
}

/// Build a mouse-button `INPUT` for one button edge.
fn mouse_button_input(btn: MouseButton, down: bool) -> INPUT {
    // For X1/X2 the button is selected via `mouseData`; the up/down flag is the generic XBUTTON.
    let (flags, mouse_data) = match (btn, down) {
        (MouseButton::Left, true) => (MOUSEEVENTF_LEFTDOWN, 0),
        (MouseButton::Left, false) => (MOUSEEVENTF_LEFTUP, 0),
        (MouseButton::Right, true) => (MOUSEEVENTF_RIGHTDOWN, 0),
        (MouseButton::Right, false) => (MOUSEEVENTF_RIGHTUP, 0),
        (MouseButton::Middle, true) => (MOUSEEVENTF_MIDDLEDOWN, 0),
        (MouseButton::Middle, false) => (MOUSEEVENTF_MIDDLEUP, 0),
        (MouseButton::X1, true) => (MOUSEEVENTF_XDOWN, XBUTTON1),
        (MouseButton::X1, false) => (MOUSEEVENTF_XUP, XBUTTON1),
        (MouseButton::X2, true) => (MOUSEEVENTF_XDOWN, XBUTTON2),
        (MouseButton::X2, false) => (MOUSEEVENTF_XUP, XBUTTON2),
    };
    mouse_input(flags, mouse_data, 0, 0)
}

/// Build a relative mouse-move `INPUT` (no `MOUSEEVENTF_ABSOLUTE` => delta).
fn mouse_move_input(dx: i32, dy: i32) -> INPUT {
    mouse_input(MOUSEEVENTF_MOVE, 0, dx, dy)
}

/// Build a wheel `INPUT`; `flags` is `MOUSEEVENTF_WHEEL` or `MOUSEEVENTF_HWHEEL`, `amount` is in
/// `WHEEL_DELTA` (±120) units packed into `mouseData` (signed -> u32 wrap, as the API expects).
fn mouse_wheel_input(flags: MOUSE_EVENT_FLAGS, amount: i32) -> INPUT {
    mouse_input(flags, amount as u32, 0, 0)
}

/// Shared mouse `INPUT` constructor: all fields explicitly initialized.
fn mouse_input(flags: MOUSE_EVENT_FLAGS, mouse_data: u32, dx: i32, dy: i32) -> INPUT {
    INPUT {
        r#type: INPUT_MOUSE,
        Anonymous: INPUT_0 {
            mi: MOUSEINPUT {
                dx,
                dy,
                mouseData: mouse_data,
                dwFlags: flags,
                time: 0,
                dwExtraInfo: 0,
            },
        },
    }
}

#[cfg(test)]
mod tests {
    // `KBM_BATCH_CAP` and all `windows` flag constants come in via `super::*`.
    use super::*;

    /// Read the keyboard payload of a staged `INPUT`. Safe in test: the record was built by
    /// `key_input`, so the `ki` union arm is the initialized one.
    fn ki(input: &INPUT) -> KEYBDINPUT {
        assert_eq!(input.r#type, INPUT_KEYBOARD);
        // SAFETY: keyboard records set the `ki` union arm.
        unsafe { input.Anonymous.ki }
    }
    /// Read the mouse payload of a staged `INPUT`.
    fn mi(input: &INPUT) -> MOUSEINPUT {
        assert_eq!(input.r#type, INPUT_MOUSE);
        // SAFETY: mouse records set the `mi` union arm.
        unsafe { input.Anonymous.mi }
    }

    #[test]
    fn scancode_extended_set_decodes_from_c_sharp_list() {
        // VK_PAUSE special-case.
        assert_eq!(scancode_from_vk(0x13), (0x45, false));
        // A known extended VK (VK_RIGHT) reports extended; its scancode is whatever MapVirtualKey
        // gives, but the extended flag is what we pin.
        assert!(scancode_from_vk(0x27).1, "VK_RIGHT is extended");
        // A plain key (VK_A) is not extended.
        assert!(!scancode_from_vk(0x41).1, "VK_A is not extended");
    }

    #[test]
    fn builds_scratch_inputs_from_synthetic_batch() {
        let mut sink = SendInputKbm::new();
        let mut batch = KbmBatch::new();
        // key down (scancode), mouse left down, relative move, vertical wheel.
        batch.push(KbmEvent::Key {
            vk: 0x41,
            down: true,
            kind: KeyKind::ScanCode,
        });
        batch.push(KbmEvent::MouseButton {
            btn: MouseButton::Left,
            down: true,
        });
        batch.push(KbmEvent::MouseMove { dx: 5, dy: -3 });
        batch.push(KbmEvent::Wheel {
            vertical: 120,
            horizontal: 0,
        });

        let n = sink.build_inputs(&batch);
        assert_eq!(n, 4, "one INPUT per non-deduped event");
        assert_eq!(sink.scratch.len(), 4);

        // [0] key down: scancode flag set, KEYUP not set, wVk zeroed (scancode mode).
        let k = ki(&sink.scratch[0]);
        assert!(k.dwFlags & KEYEVENTF_SCANCODE != KEYBD_EVENT_FLAGS(0));
        assert!(k.dwFlags & KEYEVENTF_KEYUP == KEYBD_EVENT_FLAGS(0));
        assert_eq!(k.wVk, VIRTUAL_KEY(0));

        // [1] left mouse down.
        let m1 = mi(&sink.scratch[1]);
        assert_eq!(m1.dwFlags, MOUSEEVENTF_LEFTDOWN);

        // [2] relative move carries the delta with no ABSOLUTE flag.
        let m2 = mi(&sink.scratch[2]);
        assert_eq!(m2.dwFlags, MOUSEEVENTF_MOVE);
        assert_eq!((m2.dx, m2.dy), (5, -3));

        // [3] vertical wheel, +120 into mouseData.
        let m3 = mi(&sink.scratch[3]);
        assert_eq!(m3.dwFlags, MOUSEEVENTF_WHEEL);
        assert_eq!(m3.mouseData, 120);

        // Held state now tracks the pressed key + button.
        assert!(sink.held_keys[0x41]);
        assert_eq!(sink.held_mouse, 1 << (MouseButton::Left as u8));
    }

    #[test]
    fn edge_dedupe_drops_redundant_down_and_release() {
        let mut sink = SendInputKbm::new();
        let mut batch = KbmBatch::new();
        // Two downs for the same key: only the first stages an INPUT.
        batch.push(KbmEvent::Key {
            vk: 0x42,
            down: true,
            kind: KeyKind::ScanCode,
        });
        batch.push(KbmEvent::Key {
            vk: 0x42,
            down: true,
            kind: KeyKind::ScanCode,
        });
        assert_eq!(sink.build_inputs(&batch), 1, "second down is deduped");

        // A release for a key that is not held is dropped.
        let mut rel = KbmBatch::new();
        rel.push(KbmEvent::Key {
            vk: 0x99,
            down: false,
            kind: KeyKind::ScanCode,
        });
        assert_eq!(sink.build_inputs(&rel), 0, "release of un-held key dropped");

        // A real release for the held key stages exactly one KEYUP and clears held state.
        let mut rel2 = KbmBatch::new();
        rel2.push(KbmEvent::Key {
            vk: 0x42,
            down: false,
            kind: KeyKind::ScanCode,
        });
        assert_eq!(sink.build_inputs(&rel2), 1);
        assert!(ki(&sink.scratch[0]).dwFlags & KEYEVENTF_KEYUP != KEYBD_EVENT_FLAGS(0));
        assert!(!sink.held_keys[0x42]);
    }

    #[test]
    fn fake_key_repeat_re_emits_held_key() {
        let mut sink = SendInputKbm::new();
        sink.set_fake_key_repeat(true);
        let mut batch = KbmBatch::new();
        batch.push(KbmEvent::Key {
            vk: 0x43,
            down: true,
            kind: KeyKind::ScanCode,
        });
        batch.push(KbmEvent::Key {
            vk: 0x43,
            down: true,
            kind: KeyKind::ScanCode,
        });
        assert_eq!(
            sink.build_inputs(&batch),
            2,
            "with fake_key_repeat both downs are emitted"
        );
    }

    #[test]
    fn release_all_stages_up_edges_for_held_state_only() {
        let mut sink = SendInputKbm::new();
        let mut batch = KbmBatch::new();
        batch.push(KbmEvent::Key {
            vk: 0x41,
            down: true,
            kind: KeyKind::ScanCode,
        });
        batch.push(KbmEvent::MouseButton {
            btn: MouseButton::Right,
            down: true,
        });
        sink.build_inputs(&batch);

        // Stage the release (no SendInput) and inspect: exactly one key-up + one mouse-up, and the
        // held-state is cleared.
        sink.stage_release_all();
        assert_eq!(sink.scratch.len(), 2);
        assert!(ki(&sink.scratch[0]).dwFlags & KEYEVENTF_KEYUP != KEYBD_EVENT_FLAGS(0));
        assert_eq!(mi(&sink.scratch[1]).dwFlags, MOUSEEVENTF_RIGHTUP);
        assert!(!sink.held_keys[0x41]);
        assert_eq!(sink.held_mouse, 0);

        // A second staging is a clean no-op (nothing held -> nothing staged).
        sink.stage_release_all();
        assert!(sink.scratch.is_empty());
    }

    #[test]
    fn scratch_is_presized_to_cap_and_not_grown_by_a_full_batch() {
        let mut sink = SendInputKbm::new();
        assert!(sink.scratch.capacity() >= KBM_BATCH_CAP);
        let cap_before = sink.scratch.capacity();

        // Fill a batch to its cap with distinct down-edges so none dedupe away.
        let mut batch = KbmBatch::new();
        for i in 0..KBM_BATCH_CAP as u16 {
            batch.push(KbmEvent::Key {
                vk: 0x41 + i,
                down: true,
                kind: KeyKind::ScanCode,
            });
        }
        let n = sink.build_inputs(&batch);
        assert_eq!(n, KBM_BATCH_CAP);
        assert_eq!(
            sink.scratch.capacity(),
            cap_before,
            "a full batch never reallocates the scratch buffer"
        );
    }

    #[test]
    fn x_button_uses_mousedata_selector() {
        let mut sink = SendInputKbm::new();
        let mut batch = KbmBatch::new();
        batch.push(KbmEvent::MouseButton {
            btn: MouseButton::X2,
            down: true,
        });
        sink.build_inputs(&batch);
        let m = mi(&sink.scratch[0]);
        assert_eq!(m.dwFlags, MOUSEEVENTF_XDOWN);
        assert_eq!(m.mouseData, XBUTTON2);
    }

    #[test]
    fn builds_all_mouse_event_kinds_into_scratch() {
        // M4 mouse-injection contract: a synthetic batch carrying a relative move, both wheel axes,
        // and every mouse button stages the right INPUT records (no real SendInput). Pins the
        // MOUSEEVENTF flags + WHEEL_DELTA/XBUTTON packing for the mouse-from-stick / wheel / button
        // bindings the M4 engine emits.
        let mut sink = SendInputKbm::new();
        let mut batch = KbmBatch::new();
        batch.push(KbmEvent::MouseMove { dx: -7, dy: 11 });
        batch.push(KbmEvent::Wheel {
            vertical: 120,
            horizontal: -120,
        });
        batch.push(KbmEvent::MouseButton {
            btn: MouseButton::Middle,
            down: true,
        });
        batch.push(KbmEvent::MouseButton {
            btn: MouseButton::X1,
            down: true,
        });

        // move(1) + wheel vert(1) + wheel horiz(1) + middle(1) + x1(1) == 5 records.
        let n = sink.build_inputs(&batch);
        assert_eq!(n, 5);

        // [0] relative move carries the signed delta, no ABSOLUTE flag.
        let mv = mi(&sink.scratch[0]);
        assert_eq!(mv.dwFlags, MOUSEEVENTF_MOVE);
        assert_eq!((mv.dx, mv.dy), (-7, 11));
        // [1] vertical wheel (+120 == one notch up) via MOUSEEVENTF_WHEEL.
        let wv = mi(&sink.scratch[1]);
        assert_eq!(wv.dwFlags, MOUSEEVENTF_WHEEL);
        assert_eq!(wv.mouseData as i32, 120);
        // [2] horizontal wheel (-120) via MOUSEEVENTF_HWHEEL; the signed amount wraps into u32.
        let wh = mi(&sink.scratch[2]);
        assert_eq!(wh.dwFlags, MOUSEEVENTF_HWHEEL);
        assert_eq!(wh.mouseData as i32, -120);
        // [3] middle-button down.
        assert_eq!(mi(&sink.scratch[3]).dwFlags, MOUSEEVENTF_MIDDLEDOWN);
        // [4] X1 down selects the button via mouseData == XBUTTON1.
        let x1 = mi(&sink.scratch[4]);
        assert_eq!(x1.dwFlags, MOUSEEVENTF_XDOWN);
        assert_eq!(x1.mouseData, XBUTTON1);

        // release_all lifts the two held buttons (move/wheel hold nothing).
        sink.stage_release_all();
        assert_eq!(sink.scratch.len(), 2, "two held buttons released");
        assert_eq!(sink.held_mouse, 0);
    }

    #[test]
    fn macro_and_special_events_stage_nothing() {
        let mut sink = SendInputKbm::new();
        let mut batch = KbmBatch::new();
        batch.push(KbmEvent::Macro { id: 7, start: true });
        batch.push(KbmEvent::Special { id: 3 });
        assert_eq!(sink.build_inputs(&batch), 0);
    }
}
