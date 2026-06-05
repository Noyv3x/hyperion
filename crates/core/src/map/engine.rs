//! The pure mapping engine — [`apply`] and its resident [`MapState`].
//!
//! [`apply`] is the single remap entry point: pure, alloc-free, no I/O, deterministic. It reads a
//! decoded [`ControllerState`], resolves every `Control` against an immutable
//! [`ResolvedProfile`](crate::map::ResolvedProfile), composes an [`OutputState`], and queues
//! keyboard/mouse edges into a [`KbmBatch`]. It runs inline on the hot loop exactly where the
//! single `RcFilter` step ran (blueprint §7.2).
//!
//! ## M3 subset (now extended by M4)
//! M3 implemented the spine: per-kind **digitize** (thresholds 55/127 vs 100/255 via
//! [`ControllerState::pressed`]), **half-axis identity suppression** (the `axis_remapped[4]` pass
//! — verifier FIX 1), and per-control resolve for **`Passthrough` + `GamepadButton` + `Key`**
//! (button→button, button→key with an edge/toggle vk-keyed latch — verifier FIX 6).
//!
//! ## M4 additions (blueprint §5, §12 M4)
//! * **Per-control shift selection** (step 2, verifier FIX 4b): each control's effective slot is its
//!   `shift_bind` when its `shift_trigger` reads pressed in the RAW digitized state, else its base
//!   bind. Reads RAW only (never output → no feedback); supports distinct simultaneous triggers.
//! * **Turbo** ([`turbo_gate`]): the per-control `on` is gated through the slot's [`TurboCfg`] with
//!   the phase anchored to press time (net-new, no DS4Windows golden — verifier FIX 2).
//! * **Mouse**: `Mouse(button)` → edge → [`KbmEvent::MouseButton`]; `MouseMove(src)` → feed the
//!   resident [`MouseAccumulator`] → [`KbmEvent::MouseMove`] (verifier FIX 7); `MouseWheel(dir)` →
//!   notch remainder → [`KbmEvent::Wheel`].
//! * **Macro**: `Macro(id)` → on/off edge → [`KbmEvent::Macro`].
//! * **Special**: `Special(id)` → edge → [`KbmEvent::Special`] (drained by the control plane).
//! * **`GamepadAxis` / `TouchpadClick`**: digital→axis push and the touchpad output bit.
//!
//! ## M5 additions (blueprint §12 M5)
//! * **Gyro→mouse scaling**: `MouseMove(MouseMoveSrc::Gyro)` now drives `ms.gyro_mouse` through the
//!   full DS4Windows velocity model via [`MouseAccumulator::gyro_velocity_step`], using the resolved
//!   [`GyroAccumCfg`](crate::mouse_accum::GyroAccumCfg) from `rp.gyro.to_accum_cfg()` (M4 passed raw
//!   rad/s through the carry-only path; M5 scales it). The profile's
//!   [`GyroMode`](crate::map::profile::GyroMode) gates the feed (`Off` keeps it inert) and
//!   `swap_yaw_roll` selects the horizontal axis.
//! * **Mouse-from-stick** now reads the resolved `rp.mouse` (M4 used the default), still
//!   byte-identical for an unconfigured profile.
//!
//! ## FLICK-STICK CONSUMER CONTRACT (for the engine/hot-loop integration agent — blueprint §4
//! stage 9, §12 M5)
//!
//! The resolved **stick pipeline** (`stick::pipeline::process_stick`) runs in the engine
//! (`hot.rs`), NOT inside [`apply`]. Its terminal stage 9 stashes the per-report relative-aim turn
//! in `StickState::flick_delta` (`f64`, the absolute stick is returned unchanged). `apply()` has no
//! access to that pipeline state, so the **engine routes the flick delta into the mouse output**,
//! not `apply()`. The chosen contract (cleanest of the two options — the engine adds it post-apply,
//! so [`apply`]'s pure signature is untouched and the stick pipeline stays the sole owner of flick
//! state):
//!
//! 1. After `process_stick` for each stick, the engine holds `flick_dx = stick_state[i].flick_delta`
//!    (LS and RS each have their own; sum or pick per the active flick binding — typically only one
//!    stick has flick enabled).
//! 2. After `let (out, mut kbm) = apply(&state, rp, &mut map_state, now_us);`, the engine converts
//!    the flick delta to an integer mouse-X delta and **merges it into the same `KbmBatch`**:
//!    `if flick_dx != 0.0 { let dx = flick_to_px(flick_dx); if dx != 0 { kbm.push(KbmEvent::MouseMove { dx, dy: 0 }); } }`
//!    (flick is a horizontal turn; `dy` stays 0). Use a dedicated remainder carry in `MapState`
//!    (e.g. reuse `ms.gyro_mouse` is NOT correct — add a small `flick_remainder: f64` field if
//!    sub-pixel carry is wanted; the flick delta is already a large turn so a plain
//!    `round()`/`trunc()` is acceptable for M5, documented as a follow-up to add carry).
//! 3. The `flick_to_px` scale is the reserved OneEuro/real-world-calibration contract (blueprint
//!    §14): `flick_delta` is the relative turn in the stick pipeline's angular unit; multiply by the
//!    profile's flick sensitivity (a `StickSettings::flick` field) to get pixels. Until that scale is
//!    tuned on hardware, a 1:1 pass-through (`dx = flick_delta.round() as i32`) keeps the wiring
//!    exercised. The mouse-move event is the SAME `KbmEvent::MouseMove` variant the stick/gyro path
//!    emits, so the KBM injector needs no new event type.
//!
//! Rationale for "engine adds it post-apply" over "apply() accepts the flick delta": `apply()` is
//! the pure, alloc-free remap of a `ControllerState`; the flick delta is a *derived stick-pipeline
//! output*, not a raw control. Threading it through `apply()` would couple the pure engine to the
//! stick pipeline's resident state. Merging into the returned `KbmBatch` keeps `apply()`'s signature
//! and every M3/M4 golden untouched while still funnelling all mouse motion through one batch.

use crate::input::{Control, ControlKind, ControllerState};
use crate::map::binding::{
    AxisDir, BindTarget, BindingSlot, GamepadAxis, KeyKind, MouseMoveSrc, TurboCfg, WheelDir,
};
use crate::map::profile::ResolvedProfile;
use crate::mouse_accum::{GyroAccumCfg, MouseAccumCfg, TouchAccumCfg};
use crate::output::{KbmBatch, KbmEvent, KeyKind as InjectKind, MouseButton, OutputState};

/// Re-export the real [`MouseAccumulator`](crate::mouse_accum::MouseAccumulator) so the historical
/// `map::engine::MouseAccumulator` / `map::MouseAccumulator` paths (and [`MapState`]'s fields) keep
/// resolving after the M4 implementation moved into [`crate::mouse_accum`]. The M3 placeholder that
/// lived here is gone; this is the single source of truth.
pub use crate::mouse_accum::MouseAccumulator;

/// Fixed-capacity vk→bool latch (verifier FIX 6) — alloc-free, `Copy`.
///
/// DS4Windows keys its `pressedonce[]` / `toggle` state on the **output key value** (vk), so two
/// controls bound to the same vk share one latch. We reproduce that with a tiny open-addressed
/// fixed-cap table: insert is O(cap) worst case but cap is small and the common case is a handful
/// of held keys. On overflow the latch silently no-ops (saturate, never alloc/panic) — at that
/// point far more keys are held than a HID report can carry anyway.
#[derive(Clone, Copy, Debug)]
pub struct VkLatch {
    keys: [u16; Self::CAP],
    vals: [bool; Self::CAP],
    /// Number of live entries.
    len: u8,
}

impl VkLatch {
    /// Distinct vks tracked at once. Sized well above the 6-key HID hold limit + shift churn.
    pub const CAP: usize = 16;

    /// An empty latch (clean post-reset state).
    pub const fn new() -> Self {
        Self {
            keys: [0; Self::CAP],
            vals: [false; Self::CAP],
            len: 0,
        }
    }

    /// Index of `vk` if present.
    #[inline]
    fn find(&self, vk: u16) -> Option<usize> {
        let n = self.len as usize;
        let mut i = 0;
        while i < n {
            if self.keys[i] == vk {
                return Some(i);
            }
            i += 1;
        }
        None
    }

    /// Current latched value for `vk` (default `false` if untracked).
    #[inline]
    pub fn get(&self, vk: u16) -> bool {
        match self.find(vk) {
            Some(i) => self.vals[i],
            None => false,
        }
    }

    /// Set `vk`'s latched value, inserting a slot if needed. Silently no-ops on overflow.
    #[inline]
    pub fn set(&mut self, vk: u16, on: bool) {
        if let Some(i) = self.find(vk) {
            self.vals[i] = on;
            return;
        }
        let n = self.len as usize;
        if n < Self::CAP {
            self.keys[n] = vk;
            self.vals[n] = on;
            self.len = n as u8 + 1;
        }
        // overflow: drop silently (bounded, deterministic, alloc-free).
    }

    /// Flip `vk`'s latched value and return the new value (inserting `true` if untracked).
    #[inline]
    pub fn toggle(&mut self, vk: u16) -> bool {
        let new = !self.get(vk);
        self.set(vk, new);
        new
    }
}

impl Default for VkLatch {
    fn default() -> Self {
        Self::new()
    }
}

/// Per-binding turbo phase state (net-new; gate runs in M4). Verifier FIX 2/6 — `Default` is the
/// clean post-reset state.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TurboState {
    /// Was the source active on the previous report (rising-edge detect).
    pub was_active: bool,
    /// `now_us` of the current cycle's anchor (reset on each rising edge).
    pub cycle_start_us: u64,
}

/// Resident mapping state — all `Copy`, all fixed-size arrays, **no heap on the hot path**
/// (blueprint §5). `Default` is the clean post-reset state (verifier FIX 6): the engine's
/// `ResetFilter` command sets `*ms = MapState::default()`.
#[derive(Clone, Copy, Debug)]
pub struct MapState {
    /// Per-control turbo phase (M4 consumer).
    pub turbo: [TurboState; Control::COUNT],
    /// Per-control "was on last report" for edge detection.
    pub prev_active: [bool; Control::COUNT],
    /// Stick→mouse remainder carry (M4 consumer).
    pub stick_mouse: MouseAccumulator,
    /// Gyro→mouse remainder carry (M5 consumer).
    pub gyro_mouse: MouseAccumulator,
    /// Touchpad→mouse remainder carry (M6 consumer).
    pub touch_mouse: MouseAccumulator,
    /// The finger-0 [`TouchContact`](crate::input::TouchContact) from the previous report, the
    /// delta anchor the touchpad→mouse accumulator differences against (M6 consumer). `Default`
    /// (inactive) means "no previous contact", so the first touch report produces no jump.
    pub prev_touch: crate::input::TouchContact,
    /// Mouse-wheel notch remainder (M4 consumer).
    pub wheel_remainder: f64,
    /// vk→toggle-latched-value (verifier FIX 6).
    pub toggle: VkLatch,
    /// vk→pressed-once edge guard (verifier FIX 6).
    pub pressed_once: VkLatch,
    /// `now_us` of the previous [`apply`] call, used to derive the per-report `dt` the mouse
    /// accumulator needs (the `apply()` signature carries only `now_us`, so the interval is
    /// reconstructed here — see [`apply`]'s mouse-move handling). `0` means "no previous report"
    /// (first call → a `dt` of 0 → the offset term is the only contribution, matching a cold start).
    pub prev_now_us: u64,
}

impl Default for MapState {
    fn default() -> Self {
        Self {
            turbo: [TurboState::default(); Control::COUNT],
            prev_active: [false; Control::COUNT],
            stick_mouse: MouseAccumulator::new(),
            gyro_mouse: MouseAccumulator::new(),
            touch_mouse: MouseAccumulator::new(),
            prev_touch: crate::input::TouchContact::default(),
            wheel_remainder: 0.0,
            toggle: VkLatch::new(),
            pressed_once: VkLatch::new(),
            prev_now_us: 0,
        }
    }
}

/// Write a half-axis identity passthrough into `out` for the given stick `axis`.
///
/// Called only when `!axis_remapped[axis]` (verifier FIX 1 — `ResetToDefaultValue` zeroes BOTH
/// halves of an axis if either is remapped). The full continuous stick value is the signed `state`
/// component; writing it for either half of the pair is idempotent (both halves of a pair map to
/// the same axis value), so the second half's Passthrough arm is a harmless re-write of the same
/// value.
#[inline]
fn passthrough_axis(out: &mut OutputState, state: &ControllerState, axis: usize) {
    match axis {
        0 => out.lx = state.lx,
        1 => out.ly = state.ly,
        2 => out.rx = state.rx,
        _ => out.ry = state.ry,
    }
}

/// Resolve a `Key` binding: edge/toggle latch keyed on the **output vk** (verifier FIX 6), pushing
/// `KbmEvent::Key{down}` edges into `batch`. Hold emits nothing after the first report.
#[inline]
fn resolve_key(
    vk: u16,
    kind: KeyKind,
    on: bool,
    prev_on: bool,
    ms: &mut MapState,
    batch: &mut KbmBatch,
) {
    // The mapping-layer `KeyKind` (scan_code + toggle) selects injection method; the toggle bit is
    // handled below, and the KBM injector only needs ScanCode vs Virtual.
    let inject = if kind.scan_code {
        InjectKind::ScanCode
    } else {
        InjectKind::Virtual
    };
    if kind.toggle {
        // Toggle: flip the latched value on the rising edge only; emit the new state as an edge.
        if on && !ms.pressed_once.get(vk) {
            let now = ms.toggle.toggle(vk);
            ms.pressed_once.set(vk, true);
            batch.push(KbmEvent::Key {
                vk,
                down: now,
                kind: inject,
            });
        } else if !on {
            ms.pressed_once.set(vk, false);
        }
    } else {
        // Hold: KeyDown on rising edge, KeyUp on falling edge, nothing while held.
        if on && !prev_on {
            batch.push(KbmEvent::Key {
                vk,
                down: true,
                kind: inject,
            });
        } else if !on && prev_on {
            batch.push(KbmEvent::Key {
                vk,
                down: false,
                kind: inject,
            });
        }
    }
}

/// The net-new turbo / rapid-fire gate (blueprint §5, verifier FIX 2 — no DS4Windows golden).
///
/// While the source is held, the output toggles ON/OFF on a `period_us` cycle with an ON fraction
/// of `duty_num/duty_den`. The phase is **anchored to the press time** (`cycle_start_us` reset on
/// each rising edge) so a fresh press always begins with a full ON window. No float and no division
/// on the gate decision — the duty comparison is a single `u128` cross-multiply. Releasing the
/// source clears `was_active`, so the next press re-anchors.
#[inline]
fn turbo_gate(ts: &mut TurboState, src_on: bool, t: TurboCfg, now_us: u64) -> bool {
    if !src_on {
        ts.was_active = false;
        return false;
    }
    if !ts.was_active {
        ts.cycle_start_us = now_us;
        ts.was_active = true;
    }
    let period = t.period_us.max(1) as u64;
    let phase = now_us.wrapping_sub(ts.cycle_start_us) % period;
    // ON while phase/period < duty_num/duty_den, i.e. phase·duty_den < period·duty_num. u128 to
    // avoid overflow on large periods (period_us up to ~4e9, duty_den up to 65535).
    u128::from(phase) * u128::from(t.duty_den) < u128::from(period) * u128::from(t.duty_num)
}

/// Select a control's **effective** binding under the per-control shift model (blueprint §5 step 2,
/// verifier FIX 4b). If the slot carries a `shift_trigger` whose control reads pressed in the RAW
/// digitized state, the `shift_bind` applies; otherwise the base `bind`. Reads RAW only — never
/// output — so there is no feedback loop, and distinct controls can be shifted by distinct triggers
/// simultaneously.
#[inline]
fn effective_bind(slot: &BindingSlot, active_raw: &[bool; Control::COUNT]) -> BindTarget {
    match slot.shift_trigger {
        Some(trig) if active_raw[trig.control.as_index()] => slot.shift_bind,
        _ => slot.bind,
    }
}

/// Push a digital→axis deflection into `out` for a `GamepadAxis` binding. A digital `on` source
/// drives the axis fully toward `dir`. `full` is reserved for a future scaled-push variant for
/// analog sources; a digital edge always pushes ±1 (the analog passthrough path owns continuous
/// axes), so it is accepted-and-ignored here for signature stability.
#[inline]
fn resolve_gamepad_axis(
    out: &mut OutputState,
    axis: GamepadAxis,
    dir: AxisDir,
    full: bool,
    on: bool,
) {
    let _ = full;
    if !on {
        return;
    }
    let signed = match dir {
        AxisDir::Pos => 1.0,
        AxisDir::Neg => -1.0,
    };
    match axis {
        GamepadAxis::Lx => out.lx = signed,
        GamepadAxis::Ly => out.ly = signed,
        GamepadAxis::Rx => out.rx = signed,
        GamepadAxis::Ry => out.ry = signed,
        // Triggers are unipolar [0,1]: a push is full-on regardless of the dir sign.
        GamepadAxis::Lt => out.lt = 1.0,
        GamepadAxis::Rt => out.rt = 1.0,
        GamepadAxis::Unknown => {}
    }
}

/// The per-report mouse-move tunables resolved once from the profile (kept as a small `Copy` bundle
/// so [`resolve_mouse_move`] stays under the argument-count limit and the cfgs are computed once per
/// `apply`, not per `MouseMove`-bound control).
#[derive(Clone, Copy)]
struct MouseMoveCfg {
    /// Stick→mouse tunables.
    stick: MouseAccumCfg,
    /// Gyro→mouse velocity-model tunables.
    gyro: GyroAccumCfg,
    /// Touchpad→mouse tunables (M6).
    touch: TouchAccumCfg,
    /// Read roll instead of yaw for the gyro horizontal axis.
    swap_yaw_roll: bool,
    /// Whether touchpad→mouse is enabled (`TouchpadSettings::as_mouse`); when `false` a
    /// `MouseMove(Touchpad)` binding is inert.
    touch_as_mouse: bool,
}

/// Resolve a mouse-move binding: feed the resident [`MouseAccumulator`] from the named source's
/// signed stick deflection (or scaled gyro rate) and push the integer `(dx, dy)` as a
/// [`KbmEvent::MouseMove`] (verifier FIX 7). Emits nothing when the delta is `(0, 0)` so the batch
/// stays sparse.
///
/// **M5 gyro scaling.** The `Gyro` source now runs the full DS4Windows velocity model
/// ([`MouseAccumulator::gyro_velocity_step`]) using the resolved [`GyroAccumCfg`] from the profile's
/// [`GyroSettings`](crate::map::profile::GyroSettings) — sensitivity, rate-domain dead-zone,
/// vertical-scale, jitter compensation, and invert. The horizontal axis is yaw (or roll when
/// `swap_yaw_roll` is set), and the vertical channel is `-pitch` so a nose-up tilt looks up. The
/// caller (`apply`) is responsible for the activation gate ([`GyroMode`](crate::map::profile::GyroMode));
/// this routine only runs when the gyro feed is active.
#[inline]
fn resolve_mouse_move(
    src: MouseMoveSrc,
    state: &ControllerState,
    cfg: &MouseMoveCfg,
    elapsed_s: f64,
    ms: &mut MapState,
    batch: &mut KbmBatch,
) {
    let (dx, dy) = match src {
        MouseMoveSrc::LeftStick => ms
            .stick_mouse
            .stick_step(state.lx, state.ly, elapsed_s, &cfg.stick),
        MouseMoveSrc::RightStick => ms
            .stick_mouse
            .stick_step(state.rx, state.ry, elapsed_s, &cfg.stick),
        MouseMoveSrc::Gyro => {
            // Horizontal: yaw, or roll when the axis is swapped (DS4Windows getGyroMouseHorizontalAxis).
            // Vertical: -pitch (C# `deltaY = -gyroPitch`) so nose-up looks up. The GyroAccumCfg owns
            // the M5 velocity scaling, rate-domain dead-zone, vertical-scale, jitter, and invert.
            let gx = if cfg.swap_yaw_roll {
                state.motion.gyro_roll
            } else {
                state.motion.gyro_yaw
            };
            let gz = -state.motion.gyro_pitch;
            ms.gyro_mouse
                .gyro_velocity_step(gx, gz, elapsed_s, &cfg.gyro)
        }
        MouseMoveSrc::Touchpad => {
            // Touchpad finger drag → relative mouse (M6). Inert unless `touchpad.as_mouse` is set.
            // The delta is finger 0 between this report (`state.touch[0]`) and the previous
            // (`ms.prev_touch`); `touch_step` resets on a finger lift / id change so a touch-down
            // never jumps the cursor. `prev_touch` is advanced once per report by `apply`, not here.
            if cfg.touch_as_mouse {
                ms.touch_mouse
                    .touch_step(ms.prev_touch, state.touch[0], &cfg.touch)
            } else {
                (0, 0)
            }
        }
        MouseMoveSrc::Unknown => (0, 0),
    };
    if dx != 0 || dy != 0 {
        batch.push(KbmEvent::MouseMove { dx, dy });
    }
}

/// Resolve a mouse-wheel binding while the source is held: accumulate a fractional notch into
/// `ms.wheel_remainder` and emit whole `WHEEL_DELTA` (±120) ticks (ports DS4Windows
/// `GetMouseWheelMapping`/`stickWheelRemainder`). A full deflection scrolls ~one notch per three
/// reports; partial deflection scrolls proportionally slower. Off-edge resets the remainder.
#[inline]
fn resolve_mouse_wheel(
    dir: WheelDir,
    on: bool,
    prev_on: bool,
    ms: &mut MapState,
    batch: &mut KbmBatch,
) {
    if !on {
        if prev_on {
            ms.wheel_remainder = 0.0;
        }
        return;
    }
    // C#: ratio = (1-0.05)*(value/255)+0.05, currentWheel = ratio/3. A held digital source is
    // value==255 → ratio==1 → 1/3 of a notch per report; three reports == one tick.
    let current = 1.0 / 3.0;
    let wheel = current + ms.wheel_remainder;
    if wheel >= 1.0 {
        let ticks = wheel.trunc();
        ms.wheel_remainder = wheel - ticks;
        let notches = ticks as i32 * 120;
        let (vertical, horizontal) = match dir {
            WheelDir::Up => (notches, 0),
            WheelDir::Down => (-notches, 0),
            WheelDir::Right => (0, notches),
            WheelDir::Left => (0, -notches),
            WheelDir::Unknown => (0, 0),
        };
        if vertical != 0 || horizontal != 0 {
            batch.push(KbmEvent::Wheel {
                vertical,
                horizontal,
            });
        }
    } else {
        ms.wheel_remainder = wheel;
    }
}

/// Resolve the mouse-from-stick settings carried by the resolved profile into the accumulator's
/// tunable form. For a default (unconfigured) profile this equals [`MouseAccumCfg::default`]
/// (pinned by `profile::tests::mouse_settings_default_matches_accum_default`), so the M4 mouse
/// goldens stay byte-identical.
#[inline]
fn mouse_cfg(rp: &ResolvedProfile) -> MouseAccumCfg {
    rp.mouse.to_accum_cfg()
}

/// Resolve the gyro→mouse settings carried by the resolved profile into the gyro accumulator's
/// tunable form (M5). The activation mode + horizontal-axis swap are read separately by `apply()`;
/// this carries only the velocity-model tunables.
#[inline]
fn gyro_cfg(rp: &ResolvedProfile) -> GyroAccumCfg {
    rp.gyro.to_accum_cfg()
}

/// Pure, alloc-free, no-I/O remap entry point (blueprint §5).
///
/// Resolves every `Control` against `rp`, composing an [`OutputState`] and queueing KBM edges.
/// `now_us` is the hot loop's monotonic time (reused from `busy_start/1000`).
///
/// Resolution order (blueprint §5): digitize once → per-control shift selection → half-axis identity
/// suppression → per-control resolve (turbo-gated `on`, then the bind's output/edge). M4 resolves
/// every `BindTarget` arm; only `Shift`/`Unbound`/`KeyUnbound`/`Unknown` are intentionally silent.
pub fn apply(
    state: &ControllerState,
    rp: &ResolvedProfile,
    ms: &mut MapState,
    now_us: u64,
) -> (OutputState, KbmBatch) {
    let t = &rp.thresholds;

    // Per-report dt for the mouse accumulator, reconstructed from the monotonic `now_us` (the
    // signature carries no dt). First call (prev==0) yields dt 0 → only the offset term contributes.
    let elapsed_s = if ms.prev_now_us == 0 || now_us <= ms.prev_now_us {
        0.0
    } else {
        (now_us - ms.prev_now_us) as f64 * 1e-6
    };

    // --- Step 1: digitize once into a resident bool array (kind-dependent thresholds). -----------
    let mut active_raw = [false; Control::COUNT];
    for c in Control::ALL {
        active_raw[c.as_index()] = state.pressed(c, t);
    }

    // --- Step 2: per-control shift selection (verifier FIX 4b). ----------------------------------
    // Each control's effective bind is its shift_bind iff its shift_trigger reads pressed in the RAW
    // digitized state, else its base bind. Computed once into a dense array, reads RAW only.
    let mut eff_bind = [BindTarget::Passthrough; Control::COUNT];
    for c in Control::ALL {
        eff_bind[c.as_index()] = effective_bind(rp.slot(c), &active_raw);
    }

    // --- Step 3: half-axis identity suppression pass (verifier FIX 1). ---------------------------
    // For each stick half-axis whose EFFECTIVE bind is not Passthrough, mark its axis so BOTH
    // halves' Passthrough arms suppress the identity (matching ResetToDefaultValue zeroing the pair).
    let mut axis_remapped = [false; 4];
    for c in Control::ALL {
        if let Some(axis) = c.stick_axis() {
            if eff_bind[c.as_index()].suppresses_identity() {
                axis_remapped[axis] = true;
            }
        }
    }

    // --- Step 4: per-control resolve. -----------------------------------------------------------
    let mut out = OutputState::default();
    let mut batch = KbmBatch::new();
    let mmcfg = MouseMoveCfg {
        stick: mouse_cfg(rp),
        gyro: gyro_cfg(rp),
        touch: rp.touchpad.to_accum_cfg(),
        swap_yaw_roll: rp.gyro.swap_yaw_roll,
        touch_as_mouse: rp.touchpad.as_mouse,
    };
    // Gyro→mouse activation (M5): when `GyroMode::Off` (the default) the gyro feed is fully inert no
    // matter what is bound, so a non-gyro profile never produces gyro motion. The per-control `on`
    // (a bound control, shift trigger, or always-on control) is the TriggerHeld signal; `AlwaysOn`
    // ignores it.
    let gyro_mode = rp.gyro.mode;

    for c in Control::ALL {
        let idx = c.as_index();
        let slot = rp.slot(c);
        // Effective bind under the per-control shift model (step 2).
        let bind = eff_bind[idx];

        // Previous effective `on` for this control (edge detection), captured before the end-of-loop
        // write below.
        let prev_on = ms.prev_active[idx];

        // Turbo: wrap the raw active bit through the slot's turbo gate (phase anchored to press).
        let on = match slot.turbo {
            Some(cfg) => turbo_gate(&mut ms.turbo[idx], active_raw[idx], cfg, now_us),
            None => active_raw[idx],
        };

        match bind {
            BindTarget::Passthrough => {
                resolve_passthrough(&mut out, state, c, on, &axis_remapped);
            }
            BindTarget::GamepadButton(b) => {
                if on {
                    out.buttons.set(b.bit(), true);
                }
            }
            BindTarget::GamepadAxis { axis, dir, full } => {
                resolve_gamepad_axis(&mut out, axis, dir, full, on);
            }
            BindTarget::TouchpadClick => {
                if on {
                    out.buttons.set(crate::output::PadButtons::TOUCHPAD, true);
                }
            }
            BindTarget::Key { vk, kind } => {
                resolve_key(vk, kind, on, prev_on, ms, &mut batch);
            }
            BindTarget::Mouse(btn) => {
                resolve_mouse_button(btn, on, prev_on, &mut batch);
            }
            BindTarget::MouseMove(src) => {
                // Mouse-move runs continuously while gated on (no edge): the accumulator owns the
                // sub-pixel carry. Only feed it while `on` so a turbo/shift gate can pulse it. The
                // Gyro source additionally requires the profile's gyro mode to be active (`on` is
                // the TriggerHeld signal); `Off` keeps the gyro path inert.
                let feed = on
                    && match src {
                        MouseMoveSrc::Gyro => gyro_mode.is_active(on),
                        _ => true,
                    };
                if feed {
                    resolve_mouse_move(src, state, &mmcfg, elapsed_s, ms, &mut batch);
                }
            }
            BindTarget::MouseWheel(dir) => {
                resolve_mouse_wheel(dir, on, prev_on, ms, &mut batch);
            }
            BindTarget::Macro(id) => {
                // Start on the rising edge, stop on the falling edge; timing owned by the injector.
                if on && !prev_on {
                    batch.push(KbmEvent::Macro { id, start: true });
                } else if !on && prev_on {
                    batch.push(KbmEvent::Macro { id, start: false });
                }
            }
            BindTarget::Special(id) => {
                // Edge to the control plane (drained off the KBM injector). Fire on the rising edge.
                if on && !prev_on {
                    batch.push(KbmEvent::Special { id });
                }
            }
            // No direct output: a `Shift` control only gates other controls (handled in step 2) and
            // suppresses its own identity. Identity-suppressing no-output binds emit nothing; the
            // axis pass already zeroed the pair and button identity is only written in the
            // Passthrough arm. `KeyUnbound` == explicit "no key" (DS4KeyType.Unbound, verifier FIX 5).
            BindTarget::Shift(_)
            | BindTarget::Unbound
            | BindTarget::KeyUnbound
            | BindTarget::Unknown => {}
        }

        ms.prev_active[idx] = on;
    }

    // Advance the touchpad delta anchor once per report (M6): the next `touch_step` differences the
    // next contact against finger 0 of THIS report. Stored unconditionally so a `MouseMove(Touchpad)`
    // binding always has a fresh anchor, and `touch_step`'s same-finger guard handles lifts/ids.
    ms.prev_touch = state.touch[0];
    ms.prev_now_us = now_us;
    (out, batch)
}

/// Resolve a mouse-button binding: emit a `MouseButton{down}` edge on press/release, nothing while
/// held (ports the DS4Windows mouse-button hold semantics).
#[inline]
fn resolve_mouse_button(btn: MouseButton, on: bool, prev_on: bool, batch: &mut KbmBatch) {
    if on && !prev_on {
        batch.push(KbmEvent::MouseButton { btn, down: true });
    } else if !on && prev_on {
        batch.push(KbmEvent::MouseButton { btn, down: false });
    }
}

/// The `Passthrough` arm: copy the physical control through, with half-axis identity suppression.
#[inline]
fn resolve_passthrough(
    out: &mut OutputState,
    state: &ControllerState,
    c: Control,
    on: bool,
    axis_remapped: &[bool; 4],
) {
    match c.kind() {
        ControlKind::AxisDir => {
            if let Some(axis) = c.stick_axis() {
                if !axis_remapped[axis] {
                    passthrough_axis(out, state, axis);
                }
            }
        }
        ControlKind::Trigger => {
            // Continuous trigger passthrough (analog), routed by which trigger this control is.
            let v = state.analog(c);
            match c {
                Control::L2 => out.lt = v,
                Control::R2 => out.rt = v,
                _ => {}
            }
        }
        ControlKind::Button | ControlKind::Touch | ControlKind::GyroDir => {
            // Digital identity: set the corresponding pad bit when pressed. Identity suppression
            // for buttons "falls out for free" — a remapped button never enters this arm.
            if on {
                if let Some(bit) = button_pad_bit(c) {
                    out.buttons.set(bit, true);
                }
            }
        }
    }
}

/// The default identity pad-button bit for a physical control (passthrough identity map).
///
/// Mirrors the existing `win_io::ds_buttons_to_xinput` / `pack_xinput` button list: Cross→A,
/// Circle→B, Square→X, Triangle→Y, Share→Back, Options→Start, PS→Guide, L1/R1→LB/RB,
/// L3/R3→LS/RS, dpad→dpad, touchpad click→TOUCHPAD. Controls without a virtual-pad bit (analog
/// triggers, stick axes, gyro, Edge paddles, touch regions) return `None`.
#[inline]
fn button_pad_bit(c: Control) -> Option<u32> {
    use crate::output::PadButtons as P;
    Some(match c {
        Control::Cross => P::A,
        Control::Circle => P::B,
        Control::Square => P::X,
        Control::Triangle => P::Y,
        Control::L1 => P::LB,
        Control::R1 => P::RB,
        Control::Share => P::BACK,
        Control::Options => P::START,
        Control::Ps => P::GUIDE,
        Control::L3 => P::LS,
        Control::R3 => P::RS,
        Control::DpadUp => P::DPAD_UP,
        Control::DpadDown => P::DPAD_DOWN,
        Control::DpadLeft => P::DPAD_LEFT,
        Control::DpadRight => P::DPAD_RIGHT,
        Control::TouchButton => P::TOUCHPAD,
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::input::Control;
    use crate::map::binding::{BindingSlot, KeyKind, PadBtn};
    use crate::map::profile::Profile;
    use crate::output::PadButtons;

    fn rp_with(c: Control, bind: BindTarget) -> ResolvedProfile {
        let mut p = Profile::default();
        p.bindings.insert(c, BindingSlot::from_bind(bind));
        p.resolve()
    }

    fn pressed_cross() -> ControllerState {
        ControllerState {
            cross: true,
            ..Default::default()
        }
    }

    #[test]
    fn button_to_button_sets_only_target_bit() {
        // Cross -> Xbox B. B set, A NOT set.
        let rp = rp_with(Control::Cross, BindTarget::GamepadButton(PadBtn::B));
        let mut ms = MapState::default();
        let (out, batch) = apply(&pressed_cross(), &rp, &mut ms, 0);
        assert!(out.buttons.has(PadButtons::B), "B should be set");
        assert!(
            !out.buttons.has(PadButtons::A),
            "A must not leak (identity suppressed)"
        );
        assert!(batch.is_empty());
    }

    #[test]
    fn passthrough_button_identity() {
        // No remap: Cross passes through to A.
        let rp = ResolvedProfile::default();
        let mut ms = MapState::default();
        let (out, _) = apply(&pressed_cross(), &rp, &mut ms, 0);
        assert!(out.buttons.has(PadButtons::A));
        assert!(!out.buttons.has(PadButtons::B));
    }

    #[test]
    fn button_to_key_edges() {
        // Square -> key 0x41 (hold). Down on press, nothing on hold, up on release.
        let rp = rp_with(
            Control::Square,
            BindTarget::Key {
                vk: 0x41,
                kind: KeyKind::HOLD,
            },
        );
        let mut ms = MapState::default();

        let down = ControllerState {
            square: true,
            ..Default::default()
        };
        let up = ControllerState::default();

        // Press: one KeyDown.
        let (_, b1) = apply(&down, &rp, &mut ms, 0);
        assert_eq!(
            b1.as_slice(),
            &[KbmEvent::Key {
                vk: 0x41,
                down: true,
                kind: InjectKind::Virtual
            }]
        );

        // Hold: no events.
        let (_, b2) = apply(&down, &rp, &mut ms, 1);
        assert!(b2.is_empty(), "no event while held");

        // Release: one KeyUp.
        let (_, b3) = apply(&up, &rp, &mut ms, 2);
        assert_eq!(
            b3.as_slice(),
            &[KbmEvent::Key {
                vk: 0x41,
                down: false,
                kind: InjectKind::Virtual
            }]
        );
    }

    #[test]
    fn toggle_latched_on_vk() {
        let kind = KeyKind {
            scan_code: false,
            toggle: true,
        };
        let rp = rp_with(Control::Square, BindTarget::Key { vk: 0x42, kind });
        let mut ms = MapState::default();
        let down = ControllerState {
            square: true,
            ..Default::default()
        };
        let up = ControllerState::default();

        // First press latches ON.
        let (_, b1) = apply(&down, &rp, &mut ms, 0);
        assert_eq!(
            b1.as_slice(),
            &[KbmEvent::Key {
                vk: 0x42,
                down: true,
                kind: InjectKind::Virtual
            }]
        );
        assert!(ms.toggle.get(0x42));
        // Hold: no further toggle.
        let (_, b2) = apply(&down, &rp, &mut ms, 1);
        assert!(b2.is_empty());
        // Release: no event, but pressed_once cleared.
        let (_, b3) = apply(&up, &rp, &mut ms, 2);
        assert!(b3.is_empty());
        // Second press latches OFF.
        let (_, b4) = apply(&down, &rp, &mut ms, 3);
        assert_eq!(
            b4.as_slice(),
            &[KbmEvent::Key {
                vk: 0x42,
                down: false,
                kind: InjectKind::Virtual
            }]
        );
        assert!(!ms.toggle.get(0x42));
    }

    #[test]
    fn two_controls_one_vk_share_toggle_latch() {
        // Square and Cross both -> the same vk toggle: a press on either flips the shared latch.
        let kind = KeyKind {
            scan_code: false,
            toggle: true,
        };
        let mut p = Profile::default();
        p.bindings.insert(
            Control::Square,
            BindingSlot::from_bind(BindTarget::Key { vk: 0x55, kind }),
        );
        p.bindings.insert(
            Control::Cross,
            BindingSlot::from_bind(BindTarget::Key { vk: 0x55, kind }),
        );
        let rp = p.resolve();
        let mut ms = MapState::default();

        // Press Square -> latch ON.
        let sq = ControllerState {
            square: true,
            ..Default::default()
        };
        let (_, _) = apply(&sq, &rp, &mut ms, 0);
        assert!(ms.toggle.get(0x55));
        // Release all.
        let (_, _) = apply(&ControllerState::default(), &rp, &mut ms, 1);
        // Press Cross -> shared latch flips OFF.
        let cr = ControllerState {
            cross: true,
            ..Default::default()
        };
        let (_, b) = apply(&cr, &rp, &mut ms, 2);
        assert_eq!(
            b.as_slice(),
            &[KbmEvent::Key {
                vk: 0x55,
                down: false,
                kind: InjectKind::Virtual
            }]
        );
        assert!(!ms.toggle.get(0x55));
    }

    #[test]
    fn vklatch_overflow_saturates_no_panic() {
        let mut l = VkLatch::new();
        for vk in 0..(VkLatch::CAP as u16 + 8) {
            l.set(vk, true);
        }
        // First CAP keys tracked; the rest dropped silently.
        for vk in 0..VkLatch::CAP as u16 {
            assert!(l.get(vk));
        }
        for vk in VkLatch::CAP as u16..(VkLatch::CAP as u16 + 8) {
            assert!(!l.get(vk), "overflow keys are untracked, default false");
        }
    }

    #[test]
    fn half_axis_identity_leak_guard() {
        // Bind ONLY LxPos -> a key; leave LxNeg as Passthrough. Pushing the stick to the negative
        // half must NOT leak into out.lx (both halves of Lx go neutral — verifier FIX 1).
        let rp = rp_with(
            Control::LxPos,
            BindTarget::Key {
                vk: 0x41,
                kind: KeyKind::HOLD,
            },
        );
        let mut ms = MapState::default();

        // Stick hard left (negative Lx).
        let st = ControllerState {
            lx: -1.0,
            ..Default::default()
        };
        let (out, _) = apply(&st, &rp, &mut ms, 0);
        assert_eq!(
            out.lx, 0.0,
            "negative half must not leak when the positive half is remapped"
        );
        assert_eq!(out.ly, 0.0);

        // Other axes still pass through normally.
        let st2 = ControllerState {
            lx: -1.0,
            ry: 0.5,
            ..Default::default()
        };
        let (out2, _) = apply(&st2, &rp, &mut ms, 1);
        assert_eq!(out2.lx, 0.0);
        assert_eq!(out2.ry, 0.5, "an unrelated axis is unaffected");
    }

    #[test]
    fn unremapped_axis_passes_through_both_halves() {
        // All-passthrough: a left push reaches out.lx, a right push reaches out.lx — same field.
        let rp = ResolvedProfile::default();
        let mut ms = MapState::default();
        let (out_l, _) = apply(
            &ControllerState {
                lx: -0.8,
                ..Default::default()
            },
            &rp,
            &mut ms,
            0,
        );
        assert_eq!(out_l.lx, -0.8);
        let (out_r, _) = apply(
            &ControllerState {
                lx: 0.6,
                ..Default::default()
            },
            &rp,
            &mut ms,
            1,
        );
        assert_eq!(out_r.lx, 0.6);
    }

    #[test]
    fn passthrough_precision_is_exact() {
        // A precise stick value flows through unrounded (no mid-chain quantization in apply()).
        let rp = ResolvedProfile::default();
        let mut ms = MapState::default();
        let st = ControllerState {
            lx: 0.123_456_789,
            ry: -0.987_654_321,
            ..Default::default()
        };
        let (out, _) = apply(&st, &rp, &mut ms, 0);
        assert_eq!(out.lx, 0.123_456_789);
        assert_eq!(out.ry, -0.987_654_321);
    }

    #[test]
    fn analog_to_digital_axis_threshold_55_127() {
        // GamepadButton bound to LxPos fires only past the 55/127 stick-dir threshold.
        let rp = rp_with(Control::LxPos, BindTarget::GamepadButton(PadBtn::A));
        let mut ms = MapState::default();
        let below = 54.0 / 127.0;
        let above = 56.0 / 127.0;
        let (lo, _) = apply(
            &ControllerState {
                lx: below,
                ..Default::default()
            },
            &rp,
            &mut ms,
            0,
        );
        assert!(!lo.buttons.has(PadButtons::A), "below 55/127 must not fire");
        let (hi, _) = apply(
            &ControllerState {
                lx: above,
                ..Default::default()
            },
            &rp,
            &mut ms,
            1,
        );
        assert!(hi.buttons.has(PadButtons::A), "above 55/127 must fire");
    }

    #[test]
    fn analog_to_digital_trigger_threshold_100_255() {
        // GamepadButton bound to L2 (analog trigger) fires only past 100/255.
        let rp = rp_with(Control::L2, BindTarget::GamepadButton(PadBtn::Lb));
        let mut ms = MapState::default();
        let below = 99.0 / 255.0;
        let above = 101.0 / 255.0;
        let (lo, _) = apply(
            &ControllerState {
                l2: below,
                ..Default::default()
            },
            &rp,
            &mut ms,
            0,
        );
        assert!(
            !lo.buttons.has(PadButtons::LB),
            "below 100/255 must not fire"
        );
        let (hi, _) = apply(
            &ControllerState {
                l2: above,
                ..Default::default()
            },
            &rp,
            &mut ms,
            1,
        );
        assert!(hi.buttons.has(PadButtons::LB), "above 100/255 must fire");
    }

    #[test]
    fn trigger_passthrough_is_continuous() {
        // Default profile: analog trigger flows to lt continuously (not just digital).
        let rp = ResolvedProfile::default();
        let mut ms = MapState::default();
        let (out, _) = apply(
            &ControllerState {
                l2: 0.42,
                r2: 0.7,
                ..Default::default()
            },
            &rp,
            &mut ms,
            0,
        );
        assert_eq!(out.lt, 0.42);
        assert_eq!(out.rt, 0.7);
    }

    #[test]
    fn kbm_batch_saturates_without_panic() {
        // Bind many distinct controls to distinct keys and press them all in one report. The batch
        // saturates at its cap (no panic, no alloc) and stays a valid slice.
        let mut p = Profile::default();
        // Use face + dpad + shoulder buttons, each to a unique vk, all pressed together.
        let controls = [
            Control::Square,
            Control::Triangle,
            Control::Circle,
            Control::Cross,
            Control::DpadUp,
            Control::DpadDown,
            Control::DpadLeft,
            Control::DpadRight,
            Control::L1,
            Control::R1,
            Control::L3,
            Control::R3,
            Control::Ps,
            Control::Share,
            Control::Options,
        ];
        for (i, &c) in controls.iter().enumerate() {
            p.bindings.insert(
                c,
                BindingSlot::from_bind(BindTarget::Key {
                    vk: 0x41 + i as u16,
                    kind: KeyKind::HOLD,
                }),
            );
        }
        let rp = p.resolve();
        let mut ms = MapState::default();
        let st = ControllerState {
            square: true,
            triangle: true,
            circle: true,
            cross: true,
            dpad_up: true,
            dpad_down: true,
            dpad_left: true,
            dpad_right: true,
            l1: true,
            r1: true,
            l3: true,
            r3: true,
            ps: true,
            share: true,
            options: true,
            ..Default::default()
        };
        let (_, batch) = apply(&st, &rp, &mut ms, 0);
        // 15 distinct key-down edges; well within CAP — all present, no overflow.
        assert_eq!(batch.as_slice().len(), controls.len());
    }

    #[test]
    fn all_passthrough_matches_outputstate_passthrough() {
        // Non-regression invariant (blueprint §7.2): an all-Passthrough profile produces the same
        // OutputState as the identity `OutputState::passthrough(&state)` seed.
        let rp = ResolvedProfile::default();
        let mut ms = MapState::default();
        let st = ControllerState {
            lx: 0.25,
            ly: -0.5,
            rx: -0.1,
            ry: 0.9,
            l2: 0.3,
            r2: 0.8,
            cross: true,
            circle: true,
            l1: true,
            dpad_left: true,
            options: true,
            ..Default::default()
        };
        let (out, batch) = apply(&st, &rp, &mut ms, 0);
        assert_eq!(out, OutputState::passthrough(&st));
        assert!(batch.is_empty(), "passthrough emits no KBM events");
    }

    #[test]
    fn default_mapstate_is_clean() {
        let ms = MapState::default();
        assert!(ms.prev_active.iter().all(|&b| !b));
        assert!(ms.turbo.iter().all(|&t| t == TurboState::default()));
        assert_eq!(ms.wheel_remainder, 0.0);
        assert!(!ms.toggle.get(0x41));
        assert!(!ms.pressed_once.get(0x41));
        assert_eq!(ms.prev_now_us, 0);
        assert_eq!(ms.stick_mouse.remainder(), (0.0, 0.0));
        assert_eq!(ms.gyro_mouse.remainder(), (0.0, 0.0));
        assert_eq!(ms.touch_mouse.remainder(), (0.0, 0.0));
        assert_eq!(ms.prev_touch, crate::input::TouchContact::default());
    }

    // ----------------------------- M4: shift / turbo / macro / mouse -----------------------------
    // (AxisDir/GamepadAxis/MouseMoveSrc/TurboCfg/WheelDir/MouseButton come in via `super::*`.)

    use crate::map::binding::ShiftTrigger;

    /// Build a profile with a single slot fully specified (base + shift + turbo).
    fn rp_slot(c: Control, slot: BindingSlot) -> ResolvedProfile {
        let mut p = Profile::default();
        p.bindings.insert(c, slot);
        p.resolve()
    }

    #[test]
    fn per_control_shift_two_distinct_triggers_simultaneously() {
        // Control X (Cross) is shifted by trigger T1 (L1); control Y (Circle) by T2 (R1). When BOTH
        // triggers are held, X uses its shift bind AND Y uses its shift bind — distinct, simultaneous
        // (the faithful per-control model, verifier FIX 4b).
        let mut p = Profile::default();
        p.bindings.insert(
            Control::Cross,
            BindingSlot {
                bind: BindTarget::GamepadButton(PadBtn::A),
                shift_trigger: Some(ShiftTrigger {
                    control: Control::L1,
                }),
                shift_bind: BindTarget::GamepadButton(PadBtn::X),
                turbo: None,
            },
        );
        p.bindings.insert(
            Control::Circle,
            BindingSlot {
                bind: BindTarget::GamepadButton(PadBtn::B),
                shift_trigger: Some(ShiftTrigger {
                    control: Control::R1,
                }),
                shift_bind: BindTarget::GamepadButton(PadBtn::Y),
                turbo: None,
            },
        );
        let rp = p.resolve();
        let mut ms = MapState::default();

        // No triggers: base binds (A from Cross, B from Circle).
        let base = ControllerState {
            cross: true,
            circle: true,
            ..Default::default()
        };
        let (out, _) = apply(&base, &rp, &mut ms, 0);
        assert!(out.buttons.has(PadButtons::A) && out.buttons.has(PadButtons::B));
        assert!(!out.buttons.has(PadButtons::X) && !out.buttons.has(PadButtons::Y));

        // Only T1 (L1) held: Cross shifts to X; Circle stays on its base B.
        let shifted_x = ControllerState {
            cross: true,
            circle: true,
            l1: true,
            ..Default::default()
        };
        let (out, _) = apply(&shifted_x, &rp, &mut ms, 1);
        assert!(out.buttons.has(PadButtons::X), "Cross shifted to X");
        assert!(out.buttons.has(PadButtons::B), "Circle unshifted -> B");
        assert!(!out.buttons.has(PadButtons::A) && !out.buttons.has(PadButtons::Y));

        // Both triggers held: BOTH controls use their shift binds simultaneously.
        let both = ControllerState {
            cross: true,
            circle: true,
            l1: true,
            r1: true,
            ..Default::default()
        };
        let (out, _) = apply(&both, &rp, &mut ms, 2);
        assert!(
            out.buttons.has(PadButtons::X) && out.buttons.has(PadButtons::Y),
            "both controls shifted simultaneously"
        );
        assert!(!out.buttons.has(PadButtons::A) && !out.buttons.has(PadButtons::B));
    }

    #[test]
    fn analog_shift_trigger_activates_at_trigger_threshold() {
        // Shift trigger is the analog L2 (trigger kind, 100/255). Below threshold -> base; at/above
        // -> shift bind. Pins the kind-dependent threshold on the shift trigger read (RAW only).
        let slot = BindingSlot {
            bind: BindTarget::GamepadButton(PadBtn::A),
            shift_trigger: Some(ShiftTrigger {
                control: Control::L2,
            }),
            shift_bind: BindTarget::GamepadButton(PadBtn::B),
            turbo: None,
        };
        let rp = rp_slot(Control::Cross, slot);
        let mut ms = MapState::default();

        // L2 at 99/255 (below 100/255) -> base A.
        let lo = ControllerState {
            cross: true,
            l2: 99.0 / 255.0,
            ..Default::default()
        };
        let (out, _) = apply(&lo, &rp, &mut ms, 0);
        assert!(out.buttons.has(PadButtons::A) && !out.buttons.has(PadButtons::B));

        // L2 at 101/255 (above) -> shift bind B.
        let hi = ControllerState {
            cross: true,
            l2: 101.0 / 255.0,
            ..Default::default()
        };
        let (out, _) = apply(&hi, &rp, &mut ms, 1);
        assert!(out.buttons.has(PadButtons::B) && !out.buttons.has(PadButtons::A));
    }

    #[test]
    fn turbo_phase_on_on_off_off_with_press_reset() {
        // Period 100us, 50% duty -> ON for phase [0,50), OFF for [50,100). Sampling at 0,25,50,75
        // from a press at t0 gives ON,ON,OFF,OFF. Releasing and re-pressing re-anchors the phase so
        // a fresh press starts a full ON window.
        let slot = BindingSlot {
            bind: BindTarget::GamepadButton(PadBtn::A),
            shift_trigger: None,
            shift_bind: BindTarget::Passthrough,
            turbo: Some(TurboCfg {
                period_us: 100,
                duty_num: 1,
                duty_den: 2,
            }),
        };
        let rp = rp_slot(Control::Cross, slot);
        let mut ms = MapState::default();
        let held = ControllerState {
            cross: true,
            ..Default::default()
        };

        // Press at t=1000 (anchor); sample within the cycle.
        let on_at =
            |ms: &mut MapState, t: u64| apply(&held, &rp, ms, t).0.buttons.has(PadButtons::A);
        assert!(on_at(&mut ms, 1000), "phase 0 -> ON");
        assert!(on_at(&mut ms, 1025), "phase 25 -> ON");
        assert!(!on_at(&mut ms, 1050), "phase 50 -> OFF");
        assert!(!on_at(&mut ms, 1075), "phase 75 -> OFF");

        // Release, then re-press much later: the new press re-anchors -> a full ON window again.
        let released = ControllerState::default();
        let _ = apply(&released, &rp, &mut ms, 1100);
        assert!(
            on_at(&mut ms, 9999),
            "a fresh press re-anchors the phase to a full ON window"
        );
    }

    #[test]
    fn macro_start_stop_edges() {
        let rp = rp_with(Control::Square, BindTarget::Macro(7));
        let mut ms = MapState::default();
        let down = ControllerState {
            square: true,
            ..Default::default()
        };
        let up = ControllerState::default();

        // Press -> start edge.
        let (_, b1) = apply(&down, &rp, &mut ms, 0);
        assert_eq!(b1.as_slice(), &[KbmEvent::Macro { id: 7, start: true }]);
        // Hold -> nothing.
        let (_, b2) = apply(&down, &rp, &mut ms, 1);
        assert!(b2.is_empty());
        // Release -> stop edge.
        let (_, b3) = apply(&up, &rp, &mut ms, 2);
        assert_eq!(
            b3.as_slice(),
            &[KbmEvent::Macro {
                id: 7,
                start: false
            }]
        );
    }

    #[test]
    fn special_fires_on_rising_edge_only() {
        let rp = rp_with(Control::Options, BindTarget::Special(3));
        let mut ms = MapState::default();
        let down = ControllerState {
            options: true,
            ..Default::default()
        };
        let up = ControllerState::default();

        let (_, b1) = apply(&down, &rp, &mut ms, 0);
        assert_eq!(b1.as_slice(), &[KbmEvent::Special { id: 3 }]);
        // Held -> no repeat.
        let (_, b2) = apply(&down, &rp, &mut ms, 1);
        assert!(b2.is_empty());
        // Release -> no edge.
        let (_, b3) = apply(&up, &rp, &mut ms, 2);
        assert!(b3.is_empty());
    }

    #[test]
    fn mouse_button_edges() {
        let rp = rp_with(Control::Cross, BindTarget::Mouse(MouseButton::Left));
        let mut ms = MapState::default();
        let down = pressed_cross();
        let up = ControllerState::default();

        let (_, b1) = apply(&down, &rp, &mut ms, 0);
        assert_eq!(
            b1.as_slice(),
            &[KbmEvent::MouseButton {
                btn: MouseButton::Left,
                down: true
            }]
        );
        let (_, b2) = apply(&down, &rp, &mut ms, 1);
        assert!(b2.is_empty(), "no event while held");
        let (_, b3) = apply(&up, &rp, &mut ms, 2);
        assert_eq!(
            b3.as_slice(),
            &[KbmEvent::MouseButton {
                btn: MouseButton::Left,
                down: false
            }]
        );
    }

    #[test]
    fn mouse_move_from_deflected_stick_with_remainder_carry() {
        // Right stick bound to mouse-move. A deflected stick over several reports emits MouseMove
        // events; the sub-pixel remainder carries between reports (no motion is lost). The first
        // report has dt 0 (prev_now_us == 0) so it only contributes the offset; subsequent reports
        // have a real dt and produce integer deltas.
        let rp = rp_with(
            Control::RxPos,
            BindTarget::MouseMove(MouseMoveSrc::RightStick),
        );
        let mut ms = MapState::default();

        // Full right deflection, sampled at 5ms cadence.
        let st = ControllerState {
            rx: 1.0,
            ..Default::default()
        };
        // First report: dt 0 -> with default offset only; may be (0,0). Establish prev_now_us.
        let _ = apply(&st, &rp, &mut ms, 1_000);

        // Several real-dt reports must eventually emit a non-zero MouseMove with dx > 0 (rightward).
        let mut saw_move = false;
        let mut t = 6_000u64;
        for _ in 0..8 {
            let (_, b) = apply(&st, &rp, &mut ms, t);
            for e in b.as_slice() {
                if let KbmEvent::MouseMove { dx, dy: _ } = e {
                    assert!(*dx > 0, "right deflection -> rightward mouse, got dx={dx}");
                    saw_move = true;
                }
            }
            t += 5_000;
        }
        assert!(saw_move, "a deflected stick eventually emits a MouseMove");
    }

    #[test]
    fn mouse_move_not_emitted_when_gated_off() {
        // A mouse-move bound behind a shift trigger that is NOT active stays the base Passthrough,
        // so no MouseMove is emitted from the stick (the base axis passes through instead).
        let slot = BindingSlot {
            bind: BindTarget::Passthrough,
            shift_trigger: Some(ShiftTrigger {
                control: Control::L1,
            }),
            shift_bind: BindTarget::MouseMove(MouseMoveSrc::RightStick),
            turbo: None,
        };
        let rp = rp_slot(Control::RxPos, slot);
        let mut ms = MapState::default();
        let st = ControllerState {
            rx: 1.0,
            ..Default::default()
        };
        let _ = apply(&st, &rp, &mut ms, 1_000);
        let (out, b) = apply(&st, &rp, &mut ms, 6_000);
        assert!(
            b.as_slice()
                .iter()
                .all(|e| !matches!(e, KbmEvent::MouseMove { .. })),
            "no mouse-move while the shift trigger is inactive"
        );
        // Base passthrough still reaches the stick axis.
        assert_eq!(out.rx, 1.0);
    }

    #[test]
    fn mouse_wheel_accumulates_to_a_notch() {
        // A held wheel-up source emits +120 ticks via the 1/3-notch-per-report carry: ~3 reports per
        // notch. Over 12 reports we expect ~4 notches, all positive vertical, none horizontal, and
        // the carry never loses motion.
        let rp = rp_with(Control::DpadUp, BindTarget::MouseWheel(WheelDir::Up));
        let mut ms = MapState::default();
        let down = ControllerState {
            dpad_up: true,
            ..Default::default()
        };

        let mut ticks = 0i32;
        for t in 0..12u64 {
            let (_, b) = apply(&down, &rp, &mut ms, t);
            for e in b.as_slice() {
                if let KbmEvent::Wheel {
                    vertical,
                    horizontal,
                } = e
                {
                    assert_eq!(*horizontal, 0);
                    assert!(*vertical > 0, "wheel up -> positive vertical");
                    ticks += vertical / 120;
                }
            }
        }
        assert!(
            (3..=4).contains(&ticks),
            "12 reports at 1/3-notch each -> ~4 notches, got {ticks}"
        );

        // Release resets the remainder so a later press starts fresh.
        let (_, _) = apply(&ControllerState::default(), &rp, &mut ms, 12);
        assert_eq!(ms.wheel_remainder, 0.0);
    }

    #[test]
    fn gamepad_axis_digital_push() {
        // Cross -> push the right stick fully to +X.
        let rp = rp_with(
            Control::Cross,
            BindTarget::GamepadAxis {
                axis: GamepadAxis::Rx,
                dir: AxisDir::Pos,
                full: true,
            },
        );
        let mut ms = MapState::default();
        let (out, _) = apply(&pressed_cross(), &rp, &mut ms, 0);
        assert_eq!(out.rx, 1.0);
        // Released -> neutral.
        let (out0, _) = apply(&ControllerState::default(), &rp, &mut ms, 1);
        assert_eq!(out0.rx, 0.0);
    }

    #[test]
    fn touchpad_click_output_bit() {
        let rp = rp_with(Control::Cross, BindTarget::TouchpadClick);
        let mut ms = MapState::default();
        let (out, _) = apply(&pressed_cross(), &rp, &mut ms, 0);
        assert!(out.buttons.has(PadButtons::TOUCHPAD));
    }

    #[test]
    fn turbo_gate_unit_phase() {
        // Direct gate test: period 100, 25% duty -> ON for [0,25), OFF for [25,100).
        let cfg = TurboCfg {
            period_us: 100,
            duty_num: 1,
            duty_den: 4,
        };
        let mut ts = TurboState::default();
        assert!(turbo_gate(&mut ts, true, cfg, 1000), "phase 0 -> ON");
        assert!(turbo_gate(&mut ts, true, cfg, 1024), "phase 24 -> ON");
        assert!(!turbo_gate(&mut ts, true, cfg, 1025), "phase 25 -> OFF");
        assert!(!turbo_gate(&mut ts, true, cfg, 1099), "phase 99 -> OFF");
        assert!(
            turbo_gate(&mut ts, true, cfg, 1100),
            "phase 0 (next cycle) -> ON"
        );
        // Release clears the active flag (re-anchors on next press).
        assert!(!turbo_gate(&mut ts, false, cfg, 1200));
        assert!(!ts.was_active);
    }

    #[test]
    fn shifted_half_axis_suppresses_identity_pair() {
        // LxPos's shift bind (active under L1) is a key, so while L1 is held BOTH halves of Lx go
        // neutral (the axis_remapped pass uses the EFFECTIVE bind). Without L1, the base is
        // Passthrough and the axis flows.
        let slot = BindingSlot {
            bind: BindTarget::Passthrough,
            shift_trigger: Some(ShiftTrigger {
                control: Control::L1,
            }),
            shift_bind: BindTarget::Key {
                vk: 0x41,
                kind: KeyKind::HOLD,
            },
            turbo: None,
        };
        let rp = rp_slot(Control::LxPos, slot);
        let mut ms = MapState::default();

        // No shift: the negative half passes through.
        let left = ControllerState {
            lx: -0.8,
            ..Default::default()
        };
        let (out, _) = apply(&left, &rp, &mut ms, 0);
        assert_eq!(out.lx, -0.8, "unshifted axis passes through");

        // Shift active (L1): the effective bind for LxPos is a key -> the WHOLE Lx axis is zeroed.
        let left_shift = ControllerState {
            lx: -0.8,
            l1: true,
            ..Default::default()
        };
        let (out, _) = apply(&left_shift, &rp, &mut ms, 1);
        assert_eq!(out.lx, 0.0, "shifted half remaps -> both halves neutral");
    }

    // ------------------------------------ M5: gyro→mouse -----------------------------------------

    use crate::input::Motion;
    use crate::map::profile::{GyroMode, GyroSettings};

    /// Build a profile binding `GyroZPos` (a gyro direction control) to `MouseMove(Gyro)` with the
    /// given gyro settings. `GyroZPos` is digitized active when the yaw rate exceeds `gyro_dir`, so a
    /// rightward yaw turns the control `on` and the gyro feed runs.
    fn rp_gyro(gyro: GyroSettings) -> ResolvedProfile {
        let mut p = Profile {
            gyro,
            ..Profile::default()
        };
        p.bindings.insert(
            Control::GyroZPos,
            BindingSlot::from_bind(BindTarget::MouseMove(MouseMoveSrc::Gyro)),
        );
        p.resolve()
    }

    fn yaw_state(rate: f64) -> ControllerState {
        ControllerState {
            motion: Motion {
                gyro_yaw: rate,
                ..Default::default()
            },
            ..Default::default()
        }
    }

    #[test]
    fn gyro_mode_off_is_inert() {
        // Default gyro mode is Off: even with a strong yaw and a MouseMove(Gyro) binding, no motion.
        let rp = rp_gyro(GyroSettings::default());
        let mut ms = MapState::default();
        let st = yaw_state(5.0); // well past the gyro_dir threshold
        let _ = apply(&st, &rp, &mut ms, 1_000);
        let (_, b) = apply(&st, &rp, &mut ms, 6_000);
        assert!(
            b.as_slice()
                .iter()
                .all(|e| !matches!(e, KbmEvent::MouseMove { .. })),
            "GyroMode::Off must emit no gyro mouse motion"
        );
    }

    #[test]
    fn gyro_always_on_scales_and_moves_right() {
        // AlwaysOn + a rightward yaw -> a positive dx mouse move once the velocity model + carry
        // cross a pixel. Use a high sensitivity so it lands within a few reports.
        let rp = rp_gyro(GyroSettings {
            mode: GyroMode::AlwaysOn,
            sensitivity: 80.0,
            deadzone: 0.0,
            jitter_comp: false,
            ..GyroSettings::default()
        });
        let mut ms = MapState::default();
        let st = yaw_state(5.0);
        let _ = apply(&st, &rp, &mut ms, 1_000); // prime prev_now_us (dt 0)

        let mut saw_right = false;
        let mut t = 6_000u64;
        for _ in 0..8 {
            let (_, b) = apply(&st, &rp, &mut ms, t);
            for e in b.as_slice() {
                if let KbmEvent::MouseMove { dx, dy } = e {
                    assert!(*dx > 0, "rightward yaw -> dx>0, got {dx}");
                    assert_eq!(*dy, 0, "pure yaw -> no vertical, got {dy}");
                    saw_right = true;
                }
            }
            t += 5_000;
        }
        assert!(
            saw_right,
            "AlwaysOn gyro yaw eventually emits a rightward MouseMove"
        );
    }

    #[test]
    fn gyro_invert_x_flips_direction() {
        let base = GyroSettings {
            mode: GyroMode::AlwaysOn,
            sensitivity: 80.0,
            deadzone: 0.0,
            jitter_comp: false,
            ..GyroSettings::default()
        };
        let inv = GyroSettings {
            invert_x: true,
            ..base
        };
        let drive = |rp: &ResolvedProfile| -> i32 {
            let mut ms = MapState::default();
            let st = yaw_state(5.0);
            let _ = apply(&st, rp, &mut ms, 1_000);
            let mut sum = 0;
            let mut t = 6_000u64;
            for _ in 0..8 {
                let (_, b) = apply(&st, rp, &mut ms, t);
                for e in b.as_slice() {
                    if let KbmEvent::MouseMove { dx, .. } = e {
                        sum += *dx;
                    }
                }
                t += 5_000;
            }
            sum
        };
        let normal = drive(&rp_gyro(base));
        let inverted = drive(&rp_gyro(inv));
        assert!(normal > 0, "normal yaw moves right");
        assert!(inverted < 0, "invert_x flips it left, got {inverted}");
    }

    #[test]
    fn gyro_pitch_up_moves_screen_up() {
        // A nose-up pitch (positive gyro_pitch) must move the cursor UP on screen (negative dy),
        // since apply() feeds gyro_z = -pitch. Bind GyroXNeg (pitch-down direction) is not active
        // here; we bind via an always-on control so the feed runs regardless of the gyro direction
        // digitization. Use a Passthrough+shift trick: bind a face button to MouseMove(Gyro).
        let mut p = Profile {
            gyro: GyroSettings {
                mode: GyroMode::AlwaysOn,
                sensitivity: 80.0,
                deadzone: 0.0,
                jitter_comp: false,
                ..GyroSettings::default()
            },
            ..Profile::default()
        };
        // Cross held -> on -> gyro feed runs every report (AlwaysOn ignores the trigger anyway).
        p.bindings.insert(
            Control::Cross,
            BindingSlot::from_bind(BindTarget::MouseMove(MouseMoveSrc::Gyro)),
        );
        let rp = p.resolve();
        let mut ms = MapState::default();
        let st = ControllerState {
            cross: true,
            motion: Motion {
                gyro_pitch: 5.0, // nose up
                ..Default::default()
            },
            ..Default::default()
        };
        let _ = apply(&st, &rp, &mut ms, 1_000);
        let mut saw_up = false;
        let mut t = 6_000u64;
        for _ in 0..8 {
            let (_, b) = apply(&st, &rp, &mut ms, t);
            for e in b.as_slice() {
                if let KbmEvent::MouseMove { dx, dy } = e {
                    assert_eq!(*dx, 0, "pure pitch -> no horizontal, got {dx}");
                    assert!(*dy < 0, "nose-up pitch -> screen up (dy<0), got {dy}");
                    saw_up = true;
                }
            }
            t += 5_000;
        }
        assert!(
            saw_up,
            "AlwaysOn gyro pitch-up eventually emits an upward MouseMove"
        );
    }

    #[test]
    fn gyro_swap_yaw_roll_uses_roll_for_horizontal() {
        // With swap_yaw_roll, the horizontal channel reads roll, not yaw. A pure-roll sample then
        // produces horizontal motion; a pure-yaw sample produces none.
        let rp = rp_gyro(GyroSettings {
            mode: GyroMode::AlwaysOn,
            sensitivity: 80.0,
            deadzone: 0.0,
            jitter_comp: false,
            swap_yaw_roll: true,
            ..GyroSettings::default()
        });
        // Note: rp_gyro binds GyroZPos (yaw-based digitization). With swap on, the horizontal feed
        // reads roll, but the BINDING activation still uses yaw. So drive BOTH a yaw (to activate the
        // control) and a roll (to produce horizontal motion).
        let mut ms = MapState::default();
        let st = ControllerState {
            motion: Motion {
                gyro_yaw: 5.0,  // activates GyroZPos
                gyro_roll: 5.0, // drives the (swapped) horizontal channel
                ..Default::default()
            },
            ..Default::default()
        };
        let _ = apply(&st, &rp, &mut ms, 1_000);
        let mut saw = false;
        let mut t = 6_000u64;
        for _ in 0..8 {
            let (_, b) = apply(&st, &rp, &mut ms, t);
            for e in b.as_slice() {
                if let KbmEvent::MouseMove { dx, .. } = e {
                    assert!(*dx > 0, "roll drives horizontal when swapped, got {dx}");
                    saw = true;
                }
            }
            t += 5_000;
        }
        assert!(saw, "swapped horizontal axis reads roll");
    }

    // ------------------------------------ M6: touchpad→mouse -------------------------------------

    use crate::input::TouchContact;
    use crate::map::profile::TouchpadSettings;

    fn active_contact(id: u8, x: u16, y: u16) -> TouchContact {
        TouchContact {
            is_active: true,
            id,
            x,
            y,
        }
    }

    /// Build a profile binding Cross to `MouseMove(Touchpad)` with the given touchpad settings.
    fn rp_touch(touchpad: TouchpadSettings) -> ResolvedProfile {
        let mut p = Profile {
            touchpad,
            ..Profile::default()
        };
        p.bindings.insert(
            Control::Cross,
            BindingSlot::from_bind(BindTarget::MouseMove(MouseMoveSrc::Touchpad)),
        );
        p.resolve()
    }

    fn touch_state(c: TouchContact) -> ControllerState {
        ControllerState {
            cross: true, // hold Cross so the bound MouseMove(Touchpad) feed runs
            touch: [c, TouchContact::default()],
            ..Default::default()
        }
    }

    #[test]
    fn touchpad_as_mouse_off_is_inert() {
        // Default touchpad settings have as_mouse == false: even a finger drag emits no MouseMove.
        let rp = rp_touch(TouchpadSettings::default());
        let mut ms = MapState::default();
        let _ = apply(
            &touch_state(active_contact(1, 100, 100)),
            &rp,
            &mut ms,
            1_000,
        );
        let (_, b) = apply(
            &touch_state(active_contact(1, 400, 100)),
            &rp,
            &mut ms,
            6_000,
        );
        assert!(
            b.as_slice()
                .iter()
                .all(|e| !matches!(e, KbmEvent::MouseMove { .. })),
            "as_mouse off must emit no touch mouse motion"
        );
    }

    #[test]
    fn touchpad_as_mouse_drag_moves_cursor() {
        // as_mouse on + a same-finger rightward drag -> a positive dx MouseMove. `sensitivity` is
        // the resolved coefficient (DS4Windows `getTouchSensitivity·0.01`), so 1.0 with a 100-unit
        // grid delta is ~100px — well past one pixel.
        let rp = rp_touch(TouchpadSettings {
            as_mouse: true,
            sensitivity: 1.0,
            jitter_comp: false,
            ..TouchpadSettings::default()
        });
        let mut ms = MapState::default();
        // First report establishes prev_touch (no prior contact -> no jump).
        let _ = apply(
            &touch_state(active_contact(1, 100, 200)),
            &rp,
            &mut ms,
            1_000,
        );
        let (_, b) = apply(
            &touch_state(active_contact(1, 200, 200)),
            &rp,
            &mut ms,
            6_000,
        );
        let mut saw = false;
        for e in b.as_slice() {
            if let KbmEvent::MouseMove { dx, dy } = e {
                assert!(*dx > 0, "rightward drag -> dx>0, got {dx}");
                assert_eq!(*dy, 0, "pure-x drag -> no vertical, got {dy}");
                saw = true;
            }
        }
        assert!(saw, "a same-finger drag emits a MouseMove");
    }

    #[test]
    fn touchpad_as_mouse_two_contacts_emit_delta_with_remainder_carry() {
        // M7 end-to-end wiring proof: two successive same-finger contacts feed `touch_step` through
        // apply() and emit a mouse delta, with the sub-pixel remainder carried in `ms.touch_mouse`
        // between reports (no motion lost). Pick a tiny coefficient so each +10 grid-unit slide is
        // ~0.4 px: three carry reports give 0,0,1 (rem ≈ 0.2), exactly the accumulator carry contract
        // exercised here through the full engine path (not the accumulator unit test).
        let rp = rp_touch(TouchpadSettings {
            as_mouse: true,
            sensitivity: 0.04, // 10 grid units * 0.04 = 0.4 px/report
            velocity_offset: 0.0,
            min_threshold: 1.0, // always-carry
            jitter_comp: false,
            ..TouchpadSettings::default()
        });
        let mut ms = MapState::default();

        // First report establishes prev_touch at x=0 (no prior contact -> no jump).
        let (_, b0) = apply(&touch_state(active_contact(1, 0, 0)), &rp, &mut ms, 1_000);
        assert!(
            b0.as_slice()
                .iter()
                .all(|e| !matches!(e, KbmEvent::MouseMove { .. })),
            "touch-down must not jump"
        );

        // Three same-finger slides of +10 x each: per-report 0.4 px -> dx 0, 0, 1 (carry to a pixel).
        let xs = [10u16, 20, 30];
        let mut dxs = [0i32; 3];
        let mut t = 6_000u64;
        for (i, &x) in xs.iter().enumerate() {
            let (_, b) = apply(&touch_state(active_contact(1, x, 0)), &rp, &mut ms, t);
            for e in b.as_slice() {
                if let KbmEvent::MouseMove { dx, dy } = e {
                    assert_eq!(*dy, 0, "pure-x drag -> no vertical");
                    dxs[i] = *dx;
                }
            }
            t += 5_000;
        }
        assert_eq!(
            dxs,
            [0, 0, 1],
            "0.4px×3 carries through apply() to 0,0,1 (remainder accumulates), got {dxs:?}"
        );
        // The leftover sub-pixel fraction lives in the resident touch accumulator (~0.2 px).
        let (h, _) = ms.touch_mouse.remainder();
        assert!(
            (h - 0.2).abs() < 1e-9,
            "carried remainder ≈ 0.2 after the pixel emit, got {h}"
        );
    }

    #[test]
    fn touchpad_touchdown_does_not_jump() {
        // A fresh touch-down (prev inactive) must NOT emit a jump from the origin.
        let rp = rp_touch(TouchpadSettings {
            as_mouse: true,
            sensitivity: 100.0,
            ..TouchpadSettings::default()
        });
        let mut ms = MapState::default();
        // prev_touch defaults to inactive; first contact at (900,500) must not produce motion.
        let (_, b) = apply(
            &touch_state(active_contact(1, 900, 500)),
            &rp,
            &mut ms,
            1_000,
        );
        assert!(
            b.as_slice()
                .iter()
                .all(|e| !matches!(e, KbmEvent::MouseMove { .. })),
            "touch-down (no prev contact) must not jump the cursor"
        );
    }

    #[test]
    fn default_profile_touch_emits_nothing_passthrough_unchanged() {
        // A fully default profile with an active touch contact is byte-identical passthrough.
        let rp = ResolvedProfile::default();
        let mut ms = MapState::default();
        let st = ControllerState {
            touch: [active_contact(1, 1800, 800), TouchContact::default()],
            lx: 0.3,
            ..Default::default()
        };
        let (out, b) = apply(&st, &rp, &mut ms, 1_000);
        assert_eq!(out, OutputState::passthrough(&st));
        assert!(b.is_empty(), "no touch binding -> no KBM events");
    }

    #[test]
    fn default_profile_gyro_emits_nothing_passthrough_unchanged() {
        // Sanity: a fully default profile (no gyro binding, gyro Off) with a strong gyro sample is
        // still byte-identical passthrough — no gyro leak into the output.
        let rp = ResolvedProfile::default();
        let mut ms = MapState::default();
        let st = ControllerState {
            motion: Motion {
                gyro_yaw: 9.0,
                gyro_pitch: 9.0,
                gyro_roll: 9.0,
                ..Default::default()
            },
            lx: 0.3,
            ..Default::default()
        };
        let (out, b) = apply(&st, &rp, &mut ms, 1_000);
        assert_eq!(out, OutputState::passthrough(&st));
        assert!(b.is_empty(), "no gyro binding -> no KBM events");
    }
}
