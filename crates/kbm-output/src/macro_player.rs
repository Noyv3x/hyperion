//! Macro playback on the injector thread (blueprint §6.1, §7.3, §12 M4).
//!
//! The hot loop only emits **macro start/stop edges** ([`KbmEvent::Macro`]); it never times a
//! macro. This module owns the *unbounded* part — the per-step schedule (key/mouse down/up + wait
//! delays) — entirely off the hot path. The injector thread holds one [`MacroPlayer`], hands it the
//! resolved profile's macro definitions on the config-generation gate ([`MacroPlayer::set_macros`]),
//! feeds it each [`KbmEvent::Macro`] edge it drains from the ring ([`MacroPlayer::on_edge`]), and
//! drives it on a timer ([`MacroPlayer::tick`]) so active macros advance step-by-step.
//!
//! ## Consumed types
//!
//! Playback consumes the **authoritative** [`MacroDef`]/[`MacroStep`] that live in
//! [`hyperion_core::map::profile`] (re-exported here for convenience). The engine hands the resolved
//! profile's `macros: Arc<[MacroDef]>` to the injector via [`MacroPlayer::set_macros`] — no parallel
//! step type is defined in this crate, so there is exactly one macro contract.
//!
//! ## Ported C# semantics (ground truth `Hyperion-ds4w/.../Mapping.cs PlayMacroCodeValue`)
//!
//! DS4Windows stores a macro as a flat `List<int>` of "code values" that toggle a per-macro
//! `keydown[]` flag (first occurrence presses, second releases), with `>= 300` codes meaning a
//! delay of `(code - 300)` ms executed as an async wait so the reading thread never blocks; at
//! macro end every still-held key is released (unless `keepKeyState`). We consume the cleaner
//! **structured** [`MacroStep`] form (`KeyDown`/`KeyUp`/`MouseDown`/`MouseUp`/`Wait`) the blueprint
//! pins for the GUI editor, but the playback contract is identical: down/up edges go straight to
//! `SendInput`, a `Wait` schedules the next step `ms` in the future, and on stop (falling edge or
//! natural end) every key/button the macro still holds is released so nothing sticks down. A macro
//! that has not finished when its trigger releases is **stopped immediately** and its held edges
//! released (the esports-safe choice: a held key must never outlive the press unless the macro
//! itself ended). [`MacroDef::repeat`] ports the DS4Windows run-while-held mode: a finished macro
//! that is still triggered re-runs from the top instead of going idle.
//!
//! Timing is HW-verify: the per-step `Wait` is realized against a monotonic [`Instant`] deadline,
//! and [`MacroPlayer::tick`] only advances steps whose deadline has passed, so the injector thread's
//! idle poll cadence bounds the jitter (the cadence, not this module, is the HW knob).

#![cfg(windows)]

use std::time::{Duration, Instant};

use hyperion_core::output::kbm::{KbmBatch, KbmEvent, KeyKind as InjectKind, MouseButton};

// The macro contract is owned by pure-core; re-export it so callers (the engine injector) can name
// `kbm_output::MacroDef`/`MacroStep` without reaching into core's module path.
pub use hyperion_core::map::profile::{MacroDef, MacroMouseButton, MacroStep};

use crate::{KbmSink, SendInputKbm};

/// Minimum delay (ms) between consecutive passes of a `repeat` macro whose tail scheduled no
/// `Wait`. Bounds a wait-less repeat to ~1 kHz instead of busy-looping the injector thread.
const MACRO_MIN_REPEAT_MS: u64 = 1;

/// Maximum number of distinct keys/buttons one running macro can hold at once, for the
/// end-of-macro release sweep. A macro pressing more than this many *distinct* keys without
/// releasing them is pathological; extra holds are still injected, only the auto-release sweep is
/// bounded (matching DS4Windows' fixed `keydown[]` array being the release source of truth).
const MAX_HELD_PER_MACRO: usize = 32;

/// A single in-flight macro instance: where it is in its step list and what it currently holds.
#[derive(Clone, Debug)]
struct Running {
    /// Index into the player's `macros` of the definition being played.
    def_idx: usize,
    /// Index of the next step to execute.
    step: usize,
    /// Deadline a pending `Wait` must pass before `step` advances. `None` => run now.
    wait_until: Option<Instant>,
    /// Keys currently held down BY THIS MACRO (`vk`, `scan_code`), released on stop/end if still
    /// held.
    held_keys: Vec<(u16, bool)>,
    /// Mouse buttons currently held down by this macro.
    held_mouse: Vec<MouseButton>,
    /// The macro is still triggered (its bound control is held); drives `repeat`.
    triggered: bool,
}

impl Running {
    fn new(def_idx: usize) -> Self {
        Self {
            def_idx,
            step: 0,
            wait_until: None,
            held_keys: Vec::with_capacity(MAX_HELD_PER_MACRO),
            held_mouse: Vec::with_capacity(5),
            triggered: true,
        }
    }
}

/// Drives macro playback on the injector thread.
///
/// Holds the active macro definitions (swapped on the config-generation gate) and the set of
/// in-flight instances. All injection goes through the same [`SendInputKbm`] the injector uses for
/// per-report edges, so the edge-dedupe / held-state bookkeeping is shared (a macro pressing a key
/// the report already holds is deduped, and `release_all()` lifts macro-held keys too).
pub struct MacroPlayer {
    /// Active macro definitions (immutable between config generations).
    macros: Vec<MacroDef>,
    /// In-flight macro instances (usually 0..a few).
    running: Vec<Running>,
}

impl MacroPlayer {
    /// An empty player (no macros defined, nothing running).
    pub fn new() -> Self {
        Self {
            macros: Vec::new(),
            running: Vec::new(),
        }
    }

    /// Install the resolved profile's macro definitions (called off the hot path, on the
    /// config-generation gate). Any currently-running instance is stopped first so a redefinition
    /// never leaves a key held from an old step list; the caller's `sink` releases those holds.
    ///
    /// Accepts anything iterable into [`MacroDef`] (the engine passes the resolved profile's
    /// `Arc<[MacroDef]>` via `.iter().cloned()`), keeping the crate boundary free of an `Arc`/slice
    /// type choice.
    pub fn set_macros<I>(&mut self, sink: &mut SendInputKbm, macros: I)
    where
        I: IntoIterator<Item = MacroDef>,
    {
        // Stop everything cleanly against the OLD defs before swapping (releases held edges).
        self.stop_all(sink);
        self.macros = macros.into_iter().collect();
    }

    /// Whether any macro is currently in flight (lets the injector keep ticking on a fast cadence
    /// only while playback is live, and fall back to the idle poll otherwise).
    #[inline]
    pub fn is_active(&self) -> bool {
        !self.running.is_empty()
    }

    /// Feed one [`KbmEvent::Macro`] edge drained from the ring.
    ///
    /// * `start: true` (rising edge) starts the macro if it is not already running; if it *is*
    ///   running it just re-marks it triggered (so a `repeat` macro keeps re-running).
    /// * `start: false` (falling edge) marks the instance untriggered. A `repeat` macro stops at the
    ///   end of its current pass; a non-repeat macro mid-flight is stopped immediately and its held
    ///   edges released (esports-safe: a held key must not outlive the press).
    pub fn on_edge(&mut self, sink: &mut SendInputKbm, id: u16, start: bool) {
        if start {
            self.start(id);
        } else {
            self.release_trigger(sink, id);
        }
    }

    /// Advance every in-flight macro whose pending `Wait` deadline has passed, injecting the
    /// resulting edges through `sink`. Returns the soonest future deadline across all running
    /// macros (so the injector can sleep until exactly then instead of busy-polling), or `None`
    /// when nothing is running.
    ///
    /// Call this once per injector wake-up; it is idempotent when nothing is due.
    pub fn tick(&mut self, sink: &mut SendInputKbm, now: Instant) -> Option<Instant> {
        let mut i = 0;
        while i < self.running.len() {
            // `advance_one` returns `true` when it finished (and `swap_remove`d) the instance at
            // `i`. On removal the slot now holds a not-yet-processed instance, so do NOT advance
            // `i`; otherwise step to the next one.
            if !self.advance_one(sink, i, now) {
                i += 1;
            }
        }
        self.next_deadline(now)
    }

    /// The soonest pending deadline at or after `now`, or `None` when nothing is running. A
    /// runnable-now instance (no pending wait) reports `now` so the injector ticks again at once.
    fn next_deadline(&self, now: Instant) -> Option<Instant> {
        let mut soonest: Option<Instant> = None;
        for r in &self.running {
            match r.wait_until {
                Some(t) if t > now => {
                    soonest = Some(match soonest {
                        Some(s) if s <= t => s,
                        _ => t,
                    });
                }
                // No wait (or an already-elapsed one): runnable now.
                _ => return Some(now),
            }
        }
        soonest
    }

    /// Start macro `id` if it has a definition and is not already running.
    fn start(&mut self, id: u16) {
        let Some(def_idx) = self.macros.iter().position(|m| m.id == id) else {
            return; // unknown id: silently ignore (matches DS4Windows' no-op on a missing macro)
        };
        if let Some(r) = self.running.iter_mut().find(|r| r.def_idx == def_idx) {
            // Already running: just keep it triggered (re-arms a `repeat`).
            r.triggered = true;
            return;
        }
        self.running.push(Running::new(def_idx));
        // Steps are injected on the next `tick` (a step with no leading Wait runs immediately).
    }

    /// Mark macro `id`'s running instance untriggered; stop a non-`repeat` macro immediately.
    fn release_trigger(&mut self, sink: &mut SendInputKbm, id: u16) {
        let Some(def_idx) = self.macros.iter().position(|m| m.id == id) else {
            return;
        };
        let Some(pos) = self.running.iter().position(|r| r.def_idx == def_idx) else {
            return;
        };
        self.running[pos].triggered = false;
        if !self.macros[def_idx].repeat {
            // Non-repeat macro: end of press == stop now, release everything it holds.
            self.stop_at(sink, pos);
        }
    }

    /// Run macro instance at `idx` forward until it hits a future `Wait`, finishes, or releases.
    /// Returns `true` if the instance finished and was removed (the slot at `idx` now holds a
    /// different, unprocessed instance via `swap_remove`); `false` if it is still live at `idx`.
    fn advance_one(&mut self, sink: &mut SendInputKbm, idx: usize, now: Instant) -> bool {
        // Honor a pending wait.
        if let Some(until) = self.running[idx].wait_until {
            if now < until {
                return false; // still waiting
            }
            self.running[idx].wait_until = None;
        }

        loop {
            let def_idx = self.running[idx].def_idx;
            let step_i = self.running[idx].step;
            let steps_len = self.macros[def_idx].steps.len();

            if step_i >= steps_len {
                // Reached the end. A `repeat` macro still triggered restarts from the top; otherwise
                // finish (releasing any keys still held).
                if self.macros[def_idx].repeat && self.running[idx].triggered {
                    // Restart the pass, but NEVER within this same tick. A repeat macro whose pass
                    // from the resume point to the end scheduled no `Wait` would otherwise spin
                    // forever here (busy-looping the injector thread, and hanging tests). Impose a
                    // minimum inter-pass delay so the injector re-ticks and runs the next pass then.
                    self.running[idx].step = 0;
                    self.running[idx].wait_until =
                        Some(now + Duration::from_millis(MACRO_MIN_REPEAT_MS));
                    return false;
                }
                self.stop_at(sink, idx);
                return true;
            }

            let step = self.macros[def_idx].steps[step_i];
            self.running[idx].step = step_i + 1;

            match step {
                MacroStep::KeyDown { vk, scan_code } => {
                    self.emit_key(sink, idx, vk, scan_code, true);
                }
                MacroStep::KeyUp { vk, scan_code } => {
                    self.emit_key(sink, idx, vk, scan_code, false);
                }
                MacroStep::MouseDown(b) => self.emit_mouse(sink, idx, b.to_mouse_button(), true),
                MacroStep::MouseUp(b) => self.emit_mouse(sink, idx, b.to_mouse_button(), false),
                MacroStep::Wait { ms } => {
                    // Schedule the deadline; stop running steps until a later tick passes it. A
                    // zero-ms wait is a no-op yield (still advances on the next loop turn).
                    if ms > 0 {
                        self.running[idx].wait_until =
                            Some(now + Duration::from_millis(u64::from(ms)));
                        return false;
                    }
                }
                // Unknown future step kinds are skipped (append-only forward-compat), matching the
                // core enum's documented "the injector skips it" contract.
                MacroStep::Unknown => {}
            }
        }
    }

    /// Inject one key edge and update this instance's held set, then flush the single edge.
    fn emit_key(
        &mut self,
        sink: &mut SendInputKbm,
        idx: usize,
        vk: u16,
        scan_code: bool,
        down: bool,
    ) {
        let r = &mut self.running[idx];
        if down {
            let already = r.held_keys.iter().any(|&(k, _)| k == vk);
            if !already && r.held_keys.len() < MAX_HELD_PER_MACRO {
                r.held_keys.push((vk, scan_code));
            }
        } else {
            r.held_keys.retain(|&(k, _)| k != vk);
        }
        flush_one(
            sink,
            KbmEvent::Key {
                vk,
                down,
                kind: inject_kind(scan_code),
            },
        );
    }

    /// Inject one mouse-button edge and update this instance's held set, then flush it.
    fn emit_mouse(&mut self, sink: &mut SendInputKbm, idx: usize, btn: MouseButton, down: bool) {
        let r = &mut self.running[idx];
        if down {
            if !r.held_mouse.contains(&btn) {
                r.held_mouse.push(btn);
            }
        } else {
            r.held_mouse.retain(|&b| b != btn);
        }
        flush_one(sink, KbmEvent::MouseButton { btn, down });
    }

    /// Stop the instance at `idx`: release every key/button it still holds and remove it.
    fn stop_at(&mut self, sink: &mut SendInputKbm, idx: usize) {
        // Take the instance out first so the borrow of `running` ends before flushing.
        let r = self.running.swap_remove(idx);
        let mut batch = KbmBatch::new();
        for (vk, scan_code) in r.held_keys {
            batch.push(KbmEvent::Key {
                vk,
                down: false,
                kind: inject_kind(scan_code),
            });
        }
        for btn in r.held_mouse {
            batch.push(KbmEvent::MouseButton { btn, down: false });
        }
        if !batch.is_empty() {
            let _ = sink.flush(&batch);
        }
    }

    /// Stop every in-flight macro, releasing all held edges (config swap / shutdown).
    pub fn stop_all(&mut self, sink: &mut SendInputKbm) {
        while !self.running.is_empty() {
            self.stop_at(sink, self.running.len() - 1);
        }
    }
}

impl Default for MacroPlayer {
    fn default() -> Self {
        Self::new()
    }
}

/// Map a macro step's `scan_code` flag to the injector's scancode-vs-virtual selector — matching
/// DS4Windows' `keyType.HasFlag(DS4KeyType.ScanCode)` branch in `PlayMacroCodeValue`.
#[inline]
fn inject_kind(scan_code: bool) -> InjectKind {
    if scan_code {
        InjectKind::ScanCode
    } else {
        InjectKind::Virtual
    }
}

/// Flush a single KBM event through the sink (one `SendInput`). A macro-step injection failure is
/// rare and the next step retries the shared dedupe state, so the error is dropped on this cold
/// path.
#[inline]
fn flush_one(sink: &mut SendInputKbm, ev: KbmEvent) {
    let mut batch = KbmBatch::new();
    batch.push(ev);
    let _ = sink.flush(&batch);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 3-step macro: press A, wait 10ms, release A.
    fn macro_a(id: u16, repeat: bool) -> MacroDef {
        MacroDef {
            id,
            name: String::new(),
            repeat,
            steps: vec![
                MacroStep::KeyDown {
                    vk: 0x41,
                    scan_code: false,
                },
                MacroStep::Wait { ms: 10 },
                MacroStep::KeyUp {
                    vk: 0x41,
                    scan_code: false,
                },
            ],
        }
    }

    #[test]
    fn unknown_macro_id_is_a_noop() {
        let mut sink = SendInputKbm::new();
        let mut p = MacroPlayer::new();
        p.set_macros(&mut sink, [macro_a(1, false)]);
        // Edge for id 99 (undefined): nothing starts.
        p.on_edge(&mut sink, 99, true);
        assert!(!p.is_active());
    }

    #[test]
    fn start_runs_until_first_wait_then_blocks_until_deadline() {
        let mut sink = SendInputKbm::new();
        let mut p = MacroPlayer::new();
        p.set_macros(&mut sink, [macro_a(1, false)]);
        let t0 = Instant::now();

        // Rising edge arms the macro; first tick runs KeyDown then hits the 10ms Wait.
        p.on_edge(&mut sink, 1, true);
        assert!(p.is_active(), "macro is in flight after the start edge");
        let next = p.tick(&mut sink, t0);
        assert!(p.is_active(), "still running, parked on the Wait");
        // The returned deadline is ~10ms out.
        let until = next.expect("a pending Wait yields a future deadline");
        assert!(until > t0, "deadline is in the future");

        // A tick BEFORE the deadline does not advance past the wait (still active).
        p.tick(&mut sink, t0 + Duration::from_millis(5));
        assert!(p.is_active(), "still parked before the 10ms deadline");

        // A tick AFTER the deadline runs the final KeyUp and finishes the macro.
        p.tick(&mut sink, t0 + Duration::from_millis(11));
        assert!(!p.is_active(), "macro finished after the wait elapsed");
    }

    #[test]
    fn non_repeat_macro_stops_immediately_on_trigger_release() {
        let mut sink = SendInputKbm::new();
        let mut p = MacroPlayer::new();
        p.set_macros(&mut sink, [macro_a(1, false)]);
        let t0 = Instant::now();
        p.on_edge(&mut sink, 1, true);
        p.tick(&mut sink, t0); // KeyDown, parked on Wait, A is held by the macro.
        assert!(p.is_active());
        // Releasing the trigger mid-flight stops a non-repeat macro at once (A released).
        p.on_edge(&mut sink, 1, false);
        assert!(!p.is_active(), "non-repeat macro stops on trigger release");
    }

    #[test]
    fn repeat_macro_runs_while_triggered_then_ends_on_release() {
        let mut sink = SendInputKbm::new();
        let mut p = MacroPlayer::new();
        // A 1-step macro (just press B) with repeat: it should re-fire across ticks while held.
        let m = MacroDef {
            id: 2,
            name: String::new(),
            repeat: true,
            steps: vec![MacroStep::KeyDown {
                vk: 0x42,
                scan_code: false,
            }],
        };
        p.set_macros(&mut sink, [m]);
        let mut t = Instant::now();
        p.on_edge(&mut sink, 2, true);
        // Several ticks while held: the macro keeps wrapping (stays active, no idle).
        for _ in 0..3 {
            p.tick(&mut sink, t);
            assert!(p.is_active(), "repeat macro keeps running while triggered");
            t += Duration::from_millis(1);
        }
        // Release: the current pass finishes and the macro ends (releasing B).
        p.on_edge(&mut sink, 2, false);
        p.tick(&mut sink, t);
        assert!(!p.is_active(), "repeat macro ends after release");
    }

    #[test]
    fn set_macros_stops_in_flight_playback() {
        let mut sink = SendInputKbm::new();
        let mut p = MacroPlayer::new();
        p.set_macros(&mut sink, [macro_a(1, false)]);
        p.on_edge(&mut sink, 1, true);
        p.tick(&mut sink, Instant::now());
        assert!(p.is_active());
        // Swapping defs stops everything cleanly.
        p.set_macros(&mut sink, [macro_a(1, false)]);
        assert!(!p.is_active(), "config swap stops in-flight macros");
    }

    #[test]
    fn zero_wait_is_a_passthrough_yield() {
        let mut sink = SendInputKbm::new();
        let mut p = MacroPlayer::new();
        // KeyDown, Wait 0, KeyUp — a zero wait should not park; one tick runs all three.
        let m = MacroDef {
            id: 3,
            name: String::new(),
            repeat: false,
            steps: vec![
                MacroStep::KeyDown {
                    vk: 0x43,
                    scan_code: false,
                },
                MacroStep::Wait { ms: 0 },
                MacroStep::KeyUp {
                    vk: 0x43,
                    scan_code: false,
                },
            ],
        };
        p.set_macros(&mut sink, [m]);
        p.on_edge(&mut sink, 3, true);
        p.tick(&mut sink, Instant::now());
        assert!(!p.is_active(), "a zero-wait macro completes in one tick");
    }

    #[test]
    fn mouse_steps_play() {
        let mut sink = SendInputKbm::new();
        let mut p = MacroPlayer::new();
        let m = MacroDef {
            id: 4,
            name: String::new(),
            repeat: false,
            steps: vec![
                MacroStep::MouseDown(MacroMouseButton::Left),
                MacroStep::MouseUp(MacroMouseButton::Left),
            ],
        };
        p.set_macros(&mut sink, [m]);
        p.on_edge(&mut sink, 4, true);
        p.tick(&mut sink, Instant::now());
        assert!(
            !p.is_active(),
            "a down/up mouse macro completes in one tick"
        );
    }

    #[test]
    fn unknown_step_is_skipped() {
        let mut sink = SendInputKbm::new();
        let mut p = MacroPlayer::new();
        let m = MacroDef {
            id: 5,
            name: String::new(),
            repeat: false,
            steps: vec![MacroStep::Unknown],
        };
        p.set_macros(&mut sink, [m]);
        p.on_edge(&mut sink, 5, true);
        p.tick(&mut sink, Instant::now());
        assert!(
            !p.is_active(),
            "a macro of only unknown steps completes at once"
        );
    }
}
