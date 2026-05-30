# Hyperion — DESIGN-REMAP.md (full DS4Windows-class remapper)

> Implementation-ready blueprint. Extends `DESIGN.md` (the M1 RC-filter vertical slice) to the
> full remapper. Merges **5 design tracks** (state model, stick/trigger pipeline, mapping engine,
> output backends, profiles+config+auto-switch+GUI) reconciled against **2 adversarial
> verifications** (MAPPING-ENGINE vs `Hyperion-ds4w/Hyperion/DS4Control/Mapping.cs`, and a
> real-time latency review of the hot-loop integration). Where tracks conflicted, the
> verifier-corrected option wins; see **§13 Resolved Conflicts**. Everything here preserves the
> `DESIGN.md` invariants verbatim: pure `f64` OS-free core, single i16 egress quantization, the
> lock-free `arc-swap`/generation/`ConfigStore`-single-writer topology, and the alloc-free hot loop.
>
> **Ground truth for offsets/semantics:** the existing core (`crates/core/src/input/ds_report.rs`,
> `output/mod.rs`, `config.rs`, `rc/`), the existing engine (`hot.rs`, `config_store.rs`,
> `handoff.rs`, `win_io.rs`, `control.rs`), and `Hyperion-ds4w/Hyperion/DS4Control/Mapping.cs` +
> `ScpUtil.cs` + `ProfilePropGroups.cs` + `StickOutCurve.cs` for the C# remapper semantics.

---

## 1. Feature scope vs DS4Windows

What the full remapper covers (✓ shipped by milestone, see §12), against DS4Windows feature parity.

| Area | Feature | Parity | Milestone |
|------|---------|--------|-----------|
| **Sticks** | Radial **and** axial deadzone (+ anti-deadzone, max-zone, max-output, vertical-scale) | full | M3 |
| | Sensitivity (radial-only, C# quirk preserved) | full | M3 |
| | Rotation, anti-snapback, input-fuzz | full | M3 |
| | Square-stick (CircleToSquare + roundness) | full | M3 |
| | Output curves: Linear / EnhancedPrecision / Quadratic / Cubic / EaseoutQuad / EaseoutCubic / Bezier / ApexClassicInverse(+Axial) | full | M3 |
| | RC filter (FireBird / UltimateLegacy / **UltimateDt**) as stage 0 | superset | M3 (exists) |
| | Flick stick (relative-aim terminal stage → mouse path) | full | M5 |
| **Triggers** | Deadzone / max-zone / max-output / anti-deadzone / sensitivity / curve | full | M3 |
| | Trigger→button (full-pull `==255` + threshold digital) | full | M3 |
| | Two-stage / hip-fire trigger modes | full | M6 |
| **Buttons** | button→button (gamepad remap, identity-suppress) | full | M3 |
| | button→key, button→mouse-button | full | M3 (key) / M4 (mouse) |
| | ScanCode vs VK, Toggle, hold-key | full | M3/M4 |
| | Turbo / rapid-fire (per-binding) — **NET-NEW, no C# reference** | superset | M4 |
| | Macros (timed key/mouse sequences, OnPress/Hold/Toggle/Repeat) | full | M4 |
| | Shift layers (per-control shift trigger — faithful model) | full | M4 |
| | Special actions (profile-switch, launch program, disconnect) | partial | M4/M5 |
| **Mouse** | mouse-from-stick (remainder-carry, anti-deadzone, jitter-comp) | full | M4 |
| | mouse-from-gyro (rad/s → dx/dy, OneEuro-class smoothing) | full | M5 |
| | mouse wheel from binding | full | M4 |
| **Gyro** | gyro→mouse / gyro→stick, gyro activation trigger | full | M5 |
| **Output** | virtual **X360** pad (exists) | full | M3 (exists) |
| | virtual **DS4** pad (u8 sticks, dpad nibble, special byte, touchpad) | full | M5 |
| | KBM injection via batched SendInput (edge-deduped, scancode) | full | M3 (key) |
| **Profiles** | named profiles, per-device assignment, output-kind per profile | full | M5 |
| | auto-profile-switch by foreground exe/title | full | M5 |
| | per-profile sticks/triggers/bindings/mouse/gyro/macros/specials | full | M5 |
| **Touchpad** | touchpad as buttons / regions / touch-to-mouse | partial | M6 |
| **Lightbar / rumble / device LED** | output feedback | **out of scope v1** (flagged) | — |

**Deliberate divergences from DS4Windows (documented, tested):**
1. **No mid-chain quantization.** C# truncates sticks/triggers to bytes between stages; we keep
   `f64` end-to-end and round **once** at the i16/u8 egress (`DESIGN.md` §4.1). Trigger/stick
   analog output can differ from DS4Windows by ≤1/255; pinned in tests as intended, not a bug.
2. **Turbo is net-new.** Grep of the entire `Hyperion-ds4w` tree returns zero `turbo` references
   — DS4Windows in this lineage has no per-binding rapid-fire. The duty-cycle model is our
   invention; it has no golden to port against and is validated by a standalone phase test (verifier FIX 2).
3. **Deterministic anti-snapback / mouse carry.** C# uses `DateTimeOffset.Now` + unbounded
   `Queue`; we use a `dt`-accumulated `elapsed_us` + fixed-capacity ring so the core stays pure,
   alloc-free, and Linux-testable.
4. **Single TOML tree**, not DS4Windows' multi-XML (`Profiles.xml`/`Actions.xml`/`AutoProfiles.xml`).

---

## 2. Crate + module layout (added / changed)

No new **crate** is required for the core engine; two thin Windows shell additions. Direction is
unchanged: `core` depends on nothing OS; `engine` depends on `core` always + the win crates under
`cfg(windows)`.

```
crates/core/  (hyperion_core — pure, Linux-CI-tested)
  src/input/
    mod.rs            (EXTEND) + parse_controller_state(), ReportMeta; keep InputSample as projection
    ds_report.rs      (EXTEND) + decode_controller_state(): buttons/sensors/touch on top of DsReport
    control.rs        (NEW)    Control enum (repr(u8), the single mapping-table key), ControlKind, Thresholds
    state.rs          (NEW)    ControllerState (decoded physical report), Motion, TouchContact
  src/output/
    mod.rs            (KEEP)   OutputFrame + to_xinput_thumb/_trigger UNCHANGED
    state.rs          (NEW)    OutputState (structured virtual-pad accumulator), PadButtons, PadTarget,
                               to_ds4_axis, dpad_8way (pure, Linux-tested; DS4 backend calls them)
    kbm.rs            (NEW)    KbmEvent, MouseButton, KbmBatch (fixed-cap, no alloc)
  src/stick/
    settings.rs       (NEW)    StickSettings + the per-stage param structs, OutputCurve, DeadZoneType
    pipeline.rs       (NEW)    process_stick() — the full ordered DS4Windows chain (f64 [0,255] domain)
    stages.rs         (NEW)    pure stage helpers (rotate/snapback/fuzz/deadzone/sensitivity/square/curve)
    (existing stick.rs keeps StickAlgorithm/StickSample; settings/pipeline are siblings)
  src/trigger.rs      (NEW)    TriggerSettings, TriggerCurve, process_trigger()
  src/map/
    mod.rs            (NEW)    re-exports
    binding.rs        (NEW)    Binding, BindingSlot, KeyKind, TurboCfg, DigitalThreshold, XButton, etc.
    profile.rs        (NEW)    Profile (editable serde) + ResolvedProfile (hot-facing flat arrays)
    state.rs          (NEW)    MapState (turbo/toggle/edge/mouse-accum latches, all Copy/fixed)
    engine.rs         (NEW)    apply() — the pure remap entry point
  src/mouse_accum.rs  (NEW)    MouseAccumulator (stick→mouse + gyro→mouse remainder-carry)
  src/autoswitch.rs   (NEW)    match_rules() pure foreground→profile matcher
  src/config.rs       (REWRITE) full profile tree (see §3.6, §9); DeviceConfig loses ls/rs

crates/engine/  (always-core + cfg(windows) shells)
  src/hot.rs          (EXTEND) resident MapState/StickState/TriggerState + apply() seam; KbmBatch egress
  src/handoff.rs      (EXTEND) + KbmTx/KbmRx (third SPSC rtrb ring, hot → injector)
  src/control.rs      (EXTEND) + profile/binding/assignment/auto-switch ControlMsg variants
  src/config_store.rs (EXTEND) edit_in_place arms for the new messages; validate() rebuilds `resolved`
  src/win_io.rs       (EXTEND) thread raw_buttons:u32; DynPad dispatch; wire KbmInjector
  src/supervisor.rs   (EXTEND) [cfg(windows)] ForegroundWatcher (auto-switch) + DynPad replug
  src/runtime.rs      (EXTEND) spawn ForegroundWatcher; clone control_tx

crates/vgamepad-output/  (cfg(windows))
  src/lib.rs          (EXTEND) VirtualPad::update(&OutputState); + VigemDs4Pad; DynPad{X360,Ds4}

crates/kbm-output/   (NEW crate, cfg(windows))   SendInputKbm (KbmSink impl), scancode_from_vk
                                                  windows features: Win32_UI_Input_KeyboardAndMouse, Win32_Foundation

crates/platform-win/ (cfg(windows))
  src/foreground.rs   (NEW)    read_foreground(): GetForegroundWindow→exe path+title

crates/app/  (cfg(windows) bin)
  src/gui/mod.rs      (EXTEND) tab bar + ProfileMirror (replaces DeviceMirror)
  src/gui/mapping.rs  (NEW)    binding editor / controller diagram
  src/gui/sticks.rs   (NEW)    full stick settings (RC panel reused as a sub-section)
  src/gui/triggers.rs (NEW)    trigger settings
  src/gui/mouse_gyro.rs (NEW)  mouse-from-stick + gyro settings
  src/gui/macros.rs   (NEW)    macro list + step editor
  src/gui/profiles.rs (NEW)    profile manager + auto-switch rule table
  src/gui/panels.rs   (KEEP)   RC/thread/hidhide panels reused
```

`core/src/lib.rs` adds:
```rust
pub mod stick { pub mod settings; pub mod pipeline; pub mod stages; }  // or flat modules
pub mod trigger;
pub mod map;
pub mod mouse_accum;
pub mod autoswitch;
pub use input::{Control, ControlKind, Thresholds, ControllerState, Motion, TouchContact, ReportMeta};
pub use output::{OutputState, PadButtons, PadTarget, KbmBatch, KbmEvent, MouseButton};
pub use stick::settings::StickSettings;
pub use trigger::TriggerSettings;
pub use map::{Binding, Profile, ResolvedProfile, MapState, apply};
```

---

## 3. Authoritative data model (concrete Rust)

This section is the **single source of truth** for every type the other tracks referenced under
different names. Conflicts resolved: there is **ONE** `Control` enum (Track 1's superset + Track 3's
trimmed subset reconciled — see §3.1), **ONE** `Binding` (Track 3's closed enum + the verifier's
added `TouchpadClick`/`KeyUnbound`), **ONE** `OutputState`/`PadButtons` (Track 4's, which Track 1's
`OutputState` and Track 3's `OutputState` both lower into), and **ONE** `StickSettings` (Track 2's
`[0,255]`-domain pipeline). The verifier corrections are baked in: **fixed arrays keyed by `Control`,
no `HashMap`/`BTreeMap` on the hot path; `apply()` is pure and alloc-free.**

### 3.1 `Control` — the mapping-table key (`core/src/input/control.rs`)

One `#[repr(u8)]` dense enum mirrors `DS4Controls`. It is the index for every `[T; Control::COUNT]`
table. Stick half-axes and triggers appear as BOTH analog axes and digital/directional controls.

```rust
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum Control {
    None = 0,
    // analog stick half-axes (AxisDir) — split so a direction maps independently
    LxNeg, LxPos, LyNeg, LyPos, RxNeg, RxPos, RyNeg, RyPos,           // 1..=8
    // shoulders / triggers (analog) / stick clicks
    L1, L2, L3, R1, R2, R3,                                            // L2/R2 are the analog triggers
    // face
    Square, Triangle, Circle, Cross,
    // dpad
    DpadUp, DpadRight, DpadDown, DpadLeft,
    // system
    Ps, Share, Options, Mute, Capture,
    // DualSense Edge (gated by SourceMeta capability; see §3.5 RISK)
    FnL, FnR, Blp, Brp, SideL, SideR,
    // trigger-as-button (full pull == 255)
    L2FullPull, R2FullPull,
    // stick outer-ring as trigger
    LsOuter, RsOuter,
    // touchpad click + finger-region buttons
    TouchButton, TouchLeft, TouchRight, TouchUpper, TouchMulti,
    // gyro directions (gyro-as-button / gyro→mouse trigger)
    GyroXPos, GyroXNeg, GyroZPos, GyroZNeg,
    // APPEND-ONLY below this line (serde key / table index stability); never renumber the above.
}
impl Control {
    pub const COUNT: usize = Self::GyroZNeg as usize + 1;   // table size
    #[inline] pub const fn as_index(self) -> usize { self as usize }
    pub fn kind(self) -> ControlKind { /* match -> Button|AxisDir|Trigger|Touch|GyroDir */ }
    /// For half-axis identity suppression (verifier FIX 1): which OutputState stick axis this
    /// control belongs to (0=Lx,1=Ly,2=Rx,3=Ry), or None for non-stick controls.
    pub fn stick_axis(self) -> Option<usize> { /* LxNeg|LxPos -> 0, LyNeg|LyPos -> 1, ... */ }
    pub const ALL: [Control; Self::COUNT] = [/* dense 0..COUNT */];
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControlKind { Button, AxisDir, Trigger, Touch, GyroDir }

/// Per-kind digital activation thresholds. **Verifier FIX 3:** axes use 55/127 (GetBoolMapping
/// `cState.LX < 128-55`), triggers use 100/255 (`triggers > 100`) — these are NOT the same value.
#[derive(Clone, Copy, Debug)]
pub struct Thresholds { pub stick_dir: f64, pub trigger: f64, pub gyro_dir: f64 }
impl Thresholds {
    pub const STICK_DIR_DEFAULT: f64 = 55.0 / 127.0;  // ≈ 0.4331
    pub const TRIGGER_DEFAULT:   f64 = 100.0 / 255.0; // ≈ 0.3922
}
impl Default for Thresholds {
    fn default() -> Self { Self { stick_dir: Self::STICK_DIR_DEFAULT, trigger: Self::TRIGGER_DEFAULT, gyro_dir: 0.5 } }
}
```

> **CONFLICT RESOLVED (Control surface).** Track 1 enumerates Edge/touch/gyro controls; Track 3
> trims to a 29-variant subset "what the current DualSense decode produces"; Track 5 (`DsControl`)
> sits in between. **Winner: Track 1's superset**, because keeping the enum stable and append-only
> matters once it is a serde key, and the touch/gyro variants are needed by M5/M6 without
> renumbering persisted profiles. Variants whose physical bit is not yet decoded (Edge Fn/paddles,
> touch regions, gyro) are **gated by a `SourceMeta` capability flag** and read `false`/`0.0` until
> the decode lands — they are valid table indices immediately but inert (verifier FIX 5).

### 3.2 `ControllerState` — the decoded physical report (`core/src/input/state.rs`)

```rust
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TouchContact { pub is_active: bool, pub id: u8, pub x: u16, pub y: u16 } // x:0..1919 y:0..941

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Motion {
    pub gyro_yaw: f64, pub gyro_pitch: f64, pub gyro_roll: f64,   // rad/s
    pub accel_x: f64,  pub accel_y: f64,    pub accel_z: f64,     // g
    pub gyro_raw: [i16; 3], pub accel_raw: [i16; 3],              // raw ticks (fidelity)
}

/// Fully-decoded physical report. Copy, ~200 bytes, alloc-free. Sticks [-1,1] (+y==up),
/// triggers [0,1] analog + u8 raw (raw==255 => FullPull).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ControllerState {
    pub lx: f64, pub ly: f64, pub rx: f64, pub ry: f64,
    pub l2: f64, pub r2: f64, pub l2_raw: u8, pub r2_raw: u8,
    pub square: bool, pub triangle: bool, pub circle: bool, pub cross: bool,
    pub dpad_up: bool, pub dpad_down: bool, pub dpad_left: bool, pub dpad_right: bool,
    pub l1: bool, pub r1: bool, pub l3: bool, pub r3: bool,
    pub ps: bool, pub share: bool, pub options: bool, pub mute: bool, pub capture: bool,
    pub fn_l: bool, pub fn_r: bool, pub blp: bool, pub brp: bool, pub side_l: bool, pub side_r: bool,
    pub touch_button: bool, pub touch: [TouchContact; 2],
    pub motion: Motion,
}
impl ControllerState {
    /// Analog view in native unit (sticks signed [-1,1], triggers/outer [0,1], gyro rad/s, buttons 0/1).
    pub fn analog(&self, c: Control) -> f64 { /* LxPos => self.lx.max(0.0), LxNeg => (-self.lx).max(0.0), L2 => self.l2, ... */ }
    /// Digital "pressed" view, applying kind-dependent Thresholds (verifier FIX 3).
    pub fn pressed(&self, c: Control, t: &Thresholds) -> bool { /* L2FullPull => l2_raw==255; AxisDir => component >= t.stick_dir; Trigger => l2 >= t.trigger */ }
    /// Cheap projection to the stick-only InputSample the existing hot loop consumes (no regression).
    pub fn to_input_sample(&self, m: &ReportMeta) -> super::InputSample { /* sticks/triggers/raw DS buttons + seq/dt */ }
}
```

### 3.3 `OutputState` + `PadButtons` — virtual-pad accumulator (`core/src/output/state.rs`)

This is the **single** egress value type (Track 4's), superseding `OutputFrame` as the thing the
hot loop builds. `OutputFrame` is **kept unchanged** as the X360 projection so the single-round
invariant and all existing `to_xinput_*` tests survive untouched.

```rust
/// Virtual-controller-agnostic button set the mapping engine fills (Track 4).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PadButtons(pub u32);
impl PadButtons {
    pub const A: u32=1<<0; pub const B: u32=1<<1; pub const X: u32=1<<2; pub const Y: u32=1<<3;
    pub const LB: u32=1<<4; pub const RB: u32=1<<5; pub const BACK: u32=1<<6; pub const START: u32=1<<7;
    pub const LS: u32=1<<8; pub const RS: u32=1<<9; pub const GUIDE: u32=1<<10;
    pub const DPAD_UP: u32=1<<11; pub const DPAD_DOWN: u32=1<<12; pub const DPAD_LEFT: u32=1<<13; pub const DPAD_RIGHT: u32=1<<14;
    pub const L2_CLICK: u32=1<<15; pub const R2_CLICK: u32=1<<16; pub const TOUCHPAD: u32=1<<17; // verifier FIX 5: TouchpadClick output
    #[inline] pub fn has(self, m: u32) -> bool { self.0 & m != 0 }
    #[inline] pub fn set(&mut self, m: u32, on: bool) { if on { self.0 |= m } else { self.0 &= !m } }
}

/// Full processed pad state. Sticks [-1,1] (+y up), triggers [0,1]. Copy, alloc-free.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct OutputState {
    pub lx: f64, pub ly: f64, pub rx: f64, pub ry: f64,
    pub lt: f64, pub rt: f64,
    pub buttons: PadButtons,
}
impl OutputState {
    /// Identity passthrough seed (no remaps): copy sticks/triggers + digital buttons from decoded state.
    pub fn passthrough(s: &ControllerState) -> Self { /* ... */ }
    /// Project to the X360 frame. The i16/u8 round still happens ONLY in the backend via
    /// to_xinput_thumb/_trigger; this packs the XInput button u16 and copies f64 sticks/triggers.
    pub fn to_output_frame(&self) -> OutputFrame { OutputFrame { lx:self.lx, ly:self.ly, rx:self.rx, ry:self.ry, lt:self.lt, rt:self.rt, buttons: pack_xinput(self.buttons) } }
}
/// PadButtons -> XInput u16 (same button list as the existing win_io::ds_buttons_to_xinput / C#
/// Xbox360OutDevice): A=Cross, B=Circle, X=Square, Y=Triangle, Back=Share, Start=Options,
/// Guide=PS, LS/RS thumbs, LB/RB shoulders, dpad. L2_CLICK/R2_CLICK/TOUCHPAD have no X360 bit.
pub fn pack_xinput(b: PadButtons) -> u16 { /* ... */ }

/// DS4 wire mapping (pure, Linux-tested; the cfg(windows) DS4 backend calls these).
#[inline] pub fn to_ds4_axis(n: f64, flip_y: bool) -> u8 { let v = if flip_y { -n } else { n }; (((v.clamp(-1.0,1.0)+1.0)*0.5)*255.0).round().clamp(0.0,255.0) as u8 }
#[inline] pub fn dpad_8way(u:bool,d:bool,l:bool,r:bool) -> Ds4Dpad { /* DS4OutDeviceBasic if/else ladder */ }

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum PadTarget { #[default] X360, Ds4 }
```

### 3.4 `KbmBatch` — fixed-capacity KBM accumulator (`core/src/output/kbm.rs`)

**Verifier (latency) FIX 4: the cap is PINNED so the `rtrb<KbmBatch>` element and the hot-thread
push are bounded.**

```rust
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KbmEvent {
    Key { vk: u16, down: bool, kind: KeyKind },   // vk + scancode/extended decided in the sink
    MouseButton { btn: MouseButton, down: bool },
    MouseMove { dx: i32, dy: i32 },               // relative, accumulated from stick/gyro this report
    Wheel { vertical: i32, horizontal: i32 },     // WHEEL_DELTA units (±120)
    Macro { id: u16, start: bool },               // edge; playback timing owned by the injector thread
    Special { id: u16 },                          // routed to control plane, not the KBM injector
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)] pub enum MouseButton { Left, Right, Middle, X1, X2 }

pub const KBM_BATCH_CAP: usize = 24;   // 6 HID key edges + mouse btns + move + wheel + shift churn
#[derive(Clone, Copy, Debug)]
pub struct KbmBatch { buf: [KbmEvent; KBM_BATCH_CAP], len: u8 }   // ~200 B, Copy
impl KbmBatch {
    pub const fn new() -> Self { /* len 0 */ }
    #[inline] pub fn clear(&mut self) { self.len = 0; }
    #[inline] pub fn push(&mut self, e: KbmEvent) -> bool { /* false if full (saturate, no panic/alloc) */ }
    #[inline] pub fn as_slice(&self) -> &[KbmEvent] { &self.buf[..self.len as usize] }
    #[inline] pub fn is_empty(&self) -> bool { self.len == 0 }
}
```

### 3.5 Parse extension (`core/src/input/ds_report.rs` + `input/mod.rs`)

Keep `parse_ds_usb_report` + `ds_report_to_sticks` (annotated SOLID for sticks) untouched. Layer on:

```rust
/// Decode buttons + sensors + touch from an already-parsed DsReport (+ full buf tail) into the
/// structured ControllerState. Reuses ds_report_to_sticks for LX/LY/RX/RY (no duplicate offsets).
/// `meta` supplies the Edge capability flag + which fields exist. New code is the HW-verify part.
pub fn decode_controller_state(r: &DsReport, buf: &[u8], meta: &crate::input::SourceMeta) -> ControllerState;

// input/mod.rs
#[derive(Clone, Copy, Debug, Default)]
pub struct ReportMeta { pub seq: u8, pub dropped: u16, pub is_duplicate: bool, pub dt_us: f64, pub host_qpc_ns: u64 }

/// Full parse entry: raw buf + host time -> (decoded state, derived meta). None on short/wrong-id
/// (same guard as parse_ds_usb_report). Folds the existing SeqTracker + SensorClock, unchanged.
pub fn parse_controller_state(buf: &[u8], host_qpc_ns: u64, meta: &SourceMeta, seq: &mut SeqTracker, clock: &mut SensorClock) -> Option<(ControllerState, ReportMeta)>;
```

**The btn0/btn1/btn2 → ControllerState bit map is defined NOW** (verifier FIX 5 — `apply()` cannot
read an opaque `Buttons(u32)`). It reuses the exact layout already encoded in
`engine/src/win_io.rs::ds_buttons_to_xinput`, promoted into core:
- `btn0` (byte 5): low nibble = dpad hat (`0..7` 8-way, `8`=neutral → decode to 4 bools); high nibble = Square `0x10`, Cross `0x20`, Circle `0x40`, Triangle `0x80`.
- `btn1` (byte 6): L1 `0x01`, R1 `0x02`, L2-click `0x04`, R2-click `0x08`, Share `0x10`, Options `0x20`, L3 `0x40`, R3 `0x80`.
- `btn2` (byte 7): PS `0x01`, TouchButton `0x02` (bits 2..8 = frame counter, already consumed).
- **HW-verify (gated by `SourceMeta` Edge flag):** Mute/Capture and Edge Fn/paddle bits live in the
  Edge superset (extended report); `decode_controller_state` reads them only when the capability is
  set, else `false`. The Control variants exist and are valid indices regardless.

**RISK (carried from Track 1):** dpad is a 4-bit hat nibble, not 4 independent bits — unit-test all
9 nibble values. Touch contact packing (active flag, 7-bit id, 12-bit X/Y across 3 bytes/finger in
the tail) and gyro/accel raw-i16→rad/s scale are HW-verify; raw `i16` is retained so the scale can
be corrected without re-decoding.

---

## 4. Stick / trigger pipeline (exact stage order)

The whole C# `SetCurveAndDeadzone` chain runs in the **DS4 `[0,255]` f64 domain** (128 neutral),
entered/exited **once** via the existing `core::convert::{axis_to_ds4, ds4_to_axis}` so the C#
goldens port 1:1 with zero algebraic re-derivation and zero mid-chain quantization.

> **CONFLICT RESOLVED (StickSettings shape).** Track 2 ports the chain in the `[0,255]` domain with
> concrete C# defaults (roundness 5.0, anti-snapback delta 135 / 50ms); Track 5 normalizes to
> `[0,1]` and carries an explicit `stage_order: Vec<StageKind>`. **Winner: Track 2's `[0,255]`-domain
> pipeline** (golden-portable, no off-by-asymmetry class of bugs), placed **inside the `Profile`**
> per Track 5 (sticks move out of `DeviceConfig`). The order is fixed (matches C#), so no
> `stage_order` Vec — a fixed order is auditable and avoids a per-report branch on absent stages.

**Per-stick stage order (process_stick):**

```
stage 0  RC filter        — reuse the existing bit-exact RcFilter (StickAlgorithm) as a sub-step.
                             RcConfig becomes StickSettings.rc; gate = rc_mode_on && rc.enabled.
                             (canonical [-1,1] domain, BEFORE entering [0,255])
--- enter [0,255] ONCE via axis_to_ds4 ---
stage 1  rotation         — DS4State.rotateLSCoordinates, clamp -128..127 then +128
stage 2  anti-snapback    — fixed-cap ring + 15-unit middle-circle segment test (dt-accumulated time)
stage 3  fuzz             — CalcStickAxisFuzz delta^2 gate, endpoint passthrough
stage 4  calibration      — identity hook (device-level; omitted v1)
stage 5  deadzone         — radial (fused anti-dz + max-zone + max-output + vertical-scale)
                             OR axial (per-axis dz/anti/max), per DeadZoneType
stage 6  sensitivity      — RADIAL-ONLY (C# quirk preserved; axial silently ignores it)
stage 7  square stick     — CircleToSquare with roundness
stage 8  output curve     — Mapping.ApplyStickOutputCurve form (cap-aware), NOT bare CalcOutValue
--- exit [0,255] ONCE via ds4_to_axis ---
stage 9  flick stick      — terminal; writes st.flick_delta for the mouse/gyro path, returns the
                             absolute stick unchanged (does NOT fold into x/y)
```

**Trigger pipeline (process_trigger, f64 end-to-end, single quantization deferred to egress):**

```
deadzone -> max-zone -> max-output -> anti-deadzone -> sensitivity -> output curve -> to-button threshold
```

```rust
// core/src/stick/pipeline.rs
pub fn process_stick(raw: StickSample, cfg: &StickSettings, st: &mut StickState, dt: Dt) -> StickSample;
// core/src/trigger.rs
/// raw in [0,1]; returns (analog_out [0,1], digital_pressed). to_button compares raw255 vs
/// max(button_threshold, dead_zone) (pre-quantization f64 for determinism).
pub fn process_trigger(raw: f64, cfg: &TriggerSettings, st: &mut TriggerState, dt: Dt) -> (f64, bool);
```

`StickSettings` / `TriggerSettings` (concrete) — Track 2's structs (`DeadZoneType`, `StickDeadZone`,
`OutputCurve` with C#-matching discriminants `apex=7, axial=8`, `RotationSettings`, `AntiSnapback`,
`SquareStick`, `FlickStick`, `TriggerCurve`) live in `core/src/stick/settings.rs` +
`core/src/trigger.rs`. Every field `#[serde(default)]`; clamp ranges enforced in `clamped()`
(dead_zone 0..=127, anti/max-zone 0..=100, max-zone≥1, roundness≥1, Bezier points ∈[0,1]).

**Per-stick mutable state** (resident, one per LS/RS — **verifier (latency) FIX 6: `Default` MUST be
the clean post-reset state**):
```rust
#[derive(Default)]
pub struct StickState {
    pub rc: crate::rc::RcStickState,    // existing RC state, unchanged
    pub rc_primed: bool,                // moved out of hot.rs `primed[]`
    pub elapsed_us: i64,                // monotonic accumulator (replaces wall clock)
    pub fuzz_last: [f64; 2], pub fuzz_primed: bool,
    pub snap_hist: SnapbackRing,        // fixed-cap (x,y,t) ring (replaces unbounded C# Queue)
    pub flick_in_progress: bool, pub flick_angle_remaining: f64, pub flick_last_angle: f64,
    pub flick_delta: f64,               // OUT: per-report relative turn for the mouse path
}
#[derive(Default)] pub struct TriggerState { pub last_pressed: bool, pub elapsed_us: i64 }
pub const SNAP_CAP: usize = 256; // sized to MAX(report_rate)*max_timeout; drop-oldest at extreme rates
```

**Pinned divergence:** triggers stay `f64` mid-chain where C# truncates to byte; pin both our
rounded output and the C# byte value in tests so the ≤1/255 divergence is explicit, not a regression.

---

## 5. Mapping engine `apply()` contract (`core/src/map/engine.rs`)

```rust
/// Pure, alloc-free, no-I/O. Resolves every Control against the resolved profile (per-control
/// shift, turbo phase, hold/toggle/threshold), composes OutputState, queues KBM edges. `now_us`
/// is the hot loop's monotonic time (REUSED from busy_start/1000 — verifier (latency) FIX 3).
pub fn apply(state: &ControllerState, rp: &ResolvedProfile, ms: &mut MapState, now_us: u64) -> (OutputState, KbmBatch);
```

**Resolution order (semantics ported from `MapCustom` / `ProcessControlSettingAction` /
`ShiftTrigger` / `GetBoolMapping`):**

1. **Digitize once** into a resident `active_raw: [bool; COUNT]` using `ControllerState::pressed`
   with **kind-dependent thresholds** (axis 55/127, trigger 100/255 — verifier FIX 3). Keep the
   continuous `f64` stick/trigger values for `Passthrough`.

2. **Per-control shift selection (faithful model — verifier FIX 4, option b).** DS4Windows shift is
   **per-`DS4ControlSettings`**, not a global table swap: each control carries its own
   `shift_trigger: Option<ShiftTrigger>` + `shift_bind`. For control `c`, if it has a
   `shift_trigger` and that trigger reads pressed in `active_raw` (kind-dependent threshold), use
   `shift_bind`; else use the base `bind`. This supports distinct simultaneous triggers per control
   (real DS4Windows profiles use this) and reads RAW state only (never output → no feedback).

3. **Half-axis identity suppression (verifier FIX 1 — `ResetToDefaultValue`).** First pass: build
   `axis_remapped: [bool; 4]` — for each stick half-axis control, if its effective bind ≠
   `Passthrough`, mark its `stick_axis()`. In the `Passthrough` arm for a half-axis, emit the
   identity stick value **only if `!axis_remapped[axis]`**. This zeroes BOTH halves of an axis when
   either half is remapped, matching `axisdirs[control]=128 && axisdirs[controlRelation]=128`.
   Button identity suppression "falls out for free" (button identity is only written in the
   `Passthrough` arm) — only the axis pairing needed the explicit pass.

4. **Per-control resolve.** For each `Control c` with its effective `slot`:
   ```
   let mut on = active_raw[c];
   if let Some(t) = slot.turbo { on = turbo_gate(&mut ms.turbo[c], on, t, now_us); }   // NET-NEW, no C# golden
   match slot.bind {
     Passthrough        => stick half-axis -> OutputState axis (continuous f64, gated on !axis_remapped);
                           trigger -> lt/rt (continuous); button -> PadButtons bit.
     Unbound            => emit nothing, identity suppressed.
     GamepadButton(b)   => if on { out.buttons.set(bit(b), true); }
     GamepadAxis{a,dir} => if on { write_axis(&mut out, a, dir, full); }
     TouchpadClick      => if on { out.buttons.set(PadButtons::TOUCHPAD, true); }   // verifier FIX 5
     Key{vk,kind}       => edge/toggle latch (ms.toggle[key]/pressed_once[key]) -> push Key{down}.
     KeyUnbound         => suppress identity, emit nothing (DS4KeyType.Unbound) — verifier FIX 5
     Mouse(b)           => edge -> push MouseButton{down}.
     MouseMove(src)     => feed MouseAccumulator (state in MapState) -> push MouseMove{dx,dy}. (verifier FIX 7)
     MouseWheel(dir)    => wheel remainder in MapState -> push Wheel.
     Macro(id)          => on-edge push Macro{start:true}; off-edge push Macro{start:false}.
     Shift(_)           => no direct output (handled in step 2); identity suppressed.
     Special(id)        => push Special{id} edge (drained by control plane).
   }
   ms.prev_active[c] = on;
   ```

5. Return `(out, batch)`.

**Toggle / hold semantics.** Edge-triggered: `pressed_once`/`toggle` latches reproduce DS4Windows
`pressedonce[]` / `kp.current.toggle`. **Verifier FIX 6:** to match DS4Windows bit-for-bit, the
toggle/once latch is keyed on the **OUTPUT key value (vk)**, not the input `Control` (two controls
bound to the same vk share one latch in DS4Windows). `MapState` keeps a small fixed-cap vk→latch
map (alloc-free); if per-control independence is chosen instead, that divergence is documented.

**Turbo (`turbo_gate`).** NET-NEW, no DS4Windows reference (verifier FIX 2). Time-quantized gate
keyed off `now_us`; phase anchored to press time (reset on rising edge so a fresh press starts a
full ON); duty as num/den (no float in the hot path). Pinned by a standalone phase test, no golden.

```rust
#[inline] fn turbo_gate(ts: &mut TurboState, src_on: bool, t: TurboCfg, now_us: u64) -> bool {
    if !src_on { ts.was_active = false; return false; }
    if !ts.was_active { ts.cycle_start_us = now_us; ts.was_active = true; }
    let phase = now_us.wrapping_sub(ts.cycle_start_us) % (t.period_us as u64);
    phase * (t.duty_den as u64) < (t.period_us as u64) * (t.duty_num as u64)
}
```

**Mouse-from-stick/gyro determinism (verifier FIX 7).** The wheel-remainder and mouse-accel/carry
state live IN `MapState` (Copy `MouseAccumulator` fields) and the integer `(dx,dy)` is computed
**inside `apply()`** via `MouseAccumulator::stick_step`/`gyro_step`, matching DS4Windows
`calculateFinalMouseMovement`/`stickWheelRemainder`. The KBM sink only injects the already-computed
delta — mouse feel stays deterministic and (modulo the documented f64 precision) DS4Windows-faithful.

**`MapState`** (all Copy, fixed arrays — no heap on the hot path):
```rust
#[derive(Clone, Copy, Default)]
pub struct MapState {
    pub turbo: [TurboState; Control::COUNT],
    pub prev_active: [bool; Control::COUNT],
    pub stick_mouse: MouseAccumulator, pub gyro_mouse: MouseAccumulator, pub wheel_remainder: f64,
    pub toggle: VkLatch, pub pressed_once: VkLatch,   // fixed-cap vk->bool maps (verifier FIX 6)
    // (per-control shift is resolved from ResolvedProfile each report; no layer latch needed)
}
```

---

## 6. Output backends

### 6.1 KBM (SendInput) — `crates/kbm-output` (cfg(windows))

`OutputState`/`KbmBatch` are pure-core; the sink is the Windows shell.

```rust
pub trait KbmSink { type Error;
    fn flush(&mut self, batch: &KbmBatch) -> Result<(), Self::Error>;  // ONE batched SendInput/report
    fn release_all(&mut self) -> Result<(), Self::Error>;              // on disable/profile-switch/shutdown
}
pub struct SendInputKbm {
    scratch: Vec<INPUT>,        // capacity == KBM_BATCH_CAP, cleared+reused, NEVER reallocated
    held_keys: [bool; 256],     // edge-dedupe: only newly-pressed/released VKs become INPUT records
    held_mouse: u8, fake_key_repeat: bool,
}
```
- VK→scancode + extended-key handling ported from `SendInputHandler.scancodeFromVK`. `KEYEVENTF_SCANCODE` by default for game compatibility; exposed as a policy flag (HW-verify vs target anti-cheat).
- `fake_key_repeat` defaults **false** (edge-only → zero syscalls while a key is held); enable per-key only when OS key-repeat is needed (chat). Holding a key emits nothing after the first report.
- One bounded `SendInput` call per report, only when ≥1 edge exists; otherwise zero syscalls.

### 6.2 Mouse-from-stick / gyro accumulator — `core/src/mouse_accum.rs` (pure, Linux-tested)

Ports DS4Windows `MouseCursor` remainder-carry **exactly**: per-axis `f64` remainder added back
when sign matches, truncate-to-2-decimals via `remainder_cutoff(x*100,1)/100`, `min_threshold`
distance-sq gate, atan2 direction split, anti-deadzone/deadzone, jitter compensation (`pow 1.408`).

```rust
#[inline] pub fn remainder_cutoff(dividend: f64, divisor: f64) -> f64 { dividend - divisor * (dividend / divisor).trunc() }
#[derive(Clone, Copy, Debug, Default)] pub struct MouseAccumulator { h_remainder: f64, v_remainder: f64 }
impl MouseAccumulator {
    pub fn stick_step(&mut self, nx: f64, ny: f64, elapsed_s: f64, cfg: &MouseAccumCfg) -> (i32, i32);
    pub fn gyro_step(&mut self, raw_dx: f64, raw_dy: f64, elapsed_s: f64, cfg: &MouseAccumCfg) -> (i32, i32);
    pub fn reset(&mut self) { self.h_remainder = 0.0; self.v_remainder = 0.0; }
}
```
The accumulator is **called from `apply()`** (verifier FIX 7), state lives in `MapState`; the engine
pushes `(dx,dy)` into the `KbmBatch` as `MouseMove`. The `min_threshold==1.0` special-case and the
carry/sign-flip-reset are the load-bearing precision behavior — pinned in unit tests.

### 6.3 Virtual gamepad (X360 OR DS4) — `crates/vgamepad-output` (cfg(windows))

`VirtualPad::update` changes from `&OutputFrame` to `&OutputState` (one-line trait change). Keep
`Vigem360Pad`/`XusbReport`/`from_frame` and **all existing tests** intact — the X360 report builder
consumes `OutputState` via `OutputState::to_output_frame()` (preserving the single round through
`to_xinput_thumb/_trigger`). Add `VigemDs4Pad` (u8 sticks 128-center via `to_ds4_axis`, dpad nibble
via `dpad_8way`, special byte for PS/Touchpad, `TriggerLeft/Right` flags from `L2_CLICK/R2_CLICK`).

```rust
pub enum DynPad { X360(Vigem360Pad), Ds4(VigemDs4Pad) }   // static dispatch, no vtable in hot path
impl DynPad { #[inline] pub fn update(&mut self, s: &OutputState) -> Result<(), OutErr> { match self { Self::X360(p)=>p.update(s), Self::Ds4(p)=>p.update(s) } } /* + plugin/wait_ready/unplug dispatch */ }
```
Target is chosen at (re)plug time from the active profile's `OutputKind`; switching is a full
unplug/replug via the existing `HotCommand::ReplugTarget` (ViGEm cannot morph target type — games
see a disconnect/reconnect), never per report. **HW-verify:** DS4 Y-axis wire polarity (`flip_y`).

---

## 7. Profiles + auto-switch (off hot path) + lock-free preservation

### 7.1 How it supersedes the current config while keeping the topology

The existing topology is **kept verbatim**: `ConfigHandle = Arc<ArcSwap<EngineConfig>>` +
`generation: AtomicU64` + `ConfigStore` single writer fed by `ControlMsg`. `EngineConfig` is
**extended in place** (NOT replaced) so `config_store.rs`/`handoff.rs`/`runtime.rs`/`supervisor.rs`
need no structural change.

```rust
#[derive(Clone, Debug, Default, Serialize, Deserialize)] #[serde(default)]
pub struct EngineConfig {
    pub active_device: String,                          // UNCHANGED
    pub thread: ThreadConfig, pub hidhide: HidHideConfig, // UNCHANGED
    pub devices: BTreeMap<String, DeviceConfig>,        // hardware identity only now (loses ls/rs)
    pub profiles: Arc<BTreeMap<String, Profile>>,       // NEW. Arc so the per-generation EngineConfig
                                                        //   clone bumps a refcount, not a deep copy
                                                        //   (verifier (latency) FIX 7).
    pub assignments: BTreeMap<String, String>,          // NEW device -> active profile id
    pub auto_switch: AutoSwitchConfig,                  // NEW
    #[serde(skip)] pub resolved: BTreeMap<String, Arc<ResolvedProfile>>, // NEW hot-facing flat cache
}
```

- **`DeviceConfig` loses `ls`/`rs`** (they move into `Profile`); keeps vid/pid/report_rate_hz/stick_bits.
- **`Profile`** (editable serde, string-keyed maps/Vecs) = `{ name, output_kind: PadTarget,
  bindings: BTreeMap<Control, BindingSlot> (base), per-control shift bind+trigger, ls/rs:
  StickSettings, l2/r2: TriggerSettings, mouse: MouseSettings, gyro: GyroSettings, macros:
  Vec<MacroDef>, specials: Vec<SpecialAction> }`.
- **`ResolvedProfile`** (hot-facing, `#[serde(skip)]`, rebuilt by `clamped()`) = flat arrays:
  `base: [BindingSlot; Control::COUNT]`, `shift: [(Option<ShiftTrigger>, BindingSlot); Control::COUNT]`,
  resolved `StickSettings`/`TriggerSettings`, `mouse`/`gyro`, `macros: Arc<[MacroDef]>`,
  `specials: Arc<[SpecialAction]>`. **No `String`/`BTreeMap`/hash on the hot path** — index by
  `control as usize`.

`EngineConfig::clamped()` (the existing validate funnel) additionally rebuilds `resolved` for
**assigned devices only**:
```rust
out.resolved = out.assignments.iter()
    .filter_map(|(dev, pid)| out.profiles.get(pid).map(|p| (dev.clone(), Arc::new(p.resolve()))))
    .collect();
```
`resolved` is `#[serde(skip)]`, so `to_toml(next)==to_toml(current)` no-op detection in
`ConfigStore::apply` still works and the on-disk form stays clean.

### 7.2 Hot-loop integration (latency-verified)

**The hot loop touches ZERO maps per report (verifier (latency) FIX 1/2).** On the existing
generation gate (`if cur_gen != applied_gen`), after `cfg = (*load_full()).clone()`, resolve ONCE
into the resident set:
- the active `Arc<ResolvedProfile>` (`cfg.resolved.get(&cfg.active_device)` — one Arc-ref get),
- the per-stick `StickSettings`/`TriggerSettings` **copied** into resident `[StickSettings;2]` +
  `[TriggerSettings;2]` (they are small `Copy` structs).

Per report (after RC/stick/trigger processing, before the pad submit):
```rust
let now_us = busy_start / 1000;   // REUSE the existing clock read (hot.rs:180) — no 2nd now_qpc_ns
let state: ControllerState = /* decoded by backend; or projected for passthrough */;
let (out_state, kbm) = hyperion_core::map::apply(&state, &resolved_profile, &mut map_state, now_us);
if self.target.update(&out_state).is_err() { return StopReason::DeviceLost; }
if kbm.len() > 0 { let _ = self.kbm_tx.push(kbm); }   // SPSC, drop-on-full, never blocks
```
With an all-`Passthrough` profile, `out_state` is byte-identical to today's frame (sticks filtered,
triggers/buttons copied) — provably non-regressing. **No new locks, no new atomics** (one existing
atomic load per report); `apply()`/`process_stick`/`process_trigger` run inline exactly where the
single `RcFilter` step ran. The freed old `Arc<EngineConfig>` Drop on the hot thread stays cheap
because `profiles` is `Arc`-shared (refcount decrement, not tree free).

### 7.3 KBM egress thread

`handoff.rs` gains a **third SPSC `rtrb` ring** `KbmTx`/`KbmRx` (`rtrb::RingBuffer<KbmBatch>`,
`KbmBatch` is Copy, bounded element size). The hot loop `push`es non-blocking (drop-on-full,
surfaced as a telemetry counter like dropped/duplicates). A new normal-priority **`KbmInjector`**
thread (in `kbm-output`) drains the ring, calls `SendInputKbm::flush`, and owns **macro playback
timing** (the unbounded part stays entirely off the hot thread). The hot thread NEVER calls
`SendInput`.

### 7.4 Auto-profile-switch (control plane only)

A `cfg(windows)` `ForegroundWatcher` on the supervisor thread polls `GetForegroundWindow` +
exe-path/title at `auto_switch.poll_hz` (~4 Hz, default), matches against `AutoSwitchRule`s via the
**pure** `core::autoswitch::match_rules(&[AutoSwitchRule], &ForegroundInfo, device) -> Option<&rule>`
(Linux-tested), and on a change sends `ControlMsg::SetActiveProfile{device, name}` (NEW variant) to
the single writer — exactly like a GUI edit. `read_foreground` returns `Option` (a failed read on
an elevated/protected process yields no match, keeps the current profile, never crashes). The hot
loop picks the new profile up through the existing generation gate — **100% off the hot thread**.

---

## 8. GUI screen breakdown (`crates/app/src/gui/`)

Top-level tab bar (`gui/mod.rs`): **Mapping | Sticks | Triggers | Mouse/Gyro | Macros | Profiles |
Engine** + the existing scope side panel + tray. `DeviceMirror` → `ProfileMirror` (editable clone of
the active `Profile`, seeded from `config_snapshot()`). Every screen only **sends `ControlMsg`** —
never touches the `ArcSwap`; `ControlMsg`/`config_store` single-writer path is unchanged.

- **mapping.rs** (NEW): controller diagram / control list → click a `Control` → binding picker popup
  (Pad / Key / Mouse / Macro / Shift / Special tabs) with an egui key-capture widget (`InputState`
  raw key/mouse → `vk` + scancode). Per-control shift-trigger + shift-bind editor. Emits
  `SetBinding`/`ClearBinding`/`SetShiftTrigger`.
- **sticks.rs** (NEW): per-stick deadzone (radial/axial)/anti/max-zone/max-output/sensitivity/
  rotation/fuzz/square + curve combo (incl. Apex, Bezier editor) + **the existing RC panel
  (`panels.rs::stick_panel`) reused verbatim as the `rc` sub-section** + live scope. Emits
  `SetStickSettings`.
- **triggers.rs** (NEW): deadzone/anti/max-zone/curve/sensitivity + two-stage mode + trigger→button.
  Emits `SetTriggerSettings`.
- **mouse_gyro.rs** (NEW): mouse-from-stick + gyro→mouse/stick sliders. Emits `SetMouseSettings`/`SetGyroSettings`.
- **macros.rs** (NEW): macro list + reorderable step editor (KeyDown/KeyUp/MouseDown/MouseUp/Wait).
  Emits `UpsertMacro`/`DeleteMacro`.
- **profiles.rs** (NEW): profile manager (create/rename/duplicate/delete, device→profile assignment,
  output-kind toggle) + auto-switch rule table (exe substr, title substr, device, profile). Emits
  the profile-lifecycle + auto-switch messages.
- **panels.rs** (KEEP): RC/thread/hidhide panels; RC reused by sticks.rs, thread/hidhide move under
  the Engine tab.

---

## 9. How current code/tests change (migration)

The breaking core change is `DeviceConfig{vid,pid,report_rate_hz,stick_bits,ls,rs}` →
sticks/triggers move into `Profile`, and `StickConfig{mode,rc}` → full `StickSettings` (RC becomes
one field). This is a **wide but mechanical** edit; every named call site must migrate in the same
change or the workspace won't compile.

**`core/src/config.rs`** (REWRITE):
- Delete `StickConfig`; `StickMode` is **kept only as a back-compat serde shim** that deserializes a
  legacy `[devices.x.ls] mode="Rc"` into `StickSettings.rc_mode_on=true` (anything else → false).
- `DeviceConfig` loses `ls`/`rs`. Add `profiles`/`assignments`/`auto_switch`/`resolved` to
  `EngineConfig`. `clamped()` rebuilds `resolved` and clamps the new ranges.
- **Legacy migration shim in `load_toml`:** an OLD-shape TOML with `[devices.x.ls]` synthesizes a
  `"default"` profile carrying that stick's `rc` and an assignment `x -> "default"`, so existing
  on-disk configs (and the embedded-default test) still load. The two existing config.rs tests
  (`embedded_default_toml_loads_to_expected_shape` asserts `dev.ls.mode`; `unknown_enum_strings…`
  asserts `dev.ls.mode`) are **updated** to assert `profile.ls.rc_mode_on` / the synthesized profile.

**`engine/src/hot.rs`** (EXTEND): resident set `[RcStickState;2] + primed[2]` → `[StickState;2] +
[TriggerState;2] + MapState + ResolvedProfile-ref + [StickSettings;2] + [TriggerSettings;2]`. Imports
swap to `stick::{settings,pipeline}`, `trigger`, `map`. `step_stick` → `process_stick`; add
`process_trigger`. `resolve_rc` → resolve on the generation gate (§7.2). `HotInput` gains
`raw_buttons: u32` (set from `s.buttons.0` in `win_io::hot_input_from_sample`; zero new decode — the
existing `buttons:u16` still feeds passthrough). The `HotCommand::ResetFilter` arm becomes
`stick_state = Default::default(); trig_state = Default::default(); map_state = MapState::default();`
(verifier FIX 6 — `Default` is the clean post-reset state; pinned by a unit test). Keep a
`prime_reset(&mut st)` helper for the per-report `input.is_prime` path, distinct from the
command-driven full reset.

**`engine/src/control.rs`** (EXTEND `ControlMsg`): keep existing variants; `SetStickMode`/`SetRc`
now target `profiles[active].{ls,rs}.rc`. Add `SetActiveProfile{device,name}` (the auto-switch +
manual-switch path; routed through `ConfigStore::apply` exactly like `SetActiveDevice`),
`CreateProfile`/`DuplicateProfile`/`RenameProfile`/`DeleteProfile`, `SetAssignment`,
`SetOutputKind`, `SetBinding`/`ClearBinding`/`SetShiftTrigger`, `SetStickSettings`/
`SetTriggerSettings`/`SetMouseSettings`/`SetGyroSettings`, `UpsertMacro`/`DeleteMacro`,
`UpsertSpecialAction`/`DeleteSpecialAction`, `SetAutoSwitchEnabled`/`UpsertAutoSwitchRule`/
`DeleteAutoSwitchRule`.

**`engine/src/config_store.rs`** (EXTEND): `edit_in_place` gains an arm per new `ControlMsg`
(mutating `cfg.profiles`/`cfg.assignments`/`cfg.auto_switch`; unknown profile/device id stays a
silent no-op → `false`, identical to today). `stick_mut` returns `&mut RcConfig`-bearing field
inside the active profile. `validate()` still equals `clamped()` (now also rebuilds `resolved`).
`apply`/`publish`/`update`/`generation`/`SaveToDisk`/`ReloadFromDisk` mechanism: **zero change**.
(`cfg.profiles` is `Arc<BTreeMap>` — mutating an arm uses `Arc::make_mut`.)

**`engine/src/handoff.rs`** (EXTEND): add `KbmTx(rtrb::Producer<KbmBatch>)` /
`KbmRx(rtrb::Consumer<KbmBatch>)` to `build_links`. `HotCommand` is unchanged (KBM flush is a
ring push, not a command). The existing two-SPSC-command-queue topology is untouched.

**`engine/src/win_io.rs`** (EXTEND): `hot_input_from_sample` sets `raw_buttons = s.buttons.0`.
`VirtualPad::update` adapter takes `&OutputState`; `Vigem360Target` → `DynPad` chosen from the
active profile's `OutputKind` at plug time, replug on `ReplugTarget`. Wire the `KbmInjector` drain.
Promote `ds_buttons_to_xinput`'s bit layout into `core::output::pack_xinput` + the
`decode_controller_state` button map (single source of truth for the layout).

**`engine/src/{runtime,supervisor}.rs`** (EXTEND): spawn `ForegroundWatcher` on a named thread
(Windows only) with its own stop channel, joined in `shutdown()` before the writer; clone
`control_tx` into it. `CONTROL_QUEUE_CAP` (256) absorbs ~4 Hz auto-switch + UI edits. Single-writer
guarantee intact (the watcher is just another `ControlMsg` sender). `OutputKind` is read at plug
time; runtime switch → `ReplugTarget`.

**`crates/app/src/gui/mod.rs`** (EXTEND): `DeviceMirror` → `ProfileMirror`; `set_stick_mode`/
`push_rc` carry a `profile` id; add the new tabs (§8).

**Vigem-output + new kbm-output**: per §6. `to_ds4_axis`/`dpad_8way`/`pack_xinput` live in core
(Linux-tested), the Windows backends call them — exactly as `to_xinput_thumb` is shared today.

---

## 10. Test plan (all pure-core tests on Linux CI; Windows shells type-checked on windows-latest)

**State/parse (`core`):** synthesize 64-byte reports toggling each btn0/btn1/btn2 bit → assert the
matching `ControllerState` flips; all 9 dpad-hat nibble values; trigger digital views
(`l2_raw==255`→`L2FullPull`, `analog==raw/255` within 1e-12); AxisDir resolver at the 55/127
boundary; `Control::COUNT==GyroZNeg+1`, dense `as_index` round-trip, serde PascalCase round-trip;
`decode_controller_state` sticks **equal** `ds_report_to_sticks` (no duplicate offsets); regression
guard that `to_input_sample` reproduces the old path's sticks/triggers/seq/dt byte-identically.

**Stick/trigger pipeline (`core`):** golden curve sweeps vs the closed-form C# formulas (Enhanced
0.4→0.32, 0.75→0.67; Quadratic 0.5→0.25; Cubic 0.5→0.125; EaseoutQuad 0.5→0.75; ApexAxis 0.25→0.5);
radial/axial deadzone numeric examples in the `[0,255]` domain (incl. neutral→128, endpoints→±full);
sensitivity (radial-only) `sens=2.0 @160→192`; rotation algebra; trigger chain vs C# L2 expression
**plus** the pinned rounded-vs-truncated divergence; **RC bit-exact regression** (all existing
`rc/` goldens UNCHANGED) + a `process_stick` wrapper test (only RC on, every downstream stage at
default → byte-identical to `RcFilter` directly); purity/determinism property test; a counting-
allocator test asserting **zero heap allocations** across 10k `process_stick` calls; a
`StickState::default()`/`TriggerState::default()`/`MapState::default()` == clean-state test.

**Mapping engine (`core/src/map`):** button→button (B set, A NOT set — half-axis/identity
suppression); button→key edge (one KeyDown on press, none on hold, KeyUp on release);
toggle (latched, keyed on vk); **per-control shift** (control X shifted by T1 while control Y is
shifted by T2 simultaneously); analog shift trigger at 100/255; turbo phase pattern ON,ON,OFF,OFF…
with press-reset; analog→digital threshold (axis 55/127, trigger 100/255); **half-axis identity leak
guard** (bind only `LxPos`→key, leave `LxNeg` Passthrough → BOTH halves of Lx go neutral, the
negative half does NOT leak — verifier FIX 1); passthrough precision (`LX=0.123456789` exact);
stick→stick swap; `KbmBatch` saturation (no panic/alloc, len==CAP); mouse+macro edges.

**Mouse accumulator (`core`):** `remainder_cutoff` exact values incl. negative sign; carry
accumulation (0.4×3 → 0,0,1, remainder 0.2); `min_threshold` gate defers then emits; sign-flip
reset; deadzone; invert flags; `reset()`.

**Output (`core`):** `to_ds4_axis(0.0)==128`, `(1.0)==255`, `(-1.0)==0`, `flip_y` inverts;
`dpad_8way` all 8 + neutral + diagonals + opposing-press resolution; `PadButtons`→X360 and →DS4
bit/special/dpad table tests (GUIDE→Guide on X360 but special on DS4; TOUCHPAD DS4-only;
L2_CLICK/R2_CLICK DS4 trigger flags); `OutputState::passthrough(&state).to_output_frame()`
reproduces today's button packing; default → `OutputFrame::default()`; sticks/triggers pass through
bit-identically.

**Auto-switch (`core`):** exe-substr first-match-wins; title-substr; case-insensitive; no-match →
None; per-device scoping.

**Config (`core`):** full profile-tree round-trip (`to_toml`→`load_toml`→re-serialize equal);
each `BindTarget`/`CurveKind`/`GyroMode` variant survives + unknown → `#[serde(other)]` fallback +
missing section → `#[serde(default)]`; **legacy `[devices.x.ls] mode="Rc"` migrates** to a
synthesized "default" profile + assignment; `clamped()` rebuilds `resolved` for assigned devices
only; `resolved` absent from TOML.

**Engine integration (Linux-buildable via the existing trait test doubles):** hot loop with default
(passthrough) profile produces the SAME `OutputFrame`/state as the pre-mapper baseline for a
scripted sequence (golden non-regression); generation bump that swaps the active profile takes
effect only after the gen changes (rebuild off the fast path); `ResetFilter` clears `MapState`
(turbo mid-cycle resets to a fresh ON); `KbmTx` ring pushes exactly one batch per KBM-producing
report and drops on full without blocking; extend `config_store`/`runtime` tests for the new
messages (`apply(SetBinding)` mutates + bumps gen; `apply(SetActiveProfile)` to current = no-op;
`start_then_edit` observes the swapped profile's Arc in the snapshot). proptest: random
profiles + input sequences never panic, never NaN, alloc-free.

---

## 11. CI

Unchanged gate shape (`DESIGN.md` §11). The new core modules (`control`, `state`, `stick::*`,
`trigger`, `map::*`, `mouse_accum`, `autoswitch`, the rewritten `config`) are pure → covered by the
ubuntu `core` job (`cargo test -p hyperion_core` + clippy `-D warnings`). The new `kbm-output` crate
+ `platform-win::foreground` + `vgamepad-output` DS4 path + `win_io`/`supervisor` changes are
`cfg(windows)` → type-checked + headless-smoke on windows-latest. Each milestone (§12) is gated on
the ubuntu `core`+`engine` job staying green.

---

## 12. PHASED ROADMAP (M3..M6) — each milestone independently shippable + CI-green

Continues the `DESIGN.md` M1/M2 numbering (M1 vertical slice, M2 tuning surface both land first).
Every milestone leaves the workspace compiling, all CI green, and a usable product.

### M3 — Structured state + full stick/trigger settings + basic button/key remap + SendInput KBM
**Goal:** the remapper's spine, with no behavior regression for stick-only users.
- `core`: `Control`/`ControllerState`/`Thresholds`, `OutputState`/`PadButtons`/`pack_xinput`,
  `KbmBatch`; `decode_controller_state`/`parse_controller_state` (with the btn bit map promoted from
  `win_io`); full `StickSettings`/`process_stick` (all stages **except** flick — flick state stashed
  but its consumer lands M5) + `TriggerSettings`/`process_trigger`; `map::{Binding, Profile,
  ResolvedProfile, MapState, apply}` with **button→button + button→key + Passthrough only**
  (no shift/turbo/macro/mouse yet — those bindings exist in the enum but the M3 `apply` resolves
  them as `Unbound`/no-op); `config.rs` rewrite + legacy migration shim.
- `engine`: `hot.rs` resident set + generation-gate resolve + `apply()` seam + `raw_buttons`;
  `KbmTx`/`KbmRx` ring; `control.rs`/`config_store.rs` binding/profile/assignment messages.
- `kbm-output` crate (`SendInputKbm`, edge-dedupe) + `KbmInjector` thread.
- **Exit:** Cross→Xbox-B and Square→keyboard-A work end-to-end on hardware; an all-Passthrough
  profile is byte-identical to M2; all stick/trigger goldens + RC regression green; zero-alloc test
  green. **Independently shippable:** a working stick-filter + basic-remap product.

### M4 — Shift / turbo / macros / mouse-from-stick / special actions
- `core`: per-control shift resolution in `apply()`; `turbo_gate`; toggle/once vk-keyed latches;
  macro start/stop edges; `mouse_accum` (stick→mouse) wired into `apply()` (verifier FIX 7);
  mouse-button + wheel bindings; `Special` edges to a control-plane side channel.
- `kbm-output`: macro playback timing on the injector thread; mouse-move/wheel/button injection.
- `engine`/`app`: macro + mapping GUI editors; `MouseSettings`.
- **Exit:** a shift layer, a turbo button, a 3-step macro, and right-stick-as-mouse all work; shift
  precedence + turbo phase + half-axis suppression pinned. **Independently shippable.**

### M5 — Profiles + auto-switch + DS4 output + gyro
- `core`: full `Profile` tree finalized; `autoswitch::match_rules`; `gyro_step` mouse accumulator +
  `GyroSettings`; flick-stick consumer (terminal stage → mouse path); `to_ds4_axis`/`dpad_8way`.
- `vgamepad-output`: `VigemDs4Pad` + `DynPad`; `OutputKind` per profile; `ReplugTarget` switch.
- `platform-win`: `foreground.rs`; `engine`: `ForegroundWatcher` + `SetActiveProfile`.
- `app`: profiles screen + auto-switch rule table + mouse/gyro screen.
- **Exit:** per-game auto-switch flips profiles; a DS4 virtual pad enumerates and is driven; gyro→
  mouse aims. **Independently shippable.**

### M6 — Touchpad + polish
- `core`: touchpad contact decode (active/id/12-bit X/Y); touch-region controls
  (`TouchLeft/Right/Upper/Multi`) + touch-to-mouse; two-stage/hip-fire trigger modes; Edge
  Fn/paddle/Mute/Capture decode behind the verified capability flag.
- `app`: touchpad config; profile import/export; final UX polish.
- **Exit:** touchpad-as-buttons + two-stage triggers + Edge paddles work; full DS4Windows-class
  parity (minus the explicitly out-of-scope lightbar/rumble feedback). **Independently shippable.**

---

## 13. Resolved Conflicts (winner + one-line justification)

1. **State model placement — widen `InputSample` (none) vs new `ControllerState` projecting down
   (Track 1).** Winner: **separate `ControllerState`**; `InputSample`/`HotInput` stay the stick-only
   fast projection so the proven lock-free path and every existing test are byte-identical.
2. **`Control` surface — superset (Track 1) vs 29-variant subset (Track 3) vs `DsControl` (Track 5).**
   Winner: **Track 1's append-only superset**; serde-key/table-index stability matters and M5/M6
   controls must not renumber persisted profiles. Undecoded variants are capability-gated and inert.
3. **StickSettings domain — `[0,255]` ordered chain (Track 2) vs normalized `[0,1]` + `stage_order`
   Vec (Track 5).** Winner: **Track 2's `[0,255]` fixed-order pipeline** (1:1 C# goldens, no
   off-by-asymmetry), placed inside `Profile` per Track 5. No per-report branch on absent stages.
4. **Shift model — global per-layer table swap (Track 3) vs per-control shift trigger (verifier FIX
   4b / Track 5).** Winner: **per-control `shift_trigger` + `shift_bind`** — the faithful DS4Windows
   model; supports distinct simultaneous triggers per control, which real profiles use.
5. **Analog→digital threshold — single `0.43` (Track 3) vs kind-split (verifier FIX 3).** Winner:
   **kind-split** — axes 55/127 (≈0.433), triggers 100/255 (≈0.392); the two C# constants differ.
6. **Half-axis identity — "falls out for free" (Track 3) vs explicit pair suppression (verifier FIX
   1).** Winner: **explicit `axis_remapped[4]` pass** — `ResetToDefaultValue` zeroes BOTH halves;
   without it the unbound half leaks on any partial-axis remap.
7. **`OutputState` shape — three tracks defined three.** Winner: **Track 4's `OutputState` +
   `PadButtons`** (the dual-target egress); Track 1's and Track 3's both lower into it.
   `OutputFrame` is kept unchanged as the X360 projection (single-round invariant intact). Added
   `TouchpadClick`/`KeyUnbound` per verifier FIX 5.
8. **Mouse-accel/remainder state — shell-owned (Track 3) vs in `MapState`, computed in `apply()`
   (verifier FIX 7).** Winner: **in core/`apply()`** so relative-mouse stays deterministic and
   DS4Windows-faithful; the sink only injects the computed delta.
9. **Toggle latch key — input `Control` (Track 3) vs output vk (verifier FIX 6).** Winner: **output
   vk** for DS4Windows parity (two controls on one key share a latch); kept alloc-free via a
   fixed-cap vk map.
10. **Hot-path config access — per-report `BTreeMap` lookup (tracks, inherited from `resolve_rc`)
    vs resolve-once-on-generation-gate (verifier latency FIX 1/2).** Winner: **resolve once into the
    resident set** — ZERO maps/strings/hashes per report; small `Copy` settings copied, profile held
    as an `Arc` ref.
11. **`now_us` source — new `now_qpc_ns()` call (tracks) vs reuse `busy_start/1000` (verifier latency
    FIX 3).** Winner: **reuse** — zero extra clock reads per report.
12. **`KbmBatch` cap — unstated (tracks) vs pinned `CAP=24` (verifier latency FIX 4).** Winner:
    **pinned fixed array** so the `rtrb<KbmBatch>` element and the hot-thread push memcpy are bounded;
    macro expansion (unbounded) lives on the injector thread, not the hot thread.
13. **`EngineConfig` clone cost — deep `profiles` tree (tracks) vs `Arc<BTreeMap>` (verifier latency
    FIX 7).** Winner: **`Arc`-share `profiles`/macros/specials** so the per-generation
    `EngineConfig::clone()`+Drop on the hot thread is a refcount bump, not a tree copy.
14. **Auto-switch transport — new mechanism (none) vs existing `ControlMsg`/single-writer (Track 5 +
    verifier).** Winner: **existing path** — `ForegroundWatcher` sends `SetActiveProfile`; the hot
    loop sees only a generation bump; 100% off the hot thread.

## 14. Remaining HW-gated / unresolved (flagged, not M3 blockers)
- **btn2 Mute/Capture + Edge Fn/paddle bit positions** (Edge superset, extended report) — capability-gated, HW-verify before enabling those Control variants.
- **Touchpad contact byte offsets/packing** and **gyro/accel raw-i16 → rad/s,g full-scale constants** — port from `DS4Device.cs`, pin with fixtures, HW-verify; raw retained for post-hoc correction.
- **DS4 Y-axis wire polarity** (`to_ds4_axis(flip_y)`) — confirm up/down direction on a real DS4 target.
- **SendInput scancode-vs-VK flag** vs target anti-cheat/games — exposed as a sink policy flag, HW-verify.
- **Turbo duty/period feel** — net-new, no golden; tune on hardware.
- **Flick-stick delta units** vs the OneEuro mouse contract — reserved interface (`real_world_calibration`/`min_cutoff`/`beta`), validated when the mouse path lands (M5).
- **Out of scope v1 (flagged):** lightbar / rumble / device-LED output feedback.
