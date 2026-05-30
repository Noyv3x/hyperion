# Hyperion Rust Controller Engine — DESIGN.md / AGENTS.md

> Implementation-ready blueprint. Merges 4 design tracks + 2 adversarial verifications (algo-math, engine-threading), reconciled against the ground-truth C# at `Hyperion/DS4Control/RcFilter.cs`, `Hyperion/DS4Library/DS4Device.cs:1301-1333`, `HyperionTests/RcFilterTests.cs`, and `doc/concepts/rc-timebase-dt.md`. Where tracks conflicted, the verifier-corrected or ground-truth-confirmed option wins (see **Resolved Conflicts**). This drives the scaffolding next; M1 is a working vertical slice.

---

## 1. Vision + Locked Tech Stack

**Vision.** Hyperion is an extreme-FPS-esports controller engine: it intercepts a physical pad (DualSense / DualSense Edge over USB first), runs the validated RC stick filter (FireBird-integer bit-exact, Ultimate f64, and a new **report-rate-invariant dt-compensated Ultimate**) in a pure, OS-free numeric core, and re-emits a virtual Xbox 360 pad via ViGEm — with the lowest controllable input latency, no unnecessary requantization (full-precision f64 from de-quantization to a single final i16 write), and the lowest CPU load achievable. The whole numeric core is `no_std`-friendly and 100% unit-testable on Linux CI; everything Windows-specific is `cfg(windows)` and exercised on `windows-latest`.

**Locked tech stack (do not re-litigate):**
- **Language/build:** Rust, edition 2021, cargo **workspace, `resolver = "2"`**, `workspace.lints` with `clippy::all = "warn"`, CI `-D warnings`.
- **Hot-path concurrency:** `arc-swap` (config snapshot, MPMC-safe, wait-free load), `triple-buffer` (telemetry hot→GUI), `rtrb` (SPSC command/scope queues), `crossbeam-channel` (non-hot control plane only). **No Mutex on the hot loop.**
- **Windows I/O:** `windows` crate `0.62.x` (HID, overlapped ReadFile, MMCSS/AvrtApi, timer res, HidHide IOCTL), `vigem-rust` wrapper for ViGEmBus Xbox360 target, `core_affinity` for pinning.
- **GUI:** `eframe`/`egui 0.33` + `tray-icon 0.19` sharing **one** winit event loop.
- **Config:** `serde` + `toml 0.8`; `notify 6` file-watch.
- **Test:** `approx`, `proptest` (core dev-deps).
- **Driver deps (external, EOL-noted):** ViGEmBus (LizardByte mirror / Nefarius VirtualPad successor as fallback), HidHide. Both isolated behind one crate each so they are swappable.

---

## 2. Crate Graph (one-line responsibility each)

```
hyperion-core        (pkg hyperion_core)  ZERO OS deps. All numerics + types: axis units, precision
                                          conversion, StickAlgorithm trait, 3 RC modes, dt contract,
                                          input parse/normalize, config serde, output mapping math.
                                          100% Linux-CI-testable. THE source of truth.
hid-input    [win]   #![cfg(windows)]     I/O-only HID shell: enumerate VID/PID, CreateFileW overlapped
                                          ReadFile, hand &[u8] + QPC timestamp to core parsers. DeviceSource
                                          trait + DualSenseUsb / XInput / RawHidGeneric backends.
vgamepad-output [win] #![cfg(windows)]    VirtualPad trait + Vigem360Pad over vigem-rust. Maps core f64
                                          OutputFrame -> XUSB_REPORT i16 (single final round). One IOCTL.
platform-win   [win] #![cfg(windows)]     HidHide (IOCTL + CLI fallback), timer resolution RAII, MMCSS,
                                          affinity helpers, process priority class.
engine               core (always) +      Thread architecture + lock-free handoff. Owns hot loop, dt
                     [win] the 3 above     measurement, telemetry, config store, supervisor lifecycle.
                                           Pure parts (clock, seq, handoff, config_store) Linux-testable.
app          [win]   bin                   Wires supervisor, spawns hot thread, runs egui+tray event loop.
```

Dependency direction: `core` depends on nothing OS. `engine` depends on `core` unconditionally and on `{hid-input, vgamepad-output, platform-win}` only under `[target.'cfg(windows)'.dependencies]`. `app` depends on `engine` + GUI crates. This keeps `cargo test -p hyperion_core` (and `engine`'s pure modules) green on `ubuntu-latest` with no Windows toolchain.

---

## 3. Directory / File Tree to Scaffold (exact paths)

Rust workspace lives under `rust/` to coexist with the C# tree (do not clobber `Hyperion/`).

```
rust/
  Cargo.toml                         # [workspace] resolver=2, members, workspace.deps, workspace.lints
  rustfmt.toml
  rust-toolchain.toml                # pin stable channel
  DESIGN.md                          # this file
  .github/workflows/rust.yml         # (lives at repo .github/workflows/; see CI section)

  crates/core/
    Cargo.toml                       # [lib]; deps serde,toml; dev-deps approx,proptest; no OS deps
    src/lib.rs                       # pub mod axis, convert, dt, stick, rc, input, config, output; re-exports
    src/axis.rs                      # Axis=f64 [-1,1], RawStickFormat, clamp
    src/convert.rs                   # precision-preserving conv; axis<->ds4 domain; final i16 round
    src/dt.rs                        # Dt newtype, guard window [100us,20000us], prime semantics
    src/stick.rs                     # StickAlgorithm trait, StickSample, pipeline glue
    src/rc/mod.rs                    # RcConfig, RcMode, RcStickState, RcFilter (impl StickAlgorithm), dispatch
    src/rc/coeffs.rs                 # PeriodCoeffs: i32-truncated base_value, lead_base, sample_ms
    src/rc/curve.rs                  # 4-pt piecewise param; speed_legacy + speed_dt
    src/rc/firebird.rs               # bit-exact i32 Q4 integer oracle (153.3984375)
    src/rc/ultimate.rs               # UltimateLegacy (f64) + UltimateDt (rate-invariant, CORRECTED lead)
    src/input/mod.rs                 # InputSample, StickPair, Trigger, Buttons, SourceMeta
    src/input/normalize.rs           # 8/16-bit raw -> f32/f64 unit maps
    src/input/ds_report.rs           # decode_ds_usb_raw, parse_ds_usb_report (DS4-compat report 0x01)
    src/input/dt_clock.rs            # SensorClock: u16 wrap + 16/3us unit + QPC fallback (CORRECTED)
    src/input/seq.rs                 # SeqTracker drop/dupe (mod-256, DS byte7)
    src/output/mod.rs                # OutputFrame (f64), to_xinput_thumb, to_xinput_trigger (pure, re-used)
    src/config.rs                    # EngineConfig serde tree + clamps + defaults + load/save toml
    tests/goldens.rs                 # ported RcFilterTests.cs values (FireBird), tol 1e-4
    tests/rate_invariance.rs         # proptest 250/1000/4000Hz + jitter (UltimateDt)
    tests/reduction.rs               # UltimateDt(dt==periodUs) == UltimateLegacy, 1e-12
    tests/convert.rs                 # neutral->0.0, lossless 8->16, single-round
    tests/input_parse.rs             # synthetic DS report bytes, dt_clock, seq
    tests/config_roundtrip.rs        # load(save(cfg))==cfg, clamps, enum rename

  crates/hid-input/
    Cargo.toml                       # [target.'cfg(windows)'.dependencies] windows; dev feature "bringup"
    src/lib.rs                       # #![cfg(windows)] DeviceSource trait, SourceError, backend registry
    src/win/hid.rs                   # CreateFileW + overlapped ReadFile + GetOverlappedResult (double-buffered)
    src/win/enumerate.rs             # SetupDiGetClassDevs + HidD_GetAttributes VID/PID match
    src/backends/dualsense_usb.rs    # DualSenseUsbSource (I/O only; calls core parse)
    src/backends/xinput.rs           # XInputSource (XInput API fallback ~8ms)
    src/backends/raw_hid.rs          # RawHidGenericSource (user-supplied StickLayout)
    src/bringup/hidapi_src.rs        # feature="bringup" hidapi source (CORRECTNESS only, not latency)

  crates/vgamepad-output/
    Cargo.toml                       # [target.'cfg(windows)'.dependencies] windows, vigem-rust
    src/lib.rs                       # #![cfg(windows)] VirtualPad trait, Vigem360Pad, OutErr
                                     # (mapping fns live in core::output and are re-exported here)

  crates/platform-win/
    Cargo.toml
    src/lib.rs                       # #![cfg(windows)]
    src/hidhide.rs                   # HidHide IOCTL + CLI fallback
    src/timerres.rs                  # TimerResGuard (NtSetTimerResolution RAII; original captured)
    src/sched.rs                     # apply_hot_thread_policy -> HotPolicyGuard (MMCSS/affinity/prio)
    src/priority.rs                  # SetPriorityClass(HIGH_PRIORITY_CLASS)

  crates/engine/
    Cargo.toml                       # core, arc-swap, rtrb, triple-buffer, crossbeam-channel, notify;
                                     # [target.'cfg(windows)'] hid-input, vgamepad-output, platform-win
    src/lib.rs                       # pub mod hot, handoff, telemetry, config_store, clock, supervisor; run()
    src/hot.rs                       # the hot loop (HotThread::run)
    src/handoff.rs                   # ConfigHandle, TelemetryTx/Rx, CommandTx/Rx (TWO SPSC cmd queues)
    src/telemetry.rs                 # TelemetryFrame (Copy), fixed-bucket p99 reservoir
    src/clock.rs                     # DtTracker wrapper over core::SensorClock (Instant capture)
    src/config_store.rs              # ConfigStore (arc-swap single writer), file-watch, ControlMsg apply
    src/supervisor.rs               # [cfg(windows)] lifecycle: timer res, hidhide, vigem, hotplug

  crates/app/
    Cargo.toml                       # engine, eframe/egui 0.33, tray-icon 0.19, core_affinity
    src/main.rs                      # spawn supervisor+hot, run egui+tray loop, shutdown
    src/gui/mod.rs                   # HyperionApp (eframe::App), ControlMsg, ScopeSample
    src/gui/panels.rs                # per-stick RC panels, curve editor, scope plot
    src/gui/tray.rs                  # tray-icon built into the eframe winit loop
```

---

## 4. Core Algorithm Spec (with the verifier-corrected dt math)

### 4.1 Domains and the precision contract
- **Canonical pipeline unit:** `Axis = f64` in `[-1.0, 1.0]`, neutral = `0.0`, full scale `±1.0`. Stick math is radial/signed/centered, so this avoids a re-bias at every stage and makes 8-bit and 16-bit sources symmetric.
- **Internal RC domain:** the RC filter computes in a **DS4-compatible `[0,255]` f64 domain** (legacy neutral **128**, half-scale **127**) so the C# goldens port 1:1. A thin adapter wraps the live pipeline: `axis_to_ds4(a) = a.clamp(-1,1)*127 + 128`, `ds4_to_axis(d) = ((d-128)/127).clamp(-1,1)`. Golden tests call the filter in the ds4 `[0,255]` domain **directly** (no `[-1,1]` round-trip).
- **No mid-chain quantization:** raw int → f64 → (rc ds4 domain f64) → f64 → i16 XInput; **round exactly once** at the i16 egress.
- **Per-mode clamp constants are sacred and MUST NOT be unified:** Ultimate negative state clamps to `65280.0` (= 255*256); FireBird integer state clamps to `0xfff0 = 65520`. Confirmed in `RcFilter.cs:255` and `:27`.

### 4.2 Period-derived coefficients (i32-truncating — confirmed in C#)
```
periodUs   = clamp(periodUs, 1000, 8000)
sample_ms  = max(periodUs / 1000, 1)        // i32 truncate
base_value = 100 / sample_ms                // i32 truncate  (e.g. period 4000 -> 25)
lead_base  = 200000 / periodUs              // i32 truncate  (e.g. period 4000 -> 50)
```
Use **i32** division, never f64, or FireBird AND UltimateLegacy diverge. Recompute only on `periodUs` change (cache). Note (algo verifier (d)): `base_value` jumps at `sample_ms` boundaries (period 2999→3001 gives 50→33), so `periodUs` is a quantized knob even on the dt path; this is inherited from legacy and accepted.

### 4.3 Authoritative recurrences

**FireBird integer (bit-exact oracle, rate-coupled, frozen forever).** `i32` Q4, all divisions truncating. Headline golden reproduced by hand: from 128 hold 255, param 100, period 4000 → prime 32768; `32768 + 25*(65280-32768)/125 = 39270`; `39270/256 = 153.3984375`.
```
inputQ4  = raw12 << 4
positive low-pass: state += base_value*(inputQ4 - state) / (base_value + param)
negative (p=-param):
  blended = (lead_base*inputQ4 + p*state) / (p + lead_base)
  lead    = ((p+25)*(inputQ4 - prevQ4)) / 25
  state   = clamp(blended + lead, 0, 0xfff0)
```

**UltimateLegacy (f64, rate-coupled, default feel, converges exactly).** Operates in the `*256` Q-domain, matching `ProcessAxisUltimate`:
```
bypass (param==0): return clamp(input)                        // full precision, no 1/16 snap
scaled = input*256
positive: state += base_value*(scaled - state) / (base_value + param);  out = state/256
negative (p=-param):
  blended = (lead_base*scaled + p*state) / (p + lead_base)
  lead    = ((p+25)*(scaled - prev)) / 25.0
  state   = clamp(blended + lead, 0.0, 65280.0);  out = state/256
```

**UltimateDt (NEW, report-rate invariant).** Operates directly in `[0,255]` f64. `dt_us` is the guarded real elapsed time. **The low-pass and blend are dt-exponentiated; the lead is the CORRECTED displacement form (no `periodUs/dt` factor).**

Low-pass (positive) — exact on held input, O(dt) ZOH on moving input:
```
k    = base_value / (base_value + param)
k_dt = 1 - (1 - k)^(dt_us / periodUs)
state += k_dt * (input - state)
```

Negative (blend + CORRECTED lead):
```
r       = p / (p + lead_base)
r_dt    = r^(dt_us / periodUs)                       // exact-decay blend retention
blended = state*r_dt + input*(1 - r_dt)
lead    = ((p + 25) / 25) * (input - prev)           // g * displacement THIS STEP. NO *periodUs/dt.
state   = clamp(blended + lead, 0.0, 255.0)
```
> **CORRECTION applied (algo verifier issue (b), the only hard bug).** The original track's velocity lead `((p+25)/25)*((input-prev)/dt)*periodUs` injects a dt-independent kick *every report*, so total lead over a wall-clock interval scales like `1/dt` — it SATURATES at high rates (the exact fake-4K case this targets: ramp 100→140/40ms gave 191@250Hz but 255@1000/4000Hz). The continuous-time-correct injection over a step is `g·velocity·dt = g·(input-prev)`; the `periodUs` cancels. Corrected form gives bounded `191.10/181.76/179.52` across dt=4000/1000/250 (first-order convergent), and reduces to legacy at `dt=periodUs` to ~1e-13.

**Dynamic-curve speed metric** (mode-dependent). Speed feeds the 4-point piecewise-linear `param`.
```
legacy/integer (FireBird + UltimateLegacy):  speed = min(128, max(|dRawX|,|dRawY|) * periodUs / 8000)   // i32 truncate
dt-compensated (UltimateDt):                 speed = clamp((delta/dt_us) * periodUs^2 / 8000, 0, 128)
```
For UltimateDt, to match the firebird oracle param **exactly at `dt=periodUs`**, `.trunc()` (not `.round()`) the dt speed before clamp (legacy truncates). Genuinely rate-invariant (verifier confirmed identical 20.0 / 100.0 across all three rates for fixed wall-clock velocity).

**Bypass** is dt-independent in every mode.

### 4.4 Rate-invariance guarantee + the `dt = periodUs` reduction
Reduction (proven, code-pinned by `tests/reduction.rs` @ 1e-12):
- Low-pass: `k_dt = 1-(1-k)^1 = k`. Identical recurrence.
- Blend: `r_dt = r^1 = r` ⇒ `blended = (p·state + lead_base·input)/(p+lead_base)` exactly.
- Lead (corrected): `((p+25)/25)·(input-prev)` — already the legacy lead, independent of dt.
- Speed: `(delta/periodUs)·periodUs^2/8000 = delta·periodUs/8000`.

Rate invariance:
- **Exact on held / piecewise-constant input** (low-pass and blend): residual scales by `(1-k)^(T/periodUs)` for *any* partition of wall-clock `T` into dt's (telescoping product). Held input makes `input-prev=0`, so lead is 0 every step after the first.
- **On genuinely moving input it is first-order (O(dt)) rate-invariant, NOT bit-exact.** The right-endpoint ZOH sampling that makes `dt=periodUs` reduce to legacy is the same thing that leaves an O(dt) residual on moving signals (e.g. a smooth sinusoid: low-pass 135.148@dt=4000 vs 136.585@dt=250, a ~1.4-unit spread; halving dt halves the gap). **You cannot have both bit-exact legacy reduction AND 1e-6 rate invariance on moving input with one discretization.** We keep ZOH (legacy reduction is the priority); the optional ramp-exact form (`tau=-periodUs/ln(1-k)`, `a=exp(-dt/tau)`, trapezoidal update; spread <0.05) is documented in `ultimate.rs` but **not** the default because it breaks the 1e-12 reduction test.

### 4.5 dt guard + duplicates
`dt` is f64 µs supplied by the caller, clamped to **[100µs, 20000µs]**. First report after enable/reset calls `prime` (seed history, **no IIR step**, return input). Duplicate reports are **not specially skipped** by default: a duplicate has `input-prev=0` (lead 0) and a tiny real dt (negligible blend), so correctness needs only correct dt + seq plumbing. `skip_duplicate_reports` is an optional CPU-saving flag. The corrected lead no longer divides by dt, but `speed_dt` and the blend exponent still do — **keep the guard** (it also prevents div-by-zero on identical-timestamp frames).

---

## 5. StickAlgorithm Trait + Key Core Types

```rust
// axis.rs
pub type Axis = f64;                               // semantically [-1.0, 1.0], neutral 0.0
#[inline] pub fn clamp_axis(v: f64) -> f64 { v.clamp(-1.0, 1.0) }

#[derive(Clone, Copy)]
pub struct RawStickFormat { pub bits: u8, pub neutral: i32, pub min: i32, pub max: i32 }
impl RawStickFormat {
    pub const DS_8BIT:    Self = Self { bits: 8,  neutral: 128, min: 0,      max: 255   };
    pub const XINPUT_16:  Self = Self { bits: 16, neutral: 0,   min: -32768, max: 32767 };
    pub fn xinput(bits: u8) -> Self { /* symmetric signed range */ }
    #[inline] pub fn to_axis(&self, raw: i32) -> Axis {                 // neutral -> EXACTLY 0.0
        let d = raw - self.neutral;
        if d >= 0 { d as f64 / (self.max - self.neutral) as f64 }
        else      { d as f64 / (self.neutral - self.min) as f64 }
    }
    #[inline] pub fn from_axis(&self, a: Axis) -> i32 { /* round ONCE, clamp [min,max] */ }
}

// dt.rs
#[derive(Clone, Copy)] pub struct Dt(f64);          // guarded microseconds
pub const DT_MIN_US: f64 = 100.0;
pub const DT_MAX_US: f64 = 20_000.0;
impl Dt {
    #[inline] pub fn guarded(raw_us: f64) -> Self { Self(raw_us.clamp(DT_MIN_US, DT_MAX_US)) }
    #[inline] pub fn us(self) -> f64 { self.0 }
}

// stick.rs — the pluggable contract. Pure: out = f(sample, dt, cfg, &mut state). No clock, no alloc, no interior mut.
#[derive(Clone, Copy)] pub struct StickSample { pub x: Axis, pub y: Axis }   // canonical [-1,1]

pub trait StickAlgorithm {
    type Config;
    type State: Default;
    /// First report after enable/reset: seed history, return input unchanged (NO step).
    fn prime(&self, cfg: &Self::Config, st: &mut Self::State, s: StickSample);
    /// Advance one report by the guarded real elapsed time.
    fn process(&self, cfg: &Self::Config, st: &mut Self::State, dt: Dt, s: StickSample) -> StickSample;
    fn reset(&self, st: &mut Self::State) { *st = Self::State::default(); }
}

// rc/mod.rs
#[derive(Clone, Copy)] pub enum RcMode { FireBirdInteger, UltimateLegacy, UltimateDt }
#[derive(Clone, Copy)]
pub struct RcConfig {
    pub enabled: bool, pub mode: RcMode,
    pub use_dynamic_curve: bool, pub period_us: i32, pub fixed_param: i32,
    pub curve: RcCurve,
}
#[derive(Default)]
pub struct RcStickState {
    pub fb:  [FbAxisState; 2],
    pub ult: [UltAxisState; 2],
    pub prev_raw12: [i32; 2],
    pub coeffs: Option<PeriodCoeffs>,           // cached; recomputed on period change
}
pub struct RcFilter;                            // ZST implementing StickAlgorithm
impl StickAlgorithm for RcFilter { type Config = RcConfig; type State = RcStickState; /* dispatch on mode */ }
// process(): cache coeffs on period change; compute param (fixed, or speed->curve with mode-correct speed
//            metric); per-axis call firebird / ultimate-legacy / ultimate-dt. X,Y share param, independent state.
//            On ANY mode/period/curve change the caller MUST reset state (mirrors C# ResetRCFilter).

// input/mod.rs — what the device layer produces
#[derive(Clone, Copy)]
pub struct InputSample {
    pub left: StickPair, pub right: StickPair,        // f64 [-1,1] (track 1 canonical; f32 ok internally)
    pub l2: Trigger, pub r2: Trigger,                 // [0,1]
    pub buttons: Buttons,
    pub seq: u8,            // DS byte7 frame counter
    pub dropped: u16, pub is_duplicate: bool,
    pub dt_us: f64,         // guarded real elapsed since prev report
    pub host_qpc_ns: u64,
}

// output/mod.rs — what feeds the virtual pad (pure, re-used by vgamepad-output)
pub struct OutputFrame { pub lx:f64, pub ly:f64, pub rx:f64, pub ry:f64, pub lt:f64, pub rt:f64, pub buttons:u16 }
#[inline] pub fn to_xinput_thumb(n: f64) -> i16 {     // single quantization; asymmetric i16 like C# AxisScale
    let n = n.clamp(-1.0, 1.0);
    let v = if n >= 0.0 { n * 32767.0 } else { n * 32768.0 };
    v.round().clamp(-32768.0, 32767.0) as i16
}
#[inline] pub fn to_xinput_trigger(t: f64) -> u8 { (t.clamp(0.0,1.0) * 255.0).round() as u8 }
```

---

## 6. Threading Model + Lock-Free Config Hot-Swap (threading-verifier corrections applied)

**4 threads, one hot core.**
- **HOT (T1):** owns one HID device. Single outstanding overlapped `ReadFile`; waits (Blocking or HybridSpin); parses; measures dt; runs `core` filter in place; builds XUSB frame; **submits inline** via one synchronous `vigem update()` IOCTL; publishes telemetry. Output stays on the hot thread — a separate output thread only adds a queue hop + context switch for zero benefit (ViGEm does **not** bypass the game's XInput poll cadence).
- **GUI (T2):** `egui`/`eframe` on its own physical core. Reads triple-buffered telemetry, publishes `ControlMsg` to the engine.
- **TRAY (T3):** shares the GUI's single winit event loop (`tray-icon` + `eframe`) — not truly independent on Windows.
- **SUPERVISOR (T4):** low-priority; owns device hotplug, HidHide, ViGEm target lifecycle, file-watch, timer-resolution.

**Priority/affinity (corrections applied):**
- MMCSS **"Pro Audio" + `AVRT_PRIORITY_CRITICAL`** primary; fallback `SetThreadPriority(TIME_CRITICAL)` under process `HIGH_PRIORITY_CLASS`. **Never `REALTIME_PRIORITY_CLASS`.**
- **BLOCKER FIX (verifier (c)):** the policy guard MUST be **bound** for the thread's life: `let _policy = sched::apply_hot_thread_policy(...);` — a bare statement drops the RAII guard at the semicolon and reverts MMCSS one line later. Applied on the hot thread; reverted on the same thread at exit (Avrt handle is thread-affine). The hot thread is dedicated, never pooled.
- **Affinity:** `hot_core` auto-detect must return a **physical** core and **avoid its SMT sibling**; GUI pins to a *different physical* core (never default `None` onto core 0's sibling, or a HybridSpin loop saturating one SMT thread janks the GUI and steals execution ports). Supervisor floats.
- **Timer resolution (verifier (e)):** pick **one** mechanism — `NtSetTimerResolution` (0.5ms) only; capture original via `NtQueryTimerResolution` at begin, restore with `(orig, FALSE)` on Drop. Do not also `timeBeginPeriod(1)` (coarser, redundant). Owned by the supervisor, set **before** the hot thread spawns, dropped **after** `hot.join()` (ordering already correct).

**Read pattern (corrections applied):**
- ONE outstanding overlapped read, **double-buffered** (verifier (a) data-race fix): complete into `buf[cur]`, flip `cur`, re-arm into the *other* buffer, then parse the just-completed buffer. A single buffer races the driver's next write against the parse.
- `WaitMode::Blocking` (INFINITE wait, lowest CPU) or `HybridSpin` (bounded busy-poll of the OVERLAPPED with a QPC deadline, **then fall back to a real `WaitForSingleObject`** so a stalled BT report can never spin a TIME_CRITICAL thread forever — whole-core lockup otherwise).
- **Honest claim:** steady state is **zero-ALLOC and lock-free, NOT zero-syscall** (Blocking wait = `WaitForSingleObject` per report; `vigem update()` = `DeviceIoControl` per report). The vigem wrapper MUST guarantee a non-blocking/bounded submit, or the TIME_CRITICAL thread can stall in the driver.

**Lock-free hot-swap topology (corrections applied):**
- **Config GUI→HOT:** `Arc<ArcSwap<EngineConfig>>`, wait-free `load()` once per report (cheap-check a `generation` counter; only `apply_cfg` when it changes). `apply_cfg` MUST be a pure field-copy into pre-sized storage (no Vec growth / String / Box / coefficient-table realloc) — the C# F4 cached-constant discipline, on the hot thread, allocation-free. Verifier note: the freed old `Arc<EngineConfig>` Drop can land on the hot thread, so keep `EngineConfig::Drop` trivial/alloc-free; keep `load()` Guards short-lived (or use `arc_swap::Cache` on the hot side). `ArcSwap` is MPMC-safe, so GUI + supervisor + file-watch may all `store()`.
- **Single writer:** GUI edits do **not** mutate ArcSwap directly; they send `ControlMsg` over a `crossbeam-channel` to the engine, the **sole writer** of ArcSwap + TOML (debounces, validates/clamps via core, persists, publishes one new immutable snapshot). File-watch and GUI converge on this one path.
- **Telemetry HOT→GUI:** `triple-buffer` of a `Copy` `TelemetryFrame` (writer never blocks, reader always gets a complete frame). p99 via a **fixed-size reservoir / fixed buckets — no alloc**. **No `log`/`tracing`/`println!`/`format!` on the hot thread** (they lock a global logger + alloc a String).
- **Control →HOT (verifier (b) soundness fix):** `rtrb` is **SPSC** — a single `Producer` is **not** Sync for two producers. The original "GUI/supervisor share one Producer" is unsound (and `main.rs` pushing Shutdown was a third site). Fix: **two SPSC command queues**, one for GUI, one for supervisor; the hot loop `try_pop`s both. `main.rs` Shutdown goes through the supervisor's queue.
- **Scope HOT→GUI:** separate `rtrb` SPSC (4096) drained by GUI at 60Hz; GUI must visibly decimate / keep-latest (it overflows by design at 8kHz). `crossbeam-channel` only for non-hot supervisor↔GUI lifecycle/log.

```rust
// handoff.rs (corrected)
pub type ConfigHandle = std::sync::Arc<arc_swap::ArcSwap<EngineConfig>>;     // GUI/sup: store(); HOT: load()
pub struct TelemetryTx(pub triple_buffer::Input<TelemetryFrame>);            // HOT
pub struct TelemetryRx(pub triple_buffer::Output<TelemetryFrame>);          // GUI
pub enum HotCommand { ResetFilter, Recalibrate, ReplugTarget, Shutdown }
pub struct CommandTx(pub rtrb::Producer<HotCommand>);                        // ONE owner each
pub struct CommandRx { pub gui: rtrb::Consumer<HotCommand>, pub sup: rtrb::Consumer<HotCommand> } // HOT drains both
pub fn build_links() -> (ConfigHandle, (TelemetryTx, TelemetryRx), (CommandTx /*gui*/, CommandTx /*sup*/, CommandRx));
```

Hot loop order (zero-alloc steady state): wait+complete → re-arm into other buffer → parse just-completed buffer → `dt.next()` → seq delta → `cfg.load()` + cheap-gen-check → drain gui+sup cmd queues → (optional dup skip) → `filter.process(&cfg.filter, dt, &mut sticks)` → map to XUSB → `target.update()` inline → triple-buffer telemetry write.

---

## 7. Input Device Trait + DualSense Parse Contract + High-Poll XInput (open items)

```rust
// hid-input/src/lib.rs  (#![cfg(windows)])
pub trait DeviceSource: Send {
    fn meta(&self) -> hyperion_core::input::SourceMeta;
    /// Blocking read of next report into `out`. Ok(true)=fresh, Ok(false)=benign timeout, Err=device loss.
    fn next_sample(&mut self, out: &mut hyperion_core::input::InputSample) -> Result<bool, SourceError>;
    fn device_id(&self) -> DeviceId;     // VID:PID + instance path
}
pub enum SourceError { Disconnected, Io(std::io::Error), Timeout }
```

**DualSense USB parse contract (DS4-compatible report 0x01, 64 bytes). Backend = I/O only; all parsing in `core`.**
- Enumerate VID `0x054C` / PID `0x0CE6` (DualSense) or `0x0DF2` (DualSense Edge). `CreateFileW(FILE_FLAG_OVERLAPPED)`; match `HidD_GetAttributes` + `InputReportByteLength == 64`.
- Offsets (`buf[0] == 0x01`): `lx=1 ly=2 rx=3 ry=4 l2=5 r2=6 seq=7 btn0=8 btn1=9 btn2=10`.
- **dt source = the 16-bit hardware timestamp at `buf[10..12]` (little-endian u16), unit `16/3` µs/tick, wrap modulus 65535.** This is the **validated** C# path (`DS4Device.cs:1301` reads `(ushort)(inputReport[11]<<8 | inputReport[10])`, `:1305` `*16/3`, `:1308` wraps on `ushort.MaxValue`, `:1319` duplicate when delta==0).
- Sticks: `u8` 0x80-center → signed unit with asymmetric scale (`<0: /128, >=0: /127`) so `0x00→-1, 0x80→0, 0xFF→+1` exactly. Y is negated so `+y == up`. Triggers: `u8/255`.
- seq (byte7): `dropped = (seq - prev - 1) mod 256`, `dup = (seq == prev)`.

```rust
// core::input::dt_clock — CORRECTED (verifier 2 BLOCKER: u16 not u32; 16/3us not 1/3us; bytes 10-11 not 28-31)
pub const DSE_TS_UNIT_SECONDS: f64 = (16.0 / 3.0) * 1e-6;     // 5.3333us/tick, DS4Device.cs:1305
pub struct SensorClock { prev_ts: Option<u16>, prev_qpc_ns: u64 }
impl SensorClock {
    pub fn fold(&mut self, sensor_ts: u16, host_qpc_ns: u64) -> f64 {   // returns dt microseconds
        let dt_us = match self.prev_ts {
            None => 0.0,                                                 // prime: caller does NO filter step
            Some(p) => {
                let ticks = sensor_ts.wrapping_sub(p);                   // u16 = TRUE 16-bit hardware wrap
                if ticks != 0 { ticks as f64 * (16.0/3.0) }              // 16/3 us per tick
                else { (host_qpc_ns.saturating_sub(self.prev_qpc_ns)) as f64 / 1000.0 } // identical stamp -> QPC
            }
        };
        self.prev_ts = Some(sensor_ts);
        self.prev_qpc_ns = host_qpc_ns;                                  // ALWAYS advance, every call
        dt_us.clamp(0.0, 20_000.0)
    }
}
```
> Caller treats prime (`dt==0`) as **"do not advance the IIR"**, not "step with a tiny dt" (a tiny-dt step still moves the filter). `prev_qpc_ns` advances on **every** call (device-ts path included) or the first QPC fallback after a device-ts run returns a huge accumulated dt.

**DSE caveats (HW-verify, parser reads but marks unstable):** paddle/Fn/Mute bits in btn2 (Mute byte) bits 4..7 (FnL/FnR/BLP/BRP per C#) are under-documented for the Edge superset.

**High-poll XInput controller — what we still need from the user (cannot be inferred):**
1. Exact model + VID:PID.
2. Does it expose a **non-XInput mode** (HID / DInput / vendor) at all? Standard XInput pads expose **no raw HID** and are firmware-locked to ~8ms.
3. The **HID report descriptor** (or a USBPcap/Wireshark capture of one input report) so `FieldSpec` offsets/bit-widths/center are known.
4. Confirmed achievable report rate + stick bit depth per mode.

Two backends behind the same trait: `RawHidGenericSource` (only when (2)+(3) are supplied; user `StickLayout` drives offsets) and `XInputSource` (fallback: `XInputGetState`, i16 thumbs → lossless unit, u8 triggers, synth seq, QPC-only dt, ~125Hz). **We do not fake raw reads of an arbitrary XInput pad.** If the user's pad has no non-XInput mode, the "1k–8kHz / 8–16bit" adjustability simply does not exist for it on Windows — set as expectation, not engineered around.

---

## 8. Output / HidHide Plan

**Virtual output (`vgamepad-output`, `#![cfg(windows)]`):**
```rust
pub trait VirtualPad {
    fn plugin(&mut self) -> Result<(), OutErr>;                 // create Xbox360 target, plug into ViGEmBus
    fn wait_ready(&mut self, timeout: Duration) -> Result<(), OutErr>;  // block until OS enumerates
    fn update(&mut self, f: &OutputFrame) -> Result<(), OutErr>;        // ONE synchronous DeviceIoControl IOCTL
    fn unplug(&mut self);
}
pub struct Vigem360Pad { client: vigem::Client, target: vigem::Xbox360Target }
```
`update()` maps `OutputFrame` (f64) → `XUSB_REPORT` via `core::output::to_xinput_thumb/_trigger` (single final round, asymmetric i16 like C# `AxisScale`). Load-bearing module doc: *"ViGEmBus delivers each update via one synchronous DeviceIoControl. The game still polls XInput at ITS cadence; pushing updates faster only reduces the age of the latest sample the game reads — it does not raise the game's effective poll rate."* The vigem wrapper must expose a non-blocking/bounded submit (see §6).

**HidHide (`platform-win`):** direct `DeviceIoControl` IOCTLs via `windows-rs`, with `HidHideCLI.exe` shell-out as bring-up/fallback behind a config flag.
```
on start: open() -> whitelist_self() (MANDATORY, else we hide the pad from ourselves)
          -> blacklist_device(physical DSE instance path / high-poll pad path) -> set_active(true)
on exit:  clear_blacklist() + set_active(false)  (so the pad reappears)
```
IOCTLs (`CTL_CODE(FILE_DEVICE_UNKNOWN, 0x80x, METHOD_BUFFERED, FILE_READ_DATA)` for GET/SET WHITELIST/BLACKLIST/ACTIVE) are **under-documented → HW-verify**; CLI fallback (`HidHideCLI.exe --cloak-on --app-reg <self> --dev-hide <instancePath>`) is the known-good bring-up path. ViGEmBus is EOL — isolated behind this crate so the backend is swappable (LizardByte mirror / Nefarius VirtualPad).

---

## 9. Config TOML Schema (serde types in `core`)

```toml
active_device = "dse_primary"

[thread]
hot_core = 2                      # Option<usize>; None => auto physical core (avoid SMT sibling)
gui_core = 4
timer_resolution_us = 500         # NtSetTimerResolution target
use_mmcss = true
mmcss_task = "Pro Audio"
wait_mode = "HybridSpin"          # Blocking | HybridSpin
spin_budget_us = 80               # 0 => Blocking; e.g. 80 for 8kHz
skip_duplicate_reports = false
dt_source = "DeviceTimestamp"     # DeviceTimestamp | QpcOnly  (default QpcOnly until DSE tick HW-verified)

[hidhide]
enabled = true
use_cli = false
cli_path = "C:\\Program Files\\Nefarius Software Solutions\\HidHide\\x64\\HidHideCLI.exe"

[devices.dse_primary]
vid = 0x054C
pid = 0x0DF2                       # DualSense Edge
report_rate_hz = 4000
stick_bits = 8

  [devices.dse_primary.ls]
  mode = "Rc"                      # None | Rc
    [devices.dse_primary.ls.rc]
    algorithm = "UltimateDt"       # FireBirdInteger | UltimateLegacy | UltimateDt
    use_dynamic_curve = false
    period_us = 4000               # clamp [1000, 8000]
    fixed_param = 100              # clamp [-500, 500]
    curve = { y0 = 100, x1 = 32, y1 = 100, x2 = 96, y2 = 100, y3 = 100 }

  [devices.dse_primary.rs]
  mode = "Rc"
    [devices.dse_primary.rs.rc]
    algorithm = "UltimateDt"
    use_dynamic_curve = false
    period_us = 4000
    fixed_param = 100
    curve = { y0 = 100, x1 = 32, y1 = 100, x2 = 96, y2 = 100, y3 = 100 }
```
Constants mirrored from C#: `MIN_PERIOD_US=1000`, `MAX_PERIOD_US=8000`, `MIN_PARAM=-500`, `MAX_PARAM=500`, `MIN_SPEED=0`, `MAX_SPEED=128`. `RcConfig::clamped()` clamps period/param and enforces `x2 >= x1` (C# F12). serde enum `#[serde(rename_all="PascalCase")]`; unknown enum → defensive fallback (mode→`None`, algorithm→`UltimateDt`). `load_toml`/`to_toml` round-trip; defaults match C# `Reset()`. Defaults table per AGENTS.md: disabled→here mapped via `mode`, fixed param, period 4000, fixed_param 100, curve `100/32/100/96/100/100`, algorithm Ultimate-class.

---

## 10. egui GUI Plan

`HyperionApp : eframe::App` on its own thread/core, never touching the hot path.
- **Inputs:** drains a `rtrb` `ScopeSample` ring (non-blocking pop loop) at 60Hz (`ctx.request_repaint_after(16ms)`), keeps a fixed history ring, decimates/keep-latest (the ring overflows by design at 8kHz).
- **Panels (per stick LS/RS):** mode combo `[None, Rc]`; algorithm combo `[FireBirdInteger, UltimateLegacy, UltimateDt]`; fixed/dynamic toggle; `period_us` slider `[1000..8000]`; `fixed_param` slider `[-500..500]`; curve editor (drag `y0/x1/y1/x2/y2/y3`); live input-vs-output stick scope (two egui plots: dot + trail).
- **Edits never mutate config directly, never take a Mutex.** Any widget change → `tx.send(ControlMsg::SetRc{device, stick, rc})` to the engine (sole writer). Other msgs: `SetStickMode`, `SetThread`, `SetHidHide`, `ReloadFromDisk`, `SaveToDisk`, `ToggleActive`.
- **Tray:** `tray-icon 0.19` built **into** the eframe/winit event loop (one loop per locked stack). A GUI hang freezes the tray (acceptable; neither touches the hot path), so GUI robustness matters for the Shutdown path. Pin both `eframe 0.33` / `tray-icon 0.19` versions (shared-loop pattern is version-sensitive).

```rust
#[derive(Clone, Copy)]
pub struct ScopeSample { pub t_us:u64,
    pub in_lx:f32, pub in_ly:f32, pub in_rx:f32, pub in_ry:f32,
    pub out_lx:f32, pub out_ly:f32, pub out_rx:f32, pub out_ry:f32 }
pub enum ControlMsg {
    SetStickMode{device:String, stick:Stick, mode:StickMode},
    SetRc{device:String, stick:Stick, rc:RcConfig},
    SetThread(ThreadConfig), SetHidHide(HidHideConfig),
    ReloadFromDisk, SaveToDisk, ToggleActive(bool),
}
```

---

## 11. CI Plan (`.github/workflows/rust.yml`)

Separate workflow from the existing `.NET Build` (`build.yml`); scoped to `rust/**` paths. `Swatinem/rust-cache` on both jobs.

**Job `core` (ubuntu-latest) — the gate that must always be green:**
```
cargo fmt --all --check
cargo test  -p hyperion_core --all-features            # goldens, rate-invariance, reduction, convert, parse, config
cargo test  -p engine --lib                            # pure modules: clock, seq, handoff, config_store
cargo clippy -p hyperion_core -p engine -- -D warnings # (no cfg(windows) crates compiled on linux)
```

**Job `windows` (windows-latest):**
```
cargo build  --workspace
cargo test   --workspace                               # incl. headless-safe smoke (no real device/driver)
cargo clippy --workspace -- -D warnings                # type-check all cfg(windows) HID/vigem/hidhide/sched
```
Windows smoke (no assertion on achieved latency — shared runner too noisy): `TimerResGuard` begin/drop (assert restore called), `HIGH_PRIORITY_CLASS` set, MMCSS register+revert returns a handle or falls back cleanly, `DualSenseUsbSource::next_sample` calls the core parser on an injected fake buffer (bypassing ReadFile) to test the I/O→core seam.

**Explicitly NOT in CI (HW-only, author's box):** real enumeration/open + HidHide whitelist interaction; overlapped-read latency at 1k/4k under MMCSS/affinity; DSE `sensor_timestamp` tick calibration (device-ts dt vs QPC); fake-4K dupe ratio (~3/4); ViGEm `plugin/wait_ready/update` against a real driver; end-to-end jitter (`loop_busy_ns` p50/p99); HybridSpin vs Blocking A/B; the **feel** of dt compensation.

---

## 12. ROADMAP (phased)

### M1 — End-to-end vertical slice (DualSense USB → core RC UltimateDt → virtual Xbox360)
**Goal:** a real, latency-honest slice that builds + is green on CI, with the core golden + rate-invariance tests passing.
- Scaffold the workspace + all crates per §3 (empty but compiling).
- `core`: `axis`, `convert`, `dt`, `stick`, full `rc` (FireBirdInteger + UltimateLegacy + **UltimateDt with the corrected lead**), `input` (DS parse + **corrected u16/16-3µs SensorClock** + seq), `output` mapping, `config`.
  - **Tests that MUST pass on ubuntu CI:** `goldens.rs` (FireBird `128→255 @param100,period4000 → 153.3984` at tol 1e-4, the negative-lead 255.0 clamp, dynamic-curve bypass cases, zero-param-no-refresh, independent pos/neg history, disabled/reset); `reduction.rs` (UltimateDt @ `dt==periodUs` ≡ UltimateLegacy, 1e-12, all branches + speed); `rate_invariance.rs` (proptest 250/1000/4000Hz + jitter: low-pass as a **convergence** test `|out(4000)-out(250)| ≤ C·dt` or held-segment equality — **not** 1e-6 equality on moving input; lead bounded/convergent; duplicates contribute ~0); `convert.rs` (neutral→0.0, lossless 8→16, single round); `input_parse.rs` (synthetic DS bytes, u16 wrap 65535→0, identical-stamp→QPC fallback, seq mod-256 wrap); `config_roundtrip.rs`.
- `hid-input` (`DualSenseUsbSource`), `vgamepad-output` (`Vigem360Pad`), `platform-win` (timer res + MMCSS + affinity + HidHide whitelist_self), `engine` (hot loop with double-buffered read, two SPSC cmd queues, bound policy guard, arc-swap config), `app` (spawn + minimal GUI/tray).
- **CI:** ubuntu `core`+`engine` green; windows `--workspace` builds + clippy `-D warnings` + headless smoke.
- **HW bring-up (author, off-CI):** open DSE over USB through HidHide; submit to a real ViGEm Xbox360 pad a game enumerates; cross-check device-ts dt vs QPC to calibrate/confirm `DSE_TS_UNIT_SECONDS`; default `dt_source = QpcOnly` until that cross-check passes.
- **Exit:** stick physically moves a game's right stick through `UltimateDt`; CI fully green.

### M2 — Tuning surface + dynamic curve + telemetry
- Full egui panels (all RC knobs, curve editor, input/output scope), `ControlMsg` apply path (single-writer ArcSwap + TOML persist + debounce), file-watch hot-reload, telemetry triple-buffer with fixed-bucket p99, dropped/dupe counters. UltimateLegacy + FireBirdInteger selectable. Dynamic-curve speed (legacy + dt forms).

### M3 — Latency hardening + measurement
- HybridSpin with QPC-deadline + block fallback; MMCSS p99 monitoring; SMT-sibling-aware auto core detection; non-blocking vigem submit guarantee; `loop_busy_ns`/`dt_p99` telemetry; A/B Blocking vs HybridSpin and `skip_duplicate_reports` on real hardware.

### M4 — Device breadth + robustness
- `XInputSource` fallback; `RawHidGenericSource` once the user supplies VID:PID + non-XInput mode + report descriptor; multi-device profiles; hotplug; BT-stall re-prime after a clamped-dt stall; DSE paddle/Fn/Mute bits verified on hardware.

### M5 — Output backend swap-readiness + Apex curves
- Abstract the ViGEm target so a Nefarius VirtualPad successor drops in; port Apex Classic inverse output curves (radial + axial) into `core` (they run after RC, per the validated pipeline order).

---

## Resolved Conflicts (winner + one-line justification)

1. **dt source width/unit/offset — u16 @ bytes 10-11 @ 16/3µs (engine-verifier) vs u32 @ bytes 28-31 @ 1/3µs (three tracks).** Winner: **u16, bytes 10-11, 16/3µs, wrap mod 65535** — the ground-truth `DS4Device.cs:1301-1319` reads exactly this; a u32 wrapping_sub never detects the real ~349ms hardware wrap and silently saturates dt to 20ms ~3×/sec. The DSE 32-bit `sensor_timestamp` superset stays an **unvalidated HW-verify** item, not the M1 path.
2. **Negative lead form — velocity `((p+25)/25)·(in-prev)/dt·periodUs` (track) vs displacement `((p+25)/25)·(in-prev)` (algo-verifier).** Winner: **displacement form** — the velocity form's spurious `periodUs/dt` injects a per-report kick scaling like 1/dt and saturates at high rates (the opposite of rate-invariant); displacement is continuous-time-correct and bounded.
3. **Canonical pipeline unit — f64 (tracks 1/4) vs f32 (track 3).** Winner: **f64** for the canonical pipeline/output type (52-bit mantissa, trivially lossless for 16-bit, matches C# double path and the RC ds4 domain). f32 is fine *inside* the input-normalize boundary but the public `InputSample`/`OutputFrame`/`Axis` are f64.
4. **Rate-invariance test tolerance — 1e-6 on moving input (track) vs convergence/held-segment (algo-verifier).** Winner: **convergence test** (`error ≤ C·dt`, or equality only on held segments) — ZOH leaves an irreducible O(dt) residual on moving signals; a flat 1e-6 is unachievable and would fail CI.
5. **Config hot-path primitive — ArcSwap whole-snapshot (all tracks agree) vs triple-buffer for config (rejected).** Winner: **ArcSwap** for config (infrequent writes, wait-free MPMC load); triple-buffer reserved for telemetry; rtrb for commands/scope.
6. **Command queue topology — one shared rtrb Producer (track 2) vs per-producer SPSC (engine-verifier).** Winner: **two SPSC queues** (GUI + supervisor), hot drains both — sharing one `rtrb::Producer` across threads is unsound (SPSC, not Sync).
7. **Timer resolution — `timeBeginPeriod(1)` + `NtSetTimerResolution(0.5ms)` (track) vs Nt-only (engine-verifier).** Winner: **`NtSetTimerResolution` only**, original captured/restored — the dual mechanism is redundant and the Drop only undid `timeBeginPeriod`, leaking the Nt request.
8. **Output IOCTL placement — dedicated output thread (spec) vs inline on hot thread (track 2).** Winner: **inline** — `vigem update()` is one synchronous IOCTL; a separate thread adds a queue hop + context switch for zero benefit since ViGEm doesn't bypass the game's poll cadence.

## Remaining Unresolved / HW-gated (flagged, not blockers for M1 core)
- **`DSE_TS_UNIT_SECONDS` for the overclocked-4000Hz DSE:** assumed `16/3 µs` from the validated DS4-compat path; if the Edge's native/overclocked report uses a different field/scale, dt is mis-scaled. Mitigation: ship `dt_source = QpcOnly` default + runtime device-ts-vs-QPC cross-check with auto-fallback; calibrate on hardware before trusting `DeviceTimestamp`.
- **ZOH vs ramp-exact UltimateDt discretization** is a feel choice, not a firmware-pinned constant; default ZOH (keeps legacy reduction), ramp-exact documented but off. Final lead feel needs hardware A/B.
- **HidHide IOCTL codes, DSE paddle/Fn/Mute bits, vigem-rust exact API surface, ViGEmBus EOL IOCTL stability** — all HW/Windows-verify; CLI fallback covers HidHide bring-up.
- **High-poll XInput pad** is entirely gated on the user supplying the 4 items in §7; absent a non-XInput mode it is firmware-capped at ~8ms via `XInputSource`.
