# Hyperion

An **ultra-low-latency esports controller engine**, written from scratch in Rust.

Hyperion is a clean-room Rust rewrite of **Hyperion-ds4w** (the author's GPL-3.0
DS4Windows fork). It intercepts a physical pad (DualSense / DualSense Edge over USB
first), runs a validated **RC stick filter** in a pure, OS-free numeric core, and
re-emits a virtual Xbox 360 pad via ViGEm — targeting the lowest controllable input
latency, no unnecessary requantization (full-precision `f64` from de-quantization to
a single final `i16` write), and the lowest achievable CPU load.

The RC filter ships three modes: a **bit-exact FireBird integer** oracle, the
**Ultimate** `f64` mode, and a new **report-rate-invariant, dt-compensated Ultimate**.

> See [`DESIGN.md`](DESIGN.md) for the authoritative blueprint (algorithm math,
> threading model, lock-free handoff, config schema, roadmap). It is the source of
> truth; this README is a summary.

## Crate graph

```
hyperion-core        (pkg hyperion_core)  ZERO OS deps. All numerics + types: axis units,
                                          precision conversion, the StickAlgorithm trait, the
                                          three RC modes, the dt contract, input parse/normalize,
                                          config serde, and output mapping math. 100% Linux-CI
                                          testable — the source of truth.
hid-input    [win]   #![cfg(windows)]     I/O-only HID shell: enumerate VID/PID, overlapped
                                          ReadFile, hand &[u8] + QPC timestamp to the core parsers.
vgamepad-output [win] #![cfg(windows)]    VirtualPad trait + Vigem360Pad. Maps the core OutputFrame
                                          to an XUSB report (single final round). One IOCTL.
platform-win   [win] #![cfg(windows)]     HidHide (IOCTL + CLI fallback), timer-resolution RAII,
                                          MMCSS, affinity, process priority class.
engine               core (always) +      Thread architecture + lock-free handoff. Owns the hot loop,
                     [win] the 3 above     dt measurement, telemetry, config store, supervisor.
                                          Pure parts (clock, seq, handoff, config_store) are Linux-testable.
app          [win]   bin "hyperion"       Starts the non-blocking engine Runtime, then runs the
                                          LIVE egui tuning GUI + tray on the main thread (M2). The
                                          GUI only reads the telemetry triple-buffer and sends
                                          ControlMsg edits to the single config writer.
```

Dependency direction: `core` depends on nothing OS. `engine` depends on `core`
unconditionally and on `{hid-input, vgamepad-output, platform-win}` only under
`[target.'cfg(windows)'.dependencies]`. `app` depends on `engine`. This keeps the
core (and the engine's pure modules) building and testing on Linux with no Windows
toolchain.

## Build & test

```sh
# Build everything (on Windows; the cfg(windows) crates are dependency-free skeletons in M1)
cargo build --workspace

# Run the pure-core test suite anywhere (Linux, macOS, Windows)
cargo test -p hyperion-core
```

## Dev / verify model

The numeric core is fully testable off-Windows: **`hyperion-core` (and the engine's
pure modules) are developed and tested on Linux and gated in CI on `ubuntu-latest`**.
Everything Windows-specific is `#![cfg(windows)]` and is built, tested, and linted on
`windows-latest` via **GitHub Actions** — see [`.github/workflows/ci.yml`](.github/workflows/ci.yml).
Real-hardware bring-up (HID open through HidHide, ViGEm enumeration, latency under
MMCSS/affinity) happens off-CI on the author's box.

## M1 status

M1 is an **end-to-end vertical slice**: DualSense USB → core RC `UltimateDt` →
virtual Xbox 360. The numeric core is implemented and tested (FireBird goldens,
`dt == periodUs` reduction, rate-invariance, conversion, input parse, config
round-trip); the Windows I/O crates are compiling skeletons being filled in.
There are **no `windows` / ViGEm / egui dependencies in M1** — the platform crates
are dependency-free `cfg(windows)` skeletons so the workspace builds cleanly while
the slice comes together.

## M2 status

M2 adds the **live egui tuning GUI** (Windows). `app` now starts the non-blocking
[`engine::Runtime`], hands the GUI a telemetry reader + a `ControlMsg` sender + a seed
config snapshot, and runs eframe on the main thread; on exit it calls `Runtime::shutdown`
(which joins the hot loop, restoring timer resolution, then the config-writer thread).

The GUI is strictly **off the hot path** (`DESIGN.md` §6/§10): it only *reads* the
triple-buffered telemetry frame and *sends* `engine::ControlMsg` edits to the engine's
single config-writer thread — it never locks anything the hot loop touches and never
writes the config `ArcSwap` directly. It offers:

- per-stick (LS / RS) panels: mode (None / RC), RC algorithm
  (FireBird integer / Ultimate legacy / Ultimate dt), fixed-vs-dynamic param, `period_us`
  and `fixed_param` sliders, and a 4-point **dynamic-curve editor** with a live preview;
- a global **thread / scheduling** + **HidHide** policy panel, and **Save** / **Reload**
  buttons (`SaveToDisk` / `ReloadFromDisk`);
- a live **stick scope** (input dot vs. filtered-output dot + a short trail) so you *see*
  the RC filter's effect while tuning, plus `dt`, `loop busy`, `dropped`, `duplicates`,
  and rate readouts;
- a **system tray** (Show / Hide / Quit) integrated into the shared eframe/winit loop.

The eframe/egui/tray-icon dependencies are gated under
`[target.'cfg(windows)'.dependencies]` so the Linux core CI job (which never builds `app`)
and a local Linux `cargo check --workspace` stay **gtk-free**; the GUI code is
`#[cfg(windows)]` and type-checked for `x86_64-pc-windows-msvc`.

## High-poll XInput controller (input needed)

Hyperion can drive a high-poll-rate XInput-class pad, but the engine **cannot infer**
the device's wire format. Per [`DESIGN.md` §7](DESIGN.md), the user must supply:

1. the exact model and **VID:PID**,
2. whether it exposes a **non-XInput mode** (raw HID / DInput / vendor) at all,
3. the **HID report descriptor** (or a USBPcap/Wireshark capture of one input report)
   so stick offsets / bit-widths / center are known, and
4. the confirmed achievable report rate and stick bit depth per mode.

Standard XInput pads expose no raw HID and are firmware-locked to ~8 ms; absent a
non-XInput mode, such a pad falls back to the `XInputSource` path (~125 Hz).

## License

GPL-3.0-or-later. See [`LICENSE`](LICENSE) and [`NOTICE.md`](NOTICE.md) for
attribution and provenance.
