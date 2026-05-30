# Hyperion — Usage & Hardware Bring-up

Hyperion is a Windows-only, ultra-low-latency controller engine: it reads a
physical pad over **raw HID**, runs the **RC stick filter** (rate-invariant
dt-compensated Ultimate by default) in a full-precision `f64` pipeline, and
re-emits a **virtual Xbox 360 pad** via ViGEmBus — with a live egui tuner.

> Status: the software is complete and CI-green (build + link + tests on
> Linux core + windows-latest). The numeric core is proven by 160+ tests.
> The **device/driver I/O is written but not yet hardware-validated** — the
> checklist below is the last mile, and it needs your actual hardware.

## 1. Prerequisites (install once)

1. **ViGEmBus** — the virtual-gamepad driver (required for output).
   Install a signed build (Nefarius original, or the LizardByte mirror).
2. **HidHide** *(optional but recommended)* — hides the physical pad so the
   game sees only the virtual one (prevents double input). Install the MSI;
   Hyperion drives `HidHideCLI.exe` (path in `config.toml`).
3. A **DualSense / DualSense Edge over USB** (the M1 input path). Other pads:
   see §5.

## 2. Run

1. Put `hyperion.exe` and `config.toml` in the same folder.
2. Plug in the DualSense (USB).
3. Run `hyperion.exe`. The egui tuner opens; a system-tray icon gives
   Show/Hide/Quit.
4. In game, select the **Xbox 360 controller**. Tune live in the GUI.

`config.toml` ships with a `dse_primary` profile (DualSense Edge, PID
`0x0DF2`; change to `0x0CE6` for a base DualSense). The GUI's **Save** writes
edits back to it.

## 3. Tuning (what the knobs do)

- **Algorithm**: `UltimateDt` (default — report-rate-invariant, the right
  choice for mixed 1k/4k/8k), `UltimateLegacy` (rate-coupled f64), or
  `FireBirdInteger` (bit-exact firmware replica).
- **fixed_param**: `> 0` low-pass **smoothing** (kills jitter, adds lag);
  `< 0` **lead/fast-follow** (anticipates, reduces felt latency, can
  overshoot); `0` = bypass (raw, most responsive).
- **period_us**: in `UltimateDt` this is a **true time constant** — the feel
  stays the same whether your pad reports at 1 kHz or 8 kHz.
- **Dynamic curve**: map stick **speed → param** (slow = smooth aim, fast =
  responsive flick) via the 4-point curve editor.
- The **scope** shows raw input vs filtered output per stick so you can see
  the effect while tuning, plus `dt`, dropped/duplicate counts, and loop p99.

## 4. Hardware bring-up checklist (the last mile — needs your box)

The numeric core is proven; these I/O assumptions need a real device to
confirm (each is marked `// HW-verify` in the source):

- [ ] **Enumeration/open**: `hyperion.exe` finds and opens the DualSense
      (VID `0x054C`, PID `0x0CE6`/`0x0DF2`, 64-byte report) through HidHide.
- [ ] **ViGEm output**: the virtual Xbox 360 pad appears and a game reads it;
      moving the physical stick moves the in-game stick through the filter.
- [ ] **dt source**: `dt_source = "QpcOnly"` is the safe default. To use the
      device hardware timestamp, set `"DeviceTimestamp"` and confirm the
      tick scale (`16/3 µs`) by cross-checking device-ts dt vs QPC dt at a
      known report rate — they should agree.
- [ ] **HidHide**: confirm the physical pad is hidden from the game *and*
      whitelisted to `hyperion.exe` (no self-lockout). IOCTL path is WIP; the
      CLI path (`use_cli = true`) is the working one.
- [ ] **Latency feel**: A/B `wait_mode` Blocking vs HybridSpin, and
      `skip_duplicate_reports` on/off, at your real report rate.
- [ ] **DSE specifics**: paddle/Fn/Mute bits and the overclocked-4k report
      cadence are under-documented for the Edge — verify if you remap them.

## 5. The adjustable-resolution XInput controller

Standard Xbox/XInput pads expose **no raw HID** (firmware-locked ~8 ms), so
your high-poll 1k–8k / 8–16-bit pad can only be raw-read if it has an HID /
DInput / vendor mode. To wire it up, provide: exact **model + VID:PID**,
whether it has a non-XInput mode, and its **HID report descriptor** (or a
USBPcap capture of one report). A `RawHidGenericSource` backend then maps it;
otherwise it falls back to the XInput API (~8 ms, lossless 16-bit thumbs).

## 6. Build from source

```bash
# Windows (full app):
cargo build --release -p app          # -> target/release/hyperion.exe

# Anywhere (prove the numeric core):
cargo test -p hyperion-core           # 160+ tests, no OS deps
# Type-check the Windows runtime from Linux (no linker needed):
cargo check --workspace --target x86_64-pc-windows-msvc
```

See `DESIGN.md` for the full architecture, the corrected dt-compensation
math, and the roadmap.
