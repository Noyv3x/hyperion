# NOTICE — Attribution and Provenance

Hyperion is licensed **GPL-3.0-or-later**. The full license text is in the
[`LICENSE`](LICENSE) file at the repository root.

## Why GPL-3.0-or-later

This Rust engine is a from-scratch rewrite, but its core stick-processing math is
a **port of the RC stick filter algorithm** from the author's own GPL-3.0
DS4Windows fork, **Hyperion-ds4w**. Because that algorithm is carried over, this
project inherits and preserves the copyleft license of its lineage.

## Lineage

- **DS4Windows** — the original project, in the line
  **Jays2Kings → Ryochan7 → schmaldeo**.
- **Hyperion-ds4w** — the author's GPL-3.0 fork of DS4Windows, where the RC stick
  filter (`DS4Control/RcFilter.cs` and the `DS4Library/DS4Device.cs` timestamp /
  dt path) was developed and validated.
- **Hyperion** (this repository) — a clean Rust re-implementation of that engine,
  porting the validated RC filter into a pure, OS-free numeric core
  (`hyperion-core`).

The **RC algorithm itself was reverse-engineered from FireBird firmware**; the
bit-exact FireBird integer mode in `hyperion-core` reproduces that firmware's
fixed-point recurrence as the authoritative oracle, and the Ultimate / dt-compensated
modes are derived from it.

Per the GPL, all derivative and ported work in this repository remains under
**GPL-3.0-or-later**. See [`LICENSE`](LICENSE) for the complete terms.
