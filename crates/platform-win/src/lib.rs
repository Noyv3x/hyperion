//! `hyperion-platform-win` — Windows host-policy plumbing for the hot path.
//!
//! Everything here is `#![cfg(windows)]` and shapes the OS around the latency-critical thread
//! (DESIGN §6, §8):
//!
//! * [`timerres`] — raise the system timer resolution via `NtSetTimerResolution`, restored on
//!   `Drop` ([`TimerResGuard`]).
//! * [`sched`] — apply the hot-thread MMCSS / affinity / priority policy, reverted on `Drop`
//!   ([`HotPolicyGuard`]).
//! * [`priority`] — process priority class (`HIGH_PRIORITY_CLASS`, never `REALTIME`).
//! * [`hidhide`] — hide the physical pad from everything but us via HidHide IOCTLs (CLI fallback).
//!
//! The guards' `Drop` impls are **real and no-op-safe**: dropping a guard that never acquired its
//! resource does nothing. The Win32 acquisition bodies are M1 bring-up stubs (`TODO(hardware)`);
//! the signatures and RAII lifetimes are final. On non-Windows targets the crate is empty.
#![cfg(windows)]

pub mod hidhide;
pub mod priority;
pub mod sched;
pub mod timerres;

pub use hidhide::HidHide;
pub use priority::{set_high_priority_class, PriorityClassGuard};
pub use sched::{apply_hot_thread_policy, HotPolicyGuard, HotThreadConfig};
pub use timerres::{begin_timer_resolution, TimerResGuard};
