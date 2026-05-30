//! Hot-thread scheduling policy: MMCSS + affinity + thread priority, as one RAII guard.
//!
//! DESIGN §6 (verifier (c), the BLOCKER): the policy guard MUST be **bound** for the thread's
//! life — `let _policy = sched::apply_hot_thread_policy(..);`. A bare statement drops the guard at
//! the semicolon and reverts MMCSS one line later. The policy is applied **on** the hot thread and
//! reverted **on the same thread** at exit (the Avrt handle is thread-affine; the hot thread is
//! dedicated, never pooled).
//!
//! Policy: MMCSS "Pro Audio" + `AVRT_PRIORITY_CRITICAL` primary; fall back to
//! `SetThreadPriority(TIME_CRITICAL)` under process `HIGH_PRIORITY_CLASS`. Affinity pins to a
//! **physical** core and avoids its SMT sibling (a HybridSpin loop must not saturate the GUI
//! core's sibling).

/// Wait strategy of the hot loop (affects whether SMT-sibling avoidance matters).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum WaitMode {
    /// Blocking `WaitForSingleObject` — does not busy-spin a core.
    #[default]
    Blocking,
    /// Bounded busy-poll then block — saturates its core, so SMT-sibling isolation matters.
    HybridSpin,
}

/// Configuration for the hot-thread policy.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HotThreadConfig {
    /// Logical core to pin the hot thread to. `None` ⇒ auto-detect a physical core whose SMT
    /// sibling is left free.
    pub hot_core: Option<usize>,
    /// Whether to register with MMCSS.
    pub use_mmcss: bool,
    /// MMCSS task name (e.g. "Pro Audio").
    pub mmcss_task: String,
    /// The hot loop's wait strategy.
    pub wait_mode: WaitMode,
}

impl Default for HotThreadConfig {
    fn default() -> Self {
        Self {
            hot_core: None,
            use_mmcss: true,
            mmcss_task: "Pro Audio".to_owned(),
            wait_mode: WaitMode::Blocking,
        }
    }
}

/// RAII guard holding the hot-thread MMCSS/affinity/priority policy for the calling thread's life.
///
/// MUST be bound to a named local for the whole hot loop. On `Drop` (same thread) it reverts the
/// thread priority, releases the MMCSS task handle, and restores the prior affinity mask. No-op
/// safe when nothing was applied (M1 stub).
#[must_use = "bind this guard for the hot thread's life; dropping it reverts the policy"]
pub struct HotPolicyGuard {
    /// MMCSS task handle (`AvSetMmThreadCharacteristics`), raw; `0` if not registered.
    mmcss_handle: usize,
    /// Previous affinity mask to restore, or `0` if affinity was not changed.
    prev_affinity: usize,
    /// Whether the thread priority was raised (and must be reverted).
    priority_raised: bool,
    /// Whether MMCSS registration succeeded (`AvRevertMmThreadCharacteristics` on drop).
    mmcss_active: bool,
}

impl HotPolicyGuard {
    /// Whether MMCSS registration is currently held.
    #[inline]
    pub fn mmcss_active(&self) -> bool {
        self.mmcss_active
    }
}

/// Apply the hot-thread policy described by `cfg` to the **current** (hot) thread.
///
/// Returns a [`HotPolicyGuard`] that reverts everything on `Drop`. Call it on the hot thread and
/// keep the guard bound for the loop's lifetime.
///
/// `TODO(hardware)`: set affinity (`SetThreadAffinityMask`, physical-core + sibling-avoid),
/// register MMCSS (`AvSetMmThreadCharacteristicsW(cfg.mmcss_task)` +
/// `AvSetMmThreadPriority(AVRT_PRIORITY_CRITICAL)`), else `SetThreadPriority(TIME_CRITICAL)`.
pub fn apply_hot_thread_policy(cfg: &HotThreadConfig) -> HotPolicyGuard {
    // Reference cfg so the final signature is honest under -D warnings until the body lands.
    let _ = (cfg.hot_core, cfg.use_mmcss, &cfg.mmcss_task, cfg.wait_mode);
    HotPolicyGuard {
        mmcss_handle: 0,
        prev_affinity: 0,
        priority_raised: false,
        mmcss_active: false,
    }
}

impl Drop for HotPolicyGuard {
    fn drop(&mut self) {
        // Each revert is independently guarded so a partially-applied policy unwinds cleanly.
        if self.mmcss_active {
            // TODO(hardware): AvRevertMmThreadCharacteristics(self.mmcss_handle).
            self.mmcss_active = false;
        }
        if self.priority_raised {
            // TODO(hardware): SetThreadPriority(GetCurrentThread(), prior).
            self.priority_raised = false;
        }
        if self.prev_affinity != 0 {
            // TODO(hardware): SetThreadAffinityMask(GetCurrentThread(), self.prev_affinity).
            self.prev_affinity = 0;
        }
        let _ = self.mmcss_handle;
    }
}
