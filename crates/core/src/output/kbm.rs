//! `KbmBatch` — the fixed-capacity keyboard/mouse event accumulator (blueprint §3.4).
//!
//! The mapping engine queues key/mouse edges into a `Copy`, allocation-free [`KbmBatch`] that
//! the hot loop pushes through a `rtrb<KbmBatch>` SPSC ring to the injector thread. The cap is
//! PINNED ([`KBM_BATCH_CAP`], verifier latency FIX 4) so the ring element and the hot-thread
//! push memcpy are bounded; macro expansion (unbounded) lives on the injector thread, not here.

/// How a key edge should be injected: by virtual-key code or by hardware scancode.
///
/// `ScanCode` (with extended-key handling) is the default for game compatibility; `Virtual`
/// sends the VK directly. The sink decides the final `SendInput` flags from this.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KeyKind {
    /// Inject via `KEYEVENTF_SCANCODE` (game-friendly default).
    ScanCode,
    /// Inject the virtual-key code directly.
    Virtual,
}

/// A mouse button for [`KbmEvent::MouseButton`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum MouseButton {
    Left,
    Right,
    Middle,
    /// Extra button 1 (back).
    X1,
    /// Extra button 2 (forward).
    X2,
}

/// One keyboard/mouse output event the injector thread will realize via `SendInput`.
///
/// Edges only: a held key emits a single `Key { down: true }` then nothing until release. Mouse
/// motion/wheel deltas are pre-accumulated per report (the integer delta is computed in
/// `apply()`); macro/special carry an id and are timed/routed off the hot thread.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KbmEvent {
    /// A key edge. `vk` is the virtual-key code; the scancode/extended flag is resolved in the
    /// sink from `kind`.
    Key { vk: u16, down: bool, kind: KeyKind },
    /// A mouse-button edge.
    MouseButton { btn: MouseButton, down: bool },
    /// A relative mouse move, accumulated from stick/gyro this report.
    MouseMove { dx: i32, dy: i32 },
    /// A wheel scroll in `WHEEL_DELTA` units (±120).
    Wheel { vertical: i32, horizontal: i32 },
    /// A macro start/stop edge; playback timing is owned by the injector thread.
    Macro { id: u16, start: bool },
    /// A special action id, routed to the control plane rather than the KBM injector.
    Special { id: u16 },
}

/// Pinned capacity of a [`KbmBatch`] (verifier latency FIX 4).
///
/// Sized for the worst plausible single report: 6 HID key edges + mouse buttons + a move + a
/// wheel + shift-layer churn.
pub const KBM_BATCH_CAP: usize = 24;

/// A fixed-capacity, `Copy`, allocation-free batch of [`KbmEvent`]s for one report (~200 B).
///
/// [`push`](Self::push) saturates (returns `false`, drops the event) at capacity instead of
/// panicking or allocating, so the hot path stays bounded.
#[derive(Clone, Copy, Debug)]
pub struct KbmBatch {
    buf: [KbmEvent; KBM_BATCH_CAP],
    len: u8,
}

impl KbmBatch {
    /// An empty batch.
    pub const fn new() -> Self {
        Self {
            // A harmless sentinel fill; only `buf[..len]` is ever read.
            buf: [KbmEvent::MouseMove { dx: 0, dy: 0 }; KBM_BATCH_CAP],
            len: 0,
        }
    }

    /// Reset the batch to empty (without touching the backing buffer).
    #[inline]
    pub fn clear(&mut self) {
        self.len = 0;
    }

    /// Append an event. Returns `false` (and drops it) if the batch is already full.
    #[inline]
    pub fn push(&mut self, e: KbmEvent) -> bool {
        let i = self.len as usize;
        if i >= KBM_BATCH_CAP {
            return false;
        }
        self.buf[i] = e;
        self.len += 1;
        true
    }

    /// The queued events in push order.
    #[inline]
    pub fn as_slice(&self) -> &[KbmEvent] {
        &self.buf[..self.len as usize]
    }

    /// Number of queued events.
    #[inline]
    pub fn len(&self) -> usize {
        self.len as usize
    }

    /// Whether no events are queued.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

impl Default for KbmBatch {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_is_empty() {
        let b = KbmBatch::new();
        assert!(b.is_empty());
        assert_eq!(b.len(), 0);
        assert_eq!(b.as_slice(), &[]);
    }

    #[test]
    fn push_appends_in_order() {
        let mut b = KbmBatch::new();
        let e0 = KbmEvent::Key {
            vk: 0x41,
            down: true,
            kind: KeyKind::ScanCode,
        };
        let e1 = KbmEvent::MouseButton {
            btn: MouseButton::Left,
            down: true,
        };
        assert!(b.push(e0));
        assert!(b.push(e1));
        assert_eq!(b.len(), 2);
        assert_eq!(b.as_slice(), &[e0, e1]);
        assert!(!b.is_empty());
    }

    #[test]
    fn clear_resets_len() {
        let mut b = KbmBatch::new();
        b.push(KbmEvent::Wheel {
            vertical: 120,
            horizontal: 0,
        });
        b.clear();
        assert!(b.is_empty());
        assert_eq!(b.as_slice(), &[]);
    }

    #[test]
    fn push_saturates_at_cap_without_panic() {
        let mut b = KbmBatch::new();
        let e = KbmEvent::MouseMove { dx: 1, dy: 1 };
        for _ in 0..KBM_BATCH_CAP {
            assert!(b.push(e), "fills up to cap");
        }
        assert_eq!(b.len(), KBM_BATCH_CAP);
        // Over-cap pushes are dropped, not panics/allocs.
        assert!(!b.push(e));
        assert!(!b.push(e));
        assert_eq!(b.len(), KBM_BATCH_CAP);
        assert_eq!(b.as_slice().len(), KBM_BATCH_CAP);
    }

    #[test]
    fn batch_is_copy_and_fits_the_pinned_cap() {
        // Compile-time-ish guard that KbmBatch is Copy (ring element requirement).
        fn assert_copy<T: Copy>() {}
        assert_copy::<KbmBatch>();
        let a = KbmBatch::new();
        let b = a; // moves-as-copy
        assert!(a.is_empty() && b.is_empty());
        assert_eq!(KBM_BATCH_CAP, 24);
    }
}
