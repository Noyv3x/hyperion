//! Dropped/duplicate report detection from the device frame counter.
//!
//! DualSense reports carry a frame counter (DS4-compatible report byte 7, bits 2..8 — a
//! 6-bit field the C# reads as `inputReport[7] >> 2`). The engine plumbs that as a `u8`
//! `seq`; this tracker turns consecutive `seq` values into a dropped count and a duplicate
//! flag. Arithmetic is modular over the counter's range so wrap is handled naturally.

/// Tracks the previous frame-counter value to derive dropped/duplicate counts.
#[derive(Clone, Copy, Debug, Default)]
pub struct SeqTracker {
    prev: Option<u8>,
}

impl SeqTracker {
    /// Fold the next `seq` value, returning `(dropped, is_duplicate)`.
    ///
    /// * `dropped` = number of reports skipped between the previous and current `seq`,
    ///   `(seq - prev - 1) mod 256`; `0` on the first report.
    /// * `is_duplicate` = the counter did not advance (`seq == prev`).
    pub fn update(&mut self, seq: u8) -> (u16, bool) {
        let (dropped, is_duplicate) = match self.prev {
            Some(p) => (seq.wrapping_sub(p).wrapping_sub(1) as u16, p == seq),
            None => (0, false),
        };
        self.prev = Some(seq);
        (dropped, is_duplicate)
    }

    /// Forget history so the next [`update`](Self::update) primes (returns `(0, false)`).
    pub fn reset(&mut self) {
        self.prev = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_report_primes() {
        let mut t = SeqTracker::default();
        assert_eq!(t.update(42), (0, false));
    }

    #[test]
    fn consecutive_no_drop() {
        let mut t = SeqTracker::default();
        t.update(10);
        assert_eq!(t.update(11), (0, false));
    }

    #[test]
    fn one_dropped() {
        let mut t = SeqTracker::default();
        t.update(10);
        assert_eq!(t.update(12), (1, false));
    }

    #[test]
    fn duplicate_flagged() {
        let mut t = SeqTracker::default();
        t.update(10);
        assert_eq!(t.update(10), (255, true)); // (10-10-1) mod 256 = 255, dup=true
    }

    #[test]
    fn wraps_mod_256() {
        let mut t = SeqTracker::default();
        t.update(254);
        // 0 after 254: (0 - 254 - 1) mod 256 = 1 dropped (only 255 was skipped).
        assert_eq!(t.update(0), (1, false));
        // and a clean increment across the wrap from 255 -> 0:
        let mut t2 = SeqTracker::default();
        t2.update(255);
        assert_eq!(t2.update(0), (0, false));
    }

    #[test]
    fn reset_reprimes() {
        let mut t = SeqTracker::default();
        t.update(10);
        t.reset();
        assert_eq!(t.update(200), (0, false));
    }
}
