//! Hot → GUI telemetry: a `Copy` frame and a fixed-size, alloc-free p99 estimator.
//!
//! The hot loop publishes a [`TelemetryFrame`] through a `triple-buffer` every report (the
//! writer never blocks, the reader always gets a complete frame). Per `DESIGN.md` §6 nothing
//! on the hot thread may allocate or lock, so [`TelemetryFrame`] is `Copy` with no owned heap
//! data and the latency percentile is computed by a fixed-bucket histogram
//! ([`LatencyReservoir`]) whose storage never grows.

/// One snapshot of hot-loop state, published every report. `Copy`, no heap, no `Drop` work.
///
/// Stick fields are the canonical `[-1,1]` axis values pre/post filter (`f32` is ample for a
/// scope display and halves the frame size); counters are cumulative.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TelemetryFrame {
    /// Wall-clock spent in the loop body for the last report (parse → filter → submit), ns.
    pub loop_busy_ns: u64,
    /// Guarded report interval for the last report, microseconds.
    pub dt_us: f32,
    /// Cumulative dropped reports (seq gaps).
    pub dropped: u32,
    /// Cumulative duplicate reports (seq repeats).
    pub duplicates: u32,
    /// Pre-filter left stick X/Y (canonical `[-1,1]`).
    pub in_lx: f32,
    pub in_ly: f32,
    /// Pre-filter right stick X/Y.
    pub in_rx: f32,
    pub in_ry: f32,
    /// Post-filter left stick X/Y (what is mapped to the virtual pad).
    pub out_lx: f32,
    pub out_ly: f32,
    /// Post-filter right stick X/Y.
    pub out_rx: f32,
    pub out_ry: f32,
}

/// Number of fixed histogram buckets. Bucket `i` covers `[i, i+1)` microseconds of loop-busy
/// time except the last, which is saturating (`>= NUM_BUCKETS-1 µs`). At 8 kHz the entire
/// loop body must finish in ~125 µs, so 256 one-microsecond buckets resolve the whole regime
/// of interest with a tiny, fixed footprint.
const NUM_BUCKETS: usize = 256;

/// A fixed-bucket histogram that estimates the p99 (and other percentiles) of a stream of
/// microsecond-scale durations with **zero heap growth** — suitable for the hot thread.
///
/// Each sample increments one of [`NUM_BUCKETS`] counters; percentiles are read off the
/// cumulative distribution. The estimate is quantized to the bucket width (1 µs) and the top
/// bucket saturates, both acceptable for "is the loop staying under budget" telemetry.
#[derive(Clone, Debug)]
pub struct LatencyReservoir {
    buckets: [u32; NUM_BUCKETS],
    count: u64,
}

impl LatencyReservoir {
    /// A fresh, empty reservoir.
    pub fn new() -> Self {
        Self {
            buckets: [0; NUM_BUCKETS],
            count: 0,
        }
    }

    /// Record one duration (nanoseconds). Bucketed by truncated microseconds; the top bucket
    /// saturates. Never allocates and never overflows (counters saturate).
    #[inline]
    pub fn record_ns(&mut self, ns: u64) {
        let us = (ns / 1_000) as usize;
        let idx = us.min(NUM_BUCKETS - 1);
        self.buckets[idx] = self.buckets[idx].saturating_add(1);
        self.count = self.count.saturating_add(1);
    }

    /// Number of recorded samples.
    #[inline]
    pub fn len(&self) -> u64 {
        self.count
    }

    /// Whether no samples have been recorded.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Estimate the value at percentile `p` (`0.0..=1.0`), in **microseconds**.
    ///
    /// Returns the lower edge of the bucket at which the cumulative fraction first reaches
    /// `p`. Empty reservoir → `0`. `p` is clamped to `[0,1]`.
    pub fn percentile_us(&self, p: f64) -> u32 {
        if self.count == 0 {
            return 0;
        }
        let p = p.clamp(0.0, 1.0);
        // Rank of the target sample (1-based), ceil so e.g. p=0.99 of 100 samples is the 99th.
        let target = (p * self.count as f64).ceil() as u64;
        let target = target.max(1);
        let mut cumulative: u64 = 0;
        for (i, &c) in self.buckets.iter().enumerate() {
            cumulative += c as u64;
            if cumulative >= target {
                return i as u32;
            }
        }
        (NUM_BUCKETS - 1) as u32
    }

    /// Convenience p99 in microseconds.
    #[inline]
    pub fn p99_us(&self) -> u32 {
        self.percentile_us(0.99)
    }

    /// Clear all counts back to empty (no deallocation).
    pub fn clear(&mut self) {
        self.buckets = [0; NUM_BUCKETS];
        self.count = 0;
    }
}

impl Default for LatencyReservoir {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_is_copy_and_default_zeroed() {
        let f = TelemetryFrame::default();
        let g = f; // Copy, not move.
        assert_eq!(f, g);
        assert_eq!(f.dropped, 0);
        assert_eq!(f.loop_busy_ns, 0);
    }

    #[test]
    fn empty_reservoir() {
        let r = LatencyReservoir::new();
        assert!(r.is_empty());
        assert_eq!(r.len(), 0);
        assert_eq!(r.p99_us(), 0);
    }

    #[test]
    fn p99_of_uniform_stream() {
        let mut r = LatencyReservoir::new();
        // 99 samples at 10 us, 1 sample at 200 us: p99 is still the dense 10 us bucket,
        // p100 reaches the tail.
        for _ in 0..99 {
            r.record_ns(10_000);
        }
        r.record_ns(200_000);
        assert_eq!(r.len(), 100);
        assert_eq!(r.percentile_us(0.50), 10);
        assert_eq!(r.p99_us(), 10);
        assert_eq!(r.percentile_us(1.0), 200);
    }

    #[test]
    fn top_bucket_saturates() {
        let mut r = LatencyReservoir::new();
        r.record_ns(10_000_000); // 10 ms -> way past the last bucket
        assert_eq!(r.percentile_us(1.0), (NUM_BUCKETS - 1) as u32);
    }

    #[test]
    fn clear_resets() {
        let mut r = LatencyReservoir::new();
        r.record_ns(5_000);
        assert_eq!(r.len(), 1);
        r.clear();
        assert!(r.is_empty());
        assert_eq!(r.p99_us(), 0);
    }
}
