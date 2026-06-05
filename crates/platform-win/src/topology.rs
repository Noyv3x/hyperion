//! Processor-topology probe for hot-thread core selection (DESIGN §6).
//!
//! [`auto_select_core`] queries the OS for the physical-core layout and returns the logical
//! processor index of a **physical** core that does *not* share an SMT sibling with logical CPU 0
//! (the core the GUI / main thread runs on). Pinning the HybridSpin hot loop there keeps a busy
//! core from starving the UI's hyper-thread sibling.
//!
//! ## Why this matters
//!
//! On an SMT (Hyper-Threading / "2 logical per core") machine, two logical processors share one
//! physical core's execution resources. A 100 %-busy spin on logical CPU `k` robs cycles from the
//! *other* logical CPU on the same physical core. Logical CPU 0 is where Windows starts the
//! process / pumps the GUI message loop, so we deliberately avoid core 0 **and its sibling** and
//! hand back a clean physical core's first logical index.
//!
//! ## How it works
//!
//! `GetLogicalProcessorInformationEx(RelationProcessorCore, …)` returns one variable-length record
//! per *physical* core. Each record's `PROCESSOR_RELATIONSHIP.GroupMask[0].Mask` is a bitmask of the
//! logical processors that make up that physical core: a single bit ⇒ no SMT, two-or-more bits ⇒ an
//! SMT core whose set bits are siblings of one another. We:
//!
//! 1. enumerate the physical cores (walking the buffer by each record's `Size`, since records are
//!    variable-length),
//! 2. find the physical core that owns logical CPU 0 (the GUI core),
//! 3. pick the **first** physical core that shares no logical CPU with the GUI core, and
//! 4. return that core's lowest logical-processor index.
//!
//! Any failure (query error, no SMT data, single-core box where the only core is the GUI core) maps
//! to `None`, i.e. "leave the hot thread unpinned" — the exact behaviour before this module existed.
//!
//! We only consider processor **group 0**. Hyperion targets a single hot thread on consumer
//! hardware (≤ 64 logical CPUs, one group); `SetThreadAffinityMask` likewise operates within the
//! calling thread's group, so a group-0 logical index is the correct unit to return.

use windows::Win32::Foundation::ERROR_INSUFFICIENT_BUFFER;
use windows::Win32::System::SystemInformation::{
    GetLogicalProcessorInformationEx, RelationProcessorCore, LOGICAL_PROCESSOR_RELATIONSHIP,
    PROCESSOR_RELATIONSHIP, SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX,
};

/// Pick a physical core's first logical-processor index to pin the hot thread to, avoiding the
/// SMT sibling of logical CPU 0 (the GUI / main-thread core).
///
/// Returns:
/// * `Some(idx)` — the lowest logical-processor index of a physical core that shares no logical CPU
///   with the core hosting logical CPU 0. Pin the hot thread here.
/// * `None` — the topology query failed, returned no usable data, or the machine has only the GUI
///   core to offer. The caller must then leave the thread unpinned (pre-M7 behaviour).
///
/// Never panics. Performs a single OS query; not for the hot path (call once at thread start-up).
#[must_use]
pub fn auto_select_core() -> Option<usize> {
    let cores = physical_core_masks()?;
    select_core_from_masks(&cores)
}

/// Query the OS for one affinity mask per **physical** core (group 0), as a `Vec<usize>`.
///
/// Returns `None` if the query fails or yields no `RelationProcessorCore` records. Each returned
/// `usize` has one bit set per logical processor belonging to that physical core; ≥ 2 bits ⇒ SMT.
fn physical_core_masks() -> Option<Vec<usize>> {
    let (buf, written) = query_processor_core_buffer()?;
    Some(parse_core_masks(&buf, written))
}

/// Run the two-call `GetLogicalProcessorInformationEx(RelationProcessorCore)` sizing dance and
/// return the raw record buffer (aligned for `SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX`) together
/// with the number of bytes the API actually wrote.
///
/// The returned `Vec` is in over-aligned units of the record type so the buffer base is correctly
/// aligned for the variable-length records the API writes into it; the buffer may be padded past
/// the written length, so callers must respect the returned byte count. `None` on any failure.
fn query_processor_core_buffer() -> Option<(Vec<SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX>, usize)> {
    // First call: pass a null buffer to learn the required byte length. The `windows` binding maps
    // the failing `BOOL` to `Err`; the expected error here is `ERROR_INSUFFICIENT_BUFFER`.
    let mut len: u32 = 0;
    // SAFETY: `relationshiptype` is the documented `RelationProcessorCore` value; `buffer` is `None`
    // (a null pointer, which this sizing call requires); `&mut len` is a valid out-pointer that
    // outlives the call. The call only writes the needed byte length through `len`.
    let probe = unsafe { GetLogicalProcessorInformationEx(RelationProcessorCore, None, &mut len) };
    match probe {
        // A real machine never returns `Ok` from a zero-length query, but treat it defensively:
        // no length means no data.
        Ok(()) => return None,
        Err(e) if e.code() == ERROR_INSUFFICIENT_BUFFER.to_hresult() => {}
        // Any other error (or unexpectedly small length) ⇒ give up, leave the thread unpinned.
        Err(_) => return None,
    }
    if len == 0 {
        return None;
    }

    // Allocate a buffer of `SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX` so its base is correctly
    // aligned for the records the API writes; round the reported byte length up to whole elements.
    let elem = core::mem::size_of::<SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX>();
    if elem == 0 {
        return None;
    }
    let elems = (len as usize).div_ceil(elem);
    let mut buf: Vec<SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX> = vec![Default::default(); elems];

    // Second call: hand over the aligned buffer and its capacity in bytes; `cap` is updated in-place
    // to the bytes actually written.
    let mut cap: u32 = (elems * elem) as u32;
    // SAFETY: `buf` holds `elems` correctly-aligned records (≥ `len` bytes) and outlives the call;
    // `buf.as_mut_ptr()` is the non-null base the API fills. `cap` is the buffer's byte capacity in
    // a valid in/out `*mut u32`. The callee writes at most `cap` bytes and updates `cap` to the
    // bytes actually written. We never read past the reported length below.
    let result = unsafe {
        GetLogicalProcessorInformationEx(RelationProcessorCore, Some(buf.as_mut_ptr()), &mut cap)
    };
    result.ok()?;

    // `cap` now holds the bytes actually written; clamp it to the buffer's real byte size so the
    // parser never walks the (zero-padded) tail beyond what the OS filled in.
    let written = (cap as usize).min(elems * elem);
    Some((buf, written))
}

/// Walk the variable-length record buffer and collect each `RelationProcessorCore` record's group-0
/// affinity mask. Records are read by their self-reported `Size`, never `sizeof`.
///
/// `GetLogicalProcessorInformationEx` packs records back-to-back, each only `Size` bytes wide — and
/// a `RelationProcessorCore` record's `Size` (header + one `PROCESSOR_RELATIONSHIP`) is *smaller*
/// than `size_of::<SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX>()` (whose union is dominated by the
/// larger `GROUP_RELATIONSHIP` arm). So we never bound a record by the full struct size: we require
/// only the fixed header to read `Size`, then the processor arm to read its mask, validating both
/// against `Size` and the buffer end.
fn parse_core_masks(buf: &[SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX], written: usize) -> Vec<usize> {
    /// Bytes of the fixed `{ Relationship, Size }` header common to every record.
    const HEADER: usize = core::mem::offset_of!(SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX, Anonymous);
    /// Bytes needed to also read the `Processor` (`PROCESSOR_RELATIONSHIP`) union arm in full.
    const PROC_RECORD: usize = HEADER + core::mem::size_of::<PROCESSOR_RELATIONSHIP>();

    let mut masks = Vec::new();
    let base = buf.as_ptr().cast::<u8>();
    // Walk only the bytes the OS actually wrote; the buffer may be zero-padded past `written`.
    let total = written.min(core::mem::size_of_val(buf));
    let mut offset: usize = 0;

    // Advance by each record's `Size`. Continue while the fixed header still fits; every deeper read
    // is bounds-checked below so a truncated / malformed tail cannot read out of bounds.
    while offset + HEADER <= total {
        // SAFETY: `offset + HEADER <= total` (loop condition) guarantees the `Relationship`/`Size`
        // header is fully inside the live, zero-initialised `buf`. We read the two POD header fields
        // through raw pointers (not by forming a `&` to the whole, possibly-larger record), so we
        // never require more than `HEADER` valid bytes here. `read_unaligned` is used because `offset`
        // advances by an arbitrary record `Size`, so a later record's base need not be 8-aligned.
        let relationship = unsafe {
            base.add(offset)
                .cast::<LOGICAL_PROCESSOR_RELATIONSHIP>()
                .read_unaligned()
        };
        // SAFETY: `Size` lives within the `HEADER` bytes already proven in-bounds; read it as a `u32`
        // at its field offset via a byte pointer, unaligned for the same reason as above.
        let size = unsafe {
            base.add(offset + core::mem::offset_of!(SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX, Size))
                .cast::<u32>()
                .read_unaligned()
        } as usize;

        // A `Size` that cannot hold the header (or overruns the buffer) means a malformed / truncated
        // tail — stop rather than spin (zero `Size`) or read past the end.
        if size < HEADER || offset + size > total {
            break;
        }

        if relationship == RelationProcessorCore && size >= PROC_RECORD {
            // SAFETY: `size >= PROC_RECORD` and `offset + size <= total` together guarantee a full
            // `PROCESSOR_RELATIONSHIP` lies within `buf` at the union offset; for a
            // `RelationProcessorCore` record the active arm *is* `Processor` (the documented
            // contract). `read_unaligned` copies the POD record out without assuming alignment (the
            // record base may be unaligned since `offset` advances by arbitrary `Size`s).
            let proc: PROCESSOR_RELATIONSHIP = unsafe {
                base.add(offset + HEADER)
                    .cast::<PROCESSOR_RELATIONSHIP>()
                    .read_unaligned()
            };
            // One physical core carries a single `GroupMask` entry; take group 0's mask. (Hyperion
            // is single-group; cross-group cores are out of scope and simply ignored.)
            if proc.GroupCount >= 1 {
                let affinity = proc.GroupMask[0];
                if affinity.Group == 0 {
                    masks.push(affinity.Mask);
                }
            }
        }

        offset += size;
    }

    masks
}

/// Pure selection over per-physical-core masks (no OS calls — unit-testable on any platform).
///
/// Given one affinity mask per physical core, return the lowest logical index of the first physical
/// core that shares no bit with the core owning logical CPU 0. `None` when there is no such core
/// (empty input, or every core overlaps the GUI core — e.g. a single physical core).
fn select_core_from_masks(masks: &[usize]) -> Option<usize> {
    // The GUI core is whichever physical core contains logical CPU 0 (bit 0). Its set bits are the
    // SMT siblings we must avoid. If no core claims bit 0 (unexpected), fall back to "avoid nothing"
    // so we still return a usable core.
    let gui_mask = masks.iter().copied().find(|&m| (m & 1) != 0).unwrap_or(0);

    masks
        .iter()
        .copied()
        // Skip empty masks and any core sharing a logical CPU with the GUI core (its own entry, and
        // on non-SMT hardware that is just core 0 itself).
        .filter(|&m| m != 0 && (m & gui_mask) == 0)
        // Prefer the lowest physical core, and within it its lowest logical index.
        .min_by_key(|&m| m.trailing_zeros())
        .map(|m| m.trailing_zeros() as usize)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Smoke test: `auto_select_core()` must not panic and returns `Some`/`None` either way. We do
    /// NOT assert on content — the CI runner's CPU topology is arbitrary (could be a single-core VM,
    /// a non-SMT box, or a many-core SMT host), so any concrete index would be flaky. This only
    /// exercises the unsafe query path for soundness ("does it run without UB / panic").
    #[test]
    fn auto_select_core_does_not_panic() {
        let _ = auto_select_core();
    }

    #[test]
    fn empty_topology_selects_nothing() {
        assert_eq!(select_core_from_masks(&[]), None);
    }

    #[test]
    fn single_smt_core_has_no_clean_sibling() {
        // One physical SMT core = logical 0 and 1. Nothing left for the hot thread.
        assert_eq!(select_core_from_masks(&[0b11]), None);
    }

    #[test]
    fn single_non_smt_core_has_no_clean_sibling() {
        // Only logical CPU 0 exists; nowhere to move the hot thread.
        assert_eq!(select_core_from_masks(&[0b1]), None);
    }

    #[test]
    fn smt_machine_returns_first_clean_physical_core() {
        // 4 physical SMT cores: {0,1}, {2,3}, {4,5}, {6,7}. The GUI core is {0,1}; the first clean
        // physical core is {2,3}, whose first logical index is 2.
        let masks = [0b0000_0011, 0b0000_1100, 0b0011_0000, 0b1100_0000];
        assert_eq!(select_core_from_masks(&masks), Some(2));
    }

    #[test]
    fn non_smt_machine_returns_first_non_gui_core() {
        // 4 single-thread cores: logical 0,1,2,3. GUI is core 0; first clean core is logical 1.
        let masks = [0b0001, 0b0010, 0b0100, 0b1000];
        assert_eq!(select_core_from_masks(&masks), Some(1));
    }

    #[test]
    fn picks_lowest_logical_index_regardless_of_record_order() {
        // Same cores as the SMT case but reported out of order; we must still pick the {2,3} core
        // (lowest logical index 2), not whichever record happens to come first.
        let masks = [0b1100_0000, 0b0011_0000, 0b0000_0011, 0b0000_1100];
        assert_eq!(select_core_from_masks(&masks), Some(2));
    }

    #[test]
    fn gui_core_without_bit_zero_avoids_nothing() {
        // Degenerate: no core owns logical CPU 0. `gui_mask` is 0, so the first non-empty core wins.
        let masks = [0b0100, 0b1000];
        assert_eq!(select_core_from_masks(&masks), Some(2));
    }
}
