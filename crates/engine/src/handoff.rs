//! Lock-free handoffs between the hot thread and the control plane (`DESIGN.md` §6).
//!
//! Three primitives, each chosen for a specific direction and never a `Mutex`:
//!
//! * **Config (GUI/supervisor → HOT):** [`ConfigHandle`] = `Arc<ArcSwap<EngineConfig>>`.
//!   The hot loop `load()`s wait-free once per report; the config store `store()`s a whole new
//!   immutable snapshot. `ArcSwap` is MPMC-safe, so GUI + supervisor + file-watch may all
//!   publish through the single-writer [`crate::config_store::ConfigStore`].
//! * **Telemetry (HOT → GUI):** [`TelemetryTx`] / [`TelemetryRx`], a `triple-buffer` of a
//!   `Copy` [`TelemetryFrame`] — the writer never blocks, the reader always sees a full frame.
//! * **Commands (GUI/supervisor → HOT):** **two** SPSC queues. `rtrb::Producer` is *not*
//!   `Sync`, so a single shared producer across two threads is unsound (resolved conflict #6).
//!   Each control thread owns its own [`CommandTx`]; the hot loop drains both halves of
//!   [`CommandRx`] every report.

use std::sync::Arc;

use arc_swap::ArcSwap;
use hyperion_core::config::EngineConfig;

use crate::telemetry::TelemetryFrame;

/// Capacity of each SPSC command queue. Commands are rare (user actions / lifecycle events),
/// so a small fixed ring is plenty; a full queue means the hot loop is wedged, which other
/// telemetry would already surface.
pub const COMMAND_QUEUE_CAP: usize = 64;

/// Wait-free config snapshot shared GUI/supervisor → HOT.
///
/// Publishers `store()` a fresh `Arc<EngineConfig>`; the hot loop `load()`s it once per report
/// and only re-applies fields when a generation counter changes (see
/// [`crate::config_store`]). Keep `EngineConfig`'s `Drop` trivial/alloc-free: the freed old
/// `Arc` may be dropped on the hot thread.
pub type ConfigHandle = Arc<ArcSwap<EngineConfig>>;

/// Hot-side telemetry publisher (writer never blocks).
pub struct TelemetryTx(pub triple_buffer::Input<TelemetryFrame>);

/// GUI-side telemetry reader (always reads a complete, latest frame).
pub struct TelemetryRx(pub triple_buffer::Output<TelemetryFrame>);

/// A command for the hot thread. Sent by the GUI or the supervisor; drained by the hot loop.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HotCommand {
    /// Re-prime the stick filter state (mirrors C# `ResetRCFilter`): next report primes.
    ResetFilter,
    /// Re-run device calibration / re-prime the dt clock after a stall or replug.
    Recalibrate,
    /// Tear down and re-create the virtual target (ViGEm replug).
    ReplugTarget,
    /// Stop the hot loop and exit.
    Shutdown,
}

/// One end of a single-producer command queue to the hot thread. Exactly one owner per
/// instance — `build_links` hands out one to the GUI and one to the supervisor.
pub struct CommandTx(pub rtrb::Producer<HotCommand>);

impl CommandTx {
    /// Enqueue a command. Returns `Err(cmd)` if the queue is full (the hot loop is wedged);
    /// callers may retry or surface the back-pressure. Never blocks, never allocates.
    #[inline]
    pub fn send(&mut self, cmd: HotCommand) -> Result<(), HotCommand> {
        self.0.push(cmd).map_err(|rtrb::PushError::Full(c)| c)
    }
}

/// The hot thread's receiving end: the GUI queue and the supervisor queue. The hot loop
/// `try_pop`s **both** every report (resolved conflict #6: two SPSC queues, not one shared
/// producer).
pub struct CommandRx {
    pub gui: rtrb::Consumer<HotCommand>,
    pub sup: rtrb::Consumer<HotCommand>,
}

impl CommandRx {
    /// Pop the next pending command from either queue (GUI first, then supervisor), or `None`
    /// if both are empty. Non-blocking, alloc-free; call in a loop to fully drain.
    #[inline]
    pub fn try_pop(&mut self) -> Option<HotCommand> {
        self.gui.pop().ok().or_else(|| self.sup.pop().ok())
    }
}

/// Build all hot-path handoff links from an initial config snapshot.
///
/// Returns:
/// * the shared [`ConfigHandle`] (clone for each publisher; the hot loop holds one),
/// * the telemetry pair `(TelemetryTx /*hot*/, TelemetryRx /*gui*/)`,
/// * the command triple `(CommandTx /*gui*/, CommandTx /*supervisor*/, CommandRx /*hot*/)`.
pub fn build_links(
    cfg: EngineConfig,
) -> (
    ConfigHandle,
    (TelemetryTx, TelemetryRx),
    (CommandTx, CommandTx, CommandRx),
) {
    let config = Arc::new(ArcSwap::from_pointee(cfg));

    let (tele_in, tele_out) = triple_buffer::triple_buffer(&TelemetryFrame::default());
    let telemetry = (TelemetryTx(tele_in), TelemetryRx(tele_out));

    let (gui_tx, gui_rx) = rtrb::RingBuffer::new(COMMAND_QUEUE_CAP);
    let (sup_tx, sup_rx) = rtrb::RingBuffer::new(COMMAND_QUEUE_CAP);
    let commands = (
        CommandTx(gui_tx),
        CommandTx(sup_tx),
        CommandRx {
            gui: gui_rx,
            sup: sup_rx,
        },
    );

    (config, telemetry, commands)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn two_producers_both_drain_on_one_consumer() {
        let cfg = EngineConfig::default();
        let (_config, _telemetry, (mut gui_tx, mut sup_tx, mut rx)) = build_links(cfg);

        // GUI and supervisor each push through their OWN producer (the soundness fix: no
        // shared rtrb::Producer across threads).
        gui_tx.send(HotCommand::ResetFilter).unwrap();
        sup_tx.send(HotCommand::Shutdown).unwrap();
        gui_tx.send(HotCommand::Recalibrate).unwrap();

        // The hot side drains both queues. GUI is checked first each call.
        let mut drained = Vec::new();
        while let Some(c) = rx.try_pop() {
            drained.push(c);
        }
        assert!(drained.contains(&HotCommand::ResetFilter));
        assert!(drained.contains(&HotCommand::Recalibrate));
        assert!(drained.contains(&HotCommand::Shutdown));
        assert_eq!(drained.len(), 3);
        assert!(rx.try_pop().is_none());
    }

    #[test]
    fn full_queue_returns_command_without_panicking() {
        let cfg = EngineConfig::default();
        let (_config, _telemetry, (mut gui_tx, _sup_tx, _rx)) = build_links(cfg);
        // Fill the GUI queue until it reports back-pressure (capacity is at least one).
        let mut accepted = 0usize;
        loop {
            match gui_tx.send(HotCommand::ResetFilter) {
                Ok(()) => accepted += 1,
                Err(c) => {
                    // Full: the command is handed back intact, no panic, no alloc.
                    assert_eq!(c, HotCommand::ResetFilter);
                    break;
                }
            }
        }
        assert!(accepted >= 1, "queue must accept at least one command");
        assert!(
            accepted <= COMMAND_QUEUE_CAP,
            "queue never exceeds its configured capacity"
        );
    }

    #[test]
    fn arcswap_load_then_store_observed_by_reader() {
        let cfg = EngineConfig::default();
        let (config, _telemetry, _commands) = build_links(cfg);

        // A wait-free "hot side" load sees the initial snapshot; clone it to a fresh `Arc`
        // (pointer identity differs from the live one).
        let baseline = Arc::new((*config.load_full()).clone());

        // A "publisher" stores that fresh whole snapshot.
        config.store(baseline.clone());

        // The next load observes the exact pointer that was stored — no `PartialEq` needed on
        // `EngineConfig`, so this test stays decoupled from its derives.
        let observed = config.load_full();
        assert!(Arc::ptr_eq(&observed, &baseline));
    }

    #[test]
    fn telemetry_write_read_round_trip() {
        let cfg = EngineConfig::default();
        let (_config, (mut tx, mut rx), _commands) = build_links(cfg);
        let frame = TelemetryFrame {
            dropped: 7,
            dt_us: 250.0,
            ..TelemetryFrame::default()
        };
        tx.0.write(frame);
        let got = *rx.0.read();
        assert_eq!(got.dropped, 7);
        assert_eq!(got.dt_us, 250.0);
    }
}
