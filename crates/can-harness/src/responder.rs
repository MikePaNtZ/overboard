//! A simulated VESC-shaped responder: answers a command frame with a status
//! frame on a background thread, the way a real controller would, so the
//! stack above the transport (timeouts, decode, retry) can be tested without
//! one (issue #52 AC2).
//!
//! Arbitration IDs are chosen by the caller and are test-local placeholders,
//! not real VESC packet IDs -- see `crate` docs for why.

use crate::{from_can_frame, to_can_frame};
use socketcan::{CanFrame, ShouldRetry, Socket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::Duration;
use vesc_wire::RawFrame;

/// How often the background thread checks for shutdown between reads. Short
/// enough that dropping a [`SimResponder`] at the end of a test doesn't
/// stall it, long enough not to busy-loop.
const POLL_INTERVAL: Duration = Duration::from_millis(50);

/// Runs on a background thread until dropped: reads frames from a vcan
/// interface and, for each one, writes back whatever `respond` returns on
/// the same bus. Returning `None` from `respond` simulates silence --
/// callers building the "responder never answers" case (AC4) just always
/// return `None`, no separate variant needed.
pub struct SimResponder {
    stop: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl SimResponder {
    /// Spawns the responder against an already-up vcan interface named
    /// `ifname`.
    pub fn spawn(
        ifname: &str,
        respond: impl Fn(RawFrame) -> Option<RawFrame> + Send + 'static,
    ) -> std::io::Result<Self> {
        let socket = socketcan::CanSocket::open(ifname)?;
        socket.set_read_timeout(POLL_INTERVAL)?;

        let stop = Arc::new(AtomicBool::new(false));
        let stop_thread = stop.clone();

        let handle = thread::spawn(move || {
            while !stop_thread.load(Ordering::Relaxed) {
                let frame = match socket.read_frame() {
                    Ok(CanFrame::Data(frame)) => frame,
                    Ok(_) => continue,
                    Err(e) if e.should_retry() => continue,
                    Err(_) => continue,
                };
                let Some(reply) = respond(from_can_frame(&frame)) else {
                    continue;
                };
                if let Some(reply_frame) = to_can_frame(reply) {
                    let _ = socket.write_frame(&reply_frame);
                }
            }
        });

        Ok(Self {
            stop,
            handle: Some(handle),
        })
    }
}

impl Drop for SimResponder {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}
