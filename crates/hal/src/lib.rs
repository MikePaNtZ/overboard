//! `hal` is the seam between control-core and whatever is actually driving
//! the wheel: a MuJoCo sim (`sim-backend`, today) or real hardware (a future
//! `hw-backend`, TODO).
#![no_std]

use board_types::{Command, Observation};

/// A blocking, synchronous board I/O backend.
///
/// `cycle` is intentionally synchronous/blocking: the backend owns the
/// clock (real time for hardware, sim time for MuJoCo) and is responsible
/// for pacing. `control-core` never reads a clock itself — it derives dt
/// from the timestamps carried on consecutive [`Observation`]s.
pub trait BoardIo {
    /// Apply `cmd`, block until the next cycle's sensor data is ready, and
    /// return the resulting observation.
    fn cycle(&mut self, cmd: &Command) -> Observation;
}
