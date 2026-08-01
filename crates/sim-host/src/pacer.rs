//! Fixed-rate pacing against a monotonic clock, absolute deadlines only.
//!
//! Issue #161 is explicit about the failure mode this avoids: pacing with
//! `sleep(period)` on every iteration accumulates drift, because the time
//! spent doing the tick's own work (however small) is never subtracted back
//! out -- a loop that sleeps `period` after each tick runs slower than
//! `period` on average, and the error compounds every cycle. Advancing a
//! single absolute deadline (`next_deadline += period`) instead means each
//! cycle is paced against where the clock SHOULD be, not against how long
//! the previous cycle happened to take.

use std::time::{Duration, Instant};

/// Paces a loop at a fixed period. Call [`Pacer::wait_for_next`] once per
/// iteration, after that iteration's work is done; sleep for the returned
/// duration (zero means the deadline already passed -- do not sleep, and the
/// miss has already been counted).
///
/// Takes `now` as an explicit argument rather than calling `Instant::now()`
/// itself, so the pacing arithmetic is testable without racing a real clock
/// against a real `thread::sleep` on a shared, possibly loaded CI runner --
/// see this module's tests, which never sleep at all.
pub struct Pacer {
    period: Duration,
    next_deadline: Instant,
    missed_deadlines: u64,
}

impl Pacer {
    /// Starts the clock at `now`: the first `wait_for_next()` targets
    /// `now + period`, not some later first call's `now`.
    pub fn new(period: Duration, now: Instant) -> Self {
        Pacer {
            period,
            next_deadline: now + period,
            missed_deadlines: 0,
        }
    }

    /// Total missed deadlines observed so far.
    pub fn missed_deadlines(&self) -> u64 {
        self.missed_deadlines
    }

    /// How far past `period` this call is deliberately allowed to lag
    /// before it counts as this crate's target loop rate, `period`, itself
    /// -- there is no slack constant here on purpose: ANY overrun of the
    /// absolute deadline is a miss, however small (issue #161: "do NOT
    /// silently swallow them").
    pub fn wait_for_next(&mut self, now: Instant) -> Duration {
        let sleep_for = if self.next_deadline > now {
            self.next_deadline - now
        } else {
            self.missed_deadlines += 1;
            Duration::ZERO
        };
        // Advances by exactly one period regardless of the overrun, so a
        // single slow cycle costs one miss, not a cascading pile-up of
        // catch-up iterations trying to make up lost ground (which would be
        // physically meaningless here: one loop iteration is one fixed
        // sim-time step, not a variable one).
        self.next_deadline += self.period;
        sleep_for
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Deterministic: every `now` is derived from one fixed base Instant by
    // pure Duration arithmetic, never from a real sleep -- so these tests
    // cannot flake under CI scheduler load the way a real-clock version did.
    fn base() -> Instant {
        Instant::now()
    }

    #[test]
    fn a_fast_loop_never_misses() {
        let t0 = base();
        let period = Duration::from_millis(20);
        let mut p = Pacer::new(period, t0);
        // Each iteration's "work" costs a negligible 1 us, well inside the
        // period -- the deadline is always still ahead.
        let mut now = t0;
        for _ in 0..5 {
            now += Duration::from_micros(1);
            let sleep_for = p.wait_for_next(now);
            assert!(!sleep_for.is_zero());
            now += sleep_for;
        }
        assert_eq!(p.missed_deadlines(), 0);
    }

    #[test]
    fn overshooting_the_deadline_counts_a_miss_and_returns_zero() {
        let t0 = base();
        let period = Duration::from_millis(10);
        let mut p = Pacer::new(period, t0);
        // Blown straight through the first deadline.
        let late = t0 + period + Duration::from_millis(5);
        let sleep_for = p.wait_for_next(late);
        assert!(sleep_for.is_zero(), "already late -- must not sleep more");
        assert_eq!(p.missed_deadlines(), 1);
    }

    #[test]
    fn a_small_overshoot_does_not_cost_the_next_iteration_too() {
        let t0 = base();
        let period = Duration::from_millis(10);
        let mut p = Pacer::new(period, t0);
        // Overshoot the first deadline by less than one whole period, so
        // once wait_for_next() advances next_deadline by exactly one period,
        // that new deadline lands back in the future.
        let late = t0 + period + Duration::from_millis(2);
        let sleep_for = p.wait_for_next(late);
        assert!(sleep_for.is_zero());
        assert_eq!(p.missed_deadlines(), 1);

        // Next call, at the same instant (zero elapsed "work") -- the
        // deadline it now targets is back in the future.
        let sleep_for = p.wait_for_next(late);
        assert!(
            !sleep_for.is_zero(),
            "next deadline should already be back in the future"
        );
        assert_eq!(
            p.missed_deadlines(),
            1,
            "no second miss from the same overshoot"
        );
    }

    #[test]
    fn the_deadline_advances_by_exactly_one_period_per_call_even_when_late() {
        let t0 = base();
        let period = Duration::from_millis(10);
        let mut p = Pacer::new(period, t0);
        let before = p.next_deadline;
        let late = t0 + period * 3;
        p.wait_for_next(late);
        assert_eq!(p.next_deadline, before + period);
    }

    #[test]
    fn a_run_of_misses_costs_exactly_one_miss_each() {
        // Simulates a loop that is permanently 3x too slow: each iteration's
        // own "now" arrives period*3 after the previous one, so the deadline
        // (advancing by exactly one period per call) never catches back up,
        // and every single call registers a miss -- not a burst on the first
        // call followed by silence.
        let t0 = base();
        let period = Duration::from_millis(10);
        let mut p = Pacer::new(period, t0);
        let mut now = t0;
        for _ in 0..4 {
            now += period * 3;
            let sleep_for = p.wait_for_next(now);
            assert!(sleep_for.is_zero());
        }
        assert_eq!(p.missed_deadlines(), 4);
    }
}
