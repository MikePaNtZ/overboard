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

use std::collections::VecDeque;
use std::time::{Duration, Instant};

/// How many recent per-tick lateness samples [`Pacer`] keeps for
/// [`Pacer::jitter_percentiles`]. A bounded ring, not the whole run: `Pacer`
/// can run for as long as the process does (`HostConfig::duration: None`),
/// so an unbounded `Vec` here would be a slow memory leak in exactly the
/// long-running, real-time-adjacent loop this crate cannot afford to
/// perturb. 2,000 samples is a 4 s window at 500 Hz -- long enough to be a
/// meaningful jitter picture, small enough that sorting it every
/// [`write_stats`]-interval (issue #168: currently ~100 ms) costs
/// microseconds, not milliseconds.
///
/// [`write_stats`]: crate::host
const JITTER_WINDOW: usize = 2_000;

/// Nearest-rank percentiles over a recent window of per-tick lateness, in
/// nanoseconds. Same nearest-rank convention as
/// `crates/loop-profiler/src/stats.rs::Percentiles` (a real observed sample,
/// never an interpolated value that no tick actually took) -- duplicated
/// here rather than pulled in as a dependency: this crate has no other
/// reason to depend on `loop-profiler`, and the algorithm is ~10 lines.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct JitterPercentiles {
    pub count: usize,
    pub p50_ns: u64,
    pub p99_ns: u64,
    pub max_ns: u64,
}

impl JitterPercentiles {
    pub const EMPTY: JitterPercentiles = JitterPercentiles {
        count: 0,
        p50_ns: 0,
        p99_ns: 0,
        max_ns: 0,
    };
}

fn nearest_rank(sorted: &[u64], q: f64) -> u64 {
    if sorted.is_empty() {
        return 0;
    }
    let rank = (q * sorted.len() as f64).ceil() as usize;
    let idx = rank.clamp(1, sorted.len()) - 1;
    sorted[idx]
}

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
    /// Per-call lateness (0 when on time), most recent
    /// [`JITTER_WINDOW`] calls only. See issue #168: a raw
    /// `missed_deadlines` count alone conflates "every tick was 1 ns late"
    /// with "one tick stalled 15 ms" -- the same count, very different
    /// severity. This is what lets a caller report the shape of the lag,
    /// not just how many times it happened.
    lateness_ns: VecDeque<u64>,
}

impl Pacer {
    /// Starts the clock at `now`: the first `wait_for_next()` targets
    /// `now + period`, not some later first call's `now`.
    pub fn new(period: Duration, now: Instant) -> Self {
        Pacer {
            period,
            next_deadline: now + period,
            missed_deadlines: 0,
            lateness_ns: VecDeque::with_capacity(JITTER_WINDOW),
        }
    }

    /// Total missed deadlines observed so far.
    pub fn missed_deadlines(&self) -> u64 {
        self.missed_deadlines
    }

    /// Percentiles of per-tick lateness over the most recent
    /// [`JITTER_WINDOW`] calls to [`Self::wait_for_next`] (0 for a tick that
    /// was on time or early). Cheap enough to call every stats-file write:
    /// sorts a bounded, small buffer, never the whole run.
    pub fn jitter_percentiles(&self) -> JitterPercentiles {
        if self.lateness_ns.is_empty() {
            return JitterPercentiles::EMPTY;
        }
        let mut sorted: Vec<u64> = self.lateness_ns.iter().copied().collect();
        sorted.sort_unstable();
        JitterPercentiles {
            count: sorted.len(),
            p50_ns: nearest_rank(&sorted, 0.50),
            p99_ns: nearest_rank(&sorted, 0.99),
            max_ns: *sorted.last().unwrap(),
        }
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
        let late_ns = now.saturating_duration_since(self.next_deadline).as_nanos() as u64;
        if self.lateness_ns.len() == JITTER_WINDOW {
            self.lateness_ns.pop_front();
        }
        self.lateness_ns.push_back(late_ns);
        // Advances by exactly one period regardless of the overrun, so the
        // loop never skips a physics step or accelerates sim-time to "catch
        // up" -- one call to wait_for_next() is always exactly one hal
        // cycle, whatever real time it actually took.
        //
        // CORRECTION (issue #168): an earlier revision of this comment
        // claimed this arithmetic avoids "a cascading pile-up of catch-up
        // iterations". It does not, and #168 measured exactly that: OS timer
        // coalescing (observed on macOS, not this crate's doing -- p50 ~0.09
        // ms, p99 ~15 ms) can turn a requested ~2 ms sleep into an actual
        // ~16 ms one, so the next wait_for_next() finds next_deadline
        // roughly 8 periods behind `now`. Because this only ever advances
        // the deadline by ONE period per call, the loop then runs ~8
        // back-to-back iterations with essentially no sleep between them --
        // each one legitimately a miss (each really did blow its own 2 ms
        // window) and each legitimately counted, but that IS a burst, not
        // smoothed-away drift. What this arithmetic actually guarantees is
        // narrower, and still worth having: the burst is bounded by how far
        // behind the clock fell, it does not compound further across calls,
        // and no physics step is ever skipped or doubled up to fake being
        // caught up.
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

    #[test]
    fn jitter_percentiles_are_empty_before_any_call() {
        let p = Pacer::new(Duration::from_millis(10), base());
        assert_eq!(p.jitter_percentiles(), JitterPercentiles::EMPTY);
    }

    #[test]
    fn on_time_ticks_report_zero_jitter() {
        let t0 = base();
        let period = Duration::from_millis(20);
        let mut p = Pacer::new(period, t0);
        let mut now = t0;
        for _ in 0..5 {
            now += Duration::from_micros(1);
            let sleep_for = p.wait_for_next(now);
            now += sleep_for;
        }
        let j = p.jitter_percentiles();
        assert_eq!(j.count, 5);
        assert_eq!(j.p50_ns, 0);
        assert_eq!(j.p99_ns, 0);
        assert_eq!(j.max_ns, 0);
    }

    #[test]
    fn one_late_tick_among_on_time_ones_shows_up_in_max_but_not_p50() {
        // Same shape as issue #168's own report: mostly on-time, one stall.
        // A p50 of 0 next to a non-zero max is exactly the "same fact, two
        // readings" distinction #168 asked to be able to tell apart.
        let t0 = base();
        let period = Duration::from_millis(2);
        let mut p = Pacer::new(period, t0);
        for i in 0..20u32 {
            let mut now = t0 + period * (i + 1);
            if i == 10 {
                now += Duration::from_millis(15);
            }
            p.wait_for_next(now);
        }
        let j = p.jitter_percentiles();
        assert_eq!(j.count, 20);
        assert_eq!(j.p50_ns, 0, "the other 19 ticks were on time");
        assert_eq!(j.max_ns, Duration::from_millis(15).as_nanos() as u64);
    }

    #[test]
    fn the_jitter_window_forgets_samples_older_than_its_capacity() {
        let t0 = base();
        let period = Duration::from_millis(1);
        let mut p = Pacer::new(period, t0);
        // One large stall, then enough clean ticks to push it out of the
        // bounded window -- the whole point of JITTER_WINDOW being bounded
        // rather than "the whole run" is that this DOES eventually happen.
        let mut now = t0 + period + Duration::from_millis(50);
        p.wait_for_next(now);
        for i in 0..JITTER_WINDOW {
            now = t0 + period * (i as u32 + 2);
            p.wait_for_next(now);
        }
        let j = p.jitter_percentiles();
        assert_eq!(j.count, JITTER_WINDOW);
        assert_eq!(
            j.max_ns, 0,
            "the 50 ms stall should have scrolled out of the window"
        );
    }
}
