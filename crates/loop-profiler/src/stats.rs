//! Tail statistics over a run's per-cycle samples.
//!
//! **Tail percentiles only -- never a mean as a headline number.** The
//! Stage-0B reference doc §7 is explicit about why: "a 2 ms spike at the
//! 99.9th percentile is invisible in a mean, and a mean is exactly what an
//! under-specified measurement protocol will produce." The mean is reported
//! here, but as context beside p99.9 and max, never instead of them.

/// Nearest-rank percentiles over one run's samples, in nanoseconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Percentiles {
    pub count: usize,
    pub min_ns: u64,
    pub p50_ns: u64,
    pub p99_ns: u64,
    pub p999_ns: u64,
    pub max_ns: u64,
    pub mean_ns: u64,
}

impl Percentiles {
    /// The all-zero summary, for an empty run.
    pub const EMPTY: Percentiles = Percentiles {
        count: 0,
        min_ns: 0,
        p50_ns: 0,
        p99_ns: 0,
        p999_ns: 0,
        max_ns: 0,
        mean_ns: 0,
    };
}

/// Nearest-rank percentile: the smallest value at or below which at least
/// `q` of the samples fall.
///
/// Nearest-rank rather than interpolated on purpose. An interpolated p99.9
/// invents a value that no cycle actually took, and every number this tool
/// produces is compared against a pre-registered threshold -- a synthesised
/// sample sitting either side of that line is exactly the ambiguity the
/// pre-registration exists to remove.
fn nearest_rank(sorted: &[u64], q: f64) -> u64 {
    if sorted.is_empty() {
        return 0;
    }
    // ceil(q * n), clamped to [1, n], then converted to a 0-based index.
    let rank = (q * sorted.len() as f64).ceil() as usize;
    let idx = rank.clamp(1, sorted.len()) - 1;
    sorted[idx]
}

/// Summarise `samples`, sorting them in place.
///
/// Takes `&mut` and sorts rather than copying: at 10^5 cycles this is called
/// once, after the timed loop has finished, so the sort costs nothing that
/// matters -- but allocating a second 800 KB buffer inside a process that
/// just called `mlockall` is a real cost for no benefit.
pub fn summarise(samples: &mut [u64]) -> Percentiles {
    if samples.is_empty() {
        return Percentiles::EMPTY;
    }
    samples.sort_unstable();
    // u128 so a long run of large samples cannot overflow the accumulator.
    let sum: u128 = samples.iter().map(|&s| s as u128).sum();
    Percentiles {
        count: samples.len(),
        min_ns: samples[0],
        p50_ns: nearest_rank(samples, 0.50),
        p99_ns: nearest_rank(samples, 0.99),
        p999_ns: nearest_rank(samples, 0.999),
        max_ns: samples[samples.len() - 1],
        mean_ns: (sum / samples.len() as u128) as u64,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_input_summarises_to_zero_rather_than_panicking() {
        assert_eq!(summarise(&mut []), Percentiles::EMPTY);
    }

    #[test]
    fn percentiles_are_real_samples_not_interpolated() {
        // 1000 samples: 999 cheap, one 5 ms outlier. An interpolated p99.9
        // would land somewhere between 1000 and 5_000_000; nearest-rank must
        // return a value that actually occurred.
        let mut s: Vec<u64> = (0..999).map(|_| 1_000).collect();
        s.push(5_000_000);
        let p = summarise(&mut s);
        assert_eq!(p.count, 1000);
        assert_eq!(p.p50_ns, 1_000);
        assert_eq!(p.p99_ns, 1_000);
        assert_eq!(p.max_ns, 5_000_000);
        assert!(
            p.p999_ns == 1_000 || p.p999_ns == 5_000_000,
            "p99.9 must be an observed sample, got {}",
            p.p999_ns
        );
    }

    #[test]
    fn a_single_late_cycle_is_visible_in_max_but_not_in_the_mean() {
        // The reason this module reports tails at all. One 20 ms stall in a
        // 10^4-cycle run moves the mean by 2 us -- invisible -- while max
        // shows it immediately.
        let mut clean: Vec<u64> = (0..10_000).map(|_| 100_000).collect();
        let clean_p = summarise(&mut clean);

        let mut stalled: Vec<u64> = (0..9_999).map(|_| 100_000).collect();
        stalled.push(20_000_000);
        let stalled_p = summarise(&mut stalled);

        assert!(
            stalled_p.mean_ns.abs_diff(clean_p.mean_ns) < 3_000,
            "the mean should barely move: {} vs {}",
            stalled_p.mean_ns,
            clean_p.mean_ns
        );
        assert_eq!(stalled_p.max_ns, 20_000_000);
    }

    #[test]
    fn percentiles_are_monotonic() {
        let mut s: Vec<u64> = (0..10_000).map(|i| i as u64).collect();
        let p = summarise(&mut s);
        assert!(p.min_ns <= p.p50_ns);
        assert!(p.p50_ns <= p.p99_ns);
        assert!(p.p99_ns <= p.p999_ns);
        assert!(p.p999_ns <= p.max_ns);
    }

    #[test]
    fn a_single_sample_is_every_percentile() {
        let p = summarise(&mut [42]);
        assert_eq!(p.min_ns, 42);
        assert_eq!(p.p50_ns, 42);
        assert_eq!(p.p999_ns, 42);
        assert_eq!(p.max_ns, 42);
        assert_eq!(p.mean_ns, 42);
    }
}
