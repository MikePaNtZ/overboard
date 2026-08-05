//! The timed loop: pace to an absolute deadline, run the real control law,
//! record how late we woke and how long the law took.
//!
//! # What this measures, and what it does not
//!
//! **Measures:** the two things that decide whether a 500 Hz software loop
//! holds its period on this kernel, on this hardware, under this load --
//! scheduler wake latency against an absolute deadline, and the wall-clock
//! cost of one pass through estimator → regulator → feedforward → envelope.
//!
//! **Does not measure:** anything involving a bus or a motor. There is no
//! CAN frame here, no `hal` backend, no actuation. Command→actuation latency
//! is AC-6 and needs a real drive on a real bus; this tool answers the
//! narrower question that comes first -- *can the host hold the cadence at
//! all* -- which is AC-5's territory, and it answers it without hardware.
//!
//! # Why the observations are synthetic
//!
//! Stepping a MuJoCo plant inside the timed window would put ~100 us of
//! physics into a measurement whose whole subject is a ~10 us control law,
//! and on the Pi there is no plant anyway -- the observations come off a CAN
//! bus. So the loop synthesises a deterministic, non-degenerate observation
//! stream: a sinusoidal pitch with a consistent accelerometer and gyro. It
//! costs a few hundred nanoseconds, it is generated OUTSIDE the timed span,
//! and it keeps the filter's arithmetic on a realistic trajectory rather
//! than letting a constant input collapse into a branch-predicted no-op.

use board_types::{Command, Faults, ImuSample, Observation, Params, ValidityFlags};
use control_core::{CommandFeedforward, ComplementaryFilter, Estimator, PitchRegulator};
use safety::Envelope;
use std::time::{Duration, Instant};

use crate::rt::{self, RtStatus};
use crate::stats::{summarise, Percentiles};

/// The ICD §11.2 control period: 500 Hz.
pub const DEFAULT_PERIOD_NS: u64 = 2_000_000;

/// AC-5's pre-registered wake-latency thresholds (reference doc §7).
///
/// Fixed here, in code, before anything is measured. A threshold that lives
/// only in a document is one a number gets fitted to afterwards.
pub const AC5_P999_LIMIT_NS: u64 = 150_000;
pub const AC5_MAX_LIMIT_NS: u64 = 500_000;

/// AC-5's duration requirement: `-D 30m`. At the default 500 Hz that is
/// 900,000 counted cycles, not the 100,000 (200 s) this tool's own `Config`
/// default runs -- so a default-configured run must never be able to print
/// `AC-5: PASS` (issue #226 defect 1).
pub const AC5_MIN_DURATION_NS: u64 = 30 * 60 * 1_000_000_000;

/// Control-law constants, mirroring `sim-host`'s.
///
/// **Duplicated deliberately, and safe to duplicate.** Importing them would
/// mean depending on `sim-host`, which links `hal-actuate` and MuJoCo -- the
/// two things this crate exists without (see its `Cargo.toml`). The drift
/// risk that normally makes duplication a mistake (issue #124) does not bite
/// here: this tool times the *composition*, and the cost of one pass through
/// it is insensitive to the gain values -- the same multiplies and the same
/// `atan2f` run whatever the numbers are. If these drift from `sim-host`'s,
/// the profile is unchanged; only a reader's expectation is wrong, which is
/// what this comment is for.
mod law {
    pub const KP_NM_PER_RAD: f32 = 140.0;
    pub const KD_NM_PER_RAD_S: f32 = 21.0;
    pub const KT_NM_PER_A: f32 = 0.7;
    pub const MAX_CURRENT_A: f32 = 40.0;
    pub const ESTIMATOR_TAU_S: f32 = 2.0;
    pub const ACCEL_FF_GAIN_M_S2_PER_A: f32 = 0.0584;
    /// Zero disables the accel trust gate, matching `sim-host`'s own
    /// `with_trust_band(ESTIMATOR_TAU_S, 0.0)`.
    pub const ACCEL_TRUST_BAND_M_S2: f32 = 0.0;
}

/// One run's parameters.
#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub period_ns: u64,
    /// Cycles contributing to the statistics.
    pub cycles: u64,
    /// Cycles run and discarded first. Cold caches, lazily-resolved PLT
    /// entries and first-touch page faults all land in the opening
    /// milliseconds, and reporting them as steady-state jitter would
    /// slander the platform.
    pub warmup: u64,
    /// `SCHED_FIFO` priority, or `None` to stay at normal priority.
    pub rt_prio: Option<i32>,
    /// Operator attestation that `stress-ng` CPU + network load was running
    /// throughout this run, per AC-5's `cyclictest -m -Sp95 -i 2000 -D 30m`
    /// under load. Not something this process can observe about its own
    /// machine, so it is a fact the caller supplies rather than one this
    /// tool measures (issue #226 defect 1).
    pub under_load: bool,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            period_ns: DEFAULT_PERIOD_NS,
            // 10^5 cycles is AC-6's pre-registered run length; at 500 Hz
            // that is 200 s. Matching it here means the two measurements are
            // the same size when they are eventually read side by side.
            // NOTE: this is well short of AC-5's own 30-minute duration --
            // see `AC5_MIN_DURATION_NS` -- so a default run is never
            // acceptance-grade on duration alone; `--cycles 900000` (or
            // more) is required for an actual AC-5 attempt.
            cycles: 100_000,
            warmup: 500,
            // 80 leaves headroom below the kernel's own RT threads (the
            // mcp251xfd IRQ thread among them) rather than out-prioritising
            // the driver whose latency we care about.
            rt_prio: Some(80),
            under_load: false,
        }
    }
}

/// What a run produced.
#[derive(Debug, Clone)]
pub struct Report {
    pub config: Config,
    pub rt: RtStatus,
    /// How late each cycle woke, relative to its absolute deadline.
    pub wake_late: Percentiles,
    /// How long one pass through the control law took.
    pub compute: Percentiles,
    /// Cycles whose work finished after the NEXT deadline -- a control
    /// instant genuinely lost, as distinct from merely waking late.
    pub overruns: u64,
    /// Median cost of a back-to-back `Instant::now()` pair, measured before
    /// the run. Reported because it is included in every `compute` sample
    /// and is not negligible at these magnitudes.
    pub clock_overhead_ns: u64,
    /// Measured wall-clock span of the counted cycles (warmup excluded).
    /// Because the loop paces to real deadlines this tracks `cycles *
    /// period_ns` closely, but it is measured rather than assumed so a
    /// duration gate means what it says.
    pub elapsed_ns: u64,
}

impl Report {
    /// Every AC-5 gate this run fails to clear, checked independently so a
    /// withheld verdict says exactly what is missing instead of one bare
    /// "NOT ASSESSED" (issue #226 defect 1).
    ///
    /// The `cyclictest` gap is unconditional: this tool is not `cyclictest`
    /// and can never satisfy that condition of AC-5 by gating anything, so
    /// it is always named rather than checked.
    pub fn ac5_gaps(&self) -> Vec<&'static str> {
        let mut gaps = Vec::new();
        if !self.rt.is_acceptance_grade() {
            gaps.push(
                "platform: not confirmed PREEMPT_RT + mlockall + SCHED_FIFO (see platform line)",
            );
        }
        if self.elapsed_ns < AC5_MIN_DURATION_NS {
            gaps.push("duration: AC-5 requires >= 30 min measured; this run was shorter");
        }
        if !self.config.under_load {
            gaps.push(
                "load: not confirmed under CPU + network load (rerun with --under-load once \
                 stress-ng is confirmed running)",
            );
        }
        gaps.push(
            "instrument: this tool is not cyclictest, the AC-5 instrument of record -- run \
             cyclictest alongside it",
        );
        gaps
    }

    /// Whether this run clears AC-5's thresholds.
    ///
    /// `None` when any gate in [`Self::ac5_gaps`] besides the cyclictest one
    /// is unmet -- deliberately not `Some(false)`, because "we could not
    /// measure this properly" and "the platform failed" are different
    /// findings and only one of them is about the platform. The cyclictest
    /// gap alone does not withhold a verdict: this tool can still say
    /// whether ITS OWN measurement passed the pre-registered thresholds,
    /// which `ac5_gaps` reports as a caveat next to any verdict.
    pub fn ac5_verdict(&self) -> Option<bool> {
        if !self.rt.is_acceptance_grade() {
            return None;
        }
        if self.elapsed_ns < AC5_MIN_DURATION_NS {
            return None;
        }
        if !self.config.under_load {
            return None;
        }
        Some(
            self.wake_late.p999_ns <= AC5_P999_LIMIT_NS
                && self.wake_late.max_ns <= AC5_MAX_LIMIT_NS,
        )
    }
}

/// A deterministic, non-degenerate observation for cycle `i`.
///
/// Pitch traces a slow sinusoid with a consistent gyro and accelerometer, so
/// the complementary filter integrates a real trajectory rather than a
/// constant. No RNG and no wall clock: every scenario in this project is
/// reproducible bit-for-bit, and a profiler whose input differed run to run
/// would make two runs incomparable for reasons unrelated to the platform.
fn synth_observation(i: u64, period_ns: u64) -> Observation {
    const AMPLITUDE_RAD: f32 = 0.05; // ~2.9 deg, a plausible ridden excursion
    const FREQ_HZ: f32 = 0.5;
    const G: f32 = 9.81;

    let t_ns = i * period_ns;
    let t_s = t_ns as f32 * 1e-9;
    let w = 2.0 * std::f32::consts::PI * FREQ_HZ;
    let pitch = AMPLITUDE_RAD * (w * t_s).sin();
    let pitch_rate = AMPLITUDE_RAD * w * (w * t_s).cos();

    let mut obs = Observation::COLD_START;
    obs.cycle = i;
    obs.t_recv_ns = t_ns;
    obs.imu[0] = ImuSample {
        // gyro[1] is the nose-up pitch rate (ICD §10.1).
        gyro_rad_s: [0.0, pitch_rate, 0.0],
        // Specific force for a body pitched by `pitch` under gravity alone,
        // matching ComplementaryFilter::accel_pitch's derivation.
        accel_m_s2: [G * pitch.sin(), 0.0, -G * pitch.cos()],
        t_sample_ns: t_ns,
    };
    obs.imu_count = 1;
    obs.validity = ValidityFlags::ALL_FRESH;
    obs
}

/// Median cost of reading the clock twice, the way the timed span does.
///
/// Subtracting this automatically would be tidier and wrong: the overhead is
/// genuinely part of what an instrumented loop costs, and hiding it would
/// make the reported compute time unachievable in practice. Reported
/// alongside instead, so a reader can do the subtraction knowingly.
fn calibrate_clock() -> u64 {
    const N: usize = 1_000;
    let mut samples = Vec::with_capacity(N);
    for _ in 0..N {
        let a = Instant::now();
        let b = Instant::now();
        samples.push(b.saturating_duration_since(a).as_nanos() as u64);
    }
    summarise(&mut samples).p50_ns
}

/// Run the profile.
///
/// # Pacing
///
/// Against an **absolute** deadline advanced by exactly one period per cycle,
/// for the reason `sim_host::Pacer` documents at length: sleeping for
/// `period` after each cycle's work accumulates drift, because the work's own
/// duration is never subtracted back out.
///
/// That arithmetic is duplicated here rather than imported, and the six lines
/// are worth it -- `Pacer` lives in `sim-host`, and depending on that crate
/// would drag `hal-actuate` and MuJoCo into a binary whose entire safety
/// argument is that it has neither.
pub fn run(cfg: Config) -> Report {
    let status = rt::acquire(cfg.rt_prio);
    let clock_overhead_ns = calibrate_clock();

    let params = Params {
        kp_nm_per_rad: law::KP_NM_PER_RAD,
        kd_nm_per_rad_s: law::KD_NM_PER_RAD_S,
        kt_nm_per_a: law::KT_NM_PER_A,
        max_current_a: law::MAX_CURRENT_A,
        ..Params::default()
    };

    let regulator = PitchRegulator::new(law::KP_NM_PER_RAD, law::KD_NM_PER_RAD_S);
    let mut estimator =
        ComplementaryFilter::with_trust_band(law::ESTIMATOR_TAU_S, law::ACCEL_TRUST_BAND_M_S2);
    let accel_ff = CommandFeedforward::new(law::ACCEL_FF_GAIN_M_S2_PER_A);
    let mut envelope = Envelope::new(params);
    envelope.arm();
    let mut last_amps: f32 = 0.0;

    // Allocated -- and touched -- before the timed loop starts. A `Vec` that
    // grows mid-run would allocate inside the measured window and fault in
    // fresh pages while doing it, which is precisely the artefact `mlockall`
    // was called to prevent.
    let total = cfg.cycles as usize;
    let mut wake_late_ns: Vec<u64> = vec![0; total];
    let mut compute_ns: Vec<u64> = vec![0; total];
    let mut overruns: u64 = 0;

    let period = Duration::from_nanos(cfg.period_ns);
    let start = Instant::now();
    let mut deadline = start + period;
    let mut counted_start: Option<Instant> = None;

    for i in 0..(cfg.warmup + cfg.cycles) {
        if i == cfg.warmup {
            counted_start = Some(Instant::now());
        }
        // Built outside the timed span: synthesising an observation is this
        // harness's cost, not the control law's.
        let obs = synth_observation(i, cfg.period_ns);

        let now = Instant::now();
        if deadline > now {
            std::thread::sleep(deadline - now);
        }
        let woke = Instant::now();
        let late_ns = woke.saturating_duration_since(deadline).as_nanos() as u64;

        // ---- the timed span: exactly what runs every cycle on the board ----
        let t_compute = Instant::now();
        let sample = obs.newest_imu().copied().unwrap_or(ImuSample::ZERO);
        let aiding = accel_ff.predict(last_amps);
        let attitude = estimator.update(std::slice::from_ref(&sample), aiding);
        let torque_nm = regulator.update(attitude.pitch_rad, attitude.pitch_rate_rad_s, 0.0);
        let proposed_amps = torque_nm / law::KT_NM_PER_A;
        let (bounded, _sat) = envelope.apply(
            Command::MotorCurrent {
                amps: proposed_amps,
            },
            Faults::NONE,
        );
        let compute = t_compute.elapsed().as_nanos() as u64;
        // ---- end of the timed span ----

        // POST-envelope current feeds the next cycle's feedforward, matching
        // sim-host and control-ffi: the plant only ever sees the clamped
        // value.
        last_amps = match bounded {
            Command::MotorCurrent { amps } => amps,
            Command::RemoteSpeed { .. } => 0.0,
        };

        if i >= cfg.warmup {
            let idx = (i - cfg.warmup) as usize;
            wake_late_ns[idx] = late_ns;
            compute_ns[idx] = compute;
            // An overrun is the cycle's work finishing after the NEXT
            // deadline -- a control instant actually lost. Waking late is
            // not by itself an overrun: sleep always overshoots a little,
            // and counting that as a miss would report ~100% misses on a
            // perfectly healthy loop.
            if woke + Duration::from_nanos(compute) > deadline + period {
                overruns += 1;
            }
        }

        deadline += period;
    }

    // `unwrap_or(0)` covers `cfg.cycles == 0` only: the loop body above
    // always sets `counted_start` on its first iteration whenever there is
    // at least one cycle to run, counted or warmup.
    let elapsed_ns = counted_start
        .map(|s| s.elapsed().as_nanos() as u64)
        .unwrap_or(0);

    Report {
        config: cfg,
        rt: status,
        wake_late: summarise(&mut wake_late_ns),
        compute: summarise(&mut compute_ns),
        overruns,
        clock_overhead_ns,
        elapsed_ns,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn short_run() -> Config {
        Config {
            period_ns: 1_000_000, // 1 ms, so the test finishes quickly
            cycles: 50,
            warmup: 5,
            // Never request RT in a test: on a privileged CI runner it would
            // actually succeed and pin a test thread at FIFO 80.
            rt_prio: None,
            under_load: false,
        }
    }

    #[test]
    fn a_run_produces_one_sample_per_counted_cycle() {
        let cfg = short_run();
        let r = run(cfg);
        assert_eq!(r.wake_late.count, cfg.cycles as usize);
        assert_eq!(r.compute.count, cfg.cycles as usize);
    }

    #[test]
    fn warmup_cycles_are_excluded_from_the_statistics() {
        let mut cfg = short_run();
        cfg.warmup = 40;
        let r = run(cfg);
        assert_eq!(r.wake_late.count, cfg.cycles as usize);
    }

    #[test]
    fn the_verdict_is_withheld_rather_than_failed_when_the_platform_cannot_support_it() {
        // CI is not PREEMPT_RT and the test does not request FIFO, so this
        // run can never be acceptance-grade. It must say "cannot tell", not
        // "the platform failed" -- the distinction the report exists to keep.
        let r = run(short_run());
        assert!(!r.rt.is_acceptance_grade());
        assert_eq!(r.ac5_verdict(), None);
    }

    /// A `Report` built directly rather than via `run()`, so the AC-5 gating
    /// tests below don't need a real 30-minute timed loop to exercise the
    /// duration/load gates (issue #226 defect 1).
    fn full_platform_report() -> Report {
        Report {
            config: Config {
                under_load: true,
                ..short_run()
            },
            rt: RtStatus {
                memory_locked: Some(true),
                sched_fifo_prio: Some(80),
                preempt_rt: Some(true),
                ..Default::default()
            },
            wake_late: Percentiles {
                p999_ns: AC5_P999_LIMIT_NS,
                max_ns: AC5_MAX_LIMIT_NS,
                ..Percentiles::EMPTY
            },
            compute: Percentiles::EMPTY,
            overruns: 0,
            clock_overhead_ns: 0,
            elapsed_ns: AC5_MIN_DURATION_NS,
        }
    }

    #[test]
    fn a_default_configured_run_is_too_short_to_pass_ac5_by_duration_alone() {
        // The exact bug issue #226 reported: an idle 200 s run must not be
        // able to print AC-5: PASS.
        let d = Config::default();
        assert!(
            d.cycles * d.period_ns < AC5_MIN_DURATION_NS,
            "default cycles*period must stay short of AC-5's 30-minute floor"
        );
    }

    #[test]
    fn full_platform_and_duration_but_no_load_withholds_the_verdict() {
        let mut r = full_platform_report();
        r.config.under_load = false;
        assert_eq!(r.ac5_verdict(), None);
        assert!(r.ac5_gaps().iter().any(|g| g.starts_with("load:")));
    }

    #[test]
    fn full_platform_and_load_but_short_duration_withholds_the_verdict() {
        let mut r = full_platform_report();
        r.elapsed_ns = AC5_MIN_DURATION_NS - 1;
        assert_eq!(r.ac5_verdict(), None);
        assert!(r.ac5_gaps().iter().any(|g| g.starts_with("duration:")));
    }

    #[test]
    fn every_gate_met_yields_a_real_verdict_with_the_cyclictest_gap_still_named() {
        let r = full_platform_report();
        assert_eq!(r.ac5_verdict(), Some(true));
        // The one gap that gating can never close: this tool is not the
        // instrument AC-5 names, so it is always disclosed alongside a PASS.
        assert!(r.ac5_gaps().iter().any(|g| g.starts_with("instrument:")));
        assert_eq!(
            r.ac5_gaps().len(),
            1,
            "only the unconditional gap should remain: {:?}",
            r.ac5_gaps()
        );
    }

    #[test]
    fn the_control_law_actually_runs() {
        // Guards against the loop being optimised down to nothing, which
        // would make every compute number meaningless. One pass through an
        // atan2 and three filters cannot take zero nanoseconds.
        let r = run(short_run());
        assert!(
            r.compute.max_ns > 0,
            "compute time collapsed to zero - the law is not being executed"
        );
    }

    #[test]
    fn synthetic_observations_are_deterministic() {
        // Two runs of the generator must agree exactly: reproducibility is a
        // project-wide rule (`tests/test_stage0b_runbook.py` pins the same
        // property for the runbook's dry run).
        for i in [0u64, 1, 499, 100_000] {
            assert_eq!(
                synth_observation(i, DEFAULT_PERIOD_NS),
                synth_observation(i, DEFAULT_PERIOD_NS)
            );
        }
    }

    #[test]
    fn synthetic_observations_move() {
        // A constant input would let the filter collapse into a
        // branch-predicted no-op and understate the compute cost.
        let a = synth_observation(0, DEFAULT_PERIOD_NS);
        let b = synth_observation(250, DEFAULT_PERIOD_NS); // quarter period
        assert_ne!(a.imu[0].accel_m_s2[0], b.imu[0].accel_m_s2[0]);
        assert_ne!(a.imu[0].gyro_rad_s[1], b.imu[0].gyro_rad_s[1]);
    }

    #[test]
    fn synthetic_timestamps_advance_by_exactly_one_period() {
        let a = synth_observation(10, DEFAULT_PERIOD_NS);
        let b = synth_observation(11, DEFAULT_PERIOD_NS);
        assert_eq!(
            b.imu[0].t_sample_ns - a.imu[0].t_sample_ns,
            DEFAULT_PERIOD_NS
        );
    }

    #[test]
    fn ac5_thresholds_are_the_pre_registered_ones() {
        // A test whose only job is to make moving a pre-registered threshold
        // a deliberate, reviewable act rather than a one-character edit.
        assert_eq!(AC5_P999_LIMIT_NS, 150_000);
        assert_eq!(AC5_MAX_LIMIT_NS, 500_000);
    }
}
