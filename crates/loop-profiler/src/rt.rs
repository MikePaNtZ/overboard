//! Real-time process setup, and an honest report of how much of it worked.
//!
//! Two things separate a 500 Hz loop that holds its period from one that does
//! not, and neither is the control law:
//!
//! 1. **`mlockall`** -- a major page fault inside the loop is a disk round
//!    trip, which at 2 ms is not a jitter spike but several missed cycles.
//! 2. **`SCHED_FIFO`** -- under the default `SCHED_OTHER` the loop is
//!    competing with every other runnable task, and PREEMPT_RT buys nothing.
//!
//! Both can fail for ordinary reasons (no privileges, wrong kernel), and the
//! failure is silent unless something looks. **A profiler that quietly
//! reports `SCHED_OTHER` numbers as if they were RT numbers is worse than no
//! profiler**, because the number looks like the go/no-go measurement and is
//! not one. So every acquisition is checked, and [`RtStatus`] carries what
//! was actually obtained -- the caller decides what to do about the gap, and
//! the JSON output records it beside the percentiles.

/// What the process actually managed to acquire, plus what the kernel is.
#[derive(Debug, Clone, Default)]
pub struct RtStatus {
    /// `None` where the platform has no equivalent (i.e. not Linux).
    pub memory_locked: Option<bool>,
    /// The `SCHED_FIFO` priority in force, if the switch succeeded.
    pub sched_fifo_prio: Option<i32>,
    /// From `/proc/sys/kernel/osrelease`. The kernel is the single variable
    /// that invalidates every latency number this tool produces (design §2),
    /// so it is recorded on every run rather than assumed constant.
    pub kernel_release: Option<String>,
    /// `Some(true)` only when `/sys/kernel/realtime` reads `1`. `None` means
    /// the question could not be answered, which is NOT the same as `false`
    /// and is never reported as such.
    pub preempt_rt: Option<bool>,
    /// Human-readable reasons a step did not happen, with the remedy.
    pub warnings: Vec<String>,
}

impl RtStatus {
    /// Whether a measurement taken under this status can be compared against
    /// the pre-registered AC-5 thresholds.
    ///
    /// Deliberately strict: anything less than locked memory + `SCHED_FIFO`
    /// on a confirmed PREEMPT_RT kernel produces a number that is interesting
    /// but not the acceptance number. The tool still prints the percentiles
    /// in that case -- it just refuses to call them a pass.
    pub fn is_acceptance_grade(&self) -> bool {
        self.memory_locked == Some(true)
            && self.sched_fifo_prio.is_some()
            && self.preempt_rt == Some(true)
    }

    /// One-line summary for the report header.
    pub fn summary(&self) -> String {
        let sched = match self.sched_fifo_prio {
            Some(p) => format!("SCHED_FIFO:{p}"),
            None => "SCHED_OTHER".to_string(),
        };
        let mem = match self.memory_locked {
            Some(true) => "mlockall",
            Some(false) => "unlocked",
            None => "mlock:n/a",
        };
        let rt = match self.preempt_rt {
            Some(true) => "PREEMPT_RT",
            Some(false) => "non-RT",
            None => "rt:unknown",
        };
        let kernel = self.kernel_release.as_deref().unwrap_or("kernel:unknown");
        format!("{sched}  {mem}  {rt}  {kernel}")
    }
}

/// Whether a `uname -v` string names a PREEMPT_RT build, e.g.
/// `#1 SMP PREEMPT_RT Debian 1:6.12.34-1 (2025-06-xx)`.
///
/// A free function at module scope, not inside the Linux-only `imp` module,
/// so it is unit-testable on every platform this crate builds on without a
/// subprocess or a cfg-gate.
fn detect_preempt_rt_from_uname_v(v: &str) -> bool {
    v.contains("PREEMPT_RT")
}

#[cfg(target_os = "linux")]
mod imp {
    use super::RtStatus;

    /// Locks memory and raises the scheduler, reporting whatever it got.
    ///
    /// Never returns an error: a profiler that refuses to run without
    /// privileges is a profiler nobody runs on their laptop, and the whole
    /// value of the no-Pi coverage rule (AC-12) is that this harness is
    /// exercised long before the numbers matter.
    pub fn acquire(prio: Option<i32>) -> RtStatus {
        let mut preempt_rt = read_trimmed("/sys/kernel/realtime").map(|v| v == "1");
        if preempt_rt.is_none() {
            // `/sys/kernel/realtime` is the primary source, but its absence
            // is not itself evidence of a non-RT kernel -- it is evidence
            // the file was not there to read (issue #226 defect 2: this
            // branch has never once been exercised on a real RT kernel, so
            // an unexpectedly missing file must not silently make AC-5
            // unmeasurable forever). `uname -v` is a second, independent
            // source: Debian/Raspberry Pi OS RT kernels stamp `PREEMPT_RT`
            // into it.
            preempt_rt = uname_v().map(|v| super::detect_preempt_rt_from_uname_v(&v));
        }

        let mut status = RtStatus {
            kernel_release: read_trimmed("/proc/sys/kernel/osrelease"),
            preempt_rt,
            ..Default::default()
        };

        // MCL_FUTURE as well as MCL_CURRENT: the stack grows during the run,
        // and a page faulted in on first touch inside the loop costs exactly
        // what this call exists to prevent.
        let rc = unsafe { libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE) };
        if rc == 0 {
            status.memory_locked = Some(true);
        } else {
            status.memory_locked = Some(false);
            status.warnings.push(format!(
                "mlockall failed ({}) - page faults inside the loop will show as \
                 latency spikes. Remedy: run as root, or grant CAP_IPC_LOCK.",
                std::io::Error::last_os_error()
            ));
        }

        if let Some(prio) = prio {
            // Zeroed rather than brace-initialised: `sched_param` carries
            // extra fields on some targets, and leaving them uninitialised
            // would be undefined behaviour rather than merely untidy.
            let mut param: libc::sched_param = unsafe { std::mem::zeroed() };
            param.sched_priority = prio;
            let rc = unsafe { libc::sched_setscheduler(0, libc::SCHED_FIFO, &param) };
            if rc == 0 {
                status.sched_fifo_prio = Some(prio);
            } else {
                status.warnings.push(format!(
                    "sched_setscheduler(SCHED_FIFO, {prio}) failed ({}) - running at \
                     normal priority, so these numbers measure a fair-share scheduler, \
                     not a real-time one. Remedy: run as root, or grant CAP_SYS_NICE.",
                    std::io::Error::last_os_error()
                ));
            }
        }

        if status.preempt_rt == Some(false) {
            status.warnings.push(
                "this kernel is not PREEMPT_RT - the harness is being exercised, but the \
                 numbers are not an AC-5 result."
                    .to_string(),
            );
        }

        status
    }

    fn read_trimmed(path: &str) -> Option<String> {
        std::fs::read_to_string(path)
            .ok()
            .map(|s| s.trim().to_string())
    }

    /// `uname -v` output, or `None` if the command could not be run at all --
    /// which leaves the caller with no second opinion, not a false one.
    fn uname_v() -> Option<String> {
        std::process::Command::new("uname")
            .arg("-v")
            .output()
            .ok()
            .filter(|o| o.status.success())
            .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
    }
}

#[cfg(not(target_os = "linux"))]
mod imp {
    use super::RtStatus;

    /// Non-Linux stub. Runs the loop, refuses to dress the result up.
    ///
    /// This path exists so the harness can be developed and unit-tested on a
    /// macOS laptop. macOS has no `SCHED_FIFO` equivalent worth the name and
    /// its timer coalescing already cost this project a day of confusion
    /// (issue #168: a requested 2 ms sleep observed arriving at ~16 ms), so
    /// the numbers from here are for debugging the tool, never the platform.
    pub fn acquire(_prio: Option<i32>) -> RtStatus {
        RtStatus {
            warnings: vec![format!(
                "{} is not a real-time target: no mlockall, no SCHED_FIFO, and OS timer \
                 coalescing dominates the measurement (issue #168). Numbers from this \
                 platform exercise the harness only.",
                std::env::consts::OS
            )],
            ..Default::default()
        }
    }
}

pub use imp::acquire;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_status_with_nothing_acquired_is_not_acceptance_grade() {
        assert!(!RtStatus::default().is_acceptance_grade());
    }

    #[test]
    fn fifo_on_a_non_rt_kernel_is_not_acceptance_grade() {
        // The case worth guarding: everything the process could control
        // succeeded, and the kernel still is not the one AC-5 names.
        let status = RtStatus {
            memory_locked: Some(true),
            sched_fifo_prio: Some(80),
            preempt_rt: Some(false),
            ..Default::default()
        };
        assert!(!status.is_acceptance_grade());
    }

    #[test]
    fn an_unknown_kernel_is_never_treated_as_rt() {
        let status = RtStatus {
            memory_locked: Some(true),
            sched_fifo_prio: Some(80),
            preempt_rt: None,
            ..Default::default()
        };
        assert!(!status.is_acceptance_grade());
    }

    #[test]
    fn the_full_house_is_acceptance_grade() {
        let status = RtStatus {
            memory_locked: Some(true),
            sched_fifo_prio: Some(80),
            preempt_rt: Some(true),
            ..Default::default()
        };
        assert!(status.is_acceptance_grade());
    }

    #[test]
    fn acquire_never_panics_and_always_reports_something() {
        // Runs on whatever the CI runner is, privileged or not.
        let status = acquire(None);
        assert!(!status.summary().is_empty());
    }

    #[test]
    fn uname_v_fallback_recognises_a_real_pi_rt_string() {
        // The actual string this fallback exists for (issue #226 defect 2),
        // reported from the CEO's Pi 5 Rev 1.1.
        assert!(detect_preempt_rt_from_uname_v(
            "#1 SMP PREEMPT_RT Debian 1:6.12.34-1 (2025-06-xx)"
        ));
    }

    #[test]
    fn uname_v_fallback_does_not_see_rt_in_a_stock_kernel_string() {
        assert!(!detect_preempt_rt_from_uname_v(
            "#1 SMP Debian 1:6.12.34-1 (2025-06-xx)"
        ));
    }
}
