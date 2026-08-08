//! Rendering a [`Report`] for a human and for a machine.
//!
//! The JSON follows the same conventions as the Stage-0B runbook's log
//! schema (`docs/runbook-stage0b-bench.md`): a `schema_version` that bumps on
//! any incompatible change, and a `git_sha` so a number can always be traced
//! back to the code that produced it. It is written by hand rather than via
//! `serde_json` -- the schema is flat, it is the only JSON this crate emits,
//! and a dependency is a poor trade for twenty lines.

use crate::profile::{Report, AC5_MAX_LIMIT_NS, AC5_MIN_DURATION_NS, AC5_P999_LIMIT_NS};
use crate::stats::Percentiles;

/// Bump on any incompatible field change.
///
/// v2 (issue #226): `ac5.verdict` now also gates on measured duration and
/// operator-attested load, not platform alone -- a `true` under v1 semantics
/// can now be `null` under v2 for the same platform, so a consumer diffing
/// across schema versions needs to know that changed.
pub const SCHEMA_VERSION: u32 = 2;

fn us(ns: u64) -> f64 {
    ns as f64 / 1_000.0
}

/// Two decimal places, not one: the control law comes in well under a
/// microsecond on any machine worth running this on, and at one decimal every
/// compute figure prints as `0.0` -- which reads like the law was optimised
/// away rather than like it is cheap.
fn row(label: &str, p: &Percentiles) -> String {
    format!(
        "  {label:<14} p50 {:>10.2}  p99 {:>10.2}  p99.9 {:>10.2}  max {:>10.2}  (mean {:>8.2})\n",
        us(p.p50_ns),
        us(p.p99_ns),
        us(p.p999_ns),
        us(p.max_ns),
        us(p.mean_ns),
    )
}

/// Human-readable summary. Microseconds throughout -- the thresholds are in
/// microseconds and a reader should never have to count zeroes to compare.
pub fn render_text(r: &Report) -> String {
    let hz = 1e9 / r.config.period_ns as f64;
    let mut s = String::new();

    s.push_str(&format!(
        "overboard-loop-profile - {:.1} Hz ({:.2} ms period), {} cycles after {} warmup\n",
        hz,
        r.config.period_ns as f64 / 1e6,
        r.config.cycles,
        r.config.warmup,
    ));
    s.push_str(&format!("  platform: {}\n\n", r.rt.summary()));

    s.push_str("  all figures in microseconds\n");
    s.push_str(&row("wake latency", &r.wake_late));
    s.push_str(&row("compute", &r.compute));
    s.push_str(&format!(
        "  clock overhead {:.2} us (included in every compute sample, not subtracted)\n",
        us(r.clock_overhead_ns),
    ));
    s.push_str(&format!(
        "  overruns: {} of {} cycles ({:.4}%) - work finishing after the next deadline\n\n",
        r.overruns,
        r.config.cycles,
        100.0 * r.overruns as f64 / r.config.cycles.max(1) as f64,
    ));

    s.push_str(&format!(
        "  AC-5 thresholds: p99.9 <= {:.0} us, max <= {:.0} us\n",
        us(AC5_P999_LIMIT_NS),
        us(AC5_MAX_LIMIT_NS),
    ));
    match r.ac5_verdict() {
        // A bare PASS is exactly what issue #226 found gets pasted into the
        // acceptance record as if it were the full criterion -- so the gaps
        // this run still cannot close (load self-attestation, cyclictest
        // itself) print immediately underneath every verdict, pass or fail.
        Some(true) => s.push_str("  AC-5: PASS (this tool's own measurement only -- see gaps below)\n"),
        Some(false) => {
            s.push_str("  AC-5: FAIL - escalate as an architecture finding, do not tune it away\n")
        }
        // The distinction the whole report is built around: not measurable
        // and failed are different findings, and only one is about the Pi.
        None => s.push_str(
            "  AC-5: NOT ASSESSED - this run does not clear every AC-5 gate, so these numbers\n         \
             exercise the harness and describe this machine. They are not an AC-5 result.\n",
        ),
    }
    for gap in r.ac5_gaps() {
        s.push_str(&format!("    - {gap}\n"));
    }

    for w in &r.rt.warnings {
        s.push_str(&format!("  warning: {w}\n"));
    }
    s
}

fn esc(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out
}

fn pct_json(p: &Percentiles) -> String {
    format!(
        "{{\"count\": {}, \"min_ns\": {}, \"p50_ns\": {}, \"p99_ns\": {}, \
         \"p999_ns\": {}, \"max_ns\": {}, \"mean_ns\": {}}}",
        p.count, p.min_ns, p.p50_ns, p.p99_ns, p.p999_ns, p.max_ns, p.mean_ns,
    )
}

fn opt_bool(v: Option<bool>) -> &'static str {
    match v {
        Some(true) => "true",
        Some(false) => "false",
        None => "null",
    }
}

/// Machine-readable form, for offline analysis and for CI to diff.
pub fn render_json(r: &Report, git_sha: &str) -> String {
    let warnings =
        r.rt.warnings
            .iter()
            .map(|w| format!("\"{}\"", esc(w)))
            .collect::<Vec<_>>()
            .join(", ");
    let kernel = match &r.rt.kernel_release {
        Some(k) => format!("\"{}\"", esc(k)),
        None => "null".to_string(),
    };
    let prio = match r.rt.sched_fifo_prio {
        Some(p) => p.to_string(),
        None => "null".to_string(),
    };
    let gaps = r
        .ac5_gaps()
        .iter()
        .map(|g| format!("\"{}\"", esc(g)))
        .collect::<Vec<_>>()
        .join(", ");

    format!(
        "{{\n  \"schema_version\": {SCHEMA_VERSION},\n  \
         \"tool\": \"overboard-loop-profile\",\n  \
         \"git_sha\": \"{}\",\n  \
         \"period_ns\": {},\n  \
         \"cycles\": {},\n  \
         \"warmup\": {},\n  \
         \"platform\": {{\"os\": \"{}\", \"kernel_release\": {}, \"preempt_rt\": {}, \
         \"memory_locked\": {}, \"sched_fifo_prio\": {}, \"acceptance_grade\": {}}},\n  \
         \"wake_late\": {},\n  \
         \"compute\": {},\n  \
         \"overruns\": {},\n  \
         \"clock_overhead_ns\": {},\n  \
         \"elapsed_ns\": {},\n  \
         \"ac5\": {{\"p999_limit_ns\": {AC5_P999_LIMIT_NS}, \"max_limit_ns\": {AC5_MAX_LIMIT_NS}, \
         \"min_duration_ns\": {AC5_MIN_DURATION_NS}, \"under_load\": {}, \
         \"verdict\": {}, \"gaps\": [{}]}},\n  \
         \"warnings\": [{}]\n}}\n",
        esc(git_sha),
        r.config.period_ns,
        r.config.cycles,
        r.config.warmup,
        esc(std::env::consts::OS),
        kernel,
        opt_bool(r.rt.preempt_rt),
        opt_bool(r.rt.memory_locked),
        prio,
        r.rt.is_acceptance_grade(),
        pct_json(&r.wake_late),
        pct_json(&r.compute),
        r.overruns,
        r.clock_overhead_ns,
        r.elapsed_ns,
        r.config.under_load,
        opt_bool(r.ac5_verdict()),
        gaps,
        warnings,
    )
}

/// Best-effort short commit, `"unknown"` if git is unavailable.
///
/// Called after the timed loop, never during it. `"unknown"` rather than a
/// silent omission: a provenance field that vanishes when it cannot be
/// filled makes an untraceable run look like a traceable one.
pub fn git_sha() -> String {
    let out = std::process::Command::new("git")
        .args(["rev-parse", "--short", "HEAD"])
        .output();
    match out {
        Ok(o) if o.status.success() => {
            let sha = String::from_utf8_lossy(&o.stdout).trim().to_string();
            if sha.is_empty() {
                return "unknown".to_string();
            }
            let dirty = std::process::Command::new("git")
                .args(["status", "--porcelain"])
                .output()
                .map(|s| !s.stdout.is_empty())
                .unwrap_or(false);
            if dirty {
                format!("{sha}-dirty")
            } else {
                sha
            }
        }
        _ => "unknown".to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::profile::{self, Config};

    fn a_report() -> Report {
        profile::run(Config {
            period_ns: 1_000_000,
            cycles: 20,
            warmup: 2,
            rt_prio: None,
            under_load: false,
        })
    }

    #[test]
    fn text_output_names_the_thresholds_and_withholds_the_verdict_off_rt() {
        let t = render_text(&a_report());
        assert!(t.contains("AC-5 thresholds"));
        assert!(
            t.contains("NOT ASSESSED"),
            "a non-RT run must not report a pass or a fail:\n{t}"
        );
    }

    #[test]
    fn json_is_parseable_and_carries_provenance() {
        let j = render_json(&a_report(), "abc1234");
        assert!(j.contains("\"schema_version\": 2"));
        assert!(j.contains("\"git_sha\": \"abc1234\""));
        assert!(j.contains("\"verdict\": null"));
        assert!(j.contains("\"acceptance_grade\": false"));
        assert!(j.contains("\"under_load\": false"));
        // Balanced braces/brackets is a cheap structural check on hand-rolled JSON.
        assert_eq!(
            j.chars().filter(|&c| c == '{').count(),
            j.chars().filter(|&c| c == '}').count(),
        );
        assert_eq!(
            j.chars().filter(|&c| c == '[').count(),
            j.chars().filter(|&c| c == ']').count(),
        );
    }

    #[test]
    fn a_platform_gap_and_the_cyclictest_gap_are_always_named_off_rt() {
        let j = render_json(&a_report(), "abc1234");
        assert!(j.contains("not confirmed PREEMPT_RT"));
        assert!(j.contains("this tool is not cyclictest"));
    }

    #[test]
    fn warnings_with_quotes_do_not_break_the_json() {
        let mut r = a_report();
        r.rt.warnings = vec!["a \"quoted\" \\ backslash\nnewline".to_string()];
        let j = render_json(&r, "x");
        assert!(j.contains("\\\"quoted\\\""));
        assert!(j.contains("\\\\ backslash\\nnewline"));
    }

    #[test]
    fn a_missing_kernel_release_serialises_as_null_not_empty_string() {
        let mut r = a_report();
        r.rt.kernel_release = None;
        assert!(render_json(&r, "x").contains("\"kernel_release\": null"));
    }
}
