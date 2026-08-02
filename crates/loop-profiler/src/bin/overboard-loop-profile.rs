//! CLI wrapper around [`loop_profiler::profile::run`].
//!
//! Hand-rolled argument parsing, matching `board-app-driverless` and every
//! other binary in this workspace -- the project carries no `clap`
//! dependency and six flags do not justify introducing one.

use loop_profiler::profile::{self, Config, DEFAULT_PERIOD_NS};
use loop_profiler::report;
use std::process::ExitCode;

struct Args {
    cfg: Config,
    json_path: Option<String>,
    /// Exit non-zero if the run was acceptance-grade and failed AC-5.
    strict: bool,
}

fn print_help() {
    let d = Config::default();
    println!("overboard-loop-profile - can this host hold the control loop?");
    println!();
    println!("Measures scheduler wake latency against an absolute deadline, and the");
    println!("cost of one pass through the real control law (estimator -> regulator ->");
    println!("feedforward -> envelope). No CAN, no motor, no physics: this binary has");
    println!("no actuation path and cannot command hardware.");
    println!();
    println!("USAGE:");
    println!("    overboard-loop-profile [OPTIONS]");
    println!();
    println!("OPTIONS:");
    println!(
        "    --rate-hz <HZ>     Loop rate (default: {:.0}).",
        1e9 / DEFAULT_PERIOD_NS as f64
    );
    println!(
        "    --cycles <N>       Counted cycles (default: {}).",
        d.cycles
    );
    println!(
        "    --warmup <N>       Discarded cycles first (default: {}).",
        d.warmup
    );
    println!(
        "    --rt-prio <P>      SCHED_FIFO priority (default: {}).",
        d.rt_prio.unwrap_or(0)
    );
    println!("    --no-rt            Stay at normal priority. Numbers describe a");
    println!("                       fair-share scheduler, not a real-time one.");
    println!("    --json <PATH>      Also write the machine-readable report.");
    println!("    --strict           Exit 1 if an acceptance-grade run fails AC-5.");
    println!("    -h, --help         Print this help and exit.");
    println!();
    println!("At the default rate a 100000-cycle run takes about 200 seconds.");
    println!("Run cyclictest alongside this, not instead of it: that measures the");
    println!("kernel, this measures our loop on it.");
}

fn parse_args(mut argv: impl Iterator<Item = String>) -> Result<Args, String> {
    let mut cfg = Config::default();
    let mut json_path = None;
    let mut strict = false;

    fn next_val(argv: &mut impl Iterator<Item = String>, flag: &str) -> Result<String, String> {
        argv.next()
            .ok_or_else(|| format!("{flag} requires a value"))
    }

    while let Some(arg) = argv.next() {
        match arg.as_str() {
            "--rate-hz" => {
                let v = next_val(&mut argv, "--rate-hz")?;
                let hz: f64 = v
                    .parse()
                    .map_err(|_| format!("--rate-hz: invalid number '{v}'"))?;
                if !(hz.is_finite() && hz > 0.0) {
                    return Err(format!("--rate-hz must be positive and finite, got '{v}'"));
                }
                cfg.period_ns = (1e9 / hz).round() as u64;
                if cfg.period_ns == 0 {
                    return Err(format!("--rate-hz {hz} rounds to a zero-length period"));
                }
            }
            "--cycles" => {
                let v = next_val(&mut argv, "--cycles")?;
                cfg.cycles = v
                    .parse()
                    .map_err(|_| format!("--cycles: invalid number '{v}'"))?;
                if cfg.cycles == 0 {
                    return Err("--cycles must be at least 1".to_string());
                }
            }
            "--warmup" => {
                let v = next_val(&mut argv, "--warmup")?;
                cfg.warmup = v
                    .parse()
                    .map_err(|_| format!("--warmup: invalid number '{v}'"))?;
            }
            "--rt-prio" => {
                let v = next_val(&mut argv, "--rt-prio")?;
                let p: i32 = v
                    .parse()
                    .map_err(|_| format!("--rt-prio: invalid number '{v}'"))?;
                // 1..=99 is the POSIX SCHED_FIFO range on Linux. Rejecting
                // out-of-range here gives a clear message instead of an
                // EINVAL buried in a warning line.
                if !(1..=99).contains(&p) {
                    return Err(format!("--rt-prio must be 1..=99, got {p}"));
                }
                cfg.rt_prio = Some(p);
            }
            "--no-rt" => cfg.rt_prio = None,
            "--json" => json_path = Some(next_val(&mut argv, "--json")?),
            "--strict" => strict = true,
            "-h" | "--help" => {
                print_help();
                std::process::exit(0);
            }
            other => return Err(format!("unrecognized argument '{other}'")),
        }
    }

    Ok(Args {
        cfg,
        json_path,
        strict,
    })
}

fn main() -> ExitCode {
    let args = match parse_args(std::env::args().skip(1)) {
        Ok(a) => a,
        Err(msg) => {
            eprintln!("error: {msg}");
            print_help();
            return ExitCode::FAILURE;
        }
    };

    let r = profile::run(args.cfg);
    print!("{}", report::render_text(&r));

    if let Some(path) = args.json_path {
        let json = report::render_json(&r, &report::git_sha());
        if let Err(e) = std::fs::write(&path, json) {
            eprintln!("error: could not write {path}: {e}");
            return ExitCode::FAILURE;
        }
        println!("  wrote {path}");
    }

    // A withheld verdict is not a failure: on a laptop or a non-RT CI runner
    // there is nothing to fail, and making --strict red there would mean
    // nobody could use it in CI at all.
    if args.strict && r.ac5_verdict() == Some(false) {
        return ExitCode::FAILURE;
    }
    ExitCode::SUCCESS
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(args: &[&str]) -> Result<Args, String> {
        parse_args(args.iter().map(|s| s.to_string()))
    }

    #[test]
    fn defaults_are_the_icd_control_rate() {
        let a = parse(&[]).unwrap();
        assert_eq!(a.cfg.period_ns, DEFAULT_PERIOD_NS);
        assert_eq!(a.cfg.period_ns, 2_000_000, "ICD 11.2: 500 Hz");
    }

    #[test]
    fn rate_hz_converts_to_a_period() {
        let a = parse(&["--rate-hz", "1000"]).unwrap();
        assert_eq!(a.cfg.period_ns, 1_000_000);
    }

    #[test]
    fn a_zero_or_negative_rate_is_rejected_rather_than_dividing_by_zero() {
        assert!(parse(&["--rate-hz", "0"]).is_err());
        assert!(parse(&["--rate-hz", "-5"]).is_err());
        assert!(parse(&["--rate-hz", "nonsense"]).is_err());
    }

    #[test]
    fn zero_cycles_is_rejected() {
        // Otherwise every percentile is a meaningless zero and the report
        // reads like a clean run.
        assert!(parse(&["--cycles", "0"]).is_err());
    }

    #[test]
    fn an_out_of_range_rt_priority_is_rejected_with_a_message() {
        assert!(parse(&["--rt-prio", "0"]).is_err());
        assert!(parse(&["--rt-prio", "100"]).is_err());
        assert_eq!(parse(&["--rt-prio", "80"]).unwrap().cfg.rt_prio, Some(80));
    }

    #[test]
    fn no_rt_clears_the_priority() {
        assert_eq!(parse(&["--no-rt"]).unwrap().cfg.rt_prio, None);
    }

    #[test]
    fn a_flag_missing_its_value_errors_rather_than_silently_defaulting() {
        assert!(parse(&["--cycles"]).is_err());
        assert!(parse(&["--json"]).is_err());
    }

    #[test]
    fn unknown_flags_are_rejected() {
        assert!(parse(&["--profile-everything"]).is_err());
    }

    #[test]
    fn json_and_strict_are_parsed() {
        let a = parse(&["--json", "/tmp/x.json", "--strict"]).unwrap();
        assert_eq!(a.json_path.as_deref(), Some("/tmp/x.json"));
        assert!(a.strict);
    }
}
