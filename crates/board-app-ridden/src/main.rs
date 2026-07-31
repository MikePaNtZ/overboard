//! `board-app-ridden` wires `control-core` + `safety` to a `hal` backend and
//! runs the observe-half of the ICD §5.2 loop: **observe → compute →
//! clamp → discard**.
//!
//! This binary has zero motion authority, and that is a **build-proven**
//! property, not an assertion a reviewer has to re-check every time: it does
//! not link `hal-actuate`, so there is no `apply()` to call even if a future
//! edit tried to. `crates/xtask`'s gate double-checks the same thing at the
//! dependency-graph level, so the guarantee survives `cargo build
//! --all-features` and a transitive dependency, not just a careful diff.
//!
//! [`ShadowBackend`] is a placeholder: there is no rider, no hardware and no
//! real ridden observe backend yet (Track C, procurement-gated). It produces
//! synthetic observations purely to exercise the loop shape end to end. The
//! clamp stage runs and its result is printed, then discarded — there is
//! nowhere in this binary to send it.

use board_types::{
    IoError, Observation, Params, Profile, RunMetadata, ValidityFlags, DEFAULT_R_EFF_M,
};
use control_core::Controller;
use hal::BoardObserve;
use safety::Envelope;
use std::process::ExitCode;

const DEFAULT_CYCLES: u64 = 100;
/// Nanoseconds per control cycle. 500 Hz, per ICD §11.2.
const CYCLE_NS: u64 = 2_000_000;

/// Observe-only placeholder. Produces synthetic observations so the shadow
/// loop has something to run against; not a plant and not a hardware driver.
#[derive(Debug, Default)]
struct ShadowBackend {
    cycle: u64,
    t_ns: u64,
    open: bool,
}

impl BoardObserve for ShadowBackend {
    fn open(&mut self) -> Result<(), IoError> {
        self.open = true;
        Ok(())
    }

    fn close(&mut self) -> Result<(), IoError> {
        self.open = false;
        Ok(())
    }

    fn wait_observe(&mut self) -> Result<Observation, IoError> {
        if !self.open {
            return Err(IoError::ProtocolViolation);
        }
        self.cycle += 1;
        self.t_ns += CYCLE_NS;

        let mut obs = Observation::COLD_START;
        obs.cycle = self.cycle;
        obs.t_recv_ns = self.t_ns;
        obs.validity = ValidityFlags::ALL_FRESH;
        Ok(obs)
    }

    fn run_metadata(&self) -> RunMetadata {
        RunMetadata {
            icd_version: (0, 3),
            profile: Profile::Observe,
            control_rate_hz: 1e9 / CYCLE_NS as f32,
            params: Params::default(),
            imu_mounting_rotation: [1.0, 0.0, 0.0, 0.0],
            r_eff_m: DEFAULT_R_EFF_M,
            imperfection_profile_id: None,
            schema_hash: [0; 32],
            binary_hash: [0; 32],
            git_sha: [0; 20],
        }
    }
}

struct Args {
    cycles: u64,
}

fn print_help() {
    println!("board-app-ridden - Overboard shadow-mode loop runner (no motion authority)");
    println!();
    println!("USAGE:");
    println!("    board-app-ridden [OPTIONS]");
    println!();
    println!("OPTIONS:");
    println!("    --cycles <N>   Number of fixed-step cycles to run (default: {DEFAULT_CYCLES}).");
    println!("    -h, --help     Print this help and exit.");
}

fn parse_args(mut argv: impl Iterator<Item = String>) -> Result<Args, String> {
    let mut cycles = DEFAULT_CYCLES;

    while let Some(arg) = argv.next() {
        match arg.as_str() {
            "--cycles" => {
                let val = argv
                    .next()
                    .ok_or_else(|| "--cycles requires a value".to_string())?;
                cycles = val
                    .parse()
                    .map_err(|_| format!("--cycles: invalid number '{val}'"))?;
            }
            "-h" | "--help" => {
                print_help();
                std::process::exit(0);
            }
            other => return Err(format!("unrecognized argument '{other}'")),
        }
    }

    Ok(Args { cycles })
}

fn main() -> ExitCode {
    let args = match parse_args(std::env::args().skip(1)) {
        Ok(args) => args,
        Err(msg) => {
            eprintln!("error: {msg}");
            print_help();
            return ExitCode::FAILURE;
        }
    };

    println!(
        "board-app-ridden starting: cycles={} (no motion authority)",
        args.cycles
    );

    let params = Params {
        max_current_a: 40.0,
        max_abs_pitch_rad: 0.5,
        ..Params::default()
    };

    let mut backend = ShadowBackend::default();
    let mut controller = Controller::new(params);
    // Armed so the clamp stage behaves the same as it would on the driverless
    // path -- there is nothing downstream to send the result to either way.
    let mut envelope = Envelope::new(params);
    envelope.arm();

    if let Err(e) = backend.open() {
        eprintln!("error: backend open failed: {e:?}");
        return ExitCode::FAILURE;
    }

    for i in 0..args.cycles {
        let obs = match backend.wait_observe() {
            Ok(obs) => obs,
            Err(board_types::IoError::Transient) => continue,
            Err(e) => {
                eprintln!("error: wait_observe failed at cycle {i}: {e:?}");
                return ExitCode::FAILURE;
            }
        };

        let proposed = controller.update(&obs);
        // Computed and clamped, same as the driverless path -- but there is no
        // `apply()` in scope to send it to. This binary cannot actuate.
        let (cmd, saturated) = envelope.apply(proposed, board_types::Faults(obs.fault_word));

        if i % 20 == 0 || i == args.cycles.saturating_sub(1) {
            println!(
                "heartbeat: cycle={} t_recv_ns={} would_command={cmd:?} sat={saturated:?} (discarded, no apply() in this binary)",
                obs.cycle, obs.t_recv_ns,
            );
        }
    }

    if let Err(e) = backend.close() {
        eprintln!("error: backend close failed: {e:?}");
        return ExitCode::FAILURE;
    }

    println!(
        "board-app-ridden: completed {} cycles, exiting cleanly",
        args.cycles
    );
    ExitCode::SUCCESS
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_args_defaults() {
        let args = parse_args(std::iter::empty()).unwrap();
        assert_eq!(args.cycles, DEFAULT_CYCLES);
    }

    #[test]
    fn parse_args_accepts_cycles() {
        let argv = vec!["--cycles".to_string(), "5".to_string()];
        let args = parse_args(argv.into_iter()).unwrap();
        assert_eq!(args.cycles, 5);
    }

    #[test]
    fn parse_args_rejects_unknown_flag() {
        let argv = vec!["--backend".to_string(), "sim".to_string()];
        assert!(parse_args(argv.into_iter()).is_err());
    }

    #[test]
    fn shadow_backend_advances_cycle_and_clock() {
        let mut b = ShadowBackend::default();
        b.open().unwrap();
        let o1 = b.wait_observe().unwrap();
        let o2 = b.wait_observe().unwrap();
        assert_eq!(o1.cycle, 1);
        assert_eq!(o2.cycle, 2);
        assert_eq!(o2.t_recv_ns - o1.t_recv_ns, CYCLE_NS);
    }

    #[test]
    fn shadow_backend_refuses_observe_before_open() {
        let mut b = ShadowBackend::default();
        assert_eq!(b.wait_observe(), Err(IoError::ProtocolViolation));
    }

    #[test]
    fn shadow_backend_reports_observe_only_profile() {
        let b = ShadowBackend::default();
        assert_eq!(b.run_metadata().profile, Profile::Observe);
    }
}
