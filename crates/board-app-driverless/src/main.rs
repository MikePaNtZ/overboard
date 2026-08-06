//! `board-app-driverless` wires `control-core` + `safety` to a `hal` +
//! `hal-actuate` backend and runs the ICD §5.2 control loop.
//!
//! This is the binary with motion authority (ICD §6.3, DR-MODE-1) — it links
//! `hal-actuate`. Its ridden counterpart is `board-app-ridden`, which cannot
//! link `hal-actuate` at all; `crates/xtask` gates that at the dependency
//! graph, not at review time.
//!
//! Today both `--backend sim` and `--backend null` map to the same
//! `sim-backend`, which steps the real MuJoCo onewheel plant through `hal`
//! (issue #107, I1c) — a real hardware backend is a later increment. This
//! binary exists to prove the seam and the wiring end to end: observe →
//! compute → clamp → apply, with `wait_observe()` the only call that
//! advances time. Its controller is still `control_core::Controller`'s
//! stub (always `Command::ZERO`), so this loop does not yet balance
//! anything — see `crates/control-core` for when the real law lands.

use board_types::Params;
use control_core::Controller;
use hal::BoardObserve;
use hal_actuate::BoardActuate;
use safety::Envelope;
use sim_backend::SimBackend;
use std::process::ExitCode;

const DEFAULT_CYCLES: u64 = 100;

struct Args {
    backend: String,
    cycles: u64,
}

fn print_help() {
    println!("board-app-driverless - Overboard control loop runner (motion authority)");
    println!();
    println!("USAGE:");
    println!("    board-app-driverless [OPTIONS]");
    println!();
    println!("OPTIONS:");
    println!("    --backend <sim|null>   Board I/O backend to use (default: sim).");
    println!("                           Both currently map to sim-backend, which steps");
    println!("                           the real MuJoCo onewheel plant through hal.");
    println!("    --cycles <N>           Number of fixed-step cycles to run (default: {DEFAULT_CYCLES}).");
    println!("    -h, --help             Print this help and exit.");
}

fn parse_args(mut argv: impl Iterator<Item = String>) -> Result<Args, String> {
    let mut backend = "sim".to_string();
    let mut cycles = DEFAULT_CYCLES;

    while let Some(arg) = argv.next() {
        match arg.as_str() {
            "--backend" => {
                backend = argv
                    .next()
                    .ok_or_else(|| "--backend requires a value (sim|null)".to_string())?;
            }
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

    if backend != "sim" && backend != "null" {
        return Err(format!(
            "--backend must be 'sim' or 'null', got '{backend}'"
        ));
    }

    Ok(Args { backend, cycles })
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
        "board-app-driverless starting: backend={} cycles={}",
        args.backend, args.cycles
    );

    // Placeholder limits so the clamp stages have something to enforce. Real
    // values are a Stage-0 output; these exist only so the envelope is not
    // trivially inert while the plant is still a stub.
    let params = Params {
        max_current_a: 60.0,
        max_abs_pitch_rad: 0.5,
        ..Params::default()
    };

    // Both "sim" and "null" currently resolve to the same stub backend.
    let mut backend = SimBackend::with_params(params);
    let mut controller = Controller::new(params);
    let mut envelope = Envelope::new(params);

    if let Err(e) = backend.open() {
        eprintln!("error: backend open failed: {e:?}");
        return ExitCode::FAILURE;
    }

    // The disarm handle is deliberately held for the whole run: it is the
    // capability a watchdog or panic hook would use, and dropping it here
    // would throw away the only safe-state path (ICD §9.2).
    let _disarm = match backend.arm() {
        Ok(h) => h,
        Err(e) => {
            eprintln!("error: backend arm failed: {e:?}");
            return ExitCode::FAILURE;
        }
    };
    envelope.arm();

    // The ICD §5.2 loop, in order: observe (advances time) -> compute (pure)
    // -> apply (enqueues, does not advance time).
    for i in 0..args.cycles {
        let obs = match backend.wait_observe() {
            Ok(obs) => obs,
            // Transient means skip this cycle and keep running; Fatal and a
            // protocol violation both end the run.
            Err(board_types::IoError::Transient) => continue,
            Err(e) => {
                eprintln!("error: wait_observe failed at cycle {i}: {e:?}");
                return ExitCode::FAILURE;
            }
        };

        let proposed = controller.update(&obs);
        let (cmd, envelope_saturated) =
            envelope.apply(proposed, board_types::Faults(obs.fault_word));

        let applied = match backend.apply(&cmd) {
            Ok(a) => a,
            Err(e) => {
                eprintln!("error: apply failed at cycle {i}: {e:?}");
                return ExitCode::FAILURE;
            }
        };

        if i % 20 == 0 || i == args.cycles.saturating_sub(1) {
            println!(
                "heartbeat: cycle={} t_recv_ns={} imu_samples={} measured={:.3}A \
                 commanded={:?} sat={:?}/{:?}",
                obs.cycle,
                obs.t_recv_ns,
                obs.imu_count,
                obs.motor_current_a,
                applied.commanded,
                envelope_saturated,
                applied.saturated,
            );
        }
    }

    if let Err(e) = backend.close() {
        eprintln!("error: backend close failed: {e:?}");
        return ExitCode::FAILURE;
    }

    println!(
        "board-app-driverless: completed {} cycles, exiting cleanly",
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
        assert_eq!(args.backend, "sim");
        assert_eq!(args.cycles, DEFAULT_CYCLES);
    }

    #[test]
    fn parse_args_rejects_unknown_backend() {
        let argv = vec!["--backend".to_string(), "bogus".to_string()];
        assert!(parse_args(argv.into_iter()).is_err());
    }

    #[test]
    fn parse_args_accepts_cycles() {
        let argv = vec!["--cycles".to_string(), "5".to_string()];
        let args = parse_args(argv.into_iter()).unwrap();
        assert_eq!(args.cycles, 5);
    }
}
