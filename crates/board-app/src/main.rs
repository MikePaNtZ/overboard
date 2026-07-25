//! `board-app` wires `control-core` + `safety` to a `hal::BoardIo` backend
//! and runs a fixed-step loop.
//!
//! Today both `--backend sim` and `--backend null` map to the same
//! `sim-backend` stub (synthetic zeros, incrementing clock) — MuJoCo FFI and
//! a real hardware backend are future milestones (see `sim-backend`'s
//! module docs and TODOs). This binary exists to prove the seam (`hal`) and
//! the wiring (control-core -> safety -> backend) end-to-end.

use board_types::Params;
use control_core::Controller;
use hal::BoardIo;
use safety::Envelope;
use sim_backend::SimBackend;
use std::process::ExitCode;

const DEFAULT_CYCLES: u64 = 100;

struct Args {
    backend: String,
    cycles: u64,
}

fn print_help() {
    println!("board-app - Overboard control loop runner");
    println!();
    println!("USAGE:");
    println!("    board-app [OPTIONS]");
    println!();
    println!("OPTIONS:");
    println!("    --backend <sim|null>   Board I/O backend to use (default: sim).");
    println!("                           Both currently map to the sim-backend stub;");
    println!("                           MuJoCo FFI is a future milestone.");
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
        "board-app starting: backend={} cycles={}",
        args.backend, args.cycles
    );

    // Both "sim" and "null" currently resolve to the same stub backend.
    let mut backend = SimBackend::new();
    let mut controller = Controller::new(Params::default());
    let mut envelope = Envelope::new(Params::default());
    envelope.arm();

    let mut cmd = board_types::Command::ZERO;
    for i in 0..args.cycles {
        let obs = backend.cycle(&cmd);
        let raw_cmd = controller.step(&obs);
        cmd = envelope.apply(raw_cmd, board_types::Faults(obs.fault_word));

        if i % 20 == 0 || i == args.cycles.saturating_sub(1) {
            println!(
                "heartbeat: cycle={} t_ns={} applied_current={:.3}A",
                obs.cycle, obs.timestamp_ns, obs.applied_current.0
            );
        }
    }

    println!(
        "board-app: completed {} cycles, exiting cleanly",
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
