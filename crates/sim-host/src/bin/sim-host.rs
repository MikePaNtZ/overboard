//! Thin binary wrapper over the `sim_host` library (issue #161, SR-GAME-15).
//!
//! Spawns the 500 Hz control loop on its own dedicated thread and joins it
//! from `main` -- so the actual loop is never on the process's main thread,
//! and this binary itself does no work beyond argument parsing and printing
//! a final summary when the loop stops.
//!
//! ```text
//! sim-host [--duration-secs SECONDS]
//! ```
//! With no `--duration-secs`, runs forever (Ctrl-C / SIGTERM to stop). With
//! it, stops after that many seconds and exits -- used for the issue #161
//! verification run and for anything else that wants a bounded process.

use sim_host::HostConfig;
use std::process::ExitCode;
use std::time::Duration;

fn main() -> ExitCode {
    let mut cfg = HostConfig::default();

    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--duration-secs" => {
                let Some(v) = args.get(i + 1) else {
                    eprintln!("sim-host: --duration-secs needs a value");
                    return ExitCode::FAILURE;
                };
                let Ok(secs) = v.parse::<f64>() else {
                    eprintln!("sim-host: --duration-secs value '{v}' is not a number");
                    return ExitCode::FAILURE;
                };
                cfg.duration = Some(Duration::from_secs_f64(secs));
                i += 2;
            }
            other => {
                eprintln!("sim-host: unrecognized argument '{other}'");
                return ExitCode::FAILURE;
            }
        }
    }

    eprintln!(
        "sim-host: starting -- state out to {}, input in on {}, duration={:?}",
        cfg.state_out_addr, cfg.input_in_addr, cfg.duration
    );

    let handle = sim_host::spawn(cfg);
    match handle.join() {
        Ok(Ok(summary)) => {
            eprintln!(
                "sim-host: stopped cleanly -- {} ticks, {} missed deadlines",
                summary.ticks, summary.missed_deadlines
            );
            ExitCode::SUCCESS
        }
        Ok(Err(e)) => {
            eprintln!("sim-host: control loop failed: {e}");
            ExitCode::FAILURE
        }
        Err(_) => {
            eprintln!("sim-host: control thread panicked");
            ExitCode::FAILURE
        }
    }
}
