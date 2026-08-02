//! Thin binary wrapper over the `sim_host` library (issue #161, SR-GAME-15).
//!
//! Spawns the 500 Hz control loop on its own dedicated thread and joins it
//! from `main` -- so the actual loop is never on the process's main thread,
//! and this binary itself does no work beyond argument parsing and printing
//! a final summary when the loop stops.
//!
//! ```text
//! sim-host [--duration-secs SECONDS] [--startup-kick]
//!          [--state-out-addr ADDR] [--input-in-addr ADDR]
//! ```
//! With no `--duration-secs`, runs forever (Ctrl-C / SIGTERM to stop). With
//! it, stops after that many seconds and exits -- used for verification runs
//! and anything else that wants a bounded process.
//!
//! `--startup-kick` is OFF by default (issue #169): a normal run must not
//! shove the board before a player has touched anything. Pass it explicitly
//! for a `wire-probe` diagnostic run that wants a guaranteed disturbance to
//! show recovery from.
//!
//! `--state-out-addr`/`--input-in-addr` override the documented
//! `127.0.0.1:9601`/`:9602` (still the default -- Unreal always talks to
//! those). Exists so a verification run can avoid the standard ports
//! entirely rather than colliding with a live capture session already
//! bound to them -- learned the hard way (issue #161 follow-up) when a
//! verification `sim-host` ran its full course straight into a live
//! `UnrealEditor` capture that was already listening on 9601.

use sim_host::HostConfig;
use std::net::SocketAddr;
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
            "--startup-kick" => {
                cfg.startup_kick = true;
                i += 1;
            }
            "--state-out-addr" => {
                let Some(v) = args.get(i + 1) else {
                    eprintln!("sim-host: --state-out-addr needs a value");
                    return ExitCode::FAILURE;
                };
                let Ok(addr) = v.parse::<SocketAddr>() else {
                    eprintln!("sim-host: --state-out-addr value '{v}' is not an address");
                    return ExitCode::FAILURE;
                };
                cfg.state_out_addr = addr;
                i += 2;
            }
            "--input-in-addr" => {
                let Some(v) = args.get(i + 1) else {
                    eprintln!("sim-host: --input-in-addr needs a value");
                    return ExitCode::FAILURE;
                };
                let Ok(addr) = v.parse::<SocketAddr>() else {
                    eprintln!("sim-host: --input-in-addr value '{v}' is not an address");
                    return ExitCode::FAILURE;
                };
                cfg.input_in_addr = addr;
                i += 2;
            }
            other => {
                eprintln!("sim-host: unrecognized argument '{other}'");
                return ExitCode::FAILURE;
            }
        }
    }

    eprintln!(
        "sim-host: starting -- state out to {}, input in on {}, duration={:?}, startup_kick={}",
        cfg.state_out_addr, cfg.input_in_addr, cfg.duration, cfg.startup_kick
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
