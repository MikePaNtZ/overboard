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
//!          [--scripted-scenario default|s-curve|full-stick|stick-reversal|
//!                              brake-stop|brake-turn|cruise-turn]
//!          [--stats-path PATH|none]
//!          [--free-run] [--max-sim-secs SECONDS]
//!          [--pitch-source estimator|truth] [--pitch-bias-deg DEGREES]
//!          [--cmd-reserve FRACTION] [--cmd-reserve-braking FRACTION]
//!          [--incline-deg DEGREES] [--crr-scale FACTOR]
//!          [--imu-noise-seed SEED]
//!          [--disturbance t0,dur,fx,fy,fz,tx,ty,tz] [--trace-csv PATH]
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
            // Verification only -- see HostConfig::scripted_scenario. Plays
            // one of `sim_host::scenario`'s schedules from inside the host,
            // indexed on SIMULATED time, instead of taking stick values off
            // the socket. Deterministic and repeatable; NOT how a deployed
            // host or an Unreal session runs.
            "--scripted-scenario" => {
                let Some(v) = args.get(i + 1) else {
                    eprintln!("sim-host: --scripted-scenario needs a value");
                    return ExitCode::FAILURE;
                };
                let Some(sched) = sim_host::scenario::by_name(v) else {
                    eprintln!(
                        "sim-host: unknown scenario '{v}' (want: {})",
                        sim_host::scenario::names()
                    );
                    return ExitCode::FAILURE;
                };
                cfg.scripted_scenario = Some(sched);
                i += 2;
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
            // Companion to --state-out-addr/--input-in-addr: an isolated
            // verification run must not stomp a live session's stats file
            // either. `sim-host`'s own counters go to a single fixed path by
            // default, so two hosts running at once (a capture session on
            // 9601/9602 and a verification run on throwaway ports) silently
            // overwrite each other's tick/missed-deadline counts.
            "--stats-path" => {
                let Some(v) = args.get(i + 1) else {
                    eprintln!("sim-host: --stats-path needs a value");
                    return ExitCode::FAILURE;
                };
                cfg.stats_path = if v == "none" {
                    None
                } else {
                    Some(std::path::PathBuf::from(v))
                };
                i += 2;
            }
            // --- ADR-0011 acceptance instrumentation -------------------
            // All verification-only, all documented on HostConfig. Their
            // reason for existing on the command line rather than in a test
            // fixture is that an acceptance number nobody else can re-take
            // is not a measurement, it is an assertion.
            "--free-run" => {
                cfg.free_run = true;
                i += 1;
            }
            "--max-sim-secs" => {
                let Some(v) = args.get(i + 1) else {
                    eprintln!("sim-host: --max-sim-secs needs a value");
                    return ExitCode::FAILURE;
                };
                let Ok(secs) = v.parse::<f64>() else {
                    eprintln!("sim-host: --max-sim-secs value '{v}' is not a number");
                    return ExitCode::FAILURE;
                };
                cfg.max_sim_time_s = Some(secs);
                i += 2;
            }
            // ADR-0011 criterion (f): "every pass must hold with the
            // estimator bias removed". `truth` is the measured-WORSE case on
            // this plant, not the better one -- see HostConfig::pitch_source.
            "--pitch-source" => {
                let Some(v) = args.get(i + 1) else {
                    eprintln!("sim-host: --pitch-source needs a value");
                    return ExitCode::FAILURE;
                };
                cfg.pitch_source = match v.as_str() {
                    "estimator" => sim_host::host::PitchSource::Estimator,
                    "truth" => sim_host::host::PitchSource::PlantTruth,
                    other => {
                        eprintln!(
                            "sim-host: unknown --pitch-source '{other}' (want: estimator, truth)"
                        );
                        return ExitCode::FAILURE;
                    }
                };
                i += 2;
            }
            // ADR-0011 criterion (f), the robustness form: a worst-case
            // STATIC estimator error injected on top of the real signal
            // path. See HostConfig::pitch_bias_deg for why this is a
            // different question from --pitch-source truth.
            "--pitch-bias-deg" => {
                let Some(v) = args.get(i + 1) else {
                    eprintln!("sim-host: --pitch-bias-deg needs a value");
                    return ExitCode::FAILURE;
                };
                let Ok(deg) = v.parse::<f32>() else {
                    eprintln!("sim-host: --pitch-bias-deg value '{v}' is not a number");
                    return ExitCode::FAILURE;
                };
                cfg.pitch_bias_deg = deg;
                i += 2;
            }
            // Overrides CMD_ENVELOPE_RESERVE. Sweeping this IS that
            // constant's provenance measurement; see its doc comment.
            "--cmd-reserve" => {
                let Some(v) = args.get(i + 1) else {
                    eprintln!("sim-host: --cmd-reserve needs a value");
                    return ExitCode::FAILURE;
                };
                let Ok(scale) = v.parse::<f32>() else {
                    eprintln!("sim-host: --cmd-reserve value '{v}' is not a number");
                    return ExitCode::FAILURE;
                };
                if !(0.0..=1.0).contains(&scale) {
                    eprintln!("sim-host: --cmd-reserve must be within [0, 1], got {scale}");
                    return ExitCode::FAILURE;
                }
                cfg.cmd_envelope_reserve = Some(scale);
                i += 2;
            }
            // Overrides CMD_ENVELOPE_RESERVE_BRAKING -- the reserve spent
            // when the stick opposes the motion. Sweeping this measures what
            // braking authority costs at ADR-0011's worst matrix point.
            "--cmd-reserve-braking" => {
                let Some(v) = args.get(i + 1) else {
                    eprintln!("sim-host: --cmd-reserve-braking needs a value");
                    return ExitCode::FAILURE;
                };
                let Ok(scale) = v.parse::<f32>() else {
                    eprintln!("sim-host: --cmd-reserve-braking value '{v}' is not a number");
                    return ExitCode::FAILURE;
                };
                if !(0.0..=1.0).contains(&scale) {
                    eprintln!("sim-host: --cmd-reserve-braking must be within [0, 1], got {scale}");
                    return ExitCode::FAILURE;
                }
                cfg.cmd_envelope_reserve_braking = Some(scale);
                i += 2;
            }
            // A constant ground slope, degrees, positive uphill. ADR-0011's
            // second ratification needs the tolerated incline as a MEASURED
            // number before the authored world can carry a checkable asset
            // rule; see HostConfig::incline_deg.
            "--incline-deg" => {
                let Some(v) = args.get(i + 1) else {
                    eprintln!("sim-host: --incline-deg needs a value");
                    return ExitCode::FAILURE;
                };
                let Ok(deg) = v.parse::<f64>() else {
                    eprintln!("sim-host: --incline-deg value '{v}' is not a number");
                    return ExitCode::FAILURE;
                };
                if !(-45.0..=45.0).contains(&deg) {
                    eprintln!("sim-host: --incline-deg must be within [-45, 45], got {deg}");
                    return ExitCode::FAILURE;
                }
                cfg.incline_deg = deg;
                i += 2;
            }
            // ADR-0011 criterion (g), re-derived (issue: drag-model): the
            // wheel-hinge damping="0.08" sweep this criterion originally
            // specified no longer applies -- that lumped, undocumented term
            // was replaced by a derived `frictionloss`/`damping` pair, and
            // the remaining physical uncertainty lives in Crr. A scale
            // rather than a literal, so the same flag stays correct if the
            // MJCF's own constant ever moves -- see `HostConfig::crr_scale`.
            "--crr-scale" => {
                let Some(v) = args.get(i + 1) else {
                    eprintln!("sim-host: --crr-scale needs a value");
                    return ExitCode::FAILURE;
                };
                let Ok(scale) = v.parse::<f64>() else {
                    eprintln!("sim-host: --crr-scale value '{v}' is not a number");
                    return ExitCode::FAILURE;
                };
                if scale.is_nan() || scale <= 0.0 {
                    eprintln!("sim-host: --crr-scale must be > 0, got {scale}");
                    return ExitCode::FAILURE;
                }
                cfg.crr_scale = scale;
                i += 2;
            }
            // The datasheet-derived sensor-noise model
            // (`sim_backend::imperfections::IMU_NOISE_DATASHEET_V1`),
            // seeded and enabled -- see `HostConfig::imu_noise_seed`. No
            // flag, no noise: this is the one knob in this list that
            // changes the SENSOR chain rather than the plant or the
            // envelope, and it stays off by default for the same reason
            // every other verification knob does.
            "--imu-noise-seed" => {
                let Some(v) = args.get(i + 1) else {
                    eprintln!("sim-host: --imu-noise-seed needs a value");
                    return ExitCode::FAILURE;
                };
                let Ok(seed) = v.parse::<u64>() else {
                    eprintln!("sim-host: --imu-noise-seed value '{v}' is not a u64");
                    return ExitCode::FAILURE;
                };
                cfg.imu_noise_seed = Some(seed);
                i += 2;
            }
            // `t0,duration,fx,fy,fz,tx,ty,tz` -- world frame, SI. The kerb
            // strike of ADR-0011's criterion (a) matrix is fed in this way
            // rather than derived here; see HostConfig::disturbance for why
            // the derivation stays in `sim/scenarios/plant.py`.
            "--disturbance" => {
                let Some(v) = args.get(i + 1) else {
                    eprintln!("sim-host: --disturbance needs a value");
                    return ExitCode::FAILURE;
                };
                let parts: Result<Vec<f64>, _> =
                    v.split(',').map(|p| p.trim().parse::<f64>()).collect();
                let Ok(p) = parts else {
                    eprintln!("sim-host: --disturbance '{v}' has a non-numeric field");
                    return ExitCode::FAILURE;
                };
                if p.len() != 8 {
                    eprintln!(
                        "sim-host: --disturbance wants 8 comma-separated numbers \
                         (t0,duration,fx,fy,fz,tx,ty,tz), got {}",
                        p.len()
                    );
                    return ExitCode::FAILURE;
                }
                cfg.disturbance = Some(sim_host::host::Disturbance {
                    t0_s: p[0],
                    duration_s: p[1],
                    force_n: [p[2], p[3], p[4]],
                    torque_nm: [p[5], p[6], p[7]],
                });
                i += 2;
            }
            "--trace-csv" => {
                let Some(v) = args.get(i + 1) else {
                    eprintln!("sim-host: --trace-csv needs a value");
                    return ExitCode::FAILURE;
                };
                cfg.trace_path = Some(std::path::PathBuf::from(v));
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
        "sim-host: starting -- state out to {}, input in on {}, duration={:?}, \
         startup_kick={}, scripted={}, imu_noise_seed={:?}",
        cfg.state_out_addr,
        cfg.input_in_addr,
        cfg.duration,
        cfg.startup_kick,
        cfg.scripted_scenario.is_some(),
        cfg.imu_noise_seed
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
