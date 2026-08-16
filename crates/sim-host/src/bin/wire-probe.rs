//! The CEO-visible verification tool for the issue #161 wire (W1).
//!
//! A headless UDP receiver, independent of the `sim-host` LIBRARY (it only
//! uses [`sim_host::wire`]) -- it proves the wire works from the outside,
//! the same position Unreal will be in, not from inside the process that
//! produces it.
//!
//! ```text
//! wire-probe [--seconds N] [--bind ADDR] [--host-stats PATH] [--csv PATH]
//! ```
//! Binds `--bind` (default `127.0.0.1:9601`, the documented state-out
//! address), listens for `--seconds` (default 10), and prints a plain-text
//! report: measured tick rate, inter-packet interval stats, the host's own
//! missed-deadline count (read from `--host-stats`, best-effort -- issue
//! #161's wire itself carries no room for that counter), pitch_rad
//! min/max/final, and a magic+version parse confirmation.
//!
//! `--csv PATH` (issue #161 W2 / #169) additionally writes one row per
//! received packet -- seq, sim_time_s, pos_x/y, pitch/yaw/roll_rad,
//! wheel_rate_rad_s, motor_current_a -- because "the board balances" is no
//! longer the claim W2 needs evidence for; "speed responds to lean" is, and
//! that needs a real trace, not a min/max/final summary. `pos_x`/`pos_y` are
//! `sim-host`'s dead-reckoned game path, NOT raw MuJoCo x/y -- see
//! `sim_host::wire::StateOut::pos`'s doc comment.

use sim_host::wire::{StateOut, STATE_FLAG_FALLEN, STATE_MAGIC, STATE_SCHEMA_VERSION};
use std::net::UdpSocket;
use std::path::PathBuf;
use std::time::{Duration, Instant};

struct Args {
    seconds: f64,
    bind: String,
    host_stats: PathBuf,
    csv: Option<PathBuf>,
}

impl Default for Args {
    fn default() -> Self {
        Args {
            seconds: 10.0,
            bind: sim_host::wire::STATE_OUT_ADDR.to_string(),
            host_stats: PathBuf::from(sim_host::host::DEFAULT_STATS_PATH),
            csv: None,
        }
    }
}

fn parse_args() -> Result<Args, String> {
    let mut a = Args::default();
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--seconds" => {
                let v = args.get(i + 1).ok_or("--seconds needs a value")?;
                a.seconds = v
                    .parse()
                    .map_err(|_| format!("bad --seconds value '{v}'"))?;
                i += 2;
            }
            "--bind" => {
                a.bind = args.get(i + 1).ok_or("--bind needs a value")?.clone();
                i += 2;
            }
            "--host-stats" => {
                a.host_stats = PathBuf::from(args.get(i + 1).ok_or("--host-stats needs a value")?);
                i += 2;
            }
            "--csv" => {
                a.csv = Some(PathBuf::from(args.get(i + 1).ok_or("--csv needs a value")?));
                i += 2;
            }
            other => return Err(format!("unrecognized argument '{other}'")),
        }
    }
    Ok(a)
}

/// One `--csv` output row -- see the module doc comment.
struct CsvRow {
    seq: u64,
    sim_time_s: f64,
    pos_x_m: f32,
    pos_y_m: f32,
    pitch_rad: f32,
    yaw_rad: f32,
    wheel_rate_rad_s: f32,
    motor_current_a: f32,
    /// v2: ACTUAL ballast joint position, metres -- see
    /// `sim_host::wire::StateOut::rider_fore_aft_m`'s doc comment.
    rider_fore_aft_m: f32,
    rider_lateral_m: f32,
    /// Raw `StateOut::flags`, decoded off the wire (not recomputed from
    /// `pitch_rad`) -- issue #161 follow-up, item 5: verifying `FALLEN`
    /// actually trips means checking what actually went out over the wire,
    /// not re-deriving the same threshold this column is supposed to check.
    fallen: bool,
}

fn percentile(sorted_ms: &[f64], p: f64) -> f64 {
    if sorted_ms.is_empty() {
        return f64::NAN;
    }
    let idx = ((p / 100.0) * (sorted_ms.len() - 1) as f64).round() as usize;
    sorted_ms[idx.min(sorted_ms.len() - 1)]
}

/// The subset of `sim-host`'s stats file this tool reports. `jitter_*` is
/// `None` on a stats file written before issue #168's fields existed --
/// this is internal, best-effort tooling, not the wire, so an old-format
/// file degrades to the pre-#168 report rather than failing to parse.
struct HostStats {
    ticks: u64,
    missed_deadlines: u64,
    jitter_p50_ns: Option<u64>,
    jitter_p99_ns: Option<u64>,
    jitter_max_ns: Option<u64>,
}

/// Best-effort read of `sim-host`'s own stats file. Internal tooling, not
/// the wire -- absent or unparsable is reported honestly, not faked.
fn read_host_stats(path: &std::path::Path) -> Option<HostStats> {
    let contents = std::fs::read_to_string(path).ok()?;
    let mut ticks = None;
    let mut missed = None;
    let mut jitter_p50_ns = None;
    let mut jitter_p99_ns = None;
    let mut jitter_max_ns = None;
    for line in contents.lines() {
        if let Some(v) = line.strip_prefix("ticks=") {
            ticks = v.trim().parse().ok();
        } else if let Some(v) = line.strip_prefix("missed_deadlines=") {
            missed = v.trim().parse().ok();
        } else if let Some(v) = line.strip_prefix("jitter_p50_ns=") {
            jitter_p50_ns = v.trim().parse().ok();
        } else if let Some(v) = line.strip_prefix("jitter_p99_ns=") {
            jitter_p99_ns = v.trim().parse().ok();
        } else if let Some(v) = line.strip_prefix("jitter_max_ns=") {
            jitter_max_ns = v.trim().parse().ok();
        }
    }
    match (ticks, missed) {
        (Some(ticks), Some(missed_deadlines)) => Some(HostStats {
            ticks,
            missed_deadlines,
            jitter_p50_ns,
            jitter_p99_ns,
            jitter_max_ns,
        }),
        _ => None,
    }
}

fn main() {
    let args = match parse_args() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("wire-probe: {e}");
            std::process::exit(1);
        }
    };

    let socket = UdpSocket::bind(&args.bind).unwrap_or_else(|e| {
        eprintln!("wire-probe: failed to bind {}: {e}", args.bind);
        std::process::exit(1);
    });
    // Woken periodically so the run stops close to --seconds even with no
    // traffic at all, rather than blocking forever.
    socket
        .set_read_timeout(Some(Duration::from_millis(200)))
        .expect("wire-probe: set_read_timeout failed");

    println!(
        "wire-probe: listening on {} for {:.1}s",
        args.bind, args.seconds
    );

    let run_start = Instant::now();
    let deadline = run_start + Duration::from_secs_f64(args.seconds);

    let mut buf = [0u8; 4096];
    let mut recv_times: Vec<Instant> = Vec::new();
    let mut valid_count: u64 = 0;
    let mut invalid_count: u64 = 0;
    let mut pitch_min = f32::INFINITY;
    let mut pitch_max = f32::NEG_INFINITY;
    let mut pitch_final: f32 = f32::NAN;
    let mut rider_fore_aft_min = f32::INFINITY;
    let mut rider_fore_aft_max = f32::NEG_INFINITY;
    let mut rider_lateral_min = f32::INFINITY;
    let mut rider_lateral_max = f32::NEG_INFINITY;
    let mut first_seq: Option<u64> = None;
    let mut last_seq: Option<u64> = None;
    // Buffered in memory and written once at the end, not appended live:
    // this run is at most a few thousand packets, and a single write avoids
    // interleaving I/O with the receive loop's own timing.
    let mut csv_rows: Vec<CsvRow> = Vec::new();

    while Instant::now() < deadline {
        match socket.recv_from(&mut buf) {
            Ok((n, _src)) => match StateOut::from_bytes(&buf[..n]) {
                Ok(state) => {
                    valid_count += 1;
                    recv_times.push(Instant::now());
                    let pitch = state.pitch_rad;
                    pitch_min = pitch_min.min(pitch);
                    pitch_max = pitch_max.max(pitch);
                    pitch_final = pitch;
                    let (rider_fore_aft, rider_lateral) =
                        (state.rider_fore_aft_m, state.rider_lateral_m);
                    rider_fore_aft_min = rider_fore_aft_min.min(rider_fore_aft);
                    rider_fore_aft_max = rider_fore_aft_max.max(rider_fore_aft);
                    rider_lateral_min = rider_lateral_min.min(rider_lateral);
                    rider_lateral_max = rider_lateral_max.max(rider_lateral);
                    let seq = state.seq;
                    first_seq.get_or_insert(seq);
                    last_seq = Some(seq);
                    if args.csv.is_some() {
                        let pos = state.pos;
                        csv_rows.push(CsvRow {
                            seq,
                            sim_time_s: state.sim_time_s,
                            pos_x_m: pos[0],
                            pos_y_m: pos[1],
                            pitch_rad: pitch,
                            yaw_rad: state.yaw_rad,
                            wheel_rate_rad_s: state.wheel_rate_rad_s,
                            motor_current_a: state.motor_current_a,
                            rider_fore_aft_m: rider_fore_aft,
                            rider_lateral_m: rider_lateral,
                            fallen: state.flags & STATE_FLAG_FALLEN != 0,
                        });
                    }
                }
                Err(e) => {
                    invalid_count += 1;
                    eprintln!("wire-probe: dropping packet that failed to parse: {e}");
                }
            },
            Err(e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                continue;
            }
            Err(e) => {
                eprintln!("wire-probe: recv error: {e}");
                break;
            }
        }
    }

    let elapsed_s = run_start.elapsed().as_secs_f64();
    let total_received = valid_count + invalid_count;

    // Inter-packet intervals, milliseconds, from OUR OWN receipt timestamps
    // -- not carried on the wire, and this is the only place that can
    // measure them: the actual delivered cadence, loopback jitter included.
    let mut intervals_ms: Vec<f64> = recv_times
        .windows(2)
        .map(|w| (w[1] - w[0]).as_secs_f64() * 1000.0)
        .collect();
    let (mean_ms, p50_ms, p99_ms, max_ms) = if intervals_ms.is_empty() {
        (f64::NAN, f64::NAN, f64::NAN, f64::NAN)
    } else {
        let sum: f64 = intervals_ms.iter().sum();
        let mean = sum / intervals_ms.len() as f64;
        intervals_ms.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let p50 = percentile(&intervals_ms, 50.0);
        let p99 = percentile(&intervals_ms, 99.0);
        let max = *intervals_ms.last().unwrap();
        (mean, p50, p99, max)
    };

    let mean_hz = if elapsed_s > 0.0 {
        valid_count as f64 / elapsed_s
    } else {
        f64::NAN
    };

    println!("--- wire-probe report ---");
    println!("run duration: {elapsed_s:.3} s");
    println!("total ticks (valid state packets received): {valid_count}");
    println!("measured tick rate (mean): {mean_hz:.2} Hz");
    if let (Some(first), Some(last)) = (first_seq, last_seq) {
        println!("seq range: {first}..={last} (span {})", last - first + 1);
    }
    println!("inter-packet interval (ms): mean={mean_ms:.4} p50={p50_ms:.4} p99={p99_ms:.4} max={max_ms:.4}");

    match read_host_stats(&args.host_stats) {
        Some(stats) => {
            // Issue #168: a raw miss count against a zero-slack deadline
            // reads as catastrophic ("70% missed") even when the underlying
            // jitter is small and the mean is correct. Percentiles are the
            // actionable form of the same fact; the count stays alongside
            // them rather than being replaced, since "how often" is still a
            // real question the percentiles alone don't answer.
            match (
                stats.jitter_p50_ns,
                stats.jitter_p99_ns,
                stats.jitter_max_ns,
            ) {
                (Some(p50), Some(p99), Some(max)) => {
                    println!(
                        "host-side jitter (ms, recent window): p50={:.4} p99={:.4} max={:.4} \
                         -- {}/{} ticks missed overall, stats file {}",
                        p50 as f64 / 1e6,
                        p99 as f64 / 1e6,
                        max as f64 / 1e6,
                        stats.missed_deadlines,
                        stats.ticks,
                        args.host_stats.display()
                    );
                }
                _ => {
                    println!(
                        "missed-deadline count from the host: {} (host reports {} ticks, stats \
                         file {} predates issue #168's jitter fields)",
                        stats.missed_deadlines,
                        stats.ticks,
                        args.host_stats.display()
                    );
                }
            }
        }
        None => {
            println!(
                "missed-deadline count from the host: unavailable (could not read/parse {})",
                args.host_stats.display()
            );
        }
    }

    println!(
        "pitch_rad: min={:.6} max={:.6} final={:.6} ({:.3} deg / {:.3} deg / {:.3} deg)",
        pitch_min,
        pitch_max,
        pitch_final,
        pitch_min.to_degrees(),
        pitch_max.to_degrees(),
        pitch_final.to_degrees()
    );
    println!(
        "rider_fore_aft_m: min={rider_fore_aft_min:.6} max={rider_fore_aft_max:.6} \
         (v2 -- ACTUAL ballast_fa joint position, not the commanded target)"
    );
    println!(
        "rider_lateral_m: min={rider_lateral_min:.6} max={rider_lateral_max:.6} \
         (v2 -- ACTUAL ballast_lat joint position, not the commanded target)"
    );

    if total_received == 0 {
        println!("confirmation: NO PACKETS RECEIVED -- nothing to confirm");
    } else if invalid_count == 0 {
        println!(
            "confirmation: magic ({STATE_MAGIC:#010x}) + schema_version ({STATE_SCHEMA_VERSION}) \
             parsed correctly on all {valid_count}/{total_received} received packets"
        );
    } else {
        println!(
            "confirmation: FAILED -- {invalid_count}/{total_received} received packets did not \
             parse (bad magic/version/size); {valid_count}/{total_received} were good"
        );
    }

    if let Some(path) = &args.csv {
        let mut out = String::from(
            "seq,sim_time_s,pos_x_m,pos_y_m,pitch_rad,yaw_rad,wheel_rate_rad_s,motor_current_a,rider_fore_aft_m,rider_lateral_m,fallen\n",
        );
        for r in &csv_rows {
            out.push_str(&format!(
                "{},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{}\n",
                r.seq,
                r.sim_time_s,
                r.pos_x_m,
                r.pos_y_m,
                r.pitch_rad,
                r.yaw_rad,
                r.wheel_rate_rad_s,
                r.motor_current_a,
                r.rider_fore_aft_m,
                r.rider_lateral_m,
                r.fallen
            ));
        }
        match std::fs::write(path, out) {
            Ok(()) => println!("csv: wrote {} rows to {}", csv_rows.len(), path.display()),
            Err(e) => println!("csv: FAILED to write {}: {e}", path.display()),
        }
    }
}
