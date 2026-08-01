//! The CEO-visible verification tool for the issue #161 wire (W1).
//!
//! A headless UDP receiver, independent of the `sim-host` LIBRARY (it only
//! uses [`sim_host::wire`]) -- it proves the wire works from the outside,
//! the same position Unreal will be in, not from inside the process that
//! produces it.
//!
//! ```text
//! wire-probe [--seconds N] [--bind ADDR] [--host-stats PATH]
//! ```
//! Binds `--bind` (default `127.0.0.1:9601`, the documented state-out
//! address), listens for `--seconds` (default 10), and prints a plain-text
//! report: measured tick rate, inter-packet interval stats, the host's own
//! missed-deadline count (read from `--host-stats`, best-effort -- issue
//! #161's wire itself carries no room for that counter), pitch_rad
//! min/max/final, and a magic+version parse confirmation.

use sim_host::wire::{StateOut, SCHEMA_VERSION, STATE_MAGIC};
use std::net::UdpSocket;
use std::path::PathBuf;
use std::time::{Duration, Instant};

struct Args {
    seconds: f64,
    bind: String,
    host_stats: PathBuf,
}

impl Default for Args {
    fn default() -> Self {
        Args {
            seconds: 10.0,
            bind: sim_host::wire::STATE_OUT_ADDR.to_string(),
            host_stats: PathBuf::from(sim_host::host::DEFAULT_STATS_PATH),
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
            other => return Err(format!("unrecognized argument '{other}'")),
        }
    }
    Ok(a)
}

fn percentile(sorted_ms: &[f64], p: f64) -> f64 {
    if sorted_ms.is_empty() {
        return f64::NAN;
    }
    let idx = ((p / 100.0) * (sorted_ms.len() - 1) as f64).round() as usize;
    sorted_ms[idx.min(sorted_ms.len() - 1)]
}

/// Best-effort read of `sim-host`'s own stats file. Internal tooling, not
/// the wire -- absent or unparsable is reported honestly, not faked.
fn read_host_stats(path: &std::path::Path) -> Option<(u64, u64)> {
    let contents = std::fs::read_to_string(path).ok()?;
    let mut ticks = None;
    let mut missed = None;
    for line in contents.lines() {
        if let Some(v) = line.strip_prefix("ticks=") {
            ticks = v.trim().parse().ok();
        } else if let Some(v) = line.strip_prefix("missed_deadlines=") {
            missed = v.trim().parse().ok();
        }
    }
    match (ticks, missed) {
        (Some(t), Some(m)) => Some((t, m)),
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
    let mut first_seq: Option<u64> = None;
    let mut last_seq: Option<u64> = None;

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
                    let seq = state.seq;
                    first_seq.get_or_insert(seq);
                    last_seq = Some(seq);
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
        Some((host_ticks, missed)) => {
            println!(
                "missed-deadline count from the host: {missed} (host reports {host_ticks} ticks, stats file {})",
                args.host_stats.display()
            );
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

    if total_received == 0 {
        println!("confirmation: NO PACKETS RECEIVED -- nothing to confirm");
    } else if invalid_count == 0 {
        println!(
            "confirmation: magic ({STATE_MAGIC:#010x}) + schema_version ({SCHEMA_VERSION}) \
             parsed correctly on all {valid_count}/{total_received} received packets"
        );
    } else {
        println!(
            "confirmation: FAILED -- {invalid_count}/{total_received} received packets did not \
             parse (bad magic/version/size); {valid_count}/{total_received} were good"
        );
    }
}
