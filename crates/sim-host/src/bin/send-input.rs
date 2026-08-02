//! A scripted UDP input SENDER, for verification runs only (issue #161 W2).
//!
//! **Not** part of the wire consumer set Unreal implements -- this is this
//! crate's own dev/verification tool, standing in for a player until the
//! real UE client exists. It reuses [`sim_host::wire::InputIn`] directly
//! rather than re-implementing the byte layout in a second language, so the
//! encoding is guaranteed correct the same way `wire-probe` guarantees its
//! decoding is. The schedules themselves live in [`sim_host::scenario`], so
//! this tool and the host's own deterministic playback path cannot drift
//! apart.
//!
//! Sends one `InputIn` packet every 20 ms (50 Hz, comfortably above
//! `sim-host`'s 100 ms staleness timeout) to `127.0.0.1:9602`, stepping
//! through the selected schedule.
//!
//! ```text
//! send-input [--target ADDR] [--scenario default|s-curve] [--kick-at SECONDS]
//! ```
//!
//! # Pacing: absolute deadlines, and why that turned out to matter a lot
//!
//! This loop used to `thread::sleep(20 ms)` per packet and index the schedule
//! off `Instant::elapsed()`. That is exactly the accumulating-drift mistake
//! [`sim_host::pacer`]'s own doc comment exists to warn about, and on macOS,
//! where timer coalescing readily turns a 20 ms sleep request into 70-140 ms,
//! it was not a rounding error: **measured over three full `s-curve` runs,
//! this sender delivered 12.8 Hz, 7.4 Hz and 7.6 Hz -- not 50 Hz.**
//!
//! `sim-host` zeroes any stick input older than its 100 ms staleness timeout,
//! so a 7-13 Hz sender sits directly ON that threshold. A run-varying
//! fraction of every scripted run therefore held `weight_shift_fore_aft = 0`
//! instead of the scheduled value, and the SAME command produced materially
//! different aggression run to run (measured steady-state `ballast_fa` of
//! 0.031 m in a starved run against 0.046 m in a healthy one, against the
//! same full-stick schedule). Stability results measured through the old
//! sender were measured against a de-rated schedule -- see
//! [`sim_host::scenario`]'s correction block.
//!
//! Fixed here by pacing against a single advancing absolute deadline. For a
//! run that must be repeatable rather than merely correctly paced, use
//! `sim-host --scripted-scenario`, which plays the same schedule indexed on
//! simulated time with no socket and no staleness gate at all.

use sim_host::scenario::{self, Schedule};
use sim_host::wire::{InputIn, INPUT_FLAG_KICK, INPUT_MAGIC, INPUT_SCHEMA_VERSION};
use std::net::UdpSocket;
use std::time::{Duration, Instant};

fn main() {
    let mut target = sim_host::wire::INPUT_IN_ADDR.to_string();
    let mut schedule: Schedule = scenario::DEFAULT_SCHEDULE;
    let mut schedule_name = "default";
    let mut kick_at: Option<f64> = None;
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--target" => {
                target = args.get(i + 1).cloned().unwrap_or_else(|| {
                    eprintln!("send-input: --target needs a value");
                    std::process::exit(1);
                });
                i += 2;
            }
            // Issue #161 follow-up, item 5: "make falls testable" --
            // triggers `wire::INPUT_FLAG_KICK` (a rising edge; sim-host
            // debounces retriggers itself) at the given schedule time, on
            // top of whatever the running schedule is already sending.
            // Verification tool only -- not part of the AC schedule.
            "--kick-at" => {
                let v = args.get(i + 1).cloned().unwrap_or_else(|| {
                    eprintln!("send-input: --kick-at needs a value");
                    std::process::exit(1);
                });
                kick_at = Some(v.parse::<f64>().unwrap_or_else(|_| {
                    eprintln!("send-input: --kick-at value '{v}' is not a number");
                    std::process::exit(1);
                }));
                i += 2;
            }
            "--scenario" => {
                let v = args.get(i + 1).cloned().unwrap_or_else(|| {
                    eprintln!("send-input: --scenario needs a value");
                    std::process::exit(1);
                });
                match scenario::by_name(&v) {
                    Some(s) => {
                        schedule = s;
                        // Leaked-free: look the name back up in the table so
                        // the label is 'static without leaking the argument.
                        schedule_name = scenario::BY_NAME
                            .iter()
                            .find(|(n, _)| *n == v)
                            .map(|(n, _)| *n)
                            .unwrap_or("unknown");
                    }
                    None => {
                        eprintln!(
                            "send-input: unknown scenario '{v}' (want: {})",
                            scenario::names()
                        );
                        std::process::exit(1);
                    }
                }
                i += 2;
            }
            other => {
                eprintln!("send-input: unrecognized argument '{other}'");
                std::process::exit(1);
            }
        }
    }

    let socket = UdpSocket::bind("127.0.0.1:0").expect("send-input: bind failed");
    // If --kick-at lands after the schedule would otherwise end, extend the
    // run so there is time left afterward to watch the outcome rather than
    // the process exiting mid-fall.
    let duration = kick_at.map_or(scenario::total_duration_s(schedule), |ka| {
        scenario::total_duration_s(schedule).max(ka + 3.0)
    });
    eprintln!(
        "send-input: sending to {target} for {duration:.1}s per the '{schedule_name}' schedule"
    );

    let start = Instant::now();
    let period = Duration::from_millis(20);
    let mut seq: u64 = 0;
    let mut last_label = "";
    let mut kick_logged = false;
    let mut late_packets: u64 = 0;
    // Absolute deadline, advanced by exactly one period per packet -- never
    // `sleep(period)`, which would silently accumulate every scheduler
    // overrun into the schedule itself (see this file's header).
    let mut next_deadline = start + period;

    loop {
        let t = start.elapsed().as_secs_f64();
        if t > duration {
            break;
        }
        let (fore_aft, lateral, steer, label) = scenario::value_at(schedule, t);
        if label != last_label {
            eprintln!(
                "send-input: t={t:6.2}s -> {label} (fore_aft={fore_aft:+.2} lateral={lateral:+.2} steer={steer:+.2})"
            );
            last_label = label;
        }
        // One-shot rising edge: held for a couple of packets (40 ms) so a
        // single dropped UDP datagram cannot silently swallow the whole
        // kick, but not held so long it would look like a second press.
        let in_kick_window = kick_at.is_some_and(|ka| (ka..ka + 0.04).contains(&t));
        if in_kick_window && !kick_logged {
            eprintln!("send-input: t={t:6.2}s -> KICK");
            kick_logged = true;
        }
        let flags = if in_kick_window { INPUT_FLAG_KICK } else { 0 };
        let pkt = InputIn {
            magic: INPUT_MAGIC,
            schema_version: INPUT_SCHEMA_VERSION,
            flags,
            seq,
            weight_shift_fore_aft: fore_aft,
            weight_shift_lateral: lateral,
            steer,
        };
        if let Err(e) = socket.send_to(&pkt.to_bytes(), &target) {
            eprintln!("send-input: send failed: {e}");
        }
        seq += 1;

        let now = Instant::now();
        if next_deadline > now {
            std::thread::sleep(next_deadline - now);
        } else {
            // Already past this packet's slot -- do not sleep, and count it.
            // Reported at the end rather than per packet, because a burst of
            // these is exactly the condition that used to be invisible.
            late_packets += 1;
        }
        next_deadline += period;
    }

    let elapsed = start.elapsed().as_secs_f64();
    let hz = if elapsed > 0.0 {
        seq as f64 / elapsed
    } else {
        f64::NAN
    };
    eprintln!(
        "send-input: schedule complete ({seq} packets sent over {elapsed:.2}s = {hz:.1} Hz \
         effective; {late_packets} sent late)"
    );
    // The staleness gate is the thing that actually bites, so say so rather
    // than leaving a starved run looking like a healthy one (issue #190).
    let staleness_hz = 1.0 / sim_host::host::INPUT_STALENESS_TIMEOUT.as_secs_f64();
    if hz < staleness_hz * 2.0 {
        eprintln!(
            "send-input: WARNING -- effective rate {hz:.1} Hz is within 2x of sim-host's \
             {staleness_hz:.0} Hz staleness floor. Some ticks will have run with the input \
             ZEROED, so this run is a DE-RATED version of the '{schedule_name}' schedule. \
             Do not quote a stability result from it; use \
             `sim-host --scripted-scenario {schedule_name}` instead."
        );
    }
}
