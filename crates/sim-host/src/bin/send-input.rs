//! A scripted UDP input SENDER, for verification runs only (issue #161 W2).
//!
//! **Not** part of the wire consumer set Unreal implements -- this is this
//! crate's own dev/verification tool, standing in for a player until the
//! real UE client exists. It reuses [`sim_host::wire::InputIn`] directly
//! rather than re-implementing the byte layout in a second language, so the
//! encoding is guaranteed correct the same way `wire-probe` guarantees its
//! decoding is.
//!
//! Sends one `InputIn` packet every 20 ms (50 Hz, comfortably above
//! `sim-host`'s 100 ms staleness timeout) to `127.0.0.1:9602`, stepping
//! through the fixed schedule below: settle, lean forward (accelerate),
//! release (coast), lean back (decelerate then reverse), release again,
//! lean+steer (turn) -- the sequence issue #161 W2's acceptance criterion
//! asks for ("drives forward, slows and reverses under lean, and turns").
//!
//! ```text
//! send-input [--target ADDR]
//! ```

use sim_host::wire::{InputIn, INPUT_MAGIC, INPUT_SCHEMA_VERSION};
use std::net::UdpSocket;
use std::time::{Duration, Instant};

/// `(start_s, end_s, weight_shift_fore_aft, weight_shift_lateral, steer, label)`.
/// Values are within `[-1, 1]`; `sim-host` scales them onto the model's
/// actual ballast/yaw ranges.
const SCHEDULE: &[(f64, f64, f32, f32, f32, &str)] = &[
    (0.0, 1.5, 0.0, 0.0, 0.0, "settle"),
    (1.5, 4.5, 0.6, 0.0, 0.0, "lean forward (accelerate)"),
    (4.5, 6.5, 0.0, 0.0, 0.0, "release (coast)"),
    (
        6.5,
        10.0,
        -0.6,
        0.0,
        0.0,
        "lean back (decelerate, then reverse)",
    ),
    (10.0, 12.0, 0.0, 0.0, 0.0, "release (coast reversed)"),
    (12.0, 14.0, 0.0, 0.8, 0.6, "lean right + steer (turn)"),
    (14.0, 15.0, 0.0, 0.0, 0.0, "release"),
];

fn total_duration_s() -> f64 {
    SCHEDULE.iter().map(|s| s.1).fold(0.0, f64::max)
}

fn value_at(t: f64) -> (f32, f32, f32, &'static str) {
    for &(t0, t1, fa, lat, steer, label) in SCHEDULE {
        if t0 <= t && t < t1 {
            return (fa, lat, steer, label);
        }
    }
    (0.0, 0.0, 0.0, "end")
}

fn main() {
    let mut target = sim_host::wire::INPUT_IN_ADDR.to_string();
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
            other => {
                eprintln!("send-input: unrecognized argument '{other}'");
                std::process::exit(1);
            }
        }
    }

    let socket = UdpSocket::bind("127.0.0.1:0").expect("send-input: bind failed");
    let duration = total_duration_s();
    eprintln!("send-input: sending to {target} for {duration:.1}s per the fixed schedule");

    let start = Instant::now();
    let period = Duration::from_millis(20);
    let mut seq: u64 = 0;
    let mut last_label = "";

    loop {
        let t = start.elapsed().as_secs_f64();
        if t > duration {
            break;
        }
        let (fore_aft, lateral, steer, label) = value_at(t);
        if label != last_label {
            eprintln!(
                "send-input: t={t:6.2}s -> {label} (fore_aft={fore_aft:+.2} lateral={lateral:+.2} steer={steer:+.2})"
            );
            last_label = label;
        }
        let pkt = InputIn {
            magic: INPUT_MAGIC,
            schema_version: INPUT_SCHEMA_VERSION,
            flags: 0,
            seq,
            weight_shift_fore_aft: fore_aft,
            weight_shift_lateral: lateral,
            steer,
        };
        if let Err(e) = socket.send_to(&pkt.to_bytes(), &target) {
            eprintln!("send-input: send failed: {e}");
        }
        seq += 1;
        std::thread::sleep(period);
    }

    eprintln!("send-input: schedule complete ({seq} packets sent)");
}
