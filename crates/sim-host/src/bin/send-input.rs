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
//! send-input [--target ADDR] [--scenario default|s-curve]
//! ```

use sim_host::wire::{InputIn, INPUT_MAGIC, INPUT_SCHEMA_VERSION};

type Schedule = &'static [(f64, f64, f32, f32, f32, &'static str)];
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

/// `--scenario s-curve`: hold a decent forward speed and weave, for watching the board CARVE
/// rather than checking one turn happens at all.
///
/// [`SCHEDULE`] above is issue #161 W2's acceptance criterion and is deliberately left alone --
/// it exercises accelerate / coast / reverse / turn once each, which is what that AC asks for and
/// what any regression check should keep running. This one is a viewing scenario.
///
/// Shape: build speed, then alternate steer around the original heading. The first and last lobes
/// are HALF length so the weave is centred on the road rather than walking off it -- full lobes
/// throughout would leave the board pointing 25-odd degrees off-axis at the end of every pass.
///
/// Sizing is measured, not guessed. `YAW_RATE_GAIN_RAD_S` is 1.5 rad/s at full steer and full
/// authority; a live run at steer 0.6 with lateral 0.8 produced ~0.78 rad/s, i.e. `roll_authority`
/// lands near 0.87 (well above its 0.35 floor) once the rider leans into it. At steer 0.5 that is
/// ~0.64 rad/s, so a 0.8 s half-lobe swings ~29 deg and a 1.6 s full lobe swings ~59 deg through
/// centre -- a weave that reads clearly on camera without leaving the carriageway.
///
/// The exit lobe is LONGER than the entry one (1.2 s against 0.8 s), which is not a typo. The
/// ballast takes time to build roll, so a short lobe sees a lower `roll_authority` than the same
/// fraction of a long one and under-turns: with entry and exit both at 0.8 s the left/right
/// DURATIONS balanced exactly (4.0 s each) and the heading still finished +6.4 deg off, walking
/// the board 5.8 m sideways over 50 m. Balanced time does not mean balanced yaw when the gain is
/// state-dependent. 1.2 s over-corrected to -19.8 deg, so the null sits near 0.95 s.
///
/// **Do not try to tune the residual heading to zero.** This schedule is open loop and `sim-host`
/// paces on the wall clock, missing roughly half its 500 Hz deadlines on a non-RT macOS host, so
/// the arrival timing of these 50 Hz packets shifts run to run: two runs of the SAME schedule
/// gave yaw extremes of (-28.2, +38.9) and (-33.7, +30.0) deg. The weave shape is repeatable; the
/// exact end heading is not, and chasing it would be fitting noise. Expect to finish somewhere
/// within roughly +/-15 deg of straight and drifting a few metres off the centre line.
///
/// `fore_aft` stays slightly positive (0.12) through the weave rather than zero: there is no outer
/// velocity loop, so the board coasts rather than holding speed, and a small standing lean covers
/// the losses without running away. Sign convention matches [`SCHEDULE`]: positive is right.
const S_CURVE_SCHEDULE: &[(f64, f64, f32, f32, f32, &str)] = &[
    (0.0, 1.0, 0.0, 0.0, 0.0, "settle"),
    (1.0, 4.0, 0.65, 0.0, 0.0, "lean forward (build speed)"),
    (4.0, 4.8, 0.12, 0.7, 0.5, "weave right (half lobe, enter)"),
    (4.8, 6.4, 0.12, -0.7, -0.5, "weave left"),
    (6.4, 8.0, 0.12, 0.7, 0.5, "weave right"),
    (8.0, 9.6, 0.12, -0.7, -0.5, "weave left"),
    (9.6, 11.2, 0.12, 0.7, 0.5, "weave right"),
    (11.2, 12.15, 0.12, -0.7, -0.5, "straighten (exit lobe)"),
    (12.15, 15.0, 0.0, 0.0, 0.0, "release (coast straight)"),
];

fn total_duration_s(schedule: Schedule) -> f64 {
    schedule.iter().map(|s| s.1).fold(0.0, f64::max)
}

fn value_at(schedule: Schedule, t: f64) -> (f32, f32, f32, &'static str) {
    for &(t0, t1, fa, lat, steer, label) in schedule {
        if t0 <= t && t < t1 {
            return (fa, lat, steer, label);
        }
    }
    (0.0, 0.0, 0.0, "end")
}

fn main() {
    let mut target = sim_host::wire::INPUT_IN_ADDR.to_string();
    let mut schedule: Schedule = SCHEDULE;
    let mut schedule_name = "default";
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
            "--scenario" => {
                let v = args.get(i + 1).cloned().unwrap_or_else(|| {
                    eprintln!("send-input: --scenario needs a value");
                    std::process::exit(1);
                });
                match v.as_str() {
                    "default" => {
                        schedule = SCHEDULE;
                        schedule_name = "default";
                    }
                    "s-curve" => {
                        schedule = S_CURVE_SCHEDULE;
                        schedule_name = "s-curve";
                    }
                    other => {
                        eprintln!("send-input: unknown scenario '{other}' (want: default, s-curve)");
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
    let duration = total_duration_s(schedule);
    eprintln!("send-input: sending to {target} for {duration:.1}s per the '{schedule_name}' schedule");

    let start = Instant::now();
    let period = Duration::from_millis(20);
    let mut seq: u64 = 0;
    let mut last_label = "";

    loop {
        let t = start.elapsed().as_secs_f64();
        if t > duration {
            break;
        }
        let (fore_aft, lateral, steer, label) = value_at(schedule, t);
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
