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

/// `--scenario s-curve`: hold a decent forward speed and weave wide, for watching the board CARVE
/// rather than checking one turn happens at all.
///
/// [`SCHEDULE`] above is issue #161 W2's acceptance criterion and is deliberately left alone --
/// it exercises accelerate / coast / reverse / turn once each, which is what that AC asks for and
/// what any regression check should keep running. This one is a viewing scenario.
///
/// Shape: build speed, then five alternating 2.6 s lobes around the original heading, entered and
/// left on half-length lobes so the weave is centred on the road rather than walking off it.
/// Measured result: 5 reversals, ~82 deg of heading peak-to-peak, +/-3 m of lateral excursion over
/// ~74 m at ~5.2 m/s. The road at OB_City's spawn carries road-level surface out to roughly 8.6 m
/// either side, so +/-3 m keeps a few feet of margin to the kerb.
///
/// **Width comes from lobe DURATION, not from more steer.** Lateral excursion grows with the time
/// spent holding a heading, so long gentle lobes push the board wide while keeping it pointed
/// mostly down the road; cranking `steer` instead just aims it across the carriageway at the same
/// offset. 2.6 s at steer 0.40 is the shape that reads as carving rather than swerving.
///
/// `fore_aft` stays slightly positive (0.12) through the weave rather than zero: there is no outer
/// velocity loop, so the board coasts rather than holding speed, and a small standing lean covers
/// the losses without running away. Sign convention matches [`SCHEDULE`]: positive is right.
///
/// # The entry lobe is 1.17 s and that number is load-bearing
///
/// It sets where the whole oscillation is CENTRED, and the board's lateral drift follows directly
/// from the mean heading -- ~74 m of travel at a 7 deg mean is ~9 m off line. Measured:
///
/// | entry lobe | mean yaw | lateral drift |
/// |---|---|---|
/// | 1.30 s | +7.0 deg | -10.0 m |
/// | 1.17 s | -1.1 deg | +2.5 m |
/// | 1.05 s | -6.9 deg | +11.8 m |
///
/// A quarter of a second swings the drift by twenty metres, so do not treat this as a free knob.
///
/// Two things that look like fixes and are not. Balancing the left/right DURATIONS exactly does
/// not centre it -- `roll_authority` is state-dependent, so equal time does not buy equal yaw.
/// And pre-loading roll before the first steer input (lean applied, steer still zero) makes it
/// WORSE, not better: it hands the entry lobe more authority than the full lobes get, pushing the
/// mean to +13 deg and the drift to -23 m. The entry lobe over-delivers; it does not under-deliver.
///
/// Finally: `sim-host` paces on the wall clock and misses roughly half its 500 Hz deadlines on a
/// non-RT macOS host, so packet arrival shifts between runs. Expect a couple of metres of run-to-run
/// variation in the final offset. Chasing the last metre is fitting noise; a genuinely
/// drift-free weave needs closed-loop steering, which this open-loop sender deliberately is not.
const S_CURVE_SCHEDULE: &[(f64, f64, f32, f32, f32, &str)] = &[
    (0.0, 1.0, 0.0, 0.0, 0.0, "settle"),
    (1.0, 4.0, 0.65, 0.0, 0.0, "lean forward (build speed)"),
    (4.0, 5.17, 0.12, 0.7, 0.40, "weave right (half lobe, enter)"),
    (5.17, 7.77, 0.12, -0.7, -0.40, "weave left"),
    (7.77, 10.37, 0.12, 0.7, 0.40, "weave right"),
    (10.37, 12.97, 0.12, -0.7, -0.40, "weave left"),
    (12.97, 15.57, 0.12, 0.7, 0.40, "weave right"),
    (15.57, 16.87, 0.12, -0.7, -0.40, "straighten (exit lobe)"),
    (16.87, 19.2, 0.0, 0.0, 0.0, "release (coast straight)"),
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
