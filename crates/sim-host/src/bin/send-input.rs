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
/// # Revision 2 (issue #161 follow-up): the first cut wove at walking pace
///
/// Revision 1 built speed for only 3 s before weaving, entering the weave at 0.71 m/s -- turn
/// radius is speed / yaw-rate, so however hard it steered, the lobes were geometrically tiny, and
/// the board was still accelerating THROUGH the whole weave (0.71 -> 3.6 m/s) rather than carving
/// at a settled speed. It also drifted: `wire-probe` measured heading swinging -22.1..+51.4 deg,
/// centred +14.6 deg, not 0 -- a steady sideways walk, not a weave about the road direction.
///
/// This revision:
/// - **Builds speed for 10 s, not 3 s**, at the same 0.65 lean -- sustained lean has no speed
///   ceiling on this plant (there is no outer velocity loop; PR #170/#171 already measured
///   continuous acceleration to 13+ m/s over 30 s), so the speed is available, the first cut just
///   was not waiting for it. See [`SCURVE2_BUILD_S`]'s own doc comment for why 10 s and not the
///   6 s an offline check first suggested -- the real, wire-paced host needed noticeably longer.
/// - **Raises the in-weave trim off Revision 1's insufficient 0.12** -- carving scrubs energy
///   (each lobe steers the wheel off a straight roll) -- but ends up net LOWER than the first
///   attempt at raising it (0.20), because at this revision's higher entry speed the same trim
///   value keeps adding much more absolute speed than it did at Revision 1's walking pace. See
///   [`SCURVE2_WEAVE_TRIM`]'s doc comment for the three measured values this went through.
/// - **Fewer, wider lobes than Revision 1**: two full lobes instead of four, still individually
///   longer than Revision 1's 2.6 s ("width comes from lobe DURATION, not from more steer" --
///   Revision 1's own finding, still true) but shorter than this revision's own first attempt
///   (5.0 s), which combined with the higher speed covered close to 300 m -- more ground than
///   "roughly on the road" comfortably allows with no City Park collision geometry to bound it.
///   See [`SCURVE2_LOBE_S`]'s doc comment.
///
/// **The entry lobe still sets where the whole oscillation is centred**, and remains sensitive --
/// Revision 1 measured a quarter-second swinging the drift by twenty metres. Building more speed
/// first and changing the in-weave trim both change `roll_authority`'s time history, so Revision
/// 1's measured entry-lobe timing (1.17 s) was not assumed to still centre this one; this
/// revision keeps 2.3 s (its own first guess, not re-tuned further -- see the measured result
/// below, which centres well enough that chasing it further risked being the same "fitting noise"
/// Revision 1's own comment already warned against).
///
/// # What this revision actually measured (`wire-probe --csv` over a full run)
///
/// | metric | value |
/// |---|---|
/// | weave-entry speed | 6.82 m/s (24.6 km/h) -- just under the requested 7-9 m/s band |
/// | peak speed | 9.11 m/s (32.8 km/h) -- at the top of the requested band |
/// | heading swing | -32.6 .. +46.1 deg (centred +6.75 deg off the road direction) |
/// | lateral excursion (peak-to-peak) | 17.4 m |
/// | forward distance travelled | 183 m |
/// | pitch, whole run | -5.2 .. +1.5 deg (well inside the fallen threshold throughout) |
///
/// Sign convention matches [`SCHEDULE`]: positive is right.
const SCURVE2_SETTLE_S: f64 = 1.0;

/// How long to hold [`SCURVE2_BUILD_LEAN`] before the weave starts. An offline check (steady
/// balance controller, `ballast_fa` driven straight to the stick-scaled target, no weave) of THIS
/// lean/gain pair from rest found roughly 6.46 m/s at 6.0 s and roughly 8.5 m/s at 7.2 s, with
/// pitch still comfortably inside the fallen threshold at both -- but visibly starting to
/// deteriorate by 7.6 s.
///
/// **That offline check over-predicted the real host.** A first measured pass at 6.0 s (on the
/// real wire, real `sim-host` pacing and all) reached only 3.75 m/s at weave entry, not ~6.5 m/s
/// -- packets paced on the wall clock against a host that misses roughly half its 500 Hz
/// deadlines do not apply as cleanly as an idealised, un-paced offline physics loop. 10.0 s is the
/// re-measured value; pitch stayed inside -6.3..+1.7 deg over the WHOLE run at 6.0 s (see the
/// results reported at the bottom of this file's doc comment), which is enough margin to extend
/// with confidence rather than inching up by a second at a time.
const SCURVE2_BUILD_S: f64 = 10.0;
const SCURVE2_BUILD_LEAN: f32 = 0.65;

/// In-weave forward trim. History, because this one moved a lot and the direction is
/// counter-intuitive: 0.12 (Revision 1) measured insufficient to hold speed against carving
/// losses; 0.20 measured to overshoot badly (+6 m/s of continued climb over the ~17 s of lobes at
/// a 3.75 m/s entry); 0.15, at the higher ~8 m/s entry speed `SCURVE2_BUILD_S = 10.0` produces,
/// STILL added +3.5 m/s of continued climb, i.e. the same nominal trim adds MORE absolute speed
/// at higher entry speed, not less -- this is not a fixed "hold" trim, its effect is state
/// dependent the same way `roll_authority` is. 0.08 is the current value, chosen to cut the
/// climb further rather than assume the relationship is linear enough to solve for a target in
/// one step. See this file's own measured results for where it actually landed.
const SCURVE2_WEAVE_TRIM: f32 = 0.08;
/// Full-lobe duration. Revision 1 used 2.6 s; this file's first pass at Revision 2 doubled it to
/// 5.0 s ("width comes from duration"), which combined with the higher entry speed above covered
/// close to 300 m over the full run -- more than the COO's own back-of-envelope estimate (~200 m
/// at 8 m/s) and more distance than "roughly on the road" comfortably allows without City Park
/// collision geometry to bound it. 3.5 s trades some of that width back for a shorter, more
/// contained run; still meaningfully wider than Revision 1's 2.6 s.
const SCURVE2_LOBE_S: f64 = 3.5;
/// Entry (and, by the same centring logic, exit) half-lobe duration. Revision 1's equivalent
/// (1.17 s) is NOT reused unmodified -- picked fresh at 2.3 s for this revision and left there:
/// the measured result centred at +6.75 deg (see the module-level results table above), close
/// enough that chasing it tighter risked fitting run-to-run noise the same way Revision 1's own
/// comment already warned against.
const SCURVE2_ENTRY_S: f64 = 2.3;

const S_CURVE_SCHEDULE: &[(f64, f64, f32, f32, f32, &str)] = &[
    (0.0, SCURVE2_SETTLE_S, 0.0, 0.0, 0.0, "settle"),
    (
        SCURVE2_SETTLE_S,
        SCURVE2_SETTLE_S + SCURVE2_BUILD_S,
        SCURVE2_BUILD_LEAN,
        0.0,
        0.0,
        "lean forward (build speed)",
    ),
    (
        SCURVE2_SETTLE_S + SCURVE2_BUILD_S,
        SCURVE2_SETTLE_S + SCURVE2_BUILD_S + SCURVE2_ENTRY_S,
        SCURVE2_WEAVE_TRIM,
        0.7,
        0.40,
        "weave right (half lobe, enter)",
    ),
    (
        SCURVE2_SETTLE_S + SCURVE2_BUILD_S + SCURVE2_ENTRY_S,
        SCURVE2_SETTLE_S + SCURVE2_BUILD_S + SCURVE2_ENTRY_S + SCURVE2_LOBE_S,
        SCURVE2_WEAVE_TRIM,
        -0.7,
        -0.40,
        "weave left",
    ),
    (
        SCURVE2_SETTLE_S + SCURVE2_BUILD_S + SCURVE2_ENTRY_S + SCURVE2_LOBE_S,
        SCURVE2_SETTLE_S + SCURVE2_BUILD_S + SCURVE2_ENTRY_S + 2.0 * SCURVE2_LOBE_S,
        SCURVE2_WEAVE_TRIM,
        0.7,
        0.40,
        "weave right",
    ),
    (
        SCURVE2_SETTLE_S + SCURVE2_BUILD_S + SCURVE2_ENTRY_S + 2.0 * SCURVE2_LOBE_S,
        SCURVE2_SETTLE_S + SCURVE2_BUILD_S + 2.0 * SCURVE2_ENTRY_S + 2.0 * SCURVE2_LOBE_S,
        SCURVE2_WEAVE_TRIM,
        -0.7,
        -0.40,
        "straighten (exit lobe)",
    ),
    (
        SCURVE2_SETTLE_S + SCURVE2_BUILD_S + 2.0 * SCURVE2_ENTRY_S + 2.0 * SCURVE2_LOBE_S,
        SCURVE2_SETTLE_S + SCURVE2_BUILD_S + 2.0 * SCURVE2_ENTRY_S + 2.0 * SCURVE2_LOBE_S + 3.0,
        0.0,
        0.0,
        0.0,
        "release (coast straight)",
    ),
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
                        eprintln!(
                            "send-input: unknown scenario '{other}' (want: default, s-curve)"
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
    let duration = total_duration_s(schedule);
    eprintln!(
        "send-input: sending to {target} for {duration:.1}s per the '{schedule_name}' schedule"
    );

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
