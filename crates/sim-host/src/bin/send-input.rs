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
//! send-input [--target ADDR] [--scenario default|s-curve] [--kick-at SECONDS]
//! ```

use sim_host::wire::{InputIn, INPUT_FLAG_KICK, INPUT_MAGIC, INPUT_SCHEMA_VERSION};

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
/// # Revision 3 (issue #161 follow-up): "getting a feel for it" was the wrong brief
///
/// Revision 2's clip was kept (retitled "Manny's first ride"), but the CEO's verdict on it was
/// explicit: too slow, and the carves barely happen -- roughly 3x the speed, and much harder
/// carving. This is a different brief -- ripping, not demonstrating -- and it changes more than
/// one constant:
///
/// - **`SCURVE3_BUILD_S`/`SCURVE3_BUILD_LEAN`**: build lean raised to 1.0 (full stick -- "do not
///   be shy with fore_aft during the build") and duration extended well past Revision 2's 10 s.
///   Sustained lean has no speed ceiling on this plant (PR #170/#171 already measured continuous
///   acceleration to 13+ m/s over 30 s at a much smaller lean), so ~20 m/s is reachable; the
///   question this revision answers by measurement, not derivation, is how long that takes at
///   full lean and what it costs in distance.
/// - **`lateral`/`steer` taken to their limits (1.0/1.0)**, up from Revision 2's 0.7/0.40, per the
///   explicit instruction to stop leaving carving authority on the table.
/// - **`YAW_RATE_GAIN_RAD_S` doubled (1.5 -> 3.0) in `host.rs`**, explicitly authorised for this
///   request -- the roll gate and its floor are untouched. Turn radius is speed / yaw-rate, and
///   tripling target speed alone would make carves three times lazier at the OLD gain; doubling
///   it buys back some of that geometry instead of asking speed and steer alone to fight it.
/// - **Fewer, SHORTER lobes than Revision 2** -- the higher yaw-rate gain buys back the heading
///   swing per second that shorter lobes would otherwise cost, and at a 3x higher speed every
///   second of lobe duration costs 3x the distance. See [`SCURVE3_LOBE_S`]'s doc comment for what
///   this actually cost in road length.
///
/// **Road length is the dominant constraint at this speed, explicitly, not an afterthought.**
/// Revision 2 covered 183 m at a 9 m/s peak; MuJoCo has no City Park collision geometry, so a
/// board that runs out of built environment at 20 m/s just sails into nothing, and that footage
/// is unusable. This revision is deliberately short -- brief settle, one hard build, a few hard
/// lobes, out -- rather than a long cruise, and reports distance explicitly rather than assuming
/// it is fine. **A first pass (15.0 s build, three full lobes) DID exactly what the CEO asked --
/// 15.2 m/s entry, 18.5 m/s peak, pitch a stable -7.7..+3.1 deg -- and covered 303 m doing it,**
/// past the ~200 m line the COO drew. Cut back (`SCURVE3_BUILD_S` 15.0 -> 11.0 s, one fewer full
/// lobe) rather than shipping the faster, further-reaching version and hoping -- see the measured
/// trade this cut actually cost, below. Pitch stayed stable at every speed tried in this revision;
/// **the board did not become unstable or face-plant at any point measured** -- worth saying
/// explicitly, since the alternative (it did, and got tuned around silently) is exactly what was
/// ruled out from the start.
///
/// # What this revision actually measured (`wire-probe --csv`, final landed configuration)
///
/// | metric | value |
/// |---|---|
/// | weave-entry speed | 10.29 m/s (37.0 km/h) -- up from Revision 2's 6.82 m/s, short of the ~20 m/s asked for (see the distance trade-off above) |
/// | peak speed | 13.66 m/s (49.2 km/h) |
/// | heading swing | -75.4 .. +87.2 deg, 162.5 deg peak-to-peak -- roughly DOUBLE Revision 2's 78.7 deg swing |
/// | heading centring | +5.89 deg off the road direction (Revision 2: +6.75 deg -- essentially unchanged) |
/// | lateral excursion (peak-to-peak) | 13.4 m |
/// | forward distance travelled | 174 m -- inside the ~200 m line |
/// | pitch, whole run | -7.5 .. +2.3 deg -- more excursion than Revision 2's -5.2..+1.5 deg, still well clear of the fallen threshold, no instability |
///
/// **Honest summary: faster and measurably more aggressive than Revision 2, not the full 3x/20 m/s
/// asked for.** The 15.0 s/three-lobe configuration hit the speed target exactly and stayed stable
/// doing it; the limiting factor was City Park's road length, not the board. If more road becomes
/// available (or the clip can accept sailing off the built environment at the very end), the
/// 15.0 s / three-lobe numbers above are the ones to reach for.
///
/// # Revision 4 (issue #161 follow-up): the 200 m budget was a guess, and there was road left
///
/// The COO's own correction, after watching the footage: the ~200 m line Revision 3 was cut back
/// to was invented, inferred from one earlier run, not measured against City Park's actual usable
/// road. The footage showed road still ahead at the end of the (cut-back) run, and the CEO's
/// judgement was explicit: **"we have plenty of room to go faster and turn more."**
///
/// So this revision:
/// - **Takes Revision 3's REJECTED 15.0 s / three-lobe pass as the floor, not the ceiling.**
///   `SCURVE3_BUILD_S` goes back to 15.0 (it was cut to 11.0 only for distance, and the distance
///   reason no longer holds at the same weight). Budget for THIS revision is ~400 m, and even
///   that is explicitly provisional -- "if you exceed it and the run still looks contained I
///   would rather find out from footage than from a guess of mine."
/// - **Lengthens the lobes, per "let the turn last longer left and right."** This is duration,
///   not amplitude -- `lateral`/`steer` stay at their limits (1.0/1.0) and
///   `YAW_RATE_GAIN_RAD_S` stays at 3.0 (host.rs), both already right per the COO. Fewer, longer,
///   deeper carves: two full lobes instead of three, each 1.75x Revision 3's 1.6 s -- a sustained
///   arc the board commits to and holds, not a flick. **A first attempt lengthened the lobes to
///   4.5 s (2.8x) and overshot into a different failure: at this gain and steer, a single lobe
///   swung more than a full 360 deg rotation (520.8 deg of total measured swing), which is
///   spinning, not carving.** See [`SCURVE4_LOBE_S`]'s own doc comment for the cut-back this cost.
///
/// Same standing instruction as every revision: **do not tune around a crash if one appears.**
///
/// # What this revision actually measured (`wire-probe --csv`, final landed configuration)
///
/// | metric | value | vs. Revision 3 |
/// |---|---|---|
/// | weave-entry speed | 15.40 m/s (55.4 km/h) | up from 10.29 m/s (+50%) -- matches the rejected-then-restored 15.2 m/s floor |
/// | peak speed | 18.54 m/s (66.8 km/h) | up from 13.66 m/s |
/// | heading swing | -155.3 .. +131.2 deg, 286.5 deg peak-to-peak | up from 162.5 deg peak-to-peak (+76%) -- a sustained, held arc, NOT a full rotation (see `SCURVE4_LOBE_S`'s doc comment for the 4.5 s attempt that WAS one) |
/// | heading centring | -12.1 deg off road direction | +5.89 deg (same order of magnitude, not chased further) |
/// | lateral excursion (peak-to-peak) | 68.1 m | up from 13.4 m |
/// | forward distance travelled | 335 m | up from 174 m -- inside the ~400 m provisional budget |
/// | pitch, whole run | -7.6 .. +3.2 deg | -7.5 .. +2.3 deg -- essentially unchanged, well clear of the fallen threshold, no instability at any point measured |
///
/// **The board did not become unstable or face-plant at this speed/lobe-duration combination
/// either** -- full stick lean, maxed steer/lateral, 3.0 rad/s yaw gain, 15+ m/s entry, sustained
/// ~2.8 s carves -- reported plainly per the standing instruction, same as every prior revision.
const SCURVE4_SETTLE_S: f64 = 0.5;
const SCURVE4_BUILD_S: f64 = 15.0;
/// Full stick -- unchanged from Revision 3 ("do not be shy with `fore_aft` during the build").
const SCURVE4_BUILD_LEAN: f32 = 1.0;
/// Unchanged from Revision 3: zero. Revision 2 found ANY sustained in-weave trim keeps adding
/// speed rather than merely holding it, more so at higher entry speed -- still true here.
const SCURVE4_WEAVE_TRIM: f32 = 0.0;
/// Full-lobe duration -- lengthened from Revision 3's 1.6 s per "let the turn last longer left
/// and right". **Measured, then cut back once already**: a first attempt at 4.5 s (this constant's
/// initial value) DID make the turn last longer, so much longer that at `YAW_RATE_GAIN_RAD_S`
/// (3.0) and maxed steer/lateral, a single lobe wound up MORE than a full 360 deg rotation --
/// `wire-probe` measured 520.8 deg of total heading swing peak-to-peak, i.e. the board spun in
/// circles rather than carving a held arc. That is not "a rider laying into a turn and holding the
/// line", it is a different failure mode than the ones the request called out (not a crash, not
/// instability -- pitch stayed completely stable through it -- but not the requested shape
/// either). 2.8 s is the re-measured value: still meaningfully longer than Revision 3's 1.6 s
/// (a genuinely held arc, not a flick), short enough that a lobe's own swing stays under one full
/// rotation. See this file's own measured results for the actual swing this landed on.
const SCURVE4_LOBE_S: f64 = 2.8;
/// Entry (and, by the same centring logic, exit) half-lobe duration -- kept at the same ratio to
/// `SCURVE4_LOBE_S` every prior revision used (roughly half), not re-derived: centring in this
/// system has consistently come out close enough at that ratio without further chasing (Revision
/// 2: +6.75 deg, Revision 3: +5.89 deg), and there is no reason to expect that relationship broke
/// just because the lobes got longer rather than the gains or speed changing.
const SCURVE4_ENTRY_S: f64 = 1.4;
/// Lateral and steer stick, at their limits -- unchanged from Revision 3, and explicitly confirmed
/// right by the COO this round ("keep steering and lean at their limits -- those were right").
const SCURVE4_LATERAL: f32 = 1.0;
const SCURVE4_STEER: f32 = 1.0;

const S_CURVE_SCHEDULE: &[(f64, f64, f32, f32, f32, &str)] = &[
    (0.0, SCURVE4_SETTLE_S, 0.0, 0.0, 0.0, "settle"),
    (
        SCURVE4_SETTLE_S,
        SCURVE4_SETTLE_S + SCURVE4_BUILD_S,
        SCURVE4_BUILD_LEAN,
        0.0,
        0.0,
        "lean forward (hard build)",
    ),
    (
        SCURVE4_SETTLE_S + SCURVE4_BUILD_S,
        SCURVE4_SETTLE_S + SCURVE4_BUILD_S + SCURVE4_ENTRY_S,
        SCURVE4_WEAVE_TRIM,
        SCURVE4_LATERAL,
        SCURVE4_STEER,
        "carve right (half lobe, enter)",
    ),
    (
        SCURVE4_SETTLE_S + SCURVE4_BUILD_S + SCURVE4_ENTRY_S,
        SCURVE4_SETTLE_S + SCURVE4_BUILD_S + SCURVE4_ENTRY_S + SCURVE4_LOBE_S,
        SCURVE4_WEAVE_TRIM,
        -SCURVE4_LATERAL,
        -SCURVE4_STEER,
        "carve left (hold the line)",
    ),
    (
        SCURVE4_SETTLE_S + SCURVE4_BUILD_S + SCURVE4_ENTRY_S + SCURVE4_LOBE_S,
        SCURVE4_SETTLE_S + SCURVE4_BUILD_S + SCURVE4_ENTRY_S + 2.0 * SCURVE4_LOBE_S,
        SCURVE4_WEAVE_TRIM,
        SCURVE4_LATERAL,
        SCURVE4_STEER,
        "carve right (hold the line)",
    ),
    (
        SCURVE4_SETTLE_S + SCURVE4_BUILD_S + SCURVE4_ENTRY_S + 2.0 * SCURVE4_LOBE_S,
        SCURVE4_SETTLE_S + SCURVE4_BUILD_S + 2.0 * SCURVE4_ENTRY_S + 2.0 * SCURVE4_LOBE_S,
        SCURVE4_WEAVE_TRIM,
        -SCURVE4_LATERAL,
        -SCURVE4_STEER,
        "straighten (exit lobe)",
    ),
    (
        SCURVE4_SETTLE_S + SCURVE4_BUILD_S + 2.0 * SCURVE4_ENTRY_S + 2.0 * SCURVE4_LOBE_S,
        SCURVE4_SETTLE_S + SCURVE4_BUILD_S + 2.0 * SCURVE4_ENTRY_S + 2.0 * SCURVE4_LOBE_S + 1.5,
        0.0,
        0.0,
        0.0,
        "release (coast)",
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
    // If --kick-at lands after the schedule would otherwise end, extend the
    // run so there is time left afterward to watch the outcome rather than
    // the process exiting mid-fall.
    let duration = kick_at.map_or(total_duration_s(schedule), |ka| {
        total_duration_s(schedule).max(ka + 3.0)
    });
    eprintln!(
        "send-input: sending to {target} for {duration:.1}s per the '{schedule_name}' schedule"
    );

    let start = Instant::now();
    let period = Duration::from_millis(20);
    let mut seq: u64 = 0;
    let mut last_label = "";
    let mut kick_logged = false;

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
        std::thread::sleep(period);
    }

    eprintln!("send-input: schedule complete ({seq} packets sent)");
}
