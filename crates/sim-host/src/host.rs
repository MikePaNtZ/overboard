//! The 500 Hz control loop, dedicated thread, UDP in/out (issue #161).
//!
//! Wires the SAME `control-core`/`safety` objects `control-ffi::ob_controller_
//! update` wires for the Python-hosted scenario, and `board-app-driverless`'s
//! `impulse-response-rust` wires for the Rust-hosted one -- reached here
//! through `hal` (`SimBackend::wait_observe`/`apply`) exactly like that
//! binary, just paced against a UDP client instead of a fixed step count.
//! Nothing here is a new control law.
//!
//! # W2 (issue #161/#169): the RIDDEN plant, and why it does not station-keep
//!
//! This host steps `sim/models/overboard_rider.xml`, not the driverless
//! onewheel model W1 used -- a ridden plant with a rider-scale ballast on
//! two slide joints. `weight_shift_fore_aft` drives the fore/aft joint
//! DIRECTLY AND PHYSICALLY (see that model's own header for the full
//! rationale and the sign convention): shifting the ballast forward moves
//! the effective centre of mass ahead of the axle, and the inner pitch loop
//! below -- which only ever holds the FRAME level, never a ground-speed
//! setpoint -- answers that by accelerating the wheel forward to keep the
//! frame upright. That is the entire mechanism. There is no outer velocity
//! loop here, and none is coming this weekend: `control_core::VelocityLoop`
//! exists and is deliberately unused.
//!
//! **The board does not station-keep, and is not supposed to.** A real
//! onewheel does not either -- lean forward to accelerate, level off to
//! coast at whatever speed that reached, lean back to decelerate or
//! reverse. An earlier revision of this file's controller-config comment
//! called this loop "pure station-keeping balance", which was true of W1's
//! driverless-plant, `pitch_ref`-only inner loop but became actively wrong
//! the moment this file switched to a ridden plant with a driven ballast --
//! nothing in this loop opposes net forward motion, and nothing should.
//!
//! # Issue #163: the heading now lives INSIDE the plant
//!
//! This host used to carry a synthetic heading alongside the physics: a
//! `yaw_rad` integrated from `steer`, composed onto MuJoCo's truth quaternion
//! on the way out, with the ground path dead-reckoned from real forward speed
//! projected along it. MuJoCo's own board only ever translated along one axis
//! underneath, so its true position bore no relation to what a renderer drew
//! -- which is why the drivable corridor had to be a soft host-side lean
//! rather than actual collision geometry.
//!
//! The same steering law now writes its increment straight into the plant's
//! free joint before each physics step (see the yaw block at the bottom of
//! [`run`], and `sim_backend::SimBackend::inject_kinematic_yaw`). **The
//! collision goal never required yaw to be physically GENERATED -- only
//! MuJoCo's pose to be AUTHORITATIVE**, and those are different problems.
//! Steering is still commanded rather than emergent: no tire model produces
//! this turn, and the `Playable Sim` declaration keeps saying so. But
//! position, ground path and every contact are now simulated at that heading,
//! nothing is dead-reckoned, and the MJCF is untouched.
//!
//! ## What it measured
//!
//! Every figure below is master vs this branch, run through
//! `--scripted-scenario` (issue #190/#191's sim-time-indexed player, so the
//! input path has no socket and no wall clock in it) and decoded off the wire
//! at full float precision -- not `wire-probe --csv`, whose 6-decimal
//! rounding cannot demonstrate bit identity.
//!
//! **Zero steer is bit-identical.** `--startup-kick` with no scenario and no
//! sender, so `steer` is 0 for the whole run; 7,854 common ticks. `pitch`,
//! `quat` (all four), `wheel_angle`, `wheel_rate`, `motor_current`,
//! `rider_fore_aft`, `rider_lateral`, `pos_z`, `sim_time` and `flags` are ALL
//! bit-identical. The plant is untouched, which is exactly what the
//! `dyaw == 0` gate exists to guarantee.
//!
//! Only `pos_x`/`pos_y` differ, which is the point of the change: those are
//! where the dead-reckoned path used to be reported and are now MuJoCo's own.
//! **The gap between them is the size of the error the old design was
//! shipping**: 14.7 mm over 7.37 m of travel (0.2%) in x, and 8.2 um in y --
//! where dead reckoning reported y as exactly 0.0, because it could not
//! represent lateral motion at all.
//!
//! **With steer on, the plant lands where the reckoning said it would.**
//! `--scripted-scenario default`, 9,000 ticks, tick-for-tick: `pitch` differs
//! by at most 2.1e-9 rad (1.2e-7 deg), `wheel_rate` by 4.8e-7 rad/s,
//! `motor_current` by 3.9e-7 A, the largest quaternion component by 8.3e-5,
//! and the commanded heading by 1.7e-4 rad. The path itself: 4.4 cm over a
//! 16 m run (0.28%) in x, 1.5 cm in y. The pitch envelope is IDENTICAL to
//! master's on the same schedule, -5.842..+6.390 deg -- the same envelope the
//! corridor-brake run recorded, and well inside the 10 deg carving limit.
//!
//! The reconstruction that comparison rests on is itself checked: replaying
//! the deleted dead-reckoning integral offline against master's own run
//! reproduces master's reported track to 0.0000 m, so it is the deleted
//! block and not an approximation of it. Truth attitude tracks the commanded
//! heading to 0.19 deg, and master's own truth attitude differs from its
//! `yaw_rad` by the same 0.19 deg -- that residual is the plant's own
//! contact-driven yaw, not the injection fighting the physics.
//!
//! **The `s-curve` flip (issue #190) is untouched.** Driven deterministically,
//! master and branch cross 20, 90 and 170 deg of pitch on the IDENTICAL tick
//! (seq 2933 / t = 5.868 s, seq 3070, seq 3216), and every physics column is
//! bit-identical for the first 7,375 ticks -- the whole zero-steer portion of
//! that run, which is where the flip happens. The two traces only separate
//! once `steer` goes non-zero at t = 14.75 s, by which point both boards are
//! already tumbling and diverge chaotically. That flip is issue #190's, and
//! nothing here moves it.

//! # ADR-0011: the command-envelope reserve, and the warning that was being
//! # thrown away
//!
//! Holding full forward stick from rest inverts this board (issue #190).
//! [`CMD_ENVELOPE_RESERVE`] is the fix ADR-0011 specifies -- one multiply on
//! the fore/aft stick, upstream of everything, derived from the actuator
//! envelope rather than tuned to the measured cliff. [`AUTHORITY_
//! UTILISATION_WARN`] is criterion (c): this file used to compute the safety
//! envelope's saturation bit every cycle and discard it at the binding, while
//! `FALLEN` -- which trips about a second after the outcome is decided -- was
//! the only thing anyone was told.
//!
//! **Two of the ADR's criteria are NOT met and no value of the constant would
//! meet them.** Fed MuJoCo truth (criterion (f)) the board inverts at every
//! stick fraction down to 0.05; the reserve buys time-to-inversion, not
//! survival. And during a full-stick hold the board cannot ride out much more
//! than a 1 mm pavement lip (criterion (a), kerb entry). Both are measured in
//! `tests/test_cmd_envelope_reserve.py` and recorded there as strict
//! `xfail`s. Read [`CMD_ENVELOPE_RESERVE`] and [`PitchSource`] before quoting
//! any margin out of this file.
//!
//! Everything under "verification only" on [`HostConfig`] exists to take
//! those measurements -- free running, a trace CSV, a pitch source, a static
//! pitch bias, a scheduled disturbance, and a reserve override. None of it is
//! reachable on a deployed run, and all of it is on the command line rather
//! than in a test fixture, because an acceptance number nobody else can
//! re-take is not a measurement.

use crate::pacer::{JitterPercentiles, Pacer};
use crate::wire::{self, InputIn, StateOut};
use board_types::{
    Command, Faults, ImuSample, Params, Saturation, DEFAULT_R_EFF_M, RAD_S_PER_ERPM,
};
use control_core::{
    CommandFeedforward, ComplementaryFilter, Estimator, PitchRegulator, WheelAccelEstimator,
};
use hal::BoardObserve;
use hal_actuate::BoardActuate;
use safety::Envelope;
use sim_backend::SimBackend;
use std::io::ErrorKind;
use std::net::{SocketAddr, UdpSocket};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

/// Nanoseconds per control cycle -- 500 Hz, matching `sim-backend`'s own
/// `CYCLE_NS` (ICD SS11.2). Duplicated the same way that crate duplicates
/// `KT_NM_PER_A` against the model, because there is no public constant to
/// import; [`run`] asserts it against `SimBackend::run_metadata()` on open
/// rather than trusting the duplication silently.
pub const CYCLE_NS: u64 = 2_000_000;
const DT_S: f64 = CYCLE_NS as f64 * 1e-9;

/// The ridden rider model this host steps (issue #161 W2) -- NOT the
/// driverless onewheel model `sim-backend::SimBackend::with_params` defaults
/// to. Same env-macro pattern `sim-backend`'s own (private) `model_path()`
/// uses, since this crate is at the same `crates/<name>` depth.
fn rider_model_path() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../sim/models/overboard_rider.xml")
}

/// The board state captured on the cycle a terminating event is declared,
/// and repeated on every packet from then until the latch clears (ADR-0012).
///
/// This is what "MuJoCo has stopped propagating" means on the wire: the plant
/// keeps stepping underneath -- freezing the integrator mid-run would strand
/// the control loop and the pacer -- but nothing it computes after the strike
/// reaches the client, so the client is the sole authority in fact and not
/// merely by agreement.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HandoffSnapshot {
    pub pos: [f32; 3],
    pub quat: [f32; 4],
    pub lin_vel: [f32; 3],
    pub ang_vel: [f32; 3],
}

/// A kerb to ride into (ADR-0012). Off by default, and when it is off the
/// model this host opens is the shared `overboard_rider.xml`, byte for byte.
///
/// # Why this is spliced in here and not declared in the model
///
/// `overboard_rider.xml` is shared by every scenario in the repo, several of
/// which travel a long way -- the host coasts 10.8 m in 10 s (issue #169).
/// A kerb committed into that file would start appearing in acceptance runs
/// that never asked for one, and an ADR-0011 measurement that quietly hit
/// scenery is worse than no measurement. Splicing keeps the geometry in the
/// demo that wants it.
///
/// The splice itself is the pattern `sim/scenarios/terrain.py` already uses
/// to add its `hfield` to a model at load time, rather than a new idea.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KerbSpec {
    /// World Y of the kerb's near FACE, metres -- the side the board hits.
    /// The kerb body extends away from the board from there.
    pub y_face_m: f64,
    /// Height of the kerb above the ground plane, metres.
    pub height_m: f64,
}

/// Default kerb, measured from the real City Park geometry rather than
/// invented. A kerb BOTH SIDES, mirrored about the spawn point, because that
/// is what the road actually has.
///
/// # The first value was wrong, and how it was wrong matters
///
/// This was y = 8.85 m, from a search for clusters of "risers" in
/// `dump_triangles.py`'s dump of `OB_City`. Play-test verdict: it did not line
/// up with the kerbs you can see. It was a **building wall** -- the vertical
/// faces at y ~ 8.2-8.6 m span z from -1.6 to +2.9 m, and nothing in a riser
/// filter distinguishes a 4.5 m wall from a 0.2 m kerb.
///
/// Two independent mistakes put it there. The search ranked candidates by
/// TRIANGLE COUNT, and a kerb is a long low-poly extrusion -- the whole road
/// mesh is 16,076 triangles against 8,465,926 for a single grass asset -- so
/// the real kerbs were rejected for being too cheap to notice. And the dump is
/// **94% foliage by triangle count**, so anything averaging "the surface" is
/// really measuring grass and leaf litter.
///
/// Re-measured over LARGE triangles only (>0.02 m^2, which excludes grass
/// blades by orders of magnitude) the picture is unambiguous and matches what
/// a rider sees: near-vertical faces at **y = +-4.5 m**, symmetric about the
/// spawn point, face top at z ~ +0.12 m, road between them at z ~ -0.09 m. A
/// ~9 m road with the board in the middle and a ~0.21 m step either side.
///
/// `height_m` is measured from MuJoCo's flat ground plane (z = 0) up to the
/// kerb top, i.e. what the wheel actually has to climb, not the kerb's full
/// extent including the part buried below the road.
pub const DEFAULT_KERB: KerbSpec = KerbSpec {
    y_face_m: 4.5,
    height_m: 0.12,
};

/// A City Park heightfield, read from the artifacts
/// `overboard-game/tools/terrain_probe/rasterize_hfield.py` writes.
///
/// # The repo boundary this crosses, and why it is the allowed direction
///
/// ADR-0009's rule is that nothing outside `overboard` computes board physics.
/// This is the opposite direction: authored geometry moving INTO the sim's
/// frame, to be simulated here. `overboard-game` measures the world it draws
/// and writes a file; this crate reads it and does all the physics. No board
/// state crosses, and no control decision is tuned from a renderer.
///
/// The path is supplied by the operator rather than hardcoded, so this crate
/// carries no build-time dependency on a sibling checkout -- the coupling is a
/// data contract, exactly as the repo-boundary rule requires.
#[derive(Debug, Clone)]
pub struct TerrainSpec {
    pub hfield_path: PathBuf,
    pub nrow: usize,
    pub ncol: usize,
    pub half_extent_m: f64,
    /// Real-world elevation of the grid's minimum, metres. MuJoCo normalises
    /// hfield file data to [0,1], so the geom has to be placed at this value
    /// for a post to land at the height it was measured at.
    pub z_min_m: f64,
    pub z_max_m: f64,
    /// Terrain height at the board's spawn point, metres. The shared model
    /// spawns the frame at a hardcoded z that assumes a flat plane at zero;
    /// on real terrain the board has to be lifted onto the surface or it
    /// starts the run embedded in the road.
    pub z_at_origin_m: f64,
    /// Where the board spawns along MuJoCo X, metres (`--spawn-x`), and the
    /// terrain height there. The frame (and so the map to Unreal) does not move.
    pub spawn_x_m: f64,
    pub z_at_spawn_m: f64,
}

/// Reads `metadata.json` from the same directory as the hfield binary, and
/// cross-checks it against the binary's own header.
///
/// Both are read because they can disagree: `metadata.json` is written from
/// the rasteriser's parameters while the `.bin` carries its own `nrow`/`ncol`,
/// and a stale metadata file beside a fresh binary would otherwise place the
/// terrain at the wrong scale with nothing to say so.
fn read_terrain_spec(hfield_path: &Path, spawn_x_m: f64) -> Result<TerrainSpec, HostError> {
    let dir = hfield_path.parent().unwrap_or(Path::new("."));
    let meta_path = dir.join("metadata.json");
    let meta_raw = std::fs::read_to_string(&meta_path).map_err(|e| {
        HostError::Io(std::io::Error::new(
            e.kind(),
            format!(
                "sim-host: --terrain needs {} beside the hfield binary (written by \
                 rasterize_hfield.py): {e}",
                meta_path.display()
            ),
        ))
    })?;

    // Deliberately a tiny scan rather than a serde dependency: six scalars out
    // of a file whose schema this crate does not own.
    let pick = |key: &str| -> Result<f64, HostError> {
        let needle = format!("\"{key}\"");
        let at = meta_raw.find(&needle).ok_or_else(|| {
            HostError::Io(std::io::Error::other(format!(
                "sim-host: {} has no \"{key}\"",
                meta_path.display()
            )))
        })?;
        let rest = &meta_raw[at + needle.len()..];
        let rest = rest
            .trim_start()
            .strip_prefix(':')
            .unwrap_or(rest)
            .trim_start();
        let end = rest
            .find(|c: char| !(c.is_ascii_digit() || c == '.' || c == '-' || c == '+' || c == 'e'))
            .unwrap_or(rest.len());
        rest[..end].parse::<f64>().map_err(|_| {
            HostError::Io(std::io::Error::other(format!(
                "sim-host: {} has a non-numeric \"{key}\"",
                meta_path.display()
            )))
        })
    };

    let nrow = pick("nrow")? as usize;
    let ncol = pick("ncol")? as usize;
    let half_extent_m = pick("half_extent_m")?;
    let z_min_m = pick("z_min_m")?;
    let z_max_m = pick("z_max_m")?;

    let raw = std::fs::read(hfield_path).map_err(|e| {
        HostError::Io(std::io::Error::new(
            e.kind(),
            format!("sim-host: cannot read {}: {e}", hfield_path.display()),
        ))
    })?;
    if raw.len() < 8 {
        return Err(HostError::Io(std::io::Error::other(format!(
            "sim-host: {} is too short to be an hfield",
            hfield_path.display()
        ))));
    }
    let bin_nrow = i32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]) as usize;
    let bin_ncol = i32::from_le_bytes([raw[4], raw[5], raw[6], raw[7]]) as usize;
    if bin_nrow != nrow || bin_ncol != ncol {
        return Err(HostError::Io(std::io::Error::other(format!(
            "sim-host: hfield binary is {bin_nrow}x{bin_ncol} but {} says {nrow}x{ncol} -- \
             the metadata is stale relative to the binary, and using it would place the \
             terrain at the wrong scale",
            meta_path.display()
        ))));
    }
    let expect = 8 + nrow * ncol * 4;
    if raw.len() != expect {
        return Err(HostError::Io(std::io::Error::other(format!(
            "sim-host: {} is {} bytes, expected {expect} for a {nrow}x{ncol} f32 grid",
            hfield_path.display(),
            raw.len()
        ))));
    }
    if z_max_m <= z_min_m {
        return Err(HostError::Io(std::io::Error::other(format!(
            "sim-host: degenerate terrain z range [{z_min_m}, {z_max_m}]"
        ))));
    }

    // Centre post = the board's spawn point, by construction of the grid (odd
    // post count so a post lands exactly on x=0, y=0).
    let centre = (nrow / 2) * ncol + (ncol / 2);
    let off = 8 + centre * 4;
    let z_at_origin_m =
        f32::from_le_bytes([raw[off], raw[off + 1], raw[off + 2], raw[off + 3]]) as f64;

    let spacing_m = 2.0 * half_extent_m / (ncol as f64 - 1.0);
    let col = (ncol / 2) as i64 + (spawn_x_m / spacing_m).round() as i64;
    if col < 0 || col >= ncol as i64 {
        return Err(HostError::Io(std::io::Error::other(format!(
            "sim-host: --spawn-x {spawn_x_m} m is outside the terrain (half-extent {half_extent_m} m)"
        ))));
    }
    let off = 8 + ((nrow / 2) * ncol + col as usize) * 4;
    let z_at_spawn_m =
        f32::from_le_bytes([raw[off], raw[off + 1], raw[off + 2], raw[off + 3]]) as f64;

    Ok(TerrainSpec {
        hfield_path: hfield_path.to_path_buf(),
        nrow,
        ncol,
        half_extent_m,
        z_min_m,
        z_max_m,
        z_at_origin_m,
        spawn_x_m,
        z_at_spawn_m,
    })
}

/// The spliced kerb spans the WHOLE drivable corridor in X, derived from
/// `CORRIDOR_X_MIN_M`/`CORRIDOR_X_MAX_M` rather than picked.
///
/// Measured why: at a 40 m half-length about the origin, an s-curve run
/// drifted to the kerb's Y only once it was at x = -112 m -- long past the
/// end of the kerb -- and sailed by with nothing to hit. A kerb the player
/// can outrun is a kerb that is not there.
/// How far inside the heightfield's edge the drivable corridor stops. The
/// grid ends at a cliff with no geom beyond it, and the corridor's lean-arrest
/// is soft -- a board already moving needs room to be turned around.
const TERRAIN_EDGE_MARGIN_M: f64 = 3.0;

/// The frame spawn height in the shared model, metres -- one wheel radius, on
/// the assumption of a flat plane at z = 0. Mirrored here so the terrain
/// splice can fail loudly if the model moves it, rather than silently
/// spawning the board somewhere else.
const FRAME_SPAWN_Z_M: f64 = 0.1454;

/// Extra height above the measured terrain to spawn at, metres. Small enough
/// to settle in a few steps, large enough that no interpolation detail of the
/// hfield cell under the wheel can leave the board starting inside the road.
const TERRAIN_SPAWN_CLEARANCE_M: f64 = 0.005;

const KERB_HALF_LENGTH_M: f64 = (CORRIDOR_X_MAX_M - CORRIDOR_X_MIN_M) / 2.0;
/// Centre of the spliced kerb along X -- the midpoint of the same corridor.
const KERB_CENTRE_X_M: f64 = (CORRIDOR_X_MAX_M + CORRIDOR_X_MIN_M) / 2.0;
/// Half-depth of the spliced kerb along Y, metres. Only the near face is ever
/// touched; the depth exists so the board cannot clip through the far side.
const KERB_HALF_DEPTH_M: f64 = 0.6;

/// Rider centre of mass above the deck pivot (the ballast carrier), m.
const RIDER_COM_HEIGHT_M: f32 = 0.75;
/// Pad mode (fable-oracle, 2026-10-04): the tail pad is on the road. Entered
/// at this nose-up deck pitch, at any speed.
const PAD_MODE_ENTER_RAD: f32 = 17.0 * std::f32::consts::PI / 180.0;
/// Pad mode starts only below this speed, m/s: it is for the stop. A tail
/// drag at speed keeps full motor braking (damping only there removed the
/// braking, and the board ran on and nose-struck after the exit).
const PAD_MODE_SPEED_M_S: f32 = 1.0;
/// Pad mode also ends when the rider leans forward past this stick: the
/// rider asks to go, and the full law lifts the nose off the pad.
const PAD_MODE_GO_STICK: f32 = 0.2;
/// Pad mode ends below this pitch (hysteresis). Raise it toward 15 deg if a
/// pull-away nose-strikes, before any gain changes.
const PAD_MODE_EXIT_RAD: f32 = 15.0 * std::f32::consts::PI / 180.0;
/// Pad mode: damping only, tau = -Kd * pitch rate, limited to this, N*m
/// (about 18 A). With the rider's centre of mass near the tail pad's tipping
/// edge, any larger nose-up torque pivots the board over the tail.
const PAD_MODE_TORQUE_LIMIT_NM: f32 = 12.0;
/// After pad mode, the grade load is not learned for this long, s: the deck
/// dropping back to level is not a grade.
const PAD_MODE_GRADE_HOLD_S: f64 = 1.0;

/// Ankle pivot height above the axle, m (the deck top).
const ANKLE_PIVOT_Z_M: f64 = 0.06;
/// Rider centre-of-pressure reach about the ankle, m: the ankle torque limit
/// is m g times this.
const ANKLE_COP_M: f32 = 0.12;

/// `--rider-reach-back`: the full back reach applies at or above this speed.
const BACK_REACH_FULL_SPEED_M_S: f32 = 2.0;

/// `--foot-torque`: share of the feet's torque limit that full fore/aft
/// stick asks for as heel or toe pressure (the rest balances the body).
const FOOT_TORQUE_SHARE: f32 = 0.8;

/// `--ankle-hinge`: puts the rider's slide carrier on a pitch hinge at deck
/// level (`rider_ankle`, joint `ankle_pitch`, torque motor `ankle_pitch`).
fn splice_ankle_hinge(xml: &str, rigid: bool) -> Result<String, HostError> {
    let bad = |what: &str| HostError::Io(std::io::Error::other(format!("sim-host: --ankle-hinge: {what}")));
    let open = r#"<body name="ballast_fa_carrier" pos="0 0 0.75">"#;
    let camera = r#"<camera name="side""#;
    if xml.matches(open).count() != 1 || xml.matches(camera).count() != 1 || xml.matches("</actuator>").count() != 1 {
        return Err(bad("rider carrier, side camera or actuator block not found once"));
    }
    let carrier_z = 0.75 - ANKLE_PIVOT_Z_M;
    // The bracket's stiff spring is a joint stiffness, integrated inside the
    // step: as a host torque held for 2 ms it was unstable (deck-vs-rider mode
    // about 450 rad/s) and blew up at once.
    let joint_extra = if rigid { r#"stiffness="1e5" damping="300""# } else { r#"damping="0""# };
    let mut out = xml.replace(
        open,
        &format!(
            r#"<body name="rider_ankle" pos="0 0 {ANKLE_PIVOT_Z_M}">
        <joint name="ankle_pitch" type="hinge" axis="0 1 0" pos="0 0 0" {joint_extra}/>
        <inertial pos="0 0 0" mass="0.05" diaginertia="1e-4 1e-4 1e-4"/>
      <body name="ballast_fa_carrier" pos="0 0 {carrier_z:.4}">"#
        ),
    );
    // Close the extra body before the side camera (the carrier's sibling).
    out = out.replace(camera, &format!("</body>\n      {camera}"));
    out = out.replace(
        "</actuator>",
        r#"  <motor name="ankle_pitch" joint="ankle_pitch" gear="1" ctrllimited="false"/>
  </actuator>"#,
    );
    Ok(out)
}

/// `--obstacles`: fixed obstacle geoms from the CSV (type,id,x,y,lx,ly,lz,
/// yaw_deg; full sizes), standing on the terrain (or z = 0 without one).
/// Collision bits 1 and 2: the wheel and pads (bit 2) and the bumpers and
/// rider (bit 1) all hit them.
fn splice_obstacles(
    xml: &str,
    csv: &Path,
    surface: Option<&crate::ground::GroundSurface>,
) -> Result<String, HostError> {
    let bad = |what: String| HostError::Io(std::io::Error::other(format!("sim-host: --obstacles: {what}")));
    if xml.matches("</worldbody>").count() != 1 {
        return Err(bad("no single </worldbody>".into()));
    }
    let text = std::fs::read_to_string(csv)?;
    let mut geoms = String::new();
    let mut n = 0;
    for line in text.lines().map(str::trim).filter(|l| !l.is_empty() && !l.starts_with('#')) {
        let f: Vec<&str> = line.split(',').map(str::trim).collect();
        if f.len() != 8 {
            return Err(bad(format!("expected 8 fields: {line}")));
        }
        let num = |i: usize| f[i].parse::<f64>().map_err(|_| bad(format!("bad number '{}' in: {line}", f[i])));
        let (x, y, lx, ly, lz, yaw) = (num(2)?, num(3)?, num(4)?, num(5)?, num(6)?, num(7)?);
        let z0 = surface.map_or(0.0, |g| g.height(x, y));
        let id = if f[1].is_empty() { format!("obstacle{n}") } else { f[1].to_string() };
        let bits = r#"contype="3" conaffinity="3" condim="3" friction="0.8 0.005 0.0001""#;
        match f[0] {
            "cone" => {
                // A square base and a tapered body as a cylinder of the mean radius.
                let base = 0.03;
                geoms.push_str(&format!(
                    r#"<geom name="{id}_base" type="box" pos="{x} {y} {:.4}" size="{:.4} {:.4} {:.4}" rgba="0.95 0.45 0.1 1" {bits}/>
    <geom name="{id}" type="cylinder" pos="{x} {y} {:.4}" size="{:.4} {:.4}" rgba="0.95 0.45 0.1 1" {bits}/>
    "#,
                    z0 + base / 2.0, lx / 2.0, ly / 2.0, base / 2.0,
                    z0 + base + (lz - base) / 2.0, lx * 0.3, (lz - base) / 2.0,
                ));
            }
            "debris" => geoms.push_str(&format!(
                r#"<geom name="{id}" type="box" pos="{x} {y} {:.4}" euler="0 0 {yaw}" size="{:.4} {:.4} {:.4}" rgba="0.35 0.3 0.25 1" {bits}/>
    "#,
                z0 + lz / 2.0, lx / 2.0, ly / 2.0, lz / 2.0,
            )),
            other => return Err(bad(format!("unknown type '{other}'"))),
        }
        n += 1;
    }
    eprintln!("sim-host: --obstacles: {n} fixed obstacles from {}", csv.display());
    Ok(xml.replace("</worldbody>", &format!("{geoms}</worldbody>")))
}

/// `--tumble`: a free two-part rider (torso and head; legs on a ball hip),
/// parked 50 m up on a weld until a fall. Low-resolution on purpose: it shows
/// how a rider leaves the board and slides, not a human body. It does not
/// touch the board (contact excluded), only the road.
fn splice_tumble_rider(xml: &str) -> Result<String, HostError> {
    if xml.matches("</worldbody>").count() != 1 {
        return Err(HostError::Io(std::io::Error::other("sim-host: --tumble: no single </worldbody>")));
    }
    // Skin on asphalt, about 0.6.
    let f = r#"friction="0.6 0.005 0.0001" condim="3" material="rider_mat" group="0""#;
    let body = format!(
        r#"<body name="rider_free" pos="0 0 50">
      <freejoint name="rider_free_j"/>
      <inertial pos="0 0 0.2" mass="1" diaginertia="0.1 0.1 0.02"/>
      <geom name="rider_torso" type="capsule" fromto="0 0 0.02 0 0 0.50" size="0.15" {f}/>
      <geom name="rider_head" type="sphere" pos="0 0 0.70" size="0.11" {f}/>
      <body name="rider_legs" pos="0 0 0">
        <joint name="rider_hip" type="ball" limited="true" range="0 110" damping="6"/>
        <inertial pos="0 0 -0.33" mass="1" diaginertia="0.06 0.06 0.01"/>
        <geom name="rider_leg_f" type="capsule" fromto="0 0 0 -0.17 0 -0.62" size="0.065" {f}/>
        <geom name="rider_leg_r" type="capsule" fromto="0 0 0 0.17 0 -0.62" size="0.065" {f}/>
      </body>
    </body>
  "#
    );
    let tail = r#"<equality><weld name="rider_park" body1="rider_free"/></equality>
  <contact>
    <exclude body1="rider_free" body2="frame"/>
    <exclude body1="rider_legs" body2="frame"/>
    <exclude body1="rider_free" body2="wheel"/>
    <exclude body1="rider_legs" body2="wheel"/>
  </contact>
"#;
    Ok(xml.replace("</worldbody>", &format!("{body}</worldbody>\n  {tail}")))
}

/// `--rider-reach`: widens the fore/aft slide joint and its servo range to
/// +-`reach` m. The servo gains do not change.
fn splice_rider_reach(xml: &str, reach: f64) -> Result<String, HostError> {
    let bad = |what: &str| HostError::Io(std::io::Error::other(format!("sim-host: --rider-reach: {what}")));
    if !(0.01..=0.25).contains(&reach) {
        return Err(bad("reach must be 0.01..0.25 m"));
    }
    // The joint and servo allow 0.35 m: the stick spans +-reach, and the
    // pad-rest ankle correction may move the rider further.
    let range = format!("{:.4}", reach.max(0.35));
    let joint = r#"<joint name="ballast_fa" type="slide" axis="-1 0 0" pos="0 0 0"
               range="-0.05 0.05""#;
    if xml.matches(joint).count() != 1 {
        return Err(bad("fore/aft slide joint not found"));
    }
    let mut out = xml.replace(joint, &joint.replace("-0.05 0.05", &format!("-{range} {range}")));
    let at = out.find(r#"<position name="ballast_fa""#).ok_or_else(|| bad("fore/aft servo not found"))?;
    let rel = out[at..].find(r#"ctrlrange="-0.05 0.05""#).ok_or_else(|| bad("fore/aft servo range not found"))?;
    out.replace_range(at + rel..at + rel + 22, &format!(r#"ctrlrange="-{range} {range}""#));
    Ok(out)
}

/// Lean-to-steer model changes (see [`crate::lean_steer`]), applied to the
/// shared rider model text. Each replacement must match exactly once, so a
/// change to the shared model fails loudly here rather than silently.
///
/// - Tire: the flat 0.30 m cylinder becomes an ellipsoid with semi-axes
///   0.1454 / 0.12 / 0.1454 m. The rolling radius is unchanged; the tread
///   crown radius is 0.12^2 / 0.1454 = 0.099 m, so the board can roll.
/// - Rider: lateral reach +-0.25 m (knee, hip and body lean),
///   and a faster lateral servo (kp 12000 N/m, 0.05 s lag): a rider balances
///   roll with ~0.2 s reactions, which the 1 Hz fore/aft servo cannot.
fn splice_lean_steer(xml: &str) -> Result<String, HostError> {
    let swaps = [
        (
            r#"<geom name="wheel_geom" type="cylinder" size="0.1454 0.15" euler="90 0 0""#,
            r#"<geom name="wheel_geom" type="ellipsoid" size="0.1454 0.12 0.1454""#,
        ),
        (
            r#"<joint name="ballast_lat" type="slide" axis="0 1 0" pos="0 0 0"
                 range="-0.05 0.05""#,
            r#"<joint name="ballast_lat" type="slide" axis="0 1 0" pos="0 0 0"
                 range="-0.25 0.25""#,
        ),
        (
            r#"<position name="ballast_fa" joint="ballast_fa" kp="3000" ctrlrange="-0.05 0.05"
              ctrllimited="true" timeconst="0.15"/>"#,
            r#"<position name="ballast_fa" joint="ballast_fa" kp="12000" ctrlrange="-0.05 0.05"
              ctrllimited="true" timeconst="0.05"/>"#,
        ),
        (
            r#"<position name="ballast_lat" joint="ballast_lat" kp="3000" ctrlrange="-0.05 0.05"
              ctrllimited="true" timeconst="0.15"/>"#,
            r#"<position name="ballast_lat" joint="ballast_lat" kp="12000" ctrlrange="-0.25 0.25"
              ctrllimited="true" timeconst="0.05"/>"#,
        ),
    ];
    let mut out = xml.to_string();
    let keep_cylinder = std::env::var("OVERBOARD_TIRE").as_deref() == Ok("cyl");
    for (from, to) in swaps {
        if keep_cylinder && from.contains("wheel_geom") {
            continue;
        }
        if std::env::var("OVERBOARD_TIRE").as_deref() == Ok("sphere") && from.contains("wheel_geom") {
            out = out.replace(from, r#"<geom name="wheel_geom" type="sphere" size="0.1454""#);
            continue;
        }
        if out.matches(from).count() != 1 {
            return Err(HostError::Io(std::io::Error::other(format!(
                "sim-host: --lean-steer could not find exactly one '{}' in the rider model -- \
                 the shared model has changed and this splice needs updating",
                from.lines().next().unwrap_or(from)
            ))));
        }
        out = out.replace(from, to);
    }
    Ok(out)
}

/// The X7 build's look and pads (`--plant x7`), from the hardware track's
/// proxy geometry (`sim/models/meshes/openwheel/x7/README.md`).
///
/// - Visual: the frame and wheel meshes replace the Onewheel-style visuals,
///   which move to the hidden render group 5. The meshes' origin is the
///   ground under the axle with +X = nose, so they sit at (0, 0, -0.146)
///   and turn 180 deg (the model's forward is -X).
/// - Collision: a box and a bumper at each end replace the bumper meshes.
///   The boxes are kicked 4.2 deg with the deck; the outer bottom corner,
///   0.346 m from the axle and 0.027 m below it, strikes first, at 20.4 deg.
///   `pad_z` raises or lowers all four.
/// - The nose and tail strike sensors grow to cover the new contact points.
fn splice_x7_geometry(mut xml: String, pad_z: f64, tail_friction: Option<f64>) -> Result<String, HostError> {
    let fail = |what: &str| {
        HostError::Io(std::io::Error::other(format!(
            "sim-host: --plant x7 could not find {what} to splice -- the shared model has changed"
        )))
    };
    // Assets.
    // The look: materials and per-material meshes, generated from the hardware
    // track's look.json by sim/carve/x7_look.py.
    let frag_path = rider_model_path().with_file_name("meshes/openwheel/x7/x7_visual.xml");
    let frag = std::fs::read_to_string(&frag_path).map_err(|e| {
        HostError::Io(std::io::Error::new(
            e.kind(),
            format!("sim-host: --plant x7 needs {} (run sim/carve/x7_look.py): {e}", frag_path.display()),
        ))
    })?;
    let part = |tag: &str| -> Result<String, HostError> {
        let a = frag.find(tag).ok_or_else(|| fail(tag))? + tag.len();
        let b = frag[a..].find("<!-- X7 ").map_or(frag.len(), |i| a + i);
        Ok(frag[a..b].trim().to_string())
    };
    let (assets, frame_geoms, wheel_geoms) =
        (part("<!-- X7 ASSETS -->")?, part("<!-- X7 FRAME -->")?, part("<!-- X7 WHEEL -->")?);
    if xml.matches("</asset>").count() != 1 {
        return Err(fail("</asset>"));
    }
    xml = xml.replace("</asset>", &format!("  {assets}\n  </asset>"));
    // Hide the old visuals (render group 5 is off by default).
    for name in [
        "front_enclosure_geom",
        "rear_enclosure_geom",
        "front_footpad_geom",
        "rear_footpad_geom",
        "electronics_platform_geom",
    ] {
        let from = format!(r#"<geom name="{name}" class="visual""#);
        if xml.matches(&from).count() != 1 {
            return Err(fail(name));
        }
        xml = xml.replace(&from, &format!(r#"<geom name="{name}" class="visual" group="5""#));
    }
    let from = r#"<geom name="wheel_geom" "#;
    if xml.matches(from).count() != 1 {
        return Err(fail("wheel_geom"));
    }
    xml = xml.replace(from, r#"<geom name="wheel_geom" group="5" "#);
    // The new visuals.
    let from = r#"<geom name="electronics_platform_geom""#;
    let at = xml.find(from).ok_or_else(|| fail("electronics_platform_geom"))?;
    let end = at + xml[at..].find("/>").ok_or_else(|| fail("electronics_platform_geom end"))? + 2;
    xml.insert_str(end, &format!("\n      {frame_geoms}"));
    let from = r#"<joint name="wheel_hinge""#;
    let at = xml.find(from).ok_or_else(|| fail("wheel_hinge"))?;
    let end = at + xml[at..].find("/>").ok_or_else(|| fail("wheel_hinge end"))? + 2;
    xml.insert_str(end, &format!("\n        {wheel_geoms}"));
    // Pads: replace each bumper mesh geom with a box and a bumper bar.
    // Proxy boxes (ground frame, +X = nose): box x 0.1658-0.3463, y +-0.120,
    // z 0.106-0.1885; bumper x 0.3463-0.3719, y +-0.145, z 0.146-0.200.
    // Here: forward is -X, and z is from the axle (0.146 above the ground).
    let mu_rear = tail_friction
        .map(|m| format!(r#"priority="1" friction="{m:.3} 0.005 0.0001""#))
        .unwrap_or_else(|| r#"friction="0.6 0.005 0.0001""#.to_string());
    for (end_name, sign, mu) in [
        ("front", -1.0f64, r#"friction="0.6 0.005 0.0001""#.to_string()),
        ("rear", 1.0, mu_rear),
    ] {
        let tag = format!(r#"<geom name="{end_name}_bumper_geom""#);
        let at = xml.find(&tag).ok_or_else(|| fail(&tag))?;
        let stop = at + xml[at..].find("/>").ok_or_else(|| fail(&tag))? + 2;
        // Each end box is kicked 4.2 deg up with the deck: its bottom runs
        // from 0.040 m below the axle at the inner end (0.166 m out) to 0.027 m
        // below at the outer end (0.346 m out), and the outer corner strikes
        // first, at 20.4 deg (hardware track). Box 0.180 x 0.240 x 0.0695 m.
        let bx = sign * 0.2535;
        let bb = sign * 0.3591;
        let tilt = -sign * 4.2; // the outer end up
        let boxes = format!(
            "<geom name=\"{end_name}_box_geom\" type=\"box\" pos=\"{bx:.5} 0 {:.5}\" \
             euler=\"0 {tilt:.1} 0\" size=\"0.0900 0.120 0.03475\" condim=\"3\" {mu} group=\"5\"/>\n      \
             <geom name=\"{end_name}_bumper_geom\" type=\"box\" pos=\"{bb:.5} 0 {:.5}\" \
             size=\"0.0128 0.145 0.027\" condim=\"3\" {mu} group=\"5\"/>",
            0.0012 + pad_z,
            0.027 + pad_z
        );
        xml.replace_range(at..stop, &boxes);
    }
    // Strike sensors: cover the box edge (0.346 m, -0.040 m) and the bumper.
    for (site, x) in [("nose_strike", -0.30f64), ("tail_strike", 0.30)] {
        let from = if x < 0.0 {
            r#"<site name="nose_strike" pos="-0.4072 0 0.0122" size="0.075 0.125 0.050""#
        } else {
            r#"<site name="tail_strike" pos="+0.4072 0 0.0122" size="0.075 0.125 0.050""#
        };
        if xml.matches(from).count() != 1 {
            return Err(fail(site));
        }
        xml = xml.replace(
            from,
            &format!(r#"<site name="{site}" pos="{x:.3} 0 {:.4}" size="0.095 0.165 0.075""#, 0.005 + pad_z),
        );
    }
    Ok(xml)
}

/// Writes `overboard_rider.xml` with `kerb` spliced into its `<worldbody>` to
/// a temporary file, and returns that path. The original file is never
/// modified.
fn write_model_with_kerb(
    kerb: Option<&KerbSpec>,
    terrain: Option<&TerrainSpec>,
    lean_steer: bool,
    max_current_a: Option<f32>,
    variation: PlantVariation,
    smooth_wheel_contact: bool,
    pad_solref_s: f64,
) -> Result<PathBuf, HostError> {
    let src = rider_model_path();
    let xml = std::fs::read_to_string(&src).map_err(|e| {
        HostError::Io(std::io::Error::new(
            e.kind(),
            format!("sim-host: cannot read {}: {e}", src.display()),
        ))
    })?;

    let mut xml = xml;

    // --- Terrain: the real City Park surface, in place of the flat plane ----
    if let Some(t) = terrain {
        // MuJoCo normalises hfield file data to [0,1] and scales it by the
        // asset's elevation term, so the geom must sit at the grid's MINIMUM
        // for a post to land at the height it was measured at:
        //     surface_z = pos_z + normalised * elevation
        //               = z_min  + (h - z_min)                = h
        let elevation = t.z_max_m - t.z_min_m;
        // Depth of solid below the surface. Generous, because the board must
        // not tunnel through a thin shell on a hard kerb strike.
        let base = 2.0_f64;
        let asset = format!(
            "\n    <hfield name=\"citypark\" file=\"{}\" size=\"{:.6} {:.6} {:.6} {:.6}\"/>\n  ",
            t.hfield_path.display(),
            t.half_extent_m,
            t.half_extent_m,
            elevation,
            base
        );
        if xml.matches("</asset>").count() != 1 {
            return Err(HostError::Io(std::io::Error::other(
                "sim-host: expected exactly one </asset> to splice the hfield into",
            )));
        }
        xml = xml.replace("</asset>", &format!("{asset}</asset>"));

        // Replace the plane OUTRIGHT rather than laying terrain over it. A
        // plane left in place would win every contact wherever the road dips
        // below z=0 -- and the City Park road is cambered, falling to about
        // -0.19 m at the gutter, which is exactly where the kerbs are. The
        // kerb would then be unreachable behind an invisible flat floor.
        let plane =
            "<geom name=\"ground\" type=\"plane\" size=\"20 20 0.1\" material=\"ground_mat\"/>";
        if !xml.contains(plane) {
            return Err(HostError::Io(std::io::Error::other(
                "sim-host: could not find the ground plane geom to replace with terrain -- \
                 the shared model has changed and this splice needs updating",
            )));
        }
        // SPAWN THE BOARD ON THE ROAD, not inside it. The shared model puts the
        // frame at z = 0.1454 (wheel radius) because it assumes a flat plane at
        // zero. City Park's road at the spawn point is at +0.0043 m, so the
        // unmodified spawn buries the wheel 4.3 mm into the surface -- and a
        // board that starts penetrating gets a contact impulse on frame one,
        // which the estimator snaps its initial pitch to. Measured before this
        // fix: est_pitch 5.36 deg against a true 0.00 deg, and the board was
        // flat on its face within two seconds. Same failure mode as the reset
        // bug, arriving through a different door.
        //
        // The extra clearance means it settles DOWN onto the road over the
        // first few steps rather than being pushed up out of it.
        let spawn_z = variation.wheel_radius_m.unwrap_or(FRAME_SPAWN_Z_M)
            + t.z_at_spawn_m
            + TERRAIN_SPAWN_CLEARANCE_M;
        let spawn_from = format!("<body name=\"frame\" pos=\"0 0 {FRAME_SPAWN_Z_M}\">");
        if !xml.contains(&spawn_from) {
            return Err(HostError::Io(std::io::Error::other(format!(
                "sim-host: could not find the frame spawn '{spawn_from}' to lift onto the \
                 terrain -- the shared model has changed and this splice needs updating"
            ))));
        }
        xml = xml.replace(
            &spawn_from,
            &format!("<body name=\"frame\" pos=\"{:.6} 0 {spawn_z:.6}\">", t.spawn_x_m),
        );

        xml = xml.replace(
            plane,
            &format!(
                "<geom name=\"ground\" type=\"hfield\" hfield=\"citypark\" \
                 pos=\"0 0 {:.6}\" material=\"ground_mat\" condim=\"3\" \
                 friction=\"0.8 0.005 0.0001\"/>",
                t.z_min_m
            ),
        );
    }

    // --- Kerb: an authored box, for runs without the real terrain ----------
    if lean_steer {
        xml = splice_lean_steer(&xml)?;
    }
    if let Some(reach) = variation.rider_reach_m {
        xml = splice_rider_reach(&xml, reach)?;
    }
    if let Some(amps) = max_current_a {
        // Sizing studies: the motor torque limit follows the current limit.
        let from = r#"<motor name="wheel_motor" joint="wheel_hinge" gear="1" ctrlrange="-28 28""#;
        if xml.matches(from).count() != 1 {
            return Err(HostError::Io(std::io::Error::other(
                "sim-host: --max-current could not find the wheel motor ctrlrange to splice",
            )));
        }
        let nm = amps * KT_NM_PER_A;
        xml = xml.replace(
            from,
            &format!(r#"<motor name="wheel_motor" joint="wheel_hinge" gear="1" ctrlrange="-{nm} {nm}""#),
        );
    }

    if variation.touches_frame() {
        // Board = frame body + rotating wheel + 0.5 kg shift carrier.
        let wheel = variation.wheel_rot_kg.unwrap_or(4.5);
        let board = variation.board_mass_kg.unwrap_or(13.0);
        let frame = board - wheel - 0.5;
        if frame <= 0.5 {
            return Err(HostError::Io(std::io::Error::other(
                "sim-host: the board mass leaves no frame mass (board - wheel - 0.5 kg <= 0.5 kg)",
            )));
        }
        let from = r#"<inertial pos="0 0 -0.03" mass="8.0" diaginertia="0.040 0.400 0.420"/>"#;
        if xml.matches(from).count() != 1 {
            return Err(HostError::Io(std::io::Error::other(
                "sim-host: --board-mass/--plant could not find the frame inertial to splice",
            )));
        }
        let k = variation.frame_inertia_scale.unwrap_or(frame / 8.0);
        let (cx, cz) = variation.frame_com_m.unwrap_or((0.0, -0.03));
        xml = xml.replace(
            from,
            &format!(
                r#"<inertial pos="{cx:.4} 0 {cz:.4}" mass="{frame:.3}" diaginertia="{:.4} {:.4} {:.4}"/>"#,
                0.040 * k,
                0.400 * k,
                0.420 * k
            ),
        );
    }
    if variation.wheel_rot_kg.is_some() || variation.wheel_spin_kgm2.is_some() {
        let from = r#"<inertial pos="0 0 0" mass="4.5" diaginertia="0.0635 0.0595 0.0635"/>"#;
        if xml.matches(from).count() != 1 {
            return Err(HostError::Io(std::io::Error::other(
                "sim-host: --plant could not find the wheel inertial to splice",
            )));
        }
        let m = variation.wheel_rot_kg.unwrap_or(4.5);
        let spin = variation.wheel_spin_kgm2.unwrap_or(0.0595);
        // Keep the model's ratio of the transverse to the spin inertia.
        let side = spin * 0.0635 / 0.0595;
        xml = xml.replace(
            from,
            &format!(r#"<inertial pos="0 0 0" mass="{m:.3}" diaginertia="{side:.5} {spin:.5} {side:.5}"/>"#),
        );
    }
    if let Some(r) = variation.wheel_radius_m {
        let mut n = 0;
        for (from, to) in [
            (
                r#"<geom name="wheel_geom" type="ellipsoid" size="0.1454 0.12 0.1454""#.to_string(),
                format!(r#"<geom name="wheel_geom" type="ellipsoid" size="{r:.4} 0.12 {r:.4}""#),
            ),
            (
                r#"<geom name="wheel_geom" type="cylinder" size="0.1454 0.15""#.to_string(),
                format!(r#"<geom name="wheel_geom" type="cylinder" size="{r:.4} 0.15""#),
            ),
            (
                r#"<geom name="wheel_geom" type="sphere" size="0.1454""#.to_string(),
                format!(r#"<geom name="wheel_geom" type="sphere" size="{r:.4}""#),
            ),
        ] {
            if xml.contains(&from) {
                xml = xml.replace(&from, &to);
                n += 1;
            }
        }
        if n != 1 {
            return Err(HostError::Io(std::io::Error::other(
                "sim-host: --plant radius could not find exactly one wheel geom to splice",
            )));
        }
        // Spawn on the ground, not inside it: on the plane the frame sits at
        // the axle height. (On terrain the spawn splice above uses the radius.)
        let plane_spawn = format!(r#"<body name="frame" pos="0 0 {FRAME_SPAWN_Z_M}">"#);
        if xml.contains(&plane_spawn) {
            xml = xml.replace(&plane_spawn, &format!(r#"<body name="frame" pos="0 0 {r:.4}">"#));
        }
    }
    if let Some(m) = variation.rider_mass_kg {
        // Same inertia formula as the model's 70 kg ballast (mass * 0.15, 0.15, 0.08).
        let from = r#"<inertial pos="0 0 0" mass="70.0" diaginertia="10.5000 10.5000 5.6000"/>"#;
        if xml.matches(from).count() != 1 {
            return Err(HostError::Io(std::io::Error::other(
                "sim-host: --rider-mass could not find the ballast inertial to splice",
            )));
        }
        xml = xml.replace(
            from,
            &format!(
                r#"<inertial pos="0 0 0" mass="{m:.3}" diaginertia="{:.4} {:.4} {:.4}"/>"#,
                m * 0.15,
                m * 0.15,
                m * 0.08
            ),
        );
    }
    if let Some(mu) = variation.tail_friction {
        // priority 1: this geom's friction wins over the road's.
        let from = r#"<geom name="rear_bumper_geom" type="mesh" mesh="rear_bumper" material="bumper_mat"
            group="2" condim="3" friction="0.6 0.005 0.0001"/>"#;
        if xml.matches(from).count() != 1 {
            return Err(HostError::Io(std::io::Error::other(
                "sim-host: --tail-friction could not find the rear bumper geom to splice",
            )));
        }
        xml = xml.replace(
            from,
            &format!(
                r#"<geom name="rear_bumper_geom" type="mesh" mesh="rear_bumper" material="bumper_mat"
            group="2" condim="3" priority="1" friction="{mu:.3} 0.005 0.0001"/>"#
            ),
        );
    }
    if variation.x7_geometry {
        xml = splice_x7_geometry(xml, variation.pad_z_m.unwrap_or(0.0), variation.tail_friction)?;
    }
    if let Some(k) = variation.kt_scale {
        // The true torque per commanded amp. ctrl stays the commanded current
        // times the NOMINAL Kt, so the current limit is unchanged.
        let from = r#"<motor name="wheel_motor" joint="wheel_hinge" gear="1" "#;
        if xml.matches(from).count() != 1 {
            return Err(HostError::Io(std::io::Error::other(
                "sim-host: --kt-scale could not find the wheel motor gear to splice",
            )));
        }
        xml = xml.replace(
            from,
            &format!(r#"<motor name="wheel_motor" joint="wheel_hinge" gear="{k:.4}" "#),
        );
    }

    if let (Some(t), true) = (terrain, smooth_wheel_contact) {
        // Smooth wheel contact (crate::ground): the tire collides with a
        // mocap plate only (bit 2), and everything else with the heightfield
        // (bit 1). The host moves the plate under the tire every cycle.
        let from = r#"<geom name="wheel_geom" "#;
        if xml.matches(from).count() != 1 {
            return Err(HostError::Io(std::io::Error::other(
                "sim-host: smooth wheel contact could not find the wheel geom to splice",
            )));
        }
        xml = xml.replace(from, r#"<geom name="wheel_geom" contype="2" conaffinity="2" "#);
        // The nose and tail pads ride the same plate. On the heightfield a
        // dragged pad caught every grid seam: touch, jump, touch, with 10 kN
        // spikes under a 100 kg rider. The pads sit 0.41 m from the axle, and
        // on a 60 m vertical curve the plate is within 1.3 mm of the road
        // there. Cost: the pads no longer hit kerbs; --hfield-wheel-contact
        // restores both for kerb studies.
        for pad in ["front_bumper_geom", "rear_bumper_geom", "front_box_geom", "rear_box_geom"] {
            let from = format!(r#"<geom name="{pad}" "#);
            if pad.ends_with("_box_geom") && !xml.contains(&from) {
                continue; // only the X7 pads have boxes
            }
            if xml.matches(&from).count() != 1 {
                return Err(HostError::Io(std::io::Error::other(format!(
                    "sim-host: smooth contact could not find {pad} to splice"
                ))));
            }
            // Pad contact time constant (`--pad-solref`): MuJoCo's default 0.02 s
            // is a near-rigid pad, and a dragged tail then taps at about 16 Hz
            // with 8.6 kN spikes. 0.05 s stands in for a plastic or urethane
            // pad (spikes 4 kN). The stopping distance is the same for both.
            xml = xml.replace(
                &from,
                &format!(r#"<geom name="{pad}" contype="2" conaffinity="2" solref="{pad_solref_s} 1" "#),
            );
        }
        let th = crate::ground::PLATE_HALF_THICKNESS_M;
        let half = crate::ground::PLATE_HALF_SIZE_M;
        let plate = format!(
            "\n    <!-- sim-host smooth wheel contact, NOT part of the shared model. -->\n    \
             <body name=\"wheel_ground\" mocap=\"true\" pos=\"{:.6} 0 {:.6}\">\
             <geom name=\"wheel_ground_geom\" type=\"box\" size=\"{half} {half} {th}\" \
             contype=\"2\" conaffinity=\"2\" condim=\"3\" friction=\"0.8 0.005 0.0001\" \
             rgba=\"0 0 0 0\" group=\"3\"/></body>\n  ",
            t.spawn_x_m,
            t.z_at_spawn_m - th,
        );
        if xml.matches("</worldbody>").count() != 1 {
            return Err(HostError::Io(std::io::Error::other(
                "sim-host: expected exactly one </worldbody> to splice the wheel plate into",
            )));
        }
        xml = xml.replace("</worldbody>", &format!("{plate}</worldbody>"));
    }

    if let Some(kerb) = kerb {
        let hz = kerb.height_m / 2.0;
        let mut geom = String::from(
            "\n    <!-- ADR-0012: spliced in by sim-host --kerb, NOT part of the shared model. -->",
        );
        for (tag, sign) in [("kerb_left", 1.0f64), ("kerb_right", -1.0f64)] {
            let cy = sign * (kerb.y_face_m.abs() + KERB_HALF_DEPTH_M);
            geom.push_str(&format!(
                "\n    <geom name=\"{tag}\" type=\"box\" pos=\"{KERB_CENTRE_X_M} {cy} {hz}\" \
                 size=\"{KERB_HALF_LENGTH_M} {KERB_HALF_DEPTH_M} {hz}\" \
                 rgba=\"0.62 0.60 0.57 1\" condim=\"3\" friction=\"0.8 0.005 0.0001\"/>"
            ));
        }
        geom.push_str("\n  ");

        if xml.matches("</worldbody>").count() != 1 {
            return Err(HostError::Io(std::io::Error::other(format!(
                "sim-host: expected exactly one </worldbody> in {} to splice the kerb into",
                src.display()
            ))));
        }
        xml = xml.replace("</worldbody>", &format!("{geom}</worldbody>"));
    }

    // Alongside the real model, so MuJoCo's `meshdir` (a RELATIVE path) still
    // resolves. A temp dir would break every mesh reference in the file.
    let dst = src.with_file_name(format!(
        "overboard_rider_kerb_{}.generated.xml",
        std::process::id()
    ));
    std::fs::write(&dst, xml).map_err(|e| {
        HostError::Io(std::io::Error::new(
            e.kind(),
            format!("sim-host: cannot write {}: {e}", dst.display()),
        ))
    })?;
    Ok(dst)
}

// --- Controller config, ridden variant (issue #161 W2) -- REUSED verbatim
// from `sim/scenarios/shuttle_run.py`'s own `controller_factory`, the repo's
// only existing tuned cascade config for a rider-scale plant. That scenario
// also runs an outer `VelocityLoop` this host does NOT enable (see this
// file's header) -- the inner-loop gains and estimator config below are
// reused unchanged regardless, because they are the closest available
// precedent for THIS plant's mass/inertia, not for the outer loop's absence.
//
// 2026-10-05: the gains are 3x and 1.73x the shuttle_run values (140, 21).
// The old Kp was below the rider's gravity stiffness (m g h about 660
// N*m/rad), so the law made torque only by letting the deck droop, which
// works only with a rider rigid with the deck. The VESC float package's
// Angle P (about 20 A/deg, about 800 N*m/rad here) is stiffer still. Rider-law
// Monte Carlo, X7, 200 runs: x1 191 PASS / 6 FALL; x3 196 PASS / 0 FALL;
// x3 with stage0 sensors at 2x noise and +5 ms delay 199 PASS / 0 FALL (x1
// there: 5 nose strikes); x3 with an upright (ankle) rider 0 FALL.
const KP_NM_PER_RAD: f32 = 420.0;
const KD_NM_PER_RAD_S: f32 = 36.3;
const KT_NM_PER_A: f32 = 0.7;
const MAX_CURRENT_A: f32 = 40.0;
const ESTIMATOR_TAU_S: f32 = 2.0;
/// Command-feedforward gain, m/s^2 per amp. `control_ffi::ob_controller_new`'s
/// own hardcoded fallback for `kt` 0.7 N*m/A / (`r_eff` 0.1454 m * 82.5 kg
/// total ridden mass) -- 8 kg frame + 4.5 kg wheel + 0.5 kg ballast carrier +
/// 70 kg rider = 82.5 kg, matching `overboard_rider.xml` exactly.
/// `shuttle_run.py`'s tuned controller relies on this same FFI fallback
/// rather than passing its own gain, so this host does too, duplicated here
/// because this host does not link `control-ffi`.
const ACCEL_FF_GAIN_M_S2_PER_A: f32 = 0.0584;
/// `impulse-response-rust`'s own constant (`crates/board-app-driverless/src/
/// bin/impulse-response-rust.rs`), which mirrors `RustController()`'s
/// `wheel_accel_tau_s` default (`sim/scenarios/rust_controller.py`).
/// Duplicated here for the same reason `ACCEL_FF_GAIN_M_S2_PER_A` above is:
/// this host does not link `control-ffi`.
const WHEEL_ACCEL_TAU_S: f32 = 0.05;
/// Time constant of the grade load learned by [`control_core::GradeAwareAiding`], s.
const GRADE_LOAD_TAU_S: f32 = 0.3;

/// `weight_shift_fore_aft` / `weight_shift_lateral`, both clamped to
/// `[-1, 1]` on the wire, map linearly onto this range -- the SAME +/-0.05 m
/// range `overboard_rider.xml`'s `ballast_fa` / `ballast_lat` joints
/// declare, so full-stick deflection lands exactly on the joint's own limit
/// rather than a separately chosen number that could silently drift from it.
const BALLAST_RANGE_M: f32 = 0.05;

/// Non-physical game channel, SHAPED by [`ROLL_FULL_YAW_AUTHORITY_RAD`]
/// below -- the simulated wheel is a cylinder and cannot physically carve
/// (issue #161). The real lean-steer controller is Tuesday.
///
/// **Unchanged in value and in law by issue #163's kinematic in-plant yaw
/// injection.** What changed is only where the resulting heading is APPLIED:
/// this same `steer * k * v * roll_authority` rate is now integrated into
/// MuJoCo's own free joint before each physics step, instead of being
/// composed onto the plant's output afterwards. The steering feel is signed
/// off and must not move; see [`run`]'s own yaw block.
///
/// # Revision history (moved through 1.5 -> 3.0 -> speed-proportional)
///
/// Started at 1.5 rad/s flat. Raised to 3.0 (issue #161 follow-up, CEO
/// request via the COO: "be more aggressive" for the launch-capture
/// scripted scenarios) -- a flat rad/s gain, still.
///
/// **Then the CEO drove it himself and found the real problem with a flat
/// gain**: "you can just straight up turn yourself around a bit too fast in
/// a way that I don't think you could actually do on a onewheel... it would
/// be on a very large turning radius." A flat rad/s yaw rate is independent
/// of ground speed -- the board could spin in place at a dead stop, and
/// turned equally fast at any speed, backwards from every real vehicle
/// (turn radius shrinks as you slow down, not the other way round). Fixed
/// by making yaw rate proportional to ground speed instead -- see
/// [`YAW_CURVATURE_PER_STEER_RAD_PER_M`], which replaces this constant
/// (the "gain" is no longer a flat rad/s; it's curvature per metre
/// travelled). Still an invented number for a declared non-physical
/// channel -- this changes nothing about the honesty position, only the
/// shape of the invented behaviour.
///
/// Turn radius = ground speed / yaw rate = `1 / (steer * k)` with this new
/// formulation -- roughly CONSTANT for a given `steer`, regardless of
/// speed, which is what leaning into a carve actually does; at zero speed,
/// yaw rate is zero too -- you cannot carve a stationary onewheel.
///
/// `k` (this constant) is picked for a believable radius at full steer, not
/// derived: at ANY speed at or above [`YAW_TIGHTEN_REF_SPEED_M_S`] the
/// tightest achievable radius is `1 / k` -- a wide, committed arc rather than
/// a go-kart-tight spin. "Tune for feel, he will judge" -- this is still a
/// first approximation, not a hit-an-exact-number derivation.
///
/// # Issue #201 -- 0.15 measured too wide, doubled to 0.30
///
/// The CEO's own report: **"turn radius is too wide by maybe 2x to turn
/// around completely."** `0.15` was quoted as ≈6.7 m from the formula above
/// and never actually driven through a full turn to check; issue #201 did,
/// with `--scripted-scenario turnaround` (`tests/test_turn_radius.py`) and a
/// ground-track measurement rather than the formula alone, because full-sim
/// turning also passes through `roll_authority`
/// (`YAW_AUTHORITY_FLOOR`/`ROLL_FULL_YAW_AUTHORITY_RAD`, added after this
/// constant was first picked) and the low-speed tighten ramp, neither of
/// which the plain `1 / k` formula accounts for.
///
/// **BEFORE (0.15):** measured **6.16 m** at 8.67 m/s ground speed (93% of
/// the 9.34 m/s reference speed), climbing toward the formula's 6.67 m
/// asymptote as speed approaches the reference -- i.e. the formula was
/// right, `roll_authority` saturates to ~1.0 in practice and does not
/// explain the discrepancy from a naive expectation; the number was simply
/// too wide, as reported.
///
/// **AFTER (0.30, this constant):** median **3.284 m** (10th-90th percentile
/// spread 0.096 m) across samples within 5% of the 9.34 m/s reference speed,
/// comfortably inside issue #201's `≤ 3.6 m` acceptance bound, and inside the
/// drivable corridor (`CORRIDOR_HALF_WIDTH_M` = 8.6 m half-width) throughout
/// the measurement hold -- max lateral excursion 5.05 m, not the ±8.6 m the
/// pre-fix radius would need. The median is used rather than the max because
/// the local `dheading/dt` estimate is numerically unstable for the 1-2
/// samples exactly at the ground-speed peak (`dv/dt -> 0` there); see
/// `tests/test_turn_radius.py` for the full method and the outliers this
/// excludes. Doubling `k` exactly halves the radius at every speed (the
/// formula is linear in `k`, confirmed by measurement, not just assumed).
///
/// `full_steer_at_a_standstill_injects_nothing` and
/// `the_turn_radius_shrinks_as_the_board_slows` (this file's own unit tests)
/// both re-derive their expectations from this constant directly and needed
/// no changes for the retune -- see their own doc comments.
const YAW_CURVATURE_PER_STEER_RAD_PER_M: f32 = 0.30;

/// How much tighter the turn gets at a standstill than at top speed, as a
/// multiple of [`YAW_CURVATURE_PER_STEER_RAD_PER_M`].
///
/// # Why this exists
///
/// [`YAW_CURVATURE_PER_STEER_RAD_PER_M`]'s own doc comment says the defect it
/// fixed was a board that "turned equally fast at any speed, backwards from
/// every real vehicle (**turn radius shrinks as you slow down**, not the other
/// way round)". It then made yaw RATE proportional to speed, which gives a
/// radius that is *constant* — better than the original, but still not the
/// behaviour the comment describes. Radius never shrank.
///
/// The CEO found the gap by driving it: *"in general i should be able to cut a
/// tight turn by breaking and turning at same time while keeping speed."* With
/// a speed-independent radius, braking into a turn cannot tighten it by any
/// amount, so the manoeuvre does not exist.
///
/// # What it does
///
/// Curvature ramps linearly from `1x` at [`YAW_TIGHTEN_REF_SPEED_M_S`] and
/// above, to this multiple at a standstill. At full steer and
/// `roll_authority` 1.0 that is a 6.67 m radius at top speed and 3.33 m as the
/// board comes to rest, passing through ~4.6 m at 5 m/s.
///
/// **The stationary board still cannot carve.** Yaw rate keeps its `|v|`
/// factor, so it remains zero at zero speed however tight the curvature gets
/// — the property `full_stick_at_zero_ground_speed_does_not_turn_the_board`
/// pins, and the one the original flat-gain formulation got wrong.
///
/// # Status
///
/// A FEEL number on a declared non-physical channel, picked for the manoeuvre
/// the CEO described and not derived from anything. 2x is deliberately modest:
/// it is a noticeable tightening under braking rather than a pivot. Nothing
/// here is a physics claim and no control decision may be tuned from it.
const YAW_LOW_SPEED_TIGHTEN: f32 = 2.0;

/// Speed at or above which the turn is at its widest, m/s — the reference
/// point [`YAW_LOW_SPEED_TIGHTEN`] ramps down from.
///
/// [`MAX_GROUND_SPEED_M_S`], so the ramp spans exactly the board's usable
/// speed range and the tightening is felt across all of it rather than only
/// in the last stretch before a stop. Above the cap the curvature simply
/// stays at its widest.
const YAW_TIGHTEN_REF_SPEED_M_S: f32 = MAX_GROUND_SPEED_M_S;

/// Ground-speed cap, m/s -- issue #161 follow-up, the CEO's explicit ask:
/// "look at the top speed of a Onewheel XR and set that limit +10% and cap
/// that." Future Motion's own widely-published Onewheel XR spec lists a top
/// speed of 19 mph (8.49 m/s); +10% = 20.9 mph = 9.34 m/s.
///
/// **Cited from well-known, publicly-repeated Onewheel XR spec material,
/// NOT a live re-verified lookup** -- this environment has no web access
/// this session, and the COO's own instruction was explicit ("verify that
/// figure rather than taking mine... this number is about to become a
/// public-facing claim of realism"). Flagging the distinction rather than
/// presenting this as independently re-confirmed: **spot-check the current
/// Future Motion Onewheel XR spec page against 19 mph before this cap is
/// treated as a defensible public claim**, per the standing project rule
/// against presenting an unverified number as verified.
///
/// Enforced as an authority cap on `weight_shift_fore_aft`, not a clamp on
/// the physical wheel speed itself -- this plant has no outer velocity loop
/// (see this file's own header) and no other mechanism that HOLDS a speed,
/// so the only way to cap it is to remove the rider's ability to keep
/// commanding forward lean once at the limit, the same way a real
/// Onewheel's pushback works. See the ballast-target section below for the
/// implementation and [`SPEED_CAP_MARGIN_M_S`] for why it ramps rather than
/// cuts off sharply. This also supersedes the "rip" scripted-scenario
/// tuning: `send-input`'s s-curve peaked at 18.5 m/s, roughly double this
/// cap -- that clip is now historical, not a target to preserve.
const MAX_GROUND_SPEED_M_S: f32 = 9.34;

/// How far below [`MAX_GROUND_SPEED_M_S`] the fore/aft accelerating
/// authority starts ramping down, so the cap is a smooth taper rather than
/// a hard on/off wall that could chatter (cross the cap, lose all forward
/// authority, decay slightly under drag, regain authority, repeat). Not
/// bench-tuned -- a documented default.
///
/// **Measured** (`wire-probe --csv`, sustained full lean from rest): speed
/// climbs smoothly and peaks at 9.95 m/s -- about 6.5% over the 9.34 m/s
/// cap, then settles back down through 9.3 m/s within about a second as the
/// authority ramp (and the ballast's own rate limit) catch up -- before
/// continued carving pulls it down further. Reported honestly rather than
/// re-tuned to hit the cap exactly: a documented, bounded overshoot, not an
/// unbounded one. Tighten this margin (or start the ramp earlier) if a
/// tighter cap is wanted; a smaller margin trades a harder-edged feel for
/// less overshoot.
const SPEED_CAP_MARGIN_M_S: f32 = 1.0;

/// The speed at which [`SPEED_CAP_MARGIN_M_S`]'s authority ramp starts
/// withdrawing accelerating fore/aft authority -- 8.34 m/s. Named because
/// ADR-0011's loss-of-authority warning discriminates on it (see
/// [`AUTHORITY_UTILISATION_WARN`]), not merely because the speed cap
/// arithmetic uses it.
const SPEED_CAP_ONSET_M_S: f32 = MAX_GROUND_SPEED_M_S - SPEED_CAP_MARGIN_M_S;

/// Fraction of full fore/aft stick the command map actually delivers --
/// ADR-0011 exit criterion (b), "cap commanded lean, changing nothing else".
/// Applied to `weight_shift_fore_aft` UPSTREAM of the estimator, the
/// regulator and the safety envelope, so it is pure input shaping: no gain
/// moves, `MAX_CURRENT_A` does not move (that would be a claim about
/// hardware), `MAX_GROUND_SPEED_M_S` does not move (rejected as dominated --
/// it costs top speed and this does not).
///
/// # DERIVED, NOT TUNED -- and specifically NOT tuned to the measured cliff
///
/// Holding full forward stick from rest inverts this board in ~6.5 s
/// (issue #190). The flip boundary was measured between 0.95 and 0.97 of
/// full stick, and **tuning to that boundary is forbidden**: it rests partly
/// on the estimator's operating trim and partly on `wheel_hinge`'s
/// undocumented `damping="0.08"`. A constant sitting on either is a constant
/// that moves when somebody changes something unrelated.
///
/// The derivation instead runs off the actuator envelope, per the
/// command-map rule ADR-0011 propagates to the hardware spec -- *the
/// stick->setpoint map SHALL be derived from the actuator envelope minus a
/// stated disturbance reserve*:
///
/// 1. **Measured** -- peak fore/aft current demand is linear in stick at
///    [`PEAK_DEMAND_A_PER_UNIT_STICK`] = **42.03 A per unit**. Full stick
///    therefore demands 42.03 A against a 40 A envelope: the present map
///    **over-commands by 5%**, which is a normalisation defect, not a sizing
///    gap.
/// 2. **Stated reserve** -- hold peak demand to
///    [`STATED_ENVELOPE_RESERVE_FRACTION`] = 84% of [`MAX_CURRENT_A`], a
///    policy input rather than a measurement, and labelled as one.
/// 3. **Derived:** `0.84 * 40 A / 42.03 A per unit = 0.799`, to two figures
///    **0.80**. Pinned as executable arithmetic by
///    `the_command_envelope_reserve_is_the_stated_reserve_divided_by_the_
///    measured_slope`, not merely written down here.
///
/// # TRIM-DERIVED, not geometry-derived -- read this before porting it
///
/// The step above says "measured", and the word is load-bearing in a way the
/// phrase *derived from the actuator envelope* can easily be read past.
/// [`PEAK_DEMAND_A_PER_UNIT_STICK`] was measured **at the estimator's current
/// operating trim**, and it is a property of that trim, not of the vehicle.
/// Measured in `test_f2_the_peak_demand_slope_is_trim_derived_not_geometry_
/// derived`: shifting the trim by 0.10 deg moves the slope by ~5.2%, which
/// is already outside the 5% band its own provenance check allows.
///
/// So this constant is honest for the frozen trim and for nothing else. It
/// does not travel to hardware, to a retuned filter, or to a different
/// operating point on its own authority. Describing it as geometry-derived --
/// as something the actuator envelope alone fixes -- would be a
/// rationalisation, and ADR-0011's second ratification says so in those
/// words. The trim it rests on is pinned by
/// `test_f1_the_estimator_trim_is_pinned_so_a_retune_cannot_move_it_silently`
/// precisely so this constant cannot go stale quietly; ADR-0011 names any
/// retune that moves that band as a blocking prerequisite for the
/// headroom-based fix, which is the version of this that would NOT be
/// trim-derived.
///
/// # What it buys, measured
///
/// Against the highest stick fraction that survived unshaped (0.95 -- 1.00
/// inverts, so it cannot be the baseline), over a 60 s full-stick hold on the
/// deployed estimator path:
///
/// | | 0.95 (unshaped baseline) | 0.80 (shipped) |
/// |---|---|---|
/// | peak demand | 40.25 A -- envelope reached | **33.41 A (83.5%)** |
/// | peak lean | 9.94 deg | **7.96 deg** |
/// | pitch reserve vs the 11.46 deg ceiling | 13% | **31%** |
///
/// The ceiling is `MAX_CURRENT_A * KT / KP` = 28/140 = 0.2 rad = 11.46 deg.
/// (Measured with the old Kp 140; at Kp 420 the ceiling is a third of that.)
///
/// # What it costs, measured
///
/// Top speed is **unchanged**: the board settles at 9.150 m/s against the
/// baseline's 9.176 m/s (-0.3%), because `MAX_GROUND_SPEED_M_S` governs it
/// and a shaped full stick still reaches the cap. Time to 8 m/s from rest
/// rises 5.910 s -> 6.832 s (**+0.92 s**) and distance over the first 15 s
/// falls 108.83 m -> 100.23 m (**-7.9%**).
///
/// # What it does NOT buy -- read this before quoting the numbers above
///
/// The reserve satisfies criterion (b) and two of criterion (a)'s three
/// matrix entries **on the deployed estimator path**. It does not satisfy
/// the other two, and no value of it would:
///
/// - **Criterion (f) -- fed MuJoCo truth -- fails at every value.** Sweeping
///   this constant from 1.00 down to 0.05 moves time-to-inversion from 4.42 s
///   to 28.67 s and never removes it; only exactly zero stick survives.
///   Feeding the regulator truth deletes the acceleration reference the lean
///   is generated from, so a sustained forward lean has no equilibrium and
///   the reserve buys TIME, not survival. ADR-0011's second ratification
///   replaces this criterion with freeze-and-pin for exactly that reason.
/// - **Criterion (a)'s kerb-strike entry fails** above roughly a 1 mm lip
///   struck at the worst point of a full-stick hold.
///
/// Both are measured in `tests/test_cmd_envelope_reserve.py`, which records
/// them as strict `xfail`s -- the criteria are written there as they should
/// read, and they will turn red the day somebody fixes the underlying
/// problem, which is the only way a known failure stays known.
const CMD_ENVELOPE_RESERVE: f32 = 0.80;

/// The command-envelope reserve spent when the stick OPPOSES the current
/// motion — i.e. when the rider is braking rather than accelerating.
///
/// # Why braking gets its own number at all
///
/// [`CMD_ENVELOPE_RESERVE`] was derived from a peak-demand slope measured on
/// the **forward full-stick hold**, where the board accelerates from rest and
/// full stick over-commands the envelope by 5%. Applying that same 0.80 to a
/// braking command was never derived, only inherited: the shaping was one
/// multiply and the sign never entered it. The cost is that the rider gets
/// 80% of the lean they asked for when trying to stop, for a reason that only
/// ever applied to setting off.
///
/// The CEO's report from driving the build is the ask this answers: *"you
/// should be able to stop faster by leaning back [...] but we don't need to
/// overdo it."*
///
/// # Why the test is OPPOSITION, not sign
///
/// The naive version of this is "aft stick gets more authority". That
/// reintroduces the ADR-0011 defect in mirror image: full aft from rest is a
/// backward standing start, which over-commands the envelope exactly as the
/// forward one does. Braking is not a direction, it is a *relationship*
/// between the command and the motion, so the test is `stick * speed < 0` —
/// the same one the speed cap already uses.
///
/// # What it costs, and why this value
///
/// Braking authority is not free: it is spent out of the same envelope, and
/// the worst point in ADR-0011's acceptance matrix — the full reverse-to-
/// forward stick reversal at speed — is a braking event by this definition.
/// Raising this constant eats that entry's headroom directly, and the two
/// cannot be separated, because braking hard from speed IS that load case.
///
/// At 0.90 the stop is 9.3% quicker and 7.7% shorter, and the reversal's
/// pitch headroom falls from 1.72 deg to **0.47 deg** (current headroom 4.60
/// A to 3.41 A). **That is thin, and it is a deliberate CEO decision taken
/// with those numbers in front of him**, not a default — recorded here
/// because a future reader finding 0.47 deg of margin deserves to know it was
/// chosen rather than stumbled into. The sweep is in
/// `tests/test_braking_authority.py`: 0.95 exceeds the pitch ceiling outright
/// and 1.00 inverts the board.
///
/// ADR-0011 criterion (b) quotes the pre-change margin and needs amending.
///
/// Deliberately NOT raised to 1.0. At 1.0 the braking command is unshaped,
/// which reproduces the over-command the reserve exists to remove — the
/// direction it acts in is the only thing that changed.
const CMD_ENVELOPE_RESERVE_BRAKING: f32 = 0.90;

/// Ground speed below which no command counts as braking, m/s.
///
/// Without this the opposition test fires on the standing start it is meant
/// to exclude. **Measured, not guessed:** a full-stick launch from rest rocks
/// the board BACKWARD to -0.085 m/s between t = 0.504 s and t = 1.112 s
/// before it moves off, so for six tenths of a second the forward stick
/// genuinely opposes the motion and would have drawn the braking reserve.
/// That would have changed ADR-0011's `full-stick` acceptance run — the one
/// the estimator trim and [`PEAK_DEMAND_A_PER_UNIT_STICK`] are both pinned
/// against — for a manoeuvre with no braking in it at all.
///
/// 0.25 m/s is about 3x that transient. Well under walking pace, so no stop
/// a rider would call a stop begins below it, and well above anything the
/// launch rock produces. The property that matters is pinned by
/// `the_braking_reserve_is_not_spent_on_a_standing_start`.
const BRAKING_RESERVE_MIN_SPEED_M_S: f32 = 0.25;

/// The braking reserve's two bounds, enforced by the COMPILER.
///
/// Equal to [`CMD_ENVELOPE_RESERVE`] would make the constant inert; 1.0 would
/// reproduce the over-command the reserve exists to remove, with only the
/// direction it acts in changed. And a full braking command must not demand
/// the whole envelope on its own, or there is nothing left for the
/// disturbance the reserve is named for.
const _: () = {
    assert!(
        CMD_ENVELOPE_RESERVE_BRAKING > CMD_ENVELOPE_RESERVE,
        "CMD_ENVELOPE_RESERVE_BRAKING at or below CMD_ENVELOPE_RESERVE buys nothing -- \
         delete it rather than shipping a constant that does not act"
    );
    assert!(
        CMD_ENVELOPE_RESERVE_BRAKING < 1.0,
        "an unshaped braking command is exactly the over-command ADR-0011's reserve \
         was introduced to remove"
    );
    assert!(
        CMD_ENVELOPE_RESERVE_BRAKING * PEAK_DEMAND_A_PER_UNIT_STICK < MAX_CURRENT_A,
        "a full braking command must not demand the whole actuator envelope on its own"
    );
};

/// The measurement [`CMD_ENVELOPE_RESERVE`] is derived FROM, in amps of peak
/// fore/aft current demand per unit of fore/aft stick.
///
/// A named constant rather than a number in a sentence, so the derivation is
/// arithmetic a test can check rather than prose a reader has to trust --
/// see `the_command_envelope_reserve_is_the_stated_reserve_divided_by_the_
/// measured_slope`.
///
/// **Re-measured for ADR-0011, not inherited.** Nine runs of
/// `--scripted-scenario full-stick --pitch-source estimator` at stick
/// fractions 0.20 through 0.95, peak `|proposed_amps|` taken PRE-envelope
/// (the clamp cannot hide demand); least squares through the origin gives
/// 42.03 A/unit at R^2 = 0.99936, and the per-point ratio never leaves
/// 41.6-43.5 A/unit. The ADR quoted 42 A/unit from an independent
/// measurement; these agree to 0.1%.
///
/// Measured on the ESTIMATOR path deliberately, even though ADR-0011
/// criterion (f) runs acceptance against MuJoCo truth. A command map is a
/// property of the command path, and the estimator is the only signal path a
/// real board has; more decisively, the truth-fed runs have no steady state
/// to take a peak demand FROM (see the criterion (f) results in
/// `tests/test_cmd_envelope_reserve.py`), so the slope is not measurable
/// there at all.
const PEAK_DEMAND_A_PER_UNIT_STICK: f32 = 42.03;

/// The stated disturbance reserve [`CMD_ENVELOPE_RESERVE`] is derived
/// against: peak fore/aft demand is held to this fraction of
/// [`MAX_CURRENT_A`], leaving the rest for disturbance rejection.
///
/// **A stated policy input, not a measurement** -- inherited from ADR-0011,
/// and labelled as such rather than dressed up as derived. What is derived
/// is the stick scale that delivers it. 0.84 leaves 16% (6.4 A) of the
/// envelope.
///
/// It is worth saying plainly what 6.4 A does and does not cover: it is
/// about 0.2 rad/s of pitch-rate rejection through `KD`, which is a small
/// disturbance. The reserve is sized against the envelope because that is
/// the number this repo can defend; it is NOT sized against the repo's own
/// reference disturbance, which is far larger than the envelope can answer
/// at any stick setting (issue #142, and criterion (a)'s kerb-strike entry).
const STATED_ENVELOPE_RESERVE_FRACTION: f32 = 0.84;

/// The derivation, enforced by the COMPILER rather than by a reader.
///
/// [`CMD_ENVELOPE_RESERVE`] is not an independent number: it is the stated
/// reserve fraction times the envelope, divided by the measured slope,
/// rounded to two figures. Editing any one of those four in isolation is a
/// build error, which is the point -- ADR-0011 forbids moving the constant to
/// make a scenario pass, and a rule enforced at compile time cannot be
/// forgotten by a session that never read the ADR.
///
/// The tolerance is half a rounding step. Two figures is what the
/// measurement supports; see [`PEAK_DEMAND_A_PER_UNIT_STICK`]'s spread.
const _: () = {
    let derived = STATED_ENVELOPE_RESERVE_FRACTION * MAX_CURRENT_A / PEAK_DEMAND_A_PER_UNIT_STICK;
    let err = CMD_ENVELOPE_RESERVE - derived;
    assert!(
        err < 0.005 && err > -0.005,
        "CMD_ENVELOPE_RESERVE is no longer STATED_ENVELOPE_RESERVE_FRACTION * \
         MAX_CURRENT_A / PEAK_DEMAND_A_PER_UNIT_STICK rounded to two figures. \
         Re-derive the constant from the measurement; do not adjust the \
         measurement to fit the constant."
    );
};

/// Time constant of the low-pass filter on authority utilisation, seconds --
/// the loss-of-authority warning's detector (ADR-0011 exit criterion (c)).
///
/// Not bench-tuned, and bounded rather than picked: it has to be long
/// relative to the 2 ms control period (or the warning chatters on a single
/// noisy cycle) and short relative to the lead time it exists to preserve
/// (or the filter eats the warning it is supposed to give). 0.1 s is 50
/// control cycles and 4% of the measured lead -- comfortably inside both
/// bounds, which is the whole requirement on it.
const AUTHORITY_UTILISATION_TAU_S: f32 = 0.1;

/// Filtered `|proposed current| / MAX_CURRENT_A` above which the host warns
/// that it is running out of pitch authority -- ADR-0011 exit criterion (c).
///
/// # The discriminator is the SPEED, not the saturation
///
/// Saturation alone is not a fault on this board. ADR-0011's measured
/// finding: **every run that saturated above [`SPEED_CAP_ONSET_M_S`]
/// survived, and every run that saturated below it flipped.** Once torque
/// is clamped `tau = -kp*theta` no longer holds and pitch is open-loop
/// unstable, so saturation is survivable if and only if something is
/// already unloading the board when it happens -- above the onset the speed
/// cap is doing exactly that. A warning that fired on saturation above the
/// onset would be noise, and noise is how a warning gets ignored.
///
/// # Why filtered utilisation rather than the raw saturation bit
///
/// The raw bit (`Saturation::Yes` out of `safety::Envelope::apply`, which
/// this host used to discard at the `let (bounded_cmd, _sat)` binding) fires
/// only once the envelope has ALREADY bound. Filtered utilisation crosses
/// 0.85 while the controller still has authority left, which is the
/// difference between a warning and a post-mortem.
///
/// # Measured lead, against the reference ADR-0011 uses
///
/// `--scripted-scenario full-stick --cmd-reserve 1.0`, estimator path. The
/// reference event is the FIRST ENVELOPE SATURATION -- the ADR's table puts
/// it at 4.92 s and quotes `FALLEN` at -0.95 s, which is exactly `FALLEN`
/// minus that:
///
/// | event | sim time | lead over the committed point |
/// |---|---|---|
/// | this warning | 3.000 s | **+1.92 s** |
/// | first saturation (the reference) | 4.920 s | 0 |
/// | `FALLEN` (20 deg) | 5.868 s | **-0.95 s** |
/// | inverted past 90 deg | 6.142 s | -1.22 s |
///
/// So the warning leads `FALLEN` by **2.87 s**. ADR-0011 predicted 2.69 s of
/// lead over the committed point; the measurement is **1.92 s** with this
/// filter and threshold, and 2.02 s unfiltered. Recorded as measured rather
/// than reconciled -- the ADR's figure is not reproducible from the
/// definition it states, and quoting it anyway would be exactly the habit
/// that ADR exists to break.
///
/// # What this warning is NOT
///
/// It is a diagnostic, not a safety control. The board it warns about cannot
/// ride out a 2 mm pavement lip during a full-stick hold (criterion (a)'s
/// kerb entry, `tests/test_cmd_envelope_reserve.py`), and 1.92 s of notice
/// does not change that. It also does not reach the game: the state-out wire
/// is fixed by ADR-0010 until it expires, so this goes to the host's log and
/// to the acceptance trace and nowhere else. Putting it on the wire is a
/// schema change and needs that ADR's expiry or the COO's sign-off.
const AUTHORITY_UTILISATION_WARN: f32 = 0.85;

/// Roll magnitude, radians, at which the roll-shaped yaw limiter reaches
/// full authority (issue #161 W2 item 4).
///
/// **MEASURED, not guessed -- and the honest number is tiny.** The first cut
/// of this constant (10 deg) was a placeholder that turned out to be ~400x
/// too high: issue #169's follow-up measured 0.267 deg of yaw over the whole
/// `send-input` turn phase against a 10 deg threshold, which is what a
/// limiter that never leaves its floor looks like. Diagnosed by holding a
/// steady balance controller and commanding `ballast_lat` directly
/// (`sim/models/overboard_rider.xml`, full stick = 0.05 m target): the
/// actuator DOES reach its commanded position (~0.0401 m at 0.8 stick, ~full
/// convergence, ruling out a wiring/scaling bug) -- but the resulting
/// steady-state roll tops out at only **~0.032 deg at full stick (1.0)**.
/// The widened wheel geom (issue #161 W2, same PR) is the reason: a much
/// wider flat cylinder rim sitting on the ground plane resists tipping far
/// more than the original narrow tire did, and a 70 kg / 0.05 m lateral
/// shift's ~24.5 N*m of roll torque is nowhere near enough to peel it.
///
/// Recalibrated to slightly below that measured ceiling (not the ceiling
/// itself) so a genuinely full-stick lateral command reliably saturates
/// `roll_authority` to 1.0 with some margin, rather than sitting just under
/// it. This is a real physical limit of the current geometry, not a
/// placeholder -- but see [`YAW_AUTHORITY_FLOOR`] below for why this
/// threshold alone does NOT make yaw usable.
const ROLL_FULL_YAW_AUTHORITY_RAD: f32 = 0.025 * std::f32::consts::PI / 180.0;

/// Floor on `roll_authority`, below which the roll gate would otherwise clamp
/// `steer` to near-zero.
///
/// **Read this before touching the roll gate again.** At the widened wheel's
/// achievable roll (~0.03 deg, see [`ROLL_FULL_YAW_AUTHORITY_RAD`]'s doc
/// comment), a PURE roll-gated limiter is not "shaped by lean" in any
/// perceptible sense -- lean this small is not something a player can feel
/// or control, so without a floor the gate would function as an near-binary
/// on/off switch that happens to correlate weakly with the lateral stick,
/// not a smooth lean-to-turn feel. That is not what issue #161 W2 asked
/// for, and pretending otherwise in a comment is exactly the mistake #169
/// already caught twice in this crate.
///
/// So: **this gate is now largely cosmetic, and steer is effectively the
/// primary driver of yaw this weekend.** The floor guarantees full `steer`
/// always produces a usable turn (tens of degrees over a few seconds, not
/// tenths) regardless of how much roll the geometry can actually deliver;
/// the roll term still nudges authority up smoothly from the floor toward
/// 1.0 as lean increases, which is the connection to lean the design intends
/// to keep and what the real lean-steer controller (Tuesday) inherits --
/// it just cannot be the WHOLE story on this geometry. 0.35 is chosen so
/// full steer alone clears a "tens of degrees" turn even at zero lean
/// (`0.6 (steer) * 1.5 rad/s * 0.35 * a few seconds`); not bench-tuned.
const YAW_AUTHORITY_FLOOR: f32 = 0.35;

/// How long a stale input is still trusted before the host zeroes it rather
/// than continuing to act on a value from a client that may have gone away.
/// 50 control cycles (100 ms at 500 Hz): generous relative to any plausible
/// Unreal send rate, tight relative to a human's reaction time. A documented
/// default, not a bench-tuned number.
///
/// **Not as generous as that reasoning assumed** (issue #190): this crate's
/// own `send-input`, which claimed 50 Hz, was measured delivering 7-13 Hz on
/// a loaded dev laptop -- straddling this threshold, so a run-varying
/// fraction of every scripted run silently ran with the input zeroed. That is
/// fixed in the sender (absolute-deadline pacing) rather than by widening
/// this timeout, which is a safety property and stays where it is. Public
/// so a harness can quote the number it has to beat.
pub const INPUT_STALENESS_TIMEOUT: Duration = Duration::from_millis(100);

/// Placeholder threshold for the state-out FALLEN bit.
/// `sim/scenarios/disturbance_envelope.py` derives the REAL nose-strike
/// angle (18.57 deg) from the model's collision hulls; this crate has no
/// Rust binding to that geometry query, so this is a fixed proxy near that
/// value, not the real contact test -- good enough to prove "the board is
/// clearly down", not precise enough to gate a published claim.
/// Sign of the `--grade-course` gyro fix-up in the IMU pitch axis.
const GRADE_GYRO_SIGN: f32 = 1.0;
/// Sim time at which `--start-speed` is applied, s (see `start_speed_pending`).
const START_SPEED_AT_S: f64 = 1.0;
const FALLEN_PITCH_RAD: f32 = 20.0 * std::f32::consts::PI / 180.0;

/// Soft, host-side drivable-corridor bounds, checked against **MuJoCo's own
/// truth position** -- issue #161 follow-up, item 4: the CEO wants the board
/// to stop passing through walls, curbs and map boundaries.
///
/// **CHANGED (issue #163, kinematic in-plant yaw injection): this used to be
/// checked in the DEAD-RECKONED frame, and no longer is.** The original
/// version of this comment explained at length why MuJoCo-frame bounds could
/// not work: `pos`'s x/y on the wire were dead-reckoned from real forward
/// speed projected along a synthetic heading, MuJoCo's own plant never turned
/// (it only ever translated along its own -X axis), so the position the
/// physics engine occupied bore no relation to where the board appeared on
/// screen. That is no longer true. The host now rotates the board's free
/// joint inside the plant every tick, so MuJoCo integrates the ground path
/// itself and its truth position IS the on-screen position. These bounds are
/// therefore now checked against `truth_pos_x_m`/`truth_pos_y_m` directly.
///
/// This remains a SOFT corridor -- a lean applied against travel, not a
/// contact -- because the model still declares no wall geometry. What has
/// changed is that adding some would now work: collision geometry in the MJCF
/// would be tested against the same position the renderer draws, which is the
/// thing the dead-reckoned design foreclosed.
///
/// Derived from the actual UE <-> MuJoCo origin mapping the COO supplied
/// (`OB_City`'s `BoardActor`, printed on launch: "MuJoCo origin -> UE
/// (-3880.0, -7450.0, -275.0) cm, yaw 90.0 deg"), not guessed outright, for
/// the ONE figure this repo already has independent measurement for: the
/// lateral half-width reuses `send-input`'s own already-measured value for
/// the SAME spawn point ("the road at OB_City's spawn carries road-level
/// surface to ~8.6 m either side" -- that crate's Revision 3 doc comments).
/// The longitudinal bounds have no equivalent measured figure anywhere in
/// this repo, so they are a deliberately generous placeholder -- wide
/// enough that no run measured so far against this issue (the longest,
/// 335 m, `send-input`'s Revision 4) comes close to it in either direction.
/// **Named constants specifically so the COO can tighten or widen them
/// from footage**, per their own explicit instruction.
///
/// **Verified the enforcement itself, not just the arithmetic** (`wire-probe
/// --csv`): since no scripted scenario on hand reaches these production
/// bounds (see above), `CORRIDOR_X_MIN_M` was temporarily tightened to -3.0
/// m for the measurement run and `send-input`'s `default` schedule (the
/// stable, already-validated AC schedule -- not `s-curve`) driven straight
/// at it. The edge-triggered log fired exactly once, at `(-3.0, 0.0)`;
/// the reported position (then dead-reckoned, now truth) overshot to -10.3 m
/// under residual momentum before the
/// brake arrested it, settling to a steady -9.4..-10.3 m band for the rest
/// of the run rather than continuing to run away -- an active brake against
/// momentum, not a teleport back to the line, exactly as designed. Pitch
/// stayed within -5.8..+6.4 deg throughout: the corridor brake itself did
/// not destabilize the board. Bound reverted to -700.0 after the
/// measurement.
///
/// **Re-verified after the switch to truth position** (issue #163), by the
/// identical method -- `CORRIDOR_X_MIN_M` temporarily tightened to -3.0 m,
/// `send-input`'s `default` schedule driven at it, bound reverted afterwards.
/// The edge-triggered log fired exactly once, again at `(-3.0, 0.0)`; truth
/// `x` overshot to -7.6 m under residual momentum, then held at -7.3 m for
/// the rest of the run, and pitch stayed within -4.7..+6.2 deg. Same
/// behaviour, and the overshoot differs from the -10.3 m above only because
/// the board is carrying a different speed at the crossing -- `send-input`
/// is a wall-clock sender against a tick-paced host, so no two runs of the
/// same schedule cross the line at the same moment.
const CORRIDOR_X_MIN_M: f64 = -700.0;
const CORRIDOR_X_MAX_M: f64 = 50.0;
const CORRIDOR_HALF_WIDTH_M: f64 = 8.6;

/// Fore/aft lean the corridor forces once the board is outside
/// [`CORRIDOR_X_MIN_M`]/[`CORRIDOR_X_MAX_M`]/[`CORRIDOR_HALF_WIDTH_M`],
/// opposing whatever direction it was travelling -- an actual brake, not
/// merely "stop accelerating further", which alone would let the board
/// coast on out of the corridor under its own residual speed. Same order
/// of magnitude as `send-input.rs`'s own `SCHEDULE`'s "lean back
/// (decelerate)" value (0.6); not bench-tuned.
const CORRIDOR_BRAKE_LEAN: f32 = 0.6;

/// ADR-0012 terminating event, part 1: total bumper contact force, newtons,
/// above which the board is declared down and physics authority is handed to
/// Unreal.
///
/// The two `touch` sensors in `overboard_rider.xml` read **exactly** 0.0000 N
/// through upright riding -- measured over 6000 uncontrolled steps, the
/// site geometry excludes the wheel's contact patch on two independent axes
/// -- and 560 N with the board resting on a bumper. So this threshold is not
/// separating signal from noise (there is no noise); it is set well clear of
/// zero purely so a grazing touch during a recoverable wobble does not end a
/// run, and well under the resting load so an actual strike cannot be missed.
const STRIKE_FORCE_N: f32 = 50.0;

/// ADR-0012 terminating event, part 2: tilt of the frame's up-axis away from
/// world vertical, radians, above which the board is declared down.
///
/// # Why this exists when `FALLEN_PITCH_RAD` already does
///
/// `FALLEN` tests **pitch alone**, and a kerb is hit from the SIDE. The
/// measured failure: released from upright this model settles at 18.6 deg of
/// pitch -- the nose-into-ground limit its own header documents -- which
/// never reaches the 20 deg `FALLEN` threshold at all. A board rolled onto
/// its side by a kerb strike can therefore be flat on the ground with
/// `FALLEN` reading 0 and, if it went over sideways rather than end-over,
/// neither bumper touched.
///
/// Tilt-from-vertical is one dot product (`xmat[8]`, the frame up-axis's
/// world Z component) and is direction-agnostic by construction, so it covers
/// pitch, roll and any combination.
///
/// **`FALLEN` is deliberately left alone.** Its threshold and its measured
/// lead times are quoted in ADR-0011 (warning asserts at 3.000 s, `FALLEN` at
/// 5.868 s, lead 2.868 s); redefining it would silently invalidate published
/// numbers. This is a separate, additional signal that only gates the
/// handoff.
///
/// # Where 35 deg comes from -- measured, not chosen
///
/// The risk this number carries is the FALSE POSITIVE: a threshold a hard
/// carve can reach would hand the board to Unreal mid-turn and ragdoll a
/// player who was still riding. So the legitimate envelope was measured
/// first, over all four scripted scenarios, 30 s each, with no kerb:
///
/// | scenario | peak tilt |
/// |---|---|
/// | `default` | 5.5 deg |
/// | `s-curve` | 8.0 deg |
/// | `full-stick` | 8.0 deg |
/// | `stick-reversal` | **11.0 deg** |
///
/// 35 deg is 3.2x the worst of those. For contrast, on the run that actually
/// went down the board crossed this threshold and was at 166 deg of pitch
/// moments later -- once past ~35 deg it is not coming back, so the margin
/// costs nothing in detection latency.
const HANDOFF_TILT_RAD: f32 = 35.0 * std::f32::consts::PI / 180.0;

/// A ONE-TIME startup disturbance, applied near the beginning of a run when
/// [`HostConfig::startup_kick`] is set. Not a general disturbance API -- the
/// input wire carries no such field (issue #161) -- and OFF BY DEFAULT
/// (issue #169): on the ridden plant, an ungated kick was measured handing
/// the board roughly 1.1 m/s before a player had touched anything, which is
/// enough to leave a finite Unreal level in the first few seconds of load.
/// Kept available, not deleted, for `wire-probe`/diagnostic runs that want a
/// guaranteed disturbance to show recovery from. Same magnitude and duration
/// `impulse-response-rust` uses (issue #107 AC6: `NOMINAL_IMPULSE_NS` = 20
/// N*s over 0.05 s), reused rather than invented; direction and application
/// point mirror that binary's own `ImpulseParams` defaults too (force along
/// -X, zero torque). **World frame** (issue #194): fired at `t=1.0s`, before
/// any steer input, so this has never yet been able to land at a non-zero
/// heading -- but the constant itself does not know that and does not rotate
/// with heading if that ever changes.
const STARTUP_KICK_T0_S: f64 = 1.0;
const STARTUP_KICK_DURATION_S: f64 = 0.05;
const STARTUP_KICK_FORCE_N: [f64; 3] = [-(20.0 / STARTUP_KICK_DURATION_S), 0.0, 0.0];

/// An ON-DEMAND disturbance (issue #161 follow-up, item 5): "make falls
/// testable" -- a repeatable, rising-edge-triggered "knock the board over
/// now" via [`wire::INPUT_FLAG_KICK`], reusing this same
/// `apply_external_force` mechanism the startup kick above already gates.
/// Deliberately a SEPARATE, much larger magnitude, not a reuse of
/// `STARTUP_KICK_FORCE_N` -- that one is sized to be RECOVERABLE (its own
/// doc comment: "a guaranteed disturbance to show recovery FROM"), and
/// measurement confirmed it as such on this plant: `wire-probe --csv`
/// against the 20 N*s/0.05 s startup kick alone never crossed
/// `FALLEN_PITCH_RAD` -- pitch stayed within a few degrees and recovered.
/// This one is sized to reliably NOT be recoverable. Measured
/// (`wire-probe --csv`'s new `fallen` column, decoded off the actual wire
/// bytes -- not re-derived from `pitch_rad` -- board at rest, zero
/// weight-shift/steer throughout, the ONLY input the whole run): 400 N*s
/// over 0.05 s (20x the startup kick) crossed `FALLEN_PITCH_RAD` (20 deg)
/// ~0.056 s after the kick's own force-application window (measured against
/// SIMULATED time, i.e. tick count * `DT_S` -- a verification run under this
/// dev sandbox's CPU contention showed the sender's own wall clock and the
/// host's simulated clock can drift apart by multiple real seconds, so this
/// crate's other schedule-timed tools, e.g. `send-input`'s wall-clock
/// `--kick-at`, should not be trusted for tight timing correlation here).
/// The board then went past a full flip (+-180 deg) and stayed there for
/// the rest of a 10 s run -- genuinely unrecoverable, not borderline. Same
/// direction/torque convention as the startup kick (force along -X, zero
/// applied torque -- the pitching moment comes from the wheel-ground
/// contact being below the force's application point, not from any
/// deliberately-applied torque). **World frame, not body frame** (issue
/// #194): this is operator-triggered and CAN land while the board is mid-carve
/// at a non-zero heading (unlike the startup kick above, which cannot). It
/// pushes along world -X regardless -- an exogenous "something hit the board"
/// shove is what this exists to simulate, and that does not rotate with the
/// board any more than a real kerb does. If a future need wants this to read
/// as "push the nose" during a turn, that is a body-frame push and belongs at
/// the call site (rotate before it reaches [`select_disturbance_force_torque`]),
/// not a change to this constant.
const FALL_KICK_DURATION_S: f64 = 0.05;
const FALL_KICK_FORCE_N: [f64; 3] = [-(400.0 / FALL_KICK_DURATION_S), 0.0, 0.0];

/// How much simulated time a scripted run keeps stepping for after its
/// schedule has finished, so the outcome of the last phase is visible rather
/// than the process exiting mid-manoeuvre. Same intent as `send-input`'s own
/// `--kick-at` tail.
const SCRIPTED_RUN_TAIL_S: f64 = 3.0;

/// Default path for the host's own missed-deadline/tick counters -- internal
/// tooling for `wire-probe`, NOT part of the Unreal wire (issue #161's wire
/// table has no room for either, and must not grow one for our own
/// convenience). Overwritten atomically (write-then-rename) a few times a
/// second; best-effort, since losing a write here must never interrupt the
/// control loop.
pub const DEFAULT_STATS_PATH: &str = "/tmp/overboard-sim-host-stats.txt";

/// What [`run`] needs to know before it starts.
pub struct HostConfig {
    /// Where state-out packets are sent.
    pub state_out_addr: SocketAddr,
    /// Where the input-in socket binds/listens.
    pub input_in_addr: SocketAddr,
    /// `None` runs forever; `Some(d)` stops after `d` has elapsed --
    /// primarily for tests and verification runs, not something a deployed
    /// host would set.
    pub duration: Option<Duration>,
    /// `None` disables the stats file entirely.
    pub stats_path: Option<PathBuf>,
    /// Applies the one-time startup disturbance if true. See
    /// [`STARTUP_KICK_T0_S`]'s doc comment for why this defaults to false.
    pub startup_kick: bool,
    /// **Verification only.** When set, the host plays this
    /// [`crate::scenario`] schedule itself, indexed on SIMULATED time, and
    /// ignores the input socket entirely for `weight_shift_*`/`steer`.
    ///
    /// # Why this exists (issue #190)
    ///
    /// The scripted scenarios were previously only playable through
    /// `send-input`, which indexed them on a WALL clock and shipped them over
    /// UDP into a 100 ms staleness gate. On a loaded, non-realtime dev host
    /// that made the effective input sequence a function of machine load:
    /// the same command produced a materially different run each time, and
    /// "this scenario is stable" was being concluded from a de-rated
    /// delivery of it (see [`crate::scenario`]'s header for the measured
    /// numbers).
    ///
    /// Indexing on `sim_time_s` removes the wall clock, the socket, the
    /// sender process and the staleness gate in one move. The plant is a
    /// fixed-timestep RK4 integrator with no stochastic terms, so with the
    /// input schedule pinned to tick count the whole run becomes repeatable
    /// -- which is what makes "does this schedule flip the board?" a question
    /// with an answer, rather than a coin flip.
    ///
    /// This is NOT how a deployed host runs, and it is not a substitute for
    /// `send-input`: the UDP path is the one Unreal actually uses, and only
    /// `send-input` exercises it.
    pub scripted_scenario: Option<crate::scenario::Schedule>,

    /// **Verification only.** Stops the run after this much SIMULATED time,
    /// independently of `duration`'s wall clock. A free-running acceptance
    /// sweep has no wall clock worth bounding, and a scripted run's own
    /// schedule length is not always the window a measurement wants.
    pub max_sim_time_s: Option<f64>,

    /// **Verification only.** Where the regulator's pitch and pitch rate come
    /// from. See [`PitchSource`]; ADR-0011 exit criterion (f) is why this
    /// exists.
    pub pitch_source: PitchSource,

    /// **Verification only.** Which signal aids the complementary filter's
    /// accelerometer branch. See [`EstimatorAiding`]; issue #227.
    pub estimator_aiding: EstimatorAiding,

    /// **Verification only.** A static pitch offset, DEGREES, added to
    /// whatever [`HostConfig::pitch_source`] hands the regulator. Positive is
    /// nose-up, matching ICD 10.1.
    ///
    /// This is the robustness test ADR-0011 criterion (f) was reaching for.
    /// The ADR asks for acceptance "with the estimator bias removed" and
    /// implements that as "feed the regulator MuJoCo truth" -- but the
    /// measured `est - truth` residual is not a static bias at all: settled,
    /// it is `atan(a_unaided / g)` to within 3%, i.e. 1 radian per g, i.e.
    /// the complementary filter reading the APPARENT VERTICAL. Replacing it
    /// with truth therefore does not remove an error, it deletes an
    /// acceleration reference and leaves a pitch-only regulator (see
    /// `PitchSource::PlantTruth`).
    ///
    /// Injecting a worst-case STATIC bias on top of the real signal path is
    /// the test that actually asks "does this margin survive an estimator
    /// that is wrong?" without silently changing which controller is under
    /// test. Both are run and both are reported;
    /// `tests/test_cmd_envelope_reserve.py` has the results.
    ///
    /// Second use, ADR-0011 (f2): a small offset here MOVES THE OPERATING
    /// TRIM while changing nothing else, which is how the peak-demand slope
    /// behind [`CMD_ENVELOPE_RESERVE`] is shown to be trim-derived rather
    /// than geometric.
    pub pitch_bias_deg: f32,

    /// **Verification only.** Runs the loop as fast as the CPU allows,
    /// skipping the [`Pacer`] entirely.
    ///
    /// The plant is a fixed-timestep RK4 integrator with no stochastic terms
    /// and the scripted schedule is indexed on simulated time, so a scripted
    /// free run produces bit-identically the same trajectory as a paced one
    /// -- it just produces it in a fraction of a second instead of in real
    /// time, which is what makes an acceptance SWEEP (stick fraction x
    /// scenario x pitch source) a thing that fits in a test rather than in an
    /// afternoon. Missed deadlines are not counted in this mode: there are no
    /// deadlines.
    ///
    /// NOT how a deployed host runs. The UDP state-out is still sent every
    /// tick, so a listener would see the whole run arrive at once.
    pub free_run: bool,

    /// **Verification only.** Fraction of full fore/aft stick to deliver,
    /// overriding [`CMD_ENVELOPE_RESERVE`].
    ///
    /// Exists so that constant's own provenance is re-measurable rather than
    /// merely asserted: sweeping this IS the "peak demand is linear at N amps
    /// per unit stick" measurement its doc comment cites. `None` uses the
    /// shipped constant.
    pub cmd_envelope_reserve: Option<f32>,

    /// **Verification only.** Overrides [`CMD_ENVELOPE_RESERVE_BRAKING`] --
    /// the reserve spent when the stick opposes the motion. Sweeping this is
    /// how that constant's cost against ADR-0011's worst matrix point is
    /// measured rather than assumed.
    ///
    /// `None` falls back to [`HostConfig::cmd_envelope_reserve`] if THAT is
    /// set, and only then to the shipped constant. So `--cmd-reserve X` on
    /// its own still scales the whole fore/aft command, which is what every
    /// sweep written before braking had its own reserve assumes -- including
    /// `--cmd-reserve 0` meaning "no stick at all".
    pub cmd_envelope_reserve_braking: Option<f32>,

    /// **Verification only.** A scheduled external disturbance -- see
    /// [`Disturbance`]. `None` applies none, which is the deployed
    /// behaviour.
    pub disturbance: Option<Disturbance>,

    /// **Verification only.** Ground incline, DEGREES, positive uphill in the
    /// board's forward direction. Zero is flat and leaves the plant's gravity
    /// untouched.
    ///
    /// ADR-0011's second ratification makes the authored world one of the
    /// three conditions the launch hold exits on: *the authored world is
    /// constrained to what the controller survives, and the constraint is
    /// encoded as a checkable asset rule.* An asset rule needs a number, the
    /// number is an incline, and this is how it gets measured
    /// (`tests/test_incline_tolerance.py`). It is deliberately a MEASUREMENT
    /// knob and not a scenario parameter: no shipped configuration sets it.
    ///
    /// Applied by [`sim_backend::SimBackend::set_incline_deg`], which rotates
    /// `mjModel::opt.gravity` rather than the ground geom -- see there for why
    /// that is the same problem and not an approximation of it.
    pub incline_deg: f64,

    /// **Verification only.** Multiplier on the `wheel_hinge` joint's
    /// declared `damping="0.08"`. `None` (the default) leaves the model's
    /// declared damping untouched; no shipped configuration sets this.
    ///
    /// ADR-0011 criterion (g): "every pass must hold across a damping sweep
    /// of 0.5x-2x" on that constant, which carries no provenance comment and
    /// sets the whole speed-dependence of the full-stick flip (issue #229).
    /// Applied by [`sim_backend::SimBackend::set_damping_scale`], the same
    /// "runtime knob, not a `sim/models/` edit" pattern `incline_deg` above
    /// established.
    pub damping_scale: Option<f64>,

    /// **Verification only.** Writes one CSV row per control cycle to this
    /// path when the run ends. Buffered in memory and written once, never
    /// during the loop: a 500 Hz control thread does not do file I/O per
    /// tick, and the trace exists to measure the loop, not to perturb it.
    pub trace_path: Option<PathBuf>,

    /// ADR-0012. `Some` splices a kerb into the model and arms the physics-
    /// authority handoff; `None` (the default) opens the shared model
    /// unchanged and never sets the handoff bit. See [`KerbSpec`].
    pub kerb: Option<KerbSpec>,

    /// ADR-0012. Path to a MuJoCo hfield binary produced by
    /// `overboard-game`'s `tools/terrain_probe/` pipeline. `Some` replaces the
    /// flat ground plane with the real City Park surface; `None` (the default)
    /// leaves the shared model untouched. See [`TerrainSpec`].
    pub terrain: Option<PathBuf>,

    /// Lean-to-steer (see [`crate::lean_steer`]). `true` swaps the flat
    /// cylinder tire for a rounded-crown one, gives the rider a +-0.25 m
    /// lateral reach, and replaces the commanded-yaw law with camber steer:
    /// the board turns because it rolls. `steer` becomes the rider's
    /// curvature intent, and a rider model balances the roll.
    pub lean_steer: bool,

    /// Spawn point along MuJoCo X, metres, on the `--terrain` heightmap. Lets a
    /// run start on flatter road uphill of the origin without moving the frame.
    pub spawn_x_m: f64,

    /// Add the learned grade current (`--grade-ff`) to the regulator output.
    /// Needs `EstimatorAiding::GradeAware`, which learns the grade.
    pub grade_feedforward: bool,

    /// Motor current limit for sizing studies (`--max-current A`). Sets the
    /// safety envelope, the utilisation reference and the model's motor
    /// torque limit together. `None` is the stock 40 A.
    pub max_current_a: Option<f32>,

    /// Outer speed loop target, m/s (`--speed-hold`). `None`: no outer loop.
    pub speed_hold_m_s: Option<f32>,

    /// Monte Carlo study inputs. Each one changes the PLANT only; the
    /// controller keeps its nominal 70 kg and Kt = 0.7 N.m/A design.
    /// Rider (ballast) mass, kg (`--rider-mass`). `None`: the model's 70 kg.
    pub rider_mass_kg: Option<f64>,
    /// Board mass without rider, kg (`--board-mass`). `None`: the model's 13 kg.
    pub board_mass_kg: Option<f64>,
    /// `--plant SPEC` (see [`parse_plant_spec`]); its values apply first, then
    /// `--board-mass`, `--kt-scale` and `--rider-mass` override them.
    pub plant: Option<PlantVariation>,
    /// True motor Kt as a fraction of the nominal 0.7 N.m/A (`--kt-scale`).
    pub kt_scale: Option<f64>,
    /// Forward speed set at t = 1 s, m/s (`--start-speed`). `None`: from rest.
    pub start_speed_m_s: Option<f64>,
    /// `--hfield-wheel-contact`: let the tire touch the `--terrain`
    /// heightfield directly, as before the smooth contact (see
    /// [`crate::ground`]). Only for kerb studies, where the tire must hit
    /// the kerb face; the heightfield contact chatters.
    pub hfield_wheel_contact: bool,
    /// `--pad-solref S`: contact time constant of the nose and tail pads on
    /// the smooth plate, s (default 0.05: a plastic or urethane pad).
    pub pad_solref_s: f64,
    /// `--tail-brake`: a tail-pad strike does not end the run. Leaning back
    /// onto the tail is a deliberate, safe way to brake; only a nose strike
    /// (the rider goes over the front) or a large tilt is a terminating event.
    pub tail_brake: bool,
    /// `--tail-friction MU`: the tail pad's own friction sets the pad-road
    /// contact (MuJoCo geom priority). Without it the larger of the pad (0.6)
    /// and the road (0.8 heightfield, 1.0 plane) is used.
    pub tail_friction: Option<f64>,
    /// `--authority-margin warn|limit`: the rider warning (D1) and, with
    /// `limit`, the forward-acceleration limit (D2). See
    /// `control_core::AuthorityMargin`.
    pub authority_margin: MarginMode,
    /// `--balance-comp`: the deployed balance law gets grade compensation
    /// (control_core::GradeCompensator: learned-load feedforward and a slow
    /// pitch integral). Ignored with `--speed-hold`, whose law has its own.
    pub balance_comp: bool,
    /// `--hold-until-arm`: hold the board still at its spawn pose until the
    /// first input arm bit, as a rider's foot holds it on a hill. For live
    /// play: without it the board rolls away on a slope before the player
    /// moves. Off by default (scripted and Monte Carlo runs have no arm bit).
    pub hold_until_arm: bool,
    /// `--hud-out-addr ADDR`: also send a HUD packet (crate::hud::HudOut,
    /// 56 B) at 50 Hz: speed, torque and its limit, battery, rider warning.
    pub hud_out_addr: Option<SocketAddr>,
    /// `--batt-soc0`: the pack's starting charge for the HUD, 0..1 (0.9).
    pub batt_soc0: f64,
    /// `--rider-speed V`: a rider model (test harness, not firmware) that sets
    /// the fore/aft lean to ride at V m/s, while the DEPLOYED balance law (the
    /// pitch regulator) balances: the path a person takes in the game. The
    /// law is sim/carve/rider.py's lean mode (leaky PI on speed, acceleration
    /// damping, a grade lean the rider sees). Without `--speed-hold`.
    pub rider_speed_m_s: Option<f32>,
    /// `--rider-target-change T,V`: the `--rider-speed` rider changes its
    /// target to V m/s at sim time T s, at once, without the ease-in, and may
    /// then use its full reach (a hard brake after a fast run).
    pub rider_target_change: Option<(f64, f32)>,
    /// `--rider-reach M`: see `PlantVariation::rider_reach_m`.
    pub rider_reach_m: Option<f64>,
    /// `--tumble`: at a fall (the ADR-0012 handoff) the rider comes off as a
    /// free two-part body (torso, legs on a ball hip) and MuJoCo goes on
    /// computing the board and the rider sliding on the road; the motor is
    /// cut, as the firmware does at a fall. At a `--rider-reacts` dismount the
    /// board only loses the rider's mass. The wire still freezes at the
    /// handoff (Unreal's ragdoll does not change). Scripted runs only: a
    /// reset does not put the rider's mass back.
    pub tumble: bool,
    /// `--pose-out PATH`: the full MuJoCo pose at 50 Hz for the whole run,
    /// with the tumble rider, for a MuJoCo render (sim/carve/render_pose.py).
    /// The model is kept beside it as `sim/models/<stem>.generated.xml`.
    pub pose_out: Option<PathBuf>,
    /// `--motor-limits`: the motor and the pack limit the current, as a real
    /// controller does. Driving: the back-EMF leaves (0.95 V_pack - Ke w) /
    /// R_eff, so the torque falls to zero near the top speed (the classic
    /// nosedive). Braking: the pack takes at most 45 A of charge, so the
    /// braking current falls as 1/w at speed. V_pack is the HUD pack model
    /// (it sags under load). Off by default: the Monte Carlo results do not
    /// include it.
    pub motor_limits: bool,
    /// `--sensors stage0`: see the backend construction in `run`.
    pub sensors_stage0: bool,
    /// `--noise-scale S`: scales the stage0 noise and bias (1 by default).
    pub noise_scale: f64,
    /// `--extra-delay-ms D`: added to the stage0 actuation delay.
    pub extra_delay_ms: f64,
    /// `--kp-scale S`, `--kd-scale S`: study scales on the deployed balance
    /// gains (Kp 420 N*m/rad, Kd 36.3 N*m*s/rad). 1 by default.
    pub kp_scale: f32,
    pub kd_scale: f32,
    /// `--ankle-hinge`: the rider stands on a pitch hinge at deck level (an
    /// ankle; fable-oracle, 2026-10-04). Its torque is a spring and damper on
    /// the rider's lean against GRAVITY (not against the deck), K = 1.3 m g h,
    /// zeta 0.7, limited to m g * 0.12 m (the feet's centre-of-pressure
    /// reach). The limit lets the body stay near vertical when the deck sits
    /// on a pad. The slide still carries the stick's lean intent.
    pub ankle_hinge: bool,
    /// `--obstacles CSV`: fixed obstacles on the road, from the game's course
    /// elements (sim/carve/obstacles.py): cones and debris, placed on the
    /// terrain surface. They are hard posts (they do not move); the game draws
    /// them at the same place.
    pub obstacles: Option<PathBuf>,
    /// `--rider-reach-back M`: the backward stick range, m (default: the
    /// reach). Needs `--rider-reach` (the slide allows 0.35 m). The rider
    /// model in the Monte Carlo does not use it.
    pub rider_reach_back_m: Option<f64>,
    /// `--foot-torque` (with `--ankle-hinge`): fore/aft stick also presses the
    /// heels or toes, a torque on the deck up to 80 % of the feet's limit.
    /// A real rider tilts a stiff board mostly this way; it is what drags the
    /// tail under a hard lean back.
    pub foot_torque: bool,
    /// `--ankle-rigid`: the hinge with a 1e5 N*m/rad spring on the hinge angle
    /// (against the deck) and no limit: it must reproduce the rigid rider
    /// (the bracket test).
    pub ankle_rigid: bool,
    /// `--rider-ankle`: the rider stays upright as the deck pitches (the
    /// slide moves by 0.75 m * sin(pitch)); otherwise the rider is rigid
    /// with the deck. Needs `--rider-reach` (it widens the slide to 0.35 m).
    /// DO NOT USE with the deployed law: 145 of 200 Monte Carlo runs fell
    /// (its gains assume the rigid rider). Pad mode uses the same correction
    /// on its own.
    pub rider_ankle: bool,
    /// `--stop-after-handoff S`: end the run S seconds after a fall or a
    /// dismount (to record the tumble).
    pub stop_after_handoff_s: Option<f64>,
    /// `--rider-reacts` (speed-hold harness only): a rider model that answers
    /// the warning. Pulsed for 0.5 s: the rider eases off (target 0). Solid
    /// for 0.5 s while driving: the rider steps off and the run ends (DISMOUNT).
    /// Eased and stopped (below 0.3 m/s) on a climb with the warning on: the
    /// rider steps off (DISMOUNT). Solid while braking: the rider stays on and
    /// leans onto the tail.
    pub rider_reacts: bool,
    /// A grade profile on the flat plane, by gravity (`--grade-course`).
    /// Replaces a `--terrain` heightfield for grade studies: MuJoCo's
    /// sphere-on-heightfield contact chatters at every grid edge (measured:
    /// specific force sd 3.2 m/s^2 at 4 m/s, with lift-off), and the IMU
    /// gate turns that chatter into a pitch bias (2.6 deg flat, ~7 deg on
    /// a 10 % climb). A plane has one clean contact.
    pub grade_course: Option<GradeCourse>,
}

/// Flat run-in, then a vertical curve, then a constant grade -- the same
/// profile `sim/carve/course.py` builds for its `steady_*` presets. The board
/// starts at x = 0 and travels along -X, so s = -x.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GradeCourse {
    pub run_in_m: f64,
    /// Percent, positive uphill.
    pub grade_pct: f64,
    /// Vertical-curve length scale, m: the grade changes over
    /// `radius * |grade|`, centred on the end of the run-in, by a cosine
    /// blend (the mean radius is `radius`; the tightest is `2 radius / pi`).
    pub radius_m: f64,
}

impl GradeCourse {
    /// Grade at distance `s` along the course, degrees, positive uphill.
    pub fn grade_deg(&self, s_m: f64) -> f64 {
        let g = self.grade_pct / 100.0;
        let half = 0.5 * self.radius_m * g.abs();
        let frac = if half <= 0.0 {
            if s_m >= self.run_in_m { 1.0 } else { 0.0 }
        } else {
            // Cosine blend, not linear: the grade RATE must start and end at
            // zero. The plane model turns gravity, so a board held vertical
            // rotates against the plane at the grade rate, and a rate step
            // leaves its angular momentum behind (no Euler torque in the
            // model). Measured with the linear blend: a nose-down drift at
            // every curve end that saturated the motor (103 kg, 14.8 %).
            let u = ((s_m - (self.run_in_m - half)) / (2.0 * half)).clamp(0.0, 1.0);
            0.5 - 0.5 * (std::f64::consts::PI * u).cos()
        };
        (g * frac).atan().to_degrees()
    }
}

/// `--rider-speed`: a rider who rides at a target speed by leaning, the way
/// sim/carve/rider.py's lean mode does (its gains, at 500 Hz instead of
/// 50 Hz). Output is the fore/aft stick, -1..1 (full stick = 5 cm of lean).
#[derive(Debug, Clone, Copy)]
pub struct RiderSpeedModel {
    /// Lean bound, in units of the model's 5 cm stick (`LEAN_BOUND` = 3 cm).
    pub bound: f32,
    v_ref: Option<f32>,
    integral: f32,
    acc_f: f32,
    v_prev: Option<f32>,
}

impl RiderSpeedModel {
    /// The rider eases the target in at this rate, m/s^2.
    pub const TARGET_RATE_M_S2: f32 = 0.5;
    /// Default lean bound, in 5 cm stick units: 0.6 = 3 cm.
    pub const LEAN_BOUND: f32 = 0.6;
    /// Grade lean per unit sin(downhill grade), stick units (rider.py).
    pub const GRADE_LEAN: f32 = 3.08;

    /// One cycle. `grade_down_rad` is the slope the rider sees, downhill
    /// positive (0 when the course is unknown).
    pub fn update(&mut self, v: f32, target: f32, grade_down_rad: f32, dt: f32) -> f32 {
        let r = self.v_ref.get_or_insert(v);
        let step = Self::TARGET_RATE_M_S2 * dt;
        *r += (target - *r).clamp(-step, step);
        let err = *r - v;
        // Leaky (1/s): the grade lean carries the slope; the integral trims.
        self.integral = (self.integral * (1.0 - dt) + err * dt).clamp(-3.0, 3.0);
        let acc = self.v_prev.map_or(0.0, |p| (v - p) / dt);
        self.v_prev = Some(v);
        // rider.py filters acceleration by 0.1 per 20 ms: tau about 0.19 s.
        self.acc_f += dt / (0.19 + dt) * (acc - self.acc_f);
        let raw = -Self::GRADE_LEAN * grade_down_rad.sin() + 0.30 * err + 0.10 * self.integral
            - 0.45 * self.acc_f;
        let fa = raw.clamp(-self.bound, self.bound);
        if fa != raw {
            self.integral -= err * dt; // do not integrate into a bound
        }
        fa
    }

    pub fn reset(&mut self) {
        *self = Self { bound: self.bound, ..Self::default() };
    }

    /// A sudden change of mind: the target jumps to `v` now, without the
    /// 0.5 m/s^2 ease-in (`--rider-target-change`: a hard brake).
    pub fn set_target_now(&mut self, v: f32) {
        self.v_ref = Some(v);
    }
}

impl Default for RiderSpeedModel {
    fn default() -> Self {
        Self { bound: Self::LEAN_BOUND, v_ref: None, integral: 0.0, acc_f: 0.0, v_prev: None }
    }
}

/// `--authority-margin`: off, warn only (D1), or warn and limit (D2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MarginMode {
    Off,
    Warn,
    Limit,
}

/// Pack voltage for the duty estimate, V: 20S at 3.6 V nominal. The sim has
/// no pack in the loop, so sag is not in the estimate.
const MARGIN_PACK_V: f32 = 72.0;
/// Motor back-EMF constant and phase resistance for the duty estimate
/// (`sim/carve/battery.py` defaults).
const MARGIN_KE_V_S: f32 = 0.7;
const MARGIN_R_PHASE_OHM: f32 = 0.12;
/// `--rider-reacts`: the rider's reaction time to a warning, s.
const RIDER_REACTION_S: f64 = 0.5;

/// The plant-only changes a Monte Carlo run splices into the model.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PlantVariation {
    /// Whole board without rider, kg (`--board-mass`). The model's board is
    /// 13 kg (frame 8, wheel 4.5, carrier 0.5); the extra goes into the frame
    /// at its own centre of mass (battery and controller in the deck), and
    /// the frame inertia scales with it.
    pub board_mass_kg: Option<f64>,
    pub rider_mass_kg: Option<f64>,
    pub kt_scale: Option<f64>,
    pub tail_friction: Option<f64>,
    /// `--plant`: the rotating part of the wheel (rotor can and tyre), kg.
    pub wheel_rot_kg: Option<f64>,
    /// `--plant`: wheel spin inertia about the axle, kg m^2.
    pub wheel_spin_kgm2: Option<f64>,
    /// `--plant`: frame centre of mass from the axle, m: x positive BEHIND
    /// (board forward is -X), z positive up.
    pub frame_com_m: Option<(f64, f64)>,
    /// `--plant`: frame inertia, as a scale on the model's
    /// diag(0.040, 0.400, 0.420) kg m^2. `None` scales with the frame mass.
    pub frame_inertia_scale: Option<f64>,
    /// `--plant`: tyre rolling radius, m (model 0.1454). The controller keeps
    /// its 0.1454 m belief, so a different value is a plant error.
    pub wheel_radius_m: Option<f64>,
    /// `--plant x7`: the build's look (proxy meshes) and its nose and tail
    /// collision pads (a box and a bumper at each end) in place of the
    /// Onewheel-style meshes.
    pub x7_geometry: bool,
    /// `--plant`: raise (+) or lower (-) the nose and tail pads, m. 1 cm is
    /// about 1.6 deg of strike angle. The proxy pads give 20.4 deg.
    pub pad_z_m: Option<f64>,
    /// `--rider-reach M` (or plant key `reach`): the rider's fore/aft reach,
    /// m (model 0.05). A real rider moves the body over the feet with the
    /// ankles and knees, well past 5 cm; the stick maps onto +-M.
    pub rider_reach_m: Option<f64>,
}

impl PlantVariation {
    /// True when the frame inertial must be rewritten.
    fn touches_frame(&self) -> bool {
        self.board_mass_kg.is_some()
            || self.wheel_rot_kg.is_some()
            || self.frame_com_m.is_some()
            || self.frame_inertia_scale.is_some()
    }
}

/// The Fungineers X7 / Superflux HT / Thor301 build (hardware track,
/// 2026-10-04; V = vendor, M = measured by others, I = inferred). The board
/// is 18.4 kg (I: 17.9 kg vendor kit + 0.5 kg of our parts). Of the 7.5 kg
/// wheel assembly (I), the rotor can and tyre (4.5 kg, I) turn; the stator
/// stays with the frame. Frame CoM 0.03 m behind the axle (heavier rear
/// pack) and 0.15 m above the ground (I); with the stator at the axle that
/// is (0.0244, 0.003) m from the axle for the 13.4 kg frame body. Frame
/// pitch inertia 0.47 kg m^2 (I). Tyre radius 0.146 m (V). Kt 0.658 N.m/A
/// (M, motor wizard), i.e. 0.94 of the controller's 0.7 belief.
pub fn plant_x7() -> PlantVariation {
    PlantVariation {
        board_mass_kg: Some(18.4),
        rider_mass_kg: None,
        kt_scale: Some(0.658 / 0.7),
        tail_friction: None,
        wheel_rot_kg: Some(4.5),
        wheel_spin_kgm2: Some(0.045),
        frame_com_m: Some((0.0244, 0.003)),
        frame_inertia_scale: Some(0.47 / 0.400),
        wheel_radius_m: Some(0.146),
        x7_geometry: true,
        pad_z_m: None,
        rider_reach_m: None,
    }
}

/// Parses `--plant SPEC`: `x7`, optionally followed by `,key=value`
/// overrides (board_kg, wheel_kg, wheel_spin, com_x, com_z, frame_i, radius,
/// kt). A spec without `x7` overrides the shared model's values.
pub fn parse_plant_spec(spec: &str) -> Result<PlantVariation, String> {
    let mut v = PlantVariation::default();
    for (i, part) in spec.split(',').map(str::trim).filter(|p| !p.is_empty()).enumerate() {
        if part == "x7" {
            if i != 0 {
                return Err("--plant: 'x7' must come first".into());
            }
            v = plant_x7();
            continue;
        }
        let (k, val) = part.split_once('=').ok_or(format!("--plant: '{part}' is not key=value"))?;
        let x: f64 = val.parse().map_err(|_| format!("--plant: '{val}' is not a number"))?;
        match k {
            "board_kg" => v.board_mass_kg = Some(x),
            "wheel_kg" => v.wheel_rot_kg = Some(x),
            "wheel_spin" => v.wheel_spin_kgm2 = Some(x),
            "com_x" => v.frame_com_m = Some((x, v.frame_com_m.map_or(-0.03, |c| c.1))),
            "com_z" => v.frame_com_m = Some((v.frame_com_m.map_or(0.0, |c| c.0), x)),
            "frame_i" => v.frame_inertia_scale = Some(x),
            "radius" => v.wheel_radius_m = Some(x),
            "kt" => v.kt_scale = Some(x),
            "pad_z" => v.pad_z_m = Some(x),
            "reach" => v.rider_reach_m = Some(x),
            _ => return Err(format!("--plant: unknown key '{k}'")),
        }
    }
    Ok(v)
}

/// Where the regulator's attitude comes from -- ADR-0011 exit criterion (f).
///
/// The default is the only one a deployed host may use. `PlantTruth` exists
/// because the ADR's FIRST ratification required every acceptance pass to
/// hold **with the estimator bias removed**, and on this plant that is the
/// measured-WORSE case, not the better one: feeding the controller MuJoCo
/// truth makes the board flip EARLIER.
///
/// **The reason is not an accidental bias, and the ADR's second ratification
/// corrects that reading.** The `est - truth` residual is the apparent
/// vertical, `atan(a/g)` -- 1 radian per g, which is the same 5.84 deg per
/// m/s^2 the ADR derives geometrically as the lean this board must hold to
/// sustain acceleration `a`, because it is the same physics. The estimator is
/// supplying the textbook balance-vehicle lean. Substituting truth therefore
/// does not de-bias the loop, it deletes the only mechanism generating that
/// lean and leaves a pitch-only regulator -- a different controller, not a
/// cleaner one. What criterion (f) was right about is that nothing designed
/// this and nothing held it in place; (f1)/(f2) fix that by pinning it
/// (`tests/test_cmd_envelope_reserve.py`), and (f3) makes it explicit as a
/// `theta_ref = atan(a_des / g)` feedforward, at which point this instrument
/// becomes meaningful again.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PitchSource {
    /// The `ComplementaryFilter`, fed raw IMU through `hal` -- the real
    /// signal path, and the only one a real board has.
    #[default]
    Estimator,
    /// MuJoCo's own body-frame pitch and pitch rate, straight from the
    /// plant. Physically unavailable on hardware; an acceptance-run
    /// instrument only.
    PlantTruth,
}

/// Which signal aids the complementary filter's accelerometer branch --
/// issue #227.
///
/// ADR-0011's (f1)/(f2) freeze-and-pin regression-tests only the
/// [`CommandFeedforward`] path, because until now this host offered no
/// other one to test: `hill.py`/`terrain.py` (and therefore
/// `tests/test_terrain.py`, which found the anomaly (f1)/(f2) are cited
/// against) default to `control-ffi`'s wheel-odometry aiding instead, via
/// [`WheelAccelEstimator`]. This enum makes that second path selectable
/// here too, so it can be measured rather than assumed to behave the same.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum EstimatorAiding {
    /// Predicts the acceleration from the current just commanded --
    /// `RustController()`'s mode 2. The deployed default: matches
    /// `shuttle_run.py`'s "recommended configuration" and every ADR-0011
    /// acceptance number measured so far.
    #[default]
    CommandFeedforward,
    /// Measures the acceleration by differentiating wheel speed --
    /// `RustController()`'s mode 1, and `hill.py`/`terrain.py`'s default.
    /// Not pinned before issue #227; see
    /// `tests/test_cmd_envelope_reserve.py` for the coverage this adds.
    WheelOdometry,
    /// [`control_core::GradeAwareAiding`]: the command prediction, corrected
    /// by a slow grade load learned from wheel odometry. Unbiased on grades,
    /// and robust to wheel spikes on rough ground.
    GradeAware,
}

/// A scheduled external force/torque disturbance, world frame, applied to the
/// `frame` body over a fixed window of SIMULATED time.
///
/// Generic on purpose. The disturbance ADR-0011's test matrix needs is a kerb
/// strike, and the kerb derivation this repo trusts lives in ONE place --
/// `sim/scenarios/plant.py::kerb_strike_impulse` (issue #142/#147, Sr.
/// Mechanical & Systems' call, not Controls'). Re-implementing that geometry
/// in Rust would give this repo two kerb models that could disagree, so this
/// host takes the derived impulse as a parameter and stays dumb about where
/// it came from; `tests/test_cmd_envelope_reserve.py` calls the Python
/// derivation and passes the result in.
#[derive(Debug, Clone, Copy)]
pub struct Disturbance {
    /// Simulated time the force window opens, seconds.
    pub t0_s: f64,
    /// How long it stays open, seconds. Impulse delivered is `force * this`.
    pub duration_s: f64,
    /// World-frame force, newtons.
    pub force_n: [f64; 3],
    /// World-frame torque about the frame body's own origin, N*m.
    pub torque_nm: [f64; 3],
}

/// Picks which disturbance (if any) is live this tick, and returns its
/// force/torque verbatim -- world frame, per [`apply_external_force`]'s own
/// contract and every one of these sources' doc comments
/// ([`Disturbance`], [`STARTUP_KICK_FORCE_N`], [`FALL_KICK_FORCE_N`]).
///
/// # Why this is its own function (issue #194)
///
/// Every disturbance this repo fires today -- the startup/fall kicks and the
/// ADR-0011 kerb impulse -- is a physically world-anchored event: a kerb does
/// not rotate with the board, and neither does an exogenous shove. Deliberately
/// NOT taking a heading/yaw parameter is how that stays true rather than just
/// documented: there is nothing here for a future edit to rotate by mistake,
/// and the compiler enforces it, not a comment. A scenario that genuinely
/// needs a body-frame push (e.g. a "nose shove" that must track the board
/// mid-carve) does not exist today; when one does, it should rotate its own
/// vector by the current heading before it reaches this function, not grow a
/// silent frame flag here.
///
/// Precedence when windows overlap -- unchanged from the inline form this
/// replaced: scheduled disturbance, then the startup kick, then the on-demand
/// fall kick, then nothing. Windows are not expected to overlap in practice
/// (the fall kick is operator-triggered and the other two are off by default
/// or fixed to `t=1.0s`), so precedence over correctly rejecting an overlap is
/// the acceptable simplification.
fn select_disturbance_force_torque(
    in_disturbance_window: bool,
    disturbance: Option<Disturbance>,
    in_kick_window: bool,
    in_fall_kick_window: bool,
) -> ([f64; 3], [f64; 3]) {
    if in_disturbance_window {
        let d =
            disturbance.expect("in_disturbance_window is only set when cfg.disturbance is Some");
        (d.force_n, d.torque_nm)
    } else if in_kick_window {
        (STARTUP_KICK_FORCE_N, [0.0; 3])
    } else if in_fall_kick_window {
        (FALL_KICK_FORCE_N, [0.0; 3])
    } else {
        ([0.0; 3], [0.0; 3])
    }
}

impl Default for HostConfig {
    fn default() -> Self {
        HostConfig {
            state_out_addr: wire::state_out_addr(),
            input_in_addr: wire::input_in_addr(),
            duration: None,
            stats_path: Some(PathBuf::from(DEFAULT_STATS_PATH)),
            startup_kick: false,
            scripted_scenario: None,
            max_sim_time_s: None,
            pitch_source: PitchSource::Estimator,
            estimator_aiding: EstimatorAiding::CommandFeedforward,
            pitch_bias_deg: 0.0,
            free_run: false,
            cmd_envelope_reserve: None,
            cmd_envelope_reserve_braking: None,
            disturbance: None,
            incline_deg: 0.0,
            damping_scale: None,
            trace_path: None,
            kerb: None,
            terrain: None,
            lean_steer: false,
            spawn_x_m: 0.0,
            grade_feedforward: false,
            max_current_a: None,
            speed_hold_m_s: None,
            rider_mass_kg: None,
            board_mass_kg: None,
            plant: None,
            kt_scale: None,
            start_speed_m_s: None,
            grade_course: None,
            hfield_wheel_contact: false,
            tail_brake: false,
            pad_solref_s: 0.05,
            tail_friction: None,
            authority_margin: MarginMode::Off,
            rider_reacts: false,
            rider_speed_m_s: None,
            rider_reach_m: None,
            rider_target_change: None,
            tumble: false,
            motor_limits: false,
            rider_ankle: false,
            ankle_hinge: false,
            kp_scale: 1.0,
            foot_torque: false,
            rider_reach_back_m: None,
            obstacles: None,
            sensors_stage0: false,
            noise_scale: 1.0,
            extra_delay_ms: 0.0,
            kd_scale: 1.0,
            ankle_rigid: false,
            pose_out: None,
            stop_after_handoff_s: None,
            hud_out_addr: None,
            batt_soc0: 0.9,
            hold_until_arm: false,
            balance_comp: false,
        }
    }
}

/// One control cycle's worth of acceptance-run instrumentation. See
/// [`HostConfig::trace_path`]; the column list and its order are the CSV
/// header written by [`write_trace`].
#[derive(Debug, Clone, Copy)]
struct TraceRow {
    seq: u64,
    sim_time_s: f64,
    /// Fore/aft stick as the schedule (or the socket) supplied it, before
    /// [`CMD_ENVELOPE_RESERVE`].
    stick_fore_aft: f32,
    /// After the reserve, before the speed cap and the corridor brake.
    shaped_fore_aft: f32,
    /// What actually reached `set_ballast_targets`, after everything.
    applied_fore_aft: f32,
    /// MuJoCo truth, de-yawed, degrees, nose-up positive.
    truth_pitch_deg: f32,
    /// The complementary filter's belief, degrees -- recorded even on a
    /// `PitchSource::PlantTruth` run (the estimator still runs; it is simply
    /// not the regulator's input) so the bias itself stays visible.
    est_pitch_deg: f32,
    /// ... and the rate it believes, which is the regulator's D-term input on
    /// a deployed run. Recorded alongside the truth rate because the two are
    /// the pair criterion (f) swaps, and a swap nobody can see the size of is
    /// not a measurement.
    est_pitch_rate_deg_s: f32,
    truth_pitch_rate_deg_s: f32,
    forward_speed_m_s: f32,
    /// PRE-envelope demand, amps. The number the reserve is derived from.
    proposed_amps: f32,
    /// POST-envelope command, amps.
    applied_amps: f32,
    /// `safety::Envelope` reported the clamp bound this cycle.
    saturated: bool,
    /// MuJoCo's true world-frame ground position (issue #201: the turn
    /// radius has to be fitted off an actual track, not quoted from
    /// `1 / (steer * k)`). Same values `truth_pos_x_m`/`truth_pos_y_m`
    /// carry elsewhere in this file -- see [`write_stats`].
    pos_x_m: f32,
    pos_y_m: f32,
    /// Bumper contact forces, N (ADR-0012 touch sensors).
    nose_strike_n: f32,
    tail_strike_n: f32,
    /// `--authority-margin`: margin and level (0 none, 1 pulse, 2 solid).
    margin: f32,
    margin_level: u8,
    /// Truth roll, deg (body roll; positive = deck leans right).
    truth_roll_deg: f32,
    /// `|proposed_amps| / MAX_CURRENT_A`, unfiltered.
    utilisation: f32,
    /// ... and low-passed at [`AUTHORITY_UTILISATION_TAU_S`].
    utilisation_filtered: f32,
    /// The loss-of-authority warning is asserted this cycle.
    authority_warning: bool,
    /// The wire's `FALLEN` bit, for the lead-time comparison.
    fallen: bool,
}

/// What a finished (or interrupted-by-error) run produced.
#[derive(Debug, Clone, Copy, Default)]
pub struct RunSummary {
    pub ticks: u64,
    pub missed_deadlines: u64,
}

/// Everything that can stop [`run`] early.
#[derive(Debug)]
pub enum HostError {
    Backend(board_types::IoError),
    Io(std::io::Error),
}

impl std::fmt::Display for HostError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            HostError::Backend(e) => write!(f, "sim backend error: {e:?}"),
            HostError::Io(e) => write!(f, "I/O error: {e}"),
        }
    }
}

impl std::error::Error for HostError {}

impl From<std::io::Error> for HostError {
    fn from(e: std::io::Error) -> Self {
        HostError::Io(e)
    }
}

/// The most recent input the host has heard from Unreal, plus the bits this
/// host does not yet act on (accepted, clamped, and logged on receipt --
/// issue #161 -- so a later pass only has to connect them to something).
#[derive(Debug, Clone, Copy, Default)]
struct LatestInput {
    weight_shift_fore_aft: f32,
    weight_shift_lateral: f32,
    steer: f32,
    armed_bit: bool,
    reset_bit: bool,
    kick_bit: bool,
}

impl From<InputIn> for LatestInput {
    fn from(p: InputIn) -> Self {
        let flags = p.flags;
        LatestInput {
            weight_shift_fore_aft: p.weight_shift_fore_aft,
            weight_shift_lateral: p.weight_shift_lateral,
            steer: p.steer,
            armed_bit: flags & wire::INPUT_FLAG_ARM != 0,
            reset_bit: flags & wire::INPUT_FLAG_RESET != 0,
            kick_bit: flags & wire::INPUT_FLAG_KICK != 0,
        }
    }
}

/// Spawns the control loop on its own dedicated thread (issue #161: "not on
/// the main thread") and returns the join handle. The caller decides what to
/// do with the calling thread -- `src/bin/sim-host.rs` just joins it.
/// Body-frame pitch and roll, radians, from the `frame` body's world rotation
/// matrix (row-major `xmat`) and the world-frame heading currently baked into
/// the plant.
///
/// Pitch is `atan2(R[0][2], R[2][2])` -- exactly
/// `sim/scenarios/impulse_response.py::frame_pitch_rad`'s formula against the
/// identical array, nose-up positive per ICD 10.1 -- and roll is the same
/// atan2-of-a-tilted-axis derivation applied to the Y-Z plane (about local X)
/// instead of the X-Z plane (about local Y). Both are exact only when the
/// OTHER angle is near zero: 3D rotations do not commute and this is not a
/// true Euler decomposition. Acceptable for the roll-shaped yaw limiter's
/// "cheap stopgap" status (issue #161 W2 item 4); the real lean-steer
/// controller needs a better one.
///
/// **`yaw_rad` must be removed FIRST, and that is why this function exists**
/// (issue #163). Those formulas assume the world x/y axes still line up with
/// the body's, which was true only while the board had no yaw freedom at all
/// -- precisely what the kinematic in-plant yaw injection changed. `xmat` is
/// now `Rz(yaw) * R_body`, whose third column mixes body pitch and body roll
/// by the heading angle, so after a 90 deg turn the raw formulas would report
/// the board's ROLL as its PITCH and `FALLEN` would fire on an upright board.
/// Left-multiplying by `Rz(-yaw)` -- which is all the two `deyawed_*` lines
/// are -- recovers the body-frame values the ICD and the roll gate both mean.
///
/// At `yaw_rad == 0` this reduces to `1.0 * xmat[k] + 0.0 * xmat[j]`, i.e.
/// bit-identically the pre-#163 expressions; `deyaw_at_zero_yaw_is_a_no_op`
/// pins that.
fn body_pitch_roll_rad(xmat: &[f64; 9], yaw_rad: f32) -> (f32, f32) {
    let (yaw_s, yaw_c) = yaw_rad.sin_cos();
    let deyawed_02 = yaw_c * xmat[2] as f32 + yaw_s * xmat[5] as f32;
    let deyawed_12 = -yaw_s * xmat[2] as f32 + yaw_c * xmat[5] as f32;
    (
        deyawed_02.atan2(xmat[8] as f32),
        deyawed_12.atan2(xmat[8] as f32),
    )
}

/// The command-envelope reserve, as a function (ADR-0011 criterion (b)).
///
/// One multiply, extracted from the loop body only so the property that
/// matters can be stated as a test rather than as a comment: this is a
/// LINEAR scaling of the stick, symmetric about zero, and it changes nothing
/// else. In particular it is not a clamp -- a clamp would leave full stick
/// untouched right up to a limit and then bite, which is a different feel and
/// a different failure mode.
fn shape_fore_aft_command(stick: f32, reserve: f32) -> f32 {
    stick * reserve
}

/// Like [`shape_fore_aft_command`], but spends the BRAKING reserve when the
/// command opposes the current motion. See [`CMD_ENVELOPE_RESERVE_BRAKING`].
///
/// The opposition test is `stick * speed < 0`, deliberately the same one
/// [`speed_capped_fore_aft`] already uses, so there is one definition of
/// "this command opposes the motion" in this file rather than two that can
/// drift apart.
///
/// **At rest the accelerating reserve applies** (`stick * 0.0 >= 0.0`), which
/// is the case that matters for safety: full aft from a standstill is a
/// backward standing start, not a stop, and it over-commands the envelope
/// exactly as the forward one does. That is the mirror of the defect
/// ADR-0011 was called for, and it keeps the accelerating reserve.
fn shape_fore_aft_command_directional(
    stick: f32,
    reserve: f32,
    braking_reserve: f32,
    last_forward_speed_m_s: f32,
) -> f32 {
    if last_forward_speed_m_s.abs() > BRAKING_RESERVE_MIN_SPEED_M_S
        && stick * last_forward_speed_m_s < 0.0
    {
        shape_fore_aft_command(stick, braking_reserve)
    } else {
        shape_fore_aft_command(stick, reserve)
    }
}

/// The speed cap's authority ramp, as a function -- unchanged in behaviour,
/// extracted so ADR-0011 criterion (a)'s reversal entry can be stated as a
/// unit test as well as a sim run.
///
/// The property worth pinning: a REVERSAL is never attenuated. The cap only
/// withdraws authority when the stick and the current motion share a sign,
/// so a stick slammed from one rail to the other at speed passes through at
/// FULL authority, at any speed, including above the onset where the cap is
/// the only thing unloading the board. That is why the reversal is the
/// matrix's worst case and not merely another entry in it.
fn speed_capped_fore_aft(stick: f32, last_forward_speed_m_s: f32) -> f32 {
    let speed_headroom_m_s = MAX_GROUND_SPEED_M_S - last_forward_speed_m_s.abs();
    let accel_authority = (speed_headroom_m_s / SPEED_CAP_MARGIN_M_S).clamp(0.0, 1.0);
    if stick * last_forward_speed_m_s >= 0.0 {
        stick * accel_authority
    } else {
        stick
    }
}

/// The loss-of-authority warning's trigger (ADR-0011 criterion (c)):
/// filtered authority utilisation over threshold, AND below the speed cap's
/// onset.
///
/// Both halves are load-bearing and the second is the one that is easy to
/// drop. Saturation above [`SPEED_CAP_ONSET_M_S`] is survivable -- the cap is
/// already unloading the board when it happens -- and every run in ADR-0011's
/// data that saturated above the onset survived. A warning without the speed
/// term would fire on all of those, which is a warning nobody reads.
/// Curvature per unit steer at this ground speed, rad/m — widest at
/// [`YAW_TIGHTEN_REF_SPEED_M_S`] and above, tightening toward a standstill by
/// [`YAW_LOW_SPEED_TIGHTEN`]. See that constant for why.
fn yaw_curvature_per_steer(forward_speed_m_s: f32) -> f32 {
    let slowness = 1.0 - (forward_speed_m_s.abs() / YAW_TIGHTEN_REF_SPEED_M_S).clamp(0.0, 1.0);
    YAW_CURVATURE_PER_STEER_RAD_PER_M * (1.0 + (YAW_LOW_SPEED_TIGHTEN - 1.0) * slowness)
}

/// The whole yaw law, in one place.
///
/// Extracted so the unit tests exercise the SHIPPED expression rather than a
/// copy of it — the same reason [`speed_capped_fore_aft`] is a function. The
/// zero-speed property below is the one this crate got wrong once already,
/// and a test asserting it against a re-typed formula would not have caught
/// it.
fn yaw_rate_rad_s(steer: f32, forward_speed_m_s: f32, roll_authority: f32) -> f32 {
    steer * yaw_curvature_per_steer(forward_speed_m_s) * forward_speed_m_s.abs() * roll_authority
}

fn authority_warning_active(utilisation_filtered: f32, forward_speed_m_s: f32) -> bool {
    utilisation_filtered > AUTHORITY_UTILISATION_WARN
        && forward_speed_m_s.abs() < SPEED_CAP_ONSET_M_S
}

pub fn spawn(cfg: HostConfig) -> std::thread::JoinHandle<Result<RunSummary, HostError>> {
    std::thread::Builder::new()
        .name("sim-host-control".into())
        .spawn(move || run(cfg))
        .expect("sim-host: failed to spawn the control thread")
}

/// Runs the 500 Hz closed loop until `cfg.duration` elapses (or forever, if
/// `None`), or until a backend/I-O error stops it. Blocking -- call this
/// from a spawned thread, not the process's main thread, unless the caller
/// has nothing else to do on main either.
/// Asks macOS to schedule the loop thread as real-time (Mach time-constraint
/// policy, plus user-interactive QoS), so the
/// 2 ms cycle and its state packets come out in fewer bursts under load. Live
/// play needs this; a scripted run does not, but it does no harm. It does
/// nothing on other targets (the RT target uses PREEMPT_RT, not this host).
/// The last part of each paced wait that the loop spins instead of sleeping.
const SPIN_WINDOW: Duration = Duration::from_millis(1);

fn request_interactive_scheduling() {
    #[cfg(target_os = "macos")]
    {
        // <mach/thread_policy.h> and <pthread/qos.h>. libSystem is always
        // linked on macOS.
        #[repr(C)]
        struct TimeConstraint {
            period: u32,
            computation: u32,
            constraint: u32,
            preemptible: i32,
        }
        #[repr(C)]
        struct Timebase {
            numer: u32,
            denom: u32,
        }
        extern "C" {
            fn pthread_set_qos_class_self_np(qos_class: u32, relative_priority: i32) -> i32;
            fn mach_thread_self() -> u32;
            fn mach_timebase_info(info: *mut Timebase) -> i32;
            fn thread_policy_set(thread: u32, flavor: u32, policy: *const i32, count: u32) -> i32;
        }
        const QOS_CLASS_USER_INTERACTIVE: u32 = 0x21;
        const THREAD_TIME_CONSTRAINT_POLICY: u32 = 2;
        // SAFETY: plain libSystem calls on the current thread; the pointers
        // are to live, correctly laid-out locals.
        unsafe {
            let _ = pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
            let mut tb = Timebase { numer: 0, denom: 0 };
            if mach_timebase_info(&mut tb) != 0 || tb.numer == 0 {
                eprintln!("sim-host: no Mach timebase; real-time policy not set");
                return;
            }
            let abs = |ns: u64| (ns * tb.denom as u64 / tb.numer as u64) as u32;
            // One 2 ms cycle; a tick computes in about 20 us. Ask for 0.5 ms
            // of CPU, delivered within 1 ms of the period start.
            let policy = TimeConstraint {
                period: abs(2_000_000),
                computation: abs(500_000),
                constraint: abs(1_000_000),
                preemptible: 1,
            };
            let rc = thread_policy_set(
                mach_thread_self(),
                THREAD_TIME_CONSTRAINT_POLICY,
                &policy as *const TimeConstraint as *const i32,
                4,
            );
            if rc != 0 {
                eprintln!("sim-host: real-time thread policy refused (kern {rc}); QoS only");
            }
        }
    }
}

pub fn run(cfg: HostConfig) -> Result<RunSummary, HostError> {
    request_interactive_scheduling();
    let params = Params {
        kp_nm_per_rad: KP_NM_PER_RAD * cfg.kp_scale,
        kd_nm_per_rad_s: KD_NM_PER_RAD_S * cfg.kd_scale,
        kt_nm_per_a: KT_NM_PER_A,
        max_current_a: cfg.max_current_a.unwrap_or(MAX_CURRENT_A),
        ..Params::default()
    };

    // ADR-0012: a kerb run opens a spliced copy; every other run opens the
    // shared model itself, unchanged.
    let terrain = match &cfg.terrain {
        Some(path) => Some(read_terrain_spec(path, cfg.spawn_x_m)?),
        None => None,
    };
    let base = cfg.plant.unwrap_or_default();
    let variation = PlantVariation {
        board_mass_kg: cfg.board_mass_kg.or(base.board_mass_kg),
        rider_mass_kg: cfg.rider_mass_kg.or(base.rider_mass_kg),
        kt_scale: cfg.kt_scale.or(base.kt_scale),
        tail_friction: cfg.tail_friction.or(base.tail_friction),
        rider_reach_m: cfg.rider_reach_m.or(base.rider_reach_m),
        ..base
    };
    let generated_model = if cfg.kerb.is_some()
        || terrain.is_some()
        || cfg.lean_steer
        || cfg.max_current_a.is_some()
        || variation != PlantVariation::default()
    {
        Some(write_model_with_kerb(
            cfg.kerb.as_ref(),
            terrain.as_ref(),
            cfg.lean_steer,
            cfg.max_current_a,
            variation,
            !cfg.hfield_wheel_contact,
            cfg.pad_solref_s,
        )?)
    } else {
        None
    };
    if cfg.ankle_hinge || cfg.ankle_rigid {
        let Some(path) = &generated_model else {
            return Err(HostError::Io(std::io::Error::other("sim-host: --ankle-hinge needs a generated model (use --lean-steer)")));
        };
        let xml = std::fs::read_to_string(path)?;
        std::fs::write(path, splice_ankle_hinge(&xml, cfg.ankle_rigid)?)?;
    }
    if let Some(csv) = &cfg.obstacles {
        let Some(path) = &generated_model else {
            return Err(HostError::Io(std::io::Error::other("sim-host: --obstacles needs a generated model (use --lean-steer)")));
        };
        let surface = match &terrain {
            Some(t) => Some(crate::ground::GroundSurface::from_hfield_bin(&t.hfield_path, t.half_extent_m)?),
            None => None,
        };
        let xml = std::fs::read_to_string(path)?;
        std::fs::write(path, splice_obstacles(&xml, csv, surface.as_ref())?)?;
    }
    if cfg.tumble {
        let Some(path) = &generated_model else {
            return Err(HostError::Io(std::io::Error::other("sim-host: --tumble needs a generated model (use --lean-steer)")));
        };
        let xml = std::fs::read_to_string(path)?;
        std::fs::write(path, splice_tumble_rider(&xml)?)?;
    }
    let model_path = generated_model.clone().unwrap_or_else(rider_model_path);
    if let Some(kerb) = &cfg.kerb {
        eprintln!(
            "sim-host: ADR-0012 kerb armed -- faces at y=+-{:.3} m (BOTH sides), \
             {:.0} mm tall, spanning x[{:.0},{:.0}] m; handoff will fire on \
             >{STRIKE_FORCE_N:.0} N of bumper contact or >{:.0} deg of tilt",
            kerb.y_face_m.abs(),
            kerb.height_m * 1000.0,
            CORRIDOR_X_MIN_M,
            CORRIDOR_X_MAX_M,
            HANDOFF_TILT_RAD.to_degrees(),
        );
    }
    // The backend's chain clamps at its own limit (IDEAL: 40 A); it must be
    // the host's limit, or a larger --max-current is a silent 40 A cap.
    let mut backend = SimBackend::with_model_path(params, model_path)
        .with_current_limit(cfg.max_current_a.unwrap_or(MAX_CURRENT_A) as f64);
    // `--sensors stage0`: the placeholder sensor and actuation imperfections
    // (gyro and accelerometer noise and bias, wheel-speed quantisation, 1 ms
    // delay, 1 ms current loop), with `--noise-scale` and `--extra-delay-ms`.
    // Without it the run is ideal: no noise, no delay.
    if cfg.sensors_stage0 {
        let base = sim_backend::imperfections::STAGE0_PLACEHOLDER;
        let k = cfg.noise_scale;
        let profile = sim_backend::imperfections::ImperfectionProfile {
            gyro_noise_rad_s: base.gyro_noise_rad_s * k,
            gyro_bias_rad_s: base.gyro_bias_rad_s * k,
            accel_noise_m_s2: base.accel_noise_m_s2 * k,
            actuation_delay_s: base.actuation_delay_s + cfg.extra_delay_ms / 1000.0,
            ..base
        };
        eprintln!(
            "sim-host: sensors stage0 x{k}: gyro noise {:.4} rad/s, accel noise {:.3} m/s^2, delay {:.1} ms",
            profile.gyro_noise_rad_s, profile.accel_noise_m_s2, profile.actuation_delay_s * 1000.0
        );
        backend = backend.with_imperfections(profile);
    }
    // Before `open()`, which is where the tilt is applied -- see
    // `HostConfig::incline_deg`.
    backend.set_incline_deg(cfg.incline_deg);
    // Before `open()`, same reason -- see `HostConfig::damping_scale`.
    backend.set_damping_scale(cfg.damping_scale);
    backend.open().map_err(HostError::Backend)?;
    // Smooth wheel contact: the heights the plate follows (crate::ground).
    let ground = match (&terrain, cfg.hfield_wheel_contact) {
        (Some(t), false) => Some(
            crate::ground::GroundSurface::from_hfield_bin(&t.hfield_path, t.half_extent_m)
                .map_err(HostError::Io)?,
        ),
        _ => None,
    };
    let place_wheel_ground = |backend: &mut SimBackend| {
        if let Some(g) = &ground {
            let (pos, quat) = g.plate_pose(backend.truth_frame_xpos(), DEFAULT_R_EFF_M as f64);
            backend.set_wheel_ground(pos, quat);
        }
    };
    place_wheel_ground(&mut backend);
    // `--hold-until-arm`: the spawn pose to hold, and whether a player has armed.
    let spawn_qpos = backend.truth_qpos();
    let mut armed_seen = !cfg.hold_until_arm;
    // Armed unconditionally at startup, the same way every other Rust-hosted
    // harness in this repo arms (`impulse-response-rust`, `sim-backend`'s own
    // tests): there is no synthetic Unreal client during a verification run,
    // and this host does not gate balancing on the input socket's `arm` bit
    // -- see `LatestInput::armed_bit`, tracked and logged but not wired to
    // anything.
    let _disarm = backend.arm().map_err(HostError::Backend)?;

    // CYCLE_NS is a duplicated constant, not derived -- checked against
    // sim-backend's own control rate the same way that crate checks
    // KT_NM_PER_A against the model's ctrlrange, rather than trusted blind.
    let reported_hz = backend.run_metadata().control_rate_hz as f64;
    let expected_hz = 1e9 / CYCLE_NS as f64;
    assert!(
        (reported_hz - expected_hz).abs() < 1e-6,
        "sim-host: CYCLE_NS ({CYCLE_NS} ns / {expected_hz} Hz) does not match \
         sim-backend's own control rate ({reported_hz} Hz)"
    );

    let mut envelope = Envelope::new(params);
    envelope.arm();

    let regulator = PitchRegulator::new(KP_NM_PER_RAD * cfg.kp_scale, KD_NM_PER_RAD_S * cfg.kd_scale);
    if cfg.kp_scale != 1.0 || cfg.kd_scale != 1.0 {
        eprintln!(
            "sim-host: balance gains Kp {:.0} N*m/rad, Kd {:.1} N*m*s/rad (study scales {} and {})",
            KP_NM_PER_RAD * cfg.kp_scale, KD_NM_PER_RAD_S * cfg.kd_scale, cfg.kp_scale, cfg.kd_scale
        );
    }
    let mut estimator = ComplementaryFilter::with_trust_band(ESTIMATOR_TAU_S, 0.0);
    // Lean-to-steer banks the board, and a single-axis pitch filter then
    // reads the turn's yaw rate as pitch (see `control_core::TiltFilter`).
    let mut tilt_estimator = control_core::TiltFilter::new(ESTIMATOR_TAU_S);
    let accel_ff = CommandFeedforward::new(ACCEL_FF_GAIN_M_S2_PER_A);
    // Only advanced when `cfg.estimator_aiding` selects it (issue #227) --
    // built unconditionally anyway, since a `WheelAccelEstimator` is cheap
    // and this keeps the loop body below free of a branch on construction.
    let mut wheel_accel = WheelAccelEstimator::new(WHEEL_ACCEL_TAU_S);
    // Last cycle's POST-envelope commanded current, amps -- the feedforward's
    // input (mode 2 / "commanded", matching `shuttle_run.py`'s own default
    // `accel_ff_current_source`, rather than the measured-current mode
    // `control-ffi`'s doc recommends for hardware). One cycle old by
    // construction, same as `control-ffi::ObController::last_amps`.
    let mut last_amps: f32 = 0.0;
    let mut grade_aid = control_core::GradeAwareAiding::new(ACCEL_FF_GAIN_M_S2_PER_A, GRADE_LOAD_TAU_S);
    // Same gains as hill.py / shuttle_run.py's outer loop.
    // Tuning only: OVERBOARD_SPEED_LOOP="kp,ki,max_ref_deg".
    let (sl_kp, sl_ki, sl_max_deg) = std::env::var("OVERBOARD_SPEED_LOOP")
        .ok()
        .and_then(|v| {
            let f: Vec<f32> = v.split(',').filter_map(|x| x.trim().parse().ok()).collect();
            (f.len() == 3).then(|| (f[0], f[1], f[2]))
        })
        .unwrap_or((0.05, 0.02, 5.0));
    let mut speed_loop = control_core::VelocityLoop::new(
        sl_kp,
        sl_ki,
        sl_max_deg.to_radians(),
        control_core::PlantCoupling::ComAboveAxle,
    );
    // `--speed-hold` law: full-state LQR from sim/carve/lqr_design.py.
    // Tuning only: OVERBOARD_SPEED_LQR="k_pitch,k_rate,k_speed,k_int,accel".
    let lqr_gains = std::env::var("OVERBOARD_SPEED_LQR")
        .ok()
        .and_then(|v| {
            let f: Vec<f32> = v.split(',').filter_map(|x| x.trim().parse().ok()).collect();
            (f.len() == 5).then(|| (f[0], f[1], f[2], f[3], f[4]))
        })
        .unwrap_or((376.8, 88.4, 28.35, 8.06, 0.5));
    let new_speed_lqr = || {
        control_core::SpeedHoldLqr::new(lqr_gains.0, lqr_gains.1, lqr_gains.2, lqr_gains.3, lqr_gains.4)
            // Steady lean and current per m/s^2, from lqr_design.py's linear model.
            .with_feedforward(-0.1261, 17.82)
    };
    let mut speed_lqr = new_speed_lqr();
    // OVERBOARD_SPEED_HOLD_BASELINE: the law without grade feedforward or
    // governor, kept so the 2026-10-04 Monte Carlo baseline can be re-taken.
    let speed_hold_baseline = std::env::var_os("OVERBOARD_SPEED_HOLD_BASELINE").is_some();
    // `--start-speed`: applied once the board has settled, not at t = 0. A
    // board that is moving on its first IMU sample has its estimator start
    // from a deceleration it reads as tilt (measured: -4.2 deg, 27 A), and on
    // terrain the spawn drop makes that worse. A real board also starts its
    // estimator at rest.
    let mut start_speed_pending = cfg.start_speed_m_s;
    let mut margin = control_core::AuthorityMargin::new();
    let mut rider_model = RiderSpeedModel::default();
    // `--tumble`: the rider is free (fall) or off (dismount); the motor is cut.
    let mut rider_free = false;
    // Pad rest (see the loop): stopped on a pad, motor off until the deck is level.
    let mut pad_mode = false;
    // `--ankle-hinge` (see HostConfig::ankle_hinge).
    let ankle_m = variation.rider_mass_kg.unwrap_or(70.0) as f32;
    let ankle_h = 0.75 - ANKLE_PIVOT_Z_M as f32;
    let ankle_mgh = ankle_m * 9.81 * ankle_h;
    let ankle_j = ankle_m * ankle_h * ankle_h + 0.15 * ankle_m;
    let ankle_k = 1.3 * ankle_mgh;
    let ankle_c = 2.0 * 0.7 * (ankle_j * (ankle_k - ankle_mgh)).sqrt();
    let ankle_cap = ankle_m * 9.81 * ANKLE_COP_M;
    let mut ankle_lean_prev: Option<f32> = None;
    let mut ankle_rate_f = 0.0_f32;
    let mut ankle_peak_nm = 0.0_f32;
    if cfg.ankle_hinge {
        eprintln!(
            "sim-host: --ankle-hinge: K {ankle_k:.0} N*m/rad, C {ankle_c:.0} N*m*s/rad, limit {ankle_cap:.0} N*m"
        );
    }
    let mut pad_grade_hold_s = 0.0_f64;
    let mut last_pitch_rad = 0.0_f32;
    let mut target_changed = false;
    let mut motor_cut = false;
    let mut handoff_at_s: Option<f64> = None;
    let mut pose_rows: Vec<String> = Vec::new();
    // `--rider-reach`: the stick spans the rider's reach, and the rider model
    // uses the same share of it (60 %) as of the model's 5 cm.
    let fore_aft_range_m = variation.rider_reach_m.map_or(BALLAST_RANGE_M, |r| r as f32);
    // `--rider-reach-back M`: how far back the stick takes the rider. A rider
    // who stops hard bends the knees and leans back with the deck, far past
    // the forward lean: stopping from 6.5 m/s in 7 m needs the centre of
    // mass about 0.28 m behind the wheel; 0.10 m allows about 1.1 m/s^2.
    let fore_aft_back_range_m = cfg.rider_reach_back_m.map_or(fore_aft_range_m, |r| r as f32);
    rider_model.bound = RiderSpeedModel::LEAN_BOUND * fore_aft_range_m / BALLAST_RANGE_M;
    let mut grade_comp = control_core::GradeCompensator::new();
    let mut battery = crate::hud::BatteryModel::new(cfg.batt_soc0);
    let mut hud_seq: u64 = 0;
    let hud_kt = KT_NM_PER_A as f64 * variation.kt_scale.unwrap_or(1.0);
    let hud_mass = variation.board_mass_kg.unwrap_or(13.0) + variation.rider_mass_kg.unwrap_or(70.0);
    let hud_i_limit = cfg.max_current_a.unwrap_or(MAX_CURRENT_A);
    let mut margin_level = control_core::MarginLevel::None;
    let mut margin_level_since_s = 0.0f64;
    let mut rider_eased = false;
    let mut dismount_at_s: Option<f64> = None;
    // `--grade-course`: last cycle's grade and its rate (see the gyro fix-up).
    let mut grade_rad_prev: Option<f32> = None;
    let mut grade_rate_rad_s: f32 = 0.0;
    let mut last_saturated = false;

    let out_socket = UdpSocket::bind("127.0.0.1:0")?;
    let in_socket = UdpSocket::bind(cfg.input_in_addr)?;
    // The 500 Hz loop must never block on recv (issue #161).
    in_socket.set_nonblocking(true)?;

    let mut latest_input = LatestInput::default();
    let mut latest_input_at: Option<Instant> = None;
    // Last logged scripted-schedule phase label, so a scripted run announces
    // phase changes once rather than every tick (issue #190).
    let mut scripted_label: &'static str = "";
    let mut prev_armed_bit = false;
    let mut prev_reset_bit = false;
    // ADR-0012. `handoff_latched` is a LEVEL, not an edge: once set it stays
    // set until the input `reset` bit clears it, so a client that loses the
    // announcing packet to the UDP wire still takes over on the next one.
    // `handoff_state` is the frozen snapshot every subsequent packet repeats.
    // ADR-0012: the kerb has to be REACHABLE. The soft corridor brake fires
    // at CORRIDOR_HALF_WIDTH_M = 8.6 m, and the measured City Park kerb face
    // is at 8.85 m -- so on an unmodified corridor the host arrests the
    // player's forward lean 0.25 m short of the thing they are aiming at, and
    // the strike never happens. Measured: an s-curve run reached y = 8.6 m,
    // got braked, and came back.
    //
    // When a kerb is armed the lateral limit moves outside it, so what stops
    // the board is the kerb's own geometry rather than an invisible brake.
    // That is also what ADR-0012 says the arrangement IS: the kerb is
    // authored outside the drivable corridor, and striking it terminates the
    // run. The X bounds are untouched -- they keep the board inside a finite
    // Unreal level, which a kerb strike does nothing about.
    let corridor_half_width_m = match &cfg.kerb {
        Some(k) => CORRIDOR_HALF_WIDTH_M.max(k.y_face_m.abs() + 0.5),
        None => CORRIDOR_HALF_WIDTH_M,
    };
    // The heightfield is finite. Outside its extent there is no geom at all --
    // not flat ground, NOTHING, because the terrain splice replaces the plane
    // rather than overlaying it (see write_model_with_kerb for why). A board
    // that leaves the grid falls forever. The corridor is therefore clamped
    // inside the grid with a margin, so the existing soft lean-arrest turns
    // the board back before it reaches an edge that has no floor beyond it.
    let (corridor_x_min_m, corridor_x_max_m, corridor_half_width_m) = match &terrain {
        Some(t) => {
            let limit = t.half_extent_m - TERRAIN_EDGE_MARGIN_M;
            (
                CORRIDOR_X_MIN_M.max(-limit),
                CORRIDOR_X_MAX_M.min(limit),
                corridor_half_width_m.min(limit),
            )
        }
        None => (CORRIDOR_X_MIN_M, CORRIDOR_X_MAX_M, corridor_half_width_m),
    };
    if terrain.is_some() {
        eprintln!(
            "sim-host: drivable corridor clamped to the heightfield: \
             x[{corridor_x_min_m:.1},{corridor_x_max_m:.1}] y+-{corridor_half_width_m:.1} m"
        );
    }

    let mut handoff_latched = false;
    let mut handoff_state: Option<HandoffSnapshot> = None;
    let mut prev_kick_bit = false;
    // `Some(t)` while an on-demand fall kick (issue #161 follow-up, item 5)
    // is in its force-application window, `t` being the sim time it started
    // -- mirrors `STARTUP_KICK_T0_S`'s fixed window but anchored to whenever
    // the rising edge arrived rather than a fixed offset from run start.
    // `None` both before the first trigger and again once a window has
    // finished, so a later rising edge can retrigger it.
    let mut fall_kick_window_start_s: Option<f64> = None;
    let mut prev_outside_corridor = false;
    // The heading this host has injected into the plant so far, radians,
    // unbounded (it is a running total, not an angle in `[-pi, pi]`) -- the
    // integral of the yaw law at the bottom of the loop body. Two readers:
    // the state-out wire's `yaw_rad` field, whose contract is exactly this
    // continuous running total (see `wire::StateOut::yaw_rad`), and the
    // attitude de-rotation below, which needs to know how much world-frame
    // yaw is currently baked into MuJoCo's own quaternion before it can read
    // body pitch and roll back out of it.
    let mut yaw_rad: f32 = 0.0;
    // Lean-to-steer state (only used with `cfg.lean_steer`). Roll and roll
    // rate are last tick's: the rider reacts one 2 ms cycle late.
    let lean_params = crate::lean_steer::LeanSteerParams::from_env();
    let mut tire_yaw = crate::lean_steer::TireYaw::default();
    let mut lean_roll_rad: f32 = 0.0;
    let mut lean_roll_rate_rad_s: f32 = 0.0;
    let mut rider = crate::lean_steer::Rider::default();
    let lean_debug = std::env::var("OVERBOARD_LEAN_DEBUG").is_ok();
    let mut lean_balance_torque_nm: f32 = 0.0;
    let mut lean_balance_peak_nm: f32 = 0.0;
    let mut lean_yaw_torque_nm: f64 = 0.0;
    let mut wheel_angle_rad: f32 = 0.0;
    // Previous tick's ground speed, m/s, signed (positive = forward) -- used
    // to gate THIS tick's `weight_shift_fore_aft` against MAX_GROUND_SPEED_M_S
    // before the ballast target is set (which happens before this tick's own
    // fresh speed is known -- see the "Ballast targets" section below). One
    // cycle old by construction, the same lag `last_amps` already has for the
    // command-feedforward estimator.
    let mut last_forward_speed_m_s: f32 = 0.0;
    // Latest TRUE MuJoCo x/y -- now BOTH the wire's `pos` and the corridor
    // check's input (issue #163), as well as `write_stats`'s. Kept in a
    // variable across ticks because the corridor check runs at the top of the
    // loop body, before this tick's own observation, and so reads the
    // PREVIOUS tick's truth -- the same one-cycle lag `last_forward_speed_m_s`
    // has, for the same reason.
    let mut truth_pos_x_m: f64 = 0.0;
    let mut truth_pos_y_m: f64 = 0.0;

    // --- ADR-0011 (b) and (c) state ------------------------------------
    // The fore/aft command scale actually in force this run. Normally the
    // shipped constant; a verification sweep overrides it (see
    // HostConfig::cmd_envelope_reserve, which is how the constant's own
    // provenance measurement is taken).
    let cmd_envelope_reserve = cfg.cmd_envelope_reserve.unwrap_or(CMD_ENVELOPE_RESERVE);
    // `--cmd-reserve` alone still means "scale the WHOLE fore/aft command by
    // this", which is what every sweep that predates the braking reserve
    // assumes -- `PEAK_DEMAND_A_PER_UNIT_STICK`'s provenance sweep among them,
    // and `--cmd-reserve 0` as the way to say "no stick at all". Without this
    // fallback a run asking for zero stick still gets a braking command the
    // moment the board rolls backwards, which is exactly how it was found.
    let cmd_envelope_reserve_braking = cfg
        .cmd_envelope_reserve_braking
        .or(cfg.cmd_envelope_reserve)
        .unwrap_or(CMD_ENVELOPE_RESERVE_BRAKING);
    let pitch_bias_rad = cfg.pitch_bias_deg.to_radians();
    // Low-passed |proposed current| / MAX_CURRENT_A. Starts at zero, which is
    // true: an unarmed board at rest is asking for nothing.
    let mut utilisation_filtered: f32 = 0.0;
    // One-pole coefficient for AUTHORITY_UTILISATION_TAU_S at this cycle
    // rate, precomputed rather than recomputed 500 times a second.
    let utilisation_alpha = DT_S as f32 / (AUTHORITY_UTILISATION_TAU_S + DT_S as f32);
    // Edge state for the loss-of-authority warning, so it announces itself
    // once per episode rather than 500 times a second -- the same
    // rising-edge discipline the corridor warning above uses.
    let mut prev_authority_warning = false;
    let mut trace: Vec<TraceRow> = Vec::new();
    if cfg.trace_path.is_some() {
        // A 30 s scripted run at 500 Hz is 15,000 rows; reserving avoids
        // reallocating inside the control loop.
        trace.reserve(16_384);
    }

    let start = Instant::now();
    let mut pacer = Pacer::new(Duration::from_nanos(CYCLE_NS), start);
    // Headroom over InputIn::WIRE_SIZE so an oversized datagram is caught as
    // a WrongSize mismatch by InputIn::from_bytes rather than silently
    // truncated by a too-small recv buffer.
    let mut recv_buf = [0u8; InputIn::WIRE_SIZE + 16];
    let mut last_stats_write = Instant::now();
    let mut ticks: u64 = 0;
    // Pre-step sim time, mirroring impulse-response-rust's own `t_known_s`:
    // the window check below must use the time as of the START of this
    // tick, since `apply_external_force` only takes effect on the NEXT
    // `wait_observe()` (see [`STARTUP_KICK_T0_S`]).
    let mut t_known_s: f64 = 0.0;

    loop {
        if let Some(d) = cfg.duration {
            if start.elapsed() >= d {
                break;
            }
        }
        // A scripted run also stops on SIMULATED time (issue #190), so its
        // length is a property of the schedule rather than of how fast the
        // laptop happened to be -- the same reason the schedule itself is
        // indexed on sim time. `cfg.duration`, if set, still applies as a
        // wall-clock backstop.
        if let Some(sched) = cfg.scripted_scenario {
            if t_known_s > crate::scenario::total_duration_s(sched) + SCRIPTED_RUN_TAIL_S {
                break;
            }
        }
        if let Some(limit) = cfg.max_sim_time_s {
            if t_known_s > limit {
                break;
            }
        }
        // `--rider-reacts`: the rider has stepped off; the run is over.
        if dismount_at_s.is_some_and(|t| t_known_s > t + cfg.stop_after_handoff_s.unwrap_or(0.2)) {
            break;
        }
        if let (Some(s), Some(t0)) = (cfg.stop_after_handoff_s, handoff_at_s) {
            if t_known_s > t0 + s {
                break;
            }
        }

        // Drain every pending datagram; only the most recent VALID one
        // matters (issue #161: "Use the most recent packet received").
        loop {
            match in_socket.recv_from(&mut recv_buf) {
                Ok((n, _src)) => match InputIn::from_bytes(&recv_buf[..n]) {
                    Ok(pkt) => {
                        eprintln!(
                            "sim-host: input seq={} weight_fore_aft={:.3} weight_lateral={:.3} \
                             steer={:.3} arm={} reset={}",
                            { pkt.seq },
                            { pkt.weight_shift_fore_aft },
                            { pkt.weight_shift_lateral },
                            { pkt.steer },
                            (pkt.flags & wire::INPUT_FLAG_ARM) != 0,
                            (pkt.flags & wire::INPUT_FLAG_RESET) != 0,
                        );
                        latest_input = LatestInput::from(pkt);
                        latest_input_at = Some(Instant::now());
                    }
                    // Fail loudly, drop the packet, never misparse it as a
                    // float (issue #161).
                    Err(e) => eprintln!("sim-host: dropping malformed input packet: {e}"),
                },
                Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                Err(e) => {
                    eprintln!("sim-host: input socket recv error: {e}");
                    break;
                }
            }
        }

        let stale = latest_input_at
            .map(|t| t.elapsed() > INPUT_STALENESS_TIMEOUT)
            .unwrap_or(true);
        // weight_shift_*/steer hold last, then zero after the staleness
        // timeout (issue #161) -- UNLESS a scripted scenario is driving this
        // run (issue #190), in which case the schedule is read straight off
        // SIMULATED time and the socket's stick values are ignored entirely.
        // The flag bits (arm/reset/kick) still come from the wire either way,
        // so an on-demand kick can still be injected into a scripted run.
        let (steer, stick_fore_aft, weight_shift_lateral) = match cfg.scripted_scenario {
            Some(sched) => {
                let (fa, lat, st, label) = crate::scenario::value_at(sched, t_known_s);
                if label != scripted_label {
                    eprintln!(
                        "sim-host: scripted sim_t={t_known_s:6.2}s -> {label} \
                         (fore_aft={fa:+.2} lateral={lat:+.2} steer={st:+.2})"
                    );
                    scripted_label = label;
                }
                (st, fa, lat)
            }
            None if stale => (0.0, 0.0, 0.0),
            None => (
                latest_input.steer,
                latest_input.weight_shift_fore_aft,
                latest_input.weight_shift_lateral,
            ),
        };
        // `--rider-speed`: the rider model leans; the deployed law balances.
        let stick_fore_aft = match cfg.rider_speed_m_s {
            Some(v_target) => {
                if start_speed_pending.is_some() {
                    rider_model.reset();
                    0.0
                } else {
                    let grade_down_rad = cfg
                        .grade_course
                        .as_ref()
                        .map_or(0.0, |c| -(c.grade_deg(-truth_pos_x_m) as f32).to_radians());
                    let v_target = match cfg.rider_target_change {
                        Some((t_change, v)) if t_known_s >= t_change => {
                            if !target_changed {
                                target_changed = true;
                                rider_model.set_target_now(v);
                                // A hard brake: the rider leans as far back as they can.
                                rider_model.bound = fore_aft_range_m / BALLAST_RANGE_M;
                            }
                            v
                        }
                        _ => v_target,
                    };
                    let target = if rider_eased { 0.0 } else { v_target };
                    rider_model.update(last_forward_speed_m_s, target, grade_down_rad, DT_S as f32)
                        * (BALLAST_RANGE_M / fore_aft_range_m)
                }
            }
            None => stick_fore_aft,
        };

        // --- COMMAND-ENVELOPE RESERVE (ADR-0011 exit criterion (b)) ------
        //
        // The whole fix, in one multiply. Deliberately placed HERE, at the
        // point where a stick value enters the host and before anything else
        // touches it: upstream of the speed cap, the corridor brake, the
        // ballast actuator, the plant, the estimator, the regulator and the
        // safety envelope. That ordering is the reason this is a zero-risk
        // change rather than a retune -- every downstream block sees a
        // smaller stick and is otherwise bit-identical, no gain moves, and
        // nothing in the control law learns that a limit exists.
        //
        // Fore/aft only. The failure ADR-0011 is about is a fore/aft pitch
        // authority failure; lateral stick moves `ballast_lat`, whose
        // measured full-stick effect on this geometry is 0.03 deg of roll
        // (see ROLL_FULL_YAW_AUTHORITY_RAD) and which consumes no wheel
        // torque at all. Scaling it would cost steering feel to buy nothing.
        //
        // Braking spends its own reserve (CMD_ENVELOPE_RESERVE_BRAKING).
        // Gated on `last_forward_speed_m_s` -- one cycle old, the same lag
        // the speed cap below runs on, and for the same reason.
        let weight_shift_fore_aft = shape_fore_aft_command_directional(
            stick_fore_aft,
            cmd_envelope_reserve,
            cmd_envelope_reserve_braking,
            last_forward_speed_m_s,
        );

        // `arm`/`reset` bits: accepted and tracked, logged on change rather
        // than every tick. Neither gates anything yet -- the host self-arms
        // unconditionally (see the comment on `backend.arm()` above), and
        // there is no reset implementation to wire `reset` into. `stale`
        // zeroes both the same way it zeroes weight_shift/steer.
        let input_armed_bit = !stale && latest_input.armed_bit;
        if !armed_seen && input_armed_bit {
            armed_seen = true;
            eprintln!("sim-host: armed at sim_t={t_known_s:.3}s -- the board is released");
        }
        // `--hold-until-arm`: the host puts the held board back every step, so
        // a controller that runs meanwhile winds up against it (the game saw
        // +310 A of demand and a SOLID warning after 50 s held, and a nose
        // strike 1.6 s after a late arm). While held: no current, and the
        // loops and filters start from zero at release. The attitude
        // estimator runs on, so it is settled at release.
        if !armed_seen {
            grade_aid.reset();
            grade_comp.reset();
            speed_loop.reset();
            speed_lqr.reset();
            utilisation_filtered = 0.0;
            margin = control_core::AuthorityMargin::new();
            last_amps = 0.0;
        }
        let input_reset_bit = !stale && latest_input.reset_bit;
        if input_reset_bit && !prev_reset_bit {
            // ADR-0012 gave this bit its first real job: it is the ONLY way
            // out of a handoff, and it is what returns authority to MuJoCo.
            // Outside a handoff it still does nothing, and still says so.
            // A RESET IS NOT JUST "UN-LATCH". Clearing the handoff alone
            // handed authority back to MuJoCo with the board still lying
            // wherever it had tumbled to, tens of metres downrange -- so the
            // player got control of a fallen board rather than a fresh run.
            // Reported from the first play-test, and the reason this now
            // resets the PLANT and every piece of loop state derived from it.
            //
            // Everything below is state that outlives a single cycle. Leaving
            // any of it behind puts the fresh board under the influence of the
            // crashed one: a stale `yaw_rad` respawns it pointing the wrong
            // way, a stale estimator spends its settling time regulating a
            // vertical from the old attitude, and a stale
            // `utilisation_filtered` can annunciate a loss-of-authority
            // warning about a board that no longer exists.
            backend.reset();
            handoff_latched = false;
            handoff_state = None;
            yaw_rad = 0.0;
            tire_yaw = crate::lean_steer::TireYaw::default();
            lean_roll_rad = 0.0;
            lean_roll_rate_rad_s = 0.0;
            lean_yaw_torque_nm = 0.0;
            rider = crate::lean_steer::Rider::default();
            lean_balance_torque_nm = 0.0;
            estimator = ComplementaryFilter::with_trust_band(ESTIMATOR_TAU_S, 0.0);
            tilt_estimator = control_core::TiltFilter::new(ESTIMATOR_TAU_S);
            last_amps = 0.0;
            grade_aid.reset();
            grade_comp.reset();
            speed_loop.reset();
            speed_lqr.reset();
            last_forward_speed_m_s = 0.0;
            utilisation_filtered = 0.0;
            prev_outside_corridor = false;
            fall_kick_window_start_s = None;
            eprintln!(
                "sim-host: input reset bit set -- board returned to the start, \
                 plant and loop state cleared"
            );
        }
        if input_armed_bit != prev_armed_bit {
            eprintln!(
                "sim-host: input arm bit is now {input_armed_bit} \
                 (host self-arms regardless of this bit)"
            );
        }
        prev_armed_bit = input_armed_bit;
        prev_reset_bit = input_reset_bit;

        // On-demand fall kick (issue #161 follow-up, item 5): rising-edge
        // triggered, like `reset` above, so holding the bit does not restart
        // it every tick. Does not retrigger while a window is already in
        // progress (`fall_kick_window_start_s` only goes back to `None`
        // once that window's force-application section below has finished
        // it) -- one kick per press.
        let input_kick_bit = !stale && latest_input.kick_bit;
        if input_kick_bit && !prev_kick_bit && fall_kick_window_start_s.is_none() {
            eprintln!("sim-host: input kick bit set -- inducing a fall");
            fall_kick_window_start_s = Some(t_known_s);
        }
        prev_kick_bit = input_kick_bit;

        // Speed cap (issue #161 follow-up, MAX_GROUND_SPEED_M_S's own doc
        // comment) -- attenuates weight_shift_fore_aft, not the wheel speed
        // itself: if the board is already moving in the SAME direction this
        // input would accelerate it (or is at rest), ramp authority down to
        // zero over the last SPEED_CAP_MARGIN_M_S below the cap. Braking or
        // reversing (opposite sign to current motion) is never touched.
        // Gated on last_forward_speed_m_s -- one cycle old, since this
        // tick's fresh speed is not known until wait_observe() below.
        let capped_weight_shift_fore_aft =
            speed_capped_fore_aft(weight_shift_fore_aft, last_forward_speed_m_s);

        // Corridor boundary (issue #161 follow-up, item 4) -- see
        // CORRIDOR_X_MIN_M's doc comment for the bounds and for why this is a
        // soft lean rather than contact. Checked against MuJoCo TRUTH position
        // as of the END of the PREVIOUS tick (issue #163: this used to read
        // the dead-reckoned path, which no longer exists) -- the same
        // one-cycle lag the speed cap above uses, and for the same reason.
        // The corridor bounds the flat game plane. On a `--terrain` heightmap
        // the terrain itself bounds the run (and a course may sit anywhere in
        // the frame -- an authored course starting at x = +88 m was braked by
        // the corridor the whole way), so the corridor is off there.
        let outside_corridor = cfg.terrain.is_none()
            && (!(corridor_x_min_m..=corridor_x_max_m).contains(&truth_pos_x_m)
                || truth_pos_y_m.abs() > corridor_half_width_m);
        if outside_corridor && !prev_outside_corridor {
            eprintln!(
                "sim-host: LEFT THE DRIVABLE CORRIDOR at ({truth_pos_x_m:.1}, \
                 {truth_pos_y_m:.1}) m -- arresting forward lean"
            );
        } else if prev_outside_corridor && !outside_corridor {
            eprintln!("sim-host: back inside the drivable corridor");
        }
        prev_outside_corridor = outside_corridor;
        // Overrides the speed cap's own result, not just `weight_shift_
        // fore_aft` -- an active brake against whatever direction the
        // board was travelling, not merely "stop accelerating further"
        // (which alone would let it coast on out under residual speed).
        // Lateral/steer are untouched -- only forward travel is arrested.
        let corridor_enforced_fore_aft = if outside_corridor {
            if last_forward_speed_m_s > 0.0 {
                -CORRIDOR_BRAKE_LEAN
            } else if last_forward_speed_m_s < 0.0 {
                CORRIDOR_BRAKE_LEAN
            } else {
                0.0
            }
        } else {
            capped_weight_shift_fore_aft
        };

        // Ballast targets -- weight_shift_fore_aft/lateral drive
        // overboard_rider.xml's two ballast actuators DIRECTLY AND
        // PHYSICALLY (see this file's header). Set every cycle, mirroring
        // apply_external_force's own "call every cycle or a stale value
        // persists" convention.
        // With lean-to-steer the rider model owns the hips and the balance
        // torque: it cambers the board for the curvature `steer` asks for
        // and keeps it balanced (see `lean_steer`).
        let lateral_target_m = if cfg.lean_steer {
            let obs = crate::lean_steer::RiderObs {
                roll_rad: lean_roll_rad,
                roll_rate_rad_s: lean_roll_rate_rad_s,
            };
            let cmd = rider.command(&lean_params, steer, last_forward_speed_m_s, &obs, DT_S as f32);
            lean_balance_torque_nm = cmd.roll_torque_nm;
            lean_balance_peak_nm = lean_balance_peak_nm.max(cmd.roll_torque_nm.abs());
            if lean_debug && ticks.is_multiple_of(125) {
                eprintln!(
                    "LEAN t={t_known_s:6.2} v={last_forward_speed_m_s:5.2} steer={steer:+.2} \
                     kappa={:+.3} phi_ref={:+5.1} phi={:+5.1} d={:+.3} tau={:+6.1}",
                    rider.kappa_intent_per_m,
                    crate::lean_steer::roll_reference(&lean_params, rider.kappa_intent_per_m)
                        .to_degrees(),
                    obs.roll_rad.to_degrees(),
                    cmd.offset_m,
                    cmd.roll_torque_nm
                );
            }
            cmd.offset_m
        } else {
            weight_shift_lateral * BALLAST_RANGE_M
        };
        // `--rider-ankle`: a person stays upright with the ankles and knees as
        // the deck pitches; the model's rider is otherwise rigid with the deck.
        // Without it, on the tail pad the rider's mass swings back to the pad's
        // tipping edge and the board pivots over (fable-oracle, 2026-10-04).
        let upright_m = if cfg.rider_ankle {
            (RIDER_COM_HEIGHT_M * last_pitch_rad.sin()).clamp(-0.30, 0.30)
        } else {
            0.0
        };
        if cfg.ankle_hinge || cfg.ankle_rigid {
            if let Some(lean) = backend.truth_rider_body_lean() {
                let lean = lean as f32;
                let rate = ankle_lean_prev.map_or(0.0, |p| (lean - p) / DT_S as f32);
                ankle_lean_prev = Some(lean);
                ankle_rate_f += (DT_S as f32 / (0.005 + DT_S as f32)) * (rate - ankle_rate_f);
                // A positive hinge angle tilts the body back (axis +Y, forward
                // is -X), so the restoring torque for a forward lean is +.
                // `--ankle-rigid` (the bracket): a stiff joint spring against
                // the DECK, which must reproduce the rigid rider.
                let tau = if cfg.ankle_rigid {
                    0.0 // the joint's own stiffness (splice_ankle_hinge)
                } else {
                    // `--foot-torque`: heel and toe pressure. Fore/aft stick s also
                    // pushes the deck through the feet: s = -1 (heels down)
                    // tips the deck nose-up, and the stiff law brakes to the
                    // tail. A positive hinge torque pushes the deck nose-up
                    // (measured: the other sign sped the board up).
                    let foot = if cfg.foot_torque {
                        -corridor_enforced_fore_aft * FOOT_TORQUE_SHARE * ankle_cap
                    } else {
                        0.0
                    };
                    (ankle_k * lean + ankle_c * ankle_rate_f + foot).clamp(-ankle_cap, ankle_cap)
                };
                ankle_peak_nm = ankle_peak_nm.max(tau.abs());
                backend.set_ankle_pitch_torque(tau as f64);
            }
        }
        backend.set_ballast_targets(
            corridor_enforced_fore_aft
                * if corridor_enforced_fore_aft < 0.0 {
                    // The back lean fades out below 2 m/s: a rider stands up as
                    // the board stops on its tail. Held at a standstill, even a
                    // 0.10 m lean-back put the (deck-rigid) rider's mass at the
                    // tail pad's tipping edge and the board went over.
                    let k = (last_forward_speed_m_s.abs() / BACK_REACH_FULL_SPEED_M_S).min(1.0);
                    k * fore_aft_back_range_m
                } else {
                    fore_aft_range_m
                }
                + upright_m,
            lateral_target_m,
        );

        // One-time startup kick, only when explicitly enabled (issue #169)
        // -- see STARTUP_KICK_T0_S's doc comment. Checked against the
        // PRE-step time, mirroring impulse_response.py's `if t0 <= data.time
        // < t0 + duration` (also checked there before that iteration's
        // mj_step).
        let in_kick_window = cfg.startup_kick
            && (STARTUP_KICK_T0_S..STARTUP_KICK_T0_S + STARTUP_KICK_DURATION_S)
                .contains(&t_known_s);
        // On-demand fall kick (issue #161 follow-up, item 5) -- same
        // pre-step-time window check as the startup kick above, just
        // anchored to `fall_kick_window_start_s` instead of a fixed
        // `STARTUP_KICK_T0_S`. Cleared back to `None` once the window has
        // elapsed so a later rising edge can retrigger it (see where it is
        // set, above).
        let in_fall_kick_window = fall_kick_window_start_s
            .is_some_and(|t0| (t0..t0 + FALL_KICK_DURATION_S).contains(&t_known_s));
        if let Some(t0) = fall_kick_window_start_s {
            if t_known_s >= t0 + FALL_KICK_DURATION_S {
                fall_kick_window_start_s = None;
            }
        }
        // A scheduled acceptance-run disturbance (ADR-0011 criterion (a),
        // "full stick during a kerb strike"), on the same pre-step-time
        // window rule as the two kicks above. Unlike them it carries a
        // TORQUE as well as a force: a kerb acts at the wheel, well below the
        // CoM, and the angular channel is the one that topples the board --
        // see `Disturbance` and issue #142's finding that quoting a kerb in
        // N*s alone understates it silently.
        let in_disturbance_window = cfg
            .disturbance
            .is_some_and(|d| (d.t0_s..d.t0_s + d.duration_s).contains(&t_known_s));
        let (force, torque) = select_disturbance_force_torque(
            in_disturbance_window,
            cfg.disturbance,
            in_kick_window,
            in_fall_kick_window,
        );
        let mut torque = torque;
        if cfg.lean_steer {
            // Tire turn-slip moment (see `crate::lean_steer`), about world +Z.
            if !rider_free {
                torque[2] += lean_yaw_torque_nm;
            }
            // Rider balance skill, about the board's forward axis. Forward is
            // body -X, and + rolls the board right (top towards body +Y),
            // which is a rotation about body -X.
            let xm = backend.truth_frame_xmat();
            let tau = if rider_free { 0.0 } else { lean_balance_torque_nm as f64 };
            torque[0] -= tau * xm[0];
            torque[1] -= tau * xm[3];
            torque[2] -= tau * xm[6];
        }
        backend.apply_external_force(force, torque);

        place_wheel_ground(&mut backend);
        if !armed_seen {
            backend.hold_pose(&spawn_qpos);
        }
        if let Some(course) = &cfg.grade_course {
            let grade_deg = course.grade_deg(-truth_pos_x_m);
            backend.set_grade_deg(grade_deg);
            let grade_rad = grade_deg.to_radians() as f32;
            grade_rate_rad_s = (grade_rad - grade_rad_prev.unwrap_or(grade_rad)) / DT_S as f32;
            grade_rad_prev = Some(grade_rad);
        }
        if let Some(v) = start_speed_pending.filter(|_| t_known_s >= START_SPEED_AT_S) {
            backend.set_forward_speed(v, DEFAULT_R_EFF_M as f64);
            // The reference and the grade aid restart from the new speed.
            speed_lqr = new_speed_lqr();
            grade_aid.reset();
            start_speed_pending = None;
        }
        let obs = backend.wait_observe().map_err(HostError::Backend)?;
        t_known_s = obs.t_recv_ns as f64 * 1e-9;

        // Controller: raw IMU -> estimate -> regulate -> envelope. Aiding
        // mode is `cfg.estimator_aiding` (default: command feedforward,
        // matching `shuttle_run.py`'s tuned ridden config -- "the
        // recommended configuration" per that scenario's own comment).
        // `pitch_ref` is always 0: no outer loop (see this file's header).
        let mut sample = obs.newest_imu().copied().unwrap_or(ImuSample::ZERO);
        // `--grade-course` turns gravity, not the road. A board held vertical
        // then rotates against the plane, and the gyro reads that; on a real
        // vertical curve the body stays vertical and the gyro reads ~0. Add
        // the frame rate back (nose-up positive), or the complementary filter
        // lags the curve by rate x tau: measured 2.6 deg at 2 m/s, R = 100 m,
        // which saturated the motor at the curve end.
        sample.gyro_rad_s[1] += GRADE_GYRO_SIGN * grade_rate_rad_s;
        let wheel_rate_rad_s = obs.erpm * RAD_S_PER_ERPM;
        // Real forward ground speed, m/s, signed -- computed here (rather
        // than only down in the dead-reckoning block that used to be its
        // only reader) because the speed-proportional yaw rate below now
        // needs it too. Kept for the NEXT tick's speed-cap gate (see
        // `last_forward_speed_m_s` above) at the end of this loop body.
        let forward_speed_m_s = wheel_rate_rad_s * DEFAULT_R_EFF_M;
        last_forward_speed_m_s = forward_speed_m_s;
        // Issue #227: the two aiding sources are mutually exclusive, matching
        // `control-ffi`'s "at most one is Some" (`crates/control-ffi/src/
        // lib.rs`). `wheel_accel` is only ADVANCED on the branch that uses
        // it, so its filter state does not silently accumulate on a run that
        // never reads it.
        let aiding = match cfg.estimator_aiding {
            EstimatorAiding::CommandFeedforward => accel_ff.predict(last_amps),
            EstimatorAiding::GradeAware => {
                let f = sample.accel_m_s2;
                let mag = (f[0] * f[0] + f[1] * f[1] + f[2] * f[2]).sqrt();
                grade_aid.update(last_amps, forward_speed_m_s, mag, DT_S as f32)
            }
            EstimatorAiding::WheelOdometry => wheel_accel.update(forward_speed_m_s, DT_S as f32),
        };
        // The estimator runs on EVERY run, including a `PitchSource::
        // PlantTruth` acceptance run where the regulator is not listening to
        // it -- it is the only signal path a real board has, and an
        // acceptance trace that stopped recording it would stop being able to
        // show how big the bias criterion (f) is neutralising actually is.
        if lean_debug && ticks.is_multiple_of(250) {
            let f = sample.accel_m_s2;
            eprintln!(
                "IMU t={t_known_s:6.2} f=({:+.3},{:+.3},{:+.3}) aid={aiding:+.3} \
                 accel_pitch={:+.2}deg gyro_y={:+.4}",
                f[0], f[1], f[2],
                (f[0] - aiding).atan2((f[1] * f[1] + f[2] * f[2]).sqrt()).to_degrees(),
                sample.gyro_rad_s[1]
            );
        }
        let attitude = if cfg.lean_steer && std::env::var("OVERBOARD_TILT").as_deref() != Ok("0") {
            tilt_estimator.set_speed(last_forward_speed_m_s);
            tilt_estimator.update(std::slice::from_ref(&sample), aiding)
        } else {
            estimator.update(std::slice::from_ref(&sample), aiding)
        };

        // Ground truth, never fed to the controller on a DEPLOYED run
        // (DR-OBS-1) -- reported because "the board is actually up" is what
        // the state-out wire needs to prove, and truth proves it more
        // directly than the controller's own (estimator-mediated) belief.
        // Same formula `impulse-response-rust` / `sim/scenarios/
        // impulse_response.py::frame_pitch_rad` use, against the same
        // underlying xmat.
        //
        // **Read here rather than after the wire block, where it used to
        // live** (ADR-0011 criterion (f)): a `PitchSource::PlantTruth`
        // acceptance run needs it BEFORE the regulator, not after. Nothing
        // between the old and new positions touched the plant or `yaw_rad`
        // -- `backend.apply` only buffers a current for the next step -- so
        // the values are bit-identical to the ones the old ordering read.
        let xmat = backend.truth_frame_xmat();
        if cfg.lean_steer {
            // The plant turns itself: read the heading back, kept continuous.
            let h = crate::lean_steer::heading_from_xmat(&xmat);
            let d = (h - yaw_rad + std::f32::consts::PI).rem_euclid(2.0 * std::f32::consts::PI)
                - std::f32::consts::PI;
            yaw_rad += d;
        }
        // ATTITUDE MUST BE DE-YAWED BEFORE PITCH/ROLL COME OUT OF IT (issue
        // #163). Both readings below are `atan2` on the frame's world z-axis,
        // and that derivation assumes the world x/y axes still line up with
        // the body's -- true when the board had no yaw freedom at all, which
        // is exactly what the kinematic injection changed. `xmat` is now
        // `Rz(yaw) * R_body`; its third column mixes body pitch and body roll
        // by the heading angle, so after a 90 deg turn the raw formula would
        // report the board's ROLL as its PITCH, and `FALLEN` would fire on a
        // board that is upright. Removing the heading first (`Rz(-yaw) *
        // xmat`, applied to the two elements the formulas read) recovers the
        // body-frame values the ICD and the roll gate both mean.
        //
        // `yaw_rad` here is the heading currently baked into the plant -- this
        // tick's own increment is computed and injected at the BOTTOM of the
        // loop body, after this. At `yaw_rad == 0` (zero steer, all run) this
        // reduces to `1.0 * xmat[k] + 0.0 * xmat[j]`, i.e. bit-identically the
        // pre-#163 expressions.
        let (pitch_rad, roll_rad) = body_pitch_roll_rad(&xmat, yaw_rad);
        let truth_pitch_rate_rad_s = backend.truth_body_pitch_rate_rad_s();

        // --- ADR-0011 criterion (f): the estimator bias, removed ---------
        //
        // On a deployed run this is the `Estimator` arm and the regulator
        // sees exactly what it always saw. On an acceptance run it is
        // `PlantTruth`, and the regulator sees MuJoCo -- which on this plant
        // is the measured-WORSE case, because it deletes the apparent-vertical
        // lean the estimate carries rather than de-biasing it. See
        // `PitchSource`.
        let (source_pitch_rad, regulated_pitch_rate_rad_s) = match cfg.pitch_source {
            PitchSource::Estimator => (attitude.pitch_rad, attitude.pitch_rate_rad_s),
            PitchSource::PlantTruth => (pitch_rad, truth_pitch_rate_rad_s),
        };
        // Zero on any deployed run, so this whole line is a no-op there --
        // see HostConfig::pitch_bias_deg.
        let regulated_pitch_rad = source_pitch_rad + pitch_bias_rad;
        last_pitch_rad = regulated_pitch_rad;
        // Optional outer speed loop (`--speed-hold`): sets the pitch
        // reference from the speed error, as a Segway does. Off by default:
        // the deployed board leaves speed to the rider.
        // Rider warning (D1) and drive limit (D2), from LAST cycle's applied
        // current, the wheel speed and the learned grade load.
        if cfg.authority_margin != MarginMode::Off {
            let duty = (MARGIN_KE_V_S * wheel_rate_rad_s.abs() + MARGIN_R_PHASE_OHM * last_amps.abs())
                / MARGIN_PACK_V;
            let i_max = cfg.max_current_a.unwrap_or(MAX_CURRENT_A);
            let level = margin.update(
                last_amps,
                duty,
                grade_aid.load_m_s2(),
                ACCEL_FF_GAIN_M_S2_PER_A,
                i_max,
                DT_S as f32,
            );
            if level != margin_level {
                eprintln!(
                    "sim-host: rider warning {level:?} at sim_t={t_known_s:.3}s (margin {:.2}, \
                     {last_amps:+.1} A, duty {duty:.2}, grade load {:+.2} m/s^2)",
                    margin.margin(),
                    grade_aid.load_m_s2(),
                );
                margin_level = level;
                margin_level_since_s = t_known_s;
            }
            if cfg.authority_margin == MarginMode::Limit {
                speed_lqr.set_drive_scale(margin.drive_scale());
            }
            if cfg.rider_reacts {
                let held = t_known_s - margin_level_since_s >= RIDER_REACTION_S;
                if !rider_eased && level >= control_core::MarginLevel::Pulse && held {
                    rider_eased = true;
                    eprintln!("sim-host: rider eases off at sim_t={t_known_s:.3}s (target 0 m/s)");
                }
                // Braking at the limit (negative current) is not a reason to step off:
                // the rider leans back onto the tail, which is a safe brake.
                // An eased rider who has come to a stop on a climb with the warning
                // still on steps off: nobody balances in place on a 20 % hill. Without
                // this the model rocks the board at 0 m/s until the nose strikes.
                let stopped_on_climb = rider_eased
                    && last_forward_speed_m_s.abs() < 0.3
                    && level >= control_core::MarginLevel::Pulse
                    && last_amps > 0.0;
                if dismount_at_s.is_none()
                    && (stopped_on_climb || (level == control_core::MarginLevel::Solid && held && last_amps > 0.0))
                {
                    dismount_at_s = Some(t_known_s);
                    if cfg.tumble && backend.release_rider(false) {
                        motor_cut = true;
                        eprintln!("sim-host: --tumble: the rider steps off; the motor is cut");
                    }
                    eprintln!("sim-host: rider dismount at sim_t={t_known_s:.3}s");
                }
            }
        }
        if handoff_state.is_none() {
            battery.step(last_amps as f64, wheel_rate_rad_s as f64, hud_kt, hud_mass, DT_S);
        }
        if let Some(addr) = cfg.hud_out_addr {
            if ticks.is_multiple_of(10) {
                let pkt = crate::hud::HudOut {
                    flags: margin_level as u16,
                    seq: hud_seq,
                    t_s: t_known_s,
                    speed_m_s: wheel_rate_rad_s * DEFAULT_R_EFF_M,
                    current_a: last_amps,
                    torque_nm: (hud_kt as f32) * last_amps,
                    torque_limit_nm: (hud_kt as f32) * hud_i_limit,
                    batt_soc: battery.soc as f32,
                    batt_v: battery.v as f32,
                    batt_i_a: battery.i as f32,
                    margin: margin.margin(),
                };
                let _ = out_socket.send_to(&pkt.to_bytes(), addr);
                hud_seq += 1;
            }
        }
        let pitch_ref_rad = match cfg.speed_hold_m_s {
            Some(v_ref) => speed_loop.update(forward_speed_m_s, v_ref, DT_S as f32, last_saturated),
            None => 0.0,
        };
        let proposed_torque_nm =
            regulator.update(regulated_pitch_rad, regulated_pitch_rate_rad_s, pitch_ref_rad);
        // The single kt division -- the actuation boundary (issue #137).
        let mut proposed_amps = proposed_torque_nm / KT_NM_PER_A;
        // `--speed-hold` uses ONE full-state law for balance and speed (the
        // cascade above overshot 2-3 m/s at grade changes). The legacy
        // cascade stays only behind OVERBOARD_SPEED_LOOP for reproduction.
        if let Some(v_ref) = cfg.speed_hold_m_s {
            if std::env::var_os("OVERBOARD_SPEED_LOOP").is_none() {
                // Stand still until `--start-speed` is applied.
                let target = if start_speed_pending.is_some() || rider_eased { 0.0 } else { v_ref };
                proposed_amps = if speed_hold_baseline {
                    speed_lqr.update(
                        regulated_pitch_rad,
                        regulated_pitch_rate_rad_s,
                        forward_speed_m_s,
                        target,
                        DT_S as f32,
                        last_saturated,
                    )
                } else {
                    // The learned grade load (zero unless `--estimator-aiding
                    // grade-aware`) and the envelope limit: see
                    // `SpeedHoldLqr::update_with_grade`.
                    speed_lqr.update_with_grade(
                        regulated_pitch_rad,
                        regulated_pitch_rate_rad_s,
                        forward_speed_m_s,
                        target,
                        DT_S as f32,
                        last_saturated,
                        grade_aid.load_m_s2(),
                        cfg.max_current_a.unwrap_or(MAX_CURRENT_A),
                    )
                };
            }
        }
        // Grade feedforward: the current that holds the board on the learned
        // grade, so the proportional regulator does not have to droop
        // nose-up to make it (3.8 deg on an 8 % descent, measured). The same
        // idea as the VESC Float package's adaptive torque response.
        if cfg.grade_feedforward {
            proposed_amps += grade_aid.load_m_s2() / ACCEL_FF_GAIN_M_S2_PER_A;
        }
        // Pad mode (fable-oracle, 2026-10-04): the tail pad is on the road (a
        // tail drag or a tail stop). The PD law would ask 75-90 A against the
        // pad and drive the board over its tail; the grade estimate and the
        // integral would learn the pad's support and dump it nose-down at the
        // pull-away (the game: 3 of 3 tail stops ended in a fall). In pad mode
        // only damping acts, limited to 12 N*m; the grade load and the
        // integral are held at zero, and the grade load is not learned for
        // 1 s after. Pitch only, so firmware can use the same rule.
        // Not while the rider asks to go: that exit left the deck past 17 deg,
        // and pad mode chattered on and off every step for about 50 ms.
        if !pad_mode
            && regulated_pitch_rad >= PAD_MODE_ENTER_RAD
            && forward_speed_m_s.abs() < PAD_MODE_SPEED_M_S
            && corridor_enforced_fore_aft <= PAD_MODE_GO_STICK
        {
            pad_mode = true;
            eprintln!("sim-host: pad mode at sim_t={t_known_s:.3}s (tail pad down)");
        } else if pad_mode
            && (regulated_pitch_rad <= PAD_MODE_EXIT_RAD || corridor_enforced_fore_aft > PAD_MODE_GO_STICK)
        {
            pad_mode = false;
            pad_grade_hold_s = PAD_MODE_GRADE_HOLD_S;
            eprintln!("sim-host: pad mode ends at sim_t={t_known_s:.3}s -- full balance");
        }
        if pad_mode || pad_grade_hold_s > 0.0 {
            grade_aid.reset();
            grade_comp.reset();
            pad_grade_hold_s = (pad_grade_hold_s - DT_S).max(0.0);
        }
        // `--balance-comp`: grade compensation for the deployed law.
        if cfg.balance_comp && cfg.speed_hold_m_s.is_none() {
            proposed_amps += grade_comp.update(
                regulated_pitch_rad - pitch_ref_rad,
                grade_aid.load_m_s2(),
                ACCEL_FF_GAIN_M_S2_PER_A,
                KT_NM_PER_A,
                last_saturated,
                DT_S as f32,
            );
        }
        // WARNING (measured 2026-10-04): `--grade-ff` is physically wrong for
        // a balancing board and runs away (7-14 m/s on 6-12 % descents). A
        // sustained torque needs a matching centre-of-mass offset (rider
        // lean or board tilt); added current only makes the pitch loop fight
        // it. Kept only so the result can be reproduced; do not use it.
        // --- ADR-0011 criterion (c): the saturation bit STOPS being thrown
        // --- away here ---------------------------------------------------
        //
        // This binding used to read `let (bounded_cmd, _sat) = ...`. The
        // ADR names that discard by line number: the board's own "I have run
        // out of authority" signal existed, was computed every cycle, and was
        // dropped on the floor, while `FALLEN` -- which trips about a second
        // AFTER the outcome is decided -- was the only thing anyone was told.
        if motor_cut || !armed_seen {
            proposed_amps = 0.0;
        }
        if pad_mode {
            proposed_amps = (-KD_NM_PER_RAD_S * cfg.kd_scale * regulated_pitch_rate_rad_s)
                .clamp(-PAD_MODE_TORQUE_LIMIT_NM, PAD_MODE_TORQUE_LIMIT_NM)
                / KT_NM_PER_A;
        }
        // `--motor-limits`: what the motor and the pack can give at this speed.
        let mut limit_clamped = false;
        if cfg.motor_limits {
            const DUTY_MAX: f64 = 0.95;
            const R_EFF_OHM: f64 = 1.5 * 0.0525; // Superflux phase resistance, two phases conducting
            const CHARGE_LIMIT_A: f64 = 45.0;
            let w = wheel_rate_rad_s as f64;
            let i = proposed_amps as f64;
            let limited = if i * w > 0.0 {
                let room = ((DUTY_MAX * battery.v - hud_kt * w.abs()) / R_EFF_OHM).max(0.0);
                i.clamp(-room, room)
            } else if i * w < 0.0 {
                let room = CHARGE_LIMIT_A * battery.v / (hud_kt * w.abs()).max(1e-6);
                i.clamp(-room, room)
            } else {
                i
            };
            limit_clamped = limited != i;
            proposed_amps = limited as f32;
        }
        let (bounded_cmd, saturation) = envelope.apply(
            Command::MotorCurrent {
                amps: proposed_amps,
            },
            Faults::NONE,
        );
        let saturated = saturation == Saturation::Yes || limit_clamped;
        last_saturated = saturated;
        backend.apply(&bounded_cmd).map_err(HostError::Backend)?;
        // POST-envelope current, not the proposal -- the plant only ever
        // sees the clamped value (same reasoning `control-ffi`'s own
        // `ctl.last_amps` update carries).
        last_amps = match bounded_cmd {
            Command::MotorCurrent { amps } => amps,
            Command::RemoteSpeed { .. } => 0.0,
        };

        // Authority utilisation: how much of the actuator envelope the
        // controller is ASKING for, which is the pre-envelope demand and not
        // the post-envelope command -- the clamped value can never exceed
        // 1.0 and so can never warn about anything. Low-passed at
        // AUTHORITY_UTILISATION_TAU_S.
        let utilisation = proposed_amps.abs() / cfg.max_current_a.unwrap_or(MAX_CURRENT_A);
        utilisation_filtered += (utilisation - utilisation_filtered) * utilisation_alpha;
        // The discriminator is the SPEED (see AUTHORITY_UTILISATION_WARN):
        // saturation above the speed cap's onset is survivable, because the
        // cap is already unloading the board when it happens. Below it,
        // nothing is.
        let authority_warning = authority_warning_active(utilisation_filtered, forward_speed_m_s);
        if authority_warning && !prev_authority_warning {
            eprintln!(
                "sim-host: LOSS OF PITCH AUTHORITY IMMINENT at sim_t={t_known_s:.3}s -- \
                 filtered authority utilisation {:.0}% (demand {proposed_amps:+.1} A of \
                 {:.0} A) at {forward_speed_m_s:+.2} m/s, below the \
                 {SPEED_CAP_ONSET_M_S:.2} m/s speed-cap onset. Pitch {:+.1} deg.",
                utilisation_filtered * 100.0,
                cfg.max_current_a.unwrap_or(MAX_CURRENT_A),
                pitch_rad.to_degrees(),
            );
        } else if prev_authority_warning && !authority_warning {
            eprintln!(
                "sim-host: pitch authority recovered at sim_t={t_known_s:.3}s \
                 (filtered utilisation {:.0}%)",
                utilisation_filtered * 100.0,
            );
        }
        prev_authority_warning = authority_warning;

        // wheel_angle_rad: there is no absolute wheel-angle channel on `hal`
        // (ICD carries ERPM/tacho, the same as real VESC telemetry) -- this
        // dead-reckons it from the rate `hal` already reports, exactly the
        // way a real host would have to. Not a fabricated value: it is an
        // honest integral of an actually-measured rate.
        wheel_angle_rad += wheel_rate_rad_s * DT_S as f32;

        let pos_f64 = backend.truth_frame_xpos();
        truth_pos_x_m = pos_f64[0];
        truth_pos_y_m = pos_f64[1];
        // MuJoCo's own attitude, sent to the wire UNMODIFIED (issue #163).
        // The host used to compose a synthetic heading onto this quaternion
        // on its way out, because the plant had no yaw of its own; the
        // heading now lives inside the plant, so there is nothing left to
        // bolt on and the wire carries plain MuJoCo truth.
        let quat_f64 = backend.truth_frame_xquat();
        let quat = [
            quat_f64[0] as f32,
            quat_f64[1] as f32,
            quat_f64[2] as f32,
            quat_f64[3] as f32,
        ];

        // MuJoCo truth, straight through (issue #163). Nothing is
        // dead-reckoned any more: the board's heading is injected into the
        // plant before each step (see the bottom of this loop body), so
        // MuJoCo integrates the ground path itself, against its own contacts
        // and collision geometry, and this IS where the board is.
        let pos = [
            truth_pos_x_m as f32,
            truth_pos_y_m as f32,
            pos_f64[2] as f32,
        ];

        // Wire v2 (issue #161 follow-up): the ACTUAL ballast joint
        // positions -- CEO feedback was "there is no rider, and the turn
        // is not discernible", and a renderer cannot pose a rider from data
        // it does not have. NOT the commanded target (`weight_shift_*` *
        // BALLAST_RANGE_M`) -- the actuator is rate-limited
        // (`overboard_rider.xml`'s `timeconst`), so it lags a step change,
        // and sending the real joint value means that lag is visible
        // honestly rather than hidden behind an instantaneous command. Small
        // by construction (measured ~0.04 m at 0.8 stick) -- this host does
        // NOT amplify it for legibility; that is the renderer's job, as its
        // own declared non-physical channel, not this crate's to fake.
        let (rider_fore_aft_m, rider_lateral_m) = backend.truth_ballast_positions();

        let mut flags = wire::STATE_FLAG_ARMED | wire::STATE_FLAG_VALID;
        // With `--tail-brake`, a nose-up drag on the tail pad is a brake, not a
        // fall: the tail pad touches at about 20 deg, the same angle as this
        // limit, so every wanted tail stop used to set FALLEN.
        let tail_dragging = cfg.tail_brake && pitch_rad > 0.0 && backend.truth_tail_strike_n() > 0.0;
        let fallen = pitch_rad.abs() > FALLEN_PITCH_RAD && !tail_dragging;
        if fallen {
            flags |= wire::STATE_FLAG_FALLEN;
        }
        match margin_level {
            control_core::MarginLevel::Pulse => flags |= wire::STATE_FLAG_MARGIN_PULSE,
            control_core::MarginLevel::Solid => flags |= wire::STATE_FLAG_MARGIN_SOLID,
            control_core::MarginLevel::None => {}
        }

        // --- ADR-0012 PHYSICS-AUTHORITY HANDOFF -------------------------
        //
        // Only ever armed on a run that asked for a kerb. A run without one
        // is byte-for-byte the run it was before this existed: no strike can
        // be declared, and `STATE_FLAG_HANDOFF` never leaves this host.
        let lv = backend.truth_frame_linvel();
        let av = backend.truth_frame_angvel();
        let lin_vel = [lv[0] as f32, lv[1] as f32, lv[2] as f32];
        let ang_vel = [av[0] as f32, av[1] as f32, av[2] as f32];
        // `xmat[8]` is the frame up-axis's world Z, so acos of it is tilt from
        // vertical in ANY direction -- see HANDOFF_TILT_RAD on why pitch alone
        // is not enough. Computed on every run, kerb or not: it costs one
        // acos, it is the honest "how far over is the board" number, and
        // having it on non-kerb runs too is what let the threshold be set from
        // a measured carve envelope instead of a guess.
        let tilt_rad = (xmat[8].clamp(-1.0, 1.0) as f32).acos();
        let nose_n = backend.truth_nose_strike_n();
        let tail_n = backend.truth_tail_strike_n();
        // `--tail-brake`: the tail pad is a brake, not a terminating event.
        let strike_n = nose_n + if cfg.tail_brake { 0.0 } else { tail_n };
        // OB_HANDOFF_DEBUG=1 prints every 250 ticks (2 Hz); OB_HANDOFF_DEBUG=<n>
        // prints every n ticks, which is what makes a sub-second event like a
        // post-reset transient actually observable.
        let debug_every = std::env::var("OB_HANDOFF_DEBUG")
            .ok()
            .map(|v| v.parse::<u64>().unwrap_or(250).max(1));
        if debug_every.is_some_and(|n| ticks.is_multiple_of(n)) {
            eprintln!(
                "  [handoff-debug] t={t_known_s:.2} tilt={:.1}deg strike={strike_n:.1}N \
                 y={truth_pos_y_m:.2} amps={last_amps:+.1} est_pitch={:.2}deg \
                 truth_pitch={:.2}deg",
                tilt_rad.to_degrees(),
                attitude.pitch_rad.to_degrees(),
                pitch_rad.to_degrees()
            );
        }
        // Armed by EITHER an authored kerb or real terrain. Terrain was
        // missing from this condition at first, which meant the whole point of
        // loading City Park -- striking its real kerbs -- could not fire.
        if (cfg.kerb.is_some() || cfg.terrain.is_some() || cfg.grade_course.is_some()) && !handoff_latched {
            let by_strike = strike_n > STRIKE_FORCE_N;
            let by_tilt = tilt_rad > HANDOFF_TILT_RAD;
            if by_strike || by_tilt {
                handoff_latched = true;
                handoff_at_s = Some(t_known_s);
                if cfg.tumble && !motor_cut && backend.release_rider(true) {
                    rider_free = true;
                    motor_cut = true;
                    eprintln!("sim-host: --tumble: the rider comes off at t={t_known_s:.3}s; the motor is cut");
                }
                // The state handed over is the state at the instant of the
                // strike, captured BEFORE the freeze below stops it reaching
                // the wire -- everything sent from here on is this snapshot.
                handoff_state = Some(HandoffSnapshot {
                    pos,
                    quat,
                    lin_vel,
                    ang_vel,
                });
                eprintln!(
                    "sim-host: ADR-0012 handoff at t={:.3}s -- {} \
                     (bumper {strike_n:.0} N: nose {:.0} N, tail {:.0} N, tilt {:.1} deg); \
                     MuJoCo has stopped propagating, Unreal owns the board",
                    t_known_s,
                    if by_strike { "bumper strike" } else { "tilt" },
                    backend.truth_nose_strike_n(),
                    backend.truth_tail_strike_n(),
                    tilt_rad.to_degrees(),
                );
            }
        }

        if handoff_latched {
            flags |= wire::STATE_FLAG_HANDOFF;
        }

        if cfg.pose_out.is_some() && ticks.is_multiple_of(10) {
            // Before the rider is free, the rider pose is drawn at the ballast,
            // upright with the deck.
            let rider = if rider_free {
                backend.truth_body_pose("rider_free")
            } else {
                match (backend.truth_body_pose("ballast"), backend.truth_body_pose("frame")) {
                    (Some((p, _)), Some((_, q))) => Some((p, q)),
                    _ => None,
                }
            };
            let (rp, rq) = rider.unwrap_or(([0.0; 3], [1.0, 0.0, 0.0, 0.0]));
            let ev = (rider_free as u8) | ((dismount_at_s.is_some() as u8) << 1) | ((handoff_latched as u8) << 2);
            let mut row = format!(
                "{t_known_s:.4},{ev},{:.3},{:.4},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5}",
                forward_speed_m_s, last_amps, rp[0], rp[1], rp[2], rq[0], rq[1], rq[2], rq[3]
            );
            for x in backend.truth_qpos() {
                row.push_str(&format!(",{x:.6}"));
            }
            pose_rows.push(row);
        }
        if cfg.trace_path.is_some() {
            trace.push(TraceRow {
                seq: ticks,
                sim_time_s: t_known_s,
                stick_fore_aft,
                shaped_fore_aft: weight_shift_fore_aft,
                applied_fore_aft: corridor_enforced_fore_aft,
                truth_pitch_deg: pitch_rad.to_degrees(),
                est_pitch_deg: attitude.pitch_rad.to_degrees(),
                est_pitch_rate_deg_s: attitude.pitch_rate_rad_s.to_degrees(),
                truth_pitch_rate_deg_s: truth_pitch_rate_rad_s.to_degrees(),
                forward_speed_m_s,
                proposed_amps,
                applied_amps: last_amps,
                saturated,
                pos_x_m: truth_pos_x_m as f32,
                pos_y_m: truth_pos_y_m as f32,
                nose_strike_n: nose_n,
                tail_strike_n: tail_n,
                margin: margin.margin(),
                margin_level: margin_level as u8,
                truth_roll_deg: roll_rad.to_degrees(),
                utilisation,
                utilisation_filtered,
                authority_warning,
                fallen,
            });
        }

        // ADR-0012 freeze. While latched, the board's pose and velocity are
        // the snapshot taken on the strike cycle, repeated verbatim. MuJoCo
        // keeps stepping underneath -- stopping the integrator mid-run would
        // strand the control loop and the pacer -- but nothing it computes
        // after the strike reaches the wire, which is what "stops
        // propagating" has to mean for the client to be the sole authority.
        // `seq`, `sim_time_s` and the diagnostic channels keep moving so the
        // stream stays visibly alive rather than looking hung.
        let (pos, quat, lin_vel, ang_vel) = match handoff_state {
            Some(f) => (f.pos, f.quat, f.lin_vel, f.ang_vel),
            None => (pos, quat, lin_vel, ang_vel),
        };

        let state = StateOut {
            magic: wire::STATE_MAGIC,
            schema_version: wire::STATE_SCHEMA_VERSION,
            flags,
            seq: ticks,
            sim_time_s: obs.t_recv_ns as f64 * 1e-9,
            pos,
            quat,
            wheel_angle_rad,
            wheel_rate_rad_s,
            pitch_rad,
            yaw_rad,
            motor_current_a: obs.motor_current_a,
            rider_fore_aft_m,
            rider_lateral_m,
            lin_vel,
            ang_vel,
        };
        out_socket.send_to(&state.to_bytes(), cfg.state_out_addr)?;

        // --- KINEMATIC IN-PLANT YAW INJECTION (issue #163) --------------
        //
        // The steering law below is UNCHANGED -- same constants, same roll
        // shaping, same sign, same speed-proportional curvature. What changed
        // is where its output goes. It used to integrate a `yaw_rad` that the
        // physics never saw, and the host then dead-reckoned the ground path
        // and composed the heading onto the outgoing quaternion, because
        // MuJoCo's own board only ever translated along one axis. Both of
        // those are gone. The increment is now written straight into the
        // plant's free joint, immediately below, so the very next physics step
        // integrates the board's translation along the new heading itself.
        //
        // **The reframe that motivates it: the collision goal never needed yaw
        // to be physically GENERATED, only MuJoCo's pose to be
        // AUTHORITATIVE.** Those are different problems and the second is far
        // smaller. Steering is still commanded rather than emergent -- no tire
        // model produces this turn, and this file is still the only place the
        // heading comes from -- but position, ground path and every contact
        // downstream of it are now genuinely simulated at that heading, and
        // collision geometry in the MJCF would finally be tested against the
        // position the renderer actually draws. Nothing is dead-reckoned any
        // more. The MJCF is untouched, which is the entire point of doing it
        // this way.
        //
        // **REJECTED: making the wheel carve by geometry** (replacing the
        // cylinder with a sphere, ellipsoid or torus so a lean migrates the
        // contact patch and friction generates the turn). Recorded here
        // because it is the obvious idea and it is a trap on this plant:
        // the board is currently roll-stable ONLY because the wide cylinder
        // rim physically cannot tip (measured full-stick roll is 0.03 deg --
        // see ROLL_FULL_YAW_AUTHORITY_RAD). Any laterally-migrating contact
        // profile converts the board into a roll-axis inverted pendulum about
        // the contact point, and there is NO roll/lean controller in this
        // repo yet, so it would simply fall over sideways. That option is
        // gated on lean-steer; they are one epic, not two. Also, a torus mesh
        // fails silently rather than loudly: MuJoCo convex-hulls meshes, and
        // the convex hull of a torus is a rounded-rim disc.
        //
        // Placed here, at the END of the loop body, rather than at the top:
        // this way everything already sent on the wire above describes the
        // state as OBSERVED, and the injection is unambiguously the last
        // thing that happens before the next `wait_observe()` steps the
        // physics. `roll_rad` and `forward_speed_m_s` are this tick's own
        // fresh values, exactly as the pre-#163 law used.
        //
        // `roll_authority` scales the law's magnitude between
        // YAW_AUTHORITY_FLOOR (steer alone, however little the player is
        // leaning) and 1.0 (at/above ROLL_FULL_YAW_AUTHORITY_RAD's measured,
        // physically-achievable roll). Read BOTH constants' doc comments
        // before touching it -- on the current (widened) wheel geometry,
        // achievable roll is ~0.03 deg, so `steer` is effectively the primary
        // driver of yaw, NOT roll; the floor is what makes that honest
        // instead of a limiter that reads as "off".
        let roll_authority = YAW_AUTHORITY_FLOOR
            + (1.0 - YAW_AUTHORITY_FLOOR)
                * (roll_rad.abs() / ROLL_FULL_YAW_AUTHORITY_RAD).clamp(0.0, 1.0);
        // SIGN (issue #161 follow-up, CEO-reported bug: "turns me left when
        // I turn right"): increasing yaw_rad is a positive rotation about +Z,
        // which in this Z-up right-handed frame is counter-clockwise viewed
        // from above -- LEFT, by this model's own "right = +y" convention
        // (see the `imu` site comment in `overboard_rider.xml`). So positive
        // `steer` (stick-right) must DECREASE yaw_rad -- `-`, not `+`. See
        // `wire.rs`'s `InputIn::steer` doc comment for the wire-level
        // convention this implies.
        //
        // SPEED-PROPORTIONAL (issue #161 follow-up, CEO's own diagnosis: "you
        // can just straight up turn yourself around... too fast").
        // `YAW_CURVATURE_PER_STEER_RAD_PER_M`'s own doc comment has the full
        // reasoning; the short version is that yaw RATE scales with
        // `forward_speed_m_s`, so turn radius stops depending on speed (as a
        // real vehicle's does). At a standstill this is exactly zero, and the
        // gate below then makes the whole injection a literal no-op.
        let yaw_rate_rad_s = yaw_rate_rad_s(steer, forward_speed_m_s, roll_authority);
        let dyaw_rad = -yaw_rate_rad_s * DT_S as f32;
        // GATED ON EXACT ZERO, deliberately. With no steer (or no ground
        // speed) the plant must evolve bit-identically to the pre-#163 code:
        // this loop injects nothing, writes nothing, and the only remaining
        // difference on the wire is that `pos` reports MuJoCo's own x/y
        // instead of a reckoned one. `SimBackend::inject_kinematic_yaw`
        // re-checks the same condition; the gate is repeated here so that
        // `yaw_rad` and the plant can never disagree about whether a tick's
        // increment was applied.
        if cfg.lean_steer {
            lean_roll_rate_rad_s = (roll_rad - lean_roll_rad) / DT_S as f32;
            lean_roll_rad = roll_rad;
            let yaw_rate_meas = backend.truth_frame_angvel()[2] as f32;
            lean_yaw_torque_nm = tire_yaw.step(
                &lean_params,
                forward_speed_m_s,
                roll_rad,
                yaw_rate_meas,
                DT_S as f32,
            ) as f64;
        } else if dyaw_rad != 0.0 {
            yaw_rad += dyaw_rad;
            backend.inject_kinematic_yaw(dyaw_rad as f64);
        }

        ticks += 1;

        if let Some(path) = &cfg.stats_path {
            if last_stats_write.elapsed() >= Duration::from_millis(100) {
                write_stats(
                    path,
                    ticks,
                    pacer.missed_deadlines(),
                    pacer.jitter_percentiles(),
                    truth_pos_x_m,
                    truth_pos_y_m,
                );
                last_stats_write = Instant::now();
            }
        }

        // A free run has no deadlines to miss: the pacer is skipped entirely
        // rather than consulted and ignored, so its missed-deadline counter
        // stays at the truthful zero instead of reporting one miss per tick
        // for a mode in which "late" is not defined. See
        // `HostConfig::free_run`.
        if !cfg.free_run {
            let sleep_for = pacer.wait_for_next(Instant::now());
            if sleep_for.is_zero() {
                eprintln!(
                    "sim-host: missed deadline at tick {ticks} (total missed so far: {})",
                    pacer.missed_deadlines()
                );
            } else {
                // macOS coalesces timers: a 2 ms sleep can take 16 ms, and the
                // loop then runs the missed ticks back to back (issue #168),
                // so state packets leave in bursts. Sleep to 1 ms before the
                // deadline, then spin. Paced runs only; free runs never wait.
                let deadline = Instant::now() + sleep_for;
                if let Some(coarse) = sleep_for.checked_sub(SPIN_WINDOW) {
                    std::thread::sleep(coarse);
                }
                while Instant::now() < deadline {
                    std::hint::spin_loop();
                }
            }
        }
    }

    if let Some(path) = &cfg.stats_path {
        write_stats(
            path,
            ticks,
            pacer.missed_deadlines(),
            pacer.jitter_percentiles(),
            truth_pos_x_m,
            truth_pos_y_m,
        );
    }

    if cfg.lean_steer {
        eprintln!(
            "sim-host: lean-steer rider balance torque peak {lean_balance_peak_nm:.1} N*m \
             (limit {:.0} N*m)",
            lean_params.balance_torque_limit_nm
        );
    }
    if let Some(path) = &cfg.trace_path {
        write_trace(path, &trace)?;
        eprintln!(
            "sim-host: wrote {} trace rows to {}",
            trace.len(),
            path.display()
        );
    }

    if cfg.ankle_hinge || cfg.ankle_rigid {
        eprintln!("sim-host: ankle torque peak {ankle_peak_nm:.0} N*m");
    }
    if let Some(path) = &cfg.pose_out {
        let model_note = match &generated_model {
            Some(g) => {
                let stem = path.file_stem().and_then(|s| s.to_str()).unwrap_or("pose");
                let keep = rider_model_path().with_file_name(format!("{stem}.generated.xml"));
                std::fs::copy(g, &keep)?;
                std::fs::canonicalize(&keep)?.display().to_string()
            }
            None => std::fs::canonicalize(rider_model_path())?.display().to_string(),
        };
        let mut out = format!(
            "# model={model_note}\n# t,event(bit0 rider free, bit1 dismount, bit2 handoff),speed,amps,rider_x,rider_y,rider_z,rider_qw,rider_qx,rider_qy,rider_qz,qpos...\n"
        );
        for r in &pose_rows {
            out.push_str(r);
            out.push('\n');
        }
        std::fs::write(path, out)?;
        eprintln!("sim-host: wrote {} pose rows to {}", pose_rows.len(), path.display());
    }
    let _ = backend.close();
    // The spliced model is a per-process scratch file; leaving it behind
    // would litter `sim/models/` with generated XML that looks committed.
    if let Some(path) = &generated_model {
        let _ = std::fs::remove_file(path);
    }

    Ok(RunSummary {
        ticks,
        missed_deadlines: pacer.missed_deadlines(),
    })
}

/// Writes the acceptance-run trace ([`HostConfig::trace_path`]) as CSV, once,
/// after the loop has stopped.
///
/// Unlike [`write_stats`] this one is NOT best-effort: a trace is the
/// evidence an ADR-0011 acceptance number is quoted from, and a measurement
/// whose output silently failed to be written is worse than no measurement.
/// The error propagates.
///
/// Full `f32` precision (`{:?}`) on every physical column on purpose:
/// `wire-probe --csv`'s 6-decimal rounding is enough to draw a graph and not
/// enough to demonstrate that two runs are bit-identical, and this crate has
/// already been caught out once by exactly that difference (issue #163's own
/// equivalence work).
fn write_trace(path: &std::path::Path, rows: &[TraceRow]) -> Result<(), HostError> {
    use std::fmt::Write as _;
    let mut out = String::with_capacity(rows.len() * 160 + 256);
    out.push_str(
        "seq,sim_time_s,stick_fore_aft,shaped_fore_aft,applied_fore_aft,truth_pitch_deg,\
         est_pitch_deg,est_pitch_rate_deg_s,truth_pitch_rate_deg_s,forward_speed_m_s,proposed_amps,applied_amps,\
         saturated,utilisation,utilisation_filtered,authority_warning,fallen,pos_x_m,pos_y_m,\
         nose_strike_n,tail_strike_n,margin,margin_level,truth_roll_deg\n",
    );
    for r in rows {
        let _ = writeln!(
            out,
            "{},{:?},{:?},{:?},{:?},{:?},{:?},{:?},{:?},{:?},{:?},{:?},{},{:?},{:?},{},{},{:?},{:?},{:?},{:?},{:?},{},{:?}",
            r.seq,
            r.sim_time_s,
            r.stick_fore_aft,
            r.shaped_fore_aft,
            r.applied_fore_aft,
            r.truth_pitch_deg,
            r.est_pitch_deg,
            r.est_pitch_rate_deg_s,
            r.truth_pitch_rate_deg_s,
            r.forward_speed_m_s,
            r.proposed_amps,
            r.applied_amps,
            r.saturated as u8,
            r.utilisation,
            r.utilisation_filtered,
            r.authority_warning as u8,
            r.fallen as u8,
            r.pos_x_m,
            r.pos_y_m,
            r.nose_strike_n,
            r.tail_strike_n,
            r.margin,
            r.margin_level,
            r.truth_roll_deg,
        );
    }
    std::fs::write(path, out).map_err(HostError::Io)
}

/// Best-effort, atomic (write-then-rename) write of the host's own counters
/// AND true MuJoCo ground position, for `wire-probe` (or anyone else) to
/// pick up. Never allowed to interrupt the control loop -- a failure here is
/// silently swallowed ON PURPOSE (unlike a missed deadline or a malformed
/// input packet, which issue #161 requires surfacing loudly): this file is
/// internal tooling, not the wire. `truth_pos_x_m`/`truth_pos_y_m` are kept
/// here for continuity with the tooling that already reads them -- as of
/// issue #163 they are the SAME values the wire's `pos` now carries, since
/// the dead-reckoned path is gone and MuJoCo's own position is authoritative.
///
/// `jitter` (issue #168) is `pacer::JitterPercentiles` over the pacer's
/// recent window, not the whole run -- `missed_deadlines` stays the
/// whole-run count it always was. Reporting both is the point: "70% missed"
/// and "p99 = 15 ms with a correct mean" are the same underlying fact, and
/// only the second is actionable on its own.
fn write_stats(
    path: &std::path::Path,
    ticks: u64,
    missed_deadlines: u64,
    jitter: JitterPercentiles,
    truth_pos_x_m: f64,
    truth_pos_y_m: f64,
) {
    let tmp = path.with_extension("tmp");
    let contents = format!(
        "ticks={ticks}\nmissed_deadlines={missed_deadlines}\njitter_p50_ns={}\njitter_p99_ns={}\njitter_max_ns={}\ntruth_pos_x_m={truth_pos_x_m}\ntruth_pos_y_m={truth_pos_y_m}\n",
        jitter.p50_ns, jitter.p99_ns, jitter.max_ns,
    );
    let _ = std::fs::write(&tmp, contents).and_then(|_| std::fs::rename(&tmp, path));
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build `Rz(yaw) * Ry(pitch) * Rx(-roll)`, row-major, the same layout
    /// MuJoCo's `xmat` uses. This is the composition [`body_pitch_roll_rad`]
    /// has to invert.
    ///
    /// The `-roll` is not a typo. This crate's roll is measured about the
    /// frame's FORWARD axis, which is its local **-X** (see
    /// `overboard_onewheel.xml`: "FORWARD IS -X"), so a positive roll here is
    /// a negative rotation about +X. Building the reference matrix in the same
    /// convention the function under test reports keeps the sign flip in one
    /// place instead of scattering `-` through every assertion.
    fn xmat_from(yaw: f64, pitch: f64, roll: f64) -> [f64; 9] {
        let (sy, cy) = yaw.sin_cos();
        let (sp, cp) = pitch.sin_cos();
        let (sr, cr) = (-roll).sin_cos();
        [
            cy * cp,
            cy * sp * sr - sy * cr,
            cy * sp * cr + sy * sr,
            sy * cp,
            sy * sp * sr + cy * cr,
            sy * sp * cr - cy * sr,
            -sp,
            cp * sr,
            cp * cr,
        ]
    }

    /// Guard 1's unit-test half: with no heading injected, the de-yaw must be
    /// a LITERAL no-op -- not "close to", but the identical bits the pre-#163
    /// code produced. Zero steer has to leave every wire field it can reach
    /// untouched, and `1.0 * x + 0.0 * y` is only bit-safe as long as nobody
    /// "simplifies" the expression later.
    #[test]
    fn deyaw_at_zero_yaw_is_a_no_op() {
        for &(p, r) in &[(0.0, 0.0), (0.12, -0.03), (-0.31, 0.007), (0.0, 0.4)] {
            let xmat = xmat_from(0.0, p, r);
            let (pitch, roll) = body_pitch_roll_rad(&xmat, 0.0);
            // Bit-for-bit against the exact expressions this replaced.
            assert_eq!(pitch, (xmat[2] as f32).atan2(xmat[8] as f32));
            assert_eq!(roll, (xmat[5] as f32).atan2(xmat[8] as f32));
        }
    }

    /// THE regression the in-plant injection introduces if de-yawing is
    /// forgotten: a turned board reports its ROLL as its PITCH, so `FALLEN`
    /// fires on a board that is perfectly upright.
    ///
    /// Concretely: 12 deg of body roll, no body pitch, yawed 90 deg. The raw
    /// pre-#163 formula reads the frame's world z-axis in world x, which after
    /// a quarter turn IS the roll -- it would report ~12 deg of pitch. The
    /// de-yawed reading must report ~0.
    #[test]
    fn a_turned_board_does_not_report_its_roll_as_pitch() {
        let yaw = std::f64::consts::FRAC_PI_2;
        let roll = 12.0f64.to_radians();
        let xmat = xmat_from(yaw, 0.0, roll);

        let naive_pitch = (xmat[2] as f32).atan2(xmat[8] as f32);
        assert!(
            naive_pitch.abs() > 0.15,
            "this test proves nothing unless the naive formula is badly wrong here \
             (got {naive_pitch} rad)"
        );

        let (pitch, r) = body_pitch_roll_rad(&xmat, yaw as f32);
        assert!(
            pitch.abs() < 1e-5,
            "de-yawed pitch should be ~0 on an unpitched board, got {pitch} rad"
        );
        assert!(
            (r - roll as f32).abs() < 1e-5,
            "de-yawed roll should be the body roll ({roll} rad), got {r}"
        );
    }

    /// The de-yaw must recover both angles across a range of headings, not
    /// just the one the previous test happens to pick.
    #[test]
    fn deyaw_recovers_body_pitch_and_roll_at_any_heading() {
        let pitch = 0.09f64;
        let roll = 0.02f64;
        for &yaw in &[0.0f64, 0.4, -1.1, 2.6, -3.0, 5.9] {
            let xmat = xmat_from(yaw, pitch, roll);
            let (p, r) = body_pitch_roll_rad(&xmat, yaw as f32);
            assert!(
                (p - pitch as f32).abs() < 1e-4 && (r - roll as f32).abs() < 1e-4,
                "yaw={yaw}: recovered ({p}, {r}), wanted ({pitch}, {roll})"
            );
        }
    }

    // --- Issue #194: disturbance forces are world-frame, on purpose ------

    /// `select_disturbance_force_torque` takes no heading/yaw parameter --
    /// this is checked by the compiler on every call, but this test pins the
    /// consequence in the units the issue cares about: the same inputs
    /// produce the exact same force/torque no matter what heading the board
    /// is carrying "outside" the function. There is nothing to plumb a yaw
    /// through even if a future edit wanted to.
    #[test]
    fn disturbance_force_is_independent_of_heading_by_construction() {
        let d = Disturbance {
            t0_s: 0.0,
            duration_s: 1.0,
            force_n: [12.0, -3.0, 0.0],
            torque_nm: [0.0, 0.0, 4.0],
        };
        // "Heading" isn't even representable in this call -- the assertion
        // is that the same (bool, Option<Disturbance>, bool, bool) tuple is
        // the ENTIRE input, and it always returns the same output.
        let first = select_disturbance_force_torque(true, Some(d), false, false);
        let second = select_disturbance_force_torque(true, Some(d), false, false);
        assert_eq!(first, second);
        assert_eq!(first, (d.force_n, d.torque_nm));
    }

    /// Precedence when windows overlap, unchanged by the #194 extraction:
    /// scheduled disturbance beats the startup kick beats the on-demand fall
    /// kick beats nothing. A refactor-safety net, not new behaviour.
    #[test]
    fn disturbance_window_precedence_is_scheduled_then_startup_then_fall_then_none() {
        let d = Disturbance {
            t0_s: 0.0,
            duration_s: 1.0,
            force_n: [1.0, 0.0, 0.0],
            torque_nm: [0.0, 0.0, 0.0],
        };

        // Scheduled disturbance wins even if the kick windows are also open.
        assert_eq!(
            select_disturbance_force_torque(true, Some(d), true, true),
            (d.force_n, d.torque_nm)
        );
        // No scheduled disturbance: the startup kick wins over the fall kick.
        assert_eq!(
            select_disturbance_force_torque(false, None, true, true),
            (STARTUP_KICK_FORCE_N, [0.0; 3])
        );
        // Only the fall kick window open.
        assert_eq!(
            select_disturbance_force_torque(false, None, false, true),
            (FALL_KICK_FORCE_N, [0.0; 3])
        );
        // Nothing open: zero force, zero torque.
        assert_eq!(
            select_disturbance_force_torque(false, None, false, false),
            ([0.0; 3], [0.0; 3])
        );
    }

    // --- ADR-0011 criterion (b): the command-envelope reserve -----------

    /// The constant is DERIVED. This test is the derivation, executable:
    /// stated reserve fraction x envelope / measured slope. If somebody
    /// re-measures the slope and forgets to move the constant -- or moves the
    /// constant to make a scenario pass, which is the failure ADR-0011
    /// explicitly forbids -- this goes red.
    ///
    /// The shipped constant is the derived value rounded to two figures, and
    /// the tolerance here is half a rounding step. Two figures is not
    /// laziness: the measured slope's own per-run spread is 41.6-43.5 A/unit
    /// (see [`PEAK_DEMAND_A_PER_UNIT_STICK`]), so a constant quoted to three
    /// figures would be claiming a precision the measurement does not have.
    #[test]
    fn the_command_envelope_reserve_is_the_stated_reserve_divided_by_the_measured_slope() {
        let derived =
            STATED_ENVELOPE_RESERVE_FRACTION * MAX_CURRENT_A / PEAK_DEMAND_A_PER_UNIT_STICK;
        assert!(
            (CMD_ENVELOPE_RESERVE - derived).abs() <= 0.005,
            "the shipped constant ({CMD_ENVELOPE_RESERVE}) is not the derived value \
             ({derived}) rounded to two figures. It is DERIVED -- \
             {STATED_ENVELOPE_RESERVE_FRACTION} x {MAX_CURRENT_A} A / \
             {PEAK_DEMAND_A_PER_UNIT_STICK} A-per-unit -- and ADR-0011 forbids moving it \
             to make a scenario pass. Re-measure the slope and the constant follows; \
             the reverse is not allowed."
        );
    }

    /// The defect the reserve exists to remove, stated as arithmetic: at full
    /// stick the UNSHAPED command map asks for more current than the actuator
    /// has. ADR-0011 calls this a normalisation defect rather than a sizing
    /// gap, and this is what that sentence means numerically.
    #[test]
    fn full_stick_over_commands_the_envelope_without_the_reserve_and_fits_inside_it_with() {
        let unshaped = shape_fore_aft_command(1.0, 1.0) * PEAK_DEMAND_A_PER_UNIT_STICK;
        assert!(
            unshaped > MAX_CURRENT_A,
            "the premise of this whole change is that full stick over-commands: \
             {unshaped} A demanded of a {MAX_CURRENT_A} A envelope"
        );

        let shaped =
            shape_fore_aft_command(1.0, CMD_ENVELOPE_RESERVE) * PEAK_DEMAND_A_PER_UNIT_STICK;
        // The stated reserve plus the two-figure rounding step the constant
        // carries -- 33.62 A predicted here, 33.41 A actually measured in
        // sim. Asserting against the exact stated fraction would be asserting
        // that a rounded constant is unrounded.
        let bound = (STATED_ENVELOPE_RESERVE_FRACTION + 0.01) * MAX_CURRENT_A;
        assert!(
            shaped <= bound,
            "shaped full-stick demand {shaped} A exceeds the stated \
             {STATED_ENVELOPE_RESERVE_FRACTION} of the {MAX_CURRENT_A} A envelope \
             by more than the rounding step"
        );
        assert!(
            shaped < MAX_CURRENT_A,
            "the reserve has to leave SOME headroom at full stick, or it is not a reserve"
        );
    }

    /// The reserve is a linear scaling and NOT a clamp. A clamp would leave
    /// small stick inputs untouched and bite only near the rails, which is a
    /// different control feel and a different failure mode -- and it is the
    /// shape somebody reaches for when "cap the lean" is read casually.
    #[test]
    fn the_reserve_scales_the_whole_stick_range_and_is_symmetric() {
        for &u in &[0.0f32, 0.1, 0.25, 0.5, 0.75, 1.0] {
            let shaped = shape_fore_aft_command(u, CMD_ENVELOPE_RESERVE);
            assert_eq!(
                shaped,
                u * CMD_ENVELOPE_RESERVE,
                "u={u} is not scaled linearly"
            );
            assert_eq!(
                shape_fore_aft_command(-u, CMD_ENVELOPE_RESERVE),
                -shaped,
                "u={u}: at a given reserve the shaping must not depend on the SIGN of the \
                 stick; a one-sided cap is an asymmetric-authority bug that only shows up \
                 under a hard recovery in one direction"
            );
        }
    }

    /// The braking reserve is not a one-sided cap either -- the property the
    /// test above protects still holds, one level up.
    ///
    /// `shape_fore_aft_command_directional` is asymmetric in
    /// (stick, motion) TOGETHER, not in the sign of the stick: braking is a
    /// relationship between the command and the motion. So flipping both
    /// signs must give exactly the mirrored command, at every speed. A board
    /// that stopped harder going forwards than backwards would be the
    /// asymmetric-authority bug in a new place.
    #[test]
    fn the_braking_reserve_is_symmetric_under_flipping_stick_and_motion_together() {
        for &v in &[0.0f32, 0.1, 0.25, 0.3, 1.0, 5.0, 9.34, 12.0] {
            for &u in &[0.0f32, 0.1, 0.5, 1.0] {
                let forward = shape_fore_aft_command_directional(
                    u,
                    CMD_ENVELOPE_RESERVE,
                    CMD_ENVELOPE_RESERVE_BRAKING,
                    v,
                );
                let mirrored = shape_fore_aft_command_directional(
                    -u,
                    CMD_ENVELOPE_RESERVE,
                    CMD_ENVELOPE_RESERVE_BRAKING,
                    -v,
                );
                assert_eq!(
                    forward, -mirrored,
                    "u={u}, v={v}: braking is not mirror-symmetric"
                );
            }
        }
    }

    /// Below the speed gate NOTHING draws the braking reserve, whatever the
    /// signs say.
    ///
    /// This is the property that keeps ADR-0011's `full-stick` acceptance run
    /// -- the run the estimator trim and `PEAK_DEMAND_A_PER_UNIT_STICK` are
    /// both pinned against -- bit-identical under any braking reserve. The
    /// launch transient really does move the board backward under forward
    /// stick (measured: to -0.085 m/s for six tenths of a second), so without
    /// the gate that run would silently become a braking run.
    #[test]
    fn the_braking_reserve_is_not_spent_on_a_standing_start() {
        // Every speed here is inside the gate, and -0.085 is the measured
        // launch-transient extreme the gate exists to cover.
        for &v in &[0.0f32, -0.085, 0.085, -0.24, 0.24] {
            for &u in &[-1.0f32, -0.5, 0.5, 1.0] {
                assert_eq!(
                    shape_fore_aft_command_directional(
                        u,
                        CMD_ENVELOPE_RESERVE,
                        CMD_ENVELOPE_RESERVE_BRAKING,
                        v,
                    ),
                    shape_fore_aft_command(u, CMD_ENVELOPE_RESERVE),
                    "u={u} at {v} m/s drew the braking reserve below the gate"
                );
            }
        }
    }

    /// Opposing the motion above the gate DOES draw it, or the constant is
    /// inert. The companion to the test above: together they pin both sides
    /// of the gate rather than only the safe one.
    #[test]
    fn opposing_the_motion_above_the_gate_draws_the_braking_reserve() {
        for &v in &[0.3f32, 1.0, 5.0, 9.34] {
            assert_eq!(
                shape_fore_aft_command_directional(
                    -1.0,
                    CMD_ENVELOPE_RESERVE,
                    CMD_ENVELOPE_RESERVE_BRAKING,
                    v,
                ),
                -CMD_ENVELOPE_RESERVE_BRAKING,
                "full aft at {v} m/s forward did not draw the braking reserve"
            );
        }
    }

    /// Full stick still reaches the speed cap, so the reserve costs no top
    /// speed -- which is the entire reason ADR-0011 rejected lowering
    /// `MAX_GROUND_SPEED_M_S` as dominated. Stated here as the property that
    /// makes it true: a shaped full stick is still far enough above zero that
    /// the cap, not the command map, is what stops the board.
    #[test]
    fn a_shaped_full_stick_is_still_governed_by_the_speed_cap_not_by_the_command_map() {
        let shaped = shape_fore_aft_command(1.0, CMD_ENVELOPE_RESERVE);
        // Well below the cap, the cap does not touch it: the command map is
        // the only thing acting.
        assert_eq!(speed_capped_fore_aft(shaped, 0.0), shaped);
        // At the cap, the cap takes all of it, whatever the command map left.
        assert_eq!(speed_capped_fore_aft(shaped, MAX_GROUND_SPEED_M_S), 0.0);
    }

    // --- ADR-0011 criterion (a): why the reversal is the worst case ------

    /// The reversal entry of the matrix, as a property rather than a run:
    /// the speed cap NEVER attenuates a reversal, at any speed. This is what
    /// makes "full reverse-to-forward stick reversal at speed" the worst
    /// case -- the board is above the onset, where every surviving run in
    /// ADR-0011's data was being unloaded by the cap, and the cap is not
    /// acting.
    #[test]
    fn the_speed_cap_never_attenuates_a_stick_reversal() {
        for &v in &[1.0f32, 5.0, 8.34, 9.34, 12.0] {
            // Travelling forward, stick slammed back: full authority.
            assert_eq!(
                speed_capped_fore_aft(-1.0, v),
                -1.0,
                "braking from +{v} m/s was attenuated"
            );
            // ... and the mirror image.
            assert_eq!(
                speed_capped_fore_aft(1.0, -v),
                1.0,
                "braking from -{v} m/s was attenuated"
            );
        }
    }

    /// ... whereas an input that would accelerate the board further IS
    /// attenuated, all the way to zero at the cap. Pinned alongside the test
    /// above so the pair reads as the asymmetry it is.
    #[test]
    fn the_speed_cap_withdraws_accelerating_authority_over_the_last_margin() {
        assert_eq!(speed_capped_fore_aft(1.0, 0.0), 1.0);
        assert_eq!(speed_capped_fore_aft(1.0, SPEED_CAP_ONSET_M_S), 1.0);
        let half = speed_capped_fore_aft(1.0, SPEED_CAP_ONSET_M_S + 0.5 * SPEED_CAP_MARGIN_M_S);
        assert!(
            (half - 0.5).abs() < 1e-6,
            "the ramp should be linear across the margin, got {half}"
        );
        assert_eq!(speed_capped_fore_aft(1.0, MAX_GROUND_SPEED_M_S), 0.0);
        assert_eq!(speed_capped_fore_aft(1.0, MAX_GROUND_SPEED_M_S + 5.0), 0.0);
    }

    // --- ADR-0011 criterion (c): the loss-of-authority warning -----------

    /// The discriminator, both halves. Utilisation alone is not the trigger
    /// and speed alone is not the trigger; the warning is the conjunction,
    /// and dropping the speed half turns it into noise on every survivable
    /// high-speed saturation.
    #[test]
    fn the_authority_warning_fires_only_on_high_utilisation_below_the_speed_cap_onset() {
        let hot = AUTHORITY_UTILISATION_WARN + 0.05;
        let cold = AUTHORITY_UTILISATION_WARN - 0.05;
        assert!(authority_warning_active(hot, 2.0), "hot and slow must warn");
        assert!(
            authority_warning_active(hot, -2.0),
            "the discriminator is speed MAGNITUDE -- a board reversing at 2 m/s is as \
             unloaded as one driving at 2 m/s, which is to say not at all"
        );
        assert!(
            !authority_warning_active(hot, SPEED_CAP_ONSET_M_S + 0.1),
            "saturation above the onset is survivable and warning on it is noise"
        );
        assert!(
            !authority_warning_active(cold, 2.0),
            "utilisation below the threshold must not warn"
        );
        assert!(
            !authority_warning_active(AUTHORITY_UTILISATION_WARN, 2.0),
            "the trigger is strictly greater than the threshold"
        );
    }

    /// The filter has to be fast enough to preserve the lead time it exists
    /// to give and slow enough not to chatter on one noisy cycle. Both bounds
    /// asserted, because the constant is chosen by them rather than tuned.
    #[test]
    fn the_authority_filter_is_bounded_by_the_cycle_below_and_the_lead_time_above() {
        let cycle_s = DT_S as f32;
        assert!(
            AUTHORITY_UTILISATION_TAU_S > 10.0 * cycle_s,
            "a filter shorter than ~10 control cycles is not a filter"
        );
        // The lead this warning is quoted at, measured on the estimator path
        // with the reserve disabled -- see AUTHORITY_UTILISATION_WARN's doc
        // comment and `tests/test_cmd_envelope_reserve.py`.
        let measured_lead_s = 1.92_f32;
        assert!(
            AUTHORITY_UTILISATION_TAU_S < 0.2 * measured_lead_s,
            "a filter comparable to the lead time eats the warning it is meant to give"
        );
    }

    /// A first-order filter cannot overshoot its input, so the warning can
    /// never fire on a utilisation the controller never asked for. Cheap, and
    /// it is the property that makes the filtered signal safe to quote.
    #[test]
    fn the_filtered_utilisation_never_exceeds_a_constant_input() {
        let alpha = DT_S as f32 / (AUTHORITY_UTILISATION_TAU_S + DT_S as f32);
        let input = 0.9_f32;
        let mut y = 0.0_f32;
        for _ in 0..10_000 {
            y += (input - y) * alpha;
            assert!(y <= input, "filter overshot: {y} > {input}");
        }
        assert!(
            (y - input).abs() < 1e-3,
            "the filter must actually converge to its input, got {y}"
        );
    }

    /// The steering law's standstill property, which the speed-proportional
    /// curvature formulation exists to give: full stick at zero ground speed
    /// produces EXACTLY zero heading change, so the injection guarded by it is
    /// a literal no-op. You cannot carve a stationary onewheel.
    ///
    /// Through the SHIPPED `yaw_rate_rad_s` rather than a re-typed copy of the
    /// expression: the low-speed tightening below multiplies curvature up as
    /// speed falls, and a test that re-typed the formula could not tell you
    /// whether the `|v|` factor that makes this zero had survived it.
    #[test]
    fn full_steer_at_a_standstill_injects_nothing() {
        for &steer in &[1.0f32, -1.0, 0.6] {
            let dyaw = -yaw_rate_rad_s(steer, 0.0, 1.0) * DT_S as f32;
            assert_eq!(dyaw, 0.0, "steer={steer} at rest must not turn the board");
        }
    }

    /// The turn tightens as the board slows -- the CEO's "cut a tight turn by
    /// braking and turning at the same time".
    ///
    /// Asserted as the ORDERING plus the two endpoints, not as a table of
    /// radii: the magnitude is a feel number and pinning it would turn a
    /// tuning knob into a threshold, but the direction is the whole point and
    /// a sign error here would be invisible in play until someone tried to
    /// carve.
    #[test]
    fn the_turn_radius_shrinks_as_the_board_slows() {
        let radius = |v: f32| 1.0 / (yaw_curvature_per_steer(v) * 1.0);
        let top = radius(YAW_TIGHTEN_REF_SPEED_M_S);
        let rest = radius(0.0);
        assert!(
            (top - 1.0 / YAW_CURVATURE_PER_STEER_RAD_PER_M).abs() < 1e-4,
            "at and above the reference speed the turn must be the width it always was, \
             or this change is a general steering retune wearing a low-speed label"
        );
        assert!(
            (rest - top / YAW_LOW_SPEED_TIGHTEN).abs() < 1e-4,
            "the standstill radius must be exactly the tighten factor, got {rest} vs {top}"
        );
        // Monotone all the way down, with no step. Walked from the reference
        // speed DOWN to rest, which is the direction the property is stated
        // in: each slower sample must turn at least as tightly as the one
        // before it.
        let mut previous = f32::INFINITY;
        for i in (0..=20).rev() {
            let v = YAW_TIGHTEN_REF_SPEED_M_S * (i as f32) / 20.0;
            let r = radius(v);
            assert!(r <= previous + 1e-6, "radius grew as speed fell at v={v}");
            previous = r;
        }
        // Above the reference speed it stays at its widest rather than
        // inverting into an ever-wider turn.
        assert!((radius(YAW_TIGHTEN_REF_SPEED_M_S * 2.0) - top).abs() < 1e-4);
    }

    /// Tightening the low-speed turn must not resurrect the spin-in-place the
    /// speed-proportional formulation was introduced to kill. Yaw RATE must
    /// still fall to zero with speed, even though curvature is rising.
    #[test]
    fn yaw_rate_still_falls_to_zero_with_speed_despite_the_tightening() {
        let mut previous = 0.0f32;
        for i in 0..=20 {
            let v = YAW_TIGHTEN_REF_SPEED_M_S * (i as f32) / 20.0;
            let rate = yaw_rate_rad_s(1.0, v, 1.0);
            assert!(
                rate >= previous - 1e-6,
                "yaw rate is not monotone in speed at v={v}"
            );
            previous = rate;
        }
        assert_eq!(yaw_rate_rad_s(1.0, 0.0, 1.0), 0.0);
    }
}
