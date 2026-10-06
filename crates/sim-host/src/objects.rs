//! `--objects`: moving scripted objects (cars, pedestrians, cyclists) for a
//! game level. sim-host computes their motion in the MuJoCo world, splices
//! each as a mocap body, and streams their poses on a second UDP packet
//! (`--objects-out-addr`). The state-out wire (104 B) does not change; a
//! client that wants the objects also listens on the objects packet.
//!
//! The motion is pure kinematics, not physics: each object follows its
//! polyline at a constant speed, with optional pauses, and sits on the
//! terrain. The objects DO collide with the board (same collision bits as
//! `--obstacles`), so a car that crosses the board's path strikes it.

use serde::Deserialize;

/// `b"OBO1"` read little-endian.
pub const OBJECTS_MAGIC: u32 = u32::from_le_bytes(*b"OBO1");
pub const OBJECTS_SCHEMA_VERSION: u16 = 1;

/// The most objects one level may carry. A larger file is an error: the
/// packet's `count` is a `u16`, but the real limit is the splice cost and the
/// game's own budget, so this is kept small and explicit.
pub const MAX_OBJECTS: usize = 64;

/// One object's kind. The wire code (0, 1, 2) is what the packet carries; the
/// default size, colour and geom shape follow from it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// A box, 4.6 x 1.9 x 1.5 m (L x W x H).
    Car,
    /// A vertical capsule, radius 0.25 m, total height 1.75 m.
    Pedestrian,
    /// A box, 1.8 x 0.6 x 1.7 m.
    Cyclist,
}

impl Kind {
    /// The wire code the packet carries.
    pub fn wire(self) -> u8 {
        match self {
            Kind::Car => 0,
            Kind::Pedestrian => 1,
            Kind::Cyclist => 2,
        }
    }

    /// The default size: a box's full L x W x H, or a capsule's
    /// [diameter, diameter, total height].
    pub fn default_size(self) -> [f64; 3] {
        match self {
            Kind::Car => [4.6, 1.9, 1.5],
            Kind::Pedestrian => [0.5, 0.5, 1.75],
            Kind::Cyclist => [1.8, 0.6, 1.7],
        }
    }

    /// The visible colour, as a MuJoCo `rgba` string.
    pub fn rgba(self) -> &'static str {
        match self {
            Kind::Car => "0.2 0.35 0.7 1",
            Kind::Pedestrian => "0.85 0.6 0.4 1",
            Kind::Cyclist => "0.3 0.7 0.3 1",
        }
    }

    fn parse(s: &str) -> Result<Kind, String> {
        match s {
            "car" => Ok(Kind::Car),
            "pedestrian" => Ok(Kind::Pedestrian),
            "cyclist" => Ok(Kind::Cyclist),
            other => Err(format!(
                "unknown kind '{other}' (car, pedestrian or cyclist)"
            )),
        }
    }
}

/// One validated scripted object. The `path` is in the MuJoCo world frame
/// (x, y metres); `pauses` are sorted by increasing arc length.
#[derive(Debug, Clone, PartialEq)]
pub struct ScriptedObject {
    /// The id the packet carries, unique within a file.
    pub id: u16,
    pub kind: Kind,
    /// A box's full L x W x H, or a capsule's [diameter, diameter, height].
    pub size: [f64; 3],
    /// The polyline, at least two points.
    pub path: Vec<[f64; 2]>,
    /// A closed loop wraps back to the first point; an open path stops at the
    /// end.
    pub closed: bool,
    /// The travel speed, m/s.
    pub speed_mps: f64,
    /// The object sits at the path start until this time, s.
    pub t0_s: f64,
    /// `(arc length s_m, wait seconds)` pairs, in increasing s_m. The object
    /// waits `seconds` when it reaches arc length `s_m`. On an open path the
    /// pauses run once; on a closed loop each `s_m` is within one lap and the
    /// pauses repeat every lap.
    pub pauses: Vec<(f64, f64)>,
}

impl ScriptedObject {
    /// Half the object's height, m: the mocap centre sits this far above the
    /// terrain.
    pub fn half_height(&self) -> f64 {
        self.size[2] / 2.0
    }

    /// The object's geom name in the generated model.
    pub fn geom_name(&self) -> String {
        format!("carve_obj_{}_geom", self.id)
    }

    /// The object's mocap body name in the generated model.
    pub fn body_name(&self) -> String {
        format!("carve_obj_{}", self.id)
    }

    /// The one geom for this object, as a MuJoCo `<geom .../>`. The box or
    /// capsule is centred on the body origin; the body carries the pose.
    /// Collision bits 1 and 2 (`contype`/`conaffinity` = 3), same as
    /// `--obstacles`, so the board, wheel, pads and rider all hit it.
    pub fn geom_xml(&self) -> String {
        let bits = r#"contype="3" conaffinity="3" condim="3""#;
        let name = self.geom_name();
        let rgba = self.kind.rgba();
        match self.kind {
            Kind::Pedestrian => {
                // A vertical capsule: the radius is half the diameter, and the
                // straight part spans the height less one radius at each cap.
                let r = self.size[0] / 2.0;
                let half_cyl = (self.size[2] / 2.0 - r).max(0.0);
                format!(
                    r#"<geom name="{name}" type="capsule" fromto="0 0 {:.4} 0 0 {:.4}" size="{r:.4}" rgba="{rgba}" {bits}/>"#,
                    -half_cyl, half_cyl,
                )
            }
            _ => format!(
                r#"<geom name="{name}" type="box" size="{:.4} {:.4} {:.4}" rgba="{rgba}" {bits}/>"#,
                self.size[0] / 2.0,
                self.size[1] / 2.0,
                self.size[2] / 2.0,
            ),
        }
    }

    /// The number of segments: a closed loop adds the closing segment back to
    /// the first point.
    fn n_segments(&self) -> usize {
        if self.closed {
            self.path.len()
        } else {
            self.path.len() - 1
        }
    }

    /// The length of segment `i` (from point `i` to the next, wrapping on a
    /// closed loop).
    fn seg_len(&self, i: usize) -> f64 {
        let n = self.path.len();
        let a = self.path[i];
        let b = self.path[(i + 1) % n];
        ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt()
    }

    /// The total travelled length: the sum of the segments (the closing
    /// segment is included only for a closed loop).
    pub fn path_total_len(&self) -> f64 {
        (0..self.n_segments()).map(|i| self.seg_len(i)).sum()
    }

    /// The total pause time of one lap, s.
    fn pause_total_s(&self) -> f64 {
        self.pauses.iter().map(|p| p.1).sum()
    }

    /// The arc length reached after `remaining` seconds of travel from the
    /// path start, with the pauses applied in order. Pauses stop the clock at
    /// their arc length for their duration. The result can exceed the path
    /// length when `remaining` is large (`pose_at_arc` then clamps or wraps).
    fn arc_for_budget(&self, mut remaining: f64) -> f64 {
        let mut s = 0.0;
        for &(sp, dur) in &self.pauses {
            // The pauses are sorted, so `sp >= s` and the travel time is never
            // negative.
            let to_pause = (sp - s) / self.speed_mps;
            if remaining < to_pause {
                return s + remaining * self.speed_mps;
            }
            remaining -= to_pause;
            s = sp;
            if remaining < dur {
                return s;
            }
            remaining -= dur;
        }
        s + remaining * self.speed_mps
    }

    /// The arc length travelled by time `t`, s. Before `t0` it is 0.
    ///
    /// An OPEN path runs the pauses once, then stops at the end. A CLOSED
    /// loop repeats every lap: the pauses are arc lengths within one lap, the
    /// lap takes `L / speed + sum(pause seconds)`, and the object re-enters
    /// the lap at `tau = (t - t0) mod lap`, so a car stops at the same
    /// crosswalks each time round.
    pub fn arc_length_at(&self, t: f64) -> f64 {
        if self.speed_mps <= 0.0 {
            return 0.0;
        }
        let moving = t - self.t0_s;
        if moving <= 0.0 {
            return 0.0;
        }
        if self.closed {
            let l = self.path_total_len();
            if l <= 0.0 {
                return 0.0;
            }
            let lap = l / self.speed_mps + self.pause_total_s();
            if lap <= 0.0 {
                return 0.0;
            }
            self.arc_for_budget(moving.rem_euclid(lap))
        } else {
            self.arc_for_budget(moving)
        }
    }

    /// The ground-plane pose `(x, y, yaw)` at arc length `s`. The yaw is the
    /// heading of the current segment (the +X body axis points along travel).
    pub fn pose_at_arc(&self, mut s: f64) -> (f64, f64, f64) {
        let n = self.path.len();
        let nseg = self.n_segments();
        let total = self.path_total_len();
        if self.closed && total > 0.0 {
            s = s.rem_euclid(total);
        } else {
            s = s.clamp(0.0, total);
        }
        for i in 0..nseg {
            let l = self.seg_len(i);
            // The last segment keeps the remainder so the open end lands on the
            // final point rather than falling through the loop.
            if s <= l || i == nseg - 1 {
                let frac = if l > 0.0 {
                    (s / l).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let a = self.path[i];
                let b = self.path[(i + 1) % n];
                let x = a[0] + (b[0] - a[0]) * frac;
                let y = a[1] + (b[1] - a[1]) * frac;
                let yaw = (b[1] - a[1]).atan2(b[0] - a[0]);
                return (x, y, yaw);
            }
            s -= l;
        }
        // Unreachable for a valid path (>= 2 points); the start is the safe
        // fallback.
        let a = self.path[0];
        (a[0], a[1], 0.0)
    }

    /// The ground-plane pose `(x, y, yaw)` at time `t`, s.
    pub fn pose_at(&self, t: f64) -> (f64, f64, f64) {
        self.pose_at_arc(self.arc_length_at(t))
    }
}

/// The JSON file shape, before validation.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RawFile {
    objects: Vec<RawObject>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RawObject {
    id: u16,
    kind: String,
    #[serde(default)]
    size: Option<[f64; 3]>,
    path: Vec<[f64; 2]>,
    #[serde(default)]
    closed: bool,
    speed_mps: f64,
    #[serde(default)]
    t0_s: f64,
    #[serde(default)]
    pause: Vec<[f64; 2]>,
}

/// Parses and validates an objects.json file. Returns a clear error for a
/// malformed or out-of-range file.
pub fn parse_objects(json: &str) -> Result<Vec<ScriptedObject>, String> {
    let file: RawFile = serde_json::from_str(json).map_err(|e| format!("bad JSON: {e}"))?;
    if file.objects.len() > MAX_OBJECTS {
        return Err(format!(
            "{} objects, but at most {MAX_OBJECTS} are allowed",
            file.objects.len()
        ));
    }
    let mut out = Vec::with_capacity(file.objects.len());
    let mut seen_ids = Vec::new();
    for raw in file.objects {
        let where_ = format!("object id {}", raw.id);
        if seen_ids.contains(&raw.id) {
            return Err(format!("{where_}: duplicate id"));
        }
        seen_ids.push(raw.id);
        let kind = Kind::parse(&raw.kind).map_err(|e| format!("{where_}: {e}"))?;
        let size = raw.size.unwrap_or_else(|| kind.default_size());
        if !size.iter().all(|v| v.is_finite() && *v > 0.0) {
            return Err(format!("{where_}: size must be three positive numbers"));
        }
        if raw.path.len() < 2 {
            return Err(format!("{where_}: path needs at least 2 points"));
        }
        if !raw.path.iter().flatten().all(|v| v.is_finite()) {
            return Err(format!("{where_}: path has a non-finite coordinate"));
        }
        if !raw.speed_mps.is_finite() || raw.speed_mps < 0.0 {
            return Err(format!("{where_}: speed_mps must be >= 0"));
        }
        if !raw.t0_s.is_finite() {
            return Err(format!("{where_}: t0_s must be finite"));
        }
        let mut pauses = Vec::with_capacity(raw.pause.len());
        let mut last_s = f64::NEG_INFINITY;
        for p in &raw.pause {
            let (s_m, secs) = (p[0], p[1]);
            if !s_m.is_finite() || !secs.is_finite() || s_m < 0.0 || secs < 0.0 {
                return Err(format!("{where_}: pause needs [s_m >= 0, seconds >= 0]"));
            }
            if s_m < last_s {
                return Err(format!("{where_}: pauses must be in increasing s_m"));
            }
            last_s = s_m;
            pauses.push((s_m, secs));
        }
        let obj = ScriptedObject {
            id: raw.id,
            kind,
            size,
            path: raw.path,
            closed: raw.closed,
            speed_mps: raw.speed_mps,
            t0_s: raw.t0_s,
            pauses,
        };
        // On a closed loop the pauses repeat every lap, so each must sit
        // within one lap: 0 <= s_m < loop length.
        if obj.closed {
            let l = obj.path_total_len();
            if let Some(&(sp, _)) = obj.pauses.iter().find(|&&(sp, _)| sp >= l) {
                return Err(format!(
                    "{where_}: pause s_m {sp} is not inside the {l:.3} m loop"
                ));
            }
        }
        out.push(obj);
    }
    Ok(out)
}

/// One object's row in the packet.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ObjectState {
    pub id: u16,
    pub kind: u8,
    /// Bit 0: the object's geom is in contact with the board this tick.
    pub flags: u8,
    /// The object centre, world frame, m.
    pub pos: [f32; 3],
    /// The heading, rad.
    pub yaw: f32,
}

impl ObjectState {
    /// The per-object wire size, bytes.
    pub const WIRE_SIZE: usize = 20;
}

/// One objects packet, little-endian: a 24-byte header then one 20-byte row
/// per object.
#[derive(Debug, Clone, PartialEq)]
pub struct ObjectsOut {
    pub seq: u64,
    pub sim_time_s: f64,
    pub objects: Vec<ObjectState>,
}

impl ObjectsOut {
    /// The header size, bytes.
    pub const HEADER_SIZE: usize = 24;

    pub fn to_bytes(&self) -> Vec<u8> {
        let count = self.objects.len() as u16;
        let mut b = vec![0u8; Self::HEADER_SIZE + self.objects.len() * ObjectState::WIRE_SIZE];
        b[0..4].copy_from_slice(&OBJECTS_MAGIC.to_le_bytes());
        b[4..6].copy_from_slice(&OBJECTS_SCHEMA_VERSION.to_le_bytes());
        b[6..8].copy_from_slice(&count.to_le_bytes());
        b[8..16].copy_from_slice(&self.seq.to_le_bytes());
        b[16..24].copy_from_slice(&self.sim_time_s.to_le_bytes());
        for (k, o) in self.objects.iter().enumerate() {
            let p = Self::HEADER_SIZE + k * ObjectState::WIRE_SIZE;
            b[p..p + 2].copy_from_slice(&o.id.to_le_bytes());
            b[p + 2] = o.kind;
            b[p + 3] = o.flags;
            b[p + 4..p + 8].copy_from_slice(&o.pos[0].to_le_bytes());
            b[p + 8..p + 12].copy_from_slice(&o.pos[1].to_le_bytes());
            b[p + 12..p + 16].copy_from_slice(&o.pos[2].to_le_bytes());
            b[p + 16..p + 20].copy_from_slice(&o.yaw.to_le_bytes());
        }
        b
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn car_on_line() -> ScriptedObject {
        ScriptedObject {
            id: 1,
            kind: Kind::Car,
            size: Kind::Car.default_size(),
            path: vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]],
            closed: false,
            speed_mps: 2.0,
            t0_s: 0.0,
            pauses: vec![],
        }
    }

    #[test]
    fn before_t0_sits_at_the_start() {
        let mut o = car_on_line();
        o.t0_s = 5.0;
        let (x, y, yaw) = o.pose_at(1.0);
        assert_eq!((x, y), (0.0, 0.0));
        assert!(yaw.abs() < 1e-9, "yaw {yaw}"); // first segment points +X
    }

    #[test]
    fn mid_first_segment() {
        let o = car_on_line();
        // 2 m/s for 2 s = 4 m along the first (10 m, +X) segment.
        let (x, y, yaw) = o.pose_at(2.0);
        assert!((x - 4.0).abs() < 1e-9, "x {x}");
        assert!(y.abs() < 1e-9, "y {y}");
        assert!(yaw.abs() < 1e-9, "yaw {yaw}");
    }

    #[test]
    fn second_segment_yaw_turns_north() {
        let o = car_on_line();
        // 12 m: 10 m on segment 1, then 2 m up segment 2 (+Y).
        let (x, y, yaw) = o.pose_at(6.0);
        assert!((x - 10.0).abs() < 1e-9, "x {x}");
        assert!((y - 2.0).abs() < 1e-9, "y {y}");
        assert!(
            (yaw - std::f64::consts::FRAC_PI_2).abs() < 1e-9,
            "yaw {yaw}"
        );
    }

    #[test]
    fn open_path_stops_at_the_end() {
        let o = car_on_line();
        // Total length is 20 m; long after the end it holds the last point.
        let (x, y, _) = o.pose_at(1000.0);
        assert!((x - 10.0).abs() < 1e-9, "x {x}");
        assert!((y - 10.0).abs() < 1e-9, "y {y}");
    }

    #[test]
    fn pause_holds_position() {
        let mut o = car_on_line();
        o.pauses = vec![(4.0, 3.0)]; // wait 3 s at 4 m
                                     // Reaches 4 m at t = 2 s, then waits until t = 5 s.
        let (x, _, _) = o.pose_at(3.5);
        assert!((x - 4.0).abs() < 1e-9, "paused x {x}");
        // At t = 6 s it has moved 1 s past the pause: 4 m + 2 m = 6 m.
        let (x, _, _) = o.pose_at(6.0);
        assert!((x - 6.0).abs() < 1e-9, "post-pause x {x}");
    }

    #[test]
    fn closed_loop_wraps() {
        let o = ScriptedObject {
            closed: true,
            speed_mps: 1.0,
            path: vec![[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]],
            ..car_on_line()
        };
        // The loop is 16 m. 18 m wraps to 2 m: on segment 1, at (2, 0).
        let (x, y, _) = o.pose_at(18.0);
        assert!((x - 2.0).abs() < 1e-9, "x {x}");
        assert!(y.abs() < 1e-9, "y {y}");
    }

    /// A 16 m square loop at 1 m/s. Corners at 0, 4, 8, 12 m.
    fn square_loop() -> ScriptedObject {
        ScriptedObject {
            closed: true,
            speed_mps: 1.0,
            path: vec![[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]],
            ..car_on_line()
        }
    }

    fn assert_xy(o: &ScriptedObject, t: f64, ex: f64, ey: f64) {
        let (x, y, _) = o.pose_at(t);
        assert!(
            (x - ex).abs() < 1e-9 && (y - ey).abs() < 1e-9,
            "t {t}: ({x},{y}) != ({ex},{ey})"
        );
    }

    #[test]
    fn closed_loop_pause_repeats_every_lap() {
        let mut o = square_loop();
        o.pauses = vec![(4.0, 3.0)]; // 3 s stop at arc 4 m, the corner (4, 0)
                                     // The lap is 16 m / 1 m/s + 3 s = 19 s.
                                     // Lap 1: the stop runs t = 4..7 s, held at (4, 0).
        assert_xy(&o, 4.0, 4.0, 0.0); // pause start
        assert_xy(&o, 7.0, 4.0, 0.0); // pause end
        assert_xy(&o, 8.0, 4.0, 1.0); // moving again, 1 m up segment 1
                                      // Lap 3 starts at t = 38 s; the same stop runs t = 42..45 s.
        assert_xy(&o, 42.0, 4.0, 0.0); // pause start, lap 3
        assert_xy(&o, 45.0, 4.0, 0.0); // pause end, lap 3
        assert_xy(&o, 46.0, 4.0, 1.0); // lap 3 matches lap 1 after the stop
    }

    #[test]
    fn closed_loop_with_two_pauses() {
        let mut o = square_loop();
        o.pauses = vec![(4.0, 2.0), (12.0, 3.0)]; // stops at (4, 0) and (0, 4)
                                                  // The lap is 16 + 2 + 3 = 21 s.
        assert_xy(&o, 5.0, 4.0, 0.0); // inside the first stop (t = 4..6 s)
        assert_xy(&o, 16.0, 0.0, 4.0); // inside the second stop (t = 14..17 s)
        assert_xy(&o, 18.0, 0.0, 3.0); // 1 m past the second stop, down segment 3
                                       // The second lap repeats: t = 21 + 5 is the first stop again.
        assert_xy(&o, 26.0, 4.0, 0.0);
    }

    #[test]
    fn packet_layout_is_header_plus_rows() {
        let pkt = ObjectsOut {
            seq: 9,
            sim_time_s: 1.5,
            objects: vec![ObjectState {
                id: 7,
                kind: 2,
                flags: 1,
                pos: [1.0, 2.0, 3.0],
                yaw: 0.5,
            }],
        };
        let b = pkt.to_bytes();
        assert_eq!(b.len(), 24 + 20);
        assert_eq!(&b[0..4], b"OBO1");
        assert_eq!(u16::from_le_bytes(b[4..6].try_into().unwrap()), 1);
        assert_eq!(u16::from_le_bytes(b[6..8].try_into().unwrap()), 1); // count
        assert_eq!(u64::from_le_bytes(b[8..16].try_into().unwrap()), 9);
        assert_eq!(f64::from_le_bytes(b[16..24].try_into().unwrap()), 1.5);
        assert_eq!(u16::from_le_bytes(b[24..26].try_into().unwrap()), 7);
        assert_eq!(b[26], 2);
        assert_eq!(b[27], 1);
        assert_eq!(f32::from_le_bytes(b[28..32].try_into().unwrap()), 1.0);
        assert_eq!(f32::from_le_bytes(b[40..44].try_into().unwrap()), 0.5);
    }

    #[test]
    fn parse_rejects_too_short_a_path() {
        let json = r#"{"objects":[{"id":1,"kind":"car","path":[[0,0]],"speed_mps":1.0}]}"#;
        let e = parse_objects(json).unwrap_err();
        assert!(e.contains("at least 2 points"), "{e}");
    }

    #[test]
    fn parse_rejects_unknown_kind() {
        let json = r#"{"objects":[{"id":1,"kind":"boat","path":[[0,0],[1,1]],"speed_mps":1.0}]}"#;
        let e = parse_objects(json).unwrap_err();
        assert!(e.contains("unknown kind"), "{e}");
    }

    #[test]
    fn parse_rejects_out_of_order_pauses() {
        let json = r#"{"objects":[{"id":1,"kind":"car","path":[[0,0],[9,0]],"speed_mps":1.0,"pause":[[5,1],[2,1]]}]}"#;
        let e = parse_objects(json).unwrap_err();
        assert!(e.contains("increasing s_m"), "{e}");
    }

    #[test]
    fn parse_rejects_duplicate_ids() {
        let json = r#"{"objects":[
            {"id":1,"kind":"car","path":[[0,0],[1,0]],"speed_mps":1.0},
            {"id":1,"kind":"cyclist","path":[[0,0],[1,0]],"speed_mps":1.0}]}"#;
        let e = parse_objects(json).unwrap_err();
        assert!(e.contains("duplicate id"), "{e}");
    }

    #[test]
    fn parse_accepts_defaults_and_fills_size() {
        let json =
            r#"{"objects":[{"id":3,"kind":"pedestrian","path":[[0,0],[1,0]],"speed_mps":1.4}]}"#;
        let v = parse_objects(json).unwrap();
        assert_eq!(v.len(), 1);
        assert_eq!(v[0].size, Kind::Pedestrian.default_size());
        assert!(!v[0].closed);
    }
}
