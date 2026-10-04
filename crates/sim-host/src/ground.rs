//! Smooth wheel contact on a heightfield.
//!
//! MuJoCo's tire-on-heightfield contact chatters at every grid edge: 0-6
//! contacts that change as the tire rolls, so the vertical force swings by
//! several m/s^2 and the tire leaves the ground from about 3 m/s
//! (`sim/carve/contact_chatter.py`). The board's estimator turns that into a
//! pitch bias (2.6 deg on flat ground, about 7 deg on a 10 % climb).
//!
//! The fix keeps the heightfield for everything except the tire. A thin
//! mocap box (`wheel_ground`) collides with the tire only, and each cycle it
//! is put tangent to the road under the tire contact. The road and the board
//! then rotate for real, so nothing else changes: the bumpers still strike
//! the heightfield, and gravity stays vertical.
//!
//! Limit: the plate follows a slope fitted over +-`SLOPE_HALF_SPAN_M`, so a
//! step much shorter than that (a kerb face) becomes a short ramp for the
//! tire. The bumpers still hit the kerb. `--hfield-wheel-contact` restores
//! the old contact for kerb studies.

use std::path::Path;

/// Half span of the slope fit under the tire, m: two grid cells either side,
/// so one cell's triangle seam does not tilt the plate.
pub const SLOPE_HALF_SPAN_M: f64 = 0.10;
/// Half thickness of the plate box, m.
pub const PLATE_HALF_THICKNESS_M: f64 = 0.05;
/// Half length and half width of the plate, m. Wide enough for any tire
/// contact under a banked board.
pub const PLATE_HALF_SIZE_M: f64 = 1.0;

/// The heights of a `course.py` / City Park heightfield, in metres.
pub struct GroundSurface {
    heights: Vec<f32>,
    nrow: usize,
    ncol: usize,
    half_extent_m: f64,
    spacing_m: f64,
}

impl GroundSurface {
    /// Reads the binary that `--terrain` already loads: two little-endian
    /// i32 (nrow, ncol), then nrow * ncol f32 heights, row-major, row along
    /// +Y and column along +X, centred on the origin.
    pub fn from_hfield_bin(path: &Path, half_extent_m: f64) -> std::io::Result<Self> {
        let raw = std::fs::read(path)?;
        if raw.len() < 8 {
            return Err(std::io::Error::other("hfield binary has no header"));
        }
        let nrow = i32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]) as usize;
        let ncol = i32::from_le_bytes([raw[4], raw[5], raw[6], raw[7]]) as usize;
        if raw.len() != 8 + nrow * ncol * 4 || nrow < 2 || ncol < 2 {
            return Err(std::io::Error::other("hfield binary size does not match its header"));
        }
        let heights = raw[8..]
            .chunks_exact(4)
            .map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
            .collect();
        Ok(Self::from_heights(heights, nrow, ncol, half_extent_m))
    }

    pub fn from_heights(heights: Vec<f32>, nrow: usize, ncol: usize, half_extent_m: f64) -> Self {
        let spacing_m = 2.0 * half_extent_m / (ncol as f64 - 1.0);
        GroundSurface { heights, nrow, ncol, half_extent_m, spacing_m }
    }

    fn post(&self, row: usize, col: usize) -> f64 {
        self.heights[row * self.ncol + col] as f64
    }

    /// Bilinear height at world (x, y), clamped to the grid.
    pub fn height(&self, x: f64, y: f64) -> f64 {
        let fc = ((x + self.half_extent_m) / self.spacing_m).clamp(0.0, (self.ncol - 1) as f64);
        let fr = ((y + self.half_extent_m) / self.spacing_m).clamp(0.0, (self.nrow - 1) as f64);
        let (c0, r0) = ((fc as usize).min(self.ncol - 2), (fr as usize).min(self.nrow - 2));
        let (tc, tr) = (fc - c0 as f64, fr - r0 as f64);
        let a = self.post(r0, c0) * (1.0 - tc) + self.post(r0, c0 + 1) * tc;
        let b = self.post(r0 + 1, c0) * (1.0 - tc) + self.post(r0 + 1, c0 + 1) * tc;
        a * (1.0 - tr) + b * tr
    }

    /// Unit road normal at (x, y), from central differences over
    /// +-`SLOPE_HALF_SPAN_M`.
    pub fn normal(&self, x: f64, y: f64) -> [f64; 3] {
        let d = SLOPE_HALF_SPAN_M;
        let sx = (self.height(x + d, y) - self.height(x - d, y)) / (2.0 * d);
        let sy = (self.height(x, y + d) - self.height(x, y - d)) / (2.0 * d);
        let n = (sx * sx + sy * sy + 1.0).sqrt();
        [-sx / n, -sy / n, 1.0 / n]
    }

    /// Plate centre and quaternion (w, x, y, z) for a tire whose centre is
    /// at `wheel` and whose radius is `r_m`. The contact point sits one
    /// radius below the centre, along the road normal.
    pub fn plate_pose(&self, wheel: [f64; 3], r_m: f64) -> ([f64; 3], [f64; 4]) {
        let n0 = self.normal(wheel[0], wheel[1]);
        let (cx, cy) = (wheel[0] - r_m * n0[0], wheel[1] - r_m * n0[1]);
        let n = self.normal(cx, cy);
        let z = self.height(cx, cy);
        let pos = [
            cx - PLATE_HALF_THICKNESS_M * n[0],
            cy - PLATE_HALF_THICKNESS_M * n[1],
            z - PLATE_HALF_THICKNESS_M * n[2],
        ];
        // Shortest rotation from +Z to n: axis z x n = (-n_y, n_x, 0).
        let (ax, ay) = (-n[1], n[0]);
        let s = (ax * ax + ay * ay).sqrt();
        let quat = if s < 1e-12 {
            [1.0, 0.0, 0.0, 0.0]
        } else {
            let angle = n[2].clamp(-1.0, 1.0).acos();
            let (sh, ch) = (0.5 * angle).sin_cos();
            [ch, sh * ax / s, sh * ay / s, 0.0]
        };
        (pos, quat)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A grid with z = g * x (rising toward +X).
    fn tilted(g: f64) -> GroundSurface {
        let n = 41;
        let half = 1.0;
        let sp = 2.0 * half / (n as f64 - 1.0);
        let mut h = Vec::with_capacity(n * n);
        for _row in 0..n {
            for col in 0..n {
                h.push((g * (col as f64 * sp - half)) as f32);
            }
        }
        GroundSurface::from_heights(h, n, n, half)
    }

    #[test]
    fn flat_ground_gives_a_level_plate_one_half_thickness_down() {
        let s = tilted(0.0);
        let (p, q) = s.plate_pose([0.1, -0.2, 0.1454], 0.1454);
        assert!((p[2] + PLATE_HALF_THICKNESS_M).abs() < 1e-9, "{p:?}");
        assert_eq!(q, [1.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn plate_top_lies_on_a_tilted_road_and_matches_its_slope() {
        let g = 0.2;
        let s = tilted(g);
        let (p, q) = s.plate_pose([0.0, 0.0, 0.3], 0.1454);
        // Rotation about +Y by -atan(g) tilts +Z toward -X, as the normal does.
        let angle = 2.0 * q[0].acos();
        assert!((angle - g.atan()).abs() < 1e-6, "angle {angle}");
        assert!(q[2] < 0.0 && q[1].abs() < 1e-12);
        // The top face centre is on the road.
        let n = s.normal(0.0, 0.0);
        let top = [p[0] + PLATE_HALF_THICKNESS_M * n[0], p[2] + PLATE_HALF_THICKNESS_M * n[2]];
        assert!((top[1] - g * top[0]).abs() < 1e-6, "{top:?}");
    }

    #[test]
    fn height_interpolates_between_posts() {
        let s = tilted(0.5);
        assert!((s.height(0.025, 0.3) - 0.0125).abs() < 1e-6);
    }
}
