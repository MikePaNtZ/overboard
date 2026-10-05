//! HUD telemetry: an optional second UDP packet with what a rider's display
//! shows (speed, torque against its limit, battery, the rider warning). The
//! state-out wire (104 B) does not change; a client that wants a HUD also
//! listens on `--hud-out-addr`.
//!
//! The battery is the build's pack, the same model as `sim/carve/hud_fields.py`:
//! 20S2P Molicel P50B (10 Ah), a generic Li-ion open-circuit curve, 0.15 ohm,
//! the motor's copper loss, 15 W idle, 80 % regeneration efficiency, rolling
//! and air losses, and a 45 A charge-current limit. Typical values, not
//! measurements; this is an electrical display model, not board physics.

/// `b"OBHD"` read little-endian.
pub const HUD_MAGIC: u32 = u32::from_le_bytes(*b"OBHD");
pub const HUD_SCHEMA_VERSION: u16 = 1;

/// One HUD packet, little-endian, 56 bytes.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct HudOut {
    /// Bits 0-1: rider-warning level (0 none, 1 pulsed, 2 solid).
    pub flags: u16,
    pub seq: u64,
    pub t_s: f64,
    pub speed_m_s: f32,
    pub current_a: f32,
    pub torque_nm: f32,
    pub torque_limit_nm: f32,
    pub batt_soc: f32,
    pub batt_v: f32,
    pub batt_i_a: f32,
    pub margin: f32,
}

impl HudOut {
    pub const WIRE_SIZE: usize = 56;

    pub fn to_bytes(&self) -> [u8; Self::WIRE_SIZE] {
        let mut b = [0u8; Self::WIRE_SIZE];
        b[0..4].copy_from_slice(&HUD_MAGIC.to_le_bytes());
        b[4..6].copy_from_slice(&HUD_SCHEMA_VERSION.to_le_bytes());
        b[6..8].copy_from_slice(&self.flags.to_le_bytes());
        b[8..16].copy_from_slice(&self.seq.to_le_bytes());
        b[16..24].copy_from_slice(&self.t_s.to_le_bytes());
        let f = [
            self.speed_m_s,
            self.current_a,
            self.torque_nm,
            self.torque_limit_nm,
            self.batt_soc,
            self.batt_v,
            self.batt_i_a,
            self.margin,
        ];
        for (k, x) in f.iter().enumerate() {
            b[24 + 4 * k..28 + 4 * k].copy_from_slice(&x.to_le_bytes());
        }
        b
    }
}

/// The build's pack, stepped once per control cycle.
#[derive(Debug, Clone, Copy)]
pub struct BatteryModel {
    pub soc: f64,
    pub v: f64,
    pub i: f64,
}

impl BatteryModel {
    const SERIES: f64 = 20.0;
    const CAPACITY_AH: f64 = 10.0; // 2P x 5 Ah
    const R_PACK: f64 = 0.15;
    const R_PHASE: f64 = 0.0525;
    const P_IDLE: f64 = 15.0;
    const REGEN_EFF: f64 = 0.8;
    const REGEN_LIMIT_A: f64 = 45.0;
    const SOC_PTS: [f64; 9] = [0.0, 0.05, 0.1, 0.2, 0.4, 0.6, 0.8, 0.9, 1.0];
    const OCV_PTS: [f64; 9] = [3.00, 3.30, 3.45, 3.58, 3.72, 3.87, 4.02, 4.10, 4.20];

    pub fn new(soc0: f64) -> Self {
        let mut b = BatteryModel {
            soc: soc0.clamp(0.0, 1.0),
            v: 0.0,
            i: 0.0,
        };
        b.v = b.ocv();
        b
    }

    fn ocv(&self) -> f64 {
        let s = self.soc;
        let k = Self::SOC_PTS
            .iter()
            .rposition(|&p| p <= s)
            .unwrap_or(0)
            .min(7);
        let (s0, s1, v0, v1) = (
            Self::SOC_PTS[k],
            Self::SOC_PTS[k + 1],
            Self::OCV_PTS[k],
            Self::OCV_PTS[k + 1],
        );
        Self::SERIES * (v0 + (v1 - v0) * ((s - s0) / (s1 - s0)).clamp(0.0, 1.0))
    }

    /// One step: motor current `amps` (A), wheel rate (rad/s), plant Kt,
    /// rider + board mass (kg) for rolling and air losses, time step (s).
    pub fn step(&mut self, amps: f64, wheel_rate: f64, kt: f64, mass_kg: f64, dt: f64) {
        let v = wheel_rate * 0.146;
        let loss = (0.015 * mass_kg * 9.81 + 0.5 * 1.2 * 0.5 * v * v) * 0.146 / kt * v.signum();
        let i = amps + loss;
        let mut p = kt * i * wheel_rate + 1.5 * Self::R_PHASE * i * i + Self::P_IDLE;
        if p < 0.0 {
            p *= Self::REGEN_EFF;
        }
        let ocv = self.ocv();
        let disc = (ocv * ocv - 4.0 * Self::R_PACK * p).max(0.0);
        self.i = ((ocv - disc.sqrt()) / (2.0 * Self::R_PACK)).max(-Self::REGEN_LIMIT_A);
        self.v = ocv - Self::R_PACK * self.i;
        self.soc = (self.soc - self.i * dt / 3600.0 / Self::CAPACITY_AH).clamp(0.0, 1.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packet_is_56_bytes_with_magic_first() {
        let b = HudOut {
            seq: 7,
            batt_soc: 0.9,
            ..Default::default()
        }
        .to_bytes();
        assert_eq!(b.len(), 56);
        assert_eq!(&b[0..4], b"OBHD");
        assert_eq!(u64::from_le_bytes(b[8..16].try_into().unwrap()), 7);
        assert_eq!(f32::from_le_bytes(b[40..44].try_into().unwrap()), 0.9);
    }

    #[test]
    fn idle_pack_sits_at_its_open_circuit_voltage_and_drains_slowly() {
        let mut b = BatteryModel::new(0.9);
        for _ in 0..500 {
            b.step(0.0, 0.0, 0.658, 113.0, 0.002);
        }
        assert!((b.v - 82.0).abs() < 0.2, "v {}", b.v); // 20 x 4.10 V
        assert!(b.soc < 0.9 && b.soc > 0.8999, "soc {}", b.soc);
    }

    #[test]
    fn hard_regeneration_is_limited_to_45_a() {
        let mut b = BatteryModel::new(0.9);
        b.step(-150.0, 80.0, 0.658, 113.0, 0.002); // about 6 kW of regeneration
        assert!((b.i + 45.0).abs() < 1e-9, "i {}", b.i);
    }
}
