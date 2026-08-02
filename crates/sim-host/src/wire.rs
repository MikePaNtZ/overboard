//! The `sim-host` <-> Unreal wire, v1 -- fixed by the COO (issue #161). This
//! module is not a place to redesign the protocol: the Unreal side is being
//! built against this exact byte layout right now, so a field reorder or a
//! width change here breaks a client we cannot see.
//!
//! Both structs are declared `#[repr(C, packed)]` rather than plain
//! `#[repr(C)]` -- with the field order given, `StateOut` happens to need no
//! padding either way (every field already lands on its natural alignment),
//! but `InputIn` does not: a plain `#[repr(C)]` would insert 4 bytes of
//! trailing padding after `steer` to round the struct up to `seq`'s 8-byte
//! alignment. That padding is a Rust ABI artifact with no counterpart in the
//! table below, and there is no guarantee Unreal's side packs the same way --
//! `packed` makes the on-wire size exactly the sum of the fields, which is
//! the only layout this table can mean.
//!
//! Every access into a packed field is by value (never `&field`), which is
//! what `to_le_bytes()`/copy-out reads require anyway -- so nothing here
//! trips the packed-reference lints.

use std::net::SocketAddr;

/// `magic` for [`StateOut`] -- ASCII "OBW1".
pub const STATE_MAGIC: u32 = 0x4F42_5731;
/// `magic` for [`InputIn`] -- ASCII "OBI1".
pub const INPUT_MAGIC: u32 = 0x4F42_4931;
/// [`StateOut`]'s schema version -- bumped to 2 for the rider-position
/// fields (issue #161 follow-up). `sim-host` always sends 2; the renderer is
/// being told to accept 1 (treating it as "no rider data, rider neutral")
/// and 2, so a one-sided deploy in either direction cannot produce a dead
/// window. This crate has no v1 sender left to keep around, so
/// [`StateOut::from_bytes`] only accepts 2 -- see that method's doc comment.
pub const STATE_SCHEMA_VERSION: u16 = 2;
/// [`InputIn`]'s schema version. Unchanged by the v2 state-out bump --
/// `InputIn` is explicitly untouched (issue #161 wire v2 follow-up).
pub const INPUT_SCHEMA_VERSION: u16 = 1;

/// `StateOut::flags` bit 0.
pub const STATE_FLAG_ARMED: u16 = 1 << 0;
/// `StateOut::flags` bit 1.
pub const STATE_FLAG_VALID: u16 = 1 << 1;
/// `StateOut::flags` bit 2.
pub const STATE_FLAG_FALLEN: u16 = 1 << 2;

/// `InputIn::flags` bit 0.
pub const INPUT_FLAG_ARM: u16 = 1 << 0;
/// `InputIn::flags` bit 1.
pub const INPUT_FLAG_RESET: u16 = 1 << 1;

/// Host SENDS state here.
pub const STATE_OUT_ADDR: &str = "127.0.0.1:9601";
/// Host BINDS/LISTENS for input here.
pub const INPUT_IN_ADDR: &str = "127.0.0.1:9602";

/// One tick of board state, host -> Unreal (issue #161 wire, v2 as of the
/// rider-position follow-up). v1's fields are unchanged, in the same order;
/// v2 appends `rider_fore_aft_m`/`rider_lateral_m` at the end -- ADR-0010's
/// own "Consequences" section calls this out as the designed-for case
/// ("Adding a field is a version bump, which is cheap by construction").
#[repr(C, packed)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StateOut {
    pub magic: u32,
    pub schema_version: u16,
    pub flags: u16,
    pub seq: u64,
    pub sim_time_s: f64,
    /// Board body position, metres, Z-up right-handed, in `sim-host`'s own
    /// world frame. **Not** converted to Unreal's frame -- that conversion
    /// is deliberately the game side's job, not this host's (issue #161).
    ///
    /// **`x`/`y` are PARTIALLY SYNTHETIC as of issue #161/#169's follow-up,
    /// NOT raw MuJoCo truth** -- `crate::host` dead-reckons them from real
    /// forward ground speed projected along the synthetic `yaw_rad` heading,
    /// because MuJoCo itself never turns this plant (there is no lateral
    /// force in the model; `yaw_rad` is a rendering-only overlay). Sending
    /// literal MuJoCo x/y here would make the board spin in place while
    /// sliding along its original straight line on screen -- a car spinning
    /// out, not a board carving. See `crate::host::run`'s "PARTIALLY
    /// SYNTHETIC POSITION" comment for the full reasoning; true MuJoCo x/y
    /// stays reachable out-of-band via `crate::host::write_stats`'s
    /// `truth_pos_x_m`/`truth_pos_y_m`. This deviates from this field's
    /// original description in ADR-0010's wire table (which predates the
    /// widened-wheel roll-authority finding that made it necessary); per
    /// that ADR's own "Consequences" section, the code and this comment are
    /// authoritative over the table when they disagree.
    ///
    /// `z` is untouched: vertical position is real MuJoCo truth throughout.
    pub pos: [f32; 3],
    /// Board body orientation, raw MuJoCo, **w, x, y, z**.
    pub quat: [f32; 4],
    pub wheel_angle_rad: f32,
    /// Positive = forward.
    pub wheel_rate_rad_s: f32,
    /// Nose-up positive (ICD SS10.1).
    pub pitch_rad: f32,
    /// NON-PHYSICAL game steering channel -- see
    /// `crate::host::YAW_CURVATURE_PER_STEER_RAD_PER_M`'s doc comment for
    /// the current rate model. The simulated wheel is a cylinder and cannot
    /// physically carve; this is an integration of the `steer` input,
    /// present so the game side has a heading to render with.
    ///
    /// **Sign (issue #161 follow-up): positive `yaw_rad` turns the board
    /// LEFT** (a positive rotation about MuJoCo's +Z, which is
    /// counter-clockwise viewed from above in this Z-up right-handed
    /// frame). See [`InputIn::steer`]'s doc comment for the wire-level
    /// convention this implies for the INPUT side, which is the opposite
    /// sign -- positive `steer` (stick-right) turns the board RIGHT, i.e.
    /// DEcreases `yaw_rad`.
    pub yaw_rad: f32,
    pub motor_current_a: f32,
    /// **v2.** ACTUAL `ballast_fa` joint position, metres, signed -- NOT the
    /// commanded target. The ballast is rate-limited (`overboard_rider.xml`'s
    /// `timeconst`), so it lags a step change in `weight_shift_fore_aft`;
    /// sending the real joint position means that lag is visible to the
    /// renderer, honestly, rather than hidden behind an instantaneous
    /// commanded value. Small by construction -- +/-0.05 m range, ~0.04 m
    /// achieved at 0.8 stick (measured, issue #161 follow-up) -- and this
    /// crate does NOT amplify it for legibility; a renderer that wants a
    /// more visible pose does that itself, as its own declared non-physical
    /// channel, not one faked at the source.
    pub rider_fore_aft_m: f32,
    /// **v2.** ACTUAL `ballast_lat` joint position, metres, signed. Same
    /// status as `rider_fore_aft_m` in every respect but axis.
    pub rider_lateral_m: f32,
}

impl StateOut {
    /// The exact on-wire byte count -- also asserted at compile time below.
    pub const WIRE_SIZE: usize = 80;

    #[allow(clippy::too_many_arguments)]
    pub fn to_bytes(&self) -> [u8; Self::WIRE_SIZE] {
        let mut buf = [0u8; Self::WIRE_SIZE];
        let mut off = 0;
        macro_rules! put {
            ($val:expr) => {{
                let bytes = $val.to_le_bytes();
                buf[off..off + bytes.len()].copy_from_slice(&bytes);
                off += bytes.len();
            }};
        }
        put!(self.magic);
        put!(self.schema_version);
        put!(self.flags);
        put!(self.seq);
        put!(self.sim_time_s);
        for v in self.pos {
            put!(v);
        }
        for v in self.quat {
            put!(v);
        }
        put!(self.wheel_angle_rad);
        put!(self.wheel_rate_rad_s);
        put!(self.pitch_rad);
        put!(self.yaw_rad);
        put!(self.motor_current_a);
        put!(self.rider_fore_aft_m);
        put!(self.rider_lateral_m);
        debug_assert_eq!(off, Self::WIRE_SIZE);
        buf
    }

    /// Parses a received datagram. Fails loudly (returns [`WireError`])
    /// rather than misreading a mismatched magic/version as a float --
    /// per issue #161's explicit instruction. Accepts ONLY
    /// [`STATE_SCHEMA_VERSION`] (2): this crate has no v1 sender to stay
    /// compatible with (`sim-host` always sends 2 -- see that constant's doc
    /// comment) -- v1 tolerance is the renderer's job, in a different repo.
    pub fn from_bytes(buf: &[u8]) -> Result<Self, WireError> {
        if buf.len() != Self::WIRE_SIZE {
            return Err(WireError::WrongSize {
                expected: Self::WIRE_SIZE,
                got: buf.len(),
            });
        }
        let mut off = 0;
        macro_rules! take {
            ($ty:ty) => {{
                const N: usize = core::mem::size_of::<$ty>();
                let arr: [u8; N] = buf[off..off + N].try_into().unwrap();
                off += N;
                <$ty>::from_le_bytes(arr)
            }};
        }
        let magic = take!(u32);
        let schema_version = take!(u16);
        if magic != STATE_MAGIC {
            return Err(WireError::BadMagic {
                expected: STATE_MAGIC,
                got: magic,
            });
        }
        if schema_version != STATE_SCHEMA_VERSION {
            return Err(WireError::BadVersion {
                expected: STATE_SCHEMA_VERSION,
                got: schema_version,
            });
        }
        let flags = take!(u16);
        let seq = take!(u64);
        let sim_time_s = take!(f64);
        let pos = [take!(f32), take!(f32), take!(f32)];
        let quat = [take!(f32), take!(f32), take!(f32), take!(f32)];
        let wheel_angle_rad = take!(f32);
        let wheel_rate_rad_s = take!(f32);
        let pitch_rad = take!(f32);
        let yaw_rad = take!(f32);
        let motor_current_a = take!(f32);
        let rider_fore_aft_m = take!(f32);
        let rider_lateral_m = take!(f32);
        debug_assert_eq!(off, Self::WIRE_SIZE);
        Ok(StateOut {
            magic,
            schema_version,
            flags,
            seq,
            sim_time_s,
            pos,
            quat,
            wheel_angle_rad,
            wheel_rate_rad_s,
            pitch_rad,
            yaw_rad,
            motor_current_a,
            rider_fore_aft_m,
            rider_lateral_m,
        })
    }
}

/// One tick of pilot input, Unreal -> host (issue #161 wire v1).
#[repr(C, packed)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InputIn {
    pub magic: u32,
    pub schema_version: u16,
    pub flags: u16,
    pub seq: u64,
    /// Clamped to `[-1, 1]`.
    pub weight_shift_fore_aft: f32,
    /// Clamped to `[-1, 1]`.
    pub weight_shift_lateral: f32,
    /// Clamped to `[-1, 1]`. NON-PHYSICAL -- see [`StateOut::yaw_rad`].
    ///
    /// **Sign, established explicitly (issue #161 follow-up -- a semantic
    /// clarification to ADR-0010's wire table, which did not state one):
    /// positive `steer` means turn RIGHT, matching stick-right / D / →.**
    /// This was found INVERTED in the field ("turns me left when I turn
    /// right") -- the game side already maps stick-right to positive
    /// `steer` correctly; the bug was entirely in how the host turned that
    /// into `yaw_rad` (see `crate::host`'s yaw computation, fixed in the
    /// same change). Do not re-derive this from first principles again --
    /// state it here so the next person does not have to.
    pub steer: f32,
}

impl InputIn {
    pub const WIRE_SIZE: usize = 28;

    pub fn to_bytes(&self) -> [u8; Self::WIRE_SIZE] {
        let mut buf = [0u8; Self::WIRE_SIZE];
        let mut off = 0;
        macro_rules! put {
            ($val:expr) => {{
                let bytes = $val.to_le_bytes();
                buf[off..off + bytes.len()].copy_from_slice(&bytes);
                off += bytes.len();
            }};
        }
        put!(self.magic);
        put!(self.schema_version);
        put!(self.flags);
        put!(self.seq);
        put!(self.weight_shift_fore_aft);
        put!(self.weight_shift_lateral);
        put!(self.steer);
        debug_assert_eq!(off, Self::WIRE_SIZE);
        buf
    }

    /// Parses a received datagram. Fails loudly on a magic/version mismatch
    /// (issue #161) -- the input socket is non-blocking and best-effort, but
    /// "best effort" means dropping a bad packet, not misparsing its bytes
    /// as floats.
    pub fn from_bytes(buf: &[u8]) -> Result<Self, WireError> {
        if buf.len() != Self::WIRE_SIZE {
            return Err(WireError::WrongSize {
                expected: Self::WIRE_SIZE,
                got: buf.len(),
            });
        }
        let mut off = 0;
        macro_rules! take {
            ($ty:ty) => {{
                const N: usize = core::mem::size_of::<$ty>();
                let arr: [u8; N] = buf[off..off + N].try_into().unwrap();
                off += N;
                <$ty>::from_le_bytes(arr)
            }};
        }
        let magic = take!(u32);
        let schema_version = take!(u16);
        if magic != INPUT_MAGIC {
            return Err(WireError::BadMagic {
                expected: INPUT_MAGIC,
                got: magic,
            });
        }
        if schema_version != INPUT_SCHEMA_VERSION {
            return Err(WireError::BadVersion {
                expected: INPUT_SCHEMA_VERSION,
                got: schema_version,
            });
        }
        let flags = take!(u16);
        let seq = take!(u64);
        let weight_shift_fore_aft = take!(f32).clamp(-1.0, 1.0);
        let weight_shift_lateral = take!(f32).clamp(-1.0, 1.0);
        let steer = take!(f32).clamp(-1.0, 1.0);
        debug_assert_eq!(off, Self::WIRE_SIZE);
        Ok(InputIn {
            magic,
            schema_version,
            flags,
            seq,
            weight_shift_fore_aft,
            weight_shift_lateral,
            steer,
        })
    }
}

/// A dropped packet, and why -- logged, never silently swallowed (issue
/// #161: "fail loudly on magic or version mismatch").
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WireError {
    WrongSize { expected: usize, got: usize },
    BadMagic { expected: u32, got: u32 },
    BadVersion { expected: u16, got: u16 },
}

impl core::fmt::Display for WireError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            WireError::WrongSize { expected, got } => {
                write!(f, "wrong packet size: expected {expected}, got {got}")
            }
            WireError::BadMagic { expected, got } => {
                write!(f, "bad magic: expected {expected:#010x}, got {got:#010x}")
            }
            WireError::BadVersion { expected, got } => {
                write!(f, "bad schema_version: expected {expected}, got {got}")
            }
        }
    }
}

impl std::error::Error for WireError {}

/// Parses [`SocketAddr`]s for the fixed loopback endpoints. `unwrap()` is
/// fine here -- these are compile-time-constant literals, not user input.
pub fn state_out_addr() -> SocketAddr {
    STATE_OUT_ADDR.parse().unwrap()
}

pub fn input_in_addr() -> SocketAddr {
    INPUT_IN_ADDR.parse().unwrap()
}

// Compile-time size assertions (issue #161): a future field edit that
// silently changes either struct's layout must fail the build, not ship a
// wire mismatch to Unreal.
const _: () = assert!(core::mem::size_of::<StateOut>() == StateOut::WIRE_SIZE);
const _: () = assert!(core::mem::size_of::<InputIn>() == InputIn::WIRE_SIZE);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn state_out_is_exactly_80_bytes() {
        assert_eq!(core::mem::size_of::<StateOut>(), 80);
    }

    #[test]
    fn input_in_is_exactly_28_bytes() {
        assert_eq!(core::mem::size_of::<InputIn>(), 28);
    }

    fn sample_state() -> StateOut {
        StateOut {
            magic: STATE_MAGIC,
            schema_version: STATE_SCHEMA_VERSION,
            flags: STATE_FLAG_ARMED | STATE_FLAG_VALID,
            seq: 42,
            sim_time_s: 1.5,
            pos: [1.0, 2.0, 3.0],
            quat: [1.0, 0.0, 0.0, 0.0],
            wheel_angle_rad: 0.1,
            wheel_rate_rad_s: 0.2,
            pitch_rad: -0.05,
            yaw_rad: 0.3,
            motor_current_a: 4.5,
            rider_fore_aft_m: 0.021,
            rider_lateral_m: -0.013,
        }
    }

    #[test]
    fn state_out_round_trips() {
        let s = sample_state();
        let bytes = s.to_bytes();
        assert_eq!(bytes.len(), StateOut::WIRE_SIZE);
        let back = StateOut::from_bytes(&bytes).unwrap();
        assert_eq!(back, s);
    }

    #[test]
    fn state_out_rejects_bad_magic() {
        let mut bytes = sample_state().to_bytes();
        bytes[0] = 0; // corrupt magic's first byte
        assert!(matches!(
            StateOut::from_bytes(&bytes),
            Err(WireError::BadMagic { .. })
        ));
    }

    #[test]
    fn state_out_rejects_bad_version() {
        let mut bytes = sample_state().to_bytes();
        bytes[4] = 9; // schema_version low byte
        bytes[5] = 0;
        assert!(matches!(
            StateOut::from_bytes(&bytes),
            Err(WireError::BadVersion { .. })
        ));
    }

    #[test]
    fn state_out_rejects_wrong_size() {
        let bytes = [0u8; 10];
        assert_eq!(
            StateOut::from_bytes(&bytes),
            Err(WireError::WrongSize {
                expected: StateOut::WIRE_SIZE,
                got: 10
            })
        );
    }

    /// This crate always sends v2 (issue #161 follow-up); a genuine
    /// 72-byte v1 packet -- correct magic, `schema_version = 1`, the OLD
    /// (pre-rider-fields) size -- must still be refused, by size, before
    /// `schema_version` is even inspected. v1/v2 coexistence tolerance is
    /// the renderer's job, in a different repo, not `StateOut::from_bytes`'s.
    #[test]
    fn state_out_rejects_a_genuine_v1_sized_packet() {
        let mut v1_bytes = [0u8; 72];
        v1_bytes[0..4].copy_from_slice(&STATE_MAGIC.to_le_bytes());
        v1_bytes[4..6].copy_from_slice(&1u16.to_le_bytes()); // schema_version = 1
        assert_eq!(
            StateOut::from_bytes(&v1_bytes),
            Err(WireError::WrongSize {
                expected: StateOut::WIRE_SIZE,
                got: 72,
            })
        );
    }

    fn sample_input() -> InputIn {
        InputIn {
            magic: INPUT_MAGIC,
            schema_version: INPUT_SCHEMA_VERSION,
            flags: INPUT_FLAG_ARM,
            seq: 7,
            weight_shift_fore_aft: 0.5,
            weight_shift_lateral: -0.25,
            steer: 0.1,
        }
    }

    #[test]
    fn input_in_round_trips() {
        let i = sample_input();
        let bytes = i.to_bytes();
        assert_eq!(bytes.len(), InputIn::WIRE_SIZE);
        let back = InputIn::from_bytes(&bytes).unwrap();
        assert_eq!(back, i);
    }

    #[test]
    fn input_in_rejects_bad_magic() {
        let mut bytes = sample_input().to_bytes();
        bytes[0] = 0;
        assert!(matches!(
            InputIn::from_bytes(&bytes),
            Err(WireError::BadMagic { .. })
        ));
    }

    #[test]
    fn input_in_clamps_out_of_range_axes() {
        let mut i = sample_input();
        i.weight_shift_fore_aft = 5.0;
        i.weight_shift_lateral = -5.0;
        i.steer = 2.0;
        let bytes = i.to_bytes();
        let back = InputIn::from_bytes(&bytes).unwrap();
        // Copied to locals first -- `assert_eq!` takes its arguments by
        // reference internally, which is unaligned-reference UB straight off
        // a packed field (see wire.rs's module doc comment).
        let (fore_aft, lateral, steer) = (
            back.weight_shift_fore_aft,
            back.weight_shift_lateral,
            back.steer,
        );
        assert_eq!(fore_aft, 1.0);
        assert_eq!(lateral, -1.0);
        assert_eq!(steer, 1.0);
    }

    #[test]
    fn addrs_parse_to_the_documented_ports() {
        assert_eq!(state_out_addr().port(), 9601);
        assert_eq!(input_in_addr().port(), 9602);
    }
}
