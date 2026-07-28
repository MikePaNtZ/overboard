//! VESC CAN wire format — constants and **decode only**.
//!
//! Deliberately decode-only: an encoder here would hand every crate that
//! depends on this one — including anything the ridden binary might
//! link — the raw materials to assemble an actuation frame. Encoders for
//! `SET_CURRENT` and `COMMAND_REMOTE` live in `vesc-tx`, which the ridden
//! binary must never depend on (ICD §6.3); this crate carries no such
//! restriction, because decoding a status frame grants no motion authority.
//!
//! **Byte layouts are not yet transcribed.** This crate exists now, ahead of
//! need, to prove the crate-exclusion boundary can hold (`crates/xtask`'s
//! gate, `crates/canary-ridden`'s positive control) before the first line of
//! real CAN code exists — Track C is procurement-gated, and there is no
//! hardware yet to validate a byte layout against. Filling in the real
//! packet IDs and field offsets from the ICD is separate, traceable work;
//! [`decode_motor_current_a`] reports [`DecodeError::NotYetImplemented`]
//! rather than a fabricated value in the meantime.
#![no_std]

/// One CAN frame as received from the bus: an arbitration ID and up to 8 data
/// bytes. The shared shape decode (here) and encode (`vesc-tx`) both operate
/// on; the two crates do not depend on each other.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RawFrame {
    pub id: u32,
    pub len: u8,
    pub data: [u8; 8],
}

impl RawFrame {
    /// The populated prefix of `data`. Mirrors
    /// `Observation::imu_samples`'s "only iterate the populated prefix" rule
    /// — the backing array is fixed-size, `len` says how much of it is real.
    pub fn bytes(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }
}

/// Why a frame could not be decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    /// The frame's length or arbitration ID does not match any packet type
    /// this crate knows about.
    Unrecognized,
    /// The packet type is real but its byte layout has not been transcribed
    /// from the ICD into this crate yet.
    NotYetImplemented,
}

/// Decode a VESC drive-status frame's motor current.
///
/// **Not yet implemented** — see the module docs. Returns
/// [`DecodeError::NotYetImplemented`] for every input rather than a value
/// nothing has verified.
pub fn decode_motor_current_a(_frame: RawFrame) -> Result<f32, DecodeError> {
    Err(DecodeError::NotYetImplemented)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn raw_frame_bytes_exposes_only_the_populated_prefix() {
        let mut data = [0u8; 8];
        data[0] = 0xAA;
        data[1] = 0xBB;
        let frame = RawFrame {
            id: 9,
            len: 2,
            data,
        };
        assert_eq!(frame.bytes(), &[0xAA, 0xBB]);
    }

    #[test]
    fn undecoded_layouts_report_not_yet_implemented_not_a_fabricated_value() {
        // The point of this crate existing before hardware does: it must
        // never invent a current reading it cannot back with a verified
        // layout.
        let frame = RawFrame {
            id: 0,
            len: 0,
            data: [0; 8],
        };
        assert_eq!(
            decode_motor_current_a(frame),
            Err(DecodeError::NotYetImplemented)
        );
    }
}
