//! Std-only CAN test infrastructure: a self-managed virtual CAN (`vcan`)
//! interface and a simulated VESC-shaped responder, so the CAN stack --
//! frame transport, the encode/decode seam, timeout handling -- can be
//! proven end to end with zero hardware (issue #52).
//!
//! Deliberately `std`, unlike `vesc-wire`/`vesc-tx`: sockets, threads and a
//! kernel network interface have no `no_std` story. Nothing in this crate is
//! linked by `board-app-ridden` or `board-app-driverless`; it exists only
//! under `cargo test`.
//!
//! **No real VESC packet IDs live here.** `vesc-wire::decode_motor_current_a`
//! and `vesc-tx::encode_set_current` are still `NotYetImplemented` stubs --
//! see their module docs for why a real byte layout has not been transcribed
//! (no hardware exists yet to verify one against, and this project has
//! already ruled that a plausible-but-wrong constant is worse than a blank).
//! This crate proves the transport those functions will eventually sit on
//! top of: [`RawFrame`] in, over a real Linux CAN socket, [`RawFrame`] out,
//! byte-for-byte, including the sign and scaling bits a fabricated constant
//! could not be trusted to get right anyway.

pub mod responder;
pub mod vcan;

use socketcan::{CanDataFrame, EmbeddedFrame, Frame as SocketCanFrame};
use vesc_wire::RawFrame;

/// Converts a [`RawFrame`] to the frame type `socketcan` sends on the wire.
///
/// `RawFrame::id`'s magnitude picks standard (11-bit) vs extended (29-bit)
/// automatically, the same convention `ip link`/`candump` use -- callers
/// don't choose a frame flavour, the id's range does.
pub fn to_can_frame(frame: RawFrame) -> Option<CanDataFrame> {
    CanDataFrame::from_raw_id(frame.id, frame.bytes())
}

/// Converts a received `socketcan` frame back to the shape `vesc-wire` and
/// `vesc-tx` share.
pub fn from_can_frame(frame: &CanDataFrame) -> RawFrame {
    let bytes = frame.data();
    let mut data = [0u8; 8];
    data[..bytes.len()].copy_from_slice(bytes);
    RawFrame {
        id: frame.raw_id(),
        len: bytes.len() as u8,
        data,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn raw_frame_survives_the_can_data_frame_shape_alone() {
        // Pure conversion, no vcan involved -- the transport round trip is
        // covered separately in tests/vcan_stack.rs, which needs a real
        // interface and may skip.
        let original = RawFrame {
            id: 0x123,
            len: 3,
            data: [0xDE, 0xAD, 0xBE, 0, 0, 0, 0, 0],
        };
        let can_frame = to_can_frame(original).expect("valid frame");
        assert_eq!(from_can_frame(&can_frame), original);
    }
}
