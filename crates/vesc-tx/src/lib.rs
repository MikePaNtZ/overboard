//! VESC CAN wire format — **encoders only**. This is the crate `hal-actuate`
//! implementations depend on to build actuation frames, and therefore the
//! crate `board-app-ridden` must never reach.
//!
//! ICD §6.3's motion-authority boundary is only as strong as the set of
//! crates that can *build* an actuation frame — `remote_command_input()`
//! bypasses every VESC-side config the wire alone cannot see, so the ridden
//! binary's safety rests entirely on never linking anything that can produce
//! a `SET_CURRENT` or `COMMAND_REMOTE` frame in the first place (issue #1).
//! `crates/xtask`'s gate asserts this crate is unreachable from
//! `board-app-ridden`'s normal dependency edges; `crates/canary-ridden`
//! deliberately violates that so the gate has something to catch.
//!
//! **Byte layouts are not yet transcribed** — see `vesc-wire`'s module docs
//! for why. This crate exists now, ahead of the hardware that would let a
//! layout be verified, purely to prove the exclusion boundary before the
//! first line of real CAN TX code exists (Track C, procurement-gated).
#![no_std]

use vesc_wire::RawFrame;

/// Why a command could not be encoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EncodeError {
    /// The byte layout for this packet has not been transcribed from the ICD
    /// yet.
    NotYetImplemented,
}

/// Encode a `SET_CURRENT` frame — the D-RAW profile's raw-current setpoint.
///
/// **Not yet implemented** — see the module docs. Returns
/// [`EncodeError::NotYetImplemented`] rather than a frame nothing has
/// verified against real hardware.
pub fn encode_set_current(_amps: f32) -> Result<RawFrame, EncodeError> {
    Err(EncodeError::NotYetImplemented)
}

/// Encode a `COMMAND_REMOTE` frame — the D-REMOTE profile's Refloat setpoint
/// relay. This is the specific frame issue #1 names as bypassing
/// `inputtilt_remote_type` entirely, which is exactly why building one is
/// confined to this crate and this crate alone.
///
/// **Not yet implemented** — see the module docs.
pub fn encode_command_remote(_m_s: f32) -> Result<RawFrame, EncodeError> {
    Err(EncodeError::NotYetImplemented)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn set_current_reports_not_yet_implemented_not_a_fabricated_frame() {
        assert_eq!(encode_set_current(5.0), Err(EncodeError::NotYetImplemented));
    }

    #[test]
    fn command_remote_reports_not_yet_implemented_not_a_fabricated_frame() {
        assert_eq!(
            encode_command_remote(2.5),
            Err(EncodeError::NotYetImplemented)
        );
    }
}
