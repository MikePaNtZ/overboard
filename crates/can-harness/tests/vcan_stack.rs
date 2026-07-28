//! Issue #52: prove the CAN stack -- transport, a simulated responder,
//! round-trip byte fidelity and timeout behaviour -- end to end against a
//! real Linux `vcan` interface, with zero hardware.
//!
//! Every test here goes through [`can_harness::vcan::up_or_skip`] first and
//! returns early with a printed `SKIP` line if `vcan` is unavailable (no
//! `CAP_NET_ADMIN`, no `vcan` kernel module) -- AC1's "skipped with a clear
//! message rather than failing." This crate runs as an ordinary member of
//! the existing `cargo test --workspace` step, which is unprivileged, so a
//! SKIP on every test is the expected, honest outcome there -- see this
//! issue's PR description for why turning that into a live run needs a
//! deliberate, separate CI change this PR does not make.
//!
//! Each test uses its own interface name so they can run concurrently
//! without colliding.
//!
//! Gated on `target_os = "linux"` (issue #62): `can_harness` compiles to a
//! no-op on other platforms, so this whole file would fail to compile
//! against it there. On a non-Linux `cargo test --workspace` this binary
//! builds with zero tests, which is the correct outcome, not a weakening --
//! `vcan`/`socketcan` do not exist to skip against off Linux.
#![cfg(target_os = "linux")]

use can_harness::responder::SimResponder;
use can_harness::vcan::up_or_skip;
use can_harness::{from_can_frame, to_can_frame};
use socketcan::Socket;
use std::time::{Duration, Instant};
use vesc_wire::RawFrame;

#[test]
fn vcan_lifecycle_is_self_managed() {
    let Some(fixture) = up_or_skip("vh52lifecyc") else {
        return;
    };
    // If we got this far, up() succeeded with no manual `ip link` step.
    // Opening a socket against the freshly-created interface proves it's
    // actually there, not just that create_vcan() returned Ok on paper.
    fixture.socket().expect("interface exists and is up");
    drop(fixture);
    // Teardown happened in Drop; a second bring-up under the same name
    // would fail if it hadn't, which the next test (different name) can't
    // catch, so re-create the same name here to prove the delete landed.
    let second = can_harness::vcan::VcanFixture::up("vh52lifecyc");
    assert!(
        second.is_ok(),
        "teardown of the first fixture did not actually remove the interface"
    );
}

#[test]
fn simulated_responder_answers_command_with_status() {
    let Some(fixture) = up_or_skip("vh52responder") else {
        return;
    };

    const COMMAND_ID: u32 = 0x100; // test-local placeholder, not a real VESC packet ID
    const STATUS_ID: u32 = 0x101;

    let _responder = SimResponder::spawn(fixture.name(), move |req| {
        if req.id != COMMAND_ID {
            return None;
        }
        Some(RawFrame {
            id: STATUS_ID,
            len: req.len,
            data: req.data,
        })
    })
    .expect("responder spawns against a live interface");

    let socket = fixture.socket().expect("socket opens");
    socket
        .set_read_timeout(Duration::from_secs(2))
        .expect("timeout sets");

    let command = RawFrame {
        id: COMMAND_ID,
        len: 2,
        data: [0xAB, 0xCD, 0, 0, 0, 0, 0, 0],
    };
    let frame = to_can_frame(command).expect("valid frame");
    socket.write_frame(&frame).expect("command sends");

    let reply = socket.read_frame().expect("responder answers in time");
    let socketcan::CanFrame::Data(reply) = reply else {
        panic!("expected a data frame back");
    };
    let reply = from_can_frame(&reply);
    assert_eq!(reply.id, STATUS_ID);
    assert_eq!(reply.bytes(), command.bytes());
}

/// AC3: encode -> vcan -> decode must return exactly what went in,
/// including sign and scaling.
///
/// `vesc_wire::decode_motor_current_a` / `vesc_tx::encode_set_current` are
/// still `NotYetImplemented` stubs (no hardware exists yet to verify a real
/// VESC byte layout against -- see their module docs), so this proves the
/// layer they will sit on: the transport itself does not corrupt a signed,
/// scaled value's bytes. The #12 IMU frame-reflection bug was exactly this
/// class of defect, just on a different bus.
#[test]
fn round_trip_preserves_sign_and_scaling() {
    let Some(fixture) = up_or_skip("vh52roundtrip") else {
        return;
    };

    // A test-local convention -- not vesc-wire's layout -- of two big-endian
    // i16 fields: a negative value at offset 0, a scaled positive value
    // (x10) at offset 2. Chosen to exercise sign (negative) and scaling
    // (divide-by-10) the same way a real current/duty-cycle field would.
    let raw_amps = -1234_i16; // would decode to -123.4 A at a /10 scale
    let raw_duty = 5678_i16; // would decode to 567.8 at a /10 scale

    let mut data = [0u8; 8];
    data[0..2].copy_from_slice(&raw_amps.to_be_bytes());
    data[2..4].copy_from_slice(&raw_duty.to_be_bytes());
    let sent = RawFrame {
        id: 0x102,
        len: 4,
        data,
    };

    // Two independent sockets, like two nodes on a real bus -- a socket does
    // not receive its own transmitted frames by default (CAN_RAW_RECV_OWN_MSGS
    // is off), so a single socket here would prove nothing about the
    // transport and everything about that unrelated default.
    let tx = fixture.socket().expect("tx socket opens");
    let rx = fixture.socket().expect("rx socket opens");
    rx.set_read_timeout(Duration::from_secs(2))
        .expect("timeout sets");

    let frame = to_can_frame(sent).expect("valid frame");
    tx.write_frame(&frame).expect("frame sends");

    let received = rx
        .read_frame()
        .expect("vcan carries the frame to the other socket");
    let socketcan::CanFrame::Data(received) = received else {
        panic!("expected a data frame back");
    };
    let received = from_can_frame(&received);

    assert_eq!(received, sent, "transport must not alter the frame at all");

    let decoded_amps = i16::from_be_bytes(received.data[0..2].try_into().unwrap());
    let decoded_duty = i16::from_be_bytes(received.data[2..4].try_into().unwrap());
    assert_eq!(decoded_amps, raw_amps, "sign must survive the round trip");
    assert_eq!(decoded_duty, raw_duty, "scaling field must survive intact");
    assert!(
        decoded_amps < 0 && decoded_duty > 0,
        "sanity: fixture actually exercises both signs"
    );
}

/// AC4: what the stack does when nothing answers. This is the failure that
/// matters at a bench and cannot be tested later on hardware without a
/// motor attached.
#[test]
fn silence_produces_a_bounded_timeout_not_a_hang() {
    let Some(fixture) = up_or_skip("vh52silence") else {
        return;
    };

    // Deliberately no SimResponder -- nobody answers.
    let socket = fixture.socket().expect("socket opens");
    let timeout = Duration::from_millis(300);

    let request = RawFrame {
        id: 0x100,
        len: 1,
        data: [0x01, 0, 0, 0, 0, 0, 0, 0],
    };
    let frame = to_can_frame(request).expect("valid frame");
    socket.write_frame(&frame).expect("request sends");

    let started = Instant::now();
    let result = socket.read_frame_timeout(timeout);
    let elapsed = started.elapsed();

    assert!(
        result.is_err(),
        "no responder is running; a frame arriving would mean the vcan bus echoed \
         the request back as if from a peer, which is not what this test sets up"
    );
    assert!(
        elapsed < timeout * 3,
        "timeout took {elapsed:?} for a {timeout:?} budget -- looks like a hang, not a timeout"
    );
}
