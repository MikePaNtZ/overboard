//! Issue #202 (ADR-0011 exit criterion (e)) AC1: reset was verified once, by
//! hand, and the claim never became an automated fact. This test drives the
//! REAL path -- the wire's `INPUT_FLAG_KICK`/`INPUT_FLAG_RESET` bits through
//! `sim_host::host::run`'s actual socket loop, in-process via
//! [`sim_host::spawn`] -- the same mechanism `send-input`/`wire-probe` used
//! for the manual verification issue #202 reports, just no longer manual.
//!
//! Knocks the board over for real (the on-demand fall kick, issue #161
//! follow-up item 5 -- 400 N*s, well past anything recoverable), waits for
//! `STATE_FLAG_FALLEN` to actually be observed on the wire, sends RESET, and
//! asserts both halves of the claim the removed `host.rs` comment used to
//! make on faith: `fallen` clears, and the pose lands back within tolerance
//! of the very first state packet this test received (the model's own spawn
//! configuration, since nothing has driven the board before the kick).

use sim_host::wire::{
    InputIn, StateOut, INPUT_FLAG_KICK, INPUT_FLAG_RESET, INPUT_MAGIC, INPUT_SCHEMA_VERSION,
    STATE_FLAG_FALLEN,
};
use sim_host::HostConfig;
use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicU16, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

// Deliberately NOT the documented 127.0.0.1:9601/:9602 (`wire::STATE_OUT_ADDR`
// / `wire::INPUT_IN_ADDR`) -- `sim-host.rs`'s own doc comment warns those
// collide with a live dev capture session, learned the hard way once
// already. This test gets its own pair.
const TEST_STATE_OUT_ADDR: &str = "127.0.0.1:47601";
const TEST_INPUT_IN_ADDR: &str = "127.0.0.1:47602";

// Same 40 ms one-shot window `send-input.rs` uses for both bits: long enough
// that the rising edge is reliably sampled by a 500 Hz (2 ms tick) loop even
// under scheduler jitter, short enough that a held button does not look like
// a second press.
const FLAG_HOLD: Duration = Duration::from_millis(40);
const INPUT_SEND_PERIOD: Duration = Duration::from_millis(5);

fn send_input(socket: &UdpSocket, target: SocketAddr, seq: u64, flags: u16) {
    let pkt = InputIn {
        magic: INPUT_MAGIC,
        schema_version: INPUT_SCHEMA_VERSION,
        flags,
        seq,
        weight_shift_fore_aft: 0.0,
        weight_shift_lateral: 0.0,
        steer: 0.0,
    };
    let _ = socket.send_to(&pkt.to_bytes(), target);
}

/// Blocks until a well-formed `StateOut` packet arrives or `deadline` passes.
fn recv_state(socket: &UdpSocket, deadline: Instant) -> Option<StateOut> {
    loop {
        let remaining = deadline.checked_duration_since(Instant::now())?;
        socket
            .set_read_timeout(Some(remaining.min(Duration::from_millis(200))))
            .ok()?;
        let mut buf = [0u8; StateOut::WIRE_SIZE];
        match socket.recv(&mut buf) {
            Ok(n) if n == StateOut::WIRE_SIZE => {
                if let Ok(state) = StateOut::from_bytes(&buf) {
                    return Some(state);
                }
            }
            _ => {
                if Instant::now() >= deadline {
                    return None;
                }
            }
        }
    }
}

#[test]
fn fall_then_reset_recovers() {
    let state_out_addr: SocketAddr = TEST_STATE_OUT_ADDR.parse().unwrap();
    let input_in_addr: SocketAddr = TEST_INPUT_IN_ADDR.parse().unwrap();

    let cfg = HostConfig {
        state_out_addr,
        input_in_addr,
        duration: Some(Duration::from_secs(12)),
        stats_path: None,
        ..HostConfig::default()
    };
    let host_handle = sim_host::spawn(cfg);

    // We bind the address the host SENDS to; the host itself binds
    // `input_in_addr` (see `HostConfig::input_in_addr`'s doc comment).
    let recv_socket = UdpSocket::bind(state_out_addr)
        .expect("failed to bind the test's state-out receiver -- port already in use?");
    let send_socket =
        UdpSocket::bind("127.0.0.1:0").expect("failed to bind the test's input sender");

    let seq = Arc::new(AtomicU64::new(0));
    let flags = Arc::new(AtomicU16::new(0));
    let sender_stop = Arc::new(AtomicU16::new(0));
    let sender_handle = {
        let seq = Arc::clone(&seq);
        let flags = Arc::clone(&flags);
        let sender_stop = Arc::clone(&sender_stop);
        let send_socket = send_socket.try_clone().unwrap();
        std::thread::spawn(move || {
            while sender_stop.load(Ordering::Relaxed) == 0 {
                let n = seq.fetch_add(1, Ordering::Relaxed);
                let f = flags.load(Ordering::Relaxed);
                send_input(&send_socket, input_in_addr, n, f);
                std::thread::sleep(INPUT_SEND_PERIOD);
            }
        })
    };

    // Let the host come up and publish a settled first packet -- this is
    // "spawn", the baseline the post-reset pose is measured against. Nothing
    // has driven the board yet (flags is still 0), so this IS `qpos0`,
    // modulo the fractions of a second the board's own regulator has had to
    // hold it there.
    let startup_deadline = Instant::now() + Duration::from_secs(3);
    let spawn_state =
        recv_state(&recv_socket, startup_deadline).expect("host never sent a first state packet");
    let (spawn_pos, spawn_pitch) = (spawn_state.pos, spawn_state.pitch_rad);

    // --- KICK: knock it over for real ---------------------------------
    flags.store(INPUT_FLAG_KICK, Ordering::Relaxed);
    std::thread::sleep(FLAG_HOLD);
    flags.store(0, Ordering::Relaxed);

    let fallen_deadline = Instant::now() + Duration::from_secs(5);
    let mut fell = false;
    while let Some(state) = recv_state(&recv_socket, fallen_deadline) {
        let state_flags = state.flags;
        if state_flags & STATE_FLAG_FALLEN != 0 {
            fell = true;
            break;
        }
    }
    assert!(
        fell,
        "the fall kick (400 N*s, `host::FALL_KICK_FORCE_N`) never produced a \
         measured FALLEN state within 5s -- either the kick did not fire or \
         FALLEN_PITCH_RAD stopped tripping"
    );

    // --- RESET: stand it back up ---------------------------------------
    flags.store(INPUT_FLAG_RESET, Ordering::Relaxed);
    std::thread::sleep(FLAG_HOLD);
    flags.store(0, Ordering::Relaxed);

    let recovered_deadline = Instant::now() + Duration::from_secs(5);
    let mut recovered_state = None;
    while let Some(state) = recv_state(&recv_socket, recovered_deadline) {
        let state_flags = state.flags;
        if state_flags & STATE_FLAG_FALLEN == 0 {
            recovered_state = Some(state);
            break;
        }
    }

    sender_stop.store(1, Ordering::Relaxed);
    let _ = sender_handle.join();

    let recovered_state = recovered_state.expect(
        "RESET was sent but no post-reset state packet ever cleared \
         STATE_FLAG_FALLEN within 5s",
    );
    let (recovered_flags, recovered_pos, recovered_pitch) = (
        recovered_state.flags,
        recovered_state.pos,
        recovered_state.pitch_rad,
    );
    assert_eq!(
        recovered_flags & STATE_FLAG_FALLEN,
        0,
        "sanity: the loop above already filtered on this"
    );

    // Pose within tolerance of spawn. Generous on purpose: this measures
    // "did reset put the board back", not the regulator's settled precision
    // -- `test_cmd_envelope_reserve.py` and friends already own that.
    const POS_TOL_M: f32 = 0.1;
    const PITCH_TOL_RAD: f32 = 3.0 * std::f32::consts::PI / 180.0;
    for axis in 0..3 {
        let d = (recovered_pos[axis] - spawn_pos[axis]).abs();
        assert!(
            d < POS_TOL_M,
            "post-reset pos[{axis}] = {:?} is {d:.3} m from spawn pos[{axis}] = {:?}, \
             outside the {POS_TOL_M} m tolerance",
            recovered_pos[axis],
            spawn_pos[axis]
        );
    }
    let pitch_delta = (recovered_pitch - spawn_pitch).abs();
    assert!(
        pitch_delta < PITCH_TOL_RAD,
        "post-reset pitch_rad = {recovered_pitch} is {pitch_delta:.4} rad from spawn \
         pitch_rad = {spawn_pitch}, outside the {PITCH_TOL_RAD:.4} rad tolerance"
    );

    // Deliberately not joined: `host_handle` runs out its own
    // `HostConfig::duration` (a generous upper bound on this test's own
    // deadlines, not something worth waiting on) on its own thread, and
    // dropping the handle here does not stop it -- it just stops this test
    // from blocking on wall-clock time it has no more use for. The process
    // exiting at the end of the test binary reclaims it.
    drop(host_handle);
}
