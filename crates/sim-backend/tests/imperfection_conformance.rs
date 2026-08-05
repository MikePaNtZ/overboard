//! Cross-language conformance (issue #129): `sim-backend`'s imperfection
//! chain must reproduce every DETERMINISTIC vector
//! `sim/scenarios/imperfections.py` generates, bit-for-bit.
//!
//! The generator is invoked here rather than a JSON fixture being committed,
//! per that module's own "NOT COMMITTED AS A FIXTURE, GENERATED" contract --
//! a committed fixture goes stale silently the next time a row changes; a
//! live generator call cannot.
//!
//! Stochastic rows (gyro/accel noise) are deliberately NOT checked against
//! these vectors -- see `sim/scenarios/imperfections.py`'s own conformance
//! section for why, and `imperfections::tests` in `src/imperfections.rs` for
//! where the distributional/reproducibility checks for those rows live
//! instead.

use sim_backend::imperfections::{
    ImperfectionProfile, ImperfectionState, STAGE0_CUTBACK, STAGE0_PLACEHOLDER,
};
use std::path::{Path, PathBuf};
use std::process::Command;

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn generate_vectors() -> serde_json::Value {
    let out = Command::new("python3")
        .args([
            "-m",
            "sim.scenarios.imperfections",
            "--emit-conformance-vectors",
            "-",
        ])
        .current_dir(repo_root())
        .output()
        .expect(
            "failed to run `python3 -m sim.scenarios.imperfections` -- is \
             python3 (with numpy) on PATH?",
        );
    assert!(
        out.status.success(),
        "conformance-vector generator failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr),
    );
    serde_json::from_slice(&out.stdout).expect("generator did not emit valid JSON")
}

fn profile_for(id: &str) -> ImperfectionProfile {
    match id {
        "stage0-placeholder-v1" => STAGE0_PLACEHOLDER,
        "stage0-cutback-v1" => STAGE0_CUTBACK,
        other => panic!(
            "unknown profile_id {other:?} in conformance vectors -- this crate's \
             imperfections module needs a matching constant added"
        ),
    }
}

fn f64_vec(v: &serde_json::Value) -> Vec<f64> {
    v.as_array()
        .expect("expected a JSON array")
        .iter()
        .map(|x| x.as_f64().expect("expected a JSON number"))
        .collect()
}

#[test]
fn deterministic_rows_match_the_python_generator_bit_for_bit() {
    let vectors = generate_vectors();

    assert_eq!(
        vectors["schema"], "imperfection-conformance-v1",
        "CONFORMANCE_SCHEMA changed on the Python side -- this test needs \
         updating for the new vector shape, not just re-running"
    );
    assert_eq!(
        vectors["chain_order"],
        serde_json::json!(["cutback", "saturate", "transport_delay", "current_loop_lag"]),
        "the chain order is part of the contract -- a Rust implementation \
         that clamps after the lag would let a command through as a transient"
    );

    let cases = vectors["cases"].as_array().expect("cases must be an array");
    assert!(!cases.is_empty(), "generator produced zero cases");

    for case in cases {
        let profile_id = case["profile_id"].as_str().unwrap();
        let profile = profile_for(profile_id);
        let dt_s = case["dt_s"].as_f64().unwrap();
        let model = case["model"].as_str().unwrap();
        let ctx = format!("profile={profile_id} model={model} dt_s={dt_s}");

        check_available_current(&profile, &case["available_current_a"], &ctx);
        check_apply_current(&profile, dt_s, &case["apply_current"], &ctx);
        check_wheel_rate(&profile, dt_s, &case["wheel_rate_quantisation"], &ctx);
        check_wheel_rate(&profile, dt_s, &case["wheel_rate_hold"], &ctx);
    }
}

fn check_available_current(profile: &ImperfectionProfile, case: &serde_json::Value, ctx: &str) {
    let ws = f64_vec(&case["wheel_rate_rad_s"]);
    let want = f64_vec(&case["cap_a"]);
    for (w, want) in ws.iter().zip(want.iter()) {
        let got = profile.available_current_a(*w);
        assert_eq!(got, *want, "available_current_a({w}) [{ctx}]");
    }
}

fn check_apply_current(
    profile: &ImperfectionProfile,
    dt_s: f64,
    case: &serde_json::Value,
    ctx: &str,
) {
    let commanded = f64_vec(&case["commanded_a"]);
    let wheel = f64_vec(&case["wheel_rate_rad_s"]);
    let want = f64_vec(&case["applied_a"]);
    let mut st = ImperfectionState::new(*profile, dt_s);
    for (i, ((c, w), want)) in commanded
        .iter()
        .zip(wheel.iter())
        .zip(want.iter())
        .enumerate()
    {
        let got = st.apply_current(*c, Some(*w));
        assert_eq!(got, *want, "apply_current step {i} [{ctx}]");
    }
}

fn check_wheel_rate(profile: &ImperfectionProfile, dt_s: f64, case: &serde_json::Value, ctx: &str) {
    let t = f64_vec(&case["t_s"]);
    let true_rate = f64_vec(&case["true_rate_rad_s"]);
    let want = f64_vec(&case["reported_rad_s"]);
    let mut st = ImperfectionState::new(*profile, dt_s);
    for (i, ((ti, v), want)) in t.iter().zip(true_rate.iter()).zip(want.iter()).enumerate() {
        let got = st.wheel_rate(*v, *ti);
        assert_eq!(got, *want, "wheel_rate step {i} [{ctx}]");
    }
}
