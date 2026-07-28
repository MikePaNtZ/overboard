//! Dependency-graph gate for issue #1: proves `hal-actuate` (motion
//! authority) is unreachable from `board-app-ridden` at build time, rather
//! than trusting a reviewer to notice a stray import.
//!
//! `cargo metadata`'s resolve graph already tells us, per edge, whether a
//! dependency is `Normal`, `Development` (dev-dependency) or `Build`
//! (build-dependency). Only `Normal` edges ship in the built binary — a
//! dev-dependency of some crate `board-app-ridden` happens to depend on does
//! not make it into the linked artifact, so it must not fail the gate
//! (issue #1's attack-list row 1). The walk below follows normal edges only,
//! and does so transitively, so `ridden -> shared -> vesc-tx` fails exactly
//! as `ridden -> vesc-tx` would (row 3).
//!
//! Run as `cargo run -p xtask -- gate` (also the default with no
//! subcommand, so a bare `cargo run -p xtask` in CI does the right thing).

use cargo_metadata::{CargoOpt, DependencyKind, Metadata, MetadataCommand, NodeDep, PackageId};
use std::collections::{HashMap, HashSet, VecDeque};
use std::process::ExitCode;

const RIDDEN: &str = "board-app-ridden";
const MARKER: &str = "hal-actuate";
const CANARY: &str = "canary-ridden";

/// True if at least one target (lib, bin, test, ...) links `dep` normally.
/// The same package can appear with several `dep_kinds` when different
/// targets depend on it differently; if *any* of them is `Normal`, the crate
/// ships, so the edge counts as reachable.
fn has_normal_edge(dep: &NodeDep) -> bool {
    dep.dep_kinds
        .iter()
        .any(|k| k.kind == DependencyKind::Normal)
}

/// Adjacency over normal edges only, keyed by the opaque package id string.
struct Graph {
    edges: HashMap<String, Vec<String>>,
}

impl Graph {
    fn from_metadata(meta: &Metadata) -> Result<Self, String> {
        let resolve = meta.resolve.as_ref().ok_or_else(|| {
            "`cargo metadata` returned no resolve graph -- is this workspace missing a lockfile?"
                .to_string()
        })?;

        let mut edges = HashMap::new();
        for node in &resolve.nodes {
            let normal: Vec<String> = node
                .deps
                .iter()
                .filter(|d| has_normal_edge(d))
                .map(|d| d.pkg.repr.clone())
                .collect();
            edges.insert(node.id.repr.clone(), normal);
        }
        Ok(Graph { edges })
    }

    /// Every package transitively reachable from `start` over normal edges,
    /// not including `start` itself.
    fn reachable_from(&self, start: &str) -> HashSet<String> {
        let mut seen: HashSet<String> = HashSet::new();
        let mut queue: VecDeque<String> = VecDeque::new();
        queue.push_back(start.to_string());
        seen.insert(start.to_string());

        while let Some(id) = queue.pop_front() {
            for dep in self.edges.get(&id).into_iter().flatten() {
                if seen.insert(dep.clone()) {
                    queue.push_back(dep.clone());
                }
            }
        }
        seen.remove(start);
        seen
    }
}

/// Finds a workspace package by name. Deliberately an `Err`, not a silent
/// "not found -> nothing to check": issue #1's attack list requires the gate
/// to fail loudly if the marker crate is ever renamed, rather than quietly
/// passing because it no longer recognises the name it is looking for.
fn find_package_id(meta: &Metadata, name: &str) -> Result<PackageId, String> {
    meta.packages
        .iter()
        .find(|p| p.name == name)
        .map(|p| p.id.clone())
        .ok_or_else(|| {
            format!(
                "package '{name}' was not found in `cargo metadata` output -- \
                 was it renamed or removed? the gate does not know what to check \
                 and must fail rather than silently pass."
            )
        })
}

/// Runs both halves of the gate against one resolve graph (one feature
/// configuration): the real check, and its positive control.
fn check_boundary(meta: &Metadata, config_label: &str) -> Result<(), String> {
    let graph = Graph::from_metadata(meta)?;

    let ridden = find_package_id(meta, RIDDEN)?;
    let marker = find_package_id(meta, MARKER)?;
    let canary = find_package_id(meta, CANARY)?;

    let from_ridden = graph.reachable_from(&ridden.repr);
    if from_ridden.contains(&marker.repr) {
        return Err(format!(
            "[{config_label}] {RIDDEN} can reach {MARKER} over a normal dependency edge. \
             The ridden binary must have zero motion authority (BoardIo ICD S6.3, DR-MODE-1)."
        ));
    }

    // Positive control (issue #1): if `canary-ridden`, which depends on
    // `hal-actuate` on purpose, is NOT flagged as reaching it, the checker
    // itself is broken -- e.g. the marker name changed and this file was not
    // updated -- and that is a worse failure than the boundary itself
    // breaking, because it fails silently.
    let from_canary = graph.reachable_from(&canary.repr);
    if !from_canary.contains(&marker.repr) {
        return Err(format!(
            "[{config_label}] positive control failed: {CANARY} does not reach {MARKER}, \
             but it is supposed to depend on it directly. The gate is not working."
        ));
    }

    Ok(())
}

fn run_gate() -> Result<(), String> {
    let default_meta = MetadataCommand::new()
        .exec()
        .map_err(|e| format!("`cargo metadata` failed: {e}"))?;
    check_boundary(&default_meta, "default features")?;

    // Attack-list row 2: cargo features are additive and unify across the
    // whole dependency graph, which is exactly what defeated the earlier
    // cargo-feature-gated `actuate` approach. Re-running metadata with every
    // feature enabled is what actually exercises that failure mode, rather
    // than trusting the default resolve to still hold under
    // `cargo build --workspace --all-features`.
    let all_features_meta = MetadataCommand::new()
        .features(CargoOpt::AllFeatures)
        .exec()
        .map_err(|e| format!("`cargo metadata --all-features` failed: {e}"))?;
    check_boundary(&all_features_meta, "--all-features")?;

    Ok(())
}

fn main() -> ExitCode {
    let mut args = std::env::args().skip(1);
    match args.next().as_deref() {
        None | Some("gate") => {}
        Some(other) => {
            eprintln!("xtask: unknown subcommand '{other}' (expected 'gate' or no argument)");
            return ExitCode::FAILURE;
        }
    }

    match run_gate() {
        Ok(()) => {
            println!(
                "xtask gate: {MARKER} is unreachable from {RIDDEN} \
                 (checked under default features and --all-features); \
                 {CANARY} was correctly flagged as reaching it."
            );
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("xtask gate FAILED: {e}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------
    // Graph reachability -- exercised against synthetic graphs so these
    // stay fast and do not depend on the real workspace shape.
    // -----------------------------------------------------------------

    fn graph_from(edges: &[(&str, &str)]) -> Graph {
        let mut g = Graph {
            edges: HashMap::new(),
        };
        for (from, to) in edges {
            g.edges
                .entry(from.to_string())
                .or_default()
                .push(to.to_string());
        }
        g
    }

    #[test]
    fn direct_normal_edge_is_reachable() {
        let g = graph_from(&[("ridden", "marker")]);
        assert!(g.reachable_from("ridden").contains("marker"));
    }

    #[test]
    fn attack_row_3_transitive_normal_edge_is_reachable() {
        // "telemetry -> vesc-tx transitively" from issue #1's attack list:
        // the walk must not stop at the first hop.
        let g = graph_from(&[("ridden", "telemetry"), ("telemetry", "vesc-tx")]);
        assert!(g.reachable_from("ridden").contains("vesc-tx"));
    }

    #[test]
    fn unrelated_package_is_not_reachable() {
        let g = graph_from(&[("ridden", "hal"), ("driverless", "marker")]);
        assert!(!g.reachable_from("ridden").contains("marker"));
    }

    #[test]
    fn attack_row_1_dev_only_edge_never_enters_the_graph() {
        // A dev-dependency edge is filtered out before it reaches `Graph` at
        // all (see `has_normal_edge`), so a graph built only from that
        // relationship has no edge for the walk to find -- mirroring "gate
        // passes" for a vesc-tx dev-dependency of a shared crate.
        let g = graph_from(&[("ridden", "shared")]); // shared -> vesc-tx omitted: dev-only
        assert!(!g.reachable_from("ridden").contains("vesc-tx"));
    }

    #[test]
    fn a_package_does_not_reach_itself() {
        let g = graph_from(&[("ridden", "ridden")]);
        assert!(!g.reachable_from("ridden").contains("ridden"));
    }

    // -----------------------------------------------------------------
    // dep_kinds filtering -- exercised against real cargo_metadata types,
    // deserialized from the same JSON shape `cargo metadata` emits, so this
    // tests the actual parsing logic rather than a paraphrase of it.
    // -----------------------------------------------------------------

    fn node_dep(dep_kinds_json: &str) -> NodeDep {
        let json =
            format!(r#"{{"name":"x","pkg":"path+file:///x#0.1.0","dep_kinds":{dep_kinds_json}}}"#);
        serde_json::from_str(&json).expect("fixture JSON must match NodeDep's schema")
    }

    #[test]
    fn normal_kind_counts_as_a_normal_edge() {
        // `cargo metadata` represents the normal kind as a JSON `null`, not
        // the string "normal".
        let dep = node_dep(r#"[{"kind":null,"target":null}]"#);
        assert!(has_normal_edge(&dep));
    }

    #[test]
    fn attack_row_1_dev_only_kind_does_not_count_as_a_normal_edge() {
        let dep = node_dep(r#"[{"kind":"dev","target":null}]"#);
        assert!(!has_normal_edge(&dep));
    }

    #[test]
    fn build_only_kind_does_not_count_as_a_normal_edge() {
        let dep = node_dep(r#"[{"kind":"build","target":null}]"#);
        assert!(!has_normal_edge(&dep));
    }

    #[test]
    fn attack_row_5_any_normal_kind_among_several_targets_counts() {
        // The same package can be depended on with different kinds by
        // different targets (e.g. an optional dep a shared crate's lib
        // target enables via a feature, alongside a dev-only use in its
        // tests). One real target linking it normally is enough for it to
        // ship, regardless of what else is also true about it.
        let dep = node_dep(r#"[{"kind":"dev","target":null},{"kind":null,"target":null}]"#);
        assert!(has_normal_edge(&dep));
    }

    // -----------------------------------------------------------------
    // Attack-list row 4: a renamed/missing marker crate must fail loudly.
    // -----------------------------------------------------------------

    #[test]
    fn missing_package_name_is_an_explicit_error_not_a_silent_pass() {
        let meta_json = r#"{
            "packages": [],
            "workspace_members": [],
            "workspace_default_members": [],
            "resolve": {"nodes": [], "root": null},
            "target_directory": "/tmp/target",
            "workspace_root": "/tmp",
            "build_directory": null,
            "version": 1
        }"#;
        let meta: Metadata =
            serde_json::from_str(meta_json).expect("fixture JSON must match Metadata's schema");
        let err = find_package_id(&meta, "hal-actuate").unwrap_err();
        assert!(err.contains("hal-actuate"));
        assert!(err.contains("renamed") || err.contains("removed"));
    }
}
