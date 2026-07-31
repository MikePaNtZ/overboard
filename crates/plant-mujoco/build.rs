//! Locates and links libmujoco, then compiles `src/shim.c` against its
//! headers (issue #91 / I1a).
//!
//! **Discovery is a single `MUJOCO_DIR` env var with a wheel-probe
//! fallback, and this build script panics -- loudly, with the exact thing it
//! tried -- if neither finds a real install.** A build that silently degrades
//! to "no plant" is the failure mode this project keeps hitting (see
//! `crates/sim-backend`'s header), so there is no quiet "skip this crate"
//! path here at all.
//!
//! Either source must point (directly, or via `MUJOCO_DIR/lib`) at a
//! directory containing both `include/mujoco/mujoco.h` and a
//! `libmujoco.so.*` / `libmujoco*.dylib`. The pip-installed `mujoco` wheel's
//! own package directory satisfies this shape exactly -- which is also why
//! the wheel-probe fallback works: `import mujoco` and read
//! `os.path.dirname(mujoco.__file__)`. The probe asks the `PATH`
//! interpreters first and then the repo's own `.venv` -- see
//! [`python_candidates`], which is what makes this build on a Mac (#112).
//!
//! The wheel ships no unversioned `libmujoco.so` / `libmujoco.dylib` symlink
//! for a plain `-lmujoco` to find, so this links the exact same
//! `libmujoco.so.3.10.0` (or its macOS `.dylib` equivalent) via an
//! unversioned symlink created in `OUT_DIR` at build time, rather than a raw
//! linker path. That is deliberate, not cosmetic: `cargo:rustc-link-lib` (and
//! a `cargo:rustc-link-search` pointed *inside* `OUT_DIR`) are what Cargo
//! actually carries through to every crate that depends on this one --
//! `canary-ridden`'s link and, crucially, its *runtime* dynamic-library
//! search path too. `cargo:rustc-link-arg` looked simpler but silently does
//! neither: it only affects this package's own targets, which is exactly the
//! kind of "worked here, broke one hop downstream" failure this project
//! keeps hitting.

use std::env;
use std::path::{Path, PathBuf};
use std::process::Command;

/// Relative path checked to confirm `MUJOCO_DIR` (or its wheel-probed
/// equivalent) actually looks like a MuJoCo install. The *version* itself is
/// not checked here -- `Plant::open`'s `mj_versionString()` assertion in
/// `src/lib.rs` is what enforces that, at run time, against the string, not
/// against anything this build script could infer from a path.
const HEADER: &str = "include/mujoco/mujoco.h";

fn main() {
    println!("cargo:rerun-if-env-changed=MUJOCO_DIR");
    println!("cargo:rerun-if-changed=src/shim.c");

    let mujoco_dir = locate_mujoco();
    let include_dir = mujoco_dir.join("include");
    if !mujoco_dir.join(HEADER).is_file() {
        panic!(
            "plant-mujoco: '{}' does not look like a MuJoCo install -- expected \
             a header at '{}/{HEADER}' but found none. Set MUJOCO_DIR to a \
             directory containing 'include/mujoco/mujoco.h' plus a \
             'libmujoco.so.*' / 'libmujoco*.dylib' next to it (or under \
             'lib/') -- the pip 'mujoco' wheel's own package directory \
             satisfies both.",
            mujoco_dir.display(),
            mujoco_dir.display(),
        );
    }

    let lib_dir = if mujoco_dir.join("lib").is_dir() {
        mujoco_dir.join("lib")
    } else {
        mujoco_dir.clone()
    };
    let lib_path = find_shared_lib(&lib_dir).unwrap_or_else(|| {
        panic!(
            "plant-mujoco: found headers under '{}' but no libmujoco shared \
             library in '{}' (looked for 'libmujoco.so.*' / 'libmujoco*.dylib'). \
             A headers-only install cannot be linked against.",
            mujoco_dir.display(),
            lib_dir.display(),
        )
    });

    cc::Build::new()
        .file("src/shim.c")
        .include(&include_dir)
        .compile("plant_mujoco_shim");

    // OUT_DIR gets TWO symlinks to the same real, versioned file:
    //   - the unversioned name ("libmujoco.so" / "libmujoco.dylib"), which is
    //     what a bare `-lmujoco` resolves against at LINK time;
    //   - the real, versioned name again (e.g. "libmujoco.so.3.10.0"), which
    //     is what the dynamic loader looks for at RUN time -- it resolves by
    //     the library's own recorded SONAME, not by the name `-l` was given.
    // Both matter because the search path below is INSIDE OUT_DIR, which is
    // the one case Cargo documents as also adding to the dynamic-library
    // search-path env var for every `cargo test`/`cargo run` in this
    // invocation -- including a downstream crate's, like `canary-ridden`'s.
    let out_dir = PathBuf::from(env::var("OUT_DIR").expect("OUT_DIR is always set for build.rs"));
    let target = lib_path.canonicalize().unwrap_or_else(|e| {
        panic!(
            "plant-mujoco: could not resolve '{}': {e}",
            lib_path.display()
        )
    });
    let unversioned_name = if lib_path.to_string_lossy().ends_with(".dylib") {
        "libmujoco.dylib"
    } else {
        "libmujoco.so"
    };
    let real_name = lib_path
        .file_name()
        .expect("lib_path came from read_dir, so it always has a file name")
        .to_string_lossy()
        .into_owned();
    for name in [unversioned_name.to_string(), real_name] {
        let link = out_dir.join(&name);
        let _ = std::fs::remove_file(&link); // stale symlink from a previous build
        std::os::unix::fs::symlink(&target, &link).unwrap_or_else(|e| {
            panic!(
                "plant-mujoco: could not symlink '{}' -> '{}': {e}",
                link.display(),
                target.display()
            )
        });
    }

    println!("cargo:rustc-link-search=native={}", out_dir.display());
    println!("cargo:rustc-link-lib=dylib=mujoco");
}

/// `MUJOCO_DIR` if set (a hard requirement to satisfy, not a soft hint --
/// an empty or wrong value fails loudly rather than falling through), else
/// the wheel-probe fallback: ask whichever `python3`/`python` is on `PATH`
/// where its installed `mujoco` package lives. That is the SAME libmujoco the
/// `sim` CI job already installs from `requirements-sim.txt`.
fn locate_mujoco() -> PathBuf {
    if let Ok(dir) = env::var("MUJOCO_DIR") {
        if dir.trim().is_empty() {
            panic!("plant-mujoco: MUJOCO_DIR is set but empty");
        }
        return PathBuf::from(dir);
    }

    for python in python_candidates() {
        let output = Command::new(&python)
            .args([
                "-c",
                "import mujoco, os; print(os.path.dirname(mujoco.__file__))",
            ])
            .output();
        if let Ok(out) = output {
            if out.status.success() {
                let dir = String::from_utf8_lossy(&out.stdout).trim().to_string();
                if !dir.is_empty() {
                    return PathBuf::from(dir);
                }
            }
        }
    }

    panic!(
        "plant-mujoco: no MuJoCo install found. Set MUJOCO_DIR to the 'mujoco' \
         pip package directory (contains 'include/mujoco/*.h' and \
         'libmujoco.*'), or `pip install -r requirements-sim.txt` so the \
         wheel-probe fallback can find it. Probed, in order: {:?}. This crate \
         refuses to build headless rather than silently degrading to 'no \
         plant' -- that silent degrade is the failure mode this programme \
         keeps hitting.",
        python_candidates()
    );
}

/// Interpreters to ask, in order: whatever is on `PATH` first, then the
/// repo's own `.venv`.
///
/// **The `.venv` entries are why `cargo test -p plant-mujoco` works on a Mac
/// (issue #112).** CI installs the wheel into the runner's system Python, so
/// `python3` on `PATH` finds it and the first candidate wins. A developer
/// machine almost never does: the wheel goes into the repo `.venv` that every
/// other tool here uses (`.venv/bin/python -m pytest`, `scripts/*.py`), and
/// the `python3` on `PATH` is a Homebrew or system interpreter that has never
/// heard of MuJoCo. The build then failed with "no MuJoCo install found"
/// *despite* `requirements-sim.txt` having been installed exactly as
/// documented -- which reads as a broken crate rather than an unactivated
/// virtualenv.
///
/// Ordering is deliberate: `PATH` is probed FIRST so this is strictly
/// additive and cannot change which libmujoco CI links against.
fn python_candidates() -> Vec<String> {
    let mut out = vec!["python3".to_string(), "python".to_string()];

    // CARGO_MANIFEST_DIR is <repo>/crates/plant-mujoco.
    if let Ok(manifest) = env::var("CARGO_MANIFEST_DIR") {
        if let Some(repo) = Path::new(&manifest).ancestors().nth(2) {
            for exe in ["python3", "python"] {
                let venv = repo.join(".venv/bin").join(exe);
                if venv.is_file() {
                    out.push(venv.to_string_lossy().into_owned());
                }
            }
        }
    }
    out
}

/// The versioned shared library in `dir`, if any.
fn find_shared_lib(dir: &Path) -> Option<PathBuf> {
    std::fs::read_dir(dir)
        .ok()?
        .flatten()
        .map(|entry| entry.path())
        .find(|path| {
            let Some(name) = path.file_name().and_then(|n| n.to_str()) else {
                return false;
            };
            name.starts_with("libmujoco.so")
                || (name.starts_with("libmujoco") && name.ends_with(".dylib"))
        })
}
