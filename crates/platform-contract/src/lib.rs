//! The runtime facts the control loop depends on, checked against the live
//! system rather than assumed -- issue #182 I6, "the platform contract,
//! asserted rather than documented".
//!
//! D1 (`docs/design-pi-image-stage0b.md`) argues a repo split was unsafe
//! because "nothing detects a runtime-contract mismatch except running both
//! halves together, and that it surfaces as a wrong, unattributable bench
//! number." [`STAGE0B`] plus [`evaluate`] is that detection, independent of
//! whether the repo ever splits: kernel flavour, page size, `isolcpus`
//! layout and CAN bitrate are exactly the states a card can silently drift
//! into (a re-flash with a different pin, a firmware update, a HAT on the
//! wrong bus) and produce a latency or actuation number nobody can trust.
//!
//! Every field here duplicates a value pinned elsewhere in the repo --
//! unavoidably: Rust cannot source `scripts/pi/pins.env`, and it is not this
//! crate's job to parse the YAML/systemd-unit text those pins also live in.
//! `scripts/pi/check_image_pin.py`'s own docstring names this exact shape of
//! risk for the kernel pin: "Duplication is normally a mistake ... The
//! difference between a duplicate that is fine and one that rots is whether
//! anything checks." `tests::contract_matches_checked_in_pins` is that check
//! for this crate -- it reads the source-of-truth files directly on every
//! `cargo test` and fails if [`STAGE0B`] drifts from them.
//!
//! **Not yet wired to anything that runs on a Pi.** #182 I5 (the controller
//! binary pipeline) has not landed -- `scripts/pi/image/layer/overboard-base.yaml`
//! says so directly ("no Overboard binary is in this image at all"), so there
//! is no path onto the card for *any* binary from this workspace yet, this
//! one included. The checking logic is built and tested now so it is not
//! blocked on I5; wiring it into a boot-time systemd unit or a hardware
//! backend's `arm()` (AC-14's shape, once one exists) is that later work's
//! own increment, not this one's.

use std::fmt;

#[cfg(target_os = "linux")]
pub mod linux;

/// The runtime facts a 500 Hz control loop on Stage-0B's hardware depends
/// on, and what each one is measured or pinned against.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Contract {
    /// `uname -r`. Kept as the literal string measured on real hardware
    /// (`docs/design-pi-image-stage0b-reference.md` Q11/Q12,
    /// closed 2026-08-07 via `uname -a`), not derived from the kernel
    /// package/version pin -- Raspberry Pi's package-name-to-`uname -r`
    /// mapping is not a pure rename; Q12 found the boot *filename* half of
    /// that mapping wrong once already (`kernel8_rt.img`, not `kernel8.img`).
    pub kernel_release: &'static str,
    /// The 4K-page cost `docs/design-pi-image-stage0b.md` accepts explicitly
    /// in exchange for the packaged RT kernel (no 16K-page build exists).
    pub page_size_bytes: usize,
    /// Cores handed to `isolcpus=` for the control loop
    /// (`scripts/pi/image/device/overboard-pi5/device.yaml`).
    pub isolated_cpus: &'static [u32],
    /// The ICD bus rate `overboard-can0.service` brings `can0` up at.
    pub can0_bitrate_bps: u32,
}

/// The measured/pinned contract for Stage-0B's Pi 5 image.
pub const STAGE0B: Contract = Contract {
    kernel_release: "6.12.75+rpt-rpi-v8-rt",
    page_size_bytes: 4096,
    isolated_cpus: &[3],
    can0_bitrate_bps: 500_000,
};

/// Outcome of comparing one field of a [`Contract`] against the live system.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FieldStatus {
    Match,
    Mismatch {
        expected: String,
        actual: String,
    },
    /// The system could not report this field at all. For most fields this
    /// is itself a fault -- but for `can0_bitrate`, "no HAT fitted" is a
    /// supported bench state (`overboard-can0.service`'s own boot-time
    /// posture), not a corrupt system, so [`Report::ok_to_arm`] treats it
    /// differently from the other three fields. See that method's doc.
    Unavailable {
        reason: String,
    },
}

impl FieldStatus {
    pub fn is_match(&self) -> bool {
        matches!(self, FieldStatus::Match)
    }
}

impl fmt::Display for FieldStatus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FieldStatus::Match => write!(f, "OK"),
            FieldStatus::Mismatch { expected, actual } => {
                write!(f, "MISMATCH (expected {expected}, got {actual})")
            }
            FieldStatus::Unavailable { reason } => write!(f, "UNAVAILABLE ({reason})"),
        }
    }
}

/// Result of checking a [`Contract`] against the running system.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Report {
    pub kernel_release: FieldStatus,
    pub page_size: FieldStatus,
    pub isolated_cpus: FieldStatus,
    pub can0_bitrate: FieldStatus,
}

impl Report {
    /// True if the platform matches the contract closely enough to arm on.
    ///
    /// `kernel_release`, `page_size` and `isolated_cpus` must always be
    /// readable on a Linux system, so `Unavailable` on any of them is a hard
    /// failure, same as `Mismatch`. `can0_bitrate` is different: a missing
    /// `can0` interface is `overboard-can0.service`'s own supported
    /// no-HAT-fitted state (self-test over `vcan0` covers that case
    /// instead), so `Unavailable` there does not refuse to arm -- only a
    /// `can0` that exists at the *wrong* bitrate does.
    pub fn ok_to_arm(&self) -> bool {
        let can0_ok = !matches!(self.can0_bitrate, FieldStatus::Mismatch { .. });
        self.kernel_release.is_match()
            && self.page_size.is_match()
            && self.isolated_cpus.is_match()
            && can0_ok
    }
}

impl fmt::Display for Report {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "platform-contract report:")?;
        writeln!(f, "  kernel_release : {}", self.kernel_release)?;
        writeln!(f, "  page_size      : {}", self.page_size)?;
        writeln!(f, "  isolated_cpus  : {}", self.isolated_cpus)?;
        writeln!(f, "  can0_bitrate   : {}", self.can0_bitrate)?;
        write!(f, "  ok_to_arm      : {}", self.ok_to_arm())
    }
}

/// Everything [`evaluate`] needs to read from the live system, behind a
/// trait so the comparison logic is testable with no Pi, no root and no
/// `isolcpus=` boot line -- the same reason `hal`/`sim-backend` sit behind a
/// trait rather than being called directly.
pub trait SystemProbe {
    fn kernel_release(&self) -> Result<String, String>;
    fn page_size_bytes(&self) -> Result<usize, String>;
    fn isolated_cpus(&self) -> Result<Vec<u32>, String>;
    /// `Ok(None)` means "no `can0` interface", which is not itself an error
    /// -- see [`Report::ok_to_arm`].
    fn can0_bitrate_bps(&self) -> Result<Option<u32>, String>;
}

/// Compares `contract` against whatever `probe` reports.
pub fn evaluate<P: SystemProbe>(contract: &Contract, probe: &P) -> Report {
    let kernel_release = match probe.kernel_release() {
        Ok(actual) if actual == contract.kernel_release => FieldStatus::Match,
        Ok(actual) => FieldStatus::Mismatch {
            expected: contract.kernel_release.to_string(),
            actual,
        },
        Err(reason) => FieldStatus::Unavailable { reason },
    };

    let page_size = match probe.page_size_bytes() {
        Ok(actual) if actual == contract.page_size_bytes => FieldStatus::Match,
        Ok(actual) => FieldStatus::Mismatch {
            expected: contract.page_size_bytes.to_string(),
            actual: actual.to_string(),
        },
        Err(reason) => FieldStatus::Unavailable { reason },
    };

    let isolated_cpus = match probe.isolated_cpus() {
        Ok(actual) if actual.as_slice() == contract.isolated_cpus => FieldStatus::Match,
        Ok(actual) => FieldStatus::Mismatch {
            expected: format!("{:?}", contract.isolated_cpus),
            actual: format!("{actual:?}"),
        },
        Err(reason) => FieldStatus::Unavailable { reason },
    };

    let can0_bitrate = match probe.can0_bitrate_bps() {
        Ok(Some(actual)) if actual == contract.can0_bitrate_bps => FieldStatus::Match,
        Ok(Some(actual)) => FieldStatus::Mismatch {
            expected: contract.can0_bitrate_bps.to_string(),
            actual: actual.to_string(),
        },
        Ok(None) => FieldStatus::Unavailable {
            reason: "can0 interface not present".to_string(),
        },
        Err(reason) => FieldStatus::Unavailable { reason },
    };

    Report {
        kernel_release,
        page_size,
        isolated_cpus,
        can0_bitrate,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct FakeProbe {
        kernel_release: Result<String, String>,
        page_size_bytes: Result<usize, String>,
        isolated_cpus: Result<Vec<u32>, String>,
        can0_bitrate_bps: Result<Option<u32>, String>,
    }

    impl SystemProbe for FakeProbe {
        fn kernel_release(&self) -> Result<String, String> {
            self.kernel_release.clone()
        }
        fn page_size_bytes(&self) -> Result<usize, String> {
            self.page_size_bytes.clone()
        }
        fn isolated_cpus(&self) -> Result<Vec<u32>, String> {
            self.isolated_cpus.clone()
        }
        fn can0_bitrate_bps(&self) -> Result<Option<u32>, String> {
            self.can0_bitrate_bps.clone()
        }
    }

    fn matching_probe() -> FakeProbe {
        FakeProbe {
            kernel_release: Ok(STAGE0B.kernel_release.to_string()),
            page_size_bytes: Ok(STAGE0B.page_size_bytes),
            isolated_cpus: Ok(STAGE0B.isolated_cpus.to_vec()),
            can0_bitrate_bps: Ok(Some(STAGE0B.can0_bitrate_bps)),
        }
    }

    #[test]
    fn matching_system_is_ok_to_arm() {
        let report = evaluate(&STAGE0B, &matching_probe());
        assert!(report.ok_to_arm());
        assert_eq!(report.kernel_release, FieldStatus::Match);
        assert_eq!(report.page_size, FieldStatus::Match);
        assert_eq!(report.isolated_cpus, FieldStatus::Match);
        assert_eq!(report.can0_bitrate, FieldStatus::Match);
    }

    #[test]
    fn wrong_kernel_refuses_to_arm() {
        let mut probe = matching_probe();
        probe.kernel_release = Ok("6.18.34+rpt-rpi-2712".to_string());
        let report = evaluate(&STAGE0B, &probe);
        assert!(!report.ok_to_arm());
        assert!(matches!(
            report.kernel_release,
            FieldStatus::Mismatch { .. }
        ));
    }

    #[test]
    fn wrong_isolcpus_refuses_to_arm() {
        let mut probe = matching_probe();
        probe.isolated_cpus = Ok(vec![2]);
        let report = evaluate(&STAGE0B, &probe);
        assert!(!report.ok_to_arm());
        assert!(matches!(report.isolated_cpus, FieldStatus::Mismatch { .. }));
    }

    #[test]
    fn wrong_page_size_refuses_to_arm() {
        let mut probe = matching_probe();
        probe.page_size_bytes = Ok(16384);
        let report = evaluate(&STAGE0B, &probe);
        assert!(!report.ok_to_arm());
        assert!(matches!(report.page_size, FieldStatus::Mismatch { .. }));
    }

    #[test]
    fn wrong_can0_bitrate_refuses_to_arm() {
        let mut probe = matching_probe();
        probe.can0_bitrate_bps = Ok(Some(1_000_000));
        let report = evaluate(&STAGE0B, &probe);
        assert!(!report.ok_to_arm());
        assert!(matches!(report.can0_bitrate, FieldStatus::Mismatch { .. }));
    }

    #[test]
    fn absent_can0_is_still_ok_to_arm() {
        // No HAT fitted is a supported bench state (overboard-can0.service);
        // the vcan0 self-test path exists specifically to cover it.
        let mut probe = matching_probe();
        probe.can0_bitrate_bps = Ok(None);
        let report = evaluate(&STAGE0B, &probe);
        assert!(report.ok_to_arm());
        assert!(matches!(
            report.can0_bitrate,
            FieldStatus::Unavailable { .. }
        ));
    }

    #[test]
    fn unreadable_kernel_refuses_to_arm() {
        let mut probe = matching_probe();
        probe.kernel_release = Err("boom".to_string());
        let report = evaluate(&STAGE0B, &probe);
        assert!(!report.ok_to_arm());
        assert!(matches!(
            report.kernel_release,
            FieldStatus::Unavailable { .. }
        ));
    }

    #[test]
    fn unreadable_isolcpus_refuses_to_arm() {
        let mut probe = matching_probe();
        probe.isolated_cpus = Err("no /proc/cmdline".to_string());
        let report = evaluate(&STAGE0B, &probe);
        assert!(!report.ok_to_arm());
    }

    /// The duplication guard described in this module's doc comment. Reads
    /// the actual checked-in files -- not a second hand-copied constant --
    /// so `STAGE0B` cannot silently drift from the values it restates.
    #[test]
    fn contract_matches_checked_in_pins() {
        let repo_root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .and_then(|p| p.parent())
            .expect("crates/platform-contract sits two levels under the repo root");

        let pins = std::fs::read_to_string(repo_root.join("scripts/pi/pins.env"))
            .expect("scripts/pi/pins.env should exist and be readable");
        let expected_pin = format!("KERNEL_PACKAGE=\"linux-image-{}\"", STAGE0B.kernel_release);
        assert!(
            pins.contains(&expected_pin),
            "STAGE0B.kernel_release ({}) no longer matches pins.env's KERNEL_PACKAGE -- \
             update whichever one drifted",
            STAGE0B.kernel_release,
        );

        let device_yaml = std::fs::read_to_string(
            repo_root.join("scripts/pi/image/device/overboard-pi5/device.yaml"),
        )
        .expect("scripts/pi/image/device/overboard-pi5/device.yaml should exist and be readable");
        let expected_isolcpus = STAGE0B
            .isolated_cpus
            .iter()
            .map(u32::to_string)
            .collect::<Vec<_>>()
            .join(",");
        assert!(
            device_yaml.contains(&format!("isolcpus={expected_isolcpus}")),
            "STAGE0B.isolated_cpus no longer matches device.yaml's isolcpus= line -- \
             update whichever one drifted",
        );

        let can0_service = std::fs::read_to_string(repo_root.join(
            "scripts/pi/image/layer/overboard-base.rootfs-overlay/etc/systemd/system/overboard-can0.service",
        ))
        .expect("overboard-can0.service should exist and be readable");
        assert!(
            can0_service.contains(&format!("bitrate {}", STAGE0B.can0_bitrate_bps)),
            "STAGE0B.can0_bitrate_bps no longer matches overboard-can0.service -- \
             update whichever one drifted",
        );
    }
}
