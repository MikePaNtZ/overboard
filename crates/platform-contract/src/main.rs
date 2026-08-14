//! Checks the running system against [`platform_contract::STAGE0B`] and
//! exits non-zero if it does not match closely enough to arm on.
//!
//! Not wired into anything yet -- see the crate doc comment (`src/lib.rs`)
//! for why: #182 I5, the controller binary pipeline, has not landed, so
//! there is no path onto the Pi image for this binary (or any other from
//! this workspace) today. This exists so the check itself is built and
//! tested now rather than blocked on I5.

use std::process::ExitCode;

fn main() -> ExitCode {
    #[cfg(target_os = "linux")]
    {
        let probe = platform_contract::linux::RealProbe;
        let report = platform_contract::evaluate(&platform_contract::STAGE0B, &probe);
        println!("{report}");
        if report.ok_to_arm() {
            ExitCode::SUCCESS
        } else {
            eprintln!(
                "platform-contract: refusing to arm -- the running platform does not match \
                 the checked-in contract"
            );
            ExitCode::FAILURE
        }
    }
    #[cfg(not(target_os = "linux"))]
    {
        println!("platform-contract: no-op off Linux -- the contract it checks is Pi-5-specific");
        ExitCode::SUCCESS
    }
}
