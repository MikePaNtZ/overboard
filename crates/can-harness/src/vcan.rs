//! Bring up and tear down a virtual CAN interface without a manual `ip
//! link` step (issue #52 AC1) -- the same rtnetlink calls `ip link add ...
//! type vcan` issues itself, driven directly from Rust via `socketcan`'s
//! `netlink` feature.

use socketcan::{CanInterface, CanSocket, Socket};
use std::fmt;

/// Why a vcan interface could not be brought up here.
///
/// Not a hard failure for a caller to propagate. The expected case is an
/// unprivileged `cargo test --workspace` run (no `CAP_NET_ADMIN`) or a
/// kernel with no `vcan` module loaded -- callers should treat this as
/// "skip this test with a clear message," per AC1, via [`up_or_skip`].
#[derive(Debug)]
pub struct VcanUnavailable(String);

impl fmt::Display for VcanUnavailable {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl std::error::Error for VcanUnavailable {}

/// A vcan interface this process created. Torn down automatically on drop --
/// the other half of AC1's "no manual step."
pub struct VcanFixture {
    interface: Option<CanInterface>,
    name: String,
}

impl VcanFixture {
    /// Creates and brings up a fresh vcan interface named `name`.
    ///
    /// `name` should be unique per test (e.g. include the test name) --
    /// tests may run concurrently, and creating an interface name already in
    /// use fails.
    pub fn up(name: &str) -> Result<Self, VcanUnavailable> {
        let interface = CanInterface::create_vcan(name, None).map_err(|e| {
            VcanUnavailable(format!(
                "could not create vcan interface '{name}' ({e}) -- likely missing \
                 CAP_NET_ADMIN or the kernel has no vcan module loaded"
            ))
        })?;
        interface.bring_up().map_err(|e| {
            VcanUnavailable(format!("created '{name}' but could not bring it up: {e}"))
        })?;
        Ok(Self {
            interface: Some(interface),
            name: name.to_string(),
        })
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    /// Opens a raw CAN socket bound to this interface.
    pub fn socket(&self) -> std::io::Result<CanSocket> {
        CanSocket::open(&self.name)
    }
}

impl Drop for VcanFixture {
    fn drop(&mut self) {
        // Best-effort: if the interface is already gone -- another cleanup
        // raced us, or this fixture is unwinding after a partial failure --
        // that is not this destructor's problem to report.
        if let Some(interface) = self.interface.take() {
            let _ = interface.delete();
        }
    }
}

/// Brings up a vcan fixture, or prints why it can't and returns `None` so
/// the caller can return early instead of failing a test on infrastructure
/// it doesn't control. This is the "skipped with a clear message" half of
/// AC1 -- run it, read the SKIP line in the test output.
pub fn up_or_skip(name: &str) -> Option<VcanFixture> {
    match VcanFixture::up(name) {
        Ok(fixture) => Some(fixture),
        Err(e) => {
            eprintln!("SKIP {name}: {e}");
            None
        }
    }
}
