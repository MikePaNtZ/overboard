//! The real [`SystemProbe`], reading the same places `uname -r`,
//! `getconf PAGE_SIZE`, the kernel's own `isolcpus=` cmdline parser and
//! `ip link show can0` themselves read.

use crate::SystemProbe;
use std::fs;

/// Reads the live system via `/proc` and `/sys`. No caching, no root
/// required -- every path here is world-readable.
#[derive(Debug, Default)]
pub struct RealProbe;

impl SystemProbe for RealProbe {
    fn kernel_release(&self) -> Result<String, String> {
        fs::read_to_string("/proc/sys/kernel/osrelease")
            .map(|s| s.trim().to_string())
            .map_err(|e| format!("/proc/sys/kernel/osrelease: {e}"))
    }

    fn page_size_bytes(&self) -> Result<usize, String> {
        // SAFETY: sysconf with a valid, read-only query name has no
        // preconditions beyond that; it never touches memory we own.
        let n = unsafe { libc::sysconf(libc::_SC_PAGESIZE) };
        if n <= 0 {
            Err(format!("sysconf(_SC_PAGESIZE) returned {n}"))
        } else {
            Ok(n as usize)
        }
    }

    fn isolated_cpus(&self) -> Result<Vec<u32>, String> {
        let cmdline =
            fs::read_to_string("/proc/cmdline").map_err(|e| format!("/proc/cmdline: {e}"))?;
        let token = cmdline
            .split_whitespace()
            .find_map(|tok| tok.strip_prefix("isolcpus="))
            .ok_or_else(|| "no isolcpus= token in /proc/cmdline".to_string())?;
        parse_cpu_list(token)
    }

    fn can0_bitrate_bps(&self) -> Result<Option<u32>, String> {
        let path = "/sys/class/net/can0/can_bittiming/bitrate";
        match fs::read_to_string(path) {
            Ok(s) => s
                .trim()
                .parse::<u32>()
                .map(Some)
                .map_err(|e| format!("{path}: {e}")),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
            Err(e) => Err(format!("{path}: {e}")),
        }
    }
}

/// Parses the kernel cpu-list grammar `isolcpus=` (and several other boot
/// parameters) uses: comma-separated entries, each a single core or an
/// inclusive `a-b` range.
fn parse_cpu_list(token: &str) -> Result<Vec<u32>, String> {
    let mut out = Vec::new();
    for part in token.split(',') {
        if let Some((lo, hi)) = part.split_once('-') {
            let lo: u32 = lo
                .parse()
                .map_err(|_| format!("bad isolcpus range '{part}'"))?;
            let hi: u32 = hi
                .parse()
                .map_err(|_| format!("bad isolcpus range '{part}'"))?;
            out.extend(lo..=hi);
        } else {
            out.push(
                part.parse()
                    .map_err(|_| format!("bad isolcpus entry '{part}'"))?,
            );
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_single_core() {
        assert_eq!(parse_cpu_list("3").unwrap(), vec![3]);
    }

    #[test]
    fn parses_range() {
        assert_eq!(parse_cpu_list("2-4").unwrap(), vec![2, 3, 4]);
    }

    #[test]
    fn parses_mixed_list() {
        assert_eq!(parse_cpu_list("0,2-4,7").unwrap(), vec![0, 2, 3, 4, 7]);
    }

    #[test]
    fn rejects_garbage() {
        assert!(parse_cpu_list("not-a-number").is_err());
    }
}
