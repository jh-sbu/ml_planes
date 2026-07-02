//! CLI argument parsing for `ml_planes_mcp`.
//!
//! Hand-rolled `--flag value` window scan, matching the style of `src/bin/server.rs`
//! (no `clap`). The parse core lives in [`McpArgs::parse_from`] so it is unit-testable
//! without touching `std::env`.

use std::net::SocketAddr;
use std::time::Duration;

use crate::net::DEFAULT_PORT;

/// Parsed `ml_planes_mcp` options.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct McpArgs {
    /// Address of the `ml_planes_server` to join. Default: `127.0.0.1:DEFAULT_PORT`.
    pub connect: SocketAddr,
    /// Connection/handshake window and the initial reconnect backoff. Default: 5 s.
    pub connect_timeout: Duration,
    /// Silence non-error logging (unless `RUST_LOG` overrides). Default: `false`.
    pub quiet: bool,
}

impl Default for McpArgs {
    fn default() -> Self {
        Self {
            connect: SocketAddr::from(([127, 0, 0, 1], DEFAULT_PORT)),
            connect_timeout: Duration::from_secs(5),
            quiet: false,
        }
    }
}

impl McpArgs {
    /// Parse from the process arguments (`std::env::args()`), exiting the process with a
    /// diagnostic on bad input (mirrors `server.rs`).
    pub fn parse() -> Self {
        let args: Vec<String> = std::env::args().collect();
        match Self::parse_from(&args) {
            Ok(parsed) => parsed,
            Err(err) => {
                eprintln!("ml_planes_mcp: {err}");
                std::process::exit(2);
            }
        }
    }

    /// Pure parser over an explicit argument vector. Returns `Err(message)` on malformed
    /// input so callers (and tests) can decide how to surface it.
    ///
    /// Recognised flags:
    ///   `--connect host:port`   server address (default `127.0.0.1:DEFAULT_PORT`)
    ///   `--connect-timeout SECS` connect deadline in whole seconds (default `5`)
    ///   `--quiet`               silence non-error logging (presence flag; `RUST_LOG` wins)
    pub fn parse_from(args: &[String]) -> Result<Self, String> {
        let mut parsed = Self::default();

        if let Some(raw) = get_arg(args, "--connect") {
            parsed.connect =
                parse_addr(&raw).ok_or_else(|| format!("--connect: cannot resolve '{raw}'"))?;
        }

        if let Some(raw) = get_arg(args, "--connect-timeout") {
            let secs: u64 = raw
                .parse()
                .map_err(|_| format!("--connect-timeout expects whole seconds, got '{raw}'"))?;
            parsed.connect_timeout = Duration::from_secs(secs);
        }

        parsed.quiet = args.iter().any(|a| a == "--quiet");

        Ok(parsed)
    }
}

/// Look up the value following `flag` in a `--flag value` argument list.
fn get_arg(args: &[String], flag: &str) -> Option<String> {
    args.windows(2)
        .find(|pair| pair[0] == flag)
        .map(|pair| pair[1].clone())
}

/// Resolve a `host:port` string to a [`SocketAddr`] (DNS-resolving), taking the first
/// address. Mirrors the visual client's `parse_addr` (`src/ui/menu.rs`), which is
/// `visual`-gated and thus not importable here.
fn parse_addr(raw: &str) -> Option<SocketAddr> {
    use std::net::ToSocketAddrs;
    raw.trim().to_socket_addrs().ok()?.next()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn argv(extra: &[&str]) -> Vec<String> {
        std::iter::once("ml_planes_mcp")
            .chain(extra.iter().copied())
            .map(String::from)
            .collect()
    }

    #[test]
    fn defaults_when_no_flags() {
        let parsed = McpArgs::parse_from(&argv(&[])).unwrap();
        assert_eq!(
            parsed.connect,
            SocketAddr::from(([127, 0, 0, 1], DEFAULT_PORT))
        );
        assert_eq!(parsed.connect_timeout, Duration::from_secs(5));
    }

    #[test]
    fn explicit_connect_addr() {
        let parsed = McpArgs::parse_from(&argv(&["--connect", "1.2.3.4:9000"])).unwrap();
        assert_eq!(parsed.connect, "1.2.3.4:9000".parse().unwrap());
    }

    #[test]
    fn explicit_connect_timeout() {
        let parsed = McpArgs::parse_from(&argv(&["--connect-timeout", "10"])).unwrap();
        assert_eq!(parsed.connect_timeout, Duration::from_secs(10));
    }

    #[test]
    fn both_flags_together() {
        let parsed = McpArgs::parse_from(&argv(&[
            "--connect",
            "127.0.0.1:7777",
            "--connect-timeout",
            "3",
        ]))
        .unwrap();
        assert_eq!(parsed.connect, "127.0.0.1:7777".parse().unwrap());
        assert_eq!(parsed.connect_timeout, Duration::from_secs(3));
    }

    #[test]
    fn rejects_unparseable_addr() {
        let err = McpArgs::parse_from(&argv(&["--connect", "not an address"])).unwrap_err();
        assert!(err.contains("--connect"), "unexpected error: {err}");
    }

    #[test]
    fn rejects_non_numeric_timeout() {
        let err = McpArgs::parse_from(&argv(&["--connect-timeout", "soon"])).unwrap_err();
        assert!(err.contains("--connect-timeout"), "unexpected error: {err}");
    }

    #[test]
    fn quiet_defaults_false() {
        assert!(!McpArgs::parse_from(&argv(&[])).unwrap().quiet);
    }

    #[test]
    fn quiet_flag_sets_quiet() {
        assert!(McpArgs::parse_from(&argv(&["--quiet"])).unwrap().quiet);
    }

    #[test]
    fn quiet_combines_with_other_flags() {
        let parsed =
            McpArgs::parse_from(&argv(&["--connect", "127.0.0.1:7777", "--quiet"])).unwrap();
        assert_eq!(parsed.connect, "127.0.0.1:7777".parse().unwrap());
        assert!(parsed.quiet);
    }
}
