//! A server the client launches (Start New Server) must die with the client.
//!
//! `LocalServer` (visual) wraps a [`ml_planes::net::ServerProcess`], defined in the
//! non-visual `net` module so this test runs under the supported `--features "mcp
//! server"` matrix. The point of the newtype is its `Drop`: `std::process::Child`'s
//! own drop does *not* kill the process, so an app exit that skips the `OnExit(InGame)`
//! teardown (window close / Quit) would otherwise orphan the child. Dropping a
//! `ServerProcess` must kill+reap it.
#![cfg(feature = "net")]

use ml_planes::net::ServerProcess;

/// Dropping a `ServerProcess` kills the wrapped child process.
#[test]
fn dropping_server_process_kills_child() {
    // A long-lived stand-in for `ml_planes_server`.
    let child = std::process::Command::new("sleep")
        .arg("30")
        .spawn()
        .expect("spawn sleep");
    let pid = child.id();

    // Sanity: the process exists before we drop the handle.
    assert!(
        std::path::Path::new(&format!("/proc/{pid}")).exists(),
        "child should be running right after spawn"
    );

    let proc = ServerProcess(child);
    drop(proc); // Drop must kill() + wait() (reaping the zombie).

    // wait() reaped it, so /proc/<pid> is gone immediately — no polling needed.
    assert!(
        !std::path::Path::new(&format!("/proc/{pid}")).exists(),
        "child must be dead after ServerProcess is dropped"
    );
}
