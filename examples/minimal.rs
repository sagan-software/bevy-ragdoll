//! Drops the TGF human ragdoll and exposes unattended capture options.

use bevy_ragdoll_examples::{ExampleKind, run_example};

/// Starts the Rapier-backed minimal example.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    run_example(ExampleKind::Minimal)
}
