//! Runs the partial-ragdoll example with periodic chest impacts.

use bevy_ragdoll_examples::{ExampleKind, run_example};

/// Starts the partial-control Rapier example with visible projectiles.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    run_example(ExampleKind::PartialRagdoll)
}
