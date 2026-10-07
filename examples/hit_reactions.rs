//! Runs the interactive hit-reaction example on the TGF human rig.

use bevy_ragdoll_examples::{ExampleKind, run_example};

/// Starts the interactive Rapier example with visible controls.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    run_example(ExampleKind::HitReactions)
}
