//! Loads and validates the human ragdoll profile from RON.

use bevy_ragdoll_examples::{ExampleKind, run_example};

/// Starts the RON-authored Rapier example.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    run_example(ExampleKind::FromRon)
}
