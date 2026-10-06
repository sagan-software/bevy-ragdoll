//! Builds a three-body chain with `ProfileBuilder` and pins its root.

use bevy_ragdoll_examples::{ExampleKind, run_example};

/// Starts the code-authored Rapier example.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    run_example(ExampleKind::FromCode)
}
