//! Imports the owner-authored TGF GLB and its Skein ragdoll annotations.

use bevy_ragdoll_examples::{ExampleKind, run_example};

/// Starts the GLB and Skein Rapier example.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    run_example(ExampleKind::FromGltfSkein)
}
