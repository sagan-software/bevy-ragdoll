//! Build and validate a one-body ragdoll profile.

use bevy::math::{Isometry3d, Vec3};
use bevy_ragdoll::{ProfileBuilder, RagdollProfile, ShapeSpec};

/// Builds a profile from code and prints its total mass.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut builder = ProfileBuilder::default();
    builder.add_body(
        "pelvis",
        ShapeSpec::Capsule {
            a: Vec3::ZERO,
            b: Vec3::Y,
            radius: 0.2,
        },
        80.0,
        Isometry3d::IDENTITY,
    )?;

    let profile: RagdollProfile = builder.build()?;
    let total_mass = profile.total_mass().kilograms();
    println!("{total_mass} kg");
    Ok(())
}
