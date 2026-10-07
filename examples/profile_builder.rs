//! Build and validate a one-body ragdoll profile.
//!
//! Profiles are plain data, so they can be built and checked without a Bevy
//! app. Validation rejects bad shapes, masses, and joint limits before any
//! physics body exists. The example prints the validated total mass.

use bevy::math::{Isometry3d, Vec3};
use bevy_ragdoll::{ProfileBuilder, RagdollProfile, ShapeSpec};

/// Builds a profile from code and prints its total mass.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    // One sphere body is the smallest valid profile.
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

    // Building validates the shape, mass, and tree before returning a profile.
    let profile: RagdollProfile = builder.build()?;
    let total_mass = profile.total_mass().kilograms();
    println!("{total_mass} kg");
    Ok(())
}
