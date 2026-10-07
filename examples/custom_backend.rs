//! Runs the ragdoll runtime on a custom physics backend.
//!
//! A backend is a Bevy plugin that reads the runtime's body components and
//! writes simulated poses back. This example uses `MockBackendPlugin` from
//! `bevy_ragdoll_conformance`, a semi-implicit Euler integrator. It ignores
//! joints and contacts, so the body's parts fall apart; that is expected.
//!
//! Run with `cargo run --example custom_backend`.

use bevy::prelude::*;
use bevy_ragdoll::runtime::components::{Ragdoll, RagdollDrive, RagdollMode};
use bevy_ragdoll::{RagdollDebugPlugin, 
    AngleRange, JointLimits, ProfileBuilder, ProfileError, RagdollPlugin,
    RagdollProfile, ShapeSpec,
};
use bevy_ragdoll_conformance::mock::MockBackendPlugin;

/// Runs the example.
fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(Time::<Fixed>::from_hz(60.0))
        // Swap `MockBackendPlugin` for your own backend plugin.
        .add_plugins((RagdollPlugin::default(), MockBackendPlugin))
        .add_systems(Startup, (setup_scene, spawn_ragdoll))
        .add_plugins(RagdollDebugPlugin)
        .run();
}

/// Adds a floor, a camera, a light, and a label.
fn setup_scene(
    mut commands: Commands<'_, '_>,
    mut meshes: ResMut<'_, Assets<Mesh>>,
    mut materials: ResMut<'_, Assets<StandardMaterial>>,
) {
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(12.0, 12.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.11, 0.15, 0.19))),
        Transform::from_xyz(0.0, -0.02, 0.0),
    ));
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(3.1, 2.7, 5.8).looking_at(Vec3::new(0.0, 1.25, 0.0), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 12_000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(-3.0, 6.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        Text::new("Mock backend: joint constraints are ignored."),
        Node {
            position_type: PositionType::Absolute,
            top: px(16),
            left: px(18),
            ..default()
        },
    ));
}

/// Builds the pelvis, chest, and head chain in code.
fn build_profile() -> Result<RagdollProfile, ProfileError> {
    let mut builder = ProfileBuilder::default();
    let pelvis = builder.add_body(
        "pelvis",
        ShapeSpec::Capsule {
            a: Vec3::new(-0.24, 0.0, 0.0),
            b: Vec3::new(0.24, 0.0, 0.0),
            radius: 0.22,
        },
        8.0,
        Isometry3d::from_translation(Vec3::new(0.0, 1.0, 0.0)),
    )?;
    let chest = builder.add_body(
        "chest",
        ShapeSpec::Capsule {
            a: Vec3::ZERO,
            b: Vec3::new(0.0, 0.58, 0.0),
            radius: 0.17,
        },
        12.0,
        Isometry3d::from_translation(Vec3::new(0.0, 1.7, 0.0)),
    )?;
    let head = builder.add_body(
        "head",
        ShapeSpec::Sphere {
            center: Vec3::ZERO,
            radius: 0.16,
        },
        4.0,
        Isometry3d::from_translation(Vec3::new(0.0, 2.35, 0.0)),
    )?;
    let bend = AngleRange {
        min: -0.6,
        max: 0.6,
    };
    let limits = JointLimits {
        x: bend,
        twist: bend,
        z: bend,
    };
    builder.add_joint(
        chest,
        pelvis,
        Isometry3d::from_translation(Vec3::new(0.0, 0.7, 0.0)),
        limits,
        80.0,
    );
    builder.add_joint(
        head,
        chest,
        Isometry3d::from_translation(Vec3::new(0.0, 0.65, 0.0)),
        limits,
        25.0,
    );
    builder.build()
}

/// Spawns a limp dynamic ragdoll and the named bones its profile binds to.
fn spawn_ragdoll(mut commands: Commands<'_, '_>, mut profiles: ResMut<'_, Assets<RagdollProfile>>) {
    let profile = build_profile().expect("the chain is a valid profile");
    let character = commands
        .spawn((
            Ragdoll::new(profiles.add(profile)),
            RagdollMode::Dynamic,
            RagdollDrive::new(0.0, 0.0),
            Transform::IDENTITY,
        ))
        .id();
    // Bone transforms are local to their parent bone.
    let pelvis = commands
        .spawn((
            Name::new("pelvis"),
            Transform::from_xyz(0.0, 1.0, 0.0),
            ChildOf(character),
        ))
        .id();
    let chest = commands
        .spawn((
            Name::new("chest"),
            Transform::from_xyz(0.0, 0.7, 0.0),
            ChildOf(pelvis),
        ))
        .id();
    commands.spawn((
        Name::new("head"),
        Transform::from_xyz(0.0, 0.65, 0.0),
        ChildOf(chest),
    ));
}

