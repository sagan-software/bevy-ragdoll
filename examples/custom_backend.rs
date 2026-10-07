//! Runs the ragdoll runtime on a custom physics backend.
//!
//! A backend is a Bevy plugin that reads the runtime's body components and
//! writes simulated poses back. This example uses `MockBackendPlugin` from
//! `bevy_ragdoll_conformance`, a semi-implicit Euler integrator. It ignores
//! joints and contacts, so the body's parts fall apart; that is expected.
//!
//! Run with `cargo run --example custom_backend`.

use bevy::prelude::*;
use bevy::transform::TransformSystems;
use bevy_ragdoll::runtime::body::BodyShape;
use bevy_ragdoll::runtime::components::{Ragdoll, RagdollDrive, RagdollMode};
use bevy_ragdoll::runtime::sets::RagdollSystems;
use bevy_ragdoll::{
    AngleRange, BodyIndex, JointLimits, ProfileBuilder, ProfileError, RagdollPlugin,
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
        .add_systems(
            PostUpdate,
            add_body_meshes
                .after(RagdollSystems::Bind)
                .before(TransformSystems::Propagate),
        )
        .run();
}

/// Adds a floor, a camera, a light, and a label.
fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
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
fn spawn_ragdoll(mut commands: Commands, mut profiles: ResMut<Assets<RagdollProfile>>) {
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

/// Gives each new physics body a mesh that matches its collision shape.
fn add_body_meshes(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    bodies: Query<(Entity, &BodyShape, &BodyIndex), Added<BodyShape>>,
) {
    let palette = [
        Color::srgb(0.18, 0.62, 0.76),
        Color::srgb(0.25, 0.78, 0.64),
        Color::srgb(0.92, 0.66, 0.34),
    ];
    for (body, shape, index) in &bodies {
        let (mesh, transform): (Mesh, Transform) = match shape.0 {
            ShapeSpec::Capsule { a, b, radius } => (
                Capsule3d::new(radius, a.distance(b)).into(),
                Transform::from_translation((a + b) * 0.5).with_rotation(Quat::from_rotation_arc(
                    Vec3::Y,
                    (b - a).try_normalize().unwrap_or(Vec3::Y),
                )),
            ),
            ShapeSpec::Sphere { center, radius } => (
                Sphere::new(radius).into(),
                Transform::from_translation(center),
            ),
            ShapeSpec::Cuboid {
                center,
                rotation,
                half_extents,
            } => (
                Cuboid::from_size(half_extents * 2.0).into(),
                Transform::from_translation(center).with_rotation(rotation),
            ),
        };
        commands.entity(body).insert(Visibility::Inherited);
        commands.spawn((
            Mesh3d(meshes.add(mesh)),
            MeshMaterial3d(materials.add(palette[index.get().min(2)])),
            transform,
            ChildOf(body),
        ));
    }
}
