//! Drops a ragdoll generated from the reference humanoid skeleton.
//!
//! Run with `cargo run --example minimal`.

use bevy::prelude::*;
use bevy_ragdoll::runtime::components::RagdollMode;
use bevy_ragdoll::{Ragdoll, RagdollDebugPlugin, RagdollPlugin, Skeleton};
use bevy_ragdoll_rapier3d::{RapierRagdollHooks, RapierRagdollPlugin};
use bevy_rapier3d::plugin::{RapierPhysicsPlugin, TimestepMode};
use bevy_rapier3d::prelude::{Collider, RigidBody};

/// Text shown in the top-left corner.
const LABEL: &str = "Minimal ragdoll\nThe body lands, keeps its joints together, and settles.";

/// Runs the example.
fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        // The ragdoll runtime and Rapier both step in `FixedUpdate` at 60 Hz.
        .insert_resource(Time::<Fixed>::from_hz(60.0))
        .insert_resource(TimestepMode::Fixed {
            dt: 1.0 / 60.0,
            substeps: 1,
        })
        .add_plugins((
            RagdollPlugin::default(),
            RapierPhysicsPlugin::<RapierRagdollHooks<'_, '_>>::default().in_fixed_schedule(),
            RapierRagdollPlugin,
            RagdollDebugPlugin,
        ))
        .add_systems(Startup, (setup_scene, spawn_ragdoll))
        .run();
}

/// Spawns the reference skeleton under a dynamic ragdoll character.
fn spawn_ragdoll(mut commands: Commands<'_, '_>) {
    let character = commands
        .spawn((
            Name::new("ragdoll"),
            Ragdoll::default(),
            RagdollMode::Dynamic,
            Transform::from_xyz(0.0, 0.35, 0.0),
        ))
        .id();
    Skeleton::humanoid().spawn(&mut commands, character);
}

/// Adds a fixed Rapier floor, a camera, a light, and a label.
fn setup_scene(
    mut commands: Commands<'_, '_>,
    mut meshes: ResMut<'_, Assets<Mesh>>,
    mut materials: ResMut<'_, Assets<StandardMaterial>>,
) {
    commands.spawn((
        RigidBody::Fixed,
        Collider::cuboid(6.0, 0.1, 6.0),
        Transform::from_xyz(0.0, -0.1, 0.0),
        Mesh3d(meshes.add(Plane3d::default().mesh().size(12.0, 12.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.11, 0.15, 0.19))),
    ));
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(1.6, 2.0, 2.7).looking_at(Vec3::new(0.0, 1.0, 0.0), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 14_000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(-4.0, 7.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        Text::new(LABEL),
        Node {
            position_type: PositionType::Absolute,
            top: px(18),
            left: px(20),
            ..default()
        },
    ));
}
