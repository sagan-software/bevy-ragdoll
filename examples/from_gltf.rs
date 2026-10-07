//! Generates ragdolls from skinned glTF creatures with no authored files.
//!
//! The humanoid uses UE mannequin bone names and gets the humanoid layout.
//! The quadruped and the seven-legged alien are classified by topology.
//! Press Space to drop them all.

use bevy::prelude::*;
use bevy_ragdoll::RagdollDebugPlugin;
use bevy_ragdoll::runtime::components::{RagdollDrive, RagdollMode};
use bevy_ragdoll::{Ragdoll, RagdollPlugin};
use bevy_ragdoll_rapier3d::{RapierRagdollHooks, RapierRagdollPlugin};
use bevy_rapier3d::plugin::{RapierPhysicsPlugin, TimestepMode};
use bevy_rapier3d::prelude::{Collider, RigidBody};

/// Runs the example.
fn main() -> AppExit {
    App::new()
        // Web servers answer 404 for the `.meta` files Bevy probes by default; the rigs have none.
        .add_plugins(DefaultPlugins.set(AssetPlugin {
            meta_check: bevy::asset::AssetMetaCheck::Never,
            ..default()
        }))
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
        .add_systems(Startup, setup)
        .add_systems(Update, drop_on_space)
        .run()
}

/// Spawns the floor, camera, light and the three creatures.
fn setup(
    mut commands: Commands<'_, '_>,
    assets: Res<'_, AssetServer>,
    mut meshes: ResMut<'_, Assets<Mesh>>,
    mut materials: ResMut<'_, Assets<StandardMaterial>>,
) {
    commands.spawn((
        RigidBody::Fixed,
        Collider::cuboid(8.0, 0.1, 8.0),
        Transform::from_xyz(0.0, -0.1, 0.0),
        Mesh3d(meshes.add(Plane3d::default().mesh().size(16.0, 16.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.11, 0.15, 0.19))),
    ));
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 2.2, 5.5).looking_at(Vec3::new(0.0, 0.7, 0.0), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 12_000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(-4.0, 7.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    for (x, rig) in [(-2.0, "humanoid"), (0.0, "quadruped"), (2.0, "alien")] {
        commands.spawn((
            WorldAssetRoot(
                assets.load(GltfAssetLabel::Scene(0).from_asset(format!("rigs/{rig}.glb"))),
            ),
            Ragdoll::default(),
            RagdollMode::Kinematic,
            Transform::from_xyz(x, 0.05, 0.0),
        ));
    }
}

/// Switches every ragdoll to limp dynamic simulation when Space is pressed.
fn drop_on_space(
    keys: Res<'_, ButtonInput<KeyCode>>,
    mut ragdolls: Query<'_, '_, (&mut RagdollMode, &mut RagdollDrive), With<Ragdoll>>,
) {
    if !keys.just_pressed(KeyCode::Space) {
        return;
    }
    for (mut mode, mut drive) in &mut ragdolls {
        *mode = RagdollMode::Dynamic;
        drive.set(0.0, 0.0);
    }
}
