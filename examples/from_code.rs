//! Builds a three-body chain with `ProfileBuilder` and pins its root body.
//!
//! Run with `cargo run --example from_code`.

use bevy::prelude::*;
use bevy_ragdoll::runtime::components::{
    BodyWeights, Ragdoll, RagdollBodyWeights, RagdollDrive, RagdollMode,
};
use bevy_ragdoll::{
    AngleRange, JointLimits, ProfileBuilder, ProfileError, RagdollDebugPlugin, RagdollPlugin,
    RagdollProfile, ShapeSpec,
};
use bevy_ragdoll_rapier3d::{RapierRagdollHooks, RapierRagdollPlugin};
use bevy_rapier3d::plugin::{RapierPhysicsPlugin, TimestepMode};
use bevy_rapier3d::prelude::{Collider, RigidBody};

/// Text shown in the top-left corner.
const LABEL: &str = "Profile built in Rust\nThe pinned pelvis holds while the chain swings.";

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
        ))
        .add_systems(Startup, (setup_scene, spawn_ragdoll))
        .add_plugins(RagdollDebugPlugin)
        .run();
}

/// Builds the pelvis, chest, and head chain in code.
fn build_profile() -> Result<RagdollProfile, ProfileError> {
    let mut builder = ProfileBuilder::default();
    let pelvis = builder.add_body(
        "pelvis",
        ShapeSpec::Capsule {
            a: Vec3::new(-0.24, 0.0, 0.0),
            b: Vec3::new(0.24, 0.0, 0.0),
            radius: 0.2,
        },
        8.0,
        Isometry3d::from_translation(Vec3::new(0.0, 1.0, 0.0)),
    )?;
    let chest = builder.add_body(
        "chest",
        ShapeSpec::Capsule {
            a: Vec3::ZERO,
            b: Vec3::new(0.0, 0.6, 0.0),
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
        Isometry3d::from_translation(Vec3::new(0.0, 2.36, 0.0)),
    )?;
    let bend = AngleRange {
        min: -0.7,
        max: 0.7,
    };
    let limits = JointLimits {
        x: bend,
        twist: bend,
        z: bend,
    };
    // Each joint's frame is relative to its parent body; the last value is motor strength.
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
        Isometry3d::from_translation(Vec3::new(0.0, 0.66, 0.0)),
        limits,
        25.0,
    );
    builder.build()
}

/// Spawns the chain with a full-strength pin on the pelvis and limp upper bodies.
fn spawn_ragdoll(mut commands: Commands<'_, '_>, mut profiles: ResMut<'_, Assets<RagdollProfile>>) {
    let profile = build_profile().expect("the chain is a valid profile");
    // Body 0 (pelvis) keeps its pin; the chest and head only get muscle weight.
    let weights = (0..profile.bodies().len())
        .map(|index| BodyWeights::new(1.0, if index == 0 { 1.0 } else { 0.0 }))
        .collect();
    let character = commands
        .spawn((
            Name::new("ragdoll"),
            Ragdoll::new(profiles.add(profile.clone())),
            RagdollMode::Dynamic,
            RagdollDrive::new(0.0, 1.0),
            RagdollBodyWeights::new(weights),
            Transform::from_xyz(0.0, 0.35, 0.0),
        ))
        .id();
    spawn_skeleton(&mut commands, character, &profile);
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

/// Spawns one bone entity per profile body, posed at the profile rest pose.
///
/// The ragdoll runtime binds each body to the bone with the same `Name`.
fn spawn_skeleton(commands: &mut Commands<'_, '_>, character: Entity, profile: &RagdollProfile) {
    let bodies = profile.bodies();
    let mut parents = vec![None; bodies.len()];
    for joint in profile.joints() {
        if let Some(slot) = parents.get_mut(joint.child().get()) {
            *slot = Some(joint.parent().get());
        }
    }
    // Profiles list parents before children, so each parent bone already exists.
    let mut bones = Vec::<Entity>::with_capacity(bodies.len());
    for (index, body) in bodies.iter().enumerate() {
        let rest = body.rest();
        // The parent's bone entity and rest pose, when this body has a parent.
        let parent = parents
            .get(index)
            .copied()
            .flatten()
            .and_then(|parent| Some((*bones.get(parent)?, bodies.get(parent)?.rest())));
        let (parent, transform) = match parent {
            None => (
                character,
                Transform::from_translation(rest.translation.into()).with_rotation(rest.rotation),
            ),
            Some((parent, parent_rest)) => {
                let inverse = parent_rest.rotation.inverse();
                (
                    parent,
                    Transform::from_translation(
                        (inverse * (rest.translation - parent_rest.translation)).into(),
                    )
                    .with_rotation(inverse * rest.rotation),
                )
            }
        };
        let bone = commands
            .spawn((
                Name::new(body.bone().to_owned()),
                transform,
                ChildOf(parent),
            ))
            .id();
        bones.push(bone);
    }
}
