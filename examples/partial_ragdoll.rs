//! A partial ragdoll: driven legs under a loose upper body.
//!
//! The rig is the reference humanoid skeleton with a generated profile.
//! The pelvis and legs follow a procedural idle pose at full muscle strength
//! and are pinned to their animated targets. The upper body keeps 10% muscle
//! strength and no pin, so
//! the balls launched at the chest every two seconds knock it around while
//! the legs keep standing.
//!
//! Controls:
//!
//! - `Space`: pause or resume the chest impacts.

use bevy::app::AnimationSystems;
use bevy::prelude::*;
use bevy_ragdoll::runtime::components::{
    BodyWeights, Ragdoll, RagdollBodyOf, RagdollBodyWeights, RagdollDrive, RagdollMode,
};
use bevy_ragdoll::runtime::pin::PinTargets;
use bevy_ragdoll::runtime::sets::RagdollSystems;
use bevy_ragdoll::{Body, BodyIndex, BodyRole, RagdollDebugPlugin, RagdollPlugin, RagdollProfile};
use bevy_ragdoll_rapier3d::{RapierRagdollHooks, RapierRagdollPlugin};
use bevy_rapier3d::plugin::{RapierPhysicsPlugin, TimestepMode};
use bevy_rapier3d::prelude::{Collider, Damping, Restitution, RigidBody, Velocity};

/// Loads the rig profile and runs the windowed example.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let profile = RagdollProfile::from_skeleton(&bevy_ragdoll::Skeleton::humanoid())?;
    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "Partial ragdoll".into(),
            resolution: (1280, 720).into(),
            ..default()
        }),
        ..default()
    }));
    add_ragdoll_physics(&mut app);
    app.insert_resource(ClearColor(Color::srgb(0.055, 0.075, 0.095)))
        .insert_resource(Rig(profile))
        .insert_resource(LaunchTimer(Timer::from_seconds(2.0, TimerMode::Repeating)));
    add_example_systems(&mut app);
    app.run();
    Ok(())
}

/// Adds the ragdoll runtime and Rapier, both stepping in `FixedUpdate` at 60 Hz.
fn add_ragdoll_physics(app: &mut App) {
    app.add_plugins((
        RagdollPlugin::default(),
        RapierPhysicsPlugin::<RapierRagdollHooks<'_, '_>>::default().in_fixed_schedule(),
        RapierRagdollPlugin,
        RagdollDebugPlugin,
    ));
    app.insert_resource(Time::<Fixed>::from_hz(60.0))
        .insert_resource(TimestepMode::Fixed {
            dt: 1.0 / 60.0,
            substeps: 1,
        });
}

/// Registers reflected state and adds the scene, idle animation, and input systems.
#[cfg_attr(
    dylint_lib = "sagan_lints",
    expect(
        bevy_disallow_update_schedule,
        reason = "input handling and UI react once per rendered frame, which is what Update is for"
    )
)]
fn add_example_systems(app: &mut App) {
    app.register_type::<Rig>()
        .register_type::<IdleTarget>()
        .register_type::<LaunchTimer>()
        .register_type::<Ball>();
    app.add_systems(Startup, (setup_scene, spawn_ragdoll))
        .add_systems(
            PostUpdate,
            animate_idle_targets
                .after(AnimationSystems)
                .before(RagdollSystems::CaptureTargets),
        )
        .add_systems(
            Update,
            (toggle_launcher, launch_balls, despawn_old_balls).chain(),
        );
}

/// The validated profile shared by the ragdoll and the ball launcher.
#[derive(Resource, Reflect)]
struct Rig(RagdollProfile);

/// A skeleton bone that sways around its rest rotation to give the drive a target.
#[derive(Component, Reflect)]
struct IdleTarget {
    /// Profile position, used to offset the sway phase.
    index: usize,
    /// Anatomical role, used to choose the sway amplitude.
    role: BodyRole,
    /// Local rest rotation from the profile.
    rest_rotation: Quat,
}

/// Fires every two seconds to launch a ball; `Space` pauses it.
#[derive(Resource, Reflect)]
struct LaunchTimer(Timer);

/// A launched ball, removed when its lifetime ends.
#[derive(Component, Reflect)]
struct Ball {
    /// Time left before the ball is removed.
    lifetime: Timer,
}

/// Returns whether a body is the pelvis or part of a leg.
const fn is_lower_body(body: &Body) -> bool {
    matches!(
        body.role(),
        BodyRole::Pelvis | BodyRole::Thigh | BodyRole::Calf | BodyRole::Foot
    )
}

/// Drives and pins the lower body at full strength and leaves the upper body loose.
fn ragdoll_controls(profile: &RagdollProfile) -> (RagdollBodyWeights, PinTargets) {
    // Legs get full muscle and pins; everything above the pelvis stays loose.
    let mut weights = Vec::with_capacity(profile.bodies().len());
    let mut pins = PinTargets::none();
    for (position, body) in profile.bodies().iter().enumerate() {
        if is_lower_body(body) {
            weights.push(BodyWeights::new(1.0, 1.0));
            pins.set(body_index(position), true);
        } else {
            weights.push(BodyWeights::new(0.1, 0.0));
        }
    }
    // Pins apply only to the lower-body bodies chosen above.
    (RagdollBodyWeights::new(weights), pins)
}

/// Converts a profile position to a body index.
fn body_index(position: usize) -> BodyIndex {
    BodyIndex::try_from(position).expect("validated profiles fit the body limit")
}

/// Spawns the camera, light, ground, and title.
fn setup_scene(
    mut commands: Commands<'_, '_>,
    mut meshes: ResMut<'_, Assets<Mesh>>,
    mut materials: ResMut<'_, Assets<StandardMaterial>>,
) {
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
        RigidBody::Fixed,
        Collider::cuboid(6.0, 0.1, 6.0),
        Mesh3d(meshes.add(Cuboid::new(12.0, 0.2, 12.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.11, 0.15, 0.19),
            perceptual_roughness: 0.92,
            ..default()
        })),
        Transform::from_xyz(0.0, -0.1, 0.0),
    ));
    commands.spawn((
        Text::new(
            "Partial ragdoll\nLegs stay driven while the upper body yields.\nSpace pauses or resumes chest impacts.",
        ),
        TextFont {
            font_size: FontSize::Px(20.0),
            ..default()
        },
        Node {
            position_type: PositionType::Absolute,
            top: px(18),
            left: px(20),
            ..default()
        },
    ));
}

/// Spawns the ragdoll root and one bone per profile body at its rest pose.
fn spawn_ragdoll(
    mut commands: Commands<'_, '_>,
    mut profiles: ResMut<'_, Assets<RagdollProfile>>,
    rig: Res<'_, Rig>,
) {
    let profile = &rig.0;
    // Weights and pins decide which half of the body stays driven.
    let (weights, pins) = ragdoll_controls(profile);
    let character = commands
        .spawn((
            Name::new("Partial ragdoll"),
            Ragdoll::new(profiles.add(profile.clone())),
            RagdollMode::Dynamic,
            RagdollDrive::new(1.0, 1.0),
            weights,
            pins,
            Transform::from_xyz(0.0, 0.35, 0.0),
        ))
        .id();

    // Bodies without a joint are roots and hang from the character entity.
    let bodies = profile.bodies();

    // Profiles list parents before children, so each parent bone already exists.
    let mut bones = Vec::with_capacity(bodies.len());
    for (index, body) in bodies.iter().enumerate() {
        let rest = body.rest();
        // The parent's bone entity and rest pose, when this body has a parent.
        let parent = profile
            .joint_of(body.index())
            .map(|joint| joint.parent().get())
            .and_then(|parent| Some((*bones.get(parent)?, bodies.get(parent)?.rest())));
        let (parent, transform) = match parent {
            Some((parent, parent_rest)) => {
                let inverse = parent_rest.rotation.inverse();
                let offset = inverse * (rest.translation - parent_rest.translation);
                let local = Transform::from_translation(offset.into())
                    .with_rotation(inverse * rest.rotation);
                (parent, local)
            }
            None => (
                character,
                Transform::from_translation(rest.translation.into()).with_rotation(rest.rotation),
            ),
        };
        let target = IdleTarget {
            index,
            role: body.role(),
            rest_rotation: transform.rotation,
        };
        let bone = commands
            .spawn((
                Name::new(body.bone().to_owned()),
                target,
                transform,
                ChildOf(parent),
            ))
            .id();
        bones.push(bone);
    }
}

/// Sways each bone before the runtime captures its world pose as a drive target.
fn animate_idle_targets(
    time: Res<'_, Time>,
    mut bones: Query<'_, '_, (&IdleTarget, &mut Transform)>,
) {
    // Every bone sways; the weak upper-body muscles follow it only loosely.
    for (bone, mut transform) in &mut bones {
        let phase = f32::from(u8::try_from(bone.index % 9).unwrap_or_default()) * 0.47;
        let amplitude = match bone.role {
            BodyRole::Pelvis | BodyRole::Thigh | BodyRole::Calf | BodyRole::Foot => 0.018,
            BodyRole::Spine | BodyRole::Chest => 0.035,
            BodyRole::UpperArm | BodyRole::LowerArm | BodyRole::Hand => 0.055,
            BodyRole::Head | BodyRole::Neck => 0.022,
            BodyRole::Tail | BodyRole::Other => 0.03,
        };
        let sway = f32::mul_add(time.elapsed_secs(), 0.82, phase).sin() * amplitude;
        transform.rotation = bone.rest_rotation * Quat::from_rotation_z(sway);
    }
}

/// Pauses or resumes the launch timer on `Space`.
fn toggle_launcher(keys: Res<'_, ButtonInput<KeyCode>>, mut timer: ResMut<'_, LaunchTimer>) {
    if keys.just_pressed(KeyCode::Space) {
        if timer.0.is_paused() {
            timer.0.unpause();
        } else {
            timer.0.pause();
        }
    }
}

/// Launches a ball at the chest each time the launch timer fires.
fn launch_balls(
    mut commands: Commands<'_, '_>,
    time: Res<'_, Time>,
    rig: Res<'_, Rig>,
    mut timer: ResMut<'_, LaunchTimer>,
    bodies: Query<'_, '_, (&BodyIndex, &GlobalTransform), With<RagdollBodyOf>>,
    mut meshes: ResMut<'_, Assets<Mesh>>,
    mut materials: ResMut<'_, Assets<StandardMaterial>>,
) {
    // Launch one ball each time the repeating timer fires.
    if !timer.0.tick(time.delta()).just_finished() {
        return;
    }
    let chest = rig.0.body_with_role(BodyRole::Chest).map(BodyIndex::get);
    let Some((_, chest)) = bodies.iter().find(|(index, _)| Some(index.get()) == chest) else {
        return;
    };
    // Fire from in front of the chest, slightly upward, toward -Z.
    let direction = Vec3::new(0.0, 0.04, -1.0).normalize();
    commands.spawn((
        RigidBody::Dynamic,
        Collider::ball(0.11),
        Velocity::linear(direction * 7.0),
        Damping {
            linear_damping: 0.04,
            angular_damping: 0.04,
        },
        Restitution::coefficient(0.35),
        Transform::from_translation(chest.translation() - direction * 1.15),
        Mesh3d(meshes.add(Sphere::new(0.11))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.93, 0.57, 0.3),
            metallic: 0.18,
            perceptual_roughness: 0.32,
            ..default()
        })),
        Ball {
            lifetime: Timer::from_seconds(4.0, TimerMode::Once),
        },
    ));
}

/// Removes balls after four seconds or once they fall below the floor.
fn despawn_old_balls(
    mut commands: Commands<'_, '_>,
    time: Res<'_, Time>,
    mut balls: Query<'_, '_, (Entity, &mut Ball, &Transform)>,
) {
    // Every ball's timer ticks each frame; expired or fallen balls are removed.
    balls
        .iter_mut()
        .filter_map(|(entity, mut ball, transform)| {
            let is_expired = ball.lifetime.tick(time.delta()).is_finished();
            (is_expired || transform.translation.y < -2.0).then_some(entity)
        })
        .for_each(|entity| commands.entity(entity).despawn());
}

#[cfg(test)]
/// Tests the lower-body control split against the reference humanoid.
mod tests {
    use super::*;

    /// The pelvis and legs are pinned at full strength; every other body is loose.
    #[test]
    fn lower_body_is_driven_and_upper_body_is_loose() {
        let profile = RagdollProfile::from_skeleton(&bevy_ragdoll::Skeleton::humanoid())
            .expect("profile validates");
        let (weights, pins) = ragdoll_controls(&profile);
        let mut legs = 0;
        for (position, body) in profile.bodies().iter().enumerate() {
            let weight = weights.get(position).expect("weights cover every body");
            let lower = is_lower_body(body);
            legs += usize::from(lower && body.role() != BodyRole::Pelvis);
            assert_eq!(pins.is_targeted(body_index(position)), lower);
            let expected = if lower { (1.0, 1.0) } else { (0.1, 0.0) };
            assert_eq!((weight.muscle(), weight.pin()), expected);
        }
        assert!(legs >= 4, "the humanoid includes both legs");
    }
}
