//! Public runtime behavior with the shared conformance mock backend.

use std::sync::{
    Arc,
    atomic::{AtomicUsize, Ordering},
};

use bevy::asset::{AssetPlugin, Assets};
use bevy::prelude::{
    AnimationPlugin, App, ChildOf, Entity, GlobalTransform, Handle, MinimalPlugins, Name,
    Transform, TransformPlugin, Vec3, World,
};
use bevy::time::{Fixed, Time, TimeUpdateStrategy};
use bevy_ragdoll::profile::{AngleRange, BodyIndex, ProfileBuilder};
use bevy_ragdoll::runtime::body::{
    BodyAtRest, BodyDriveOutput, BodyKind, BodyMass, BodyPhysicsPose, BodyShape, BodyVelocity,
};
use bevy_ragdoll::runtime::budget::RagdollBudget;
use bevy_ragdoll::runtime::components::{
    BodyWeights, Ragdoll, RagdollBlend, RagdollBodyOf, RagdollBodyWeights, RagdollDrive,
    RagdollMode, RagdollTargetPose,
};
use bevy_ragdoll::runtime::drive::{interpolate_pose, joint_motor_values};
use bevy_ragdoll::runtime::events::{RagdollBudgetEvicted, RagdollSettled};
use bevy_ragdoll::runtime::messages::RagdollImpulse;
use bevy_ragdoll::runtime::settings::RagdollPhysicsSettings;
use bevy_ragdoll::{JointLimits, RagdollPlugin, RagdollProfile, ShapeSpec};
use bevy_ragdoll_conformance::mock::MockBackendPlugin;

/// Makes the two-body profile used by the runtime contract tests.
fn profile() -> RagdollProfile {
    let mut builder = ProfileBuilder::default();
    let shape = ShapeSpec::Capsule {
        a: Vec3::ZERO,
        b: Vec3::Y,
        radius: 0.1,
    };
    let root = builder
        .add_body("root", shape, 1.0, bevy::math::Isometry3d::IDENTITY)
        .expect("the root body index fits the profile");
    let child = builder
        .add_body(
            "child",
            shape,
            1.0,
            bevy::math::Isometry3d::from_translation(Vec3::Y),
        )
        .expect("the child body index fits the profile");
    let locked = AngleRange { min: 0.0, max: 0.0 };
    builder.add_joint(
        child,
        root,
        bevy::math::Isometry3d::from_translation(Vec3::Y),
        JointLimits {
            x: AngleRange {
                min: -1.0,
                max: 1.0,
            },
            twist: locked,
            z: locked,
        },
        10.0,
    );
    builder
        .build()
        .expect("the root and child form a valid profile tree")
}

/// Makes the single-body profile used by the missing-name case.
fn profile_with_missing_bone() -> RagdollProfile {
    let mut builder = ProfileBuilder::default();
    builder
        .add_body(
            "missing",
            ShapeSpec::Sphere {
                center: Vec3::ZERO,
                radius: 0.1,
            },
            1.0,
            bevy::math::Isometry3d::IDENTITY,
        )
        .expect("the single body index fits the profile");
    builder
        .build()
        .expect("a profile with one root body is valid")
}

/// Builds the headless app with one fixed update per call to `update`.
fn app() -> App {
    let mut app = App::new();
    app.add_plugins((
        MinimalPlugins,
        TransformPlugin,
        AssetPlugin::default(),
        AnimationPlugin,
        RagdollPlugin::default(),
        MockBackendPlugin,
    ));
    app.insert_resource(Time::<Fixed>::from_hz(60.0));
    app.insert_resource(TimeUpdateStrategy::FixedTimesteps(1));
    app
}

/// Adds a profile asset and a named two-bone skeleton to the app.
fn spawn_character_with_profile(
    app: &mut App,
    profile: RagdollProfile,
    mode: RagdollMode,
) -> (Entity, Entity, Entity) {
    let handle = app
        .world_mut()
        .resource_mut::<Assets<RagdollProfile>>()
        .add(profile);
    let character = app
        .world_mut()
        .spawn((Ragdoll::new(handle), mode, Transform::IDENTITY))
        .id();
    let root = app
        .world_mut()
        .spawn((Name::new("root"), Transform::IDENTITY, ChildOf(character)))
        .id();
    let child = app
        .world_mut()
        .spawn((
            Name::new("child"),
            Transform::from_translation(Vec3::Y),
            ChildOf(root),
        ))
        .id();
    (character, root, child)
}

/// Adds the standard two-body profile and its named skeleton.
fn spawn_character(app: &mut App, mode: RagdollMode) -> (Entity, Entity, Entity) {
    spawn_character_with_profile(app, profile(), mode)
}

/// Collects body entities owned by `character` in profile order.
fn body_entities(world: &mut World, character: Entity) -> Vec<(Entity, usize)> {
    let mut query = world.query::<(Entity, &RagdollBodyOf, &BodyIndex)>();
    let mut bodies: Vec<_> = query
        .iter(world)
        .filter(|(_, owner, _)| owner.0 == character)
        .map(|(entity, _, index)| (entity, index.get()))
        .collect();
    bodies.sort_by_key(|(_, index)| *index);
    bodies
}

#[test]
fn binding_finds_every_body_bone() {
    let mut app = app();
    let (character, _, _) = spawn_character(&mut app, RagdollMode::Dynamic);

    app.update();

    assert_eq!(body_entities(app.world_mut(), character).len(), 2);
}

#[test]
fn binding_reports_a_missing_bone() {
    let mut app = app();
    let (character, _, _) =
        spawn_character_with_profile(&mut app, profile_with_missing_bone(), RagdollMode::Dynamic);

    app.update();

    assert_eq!(
        app.world().get::<RagdollMode>(character),
        Some(&RagdollMode::Animated)
    );
    assert_eq!(
        app.world()
            .get::<bevy_ragdoll::runtime::RagdollError>(character),
        Some(&bevy_ragdoll::runtime::RagdollError::MissingBone(
            "missing".to_owned()
        ))
    );

    app.update();

    assert_eq!(
        app.world().get::<RagdollMode>(character),
        Some(&RagdollMode::Animated)
    );
}

#[test]
fn binding_waits_for_a_profile_asset_to_load() {
    let mut app = app();
    let character = app
        .world_mut()
        .spawn((
            Ragdoll::new(Handle::<RagdollProfile>::default()),
            RagdollMode::Dynamic,
            Transform::IDENTITY,
        ))
        .id();

    app.update();

    assert!(
        app.world()
            .get::<bevy_ragdoll::runtime::RagdollError>(character)
            .is_none()
    );
    assert!(body_entities(app.world_mut(), character).is_empty());
}

#[test]
fn binding_waits_for_skeleton_descendants() {
    let mut app = app();
    let handle = app
        .world_mut()
        .resource_mut::<Assets<RagdollProfile>>()
        .add(profile());
    let character = app
        .world_mut()
        .spawn((
            Ragdoll::new(handle),
            RagdollMode::Dynamic,
            Transform::IDENTITY,
        ))
        .id();

    app.update();

    assert!(
        app.world()
            .get::<bevy_ragdoll::runtime::RagdollError>(character)
            .is_none()
    );
    assert!(body_entities(app.world_mut(), character).is_empty());
}

#[test]
fn binding_uses_identity_for_a_bone_without_a_transform() {
    let mut app = app();
    let (character, root, _) = spawn_character(&mut app, RagdollMode::Dynamic);
    app.world_mut().entity_mut(root).remove::<Transform>();

    app.update();

    let root_body = body_entities(app.world_mut(), character)
        .into_iter()
        .find(|(_, index)| *index == 0)
        .map(|(entity, _)| entity)
        .expect("binding creates the body for the named root");
    let pose = app
        .world()
        .get::<BodyPhysicsPose>(root_body)
        .expect("body creation records its initial pose");
    assert!(
        pose.current
            .translation
            .abs_diff_eq(Vec3::ZERO.into(), 1.0e-5)
    );
}

#[test]
fn animated_mode_has_no_body_entities() {
    let mut app = app();
    let (character, _, _) = spawn_character(&mut app, RagdollMode::Animated);

    app.update();

    assert!(body_entities(app.world_mut(), character).is_empty());
}

#[test]
fn dynamic_mode_spawns_one_entity_per_body() {
    let mut app = app();
    let (character, _, _) = spawn_character(&mut app, RagdollMode::Dynamic);

    app.update();

    let bodies = body_entities(app.world_mut(), character);
    assert_eq!(bodies.len(), profile().bodies().len());
    for (body, index) in bodies {
        assert_eq!(app.world().get::<BodyKind>(body), Some(&BodyKind::Dynamic));
        assert!(app.world().get::<ChildOf>(body).is_none());
        assert_eq!(
            app.world().get::<BodyIndex>(body).map(|value| value.get()),
            Some(index)
        );
        assert!(app.world().get::<BodyShape>(body).is_some());
        assert!(app.world().get::<BodyMass>(body).is_some());
        assert!(app.world().get::<BodyVelocity>(body).is_some());
        assert!(app.world().get::<BodyPhysicsPose>(body).is_some());
        assert!(app.world().get::<BodyDriveOutput>(body).is_some());
        assert!(app.world().get::<BodyAtRest>(body).is_none());
    }
}

#[test]
fn off_center_impulse_changes_angular_velocity_about_body_center() {
    let mut app = app();
    let (character, _, _) = spawn_character(&mut app, RagdollMode::Dynamic);
    app.update();
    let body = body_entities(app.world_mut(), character)
        .first()
        .map(|(entity, _)| *entity)
        .expect("the profile has a root body");
    let center = app
        .world()
        .get::<Transform>(body)
        .expect("the body has a world transform")
        .translation;
    app.world_mut().write_message(RagdollImpulse {
        body,
        point: center + Vec3::Y,
        impulse: Vec3::X * 2.0,
    });

    app.update();

    let velocity = app
        .world()
        .get::<BodyVelocity>(body)
        .expect("the backend retains body velocity");
    assert!(velocity.angular.z < 0.0);
}

#[test]
fn returning_to_animated_despawns_bodies() {
    let mut app = app();
    let (character, _, _) = spawn_character(&mut app, RagdollMode::Dynamic);
    app.update();
    assert_eq!(body_entities(app.world_mut(), character).len(), 2);

    app.world_mut()
        .entity_mut(character)
        .insert(RagdollMode::Animated);
    app.update();

    assert!(body_entities(app.world_mut(), character).is_empty());
}

#[test]
fn bodies_spawn_at_the_target_pose_with_its_velocity() {
    let mut app = app();
    let (character, root, _) = spawn_character(&mut app, RagdollMode::Animated);
    app.update();
    app.world_mut()
        .get_mut::<Transform>(root)
        .expect("the root bone has a transform")
        .translation
        .x += 2.0 / 60.0;
    app.update();
    *app.world_mut()
        .get_mut::<Transform>(character)
        .expect("the character has a transform") = Transform::from_xyz(10.0, 2.0, 0.0)
        .with_rotation(bevy::math::Quat::from_rotation_z(
            std::f32::consts::FRAC_PI_2,
        ));

    app.world_mut()
        .entity_mut(character)
        .insert(RagdollMode::Dynamic);
    app.update();

    let bodies = body_entities(app.world_mut(), character);
    let (root_body, _) = bodies
        .first()
        .copied()
        .expect("the profile has a root body");
    let velocity = app
        .world()
        .get::<BodyVelocity>(root_body)
        .expect("the spawned body has its target velocity");
    assert!(velocity.linear.x.abs() < 0.02);
    assert!((velocity.linear.y - 2.0).abs() < 0.02);
    let pose = app
        .world()
        .get::<BodyPhysicsPose>(root_body)
        .expect("the spawned body has its world pose");
    assert!(
        pose.current
            .translation
            .abs_diff_eq(Vec3::new(10.0, 2.0 + 2.0 / 60.0, 0.0).into(), 1.0e-4,)
    );
}

#[test]
fn capture_reads_animated_locals_without_global_transform() {
    let mut app = app();
    let (character, root, _) = spawn_character(&mut app, RagdollMode::Animated);
    app.update();
    let index = BodyIndex::try_from(0_usize).expect("the root index is valid");
    let first = app
        .world()
        .get::<RagdollTargetPose>(character)
        .and_then(|target| target.current_pose(index))
        .expect("capture records the root pose");
    *app.world_mut()
        .get_mut::<GlobalTransform>(root)
        .expect("transform propagation created the root global transform") =
        GlobalTransform::from_translation(Vec3::splat(500.0));
    app.world_mut()
        .get_mut::<Transform>(character)
        .expect("the character has a transform")
        .translation
        .x = 100.0;
    app.update();
    let current = app
        .world()
        .get::<RagdollTargetPose>(character)
        .and_then(|target| target.current_pose(index))
        .expect("capture keeps the root pose");

    assert_eq!(first.translation.x, current.translation.x);
    assert!(current.translation.x.abs() < 1.0e-4);
}

#[test]
fn writeback_puts_each_body_bone_at_its_body_pose() {
    let mut app = app();
    let (character, root, child) = spawn_character(&mut app, RagdollMode::Dynamic);
    app.update();

    let mut body_query = app
        .world_mut()
        .query::<(Entity, &RagdollBodyOf, &BodyIndex)>();
    let body_bones: Vec<_> = body_query
        .iter(app.world())
        .filter(|(_, owner, _)| owner.0 == character)
        .map(|(body, _, index)| (index.get(), body))
        .collect();
    for (index, bone) in [(0, root), (1, child)] {
        let body = body_bones
            .iter()
            .find(|(body_index, _)| *body_index == index)
            .map(|(_, entity)| *entity)
            .expect("the profile body spawned");
        let body_pose = app
            .world()
            .get::<BodyPhysicsPose>(body)
            .expect("the backend wrote the body pose");
        let bone_world = app
            .world()
            .get::<GlobalTransform>(bone)
            .expect("transform propagation wrote the bone world pose");
        assert!(
            bone_world
                .translation()
                .abs_diff_eq(body_pose.current.translation.into(), 1.0e-4)
        );
    }
}

#[test]
fn bones_without_bodies_keep_their_animated_locals() {
    let mut app = app();
    let (_, root, child) = spawn_character(&mut app, RagdollMode::Dynamic);
    let spacer = app
        .world_mut()
        .spawn((
            Name::new("spacer"),
            Transform::from_xyz(0.0, 0.5, 0.0),
            ChildOf(root),
        ))
        .id();
    app.world_mut().entity_mut(child).insert(ChildOf(spacer));
    app.update();
    let animated_local = *app
        .world()
        .get::<Transform>(spacer)
        .expect("the unbound bone keeps a local transform");

    app.update();

    assert_eq!(app.world().get::<Transform>(spacer), Some(&animated_local));
}

#[test]
fn blend_zero_shows_animation_and_one_shows_physics_with_halfway_between() {
    for (blend, expected_x) in [(0.0, 2.0), (0.5, 6.0), (1.0, 10.0)] {
        let mut app = app();
        let (character, root, _) = spawn_character(&mut app, RagdollMode::Dynamic);
        app.world_mut()
            .entity_mut(character)
            .insert(RagdollBlend::new(blend));
        app.update();
        let root_body = body_entities(app.world_mut(), character)
            .first()
            .map(|(entity, _)| *entity)
            .expect("the profile has a root body");
        app.world_mut()
            .get_mut::<Transform>(root)
            .expect("the root bone has a transform")
            .translation
            .x = 2.0;
        let physics_pose = bevy::math::Isometry3d::from_translation(Vec3::new(10.0, 0.0, 0.0));
        {
            let mut body_pose = app
                .world_mut()
                .get_mut::<BodyPhysicsPose>(root_body)
                .expect("the backend created body pose history");
            body_pose.previous = physics_pose;
            body_pose.current = physics_pose;
        }
        app.world_mut().run_schedule(bevy::app::PostUpdate);

        let actual_x = app
            .world()
            .get::<Transform>(root)
            .expect("writeback updates the root bone")
            .translation
            .x;
        assert!((actual_x - expected_x).abs() < 1.0e-4);
    }
}

#[test]
fn interpolation_uses_overstep() {
    let previous = bevy::math::Isometry3d::IDENTITY;
    let current = bevy::math::Isometry3d::from_translation(Vec3::X);

    assert_eq!(
        interpolate_pose(previous, current, 0.0).translation,
        Vec3::ZERO.into()
    );
    assert_eq!(
        interpolate_pose(previous, current, 0.5).translation,
        (Vec3::X * 0.5).into()
    );
    assert_eq!(
        interpolate_pose(previous, current, 1.0).translation,
        Vec3::X.into()
    );
}

#[test]
fn budget_freezes_the_oldest_and_triggers_one_event() {
    let mut app = app();
    app.insert_resource(RagdollBudget::new(2));
    let (first, _, _) = spawn_character(&mut app, RagdollMode::Animated);
    let (second, _, _) = spawn_character(&mut app, RagdollMode::Animated);
    let (third, _, _) = spawn_character(&mut app, RagdollMode::Animated);
    app.update();
    let evictions = Arc::new(AtomicUsize::new(0));
    let count = Arc::clone(&evictions);
    app.world_mut().entity_mut(first).observe(
        move |_event: bevy::ecs::observer::On<RagdollBudgetEvicted>| {
            count.fetch_add(1, Ordering::Relaxed);
        },
    );

    for character in [first, second, third] {
        app.world_mut()
            .entity_mut(character)
            .insert(RagdollMode::Dynamic);
        app.update();
    }

    assert_eq!(
        app.world().get::<RagdollMode>(first),
        Some(&RagdollMode::Frozen)
    );
    assert_eq!(
        app.world().get::<RagdollMode>(second),
        Some(&RagdollMode::Dynamic)
    );
    assert_eq!(
        app.world().get::<RagdollMode>(third),
        Some(&RagdollMode::Dynamic)
    );
    assert_eq!(evictions.load(Ordering::Relaxed), 1);
}

#[test]
fn limp_dynamic_ragdolls_settle_once_and_freeze_when_configured() {
    for (should_freeze_when_settled, expected_mode) in
        [(false, RagdollMode::Dynamic), (true, RagdollMode::Frozen)]
    {
        let mut app = app();
        app.insert_resource(RagdollPhysicsSettings {
            settle_after: 2.0 / 60.0,
            settle_speed: 10.0,
            should_freeze_when_settled,
            ..Default::default()
        });
        let (character, _, _) = spawn_character(&mut app, RagdollMode::Dynamic);
        app.world_mut()
            .entity_mut(character)
            .insert(RagdollDrive::new(0.0, 0.0));
        let events = Arc::new(AtomicUsize::new(0));
        let event_count = Arc::clone(&events);
        app.world_mut().entity_mut(character).observe(
            move |_event: bevy::ecs::observer::On<RagdollSettled>| {
                event_count.fetch_add(1, Ordering::Relaxed);
            },
        );

        for _ in 0..8 {
            app.update();
        }

        assert_eq!(events.load(Ordering::Relaxed), 1);
        assert_eq!(
            app.world().get::<RagdollMode>(character),
            Some(&expected_mode)
        );
    }
}

#[test]
fn settling_resets_its_timer_when_any_body_speeds_up() {
    let mut app = app();
    app.insert_resource(RagdollPhysicsSettings {
        settle_after: 3.0 / 60.0,
        settle_speed: 2.0,
        ..Default::default()
    });
    let (character, _, _) = spawn_character(&mut app, RagdollMode::Dynamic);
    app.world_mut()
        .entity_mut(character)
        .insert(RagdollDrive::new(0.0, 0.0));
    app.update();
    let bodies = body_entities(app.world_mut(), character)
        .into_iter()
        .map(|(entity, _)| entity)
        .collect::<Vec<_>>();
    let events = Arc::new(AtomicUsize::new(0));
    let event_count = Arc::clone(&events);
    app.world_mut().entity_mut(character).observe(
        move |_event: bevy::ecs::observer::On<RagdollSettled>| {
            event_count.fetch_add(1, Ordering::Relaxed);
        },
    );

    for body in &bodies {
        app.world_mut()
            .get_mut::<BodyVelocity>(*body)
            .expect("dynamic bodies have velocity")
            .linear = Vec3::X * 10.0;
    }
    app.update();
    for body in &bodies {
        app.world_mut()
            .get_mut::<BodyVelocity>(*body)
            .expect("dynamic bodies have velocity")
            .linear = Vec3::ZERO;
    }
    app.update();
    for body in &bodies {
        app.world_mut()
            .get_mut::<BodyVelocity>(*body)
            .expect("dynamic bodies have velocity")
            .linear = Vec3::ZERO;
    }
    app.update();
    assert_eq!(events.load(Ordering::Relaxed), 0);

    for body in &bodies {
        app.world_mut()
            .get_mut::<BodyVelocity>(*body)
            .expect("dynamic bodies have velocity")
            .linear = Vec3::ZERO;
    }
    app.update();

    assert_eq!(events.load(Ordering::Relaxed), 1);
}

#[test]
fn drive_values_match_algorithms() {
    let settings = RagdollPhysicsSettings::default();
    let full = joint_motor_values(1.0, 10.0, &settings);
    let limp = joint_motor_values(0.0, 10.0, &settings);

    assert!((full.stiffness - 631.6547).abs() < 0.01);
    assert!((full.damping - 70.2655).abs() < 0.01);
    assert!((full.max_torque - 10.5).abs() < 1.0e-4);
    assert_eq!(limp.stiffness, 0.0);
    assert_eq!(limp.damping, 20.0);
    assert!((limp.max_torque - 0.5).abs() < 1.0e-4);
}

#[test]
fn dynamic_drive_system_writes_pin_force_and_fallback_joint_torque() {
    let mut app = app();
    let (character, root, child) = spawn_character(&mut app, RagdollMode::Dynamic);
    app.world_mut()
        .entity_mut(character)
        .insert((RagdollBlend::new(0.0), RagdollDrive::new(1.0, 1.0)));
    app.update();
    app.world_mut()
        .get_mut::<Transform>(root)
        .expect("root has an animated transform")
        .translation
        .x = 0.5;
    app.world_mut()
        .get_mut::<Transform>(child)
        .expect("child has an animated transform")
        .rotation = bevy::math::Quat::from_rotation_x(0.4);
    app.update();
    app.update();

    let bodies = body_entities(app.world_mut(), character);
    let root_body = bodies
        .iter()
        .find(|(_, index)| *index == 0)
        .map(|(entity, _)| *entity)
        .expect("the profile has a root body");
    let child_body = bodies
        .iter()
        .find(|(_, index)| *index == 1)
        .map(|(entity, _)| *entity)
        .expect("the profile has a child body");
    let root_output = app
        .world()
        .get::<BodyDriveOutput>(root_body)
        .expect("the drive system writes root forces");
    assert!(root_output.pin_force.x > 0.0);
    assert!(root_output.pin_force.length() <= 340.0 + 1.0e-4);
    let child_output = app
        .world()
        .get::<BodyDriveOutput>(child_body)
        .expect("the drive system writes child torques");
    assert!(child_output.joint_torque.length() > 0.0);
    assert!(
        app.world()
            .get::<bevy_ragdoll::runtime::body::JointDriveTarget>(child_body)
            .is_some_and(|target| target.stiffness > 0.0)
    );
}

#[test]
fn drive_clamps_inputs_to_unit_range() {
    let drive = RagdollDrive::new(-1.0, 2.0);
    assert_eq!(drive.muscle(), 0.0);
    assert_eq!(drive.pin(), 1.0);
    assert_eq!(RagdollDrive::new(0.0, 1.0).muscle(), 0.0);
    assert_eq!(RagdollDrive::new(1.0, 0.0).muscle(), 1.0);
    assert_eq!(RagdollDrive::new(f32::NAN, f32::INFINITY).muscle(), 0.0);
    assert_eq!(RagdollDrive::new(f32::NAN, f32::INFINITY).pin(), 0.0);

    let weights = RagdollBodyWeights::new(vec![BodyWeights::new(-1.0, 2.0)]);
    assert_eq!(
        weights.get(0).expect("the body weight is present").muscle(),
        0.0
    );
    assert_eq!(
        weights.get(0).expect("the body weight is present").pin(),
        1.0
    );
}
