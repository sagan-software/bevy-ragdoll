//! Public runtime behavior with the shared conformance mock backend.
//!
//! The tests drive characters through mode changes, binding, budgets, pins,
//! settling, and writeback in a headless app, so they cover the core runtime
//! without depending on a real physics engine or a window.

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
    // Two one-metre capsules stacked on Y.
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
    // The joint bends only about X, so tests can detect any twist or side swing.
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
    // Builder errors would mean the fixture itself is wrong.
    builder
        .build()
        .expect("the root and child form a valid profile tree")
}

/// Makes the single-body profile used by the missing-name case.
fn profile_with_missing_bone() -> RagdollProfile {
    // One body whose bone name no skeleton in these tests provides.
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
    // Headless plugins plus the mock backend, so the runtime runs without a physics engine.
    let mut app = App::new();
    app.add_plugins((
        MinimalPlugins,
        TransformPlugin,
        AssetPlugin::default(),
        AnimationPlugin,
        RagdollPlugin::default(),
        MockBackendPlugin,
    ));
    // One fixed step per update keeps every test frame-exact.
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
    // Register the profile so the character can reference it by handle.
    let handle = app
        .world_mut()
        .get_resource_mut::<Assets<RagdollProfile>>()
        .unwrap()
        .add(profile);
    let character = app
        .world_mut()
        .spawn((Ragdoll::new(handle), mode, Transform::IDENTITY))
        .id();
    // Bones named after the profile bodies, the child one metre above the root.
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
    // The profile names a bone that the skeleton does not have.
    let mut app = app();
    let (character, _, _) =
        spawn_character_with_profile(&mut app, profile_with_missing_bone(), RagdollMode::Dynamic);

    app.update();

    // Binding fails, keeps the character Animated, and reports the missing name.
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

    // A later frame must not retry into a different state.
    app.update();

    assert_eq!(
        app.world().get::<RagdollMode>(character),
        Some(&RagdollMode::Animated)
    );
}

#[test]
fn binding_waits_for_a_profile_asset_to_load() {
    let mut app = app();
    // The default handle never loads.
    let character = app
        .world_mut()
        .spawn((
            Ragdoll::new(Handle::<RagdollProfile>::default()),
            RagdollMode::Dynamic,
            Transform::IDENTITY,
        ))
        .id();

    app.update();

    // Binding waits quietly instead of reporting an error or creating bodies.
    assert!(
        app.world()
            .get::<bevy_ragdoll::runtime::RagdollError>(character)
            .is_none()
    );
    assert_eq!(body_entities(app.world_mut(), character), []);
}

#[test]
fn binding_waits_for_skeleton_descendants() {
    let mut app = app();
    // A loaded profile but no bones under the character yet.
    let handle = app
        .world_mut()
        .get_resource_mut::<Assets<RagdollProfile>>()
        .unwrap()
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

    // Binding waits for the skeleton instead of reporting an error or creating bodies.
    assert!(
        app.world()
            .get::<bevy_ragdoll::runtime::RagdollError>(character)
            .is_none()
    );
    assert_eq!(body_entities(app.world_mut(), character), []);
}

#[test]
fn binding_uses_identity_for_a_bone_without_a_transform() {
    let mut app = app();
    // A root bone without a Transform.
    let (character, root, _) = spawn_character(&mut app, RagdollMode::Dynamic);
    app.world_mut()
        .get_entity_mut(root)
        .unwrap()
        .remove::<Transform>();

    app.update();

    // Binding treats the missing local transform as identity.
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

    assert_eq!(body_entities(app.world_mut(), character), []);
}

/// The backend-contract components found on one body entity.
#[derive(Debug, PartialEq)]
struct BodyContract {
    /// The body's simulation kind.
    kind: Option<BodyKind>,
    /// The body's profile index.
    index: Option<usize>,
    /// The body's parent entity, if any.
    parent: Option<Entity>,
    /// The backend input and output components the body lacks.
    missing_components: Vec<&'static str>,
    /// Whether the body is marked at rest.
    is_at_rest: bool,
}

impl BodyContract {
    /// Reads the contract components of `body`.
    fn read(world: &World, body: Entity) -> Self {
        // Each body carries the shape, mass, motion, and drive data a backend reads.
        let presence = [
            ("BodyShape", world.get::<BodyShape>(body).is_some()),
            ("BodyMass", world.get::<BodyMass>(body).is_some()),
            ("BodyVelocity", world.get::<BodyVelocity>(body).is_some()),
            (
                "BodyPhysicsPose",
                world.get::<BodyPhysicsPose>(body).is_some(),
            ),
            (
                "BodyDriveOutput",
                world.get::<BodyDriveOutput>(body).is_some(),
            ),
        ];
        Self {
            kind: world.get::<BodyKind>(body).copied(),
            index: world.get::<BodyIndex>(body).map(|value| value.get()),
            parent: world.get::<ChildOf>(body).map(ChildOf::parent),
            missing_components: presence
                .into_iter()
                .filter(|(_, is_present)| !is_present)
                .map(|(name, _)| name)
                .collect(),
            is_at_rest: world.get::<BodyAtRest>(body).is_some(),
        }
    }

    /// The contract of a freshly spawned dynamic body at profile `index`.
    const fn fresh_dynamic(index: usize) -> Self {
        Self {
            kind: Some(BodyKind::Dynamic),
            index: Some(index),
            parent: None,
            missing_components: Vec::new(),
            is_at_rest: false,
        }
    }
}

#[test]
fn dynamic_mode_spawns_one_entity_per_body() {
    let mut app = app();
    // Switching to Dynamic happens on the first update.
    let (character, _, _) = spawn_character(&mut app, RagdollMode::Dynamic);

    app.update();

    // One top-level body per profile body, each carrying the full backend contract
    // and none at rest yet.
    let contracts: Vec<_> = body_entities(app.world_mut(), character)
        .into_iter()
        .map(|(body, _)| BodyContract::read(app.world(), body))
        .collect();
    let expected: Vec<_> = (0..profile().bodies().len())
        .map(BodyContract::fresh_dynamic)
        .collect();
    assert_eq!(contracts, expected);
}

#[test]
fn off_center_impulse_changes_angular_velocity_about_body_center() {
    let mut app = app();
    // Bind first so the body exists before the impulse is written.
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
    // Push +X at a point one metre above the centre.
    app.world_mut().write_message(RagdollImpulse {
        body,
        point: center + Vec3::Y,
        impulse: Vec3::X * 2.0,
    });

    app.update();

    // That off-centre push spins the body about -Z.
    let velocity = app
        .world()
        .get::<BodyVelocity>(body)
        .expect("the backend retains body velocity");
    assert!(velocity.angular.z < 0.0);
}

#[test]
fn returning_to_animated_despawns_bodies() {
    let mut app = app();
    // Bind as Dynamic and confirm both bodies exist.
    let (character, _, _) = spawn_character(&mut app, RagdollMode::Dynamic);
    app.update();
    assert_eq!(body_entities(app.world_mut(), character).len(), 2);

    app.world_mut()
        .get_entity_mut(character)
        .unwrap()
        .insert(RagdollMode::Animated);
    // Switching back to Animated removes them.
    app.update();

    assert_eq!(body_entities(app.world_mut(), character), []);
}

/// Sets the ragdoll mode of `character`.
fn set_mode(app: &mut App, character: Entity, mode: RagdollMode) {
    app.world_mut()
        .get_entity_mut(character)
        .unwrap()
        .insert(mode);
}

/// Returns the body entity at profile `index` for `character`.
fn body_at(app: &mut App, character: Entity, index: usize) -> Entity {
    body_entities(app.world_mut(), character)
        .into_iter()
        .find(|(_, body_index)| *body_index == index)
        .map(|(entity, _)| entity)
        .expect("the profile body spawned")
}

/// Spawns an Animated character whose root moved at 2 m/s along X last frame
/// and whose transform now turns that motion into world +Y.
fn spawn_moving_rotated_character(app: &mut App) -> Entity {
    // Animate the root at 2 m/s along X for one frame.
    let (character, root, _) = spawn_character(app, RagdollMode::Animated);
    app.update();
    app.world_mut()
        .get_mut::<Transform>(root)
        .expect("the root bone has a transform")
        .translation
        .x += 2.0 / 60.0;
    // Rotate the character a quarter turn, so the animated +X motion becomes world +Y.
    app.update();
    *app.world_mut()
        .get_mut::<Transform>(character)
        .expect("the character has a transform") = Transform::from_xyz(10.0, 2.0, 0.0)
        .with_rotation(bevy::math::Quat::from_rotation_z(
            std::f32::consts::FRAC_PI_2,
        ));
    character
}

#[test]
fn bodies_spawn_at_the_target_pose_with_its_velocity() {
    let mut app = app();
    let character = spawn_moving_rotated_character(&mut app);

    // Activate physics on the frame after the motion.
    set_mode(&mut app, character, RagdollMode::Dynamic);
    app.update();

    // The new body inherits the animated velocity and pose in world space.
    let root_body = body_at(&mut app, character, 0);
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
    // Record a first target pose while Animated.
    let (character, root, _) = spawn_character(&mut app, RagdollMode::Animated);
    app.update();
    let index = BodyIndex::try_from(0_usize).expect("the root index is valid");
    let first = app
        .world()
        .get::<RagdollTargetPose>(character)
        .and_then(|target| target.current_pose(index))
        // A stale GlobalTransform on the bone must not leak into capture.
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
    // Capture must follow the character's local transform change instead.
    app.update();
    let current = app
        .world()
        .get::<RagdollTargetPose>(character)
        .and_then(|target| target.current_pose(index))
        .expect("capture keeps the root pose");

    assert_eq!(first.translation.x, current.translation.x);
    assert!(current.translation.x.abs() < 1.0e-4);
}

/// Binds a Dynamic character and returns the world translation of the bone
/// for profile `index` together with its body's physics translation.
fn bone_and_body_translation(index: usize) -> (Vec3, Vec3) {
    // Bind a Dynamic character so writeback runs once.
    let mut app = app();
    let (character, root, child) = spawn_character(&mut app, RagdollMode::Dynamic);
    app.update();
    // Profile index 0 names the root bone and index 1 names the child bone.
    let bone = if index == 0 { root } else { child };
    let body = body_at(&mut app, character, index);
    // Compare the bone world pose with the body pose writeback read.
    let body_pose = app
        .world()
        .get::<BodyPhysicsPose>(body)
        .expect("the backend wrote the body pose");
    let bone_world = app
        .world()
        .get::<GlobalTransform>(bone)
        .expect("transform propagation wrote the bone world pose");
    (
        bone_world.translation(),
        body_pose.current.translation.into(),
    )
}

#[test]
fn writeback_puts_the_root_bone_at_its_body_pose() {
    let (bone, body) = bone_and_body_translation(0);

    assert!(bone.abs_diff_eq(body, 1.0e-4));
}

#[test]
fn writeback_puts_the_child_bone_at_its_body_pose() {
    let (bone, body) = bone_and_body_translation(1);

    assert!(bone.abs_diff_eq(body, 1.0e-4));
}

#[test]
fn bones_without_bodies_keep_their_animated_locals() {
    let mut app = app();
    let (_, root, child) = spawn_character(&mut app, RagdollMode::Dynamic);
    // Insert a bone with no profile body between the root and child bones.
    let spacer = app
        .world_mut()
        .spawn((
            Name::new("spacer"),
            Transform::from_xyz(0.0, 0.5, 0.0),
            ChildOf(root),
        ))
        .id();
    app.world_mut()
        .get_entity_mut(child)
        .unwrap()
        .insert(ChildOf(spacer));
    // Record the spacer's animated local transform after binding.
    app.update();
    let animated_local = *app
        .world()
        .get::<Transform>(spacer)
        .expect("the unbound bone keeps a local transform");

    app.update();

    // Writeback must leave the unmapped bone's local transform alone.
    assert_eq!(app.world().get::<Transform>(spacer), Some(&animated_local));
}

/// Pins both the previous and current physics pose of `body` to `pose`.
fn pin_physics_pose(app: &mut App, body: Entity, pose: bevy::math::Isometry3d) {
    let mut body_pose = app
        .world_mut()
        .get_mut::<BodyPhysicsPose>(body)
        .expect("the backend created body pose history");
    body_pose.previous = pose;
    body_pose.current = pose;
}

/// Runs writeback with `blend` when animation puts the root at x = 2 and
/// physics puts it at x = 10, then returns the root bone's x.
fn blended_root_x(blend: f32) -> f32 {
    let mut app = app();
    let (character, root, _) = spawn_character(&mut app, RagdollMode::Dynamic);
    app.world_mut()
        .get_entity_mut(character)
        .unwrap()
        // Bind a Dynamic character with the blend under test.
        .insert(RagdollBlend::new(blend));
    app.update();
    let root_body = body_at(&mut app, character, 0);
    app.world_mut()
        .get_mut::<Transform>(root)
        .expect("the root bone has a transform")
        // Animation puts the root at x = 2.
        .translation
        .x = 2.0;
    // Pin both physics poses so interpolation cannot move the result.
    let physics_pose = bevy::math::Isometry3d::from_translation(Vec3::new(10.0, 0.0, 0.0));
    pin_physics_pose(&mut app, root_body, physics_pose);
    // Writeback runs in PostUpdate and blends the two poses.
    app.world_mut()
        .try_run_schedule(bevy::app::PostUpdate)
        .expect("the ragdoll plugin installs PostUpdate");
    app.world()
        .get::<Transform>(root)
        .expect("writeback updates the root bone")
        .translation
        .x
}

#[test]
fn blend_zero_shows_animation() {
    assert!((blended_root_x(0.0) - 2.0).abs() < 1.0e-4);
}

#[test]
fn blend_half_lands_halfway_between_animation_and_physics() {
    assert!((blended_root_x(0.5) - 6.0).abs() < 1.0e-4);
}

#[test]
fn blend_one_shows_physics() {
    assert!((blended_root_x(1.0) - 10.0).abs() < 1.0e-4);
}

#[test]
fn interpolation_uses_overstep() {
    let previous = bevy::math::Isometry3d::IDENTITY;
    let current = bevy::math::Isometry3d::from_translation(Vec3::X);

    // Overstep 0, 0.5, and 1 select the previous pose, the midpoint, and the current pose.
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

/// Observes `E` on `entity` and returns the shared count of triggered events.
fn count_events<E: bevy::ecs::event::EntityEvent>(
    app: &mut App,
    entity: Entity,
) -> Arc<AtomicUsize> {
    let events = Arc::new(AtomicUsize::new(0));
    let count = Arc::clone(&events);
    app.world_mut().get_entity_mut(entity).unwrap().observe(
        move |_event: bevy::ecs::observer::On<'_, '_, E>| {
            count.fetch_add(1, Ordering::Relaxed);
        },
    );
    events
}

#[test]
fn budget_freezes_the_oldest_and_triggers_one_event() {
    let mut app = app();
    // A budget of two dynamic ragdolls and three characters.
    app.insert_resource(RagdollBudget::new(2));
    let (first, _, _) = spawn_character(&mut app, RagdollMode::Animated);
    let (second, _, _) = spawn_character(&mut app, RagdollMode::Animated);
    let (third, _, _) = spawn_character(&mut app, RagdollMode::Animated);
    app.update();
    // Count eviction events observed on the first character.
    let evictions = count_events::<RagdollBudgetEvicted>(&mut app, first);

    // Activate the characters one per frame, oldest first.
    set_mode(&mut app, first, RagdollMode::Dynamic);
    app.update();
    set_mode(&mut app, second, RagdollMode::Dynamic);
    app.update();
    set_mode(&mut app, third, RagdollMode::Dynamic);
    app.update();

    // The oldest is frozen once, with exactly one event, and the others stay dynamic.
    let modes =
        [first, second, third].map(|character| app.world().get::<RagdollMode>(character).copied());
    assert_eq!(
        modes,
        [
            Some(RagdollMode::Frozen),
            Some(RagdollMode::Dynamic),
            Some(RagdollMode::Dynamic)
        ]
    );
    assert_eq!(evictions.load(Ordering::Relaxed), 1);
}

/// Sets the settle window and speed threshold, and optionally freezing.
fn configure_settling(app: &mut App, settle_after: f32, settle_speed: f32, should_freeze: bool) {
    let mut settings = app
        .world_mut()
        .get_resource_mut::<RagdollPhysicsSettings>()
        .expect("the ragdoll plugin installs physics settings");
    settings.settle_after = settle_after;
    settings.settle_speed = settle_speed;
    settings.should_freeze_when_settled = should_freeze;
}

/// Spawns a Dynamic character with zero muscle and pin, so settling is the
/// only behavior that changes its mode.
fn spawn_limp_character(app: &mut App) -> Entity {
    let (character, _, _) = spawn_character(app, RagdollMode::Dynamic);
    app.world_mut()
        .get_entity_mut(character)
        .unwrap()
        .insert(RagdollDrive::new(0.0, 0.0));
    character
}

/// Lets a limp ragdoll settle and returns its settle-event count and final mode.
fn settle_limp_ragdoll(should_freeze_when_settled: bool) -> (usize, Option<RagdollMode>) {
    // A short settle window and a generous speed threshold settle a limp ragdoll quickly.
    let mut app = app();
    configure_settling(&mut app, 2.0 / 60.0, 10.0, should_freeze_when_settled);
    let character = spawn_limp_character(&mut app);
    let events = count_events::<RagdollSettled>(&mut app, character);

    // Eight frames are well past the two-frame settle window.
    for _ in 0..8 {
        app.update();
    }

    (
        events.load(Ordering::Relaxed),
        app.world().get::<RagdollMode>(character).copied(),
    )
}

#[test]
fn limp_dynamic_ragdolls_settle_once_and_stay_dynamic_by_default() {
    assert_eq!(settle_limp_ragdoll(false), (1, Some(RagdollMode::Dynamic)));
}

#[test]
fn limp_dynamic_ragdolls_settle_once_and_freeze_when_configured() {
    assert_eq!(settle_limp_ragdoll(true), (1, Some(RagdollMode::Frozen)));
}

/// Sets the linear velocity of every body in `bodies`.
fn set_linear_velocity(app: &mut App, bodies: &[Entity], linear: Vec3) {
    for body in bodies {
        app.world_mut()
            .get_mut::<BodyVelocity>(*body)
            .expect("dynamic bodies have velocity")
            .linear = linear;
    }
}

#[test]
fn settling_resets_its_timer_when_any_body_speeds_up() {
    let mut app = app();
    // A three-frame settle window with a 2 m/s threshold.
    configure_settling(&mut app, 3.0 / 60.0, 2.0, false);
    let character = spawn_limp_character(&mut app);
    app.update();
    let bodies = body_entities(app.world_mut(), character)
        .into_iter()
        .map(|(entity, _)| entity)
        .collect::<Vec<_>>();
    let events = count_events::<RagdollSettled>(&mut app, character);

    // One fast frame, then two slow frames: not yet three slow frames in a row.
    set_linear_velocity(&mut app, &bodies, Vec3::X * 10.0);
    app.update();
    set_linear_velocity(&mut app, &bodies, Vec3::ZERO);
    app.update();
    set_linear_velocity(&mut app, &bodies, Vec3::ZERO);
    app.update();
    // The fast frame restarted the timer, so nothing has settled yet.
    assert_eq!(events.load(Ordering::Relaxed), 0);

    // A third slow frame completes the window.
    set_linear_velocity(&mut app, &bodies, Vec3::ZERO);
    app.update();

    assert_eq!(events.load(Ordering::Relaxed), 1);
}

#[test]
fn full_muscle_drive_values_match_critical_damping() {
    // Full muscle at 10 N m joint strength.
    let full = joint_motor_values(1.0, 10.0, &RagdollPhysicsSettings::default());

    assert!((full.stiffness - 631.6547).abs() < 0.01);
    assert!((full.damping - 70.2655).abs() < 0.01);
    assert!((full.max_torque - 10.5).abs() < 1.0e-4);
}

#[test]
fn limp_drive_values_keep_damping_and_a_small_torque() {
    // Limp muscle at the same 10 N m joint strength.
    let limp = joint_motor_values(0.0, 10.0, &RagdollPhysicsSettings::default());

    assert_eq!((limp.stiffness, limp.damping), (0.0, 20.0));
    assert!((limp.max_torque - 0.5).abs() < 1.0e-4);
}

/// Drives a full-muscle, full-pin character toward a moved root target and a
/// bent child target, then returns its root and child body entities.
fn driven_bodies(app: &mut App) -> (Entity, Entity) {
    let (character, root, child) = spawn_character(app, RagdollMode::Dynamic);
    // Full muscle and pin with the physics view hidden behind animation.
    app.world_mut()
        .get_entity_mut(character)
        .unwrap()
        .insert((RagdollBlend::new(0.0), RagdollDrive::new(1.0, 1.0)));
    app.update();
    // Move the root target and bend the child target so both drives have work.
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
    (body_at(app, character, 0), body_at(app, character, 1))
}

#[test]
fn dynamic_drive_system_pins_the_root_toward_its_target_within_the_force_cap() {
    let mut app = app();
    // Drive the root toward a target moved along +X.
    let (root_body, _) = driven_bodies(&mut app);

    let root_output = app
        .world()
        .get::<BodyDriveOutput>(root_body)
        .expect("the drive system writes root forces");
    // The pin pushes toward +X within the 340 N force cap.
    assert!(root_output.pin_force.x > 0.0);
    assert!(root_output.pin_force.length() <= 340.0 + 1.0e-4);
}

#[test]
fn dynamic_drive_system_writes_fallback_joint_torque() {
    let mut app = app();
    // Drive the child toward a bent target.
    let (_, child_body) = driven_bodies(&mut app);

    let child_output = app
        .world()
        .get::<BodyDriveOutput>(child_body)
        .expect("the drive system writes child torques");
    // Without a backend motor, the joint gets fallback torque and a stiff target.
    assert!(child_output.joint_torque.length() > 0.0);
    assert!(
        app.world()
            .get::<bevy_ragdoll::runtime::body::JointDriveTarget>(child_body)
            .is_some_and(|target| target.stiffness > 0.0)
    );
}

#[test]
fn drive_clamps_out_of_range_inputs_to_unit_range() {
    let drive = RagdollDrive::new(-1.0, 2.0);

    assert_eq!((drive.muscle(), drive.pin()), (0.0, 1.0));
}

#[test]
fn drive_keeps_unit_range_endpoints() {
    let muscles = [
        RagdollDrive::new(0.0, 1.0).muscle(),
        RagdollDrive::new(1.0, 0.0).muscle(),
    ];

    assert_eq!(muscles, [0.0, 1.0]);
}

#[test]
fn drive_turns_non_finite_inputs_into_zero() {
    let drive = RagdollDrive::new(f32::NAN, f32::INFINITY);

    assert_eq!((drive.muscle(), drive.pin()), (0.0, 0.0));
}

#[test]
fn body_weights_clamp_inputs_to_unit_range() {
    let weights = RagdollBodyWeights::new(vec![BodyWeights::new(-1.0, 2.0)]);
    let weight = weights.get(0).expect("the body weight is present");

    assert_eq!((weight.muscle(), weight.pin()), (0.0, 1.0));
}
