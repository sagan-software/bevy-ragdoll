//! Reusable public checks shared by every backend conformance suite.
//!
//! Each check creates a minimal headless Bevy app with fixed time, inserts a
//! two-body profile, and invokes one backend plugin through [`AddBackend`]. The
//! functions assert observable behavior at the shared component and message
//! boundary, so backend crates can run identical cases without naming
//! engine-specific resources, query parameters, or solver types.

use bevy::asset::{AssetPlugin, Assets};
use bevy::ecs::message::Messages;
use bevy::math::{Isometry3d, Vec3};
use bevy::prelude::{
    AnimationPlugin, App, ChildOf, Entity, MinimalPlugins, Name, Transform, TransformPlugin, World,
};
use bevy::time::{Fixed, Time, TimeUpdateStrategy};
use bevy_ragdoll::profile::{AngleRange, BodyIndex, ProfileBuilder};
use bevy_ragdoll::runtime::RagdollPlugin;
use bevy_ragdoll::runtime::body::{
    BodyKind, BodyMass, BodyPhysicsPose, BodyVelocity, JointToParent,
};
use bevy_ragdoll::runtime::components::{Ragdoll, RagdollBodyOf, RagdollMode};
use bevy_ragdoll::runtime::messages::{
    RagdollImpulse, RagdollRaycast, RagdollRaycastResponse, RagdollRequestId,
};
use bevy_ragdoll::{JointLimits, RagdollProfile, ShapeSpec};

/// Function pointer used to add one backend plugin to each fresh contract-test
/// app.
///
/// The harness adds Bevy's minimal, transform, asset, and animation plugins
/// plus `RagdollPlugin` before calling this function. A backend can therefore
/// read the configured fixed schedule and register its systems without
/// replacing shared app setup.
///
/// # Examples
///
/// ```
/// use bevy::prelude::App; use bevy_ragdoll_conformance::contract::AddBackend;
/// use bevy_ragdoll_conformance::mock::MockBackendPlugin;
///
/// fn add_mock_backend(app: &mut App) { app.add_plugins(MockBackendPlugin); }
///
/// let add_backend: AddBackend = add_mock_backend;
/// ```
pub type AddBackend = fn(&mut App);

/// Builds a two-body capsule profile with one parent constraint.
fn profile() -> RagdollProfile {
    let mut builder = ProfileBuilder::default();
    // Give the contract one root and one child with valid capsule geometry and mass.
    let shape = ShapeSpec::Capsule {
        a: Vec3::ZERO,
        b: Vec3::Y,
        radius: 0.25,
    };
    let root = builder
        .add_body("root", shape, 2.0, Isometry3d::IDENTITY)
        .expect("the root fits the profile");
    let child = builder
        .add_body(
            "child",
            shape,
            1.0,
            Isometry3d::from_translation(Vec3::new(0.0, 2.0, 0.0)),
        )
        .expect("the child fits the profile");
    let locked = AngleRange { min: 0.0, max: 0.0 };
    // Connect the child to the root so the suite can inspect joint construction.
    builder.add_joint(
        child,
        root,
        Isometry3d::from_translation(Vec3::new(0.0, 2.0, 0.0)),
        JointLimits {
            x: locked,
            twist: locked,
            z: locked,
        },
        10.0,
    );
    builder.build().expect("the two-body tree is valid")
}

/// Creates the headless app and adds the selected backend after the core.
fn app(add_backend: AddBackend) -> App {
    let mut app = App::new();
    // Install only headless plugins required for assets, animation, transforms, and scheduling.
    app.add_plugins((
        MinimalPlugins,
        TransformPlugin,
        AssetPlugin::default(),
        AnimationPlugin,
    ));
    app.insert_resource(Time::<Fixed>::from_hz(60.0));
    app.insert_resource(TimeUpdateStrategy::FixedTimesteps(1));
    // Add the core first so each backend can read its fixed schedule resource.
    app.add_plugins(RagdollPlugin::default());
    add_backend(&mut app);
    app
}

/// Spawns a named two-bone character and advances binding once.
fn spawn_character(app: &mut App, mode: RagdollMode) -> (Entity, Entity, Entity) {
    // Store the validated contract profile as an app asset before attaching its handle.
    let handle = app
        .world_mut()
        .get_resource_mut::<Assets<RagdollProfile>>()
        .expect("RagdollPlugin initializes profile assets")
        .add(profile());
    let character = app
        .world_mut()
        .spawn((Ragdoll::new(handle), mode, Transform::IDENTITY))
        .id();
    // Create exact profile bone names so binding can complete on the next update.
    let root = app
        .world_mut()
        .spawn((Name::new("root"), Transform::IDENTITY, ChildOf(character)))
        .id();
    let child = app
        .world_mut()
        .spawn((
            Name::new("child"),
            Transform::from_translation(Vec3::new(0.0, 2.0, 0.0)),
            ChildOf(root),
        ))
        .id();
    app.update();
    (character, root, child)
}

/// Collects related body entities in validated profile order.
fn body_entities(world: &mut World, character: Entity) -> Vec<(Entity, usize)> {
    let mut query = world.query::<(Entity, &RagdollBodyOf, &BodyIndex)>();
    let mut bodies = query
        .iter(world)
        .filter(|(_, owner, _)| owner.0 == character)
        .map(|(entity, _, index)| (entity, index.get()))
        .collect::<Vec<_>>();
    bodies.sort_unstable_by_key(|(_, index)| *index);
    bodies
}

/// Reads one body in validated profile order from the fixed two-body contract
/// profile.
fn body_at(bodies: &[(Entity, usize)], index: usize) -> (Entity, usize) {
    bodies
        .get(index)
        .copied()
        .expect("the conformance profile contains two bodies")
}

/// Confirms that one spawned entity carries its indexed motion and mass state.
fn assert_profile_body(world: &World, body: Entity, index: usize) {
    assert_eq!(
        world.get::<BodyIndex>(body).map(|value| value.get()),
        Some(index)
    );
    assert!(world.get::<BodyKind>(body).is_some());
    assert!(world.get::<BodyMass>(body).is_some());
}

/// Adds the parent relationship and animation transform used by the nested target test.
fn move_nested_kinematic_target(app: &mut App, character: Entity, root: Entity) {
    let ancestor = app
        .world_mut()
        .spawn(Transform::from_xyz(9.0, 1.0, 0.0))
        .id();
    app.world_mut()
        .get_entity_mut(character)
        .expect("the character remains alive")
        .insert(ChildOf(ancestor));
    *app.world_mut()
        .get_mut::<Transform>(character)
        .expect("the character has a transform") = Transform::from_xyz(1.0, 0.0, 0.0)
        .with_rotation(bevy::math::Quat::from_rotation_z(
            std::f32::consts::FRAC_PI_2,
        ));
    app.world_mut()
        .get_mut::<Transform>(root)
        .expect("root bone has a transform")
        .translation
        .x = 0.5;
}

/// Checks that activation creates every profile body and each parent-child
/// joint.
///
/// The assertion verifies profile indexes, motion kinds, and masses on spawned
/// bodies. It also counts child constraints, so a backend cannot pass by
/// creating only the visible body entities.
///
/// # Examples
///
/// ```
/// # use bevy::prelude::App;
/// # use bevy_ragdoll_conformance::mock::MockBackendPlugin;
/// # fn add_mock(app: &mut App) { app.add_plugins(MockBackendPlugin); }
/// bevy_ragdoll_conformance::contract::bodies_and_joints_exist_for_each_profile_entry(add_mock);
/// ```
pub fn bodies_and_joints_exist_for_each_profile_entry(add_backend: AddBackend) {
    // Activate the profile before counting entities and constraints created by the backend.
    let mut app = app(add_backend);
    let (character, _, _) = spawn_character(&mut app, RagdollMode::Dynamic);
    let bodies = body_entities(app.world_mut(), character);
    // Count profile constraints independently from relationship-target body discovery.
    let joint_count = {
        let mut query = app.world_mut().query::<&JointToParent>();
        query.iter(app.world()).count()
    };

    assert_eq!(bodies.len(), profile().bodies().len());
    assert_eq!(joint_count, profile().joints().len());
    // Validate the public component contract after matching entity counts to profile entries.
    for (body, index) in bodies {
        assert_profile_body(app.world(), body, index);
    }
}

/// Checks that a point impulse changes linear momentum by its magnitude within
/// tolerance.
///
/// The test measures the X component so gravity does not affect the result, and
/// it compares mass times velocity change with the requested `10 N·s` impulse
/// to within `0.1 N·s`.
///
/// # Examples
///
/// ```
/// # use bevy::prelude::App;
/// # use bevy_ragdoll_conformance::mock::MockBackendPlugin;
/// # fn add_mock(app: &mut App) { app.add_plugins(MockBackendPlugin); }
/// bevy_ragdoll_conformance::contract::an_impulse_changes_momentum_by_its_size(add_mock);
/// ```
pub fn an_impulse_changes_momentum_by_its_size(add_backend: AddBackend) {
    // Measure initial mass and velocity before sending the one-step impulse message.
    let mut app = app(add_backend);
    let (character, _, _) = spawn_character(&mut app, RagdollMode::Dynamic);
    let (body, _) = body_at(&body_entities(app.world_mut(), character), 0);
    let mass = app
        .world()
        .get::<BodyMass>(body)
        .expect("body has mass")
        .mass;
    let initial_velocity = app
        .world()
        .get::<BodyVelocity>(body)
        .expect("body has velocity")
        .linear;
    app.world_mut().write_message(RagdollImpulse {
        body,
        point: Vec3::ZERO,
        impulse: Vec3::new(10.0, 0.0, 0.0),
    });
    // Read velocity after the backend completes the fixed integration step.
    app.update();
    let final_velocity = app
        .world()
        .get::<BodyVelocity>(body)
        .expect("backend reads velocity back")
        .linear;

    let momentum_change = mass * (final_velocity.x - initial_velocity.x);
    assert!((momentum_change - 10.0).abs() <= 0.1);
}

/// Checks that each fixed step reads the new pose and velocity back into shared
/// components.
///
/// A falling child must advance its previous pose, move downward, and report
/// downward velocity while retaining its validated profile index for later
/// drive and query lookups.
///
/// # Examples
///
/// ```
/// # use bevy::prelude::App;
/// # use bevy_ragdoll_conformance::mock::MockBackendPlugin;
/// # fn add_mock(app: &mut App) { app.add_plugins(MockBackendPlugin); }
/// bevy_ragdoll_conformance::contract::pose_and_velocity_are_read_back_every_step(add_mock);
/// ```
pub fn pose_and_velocity_are_read_back_every_step(add_backend: AddBackend) {
    // Capture the body pose written at activation before advancing physics once.
    let mut app = app(add_backend);
    let (character, _, _) = spawn_character(&mut app, RagdollMode::Dynamic);
    let (body, index) = body_at(&body_entities(app.world_mut(), character), 1);
    let before = app
        .world()
        .get::<BodyPhysicsPose>(body)
        .expect("spawn writes an initial pose")
        .current;
    // Let the backend integrate gravity and publish the completed body state.
    app.update();
    // Read both components after the fixed stage so this checks the completed step.
    let pose = app
        .world()
        .get::<BodyPhysicsPose>(body)
        .expect("backend writes pose history after each step");
    let velocity = app
        .world()
        .get::<BodyVelocity>(body)
        .expect("backend writes velocity after each step");

    assert_eq!(pose.previous, before);
    assert!(pose.current.translation.y < pose.previous.translation.y);
    assert!(velocity.linear.y < 0.0);
    assert_eq!(
        app.world().get::<BodyIndex>(body).map(|value| value.get()),
        Some(index)
    );
}

/// Checks that kinematic bodies follow target poses through translated and
/// rotated ancestors.
///
/// The test moves the character below a transformed parent, changes one bone,
/// and allows two fixed updates for capture and following. The resulting body
/// translation must remain within one millimetre of the composed target.
///
/// # Examples
///
/// ```
/// # use bevy::prelude::App;
/// # use bevy_ragdoll_conformance::mock::MockBackendPlugin;
/// # fn add_mock(app: &mut App) { app.add_plugins(MockBackendPlugin); }
/// bevy_ragdoll_conformance::contract::kinematic_bodies_follow_targets_exactly(add_mock);
/// ```
pub fn kinematic_bodies_follow_targets_exactly(add_backend: AddBackend) {
    // Build a kinematic character before adding a parent transform and changing its target.
    let mut app = app(add_backend);
    let (character, root, _) = spawn_character(&mut app, RagdollMode::Kinematic);
    move_nested_kinematic_target(&mut app, character, root);
    // Capture the animated change, then let the backend follow the resulting target.
    app.update();
    app.update();
    let (body, _) = body_at(&body_entities(app.world_mut(), character), 0);
    let pose = app
        .world()
        .get::<BodyPhysicsPose>(body)
        .expect("backend reads the kinematic target back");

    assert!(
        pose.current
            .translation
            .abs_diff_eq(Vec3::new(10.0, 1.5, 0.0).into(), 0.001,)
    );
}

/// Checks that frozen bodies retain their physics pose when an impulse message
/// arrives.
///
/// The backend must ignore dynamic integration for `Fixed` body kinds even when
/// the message names a live body and carries nonzero momentum.
///
/// # Examples
///
/// ```
/// # use bevy::prelude::App;
/// # use bevy_ragdoll_conformance::mock::MockBackendPlugin;
/// # fn add_mock(app: &mut App) { app.add_plugins(MockBackendPlugin); }
/// bevy_ragdoll_conformance::contract::frozen_bodies_do_not_move_under_impulses(add_mock);
/// ```
pub fn frozen_bodies_do_not_move_under_impulses(add_backend: AddBackend) {
    // Snapshot a fixed body's pose before publishing a nonzero impulse.
    let mut app = app(add_backend);
    let (character, _, _) = spawn_character(&mut app, RagdollMode::Frozen);
    let (body, _) = body_at(&body_entities(app.world_mut(), character), 0);
    let before = app
        .world()
        .get::<BodyPhysicsPose>(body)
        .expect("spawn writes an initial pose")
        .current;
    app.world_mut().write_message(RagdollImpulse {
        body,
        point: Vec3::ZERO,
        impulse: Vec3::X * 10.0,
    });
    // Advance one complete fixed stage and compare the exact rigid pose.
    app.update();
    let after = app
        .world()
        .get::<BodyPhysicsPose>(body)
        .expect("fixed body retains its pose")
        .current;

    assert_eq!(after, before);
}

/// Checks that a backend ray query reports the body entity it intersects.
///
/// The request carries caller-owned identity and travels through shared request
/// and response messages; the assertion matches the returned ragdoll body
/// rather than backend query state.
///
/// # Examples
///
/// ```
/// # use bevy::prelude::App;
/// # use bevy_ragdoll_conformance::mock::MockBackendPlugin;
/// # fn add_mock(app: &mut App) { app.add_plugins(MockBackendPlugin); }
/// bevy_ragdoll_conformance::contract::raycast_reports_the_body_hit(add_mock);
/// ```
pub fn raycast_reports_the_body_hit(add_backend: AddBackend) {
    // Subscribe before sending the request so the response reader observes this fixed step.
    let mut app = app(add_backend);
    let (character, _, _) = spawn_character(&mut app, RagdollMode::Dynamic);
    let (expected_body, _) = body_at(&body_entities(app.world_mut(), character), 0);
    let mut cursor = app
        .world()
        .get_resource::<Messages<RagdollRaycastResponse>>()
        .expect("RagdollPlugin registers raycast responses")
        .get_cursor();
    app.world_mut().write_message(RagdollRaycast {
        request_id: RagdollRequestId::new(91),
        origin: Vec3::new(-2.0, 0.5, 0.0),
        direction: Vec3::X,
        max_distance: 4.0,
        filter: None,
    });
    // Read the matching response after the backend processes raycasts in its read stage.
    app.update();
    let response = cursor
        .read(
            app.world()
                .get_resource::<Messages<RagdollRaycastResponse>>()
                .expect("RagdollPlugin retains raycast responses"),
        )
        .find(|response| response.request_id.get() == 91)
        .copied()
        .expect("the backend answers the ray request");

    assert_eq!(response.hit.and_then(|hit| hit.body), Some(expected_body));
}

/// Checks that despawning the character removes every physics body in its
/// relationship target.
///
/// The assertion first confirms the profile spawned its full body set, then
/// removes the owner and verifies every captured body entity is gone from the
/// world.
///
/// # Examples
///
/// ```
/// # use bevy::prelude::App;
/// # use bevy_ragdoll_conformance::mock::MockBackendPlugin;
/// # fn add_mock(app: &mut App) { app.add_plugins(MockBackendPlugin); }
/// bevy_ragdoll_conformance::contract::despawning_the_character_removes_every_body(add_mock);
/// ```
pub fn despawning_the_character_removes_every_body(add_backend: AddBackend) {
    // Confirm all expected bodies exist before despawning the relationship owner.
    let mut app = app(add_backend);
    let (character, _, _) = spawn_character(&mut app, RagdollMode::Dynamic);
    let bodies = body_entities(app.world_mut(), character);
    assert_eq!(bodies.len(), profile().bodies().len());

    app.world_mut().despawn(character);

    // Check each captured entity after relationship cleanup has completed.
    bodies.iter().for_each(|(body, _)| {
        assert!(app.world().get_entity(*body).is_err());
    });
}
