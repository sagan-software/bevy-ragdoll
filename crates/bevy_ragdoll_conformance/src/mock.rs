//! Semi-implicit Euler backend used by runtime and contract tests.
//!
//! The mock follows kinematic target poses, integrates dynamic forces and
//! impulses, clamps bodies against a plane at `y = 0`, and answers shared
//! raycast messages. It deliberately ignores joints and contact response beyond
//! that plane, which keeps it useful for contract checks without implying that
//! its disconnected bodies form a physical ragdoll.

use std::collections::HashMap;
use std::collections::HashSet;

use bevy::ecs::schedule::IntoScheduleConfigs;
use bevy::ecs::system::SystemParam;
use bevy::math::{Affine3A, Isometry3d, Quat, Vec3};
use bevy::prelude::{
    App, ChildOf, Commands, Entity, MessageReader, MessageWriter, ParamSet, Plugin, Query, Res,
    Transform, With,
};
use bevy::time::{Fixed, Time};

use bevy_ragdoll::ShapeSpec;
use bevy_ragdoll::profile::BodyIndex;
use bevy_ragdoll::runtime::RagdollFixedSchedule;
use bevy_ragdoll::runtime::backend::{BackendCapabilities, RayHit};
use bevy_ragdoll::runtime::body::{
    BodyAtRest, BodyDriveOutput, BodyKind, BodyMass, BodyPhysicsPose, BodyShape, BodyVelocity,
};
use bevy_ragdoll::runtime::components::{RagdollBodyOf, RagdollTargetPose};
use bevy_ragdoll::runtime::messages::{RagdollImpulse, RagdollRaycast, RagdollRaycastResponse};
use bevy_ragdoll::runtime::sets::RagdollFixedSystems;
use bevy_ragdoll::runtime::settings::RagdollPhysicsSettings;

/// Provides a deliberately approximate backend for shared core contract checks.
///
/// The plugin uses semi-implicit Euler integration, gravity, impulses, drive
/// forces, a ground plane, and capsule ray tests. It ignores joints and
/// contacts, so it checks backend-neutral behavior without claiming to produce
/// a stable connected ragdoll.
#[derive(Clone, Copy, Debug, Default)]
pub struct MockBackendPlugin;

impl Plugin for MockBackendPlugin {
    fn build(&self, app: &mut App) {
        // The core owns the schedule label, so fail immediately when plugin order is invalid.
        let fixed_schedule = app
            .world()
            .get_resource::<RagdollFixedSchedule>()
            .map(|schedule| schedule.label)
            .expect("MockBackendPlugin requires RagdollPlugin to be added first");
        // Advertise fallback motors before the core drive stage calculates joint torque.
        app.insert_resource(BackendCapabilities {
            has_native_joint_motors: false,
            has_asymmetric_swing_limits: false,
            is_deterministic: true,
            can_run_on_wasm: true,
        });
        // Apply targets and integrate the mock world before backend-neutral queries read bodies.
        app.add_systems(
            fixed_schedule,
            apply_kinematic_targets.in_set(RagdollFixedSystems::Apply),
        );
        // Integrate dynamic bodies after kinematic targets have been copied from animation.
        app.add_systems(
            fixed_schedule,
            integrate_dynamic_bodies
                .after(apply_kinematic_targets)
                .in_set(RagdollFixedSystems::Apply),
        );
        // Publish shared body state after the mock completes its integration stage.
        app.add_systems(
            fixed_schedule,
            refresh_dynamic_pose_history.in_set(RagdollFixedSystems::Read),
        );
        app.add_systems(
            fixed_schedule,
            raycast_requests.in_set(RagdollFixedSystems::Read),
        );
    }
}

/// Components read or changed while moving one kinematic body to its target.
type KinematicBodyData = (
    Entity,
    &'static BodyKind,
    &'static RagdollBodyOf,
    &'static BodyIndex,
    &'static mut BodyVelocity,
    &'static mut BodyPhysicsPose,
    &'static mut Transform,
);

/// Filtered body query used by kinematic target application.
type KinematicBodyQuery<'w, 's> = Query<'w, 's, KinematicBodyData, With<BodyShape>>;

/// Read-only local transform and parent links used to compose hierarchy poses.
type LocalTransformQuery<'w, 's> =
    Query<'w, 's, (Option<&'static Transform>, Option<&'static ChildOf>)>;

/// Mutually exclusive body and hierarchy queries used by target following.
#[derive(SystemParam)]
struct KinematicQueries<'w, 's> {
    /// Provides mutable kinematic bodies and read-only character hierarchy access.
    queries: ParamSet<'w, 's, (KinematicBodyQuery<'w, 's>, LocalTransformQuery<'w, 's>)>,
}

/// Moves kinematic bodies to captured targets during the backend apply stage.
fn apply_kinematic_targets(
    mut queries: KinematicQueries<'_, '_>,
    targets: Query<'_, '_, &RagdollTargetPose>,
) {
    // Gather character identities before reading their parent-first local transforms.
    let character_entities = {
        let bodies = queries.queries.p0();
        bodies
            .iter()
            .map(|(_, _, body_of, _, _, _, _)| body_of.0)
            .collect::<HashSet<_>>()
    };
    // Compose each character pose once so bodies with the same owner share the result.
    let character_poses = {
        let transforms = queries.queries.p1();
        character_entities
            .into_iter()
            .map(|entity| (entity, character_world_pose(entity, &transforms)))
            .collect::<HashMap<_, _>>()
    };

    let mut bodies = queries.queries.p0();
    for (_entity, kind, body_of, body_index, mut velocity, mut physics_pose, mut transform) in
        &mut bodies
    {
        if *kind != BodyKind::Kinematic {
            continue;
        }
        // Wait until both target history and the indexed body pose are available.
        let Ok(targets) = targets.get(body_of.0) else {
            continue;
        };
        let Some(target_pose) = targets.current_pose(*body_index) else {
            continue;
        };
        let character_pose = character_poses
            .get(&body_of.0)
            .copied()
            .unwrap_or(Isometry3d::IDENTITY);
        let target_pose = character_pose * target_pose;
        let target_velocity = targets.velocity(*body_index).unwrap_or_default();

        // Rotate captured bone velocity into world space with the composed character orientation.
        // Move the body to its target and preserve the previous sample for rendering.
        let previous = physics_pose.current;
        transform.translation = target_pose.translation.into();
        transform.rotation = target_pose.rotation;
        *velocity = BodyVelocity {
            linear: character_pose.rotation * target_velocity.linear,
            angular: character_pose.rotation * target_velocity.angular,
        };
        physics_pose.previous = previous;
        physics_pose.current = target_pose;
    }
}

/// Accumulates world-space linear and angular impulses for one rigid body.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct ImpulseAccumulator {
    /// Summed linear impulse in newton seconds for one body.
    linear: Vec3,
    /// Summed moment about the world origin before conversion to a body's
    /// center.
    torque_about_origin: Vec3,
}

impl ImpulseAccumulator {
    /// Adds one world-space point impulse to the bounded per-body accumulator.
    fn add(&mut self, point: Vec3, impulse: Vec3) {
        // Accumulate linear and origin-relative angular momentum without storing each message.
        self.linear += impulse;
        self.torque_about_origin += point.cross(impulse);
    }

    /// Returns the summed angular impulse around the body's current center.
    fn angular_at_center(self, center: Vec3) -> Vec3 {
        // Shift moments from world origin to center using center cross total linear impulse.
        self.torque_about_origin - center.cross(self.linear)
    }
}

/// Components accessed while integrating dynamic bodies.
type DynamicBodyData = (
    Entity,
    &'static BodyKind,
    &'static BodyMass,
    &'static mut BodyVelocity,
    &'static mut Transform,
    &'static BodyDriveOutput,
);

/// Query for the mutable state needed by dynamic body integration.
type DynamicBodyQuery<'w, 's> = Query<'w, 's, DynamicBodyData>;

/// Integrates dynamic bodies with semi-implicit Euler and the configured ground
/// plane.
fn integrate_dynamic_bodies(
    time: Res<'_, Time<Fixed>>,
    settings: Res<'_, RagdollPhysicsSettings>,
    mut impulses: MessageReader<'_, '_, RagdollImpulse>,
    mut bodies: DynamicBodyQuery<'_, '_>,
) {
    let delta_seconds = time.delta_secs();
    // Skip invalid or zero-length steps before any impulse changes body state.
    if !delta_seconds.is_finite() || delta_seconds <= 0.0 {
        return;
    }

    // Combine same-step impulses so each body receives their summed momentum change.
    let mut impulse_by_body = HashMap::<Entity, ImpulseAccumulator>::new();
    for impulse in impulses.read() {
        impulse_by_body
            .entry(impulse.body)
            .or_default()
            .add(impulse.point, impulse.impulse);
    }

    // Integrate only dynamic bodies; fixed and kinematic state is handled in separate paths.
    for (entity, kind, mass, mut velocity, mut transform, drive) in &mut bodies {
        if *kind == BodyKind::Fixed {
            // Frozen bodies retain their poses, so discard motion from dynamic mode.
            *velocity = BodyVelocity::default();
        }
        if *kind != BodyKind::Dynamic {
            continue;
        }
        let inertia = (mass.mass * mass.min_inertia_radius.powi(2)).max(1.0e-6);
        // Apply point impulses as linear and center-relative angular momentum changes.
        if let Some(impulse) = impulse_by_body.get(&entity) {
            velocity.linear += impulse.linear / mass.mass;
            velocity.angular += impulse.angular_at_center(transform.translation) / inertia;
        }
        // Apply gravity and pin force before updating linear position.
        velocity.linear += (settings.gravity + drive.pin_force / mass.mass) * delta_seconds;
        velocity.angular += (drive.pin_torque + drive.joint_torque) / inertia * delta_seconds;
        // Apply exponential damping after acceleration and before pose integration.
        velocity.linear *= (-settings.linear_damping * delta_seconds).exp();
        velocity.angular *= (-settings.angular_damping * delta_seconds).exp();
        drive.clamp_velocity(&mut velocity);

        transform.translation += velocity.linear * delta_seconds;
        // The mock ground plane prevents penetration and removes downward velocity.
        if transform.translation.y < 0.0 {
            transform.translation.y = 0.0;
            velocity.linear.y = velocity.linear.y.max(0.0);
        }
        // Integrate orientation from angular velocity using the current world-space rotation.
        transform.rotation = (Quat::from_scaled_axis(velocity.angular * delta_seconds)
            * transform.rotation)
            .normalize();
    }
}

/// Components read or updated after the mock backend completes integration.
type DynamicReadbackData = (
    Entity,
    &'static BodyKind,
    &'static Transform,
    &'static BodyVelocity,
    &'static mut BodyPhysicsPose,
    Option<&'static BodyAtRest>,
);

/// Query for dynamic body values read after integration.
type DynamicReadbackQuery<'w, 's> = Query<'w, 's, DynamicReadbackData>;

/// Advances dynamic body pose history after integration updates body transforms.
fn refresh_dynamic_pose_history(
    settings: Res<'_, RagdollPhysicsSettings>,
    mut bodies: DynamicReadbackQuery<'_, '_>,
    mut commands: Commands<'_, '_>,
) {
    // Fixed and kinematic bodies maintain pose history in their own backend path.
    for (entity, kind, transform, velocity, mut physics_pose, at_rest) in &mut bodies {
        if *kind != BodyKind::Dynamic {
            continue;
        }
        // Preserve the pre-step pose and publish the transform completed in Apply.
        physics_pose.previous = physics_pose.current;
        physics_pose.current = Isometry3d::new(transform.translation, transform.rotation);
        update_rest_marker(entity, *velocity, at_rest, &settings, commands.reborrow());
    }
}

/// Updates the mock sleeping marker from configured linear and angular
/// thresholds.
fn update_rest_marker(
    entity: Entity,
    velocity: BodyVelocity,
    at_rest: Option<&BodyAtRest>,
    settings: &RagdollPhysicsSettings,
    mut commands: Commands<'_, '_>,
) {
    // Require both motion channels to remain below their independent thresholds.
    let is_sleeping = velocity.linear.length() < settings.sleep_linear_threshold
        && velocity.angular.length() < settings.sleep_angular_threshold;
    if is_sleeping && at_rest.is_none() {
        commands.entity(entity).insert(BodyAtRest);
    } else if !is_sleeping && at_rest.is_some() {
        commands.entity(entity).remove::<BodyAtRest>();
    }
}

/// Composes local transforms through the character's current hierarchy.
fn character_world_pose(
    entity: Entity,
    transforms: &Query<'_, '_, (Option<&Transform>, Option<&ChildOf>)>,
) -> Isometry3d {
    // Collect local ancestors because globals may still describe the prior animation frame.
    let mut hierarchy = vec![entity];
    let mut current = entity;
    while let Ok((_, Some(parent))) = transforms.get(current) {
        current = parent.0;
        hierarchy.push(current);
    }
    hierarchy.reverse();

    // Multiply local transforms in root-first order so parent scale affects child positions.
    let mut affine = Affine3A::IDENTITY;
    for node in hierarchy {
        if let Ok((Some(transform), _)) = transforms.get(node) {
            affine *= transform.compute_affine();
        }
    }
    // Remove scale only after the complete affine hierarchy has been composed.
    let (_, rotation, translation) = affine.to_scale_rotation_translation();
    Isometry3d::new(translation, rotation)
}

/// Answers backend-neutral raycast messages with the closest body-shape hit.
fn raycast_requests(
    mut requests: MessageReader<'_, '_, RagdollRaycast>,
    mut responses: MessageWriter<'_, RagdollRaycastResponse>,
    bodies: Query<'_, '_, (Entity, &Transform, &BodyShape, &RagdollBodyOf), With<BodyKind>>,
) {
    // Answer requests in message order while preserving each caller-owned request identity.
    for request in requests.read() {
        let direction = request.direction.try_normalize();
        let hit = direction.and_then(|direction| {
            if !request.max_distance.is_finite() || request.max_distance <= 0.0 {
                return None;
            }
            bodies
                .iter()
                .filter(|(entity, _, _, owner)| {
                    Some(*entity) != request.filter && Some(owner.0) != request.filter
                })
                .filter_map(|(entity, transform, shape, _)| {
                    ray_shape(
                        request.origin,
                        direction,
                        request.max_distance,
                        transform,
                        shape,
                    )
                    .map(|(distance, normal)| (entity, distance, normal))
                })
                .min_by(|left, right| left.1.total_cmp(&right.1))
                .map(|(entity, distance, normal)| RayHit {
                    entity,
                    body: Some(entity),
                    point: request.origin + direction * distance,
                    normal,
                    distance,
                })
        });
        responses.write(RagdollRaycastResponse {
            request_id: request.request_id,
            hit,
        });
    }
}

/// Intersects a world ray with a profile shape transformed by its body entity.
fn ray_shape(
    origin: Vec3,
    direction: Vec3,
    max_distance: f32,
    transform: &Transform,
    shape: &BodyShape,
) -> Option<(f32, Vec3)> {
    let affine = transform.compute_affine();
    let scale = transform.scale.abs().max_element();
    match &shape.0 {
        ShapeSpec::Sphere { center, radius } => ray_sphere(
            origin,
            direction,
            affine.transform_point3(*center),
            radius * scale,
        ),
        ShapeSpec::Capsule { a, b, radius } => ray_capsule(
            origin,
            direction,
            affine.transform_point3(*a),
            affine.transform_point3(*b),
            radius * scale,
        ),
        ShapeSpec::Cuboid {
            center,
            half_extents,
            ..
        } => ray_sphere(
            origin,
            direction,
            affine.transform_point3(*center),
            half_extents.length() * scale,
        ),
    }
    .filter(|(distance, _)| distance.is_finite() && *distance <= max_distance)
}

/// Intersects a normalized world ray with a sphere and returns distance and
/// normal.
fn ray_sphere(origin: Vec3, direction: Vec3, center: Vec3, radius: f32) -> Option<(f32, Vec3)> {
    // Reject malformed geometry before evaluating the ray-sphere quadratic.
    if !radius.is_finite() || radius < 0.0 {
        return None;
    }
    let offset = origin - center;
    let along = offset.dot(direction);
    let discriminant = along.mul_add(along, -radius.mul_add(-radius, offset.length_squared()));
    // A negative discriminant means the normalized ray misses the sphere.
    if discriminant < 0.0 {
        return None;
    }
    let root = discriminant.sqrt();
    // Select the near intersection unless the origin starts inside the sphere.
    let mut distance = -along - root;
    if distance < 0.0 {
        distance = -along + root;
    }
    (distance >= 0.0).then(|| {
        let point = origin + direction * distance;
        (distance, (point - center).normalize_or_zero())
    })
}

/// Intersects a normalized world ray with a capsule's cylinder and end caps.
fn ray_capsule(
    origin: Vec3,
    direction: Vec3,
    start: Vec3,
    end: Vec3,
    radius: f32,
) -> Option<(f32, Vec3)> {
    // Reject invalid radii before using them in cylinder or sphere equations.
    if !radius.is_finite() || radius < 0.0 {
        return None;
    }
    let side = ray_capsule_side(origin, direction, start, end, radius);
    let start_cap = ray_sphere(origin, direction, start, radius);
    let end_cap = ray_sphere(origin, direction, end, radius);
    // Return the nearest valid cylinder or cap intersection.
    [side, start_cap, end_cap]
        .into_iter()
        .flatten()
        .min_by(|left, right| left.0.total_cmp(&right.0))
}

/// Intersects a normalized ray with the cylindrical portion between capsule
/// endpoints.
fn ray_capsule_side(
    origin: Vec3,
    direction: Vec3,
    start: Vec3,
    end: Vec3,
    radius: f32,
) -> Option<(f32, Vec3)> {
    // Project the ray and origin offset onto the capsule axis for the cylinder equation.
    let axis = end - start;
    let offset = origin - start;
    let axis_sq = axis.length_squared();
    let axis_dot_direction = axis.dot(direction);
    let axis_dot_offset = axis.dot(offset);
    let direction_dot_offset = direction.dot(offset);
    let offset_sq = offset.length_squared();
    let a = axis_dot_direction.mul_add(-axis_dot_direction, axis_sq);
    // A parallel ray has no isolated cylinder-side intersection.
    if a <= 1.0e-6 {
        return None;
    }
    // Build the quadratic coefficients without transforming the ray into local space.
    let b = axis_dot_offset.mul_add(-axis_dot_direction, axis_sq * direction_dot_offset);
    let c = (radius * radius).mul_add(
        -axis_sq,
        axis_dot_offset.mul_add(-axis_dot_offset, axis_sq * offset_sq),
    );
    let discriminant = a.mul_add(-c, b * b);
    // A negative discriminant means the ray misses the infinite cylinder.
    if discriminant < 0.0 {
        return None;
    }
    let root = discriminant.sqrt();
    let mut distance = (-b - root) / a;
    if distance < 0.0 {
        distance = (-b + root) / a;
    }
    // Convert the nearest cylinder root back to a bounded point on the capsule axis.
    let along_axis = distance.mul_add(axis_dot_direction, axis_dot_offset);
    // Keep side hits between endpoint planes; the cap tests cover their hemispheres.
    if distance < 0.0 || along_axis <= 0.0 || along_axis >= axis_sq {
        return None;
    }
    let point = origin + direction * distance;
    let closest = start + axis * (along_axis / axis_sq);
    Some((distance, (point - closest).normalize_or_zero()))
}

#[cfg(test)]
mod tests {
    use bevy::app::FixedUpdate;
    use bevy::asset::AssetPlugin;
    use bevy::ecs::message::Messages;
    use bevy::math::Vec3;
    use bevy::prelude::{AnimationPlugin, App, Entity, MinimalPlugins, Transform, TransformPlugin};
    use bevy::time::{Fixed, Time, TimeUpdateStrategy};

    use bevy_ragdoll::ShapeSpec;
    use bevy_ragdoll::profile::BodyIndex;
    use bevy_ragdoll::runtime::RagdollPlugin;
    use bevy_ragdoll::runtime::body::{BodyKind, BodyPhysicsPose, BodyShape, BodyVelocity};
    use bevy_ragdoll::runtime::components::{RagdollBodyOf, RagdollTargetPose};
    use bevy_ragdoll::runtime::messages::{RagdollRaycast, RagdollRaycastResponse};

    use super::{ImpulseAccumulator, MockBackendPlugin, ray_capsule, ray_shape, ray_sphere};

    /// Point impulses preserve angular momentum around the body's current
    /// center.
    #[test]
    fn point_impulse_accumulates_center_relative_angular_momentum() {
        let mut accumulator = ImpulseAccumulator::default();
        accumulator.add(Vec3::Y, Vec3::X * 2.0);

        assert_eq!(accumulator.linear, Vec3::X * 2.0);
        assert_eq!(accumulator.angular_at_center(Vec3::ZERO), -Vec3::Z * 2.0);
        assert_eq!(accumulator.angular_at_center(Vec3::Y), Vec3::ZERO);
    }

    /// Builds a minimal app with the core messages and mock raycast system.
    fn raycast_app() -> App {
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

    /// Adds a kinematic mock body at a visible position with the required contract data.
    fn kinematic_body(app: &mut App, character: Entity, index: usize) -> Entity {
        app.world_mut()
            .spawn((
                BodyKind::Kinematic,
                RagdollBodyOf(character),
                BodyIndex::try_from(index).expect("the test body index fits the profile"),
                BodyVelocity::default(),
                BodyPhysicsPose::default(),
                BodyShape(ShapeSpec::Sphere {
                    center: Vec3::ZERO,
                    radius: 0.1,
                }),
                Transform::from_translation(Vec3::X * 4.0),
            ))
            .id()
    }

    /// Kinematic target application skips characters without target history.
    #[test]
    fn kinematic_target_skips_missing_history() {
        let mut app = raycast_app();
        let character = app.world_mut().spawn_empty().id();
        let body = kinematic_body(&mut app, character, 0);

        app.world_mut().run_schedule(FixedUpdate);

        assert_eq!(
            app.world()
                .get::<Transform>(body)
                .map(|transform| transform.translation),
            Some(Vec3::X * 4.0)
        );
    }

    /// Kinematic target application skips indexes beyond captured target history.
    #[test]
    fn kinematic_target_skips_an_out_of_range_pose() {
        let mut app = raycast_app();
        let character = app.world_mut().spawn(RagdollTargetPose::default()).id();
        let body = kinematic_body(&mut app, character, 1);

        app.world_mut().run_schedule(FixedUpdate);

        assert_eq!(
            app.world()
                .get::<Transform>(body)
                .map(|transform| transform.translation),
            Some(Vec3::X * 4.0)
        );
    }

    /// The mock backend cannot register its systems before the core plugin.
    #[test]
    #[should_panic(expected = "MockBackendPlugin requires RagdollPlugin to be added first")]
    fn mock_backend_requires_the_core_plugin() {
        let mut app = App::new();

        app.add_plugins(MockBackendPlugin);
    }

    /// Sphere and cuboid rays return hits within the requested maximum
    /// distance.
    #[test]
    fn ray_shapes_cover_sphere_cuboid_and_distance_limits() {
        let transform = Transform::IDENTITY;
        let sphere = BodyShape(ShapeSpec::Sphere {
            center: Vec3::ZERO,
            radius: 0.5,
        });
        let cuboid = BodyShape(ShapeSpec::Cuboid {
            center: Vec3::ZERO,
            half_extents: Vec3::splat(0.5),
            rotation: bevy::math::Quat::IDENTITY,
        });

        let sphere_hit = ray_shape(Vec3::new(-2.0, 0.0, 0.0), Vec3::X, 2.0, &transform, &sphere)
            .expect("the ray reaches the sphere");
        let cuboid_hit = ray_shape(Vec3::new(-2.0, 0.0, 0.0), Vec3::X, 2.0, &transform, &cuboid)
            .expect("the mock approximates the cuboid by its bounding sphere");

        assert!((sphere_hit.0 - 1.5).abs() < 1.0e-6);
        assert!((cuboid_hit.0 - (2.0 - 0.75_f32.sqrt())).abs() < 1.0e-6);
        assert!(ray_shape(Vec3::new(-2.0, 0.0, 0.0), Vec3::X, 1.0, &transform, &sphere,).is_none());
    }

    /// Sphere intersections cover inside origins, misses, and invalid radii.
    #[test]
    fn ray_sphere_handles_inside_origins_and_invalid_inputs() {
        let inside = ray_sphere(Vec3::ZERO, Vec3::X, Vec3::ZERO, 1.0)
            .expect("an inside origin exits through the far side");

        assert_eq!(inside.0, 1.0);
        assert_eq!(inside.1, Vec3::X);
        assert!(ray_sphere(Vec3::new(0.0, 2.0, 0.0), Vec3::X, Vec3::ZERO, 1.0).is_none());
        assert!(ray_sphere(Vec3::ZERO, Vec3::X, Vec3::ZERO, -1.0).is_none());
        assert!(ray_sphere(Vec3::ZERO, Vec3::X, Vec3::ZERO, f32::NAN).is_none());
    }

    /// Ray requests with a zero direction or invalid distance return no hit.
    #[test]
    fn raycast_rejects_invalid_direction_and_distance() {
        let mut app = raycast_app();
        let mut cursor = app
            .world()
            .get_resource::<Messages<RagdollRaycastResponse>>()
            .expect("RagdollPlugin registers raycast responses")
            .get_cursor();

        for (request_id, direction, max_distance) in [
            (1, Vec3::ZERO, 4.0),
            (2, Vec3::X, -1.0),
            (3, Vec3::X, f32::NAN),
            (4, Vec3::X, 0.0),
        ] {
            app.world_mut().write_message(RagdollRaycast {
                request_id: bevy_ragdoll::runtime::messages::RagdollRequestId::new(request_id),
                origin: Vec3::ZERO,
                direction,
                max_distance,
                filter: None,
            });
            app.world_mut().run_schedule(FixedUpdate);
            let response = cursor
                .read(
                    app.world()
                        .get_resource::<Messages<RagdollRaycastResponse>>()
                        .expect("RagdollPlugin retains raycast responses"),
                )
                .find(|response| response.request_id.get() == request_id)
                .copied()
                .expect("the raycast system answers each request");
            assert!(response.hit.is_none());
        }
    }

    #[test]
    fn ray_capsule_handles_cylinder_caps_and_invalid_radii() {
        let side = ray_capsule(Vec3::new(-2.0, 0.5, 0.0), Vec3::X, Vec3::ZERO, Vec3::Y, 0.5)
            .expect("the ray hits the capsule cylinder");
        assert!((side.0 - 1.5).abs() < 1.0e-6);
        assert!(side.1.abs_diff_eq(-Vec3::X, 1.0e-6));

        let cap = ray_capsule(
            Vec3::new(-2.0, 0.0, 0.0),
            Vec3::X,
            Vec3::ZERO,
            Vec3::ZERO,
            0.5,
        )
        .expect("a zero-length capsule reduces to its end caps");
        assert!((cap.0 - 1.5).abs() < 1.0e-6);
        let end_cap = ray_capsule(Vec3::new(-2.0, 1.0, 0.0), Vec3::X, Vec3::ZERO, Vec3::Y, 0.5)
            .expect("the ray hits the capsule's end cap");
        assert!((end_cap.0 - 1.5).abs() < 1.0e-6);
        let inside = ray_capsule(Vec3::ZERO, Vec3::X, Vec3::ZERO, Vec3::Y, 0.5)
            .expect("an inside origin exits through the capsule cap");
        assert!((inside.0 - 0.5).abs() < 1.0e-6);
        assert!(
            ray_capsule(Vec3::new(-2.0, 2.0, 0.0), Vec3::X, Vec3::ZERO, Vec3::Y, 0.5,).is_none()
        );
        assert!(
            ray_capsule(
                Vec3::new(-2.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 1.0).normalize(),
                Vec3::ZERO,
                Vec3::Y,
                0.5,
            )
            .is_none()
        );
        assert!(ray_capsule(Vec3::ZERO, Vec3::X, Vec3::ZERO, Vec3::Y, -0.5).is_none());
        assert!(ray_capsule(Vec3::ZERO, Vec3::X, Vec3::ZERO, Vec3::Y, f32::NAN,).is_none());
    }
}
