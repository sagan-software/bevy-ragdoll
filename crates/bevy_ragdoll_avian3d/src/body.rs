//! Convert backend-neutral body state into Avian components and fixed-step updates.

use std::collections::BTreeMap;

use avian3d::prelude::{
    ActiveCollisionHooks, AngularDamping, AngularInertia, AngularVelocity, CenterOfMass,
    ColliderOf, Forces, Friction, LinearDamping, LinearVelocity, Mass, Position, Restitution,
    RigidBody, Rotation, SleepThreshold, SleepingDisabled, SpatialQuery, SpeculativeMargin,
    SweptCcd, WakeBody, WriteRigidBodyForces,
};
use bevy::ecs::system::SystemParam;
use bevy::math::{Isometry3d, Vec3};
use bevy::prelude::{
    Added, Command, Commands, Entity, GlobalTransform, MessageReader, Query, Res, Transform, With,
    World,
};
use bevy_ragdoll::profile::ShapeSpec;
use bevy_ragdoll::runtime::backend::BodyContacts;
use bevy_ragdoll::runtime::body::{
    BodyDriveOutput, BodyKind, BodyMass, BodyPhysicsPose, BodyShape, BodyVelocity,
};
use bevy_ragdoll::runtime::components::{Ragdoll, RagdollBodyOf, RagdollDrive, RagdollTargetPose};
use bevy_ragdoll::runtime::messages::RagdollImpulse;
use bevy_ragdoll::runtime::settings::RagdollPhysicsSettings;

use crate::settings::AvianRagdollSettings;
use crate::shape::collider_for_shape;
use crate::spawn::spawn_lift;

/// New body entities and the shared state that builds their Avian body.
type NewBodyQuery<'w, 's> = Query<
    'w,
    's,
    (
        Entity,
        &'static RagdollBodyOf,
        &'static BodyShape,
        &'static BodyMass,
        &'static BodyVelocity,
        &'static BodyKind,
    ),
    Added<BodyShape>,
>;

/// Pose state that spawn lift moves before Avian copies it.
type BodyPoseQuery<'w, 's> = Query<'w, 's, (&'static mut BodyPhysicsPose, &'static mut Transform)>;

/// World colliders that spawn lift may push ragdolls out of.
#[derive(SystemParam)]
pub(crate) struct SpawnLiftWorld<'w, 's> {
    /// Avian shape-intersection queries.
    spatial_query: SpatialQuery<'w, 's>,
    /// Ragdoll bodies, which never count as world geometry.
    ragdoll_bodies: Query<'w, 's, (), With<RagdollBodyOf>>,
    /// Collider-to-body links used to find static colliders.
    collider_bodies: Query<'w, 's, &'static ColliderOf>,
    /// Rigid-body kinds of collider owners.
    rigid_bodies: Query<'w, 's, &'static RigidBody>,
}

impl SpawnLiftWorld<'_, '_> {
    /// Returns whether a collider is static, non-ragdoll world geometry.
    fn is_world_collider(&self, collider: Entity) -> bool {
        if self.ragdoll_bodies.contains(collider) {
            return false;
        }
        self.collider_bodies
            .get(collider)
            .ok()
            .and_then(|link| self.rigid_bodies.get(link.body).ok())
            .is_none_or(RigidBody::is_static)
    }

    /// Measures one shared upward lift per owner of the new bodies.
    ///
    /// Owners are visited in entity order so the result does not depend on
    /// query order.
    fn owner_lifts(
        &self,
        bodies: &NewBodyQuery<'_, '_>,
        poses: &Query<'_, '_, (&'static BodyPhysicsPose, &'static Transform)>,
        max_spawn_lift: f32,
    ) -> BTreeMap<Entity, f32> {
        // Group each owner's new shapes at their captured world poses.
        let mut shapes_by_owner = BTreeMap::<Entity, Vec<(ShapeSpec, Isometry3d)>>::new();
        for (entity, owner, shape, ..) in bodies {
            if let Ok((pose, _)) = poses.get(entity) {
                shapes_by_owner
                    .entry(owner.0)
                    .or_default()
                    .push((shape.0, pose.current));
            }
        }
        // Search each owner's clear offset against static world geometry only.
        let is_world_collider = |collider| self.is_world_collider(collider);
        shapes_by_owner
            .into_iter()
            .map(|(owner, shapes)| {
                let lift = spawn_lift(
                    &self.spatial_query,
                    &shapes,
                    max_spawn_lift,
                    &is_world_collider,
                );
                (owner, lift)
            })
            .collect()
    }
}

/// Creates Avian components for new backend-neutral body entities.
///
/// Spawn lift runs first: each owner's bodies move up together by the largest
/// clear centimetre offset, so joint frames stay aligned.
pub(crate) fn create_avian_bodies(
    mut commands: Commands<'_, '_>,
    settings: Res<'_, RagdollPhysicsSettings>,
    avian_settings: Res<'_, AvianRagdollSettings>,
    drivers: Query<'_, '_, &RagdollDrive>,
    lift_world: SpawnLiftWorld<'_, '_>,
    bodies: NewBodyQuery<'_, '_>,
    mut poses: BodyPoseQuery<'_, '_>,
) {
    // Most fixed steps add no bodies; skip the lift search entirely.
    if bodies.is_empty() {
        return;
    }
    // Measure one lift per owner and resolve the CCD policy once per step.
    // Measure one lift per owner and resolve the CCD policy once per step.
    let max_spawn_lift = finite_nonnegative(settings.max_spawn_lift).unwrap_or(0.0);
    let lift_by_owner = lift_world.owner_lifts(&bodies, &poses.as_readonly(), max_spawn_lift);
    let is_swept_ccd_used = settings.is_ccd_enabled && avian_settings.is_swept_ccd_enabled;
    for (entity, owner, shape, mass, velocity, kind) in &bodies {
        // Every new body has a pose; skip any entity that lost it this step.
        // Every new body has a pose; skip any entity that lost it this step.
        let Ok((mut pose, mut transform)) = poses.get_mut(entity) else {
            continue;
        };
        // Lift the core pose and the transform together before Avian reads them.
        let lift = lift_by_owner.get(&owner.0).copied().unwrap_or(0.0);
        if lift > 0.0 {
            transform.translation.y += lift;
            pose.previous.translation.y += lift;
            pose.current.translation.y += lift;
        }
        // Attach the body, then the optional sleep and CCD markers.
        let drive = drivers.get(owner.0).copied().unwrap_or_default();
        let mut body = commands.entity(entity);
        body.insert(body_components(
            shape.0, *mass, *velocity, *kind, *transform, &settings,
        ));
        if is_driven(drive) {
            body.insert(SleepingDisabled);
        }
        if let Some(swept_ccd) = ccd(is_swept_ccd_used) {
            body.insert(swept_ccd);
        }
    }
}

/// Builds the rigid body, collider, mass, pose, and readback components.
fn body_components(
    shape: ShapeSpec,
    mass: BodyMass,
    velocity: BodyVelocity,
    kind: BodyKind,
    transform: Transform,
    settings: &RagdollPhysicsSettings,
) -> impl bevy::ecs::bundle::Bundle {
    let collider = collider_for_shape(shape);
    // Explicit mass components on the body override Avian's collider-derived values.
    let inertia = floored_inertia(AngularInertia::from_shape(&collider, mass.mass), mass);
    let center_of_mass = CenterOfMass::from_shape(&collider);
    (
        kind_to_rigid_body(kind),
        collider,
        Mass(mass.mass),
        inertia,
        center_of_mass,
        Position(transform.translation),
        Rotation(transform.rotation),
        LinearVelocity(velocity.linear),
        AngularVelocity(velocity.angular),
        body_material(settings),
        ActiveCollisionHooks::FILTER_PAIRS,
        BodyContacts::default(),
    )
}

/// Builds friction, restitution, damping, speculative margin, and sleep
/// threshold from the shared settings, replacing invalid values with zero.
fn body_material(settings: &RagdollPhysicsSettings) -> impl bevy::ecs::bundle::Bundle {
    let restitution = finite_nonnegative(settings.restitution).unwrap_or(0.0);
    (
        Friction::new(finite_nonnegative(settings.friction).unwrap_or(0.0)),
        Restitution::new(restitution.clamp(0.0, 1.0)),
        LinearDamping(finite_nonnegative(settings.linear_damping).unwrap_or(0.0)),
        AngularDamping(finite_nonnegative(settings.angular_damping).unwrap_or(0.0)),
        SpeculativeMargin(finite_nonnegative(settings.soft_ccd_prediction).unwrap_or(0.0)),
        sleep_threshold(settings),
    )
}

/// Returns swept CCD when enabled, or an empty bundle.
fn ccd(is_enabled: bool) -> Option<SweptCcd> {
    is_enabled.then(SweptCcd::default)
}

/// Applies the profile inertia floor to each principal axis.
fn floored_inertia(inertia: AngularInertia, mass: BodyMass) -> AngularInertia {
    let min_radius = finite_nonnegative(mass.min_inertia_radius).unwrap_or(0.0);
    let floor = mass.mass * min_radius * min_radius;
    AngularInertia::new_with_local_frame(
        inertia.principal.max(Vec3::splat(floor)),
        inertia.local_frame,
    )
}

/// Converts the shared sleep thresholds into Avian's per-body threshold.
fn sleep_threshold(settings: &RagdollPhysicsSettings) -> SleepThreshold {
    SleepThreshold {
        linear: finite_nonnegative(settings.sleep_linear_threshold).unwrap_or(0.0),
        angular: finite_nonnegative(settings.sleep_angular_threshold).unwrap_or(0.0),
    }
}

/// Returns whether muscle or pin output can produce force for this owner.
fn is_driven(drive: RagdollDrive) -> bool {
    drive.muscle() > 0.0 || drive.pin() > 0.0
}

/// Body state used to apply kind changes and the drive-based sleep policy.
type BodyKindQuery<'w, 's> = Query<
    'w,
    's,
    (
        Entity,
        &'static BodyKind,
        &'static RagdollBodyOf,
        &'static RigidBody,
        &'static mut LinearVelocity,
        &'static mut AngularVelocity,
        bevy::prelude::Has<SleepingDisabled>,
    ),
>;

/// Applies body kind transitions and keeps driven bodies awake.
///
/// `RigidBody` is immutable in Avian, so a kind change re-inserts it, clears
/// both velocities, and wakes the body's island.
pub(crate) fn apply_body_kinds_and_sleeping(
    mut commands: Commands<'_, '_>,
    mut bodies: BodyKindQuery<'_, '_>,
    drivers: Query<'_, '_, &RagdollDrive>,
) {
    for (entity, kind, owner, rigid_body, mut linear, mut angular, sleep_disabled) in &mut bodies {
        // Re-insert the immutable rigid body only when the shared kind changes.
        let target = kind_to_rigid_body(*kind);
        if *rigid_body != target {
            // Avian forbids mutating `RigidBody`, so insert the new kind and stop the body.
            // Avian forbids mutating `RigidBody`, so insert the new kind and stop the body.
            commands.entity(entity).insert(target);
            linear.0 = Vec3::ZERO;
            angular.0 = Vec3::ZERO;
            commands.queue(TryWakeBody(entity));
        }
        // Muscle or pin output disables sleeping; limp bodies may sleep again.
        let is_owner_driven = is_driven(drivers.get(owner.0).copied().unwrap_or_default());
        if is_owner_driven && !sleep_disabled {
            commands.entity(entity).insert(SleepingDisabled);
        } else if !is_owner_driven && sleep_disabled {
            commands.entity(entity).remove::<SleepingDisabled>();
        }
    }
}

/// Wakes a body when it belongs to an island and ignores it otherwise.
///
/// A body created this step has no island yet; it starts awake.
pub(crate) struct TryWakeBody(pub(crate) Entity);

impl Command for TryWakeBody {
    type Out = ();

    fn apply(self, world: &mut World) {
        // A body without an island is already awake, so the error carries no action.
        let _ignored = WakeBody(self.0).apply(world);
    }
}

/// Moves kinematic body transforms to the owning ragdoll's captured pose.
///
/// Avian copies a changed `Transform` into `Position` and `Rotation` before
/// the step, so kinematic bodies land on the target exactly.
pub(crate) fn apply_kinematic_targets(
    roots: Query<'_, '_, (&GlobalTransform, &RagdollTargetPose), With<Ragdoll>>,
    mut bodies: Query<
        '_,
        '_,
        (
            &BodyKind,
            &RagdollBodyOf,
            &bevy_ragdoll::profile::BodyIndex,
            &mut Transform,
        ),
    >,
) {
    for (kind, owner, index, mut transform) in &mut bodies {
        // Only kinematic bodies follow animation; Avian moves the rest.
        if *kind != BodyKind::Kinematic {
            continue;
        }
        let Ok((root_global, targets)) = roots.get(owner.0) else {
            continue;
        };
        // A missing captured pose leaves the body where it is.
        let Some(target) = targets.current_pose(*index) else {
            continue;
        };
        // GlobalTransform includes the character's parent hierarchy after propagation.
        let (scale, root_rotation, root_translation) = root_global.to_scale_rotation_translation();
        transform.translation =
            root_translation + root_rotation * (scale * Vec3::from(target.translation));
        transform.rotation = root_rotation * target.rotation;
    }
}

/// Applies this step's pin force and combined torque to dynamic bodies.
///
/// Avian clears applied forces after every step, so a zero output leaves no
/// stale force behind.
pub(crate) fn apply_body_forces(mut bodies: Query<'_, '_, (&BodyDriveOutput, &RigidBody, Forces)>) {
    for (output, rigid_body, mut forces) in &mut bodies {
        // Kinematic and frozen bodies ignore drive output.
        if !rigid_body.is_dynamic() {
            continue;
        }
        forces.apply_force(output.pin_force);
        forces.apply_torque(output.pin_torque + output.joint_torque);
    }
}

/// Applies valid one-shot impulses to dynamic bodies at their world points.
pub(crate) fn apply_impulses(
    mut messages: MessageReader<'_, '_, RagdollImpulse>,
    mut bodies: Query<'_, '_, (&RigidBody, Forces)>,
) {
    for message in messages.read() {
        // Drop malformed messages and messages for despawned bodies.
        if !message.point.is_finite() || !message.impulse.is_finite() {
            continue;
        }
        let Ok((rigid_body, mut forces)) = bodies.get_mut(message.body) else {
            continue;
        };
        // Frozen and kinematic bodies ignore impulses.
        if rigid_body.is_dynamic() {
            forces.apply_linear_impulse_at_point(message.impulse, message.point);
        }
    }
}

/// Maps the shared motion state to its Avian rigid-body kind.
const fn kind_to_rigid_body(kind: BodyKind) -> RigidBody {
    match kind {
        BodyKind::Dynamic => RigidBody::Dynamic,
        // Avian 0.7 panics when a joint links two static bodies, because static
        // bodies have no island. A frozen kinematic body with zero velocity stays put.
        BodyKind::Kinematic | BodyKind::Fixed => RigidBody::Kinematic,
    }
}

/// Returns a finite nonnegative scalar for Avian material and force settings.
pub(crate) fn finite_nonnegative(value: f32) -> Option<f32> {
    (value.is_finite() && value >= 0.0).then_some(value)
}

#[cfg(test)]
mod tests {
    //! Checks pure body mapping helpers and skipped update branches.

    use avian3d::prelude::{AngularInertia, RigidBody, SweptCcd};
    use bevy::math::{Quat, Vec3};
    use bevy::prelude::{Command, World};
    use bevy_ragdoll::runtime::body::{BodyKind, BodyMass};
    use bevy_ragdoll::runtime::components::RagdollDrive;
    use bevy_ragdoll::runtime::settings::RagdollPhysicsSettings;

    use super::{
        TryWakeBody, ccd, finite_nonnegative, floored_inertia, is_driven, kind_to_rigid_body,
        sleep_threshold,
    };

    /// Every shared kind maps to the matching Avian kind.
    #[test]
    fn kinds_map_to_avian_rigid_bodies() {
        assert_eq!(
            kind_to_rigid_body(BodyKind::Kinematic),
            RigidBody::Kinematic
        );
        assert_eq!(kind_to_rigid_body(BodyKind::Dynamic), RigidBody::Dynamic);
        assert_eq!(kind_to_rigid_body(BodyKind::Fixed), RigidBody::Kinematic);
    }

    /// The inertia floor raises small axes and keeps larger ones and the frame.
    #[test]
    fn inertia_floor_raises_only_small_axes() {
        let frame = Quat::from_rotation_y(0.3);
        // The X axis lies below the 0.02 floor; Y and Z lie above it.
        // The X axis lies below the 0.02 floor; Y and Z lie above it.
        let inertia = AngularInertia::new_with_local_frame(Vec3::new(0.001, 1.0, 0.5), frame);
        let floored = floored_inertia(
            inertia,
            BodyMass {
                mass: 2.0,
                min_inertia_radius: 0.1,
            },
        );
        assert!((floored.principal - Vec3::new(0.02, 1.0, 0.5)).length() < 1.0e-6);
        assert_eq!(floored.local_frame, frame);
    }

    /// A non-finite inertia radius applies no floor.
    #[test]
    fn invalid_inertia_radius_applies_no_floor() {
        let inertia = AngularInertia::new(Vec3::new(0.001, 1.0, 0.5));
        let mass = BodyMass {
            mass: 2.0,
            min_inertia_radius: f32::NAN,
        };
        assert_eq!(floored_inertia(inertia, mass).principal, inertia.principal);
    }

    /// Invalid sleep thresholds become zero.
    #[test]
    fn sleep_threshold_sanitizes_invalid_values() {
        let base = RagdollPhysicsSettings::default();
        let settings = RagdollPhysicsSettings {
            sleep_linear_threshold: f32::NAN,
            sleep_angular_threshold: 0.2,
            ..base
        };
        // NaN becomes zero while the valid angular value passes through.
        // NaN becomes zero while the valid angular value passes through.
        let threshold = sleep_threshold(&settings);
        assert_eq!(threshold.linear, 0.0);
        assert_eq!(threshold.angular, 0.2);
    }

    /// Either muscle or pin output counts as driven.
    #[test]
    fn drive_disables_sleep_when_muscle_or_pin_is_positive() {
        assert!(is_driven(RagdollDrive::new(1.0, 0.0)));
        assert!(is_driven(RagdollDrive::new(0.0, 1.0)));
        assert!(!is_driven(RagdollDrive::new(0.0, 0.0)));
    }

    /// Swept CCD is present only when enabled.
    #[test]
    fn ccd_component_follows_the_setting() {
        assert_eq!(ccd(true), Some(SweptCcd::default()));
        assert_eq!(ccd(false), None);
    }

    /// Scalars must be finite and nonnegative.
    #[test]
    fn finite_nonnegative_rejects_invalid_values() {
        assert_eq!(finite_nonnegative(1.5), Some(1.5));
        assert_eq!(finite_nonnegative(-1.0), None);
        assert_eq!(finite_nonnegative(f32::INFINITY), None);
    }

    /// Waking an entity that is not a body does nothing and does not panic.
    #[test]
    fn waking_a_non_body_is_ignored() {
        let mut world = World::new();
        let entity = world.spawn_empty().id();
        TryWakeBody(entity).apply(&mut world);
        assert!(world.get_entity(entity).is_ok());
    }
}
