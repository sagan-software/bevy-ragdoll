//! Convert backend-neutral body state into Rapier components and fixed-step updates.

use std::collections::{HashMap, HashSet};

use bevy::ecs::system::{ParamSet, SystemParam};
use bevy::math::Vec3;
use bevy::prelude::{Added, Commands, Entity, GlobalTransform, Query, Res, Transform, With};
use bevy_ragdoll::profile::{BodyIndex, ShapeSpec};
use bevy_ragdoll::runtime::backend::{BackendCapabilities, BodyContacts};
use bevy_ragdoll::runtime::body::{
    BodyDriveOutput, BodyKind, BodyMass, BodyPhysicsPose, BodyShape, BodyVelocity, JointToParent,
};
use bevy_ragdoll::runtime::components::{Ragdoll, RagdollBodyOf, RagdollDrive, RagdollTargetPose};
use bevy_ragdoll::runtime::messages::RagdollImpulse;
use bevy_ragdoll::runtime::settings::RagdollPhysicsSettings;
use bevy_rapier3d::plugin::ReadRapierContext;
use bevy_rapier3d::prelude::{
    ActiveHooks, AdditionalSolverIterations, Ccd, Collider, ColliderMassProperties, Damping,
    ExternalForce, ExternalImpulse, Friction, MassProperties, ReadMassProperties, Restitution,
    RigidBody, Sleeping, SoftCcd, Velocity,
};

use crate::settings::RapierRagdollSettings;
use crate::shape::collider_for_shape;
use crate::spawn::spawn_lift;

/// Shared and Rapier-only resources used during new body construction.
#[derive(SystemParam)]
pub(crate) struct BodyPhysicsResources<'w> {
    /// Shared material, sleep, collision, and spawn-lift settings.
    ragdoll: Res<'w, RagdollPhysicsSettings>,
    /// Rapier-specific CCD and root-iteration settings.
    rapier: Res<'w, RapierRagdollSettings>,
    /// Capabilities selecting backend-specific body setup.
    capabilities: Res<'w, BackendCapabilities>,
}

/// Complete body query used while adding backend components to new entities.
type ExistingBodyQuery<'w, 's> = Query<
    'w,
    's,
    (
        Entity,
        &'static RagdollBodyOf,
        &'static BodyIndex,
        &'static BodyShape,
        &'static BodyMass,
        &'static BodyVelocity,
        &'static BodyKind,
        &'static mut BodyPhysicsPose,
        &'static mut Transform,
        &'static mut GlobalTransform,
    ),
>;

/// New body entities whose shared shape component was just added.
type AddedBodyQuery<'w, 's> = Query<
    'w,
    's,
    (
        Entity,
        &'static RagdollBodyOf,
        Option<&'static JointToParent>,
    ),
    Added<BodyShape>,
>;

/// Disjoint reads used to snapshot added bodies and update full body state.
#[derive(SystemParam)]
pub(crate) struct BodyCreationQueries<'w, 's> {
    /// Added-body scan and complete body-state query.
    queries: ParamSet<'w, 's, (ExistingBodyQuery<'w, 's>, AddedBodyQuery<'w, 's>)>,
}

/// Body state needed when applying mode and target changes.
type BodyKindQuery<'w, 's> = Query<
    'w,
    's,
    (
        bevy::prelude::Ref<'static, BodyKind>,
        &'static RagdollBodyOf,
        &'static BodyIndex,
        &'static mut RigidBody,
        &'static mut Velocity,
        &'static mut Sleeping,
    ),
>;

/// Narrow body query used to move kinematic bodies toward captured animation targets.
type KinematicTargetQuery<'w, 's> = Query<
    'w,
    's,
    (
        &'static BodyKind,
        &'static RagdollBodyOf,
        &'static BodyIndex,
        &'static mut Transform,
    ),
>;

/// New ragdoll body with its owner and root-body role.
#[derive(Clone, Copy, Debug)]
struct AddedBody {
    /// Entity receiving Rapier rigid-body and collider components.
    entity: Entity,
    /// Character whose full profile may require a shared spawn lift.
    owner: Entity,
    /// Whether this body receives root-only additional solver iterations.
    is_root: bool,
}

/// Creates Rapier components for new backend-neutral body entities.
///
/// Profiles contain at most 64 bodies. Added-body snapshots use O(n) temporary
/// memory; owner grouping costs O(n + c log c), owner filtering costs O(n*c),
/// and lift search runs up to k + 1 Rapier shape queries per body, where c is
/// the owner count and k is the configured centimetre-step bound.
pub(crate) fn create_rapier_bodies(
    mut commands: Commands<'_, '_>,
    settings: BodyPhysicsResources<'_>,
    drivers: Query<'_, '_, &RagdollDrive>,
    context: ReadRapierContext<'_, '_>,
    ragdoll_colliders: Query<'_, '_, (), With<RagdollBodyOf>>,
    mut bodies: BodyCreationQueries<'_, '_>,
) {
    // Snapshot added entities before borrowing the disjoint full-state query.
    let new_bodies = bodies
        .queries
        .p1()
        .iter()
        .map(|(entity, owner, joint)| AddedBody {
            entity,
            owner: owner.0,
            is_root: joint.is_none(),
        })
        .collect::<Vec<_>>();
    if new_bodies.is_empty() {
        return;
    }
    // Measure fixed-world overlaps before translating any member of each rig.
    let owners = new_body_owners(&new_bodies);
    let lift_by_owner = measure_spawn_lifts(
        &settings,
        &context,
        &ragdoll_colliders,
        &mut bodies,
        &owners,
    );
    apply_spawn_lifts_to_body_poses(&lift_by_owner, &mut bodies);

    // Attach each body after measuring and applying its owner's shared lift.
    for added in &new_bodies {
        let mut body_query = bodies.queries.p0();
        let Ok((_, _, _, shape, mass, velocity, kind, _, _, _)) = body_query.get_mut(added.entity)
        else {
            continue;
        };
        let drive = drivers.get(added.owner).copied().unwrap_or_default();
        commands.entity(added.entity).insert(body_components(
            shape.0, *mass, *velocity, *kind, drive, &settings,
        ));
        let root_solver_iterations = settings.rapier.root_additional_solver_iterations;
        if added.is_root
            && settings.capabilities.has_native_joint_motors
            && root_solver_iterations > 0
        {
            // Extra passes improve native-motor constraints without changing fallback torque.
            commands
                .entity(added.entity)
                .insert(AdditionalSolverIterations(root_solver_iterations));
        }
    }
}

/// Applies each owner's maximum spawn correction to every stored body pose.
fn apply_spawn_lifts_to_body_poses(
    lift_by_owner: &HashMap<Entity, f32>,
    bodies: &mut BodyCreationQueries<'_, '_>,
) {
    // Skip a scan when every owner already has a clear spawn pose.
    if lift_by_owner.is_empty() {
        return;
    }
    // Translate all three pose states so authored joint frames remain aligned.
    for (_, owner, .., mut pose, mut transform, mut global_transform) in
        bodies.queries.p0().iter_mut()
    {
        let Some(lift) = lift_by_owner.get(&owner.0).copied() else {
            continue;
        };
        if lift > 0.0 {
            transform.translation.y += lift;
            *global_transform = GlobalTransform::from(*transform);
            pose.previous.translation.y += lift;
            pose.current.translation.y += lift;
        }
    }
}

/// Returns distinct body owners in deterministic entity-bit order.
fn new_body_owners(new_bodies: &[AddedBody]) -> Vec<Entity> {
    // De-duplicate linearly, then sort so spawn-query order cannot affect results.
    let mut seen = HashSet::with_capacity(new_bodies.len());
    let mut owners = Vec::with_capacity(new_bodies.len());
    new_bodies
        .iter()
        .map(|body| body.owner)
        .filter(|owner| seen.insert(*owner))
        .for_each(|owner| owners.push(owner));
    owners.sort_unstable_by_key(|entity| entity.to_bits());
    owners
}

/// Measures one shared lift for each newly added owner with fixed-world overlap.
fn measure_spawn_lifts(
    settings: &BodyPhysicsResources<'_>,
    context_param: &ReadRapierContext<'_, '_>,
    ragdoll_colliders: &Query<'_, '_, (), With<RagdollBodyOf>>,
    bodies: &mut BodyCreationQueries<'_, '_>,
    owners: &[Entity],
) -> HashMap<Entity, f32> {
    // Missing contexts or invalid bounds disable spawn correction for this update.
    let (Ok(context), Some(max_spawn_lift)) = (
        context_param.single(),
        finite_nonnegative(settings.ragdoll.max_spawn_lift),
    ) else {
        return HashMap::new();
    };
    let mut lifts = HashMap::with_capacity(owners.len());
    for owner in owners {
        // Test all profile body shapes before applying one correction to the owner.
        let profile_bodies = bodies
            .queries
            .p0()
            .iter()
            .filter_map(|(_, body_owner, _, shape, _, _, _, pose, _, _)| {
                (body_owner.0 == *owner).then_some((shape.0, pose.current))
            })
            .collect::<Vec<_>>();
        let lift = spawn_lift(&context, &profile_bodies, max_spawn_lift, ragdoll_colliders);
        lifts.insert(*owner, lift);
    }
    lifts
}

/// Builds body materials, mass, drive state, and backend-neutral readback components.
fn body_components(
    shape: ShapeSpec,
    mass: BodyMass,
    velocity: BodyVelocity,
    kind: BodyKind,
    drive: RagdollDrive,
    settings: &BodyPhysicsResources<'_>,
) -> impl bevy::ecs::bundle::Bundle {
    // Derive inertia from the same collider that will be inserted on the body.
    let collider = collider_for_shape(shape);
    let mass_properties = mass_properties(&collider, mass);
    // Disable sleeping while either target controller can produce force.
    let sleeping = sleeping_for_drive(drive, *settings.ragdoll, false);
    (
        kind_to_rigid_body(kind),
        collider,
        ColliderMassProperties::MassProperties(mass_properties),
        Velocity {
            linear: velocity.linear,
            angular: velocity.angular,
        },
        Damping {
            linear_damping: finite_nonnegative(settings.ragdoll.linear_damping).unwrap_or(0.0),
            angular_damping: finite_nonnegative(settings.ragdoll.angular_damping).unwrap_or(0.0),
        },
        Ccd {
            enabled: settings.ragdoll.is_ccd_enabled,
        },
        SoftCcd {
            prediction: finite_nonnegative(settings.ragdoll.soft_ccd_prediction).unwrap_or(0.0),
        },
        Friction::new(finite_nonnegative(settings.ragdoll.friction).unwrap_or(0.0)),
        Restitution::new(
            finite_nonnegative(settings.ragdoll.restitution)
                .unwrap_or(0.0)
                .clamp(0.0, 1.0),
        ),
        ExternalImpulse::default(),
        ExternalForce::default(),
        ReadMassProperties::default(),
        ActiveHooks::FILTER_CONTACT_PAIRS,
        sleeping,
        BodyContacts::default(),
    )
}

/// Applies the shared sleeping policy while preserving an existing sleep state.
fn sleeping_for_drive(
    drive: RagdollDrive,
    settings: RagdollPhysicsSettings,
    is_sleeping: bool,
) -> Sleeping {
    if drive.muscle() > 0.0 || drive.pin() > 0.0 {
        Sleeping::disabled()
    } else {
        Sleeping {
            normalized_linear_threshold: finite_nonnegative(settings.sleep_linear_threshold)
                .unwrap_or(0.0),
            angular_threshold: finite_nonnegative(settings.sleep_angular_threshold).unwrap_or(0.0),
            sleeping: is_sleeping,
        }
    }
}

/// Applies body type transitions and the shared drive-based sleeping policy.
pub(crate) fn apply_body_kinds_and_sleeping(
    mut bodies: BodyKindQuery<'_, '_>,
    drivers: Query<'_, '_, &RagdollDrive>,
    settings: Res<'_, RagdollPhysicsSettings>,
) {
    // Reset velocity only when the Rapier body type actually changes.
    for (kind, owner, _, mut rigid_body, mut velocity, mut sleeping) in &mut bodies {
        let target_body_type = kind_to_rigid_body(*kind);
        if *rigid_body != target_body_type {
            *rigid_body = target_body_type;
            velocity.linear = Vec3::ZERO;
            velocity.angular = Vec3::ZERO;
            sleeping.sleeping = false;
        }
        // Preserve sleep for limp bodies while active muscle or pin output disables it.
        // A nonzero drive must keep Rapier awake even when the core mode is limp.
        let drive = drivers.get(owner.0).copied().unwrap_or_default();
        let target_sleep = sleeping_for_drive(drive, *settings, sleeping.sleeping);
        if *sleeping != target_sleep {
            *sleeping = target_sleep;
        }
    }
}

/// Moves kinematic body transforms toward the owning ragdoll's captured pose.
pub(crate) fn apply_kinematic_targets(
    roots: Query<'_, '_, (&GlobalTransform, &RagdollTargetPose), With<Ragdoll>>,
    mut bodies: KinematicTargetQuery<'_, '_>,
) {
    for (kind, owner, index, mut transform) in &mut bodies {
        // Dynamic and fixed bodies are controlled by Rapier, not animation targets.
        if *kind != BodyKind::Kinematic {
            continue;
        }
        let Ok((root_global, targets)) = roots.get(owner.0) else {
            continue;
        };
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

/// Rewrites continuous force state every fixed step, including zero outputs.
pub(crate) fn apply_body_forces(mut bodies: Query<'_, '_, (&BodyDriveOutput, &mut ExternalForce)>) {
    // Rewrite every fixed step so a previous nonzero output cannot linger.
    for (output, mut force) in &mut bodies {
        force.force = output.pin_force;
        force.torque = output.pin_torque + output.joint_torque;
    }
}

/// Accumulates valid one-shot impulses at their world-space points.
pub(crate) fn apply_impulses(
    mut messages: bevy::prelude::MessageReader<'_, '_, RagdollImpulse>,
    mut bodies: Query<'_, '_, (&Transform, &ReadMassProperties, &mut ExternalImpulse)>,
) {
    // Reject malformed events before mutating Rapier's one-shot impulse component.
    for message in messages.read() {
        if !message.point.is_finite() || !message.impulse.is_finite() {
            continue;
        }
        let Ok((transform, mass_properties, mut external)) = bodies.get_mut(message.body) else {
            continue;
        };
        let local_center = mass_properties.get().local_center_of_mass;
        // Convert the mass-property center into world coordinates for point torque.
        let world_center = transform.translation + transform.rotation * local_center;
        *external += ExternalImpulse::at_point(message.impulse, message.point, world_center);
    }
}

/// Computes shape mass properties and applies the profile inertia floor.
fn mass_properties(collider: &Collider, body_mass: BodyMass) -> MassProperties {
    // Rapier computes unit-density inertia before the profile's physical mass is applied.
    let mut properties = collider.raw.mass_properties(1.0);
    properties.set_mass(body_mass.mass, true);
    let mut properties = MassProperties::from_rapier(properties);
    // Enforce a radius-based inertia floor to avoid unstable tiny-profile axes.
    let min_radius = finite_nonnegative(body_mass.min_inertia_radius).unwrap_or(0.0);
    let inertia_floor = body_mass.mass * min_radius * min_radius;
    properties.principal_inertia = properties.principal_inertia.max(Vec3::splat(inertia_floor));
    properties
}

/// Maps the shared motion state to its Rapier rigid-body kind.
const fn kind_to_rigid_body(kind: BodyKind) -> RigidBody {
    match kind {
        BodyKind::Kinematic => RigidBody::KinematicPositionBased,
        BodyKind::Dynamic => RigidBody::Dynamic,
        BodyKind::Fixed => RigidBody::Fixed,
    }
}

/// Returns a finite nonnegative scalar for Rapier material and force settings.
fn finite_nonnegative(value: f32) -> Option<f32> {
    (value.is_finite() && value >= 0.0).then_some(value)
}

#[cfg(test)]
mod tests {
    //! Checks that non-capsule profile shapes retain their local geometry.

    use bevy::ecs::message::Messages;
    use bevy::ecs::system::{Res, SystemState};
    use bevy::math::{Quat, Vec3};
    use bevy::prelude::{Entity, GlobalTransform, Query, Ref, Transform, World};
    use bevy_rapier3d::geometry::ColliderView;
    use bevy_rapier3d::prelude::{
        ExternalImpulse, ReadMassProperties, RigidBody, Sleeping, Velocity,
    };

    use super::{apply_body_kinds_and_sleeping, apply_impulses, apply_kinematic_targets};
    use crate::shape::collider_for_shape;
    use bevy_ragdoll::ShapeSpec;
    use bevy_ragdoll::profile::BodyIndex;
    use bevy_ragdoll::runtime::body::BodyKind;
    use bevy_ragdoll::runtime::components::{
        Ragdoll, RagdollBodyOf, RagdollDrive, RagdollTargetPose,
    };
    use bevy_ragdoll::runtime::messages::RagdollImpulse;
    use bevy_ragdoll::runtime::settings::RagdollPhysicsSettings;

    /// Root transforms and captured targets used by kinematic body updates.
    type TargetRootQuery = Query<
        'static,
        'static,
        (&'static GlobalTransform, &'static RagdollTargetPose),
        bevy::prelude::With<Ragdoll>,
    >;

    /// Backend body state used by rigid-body mode and sleep updates.
    type BodyKindUpdateQuery = Query<
        'static,
        'static,
        (
            Ref<'static, BodyKind>,
            &'static RagdollBodyOf,
            &'static BodyIndex,
            &'static mut RigidBody,
            &'static mut Velocity,
            &'static mut Sleeping,
        ),
    >;

    /// Transform state used by kinematic animation-target updates.
    type KinematicTargetQuery = Query<
        'static,
        'static,
        (
            &'static BodyKind,
            &'static RagdollBodyOf,
            &'static BodyIndex,
            &'static mut Transform,
        ),
    >;

    /// Character drive state read while applying body modes.
    type BodyDriverQuery = Query<'static, 'static, &'static RagdollDrive>;

    /// System parameters used to test kinematic body state transitions.
    type KinematicUpdateState = SystemState<(
        TargetRootQuery,
        BodyKindUpdateQuery,
        KinematicTargetQuery,
        BodyDriverQuery,
        Res<'static, RagdollPhysicsSettings>,
    )>;

    /// Impulse messages consumed by the body update system.
    type ImpulseReader = bevy::prelude::MessageReader<'static, 'static, RagdollImpulse>;

    /// Backend properties and actuator state used while applying impulses.
    type ImpulseBodyQuery = Query<
        'static,
        'static,
        (
            &'static Transform,
            &'static ReadMassProperties,
            &'static mut ExternalImpulse,
        ),
    >;

    /// System parameters used to test impulse message handling.
    type ImpulseUpdateState = SystemState<(ImpulseReader, ImpulseBodyQuery)>;

    /// Spawns a sleep-disabled kinematic Rapier body owned by `owner` at the
    /// identity transform.
    fn spawn_kinematic(world: &mut World, owner: Entity, index: BodyIndex) -> Entity {
        world
            .spawn((
                BodyKind::Kinematic,
                RagdollBodyOf(owner),
                index,
                RigidBody::KinematicPositionBased,
                Transform::IDENTITY,
                Velocity::default(),
                Sleeping::disabled(),
            ))
            .id()
    }

    /// Runs the kind, sleep, and kinematic target systems once and applies
    /// their deferred commands.
    fn run_kinematic_systems(world: &mut World) {
        let mut system: KinematicUpdateState = SystemState::new(world);
        // The test world always holds the settings resource.
        if let Ok((roots, bodies, targets, drivers, settings)) = system.get_mut(world) {
            apply_body_kinds_and_sleeping(bodies, drivers, settings);
            apply_kinematic_targets(roots, targets);
        }
        system.apply(world);
    }

    /// Missing roots and targets keep kinematic bodies at their current transforms.
    #[test]
    fn kinematic_updates_skip_missing_roots_and_target_poses() {
        // A limp owner with an empty target history, and a root that is not a ragdoll.
        let mut world = World::new();
        world.insert_resource(RagdollPhysicsSettings::default());
        let owner = world
            .spawn((
                Ragdoll::new(bevy::asset::Handle::default()),
                GlobalTransform::default(),
                RagdollTargetPose::default(),
                RagdollDrive::new(0.0, 0.0),
            ))
            .id();
        let index = BodyIndex::try_from(0).expect("profile body index zero is valid");
        let missing_root = world.spawn(GlobalTransform::default()).id();
        // One body has a valid owner and one names a root without targets.
        let body = spawn_kinematic(&mut world, owner, index);
        let orphan = spawn_kinematic(&mut world, missing_root, index);
        run_kinematic_systems(&mut world);

        // Neither body moves, and the limp owner's body may sleep again.
        assert_eq!(world.get::<Transform>(body), Some(&Transform::IDENTITY));
        assert_eq!(world.get::<Transform>(orphan), Some(&Transform::IDENTITY));
        assert_ne!(world.get::<Sleeping>(body), Some(&Sleeping::disabled()));
    }

    /// Invalid messages and references to removed entities leave impulses unchanged.
    #[test]
    fn impulses_skip_invalid_values_and_missing_bodies() {
        let mut world = World::new();
        world.init_resource::<Messages<RagdollImpulse>>();
        let body = world
            .spawn((
                Transform::IDENTITY,
                ReadMassProperties::default(),
                ExternalImpulse::default(),
            ))
            .id();
        // Queue a NaN impulse and an impulse for an entity that does not exist.
        let mut system: ImpulseUpdateState = SystemState::new(&mut world);

        world.write_message(RagdollImpulse {
            body,
            point: Vec3::ZERO,
            impulse: Vec3::splat(f32::NAN),
        });
        world.write_message(RagdollImpulse {
            body: Entity::PLACEHOLDER,
            point: Vec3::ZERO,
            impulse: Vec3::ONE,
        });
        // Neither message may reach the body's external impulse.
        let (messages, bodies) = system
            .get_mut(&mut world)
            .expect("the impulse message channel remains available");
        apply_impulses(messages, bodies);
        system.apply(&mut world);

        assert_eq!(
            world.get::<ExternalImpulse>(body),
            Some(&ExternalImpulse::default())
        );
    }

    /// Sphere compounds retain their offset and radius.
    #[test]
    fn sphere_collider_preserves_offset_and_radius() {
        let center = Vec3::new(0.1, 0.2, 0.3);
        let radius = 0.4;
        // Offset the sphere so the compound must carry its position.
        let collider = collider_for_shape(ShapeSpec::Sphere { center, radius });
        let compound = collider
            .as_compound()
            .expect("profile spheres use one-shape compounds");
        let mut shapes = compound.shapes();
        let (offset, rotation, child) = shapes.next().expect("sphere child exists");

        // The single child keeps the offset with no rotation.
        assert_eq!(
            (offset, rotation, shapes.len()),
            (center, Quat::IDENTITY, 0)
        );
        match child {
            ColliderView::Ball(ball) => assert_eq!(ball.radius(), radius),
            _ => panic!("sphere profile maps to a Rapier ball"),
        }
    }

    /// Cuboid compounds retain their offset, rotation, and half-extents.
    #[test]
    fn cuboid_collider_preserves_local_transform_and_extents() {
        let center = Vec3::new(0.1, 0.2, 0.3);
        let rotation = Quat::from_rotation_y(0.25);
        let half_extents = Vec3::new(0.4, 0.5, 0.6);
        // Offset and rotate the cuboid so the compound must carry both.
        let collider = collider_for_shape(ShapeSpec::Cuboid {
            center,
            rotation,
            half_extents,
        });
        // The compound holds exactly one cuboid child.
        let compound = collider
            .as_compound()
            .expect("profile cuboids use one-shape compounds");
        let mut shapes = compound.shapes();
        let (offset, child_rotation, child) = shapes.next().expect("cuboid child exists");

        // The single child keeps the offset and rotation.
        assert_eq!(
            (offset, child_rotation, shapes.len()),
            (center, rotation, 0)
        );
        match child {
            ColliderView::Cuboid(cuboid) => assert_eq!(cuboid.half_extents(), half_extents),
            _ => panic!("cuboid profile maps to a Rapier cuboid"),
        }
    }
}
