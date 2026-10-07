//! Build spherical constraints and apply the runtime's native motor targets.

use bevy::math::{Quat, Vec3};
use bevy::prelude::{Added, Commands, Entity, Query, Res};
use bevy_ragdoll::runtime::backend::BackendCapabilities;
use bevy_ragdoll::runtime::body::{JointBasis, JointDriveTarget, JointToParent};
use bevy_rapier3d::prelude::{
    GenericJointBuilder, ImpulseJoint, JointAxesMask, JointAxis, MotorModel, TypedJoint,
};
use bevy_rapier3d::rapier::dynamics::SpringCoefficients;

/// Last motor target applied to one Rapier joint.
#[derive(bevy::prelude::Component, Default, bevy::prelude::Reflect)]
pub(crate) struct RapierJointMotorState {
    /// Target whose position, velocity, gains, and force cap are in Rapier.
    applied: Option<JointDriveTarget>,
}

/// Creates Rapier's locked-linear, limited-angular joint for each new profile joint.
pub(crate) fn create_rapier_joints(
    mut commands: Commands<'_, '_>,
    capabilities: Res<'_, BackendCapabilities>,
    joints: Query<'_, '_, (Entity, &JointToParent, Option<&JointBasis>), Added<JointToParent>>,
) {
    for (entity, joint, basis) in &joints {
        let basis = basis.map_or(Quat::IDENTITY, |basis| basis.0);
        // Native motor profiles use a softer joint spring than fallback torque profiles.
        // Measured physics conformance uses softer limits for native motors and firmer limits for fallback torque.
        let softness_hz = if capabilities.has_native_joint_motors {
            1.0e4
        } else {
            1.0e5
        };
        let mut generic = GenericJointBuilder::new(JointAxesMask::LOCKED_SPHERICAL_AXES)
            .local_anchor1(joint.frame.translation.into())
            .local_basis1(joint.frame.rotation * basis)
            .local_anchor2(Vec3::ZERO)
            .local_basis2(basis)
            .build();
        generic.raw.softness = SpringCoefficients::new(softness_hz, 1.0);
        generic.set_contacts_enabled(false);
        // Lock degenerate axes and install every authored range before attaching the constraint.
        for (axis, range) in [
            (JointAxis::AngX, joint.limits.x),
            (JointAxis::AngY, joint.limits.twist),
            (JointAxis::AngZ, joint.limits.z),
        ] {
            if range.is_locked() {
                generic.lock_axes(axis_mask(axis));
            } else {
                generic.set_limits(axis, [range.min, range.max]);
            }
        }
        commands.entity(entity).insert((
            ImpulseJoint::new(joint.parent, TypedJoint::GenericJoint(generic)),
            RapierJointMotorState::default(),
        ));
    }
}

/// Applies changed backend-neutral targets to native Rapier joint motors.
pub(crate) fn apply_joint_motors(
    capabilities: Res<'_, BackendCapabilities>,
    mut joints: Query<
        '_,
        '_,
        (
            &JointDriveTarget,
            &JointToParent,
            &mut ImpulseJoint,
            &mut RapierJointMotorState,
        ),
    >,
) {
    // Fallback torque systems own joint actuation when native motors are unavailable.
    if !capabilities.has_native_joint_motors {
        return;
    }
    for (target, profile_joint, mut impulse_joint, mut state) in &mut joints {
        // Avoid rewriting Rapier motor state when the shared target did not change.
        if state.applied == Some(*target) {
            continue;
        }
        let Some(angles) = motor_angles(target.rotation) else {
            continue;
        };
        let angular_velocity = finite_vec3(target.angular_velocity);
        let stiffness = finite_nonnegative(target.stiffness);
        let damping = finite_nonnegative(target.damping);
        let max_torque = finite_nonnegative(target.max_torque);
        // Preserve locked axes while mapping valid target values to all free axes.
        for (index, (axis, range)) in [
            (JointAxis::AngX, profile_joint.limits.x),
            (JointAxis::AngY, profile_joint.limits.twist),
            (JointAxis::AngZ, profile_joint.limits.z),
        ]
        .into_iter()
        .enumerate()
        {
            if range.is_locked() {
                continue;
            }
            impulse_joint
                .data
                .as_mut()
                .set_motor_model(axis, MotorModel::AccelerationBased)
                .set_motor(
                    axis,
                    angles[index],
                    angular_velocity[index],
                    stiffness,
                    damping,
                )
                .set_motor_max_force(axis, max_torque);
        }
        // Cache only after Rapier accepted every axis update for this target.
        state.applied = Some(*target);
    }
}

/// Converts a target quaternion to Rapier's wrapped per-axis joint coordinates.
fn motor_angles(rotation: Quat) -> Option<Vec3> {
    // Reject malformed quaternions before normalization can propagate invalid values.
    if !rotation.is_finite() || rotation.length_squared() <= f32::EPSILON {
        return None;
    }
    let mut rotation = rotation.normalize();
    // Select one quaternion hemisphere so equivalent rotations produce stable angles.
    if rotation.w < 0.0 {
        rotation = -rotation;
    }
    Some(Vec3::new(
        2.0 * rotation.x.atan2(rotation.w),
        2.0 * rotation.y.atan2(rotation.w),
        2.0 * rotation.z.atan2(rotation.w),
    ))
}

/// Keeps finite motor velocities while replacing malformed vectors with zero.
fn finite_vec3(value: Vec3) -> Vec3 {
    if value.is_finite() { value } else { Vec3::ZERO }
}

/// Keeps a finite nonnegative motor scalar while replacing invalid values with zero.
fn finite_nonnegative(value: f32) -> f32 {
    if value.is_finite() {
        value.max(0.0)
    } else {
        0.0
    }
}

/// Maps one angular axis to its Rapier lock mask.
const fn axis_mask(axis: JointAxis) -> JointAxesMask {
    match axis {
        JointAxis::AngX => JointAxesMask::ANG_X,
        JointAxis::AngY => JointAxesMask::ANG_Y,
        JointAxis::AngZ => JointAxesMask::ANG_Z,
        JointAxis::LinX => JointAxesMask::LIN_X,
        JointAxis::LinY => JointAxesMask::LIN_Y,
        JointAxis::LinZ => JointAxesMask::LIN_Z,
    }
}

#[cfg(test)]
mod tests {
    //! Checks malformed targets and complete Rapier joint-axis mapping.

    use bevy::ecs::system::{Res, SystemState};
    use bevy::math::{Isometry3d, Quat, Vec3};
    use bevy::prelude::{Query, World};
    use bevy_rapier3d::prelude::{
        GenericJoint, ImpulseJoint, JointAxesMask, JointAxis, TypedJoint,
    };

    use super::{RapierJointMotorState, apply_joint_motors, axis_mask, finite_nonnegative};
    use super::{finite_vec3, motor_angles};
    use bevy_ragdoll::profile::{AngleRange, JointLimits};
    use bevy_ragdoll::runtime::backend::BackendCapabilities;
    use bevy_ragdoll::runtime::body::{JointDriveTarget, JointToParent};

    /// Joint components mutated while applying native motor targets.
    type MotorJointQuery = Query<
        'static,
        'static,
        (
            &'static JointDriveTarget,
            &'static JointToParent,
            &'static mut ImpulseJoint,
            &'static mut RapierJointMotorState,
        ),
    >;

    /// Capability resource and joint query used by the motor update system.
    type JointMotorUpdateState = SystemState<(Res<'static, BackendCapabilities>, MotorJointQuery)>;

    /// A malformed public target leaves its previous Rapier motor state unchanged.
    #[test]
    fn apply_motor_skips_invalid_rotation() {
        let mut world = World::new();
        world.insert_resource(BackendCapabilities {
            has_native_joint_motors: true,
            ..Default::default()
        });
        let parent = world.spawn_empty().id();
        let locked = AngleRange { min: 0.0, max: 0.0 };
        let limits = JointLimits {
            x: locked,
            twist: locked,
            z: locked,
        };
        let joint = world
            .spawn((
                JointDriveTarget {
                    rotation: Quat::from_xyzw(f32::NAN, 0.0, 0.0, 1.0),
                    ..Default::default()
                },
                JointToParent {
                    parent,
                    frame: Isometry3d::IDENTITY,
                    limits,
                    max_torque: 0.0,
                },
                ImpulseJoint::new(
                    parent,
                    TypedJoint::GenericJoint(GenericJoint::new(
                        JointAxesMask::LOCKED_SPHERICAL_AXES,
                    )),
                ),
                RapierJointMotorState::default(),
            ))
            .id();
        let mut system: JointMotorUpdateState = SystemState::new(&mut world);
        let (capabilities, joints) = system
            .get_mut(&mut world)
            .expect("backend capabilities remain available");

        apply_joint_motors(capabilities, joints);
        system.apply(&mut world);

        assert!(
            world
                .get::<RapierJointMotorState>(joint)
                .expect("joint motor state remains attached")
                .applied
                .is_none()
        );
    }

    /// Invalid quaternions do not produce motor angles.
    #[test]
    fn motor_angles_reject_non_finite_and_zero_quaternions() {
        assert_eq!(motor_angles(Quat::from_xyzw(0.0, 0.0, 0.0, 0.0)), None);
        assert_eq!(motor_angles(Quat::from_xyzw(f32::NAN, 0.0, 0.0, 1.0)), None);
    }

    /// Non-finite velocities become zero while valid vectors pass through.
    #[test]
    fn motor_velocity_rejects_non_finite_components() {
        assert_eq!(finite_vec3(Vec3::new(1.0, f32::INFINITY, 3.0)), Vec3::ZERO);
        assert_eq!(
            finite_vec3(Vec3::new(1.0, 2.0, 3.0)),
            Vec3::new(1.0, 2.0, 3.0)
        );
    }

    /// Motor scalar validation rejects non-finite values and clamps negatives.
    #[test]
    fn motor_scalar_rejects_non_finite_and_negative_values() {
        assert_eq!(finite_nonnegative(f32::NAN), 0.0);
        assert_eq!(finite_nonnegative(-2.0), 0.0);
        assert_eq!(finite_nonnegative(2.0), 2.0);
    }

    /// All six Rapier axes map to the matching lock mask.
    #[test]
    fn axis_masks_cover_linear_and_angular_axes() {
        assert_eq!(axis_mask(JointAxis::AngX), JointAxesMask::ANG_X);
        assert_eq!(axis_mask(JointAxis::AngY), JointAxesMask::ANG_Y);
        assert_eq!(axis_mask(JointAxis::AngZ), JointAxesMask::ANG_Z);
        assert_eq!(axis_mask(JointAxis::LinX), JointAxesMask::LIN_X);
        assert_eq!(axis_mask(JointAxis::LinY), JointAxesMask::LIN_Y);
        assert_eq!(axis_mask(JointAxis::LinZ), JointAxesMask::LIN_Z);
    }
}
