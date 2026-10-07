//! Map profile joints to Avian fixed, revolute, and spherical joints.
//!
//! The core gives each joint a frame in the parent body and X bend, Y twist,
//! and Z bend ranges for the child relative to that frame. All frame handling
//! lives in [`avian_joint`] so a change to the core joint basis only touches
//! this function.
//!
//! Avian 0.7 measures the spherical swing cone around
//! `twist_axis.any_orthonormal_vector()`, and its "twist" limit is the rotation
//! about that same vector. With `twist_axis = X` that vector is `+Y`, so the
//! twist limit bounds rotation about the profile Y axis and the swing limit is a
//! cone around profile Y. The `swing_limit_axis_is_as_documented_in_this_crate`
//! test pins this behavior.

use avian3d::prelude::{FixedJoint, JointCollisionDisabled, RevoluteJoint, SphericalJoint};
use bevy::math::{Isometry3d, Quat, Vec3};
use bevy::prelude::{Added, ChildOf, Commands, Entity, Query};
use bevy_ragdoll::runtime::body::{JointBasis, JointToParent};

/// The Avian joint selected for one profile joint.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum AvianJoint {
    /// Every angular range is locked.
    Fixed(FixedJoint),
    /// Twist and Z bend are locked; X bend keeps its exact asymmetric range.
    Revolute(RevoluteJoint),
    /// Any other combination; the swing cone is symmetric.
    Spherical(SphericalJoint),
}

/// Creates one Avian joint entity for each new profile joint.
///
/// The joint entity is a child of the child body, so despawning the body
/// removes its joint. `JointCollisionDisabled` stops the two joined bodies
/// from colliding with each other.
pub(crate) fn create_avian_joints(
    mut commands: Commands<'_, '_>,
    joints: Query<'_, '_, (Entity, &JointToParent, Option<&JointBasis>), Added<JointToParent>>,
) {
    for (child, joint, basis) in &joints {
        // A missing basis means the limit axes are the child body's own axes.
        let basis = basis.map_or(Quat::IDENTITY, |basis| basis.0);
        let mut joint_entity = commands.spawn((JointCollisionDisabled, ChildOf(child)));
        match avian_joint(child, joint, basis) {
            AvianJoint::Fixed(fixed) => joint_entity.insert(fixed),
            AvianJoint::Revolute(revolute) => joint_entity.insert(revolute),
            AvianJoint::Spherical(spherical) => joint_entity.insert(spherical),
        };
    }
}

/// Builds the Avian joint for one child body and its profile joint.
///
/// `basis` is the [`JointBasis`] rotation: the joint frame sits at
/// `joint.frame.rotation * basis` on the parent and at `basis` on the child.
pub(crate) fn avian_joint(child: Entity, joint: &JointToParent, basis: Quat) -> AvianJoint {
    let limits = joint.limits;
    // Anchor at the parent frame and the child origin, both rotated by the basis.
    let frame1 = Isometry3d::new(joint.frame.translation, joint.frame.rotation * basis);
    let frame2 = Isometry3d::from_rotation(basis);
    // Select the joint kind from which of the X, twist and Z ranges are locked.
    match (
        limits.x.is_locked(),
        limits.twist.is_locked(),
        limits.z.is_locked(),
    ) {
        (true, true, true) => AvianJoint::Fixed(
            FixedJoint::new(joint.parent, child)
                .with_local_frame1(frame1)
                .with_local_frame2(frame2),
        ),
        (false, true, true) => AvianJoint::Revolute(
            RevoluteJoint::new(joint.parent, child)
                .with_local_frame1(frame1)
                .with_local_frame2(frame2)
                .with_hinge_axis(Vec3::X)
                .with_angle_limits(limits.x.min, limits.x.max),
        ),
        // The cone is a hard backstop at the largest bend extent; the core's
        // soft limit torque enforces the asymmetric X and Z ranges inside it.
        _ => AvianJoint::Spherical(
            SphericalJoint::new(joint.parent, child)
                .with_local_frame1(frame1)
                .with_local_frame2(frame2)
                .with_twist_axis(Vec3::X)
                .with_twist_limits(limits.twist.min, limits.twist.max)
                .with_swing_limits(
                    0.0,
                    [limits.x.min, limits.x.max, limits.z.min, limits.z.max]
                        .into_iter()
                        .fold(0.0_f32, |extent, angle| extent.max(angle.abs())),
                ),
        ),
    }
}

#[cfg(test)]
mod tests {
    //! Checks the joint type and limit mapping for each lock combination.

    use std::time::Duration;

    use avian3d::prelude::{
        AngleLimit, AngularDamping, Collider, ConstantTorque, Gravity, JointCollisionDisabled,
        PhysicsPlugins, RigidBody,
    };
    use bevy::math::{Isometry3d, Quat, Vec3};
    use bevy::prelude::{App, Entity, MinimalPlugins, Transform, TransformPlugin, World};
    use bevy::time::TimeUpdateStrategy;
    use bevy_ragdoll::profile::{AngleRange, JointLimits};
    use bevy_ragdoll::runtime::body::JointToParent;

    use super::{AvianJoint, avian_joint};

    /// A locked range.
    const LOCKED: AngleRange = AngleRange { min: 0.0, max: 0.0 };

    /// Builds a profile joint with the given limits at a fixed parent frame.
    fn joint(parent: Entity, limits: JointLimits) -> JointToParent {
        JointToParent {
            parent,
            frame: Isometry3d::from_translation(Vec3::Y),
            limits,
            max_torque: 10.0,
        }
    }

    /// Fully locked joints become fixed joints.
    #[test]
    fn locked_joint_is_fixed() {
        let mut world = World::new();
        let (parent, child) = (world.spawn_empty().id(), world.spawn_empty().id());
        // Lock X, twist and Z.
        let limits = JointLimits {
            x: LOCKED,
            twist: LOCKED,
            z: LOCKED,
        };
        // All three ranges are locked, so the mapping must pick a fixed joint.
        let AvianJoint::Fixed(fixed) = avian_joint(child, &joint(parent, limits), Quat::IDENTITY)
        else {
            panic!("a fully locked joint is fixed");
        };
        assert_eq!((fixed.body1, fixed.body2), (parent, child));
    }

    /// X-only joints become hinges with the exact asymmetric range.
    #[test]
    fn x_only_joint_is_revolute_with_exact_limits() {
        let mut world = World::new();
        let (parent, child) = (world.spawn_empty().id(), world.spawn_empty().id());
        // Free only X with an asymmetric range.
        let limits = JointLimits {
            x: AngleRange {
                min: -0.1,
                max: 2.0,
            },
            twist: LOCKED,
            z: LOCKED,
        };
        // Only X is free, so the mapping must pick a hinge.
        let AvianJoint::Revolute(revolute) =
            avian_joint(child, &joint(parent, limits), Quat::IDENTITY)
        else {
            panic!("an X-only joint is a hinge");
        };
        assert_eq!(revolute.hinge_axis, Vec3::X);
        assert_eq!(revolute.angle_limit, Some(AngleLimit::new(-0.1, 2.0)));
    }

    /// Builds a zero-gravity Avian app that advances 1/60 s per update.
    fn joint_test_app() -> App {
        let mut app = App::new();
        // Remove gravity so only the applied torque moves the child.
        app.add_plugins((MinimalPlugins, TransformPlugin, PhysicsPlugins::default()));
        app.insert_resource(Gravity(Vec3::ZERO));
        app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
            1.0 / 60.0,
        )));
        // Finish plugin setup so the first update steps physics.
        app.finish();
        app.cleanup();
        app
    }

    /// Spawns a static parent sphere and a damped dynamic child sphere one
    /// metre above it under a constant `torque`.
    fn spawn_spin_pair(app: &mut App, torque: Vec3) -> (Entity, Entity) {
        let parent = app
            .world_mut()
            .spawn((
                RigidBody::Static,
                Collider::sphere(0.1),
                Transform::IDENTITY,
            ))
            .id();
        let child = app
            .world_mut()
            .spawn((
                RigidBody::Dynamic,
                Collider::sphere(0.1),
                Transform::from_translation(Vec3::Y),
                ConstantTorque(torque),
                AngularDamping(5.0),
            ))
            .id();
        (parent, child)
    }

    /// Pushes a child with a constant `torque` against a joint to a static
    /// parent built by [`avian_joint`] and returns the child's final rotation.
    fn spin_child(limits: JointLimits, torque: Vec3) -> Quat {
        // Pair a static parent with a torqued child.
        let mut app = joint_test_app();
        let (parent, child) = spawn_spin_pair(&mut app, torque);
        // Insert whichever joint kind the limits select.
        let mut joint_entity = app.world_mut().spawn(JointCollisionDisabled);
        match avian_joint(child, &joint(parent, limits), Quat::IDENTITY) {
            AvianJoint::Spherical(spherical) => joint_entity.insert(spherical),
            AvianJoint::Revolute(revolute) => joint_entity.insert(revolute),
            AvianJoint::Fixed(fixed) => joint_entity.insert(fixed),
        };
        // Two seconds lets the damped child settle against its limit.
        for _ in 0..120 {
            app.update();
        }
        app.world()
            .get::<Transform>(child)
            .map_or(Quat::IDENTITY, |transform| transform.rotation)
    }

    /// Returns the rotation angle about the unit `axis` for a positive-w quaternion.
    fn angle_about(rotation: Quat, axis: Vec3) -> f32 {
        let rotation = if rotation.w < 0.0 {
            -rotation
        } else {
            rotation
        };
        2.0 * Vec3::new(rotation.x, rotation.y, rotation.z)
            .dot(axis)
            .atan2(rotation.w)
    }

    /// Settling tolerance for limit tests, in radians.
    const TOLERANCE: f32 = 0.05;

    /// Twist range used by the swing-axis tests.
    const TWIST: AngleRange = AngleRange {
        min: -0.2,
        max: 0.6,
    };

    /// Limits with a 0.4 rad bend extent and the [`TWIST`] range.
    const fn cone_limits() -> JointLimits {
        let bend = AngleRange {
            min: -0.4,
            max: 0.4,
        };
        JointLimits {
            x: bend,
            twist: TWIST,
            z: bend,
        }
    }

    /// Settles physics-api B5: with the adapter's `twist_axis = X`, Avian's
    /// twist limit stops rotation about the frame Y axis with its signed range.
    #[test]
    fn twist_limit_bounds_rotation_about_y() {
        let positive = angle_about(spin_child(cone_limits(), Vec3::Y * 0.05), Vec3::Y);
        let negative = angle_about(spin_child(cone_limits(), Vec3::Y * -0.05), Vec3::Y);
        assert!(
            (positive - TWIST.max).abs() < TOLERANCE,
            "+Y twist {positive}"
        );
        assert!(
            (negative - TWIST.min).abs() < TOLERANCE,
            "-Y twist {negative}"
        );
    }

    /// The swing cone stops bending about X and about Z at the 0.4 rad extent,
    /// not at the twist range.
    #[test]
    fn swing_cone_bounds_x_and_z_bends() {
        let bend_x = angle_about(spin_child(cone_limits(), Vec3::X * 0.05), Vec3::X);
        let bend_z = angle_about(spin_child(cone_limits(), Vec3::Z * 0.05), Vec3::Z);
        assert!((bend_x - 0.4).abs() < TOLERANCE, "X bend {bend_x}");
        assert!((bend_z - 0.4).abs() < TOLERANCE, "Z bend {bend_z}");
    }

    /// A wide twist range does not limit bending.
    #[test]
    fn wide_twist_does_not_limit_bending() {
        let bend = AngleRange {
            min: -1.0,
            max: 1.0,
        };
        let loose = JointLimits {
            x: bend,
            twist: AngleRange {
                min: -1.5,
                max: 1.5,
            },
            z: bend,
        };
        let loose_bend = angle_about(spin_child(loose, Vec3::X * 0.05), Vec3::X);
        assert!(
            (loose_bend - 1.0).abs() < TOLERANCE,
            "loose X bend {loose_bend}"
        );
    }

    /// Hinges stop at the signed X range.
    #[test]
    fn revolute_limit_sign_matches_the_profile_x_range() {
        // A hinge with a signed X range and locked twist and Z.
        let limits = JointLimits {
            x: AngleRange {
                min: -0.2,
                max: 0.6,
            },
            twist: LOCKED,
            z: LOCKED,
        };
        // Push both ways about the hinge axis, then about a locked axis.
        let positive = angle_about(spin_child(limits, Vec3::X * 0.05), Vec3::X);
        let negative = angle_about(spin_child(limits, Vec3::X * -0.05), Vec3::X);
        assert!((positive - 0.6).abs() < 0.05, "+X hinge {positive}");
        assert!((negative + 0.2).abs() < 0.05, "-X hinge {negative}");
        let locked = spin_child(limits, Vec3::Y * 0.05);
        assert!(
            angle_about(locked, Vec3::Y).abs() < 0.05,
            "hinge twist {locked}"
        );
    }

    /// Other joints become spherical with a cone at the largest bend extent.
    #[test]
    fn free_joint_is_spherical_with_largest_bend_cone() {
        let mut world = World::new();
        let (parent, child) = (world.spawn_empty().id(), world.spawn_empty().id());
        // Free all three ranges with different extents.
        let limits = JointLimits {
            x: AngleRange {
                min: -0.3,
                max: 0.2,
            },
            twist: AngleRange {
                min: -0.5,
                max: 0.4,
            },
            z: AngleRange {
                min: -0.9,
                max: 0.1,
            },
        };
        // Every range is free; the cone takes the 0.9 rad Z minimum.
        let AvianJoint::Spherical(spherical) =
            avian_joint(child, &joint(parent, limits), Quat::IDENTITY)
        else {
            panic!("a free joint is spherical");
        };
        assert_eq!(spherical.twist_axis, Vec3::X);
        assert_eq!(spherical.twist_limit, Some(AngleLimit::new(-0.5, 0.4)));
        assert_eq!(spherical.swing_limit, Some(AngleLimit::new(0.0, 0.9)));
    }

    /// A non-identity basis rotates the parent frame after the profile frame
    /// and becomes the child frame rotation, with both anchors unchanged.
    #[test]
    fn joint_basis_rotates_both_frames() {
        let mut world = World::new();
        let (parent, child) = (world.spawn_empty().id(), world.spawn_empty().id());
        // Rotate both the profile frame and the basis so neither is identity.
        let basis = Quat::from_rotation_z(0.7);
        let free = AngleRange {
            min: -0.5,
            max: 0.5,
        };
        let limits = JointLimits {
            x: free,
            twist: free,
            z: free,
        };
        // Build a free joint so the spherical path carries both frames.
        let mut profile_joint = joint(parent, limits);
        profile_joint.frame.rotation = Quat::from_rotation_y(0.3);
        let AvianJoint::Spherical(spherical) = avian_joint(child, &profile_joint, basis) else {
            panic!("a free joint is spherical");
        };
        // Avian stores both frames as local isometries.
        let frames = (
            spherical.frame1.get_local_isometry(),
            spherical.frame2.get_local_isometry(),
        );
        let (Some(parent_frame), Some(child_frame)) = frames else {
            panic!("both frames are local");
        };
        // Rotations match up to float error; anchors match exactly.
        assert!(
            parent_frame
                .rotation
                .angle_between(profile_joint.frame.rotation * basis)
                < 1.0e-6
        );
        assert!(child_frame.rotation.angle_between(basis) < 1.0e-6);
        assert_eq!(
            (
                Vec3::from(parent_frame.translation),
                Vec3::from(child_frame.translation)
            ),
            (Vec3::Y, Vec3::ZERO)
        );
    }
}
