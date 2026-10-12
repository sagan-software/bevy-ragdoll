//! Map profile joints to Avian fixed, revolute, and spherical joints.
//!
//! The core gives each joint a frame in the parent body and X bend, Y twist,
//! and Z bend ranges for the child relative to that frame. All frame handling
//! lives in [`avian_joint`] so a change to the core joint basis only touches
//! this function.
//!
//! Avian 0.8 measures the spherical swing cone around
//! `twist_axis.any_orthonormal_vector()`, and its "twist" limit is the rotation
//! about that same vector. With `twist_axis = X` that vector is `+Y`, so the
//! twist limit bounds rotation about the profile Y axis and the swing limit is a
//! cone around profile Y. The `swing_limit_axis_is_as_documented_in_this_crate`
//! test pins this behavior.

use std::collections::HashMap;

use avian3d::prelude::{FixedJoint, JointCollisionDisabled, RevoluteJoint, SphericalJoint};
use bevy::math::{Isometry3d, Quat, Vec3};
use bevy::prelude::{Added, ChildOf, Commands, Entity, Query, Transform};
use bevy_ragdoll::profile::{AngleRange, BodyIndex, JointLimits};
use bevy_ragdoll::runtime::body::{BodyPhysicsPose, BodyShape, JointBasis, JointToParent};

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

/// New bodies with their profile index, pose and optional joint.
type NewJointBodies<'w, 's> = Query<
    'w,
    's,
    (
        Entity,
        &'static BodyIndex,
        &'static mut Transform,
        &'static mut BodyPhysicsPose,
        Option<&'static JointToParent>,
        Option<&'static JointBasis>,
    ),
    Added<BodyShape>,
>;

/// Turns each new body into its joint limits before Avian reads its pose.
///
/// A ragdoll that activates from an animated pose can start with joints past
/// their limits. Avian corrects a limit violation within one substep, which
/// turns a large violation into a launch. Bodies are visited parent first:
/// each child keeps its rotation clamped into its parent's limits and moves
/// to its parent's joint anchor, so its own children are then fitted against
/// the corrected pose.
pub(crate) fn fit_new_bodies_to_limits(mut bodies: NewJointBodies<'_, '_>) {
    // Profile indexes are parent first, so sorting by index visits parents first.
    let mut order = bodies
        .iter()
        .map(|(entity, index, ..)| (index.get(), entity))
        .collect::<Vec<_>>();
    if order.is_empty() {
        return;
    }
    order.sort_unstable();
    // Fitted poses of bodies already visited, read by their children.
    let mut poses = HashMap::new();
    for (_, entity) in order {
        let Ok((_, _, mut transform, mut pose, joint, basis)) = bodies.get_mut(entity) else {
            continue;
        };
        // A root, or a child whose parent is not new, keeps its current pose.
        let current = Isometry3d::new(transform.translation, transform.rotation);
        let fitted = joint
            .and_then(|joint| {
                let parent = poses.get(&joint.parent)?;
                let basis = basis.map_or(Quat::IDENTITY, |basis| basis.0);
                Some(fit_child_pose(*parent, current, joint, basis))
            })
            .unwrap_or(current);
        poses.insert(entity, fitted);
        // Write both readback poses so interpolation starts from the fitted pose.
        if fitted != current {
            transform.translation = fitted.translation.into();
            transform.rotation = fitted.rotation;
            pose.previous = fitted;
            pose.current = fitted;
        }
    }
}

/// Returns the child pose with its rotation clamped into the joint limits and
/// its origin on the parent's joint anchor.
fn fit_child_pose(
    parent: Isometry3d,
    child: Isometry3d,
    joint: &JointToParent,
    basis: Quat,
) -> Isometry3d {
    // Express the child rotation in the joint frame on the parent.
    let anchor = parent * Isometry3d::new(joint.frame.translation, joint.frame.rotation * basis);
    let relative = anchor.rotation.inverse() * child.rotation * basis;
    let fitted = fit_relative_rotation(joint.limits, relative);
    // A degenerate input rotation leaves the child where it was.
    if !fitted.is_finite() {
        return child;
    }
    Isometry3d::new(
        anchor.translation,
        anchor.rotation * fitted * basis.inverse(),
    )
}

/// Fraction of each limit used when clamping, so the fitted pose starts just
/// inside the limit rather than on it.
// provenance: chosen so a fitted joint needs no correction on its first step.
const LIMIT_FIT: f32 = 0.98;

/// Clamps a rotation in the joint frame into the limits as [`avian_joint`]
/// maps them: nothing for a fixed joint, the X or Z angle for a hinge, and a
/// twist about Y plus a symmetric swing cone around Y for a spherical joint.
pub(crate) fn fit_relative_rotation(limits: JointLimits, rotation: Quat) -> Quat {
    let rotation = rotation.normalize();
    // Match the joint kind selection in `avian_joint`.
    match (
        limits.x.is_locked(),
        limits.twist.is_locked(),
        limits.z.is_locked(),
    ) {
        (true, true, true) => Quat::IDENTITY,
        // A hinge keeps only its angle about the hinge axis.
        (false, true, true) => {
            let angle = 2.0 * rotation.x.atan2(rotation.w);
            Quat::from_rotation_x(clamp_angle(angle, limits.x.min, limits.x.max))
        }
        (true, true, false) => {
            let angle = 2.0 * rotation.z.atan2(rotation.w);
            Quat::from_rotation_z(clamp_angle(angle, limits.z.min, limits.z.max))
        }
        _ => fit_swing_twist(limits, rotation),
    }
}

/// Clamps a spherical joint rotation: the twist about Y into the twist range
/// and the swing of the Y axis into the cone from [`swing_cone`].
fn fit_swing_twist(limits: JointLimits, rotation: Quat) -> Quat {
    // Split the rotation into a twist about Y and a swing of the Y axis.
    // A rotation of a half turn about an axis across Y has no twist part.
    let twist_length = rotation.y.hypot(rotation.w);
    let twist = if twist_length > f32::EPSILON {
        Quat::from_xyzw(
            0.0,
            rotation.y / twist_length,
            0.0,
            rotation.w / twist_length,
        )
    } else {
        Quat::IDENTITY
    };
    // The positive-w representative keeps the swing angle at or below a half turn.
    let swing = rotation * twist.inverse();
    let swing = if swing.w < 0.0 { -swing } else { swing };
    // Clamp the twist angle into its range.
    let twist_angle = 2.0 * twist.y.atan2(twist.w);
    let twist = Quat::from_rotation_y(clamp_angle(twist_angle, limits.twist.min, limits.twist.max));
    // Pull a swing past the cone back to just inside it, keeping its axis.
    let cone = swing_cone(limits);
    let (axis, angle) = swing.to_axis_angle();
    let swing = if angle > cone {
        Quat::from_axis_angle(axis, cone * LIMIT_FIT)
    } else {
        swing
    };
    swing * twist
}

/// Returns the symmetric swing cone: the largest X or Z bend extent.
fn swing_cone(limits: JointLimits) -> f32 {
    [limits.x.min, limits.x.max, limits.z.min, limits.z.max]
        .into_iter()
        .fold(0.0_f32, |extent, angle| extent.max(angle.abs()))
}

/// Keeps a wrapped angle inside `min..=max`; an angle outside goes to the
/// range shrunk toward its middle by [`LIMIT_FIT`].
fn clamp_angle(angle: f32, min: f32, max: f32) -> f32 {
    // Wrap into `-PI..PI` so the comparison matches the limit convention.
    let wrapped =
        (angle + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI;
    if (min..=max).contains(&wrapped) {
        return wrapped;
    }
    // Clamp into the shrunken range so the fitted angle sits just inside.
    let middle = f32::midpoint(min, max);
    let half = (max - min) * 0.5 * LIMIT_FIT;
    wrapped.clamp(middle - half, middle + half)
}

/// Builds the Avian joint for one child body and its profile joint.
///
/// `basis` is the [`JointBasis`] rotation: the joint frame sits at
/// `joint.frame.rotation * basis` on the parent and at `basis` on the child.
pub(crate) fn avian_joint(child: Entity, joint: &JointToParent, basis: Quat) -> AvianJoint {
    let limits = joint.limits;
    // Anchor at the parent frame and the child origin, both rotated by the basis.
    let frames = [
        Isometry3d::new(joint.frame.translation, joint.frame.rotation * basis),
        Isometry3d::from_rotation(basis),
    ];
    // Select the joint kind from which of the X, twist and Z ranges are locked.
    match (
        limits.x.is_locked(),
        limits.twist.is_locked(),
        limits.z.is_locked(),
    ) {
        (true, true, true) => AvianJoint::Fixed(
            FixedJoint::new(joint.parent, child)
                .with_local_frame1(frames[0])
                .with_local_frame2(frames[1]),
        ),
        (false, true, true) => {
            AvianJoint::Revolute(hinge(joint.parent, child, frames, Vec3::X, limits.x))
        }
        // A Z-only range is a hinge too; a wide swing cone with a locked twist
        // would leave the twist split ill-conditioned near a half turn.
        (true, true, false) => {
            AvianJoint::Revolute(hinge(joint.parent, child, frames, Vec3::Z, limits.z))
        }
        // The cone is a hard backstop at the largest bend extent; the core's
        // soft limit torque enforces the asymmetric X and Z ranges inside it.
        _ => AvianJoint::Spherical(
            SphericalJoint::new(joint.parent, child)
                .with_local_frame1(frames[0])
                .with_local_frame2(frames[1])
                .with_twist_axis(Vec3::X)
                .with_twist_limits(limits.twist.min, limits.twist.max)
                .with_swing_limits(0.0, swing_cone(limits)),
        ),
    }
}

/// Builds a revolute joint about `axis` that keeps the exact asymmetric range.
fn hinge(
    parent: Entity,
    child: Entity,
    [frame1, frame2]: [Isometry3d; 2],
    axis: Vec3,
    range: AngleRange,
) -> RevoluteJoint {
    RevoluteJoint::new(parent, child)
        .with_local_frame1(frame1)
        .with_local_frame2(frame2)
        .with_hinge_axis(axis)
        .with_angle_limits(range.min, range.max)
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

    use super::{AvianJoint, avian_joint, fit_new_bodies_to_limits, fit_relative_rotation};

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

    /// Z-only joints become hinges about Z that stop at the signed Z range.
    #[test]
    fn z_only_joint_is_revolute_with_the_signed_z_range() {
        // A knee-like hinge: X and twist locked, a one-sided Z range.
        let limits = JointLimits {
            x: LOCKED,
            twist: LOCKED,
            z: AngleRange {
                min: -0.6,
                max: 0.2,
            },
        };
        let mut world = World::new();
        let (parent, child) = (world.spawn_empty().id(), world.spawn_empty().id());
        let AvianJoint::Revolute(revolute) =
            avian_joint(child, &joint(parent, limits), Quat::IDENTITY)
        else {
            panic!("a Z-only joint is a hinge");
        };
        assert_eq!(revolute.hinge_axis, Vec3::Z);
        // Push both ways about the hinge axis.
        let positive = angle_about(spin_child(limits, Vec3::Z * 0.05), Vec3::Z);
        let negative = angle_about(spin_child(limits, Vec3::Z * -0.05), Vec3::Z);
        assert!((positive - 0.2).abs() < TOLERANCE, "+Z hinge {positive}");
        assert!((negative + 0.6).abs() < TOLERANCE, "-Z hinge {negative}");
    }

    /// An X hinge from -0.2 to 0.6 rad.
    const HINGE_X: JointLimits = JointLimits {
        x: AngleRange {
            min: -0.2,
            max: 0.6,
        },
        twist: LOCKED,
        z: LOCKED,
    };

    /// A Z hinge from -2.0 to 0.0 rad.
    const HINGE_Z: JointLimits = JointLimits {
        x: LOCKED,
        twist: LOCKED,
        z: AngleRange {
            min: -2.0,
            max: 0.0,
        },
    };

    /// Hinge rotations inside their ranges, including a range end, are kept.
    #[test]
    fn fit_keeps_hinge_rotations_inside_the_limits() {
        // A rotation strictly inside the X range is unchanged.
        let inside = Quat::from_rotation_x(0.3);
        assert!(fit_relative_rotation(HINGE_X, inside).angle_between(inside) < 1.0e-5);
        // A hinge at the end of its range stays put.
        let straight = fit_relative_rotation(HINGE_Z, Quat::IDENTITY);
        assert!(
            straight.angle_between(Quat::IDENTITY) < 1.0e-5,
            "{straight}"
        );
        let bent = Quat::from_rotation_z(-1.0);
        assert!(fit_relative_rotation(HINGE_Z, bent).angle_between(bent) < 1.0e-5);
    }

    /// Hinge rotations outside their ranges go to the near end, and off-axis
    /// parts are dropped.
    #[test]
    fn fit_clamps_hinge_rotations_onto_the_near_end() {
        // A bend past 0.6 rad with a twist goes to 0.6 rad without the twist.
        let past = Quat::from_rotation_x(1.5) * Quat::from_rotation_y(0.4);
        let fitted = fit_relative_rotation(HINGE_X, past);
        assert!(
            (angle_about(fitted, Vec3::X) - 0.6).abs() < 0.03,
            "{fitted}"
        );
        assert!(angle_about(fitted, Vec3::Y).abs() < 1.0e-5);
        // A bend the wrong way about Z goes to the 0.0 rad end.
        let wrong_way = fit_relative_rotation(HINGE_Z, Quat::from_rotation_z(1.0));
        assert!(angle_about(wrong_way, Vec3::Z).abs() < 0.03, "{wrong_way}");
    }

    /// Fixed joints fit to the joint frame.
    #[test]
    fn fit_turns_fixed_joints_to_the_frame() {
        let fixed = JointLimits {
            x: LOCKED,
            twist: LOCKED,
            z: LOCKED,
        };
        let past = Quat::from_rotation_x(1.5) * Quat::from_rotation_y(0.4);
        assert_eq!(fit_relative_rotation(fixed, past), Quat::IDENTITY);
    }

    /// Spherical joints clamp the twist about Y and the swing cone, and keep
    /// a small swing.
    #[test]
    fn fit_clamps_spherical_twist_and_swing() {
        let cone = cone_limits();
        // A twist past its range is clamped to the near end.
        let twisted = fit_relative_rotation(cone, Quat::from_rotation_y(1.5));
        assert!(
            (angle_about(twisted, Vec3::Y) - TWIST.max).abs() < 0.03,
            "{twisted}"
        );
        // A swing past the 0.4 rad cone is pulled inside it.
        let swung = fit_relative_rotation(cone, Quat::from_rotation_x(1.2));
        assert!(
            swung.angle_between(Quat::IDENTITY) <= 0.4 + 1.0e-4,
            "{swung}"
        );
        let small = Quat::from_rotation_z(0.2);
        assert!(fit_relative_rotation(cone, small).angle_between(small) < 1.0e-5);
    }

    /// A half-turn swing across Y has no twist part and still fits.
    #[test]
    fn fit_handles_a_half_turn_swing() {
        let flipped =
            fit_relative_rotation(cone_limits(), Quat::from_rotation_x(std::f32::consts::PI));
        assert!(flipped.is_finite() && flipped.angle_between(Quat::IDENTITY) <= 0.4 + 1.0e-4);
    }

    /// Spawns a root body and a child bent 1.5 rad past a 0.6 rad X hinge,
    /// runs the fit system once, and returns the world, root and child.
    fn fitted_hinge_world() -> (World, Entity, Entity) {
        use bevy::ecs::system::RunSystemOnce;
        use bevy_ragdoll::profile::{BodyIndex, ShapeSpec};
        use bevy_ragdoll::runtime::body::{BodyPhysicsPose, BodyShape};

        let mut world = World::new();
        let shape = BodyShape(ShapeSpec::Sphere {
            center: Vec3::ZERO,
            radius: 0.1,
        });
        // Each body starts with matching transform and readback poses.
        let body = |index: usize, pose: Isometry3d| {
            (
                BodyIndex::try_from(index).expect("small index"),
                shape,
                Transform::from_translation(pose.translation.into()).with_rotation(pose.rotation),
                BodyPhysicsPose {
                    previous: pose,
                    current: pose,
                },
            )
        };
        let parent = world.spawn(body(0, Isometry3d::IDENTITY)).id();
        // The child sits off the anchor and bent past the hinge range.
        let start = Isometry3d::new(Vec3::new(0.2, 1.0, 0.0), Quat::from_rotation_x(1.5));
        let child = world.spawn((body(1, start), joint(parent, HINGE_X))).id();
        world
            .run_system_once(fit_new_bodies_to_limits)
            .expect("the fit system runs");
        (world, parent, child)
    }

    /// A new child body past its limit is turned inside the limit and moved
    /// onto its parent's joint anchor before Avian reads it.
    #[test]
    fn new_bodies_start_inside_their_limits() {
        use bevy_ragdoll::runtime::body::BodyPhysicsPose;

        let (world, _, child) = fitted_hinge_world();
        // The child moves onto the anchor and turns to the 0.6 rad end.
        let transform = world.get::<Transform>(child).copied().unwrap_or_default();
        assert!(
            (transform.translation - Vec3::Y).length() < 1.0e-5,
            "{transform:?}"
        );
        assert!((angle_about(transform.rotation, Vec3::X) - 0.6).abs() < 0.03);
        // The readback pose follows the fitted transform.
        let pose = world.get::<BodyPhysicsPose>(child).map(|pose| pose.current);
        assert_eq!(pose.map(|pose| pose.rotation), Some(transform.rotation));
    }

    /// The root body has no joint and keeps its pose.
    #[test]
    fn new_root_bodies_keep_their_pose() {
        let (world, parent, _) = fitted_hinge_world();
        assert_eq!(
            world.get::<Transform>(parent).copied(),
            Some(Transform::IDENTITY)
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
