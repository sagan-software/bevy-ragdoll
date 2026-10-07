//! Physics-neutral components exchanged between the runtime and one backend
//! body.
//!
//! The runtime creates these values from validated profile data and writes
//! drive targets before the backend's fixed-step apply stage. A backend updates
//! pose, velocity, contact, and sleep state after stepping, then reads the next
//! targets on the following tick. Geometry and mass use metres and kilograms so
//! backend adapters can perform unit conversion at their integration boundary.

use bevy::math::{Isometry3d, Quat, Vec3};
use bevy::prelude::{Component, Entity};

use crate::profile::{JointLimits, ShapeSpec};

/// The profile-validated collision geometry expressed in the physics body's
/// local frame.
///
/// Shapes are copied from profile assets when body entities spawn and remain
/// unchanged while that ragdoll exists. Backends interpret capsule endpoints,
/// sphere centres, and cuboid extents in metres, then apply the body's current
/// pose when querying contacts or ray intersections.
#[derive(Component, Clone, Copy, Debug, PartialEq, bevy::prelude::Reflect)]
pub struct BodyShape(pub ShapeSpec);

/// The body's mass and minimum principal inertia radius used by backend
/// adapters.
///
/// Mass is measured in kilograms and the radius in metres. The profile
/// validates positive finite mass; the minimum radius prevents nearly
/// point-like geometry from producing unstable principal inertia during
/// backend-specific mass-property construction.
#[derive(Component, Clone, Copy, Debug, PartialEq, bevy::prelude::Reflect)]
pub struct BodyMass {
    /// Positive body mass in kilograms, validated before the runtime creates
    /// this entity. The runtime stores or reads this value through the shared
    /// backend contract during fixed simulation.
    pub mass: f32,
    /// Minimum radius in metres used when estimating each principal inertia
    /// component. The runtime stores or reads this value through the shared
    /// backend contract during fixed simulation.
    pub min_inertia_radius: f32,
}

/// Linear and angular body velocity in world coordinates after the latest
/// physics step.
///
/// Linear velocity is measured in metres per second and angular velocity in
/// radians per second. The runtime also uses this type for animation-derived
/// targets, transforming those values into world coordinates before drive
/// calculations so backend and animation values share one frame.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, bevy::prelude::Reflect)]
pub struct BodyVelocity {
    /// World-space linear velocity in metres per second, with positive axes
    /// matching Bevy transforms. The runtime stores or reads this value through
    /// the shared backend contract during fixed simulation.
    pub linear: Vec3,
    /// World-space angular velocity in radians per second, using the
    /// right-handed rotation frame. The runtime stores or reads this value
    /// through the shared backend contract during fixed simulation.
    pub angular: Vec3,
}

/// Previous and current world-space rigid body poses surrounding a backend
/// step.
///
/// The backend copies `current` to `previous` before integration, then stores
/// the new pose in `current`. Render-time writeback interpolates these
/// isometries using fixed-step overstep and therefore expects both values to
/// contain rigid transforms without scale.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, bevy::prelude::Reflect)]
pub struct BodyPhysicsPose {
    /// Rigid pose in world space before the most recent backend integration
    /// step began. The runtime stores or reads this value through the shared
    /// backend contract during fixed simulation.
    pub previous: Isometry3d,
    /// Rigid pose in world space after the most recent backend integration step
    /// completed. The runtime stores or reads this value through the shared
    /// backend contract during fixed simulation.
    pub current: Isometry3d,
}

/// Pin and fallback joint forces computed before the backend applies its next
/// fixed step.
///
/// Forces use newtons and torques use newton metres, all expressed in world
/// coordinates. Native joint backends consume `joint_torque` as zero, while
/// fallback backends apply equal and opposite torques to parent and child
/// bodies to preserve angular momentum.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, bevy::prelude::Reflect)]
pub struct BodyDriveOutput {
    /// World-space force in newtons pulling this body toward its animated
    /// target pose. The runtime stores or reads this value through the shared
    /// backend contract during fixed simulation.
    pub pin_force: Vec3,
    /// World-space torque in newton metres rotating this body toward its
    /// animated target pose. The runtime stores or reads this value through the
    /// shared backend contract during fixed simulation.
    pub pin_torque: Vec3,
    /// World-space fallback torque in newton metres for backends without native
    /// joint motors. The runtime stores or reads this value through the shared
    /// backend contract during fixed simulation.
    pub joint_torque: Vec3,
    /// Maximum world-space linear speed in metres per second while muscle drive
    /// is active. `None` disables this limit for a fully limp character.
    pub max_linear_speed: Option<f32>,
    /// Maximum world-space angular speed in radians per second while muscle
    /// drive is active. `None` disables this limit for a fully limp character.
    pub max_angular_speed: Option<f32>,
}

impl BodyDriveOutput {
    /// Clamps finite linear and angular velocities to the limits in this output.
    ///
    /// A missing or invalid limit is ignored. A non-finite velocity component
    /// becomes zero before a backend uses it for integration.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy::math::Vec3;
    /// use bevy_ragdoll::runtime::body::{BodyDriveOutput, BodyVelocity};
    ///
    /// let output = BodyDriveOutput { max_linear_speed: Some(5.0), ..Default::default() };
    /// let mut velocity = BodyVelocity { linear: Vec3::X * 8.0, ..Default::default() };
    /// output.clamp_velocity(&mut velocity);
    /// assert!((velocity.linear.length() - 5.0).abs() < 1.0e-5);
    /// ```
    pub fn clamp_velocity(&self, velocity: &mut BodyVelocity) {
        clamp_speed(&mut velocity.linear, self.max_linear_speed);
        clamp_speed(&mut velocity.angular, self.max_angular_speed);
    }
}

/// Applies one optional finite nonnegative magnitude cap to a velocity vector.
fn clamp_speed(vector: &mut Vec3, maximum: Option<f32>) {
    // Clear invalid velocity values before evaluating any optional cap.
    if !vector.is_finite() {
        *vector = Vec3::ZERO;
        return;
    }
    // Preserve uncapped or below-cap values without changing their direction.
    let Some(maximum) = maximum.filter(|maximum| maximum.is_finite() && *maximum >= 0.0) else {
        return;
    };
    let speed = vector.length();
    if speed > maximum {
        *vector *= maximum / speed;
    }
}

#[cfg(test)]
mod tests {
    //! Boundary coverage for backend-neutral velocity limits.

    use bevy::math::Vec3;

    use super::{BodyDriveOutput, BodyVelocity};

    /// Applies independent finite limits without changing values below either cap.
    #[test]
    fn velocity_limits_clamp_only_excess_speed() {
        let output = BodyDriveOutput {
            max_linear_speed: Some(10.0),
            max_angular_speed: Some(20.0),
            ..Default::default()
        };
        let mut velocity = BodyVelocity {
            linear: Vec3::X * 12.0,
            angular: Vec3::Y * 24.0,
        };

        output.clamp_velocity(&mut velocity);

        assert!((velocity.linear.length() - 10.0).abs() < 1.0e-5);
        assert!((velocity.angular.length() - 20.0).abs() < 1.0e-5);
    }

    /// Keeps velocity unchanged when limits are absent and clears malformed vectors.
    #[test]
    fn velocity_limits_ignore_absent_caps_and_clear_nonfinite_values() {
        let output = BodyDriveOutput::default();
        let mut velocity = BodyVelocity {
            linear: Vec3::X * 12.0,
            angular: Vec3::splat(f32::NAN),
        };

        output.clamp_velocity(&mut velocity);

        assert_eq!(velocity.linear, Vec3::X * 12.0);
        assert_eq!(velocity.angular, Vec3::ZERO);
    }

    /// Ignores a non-finite cap and clamps to a finite zero cap.
    #[test]
    fn velocity_limits_validate_public_caps() {
        let output = BodyDriveOutput {
            max_linear_speed: Some(f32::NAN),
            max_angular_speed: Some(-1.0),
            ..Default::default()
        };
        let mut velocity = BodyVelocity {
            linear: Vec3::X,
            angular: Vec3::Y,
        };

        output.clamp_velocity(&mut velocity);

        assert_eq!(velocity.linear, Vec3::X);
        assert_eq!(velocity.angular, Vec3::Y);

        let zero_limit = BodyDriveOutput {
            max_angular_speed: Some(0.0),
            ..Default::default()
        };
        zero_limit.clamp_velocity(&mut velocity);
        assert_eq!(velocity.angular, Vec3::ZERO);
    }
}

/// Motion state that tells a backend how to integrate or constrain one body.
///
/// `Kinematic` bodies follow the current animation target, `Dynamic` bodies
/// integrate forces, and `Fixed` bodies retain their last pose. The runtime
/// maps ragdoll modes to these states before the backend's apply stage and
/// updates the state when a character changes mode.
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq, bevy::prelude::Reflect)]
pub enum BodyKind {
    /// Follows the latest target pose without force integration and remains
    /// queryable by physics. The backend follows targets without force
    /// integration, while contacts and query results remain available to
    /// callers.
    Kinematic,
    /// Integrates forces, torques, and impulses through the active physics
    /// backend. The backend integrates forces and impulses, then reports
    /// updated poses and velocities for the next core step.
    Dynamic,
    /// Remains fixed at the current pose until the runtime changes its ragdoll
    /// mode. The backend preserves the current pose and excludes this body from
    /// dynamic integration until its kind changes.
    Fixed,
}

/// The parent body and validated constraint data for one non-root profile body.
///
/// The joint frame is local to the parent, limits are X bend, Y twist, and Z
/// bend radians, and `max_torque` is measured in newton metres. Backends use
/// the referenced parent entity when they build or update the constraint
/// corresponding to this child body.
#[derive(Component, Clone, Copy, Debug, PartialEq, bevy::prelude::Reflect)]
pub struct JointToParent {
    /// Parent physics entity whose frame anchors this child's joint constraint.
    /// The runtime stores or reads this value through the shared backend
    /// contract during fixed simulation.
    pub parent: Entity,
    /// Validated joint frame relative to the parent body, including translation
    /// and rotation. The runtime stores or reads this value through the shared
    /// backend contract during fixed simulation.
    pub frame: Isometry3d,
    /// Allowed child-frame angular ranges in radians for X bend, Y twist, and Z
    /// bend. The runtime stores or reads this value through the shared backend
    /// contract during fixed simulation.
    pub limits: JointLimits,
    /// Maximum motor torque in newton metres, copied from the validated profile
    /// joint. The runtime stores or reads this value through the shared backend
    /// contract during fixed simulation.
    pub max_torque: f32,
}

/// Limit axes of a joint in the child body frame, present only when they
/// differ from the child body's own axes.
///
/// Backends place the joint frame at `JointToParent::frame.rotation * basis`
/// on the parent body and at `basis` on the child body, and apply
/// `JointToParent::limits` and `JointDriveTarget::rotation` about those axes.
#[derive(Component, Clone, Copy, Debug, PartialEq, bevy::prelude::Reflect)]
pub struct JointBasis(pub Quat);

/// Target rotation, angular velocity, and gains for a backend's native joint
/// motor.
///
/// Rotation is relative to the parent joint frame, angular velocity uses
/// radians per second, and stiffness and damping are acceleration-based
/// coefficients. A backend clamps the resulting motor torque to `max_torque`
/// rather than interpreting these values as impulses.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, bevy::prelude::Reflect)]
pub struct JointDriveTarget {
    /// Desired child rotation relative to the parent joint frame after target
    /// composition. The runtime stores or reads this value through the shared
    /// backend contract during fixed simulation.
    pub rotation: bevy::math::Quat,
    /// Desired relative angular velocity in radians per second for the child
    /// joint frame. The runtime stores or reads this value through the shared
    /// backend contract during fixed simulation.
    pub angular_velocity: Vec3,
    /// Acceleration-based angular stiffness in inverse seconds squared, scaled
    /// by muscle strength. The runtime stores or reads this value through the
    /// shared backend contract during fixed simulation.
    pub stiffness: f32,
    /// Acceleration-based angular damping in inverse seconds, including
    /// configured friction rate. The runtime stores or reads this value through
    /// the shared backend contract during fixed simulation.
    pub damping: f32,
    /// Upper bound for the backend motor torque in newton metres during this
    /// fixed step. The runtime stores or reads this value through the shared
    /// backend contract during fixed simulation.
    pub max_torque: f32,
}

/// Contact-exclusion mask copied from the validated profile for this body
/// index.
///
/// Bit `j` excludes profile body `j` from contact with this body, and the
/// profile stores symmetric bits for each excluded pair. Backends apply the
/// mask during collider setup and must retain its interpretation for the
/// lifetime of the spawned body entities.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq, bevy::prelude::Reflect)]
pub struct NoContactWith(pub u64);

/// Marks a body that the backend reports as sleeping after its latest completed
/// physics step.
///
/// The runtime uses this marker when updating settle state, while the backend
/// removes it as soon as a force, impulse, target change, or collision wakes
/// the body for additional integration.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq, bevy::prelude::Reflect)]
pub struct BodyAtRest;
