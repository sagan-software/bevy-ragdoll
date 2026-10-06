//! Physics-neutral components exchanged between the runtime and one backend
//! body.
//!
//! The runtime creates these values from validated profile data and writes
//! drive targets before the backend's fixed-step apply stage. A backend updates
//! pose, velocity, contact, and sleep state after stepping, then reads the next
//! targets on the following tick. Geometry and mass use metres and kilograms so
//! backend adapters can perform unit conversion at their integration boundary.

use bevy::math::{Isometry3d, Vec3};
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
