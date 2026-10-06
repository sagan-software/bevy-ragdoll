//! A validated joint entry between profile bodies.

use bevy::math::Isometry3d;

use super::{AngleRange, BodyIndex, JointLimits};

/// A joint whose body indices, frame, limits, and torque passed validation.
///
/// The child always follows its parent in profile order. Private fields keep
/// malformed indexes and unvalidated limit combinations out of runtime access.
#[derive(Clone, Copy, Debug, PartialEq, bevy::prelude::Reflect)]
pub struct Joint {
    /// The child body index.
    child: BodyIndex,
    /// The parent body index.
    parent: BodyIndex,
    /// The child's rest frame expressed in the parent body frame.
    frame: Isometry3d,
    /// The allowed angular ranges about X, twist, and Z.
    limits: JointLimits,
    /// The maximum motor torque in newton metres.
    max_torque: f32,
}

impl Joint {
    /// Creates a joint after its source spec has passed validation.
    pub(super) const fn new(
        child: BodyIndex,
        parent: BodyIndex,
        frame: Isometry3d,
        limits: JointLimits,
        max_torque: f32,
    ) -> Self {
        Self {
            child,
            parent,
            frame,
            limits,
            max_torque,
        }
    }

    /// Returns the checked child body index constrained by this joint.
    ///
    /// The child index follows the parent-first body order and cannot refer to
    /// the root, because the root has no incoming joint.
    ///
    /// # Examples
    ///
    /// ```
    /// # use bevy_ragdoll::Joint;
    /// # fn child(joint: &Joint) -> usize {
    /// joint.child().get()
    /// # }
    /// ```
    pub const fn child(&self) -> BodyIndex {
        self.child
    }

    /// Returns the checked parent body index that anchors this joint.
    ///
    /// The parent precedes the child in profile order, which lets backends
    /// construct joint hierarchies without resolving an arbitrary graph.
    ///
    /// # Examples
    ///
    /// ```
    /// # use bevy_ragdoll::Joint;
    /// # fn parent(joint: &Joint) -> usize {
    /// joint.parent().get()
    /// # }
    /// ```
    pub const fn parent(&self) -> BodyIndex {
        self.parent
    }

    /// Returns the validated child joint frame relative to the parent body.
    ///
    /// The returned rigid transform expresses the joint anchor used by a
    /// backend when it creates a constraint between the two profile bodies.
    ///
    /// # Examples
    ///
    /// ```
    /// # use bevy::math::Isometry3d;
    /// # use bevy_ragdoll::Joint;
    /// # fn frame(joint: &Joint) -> Isometry3d {
    /// joint.frame()
    /// # }
    /// ```
    pub const fn frame(&self) -> Isometry3d {
        self.frame
    }

    /// Returns validated X bend, Y twist, and Z bend limits measured in radians.
    ///
    /// Each interval contains zero and stays within the inclusive `-PI..=PI`
    /// range, so backends receive the same angular convention.
    ///
    /// # Examples
    ///
    /// ```
    /// # use bevy_ragdoll::{Joint, JointLimits};
    /// # fn limits(joint: &Joint) -> JointLimits {
    /// joint.limits()
    /// # }
    /// ```
    pub const fn limits(&self) -> JointLimits {
        self.limits
    }

    /// Returns the finite nonnegative motor torque ceiling in newton metres.
    ///
    /// A zero value disables motor force; a positive value caps the backend's
    /// actuator torque without changing the joint's angular limits.
    ///
    /// # Examples
    ///
    /// ```
    /// # use bevy_ragdoll::Joint;
    /// # fn torque(joint: &Joint) -> f32 {
    /// joint.max_torque()
    /// # }
    /// ```
    pub const fn max_torque(&self) -> f32 {
        self.max_torque
    }

    /// Returns whether the Y twist and Z bend axes are locked at zero radians.
    ///
    /// A true result identifies an X-axis hinge after profile validation has
    /// accepted every angular interval and the joint frame.
    ///
    /// # Examples
    ///
    /// ```
    /// # use bevy_ragdoll::Joint;
    /// # fn is_hinge(joint: &Joint) -> bool {
    /// joint.is_hinge()
    /// # }
    /// ```
    pub const fn is_hinge(&self) -> bool {
        self.limits.is_hinge()
    }

    /// Returns the validated X-axis bend interval for callers needing that axis.
    ///
    /// The copied value remains in radians and retains its inclusive minimum
    /// and maximum endpoints from the full joint limits.
    ///
    /// # Examples
    ///
    /// ```
    /// # use bevy_ragdoll::{AngleRange, Joint};
    /// # fn bend(joint: &Joint) -> AngleRange {
    /// joint.bend_range()
    /// # }
    /// ```
    pub const fn bend_range(&self) -> AngleRange {
        self.limits.x
    }
}
