//! Joint angle ranges and axes used by a validated profile.

/// A joint-angle interval in radians that contains zero and stays in `-PI..=PI`.
///
/// Profile validation requires finite endpoints, zero within the interval, and
/// both bounds inside the inclusive `-PI..=PI` range before runtime use.
#[derive(Clone, Copy, Debug, PartialEq, bevy::prelude::Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "serialize", serde(deny_unknown_fields))]
pub struct AngleRange {
    /// Inclusive lower endpoint in radians; validated joint angles may not
    /// fall below this value when the constrained child body is evaluated.
    pub min: f32,
    /// Inclusive upper endpoint in radians; validated joint angles may not
    /// exceed this value when the constrained child body is evaluated.
    pub max: f32,
}

impl AngleRange {
    /// Returns whether both endpoints lock the axis at zero radians.
    ///
    /// This predicate checks exact zero bounds; it does not treat a narrow
    /// interval around zero as a locked joint axis.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::AngleRange;
    ///
    /// let locked = AngleRange { min: 0.0, max: 0.0 };
    /// assert!(locked.is_locked());
    /// ```
    pub const fn is_locked(self) -> bool {
        self.min == 0.0 && self.max == 0.0
    }

    /// Returns whether `angle` lies within both inclusive endpoints in radians.
    ///
    /// This method compares the supplied value directly and leaves finite-value
    /// and profile-bound validation to [`super::RagdollProfile::new`].
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::AngleRange;
    ///
    /// let range = AngleRange { min: -0.5, max: 0.5 };
    /// assert!(range.is_angle_within_range(0.25));
    /// ```
    pub fn is_angle_within_range(self, angle: f32) -> bool {
        self.min <= angle && angle <= self.max
    }
}

/// Joint limits about the child's rest-frame X, Y twist, and Z axes.
///
/// Each range is measured in radians, includes zero, and stays within one
/// half-turn after the containing profile validates its authoring data.
#[derive(Clone, Copy, Debug, PartialEq, bevy::prelude::Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "serialize", serde(deny_unknown_fields))]
pub struct JointLimits {
    /// Inclusive X-axis bend range in radians; profile validation requires
    /// finite endpoints inside the half-turn interval and inclusion of zero
    /// for the constrained child joint.
    pub x: AngleRange,
    /// Inclusive Y-axis twist range in radians; profile validation requires
    /// finite endpoints inside the half-turn interval and inclusion of zero
    /// for the constrained child joint.
    pub twist: AngleRange,
    /// Inclusive Z-axis bend range in radians; profile validation requires
    /// finite endpoints inside the half-turn interval and inclusion of zero
    /// for the constrained child joint.
    pub z: AngleRange,
}

impl JointLimits {
    /// Returns whether the Y twist and Z bend intervals are locked at zero.
    ///
    /// A true result describes an X-axis hinge for the limits as authored; the
    /// profile still validates every interval before storing the joint.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::{AngleRange, JointLimits};
    ///
    /// let locked = AngleRange { min: 0.0, max: 0.0 };
    /// let limits = JointLimits { x: AngleRange { min: -1.0, max: 1.0 }, twist: locked, z: locked };
    /// assert!(limits.is_hinge());
    /// ```
    pub const fn is_hinge(self) -> bool {
        self.twist.is_locked() && self.z.is_locked()
    }
}
