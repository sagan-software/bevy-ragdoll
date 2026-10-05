//! Reflected Skein authoring components stored in glTF node extras.
//!
//! These types represent imported and editor-authored component data in
//! degrees and kilograms. Profile construction converts angle ranges to
//! radians and validates body and joint constraints before runtime use.
//! GLB import accepts both this crate's reflected type paths and the legacy
//! TGF paths, while newly authored files should use `bevy_ragdoll::skein`.

use bevy::ecs::reflect::ReflectComponent;
use bevy::prelude::{Component, Reflect};
use bevy::reflect::std_traits::ReflectDefault;

/// The largest accepted absolute Skein joint limit, measured in degrees.
///
/// Profile import accepts endpoint values from negative one hundred eighty
/// through positive one hundred eighty degrees, inclusive.
pub const MAX_ANGLE_DEG: f32 = 180.0;

/// A ragdoll body annotation attached to a capsule mesh node.
///
/// Skein serializes this component in the node's `extras` member. Its mass is
/// expressed in kilograms and must be finite and greater than zero.
#[derive(Component, Reflect, Clone, Copy, Debug, Default, PartialEq)]
#[reflect(Component, Default)]
#[cfg_attr(feature = "serialize", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "serialize", serde(deny_unknown_fields))]
pub struct RagdollBody {
    /// Body mass in kilograms; import rejects non-finite or non-positive
    /// values before profile construction stores this annotation as a body
    /// component.
    pub mass_kg: f32,
}

impl RagdollBody {
    /// Explains why this annotation's mass is invalid, or returns `None` when
    /// the value is finite and positive for profile construction.
    ///
    /// The message names the rejected field so import diagnostics can identify
    /// the source component without replacing the machine-facing profile error.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::skein::RagdollBody;
    ///
    /// let body = RagdollBody { mass_kg: 4.0 };
    /// assert!(body.problem().is_none());
    /// ```
    pub fn problem(&self) -> Option<String> {
        let mass = self.mass_kg;
        (!(mass.is_finite() && mass > 0.0))
            .then(|| format!("mass_kg {mass} is not a positive number"))
    }
}

/// A joint angle interval in degrees for Skein authoring data.
///
/// Both endpoints must be finite, ordered, and within the inclusive
/// `-MAX_ANGLE_DEG..=MAX_ANGLE_DEG` profile before import converts them to radians.
#[derive(Reflect, Clone, Copy, Debug, Default, PartialEq)]
#[reflect(Default)]
#[cfg_attr(feature = "serialize", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "serialize", serde(deny_unknown_fields))]
pub struct AngleRange {
    /// Inclusive lower angle in degrees; values must be finite and cannot fall
    /// below negative 180 before conversion to profile radians.
    pub min_deg: f32,
    /// Inclusive upper angle in degrees; values must be finite and cannot rise
    /// above positive 180 before conversion to profile radians.
    pub max_deg: f32,
}

impl AngleRange {
    /// Returns whether both authoring endpoints lock the axis at zero degrees.
    ///
    /// The predicate checks exact zero endpoints; a small nonzero interval does
    /// not count as locked when Skein import selects hinge behavior.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::skein::AngleRange;
    ///
    /// let locked = AngleRange { min_deg: 0.0, max_deg: 0.0 };
    /// assert!(locked.is_locked());
    /// ```
    pub const fn is_locked(self) -> bool {
        self.min_deg == 0.0 && self.max_deg == 0.0
    }

    /// Explains why this range is invalid, or returns `None` for valid degree endpoints.
    ///
    /// Validation checks finite numbers, ascending endpoint order, then the
    /// configured absolute degree limit so diagnostics remain deterministic.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::skein::AngleRange;
    ///
    /// let range = AngleRange { min_deg: -45.0, max_deg: 90.0 };
    /// assert!(range.problem("limit_x").is_none());
    /// ```
    pub fn problem(self, axis: &str) -> Option<String> {
        let (min, max) = (self.min_deg, self.max_deg);
        // Reject non-finite values before comparing endpoint order.
        if !(min.is_finite() && max.is_finite()) {
            return Some(format!("{axis} is not finite"));
        }
        // Report reversed ranges before checking the absolute supported bounds.
        if min > max {
            return Some(format!("{axis} runs backwards ({min} > {max})"));
        }
        // Keep authored limits within the profile's closed degree vocabulary.
        if min < -MAX_ANGLE_DEG || max > MAX_ANGLE_DEG {
            return Some(format!("{axis} leaves -180..180 degrees"));
        }
        None
    }
}

/// A joint annotation attached to a non-root ragdoll body.
///
/// The three angular ranges use Skein degrees, and the torque ceiling uses
/// newton metres. Profile import checks these values before creating a joint.
#[derive(Component, Reflect, Clone, Copy, Debug, Default, PartialEq)]
#[reflect(Component, Default)]
#[cfg_attr(feature = "serialize", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "serialize", serde(deny_unknown_fields))]
pub struct RagdollJoint {
    /// Child bone X-axis bend range in inclusive degrees; finite ordered
    /// endpoints must include zero and stay within negative to positive 180.
    pub limit_x: AngleRange,
    /// Child bone Y-axis twist range in inclusive degrees; finite ordered
    /// endpoints must include zero and stay within negative to positive 180.
    pub limit_y: AngleRange,
    /// Child bone Z-axis bend range in inclusive degrees; finite ordered
    /// endpoints must include zero and stay within negative to positive 180.
    pub limit_z: AngleRange,
    /// Maximum motor torque in newton metres; it must be finite and
    /// nonnegative, while zero disables motor force during physics control.
    pub torque_nm: f32,
}

impl RagdollJoint {
    /// Returns whether the Y twist and Z bend axes are locked at zero degrees.
    ///
    /// A true result describes an X-axis hinge for the annotation as authored;
    /// profile import still validates all three degree ranges and torque.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::skein::{AngleRange, RagdollJoint};
    ///
    /// let locked = AngleRange { min_deg: 0.0, max_deg: 0.0 };
    /// let joint = RagdollJoint { limit_x: AngleRange { min_deg: -90.0, max_deg: 90.0 }, limit_y: locked, limit_z: locked, torque_nm: 12.0 };
    /// assert!(joint.is_hinge());
    /// ```
    pub const fn is_hinge(&self) -> bool {
        self.limit_y.is_locked() && self.limit_z.is_locked()
    }

    /// Explains the first invalid range or torque, or returns `None` when values are valid.
    ///
    /// The method checks X, Y, and Z ranges in that order, then checks the
    /// torque ceiling so a source component reports one stable diagnostic.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::skein::{AngleRange, RagdollJoint};
    ///
    /// let locked = AngleRange { min_deg: 0.0, max_deg: 0.0 };
    /// let joint = RagdollJoint { limit_x: locked, limit_y: locked, limit_z: locked, torque_nm: 0.0 };
    /// assert!(joint.problem().is_none());
    /// ```
    pub fn problem(&self) -> Option<String> {
        let torque = self.torque_nm;
        // Return the first axis error before evaluating torque.
        self.limit_x
            .problem("limit_x")
            .or_else(|| self.limit_y.problem("limit_y"))
            .or_else(|| self.limit_z.problem("limit_z"))
            .or_else(|| {
                // Torque is valid only when finite and nonnegative.
                (!(torque.is_finite() && torque >= 0.0))
                    .then(|| format!("torque_nm {torque} is not a number of at least 0"))
            })
    }
}

#[cfg(test)]
mod tests;
