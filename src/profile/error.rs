//! Errors reported while a profile crosses the validation boundary.

use super::BodyIndex;

/// The joint axis or frame that failed profile validation.
///
/// This closed vocabulary lets callers distinguish angular-limit failures from
/// invalid joint transforms without inspecting diagnostic text.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointAxis {
    /// The rigid transform locating the child's joint anchor relative to its
    /// parent body; this category identifies frame failures before physics setup.
    Frame,
    /// The child's X-axis bend interval, measured in radians and checked
    /// against the inclusive profile range during joint validation for each
    /// constrained child body.
    X,
    /// The child's Y-axis twist interval, measured in radians and checked
    /// against the inclusive profile range during joint validation for each
    /// constrained child body.
    Twist,
    /// The child's Z-axis bend interval, measured in radians and checked
    /// against the inclusive profile range during joint validation for each
    /// constrained child body.
    Z,
}

/// A profile validation failure, reported in deterministic validation order.
///
/// The variants separate empty and oversized profiles, tree failures, invalid
/// body data, invalid joint data, and duplicate bone names for caller handling.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum ProfileError {
    /// Returned when the authoring spec contains no bodies and therefore cannot
    /// identify a root body or produce a runtime profile for any backend.
    #[error("a ragdoll profile must contain at least one body")]
    Empty,
    /// Returned when the body count exceeds 64, the maximum supported by the
    /// one-bit-per-body relationship masks in this crate's runtime data model.
    #[error("{0} bodies exceed the maximum of 64")]
    TooManyBodies(usize),
    /// Returned when joints fail to form one parent-first tree rooted at body
    /// zero, including missing, duplicate, self-parented, or backward links.
    #[error("the joints do not form a parent-first tree rooted at body zero")]
    NotATree,
    /// Returned when a body's mass is not finite and positive, or its addition
    /// makes the accumulated total mass non-finite in profile order.
    #[error("body {body:?} has an invalid mass")]
    BadMass {
        /// Checked profile position whose source mass or contribution to the
        /// accumulated kilogram total failed validation during profile creation
        /// before the profile can reach a runtime backend.
        body: BodyIndex,
    },
    /// Returned when a body has an empty bone name, invalid collision
    /// dimensions, or a rest transform that is not a finite rigid isometry.
    #[error("body {body:?} has an invalid name, shape, or rest frame")]
    BadShape {
        /// Checked profile position whose bone name, local collision shape, or
        /// skeleton-space rest frame failed profile validation before body
        /// construction and contact-mask derivation.
        body: BodyIndex,
    },
    /// Returned when a joint frame is non-rigid or one angular range violates
    /// finite endpoint, zero inclusion, or inclusive half-turn constraints.
    #[error("joint {joint:?} has an invalid {axis:?} limit or frame")]
    BadLimit {
        /// Checked child-body position identifying the joint whose frame or
        /// angular data failed validation in the documented validation order
        /// before torque or duplicate-name checks run.
        joint: BodyIndex,
        /// Exact angular-axis or frame category that failed, allowing callers
        /// to select handling without parsing diagnostic text or depending on
        /// an error's human-readable wording.
        axis: JointAxis,
    },
    /// Returned when maximum joint torque is negative or non-finite after
    /// interpreting its numeric value in newton metres; no backend can receive
    /// an invalid torque ceiling from a constructed profile.
    #[error("joint {joint:?} has invalid maximum torque")]
    BadTorque {
        /// Checked child-body position identifying the joint whose torque
        /// ceiling cannot be passed to a physics backend after profile
        /// construction checks all angular and frame data.
        joint: BodyIndex,
    },
    /// Returned when multiple body specifications name the same skeleton bone,
    /// which would make profile-to-skeleton binding ambiguous and prevent a
    /// stable one-body-per-bone runtime mapping.
    #[error("bone {bone:?} is used by more than one body")]
    DuplicateBone {
        /// Repeated source skeleton name shared by multiple body specifications
        /// and rejected before the profile stores its validated bodies or
        /// exposes them to a skeleton binding system.
        bone: String,
    },
}
