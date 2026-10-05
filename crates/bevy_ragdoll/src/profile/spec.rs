//! Serializable profile data that has not passed validation.

use bevy::math::{Isometry3d, Quat, Vec3};

/// The serializable, unvalidated data used to construct a profile.
///
/// Bodies use parent-first order, and joints describe every non-root body's
/// parent relationship. Pass the spec to [`super::RagdollProfile::new`] before
/// runtime use so mass, transform, shape, tree, and limit invariants are checked.
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "serialize", serde(deny_unknown_fields))]
pub struct ProfileSpec {
    /// Bodies in parent-first traversal order; body zero is root, and each
    /// child appears after the body that directly parents it in the tree.
    pub bodies: Vec<BodySpec>,
    /// One joint connects each non-root body to an earlier parent index;
    /// `child` identifies its position in `bodies` without reordering authoring
    /// data for backend use.
    pub joints: Vec<JointSpec>,
}

/// An unvalidated body's bone, collision shape, mass, and rest transform.
///
/// Authoring values retain their source units and coordinate frames until
/// [`super::RagdollProfile::new`] validates them and creates a runtime `Body`.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "serialize", serde(deny_unknown_fields))]
pub struct BodySpec {
    /// Exact skeleton bone name used to bind this body; profile validation
    /// rejects empty or duplicate names before runtime profile construction.
    pub bone: String,
    /// Collision shape in this body's local frame, measured in metres and
    /// transformed by `rest` when geometry enters shared skeleton space at runtime.
    pub shape: ShapeSpec,
    /// Body mass in kilograms; the value must be finite and positive, and
    /// adding it in profile order must leave the total finite.
    pub mass: f32,
    /// Rigid transform from the bone's local rest frame into skeleton space;
    /// translation must be finite and rotation must be finite and unit length.
    pub rest: Isometry3d,
}

/// A collision shape expressed in its body's local frame.
///
/// Geometry is measured in metres, and profile validation rejects non-finite
/// coordinates, non-positive dimensions, and invalid cuboid rotations.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Deserialize, serde::Serialize))]
pub enum ShapeSpec {
    /// A capsule whose centre segment joins cap centres `a` and `b`; radius
    /// extends from that segment to each curved surface and is measured in metres.
    Capsule {
        /// First spherical cap centre in body-local metres; it may equal the
        /// second centre to represent a sphere using the same capsule variant.
        a: Vec3,
        /// Second spherical cap centre in body-local metres; profile validation
        /// requires finite coordinates even when both endpoints are identical
        /// in body coordinates.
        b: Vec3,
        /// Positive finite capsule radius in metres, measured from the centre
        /// segment to its cylindrical surface and spherical cap surfaces during
        /// collision queries.
        radius: f32,
    },
    /// A sphere positioned in body-local coordinates, with a positive radius
    /// measured in metres and centred at the supplied local point.
    Sphere {
        /// Sphere centre in body-local metres; all three coordinates must be
        /// finite before profile construction accepts this collision shape in
        /// the source profile.
        center: Vec3,
        /// Positive finite sphere radius in metres, measured from the centre
        /// to every surface point used by contact exclusion calculations at
        /// runtime.
        radius: f32,
    },
    /// An oriented cuboid with positive half extents measured in metres and
    /// local axes rotated by a finite unit quaternion for body collision use.
    Cuboid {
        /// Cuboid centre in body-local metres; profile validation requires
        /// finite coordinates before the rest transform is applied for every
        /// profile instance.
        center: Vec3,
        /// Finite unit quaternion rotating cuboid-local axes into this body's
        /// local frame before profile rest placement determines collision
        /// orientation for contact checks.
        rotation: Quat,
        /// Positive finite half sizes along the cuboid's local X, Y, and Z
        /// axes, measured in metres from centre to each face.
        half_extents: Vec3,
    },
}

/// An unvalidated parent-child joint description.
///
/// The child index identifies the body being constrained, and the frame is
/// expressed in its parent body. Limits use radians and torque uses newton metres.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "serialize", serde(deny_unknown_fields))]
pub struct JointSpec {
    /// Child body index in parent-first `ProfileSpec::bodies` order; body
    /// zero cannot be a child because the root has no incoming joint constraint.
    pub child: u8,
    /// Parent body index in `ProfileSpec::bodies`; validation requires this
    /// index to precede the child and form one connected tree before runtime
    /// profile creation.
    pub parent: u8,
    /// Child joint frame expressed relative to the parent body's frame;
    /// translation must be finite and rotation must be a unit quaternion.
    pub frame: Isometry3d,
    /// Allowed X bend, Y twist, and Z bend intervals in radians; each finite
    /// interval includes zero and stays within inclusive `-PI..=PI` bounds.
    pub limits: super::JointLimits,
    /// Maximum motor torque in newton metres; the value must be finite and
    /// nonnegative before a backend creates the corresponding constraint.
    pub max_torque: f32,
}
