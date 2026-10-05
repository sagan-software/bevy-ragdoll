//! A validated body entry in a ragdoll profile.

use bevy::math::Isometry3d;

use super::{BodyIndex, Mass, ShapeSpec};

/// A profile body whose mass, shape, rest frame, and index passed validation.
///
/// A `Body` stores the checked skeleton binding and collision geometry used by
/// runtimes. Its fields remain private so invalid mass and relationship states
/// cannot be created after profile construction.
#[derive(Clone, Debug, PartialEq)]
pub struct Body {
    /// The body's index in profile order.
    index: BodyIndex,
    /// The skeleton bone that this body follows.
    bone: String,
    /// The collision shape in the body frame.
    shape: ShapeSpec,
    /// The validated mass in kilograms.
    mass: Mass,
    /// The bone's rest transform in skeleton space.
    rest: Isometry3d,
}

impl Body {
    /// Creates a body after its source spec has passed validation.
    pub(super) fn new(
        index: BodyIndex,
        bone: String,
        shape: ShapeSpec,
        mass: Mass,
        rest: Isometry3d,
    ) -> Self {
        Self {
            index,
            bone,
            shape,
            mass,
            rest,
        }
    }

    /// Returns this body's checked profile index in parent-first profile order.
    ///
    /// The index is bounded by [`super::MAX_BODIES`] and can address the
    /// profile's relationship masks without converting from unchecked input.
    ///
    /// # Examples
    ///
    /// ```
    /// # use bevy_ragdoll::Body;
    /// # fn index(body: &Body) -> usize {
    /// body.index().get()
    /// # }
    /// ```
    pub const fn index(&self) -> BodyIndex {
        self.index
    }

    /// Returns the exact skeleton bone name used to bind this body.
    ///
    /// Profile validation rejects empty and duplicate names, so this borrowed
    /// value identifies one profile body without allocating a new string.
    ///
    /// # Examples
    ///
    /// ```
    /// # use bevy_ragdoll::Body;
    /// # fn bone(body: &Body) -> &str {
    /// body.bone()
    /// # }
    /// ```
    pub fn bone(&self) -> &str {
        &self.bone
    }

    /// Returns the validated collision shape in this body's local frame.
    ///
    /// Shape coordinates and dimensions use metres; the returned reference
    /// borrows the shape stored in this body without cloning geometry.
    ///
    /// # Examples
    ///
    /// ```
    /// # use bevy_ragdoll::{Body, ShapeSpec};
    /// # fn shape(body: &Body) -> &ShapeSpec {
    /// body.shape()
    /// # }
    /// ```
    pub const fn shape(&self) -> &ShapeSpec {
        &self.shape
    }

    /// Returns this body's finite positive mass in kilograms.
    ///
    /// Profile construction checked this value before storing it, and the
    /// accessor preserves the kilogram unit without converting or allocating.
    ///
    /// # Examples
    ///
    /// ```
    /// # use bevy_ragdoll::Body;
    /// # fn mass(body: &Body) -> f32 {
    /// body.mass().kilograms()
    /// # }
    /// ```
    pub const fn mass(&self) -> Mass {
        self.mass
    }

    /// Returns the bone's validated rigid rest transform in skeleton space.
    ///
    /// The transform maps body-local points into the shared skeleton frame and
    /// keeps finite translation with a finite unit rotation.
    ///
    /// # Examples
    ///
    /// ```
    /// # use bevy::math::Isometry3d;
    /// # use bevy_ragdoll::Body;
    /// # fn rest(body: &Body) -> Isometry3d {
    /// body.rest()
    /// # }
    /// ```
    pub const fn rest(&self) -> Isometry3d {
        self.rest
    }
}
