//! Code-oriented construction of unvalidated profile data.

use bevy::math::Isometry3d;

use super::{
    BodyIndex, BodySpec, JointLimits, JointSpec, MAX_BODIES, ProfileError, ProfileSpec,
    RagdollProfile, ShapeSpec,
};

/// Builds profile data from typed bodies and joints before validation.
///
/// The builder assigns checked profile indexes as bodies are added, retains the
/// authoring data in insertion order, and validates it when [`build`](Self::build)
/// creates a runtime-ready profile.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ProfileBuilder {
    /// The unvalidated profile data accumulated by this builder.
    spec: ProfileSpec,
}

impl ProfileBuilder {
    /// Adds a body and returns its checked profile index.
    ///
    /// The bone name, local collision shape, mass in kilograms, and skeleton
    /// rest transform remain unvalidated until `build` checks the full profile.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy::math::{Isometry3d, Vec3};
    /// use bevy_ragdoll::{ProfileBuilder, ShapeSpec};
    ///
    /// let mut builder = ProfileBuilder::default();
    /// let root = builder.add_body("pelvis", ShapeSpec::Sphere { center: Vec3::ZERO, radius: 0.2 }, 8.0, Isometry3d::IDENTITY)?;
    /// assert_eq!(root.get(), 0);
    /// # Ok::<(), bevy_ragdoll::ProfileError>(())
    /// ```
    pub fn add_body(
        &mut self,
        bone: impl Into<String>,
        shape: ShapeSpec,
        mass: f32,
        rest: Isometry3d,
    ) -> Result<BodyIndex, ProfileError> {
        // Preserve insertion order because body indexes define joint and mask positions.
        let index = self.spec.bodies.len();
        // Reject capacity before converting the source position into a mask index.
        if index >= MAX_BODIES {
            return Err(ProfileError::TooManyBodies(index + 1));
        }
        let body_index = BodyIndex(index as u8);
        // Retain the name and geometry only after the body position is representable.
        self.spec.bodies.push(BodySpec {
            bone: bone.into(),
            shape,
            mass,
            rest,
            role: None,
        });
        Ok(body_index)
    }

    /// Adds an unvalidated joint from `parent` to `child` and returns the builder.
    ///
    /// The supplied frame is relative to the parent body, limits use radians,
    /// and torque uses newton metres; `build` checks all joint constraints.
    ///
    /// # Examples
    ///
    /// ```
    /// # use bevy_ragdoll::{AngleRange, JointLimits, ProfileBuilder};
    /// # fn connect(builder: &mut ProfileBuilder, child: bevy_ragdoll::BodyIndex, parent: bevy_ragdoll::BodyIndex) {
    /// let locked = AngleRange { min: 0.0, max: 0.0 };
    /// let limits = JointLimits {
    ///     x: AngleRange { min: -1.0, max: 1.0 },
    ///     twist: locked,
    ///     z: locked,
    /// };
    /// builder.add_joint(child, parent, bevy::math::Isometry3d::IDENTITY, limits, 12.0);
    /// # }
    /// ```
    pub fn add_joint(
        &mut self,
        child: BodyIndex,
        parent: BodyIndex,
        frame: Isometry3d,
        limits: JointLimits,
        max_torque: f32,
    ) -> &mut Self {
        self.spec.joints.push(JointSpec {
            child: child.0,
            parent: parent.0,
            frame,
            limits,
            max_torque,
        });
        self
    }

    /// Borrows the unvalidated data accumulated so far in insertion order.
    ///
    /// The returned spec can be inspected or serialized before `build` applies
    /// profile validation and derives runtime relationships.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::ProfileBuilder;
    ///
    /// let builder = ProfileBuilder::default();
    /// assert!(builder.spec().bodies.is_empty());
    /// ```
    pub const fn spec(&self) -> &ProfileSpec {
        &self.spec
    }

    /// Returns the unvalidated data accumulated so far by consuming the builder.
    ///
    /// Use this conversion when another authoring step needs ownership of the
    /// spec without applying profile validation or deriving runtime masks.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::ProfileBuilder;
    ///
    /// let spec = ProfileBuilder::default().into_spec();
    /// assert!(spec.bodies.is_empty());
    /// ```
    pub fn into_spec(self) -> ProfileSpec {
        self.spec
    }

    /// Validates the accumulated data and returns a runtime-ready profile.
    ///
    /// This consumes the builder so the checked bodies and joints move into the
    /// profile without cloning authoring names or collision shapes.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy::math::{Isometry3d, Vec3};
    /// use bevy_ragdoll::{ProfileBuilder, ShapeSpec};
    ///
    /// let mut builder = ProfileBuilder::default();
    /// builder.add_body("pelvis", ShapeSpec::Sphere { center: Vec3::ZERO, radius: 0.2 }, 8.0, Isometry3d::IDENTITY)?;
    /// assert_eq!(builder.build()?.bodies().len(), 1);
    /// # Ok::<(), bevy_ragdoll::ProfileError>(())
    /// ```
    pub fn build(self) -> Result<RagdollProfile, ProfileError> {
        RagdollProfile::new(self.spec)
    }
}
