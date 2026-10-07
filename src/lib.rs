//! Profile-driven ragdolls for Bevy skeletons and physics backends.
//!
//! This crate validates reusable body and joint profiles, reflects Skein
//! annotations, and provides an engine-independent runtime. The [`runtime`]
//! module contains ECS state, schedules, events, and backend contracts. The
//! [`profile`] module contains validated authoring data, while [`skein`]
//! documents the reflected annotation schema. Enable `serialize` for RON assets
//! and `gltf` for GLB rig import. Add [`RagdollPlugin`] before using profile
//! assets or runtime systems. Physics backends and active-control layers use
//! the same profile and runtime contracts.
//!
//! ```ignore
//! use bevy::prelude::*; use bevy_ragdoll::RagdollPlugin;
//!
//! App::new() .add_plugins((MinimalPlugins, RagdollPlugin::default())) .run();
//! ```

#[cfg(feature = "gltf")]
pub mod gltf;
pub mod profile;
pub mod runtime;
pub mod skein;

pub use self::profile::{
    AngleRange, Body, BodyIndex, BodyRole, BodySpec, Joint, JointAxis, JointLimits, JointSpec,
    MAX_BODIES, Mass, MassError, ProfileBuilder, ProfileError, ProfileSpec, RagdollProfile,
    ShapeSpec,
};
#[cfg(feature = "serialize")]
pub use self::profile::{RagdollProfileLoader, RagdollProfileLoaderError};
pub use self::runtime::{RagdollError, RagdollPlugin};
