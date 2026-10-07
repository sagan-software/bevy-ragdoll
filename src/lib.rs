//! Profile-driven ragdolls for Bevy skeletons and physics backends.
//!
//! Insert [`Ragdoll::default()`](runtime::components::Ragdoll) on a character
//! whose descendants form a skeleton, such as a glTF scene root. The runtime
//! generates a profile from the bones with no authored files (see [`auto`]).
//! The [`runtime`] module contains ECS state, schedules, events, and backend
//! contracts. The [`profile`] module contains validated profile data. Enable
//! `serialize` for `.ragdoll.ron` override files.
//!
//! ```ignore
//! use bevy::prelude::*;
//! use bevy_ragdoll::{Ragdoll, RagdollPlugin};
//!
//! fn spawn(mut commands: Commands, assets: Res<AssetServer>) {
//!     commands.spawn((
//!         WorldAssetRoot(assets.load(GltfAssetLabel::Scene(0).from_asset("rigs/alien.glb"))),
//!         Ragdoll::default(),
//!     ));
//! }
//! ```

#![expect(
    clippy::suboptimal_flops,
    reason = "mul_add calls software fmaf on x86-64 without FMA and changes physics results"
)]
#![cfg_attr(
    dylint_lib = "sagan_lints",
    expect(
        public_surface_size,
        reason = "the root re-exports the profile vocabulary that examples, backends and tests import; moving it is a breaking change"
    )
)]

pub mod auto;
#[cfg(feature = "debug")]
mod debug;
pub mod profile;
pub mod runtime;

pub use self::auto::{BoneBody, RagdollBone, RagdollOverrides, Skeleton, SkeletonBone};
#[cfg(feature = "debug")]
pub use self::debug::RagdollDebugPlugin;
pub use self::profile::{
    AngleRange, Body, BodyIndex, BodyRole, BodySpec, Joint, JointAxis, JointLimits, JointSpec,
    MAX_BODIES, Mass, MassError, ProfileBuilder, ProfileError, ProfileSpec, RagdollProfile,
    ShapeSpec,
};
pub use self::runtime::components::Ragdoll;
pub use self::runtime::{RagdollError, RagdollPlugin};
