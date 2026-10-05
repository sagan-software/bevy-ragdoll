//! Profile-driven ragdolls for Bevy skeletons and physics backends.
//!
//! This crate defines serializable profile data, validates body and joint
//! invariants, derives contact and child masks, and exposes Skein authoring
//! components. Enable `serialize` for RON assets and `gltf` for GLB rig import.
//! The `skein` module documents the reflected annotation schema, while the
//! profile module's root re-exports cover validated data and authoring builders.
//! Add [`RagdollPlugin`] to initialize the profile asset type and reflected
//! Skein components. Physics backend integrations and active control build on
//! the validated [`RagdollProfile`] data model.
//!
//! ```ignore
//! use bevy::prelude::*;
//! use bevy_ragdoll::RagdollPlugin;
//!
//! App::new()
//!     .add_plugins((MinimalPlugins, RagdollPlugin))
//!     .run();
//! ```

use bevy::asset::AssetApp;
use bevy::prelude::{App, Plugin};
use bevy_transform as _;

#[cfg(feature = "gltf")]
pub mod gltf;
mod profile;
pub mod skein;

pub use self::profile::{
    AngleRange, Body, BodyIndex, BodySpec, Joint, JointAxis, JointLimits, JointSpec, MAX_BODIES,
    Mass, MassError, ProfileBuilder, ProfileError, ProfileSpec, RagdollProfile, ShapeSpec,
};
#[cfg(feature = "serialize")]
pub use self::profile::{RagdollProfileLoader, RagdollProfileLoaderError};

/// Registers profile assets and reflected Skein authoring components with Bevy.
///
/// Add the plugin once before loading `.ragdoll.ron` assets or reflecting body
/// and joint annotations. It initializes the loader only when `serialize` is
/// enabled, and it does not start a physics backend by itself.
#[derive(Clone, Copy, Debug)]
pub struct RagdollPlugin;

impl Plugin for RagdollPlugin {
    fn build(&self, app: &mut App) {
        app.init_asset::<RagdollProfile>();
        #[cfg(feature = "serialize")]
        app.init_asset_loader::<RagdollProfileLoader>();
        app.register_type::<skein::RagdollBody>()
            .register_type::<skein::AngleRange>()
            .register_type::<skein::RagdollJoint>();
    }
}
