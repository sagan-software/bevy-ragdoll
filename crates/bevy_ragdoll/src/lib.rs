//! Active ragdolls for Bevy skeletons and physics backends.
//!
//! This crate is the public home for the Bevy ragdoll plugin, configuration, and
//! runtime components. Later phases add profile loading, skeleton binding, and
//! backend behavior. Phase 1 registers no systems and performs no physics work.
//! Add `RagdollPlugin` to an `App` as the integration entry point.
//!
//! ```ignore
//! use bevy::prelude::*;
//! use bevy_ragdoll::RagdollPlugin;
//!
//! App::new()
//!     .add_plugins((MinimalPlugins, RagdollPlugin))
//!     .run();
//! ```

use bevy::prelude::{App, Plugin};
use bevy_transform as _;

/// Registers the phase 1 ragdoll integration point with a Bevy app.
///
/// Later phases add profile loading, skeleton binding, and backend systems to
/// this plugin.
#[derive(Clone, Copy, Debug)]
pub struct RagdollPlugin;

impl Plugin for RagdollPlugin {
    fn build(&self, _app: &mut App) {}
}
