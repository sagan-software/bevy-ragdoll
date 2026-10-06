//! Runnable Bevy applications for the public ragdoll runtime and physics backends.
//!
//! The `rapier` module contains the Rapier 3D setup shared by visible and
//! headless examples. The `mock` module demonstrates a custom backend without
//! a physics engine. Each example binary owns its command-line interface and
//! delegates profile loading, scene construction, and simulation controls here.

#[cfg(feature = "custom-backend")]
mod mock;
#[cfg(feature = "rapier3d")]
mod rapier;
#[cfg(feature = "rapier3d")]
pub mod stress;

#[cfg(feature = "custom-backend")]
pub use self::mock::run_custom_backend;
#[cfg(feature = "rapier3d")]
pub use self::rapier::{ExampleKind, load_profile, run_example};
