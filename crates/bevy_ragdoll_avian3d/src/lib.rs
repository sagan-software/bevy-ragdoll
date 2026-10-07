//! Avian 3D physics adapter for the backend-neutral ragdoll runtime, mapping
//! runtime body, joint, contact, raycast, and fixed-step state into a
//! user-owned Avian simulation.
//!
//! Applications add `RagdollPlugin`, then Avian's `PhysicsPlugins` in
//! `FixedUpdate` with [`AvianRagdollHooks`] as collision hooks, then
//! [`AvianRagdollPlugin`]. Avian must run in the same schedule as
//! `RagdollPlugin`'s fixed schedule.
//!
//! # Joint mapping
//!
//! - All three ranges locked: `FixedJoint`.
//! - Twist and Z locked: `RevoluteJoint` about the frame X axis with the exact
//!   X range.
//! - Otherwise: `SphericalJoint` with exact twist limits about the frame Y
//!   axis and a symmetric swing cone at the largest X or Z extent. The core's
//!   soft-limit torque enforces the asymmetric X and Z ranges inside the cone.
//!
//! Avian 0.7 measures the spherical swing cone around
//! `twist_axis.any_orthonormal_vector()` rather than around `twist_axis`, and
//! its twist limit bounds rotation about that same vector. The adapter sets
//! `twist_axis = X`, whose orthonormal vector is `+Y`, so the profile's Y twist
//! is Avian's twist limit and the cone surrounds profile Y. The
//! `swing_limit_axis_is_as_documented_in_this_crate` test checks this.
//!
//! # Capabilities
//!
//! See [`AVIAN_CAPABILITIES`]: no native joint motors, no asymmetric swing
//! limits, not yet measured as deterministic, runs on WebAssembly.

mod body;
mod contact;
mod joint;
mod plugin;
mod query;
mod settings;
mod shape;
mod spawn;

pub use self::contact::{AvianRagdollHooks, RagdollPairQuery, should_ragdoll_pair_collide};
pub use self::plugin::{AVIAN_CAPABILITIES, AvianRagdollPlugin};
pub use self::settings::AvianRagdollSettings;
pub use self::shape::collider_for_shape;
