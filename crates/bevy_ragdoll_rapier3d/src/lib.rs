//! Rapier 3D physics adapter for the backend-neutral ragdoll runtime, mapping
//! runtime body, joint, contact, raycast, and fixed-step state into a user-owned
//! Rapier simulation.
//!
//! Applications add `RagdollPlugin`, configure their own fixed-schedule
//! `RapierPhysicsPlugin` with [`RapierRagdollHooks`], then add
//! [`RapierRagdollPlugin`]. The adapter leaves Rapier world ownership with the
//! application and reports backend-neutral contacts and query responses.

mod body;
mod contact;
mod joint;
mod plugin;
mod query;
mod settings;
mod shape;
mod spawn;

pub use self::contact::{RapierRagdollHooks, ragdoll_filter_contact_pair};
pub use self::plugin::RapierRagdollPlugin;
pub use self::settings::RapierRagdollSettings;
