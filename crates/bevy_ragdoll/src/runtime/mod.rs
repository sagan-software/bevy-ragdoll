//! Bevy runtime state and contracts for profile-driven ragdolls.
//!
//! The `components` module stores character state, and `body` defines the
//! physics-neutral body contract. Backends read and write those body components
//! around their fixed physics step. The `sets`, `messages`, and `events`
//! modules expose scheduling and communication points for game systems. The
//! `drive` module contains shared control calculations, while `backend`
//! describes supported physics features and query results. Add `RagdollPlugin`
//! before attaching runtime components or loading profile-driven skeletons.

pub mod backend;
pub mod body;
pub mod budget;
mod capture;
pub mod components;
pub mod drive;
pub mod events;
pub mod messages;
mod plugin;
pub mod sets;
pub mod settings;
mod skeleton;
pub mod writeback;

pub use self::plugin::{RagdollFixedSchedule, RagdollPlugin};
pub use self::skeleton::RagdollError;
