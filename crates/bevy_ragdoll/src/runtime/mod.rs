//! Bevy runtime state and contracts for profile-driven ragdolls.
//!
//! The `components` module stores character state, and `body` defines the
//! physics-neutral body contract. Backends read and write body components
//! around each fixed physics step, using profile indexes to preserve skeleton
//! ordering and shared units for forces, poses, and velocities.
//! Backends never own profile authoring assets, which lets applications share
//! checked profiles across independent character instances.
//!
//! The `sets`, `messages`, and `events` modules expose scheduling and
//! communication points for game systems. `drive` contains shared control
//! calculations, while `backend` describes supported physics features and
//! query results. Add `RagdollPlugin` before attaching runtime components or
//! loading profile-driven skeletons, then install a backend plugin separately.
//! The `hit` and `pin` modules provide active-control messages, masks, and
//! recovery settings without requiring backend-specific component access.

pub mod backend;
pub mod body;
pub mod budget;
mod capture;
pub mod components;
pub mod drive;
pub mod events;
pub mod hit;
pub mod messages;
pub mod pin;
mod plugin;
pub mod sets;
pub mod settings;
mod skeleton;
pub mod writeback;

pub use self::plugin::{RagdollFixedSchedule, RagdollPlugin};
pub use self::skeleton::RagdollError;
