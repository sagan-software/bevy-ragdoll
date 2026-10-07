//! Backend feature declarations and physics-query results for the shared
//! runtime.
//!
//! A backend inserts [`BackendCapabilities`] before the fixed drive stage so
//! the core can choose native motors or its stable-PD fallback. Backend query
//! systems publish [`BodyContacts`] after stepping, while raycast requests use
//! the message types in `messages`. These values describe backend behavior
//! without exposing a concrete physics engine or requiring one plugin family.

use bevy::math::Vec3;
use bevy::prelude::{Entity, Resource};

/// Declares which optional physics features the installed backend implements
/// natively.
///
/// The core reads these predicates once for each driven character and selects
/// fallback torque behavior when a native constraint is unavailable. Backends
/// should replace the whole resource during plugin setup so every capability
/// describes the same active physics implementation.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Resource, bevy::prelude::Reflect)]
pub struct BackendCapabilities {
    /// Whether the backend applies joint motor targets as native physics
    /// constraints each step. The core or query reader consumes this member
    /// without accessing engine-specific storage.
    pub has_native_joint_motors: bool,
    /// Whether native joint constraints enforce independent asymmetric angular
    /// swing ranges. The core or query reader consumes this member without
    /// accessing engine-specific storage.
    pub has_asymmetric_swing_limits: bool,
    /// Whether identical inputs and fixed-step settings produce reproducible
    /// simulation results. The core or query reader consumes this member
    /// without accessing engine-specific storage.
    pub is_deterministic: bool,
    /// Whether this backend implementation builds and runs on supported
    /// WebAssembly targets. The core or query reader consumes this member
    /// without accessing engine-specific storage.
    pub can_run_on_wasm: bool,
}

/// The closest accepted intersection returned by a backend-neutral raycast
/// query.
///
/// `entity` identifies the physics object, while `body` is populated only for a
/// ragdoll body. Position, normal, and distance are expressed in world space
/// and metres, with the normal normalized before publication so callers can
/// apply consistent surface effects.
#[derive(Clone, Copy, Debug, PartialEq, bevy::prelude::Reflect)]
pub struct RayHit {
    /// Physics object entity returned by the query, including static geometry
    /// when hit. The core or query reader consumes this member without
    /// accessing engine-specific storage.
    pub entity: Entity,
    /// Ragdoll body entity when the hit belongs to a body; static objects use
    /// `None` here. The core or query reader consumes this member without
    /// accessing engine-specific storage.
    pub body: Option<Entity>,
    /// World-space intersection point in metres, measured from the physics
    /// world's origin. The core or query reader consumes this member without
    /// accessing engine-specific storage.
    pub point: Vec3,
    /// Unit surface normal in world coordinates, directed away from the hit
    /// surface. The core or query reader consumes this member without accessing
    /// engine-specific storage.
    pub normal: Vec3,
    /// Nonnegative distance from the ray origin to the hit point, measured in
    /// metres. The core or query reader consumes this member without accessing
    /// engine-specific storage.
    pub distance: f32,
}

/// One contact point copied from the backend's most recent completed physics
/// step.
///
/// The point and normal use world coordinates, and `other` identifies the other
/// collider in the pair. Contact normals point away from `other`, allowing core
/// behavior systems to interpret support contacts consistently across engines
/// that order collider pairs differently.
#[derive(Clone, Copy, Debug, PartialEq, bevy::prelude::Reflect)]
pub struct BodyContact {
    /// World-space point of contact in metres, sampled during the latest
    /// backend step. The core or query reader consumes this member without
    /// accessing engine-specific storage.
    pub point: Vec3,
    /// Unit world-space contact normal directed away from the entity stored in
    /// `other`. The core or query reader consumes this member without accessing
    /// engine-specific storage.
    pub normal: Vec3,
    /// Other collider entity in the contact pair; it may be a ragdoll or world
    /// object. The core or query reader consumes this member without accessing
    /// engine-specific storage.
    pub other: Entity,
    /// Whether `other` belongs to static world geometry rather than a moving
    /// physics body. The core or query reader consumes this member without
    /// accessing engine-specific storage.
    pub other_is_static: bool,
}

/// Contact points cached on a body for backend-independent behavior systems.
///
/// The backend replaces this vector after every physics read stage, including
/// an empty vector when no contacts remain. Consumers should treat the order as
/// backend-defined and use each contact's normal and static-object flag instead
/// of inferring support from vector position.
#[derive(bevy::prelude::Component, Clone, Debug, Default, PartialEq, bevy::prelude::Reflect)]
pub struct BodyContacts(
    /// Contacts from the latest read phase, in backend order, with an empty
    /// vector meaning none. The core reads this collection after the backend
    /// replaces contacts for each completed step.
    pub Vec<BodyContact>,
);
