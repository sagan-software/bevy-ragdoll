//! Messages exchanged between game systems, the ragdoll runtime, and physics
//! backends.
//!
//! Applications write impulses and hits to request body actions, while backends
//! read raycast requests and publish matching responses. Request identities are
//! copied unchanged so callers can correlate asynchronous results without
//! coupling their own sequence policy to the runtime. Every position,
//! direction, distance, and impulse uses world coordinates and SI units.

use bevy::math::Vec3;
use bevy::prelude::{Entity, Message};

/// A caller-owned identity that correlates one raycast request with its
/// response.
///
/// The wrapper keeps request identifiers distinct from counts, indexes, and
/// physics values while preserving every `u64` bit supplied by the caller. The
/// runtime copies the identity unchanged.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, bevy::prelude::Reflect)]
pub struct RagdollRequestId(u64);

impl RagdollRequestId {
    /// Creates a request identity from an application-selected monotonically
    /// managed value.
    ///
    /// The identifier has no runtime allocation or global uniqueness guarantee;
    /// callers choose a scope that prevents collisions while requests remain
    /// outstanding in their message reader.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::messages::RagdollRequestId;
    ///
    /// let request_id = RagdollRequestId::new(17); assert_eq!(request_id.get(),
    /// 17);
    /// ```
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Returns the exact caller-supplied integer without changing its bits.
    ///
    /// This conversion is useful when persisting or logging an
    /// application-owned request sequence; the returned value has no meaning
    /// outside that caller's correlation policy.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::messages::RagdollRequestId;
    ///
    /// assert_eq!(RagdollRequestId::new(17).get(), 17);
    /// ```
    pub const fn get(self) -> u64 {
        self.0
    }
}

/// Requests an instantaneous point impulse on one ragdoll body during backend
/// processing.
///
/// The backend converts the impulse into the units and operation required by
/// its physics engine, while preserving the world-space application point.
/// Writing this message does not guarantee a body still exists when the backend
/// consumes it, so stale entity requests are ignored.
#[derive(Clone, Copy, Debug, Message, PartialEq, bevy::prelude::Reflect)]
pub struct RagdollImpulse {
    /// Existing ragdoll body entity that should receive the requested impulse
    /// when consumed. The message reader uses this member during the requested
    /// operation and preserves it when publishing a result.
    pub body: Entity,
    /// World-space point of application in metres, used to derive linear and
    /// angular momentum. The message reader uses this member during the
    /// requested operation and preserves it when publishing a result.
    pub point: Vec3,
    /// World-space impulse in newton seconds, applied once rather than as a
    /// sustained force. The message reader uses this member during the
    /// requested operation and preserves it when publishing a result.
    pub impulse: Vec3,
}

/// Closed source classification used by gameplay hit policies and telemetry
/// consumers.
///
/// The category distinguishes physical impacts from application-authored events
/// while keeping damage interpretation outside the core runtime. A backend may
/// use either value for the same impulse integration path.
#[derive(Clone, Copy, Debug, PartialEq, Eq, bevy::prelude::Reflect)]
pub enum HitKind {
    /// A collision or weapon impact originating from a physical projectile or
    /// other moving object. Reaction systems may combine this category with
    /// contact data when selecting local muscle response.
    Impact,
    /// An application-authored or environmental hit that did not originate from
    /// physical collision. Applications may use this category for scripted hits
    /// while sharing the same reaction path.
    Environmental,
}

/// Describes a hit and its impulse for gameplay layers that coordinate active
/// ragdoll control.
///
/// The core message preserves the source category and world-space contact data
/// but does not apply damage, choose a reaction, or activate muscles. Consumers
/// can forward the impulse separately to the backend after their own policy has
/// selected a response.
#[derive(Clone, Copy, Debug, Message, PartialEq, bevy::prelude::Reflect)]
pub struct RagdollHit {
    /// Ragdoll body entity at the impact site, used by a reaction-selection
    /// policy. The message reader uses this member during the requested
    /// operation and preserves it when publishing a result.
    pub body: Entity,
    /// World-space contact point in metres, supplied to the selected reaction
    /// behavior. The message reader uses this member during the requested
    /// operation and preserves it when publishing a result.
    pub point: Vec3,
    /// World-space impulse in newton seconds associated with the impact event.
    /// The message reader uses this member during the requested operation and
    /// preserves it when publishing a result.
    pub impulse: Vec3,
    /// Source category used by application policy to distinguish impacts from
    /// scripted events. The message reader uses this member during the
    /// requested operation and preserves it when publishing a result.
    pub kind: HitKind,
}

/// Requests a backend-neutral raycast and carries caller identity through the
/// response message.
///
/// The backend normalizes valid directions, rejects non-finite or nonpositive
/// distances, and returns the closest accepted hit. `filter` excludes one
/// entity and its body representation from the query; no response is inferred
/// from request order, so callers match by `request_id`.
#[derive(Clone, Copy, Debug, Message, PartialEq, bevy::prelude::Reflect)]
pub struct RagdollRaycast {
    /// Caller-chosen identity copied unchanged into exactly the response
    /// generated for this request. The message reader uses this member during
    /// the requested operation and preserves it when publishing a result.
    pub request_id: RagdollRequestId,
    /// World-space ray origin in metres, measured from the physics world's
    /// origin. The message reader uses this member during the requested
    /// operation and preserves it when publishing a result.
    pub origin: Vec3,
    /// Finite nonzero world-space direction that the backend normalizes before
    /// intersection tests. The message reader uses this member during the
    /// requested operation and preserves it when publishing a result.
    pub direction: Vec3,
    /// Positive finite maximum travel distance in metres before the ray query
    /// reports a miss. The message reader uses this member during the requested
    /// operation and preserves it when publishing a result.
    pub max_distance: f32,
    /// Optional character entity excluded along with its ragdoll bodies when
    /// supplied by the caller. The message reader uses this member during the
    /// requested operation and preserves it when publishing a result.
    pub filter: Option<Entity>,
}

/// Reports the closest accepted hit, or an explicit miss, for one raycast
/// request identity.
///
/// A backend emits one response for each consumed request, including requests
/// with invalid direction or distance. The response copies `request_id` exactly
/// so concurrent callers can safely correlate completion without relying on
/// message-reader order.
#[derive(Clone, Copy, Debug, Message, PartialEq, bevy::prelude::Reflect)]
pub struct RagdollRaycastResponse {
    /// Exact request identity copied from the request that produced this
    /// response message. The message reader uses this member during the
    /// requested operation and preserves it when publishing a result.
    pub request_id: RagdollRequestId,
    /// Closest accepted hit in world coordinates, or `None` for invalid input
    /// and ordinary misses. The message reader uses this member during the
    /// requested operation and preserves it when publishing a result.
    pub hit: Option<super::backend::RayHit>,
}
