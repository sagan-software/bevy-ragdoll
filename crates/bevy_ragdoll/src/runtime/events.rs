//! Lifecycle events for skeleton binding, settling, freezing, and budget
//! eviction.
//!
//! Observers use these events to start or stop gameplay effects without
//! querying runtime internals. Each event targets the character entity whose
//! ragdoll changed; related body entities remain discoverable through
//! [`crate::runtime::components::RagdollBodies`] while that ragdoll is active.

use bevy::prelude::{Entity, EntityEvent};

/// Fires after a skeleton binds successfully and the runtime creates its
/// physics body entities.
///
/// The event occurs once for an activation transition, after body components
/// and their relationship to the character exist. Observers should attach
/// gameplay state to the character rather than relying on the order in which
/// body entities were created.
#[derive(Clone, Copy, Debug, EntityEvent, bevy::prelude::Reflect)]
pub struct RagdollActivated {
    /// Character entity whose validated profile bound and whose physics bodies
    /// were created. Observers inspect this target after the state transition
    /// and can read current mode and body relationships.
    pub entity: Entity,
}

/// Fires after every body remains below the configured settle-speed threshold
/// for the full interval.
///
/// The runtime emits this event once when the whole dynamic character settles.
/// The character stays dynamic unless settings request automatic freezing, so
/// observers can apply their own follow-up policy without inferring state from
/// a single body velocity.
#[derive(Clone, Copy, Debug, EntityEvent, bevy::prelude::Reflect)]
pub struct RagdollSettled {
    /// Character entity whose complete body set satisfied the configured settle
    /// interval. Observers inspect this target after the state transition and
    /// can read current mode and body relationships.
    pub entity: Entity,
}

/// Fires after a settled or budget-evicted ragdoll changes into `Frozen` mode.
///
/// Freezing retains body entities and their last physics poses while preventing
/// dynamic integration. The event identifies the character so observers can
/// preserve its pose or remove transient effects without searching all physics
/// bodies.
#[derive(Clone, Copy, Debug, EntityEvent, bevy::prelude::Reflect)]
pub struct RagdollFrozen {
    /// Character entity whose body kinds changed to fixed after the freeze
    /// transition completed. Observers inspect this target after the state
    /// transition and can read current mode and body relationships.
    pub entity: Entity,
}

/// Fires when a new dynamic activation freezes the oldest character to satisfy
/// the configured limit.
///
/// The event is sent after the evicted character enters `Frozen` mode and
/// identifies that character, not the character requesting dynamic mode.
/// Observers can use the stable ragdoll identity to correlate the eviction with
/// gameplay or telemetry state.
#[derive(Clone, Copy, Debug, EntityEvent, bevy::prelude::Reflect)]
pub struct RagdollBudgetEvicted {
    /// Character entity frozen because the configured dynamic ragdoll limit was
    /// full. Observers inspect this target after the state transition and can
    /// read current mode and body relationships.
    pub entity: Entity,
}
