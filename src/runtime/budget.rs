//! Runtime capacity limits and deterministic eviction policy for dynamic
//! ragdolls.
//!
//! Insert [`RagdollBudget`] to bound the number of characters that consume
//! dynamic physics bodies. When a new character exceeds that limit, the runtime
//! freezes the oldest dynamic character and emits a budget event. A limit of
//! `usize::MAX` disables practical eviction while preserving the same
//! accounting path used by constrained games and multiplayer sessions.

use bevy::prelude::Resource;

/// Policy used when activating a dynamic ragdoll would exceed the configured
/// budget.
///
/// Eviction changes the selected character to `Frozen` before the new character
/// enters `Dynamic` mode, then triggers an event for the evicted character so
/// gameplay can release related state.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, bevy::prelude::Reflect)]
pub enum EvictPolicy {
    /// Freeze the longest-running dynamic ragdoll before activating the new
    /// character.
    #[default]
    /// Stable activation identities determine which dynamic character leaves
    /// simulation when the configured count is exceeded.
    FreezeOldest,
}

/// Maximum dynamic ragdoll count and the action used when new activation
/// exceeds it.
///
/// The runtime counts characters in `Dynamic` mode and preserves activation
/// order through stable ragdoll identities. Set `max_dynamic` to zero to
/// prevent dynamic activation, or use the default unlimited count when the game
/// delegates capacity policy to its physics backend.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Resource, bevy::prelude::Reflect)]
pub struct RagdollBudget {
    /// Maximum characters allowed in `Dynamic` mode before the oldest one is
    /// frozen. The runtime reads this value when accounting for active dynamics
    /// and applies the selected policy before activation.
    pub max_dynamic: usize,
    /// Deterministic action applied before a new dynamic ragdoll exceeds the
    /// count limit. The runtime reads this value when accounting for active
    /// dynamics and applies the selected policy before activation.
    pub policy: EvictPolicy,
}

impl Default for RagdollBudget {
    fn default() -> Self {
        Self::new(usize::MAX)
    }
}

impl RagdollBudget {
    /// Creates a budget with the freeze-oldest policy and the supplied
    /// dynamic-character limit.
    ///
    /// A zero limit allows no dynamic ragdolls, while `usize::MAX` effectively
    /// disables eviction. The runtime still uses this budget resource to keep
    /// activation accounting consistent.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::budget::RagdollBudget;
    ///
    /// let budget = RagdollBudget::new(8); assert_eq!(budget.max_dynamic, 8);
    /// ```
    #[must_use]
    pub const fn new(max_dynamic: usize) -> Self {
        Self {
            max_dynamic,
            policy: EvictPolicy::FreezeOldest,
        }
    }
}
