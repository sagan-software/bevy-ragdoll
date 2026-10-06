//! Exclude profile-defined ragdoll body pairs through Rapier contact hooks.

use bevy::ecs::system::SystemParam;
use bevy::prelude::Query;
use bevy_ragdoll::profile::BodyIndex;
use bevy_ragdoll::runtime::body::NoContactWith;
use bevy_ragdoll::runtime::components::RagdollBodyOf;
use bevy_rapier3d::pipeline::{BevyPhysicsHooks, PairFilterContextView};
use bevy_rapier3d::rapier::prelude::SolverFlags;

/// Default Rapier hook that preserves ordinary contacts and reads each ragdoll
/// body's owner, profile index, and exclusion mask to suppress only masked pairs
/// within the same character. Configure `RapierPhysicsPlugin` with this system
/// parameter when the application has no custom hook. Use
/// [`ragdoll_filter_contact_pair`] from a custom hook otherwise.
#[derive(Debug, SystemParam)]
pub struct RapierRagdollHooks<'w, 's> {
    /// Body owner, profile index, and symmetric no-contact mask for each body.
    ragdoll_bodies: Query<
        'w,
        's,
        (
            &'static RagdollBodyOf,
            &'static BodyIndex,
            &'static NoContactWith,
        ),
    >,
}

impl BevyPhysicsHooks for RapierRagdollHooks<'_, '_> {
    fn filter_contact_pair(&self, context: PairFilterContextView<'_>) -> Option<SolverFlags> {
        ragdoll_filter_contact_pair(context, &self.ragdoll_bodies)
    }
}

/// Applies the ragdoll contact policy from a user-defined Rapier hook.
///
/// Call this from a custom `BevyPhysicsHooks::filter_contact_pair` method when
/// the application already supplies its own Rapier hook type. The helper must
/// be used with colliders that enable `ActiveHooks::FILTER_CONTACT_PAIRS`.
///
/// # Examples
///
/// ```no_run
/// use bevy::prelude::Query;
/// use bevy_ragdoll::profile::BodyIndex;
/// use bevy_ragdoll::runtime::body::NoContactWith;
/// use bevy_ragdoll::runtime::components::RagdollBodyOf;
/// use bevy_rapier3d::pipeline::PairFilterContextView;
/// use bevy_rapier3d::rapier::prelude::SolverFlags;
/// use bevy_ragdoll_rapier3d::ragdoll_filter_contact_pair;
///
/// fn filter_contact_pair(
///     context: PairFilterContextView<'_>,
///     bodies: &Query<'_, '_, (&RagdollBodyOf, &BodyIndex, &NoContactWith)>,
/// ) -> Option<SolverFlags> {
///     ragdoll_filter_contact_pair(context, bodies)
/// }
/// ```
pub fn ragdoll_filter_contact_pair(
    context: PairFilterContextView<'_>,
    ragdoll_bodies: &Query<'_, '_, (&RagdollBodyOf, &BodyIndex, &NoContactWith)>,
) -> Option<SolverFlags> {
    // Preserve Rapier's default rejection for pairs without a dynamic body.
    if let (Some(first), Some(second)) = (context.raw.rigid_body1, context.raw.rigid_body2) {
        let first_body = &context.raw.bodies[first];
        let second_body = &context.raw.bodies[second];
        if !first_body.is_dynamic() && !second_body.is_dynamic() {
            return None;
        }
    }

    // Suppress only profile-masked pairs owned by the same ragdoll.
    if let (Some(first_entity), Some(second_entity)) =
        (context.rigid_body1(), context.rigid_body2())
        && let (
            Ok((first_owner, first_index, first_mask)),
            Ok((second_owner, second_index, second_mask)),
        ) = (
            ragdoll_bodies.get(first_entity),
            ragdoll_bodies.get(second_entity),
        )
        && first_owner.0 == second_owner.0
        && pair_is_excluded(*first_index, first_mask.0, *second_index, second_mask.0)
    {
        return None;
    }

    Some(SolverFlags::COMPUTE_IMPULSES)
}

/// Checks symmetric profile exclusion masks for two bodies in one ragdoll.
fn pair_is_excluded(
    first_index: BodyIndex,
    first_mask: u64,
    second_index: BodyIndex,
    second_mask: u64,
) -> bool {
    let is_first_mask_excluding_second = first_mask & (1_u64 << second_index.get()) != 0;
    let is_second_mask_excluding_first = second_mask & (1_u64 << first_index.get()) != 0;
    is_first_mask_excluding_second || is_second_mask_excluding_first
}

#[cfg(test)]
mod tests {
    //! Checks pair filtering across owner and profile-mask boundaries.

    use bevy_ragdoll::profile::BodyIndex;

    use super::pair_is_excluded;

    /// Either body's mask excludes the pair when the owner check has matched.
    #[test]
    fn one_sided_mask_bit_excludes_pair() {
        let first = BodyIndex::try_from(1).expect("profile body index one is valid");
        let second = BodyIndex::try_from(4).expect("profile body index four is valid");
        assert!(pair_is_excluded(first, 1_u64 << 4, second, 0));
        assert!(pair_is_excluded(first, 0, second, 1_u64 << 1));
    }

    /// No matching mask bit leaves the pair enabled.
    #[test]
    fn absent_mask_bits_keep_pair() {
        let first = BodyIndex::try_from(1).expect("profile body index one is valid");
        let second = BodyIndex::try_from(4).expect("profile body index four is valid");
        assert!(!pair_is_excluded(first, 0, second, 0));
    }
}
