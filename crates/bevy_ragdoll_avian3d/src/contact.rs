//! Exclude profile-defined ragdoll body pairs through Avian collision hooks.

use avian3d::prelude::CollisionHooks;
use bevy::ecs::system::SystemParam;
use bevy::prelude::{Commands, Entity, Query};
use bevy_ragdoll::profile::BodyIndex;
use bevy_ragdoll::runtime::body::NoContactWith;
use bevy_ragdoll::runtime::components::RagdollBodyOf;

/// Ragdoll body owner, profile index, and exclusion mask read by the hooks.
pub type RagdollPairQuery<'w, 's> = Query<
    'w,
    's,
    (
        &'static RagdollBodyOf,
        &'static BodyIndex,
        &'static NoContactWith,
    ),
>;

/// Default Avian collision hooks that suppress only masked pairs within the
/// same character and keep every other pair.
///
/// Register them with
/// `PhysicsPlugins::new(FixedUpdate).with_collision_hooks::<AvianRagdollHooks>()`
/// when the application has no hooks of its own. Call
/// [`ragdoll_filter_pairs`] from a custom `CollisionHooks::filter_pairs`
/// otherwise. The adapter inserts `ActiveCollisionHooks::FILTER_PAIRS` on every
/// ragdoll collider.
#[derive(SystemParam)]
pub struct AvianRagdollHooks<'w, 's> {
    /// Body owner, profile index, and symmetric no-contact mask for each body.
    ragdoll_bodies: RagdollPairQuery<'w, 's>,
}

impl CollisionHooks for AvianRagdollHooks<'_, '_> {
    fn filter_pairs(&self, collider1: Entity, collider2: Entity, _: &mut Commands) -> bool {
        ragdoll_filter_pairs(collider1, collider2, &self.ragdoll_bodies)
    }
}

/// Applies the ragdoll contact policy from a user-defined Avian hook.
///
/// Returns `false` only when both colliders are ragdoll bodies of the same
/// character and either profile mask excludes the other body. Ragdoll colliders
/// are their own rigid bodies, so collider entities are body entities.
///
/// # Examples
///
/// ```no_run
/// use avian3d::prelude::CollisionHooks;
/// use bevy::ecs::system::SystemParam;
/// use bevy::prelude::{Commands, Entity};
/// use bevy_ragdoll_avian3d::{RagdollPairQuery, ragdoll_filter_pairs};
///
/// #[derive(SystemParam)]
/// struct MyHooks<'w, 's> {
///     bodies: RagdollPairQuery<'w, 's>,
/// }
///
/// impl CollisionHooks for MyHooks<'_, '_> {
///     fn filter_pairs(&self, a: Entity, b: Entity, _: &mut Commands) -> bool {
///         ragdoll_filter_pairs(a, b, &self.bodies)
///     }
/// }
/// ```
pub fn ragdoll_filter_pairs(
    collider1: Entity,
    collider2: Entity,
    ragdoll_bodies: &RagdollPairQuery<'_, '_>,
) -> bool {
    let (Ok((first_owner, first_index, first_mask)), Ok((second_owner, second_index, second_mask))) =
        (ragdoll_bodies.get(collider1), ragdoll_bodies.get(collider2))
    else {
        return true;
    };
    first_owner.0 != second_owner.0
        || !pair_is_excluded(*first_index, first_mask.0, *second_index, second_mask.0)
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
    //! Checks pair filtering across non-ragdoll, owner, and mask boundaries.

    use bevy::ecs::system::SystemState;
    use bevy::prelude::{Entity, World};
    use bevy_ragdoll::profile::BodyIndex;
    use bevy_ragdoll::runtime::body::NoContactWith;
    use bevy_ragdoll::runtime::components::RagdollBodyOf;

    use super::{RagdollPairQuery, ragdoll_filter_pairs};

    /// Spawns one tagged body with a profile index and exclusion mask.
    fn body(world: &mut World, owner: Entity, index: usize, mask: u64) -> Entity {
        let index = BodyIndex::try_from(index).expect("test body index is valid");
        world
            .spawn((RagdollBodyOf(owner), index, NoContactWith(mask)))
            .id()
    }

    /// Only a masked pair with one owner is rejected.
    #[test]
    fn filter_rejects_only_masked_same_owner_pairs() {
        let mut world = World::new();
        let owner = world.spawn_empty().id();
        let other_owner = world.spawn_empty().id();
        let pelvis = body(&mut world, owner, 0, 1 << 1);
        let spine = body(&mut world, owner, 1, 0);
        let head = body(&mut world, owner, 2, 0);
        let foreign_spine = body(&mut world, other_owner, 1, 0);
        let floor = world.spawn_empty().id();
        let mut state: SystemState<RagdollPairQuery> = SystemState::new(&mut world);
        let bodies = state.get(&world).expect("the query is valid");

        assert!(!ragdoll_filter_pairs(pelvis, spine, &bodies));
        assert!(!ragdoll_filter_pairs(spine, pelvis, &bodies));
        assert!(ragdoll_filter_pairs(pelvis, head, &bodies));
        assert!(ragdoll_filter_pairs(pelvis, foreign_spine, &bodies));
        assert!(ragdoll_filter_pairs(pelvis, floor, &bodies));
        assert!(ragdoll_filter_pairs(floor, pelvis, &bodies));
    }
}
