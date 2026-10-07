//! Exclude profile-defined ragdoll body pairs through Avian collision hooks.

use avian3d::prelude::CollisionHooks;
use bevy::ecs::system::SystemParam;
use bevy::prelude::{Commands, Entity, Query};
use bevy_ragdoll::profile::BodyIndex;
use bevy_ragdoll::runtime::body::NoContactWith;
use bevy_ragdoll::runtime::components::RagdollBodyOf;

/// Ragdoll body owner, profile index, and exclusion mask read by the hooks.
///
/// Custom hook types hold this query and pass it to
/// [`should_ragdoll_pair_collide`], so they apply the same same-owner mask
/// policy as [`AvianRagdollHooks`].
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
/// [`should_ragdoll_pair_collide`] from a custom `CollisionHooks::filter_pairs`
/// otherwise. The adapter inserts `ActiveCollisionHooks::FILTER_PAIRS` on every
/// ragdoll collider.
#[derive(Debug, SystemParam)]
pub struct AvianRagdollHooks<'w, 's> {
    /// Body owner, profile index, and symmetric no-contact mask for each body.
    ragdoll_bodies: RagdollPairQuery<'w, 's>,
}

impl CollisionHooks for AvianRagdollHooks<'_, '_> {
    fn filter_pairs(&self, collider1: Entity, collider2: Entity, _: &mut Commands<'_, '_>) -> bool {
        should_ragdoll_pair_collide(collider1, collider2, &self.ragdoll_bodies)
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
/// use bevy_ragdoll_avian3d::{RagdollPairQuery, should_ragdoll_pair_collide};
///
/// #[derive(SystemParam)]
/// struct MyHooks<'w, 's> {
///     bodies: RagdollPairQuery<'w, 's>,
/// }
///
/// impl CollisionHooks for MyHooks<'_, '_> {
///     fn filter_pairs(&self, a: Entity, b: Entity, _: &mut Commands<'_, '_>) -> bool {
///         should_ragdoll_pair_collide(a, b, &self.bodies)
///     }
/// }
/// ```
#[must_use]
pub fn should_ragdoll_pair_collide(
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
const fn pair_is_excluded(
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

    use super::{RagdollPairQuery, should_ragdoll_pair_collide};

    /// Spawns one tagged body with a profile index and exclusion mask.
    fn body(world: &mut World, owner: Entity, index: usize, mask: u64) -> Entity {
        let index = BodyIndex::try_from(index).expect("test body index is valid");
        world
            .spawn((RagdollBodyOf(owner), index, NoContactWith(mask)))
            .id()
    }

    /// Bodies of one test world: two owners and one non-ragdoll collider.
    struct PairWorld {
        /// World holding every test entity.
        world: World,
        /// Pelvis whose mask excludes the spine.
        pelvis: Entity,
        /// Spine excluded by the pelvis mask.
        spine: Entity,
        /// Head that no mask excludes.
        head: Entity,
        /// Spine of a second character.
        foreign_spine: Entity,
        /// Collider without ragdoll components.
        floor: Entity,
    }

    /// Builds one character with a masked pelvis-spine pair, a second
    /// character's spine, and a plain floor entity.
    fn pair_world() -> PairWorld {
        let mut world = World::new();
        let owner = world.spawn_empty().id();
        let other_owner = world.spawn_empty().id();
        // The pelvis mask excludes body index 1, the spine.
        let pelvis = body(&mut world, owner, 0, 1 << 1);
        let spine = body(&mut world, owner, 1, 0);
        let head = body(&mut world, owner, 2, 0);
        let foreign_spine = body(&mut world, other_owner, 1, 0);
        // The floor carries no ragdoll components.
        let floor = world.spawn_empty().id();
        PairWorld {
            world,
            pelvis,
            spine,
            head,
            foreign_spine,
            floor,
        }
    }

    /// Evaluates the pair filter for two entities of a test world.
    fn is_colliding(pairs: &mut PairWorld, first: Entity, second: Entity) -> bool {
        let mut state: SystemState<RagdollPairQuery<'static, 'static>> =
            SystemState::new(&mut pairs.world);
        state
            .get(&pairs.world)
            .is_ok_and(|bodies| should_ragdoll_pair_collide(first, second, &bodies))
    }

    /// A masked pair with one owner is rejected in either order.
    #[test]
    fn masked_same_owner_pair_does_not_collide() {
        let mut pairs = pair_world();
        let (pelvis, spine) = (pairs.pelvis, pairs.spine);
        assert!(!is_colliding(&mut pairs, pelvis, spine));
        assert!(!is_colliding(&mut pairs, spine, pelvis));
    }

    /// Unmasked bodies and bodies of another character still collide.
    #[test]
    fn unmasked_or_foreign_pairs_collide() {
        let mut pairs = pair_world();
        let (pelvis, head, foreign) = (pairs.pelvis, pairs.head, pairs.foreign_spine);
        assert!(is_colliding(&mut pairs, pelvis, head));
        assert!(is_colliding(&mut pairs, pelvis, foreign));
    }

    /// A non-ragdoll collider collides with ragdoll bodies in either order.
    #[test]
    fn non_ragdoll_pairs_collide() {
        let mut pairs = pair_world();
        let (pelvis, floor) = (pairs.pelvis, pairs.floor);
        assert!(is_colliding(&mut pairs, pelvis, floor));
        assert!(is_colliding(&mut pairs, floor, pelvis));
    }
}
