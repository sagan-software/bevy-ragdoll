//! Interpolate physics poses and write blended local transforms to skeleton
//! bones.
//!
//! Writeback runs after animation and before global transform propagation. It
//! reads fixed-step history, composes each bone from parent-first local
//! transforms, applies the animation-to-physics blend, and skips unchanged
//! transforms to limit Bevy change detection. Physics pose history is indexed
//! by validated profile body positions, while non-body bones keep their
//! animated locals.

use bevy::math::{Affine3A, Isometry3d, Vec3};
use bevy::prelude::{ChildOf, Entity, Transform, World};
use bevy::time::{Fixed, Time};

use crate::profile::BodyIndex;

use super::body::BodyPhysicsPose;
use super::components::{Ragdoll, RagdollBlend, RagdollBodies, RagdollBodyWeights, RagdollMode};
use super::drive::interpolate_pose;
use super::skeleton::SkeletonMap;

/// Linearly interpolates two positions after clamping the weight to the
/// inclusive unit interval.
///
/// This helper is useful to backend adapters that need the same render-time
/// interpolation as core skeleton writeback. Values below zero select
/// `previous`; values above one select `current`.
///
/// # Examples
///
/// ```
/// use bevy::math::Vec3; use
/// bevy_ragdoll::runtime::writeback::interpolate_translation;
///
/// assert_eq!(interpolate_translation(Vec3::ZERO, Vec3::X, 0.5), Vec3::X *
/// 0.5);
/// ```
#[must_use]
pub fn interpolate_translation(previous: Vec3, current: Vec3, alpha: f32) -> Vec3 {
    previous.lerp(current, alpha.clamp(0.0, 1.0))
}

/// Interpolates rigid poses using clamped translation and shortest-path
/// rotation weights.
///
/// The input isometries must represent rigid transforms; scale is not stored in
/// this physics pose type. Weights outside `0..=1` select the corresponding
/// endpoint without extrapolation.
///
/// # Examples
///
/// ```
/// use bevy::math::{Isometry3d, Vec3}; use
/// bevy_ragdoll::runtime::writeback::interpolate_physics_pose;
///
/// let end = Isometry3d::from_translation(Vec3::X);
/// assert_eq!(interpolate_physics_pose(Isometry3d::IDENTITY, end, 1.0), end);
/// ```
#[must_use]
pub fn interpolate_physics_pose(
    previous: Isometry3d,
    current: Isometry3d,
    alpha: f32,
) -> Isometry3d {
    interpolate_pose(previous, current, alpha)
}

/// Interpolates body poses and blends them into each bound bone's local
/// transform.
pub(crate) fn writeback(world: &mut World) {
    // Skip writeback when the fixed clock is unavailable instead of panicking on resource lookup.
    let Some(alpha) = world
        .get_resource::<Time<Fixed>>()
        .map(Time::overstep_fraction)
    else {
        return;
    };
    // Snapshot characters because local transforms are mutated during the write pass.
    let characters = {
        let mut query = world.query_filtered::<Entity, bevy::prelude::With<Ragdoll>>();
        query.iter(world).collect::<Vec<_>>()
    };
    for character in characters {
        write_character(world, character, alpha);
    }
}

/// Computes and applies one character's blended local transforms when physics
/// owns its pose.
fn write_character(world: &mut World, character: Entity, alpha: f32) {
    // Only dynamic and frozen characters have physics poses that affect animation output.
    if !matches!(
        world.get::<RagdollMode>(character),
        Some(RagdollMode::Dynamic | RagdollMode::Frozen)
    ) {
        return;
    }
    let Some(map) = world.get::<SkeletonMap>(character).cloned() else {
        return;
    };

    // Snapshot blend controls before composing local and physics transforms.
    let blend = world
        .get::<RagdollBlend>(character)
        .copied()
        .unwrap_or_default()
        .get();
    let weights = world
        .get::<RagdollBodyWeights>(character)
        .cloned()
        .unwrap_or_default();
    let body_poses = body_poses(world, character, map.body_to_bone.len(), alpha);
    let character_pose = world_pose(world, character);

    // Compose every bone parent-first and retain only changed transforms for mutation.
    let changes = blended_local_changes(world, &map, &body_poses, &weights, blend, character_pose);
    for (bone_entity, transform) in changes {
        if let Ok(mut entity) = world.get_entity_mut(bone_entity) {
            entity.insert(transform);
        }
    }
}

/// Composes parent-first physics poses and calculates changed local transforms
/// for body bones.
fn blended_local_changes(
    world: &World,
    map: &SkeletonMap,
    body_poses: &[Option<Isometry3d>],
    weights: &RagdollBodyWeights,
    blend: f32,
    character_pose: Isometry3d,
) -> Vec<(Entity, Transform)> {
    let mut physics_world = Vec::with_capacity(map.bones.len());
    let mut changes = Vec::with_capacity(map.body_to_bone.len());

    for bone in &map.bones {
        // Use an already-composed parent pose, or the character pose for a root bone.
        let parent_pose = bone
            .parent
            .and_then(|parent| physics_world.get(parent).copied())
            .unwrap_or(character_pose);
        let animated = world
            .get::<Transform>(bone.entity)
            .copied()
            .unwrap_or(bone.rest_local);
        let animated_local = Isometry3d::new(animated.translation, animated.rotation);

        // Prefer the interpolated body pose and otherwise preserve the animated hierarchy pose.
        let pose = bone
            .body
            .and_then(|index| body_poses.get(index).copied().flatten())
            .unwrap_or(parent_pose * animated_local);
        physics_world.push(pose);

        // Non-body bones contribute hierarchy transforms and retain their animated local values.
        let Some(body_index) = bone.body else {
            continue;
        };
        // Combine global blend with the indexed per-body override before interpolation.
        let local_physics = parent_pose.inverse() * pose;
        let body_blend = blend
            * weights
                .get(body_index)
                .map_or(1.0, super::components::BodyWeights::muscle);
        let next = blended_body_transform(animated, local_physics, body_blend);

        // Preserve scale and skip sub-micrometre changes to limit Bevy change detection.
        if !animated.translation.abs_diff_eq(next.translation, 1.0e-6)
            || 1.0 - animated.rotation.dot(next.rotation).abs() > 1.0e-6
        {
            changes.push((bone.entity, next));
        }
    }

    changes
}

/// Interpolates a body's local translation and rotation while preserving animated scale.
fn blended_body_transform(animated: Transform, physics: Isometry3d, blend: f32) -> Transform {
    // Keep scale from animation because rigid physics poses contain no scale information.
    let mut blended = animated;
    blended.translation = animated.translation.lerp(physics.translation.into(), blend);
    blended.rotation = animated.rotation.slerp(physics.rotation, blend);
    blended
}

/// Reads indexed body poses and interpolates them by the fixed-step overstep.
fn body_poses(
    world: &World,
    character: Entity,
    body_count: usize,
    alpha: f32,
) -> Vec<Option<Isometry3d>> {
    // Allocate one optional pose per profile entry so sparse backend state stays index-stable.
    let mut poses = vec![None; body_count];
    let Some(bodies) = world.get::<RagdollBodies>(character) else {
        return poses;
    };
    // Ignore bodies that have not reported both a checked profile index and pose history.
    for entity in bodies.iter() {
        let Some(index) = world.get::<BodyIndex>(entity).map(|index| index.get()) else {
            continue;
        };
        let Some(body_pose) = world.get::<BodyPhysicsPose>(entity) else {
            continue;
        };
        if let Some(slot) = poses.get_mut(index) {
            // Interpolate only after both profile identity and backend pose are available.
            *slot = Some(interpolate_pose(
                body_pose.previous,
                body_pose.current,
                alpha,
            ));
        }
    }
    poses
}

/// Computes a current world pose from local transforms, without reading
/// globals.
pub(crate) fn world_pose(world: &World, entity: Entity) -> Isometry3d {
    // Collect ancestors because local transforms must be multiplied from root to child.
    let mut hierarchy = vec![entity];
    let mut current = entity;
    while let Some(parent) = world.get::<ChildOf>(current).map(|relation| relation.0) {
        hierarchy.push(parent);
        current = parent;
    }
    hierarchy.reverse();
    // Accumulate affine transforms so parent scale affects descendant translations.
    let mut affine = Affine3A::IDENTITY;
    for node in hierarchy {
        if let Some(transform) = world.get::<Transform>(node) {
            affine *= transform.compute_affine();
        }
    }
    // Drop scale only at the rigid-pose boundary after all local transforms were composed.
    let (_, rotation, translation) = affine.to_scale_rotation_translation();
    Isometry3d::new(translation, rotation)
}

#[cfg(test)]
mod tests {
    //! Edge checks for interpolation, body lookup, and hierarchy composition.

    use bevy::math::{Isometry3d, Quat, Vec3};
    use bevy::prelude::{ChildOf, Entity, Handle, Transform, World};
    use bevy::time::{Fixed, Time};

    use crate::profile::BodyIndex;
    use crate::profile::RagdollProfile;
    use crate::runtime::body::BodyPhysicsPose;
    use crate::runtime::components::{Ragdoll, RagdollBodyOf, RagdollMode};

    use super::{
        body_poses, interpolate_physics_pose, interpolate_translation, world_pose, writeback,
    };

    /// Converts a small test index to the profile's validated body index.
    fn body_index(value: usize) -> BodyIndex {
        BodyIndex::try_from(value).expect("test body indices are in range")
    }

    /// Public interpolation helpers clamp weights below zero and above one.
    #[test]
    fn interpolation_clamps_out_of_range_weights() {
        // Weights outside 0..=1 would extrapolate past the two physics poses.
        let previous = Isometry3d::IDENTITY;
        let current = Isometry3d::new(Vec3::new(2.0, 0.0, 0.0), Quat::from_rotation_z(1.0));

        assert_eq!(
            [-1.0, 2.0].map(|weight| interpolate_translation(Vec3::ZERO, Vec3::X * 2.0, weight)),
            [Vec3::ZERO, Vec3::X * 2.0]
        );
        // Whole poses clamp the same way as translations.
        assert_eq!(
            [-1.0, 2.0].map(|weight| interpolate_physics_pose(previous, current, weight)),
            [previous, current]
        );
    }

    /// Writeback skips when the fixed clock is unavailable.
    #[test]
    fn writeback_requires_fixed_time() {
        let mut world = World::new();

        // A world without Time<Fixed> has no overstep to interpolate, so writeback returns early.
        writeback(&mut world);
    }

    /// Body lookup ignores missing fields and indices outside the profile
    /// length.
    #[test]
    fn body_pose_lookup_handles_missing_and_out_of_range_components() {
        let mut world = World::new();
        // Bodies without an index or with an out-of-range index must be skipped.
        let character = world.spawn_empty().id();
        world.spawn(RagdollBodyOf(character));
        world.spawn((RagdollBodyOf(character), body_index(1)));
        world.spawn((
            RagdollBodyOf(character),
            body_index(2),
            BodyPhysicsPose {
                previous: Isometry3d::IDENTITY,
                current: Isometry3d::from_translation(Vec3::splat(20.0)),
            },
        ));
        // The valid body 0 interpolates halfway between its two poses.
        world.spawn((
            RagdollBodyOf(character),
            body_index(0),
            BodyPhysicsPose {
                previous: Isometry3d::IDENTITY,
                current: Isometry3d::from_translation(Vec3::X * 2.0),
            },
        ));

        let poses = body_poses(&world, character, 1, 0.5);

        assert_eq!(poses, [Some(Isometry3d::from_translation(Vec3::X))]);
        // An unknown character yields one empty slot per requested body.
        assert_eq!(
            body_poses(&world, Entity::PLACEHOLDER, 2, 0.5),
            [None, None]
        );
    }

    /// World pose composition follows parents and treats a missing local
    /// transform as identity.
    #[test]
    fn world_pose_composes_transforms_and_skips_missing_transforms() {
        let mut world = World::new();
        // The parent is rotated a quarter turn, so the child's +X offset maps to +Y.
        let parent = world
            .spawn(
                Transform::from_xyz(3.0, 1.0, 0.0)
                    .with_rotation(Quat::from_rotation_z(std::f32::consts::FRAC_PI_2)),
            )
            .id();
        let child = world
            .spawn((ChildOf(parent), Transform::from_xyz(1.0, 0.0, 0.0)))
            .id();
        // An entity without a Transform contributes identity and inherits its parent pose.
        let no_transform = world.spawn(ChildOf(parent)).id();

        let child_pose = world_pose(&world, child);
        let parent_pose = world_pose(&world, parent);
        let no_transform_pose = world_pose(&world, no_transform);

        // The composed child pose and the transform-less child pose are both checked.
        assert!(
            child_pose
                .translation
                .abs_diff_eq(Vec3::new(3.0, 2.0, 0.0).into(), 1.0e-5)
        );
        assert!(
            no_transform_pose
                .translation
                .abs_diff_eq(parent_pose.translation, 1.0e-5)
        );
    }

    /// Writeback skips characters without an active mode or a bound skeleton
    /// map.
    #[test]
    fn writeback_skips_inactive_and_unbound_characters() {
        let mut world = World::new();
        world.insert_resource(Time::<Fixed>::from_hz(60.0));
        // An Animated character and an unbound Dynamic character both have nothing to write back.
        world.spawn((
            Ragdoll::new(Handle::<RagdollProfile>::default()),
            RagdollMode::Animated,
        ));
        world.spawn((
            Ragdoll::new(Handle::<RagdollProfile>::default()),
            RagdollMode::Dynamic,
        ));

        // Writeback must finish without touching either character.
        writeback(&mut world);
    }
}
