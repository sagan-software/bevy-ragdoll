//! Builds a [`Skeleton`] from a character's entity hierarchy.

use std::collections::{HashSet, VecDeque};

use bevy::math::{Affine3A, Isometry3d};
use bevy::mesh::skinning::SkinnedMesh;
use bevy::prelude::{Children, Entity, Name, Transform, World};

use super::{RagdollBone, RagdollOverrides, Skeleton, SkeletonBone};

/// Reads the skeleton below `character`, or `None` while it has no bones.
///
/// When skinned meshes exist, their joints are the bones; otherwise every
/// named descendant is a bone. Rest poses compose local transforms from the
/// character down. A [`RagdollBone`] component on a bone beats `overrides`.
#[expect(
    clippy::indexing_slicing,
    reason = "bone indexes come from the same parent-first skeleton vectors"
)]
pub(crate) fn skeleton_from_world(
    world: &World,
    character: Entity,
    overrides: Option<&RagdollOverrides>,
) -> Option<Skeleton> {
    let (order, joints) = descendants(world, character);
    // Keep joints of skinned meshes when present, else all named entities.
    let is_bone = |entity: Entity| {
        world.get::<Name>(entity).is_some() && (joints.is_empty() || joints.contains(&entity))
    };
    // `bone_of` maps each listed entity to its nearest bone, itself included.
    let mut bone_of = vec![None; order.len()];
    let mut skeleton = Skeleton::default();
    for (index, (entity, pose, parent)) in order.iter().enumerate() {
        // The nearest bone ancestor is the parent; non-bone nodes pass theirs on.
        let parent_bone = parent.and_then(|parent| bone_of[parent]);
        let Some(name) = world.get::<Name>(*entity).filter(|_| is_bone(*entity)) else {
            bone_of[index] = parent_bone;
            continue;
        };
        let name = name.as_str().to_owned();
        let (_, rotation, translation) = pose.to_scale_rotation_translation();
        // A component on the bone entity beats the overrides asset.
        let overrides = world
            .get::<RagdollBone>(*entity)
            .or_else(|| overrides.and_then(|overrides| overrides.get(&name)))
            .copied()
            .unwrap_or_default();
        skeleton.bones.push(SkeletonBone {
            name,
            parent: parent_bone,
            rest: Isometry3d::new(translation, rotation.normalize()),
            overrides,
        });
        bone_of[index] = Some(skeleton.bones.len() - 1);
    }
    (!skeleton.bones.is_empty()).then_some(skeleton)
}

/// Lists descendants of `character` with their character-space pose and parent.
///
/// Each entry's parent is the index of its nearest listed ancestor. The set
/// holds every joint entity named by a skinned mesh below `character`.
fn descendants(
    world: &World,
    character: Entity,
) -> (Vec<(Entity, Affine3A, Option<usize>)>, HashSet<Entity>) {
    // Breadth-first order keeps parents before children.
    let mut order = Vec::new();
    let mut queue = VecDeque::from([(character, Affine3A::IDENTITY, None::<usize>)]);
    let mut joints = HashSet::new();
    while let Some((entity, parent_pose, parent)) = queue.pop_front() {
        // The character itself is the origin of skeleton space.
        let pose = if entity == character {
            Affine3A::IDENTITY
        } else {
            parent_pose
                * world
                    .get::<Transform>(entity)
                    .copied()
                    .unwrap_or_default()
                    .compute_affine()
        };
        if let Some(skin) = world.get::<SkinnedMesh>(entity) {
            joints.extend(skin.joints.iter().copied());
        }
        // The character is not listed, so its children get no parent index.
        let index = (entity != character).then(|| {
            order.push((entity, pose, parent));
            order.len() - 1
        });
        if let Some(children) = world.get::<Children>(entity) {
            queue.extend(
                children
                    .iter()
                    .map(|child| (*child, pose, index.or(parent))),
            );
        }
    }
    (order, joints)
}
