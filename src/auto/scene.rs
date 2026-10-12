//! Builds a [`Skeleton`] from a character's entity hierarchy.

use std::collections::{HashMap, HashSet, VecDeque};

use bevy::asset::Assets;
use bevy::math::{Affine3A, Isometry3d};
use bevy::mesh::skinning::{SkinnedMesh, SkinnedMeshInverseBindposes};
use bevy::prelude::{ChildOf, Children, Entity, Name, Transform, World};

use super::{RagdollBone, RagdollOverrides, Skeleton, SkeletonBone};

/// Reads the skeleton below `character`, or `None` while it has no bones.
///
/// When skinned meshes exist, their joints are the bones; otherwise every
/// named descendant is a bone. Rest poses compose local transforms from the
/// character down. A joint whose parent is a joint of the same skin takes its
/// local transform from the skin's inverse bind matrices, so the rest pose is
/// the bind pose even after animation has moved the bones. A [`RagdollBone`]
/// component on a bone beats `overrides`.
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
    let bind_locals = bind_locals(world, character);
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
                * bind_locals.get(&entity).copied().unwrap_or_else(|| {
                    world
                        .get::<Transform>(entity)
                        .copied()
                        .unwrap_or_default()
                        .compute_affine()
                })
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

/// Returns the bind-pose local transform of every skin joint below
/// `character` whose parent is a joint of the same skin.
///
/// A joint's bind pose in mesh space is the inverse of its inverse bind
/// matrix, so its local transform is the parent's inverse bind matrix times
/// its own bind pose; the unknown mesh-space placement cancels. Skins whose
/// matrices have not loaded, non-invertible matrices and joints already seen
/// in an earlier skin are skipped.
fn bind_locals(world: &World, character: Entity) -> HashMap<Entity, Affine3A> {
    let mut locals = HashMap::new();
    // Without the bind pose asset store no skin can contribute a rest pose.
    let Some(bindposes) = world.get_resource::<Assets<SkinnedMeshInverseBindposes>>() else {
        return locals;
    };
    // Walk every descendant once to find the skins.
    let mut stack = vec![character];
    while let Some(entity) = stack.pop() {
        if let Some(children) = world.get::<Children>(entity) {
            stack.extend(children.iter().copied());
        }
        // Skip entities without a skin and skins whose matrices have not loaded.
        let Some(skin) = world.get::<SkinnedMesh>(entity) else {
            continue;
        };
        if let Some(matrices) = bindposes.get(&skin.inverse_bindposes) {
            insert_skin_locals(world, &skin.joints, matrices, &mut locals);
        }
    }
    locals
}

/// Adds the bind-pose local transform of each joint in one skin whose parent
/// is a joint of the same skin, keeping entries from earlier skins.
fn insert_skin_locals(
    world: &World,
    joints: &[Entity],
    matrices: &[bevy::math::Mat4],
    locals: &mut HashMap<Entity, Affine3A>,
) {
    // Each joint's inverse bind matrix, by entity, for the parent lookup.
    let inverse = joints
        .iter()
        .zip(matrices)
        .map(|(joint, matrix)| (*joint, Affine3A::from_mat4(*matrix)))
        .collect::<HashMap<_, _>>();
    for (joint, joint_inverse) in &inverse {
        // Root joints of the skin keep their scene transform.
        let parent = world.get::<ChildOf>(*joint).map(ChildOf::parent);
        let Some(parent_inverse) = parent.and_then(|parent| inverse.get(&parent)) else {
            continue;
        };
        // Reject non-invertible matrices instead of producing a degenerate pose.
        let local = *parent_inverse * joint_inverse.inverse();
        if local.is_finite() && joint_inverse.matrix3.determinant().abs() > f32::EPSILON {
            locals.entry(*joint).or_insert(local);
        }
    }
}
