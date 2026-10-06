//! Capture animated local transforms into profile-ordered skeleton-space
//! targets.
//!
//! The capture system runs after Bevy animation and composes local transforms
//! from each mapped skeleton root, avoiding stale `GlobalTransform` values from
//! earlier propagation. It preserves scale effects on child positions, then
//! stores rigid poses and per-second velocities for the fixed drive stage.
//! Missing character resources or stale entities are skipped without panicking.

use bevy::math::{Isometry3d, Vec3};
use bevy::prelude::{Component, Entity, World};
use bevy::time::Time;

use super::body::BodyVelocity;
use super::components::RagdollTargetPose;
use super::drive::rotation_error;
use super::skeleton::SkeletonMap;

/// Marks target poses that the runtime refreshes from animation after each
/// variable-rate update.
///
/// The marker is private to the runtime because capture eligibility follows
/// successful skeleton binding; applications control target adjustment through
/// public components and schedule sets.
#[derive(Component, Clone, Copy, Debug, Default, bevy::prelude::Reflect)]
pub(crate) struct AutoCapture;

/// Captures target poses and derives their velocities after animation systems.
pub(crate) fn capture_targets(world: &mut World) {
    // Snapshot eligible characters before mutating their captured target components.
    let characters = {
        let mut query = world.query_filtered::<Entity, (
            bevy::prelude::With<SkeletonMap>,
            bevy::prelude::With<AutoCapture>,
            bevy::prelude::With<RagdollTargetPose>,
        )>();
        query.iter(world).collect::<Vec<_>>()
    };
    let Some(delta_seconds) = world.get_resource::<Time>().map(Time::delta_secs) else {
        return;
    };

    for character in characters {
        // Clone only the small mapping so the world can be immutably read during pose composition.
        let Some(map) = world.get::<SkeletonMap>(character).cloned() else {
            continue;
        };
        let Some(previous) = world.get::<RagdollTargetPose>(character).cloned() else {
            continue;
        };
        // Derive velocities from the prior capture before replacing its history.
        let poses = capture_poses(world, &map);
        let velocities = derive_velocities(previous.current(), &poses, delta_seconds);
        // The character may disappear only after the query snapshot, so skip stale entities.
        if let Some(mut target_pose) = world.get_mut::<RagdollTargetPose>(character) {
            target_pose.record(poses, velocities);
        }
    }
}

/// Composes animated local affines parent-first and returns rigid body poses.
///
/// The map is built from validated profile order, with parents before children
/// and body indexes pointing into that ordered vector. Invalid internal indexes
/// are skipped safely instead of panicking; normal binding produces one output
/// pose for every profile body.
pub(crate) fn capture_poses(world: &World, map: &SkeletonMap) -> Vec<Isometry3d> {
    // Compose each local transform so parent scale continues to affect descendant translations.
    let mut skeleton_affines = Vec::with_capacity(map.bones.len());
    for bone in &map.bones {
        // Read animated local transforms and compose them without stale globals.
        let local = world
            .get::<bevy::prelude::Transform>(bone.entity)
            .copied()
            .unwrap_or(bone.rest_local)
            .compute_affine();
        let affine = bone
            .parent
            .and_then(|parent| skeleton_affines.get(parent).copied())
            .map(|parent| parent * local)
            .unwrap_or(local);
        skeleton_affines.push(affine);
    }

    // Convert mapped bones after composition, dropping scale only after its effect on positions.
    map.body_to_bone
        .iter()
        .filter_map(|bone_index| skeleton_affines.get(*bone_index))
        .map(|affine| {
            let (_, rotation, translation) = affine.to_scale_rotation_translation();
            Isometry3d::new(translation, rotation)
        })
        .collect()
}

/// Derives per-second linear and shortest-arc angular velocities.
fn derive_velocities(
    previous: &[Isometry3d],
    current: &[Isometry3d],
    delta_seconds: f32,
) -> Vec<BodyVelocity> {
    if !delta_seconds.is_finite() || delta_seconds <= 0.0 {
        return vec![BodyVelocity::default(); current.len()];
    }

    current
        .iter()
        .enumerate()
        .map(|(index, pose)| {
            let Some(previous) = previous.get(index) else {
                return BodyVelocity::default();
            };
            BodyVelocity {
                linear: Vec3::from(pose.translation - previous.translation) / delta_seconds,
                angular: rotation_error(previous.rotation, pose.rotation) / delta_seconds,
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    //! Focused checks for target velocity derivation at time and history edges.

    use bevy::math::{Isometry3d, Vec3};
    use bevy::prelude::World;

    use crate::runtime::body::BodyVelocity;

    use super::{capture_targets, derive_velocities};

    /// Capture skips when Bevy has no frame clock resource.
    #[test]
    fn capture_requires_frame_time() {
        let mut world = World::new();

        capture_targets(&mut world);
    }

    /// Invalid or nonpositive frame durations produce zero velocities.
    #[test]
    fn invalid_delta_produces_zero_velocities() {
        let previous = [Isometry3d::IDENTITY];
        let current = [Isometry3d::from_translation(Vec3::X)];

        for delta_seconds in [0.0, -1.0, f32::INFINITY, f32::NAN] {
            assert_eq!(
                derive_velocities(&previous, &current, delta_seconds),
                [BodyVelocity::default()]
            );
        }
    }

    /// New target entries start with zero velocity while prior entries use both
    /// poses.
    #[test]
    fn missing_history_defaults_only_the_new_target_velocity() {
        let previous = [Isometry3d::IDENTITY];
        let current = [
            Isometry3d::from_translation(Vec3::X),
            Isometry3d::from_translation(Vec3::Y),
        ];

        let velocities = derive_velocities(&previous, &current, 0.5);

        assert_eq!(
            velocities.first().map(|velocity| velocity.linear),
            Some(Vec3::X * 2.0)
        );
        assert_eq!(velocities.get(1).copied(), Some(BodyVelocity::default()));
    }
}
