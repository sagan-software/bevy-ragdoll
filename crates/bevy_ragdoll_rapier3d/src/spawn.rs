//! Build Rapier body shapes and lift profile poses clear of fixed geometry.

use num_traits::ToPrimitive;

use bevy::math::Isometry3d;
use bevy::prelude::Query;
use bevy::prelude::With;
use bevy_ragdoll::profile::ShapeSpec;
use bevy_ragdoll::runtime::components::RagdollBodyOf;
use bevy_rapier3d::pipeline::{QueryFilter, QueryFilterFlags};
use bevy_rapier3d::plugin::RapierContext;

use crate::shape::collider_for_shape;

/// Finds a shared upward correction for bodies overlapping fixed world geometry.
///
/// The adapter checks each body at 1 cm intervals up to the configured bound,
/// matching the reference spawn-lift policy. A body with no clear position in range
/// contributes no lift; the largest successful correction moves the whole rig.
pub(crate) fn spawn_lift(
    context: &RapierContext<'_>,
    bodies: &[(ShapeSpec, Isometry3d)],
    max_spawn_lift: f32,
    ragdoll_colliders: &Query<'_, '_, (), With<RagdollBodyOf>>,
) -> f32 {
    // Quantize the configured bound to centimetre search positions.
    let steps = lift_steps(max_spawn_lift);
    // Exclude ragdoll bodies so profiles do not push one another during spawn correction.
    let is_world_collider = |entity| ragdoll_colliders.get(entity).is_err();
    let filter = QueryFilter {
        flags: QueryFilterFlags::ONLY_FIXED,
        groups: None,
        exclude_collider: None,
        exclude_rigid_body: None,
        predicate: Some(&is_world_collider),
    };
    let mut lift = 0.0_f32;
    for (shape, pose) in bodies {
        // Test each shape at its captured world pose before trying upward offsets.
        let collider = collider_for_shape(*shape);
        let is_overlapping = shape_is_intersecting_fixed(context, &collider, *pose, filter);
        if !is_overlapping {
            continue;
        }
        // Search increasing offsets and keep the first clear centimetre position.
        let mut body_lift = None;
        for step in 1..=steps {
            let distance = step as f32 * 0.01;
            let candidate = Isometry3d::new(
                pose.translation + bevy::math::Vec3A::Y * distance,
                pose.rotation,
            );
            if !shape_is_intersecting_fixed(context, &collider, candidate, filter) {
                body_lift = Some(distance);
                break;
            }
        }
        if let Some(body_lift) = body_lift {
            // One maximum correction keeps every body and authored joint frame aligned.
            lift = lift.max(body_lift);
        }
    }
    lift
}

/// Returns whether one collider shape intersects a non-ragdoll fixed collider.
fn shape_is_intersecting_fixed(
    context: &RapierContext<'_>,
    collider: &bevy_rapier3d::prelude::Collider,
    pose: Isometry3d,
    filter: QueryFilter<'_>,
) -> bool {
    let mut is_intersecting = false;
    context.intersect_shape(
        pose.translation.into(),
        pose.rotation,
        &*collider.raw,
        filter,
        |_| {
            is_intersecting = true;
            false
        },
    );
    is_intersecting
}

/// Converts a finite lift bound to a rounded centimetre count.
fn lift_steps(max_spawn_lift: f32) -> usize {
    if max_spawn_lift.is_finite() && max_spawn_lift > 0.0 {
        let rounded_steps = (max_spawn_lift / 0.01).round();
        rounded_steps.to_usize().unwrap_or(usize::MAX)
    } else {
        0
    }
}

#[cfg(test)]
mod tests {
    //! Checks fixed-step spawn-lift discretization at its configured boundaries.

    use super::lift_steps;

    /// Zero, invalid, and negative bounds disable the lift scan.
    #[test]
    fn invalid_or_zero_bound_has_no_steps() {
        for bound in [0.0, -0.01, f32::NAN, f32::INFINITY] {
            assert_eq!(lift_steps(bound), 0);
        }
    }

    /// A valid half-metre limit checks fifty centimetre positions.
    #[test]
    fn half_metre_bound_has_fifty_steps() {
        assert_eq!(lift_steps(0.5), 50);
    }

    /// Bounds beyond the addressable step count saturate without a lossy cast.
    #[test]
    fn oversized_bound_saturates_at_the_usize_limit() {
        assert_eq!(lift_steps(f32::MAX), usize::MAX);
    }
}
