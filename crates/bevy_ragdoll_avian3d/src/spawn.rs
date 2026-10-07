//! Lift new ragdoll poses clear of static Avian geometry.

use avian3d::prelude::{SpatialQuery, SpatialQueryFilter};
use bevy::math::{Isometry3d, Vec3};
use bevy::prelude::Entity;
use bevy_ragdoll::profile::ShapeSpec;
use num_traits::ToPrimitive;

use crate::shape::collider_for_shape;

/// Finds a shared upward correction for bodies overlapping static geometry.
///
/// The adapter checks each body at 1 cm intervals up to the configured bound,
/// matching TGF's spawn-lift policy. A body with no clear position in range
/// contributes no lift; the largest successful correction moves the whole rig.
/// `is_world_collider` accepts only static, non-ragdoll colliders.
pub(crate) fn spawn_lift(
    spatial_query: &SpatialQuery<'_, '_>,
    bodies: &[(ShapeSpec, Isometry3d)],
    max_spawn_lift: f32,
    is_world_collider: &dyn Fn(Entity) -> bool,
) -> f32 {
    let steps = lift_steps(max_spawn_lift);
    let mut lift = 0.0_f32;
    for (shape, pose) in bodies {
        // Test each shape at its captured world pose before trying upward offsets.
        let collider = collider_for_shape(*shape);
        let overlaps = |offset: f32| {
            let mut is_intersecting = false;
            spatial_query.shape_intersections_callback(
                &collider,
                Vec3::from(pose.translation) + Vec3::Y * offset,
                pose.rotation,
                &SpatialQueryFilter::default(),
                |entity| {
                    is_intersecting = is_world_collider(entity);
                    !is_intersecting
                },
            );
            is_intersecting
        };
        if !overlaps(0.0) {
            continue;
        }
        // Keep the first clear centimetre position; one maximum moves every body.
        if let Some(body_lift) = (1..=steps)
            .map(|step| step as f32 * 0.01)
            .find(|distance| !overlaps(*distance))
        {
            lift = lift.max(body_lift);
        }
    }
    lift
}

/// Converts a finite lift bound to TGF's rounded centimetre count.
fn lift_steps(max_spawn_lift: f32) -> usize {
    if max_spawn_lift.is_finite() && max_spawn_lift > 0.0 {
        (max_spawn_lift / 0.01)
            .round()
            .to_usize()
            .unwrap_or(usize::MAX)
    } else {
        0
    }
}

#[cfg(test)]
mod tests {
    //! Checks spawn-lift discretization at its configured boundaries.

    use super::lift_steps;

    /// Zero, invalid, and negative bounds disable the lift scan.
    #[test]
    fn invalid_or_zero_bound_has_no_steps() {
        for bound in [0.0, -0.01, f32::NAN, f32::INFINITY] {
            assert_eq!(lift_steps(bound), 0);
        }
    }

    /// A half-metre limit checks fifty centimetre positions.
    #[test]
    fn half_metre_bound_has_fifty_steps() {
        assert_eq!(lift_steps(0.5), 50);
    }

    /// Bounds beyond the addressable step count saturate.
    #[test]
    fn oversized_bound_saturates_at_the_usize_limit() {
        assert_eq!(lift_steps(f32::MAX), usize::MAX);
    }
}
