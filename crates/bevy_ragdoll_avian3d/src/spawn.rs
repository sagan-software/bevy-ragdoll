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
/// matching the Rapier adapter's spawn-lift policy. A body with no clear
/// position in range contributes no lift; the largest successful correction
/// moves the whole rig.
/// `is_world_collider` accepts only static, non-ragdoll colliders.
pub(crate) fn spawn_lift(
    spatial_query: &SpatialQuery<'_, '_>,
    bodies: &[(ShapeSpec, Isometry3d)],
    max_spawn_lift: f32,
    is_world_collider: &dyn Fn(Entity) -> bool,
) -> f32 {
    // Quantize the bound to centimetre steps and start with no lift.
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
        // A body that already clears static geometry needs no lift.
        if !overlaps(0.0) {
            continue;
        }
        // Keep the first clear centimetre position; one maximum moves every body.
        if let Some(body_lift) = (1..=steps)
            .map(|step| f32::from(step) * 0.01)
            .find(|distance| !overlaps(*distance))
        {
            lift = lift.max(body_lift);
        }
    }
    lift
}

/// Converts a finite lift bound to a rounded centimetre count, saturating at
/// `u16::MAX` (655.35 m).
fn lift_steps(max_spawn_lift: f32) -> u16 {
    if max_spawn_lift.is_finite() && max_spawn_lift > 0.0 {
        (max_spawn_lift / 0.01).round().to_u16().unwrap_or(u16::MAX)
    } else {
        0
    }
}

#[cfg(test)]
mod tests {
    //! Checks spawn-lift discretization at its configured boundaries.

    use super::lift_steps;

    /// A zero or negative bound disables the lift scan.
    #[test]
    fn zero_or_negative_bound_has_no_steps() {
        assert_eq!(lift_steps(0.0), 0);
        assert_eq!(lift_steps(-0.01), 0);
    }

    /// A non-finite bound disables the lift scan.
    #[test]
    fn non_finite_bound_has_no_steps() {
        assert_eq!(lift_steps(f32::NAN), 0);
        assert_eq!(lift_steps(f32::INFINITY), 0);
    }

    /// A half-metre limit checks fifty centimetre positions.
    #[test]
    fn half_metre_bound_has_fifty_steps() {
        assert_eq!(lift_steps(0.5), 50);
    }

    /// Bounds beyond the addressable step count saturate.
    #[test]
    fn oversized_bound_saturates_at_the_u16_limit() {
        assert_eq!(lift_steps(f32::MAX), u16::MAX);
    }
}
