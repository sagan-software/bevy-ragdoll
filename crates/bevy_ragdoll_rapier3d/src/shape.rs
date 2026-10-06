//! Translate shared profile shapes into Rapier colliders.

use bevy::math::Quat;
use bevy_ragdoll::profile::ShapeSpec;
use bevy_rapier3d::prelude::Collider;

/// Converts a profile shape into a collider with the same local geometry.
pub(crate) fn collider_for_shape(shape: ShapeSpec) -> Collider {
    match shape {
        ShapeSpec::Capsule { a, b, radius } => Collider::capsule(a, b, radius),
        ShapeSpec::Sphere { center, radius } => {
            Collider::compound(vec![(center, Quat::IDENTITY, Collider::ball(radius))])
        }
        ShapeSpec::Cuboid {
            center,
            rotation,
            half_extents,
        } => Collider::compound(vec![(
            center,
            rotation,
            Collider::cuboid(half_extents.x, half_extents.y, half_extents.z),
        )]),
    }
}
