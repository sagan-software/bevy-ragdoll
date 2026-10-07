//! Translate shared profile shapes into Avian colliders.

use avian3d::prelude::Collider;
use bevy::math::Quat;
use bevy_ragdoll::profile::ShapeSpec;

/// Converts a profile shape into a collider with the same local geometry.
///
/// Avian cuboids take full side lengths, so profile half extents are doubled.
/// Offset spheres and cuboids use one-shape compounds to keep their local pose.
///
/// # Examples
///
/// ```
/// use bevy::math::Vec3;
/// use bevy_ragdoll::profile::ShapeSpec;
/// use bevy_ragdoll_avian3d::collider_for_shape;
///
/// let shape = ShapeSpec::Sphere {
///     center: Vec3::ZERO,
///     radius: 0.1,
/// };
/// let collider = collider_for_shape(shape);
/// assert!(collider.shape().as_compound().is_some());
/// ```
#[must_use]
pub fn collider_for_shape(shape: ShapeSpec) -> Collider {
    match shape {
        ShapeSpec::Capsule { a, b, radius } => Collider::capsule_endpoints(radius, a, b),
        ShapeSpec::Sphere { center, radius } => {
            Collider::compound(vec![(center, Quat::IDENTITY, Collider::sphere(radius))])
        }
        ShapeSpec::Cuboid {
            center,
            rotation,
            half_extents,
        } => {
            let lengths = half_extents * 2.0;
            Collider::compound(vec![(
                center,
                rotation,
                Collider::cuboid(lengths.x, lengths.y, lengths.z),
            )])
        }
    }
}

#[cfg(test)]
mod tests {
    //! Checks that every profile shape keeps its local geometry in Avian.

    use bevy::math::{Quat, Vec3};
    use bevy_ragdoll::profile::ShapeSpec;

    use super::collider_for_shape;

    /// Capsules keep both endpoints and the radius.
    #[test]
    fn capsule_keeps_endpoints_and_radius() {
        let a = Vec3::new(0.0, 0.1, 0.0);
        let b = Vec3::new(0.0, 0.9, 0.0);
        // Use a vertical capsule whose endpoints differ from the origin.
        let collider = collider_for_shape(ShapeSpec::Capsule { a, b, radius: 0.2 });
        let capsule = collider
            .shape()
            .as_capsule()
            .expect("profile capsules map to capsules");

        assert_eq!(capsule.segment.a, a);
        assert_eq!(capsule.segment.b, b);
        assert_eq!(capsule.radius, 0.2);
    }

    /// Sphere compounds keep their offset and radius.
    #[test]
    fn sphere_keeps_offset_and_radius() {
        let center = Vec3::new(0.1, 0.2, 0.3);
        // Offset the sphere so the compound must carry its pose.
        let collider = collider_for_shape(ShapeSpec::Sphere {
            center,
            radius: 0.4,
        });
        // The compound holds exactly one ball child.
        let compound = collider
            .shape()
            .as_compound()
            .expect("spheres use compounds");
        let [(pose, child)] = compound.shapes() else {
            panic!("a profile sphere has one child shape");
        };

        assert_eq!(pose.translation, center);
        assert_eq!(child.as_ball().expect("child is a ball").radius, 0.4);
    }

    /// Cuboid compounds keep their pose and convert half extents to lengths.
    #[test]
    fn cuboid_keeps_pose_and_half_extents() {
        let center = Vec3::new(0.1, 0.2, 0.3);
        let rotation = Quat::from_rotation_y(0.25);
        let half_extents = Vec3::new(0.4, 0.5, 0.6);
        // Offset and rotate the cuboid so the compound must carry both.
        let collider = collider_for_shape(ShapeSpec::Cuboid {
            center,
            rotation,
            half_extents,
        });
        // The compound holds exactly one cuboid child.
        let compound = collider
            .shape()
            .as_compound()
            .expect("cuboids use compounds");
        let [(pose, child)] = compound.shapes() else {
            panic!("a profile cuboid has one child shape");
        };
        let cuboid = child.as_cuboid().expect("child is a cuboid");

        // Avian stores half extents, so they match the profile values.
        assert_eq!(pose.translation, center);
        assert!(pose.rotation.angle_between(rotation) < 1.0e-6);
        assert!((cuboid.half_extents - half_extents).length() < 1.0e-6);
    }
}
