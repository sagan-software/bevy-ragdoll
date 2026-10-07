//! Tests for private rest-contact geometry.

use std::f32::consts::FRAC_PI_4;

use super::*;

/// Creates a unit-oriented box at the supplied centre and half extents.
fn obb(center: Vec3, half_extents: Vec3) -> Obb {
    Obb::new(center, Quat::IDENTITY, half_extents)
}

/// Checks every convex shape pairing and both symmetric argument orders.
#[test]
fn shape_gap_covers_every_pairing() {
    // One of each world shape; the cuboid sits apart from the other two.
    let sphere = WorldShape::Sphere {
        center: Vec3::ZERO,
        radius: 0.5,
    };
    let capsule = WorldShape::Capsule {
        a: Vec3::ZERO,
        b: Vec3::Y,
        radius: 0.25,
    };
    let cuboid = WorldShape::Cuboid {
        center: Vec3::new(3.0, 0.0, 0.0),
        rotation: Quat::IDENTITY,
        half_extents: Vec3::splat(0.5),
    };

    // Self-overlap is negative by the summed radii, and every pairing is symmetric.
    assert!((shape_gap(sphere, sphere) + 1.0).abs() < 1.0e-6);
    assert!((shape_gap(capsule, capsule) + 0.5).abs() < 1.0e-6);
    assert_eq!(shape_gap(sphere, capsule), shape_gap(capsule, sphere));
    assert_eq!(shape_gap(sphere, cuboid), shape_gap(cuboid, sphere));
    assert_eq!(shape_gap(capsule, cuboid), shape_gap(cuboid, capsule));
    assert!(shape_gap(cuboid, cuboid).abs() < 1.0e-6);
}

/// Transforms each shape kind and applies the strict surface-gap margin.
#[test]
fn world_shapes_transform_and_compare_at_the_margin() {
    // Local shapes placed by rest poses before the margin test.
    let sphere = ShapeSpec::Sphere {
        center: Vec3::ZERO,
        radius: 0.5,
    };
    let capsule = ShapeSpec::Capsule {
        a: Vec3::ZERO,
        b: Vec3::Y,
        radius: 0.25,
    };
    let cuboid = ShapeSpec::Cuboid {
        center: Vec3::ZERO,
        rotation: Quat::from_rotation_z(FRAC_PI_4),
        half_extents: Vec3::splat(0.5),
    };
    // Poses that separate, just touch, or overlap the shapes.
    let separated = Isometry3d::from_xyz(2.0, 0.0, 0.0);
    let touching = Isometry3d::from_xyz(1.0, 0.0, 0.0);
    let capsule_touching = Isometry3d::from_xyz(0.5, 0.0, 0.0);
    // Separated spheres are outside the margin; touching pairs of every kind are inside.
    assert!(!shapes_are_within_rest_contact_margin(
        &sphere,
        Isometry3d::IDENTITY,
        &sphere,
        separated
    ));
    assert!(shapes_are_within_rest_contact_margin(
        &sphere,
        Isometry3d::IDENTITY,
        &sphere,
        touching
    ));
    assert!(shapes_are_within_rest_contact_margin(
        &capsule,
        Isometry3d::IDENTITY,
        &capsule,
        capsule_touching
    ));
    assert!(shapes_are_within_rest_contact_margin(
        &sphere,
        Isometry3d::IDENTITY,
        &cuboid,
        touching
    ));
    assert!(shapes_are_within_rest_contact_margin(
        &capsule,
        Isometry3d::IDENTITY,
        &cuboid,
        Isometry3d::IDENTITY
    ));
    assert!(shapes_are_within_rest_contact_margin(
        &cuboid,
        Isometry3d::IDENTITY,
        &cuboid,
        touching
    ));
}

/// Covers point-point, point-segment, parallel, interior, and endpoint cases.
#[test]
fn segment_distance_handles_degenerate_parallel_and_clamped_segments() {
    // Degenerate segments collapse to points.
    let origin = Vec3::ZERO;
    assert_eq!(segment_distance(origin, origin, Vec3::X, Vec3::X), 1.0);
    assert!((segment_distance(origin, origin, Vec3::X, Vec3::Y) - 0.5_f32.sqrt()).abs() < 1e-6);
    assert_eq!(segment_distance(origin, Vec3::X, Vec3::Y, Vec3::Y), 1.0);
    assert!((segment_distance(origin, Vec3::X, Vec3::Y, Vec3::X + Vec3::Y) - 1.0).abs() < 1e-6);
    // Parallel, crossing, and endpoint-clamped segment pairs.
    assert_eq!(
        segment_distance(origin, Vec3::X, Vec3::Y, Vec3::Y + Vec3::X),
        1.0
    );
    assert_eq!(
        segment_distance(
            origin,
            Vec3::X,
            Vec3::new(0.5, -1.0, 0.0),
            Vec3::new(0.5, 1.0, 0.0)
        ),
        0.0
    );
    assert_eq!(
        segment_distance(
            origin,
            Vec3::X,
            Vec3::new(2.0, 1.0, 0.0),
            Vec3::new(2.0, 2.0, 0.0)
        ),
        2.0_f32.sqrt()
    );
    assert_eq!(
        segment_distance(
            origin,
            Vec3::X,
            Vec3::new(2.0, -2.0, 0.0),
            Vec3::new(2.0, -1.0, 0.0)
        ),
        2.0_f32.sqrt()
    );
}

/// Covers degenerate and clamped point-to-segment projections.
#[test]
fn point_segment_distance_handles_degenerate_and_endpoints() {
    // A degenerate segment and points nearest each endpoint and the interior.
    assert_eq!(point_segment_distance(Vec3::X, Vec3::ZERO, Vec3::ZERO), 1.0);
    assert_eq!(
        [
            Vec3::new(-1.0, 1.0, 0.0),
            Vec3::new(2.0, 1.0, 0.0),
            Vec3::new(0.5, 1.0, 0.0),
        ]
        .map(|point| point_segment_distance(point, Vec3::ZERO, Vec3::X)),
        [2.0_f32.sqrt(), 2.0_f32.sqrt(), 1.0]
    );
}

/// Covers points inside, on, and outside each box region.
#[test]
fn point_box_distance_clamps_to_box_surface() {
    // Points inside, on the surface, and outside the box.
    let box_shape = obb(Vec3::ZERO, Vec3::ONE);
    assert_eq!(
        [Vec3::ZERO, Vec3::X, Vec3::new(2.0, 0.0, 0.0)]
            .map(|point| point_obb_distance(point, box_shape)),
        [0.0, 0.0, 1.0]
    );
    // The squared AABB distance sums the per-axis excess.
    assert_eq!(
        point_aabb_distance_squared(Vec3::new(2.0, 3.0, 0.0), Vec3::ONE),
        5.0
    );
}

/// Covers stationary, interior-crossing, and outside segment-to-box cases.
#[test]
fn segment_box_distance_splits_at_all_faces() {
    // A degenerate segment inside and outside the box.
    let extents = Vec3::ONE;
    assert_eq!(segment_aabb_distance(Vec3::ZERO, Vec3::ZERO, extents), 0.0);
    assert_eq!(
        segment_aabb_distance(Vec3::new(2.0, 2.0, 0.0), Vec3::new(2.0, 2.0, 0.0), extents),
        2.0_f32.sqrt()
    );
    // Segments that pierce, run past, or stay beyond a corner of the box.
    assert_eq!(
        segment_aabb_distance(Vec3::new(-2.0, 0.0, 0.0), Vec3::new(2.0, 0.0, 0.0), extents),
        0.0
    );
    assert_eq!(
        segment_aabb_distance(Vec3::new(-2.0, 2.0, 0.0), Vec3::new(2.0, 2.0, 0.0), extents),
        1.0
    );
    assert_eq!(
        segment_aabb_distance(Vec3::new(2.0, 2.0, 2.0), Vec3::new(3.0, 3.0, 3.0), extents),
        3.0_f32.sqrt()
    );
    // An oriented box gives the same answer through the OBB entry point.
    assert_eq!(
        segment_obb_distance(
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(2.0, 1.0, 0.0),
            obb(Vec3::ZERO, extents)
        ),
        1.0
    );
}

/// Covers box intersection, face separation, and exact edge-to-edge distance.
#[test]
fn box_distance_covers_sat_and_feature_distances() {
    // Touching and separated axis-aligned boxes.
    let unit = obb(Vec3::ZERO, Vec3::ONE);
    assert_eq!(obb_distance(unit, obb(Vec3::X, Vec3::ONE)), 0.0);
    assert_eq!(
        obb_distance(unit, obb(Vec3::new(3.0, 0.0, 0.0), Vec3::ONE)),
        1.0
    );
    // Thin diagonal boxes that only an edge-pair axis separates.
    let thin = Vec3::new(2.0, 0.1, 0.1);
    let diagonal_a = Obb::new(Vec3::ZERO, Quat::from_rotation_z(FRAC_PI_4), thin);
    let diagonal_b = Obb::new(
        Vec3::new(0.0, 3.0, 0.3),
        Quat::from_rotation_y(FRAC_PI_4) * Quat::from_rotation_z(-FRAC_PI_4),
        thin,
    );
    assert!(obb_distance(diagonal_a, diagonal_b) > 0.0);
    // Edge enumeration and the separating-axis test itself.
    assert_eq!(obb_edges(unit).len(), 12);
    assert!(!is_separating_axis(unit, unit, Vec3::ZERO, Vec3::ZERO));
    assert!(is_separating_axis(
        unit,
        obb(Vec3::new(4.0, 0.0, 0.0), Vec3::ONE),
        Vec3::new(4.0, 0.0, 0.0),
        Vec3::X
    ));
}

/// Finds a deterministic edge-axis separating case that face axes cannot detect.
#[test]
fn box_sat_checks_cross_product_axes() {
    // Sweep a second thin box through a grid of rotations and offsets.
    let first = obb(Vec3::ZERO, Vec3::new(2.0, 0.2, 0.15));
    let offsets = [-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0];
    let angles = [-1.1, -FRAC_PI_4, -0.31, 0.31, FRAC_PI_4, 1.1];
    for x_angle in angles {
        for y_angle in angles {
            for z_angle in angles {
                let rotation = Quat::from_rotation_x(x_angle)
                    * Quat::from_rotation_y(y_angle)
                    * Quat::from_rotation_z(z_angle);
                for x in offsets {
                    for y in offsets {
                        for z in offsets {
                            let second =
                                Obb::new(Vec3::new(x, y, z), rotation, Vec3::new(2.0, 0.2, 0.15));
                            // Skip poses that a face axis already separates.
                            let delta = second.center - first.center;
                            let is_face_separated = first
                                .axes
                                .into_iter()
                                .chain(second.axes)
                                .any(|axis| is_separating_axis(first, second, delta, axis));
                            if is_face_separated {
                                continue;
                            }
                            // The first pose that only an edge cross-product separates must not intersect.
                            let is_edge_separated = first.axes.into_iter().any(|first_axis| {
                                second.axes.into_iter().any(|second_axis| {
                                    is_separating_axis(
                                        first,
                                        second,
                                        delta,
                                        first_axis.cross(second_axis),
                                    )
                                })
                            });
                            if is_edge_separated {
                                assert!(!is_obb_intersection(first, second));
                                return;
                            }
                        }
                    }
                }
            }
        }
    }
    // The grid is chosen to contain such a pose, so reaching here is a fixture bug.
    panic!("the deterministic box grid contains an edge-axis separation");
}
