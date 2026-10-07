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
    let errors = [
        shape_gap(sphere, sphere) + 1.0,
        shape_gap(capsule, capsule) + 0.5,
        shape_gap(sphere, capsule) - shape_gap(capsule, sphere),
        shape_gap(sphere, cuboid) - shape_gap(cuboid, sphere),
        shape_gap(capsule, cuboid) - shape_gap(cuboid, capsule),
        shape_gap(cuboid, cuboid),
    ];
    assert!(
        errors.iter().all(|error| error.abs() < 1.0e-6),
        "{errors:?}"
    );
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
    let within = [
        (&sphere, &sphere, separated),
        (&sphere, &sphere, touching),
        (&capsule, &capsule, capsule_touching),
        (&sphere, &cuboid, touching),
        (&capsule, &cuboid, Isometry3d::IDENTITY),
        (&cuboid, &cuboid, touching),
    ]
    .map(|(first, second, pose)| {
        shapes_are_within_rest_contact_margin(first, Isometry3d::IDENTITY, second, pose)
    });
    assert_eq!(within, [false, true, true, true, true, true]);
}

/// Covers point-point, point-segment, parallel, interior, and endpoint cases.
#[test]
fn segment_distance_handles_degenerate_parallel_and_clamped_segments() {
    let origin = Vec3::ZERO;
    let cases = [
        // Degenerate segments collapse to points.
        ([origin, origin, Vec3::X, Vec3::X], 1.0),
        ([origin, origin, Vec3::X, Vec3::Y], 0.5_f32.sqrt()),
        ([origin, Vec3::X, Vec3::Y, Vec3::Y], 1.0),
        ([origin, Vec3::X, Vec3::Y, Vec3::X + Vec3::Y], 1.0),
        // Parallel, crossing, and endpoint-clamped segment pairs.
        ([origin, Vec3::X, Vec3::Y, Vec3::Y + Vec3::X], 1.0),
        (
            [
                origin,
                Vec3::X,
                Vec3::new(0.5, -1.0, 0.0),
                Vec3::new(0.5, 1.0, 0.0),
            ],
            0.0,
        ),
        (
            [
                origin,
                Vec3::X,
                Vec3::new(2.0, 1.0, 0.0),
                Vec3::new(2.0, 2.0, 0.0),
            ],
            2.0_f32.sqrt(),
        ),
        (
            [
                origin,
                Vec3::X,
                Vec3::new(2.0, -2.0, 0.0),
                Vec3::new(2.0, -1.0, 0.0),
            ],
            2.0_f32.sqrt(),
        ),
    ];
    // Collect every pair whose distance differs from the expected value.
    let mismatches = cases
        .into_iter()
        .filter(|([p0, p1, q0, q1], expected)| {
            (segment_distance(*p0, *p1, *q0, *q1) - expected).abs() >= 1.0e-6
        })
        .collect::<Vec<_>>();
    assert_eq!(mismatches, []);
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
    let extents = Vec3::ONE;
    // Degenerate segments inside and outside the box, then segments that
    // pierce, run past, or stay beyond a corner of the box.
    let segments = [
        (Vec3::ZERO, Vec3::ZERO),
        (Vec3::new(2.0, 2.0, 0.0), Vec3::new(2.0, 2.0, 0.0)),
        (Vec3::new(-2.0, 0.0, 0.0), Vec3::new(2.0, 0.0, 0.0)),
        (Vec3::new(-2.0, 2.0, 0.0), Vec3::new(2.0, 2.0, 0.0)),
        (Vec3::new(2.0, 2.0, 2.0), Vec3::new(3.0, 3.0, 3.0)),
    ]
    .map(|(start, end)| segment_aabb_distance(start, end, extents));
    assert_eq!(segments, [0.0, 2.0_f32.sqrt(), 0.0, 1.0, 3.0_f32.sqrt()]);
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

/// Covers box intersection and face separation for axis-aligned boxes.
#[test]
fn box_distance_covers_touching_and_separated_boxes() {
    // Touching and separated axis-aligned boxes, and the twelve box edges.
    let unit = obb(Vec3::ZERO, Vec3::ONE);
    let distances = [Vec3::X, Vec3::new(3.0, 0.0, 0.0)]
        .map(|center| obb_distance(unit, obb(center, Vec3::ONE)));
    assert_eq!((distances, obb_edges(unit).len()), ([0.0, 1.0], 12));
    // A zero axis never separates, and a face axis between distant boxes does.
    let separating = [
        (Vec3::ZERO, Vec3::ZERO),
        (Vec3::new(4.0, 0.0, 0.0), Vec3::X),
    ]
    .map(|(delta, axis)| is_separating_axis(unit, obb(delta, Vec3::ONE), delta, axis));
    assert_eq!(separating, [false, true]);
}

/// Covers the exact edge-to-edge distance of thin diagonal boxes.
#[test]
fn box_distance_covers_edge_pair_separation() {
    // Thin diagonal boxes that only an edge-pair axis separates.
    let thin = Vec3::new(2.0, 0.1, 0.1);
    let diagonal_a = Obb::new(Vec3::ZERO, Quat::from_rotation_z(FRAC_PI_4), thin);
    let diagonal_b = Obb::new(
        Vec3::new(0.0, 3.0, 0.3),
        Quat::from_rotation_y(FRAC_PI_4) * Quat::from_rotation_z(-FRAC_PI_4),
        thin,
    );
    assert!(obb_distance(diagonal_a, diagonal_b) > 0.0);
}

/// Returns whether only an edge cross-product axis separates `first` and `second`.
fn is_edge_only_separated(first: Obb, second: Obb) -> bool {
    let delta = second.center - first.center;
    // Poses that a face axis already separates do not exercise edge axes.
    let is_face_separated = first
        .axes
        .into_iter()
        .chain(second.axes)
        .any(|axis| is_separating_axis(first, second, delta, axis));
    !is_face_separated
        && first.axes.into_iter().any(|first_axis| {
            second.axes.into_iter().any(|second_axis| {
                is_separating_axis(first, second, delta, first_axis.cross(second_axis))
            })
        })
}

/// Finds a deterministic edge-axis separating case that face axes cannot detect.
#[test]
fn box_sat_checks_cross_product_axes() {
    // Sweep a second thin box through a grid of rotations and offsets.
    let extents = Vec3::new(2.0, 0.2, 0.15);
    let first = obb(Vec3::ZERO, extents);
    let offsets = [-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0];
    let angles = [-1.1, -FRAC_PI_4, -0.31, 0.31, FRAC_PI_4, 1.1];
    // Compose the three angles so every axis pairing gets exercised.
    let rotations = angles.into_iter().flat_map(|x| {
        angles.into_iter().flat_map(move |y| {
            angles.into_iter().map(move |z| {
                Quat::from_rotation_x(x) * Quat::from_rotation_y(y) * Quat::from_rotation_z(z)
            })
        })
    });
    let centers = offsets.into_iter().flat_map(|x| {
        offsets
            .into_iter()
            .flat_map(move |y| offsets.into_iter().map(move |z| Vec3::new(x, y, z)))
    });
    let centers = centers.collect::<Vec<_>>();
    // The first pose that only an edge cross-product separates must not intersect.
    let found = rotations
        .flat_map(|rotation| {
            centers
                .iter()
                .map(move |center| Obb::new(*center, rotation, extents))
        })
        .find(|second| is_edge_only_separated(first, *second));
    let second = found.expect("the deterministic box grid contains an edge-axis separation");
    assert!(!is_obb_intersection(first, second));
}
