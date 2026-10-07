//! Private distance calculations used to derive rest-contact exclusions.

use bevy::math::{Isometry3d, Quat, Vec3};

use super::ShapeSpec;

/// Resting shapes closer than this surface gap do not collide.
pub(super) const REST_CONTACT_MARGIN: f32 = 0.01;

/// Shape geometry transformed into the shared skeleton rest frame.
#[derive(Clone, Copy)]
enum WorldShape {
    /// A sphere centre and radius.
    Sphere {
        /// The centre in skeleton space.
        center: Vec3,
        /// The sphere radius in metres.
        radius: f32,
    },
    /// Capsule cap centres and radius.
    Capsule {
        /// The first cap centre in skeleton space.
        a: Vec3,
        /// The second cap centre in skeleton space.
        b: Vec3,
        /// The capsule radius in metres.
        radius: f32,
    },
    /// An oriented box centre, rotation, and half extents.
    Cuboid {
        /// The centre in skeleton space.
        center: Vec3,
        /// The rotation from box-local to skeleton space.
        rotation: Quat,
        /// Half sizes along the local X, Y, and Z axes in metres.
        half_extents: Vec3,
    },
}

/// A cuboid represented by orthonormal world axes and positive extents.
#[derive(Clone, Copy)]
struct Obb {
    /// The centre in skeleton space.
    center: Vec3,
    /// The local X, Y, and Z directions in skeleton space.
    axes: [Vec3; 3],
    /// The orientation used to transform points into box-local space.
    rotation: Quat,
    /// The half size along each local axis.
    half_extents: Vec3,
}

/// Returns whether two body shapes are closer than the rest contact margin.
///
/// The method transforms each body-local shape into skeleton rest space and
/// compares their signed surface gap against the configured metre threshold.
pub(super) fn shapes_are_within_rest_contact_margin(
    a: &ShapeSpec,
    a_rest: Isometry3d,
    b: &ShapeSpec,
    b_rest: Isometry3d,
) -> bool {
    let a = world_shape(a, a_rest);
    let b = world_shape(b, b_rest);
    shape_gap(a, b) < REST_CONTACT_MARGIN
}

/// Transforms a body-local shape into skeleton rest space.
fn world_shape(shape: &ShapeSpec, rest: Isometry3d) -> WorldShape {
    match *shape {
        ShapeSpec::Sphere { center, radius } => WorldShape::Sphere {
            center: rest.transform_point(center).into(),
            radius,
        },
        ShapeSpec::Capsule { a, b, radius } => WorldShape::Capsule {
            a: rest.transform_point(a).into(),
            b: rest.transform_point(b).into(),
            radius,
        },
        ShapeSpec::Cuboid {
            center,
            rotation,
            half_extents,
        } => WorldShape::Cuboid {
            center: rest.transform_point(center).into(),
            rotation: rest.rotation * rotation,
            half_extents,
        },
    }
}

/// Returns the signed surface gap between two convex body shapes.
fn shape_gap(a: WorldShape, other: WorldShape) -> f32 {
    match a {
        WorldShape::Sphere { center, radius } => sphere_gap(center, radius, other),
        WorldShape::Capsule { a, b, radius } => capsule_gap(a, b, radius, other),
        WorldShape::Cuboid {
            center,
            rotation,
            half_extents,
        } => cuboid_gap(center, rotation, half_extents, other),
    }
}

/// Computes the signed surface gap from a sphere to another supported shape.
fn sphere_gap(center: Vec3, radius: f32, other: WorldShape) -> f32 {
    // Subtract both radii for spheres and one capsule radius for segments.
    match other {
        WorldShape::Sphere {
            center: other_center,
            radius: other_radius,
        } => center.distance(other_center) - radius - other_radius,
        WorldShape::Capsule {
            a,
            b,
            radius: other_radius,
        } => point_segment_distance(center, a, b) - radius - other_radius,
        WorldShape::Cuboid {
            center: box_center,
            rotation,
            half_extents,
        } => point_obb_distance(center, Obb::new(box_center, rotation, half_extents)) - radius,
    }
}

/// Computes the signed surface gap from a capsule to another supported shape.
fn capsule_gap(a: Vec3, b: Vec3, radius: f32, other: WorldShape) -> f32 {
    // Compare centre segments directly, then subtract their surface radii.
    match other {
        WorldShape::Sphere {
            center,
            radius: other_radius,
        } => point_segment_distance(center, a, b) - radius - other_radius,
        WorldShape::Capsule {
            a: other_a,
            b: other_b,
            radius: other_radius,
        } => segment_distance(a, b, other_a, other_b) - radius - other_radius,
        WorldShape::Cuboid {
            center,
            rotation,
            half_extents,
        } => segment_obb_distance(a, b, Obb::new(center, rotation, half_extents)) - radius,
    }
}

/// Computes the signed surface gap from an oriented cuboid to another shape.
fn cuboid_gap(center: Vec3, rotation: Quat, half_extents: Vec3, other: WorldShape) -> f32 {
    // Reuse point, segment, and oriented-box distance routines by shape pair.
    match other {
        WorldShape::Sphere {
            center: sphere_center,
            radius,
        } => point_obb_distance(sphere_center, Obb::new(center, rotation, half_extents)) - radius,
        WorldShape::Capsule { a, b, radius } => {
            segment_obb_distance(a, b, Obb::new(center, rotation, half_extents)) - radius
        }
        WorldShape::Cuboid {
            center: other_center,
            rotation: other_rotation,
            half_extents: other_extents,
        } => obb_distance(
            Obb::new(center, rotation, half_extents),
            Obb::new(other_center, other_rotation, other_extents),
        ),
    }
}

impl Obb {
    /// Creates orthonormal axes from a validated rotation.
    fn new(center: Vec3, rotation: Quat, half_extents: Vec3) -> Self {
        Self {
            center,
            axes: [rotation * Vec3::X, rotation * Vec3::Y, rotation * Vec3::Z],
            rotation,
            half_extents,
        }
    }

    /// Returns one of the cuboid's eight vertices.
    fn vertex(self, corner: usize) -> Vec3 {
        // Decode three corner bits into signs for the local half extents.
        let signs = Vec3::new(
            if corner & 1 == 0 { -1.0 } else { 1.0 },
            if corner & 2 == 0 { -1.0 } else { 1.0 },
            if corner & 4 == 0 { -1.0 } else { 1.0 },
        );
        // Pair each world axis with its matching half extent and corner sign.
        self.axes
            .iter()
            .zip(self.half_extents.to_array())
            .zip(signs.to_array())
            .fold(self.center, |vertex, ((axis, extent), sign)| {
                vertex + *axis * (extent * sign)
            })
    }
}

/// Computes Ericson's closest distance between two finite segments.
fn segment_distance(p0: Vec3, p1: Vec3, q0: Vec3, q1: Vec3) -> f32 {
    // Compute each segment direction and the vector between their starting points.
    let (d1, d2, r) = (p1 - p0, q1 - q0, p0 - q0);
    let (a, e, f) = (d1.length_squared(), d2.length_squared(), d2.dot(r));
    // Handle point segments before dividing by either squared segment length.
    let (s, t) = if a <= 1.0e-12 && e <= 1.0e-12 {
        (0.0, 0.0)
    } else if a <= 1.0e-12 {
        (0.0, (f / e).clamp(0.0, 1.0))
    } else {
        let c = d1.dot(r);
        if e <= 1.0e-12 {
            ((-c / a).clamp(0.0, 1.0), 0.0)
        } else {
            // Solve the unconstrained closest pair, using zero for parallel lines.
            let b = d1.dot(d2);
            let denominator = a * e - b * b;
            let mut s = if denominator > 1.0e-12 {
                ((b * f - c * e) / denominator).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let mut t = (b * s + f) / e;
            // Clamp the second parameter and recompute the first at each endpoint.
            if t < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if t > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            }
            (s, t)
        }
    };
    // Evaluate both closest points after the constrained parameters are settled.
    ((p0 + d1 * s) - (q0 + d2 * t)).length()
}

/// Returns the shortest distance from a point to a segment.
fn point_segment_distance(point: Vec3, a: Vec3, b: Vec3) -> f32 {
    let segment = b - a;
    let length_squared = segment.length_squared();
    // Degenerate segments use their first endpoint; other projections stay on the segment.
    let fraction = if length_squared <= 1.0e-12 {
        0.0
    } else {
        ((point - a).dot(segment) / length_squared).clamp(0.0, 1.0)
    };
    // The clamped interpolation point is the unique closest point on this segment.
    point.distance(a + segment * fraction)
}

/// Returns the shortest distance from a point to an oriented box.
fn point_obb_distance(point: Vec3, obb: Obb) -> f32 {
    let local = obb.rotation.inverse() * (point - obb.center);
    let closest = local.clamp(-obb.half_extents, obb.half_extents);
    local.distance(closest)
}

/// Returns the shortest distance from a segment to an oriented box.
fn segment_obb_distance(a: Vec3, b: Vec3, obb: Obb) -> f32 {
    let rotation = obb.rotation.inverse();
    segment_aabb_distance(
        rotation * (a - obb.center),
        rotation * (b - obb.center),
        obb.half_extents,
    )
}

/// Finds the exact closest segment-to-AABB distance over piecewise quadratic intervals.
fn segment_aabb_distance(a: Vec3, b: Vec3, half_extents: Vec3) -> f32 {
    let delta = b - a;
    // Endpoints plus six possible face crossings fit in this fixed array.
    let mut breakpoints = [
        0.0,
        1.0,
        f32::INFINITY,
        f32::INFINITY,
        f32::INFINITY,
        f32::INFINITY,
        f32::INFINITY,
        f32::INFINITY,
    ];
    // Store each strict interior face crossing after the two segment endpoints.
    let mut slots = breakpoints.iter_mut().skip(2);
    for ((start, change), extent) in a
        .to_array()
        .into_iter()
        .zip(delta.to_array())
        .zip(half_extents.to_array())
    {
        if change == 0.0 {
            continue;
        }
        for boundary in [-extent, extent] {
            let fraction = (boundary - start) / change;
            if 0.0 < fraction
                && fraction < 1.0
                && let Some(slot) = slots.next()
            {
                *slot = fraction;
            }
        }
    }
    // Sorting puts all finite breakpoints before the unused infinity sentinels.
    breakpoints.sort_by(f32::total_cmp);
    // Minimize independently on intervals where the active box faces do not change.
    let mut previous = None;
    let mut minimum_squared = f32::INFINITY;
    for high in breakpoints
        .into_iter()
        .take_while(|value| value.is_finite())
    {
        if let Some(low) = previous {
            minimum_squared = minimum_squared.min(segment_aabb_interval_distance(
                a,
                delta,
                half_extents,
                low,
                high,
            ));
        }
        previous = Some(high);
    }
    minimum_squared.sqrt()
}

/// Minimizes the segment-to-box distance over one face-stable interval.
fn segment_aabb_interval_distance(
    a: Vec3,
    delta: Vec3,
    half_extents: Vec3,
    low: f32,
    high: f32,
) -> f32 {
    // The midpoint identifies which box faces contribute throughout this interval.
    let middle = (low + high) * 0.5;
    let point = a + delta * middle;
    let mut denominator = 0.0;
    let mut numerator = 0.0;
    // On this interval, the active box faces stay constant along each axis.
    for ((start, change), (coordinate, extent)) in a
        .to_array()
        .into_iter()
        .zip(delta.to_array())
        .zip(point.to_array().into_iter().zip(half_extents.to_array()))
    {
        let boundary = if coordinate < -extent {
            Some(-extent)
        } else if coordinate > extent {
            Some(extent)
        } else {
            None
        };
        if let Some(boundary) = boundary {
            denominator += change * change;
            numerator += change * (start - boundary);
        }
    }
    // Accumulate the quadratic distance's coefficient and linear term.
    // Compare interval ends with its quadratic minimum, when active faces move.
    let closest = if denominator > 0.0 {
        (-numerator / denominator).clamp(low, high)
    } else {
        middle
    };
    [low, high, closest]
        .into_iter()
        .map(|fraction| point_aabb_distance_squared(a + delta * fraction, half_extents))
        .fold(f32::INFINITY, f32::min)
}

/// Returns squared distance from a point to an axis-aligned box at the origin.
fn point_aabb_distance_squared(point: Vec3, half_extents: Vec3) -> f32 {
    // Clamp each coordinate to the box and measure the squared residual vector.
    let closest = point.clamp(-half_extents, half_extents);
    point.distance_squared(closest)
}

/// Returns the surface distance between two oriented cuboids.
fn obb_distance(a: Obb, b: Obb) -> f32 {
    // Intersecting boxes have zero surface separation and need no closest-feature search.
    if is_obb_intersection(a, b) {
        return 0.0;
    }

    // Compare all vertices and edges because skewed boxes can be closest between edges.
    // Check vertices against opposite boxes before comparing every edge pair.
    let mut minimum = f32::INFINITY;
    for corner in 0..8 {
        minimum = minimum.min(point_obb_distance(a.vertex(corner), b));
        minimum = minimum.min(point_obb_distance(b.vertex(corner), a));
    }
    let a_edges = obb_edges(a);
    let b_edges = obb_edges(b);
    for (a0, a1) in a_edges {
        for (b0, b1) in b_edges {
            minimum = minimum.min(segment_distance(a0, a1, b0, b1));
        }
    }
    minimum
}

/// Returns whether two oriented cuboids overlap on any separating axis.
fn is_obb_intersection(a: Obb, b: Obb) -> bool {
    let delta = b.center - a.center;
    // Face normals can separate the boxes without any edge-axis tests.
    for axis in a.axes.into_iter().chain(b.axes) {
        if is_separating_axis(a, b, delta, axis) {
            return false;
        }
    }
    // Cross products add every nonparallel edge-axis candidate from the SAT theorem.
    for a_axis in a.axes {
        for b_axis in b.axes {
            let axis = a_axis.cross(b_axis);
            if is_separating_axis(a, b, delta, axis) {
                return false;
            }
        }
    }
    // No separating candidate means the oriented boxes overlap.
    true
}

/// Tests one nonzero separating axis for two oriented cuboids.
fn is_separating_axis(a: Obb, b: Obb, delta: Vec3, axis: Vec3) -> bool {
    // Cross products can collapse for parallel box edges and add no separating plane.
    let length_squared = axis.length_squared();
    if length_squared <= 1.0e-12 {
        return false;
    }
    let unit_axis = axis / length_squared.sqrt();
    // Compare centre separation with each cuboid's projected half-width.
    let projection = delta.dot(unit_axis).abs();
    let radius_a = projected_radius(a, unit_axis);
    let radius_b = projected_radius(b, unit_axis);
    projection > radius_a + radius_b
}

/// Projects an oriented cuboid's half extents onto a unit axis.
fn projected_radius(obb: Obb, axis: Vec3) -> f32 {
    // Sum each half extent weighted by its axis alignment with the projection.
    obb.half_extents
        .to_array()
        .into_iter()
        .zip(obb.axes)
        .map(|(extent, direction)| extent * direction.dot(axis).abs())
        .sum()
}

/// Returns the twelve cuboid edges in deterministic corner and axis order.
fn obb_edges(obb: Obb) -> [(Vec3, Vec3); 12] {
    let mut edges = [(Vec3::ZERO, Vec3::ZERO); 12];
    let mut edge_slots = edges.iter_mut();
    // Visit corners and bit axes in stable order, emitting only one direction per edge.
    for corner in 0..8 {
        for axis in [1, 2, 4] {
            if corner & axis == 0
                && let Some(edge) = edge_slots.next()
            {
                *edge = (obb.vertex(corner), obb.vertex(corner | axis));
            }
        }
    }
    // The fixed array matches the twelve unique edges of a cuboid.
    edges
}

#[cfg(test)]
mod tests;
