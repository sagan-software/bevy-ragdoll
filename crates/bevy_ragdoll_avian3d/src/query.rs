//! Read Avian results back into core body state and answer raycasts.

use avian3d::prelude::{
    AngularVelocity, ColliderOf, Collisions, ContactPair, LinearVelocity, Position, RigidBody,
    Rotation, Sleeping, SpatialQuery, SpatialQueryFilter,
};
use bevy::math::{Dir3, Isometry3d, Vec3};
use bevy::prelude::{Commands, Entity, Has, MessageReader, MessageWriter, Query, With};
use bevy_ragdoll::runtime::backend::{BodyContact, BodyContacts, RayHit};
use bevy_ragdoll::runtime::body::{
    BodyAtRest, BodyDriveOutput, BodyKind, BodyPhysicsPose, BodyVelocity,
};
use bevy_ragdoll::runtime::components::RagdollBodyOf;
use bevy_ragdoll::runtime::messages::{RagdollRaycast, RagdollRaycastResponse};

/// Copies Avian's completed position and rotation into each core body pose.
pub(crate) fn read_body_pose(
    mut bodies: Query<
        '_,
        '_,
        (&Position, &Rotation, &BodyKind, &mut BodyPhysicsPose),
        With<RagdollBodyOf>,
    >,
) {
    for (position, rotation, kind, mut pose) in &mut bodies {
        pose.previous = pose.current;
        // Frozen bodies keep their pose exactly; Avian's kinematic writeback adds round-off.
        if *kind != BodyKind::Fixed {
            pose.current = Isometry3d::new(position.0, rotation.0);
        }
    }
}

/// Avian motion state and core readback components for each ragdoll body.
type BodyMotionQuery<'w, 's> = Query<
    'w,
    's,
    (
        Entity,
        &'static BodyKind,
        &'static mut LinearVelocity,
        &'static mut AngularVelocity,
        Has<Sleeping>,
        &'static BodyDriveOutput,
        &'static mut BodyVelocity,
    ),
    With<RagdollBodyOf>,
>;

/// Copies Avian velocities and sleep state into core components and applies
/// the drive's speed caps back to Avian.
pub(crate) fn read_body_motion(
    mut commands: Commands<'_, '_>,
    mut bodies: BodyMotionQuery<'_, '_>,
) {
    for (entity, kind, mut linear, mut angular, is_sleeping, drive, mut velocity) in &mut bodies {
        // Frozen bodies are kinematic in Avian; the joint solver can leave them a
        // small velocity, so clear it to keep them still.
        if *kind == BodyKind::Fixed {
            linear.0 = Vec3::ZERO;
            angular.0 = Vec3::ZERO;
        }
        velocity.linear = linear.0;
        velocity.angular = angular.0;
        if drive.max_linear_speed.is_some() || drive.max_angular_speed.is_some() {
            drive.clamp_velocity(&mut velocity);
            linear.0 = velocity.linear;
            angular.0 = velocity.angular;
        }
        if is_sleeping {
            commands.entity(entity).insert(BodyAtRest);
        } else {
            commands.entity(entity).remove::<BodyAtRest>();
        }
    }
}

/// Replaces body contacts and publishes one response per consumed raycast.
pub(crate) fn read_ragdoll_queries(
    collisions: Collisions<'_>,
    spatial_query: SpatialQuery<'_, '_>,
    mut bodies: Query<'_, '_, (Entity, &mut BodyContacts)>,
    body_tags: Query<'_, '_, &RagdollBodyOf>,
    collider_bodies: Query<'_, '_, &ColliderOf>,
    rigid_bodies: Query<'_, '_, &RigidBody>,
    mut raycasts: MessageReader<'_, '_, RagdollRaycast>,
    mut responses: MessageWriter<'_, RagdollRaycastResponse>,
) {
    for (entity, mut contacts) in &mut bodies {
        contacts.0.clear();
        for pair in collisions.collisions_with(entity) {
            append_pair_contacts(entity, pair, &rigid_bodies, &mut contacts.0);
        }
    }
    for request in raycasts.read() {
        let hit = raycast_hit(&spatial_query, request, &body_tags, &collider_bodies);
        responses.write(RagdollRaycastResponse {
            request_id: request.request_id,
            hit,
        });
    }
}

/// Appends the touching points of one pair with normals pointing away from the other collider.
fn append_pair_contacts(
    entity: Entity,
    pair: &ContactPair,
    rigid_bodies: &Query<'_, '_, &RigidBody>,
    contacts: &mut Vec<BodyContact>,
) {
    if !pair.is_touching() {
        return;
    }
    let (other, other_body, body_is_first) = if pair.collider1 == entity {
        (pair.collider2, pair.body2, true)
    } else {
        (pair.collider1, pair.body1, false)
    };
    let other_is_static = other_body
        .and_then(|body| rigid_bodies.get(body).ok())
        .is_none_or(RigidBody::is_static);
    for manifold in &pair.manifolds {
        let normal = contact_normal(manifold.normal, body_is_first);
        contacts.extend(manifold.points.iter().map(|point| BodyContact {
            point: point.point,
            normal,
            other,
            other_is_static,
        }));
    }
}

/// Directs a manifold normal (first shape to second) away from the other collider.
fn contact_normal(pair_normal: Vec3, body_is_first: bool) -> Vec3 {
    if body_is_first {
        -pair_normal
    } else {
        pair_normal
    }
}

/// Validates a request and returns its nearest filtered hit.
fn raycast_hit(
    spatial_query: &SpatialQuery<'_, '_>,
    request: &RagdollRaycast,
    body_tags: &Query<'_, '_, &RagdollBodyOf>,
    collider_bodies: &Query<'_, '_, &ColliderOf>,
) -> Option<RayHit> {
    if !request.origin.is_finite()
        || !request.max_distance.is_finite()
        || request.max_distance <= 0.0
    {
        return None;
    }
    let direction = Dir3::new(request.direction).ok()?;
    let body_of = |collider: Entity| {
        collider_bodies
            .get(collider)
            .map_or(collider, |link| link.body)
    };
    // Exclude the requested body and every body owned by a requested character.
    let accepts = |collider: Entity| {
        let body = body_of(collider);
        match (request.filter, body_tags.get(body)) {
            (Some(excluded), _) if body == excluded => false,
            (Some(excluded), Ok(owner)) if owner.0 == excluded => false,
            _ => true,
        }
    };
    let hit = spatial_query.cast_ray_predicate(
        request.origin,
        direction,
        request.max_distance,
        true,
        &SpatialQueryFilter::default(),
        &accepts,
    )?;
    let body = body_of(hit.entity);
    Some(RayHit {
        entity: hit.entity,
        body: body_tags.contains(body).then_some(body),
        point: request.origin + direction * hit.distance,
        normal: hit.normal.normalize_or_zero(),
        distance: hit.distance,
    })
}

#[cfg(test)]
mod tests {
    //! Checks contact-normal orientation at each collider endpoint.

    use bevy::math::Vec3;

    use super::contact_normal;

    /// Each body receives a normal pointing away from its contact partner.
    #[test]
    fn contact_normal_flips_for_the_first_endpoint() {
        assert_eq!(contact_normal(Vec3::Y, true), -Vec3::Y);
        assert_eq!(contact_normal(Vec3::Y, false), Vec3::Y);
    }
}
