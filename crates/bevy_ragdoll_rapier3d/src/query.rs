//! Answer backend-neutral raycast and contact requests from Rapier contexts.

use bevy::math::{Isometry3d, Vec3};
use bevy::prelude::{Commands, Entity, MessageReader, MessageWriter, Query};
use bevy_ragdoll::profile::BodyIndex;
use bevy_ragdoll::runtime::backend::{BodyContact, BodyContacts, RayHit};
use bevy_ragdoll::runtime::body::{BodyAtRest, BodyPhysicsPose, BodyVelocity};
use bevy_ragdoll::runtime::components::RagdollBodyOf;
use bevy_ragdoll::runtime::messages::{RagdollRaycast, RagdollRaycastResponse};
use bevy_rapier3d::pipeline::{QueryFilter, QueryFilterFlags};
use bevy_rapier3d::plugin::{ContactPairView, ReadRapierContext};
use bevy_rapier3d::prelude::{RigidBody, Sleeping, Velocity};

/// Copies Rapier's completed transform, velocity, and sleep state into core components.
pub(crate) fn read_body_state(
    mut commands: Commands<'_, '_>,
    mut bodies: Query<
        '_,
        '_,
        (
            Entity,
            &bevy::prelude::Transform,
            &Velocity,
            &Sleeping,
            &mut BodyPhysicsPose,
            &mut BodyVelocity,
        ),
    >,
) {
    // Preserve the previous completed pose before recording Rapier's new writeback.
    for (entity, transform, velocity, sleeping, mut pose, mut body_velocity) in &mut bodies {
        pose.previous = pose.current;
        pose.current = Isometry3d::new(transform.translation, transform.rotation);
        body_velocity.linear = velocity.linear;
        body_velocity.angular = velocity.angular;
        // Mirror Rapier sleep state into the backend-neutral runtime marker.
        if sleeping.sleeping {
            commands.entity(entity).insert(BodyAtRest);
        } else {
            commands.entity(entity).remove::<BodyAtRest>();
        }
    }
}

/// Replaces body contacts and publishes one response for each consumed raycast.
pub(crate) fn read_ragdoll_queries(
    context_param: ReadRapierContext<'_, '_>,
    mut bodies: Query<'_, '_, (Entity, &mut BodyContacts)>,
    body_tags: Query<'_, '_, (&RagdollBodyOf, &BodyIndex)>,
    rigid_bodies: Query<'_, '_, &RigidBody>,
    mut raycasts: MessageReader<'_, '_, RagdollRaycast>,
    mut responses: MessageWriter<'_, RagdollRaycastResponse>,
) {
    // An unavailable Rapier context clears contact caches and yields raycast misses.
    let context = context_param.single().ok();
    // Clear each cache before appending the current fixed-step narrow-phase contacts.
    for (entity, mut contacts) in &mut bodies {
        contacts.0.clear();
        let Some(context) = context.as_ref() else {
            continue;
        };
        // Rapier returns contacts with this collider as either pair endpoint.
        for pair in context.contact_pairs_with(entity) {
            append_active_pair_contacts(entity, pair, &rigid_bodies, &mut contacts.0);
        }
    }
    // Consume each request once and always publish its matching response ID.
    for request in raycasts.read() {
        let hit = context
            .as_ref()
            .and_then(|context| raycast_hit(context, request, &body_tags));
        responses.write(RagdollRaycastResponse {
            request_id: request.request_id,
            hit,
        });
    }
}

/// Appends active solver contacts for one pair involving the requested body.
fn append_active_pair_contacts(
    entity: Entity,
    pair: ContactPairView<'_>,
    rigid_bodies: &Query<'_, '_, &RigidBody>,
    contacts: &mut Vec<BodyContact>,
) {
    // Ignore broad-phase overlaps without an active narrow-phase contact.
    if !pair.has_any_active_contact() {
        return;
    }
    let (Some(first_collider), Some(second_collider)) = (pair.collider1(), pair.collider2()) else {
        return;
    };
    // Orient each manifold normal away from the other collider for this endpoint.
    let (other, other_rigid_body, body_is_first) = if first_collider == entity {
        (
            second_collider,
            pair.manifolds().find_map(|manifold| manifold.rigid_body2()),
            true,
        )
    } else if second_collider == entity {
        (
            first_collider,
            pair.manifolds().find_map(|manifold| manifold.rigid_body1()),
            false,
        )
    } else {
        return;
    };
    let other_is_static = other_rigid_body
        .and_then(|rigid_body| rigid_bodies.get(rigid_body).ok())
        .is_none_or(|rigid_body| *rigid_body == RigidBody::Fixed);
    for manifold in pair.manifolds() {
        let normal = contact_normal(manifold.normal(), body_is_first);
        // Preserve Rapier's solver contact order in the backend-neutral cache.
        contacts.extend(manifold.solver_contacts().map(|point| BodyContact {
            point: point.point(),
            normal,
            other,
            other_is_static,
        }));
    }
}

/// Validates a request and returns its nearest filtered Rapier body hit.
fn raycast_hit(
    context: &bevy_rapier3d::plugin::RapierContext<'_>,
    request: &RagdollRaycast,
    body_tags: &Query<'_, '_, (&RagdollBodyOf, &BodyIndex)>,
) -> Option<RayHit> {
    // Reject malformed origins, directions, and distances before querying Rapier.
    if !request.origin.is_finite()
        || !request.direction.is_finite()
        || request.direction.length_squared() <= f32::EPSILON
        || !request.max_distance.is_finite()
        || request.max_distance <= 0.0
    {
        return None;
    }
    // Exclude the requested body and every body owned by a requested character.
    let direction = request.direction.normalize();
    let accepts = |entity| match (request.filter, body_tags.get(entity)) {
        (Some(excluded), _) if entity == excluded => false,
        (Some(excluded), Ok((owner, _))) if owner.0 == excluded => false,
        _ => true,
    };
    let filter = QueryFilter {
        flags: QueryFilterFlags::empty(),
        groups: None,
        exclude_collider: None,
        exclude_rigid_body: None,
        predicate: Some(&accepts),
    };
    // Preserve the hit point, normalized normal, and time-of-impact distance.
    let (entity, intersection) = context.cast_ray_and_get_normal(
        request.origin,
        direction,
        request.max_distance,
        true,
        filter,
    )?;
    let body = body_tags.get(entity).ok().map(|_| entity);
    Some(RayHit {
        entity,
        body,
        point: intersection.point,
        normal: intersection.normal.normalize_or_zero(),
        distance: intersection.time_of_impact,
    })
}

/// Directs a pair normal away from the other collider for this body.
fn contact_normal(pair_normal: Vec3, body_is_first: bool) -> Vec3 {
    if body_is_first {
        -pair_normal
    } else {
        pair_normal
    }
}

#[cfg(test)]
mod tests {
    //! Checks Rapier pair-normal orientation at each collider endpoint.

    use bevy::math::Vec3;

    use super::contact_normal;

    /// Each body receives a normal that points away from its contact partner.
    #[test]
    fn contact_normal_changes_for_the_second_endpoint() {
        let pair_normal = Vec3::Y;
        assert_eq!(contact_normal(pair_normal, true), -Vec3::Y);
        assert_eq!(contact_normal(pair_normal, false), Vec3::Y);
    }
}
