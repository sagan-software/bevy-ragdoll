//! Tests for reflected Skein components and degree limits.

use bevy::ecs::reflect::ReflectComponent;
use bevy::prelude::World;
use bevy::reflect::std_traits::ReflectDefault;
use bevy::reflect::{TypePath, TypeRegistry};

use super::{AngleRange, RagdollBody, RagdollJoint};

/// Exercises generated reflection metadata, defaults, and component access.
#[test]
fn reflected_types_register_defaults_and_component_access() {
    let mut registry = TypeRegistry::new();
    registry.register::<RagdollBody>();
    registry.register::<AngleRange>();
    registry.register::<RagdollJoint>();

    let body_registration = registry
        .get_with_type_path(RagdollBody::type_path())
        .expect("the body type is registered");
    let body_default = body_registration
        .data::<ReflectDefault>()
        .expect("the body has reflected Default")
        .default();
    assert_eq!(
        body_default.downcast_ref::<RagdollBody>(),
        Some(&RagdollBody::default())
    );
    let body_component = body_registration
        .data::<ReflectComponent>()
        .expect("the body has reflected Component access");

    let range_registration = registry
        .get_with_type_path(AngleRange::type_path())
        .expect("the angle range type is registered");
    let range_default = range_registration
        .data::<ReflectDefault>()
        .expect("the angle range has reflected Default")
        .default();
    assert_eq!(
        range_default.downcast_ref::<AngleRange>(),
        Some(&AngleRange::default())
    );

    let joint_registration = registry
        .get_with_type_path(RagdollJoint::type_path())
        .expect("the joint type is registered");
    let joint_default = joint_registration
        .data::<ReflectDefault>()
        .expect("the joint has reflected Default")
        .default();
    assert_eq!(
        joint_default.downcast_ref::<RagdollJoint>(),
        Some(&RagdollJoint::default())
    );
    let joint_component = joint_registration
        .data::<ReflectComponent>()
        .expect("the joint has reflected Component access");

    let mut world = World::new();
    let body_entity = world.spawn(RagdollBody { mass_kg: 4.0 }).id();
    assert!(body_component.contains(world.entity(body_entity)));
    assert_eq!(
        body_component
            .reflect(world.entity(body_entity))
            .and_then(|value| value.downcast_ref::<RagdollBody>())
            .map(|body| body.mass_kg),
        Some(4.0)
    );
    {
        let mut entity = world.entity_mut(body_entity);
        let mut reflected = body_component
            .reflect_mut(&mut entity)
            .expect("the body is reflectively mutable");
        reflected.downcast_mut::<RagdollBody>().unwrap().mass_kg = 6.0;
    }
    assert_eq!(world.get::<RagdollBody>(body_entity).unwrap().mass_kg, 6.0);

    let joint_entity = world.spawn(RagdollJoint::default()).id();
    assert!(joint_component.contains(world.entity(joint_entity)));
    assert!(
        joint_component
            .reflect(world.entity(joint_entity))
            .and_then(|value| value.downcast_ref::<RagdollJoint>())
            .is_some()
    );
}
