//! Loads the Blender-generated creature GLBs headlessly and generates profiles.
//!
//! Each rig under `assets/rigs` goes through the real glTF loader and scene
//! spawn, so these tests catch naming or hierarchy changes in the generated
//! assets that code-built skeletons would miss.

use bevy::animation::AnimationPlugin;
use bevy::asset::{AssetPlugin, Assets};
use bevy::gltf::{GltfAssetLabel, GltfPlugin};
use bevy::image::ImagePlugin;
use bevy::mesh::MeshPlugin;
use bevy::prelude::{App, AssetServer, Entity, MinimalPlugins, Transform, TransformPlugin};
use bevy::world_serialization::{WorldAssetRoot, WorldSerializationPlugin};
use bevy_ragdoll::{BodyRole, Ragdoll, RagdollError, RagdollPlugin, RagdollProfile};

/// Spawns `rig` with `Ragdoll::default()` and returns its generated profile.
fn generated_profile(rig: &str) -> RagdollProfile {
    let mut app = App::new();
    app.add_plugins((
        MinimalPlugins,
        AssetPlugin {
            file_path: concat!(env!("CARGO_MANIFEST_DIR"), "/assets").to_owned(),
            ..Default::default()
        },
        TransformPlugin,
        MeshPlugin,
        ImagePlugin::default(),
        AnimationPlugin,
        WorldSerializationPlugin,
        GltfPlugin::default(),
        RagdollPlugin::default(),
    ));
    // The glTF loader registers in `Plugin::finish`.
    app.finish();
    app.cleanup();
    let scene = app
        .world()
        .get_resource::<AssetServer>()
        .unwrap()
        .load(GltfAssetLabel::Scene(0).from_asset(format!("rigs/{rig}.glb")));
    let scene_id = scene.id();
    let character: Entity = app
        .world_mut()
        .spawn((
            WorldAssetRoot(scene),
            Ragdoll::default(),
            Transform::IDENTITY,
        ))
        .id();
    // Asset loading runs on other threads, so poll against a wall-clock deadline.
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(30);
    while std::time::Instant::now() < deadline {
        app.update();
        assert!(app.world().get::<RagdollError>(character).is_none());
        let handle = app
            .world()
            .get::<Ragdoll>(character)
            .and_then(|r| r.profile.clone());
        if let Some(profile) = handle.and_then(|handle| {
            app.world()
                .get_resource::<Assets<RagdollProfile>>()
                .unwrap()
                .get(&handle)
                .cloned()
        }) {
            return profile;
        }
    }
    let state = app
        .world()
        .get_resource::<AssetServer>()
        .unwrap()
        .get_load_state(scene_id);
    let children = app
        .world()
        .get::<bevy::prelude::Children>(character)
        .map(|c| c.len());
    panic!("{rig}.glb did not produce a profile: {state:?} children {children:?}");
}

/// Counts the bodies with `role`.
fn count(profile: &RagdollProfile, role: BodyRole) -> usize {
    profile
        .bodies()
        .iter()
        .filter(|body| body.role() == role)
        .count()
}

#[test]
fn humanoid_glb_gets_the_humanoid_layout() {
    let profile = generated_profile("humanoid");
    assert_eq!(profile.bodies().len(), 16);
    assert_eq!(profile.bodies()[0].bone(), "pelvis");
    assert_eq!(count(&profile, BodyRole::Calf), 2);
}

#[test]
fn quadruped_glb_gets_four_legs_and_a_tail() {
    let profile = generated_profile("quadruped");
    assert_eq!(count(&profile, BodyRole::Thigh), 4);
    assert_eq!(count(&profile, BodyRole::Head), 1);
    assert!(count(&profile, BodyRole::Tail) >= 3);
}

#[test]
fn alien_glb_gets_seven_legs_and_three_arms() {
    let profile = generated_profile("alien");
    assert_eq!(count(&profile, BodyRole::Thigh), 7);
    assert_eq!(count(&profile, BodyRole::UpperArm), 3);
    assert_eq!(count(&profile, BodyRole::Head), 1);
}
