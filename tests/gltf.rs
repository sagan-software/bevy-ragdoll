//! Loads the Blender-generated creature GLBs headlessly and generates profiles.
//!
//! Each rig under `assets/rigs` goes through the real glTF loader and scene
//! spawn, so these tests catch naming or hierarchy changes in the generated
//! assets that code-built skeletons would miss. They need no window or GPU.

use bevy::animation::AnimationPlugin;
use bevy::asset::{AssetId, AssetPlugin, Assets};
use bevy::gltf::{GltfAssetLabel, GltfPlugin};
use bevy::image::ImagePlugin;
use bevy::mesh::MeshPlugin;
use bevy::prelude::{App, AssetServer, Entity, MinimalPlugins, Transform, TransformPlugin};
use bevy::world_serialization::{WorldAsset, WorldAssetRoot, WorldSerializationPlugin};
use bevy_ragdoll::{BodyRole, Ragdoll, RagdollError, RagdollPlugin, RagdollProfile};

/// Builds a headless app that loads glTF through the real asset pipeline,
/// rooted at the crate's assets directory.
fn gltf_app() -> App {
    let mut app = App::new();
    #[cfg_attr(
        dylint_lib = "sagan_lints",
        expect(
            struct_update_default,
            reason = "conflicts with clippy::field_reassign_with_default"
        )
    )]
    // Root asset paths at the crate assets directory.
    let asset_plugin = AssetPlugin {
        file_path: concat!(env!("CARGO_MANIFEST_DIR"), "/assets").to_owned(),
        ..Default::default()
    };
    // Install the glTF, scene, and ragdoll plugins headlessly.
    app.add_plugins((
        MinimalPlugins,
        asset_plugin,
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
    app
}

/// Spawns the first scene of `rig` under a default `Ragdoll` and returns the
/// character and the scene asset id.
fn spawn_rig(app: &mut App, rig: &str) -> (Entity, AssetId<WorldAsset>) {
    // Start loading the first scene of the rig file.
    let scene = app
        .world()
        .get_resource::<AssetServer>()
        .unwrap()
        .load(GltfAssetLabel::Scene(0).from_asset(format!("rigs/{rig}.glb")));
    let scene_id = scene.id();
    // Spawn that scene under a default Ragdoll so the runtime generates its profile.
    let character = app
        .world_mut()
        .spawn((
            WorldAssetRoot(scene),
            Ragdoll::default(),
            Transform::IDENTITY,
        ))
        .id();
    (character, scene_id)
}

/// Returns the profile generated for `character`, once it has loaded.
fn generated_profile_of(app: &App, character: Entity) -> Option<RagdollProfile> {
    let handle = app
        .world()
        .get::<Ragdoll>(character)
        .and_then(|r| r.profile.clone())?;
    app.world()
        .get_resource::<Assets<RagdollProfile>>()
        .unwrap()
        .get(&handle)
        .cloned()
}

/// Spawns `rig` with `Ragdoll::default()` and returns its generated profile.
fn generated_profile(rig: &str) -> RagdollProfile {
    // Load the rig in a fresh headless glTF app.
    let mut app = gltf_app();
    let (character, scene_id) = spawn_rig(&mut app, rig);
    // Asset loading runs on other threads, so poll against a wall-clock deadline.
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(30);
    while std::time::Instant::now() < deadline {
        app.update();
        // Generation must never store an error while the rig loads.
        assert!(app.world().get::<RagdollError>(character).is_none());
        if let Some(profile) = generated_profile_of(&app, character) {
            return profile;
        }
    }
    // Report the scene load state and spawned children when the deadline passes.
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
