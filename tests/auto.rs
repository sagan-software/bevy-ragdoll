//! `Ragdoll::default()` generates a profile from the character's skeleton.

use bevy::asset::{AssetPlugin, Assets};
use bevy::prelude::{App, Entity, MinimalPlugins, Transform};
use bevy_ragdoll::runtime::RagdollError;
use bevy_ragdoll::{
    BoneBody, BodyRole, Ragdoll, RagdollBone, RagdollOverrides, RagdollPlugin, RagdollProfile,
    Skeleton,
};

/// Builds a headless app with the ragdoll runtime and asset storage.
fn app() -> App {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, AssetPlugin::default(), RagdollPlugin::default()));
    app
}

/// Spawns `ragdoll` with the bones of `skeleton` below it.
fn spawn(app: &mut App, ragdoll: Ragdoll, skeleton: &Skeleton) -> Entity {
    let world = app.world_mut();
    let character = world.spawn((ragdoll, Transform::IDENTITY)).id();
    skeleton.spawn(&mut world.commands(), character);
    world.flush();
    character
}

/// Returns the profile generated for `character`.
fn profile(app: &App, character: Entity) -> RagdollProfile {
    let handle = app
        .world()
        .get::<Ragdoll>(character)
        .and_then(|ragdoll| ragdoll.profile.clone())
        .expect("the runtime stores the generated profile handle");
    app.world()
        .resource::<Assets<RagdollProfile>>()
        .get(&handle)
        .expect("the generated profile is stored")
        .clone()
}

#[test]
fn default_ragdoll_generates_a_profile_from_its_bones() {
    let mut app = app();
    let character = spawn(&mut app, Ragdoll::default(), &Skeleton::humanoid());
    app.update();
    let profile = profile(&app, character);
    assert_eq!(profile.bodies().len(), 16);
    assert!(app.world().get::<RagdollError>(character).is_none());
}

#[test]
fn a_character_without_bones_waits() {
    let mut app = app();
    let character = app.world_mut().spawn(Ragdoll::default()).id();
    app.update();
    assert!(app.world().get::<Ragdoll>(character).unwrap().profile.is_none());
}

#[test]
fn bone_components_and_override_assets_change_the_profile() {
    let mut app = app();
    let overrides = app
        .world_mut()
        .resource_mut::<Assets<RagdollOverrides>>()
        .add(RagdollOverrides {
            mass: Some(60.0),
            bones: [(
                "hand_*".to_owned(),
                RagdollBone {
                    body: BoneBody::Skip,
                    ..Default::default()
                },
            )]
            .into(),
        });
    let mut skeleton = Skeleton::humanoid();
    skeleton.mass = None;
    let head = skeleton.bone_index("head").unwrap();
    skeleton.bones[head].overrides.role = Some(BodyRole::Other);
    let ragdoll = Ragdoll {
        overrides: Some(overrides),
        ..Default::default()
    };
    let character = spawn(&mut app, ragdoll, &skeleton);
    app.update();
    let profile = profile(&app, character);
    assert_eq!(profile.bodies().len(), 14);
    assert!((profile.total_mass().kilograms() - 60.0).abs() < 1.0e-3);
    let head = profile.body_index("head").unwrap();
    assert_eq!(profile.bodies()[head.get()].role(), BodyRole::Other);
}

#[test]
fn an_unloaded_override_asset_delays_generation() {
    let mut app = app();
    let ragdoll = Ragdoll {
        overrides: Some(bevy::asset::Handle::default()),
        ..Default::default()
    };
    let character = spawn(&mut app, ragdoll, &Skeleton::humanoid());
    app.update();
    assert!(app.world().get::<Ragdoll>(character).unwrap().profile.is_none());
}

#[test]
fn an_invalid_override_stores_a_generation_error() {
    let mut app = app();
    let mut skeleton = Skeleton::humanoid();
    skeleton.bones[0].overrides.mass = Some(-1.0);
    let character = spawn(&mut app, Ragdoll::default(), &skeleton);
    app.update();
    assert!(matches!(
        app.world().get::<RagdollError>(character),
        Some(RagdollError::InvalidSkeleton(_))
    ));
}
