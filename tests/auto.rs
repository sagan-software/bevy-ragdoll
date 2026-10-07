//! `Ragdoll::default()` generates a profile from the character's skeleton.
//!
//! These tests spawn code-built skeletons under a headless app and check the
//! generated bodies, masses, roles, and overrides, including the cases where
//! generation must wait for an asset or report an invalid skeleton.

use bevy::asset::{AssetPlugin, Assets};
use bevy::prelude::{App, Entity, MinimalPlugins, Transform};
use bevy_ragdoll::runtime::RagdollError;
use bevy_ragdoll::{
    BodyRole, BoneBody, Ragdoll, RagdollBone, RagdollOverrides, RagdollPlugin, RagdollProfile,
    Skeleton,
};

/// Builds a headless app with the ragdoll runtime and asset storage.
fn app() -> App {
    let mut app = App::new();
    app.add_plugins((
        MinimalPlugins,
        AssetPlugin::default(),
        RagdollPlugin::default(),
    ));
    app
}

/// Spawns `ragdoll` with the bones of `skeleton` below it.
fn spawn(app: &mut App, ragdoll: Ragdoll, skeleton: &Skeleton) -> Entity {
    let world = app.world_mut();
    // Spawn the skeleton through commands, then flush so the bones exist before the first update.
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
        .get_resource::<Assets<RagdollProfile>>()
        .unwrap()
        .get(&handle)
        .expect("the generated profile is stored")
        .clone()
}

#[test]
fn default_ragdoll_generates_a_profile_from_its_bones() {
    let mut app = app();
    // The reference humanoid gives 16 bodies once its fingers and toes are skipped.
    let character = spawn(&mut app, Ragdoll::default(), &Skeleton::humanoid());
    app.update();
    // Generation succeeded, so no error is stored.
    let profile = profile(&app, character);
    assert_eq!(profile.bodies().len(), 16);
    assert!(app.world().get::<RagdollError>(character).is_none());
}

#[test]
fn a_character_without_bones_waits() {
    let mut app = app();
    // A Ragdoll with no bones under it.
    let character = app.world_mut().spawn(Ragdoll::default()).id();
    app.update();
    // Generation waits instead of storing an empty profile.
    assert!(
        app.world()
            .get::<Ragdoll>(character)
            .unwrap()
            .profile
            .is_none()
    );
}

/// Adds a file-style override that sets total mass and skips both hands.
fn add_hand_skipping_overrides(app: &mut App) -> bevy::asset::Handle<RagdollOverrides> {
    let skip_hand = RagdollBone {
        body: BoneBody::Skip,
        role: None,
        mass: None,
        radius: None,
        limits: None,
        max_torque: None,
    };
    app.world_mut()
        .get_resource_mut::<Assets<RagdollOverrides>>()
        .unwrap()
        .add(RagdollOverrides {
            mass: Some(60.0),
            bones: [("hand_*".to_owned(), skip_hand)].into(),
        })
}

/// Returns the humanoid without a skeleton mass and with a bone component
/// override that renames the head's role.
fn humanoid_with_other_head_role() -> Skeleton {
    // Clear the skeleton mass so only the override asset sets the total.
    let mut skeleton = Skeleton::humanoid();
    skeleton.mass = None;
    // Give the head bone an explicit Other role through its bone component.
    let head = skeleton.bone_index("head").unwrap();
    skeleton
        .bones
        .get_mut(head)
        .expect("the bone index is in range")
        .overrides
        .role = Some(BodyRole::Other);
    skeleton
}

#[test]
fn bone_components_and_override_assets_change_the_profile() {
    let mut app = app();
    // Reference the hand-skipping override asset from the character.
    let ragdoll = Ragdoll {
        profile: None,
        mass: None,
        overrides: Some(add_hand_skipping_overrides(&mut app)),
    };
    // Both override sources apply in one generated profile.
    let character = spawn(&mut app, ragdoll, &humanoid_with_other_head_role());
    app.update();
    let profile = profile(&app, character);
    // Skipping both hands leaves 14 bodies, scaled to 60 kg, with the head role overridden.
    assert_eq!(profile.bodies().len(), 14);
    assert!((profile.total_mass().kilograms() - 60.0).abs() < 1.0e-3);
    let head = profile.body_index("head").unwrap();
    assert_eq!(profile.bodies()[head.get()].role(), BodyRole::Other);
}

#[test]
fn an_unloaded_override_asset_delays_generation() {
    let mut app = app();
    // The default handle never loads.
    let ragdoll = Ragdoll {
        profile: None,
        mass: None,
        overrides: Some(bevy::asset::Handle::default()),
    };
    let character = spawn(&mut app, ragdoll, &Skeleton::humanoid());
    // Generation waits for the overrides instead of ignoring them.
    app.update();
    assert!(
        app.world()
            .get::<Ragdoll>(character)
            .unwrap()
            .profile
            .is_none()
    );
}

#[test]
fn an_invalid_override_stores_a_generation_error() {
    let mut app = app();
    // A negative mass override makes the skeleton invalid.
    let mut skeleton = Skeleton::humanoid();
    skeleton.bones[0].overrides.mass = Some(-1.0);
    let character = spawn(&mut app, Ragdoll::default(), &skeleton);
    app.update();
    // The failure is stored on the character as InvalidSkeleton.
    assert!(matches!(
        app.world().get::<RagdollError>(character),
        Some(RagdollError::InvalidSkeleton(_))
    ));
}

#[test]
fn x_along_bone_rigs_insert_a_joint_basis_on_their_bodies() {
    let mut app = app();
    // Rotate every bone so its length runs along X instead of Y.
    let mut skeleton = Skeleton::humanoid();
    let quarter = bevy::math::Quat::from_rotation_z(std::f32::consts::FRAC_PI_2);
    for bone in &mut skeleton.bones {
        bone.rest.rotation *= quarter;
    }
    let character = spawn(&mut app, Ragdoll::default(), &skeleton);
    // Kinematic mode creates bodies without simulating them.
    app.world_mut()
        .get_entity_mut(character)
        .unwrap()
        .insert(bevy_ragdoll::runtime::components::RagdollMode::Kinematic);
    app.update();
    app.update();
    // Every body except the root gets a joint basis that corrects the axis.
    let mut bases = app
        .world_mut()
        .query::<&bevy_ragdoll::runtime::body::JointBasis>();
    assert_eq!(bases.iter(app.world()).count(), 15);
}
