//! Public hit-control behavior at the core runtime boundary.

use bevy::asset::{AssetPlugin, Assets};
use bevy::ecs::message::Messages;
use bevy::math::{Isometry3d, Vec3};
use bevy::prelude::{
    AnimationPlugin, App, ChildOf, Entity, MinimalPlugins, Name, Transform, TransformPlugin,
};
use bevy::time::{Fixed, Time, TimeUpdateStrategy};
use bevy_ragdoll::profile::{
    AngleRange, BodyIndex, BodyRole, BodySpec, ProfileBuilder, ProfileSpec, RagdollProfile,
};
use bevy_ragdoll::runtime::body::{BodyDriveOutput, BodyPhysicsPose, BodyVelocity};
use bevy_ragdoll::runtime::components::{
    BodyWeights, Ragdoll, RagdollBodyOf, RagdollBodyWeights, RagdollMode,
};
use bevy_ragdoll::runtime::hit::{HitProfile, HitSettings, LastHit};
use bevy_ragdoll::runtime::messages::{HitKind, RagdollHit, RagdollImpulse};
use bevy_ragdoll::runtime::pin::{PinSettings, PinTargets};
use bevy_ragdoll::{JointLimits, RagdollPlugin, ShapeSpec};
use bevy_ragdoll_conformance::mock::MockBackendPlugin;

#[test]
/// Keeps the named impulse presets equal to the design table.
fn hit_profiles_use_the_documented_impulse_table() {
    let settings = HitSettings::default();
    let presets = [
        (HitProfile::Pistol, 12.0),
        (HitProfile::Rifle, 20.0),
        (HitProfile::Shotgun, 60.0),
        (HitProfile::Punch, 30.0),
        (HitProfile::Kick, 60.0),
        (HitProfile::Heavy, 120.0),
        (HitProfile::Explosion, 200.0),
    ];

    for (profile, expected) in presets {
        assert_eq!(settings.impulse_magnitude(profile), Some(expected));
    }

    assert_eq!(
        settings.impulse_magnitude(HitProfile::Custom(27.0)),
        Some(27.0)
    );
    assert_eq!(settings.impulse_magnitude(HitProfile::Custom(-1.0)), None);
    assert_eq!(
        settings.impulse_magnitude(HitProfile::Custom(f32::NAN)),
        None
    );
    assert_eq!(
        settings.impulse_magnitude(HitProfile::Custom(f32::INFINITY)),
        None
    );
}

#[test]
/// Maps profile bone-name hints to the closed body-role vocabulary.
fn body_roles_follow_profile_bone_name_hints() {
    let cases = [
        ("pelvis", BodyRole::Pelvis),
        ("spine_03", BodyRole::Spine),
        ("chest", BodyRole::Chest),
        ("neck_01", BodyRole::Neck),
        ("head", BodyRole::Head),
        ("upperarm_l", BodyRole::UpperArm),
        ("forearm_r", BodyRole::LowerArm),
        ("hand_l", BodyRole::Hand),
        ("thigh_r", BodyRole::Thigh),
        ("shin_l", BodyRole::Calf),
        ("foot_r", BodyRole::Foot),
        ("tail_02", BodyRole::Tail),
        ("custom_bone", BodyRole::Other),
    ];

    for (bone_name, expected) in cases {
        assert_eq!(BodyRole::from(bone_name), expected);
    }

    assert_eq!(BodyRole::Calf.muscle_floor(), 0.08);
    assert_eq!(
        BodyRole::Thigh.recovery_order_delay(),
        std::time::Duration::from_millis(100)
    );
    assert_eq!(
        BodyRole::Calf.recovery_order_delay(),
        std::time::Duration::from_millis(150)
    );
}

#[test]
/// Uses an explicit body role when it overrides the bone-name inference.
fn explicit_profile_roles_override_bone_name_hints() {
    let spec = ProfileSpec {
        bodies: vec![BodySpec {
            bone: "custom_bone".to_owned(),
            shape: ShapeSpec::Sphere {
                center: Vec3::ZERO,
                radius: 0.1,
            },
            mass: 1.0,
            rest: Isometry3d::IDENTITY,
            role: Some(BodyRole::Tail),
        }],
        joints: Vec::new(),
    };
    let profile = RagdollProfile::new(spec).expect("the single-body profile is valid");

    assert_eq!(profile.bodies()[0].role(), BodyRole::Tail);
}

#[test]
/// Loads pre-role RON data and infers roles from its existing bone names.
fn legacy_profile_ron_infers_body_roles() {
    let spec: ProfileSpec = ron::from_str(include_str!("../assets/profiles/human.ragdoll.ron"))
        .expect("the existing profile RON remains valid without role fields");
    let profile = RagdollProfile::new(spec).expect("the human profile is valid");

    assert_eq!(profile.bodies()[0].role(), BodyRole::Pelvis);
    assert_eq!(profile.bodies()[1].role(), BodyRole::Spine);
}

/// Creates the core and mock backend with one fixed step per app update.
fn app() -> App {
    let mut app = App::new();
    app.add_plugins((
        MinimalPlugins,
        TransformPlugin,
        AssetPlugin::default(),
        AnimationPlugin,
        RagdollPlugin::default(),
        MockBackendPlugin,
    ));
    app.insert_resource(Time::<Fixed>::from_hz(60.0));
    app.insert_resource(TimeUpdateStrategy::FixedTimesteps(1));
    app
}

/// Builds a two-body pelvis and spine profile for one hit behavior check.
fn two_body_profile() -> bevy_ragdoll::RagdollProfile {
    let shape = ShapeSpec::Sphere {
        center: Vec3::ZERO,
        radius: 0.1,
    };
    let mut builder = ProfileBuilder::default();
    let pelvis = builder
        .add_body("pelvis", shape, 2.0, Isometry3d::IDENTITY)
        .expect("the pelvis index is within the profile limit");
    let spine = builder
        .add_body("spine", shape, 2.0, Isometry3d::from_translation(Vec3::Y))
        .expect("the spine index is within the profile limit");
    let bend = AngleRange {
        min: -0.5,
        max: 0.5,
    };
    builder.add_joint(
        spine,
        pelvis,
        Isometry3d::from_translation(Vec3::Y),
        JointLimits {
            x: bend,
            twist: bend,
            z: bend,
        },
        80.0,
    );
    builder.build().expect("the two-body profile forms a tree")
}

/// Finds one ragdoll body by its validated profile position.
fn body_at_index(world: &mut bevy::prelude::World, character: Entity, index: usize) -> Entity {
    let mut query = world.query::<(Entity, &BodyIndex, &RagdollBodyOf)>();
    query
        .iter(world)
        .find(|(_, body_index, owner)| body_index.get() == index && owner.0 == character)
        .map(|(entity, _, _)| entity)
        .expect("the bound character owns a body at this profile index")
}

/// Binds the shared two-body profile and returns its character entity.
fn spawn_two_body_character(app: &mut App) -> Entity {
    let profile = app
        .world_mut()
        .resource_mut::<Assets<bevy_ragdoll::RagdollProfile>>()
        .add(two_body_profile());
    let character = app
        .world_mut()
        .spawn((
            Ragdoll::new(profile),
            RagdollMode::Dynamic,
            Transform::IDENTITY,
        ))
        .id();
    let pelvis = app
        .world_mut()
        .spawn((Name::new("pelvis"), Transform::IDENTITY, ChildOf(character)))
        .id();
    app.world_mut().spawn((
        Name::new("spine"),
        Transform::from_translation(Vec3::Y),
        ChildOf(pelvis),
    ));
    app.update();
    character
}

/// Builds and binds a named parent-first chain for role and hop tests.
fn spawn_chain(app: &mut App, names: &[&str]) -> Entity {
    let masses = vec![2.0; names.len()];
    spawn_chain_with_masses(app, names, &masses)
}

/// Builds a named parent-first chain with test-selected body masses.
fn spawn_chain_with_masses(app: &mut App, names: &[&str], masses: &[f32]) -> Entity {
    assert_eq!(names.len(), masses.len());
    let shape = ShapeSpec::Sphere {
        center: Vec3::ZERO,
        radius: 0.1,
    };
    let bend = AngleRange {
        min: -0.5,
        max: 0.5,
    };
    let mut builder = ProfileBuilder::default();
    let mut parent = None;
    for (name, mass) in names.iter().zip(masses) {
        let child = builder
            .add_body(*name, shape, *mass, Isometry3d::IDENTITY)
            .expect("the test chain stays within the profile limit");
        if let Some(parent) = parent {
            builder.add_joint(
                child,
                parent,
                Isometry3d::IDENTITY,
                JointLimits {
                    x: bend,
                    twist: bend,
                    z: bend,
                },
                80.0,
            );
        }
        parent = Some(child);
    }
    let profile = app
        .world_mut()
        .resource_mut::<Assets<RagdollProfile>>()
        .add(builder.build().expect("the named chain is a valid profile"));
    let character = app
        .world_mut()
        .spawn((
            Ragdoll::new(profile),
            RagdollMode::Dynamic,
            Transform::IDENTITY,
        ))
        .id();
    let mut parent = character;
    for name in names {
        parent = app
            .world_mut()
            .spawn((
                Name::new((*name).to_owned()),
                Transform::IDENTITY,
                ChildOf(parent),
            ))
            .id();
    }
    app.update();
    character
}

#[test]
/// Replacing one body override preserves all later profile entries.
fn setting_a_body_weight_preserves_later_entries() {
    let mut weights = RagdollBodyWeights::new(vec![BodyWeights::default(); 3]);
    let index = BodyIndex::try_from(0).expect("zero is a valid profile body index");
    weights.set(index, BodyWeights::new(0.25, 0.75));

    assert_eq!(weights.as_ref().len(), 3);
    assert_eq!(weights.get(0), Some(BodyWeights::new(0.25, 0.75)));
    assert_eq!(weights.get(1), Some(BodyWeights::default()));
    assert_eq!(weights.get(2), Some(BodyWeights::default()));
}

#[test]
/// Defaults to pinning every valid profile body and can select a subset.
fn pin_targets_default_to_all_and_accept_a_subset() {
    let pelvis = BodyIndex::try_from(0).expect("zero is a valid body index");
    let chest = BodyIndex::try_from(7).expect("seven is a valid body index");
    let hand = BodyIndex::try_from(12).expect("twelve is a valid body index");

    assert!(PinTargets::default().is_targeted(pelvis));
    assert!(PinTargets::default().is_targeted(hand));

    let mut targets = PinTargets::only([pelvis, chest]);
    assert!(targets.is_targeted(pelvis));
    assert!(targets.is_targeted(chest));
    assert!(!targets.is_targeted(hand));
    targets.set(chest, false);
    assert!(!targets.is_targeted(chest));
}

#[test]
/// Uses the documented idle pin values and clamps invalid tuning at construction.
fn pin_settings_match_the_idle_defaults() {
    let defaults = PinSettings::default();
    assert_eq!(defaults.frequency_hz(), 1.5);
    assert_eq!(defaults.damping_ratio(), 1.0);
    assert_eq!(defaults.max_force(), 340.0);
    assert_eq!(defaults.max_torque(), 400.0);
    assert_eq!(defaults.distance_falloff(), 2.0);

    let invalid = PinSettings::new(f32::NAN, -1.0, f32::INFINITY, -3.0, f32::NAN);
    assert_eq!(invalid.frequency_hz(), 0.0);
    assert_eq!(invalid.damping_ratio(), 0.0);
    assert_eq!(invalid.max_force(), 0.0);
    assert_eq!(invalid.max_torque(), 0.0);
    assert_eq!(invalid.distance_falloff(), 0.0);
}

#[test]
/// Inserts one full-strength weight for every profile body during binding.
fn binding_initializes_every_body_weight_to_one() {
    let mut app = app();
    let character = spawn_two_body_character(&mut app);

    let weights = app
        .world()
        .get::<RagdollBodyWeights>(character)
        .expect("binding inserts per-body weights");
    assert_eq!(weights.as_ref(), &[BodyWeights::default(); 2]);
}

#[test]
/// Applies ancestor and descendant falloff through two hops only.
fn strength_drop_follows_the_falloff_table() {
    let mut app = app();
    let character = spawn_chain(
        &mut app,
        &[
            "pelvis",
            "spine_02",
            "spine_03",
            "chest",
            "upperarm_l",
            "hand_l",
        ],
    );
    let hit_body = body_at_index(app.world_mut(), character, 2);
    app.world_mut().write_message(RagdollHit {
        body: hit_body,
        point: Vec3::ZERO,
        impulse: Vec3::X * 40.0,
        kind: HitKind::Impact,
    });

    app.update();

    let weights = app
        .world()
        .get::<RagdollBodyWeights>(character)
        .expect("binding inserts per-body weights");
    let expected = [0.7875, 0.575, 0.15, 0.405, 0.5835, 1.0];
    assert_eq!(
        weights.as_ref().len(),
        expected.len(),
        "weights: {:?}",
        weights.as_ref()
    );
    for (index, expected_muscle) in expected.into_iter().enumerate() {
        let muscle = weights
            .get(index)
            .expect("every bound profile body has a weight")
            .muscle();
        assert!(
            (muscle - expected_muscle).abs() < 1.0e-5,
            "body {index} has muscle {muscle}, expected {expected_muscle}"
        );
    }
}

#[test]
/// Stacks a second hit that arrives 0.29 seconds after the first hit.
fn streaks_stack_within_the_window() {
    let mut app = app();
    app.insert_resource(Time::<Fixed>::from_hz(100.0));
    let character = spawn_chain(&mut app, &["pelvis", "spine"]);
    let hit_body = body_at_index(app.world_mut(), character, 1);
    app.world_mut().write_message(RagdollHit {
        body: hit_body,
        point: Vec3::ZERO,
        impulse: Vec3::X * 20.0,
        kind: HitKind::Impact,
    });
    app.update();
    for _ in 0..28 {
        app.update();
    }
    app.world_mut().write_message(RagdollHit {
        body: hit_body,
        point: Vec3::ZERO,
        impulse: Vec3::X * 20.0,
        kind: HitKind::Impact,
    });
    app.update();

    let weights = app
        .world()
        .get::<RagdollBodyWeights>(character)
        .expect("binding inserts per-body weights");
    let hit_muscle = weights.get(1).expect("the hit body has a weight").muscle();
    assert!((hit_muscle - 0.4475).abs() < 1.0e-5);
    let last_hit = app
        .world()
        .get::<LastHit>(character)
        .expect("binding inserts hit history");
    assert_eq!(last_hit.streak(), 1);
}

#[test]
/// Resets the strength-drop streak when the next hit arrives after 0.3 seconds.
fn do_not_stack_after_it() {
    let mut app = app();
    app.insert_resource(Time::<Fixed>::from_hz(100.0));
    let character = spawn_chain(&mut app, &["pelvis", "spine"]);
    let hit_body = body_at_index(app.world_mut(), character, 1);
    app.world_mut().write_message(RagdollHit {
        body: hit_body,
        point: Vec3::ZERO,
        impulse: Vec3::X * 20.0,
        kind: HitKind::Impact,
    });
    app.update();
    for _ in 0..30 {
        app.update();
    }
    app.world_mut().write_message(RagdollHit {
        body: hit_body,
        point: Vec3::ZERO,
        impulse: Vec3::X * 20.0,
        kind: HitKind::Impact,
    });
    app.update();

    let weights = app
        .world()
        .get::<RagdollBodyWeights>(character)
        .expect("binding inserts per-body weights");
    let hit_muscle = weights.get(1).expect("the hit body has a weight").muscle();
    assert!((hit_muscle - 0.575).abs() < 1.0e-5);
    let last_hit = app
        .world()
        .get::<LastHit>(character)
        .expect("binding inserts hit history");
    assert_eq!(last_hit.streak(), 0);
}

#[test]
/// Starts recovery at the core before delaying hand recovery by its role order.
fn recovery_starts_after_the_delay_and_core_first() {
    let mut app = app();
    app.insert_resource(Time::<Fixed>::from_hz(100.0));
    let character = spawn_chain(&mut app, &["pelvis", "spine", "hand_l"]);
    let hand_body = body_at_index(app.world_mut(), character, 2);
    app.world_mut().write_message(RagdollHit {
        body: hand_body,
        point: Vec3::ZERO,
        impulse: Vec3::X * 40.0,
        kind: HitKind::Impact,
    });
    app.update();
    let weights_before = app
        .world()
        .get::<RagdollBodyWeights>(character)
        .expect("binding inserts per-body weights")
        .as_ref()
        .to_vec();

    for _ in 0..19 {
        app.update();
    }

    let weights_after = app
        .world()
        .get::<RagdollBodyWeights>(character)
        .expect("binding inserts per-body weights");
    assert!(
        weights_after.get(0).expect("pelvis has a weight").muscle() > weights_before[0].muscle()
    );
    assert_eq!(
        weights_after.get(2).expect("hand has a weight").muscle(),
        weights_before[2].muscle()
    );
}

#[test]
/// Limits a light hand to 3 m/s of impulse and sends the excess toward the root.
fn impulse_clamp_passes_the_excess_to_the_parent() {
    let mut app = app();
    let character = spawn_chain_with_masses(
        &mut app,
        &["pelvis", "forearm_l", "hand_l"],
        &[2.0, 2.0, 0.5],
    );
    let pelvis = body_at_index(app.world_mut(), character, 0);
    let forearm = body_at_index(app.world_mut(), character, 1);
    let hand = body_at_index(app.world_mut(), character, 2);
    app.world_mut().write_message(RagdollHit {
        body: hand,
        point: Vec3::new(1.0, 2.0, 3.0),
        impulse: Vec3::X * 12.0,
        kind: HitKind::Impact,
    });
    app.update();

    let messages = app.world().resource::<Messages<RagdollImpulse>>();
    let delivered = messages
        .iter_current_update_messages()
        .copied()
        .collect::<Vec<_>>();
    assert_eq!(delivered.len(), 3);
    let hand_impulse = delivered
        .iter()
        .find(|impulse| impulse.body == hand)
        .expect("the hand receives its bounded portion");
    assert!((hand_impulse.impulse.length() - 1.5).abs() < 1.0e-5);
    assert!(
        hand_impulse
            .point
            .abs_diff_eq(Vec3::new(1.0, 2.0, 3.0), 1.0e-6)
    );
    assert!(
        (delivered
            .iter()
            .find(|impulse| impulse.body == forearm)
            .expect("the forearm receives the second bounded portion")
            .impulse
            .x
            - 6.0)
            .abs()
            < 1.0e-5
    );
    assert!(
        (delivered
            .iter()
            .find(|impulse| impulse.body == pelvis)
            .expect("the root receives the remainder")
            .impulse
            .x
            - 4.5)
            .abs()
            < 1.0e-5
    );
    assert!(
        (delivered
            .iter()
            .map(|impulse| impulse.impulse)
            .sum::<Vec3>()
            .length()
            - 12.0)
            .abs()
            < 1.0e-5
    );
}

#[test]
/// Clamps active ragdoll linear and angular speeds to the fixed limits.
fn velocity_limits_apply_while_muscles_are_active() {
    let mut app = app();
    let character = spawn_two_body_character(&mut app);
    let body = body_at_index(app.world_mut(), character, 0);
    {
        let mut velocity = app
            .world_mut()
            .get_mut::<BodyVelocity>(body)
            .expect("backend body exposes its velocity");
        velocity.linear = Vec3::X * 12.0;
        velocity.angular = Vec3::Y * 24.0;
    }

    app.update();

    let velocity = app
        .world()
        .get::<BodyVelocity>(body)
        .expect("backend body retains its clamped velocity");
    assert!(velocity.linear.length() <= 10.0 + 1.0e-5);
    assert!(velocity.angular.length() <= 20.0 + 1.0e-5);
}

#[test]
/// Leaves velocity unclamped when every body has zero muscle strength.
fn full_limp_disables_velocity_limits() {
    let mut app = app();
    let character = spawn_two_body_character(&mut app);
    let body = body_at_index(app.world_mut(), character, 0);
    {
        let mut weights = app
            .world_mut()
            .get_mut::<RagdollBodyWeights>(character)
            .expect("binding inserts per-body weights");
        for position in 0..2 {
            let index = BodyIndex::try_from(position).expect("profile body indexes are valid");
            weights.set(index, BodyWeights::new(0.0, 0.0));
        }
    }
    {
        let mut velocity = app
            .world_mut()
            .get_mut::<BodyVelocity>(body)
            .expect("backend body exposes its velocity");
        velocity.linear = Vec3::X * 12.0;
        velocity.angular = Vec3::Y * 24.0;
    }

    app.update();

    let velocity = app
        .world()
        .get::<BodyVelocity>(body)
        .expect("backend body retains its integrated velocity");
    assert!(velocity.linear.length() > 10.0);
    let output = app
        .world()
        .get::<BodyDriveOutput>(body)
        .expect("backend body exposes its speed-limit output");
    assert_eq!(output.max_linear_speed, None);
    assert_eq!(output.max_angular_speed, None);
}

#[test]
/// Applies an incoming hit before the core drive stage updates the body.
fn a_hit_reduces_the_hit_body_muscle_weight() {
    let mut app = app();
    let profile = app
        .world_mut()
        .resource_mut::<Assets<bevy_ragdoll::RagdollProfile>>()
        .add(two_body_profile());
    let character = app
        .world_mut()
        .spawn((
            Ragdoll::new(profile),
            RagdollMode::Dynamic,
            Transform::IDENTITY,
        ))
        .id();
    let pelvis = app
        .world_mut()
        .spawn((Name::new("pelvis"), Transform::IDENTITY, ChildOf(character)))
        .id();
    app.world_mut().spawn((
        Name::new("spine"),
        Transform::from_translation(Vec3::Y),
        ChildOf(pelvis),
    ));
    app.update();
    let spine_body = body_at_index(app.world_mut(), character, 1);

    app.world_mut().write_message(RagdollHit {
        body: spine_body,
        point: Vec3::Y,
        impulse: Vec3::X * 12.0,
        kind: HitKind::Impact,
    });
    app.update();

    let weights = app
        .world()
        .get::<RagdollBodyWeights>(character)
        .expect("binding inserts per-body weights");
    assert!(weights.get(1).is_some_and(|weight| weight.muscle() < 1.0));
}

#[test]
/// Ignores non-finite points, non-finite magnitudes, and zero impulses.
fn malformed_hit_vectors_do_not_change_strength_or_publish_impulses() {
    let mut app = app();
    let character = spawn_two_body_character(&mut app);
    let body = body_at_index(app.world_mut(), character, 1);
    for (point, impulse) in [
        (Vec3::splat(f32::NAN), Vec3::X),
        (Vec3::ZERO, Vec3::splat(f32::NAN)),
        (Vec3::ZERO, Vec3::splat(f32::MAX)),
        (Vec3::ZERO, Vec3::ZERO),
    ] {
        app.world_mut().write_message(RagdollHit {
            body,
            point,
            impulse,
            kind: HitKind::Impact,
        });
    }

    app.update();

    let weights = app
        .world()
        .get::<RagdollBodyWeights>(character)
        .expect("binding inserts per-body weights");
    assert_eq!(weights.as_ref(), &[BodyWeights::default(); 2]);
    assert_eq!(
        app.world()
            .get::<LastHit>(character)
            .expect("binding inserts hit history")
            .last_at(),
        None
    );
    assert!(
        app.world()
            .resource::<Messages<RagdollImpulse>>()
            .iter_current_update_messages()
            .next()
            .is_none()
    );
}

#[test]
/// Ignores hit messages that refer to a body entity that no longer exists.
fn stale_body_hit_does_not_change_character_state() {
    let mut app = app();
    let character = spawn_two_body_character(&mut app);
    let stale_body = app.world_mut().spawn_empty().id();
    app.world_mut().despawn(stale_body);
    app.world_mut().write_message(RagdollHit {
        body: stale_body,
        point: Vec3::ZERO,
        impulse: Vec3::X * 20.0,
        kind: HitKind::Impact,
    });

    app.update();

    assert_eq!(
        app.world()
            .get::<LastHit>(character)
            .expect("binding inserts hit history")
            .last_at(),
        None
    );
    assert!(
        app.world()
            .resource::<Messages<RagdollImpulse>>()
            .iter_current_update_messages()
            .next()
            .is_none()
    );
}

#[test]
/// Ignores hits whose body relationship points to an entity without ragdoll state.
fn body_with_missing_character_state_does_not_publish_an_impulse() {
    let mut app = app();
    let owner = app
        .world_mut()
        .spawn((
            RagdollMode::Dynamic,
            RagdollBodyWeights::default(),
            LastHit::default(),
        ))
        .id();
    let body = app
        .world_mut()
        .spawn((
            BodyIndex::try_from(0).expect("zero is a valid body index"),
            BodyRole::Spine,
            RagdollBodyOf(owner),
        ))
        .id();
    app.world_mut().entity_mut(owner).remove::<RagdollMode>();
    assert!(app.world().get::<RagdollMode>(owner).is_none());
    assert!(app.world().get::<RagdollBodyWeights>(owner).is_some());
    assert!(app.world().get::<LastHit>(owner).is_some());
    app.world_mut().write_message(RagdollHit {
        body,
        point: Vec3::ZERO,
        impulse: Vec3::X * 20.0,
        kind: HitKind::Impact,
    });

    app.update();

    assert!(
        app.world()
            .resource::<Messages<RagdollImpulse>>()
            .iter_current_update_messages()
            .next()
            .is_none()
    );
}

#[test]
/// Filters another character's bodies before changing local hit weights.
fn hit_processing_keeps_each_characters_body_tree_separate() {
    let mut app = app();
    let first = spawn_chain(&mut app, &["pelvis", "spine"]);
    let second = spawn_chain(&mut app, &["pelvis", "spine"]);
    let first_spine = body_at_index(app.world_mut(), first, 1);
    app.world_mut().write_message(RagdollHit {
        body: first_spine,
        point: Vec3::ZERO,
        impulse: Vec3::X * 20.0,
        kind: HitKind::Impact,
    });

    app.update();

    assert!(
        app.world()
            .get::<LastHit>(first)
            .expect("first character has hit history")
            .last_at()
            .is_some()
    );
    assert_eq!(
        app.world()
            .get::<LastHit>(second)
            .expect("second character has hit history")
            .last_at(),
        None
    );
    assert_eq!(
        app.world()
            .get::<RagdollBodyWeights>(second)
            .expect("second character has body weights")
            .as_ref(),
        &[BodyWeights::default(); 2]
    );
}

#[test]
/// Ignores hits while the owning character is frozen.
fn frozen_ragdolls_ignore_hit_messages() {
    let mut app = app();
    let character = spawn_two_body_character(&mut app);
    let body = body_at_index(app.world_mut(), character, 1);
    *app.world_mut()
        .get_mut::<RagdollMode>(character)
        .expect("character has a mode") = RagdollMode::Frozen;
    app.world_mut().write_message(RagdollHit {
        body,
        point: Vec3::ZERO,
        impulse: Vec3::X * 20.0,
        kind: HitKind::Impact,
    });

    app.update();

    assert_eq!(
        app.world()
            .get::<LastHit>(character)
            .expect("binding inserts hit history")
            .last_at(),
        None
    );
    assert!(
        app.world()
            .resource::<Messages<RagdollImpulse>>()
            .iter_current_update_messages()
            .next()
            .is_none()
    );
}

#[test]
/// Limits hit falloff to three ancestors when impulse magnitude exceeds 40.
fn heavy_hits_reach_three_ancestors_and_stop_before_the_fourth() {
    let mut app = app();
    let character = spawn_chain(
        &mut app,
        &["pelvis", "spine_01", "spine_02", "chest", "head"],
    );
    let head = body_at_index(app.world_mut(), character, 4);
    app.world_mut().write_message(RagdollHit {
        body: head,
        point: Vec3::ZERO,
        impulse: Vec3::X * 60.0,
        kind: HitKind::Impact,
    });

    app.update();

    let weights = app
        .world()
        .get::<RagdollBodyWeights>(character)
        .expect("binding inserts per-body weights");
    assert!(weights.get(1).expect("three-hop body exists").muscle() < 1.0);
    assert_eq!(weights.get(0), Some(BodyWeights::default()));
}

#[test]
/// Does not apply local hit falloff across sibling branches of one profile tree.
fn hit_falloff_does_not_cross_between_sibling_branches() {
    let mut app = app();
    let character = spawn_branch_character(&mut app);
    let left = body_at_index(app.world_mut(), character, 2);
    app.world_mut().write_message(RagdollHit {
        body: left,
        point: Vec3::ZERO,
        impulse: Vec3::X * 20.0,
        kind: HitKind::Impact,
    });

    app.update();

    let weights = app
        .world()
        .get::<RagdollBodyWeights>(character)
        .expect("binding inserts per-body weights");
    assert!(weights.get(2).expect("hit branch exists").muscle() < 1.0);
    assert_eq!(weights.get(3), Some(BodyWeights::default()));
}

#[test]
/// Sends a root hit from its body centre when backend pose data is available.
fn root_hit_uses_the_current_body_centre() {
    let mut app = app();
    let character = spawn_chain(&mut app, &["pelvis"]);
    let root = body_at_index(app.world_mut(), character, 0);
    let centre = app
        .world()
        .get::<BodyPhysicsPose>(root)
        .expect("mock backend provides the current body pose")
        .current
        .translation
        .into();
    app.world_mut().write_message(RagdollHit {
        body: root,
        point: Vec3::new(1.0, 2.0, 3.0),
        impulse: Vec3::X * 2.0,
        kind: HitKind::Impact,
    });

    app.update();

    let impulses = app.world().resource::<Messages<RagdollImpulse>>();
    let impulse = impulses
        .iter_current_update_messages()
        .next()
        .expect("a valid root hit publishes one impulse");
    assert_eq!(impulse.body, root);
    assert!(impulse.point.abs_diff_eq(centre, 1.0e-6));
    assert_eq!(impulse.impulse, Vec3::X * 2.0);
}

#[test]
/// Uses the supplied contact point if a root body has no current backend pose.
fn root_hit_falls_back_to_the_supplied_contact_point() {
    let mut app = app();
    let character = spawn_chain(&mut app, &["pelvis"]);
    let root = body_at_index(app.world_mut(), character, 0);
    app.world_mut().entity_mut(root).remove::<BodyPhysicsPose>();
    let point = Vec3::new(1.0, 2.0, 3.0);
    app.world_mut().write_message(RagdollHit {
        body: root,
        point,
        impulse: Vec3::X * 2.0,
        kind: HitKind::Impact,
    });

    app.update();

    let impulses = app.world().resource::<Messages<RagdollImpulse>>();
    let impulse = impulses
        .iter_current_update_messages()
        .next()
        .expect("a valid root hit publishes one impulse");
    assert_eq!(impulse.body, root);
    assert_eq!(impulse.point, point);
}

#[test]
/// Stops parent impulse propagation when the parent's physics pose is absent.
fn hit_does_not_propagate_through_a_parent_without_pose_data() {
    let mut app = app();
    let character = spawn_two_body_character(&mut app);
    let pelvis = body_at_index(app.world_mut(), character, 0);
    let spine = body_at_index(app.world_mut(), character, 1);
    app.world_mut()
        .entity_mut(pelvis)
        .remove::<BodyPhysicsPose>();
    app.world_mut().write_message(RagdollHit {
        body: spine,
        point: Vec3::ZERO,
        impulse: Vec3::X * 20.0,
        kind: HitKind::Impact,
    });

    app.update();

    let impulses = app.world().resource::<Messages<RagdollImpulse>>();
    let delivered = impulses
        .iter_current_update_messages()
        .copied()
        .collect::<Vec<_>>();
    assert_eq!(delivered.len(), 1);
    assert_eq!(delivered[0].body, spine);
    assert_eq!(delivered[0].impulse, Vec3::X * 6.0);
}

#[test]
/// Stops propagation when the addressed non-root body receives the full impulse.
fn small_hit_does_not_continue_past_the_addressed_body() {
    let mut app = app();
    let character = spawn_two_body_character(&mut app);
    let spine = body_at_index(app.world_mut(), character, 1);
    app.world_mut().write_message(RagdollHit {
        body: spine,
        point: Vec3::new(1.0, 2.0, 3.0),
        impulse: Vec3::X * 2.0,
        kind: HitKind::Impact,
    });

    app.update();

    let impulses = app.world().resource::<Messages<RagdollImpulse>>();
    let delivered = impulses
        .iter_current_update_messages()
        .copied()
        .collect::<Vec<_>>();
    assert_eq!(delivered.len(), 1);
    assert_eq!(delivered[0].body, spine);
    assert_eq!(delivered[0].point, Vec3::new(1.0, 2.0, 3.0));
    assert_eq!(delivered[0].impulse, Vec3::X * 2.0);
}

/// Builds one root and two sibling bodies to check branch-local hit falloff.
fn spawn_branch_character(app: &mut App) -> Entity {
    let shape = ShapeSpec::Sphere {
        center: Vec3::ZERO,
        radius: 0.1,
    };
    let mut builder = ProfileBuilder::default();
    let root = builder
        .add_body("pelvis", shape, 2.0, Isometry3d::IDENTITY)
        .expect("the root index is valid");
    let chest = builder
        .add_body("spine", shape, 2.0, Isometry3d::IDENTITY)
        .expect("the chest index is valid");
    let left = builder
        .add_body("upperarm_l", shape, 2.0, Isometry3d::IDENTITY)
        .expect("the left arm index is valid");
    let right = builder
        .add_body("upperarm_r", shape, 2.0, Isometry3d::IDENTITY)
        .expect("the right arm index is valid");
    let bend = AngleRange {
        min: -0.5,
        max: 0.5,
    };
    let limits = JointLimits {
        x: bend,
        twist: bend,
        z: bend,
    };
    builder.add_joint(chest, root, Isometry3d::IDENTITY, limits, 80.0);
    builder.add_joint(left, chest, Isometry3d::IDENTITY, limits, 30.0);
    builder.add_joint(right, chest, Isometry3d::IDENTITY, limits, 30.0);
    let profile = app
        .world_mut()
        .resource_mut::<Assets<RagdollProfile>>()
        .add(builder.build().expect("the branch profile forms a tree"));
    let character = app
        .world_mut()
        .spawn((
            Ragdoll::new(profile),
            RagdollMode::Dynamic,
            Transform::IDENTITY,
        ))
        .id();
    let root_bone = app
        .world_mut()
        .spawn((Name::new("pelvis"), Transform::IDENTITY, ChildOf(character)))
        .id();
    let chest_bone = app
        .world_mut()
        .spawn((Name::new("spine"), Transform::IDENTITY, ChildOf(root_bone)))
        .id();
    app.world_mut().spawn((
        Name::new("upperarm_l"),
        Transform::IDENTITY,
        ChildOf(chest_bone),
    ));
    app.world_mut().spawn((
        Name::new("upperarm_r"),
        Transform::IDENTITY,
        ChildOf(chest_bone),
    ));
    app.update();
    character
}
