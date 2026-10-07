//! Public profile, generator, builder, and asset-loader behavior.
//!
//! The tests build profiles through every public entry point and check that
//! validation rejects malformed shapes, joints, and limits while valid input
//! keeps its bodies, joints, masses, and rest poses exactly as written, including
//! after a RON round trip.

#![cfg(feature = "serialize")]

use std::f32::consts::{FRAC_PI_2, PI};
use std::fs;

use bevy::asset::{AssetPlugin, AssetServer, Assets};
use bevy::math::{Isometry3d, Quat, Vec3};
use bevy::prelude::{App, MinimalPlugins};
use bevy_ragdoll::{
    AngleRange, BodyIndex, BodySpec, JointLimits, JointSpec, ProfileBuilder, ProfileError,
    ProfileSpec, RagdollOverrides, RagdollPlugin, RagdollProfile, ShapeSpec, Skeleton,
};

/// Creates a capsule in body-local coordinates.
const fn capsule(a: Vec3, b: Vec3, radius: f32) -> ShapeSpec {
    ShapeSpec::Capsule { a, b, radius }
}

/// Creates the two-body valid spec used by single-error cases.
fn valid_spec() -> ProfileSpec {
    let root_rest = Isometry3d::IDENTITY;
    let child_rest = Isometry3d::from_xyz(0.0, 1.0, 0.0);
    ProfileSpec {
        bodies: vec![
            BodySpec {
                bone: "root".to_owned(),
                shape: capsule(Vec3::ZERO, Vec3::Y, 0.1),
                mass: 1.0,
                rest: root_rest,
                role: None,
            },
            BodySpec {
                bone: "child".to_owned(),
                shape: capsule(Vec3::ZERO, Vec3::Y, 0.1),
                mass: 1.0,
                rest: child_rest,
                role: None,
            },
        ],
        joints: vec![JointSpec {
            child: 1,
            parent: 0,
            frame: root_rest.inverse() * child_rest,
            limits: JointLimits {
                x: AngleRange {
                    min: -FRAC_PI_2,
                    max: FRAC_PI_2,
                },
                twist: AngleRange { min: 0.0, max: 0.0 },
                z: AngleRange { min: 0.0, max: 0.0 },
            },
            max_torque: 5.0,
            basis: Quat::IDENTITY,
        }],
    }
}

/// Generates the profile of the reference humanoid skeleton.
fn human_profile() -> RagdollProfile {
    RagdollProfile::from_skeleton(&Skeleton::humanoid()).expect("the human profile validates")
}

/// Generates the expected sixteen bodies and 80 kilograms.
#[test]
fn human_has_sixteen_bodies_and_eighty_kilograms() {
    // The reference humanoid's generated profile.
    let profile = human_profile();
    assert_eq!(profile.bodies().len(), 16);
    let total_mass = profile.total_mass().kilograms();
    assert!(
        (total_mass - 80.0).abs() < 0.05,
        "total mass is {total_mass}"
    );
    // The pelvis comes first, so it is the root body.
    assert_eq!(
        profile
            .bodies()
            .first()
            .expect("the profile has a root")
            .bone(),
        "pelvis"
    );
}

/// Returns the joint limits of the generated humanoid body named `bone`.
fn humanoid_joint_limits(profile: &RagdollProfile, bone: &str) -> JointLimits {
    let index = profile.body_index(bone).expect("the rig has the bone");
    profile
        .joint_of(index)
        .expect("the bone has a joint")
        .limits()
}

/// Turns the capsule body named `bone` about its local X axis by `angle`
/// radians from rest and returns the tip's Z at rest and after the turn.
fn capsule_tip_z_after_turn(profile: &RagdollProfile, bone: &str, angle: f32) -> (f32, f32) {
    // Find the bone rest pose and its capsule tip.
    let index = profile.body_index(bone).expect("the rig has the bone");
    let rest_pose = profile
        .rest_poses(Isometry3d::IDENTITY)
        .nth(index.get())
        .expect("the bone has a rest pose");
    let body = profile
        .bodies()
        .get(index.get())
        .expect("the bone has a body entry");
    let ShapeSpec::Capsule { b, .. } = body.shape() else {
        panic!("the {bone} shape is a capsule");
    };
    // Turn the body about its local X axis without moving its origin.
    let turned = Isometry3d::new(
        rest_pose.translation,
        rest_pose.rotation * Quat::from_rotation_x(angle),
    );
    (
        rest_pose.transform_point(*b).z,
        turned.transform_point(*b).z,
    )
}

/// Checks hinge limits and backward knee flexion from the imported rest pose.
#[test]
fn left_knee_is_a_hinge_that_bends_backward() {
    let profile = human_profile();
    // A knee is a single-axis hinge whose range includes bending backward.
    let limits = humanoid_joint_limits(&profile, "calf_l");
    assert!(limits.is_hinge());
    assert!(limits.x.is_angle_within_range(-1.0));
    // Turning the calf backward by 60 degrees moves its tip toward -Z.
    let (tip_at_rest, tip_turned) = capsule_tip_z_after_turn(&profile, "calf_l", -PI / 3.0);
    assert!(tip_turned < tip_at_rest - 0.1);
}

/// Checks hinge limits and backward knee flexion from the imported rest pose.
#[test]
fn right_knee_is_a_hinge_that_bends_backward() {
    let profile = human_profile();
    // A knee is a single-axis hinge whose range includes bending backward.
    let limits = humanoid_joint_limits(&profile, "calf_r");
    assert!(limits.is_hinge());
    assert!(limits.x.is_angle_within_range(-1.0));
    // Turning the calf backward by 60 degrees moves its tip toward -Z.
    let (tip_at_rest, tip_turned) = capsule_tip_z_after_turn(&profile, "calf_r", -PI / 3.0);
    assert!(tip_turned < tip_at_rest - 0.1);
}

/// Checks that hip flexion moves the left thigh tip forward.
#[test]
fn left_hip_bends_forward() {
    let profile = human_profile();
    // A hip's X range includes bending forward.
    assert!(
        humanoid_joint_limits(&profile, "thigh_l")
            .x
            .is_angle_within_range(1.0)
    );
    // Turning the thigh forward by 60 degrees moves its tip toward +Z.
    let (tip_at_rest, tip_turned) = capsule_tip_z_after_turn(&profile, "thigh_l", PI / 3.0);
    assert!(tip_turned > tip_at_rest + 0.1);
}

/// Checks that hip flexion moves the right thigh tip forward.
#[test]
fn right_hip_bends_forward() {
    let profile = human_profile();
    // A hip's X range includes bending forward.
    assert!(
        humanoid_joint_limits(&profile, "thigh_r")
            .x
            .is_angle_within_range(1.0)
    );
    // Turning the thigh forward by 60 degrees moves its tip toward +Z.
    let (tip_at_rest, tip_turned) = capsule_tip_z_after_turn(&profile, "thigh_r", PI / 3.0);
    assert!(tip_turned > tip_at_rest + 0.1);
}

/// Validates `valid_spec` after `edit` breaks one or more of its rules.
fn validate_edited(edit: impl FnOnce(&mut ProfileSpec)) -> Result<RagdollProfile, ProfileError> {
    let mut spec = valid_spec();
    edit(&mut spec);
    RagdollProfile::new(spec)
}

/// Rejects a profile without bodies.
#[test]
fn empty_profile_is_rejected() {
    let result = validate_edited(|spec| spec.bodies.clear());

    assert!(matches!(result, Err(ProfileError::Empty)));
}

/// Rejects one body more than the 64-bit contact masks can hold.
#[test]
fn too_many_bodies_are_rejected() {
    let result = validate_edited(|spec| {
        let body = spec.bodies[0].clone();
        spec.bodies.resize(65, body);
    });

    assert!(matches!(result, Err(ProfileError::TooManyBodies(65))));
}

/// Rejects a joint whose parent is its own child, which forms a cycle.
#[test]
fn joint_cycle_is_rejected() {
    let result = validate_edited(|spec| spec.joints[0].parent = 1);

    assert!(matches!(result, Err(ProfileError::NotATree)));
}

/// Rejects a body mass that is not positive.
#[test]
fn zero_mass_is_rejected() {
    let result = validate_edited(|spec| spec.bodies[0].mass = 0.0);

    assert!(matches!(
        result,
        Err(ProfileError::BadMass { body }) if body.get() == 0
    ));
}

/// Rejects a shape without volume.
#[test]
fn zero_radius_shape_is_rejected() {
    let result = validate_edited(|spec| spec.bodies[1].shape = capsule(Vec3::ZERO, Vec3::Y, 0.0));

    assert!(matches!(
        result,
        Err(ProfileError::BadShape { body }) if body.get() == 1
    ));
}

/// Rejects a joint limit range that does not contain zero.
#[test]
fn limit_without_zero_is_rejected() {
    let result =
        validate_edited(|spec| spec.joints[0].limits.x = AngleRange { min: 0.1, max: 0.5 });

    assert!(matches!(
        result,
        Err(ProfileError::BadLimit { joint, axis })
            if joint.get() == 1 && axis == bevy_ragdoll::JointAxis::X
    ));
}

/// Rejects a joint frame that is not finite.
#[test]
fn non_finite_joint_frame_is_rejected() {
    let result = validate_edited(|spec| {
        spec.joints[0].frame.rotation = Quat::from_xyzw(f32::NAN, 0.0, 0.0, 1.0);
    });

    assert!(matches!(
        result,
        Err(ProfileError::BadLimit { joint, axis })
            if joint.get() == 1 && axis == bevy_ragdoll::JointAxis::Frame
    ));
}

/// Rejects a negative joint torque.
#[test]
fn negative_torque_is_rejected() {
    let result = validate_edited(|spec| spec.joints[0].max_torque = -1.0);

    assert!(matches!(
        result,
        Err(ProfileError::BadTorque { joint }) if joint.get() == 1
    ));
}

/// Rejects two bodies with the same bone name.
#[test]
fn duplicate_bone_is_rejected() {
    let result = validate_edited(|spec| spec.bodies[1].bone = "root".to_owned());

    assert!(matches!(
        result,
        Err(ProfileError::DuplicateBone { bone }) if bone == "root"
    ));
}

/// Reports the earlier body check first when a spec has two errors.
#[test]
fn validation_reports_the_earlier_body_error_first() {
    let result = validate_edited(|spec| {
        spec.bodies[0].mass = -1.0;
        spec.bodies[1].shape = capsule(Vec3::ZERO, Vec3::Y, 0.0);
    });

    assert!(matches!(
        result,
        Err(ProfileError::BadMass { body }) if body.get() == 0
    ));
}

/// Derives joint neighbours and touching capsule pairs without excluding a far pair.
#[test]
fn no_contact_holds_neighbours_and_touching_capsules() {
    // Three neighbours side by side under one root, and one far away.
    let mut spec = valid_spec();
    let far_rest = Isometry3d::from_xyz(10.0, 0.0, 0.0);
    spec.bodies = vec![
        BodySpec {
            bone: "root".to_owned(),
            shape: capsule(Vec3::ZERO, Vec3::Y, 0.05),
            mass: 1.0,
            rest: Isometry3d::IDENTITY,
            role: None,
        },
        BodySpec {
            bone: "left".to_owned(),
            shape: capsule(Vec3::ZERO, Vec3::Y, 0.05),
            mass: 1.0,
            rest: Isometry3d::from_xyz(0.1, 0.0, 0.0),
            role: None,
        },
        BodySpec {
            bone: "right".to_owned(),
            shape: capsule(Vec3::ZERO, Vec3::Y, 0.05),
            mass: 1.0,
            rest: Isometry3d::from_xyz(0.2, 0.0, 0.0),
            role: None,
        },
        BodySpec {
            bone: "far".to_owned(),
            shape: capsule(Vec3::ZERO, Vec3::Y, 0.05),
            mass: 1.0,
            rest: far_rest,
            role: None,
        },
    ];
    // Each neighbour hangs from the root.
    spec.joints = [1, 2, 3]
        .into_iter()
        .map(|child| JointSpec {
            child,
            parent: 0,
            frame: Isometry3d::from_xyz(f32::from(child) * 0.1, 0.0, 0.0),
            limits: JointLimits {
                x: AngleRange {
                    min: -1.0,
                    max: 1.0,
                },
                twist: AngleRange { min: 0.0, max: 0.0 },
                z: AngleRange { min: 0.0, max: 0.0 },
            },
            max_torque: 5.0,
            basis: Quat::IDENTITY,
        })
        .collect();

    let profile = RagdollProfile::new(spec).expect("the four-body tree validates");
    // Jointed and touching pairs skip contact; the far body still collides.
    let masks = profile.no_contact_masks();
    let skips_contact = [(0, 1), (1, 2), (1, 3), (2, 1)]
        .map(|(body, other)| (body, other, masks[body] & (1 << other) != 0));
    assert_eq!(
        skips_contact,
        [(0, 1, true), (1, 2, true), (1, 3, false), (2, 1, true)]
    );
    assert_eq!(profile.children_masks()[0], (1 << 1) | (1 << 2) | (1 << 3));
}

/// Serializes a generated profile spec and parses the same RON value back.
#[test]
fn ron_round_trip_is_exact() {
    // Serialize the generated humanoid spec and parse it back.
    let spec = ProfileSpec::from(&Skeleton::humanoid());
    let ron = ron::to_string(&spec).expect("the profile spec serializes");
    let round_trip: ProfileSpec = ron::from_str(&ron).expect("the profile spec parses");
    // The value and its text form both survive the round trip.
    assert_eq!(round_trip, spec);
    assert_eq!(
        ron::to_string(&round_trip).expect("the spec reserializes"),
        ron
    );
}

/// The single-axis limit set shared by both joints of the three-body chain.
const fn chain_limits() -> JointLimits {
    JointLimits {
        x: AngleRange {
            min: -1.0,
            max: 1.0,
        },
        twist: AngleRange { min: 0.0, max: 0.0 },
        z: AngleRange { min: 0.0, max: 0.0 },
    }
}

/// The three-body chain spec, written out by hand.
fn three_body_chain_spec() -> ProfileSpec {
    // Two capsules and an end sphere, each one metre above the previous body.
    let root = BodySpec {
        bone: "root".to_owned(),
        shape: capsule(Vec3::ZERO, Vec3::Y, 0.1),
        mass: 3.0,
        rest: Isometry3d::IDENTITY,
        role: None,
    };
    let middle = BodySpec {
        bone: "middle".to_owned(),
        shape: capsule(Vec3::ZERO, Vec3::Y, 0.08),
        mass: 2.0,
        rest: Isometry3d::from_xyz(0.0, 1.0, 0.0),
        role: None,
    };
    let end = BodySpec {
        bone: "end".to_owned(),
        shape: ShapeSpec::Sphere {
            center: Vec3::ZERO,
            radius: 0.05,
        },
        mass: 1.0,
        rest: Isometry3d::from_xyz(0.0, 2.0, 0.0),
        role: None,
    };
    // Each joint hangs a body from the previous one.
    let limits = chain_limits();
    ProfileSpec {
        bodies: vec![root, middle, end],
        joints: vec![
            JointSpec {
                child: 1,
                parent: 0,
                frame: Isometry3d::from_xyz(0.0, 1.0, 0.0),
                limits,
                max_torque: 12.0,
                basis: Quat::IDENTITY,
            },
            JointSpec {
                child: 2,
                parent: 1,
                frame: Isometry3d::from_xyz(0.0, 1.0, 0.0),
                limits,
                max_torque: 8.0,
                basis: Quat::IDENTITY,
            },
        ],
    }
}

/// Builds a three-body chain with the same value as a hand-written profile spec.
#[test]
fn builder_matches_spec() {
    // Both joints share one single-axis limit set.
    let limits = chain_limits();
    // The same chain through the builder must produce an identical spec.
    let mut builder = ProfileBuilder::default();
    let root = builder
        .add_body(
            "root",
            capsule(Vec3::ZERO, Vec3::Y, 0.1),
            3.0,
            Isometry3d::IDENTITY,
        )
        .expect("root index fits");
    let middle = builder
        .add_body(
            "middle",
            capsule(Vec3::ZERO, Vec3::Y, 0.08),
            2.0,
            Isometry3d::from_xyz(0.0, 1.0, 0.0),
        )
        .expect("middle index fits");
    let end = builder
        .add_body(
            "end",
            ShapeSpec::Sphere {
                center: Vec3::ZERO,
                radius: 0.05,
            },
            1.0,
            Isometry3d::from_xyz(0.0, 2.0, 0.0),
        )
        .expect("end index fits");
    // Join the bodies in the same parent-first order as the spec.
    builder.add_joint(
        middle,
        root,
        Isometry3d::from_xyz(0.0, 1.0, 0.0),
        limits,
        12.0,
    );
    builder.add_joint(
        end,
        middle,
        Isometry3d::from_xyz(0.0, 1.0, 0.0),
        limits,
        8.0,
    );
    assert_eq!(builder.into_spec(), three_body_chain_spec());
}

/// Measures and reconstructs the five planned angles around each joint axis.
#[test]
fn joint_angles_round_trip_each_axis() {
    // Rotate the child about each axis by angles across the joint range.
    let spec = valid_spec();
    let profile = RagdollProfile::new(spec).expect("the profile validates");
    let child = BodyIndex::try_from(1).expect("child index fits");
    // Collect every angle and axis whose measured angle differs.
    let mismatches = [-FRAC_PI_2, -0.1, 0.0, 0.1, FRAC_PI_2]
        .into_iter()
        .flat_map(|angle| {
            [
                (Vec3::X, Quat::from_rotation_x(angle)),
                (Vec3::Y, Quat::from_rotation_y(angle)),
                (Vec3::Z, Quat::from_rotation_z(angle)),
            ]
            .map(|(axis, rotation)| (angle, axis, rotation))
        })
        .filter(|(angle, axis, rotation)| {
            let poses = [Isometry3d::IDENTITY, Isometry3d::from_rotation(*rotation)];
            let measured = profile
                .joint_angles(child, &poses)
                .expect("the child has a joint and a pose");
            (measured - *axis * *angle).length() >= 1.0e-5
        })
        .map(|(angle, axis, _)| (angle, axis))
        .collect::<Vec<_>>();
    assert_eq!(mismatches, []);
    // A body without a joint has no angles.
    assert!(
        profile
            .joint_angles(BodyIndex::try_from(0).unwrap(), &[])
            .is_none()
    );
}

/// Demonstrates a cuboid shape in the profile API's accepted spec data.
#[test]
fn cuboid_shapes_round_trip_through_ron() {
    // Replace one capsule with a cuboid and round-trip it through RON.
    let mut spec = valid_spec();
    spec.bodies[1].shape = ShapeSpec::Cuboid {
        center: Vec3::ZERO,
        rotation: Quat::IDENTITY,
        half_extents: Vec3::splat(0.25),
    };
    let encoded = ron::to_string(&spec).unwrap();
    // Decoding must give back the cuboid unchanged.
    let decoded = ron::from_str::<ProfileSpec>(&encoded).unwrap();
    assert_eq!(decoded, spec);
    assert_eq!(decoded.bodies[1].shape, spec.bodies[1].shape);
}

/// Keeps a typed body index bounded to the profile mask width.
#[test]
fn body_index_rejects_indices_outside_the_mask() {
    // Index 63 is the last bit in the 64-body mask.
    assert_eq!(BodyIndex::try_from(63).unwrap().get(), 63);
    assert_eq!(BodyIndex::try_from(64), Err(64));
}

/// Writes `fox.ragdoll.ron` into a private temporary asset directory and
/// returns that directory.
fn write_overrides_asset_directory() -> std::path::PathBuf {
    // Use a per-process directory so parallel test runs do not collide.
    let directory =
        std::env::temp_dir().join(format!("bevy_ragdoll_overrides_{}", std::process::id()));
    fs::create_dir_all(&directory).expect("the temporary asset directory is created");
    fs::write(
        directory.join("fox.ragdoll.ron"),
        r#"(mass: Some(12.0), bones: { "tail*": (body: Skip), "head": (radius: Some(0.1)) })"#,
    )
    .expect("the overrides file is written");
    directory
}

/// Loads `fox.ragdoll.ron` through a real `AssetServer` rooted at `directory`.
fn load_overrides(directory: &std::path::Path) -> RagdollOverrides {
    let mut app = App::new();
    // The ragdoll plugin registers the `.ragdoll.ron` loader.
    #[cfg_attr(
        dylint_lib = "sagan_lints",
        expect(
            struct_update_default,
            reason = "conflicts with clippy::field_reassign_with_default"
        )
    )]
    let asset_plugin = AssetPlugin {
        file_path: directory.to_string_lossy().into_owned(),
        ..Default::default()
    };
    app.add_plugins(MinimalPlugins)
        .add_plugins(asset_plugin)
        .add_plugins(RagdollPlugin::default());
    let handle = app
        .world()
        .get_resource::<AssetServer>()
        .unwrap()
        .load::<RagdollOverrides>("fox.ragdoll.ron");
    // Poll until the loader finishes.
    for _ in 0..10_000 {
        app.update();
        if let Some(overrides) = app
            .world()
            .get_resource::<Assets<RagdollOverrides>>()
            .unwrap()
            .get(&handle)
        {
            return overrides.clone();
        }
    }
    panic!("the overrides asset loads within 10000 updates");
}

/// Loads a sparse `.ragdoll.ron` overrides file through Bevy's asset server.
#[test]
fn overrides_asset_loads_through_the_asset_server() {
    let overrides = load_overrides(&write_overrides_asset_directory());

    let parsed = (
        overrides.mass,
        overrides.get("tail_3").map(|bone| bone.body),
        overrides.get("head").and_then(|bone| bone.radius),
    );
    assert_eq!(
        parsed,
        (Some(12.0), Some(bevy_ragdoll::BoneBody::Skip), Some(0.1))
    );
}
