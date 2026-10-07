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

/// Checks hinge limits and backward knee flexion from the imported rest pose.
#[test]
fn knees_are_hinges_that_bend_backward() {
    let profile = human_profile();
    // Turn each calf backward by 60 degrees from rest.
    let rest = profile.rest_poses(Isometry3d::IDENTITY).collect::<Vec<_>>();
    for bone in ["calf_l", "calf_r"] {
        let index = profile.body_index(bone).expect("the rig has each calf");
        let joint = profile.joint_of(index).expect("each calf has a joint");
        // A knee is a single-axis hinge whose range includes bending backward.
        assert!(joint.limits().is_hinge());
        assert!(joint.limits().x.is_angle_within_range(-1.0));
        let rest_pose = *rest.get(index.get()).expect("the calf has a rest pose");
        let body = profile
            .bodies()
            .get(index.get())
            .expect("the calf has a body entry");
        let ShapeSpec::Capsule { b, .. } = body.shape() else {
            panic!("the calf shape is a capsule");
        };
        let turned = Isometry3d::new(
            rest_pose.translation,
            rest_pose.rotation * Quat::from_rotation_x(-PI / 3.0),
        );
        // A knee bends backward, so the calf tip moves toward -Z.
        let tip_at_rest = rest_pose.transform_point(*b);
        let tip_turned = turned.transform_point(*b);
        assert!(tip_turned.z < tip_at_rest.z - 0.1);
    }
}

/// Checks that hip flexion moves both thigh tips forward.
#[test]
fn hips_bend_forward() {
    let profile = human_profile();
    // Turn each thigh forward by 60 degrees from rest.
    let rest = profile.rest_poses(Isometry3d::IDENTITY).collect::<Vec<_>>();
    for bone in ["thigh_l", "thigh_r"] {
        let index = profile.body_index(bone).expect("the rig has each thigh");
        let joint = profile.joint_of(index).expect("each thigh has a joint");
        // A hip's X range includes bending forward.
        assert!(joint.limits().x.is_angle_within_range(1.0));
        let rest_pose = *rest.get(index.get()).expect("the thigh has a rest pose");
        let body = profile
            .bodies()
            .get(index.get())
            .expect("the thigh has a body entry");
        let ShapeSpec::Capsule { b, .. } = body.shape() else {
            panic!("the thigh shape is a capsule");
        };
        let turned = Isometry3d::new(
            rest_pose.translation,
            rest_pose.rotation * Quat::from_rotation_x(PI / 3.0),
        );
        // A hip bends forward, so the thigh tip moves toward +Z.
        let tip_at_rest = rest_pose.transform_point(*b);
        let tip_turned = turned.transform_point(*b);
        assert!(tip_turned.z > tip_at_rest.z + 0.1);
    }
}

/// Reports each validation error and preserves the documented validation order.
#[test]
fn each_error_variant_is_reported() {
    // Each case breaks one rule of an otherwise valid spec.
    let mut empty = valid_spec();
    empty.bodies.clear();
    assert!(matches!(
        RagdollProfile::new(empty),
        Err(ProfileError::Empty)
    ));

    // One body more than the 64-bit contact masks can hold.
    let mut too_many = valid_spec();
    let body = too_many.bodies[0].clone();
    too_many.bodies.resize(65, body);
    assert!(matches!(
        RagdollProfile::new(too_many),
        Err(ProfileError::TooManyBodies(65))
    ));

    // A joint whose parent is its own child forms a cycle.
    let mut not_a_tree = valid_spec();
    not_a_tree.joints[0].parent = 1;
    assert!(matches!(
        RagdollProfile::new(not_a_tree),
        Err(ProfileError::NotATree)
    ));

    // Mass must be positive and shapes must have volume.
    let mut bad_mass = valid_spec();
    bad_mass.bodies[0].mass = 0.0;
    assert!(matches!(
        RagdollProfile::new(bad_mass),
        Err(ProfileError::BadMass { body }) if body.get() == 0
    ));

    let mut bad_shape = valid_spec();
    bad_shape.bodies[1].shape = capsule(Vec3::ZERO, Vec3::Y, 0.0);
    assert!(matches!(
        RagdollProfile::new(bad_shape),
        Err(ProfileError::BadShape { body }) if body.get() == 1
    ));

    // Limits must contain zero, and the joint frame must be finite.
    let mut bad_limit = valid_spec();
    bad_limit.joints[0].limits.x = AngleRange { min: 0.1, max: 0.5 };
    assert!(matches!(
        RagdollProfile::new(bad_limit),
        Err(ProfileError::BadLimit { joint, axis })
            if joint.get() == 1 && axis == bevy_ragdoll::JointAxis::X
    ));

    let mut bad_frame = valid_spec();
    bad_frame.joints[0].frame.rotation = Quat::from_xyzw(f32::NAN, 0.0, 0.0, 1.0);
    assert!(matches!(
        RagdollProfile::new(bad_frame),
        Err(ProfileError::BadLimit { joint, axis })
            if joint.get() == 1 && axis == bevy_ragdoll::JointAxis::Frame
    ));

    // Torque must be nonnegative and bone names unique.
    let mut bad_torque = valid_spec();
    bad_torque.joints[0].max_torque = -1.0;
    assert!(matches!(
        RagdollProfile::new(bad_torque),
        Err(ProfileError::BadTorque { joint }) if joint.get() == 1
    ));

    let mut duplicate_bone = valid_spec();
    duplicate_bone.bodies[1].bone = "root".to_owned();
    assert!(matches!(
        RagdollProfile::new(duplicate_bone),
        Err(ProfileError::DuplicateBone { bone }) if bone == "root"
    ));

    // With two errors, the earlier body check is reported first.
    let mut precedence = valid_spec();
    precedence.bodies[0].mass = -1.0;
    precedence.bodies[1].shape = capsule(Vec3::ZERO, Vec3::Y, 0.0);
    assert!(matches!(
        RagdollProfile::new(precedence),
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
    assert_ne!(masks[0] & (1 << 1), 0);
    assert_ne!(masks[1] & (1 << 2), 0);
    assert_eq!(masks[1] & (1 << 3), 0);
    assert_ne!(masks[2] & (1 << 1), 0);
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

/// Builds a three-body chain with the same value as a hand-written profile spec.
#[test]
fn builder_matches_spec() {
    // The expected value, written out by hand.
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
    // Both joints share one single-axis limit set.
    let limits = JointLimits {
        x: AngleRange {
            min: -1.0,
            max: 1.0,
        },
        twist: AngleRange { min: 0.0, max: 0.0 },
        z: AngleRange { min: 0.0, max: 0.0 },
    };
    let expected = ProfileSpec {
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
    };

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
    assert_eq!(builder.into_spec(), expected);
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

/// Loads a sparse `.ragdoll.ron` overrides file through Bevy's asset server.
#[test]
fn overrides_asset_loads_through_the_asset_server() {
    // Write an overrides file into a private temporary asset directory.
    let directory =
        std::env::temp_dir().join(format!("bevy_ragdoll_overrides_{}", std::process::id()));
    fs::create_dir_all(&directory).expect("the temporary asset directory is created");
    fs::write(
        directory.join("fox.ragdoll.ron"),
        r#"(mass: Some(12.0), bones: { "tail*": (body: Skip), "head": (radius: Some(0.1)) })"#,
    )
    .expect("the overrides file is written");
    // Load it through a real AssetServer rooted at that directory.
    let mut app = App::new();
    // The ragdoll plugin registers the `.ragdoll.ron` loader.
    app.add_plugins(MinimalPlugins)
        .add_plugins(AssetPlugin {
            file_path: directory.to_string_lossy().into_owned(),
            ..Default::default()
        })
        .add_plugins(RagdollPlugin::default());
    let handle = app
        .world()
        .get_resource::<AssetServer>()
        .unwrap()
        .load::<RagdollOverrides>("fox.ragdoll.ron");
    // Poll until the loader finishes, then check the parsed values.
    for _ in 0..10_000 {
        app.update();
        if let Some(overrides) = app
            .world()
            .get_resource::<Assets<RagdollOverrides>>()
            .unwrap()
            .get(&handle)
        {
            assert_eq!(overrides.mass, Some(12.0));
            assert_eq!(
                overrides.get("tail_3").map(|bone| bone.body),
                Some(bevy_ragdoll::BoneBody::Skip)
            );
            assert_eq!(
                overrides.get("head").and_then(|bone| bone.radius),
                Some(0.1)
            );
            return;
        }
    }
    panic!("the overrides asset loads within 10000 updates");
}
