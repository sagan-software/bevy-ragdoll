//! Public profile, importer, builder, and asset-loader behavior.

#![cfg(all(feature = "gltf", feature = "serialize"))]

use std::f32::consts::{FRAC_PI_2, PI};
use std::fs;
use std::path::Path;
use std::time::Duration;

use bevy::asset::{AssetPlugin, AssetServer, Assets};
use bevy::math::{Isometry3d, Quat, Vec3};
use bevy::prelude::{App, MinimalPlugins};
use bevy_ragdoll::skein::{AngleRange as Degrees, RagdollBody as SkeinBody, RagdollJoint};
use bevy_ragdoll::{
    AngleRange, BodyIndex, BodySpec, JointLimits, JointSpec, ProfileBuilder, ProfileError,
    ProfileSpec, RagdollPlugin, RagdollProfile, ShapeSpec,
};

const HUMAN_GLB: &[u8] = include_bytes!("../../../assets/rigs/tgf_human/tgf_human.glb");
const OLD_BODY_PATH: &str = "tgf_rig::ragdoll::RagdollBody";
const OLD_JOINT_PATH: &str = "tgf_rig::ragdoll::RagdollJoint";
const NEW_BODY_PATH: &str = "bevy_ragdoll::skein::RagdollBody";
const NEW_JOINT_PATH: &str = "bevy_ragdoll::skein::RagdollJoint";

/// Creates a capsule in body-local coordinates.
fn capsule(a: Vec3, b: Vec3, radius: f32) -> ShapeSpec {
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
            },
            BodySpec {
                bone: "child".to_owned(),
                shape: capsule(Vec3::ZERO, Vec3::Y, 0.1),
                mass: 1.0,
                rest: child_rest,
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
        }],
    }
}

/// Loads and validates the checked-in human rig.
fn human_profile() -> RagdollProfile {
    RagdollProfile::new(ProfileSpec::from_glb(HUMAN_GLB).expect("the human GLB imports"))
        .expect("the human profile validates")
}

/// Serializes the checked-in GLB's profile as the canonical RON asset.
fn generated_human_ron() -> String {
    let spec = ProfileSpec::from_glb(HUMAN_GLB).expect("the human GLB imports");
    ron::ser::to_string_pretty(&spec, ron::ser::PrettyConfig::new())
        .expect("the human profile serializes")
}

/// Writes the canonical GLB-derived RON asset once during repository setup.
#[test]
#[ignore = "run once to generate assets/profiles/tgf_human.ragdoll.ron"]
fn generate_tgf_human_profile_asset() {
    let asset_path = Path::new("../../assets/profiles/tgf_human.ragdoll.ron");
    fs::create_dir_all(asset_path.parent().expect("asset path has a parent"))
        .expect("profile asset directory is created");
    fs::write(asset_path, generated_human_ron()).expect("profile asset is written");
}

/// Rewrites Skein component paths in the JSON chunk while preserving other GLB chunks.
fn relabel_skein_components(bytes: &[u8], old_body: &str, old_joint: &str) -> Vec<u8> {
    let json_length = u32::from_le_bytes(bytes[12..16].try_into().expect("chunk length")) as usize;
    let old_json_end = 20 + json_length;
    let mut json: serde_json::Value =
        serde_json::from_slice(&bytes[20..old_json_end]).expect("GLB JSON parses");
    for node in json["nodes"].as_array_mut().expect("nodes array") {
        let Some(components) = node
            .pointer_mut("/extras/skein")
            .and_then(serde_json::Value::as_array_mut)
        else {
            continue;
        };
        for component in components {
            let Some(entries) = component.as_object_mut() else {
                continue;
            };
            if let Some(body) = entries.remove(OLD_BODY_PATH) {
                entries.insert(old_body.to_owned(), body);
            }
            if let Some(joint) = entries.remove(OLD_JOINT_PATH) {
                entries.insert(old_joint.to_owned(), joint);
            }
        }
    }

    let mut json_bytes = serde_json::to_vec(&json).expect("GLB JSON serializes");
    while !json_bytes.len().is_multiple_of(4) {
        json_bytes.push(b' ');
    }
    let total_length = 12 + 8 + json_bytes.len() + bytes.len() - old_json_end;
    let mut rewritten = bytes[..12].to_vec();
    rewritten.extend_from_slice(&(json_bytes.len() as u32).to_le_bytes());
    rewritten.extend_from_slice(&0x4E4F_534Au32.to_le_bytes());
    rewritten.extend_from_slice(&json_bytes);
    rewritten.extend_from_slice(&bytes[old_json_end..]);
    rewritten[8..12].copy_from_slice(&(total_length as u32).to_le_bytes());
    rewritten
}

/// Imports the expected sixteen bodies and eighty kilograms from the GLB.
#[test]
fn tgf_human_has_sixteen_bodies_and_eighty_kilograms() {
    let profile = human_profile();
    assert_eq!(profile.bodies().len(), 16);
    let total_mass = profile.total_mass().kilograms();
    assert!(
        (total_mass - 80.0).abs() < 0.05,
        "total mass is {total_mass}"
    );
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
    let rest = profile.rest_poses(Isometry3d::IDENTITY).collect::<Vec<_>>();
    for bone in ["calf_l", "calf_r"] {
        let index = profile.body_index(bone).expect("the rig has each calf");
        let joint = profile.joint_of(index).expect("each calf has a joint");
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
        let tip_at_rest = rest_pose.transform_point(*b);
        let tip_turned = turned.transform_point(*b);
        assert!(tip_turned.z < tip_at_rest.z - 0.1);
    }
}

/// Checks that hip flexion moves both thigh tips forward.
#[test]
fn hips_bend_forward() {
    let profile = human_profile();
    let rest = profile.rest_poses(Isometry3d::IDENTITY).collect::<Vec<_>>();
    for bone in ["thigh_l", "thigh_r"] {
        let index = profile.body_index(bone).expect("the rig has each thigh");
        let joint = profile.joint_of(index).expect("each thigh has a joint");
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
        let tip_at_rest = rest_pose.transform_point(*b);
        let tip_turned = turned.transform_point(*b);
        assert!(tip_turned.z > tip_at_rest.z + 0.1);
    }
}

/// Reports each validation error and preserves the documented validation order.
#[test]
fn each_error_variant_is_reported() {
    let mut empty = valid_spec();
    empty.bodies.clear();
    assert!(matches!(
        RagdollProfile::new(empty),
        Err(ProfileError::Empty)
    ));

    let mut too_many = valid_spec();
    let body = too_many.bodies[0].clone();
    too_many.bodies.resize(65, body);
    assert!(matches!(
        RagdollProfile::new(too_many),
        Err(ProfileError::TooManyBodies(65))
    ));

    let mut not_a_tree = valid_spec();
    not_a_tree.joints[0].parent = 1;
    assert!(matches!(
        RagdollProfile::new(not_a_tree),
        Err(ProfileError::NotATree)
    ));

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
    let mut spec = valid_spec();
    let far_rest = Isometry3d::from_xyz(10.0, 0.0, 0.0);
    spec.bodies = vec![
        BodySpec {
            bone: "root".to_owned(),
            shape: capsule(Vec3::ZERO, Vec3::Y, 0.05),
            mass: 1.0,
            rest: Isometry3d::IDENTITY,
        },
        BodySpec {
            bone: "left".to_owned(),
            shape: capsule(Vec3::ZERO, Vec3::Y, 0.05),
            mass: 1.0,
            rest: Isometry3d::from_xyz(0.1, 0.0, 0.0),
        },
        BodySpec {
            bone: "right".to_owned(),
            shape: capsule(Vec3::ZERO, Vec3::Y, 0.05),
            mass: 1.0,
            rest: Isometry3d::from_xyz(0.2, 0.0, 0.0),
        },
        BodySpec {
            bone: "far".to_owned(),
            shape: capsule(Vec3::ZERO, Vec3::Y, 0.05),
            mass: 1.0,
            rest: far_rest,
        },
    ];
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
        })
        .collect();

    let profile = RagdollProfile::new(spec).expect("the four-body tree validates");
    let masks = profile.no_contact_masks();
    assert_ne!(masks[0] & (1 << 1), 0);
    assert_ne!(masks[1] & (1 << 2), 0);
    assert_eq!(masks[1] & (1 << 3), 0);
    assert_ne!(masks[2] & (1 << 1), 0);
    assert_eq!(profile.children_masks()[0], (1 << 1) | (1 << 2) | (1 << 3));
}

/// Serializes a GLB-derived profile spec and parses the same RON value back.
#[test]
fn ron_round_trip_is_exact() {
    let spec = ProfileSpec::from_glb(HUMAN_GLB).expect("the human GLB imports");
    let ron = ron::to_string(&spec).expect("the profile spec serializes");
    let round_trip: ProfileSpec = ron::from_str(&ron).expect("the profile spec parses");
    assert_eq!(round_trip, spec);
    assert_eq!(
        ron::to_string(&round_trip).expect("the spec reserializes"),
        ron
    );
}

/// Keeps the checked-in RON profile synchronized with its source GLB.
#[test]
fn checked_in_ron_matches_glb() {
    let asset_path = Path::new("../../assets/profiles/tgf_human.ragdoll.ron");
    let checked_in = fs::read_to_string(asset_path).expect("the profile RON asset exists");
    assert_eq!(checked_in, generated_human_ron());
}

/// Loads the checked-in RON profile through Bevy's asset server.
#[test]
fn ron_asset_loads_through_the_asset_server() {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins)
        .add_plugins(AssetPlugin {
            file_path: "../../assets".to_owned(),
            ..Default::default()
        })
        .add_plugins(RagdollPlugin::default());
    let handle = app
        .world()
        .resource::<AssetServer>()
        .load::<RagdollProfile>("profiles/tgf_human.ragdoll.ron");

    for _ in 0..100 {
        app.update();
        if let Some(profile) = app
            .world()
            .resource::<Assets<RagdollProfile>>()
            .get(&handle)
        {
            assert_eq!(profile.bodies().len(), 16);
            return;
        }
        std::thread::sleep(Duration::from_millis(1));
    }
    panic!("the profile asset loads within 100 updates");
}

/// Builds a three-body chain with the same value as a hand-written profile spec.
#[test]
fn builder_matches_spec() {
    let root = BodySpec {
        bone: "root".to_owned(),
        shape: capsule(Vec3::ZERO, Vec3::Y, 0.1),
        mass: 3.0,
        rest: Isometry3d::IDENTITY,
    };
    let middle = BodySpec {
        bone: "middle".to_owned(),
        shape: capsule(Vec3::ZERO, Vec3::Y, 0.08),
        mass: 2.0,
        rest: Isometry3d::from_xyz(0.0, 1.0, 0.0),
    };
    let end = BodySpec {
        bone: "end".to_owned(),
        shape: ShapeSpec::Sphere {
            center: Vec3::ZERO,
            radius: 0.05,
        },
        mass: 1.0,
        rest: Isometry3d::from_xyz(0.0, 2.0, 0.0),
    };
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
            },
            JointSpec {
                child: 2,
                parent: 1,
                frame: Isometry3d::from_xyz(0.0, 1.0, 0.0),
                limits,
                max_torque: 8.0,
            },
        ],
    };

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

/// Checks body masses, degree limits, hinge detection, and joint validation.
#[test]
fn skein_components_validate_like_tgf() {
    let body = SkeinBody { mass_kg: 5.5 };
    assert_eq!(body.problem(), None);
    for mass_kg in [0.0, -1.0, f32::NAN, f32::INFINITY] {
        assert!(SkeinBody { mass_kg }.problem().is_some());
    }

    let valid = RagdollJoint {
        limit_x: Degrees {
            min_deg: -180.0,
            max_deg: 180.0,
        },
        limit_y: Degrees {
            min_deg: -30.0,
            max_deg: 30.0,
        },
        limit_z: Degrees {
            min_deg: -20.0,
            max_deg: 45.0,
        },
        torque_nm: 150.0,
    };
    assert_eq!(valid.problem(), None);
    for limit_x in [
        Degrees {
            min_deg: f32::NAN,
            max_deg: 0.0,
        },
        Degrees {
            min_deg: 1.0,
            max_deg: 0.0,
        },
        Degrees {
            min_deg: -181.0,
            max_deg: 0.0,
        },
        Degrees {
            min_deg: 0.0,
            max_deg: 181.0,
        },
    ] {
        assert!(RagdollJoint { limit_x, ..valid }.problem().is_some());
    }
    assert!(
        Degrees {
            min_deg: 0.0,
            max_deg: 0.0
        }
        .is_locked()
    );
    assert!(
        !Degrees {
            min_deg: -1.0,
            max_deg: 0.0
        }
        .is_locked()
    );

    assert!(!valid.is_hinge());
    let hinge = RagdollJoint {
        limit_x: Degrees {
            min_deg: 0.0,
            max_deg: 140.0,
        },
        ..Default::default()
    };
    assert!(hinge.is_hinge());
    for torque_nm in [-1.0, f32::NAN, f32::INFINITY] {
        assert!(RagdollJoint { torque_nm, ..valid }.problem().is_some());
    }
    assert_eq!(
        RagdollJoint {
            torque_nm: 0.0,
            ..valid
        }
        .problem(),
        None
    );
    assert!(
        RagdollJoint {
            limit_y: Degrees {
                min_deg: f32::NAN,
                max_deg: 0.0
            },
            ..valid
        }
        .problem()
        .expect("invalid twist is reported")
        .starts_with("limit_y")
    );
    assert!(
        RagdollJoint {
            limit_z: Degrees {
                min_deg: 0.0,
                max_deg: 181.0
            },
            ..valid
        }
        .problem()
        .expect("invalid Z range is reported")
        .starts_with("limit_z")
    );
    assert!(serde_json::from_str::<RagdollJoint>(
        r#"{"limit_x":{"min_deg":0.0,"max_deg":0.0},"limit_y":{"min_deg":0.0,"max_deg":0.0},"limit_z":{"min_deg":0.0,"max_deg":0.0},"torque_nm":1.0,"extra":true}"#
    )
    .is_err());
}

/// Loads the TGF component type paths preserved in the imported GLB.
#[test]
fn old_skein_type_path_is_accepted() {
    assert_eq!(human_profile().bodies().len(), 16);
}

/// Loads the new crate component paths after rewriting the GLB extras.
#[test]
fn new_skein_type_path_is_accepted() {
    let glb = relabel_skein_components(HUMAN_GLB, NEW_BODY_PATH, NEW_JOINT_PATH);
    let spec = ProfileSpec::from_glb(&glb).expect("new Skein paths are accepted");
    assert_eq!(RagdollProfile::new(spec).unwrap().bodies().len(), 16);
}

/// Measures and reconstructs the five planned angles around each joint axis.
#[test]
fn joint_angles_round_trip_each_axis() {
    let spec = valid_spec();
    let profile = RagdollProfile::new(spec).expect("the profile validates");
    let child = BodyIndex::try_from(1).expect("child index fits");
    for angle in [-FRAC_PI_2, -0.1, 0.0, 0.1, FRAC_PI_2] {
        for (axis, rotation) in [
            (Vec3::X, Quat::from_rotation_x(angle)),
            (Vec3::Y, Quat::from_rotation_y(angle)),
            (Vec3::Z, Quat::from_rotation_z(angle)),
        ] {
            let poses = [Isometry3d::IDENTITY, Isometry3d::from_rotation(rotation)];
            let measured = profile
                .joint_angles(child, &poses)
                .expect("the child has a joint and a pose");
            assert!((measured - axis * angle).length() < 1.0e-5);
        }
    }
    assert!(
        profile
            .joint_angles(BodyIndex::try_from(0).unwrap(), &[])
            .is_none()
    );
}

/// Demonstrates a cuboid shape in the profile API's accepted spec data.
#[test]
fn cuboid_shapes_round_trip_through_ron() {
    let mut spec = valid_spec();
    spec.bodies[1].shape = ShapeSpec::Cuboid {
        center: Vec3::ZERO,
        rotation: Quat::IDENTITY,
        half_extents: Vec3::splat(0.25),
    };
    let encoded = ron::to_string(&spec).unwrap();
    let decoded = ron::from_str::<ProfileSpec>(&encoded).unwrap();
    assert_eq!(decoded, spec);
    assert_eq!(decoded.bodies[1].shape, spec.bodies[1].shape);
}

/// Keeps a typed body index bounded to the profile mask width.
#[test]
fn body_index_rejects_indices_outside_the_mask() {
    assert_eq!(BodyIndex::try_from(63).unwrap().get(), 63);
    assert_eq!(BodyIndex::try_from(64), Err(64));
}
