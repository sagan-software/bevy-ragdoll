//! Tests for profile validation and private invariants.

use super::*;

/// Creates valid one-joint source data for validation and accessor tests.
fn spec() -> ProfileSpec {
    let limits = JointLimits {
        x: AngleRange { min: -PI, max: PI },
        twist: AngleRange { min: 0.0, max: 0.0 },
        z: AngleRange { min: 0.0, max: 0.0 },
    };
    ProfileSpec {
        bodies: vec![
            BodySpec {
                bone: "root".to_owned(),
                shape: ShapeSpec::Sphere {
                    center: Vec3::ZERO,
                    radius: 0.5,
                },
                mass: 2.0,
                rest: Isometry3d::IDENTITY,
            },
            BodySpec {
                bone: "child".to_owned(),
                shape: ShapeSpec::Capsule {
                    a: Vec3::ZERO,
                    b: Vec3::Y,
                    radius: 0.25,
                },
                mass: 3.0,
                rest: Isometry3d::from_xyz(0.0, 1.0, 0.0),
            },
        ],
        joints: vec![JointSpec {
            child: 1,
            parent: 0,
            frame: Isometry3d::from_xyz(0.0, 1.0, 0.0),
            limits,
            max_torque: 4.0,
        }],
    }
}

/// Validates the shared one-joint fixture.
fn profile() -> RagdollProfile {
    RagdollProfile::new(spec()).expect("the profile fixture is valid")
}

/// Covers the body, joint, profile, builder, and checked-index accessors.
#[test]
fn accessors_and_builder_paths_return_profile_data() {
    let profile = profile();
    let root = profile.bodies()[0].clone();
    assert_eq!(root.index().get(), 0);
    assert_eq!(root.bone(), "root");
    assert_eq!(root.mass().kilograms(), 2.0);
    assert_eq!(root.rest(), Isometry3d::IDENTITY);
    assert!(matches!(
        root.shape(),
        ShapeSpec::Sphere { radius: 0.5, .. }
    ));

    let child = BodyIndex::try_from(1).expect("child index fits");
    let joint = profile.joint_of(child).expect("the child has a joint");
    assert_eq!(joint.child(), child);
    assert_eq!(joint.parent(), BodyIndex::try_from(0).unwrap());
    assert_eq!(joint.frame(), Isometry3d::from_xyz(0.0, 1.0, 0.0));
    assert_eq!(joint.limits().x, AngleRange { min: -PI, max: PI });
    assert_eq!(joint.max_torque(), 4.0);
    assert!(joint.is_hinge());
    assert_eq!(joint.bend_range(), AngleRange { min: -PI, max: PI });
    assert!(profile.joint_of(BodyIndex::try_from(0).unwrap()).is_none());
    assert_eq!(profile.body_index("child"), Some(child));
    assert_eq!(profile.body_index("missing"), None);
    assert_eq!(profile.total_mass().kilograms(), 5.0);
    assert_eq!(profile.joints().len(), 1);

    let root_pose = Isometry3d::from_xyz(4.0, 0.0, 0.0);
    let rest_pose = profile.rest_poses(root_pose).nth(1);
    let child = profile.bodies().get(1).expect("the profile has a child");
    assert_eq!(rest_pose, Some(root_pose * child.rest()));

    let mut builder = ProfileBuilder::default();
    let root = builder
        .add_body(
            "root",
            ShapeSpec::Sphere {
                center: Vec3::ZERO,
                radius: 0.5,
            },
            2.0,
            Isometry3d::IDENTITY,
        )
        .unwrap();
    let child = builder
        .add_body(
            "child",
            ShapeSpec::Capsule {
                a: Vec3::ZERO,
                b: Vec3::Y,
                radius: 0.25,
            },
            3.0,
            Isometry3d::from_xyz(0.0, 1.0, 0.0),
        )
        .unwrap();
    builder.add_joint(
        child,
        root,
        Isometry3d::from_xyz(0.0, 1.0, 0.0),
        JointLimits {
            x: AngleRange { min: -PI, max: PI },
            twist: AngleRange { min: 0.0, max: 0.0 },
            z: AngleRange { min: 0.0, max: 0.0 },
        },
        4.0,
    );
    assert_eq!(builder.spec().bodies.len(), 2);
    assert_eq!(builder.clone().into_spec(), builder.spec().clone());
    assert_eq!(builder.clone().build().unwrap(), profile);
    let mut oversized = ProfileBuilder::default();
    for index in 0..MAX_BODIES {
        oversized
            .add_body(
                index.to_string(),
                ShapeSpec::Sphere {
                    center: Vec3::ZERO,
                    radius: 0.1,
                },
                1.0,
                Isometry3d::IDENTITY,
            )
            .expect("every mask index is accepted");
    }
    assert!(matches!(
        oversized.add_body(
            "overflow",
            ShapeSpec::Sphere {
                center: Vec3::ZERO,
                radius: 0.1,
            },
            1.0,
            Isometry3d::IDENTITY,
        ),
        Err(ProfileError::TooManyBodies(65))
    ));
    assert!(matches!(
        ProfileBuilder::default().build(),
        Err(ProfileError::Empty)
    ));
}

/// Covers validation failures for every numeric and geometric domain.
#[test]
fn validation_checks_all_shape_range_and_transform_boundaries() {
    let mut overflow = spec();
    overflow.bodies[0].mass = f32::MAX;
    overflow.bodies[1].mass = f32::MAX;
    assert!(
        matches!(RagdollProfile::new(overflow), Err(ProfileError::BadMass { body }) if body.get() == 1)
    );

    for shape in [
        ShapeSpec::Capsule {
            a: Vec3::NAN,
            b: Vec3::Y,
            radius: 0.1,
        },
        ShapeSpec::Capsule {
            a: Vec3::ZERO,
            b: Vec3::INFINITY,
            radius: 0.1,
        },
        ShapeSpec::Capsule {
            a: Vec3::ZERO,
            b: Vec3::Y,
            radius: f32::NAN,
        },
        ShapeSpec::Sphere {
            center: Vec3::INFINITY,
            radius: 0.1,
        },
        ShapeSpec::Sphere {
            center: Vec3::ZERO,
            radius: f32::INFINITY,
        },
        ShapeSpec::Cuboid {
            center: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            half_extents: Vec3::new(1.0, 0.0, 1.0),
        },
        ShapeSpec::Cuboid {
            center: Vec3::ZERO,
            rotation: Quat::from_xyzw(0.0, 0.0, 0.0, 2.0),
            half_extents: Vec3::ONE,
        },
    ] {
        let mut invalid = spec();
        invalid.bodies[1].shape = shape;
        assert!(
            matches!(RagdollProfile::new(invalid), Err(ProfileError::BadShape { body }) if body.get() == 1)
        );
    }
    let mut bad_rest = spec();
    bad_rest.bodies[1].rest.translation.x = f32::NAN;
    assert!(
        matches!(RagdollProfile::new(bad_rest), Err(ProfileError::BadShape { body }) if body.get() == 1)
    );

    for range in [
        AngleRange {
            min: f32::NAN,
            max: 0.0,
        },
        AngleRange {
            min: 0.0,
            max: f32::INFINITY,
        },
        AngleRange { min: 0.1, max: PI },
        AngleRange {
            min: -PI,
            max: -0.1,
        },
        AngleRange {
            min: -PI - 0.01,
            max: 0.0,
        },
        AngleRange {
            min: 0.0,
            max: PI + 0.01,
        },
    ] {
        let mut invalid = spec();
        invalid.joints[0].limits.x = range;
        assert!(matches!(
            RagdollProfile::new(invalid),
            Err(ProfileError::BadLimit {
                axis: JointAxis::X,
                ..
            })
        ));
    }
    for axis in [JointAxis::Twist, JointAxis::Z] {
        let mut invalid = spec();
        let range = AngleRange {
            min: f32::NEG_INFINITY,
            max: 0.0,
        };
        if axis == JointAxis::Twist {
            invalid.joints[0].limits.twist = range;
        } else {
            invalid.joints[0].limits.z = range;
        }
        assert!(
            matches!(RagdollProfile::new(invalid), Err(ProfileError::BadLimit { axis: found, .. }) if found == axis)
        );
    }
    let mut bad_frame = spec();
    bad_frame.joints[0].frame.translation.x = f32::INFINITY;
    assert!(matches!(
        RagdollProfile::new(bad_frame),
        Err(ProfileError::BadLimit {
            axis: JointAxis::Frame,
            ..
        })
    ));
    let mut bad_torque = spec();
    bad_torque.joints[0].max_torque = f32::NAN;
    assert!(matches!(
        RagdollProfile::new(bad_torque),
        Err(ProfileError::BadTorque { .. })
    ));
}

/// Covers the private total-mass conversion error mapping independently.
#[test]
fn invalid_total_mass_preserves_the_last_body_index() {
    let last_body = BodyIndex::try_from(1).expect("the second body index is valid");
    assert!(matches!(
        validate_total_mass(f32::NAN, last_body),
        Err(ProfileError::BadMass { body }) if body == last_body
    ));
}

/// Covers invalid tree layouts, empty names, and angle quaternion sign choice.
#[test]
fn tree_validation_and_negative_quaternion_are_handled() {
    for (child, parent) in [(0, 0), (1, 1)] {
        let mut invalid = spec();
        invalid.joints[0].child = child;
        invalid.joints[0].parent = parent;
        assert!(matches!(
            RagdollProfile::new(invalid),
            Err(ProfileError::NotATree)
        ));
    }
    let mut missing_joint = spec();
    missing_joint.joints.clear();
    assert!(matches!(
        RagdollProfile::new(missing_joint),
        Err(ProfileError::NotATree)
    ));
    let mut repeated_joint = spec();
    repeated_joint.joints.push(repeated_joint.joints[0]);
    assert!(matches!(
        RagdollProfile::new(repeated_joint),
        Err(ProfileError::NotATree)
    ));
    let mut empty_name = spec();
    empty_name.bodies[1].bone.clear();
    assert!(
        matches!(RagdollProfile::new(empty_name), Err(ProfileError::BadShape { body }) if body.get() == 1)
    );

    let profile = profile();
    let child = BodyIndex::try_from(1).unwrap();
    let angle = 0.3;
    let poses = [
        Isometry3d::IDENTITY,
        Isometry3d::from_rotation(-Quat::from_rotation_x(angle)),
    ];
    let measured = profile.joint_angles(child, &poses).unwrap();
    assert!((measured.x - angle).abs() < 1.0e-6);
    assert!(
        profile
            .joint_angles(child, &[Isometry3d::IDENTITY])
            .is_none()
    );
    let invalid_child = [
        Isometry3d::IDENTITY,
        Isometry3d::from_rotation(Quat::from_xyzw(0.0, 0.0, 0.0, 2.0)),
    ];
    assert!(profile.joint_angles(child, &invalid_child).is_none());
    let invalid_parent = [
        Isometry3d::from_rotation(Quat::from_xyzw(0.0, 0.0, 0.0, 2.0)),
        Isometry3d::IDENTITY,
    ];
    assert!(profile.joint_angles(child, &invalid_parent).is_none());
    let mut invalid_frame = profile;
    let joint = invalid_frame.joints[0];
    invalid_frame.joints[0] = Joint::new(
        joint.child(),
        joint.parent(),
        Isometry3d::from_rotation(Quat::from_xyzw(0.0, 0.0, 0.0, 2.0)),
        joint.limits(),
        joint.max_torque(),
    );
    assert!(
        invalid_frame
            .joint_angles(child, &[Isometry3d::IDENTITY, Isometry3d::IDENTITY])
            .is_none()
    );
}
