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
                role: None,
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
                role: None,
            },
        ],
        joints: vec![JointSpec {
            child: 1,
            parent: 0,
            frame: Isometry3d::from_xyz(0.0, 1.0, 0.0),
            limits,
            max_torque: 4.0,
            basis: Quat::IDENTITY,
        }],
    }
}

/// Validates the shared one-joint fixture.
fn profile() -> RagdollProfile {
    RagdollProfile::new(spec()).expect("the profile fixture is valid")
}

/// Covers the body, joint, profile, builder, and checked-index accessors.
#[test]
fn body_accessors_return_the_validated_root() {
    // Body accessors return the validated root body unchanged.
    let profile = profile();
    let root = profile.bodies()[0].clone();
    assert_eq!(
        (
            root.index().get(),
            root.bone(),
            root.mass().kilograms(),
            root.rest()
        ),
        (0, "root", 2.0, Isometry3d::IDENTITY)
    );
    assert!(matches!(
        root.shape(),
        ShapeSpec::Sphere { radius: 0.5, .. }
    ));
}

#[test]
fn joint_accessors_return_the_validated_joint() {
    let profile = profile();
    let child = BodyIndex::try_from(1).expect("child index fits");
    let joint = profile.joint_of(child).expect("the child has a joint");
    // Indexes, frame and torque come back as authored.
    assert_eq!(
        (
            joint.child(),
            joint.parent(),
            joint.frame(),
            joint.max_torque()
        ),
        (
            child,
            BodyIndex::try_from(0).unwrap(),
            Isometry3d::from_xyz(0.0, 1.0, 0.0),
            4.0
        )
    );
    // A single free axis makes the joint a hinge whose bend range is that axis.
    let full = AngleRange { min: -PI, max: PI };
    assert_eq!(
        (joint.limits().x, joint.is_hinge(), joint.bend_range()),
        (full, true, full)
    );
}

#[test]
fn profile_lookups_cover_the_root_and_missing_bones() {
    let profile = profile();
    let child = BodyIndex::try_from(1).expect("child index fits");
    // The root has no joint, and an unknown bone has no index.
    assert_eq!(
        (
            profile.joint_of(BodyIndex::try_from(0).unwrap()).is_none(),
            profile.body_index("child"),
            profile.body_index("missing"),
        ),
        (true, Some(child), None)
    );
    assert_eq!(
        (profile.total_mass().kilograms(), profile.joints().len()),
        (5.0, 1)
    );
    // Rest poses compose with the supplied root pose.
    let root_pose = Isometry3d::from_xyz(4.0, 0.0, 0.0);
    let rest_pose = profile.rest_poses(root_pose).nth(1);
    assert_eq!(rest_pose, Some(root_pose * profile.bodies()[1].rest()));
}

/// Builds the test profile body by body through `ProfileBuilder`.
fn built_profile() -> ProfileBuilder {
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
    builder
}

#[test]
fn the_builder_reproduces_the_profile() {
    let builder = built_profile();
    // Spec access, conversion, and build all agree.
    assert_eq!(builder.spec().bodies.len(), 2);
    assert_eq!(builder.clone().into_spec(), builder.spec().clone());
    assert_eq!(builder.build().unwrap(), profile());
}

/// A small sphere shape for filler bodies.
const SMALL_SPHERE: ShapeSpec = ShapeSpec::Sphere {
    center: Vec3::ZERO,
    radius: 0.1,
};

#[test]
fn the_builder_rejects_overflow_and_emptiness() {
    // Fill the builder to MAX_BODIES so the next body overflows.
    let mut oversized = ProfileBuilder::default();
    for index in 0..MAX_BODIES {
        oversized
            .add_body(index.to_string(), SMALL_SPHERE, 1.0, Isometry3d::IDENTITY)
            .expect("every mask index is accepted");
    }
    assert!(matches!(
        oversized.add_body("overflow", SMALL_SPHERE, 1.0, Isometry3d::IDENTITY),
        Err(ProfileError::TooManyBodies(65))
    ));
    // Building with no bodies reports Empty.
    assert!(matches!(
        ProfileBuilder::default().build(),
        Err(ProfileError::Empty)
    ));
}

/// Returns the validation error for the test spec after `edit`, if any.
fn error_after(edit: impl FnOnce(&mut ProfileSpec)) -> Option<ProfileError> {
    let mut invalid = spec();
    edit(&mut invalid);
    RagdollProfile::new(invalid).err()
}

/// The second body, which every single-field edit below breaks.
fn second() -> BodyIndex {
    BodyIndex::try_from(1).expect("the second body index is valid")
}

/// Covers mass overflow and every malformed body shape or rest pose.
#[test]
fn validation_rejects_bad_masses_shapes_and_rest_poses() {
    // Every shape with a non-finite field, zero extent, or a non-unit rotation.
    let shapes = [
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
    ];
    let shape_errors = shapes.map(|shape| error_after(|spec| spec.bodies[1].shape = shape));
    let bad_shape = Some(ProfileError::BadShape { body: second() });
    assert_eq!(vec![bad_shape.clone(); 7], shape_errors);
    // A non-finite rest pose is reported against its body.
    assert_eq!(
        error_after(|spec| spec.bodies[1].rest.translation.x = f32::NAN),
        bad_shape
    );
    // Two finite masses whose sum overflows name the body that overflowed.
    let overflow = error_after(|spec| {
        spec.bodies[0].mass = f32::MAX;
        spec.bodies[1].mass = f32::MAX;
    });
    assert_eq!(overflow, Some(ProfileError::BadMass { body: second() }));
}

/// Covers every malformed joint range, frame and torque.
#[test]
fn validation_rejects_bad_joint_ranges_frames_and_torques() {
    // X ranges that are non-finite, exclude zero, or exceed half a turn.
    let ranges = [
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
    ];
    let bad_limit = |axis| {
        Some(ProfileError::BadLimit {
            joint: second(),
            axis,
        })
    };
    let x_errors = ranges.map(|range| error_after(|spec| spec.joints[0].limits.x = range));
    assert_eq!(vec![bad_limit(JointAxis::X); 6], x_errors);
    // The twist and Z axes, the frame and the torque each report their own field.
    let unbounded = AngleRange {
        min: f32::NEG_INFINITY,
        max: 0.0,
    };
    let other_errors = [
        error_after(|spec| spec.joints[0].limits.twist = unbounded),
        error_after(|spec| spec.joints[0].limits.z = unbounded),
        error_after(|spec| spec.joints[0].frame.translation.x = f32::INFINITY),
        error_after(|spec| spec.joints[0].max_torque = f32::NAN),
    ];
    let expected = [
        bad_limit(JointAxis::Twist),
        bad_limit(JointAxis::Z),
        bad_limit(JointAxis::Frame),
        Some(ProfileError::BadTorque { joint: second() }),
    ];
    assert_eq!(other_errors, expected);
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

/// Covers invalid tree layouts and empty names.
#[test]
fn validation_rejects_broken_trees_and_empty_names() {
    // Self-parenting, missing and repeated joints all break the tree.
    let tree_errors = [
        error_after(|spec| {
            spec.joints[0].child = 0;
            spec.joints[0].parent = 0;
        }),
        error_after(|spec| spec.joints[0].parent = 1),
        error_after(|spec| spec.joints.clear()),
        error_after(|spec| spec.joints.push(spec.joints[0])),
    ];
    assert_eq!(vec![Some(ProfileError::NotATree); 4], tree_errors);
    // An empty bone name is rejected as a bad body.
    assert_eq!(
        error_after(|spec| spec.bodies[1].bone.clear()),
        Some(ProfileError::BadShape { body: second() })
    );
}

/// A rotation whose quaternion is not unit length.
const NON_UNIT: Quat = Quat::from_xyzw(0.0, 0.0, 0.0, 2.0);

/// Covers the quaternion sign choice in measured joint angles.
#[test]
fn a_negated_quaternion_measures_the_same_angle() {
    // The parent stays at identity so the child pose is the joint rotation.
    let profile = profile();
    let angle = 0.3;
    let poses = [
        Isometry3d::IDENTITY,
        Isometry3d::from_rotation(-Quat::from_rotation_x(angle)),
    ];
    let measured = profile.joint_angles(second(), &poses).unwrap();
    assert!((measured.x - angle).abs() < 1.0e-6);
}

/// Covers every pose and frame input that leaves joint angles undefined.
#[test]
fn joint_angles_need_enough_valid_poses_and_a_valid_frame() {
    let profile = profile();
    // Too few poses, or a non-unit child or parent rotation, give no angles.
    let pose_sets: [&[Isometry3d]; 3] = [
        &[Isometry3d::IDENTITY],
        &[Isometry3d::IDENTITY, Isometry3d::from_rotation(NON_UNIT)],
        &[Isometry3d::from_rotation(NON_UNIT), Isometry3d::IDENTITY],
    ];
    let measured = pose_sets.map(|poses| profile.joint_angles(second(), poses));
    assert_eq!(measured, [None; 3]);
    // A non-unit joint frame also leaves the angles undefined.
    let mut invalid_frame = profile;
    let joint = invalid_frame.joints[0];
    invalid_frame.joints[0] = Joint::new(
        joint.child(),
        joint.parent(),
        Isometry3d::from_rotation(NON_UNIT),
        joint.limits(),
        joint.max_torque(),
        joint.basis(),
    );
    let identity = [Isometry3d::IDENTITY, Isometry3d::IDENTITY];
    assert_eq!(invalid_frame.joint_angles(second(), &identity), None);
}
