//! Unit tests for skeleton-based profile generation.

use super::humanoid::{Side, normalize, role_of};
use super::*;
use crate::profile::{AngleRange, BodyRole, JointLimits, ShapeSpec};

/// Generates a profile and panics with the error when it is invalid.
fn generate(skeleton: &Skeleton) -> RagdollProfile {
    RagdollProfile::from_skeleton(skeleton).expect("generated profile is valid")
}

/// Returns `(bone, role)` pairs of a profile in body order.
fn roles(profile: &RagdollProfile) -> Vec<(&str, BodyRole)> {
    profile
        .bodies()
        .iter()
        .map(|body| (body.bone(), body.role()))
        .collect()
}

/// Counts the bodies with `role`.
fn count(profile: &RagdollProfile, role: BodyRole) -> usize {
    profile
        .bodies()
        .iter()
        .filter(|body| body.role() == role)
        .count()
}

/// Counts the bodies with each of `wanted`, in the same order.
fn counts<const N: usize>(profile: &RagdollProfile, wanted: [BodyRole; N]) -> [usize; N] {
    wanted.map(|role| count(profile, role))
}

/// The reference humanoid with every bone renamed through `rename`.
fn renamed(rename: impl Fn(&str) -> String) -> Skeleton {
    let mut skeleton = Skeleton::humanoid();
    skeleton
        .bones
        .iter_mut()
        .for_each(|bone| bone.name = rename(&bone.name));
    skeleton
}

/// A dog-like quadruped facing +Z with a tail, neck and head.
fn quadruped() -> Skeleton {
    let mut bones = vec![
        ("hips", None, Vec3::new(0.0, 0.6, -0.3)),
        ("chest", Some("hips"), Vec3::new(0.0, 0.62, 0.3)),
        ("neck", Some("chest"), Vec3::new(0.0, 0.75, 0.45)),
        ("skull", Some("neck"), Vec3::new(0.0, 0.85, 0.6)),
        ("tail_1", Some("hips"), Vec3::new(0.0, 0.62, -0.4)),
        ("tail_2", Some("tail_1"), Vec3::new(0.0, 0.64, -0.55)),
        ("tail_3", Some("tail_2"), Vec3::new(0.0, 0.66, -0.7)),
    ];
    // Front legs hang from the chest and back legs from the hips.
    let legs: [(&str, &str, f32, f32); 4] = [
        ("leg_fl", "chest", 0.12, 0.3),
        ("leg_fr", "chest", -0.12, 0.3),
        ("leg_bl", "hips", 0.12, -0.3),
        ("leg_br", "hips", -0.12, -0.3),
    ];
    let names = legs
        .iter()
        .map(|(name, ..)| [1, 2, 3].map(|i| format!("{name}_{i}")))
        .collect::<Vec<_>>();
    // Each leg bends slightly forward at the knee and ends on the ground.
    for ((_, parent, x, z), names) in legs.iter().zip(&names) {
        bones.push((&names[0], Some(*parent), Vec3::new(*x, 0.55, *z)));
        bones.push((&names[1], Some(&names[0]), Vec3::new(*x, 0.3, *z + 0.03)));
        bones.push((&names[2], Some(&names[1]), Vec3::new(*x, 0.02, *z)));
    }
    Skeleton::from_positions(bones)
}

/// A torso with seven radial three-segment legs, three raised arms and a head.
fn alien() -> Skeleton {
    // Seven leg chains come first, then three arm chains.
    let limb = |kind: &str, number: usize| ["a", "b", "c"].map(|s| format!("{kind}_{number}_{s}"));
    let names = (1..=7)
        .map(|leg| limb("leg", leg))
        .chain((1..=3).map(|arm| limb("arm", arm)))
        .collect::<Vec<_>>();
    let mut bones = vec![
        ("torso", None, Vec3::new(0.0, 0.8, 0.0)),
        ("upper_torso", Some("torso"), Vec3::new(0.0, 1.1, 0.0)),
        ("head", Some("upper_torso"), Vec3::new(0.0, 1.4, 0.0)),
    ];
    // Legs spread around the torso to the ground; arms rise from the upper torso.
    for (index, chain) in names.iter().enumerate() {
        let (parent, count, base_y) = if index < 7 {
            ("torso", 7.0, 0.8)
        } else {
            ("upper_torso", 3.0, 1.2)
        };
        let angle = float(index) / count * std::f32::consts::TAU;
        let out = Vec3::new(angle.cos(), 0.0, angle.sin());
        let points = if index < 7 {
            [
                out * 0.15 + Vec3::Y * base_y,
                out * 0.45 + Vec3::Y * 1.0,
                out * 0.75 + Vec3::Y * 0.02,
            ]
        } else {
            [
                out * 0.15 + Vec3::Y * base_y,
                out * 0.35 + Vec3::Y * 1.35,
                out * 0.5 + Vec3::Y * 1.6,
            ]
        };
        bones.push((&chain[0], Some(parent), points[0]));
        bones.push((&chain[1], Some(&chain[0]), points[1]));
        bones.push((&chain[2], Some(&chain[1]), points[2]));
    }
    Skeleton::from_positions(bones)
}

#[test]
fn reference_humanoid_gets_sixteen_named_bodies() {
    // Clavicles, the neck and the balls merge into neighbouring bodies.
    let profile = generate(&Skeleton::humanoid());
    let bones = profile
        .bodies()
        .iter()
        .map(crate::Body::bone)
        .collect::<Vec<_>>();
    assert_eq!(
        bones,
        [
            "pelvis",
            "spine_01",
            "spine_02",
            "upperarm_l",
            "lowerarm_l",
            "hand_l",
            "upperarm_r",
            "lowerarm_r",
            "hand_r",
            "head",
            "thigh_l",
            "calf_l",
            "foot_l",
            "thigh_r",
            "calf_r",
            "foot_r",
        ]
    );
    // The chest is the second spine bone, where the arms branch.
    assert_eq!(
        profile.body_with_role(BodyRole::Chest),
        profile.body_index("spine_02")
    );
}

#[test]
fn reference_humanoid_weighs_eighty_kilograms() {
    let profile = generate(&Skeleton::humanoid());
    assert!((profile.total_mass().kilograms() - 80.02).abs() < 1.0e-3);
}

#[test]
fn knees_and_elbows_are_hinges_that_flex_in_opposite_directions() {
    let profile = generate(&Skeleton::humanoid());
    let joint = |bone: &str| *profile.joint_of(profile.body_index(bone).unwrap()).unwrap();
    // Every elbow and knee locks twist and side bend.
    let hinges =
        ["calf_l", "calf_r", "lowerarm_l", "lowerarm_r"].map(|bone| joint(bone).is_hinge());
    assert_eq!(hinges, [true; 4]);
    // The knee flexes backward and the elbow forward, so world-space flex differs.
    let knee = joint("calf_l").limits();
    assert!(knee.x.min < -2.0 || knee.x.max > 2.0);
    // Ball joints such as the hip keep a twist range.
    let hip = joint("thigh_l").limits();
    assert!(hip.twist.max > 0.0);
}

#[test]
fn joint_torque_scales_with_total_mass() {
    // Torque scales linearly, so doubling the mass doubles every joint torque.
    let light = generate(&Skeleton {
        mass: Mass::try_from(40.0).ok(),
        ..Skeleton::humanoid()
    });
    let heavy = generate(&Skeleton {
        mass: Mass::try_from(80.0).ok(),
        ..Skeleton::humanoid()
    });
    let torque = |profile: &RagdollProfile| profile.joints()[0].max_torque();
    assert!((torque(&heavy) - 2.0 * torque(&light)).abs() < 1.0e-3);
}

#[test]
fn humanoid_mass_follows_the_segment_table() {
    // Without a requested total, only the table ratios are fixed.
    let profile = generate(&Skeleton {
        mass: None,
        ..Skeleton::humanoid()
    });
    let mass = |bone: &str| {
        profile.bodies()[profile.body_index(bone).unwrap().get()]
            .mass()
            .kilograms()
    };
    assert!((mass("thigh_l") / mass("calf_l") - 11.33 / 3.46).abs() < 1.0e-4);
}

#[test]
fn unnormalized_mass_comes_from_capsule_volume() {
    // A non-humanoid without a total mass keeps the volume-derived mass.
    let profile = generate(&quadruped());
    let body = &profile.bodies()[profile.body_index("leg_fl_1").unwrap().get()];
    let ShapeSpec::Capsule { a, b, radius } = body.shape() else {
        panic!("generated bodies are capsules");
    };
    // Capsule volume is a cylinder plus one sphere, at 985 kg/m^3.
    let volume = std::f32::consts::PI * radius * radius * (a.distance(*b) + 4.0 / 3.0 * radius);
    assert!((body.mass().kilograms() - volume * 985.0).abs() < 1.0e-3);
}

#[test]
fn names_normalize_across_conventions() {
    // Each case covers one prefix, separator or side convention.
    let names = [
        "mixamorig:LeftForeArm",
        "mixamorig1_RightUpLeg",
        "DEF-upper_arm.L.001",
        "Armature|thigh_r",
        "l_hand",
        "leftUpperArm",
        "spine_01",
        "l",
    ];
    let expected = [
        ("forearm", Some(Side::Left)),
        ("upleg", Some(Side::Right)),
        ("upperarm", Some(Side::Left)),
        ("thigh", Some(Side::Right)),
        ("hand", Some(Side::Left)),
        ("upperarm", Some(Side::Left)),
        ("spine", None),
        ("l", None),
    ]
    .map(|(base, side)| (base.to_owned(), side));
    assert_eq!(names.map(normalize), expected);
}

#[test]
fn mixamo_leg_is_the_lower_leg_and_unsided_limbs_do_not_match() {
    // Limb aliases need a side; the hips and head do not.
    let roles = ["mixamorig:LeftLeg", "mixamorig:Hips", "leg", "HeadTop_End"].map(role_of);
    let expected = [
        Some((BodyRole::Calf, Some(Side::Left))),
        Some((BodyRole::Pelvis, None)),
        None,
        None,
    ];
    assert_eq!(roles, expected);
}

/// Mixamo names for the reference humanoid bones; `ball_r` falls back to `RightToeBase`.
const MIXAMO: &[(&str, &str)] = &[
    ("pelvis", "Hips"),
    ("spine_01", "Spine"),
    ("spine_02", "Spine1"),
    ("spine_03", "Spine2"),
    ("neck_01", "Neck"),
    ("head", "Head"),
    ("clavicle_l", "LeftShoulder"),
    ("upperarm_l", "LeftArm"),
    ("lowerarm_l", "LeftForeArm"),
    ("hand_l", "LeftHand"),
    ("clavicle_r", "RightShoulder"),
    ("upperarm_r", "RightArm"),
    ("lowerarm_r", "RightForeArm"),
    ("hand_r", "RightHand"),
    ("thigh_l", "LeftUpLeg"),
    ("calf_l", "LeftLeg"),
    ("foot_l", "LeftFoot"),
    ("ball_l", "LeftToeBase"),
    ("thigh_r", "RightUpLeg"),
    ("calf_r", "RightLeg"),
    ("foot_r", "RightFoot"),
];

/// Unity humanoid names that differ from the reference names.
const UNITY: &[(&str, &str)] = &[
    ("pelvis", "Hips"),
    ("upperarm_l", "LeftUpperArm"),
    ("lowerarm_l", "LeftLowerArm"),
    ("upperarm_r", "RightUpperArm"),
    ("lowerarm_r", "RightLowerArm"),
    ("thigh_l", "LeftUpperLeg"),
    ("calf_l", "LeftLowerLeg"),
    ("thigh_r", "RightUpperLeg"),
    ("calf_r", "RightLowerLeg"),
];

/// Rigify deform names that a suffix rewrite cannot derive.
const RIGIFY: &[(&str, &str)] = &[
    ("pelvis", "DEF-spine"),
    ("upperarm_l", "DEF-upper_arm.L"),
    ("lowerarm_l", "DEF-forearm.L"),
    ("calf_l", "DEF-shin.L"),
];

/// Returns the name `table` maps `name` to, if any.
fn lookup(table: &[(&str, &'static str)], name: &str) -> Option<&'static str> {
    table
        .iter()
        .find(|(from, _)| *from == name)
        .map(|(_, to)| *to)
}

#[test]
fn every_convention_yields_the_same_humanoid_layout() {
    let reference = roles(&generate(&Skeleton::humanoid()))
        .into_iter()
        .map(|(_, role)| role)
        .collect::<Vec<_>>();
    // Rename through each convention's table; unlisted bones keep a derived name.
    let mixamo = |name: &str| {
        format!(
            "mixamorig:{}",
            lookup(MIXAMO, name).unwrap_or("RightToeBase")
        )
    };
    let unity = |name: &str| lookup(UNITY, name).unwrap_or(name).to_owned();
    // Rigify has no hips name: the hips come from where the thighs meet.
    let rigify = |name: &str| {
        lookup(RIGIFY, name).map_or_else(
            || format!("DEF-{}", name.replace("_l", ".L").replace("_r", ".R")),
            str::to_owned,
        )
    };
    // Every convention must produce the reference role sequence.
    let generated = [renamed(mixamo), renamed(unity), renamed(rigify)].map(|skeleton| {
        roles(&generate(&skeleton))
            .into_iter()
            .map(|(_, role)| role)
            .collect::<Vec<_>>()
    });
    assert_eq!(generated, [reference.clone(), reference.clone(), reference]);
}

#[test]
fn a_missing_limb_falls_back_to_topology() {
    // Without a right hand the preset fails, and topology still finds the limbs.
    let skeleton = renamed(|name| name.replace("hand_r", "paw"));
    let profile = generate(&skeleton);
    assert_eq!(profile.bodies()[0].role(), BodyRole::Pelvis);
    assert!(
        profile.body_index("clavicle_l").is_some(),
        "topology keeps clavicles"
    );
    let wanted = [BodyRole::Thigh, BodyRole::UpperArm, BodyRole::Head];
    assert_eq!(counts(&profile, wanted), [2, 2, 1]);
}

#[test]
fn quadruped_gets_four_legs_a_head_and_a_tail() {
    // Ground contact makes legs; the backward chain makes a tail.
    let profile = generate(&quadruped());
    let wanted = [
        BodyRole::Thigh,
        BodyRole::Calf,
        BodyRole::Foot,
        BodyRole::Tail,
        BodyRole::Neck,
        BodyRole::Head,
    ];
    assert_eq!(counts(&profile, wanted), [4, 4, 4, 3, 1, 1]);
    assert_eq!(
        profile.bodies()[profile.body_index("chest").unwrap().get()].role(),
        BodyRole::Chest
    );
}

#[test]
fn alien_gets_seven_legs_three_arms_and_a_head() {
    // Three torso bodies plus ten three-segment limbs.
    let profile = generate(&alien());
    assert_eq!(profile.bodies().len(), 3 + 30);
    let wanted = [
        BodyRole::Thigh,
        BodyRole::Foot,
        BodyRole::UpperArm,
        BodyRole::Hand,
        BodyRole::Head,
    ];
    assert_eq!(counts(&profile, wanted), [7, 7, 3, 3, 1]);
}

#[test]
fn a_creature_of_only_legs_has_no_spine() {
    // Four two-segment legs radiate from one body and reach the ground.
    let mut bones = vec![("body", None, Vec3::new(0.0, 0.5, 0.0))];
    let names = (0..4)
        .map(|i| [format!("a{i}"), format!("b{i}")])
        .collect::<Vec<_>>();
    for (i, [a, b]) in names.iter().enumerate() {
        let out = Vec3::new(float(i).cos(), 0.0, float(i).sin());
        bones.push((
            a.as_str(),
            Some("body"),
            Vec3::new(0.0, 0.5, 0.0) + out * 0.2,
        ));
        bones.push((b.as_str(), Some(a.as_str()), out * 0.6));
    }
    // No unique largest child exists, so no spine starts at the core.
    let profile = generate(&Skeleton::from_positions(bones));
    let wanted = [BodyRole::Spine, BodyRole::Thigh, BodyRole::Calf];
    assert_eq!(counts(&profile, wanted), [0, 4, 4]);
}

#[test]
fn a_snake_is_a_spine_chain() {
    // Six segments in a straight horizontal chain.
    let names = (0..6).map(|i| format!("segment_{i}")).collect::<Vec<_>>();
    let bones = names.iter().enumerate().map(|(i, name)| {
        let parent = i.checked_sub(1).map(|p| names[p].as_str());
        (name.as_str(), parent, Vec3::new(0.0, 0.05, float(i) * 0.2))
    });
    // Every body after the root continues the single chain, so it is spine.
    let profile = generate(&Skeleton::from_positions(bones));
    assert_eq!(profile.bodies().len(), 6);
    assert_eq!(count(&profile, BodyRole::Spine), 5);
}

#[test]
fn helper_and_twist_bones_get_no_body() {
    // Re-root the humanoid under a `root` bone and add helper bones.
    let mut skeleton = Skeleton::humanoid();
    skeleton.bones = Skeleton::from_positions(
        std::iter::once(("root", None, Vec3::ZERO))
            .chain(skeleton.bones.iter().map(|bone| {
                let parent = bone
                    .parent
                    .map_or("root", |parent| skeleton.bones[parent].name.as_str());
                (
                    bone.name.as_str(),
                    Some(parent),
                    Vec3::from(bone.rest.translation),
                )
            }))
            .chain([
                ("ik_foot_l", Some("root"), Vec3::new(0.1, 0.08, 0.0)),
                (
                    "upperarm_twist_01_l",
                    Some("upperarm_l"),
                    Vec3::new(0.3, 1.45, 0.0),
                ),
                ("HeadTop_End", Some("head"), Vec3::new(0.0, 1.8, 0.0)),
            ]),
    )
    .bones;
    // The helper root, IK target, twist and end bones add no body.
    let profile = generate(&skeleton);
    assert_eq!(profile.bodies().len(), 16);
    assert_eq!(profile.bodies()[0].bone(), "pelvis");
    // The head capsule reaches toward the merged head-top bone.
    let head = &profile.bodies()[profile.body_index("head").unwrap().get()];
    let ShapeSpec::Capsule { b, radius, .. } = head.shape() else {
        panic!("capsule");
    };
    assert!(b.length() + radius > 0.2);
}

#[test]
fn topology_merges_short_bones_into_their_parent() {
    // The finger bones are short relative to the 1 m skeleton.
    let skeleton = Skeleton::from_positions([
        ("a", None, Vec3::new(0.0, 1.0, 0.0)),
        ("b", Some("a"), Vec3::new(0.0, 0.5, 0.0)),
        ("finger", Some("b"), Vec3::new(0.0, 0.0, 0.0)),
        ("tip", Some("finger"), Vec3::new(0.0, -0.01, 0.0)),
    ]);
    let profile = generate(&skeleton);
    let bones = profile
        .bodies()
        .iter()
        .map(crate::Body::bone)
        .collect::<Vec<_>>();
    assert_eq!(bones, ["a", "b"]);
    // The leaf body reaches its farthest merged bone.
    let ShapeSpec::Capsule { b, radius, .. } = profile.bodies()[1].shape() else {
        panic!("capsule");
    };
    assert!(b.length() + radius > 0.45);
}

#[test]
fn oversized_skeletons_raise_the_minimum_segment() {
    let names = (0..100).map(|i| format!("b{i}")).collect::<Vec<_>>();
    let bones = names.iter().enumerate().map(|(i, name)| {
        let parent = i.checked_sub(1).map(|p| names[p].as_str());
        (name.as_str(), parent, Vec3::new(0.0, float(i) * 0.01, 0.0))
    });
    let profile = generate(&Skeleton::from_positions(bones));
    assert!(profile.bodies().len() <= crate::MAX_BODIES);
}

#[test]
fn extra_roots_keep_only_the_largest_tree() {
    let skeleton = Skeleton::from_positions([
        ("prop_a", None, Vec3::new(2.0, 0.0, 0.0)),
        ("hips", None, Vec3::new(0.0, 1.0, 0.0)),
        ("chest", Some("hips"), Vec3::new(0.0, 1.5, 0.0)),
        ("lone", None, Vec3::new(3.0, 1.0, 0.0)),
    ]);
    let profile = generate(&skeleton);
    let bones = profile
        .bodies()
        .iter()
        .map(crate::Body::bone)
        .collect::<Vec<_>>();
    assert_eq!(bones, ["hips", "chest"]);
}

#[test]
fn an_empty_or_skipped_skeleton_is_rejected() {
    assert_eq!(
        RagdollProfile::from_skeleton(&Skeleton::default()),
        Err(ProfileError::Empty)
    );
    let mut skeleton = quadruped();
    skeleton.bones[0].overrides.body = BoneBody::Skip;
    assert_eq!(
        RagdollProfile::from_skeleton(&skeleton),
        Err(ProfileError::Empty)
    );
}

/// Joint limits used by the value-override tests.
const OVERRIDE_LIMITS: JointLimits = JointLimits {
    x: AngleRange {
        min: -0.1,
        max: 0.1,
    },
    twist: AngleRange { min: 0.0, max: 0.0 },
    z: AngleRange {
        min: -0.2,
        max: 0.2,
    },
};

/// The reference humanoid with `bone` set on the bone named `name`.
fn with_override(name: &str, bone: RagdollBone) -> Skeleton {
    let mut skeleton = Skeleton::humanoid();
    let index = skeleton.bone_index(name).unwrap();
    skeleton.bones[index].overrides = bone;
    skeleton
}

/// The reference humanoid whose head overrides every value.
fn head_override_profile() -> RagdollProfile {
    generate(&with_override(
        "head",
        RagdollBone {
            mass: Some(9.0),
            radius: Some(0.2),
            limits: Some(OVERRIDE_LIMITS),
            max_torque: Some(77.0),
            role: Some(BodyRole::Other),
            ..default()
        },
    ))
}

#[test]
fn body_overrides_skip_and_force_bodies() {
    // Skipping the left hand and forcing the neck keeps the body count.
    let mut skeleton = with_override(
        "hand_l",
        RagdollBone {
            body: BoneBody::Skip,
            ..default()
        },
    );
    let neck = skeleton.bone_index("neck_01").unwrap();
    skeleton.bones[neck].overrides.body = BoneBody::Body;
    let profile = generate(&skeleton);
    let present = ["hand_l", "neck_01"].map(|bone| profile.body_index(bone).is_some());
    assert_eq!(present, [false, true]);
    assert_eq!(profile.bodies().len(), 16);
}

#[test]
fn value_overrides_replace_body_values() {
    // Mass, role and radius come straight from the override.
    let profile = head_override_profile();
    let body = &profile.bodies()[profile.body_index("head").unwrap().get()];
    let ShapeSpec::Capsule { radius, .. } = body.shape() else {
        panic!("generated bodies are capsules");
    };
    assert_eq!(
        (body.mass().kilograms(), body.role(), *radius),
        (9.0, BodyRole::Other, 0.2)
    );
}

#[test]
fn value_overrides_replace_joint_values() {
    // Limits and torque come straight from the override.
    let profile = head_override_profile();
    let joint = profile
        .joint_of(profile.body_index("head").unwrap())
        .unwrap();
    assert_eq!(
        (joint.limits(), joint.max_torque()),
        (OVERRIDE_LIMITS, 77.0)
    );
}

#[test]
fn a_mass_override_keeps_the_requested_total() {
    // The other bodies absorb the difference, so the total is unchanged.
    let profile = head_override_profile();
    assert!((profile.total_mass().kilograms() - 80.02).abs() < 1.0e-3);
}

#[test]
fn merge_override_removes_a_topology_body() {
    // A merged neck leaves the skull as the head body on the chest.
    let mut skeleton = quadruped();
    let neck = skeleton.bone_index("neck").unwrap();
    skeleton.bones[neck].overrides.body = BoneBody::Merge;
    let profile = generate(&skeleton);
    assert!(profile.body_index("neck").is_none());
    assert_eq!(count(&profile, BodyRole::Head), 1);
}

#[test]
fn override_assets_prefer_exact_keys_then_longer_prefixes() {
    // Three overlapping keys of increasing specificity.
    let bone = |radius| RagdollBone {
        radius: Some(radius),
        ..default()
    };
    let overrides = RagdollOverrides {
        mass: None,
        bones: [
            ("tail*".to_owned(), bone(1.0)),
            ("tail_1*".to_owned(), bone(2.0)),
            ("tail_12".to_owned(), bone(3.0)),
        ]
        .into(),
    };
    // The radius identifies which key matched each bone name.
    let radii = ["tail_12", "tail_13", "tail_2", "ear"]
        .map(|bone| overrides.get(bone).and_then(|bone| bone.radius));
    assert_eq!(radii, [Some(3.0), Some(2.0), Some(1.0), None]);
}

#[test]
fn a_mass_override_that_exceeds_the_total_keeps_volume_masses() {
    let mut skeleton = Skeleton::humanoid();
    skeleton.bones[0].overrides.mass = Some(500.0);
    let profile = generate(&skeleton);
    assert!(profile.total_mass().kilograms() > 500.0);
}

#[test]
fn from_positions_points_bones_at_their_first_child() {
    let skeleton = Skeleton::from_positions([
        ("a", None, Vec3::ZERO),
        ("b", Some("a"), Vec3::X),
        ("c", Some("missing"), Vec3::Z),
    ]);
    // `b` is a leaf that follows its parent link; `c` is a root with no children.
    let along = |index: usize| skeleton.bones[index].rest.rotation * Vec3::Y;
    let errors = [(0, Vec3::X), (1, Vec3::X), (2, Vec3::Y)]
        .map(|(index, expected)| along(index).distance(expected) < 1.0e-5);
    assert_eq!(errors, [true; 3]);
    assert_eq!(skeleton.bones[2].parent, None);
}

/// Shorthand for `Default::default()` in struct update syntax.
fn default<T: Default>() -> T {
    T::default()
}

#[test]
fn x_along_bone_rigs_get_the_same_limits_through_a_basis() {
    // UE-style bones: local X points along the bone instead of local Y.
    let quarter = Quat::from_rotation_z(std::f32::consts::FRAC_PI_2);
    let y_rig = Skeleton::humanoid();
    let mut x_rig = y_rig.clone();
    x_rig
        .bones
        .iter_mut()
        .for_each(|bone| bone.rest.rotation *= quarter);
    let (y_profile, x_profile) = (generate(&y_rig), generate(&x_rig));
    // Any world-space pose measures the same joint angles on both rigs.
    let poses = |profile: &RagdollProfile| {
        profile
            .rest_poses(Isometry3d::IDENTITY)
            .enumerate()
            .map(|(index, mut pose)| {
                let bend = Quat::from_rotation_x(0.3 * float(index));
                pose.rotation = bend * pose.rotation;
                pose
            })
            .collect::<Vec<_>>()
    };
    let (y_poses, x_poses) = (poses(&y_profile), poses(&x_profile));
    // Collect every joint whose basis, limits or measured angles differ.
    let close = |a: f32, b: f32| (a - b).abs() < 1.0e-4;
    let same_range = |a: AngleRange, b: AngleRange| close(a.min, b.min) && close(a.max, b.max);
    let mismatches = y_profile
        .joints()
        .iter()
        .zip(x_profile.joints())
        .filter(|(y_joint, x_joint)| {
            let (y_limits, x_limits) = (y_joint.limits(), x_joint.limits());
            let child = y_joint.child();
            let y_angles = y_profile.joint_angles(child, &y_poses).unwrap();
            let x_angles = x_profile.joint_angles(child, &x_poses).unwrap();
            y_joint.basis() != Quat::IDENTITY
                || x_joint.basis().angle_between(Quat::IDENTITY) <= 1.0
                || !same_range(y_limits.x, x_limits.x)
                || !same_range(y_limits.twist, x_limits.twist)
                || !same_range(y_limits.z, x_limits.z)
                || y_angles.distance(x_angles) >= 1.0e-4
        })
        .map(|(y_joint, _)| y_joint.child())
        .collect::<Vec<_>>();
    assert_eq!(mismatches, []);
}

#[test]
fn twist_basis_maps_y_onto_the_nearest_signed_axis() {
    use super::generate::twist_basis;
    // A slightly skewed direction still picks the nearest signed axis.
    let mapped = [Vec3::X, Vec3::NEG_X, Vec3::Z, Vec3::NEG_Z, Vec3::NEG_Y].map(|axis| {
        (twist_basis(axis * 0.9 + Vec3::splat(0.05)) * Vec3::Y).distance(axis) < 1.0e-5
    });
    assert_eq!(mapped, [true; 5]);
    assert_eq!(twist_basis(Vec3::Y), Quat::IDENTITY);
}

/// Converts a small test index to `f32` without a lossy cast.
fn float(index: usize) -> f32 {
    f32::from(u16::try_from(index).unwrap())
}
