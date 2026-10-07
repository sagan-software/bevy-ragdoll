//! Rapier-facing hit response and fully limp physical behavior checks.

use bevy::math::{Isometry3d, Vec3};
use bevy::prelude::Entity;
use bevy_ragdoll::profile::BodyIndex;
use bevy_ragdoll::runtime::components::{BodyWeights, RagdollBodyWeights, RagdollDrive};
use bevy_ragdoll::runtime::messages::{HitKind, RagdollHit};
use bevy_ragdoll::runtime::pin::PinTargets;

use super::{
    PhysicsBackend, PhysicsScene, ScenarioSetup, body_index, human_profile, lowest_point, scenario,
    snapshots,
};

/// Runs the shared pistol-to-chest physics check for a standing pinned human.
/// It asserts limited pelvis motion, chest rotation, and muscle recovery within
/// one second using backend-neutral body snapshots.
///
/// # Examples
///
/// ```no_run
/// use bevy::math::{Isometry3d, Vec3};
/// use bevy::prelude::{App, Entity};
/// use bevy_ragdoll::ShapeSpec;
/// use bevy_ragdoll_conformance::physics::{
///     PhysicsBackend, pistol_to_the_chest_does_not_move_the_pelvis_far,
/// };
///
/// let backend = PhysicsBackend::new(
///     |_| {},
///     |app: &mut App, _: ShapeSpec, _: Isometry3d| app.world_mut().spawn_empty().id(),
///     |_: &mut App, _: Entity, _: Vec3| 0.0,
/// );
/// pistol_to_the_chest_does_not_move_the_pelvis_far(backend);
/// ```
pub fn pistol_to_the_chest_does_not_move_the_pelvis_far(backend: PhysicsBackend) {
    let mut scene = standing_scene(backend);
    // Settle the standing rig before recording the comparison pose.
    settle_scene(&mut scene);
    // Capture body identity and pose from the checked parent-first profile.
    let baseline = pistol_chest_baseline(&mut scene);
    // Apply one chest impulse, then sample the peak response and recovery state.
    let response = measure_pistol_chest_response(&mut scene, baseline);
    // Verify the backend-neutral thresholds from the hit-reaction plan.
    assert_pistol_chest_result(response);
}

/// Profile positions and pre-hit poses needed to measure a chest pistol hit.
struct PistolChestBaseline {
    /// Checked profile position of the pelvis body.
    pelvis_index: BodyIndex,
    /// Checked profile position of the chest body.
    chest_index: BodyIndex,
    /// Pelvis pose before the hit in world coordinates.
    pelvis_pose: Isometry3d,
    /// Chest body entity addressed by the hit message.
    chest_entity: Entity,
    /// Chest pose before the hit in world coordinates.
    chest_pose: Isometry3d,
}

/// Peak backend-neutral pelvis, chest, and recovery values after a pistol hit.
struct PistolChestResponse {
    /// Maximum pelvis displacement in metres during the response window.
    pelvis_displacement: f32,
    /// Maximum chest rotation in radians during the response window.
    chest_rotation: f32,
    /// Whether chest muscle strength reached the recovery threshold.
    has_recovered_muscle: bool,
}

/// Captures validated body positions and world poses before the pistol hit.
fn pistol_chest_baseline(scene: &mut PhysicsScene) -> PistolChestBaseline {
    // Resolve the profile bodies before requesting indexed backend snapshots.
    let pelvis_index = body_index(&scene.profile, "pelvis");
    let chest_index = body_index(&scene.profile, "spine_03");
    // Copy the pre-hit values so later simulation updates cannot alter the baseline.
    let before = snapshots(scene.app.world_mut(), scene.character);
    let pelvis_pose = body_pose(&before, pelvis_index);
    let chest = *body_snapshot(&before, chest_index);
    PistolChestBaseline {
        pelvis_index,
        chest_index,
        pelvis_pose,
        chest_entity: chest.entity,
        chest_pose: chest.pose,
    }
}

/// Applies the chest impulse and records peak body motion across sixty fixed steps.
fn measure_pistol_chest_response(
    scene: &mut PhysicsScene,
    baseline: PistolChestBaseline,
) -> PistolChestResponse {
    // Address the chest at a point above its centre to produce a rotational response.
    scene.app.world_mut().write_message(RagdollHit {
        body: baseline.chest_entity,
        point: Vec3::from(baseline.chest_pose.translation) + Vec3::Y * 0.06,
        impulse: Vec3::NEG_Z * 12.0,
        kind: HitKind::Impact,
    });

    // Track maximum displacement and rotation while recovery advances.
    let mut response = PistolChestResponse {
        pelvis_displacement: 0.0,
        chest_rotation: 0.0,
        has_recovered_muscle: false,
    };
    for _ in 0..60 {
        super::restore_targets(scene);
        scene.app.update();
        let current = snapshots(scene.app.world_mut(), scene.character);
        response.pelvis_displacement = response.pelvis_displacement.max(
            body_pose(&current, baseline.pelvis_index)
                .translation
                .distance(baseline.pelvis_pose.translation),
        );
        response.chest_rotation = response.chest_rotation.max(rotation_delta(
            baseline.chest_pose.rotation,
            body_pose(&current, baseline.chest_index).rotation,
        ));

        // Preserve a successful recovery observation across later fixed steps.
        response.has_recovered_muscle |= scene
            .app
            .world()
            .get::<RagdollBodyWeights>(scene.character)
            .and_then(|weights| weights.get(baseline.chest_index.get()))
            .is_some_and(|weights| weights.muscle() >= 0.95);
    }
    response
}

/// Checks the pelvis, chest, and muscle-recovery thresholds for the pistol scenario.
fn assert_pistol_chest_result(response: PistolChestResponse) {
    assert!(
        response.pelvis_displacement < 0.08,
        "pelvis moved {} m",
        response.pelvis_displacement
    );
    assert!(
        response.chest_rotation >= 4.0_f32.to_radians(),
        "chest rotated {} degrees",
        response.chest_rotation.to_degrees()
    );
    assert!(
        response.has_recovered_muscle,
        "chest muscle did not recover to 0.95 within 1 s"
    );
}

/// Runs the headshot physics check and measures neck-proxy and head rotation.
/// The rig uses `spine_04` as the head parent because its profile has no
/// separate neck body.
///
/// # Examples
///
/// ```no_run
/// use bevy::math::{Isometry3d, Vec3};
/// use bevy::prelude::{App, Entity};
/// use bevy_ragdoll::ShapeSpec;
/// use bevy_ragdoll_conformance::physics::{PhysicsBackend, headshot_turns_the_head};
///
/// let backend = PhysicsBackend::new(
///     |_| {},
///     |app: &mut App, _: ShapeSpec, _: Isometry3d| app.world_mut().spawn_empty().id(),
///     |_: &mut App, _: Entity, _: Vec3| 0.0,
/// );
/// headshot_turns_the_head(backend);
/// ```
pub fn headshot_turns_the_head(backend: PhysicsBackend) {
    let mut scene = standing_scene(backend);
    // Settle the standing rig before recording the relative rotations.
    settle_scene(&mut scene);
    // Measure both child joints against their pre-hit orientations.
    let response = measure_headshot_response(&mut scene);
    // Require visible rotation in at least one measured head-chain segment.
    assert!(
        response.peak_degrees >= 8.0,
        "neck and head turned {} degrees",
        response.peak_degrees
    );
}

/// Removes all per-body muscle and pin strength, then checks that the pelvis
/// falls below 0.4 metres within 1.5 seconds of simulated physics.
///
/// # Examples
///
/// ```no_run
/// use bevy::math::{Isometry3d, Vec3};
/// use bevy::prelude::{App, Entity};
/// use bevy_ragdoll::ShapeSpec;
/// use bevy_ragdoll_conformance::physics::{
///     PhysicsBackend, limp_weights_make_a_powered_ragdoll_collapse,
/// };
///
/// let backend = PhysicsBackend::new(
///     |_| {},
///     |app: &mut App, _: ShapeSpec, _: Isometry3d| app.world_mut().spawn_empty().id(),
///     |_: &mut App, _: Entity, _: Vec3| 0.0,
/// );
/// limp_weights_make_a_powered_ragdoll_collapse(backend);
/// ```
pub fn limp_weights_make_a_powered_ragdoll_collapse(backend: PhysicsBackend) {
    let mut scene = standing_scene(backend);
    // Preserve muscle drive while setting the whole-body pin strength to zero.
    {
        let mut drive = scene
            .app
            .world_mut()
            .get_mut::<RagdollDrive>(scene.character)
            .expect("the standing scenario inserts its drive component");
        drive.set(1.0, 0.0);
    }

    // Set both per-body multipliers to zero so no body receives active support.
    {
        let mut weights = scene
            .app
            .world_mut()
            .get_mut::<RagdollBodyWeights>(scene.character)
            .expect("the standing scenario initializes per-body weights");
        for body in scene.profile.bodies() {
            weights.set(body.index(), BodyWeights::new(0.0, 0.0));
        }
    }

    // Run ninety fixed updates and measure the final pelvis height.
    for _ in 0..90 {
        scene.app.update();
    }
    let pelvis_index = body_index(&scene.profile, "pelvis");
    let pelvis_height = body_pose(
        &snapshots(scene.app.world_mut(), scene.character),
        pelvis_index,
    )
    .translation
    .y;
    assert!(
        pelvis_height < 0.4,
        "limp pelvis stayed at {pelvis_height} m"
    );
}

/// Relative peak rotation observed in the neck-proxy and head joints.
struct HeadshotResponse {
    /// Greater of the neck-proxy and head rotation peaks in degrees.
    peak_degrees: f32,
}

/// Captures the head chain, applies an impulse, and samples nine fixed steps.
fn measure_headshot_response(scene: &mut PhysicsScene) -> HeadshotResponse {
    // Resolve the head chain; spine_04 is the head's parent segment.
    let neck_proxy_index = body_index(&scene.profile, "spine_03");
    let upper_spine_index = body_index(&scene.profile, "spine_02");
    let head_index = body_index(&scene.profile, "head");
    // Save relative poses before publishing the hit message.
    let before = snapshots(scene.app.world_mut(), scene.character);
    let head = *body_snapshot(&before, head_index);
    let neck_relative = relative_pose(&before, upper_spine_index, neck_proxy_index);
    let head_relative = relative_pose(&before, neck_proxy_index, head_index);
    scene.app.world_mut().write_message(RagdollHit {
        body: head.entity,
        point: Vec3::from(head.pose.translation) + Vec3::Y * 0.20,
        impulse: Vec3::NEG_Z * 12.0,
        kind: HitKind::Impact,
    });

    sample_headshot_rotations(
        scene,
        upper_spine_index,
        neck_proxy_index,
        head_index,
        neck_relative,
        head_relative,
    )
}

/// Tracks each head-chain joint's relative rotation across nine fixed steps.
fn sample_headshot_rotations(
    scene: &mut PhysicsScene,
    upper_spine_index: BodyIndex,
    neck_proxy_index: BodyIndex,
    head_index: BodyIndex,
    neck_relative: bevy::math::Quat,
    head_relative: bevy::math::Quat,
) -> HeadshotResponse {
    // Accumulate each joint's maximum angular difference through the response window.
    let mut neck_rotation = 0.0_f32;
    let mut head_rotation = 0.0_f32;
    // Restore animation targets before each backend fixed-step update.
    for _ in 0..9 {
        super::restore_targets(scene);
        scene.app.update();
        let current = snapshots(scene.app.world_mut(), scene.character);
        // Compare parent-relative orientations so root motion cannot inflate the angles.
        neck_rotation = neck_rotation.max(relative_rotation_delta(
            neck_relative,
            relative_pose(&current, upper_spine_index, neck_proxy_index),
        ));
        head_rotation = head_rotation.max(relative_rotation_delta(
            head_relative,
            relative_pose(&current, neck_proxy_index, head_index),
        ));
    }
    HeadshotResponse {
        peak_degrees: neck_rotation.max(head_rotation).to_degrees(),
    }
}

/// Advances a scene while restoring its procedural animation targets each tick.
fn settle_scene(scene: &mut PhysicsScene) {
    // Give the pinned rig thirty fixed updates to reach its standing pose.
    for _ in 0..30 {
        super::restore_targets(scene);
        scene.app.update();
    }
}

/// Builds a floor-supported human rig with core-only pin targets.
fn standing_scene(backend: PhysicsBackend) -> PhysicsScene {
    // Reuse the checked human profile and its parent-first rest-pose order.
    let profile = human_profile();
    let bone_poses = profile
        .bodies()
        .iter()
        .map(|body| body.rest())
        .collect::<Vec<_>>();
    let lowest = profile
        .bodies()
        .iter()
        .zip(&bone_poses)
        .map(|(body, pose)| lowest_point(*body.shape(), *pose))
        .fold(f32::INFINITY, f32::min);
    // Place the profile just above the floor using the lowest transformed shape point.
    let root = Isometry3d::from_translation(Vec3::Y * (0.01 - lowest));
    let fixed_shapes = [super::floor_shape(Vec3::new(51.2, 0.8, 51.2))];
    // Build a dynamic, pinned scene using backend-neutral floor geometry.
    let mut scene = scenario(
        backend,
        super::scenario_settings(Vec3::new(0.0, -9.81, 0.0), true, 0.5),
        profile,
        ScenarioSetup {
            root,
            muscle: 1.0,
            pin: 1.0,
            mode: super::RagdollMode::Dynamic,
            bone_poses,
            initial_velocity: Vec3::ZERO,
            fixed_shapes: &fixed_shapes,
        },
    );
    let pelvis = body_index(&scene.profile, "pelvis");
    let chest = body_index(&scene.profile, "spine_03");
    // Restrict world-space pins to the pelvis and chest bodies.
    scene
        .app
        .world_mut()
        .get_entity_mut(scene.character)
        .expect("standing scenario retains its character entity")
        .insert(PinTargets::only([pelvis, chest]));
    scene
}

/// Returns one profile body snapshot or fails if the bound profile is incomplete.
fn body_snapshot(bodies: &[super::BodySnapshot], index: BodyIndex) -> &super::BodySnapshot {
    bodies
        .get(index.get())
        .expect("the body snapshot follows the profile's validated order")
}

/// Copies one body pose from the profile-ordered snapshot.
fn body_pose(bodies: &[super::BodySnapshot], index: BodyIndex) -> Isometry3d {
    body_snapshot(bodies, index).pose
}

/// Returns one body orientation relative to a checked parent body.
fn relative_pose(
    bodies: &[super::BodySnapshot],
    parent: BodyIndex,
    child: BodyIndex,
) -> bevy::math::Quat {
    body_pose(bodies, parent).rotation.inverse() * body_pose(bodies, child).rotation
}

/// Returns the shortest quaternion angular distance in radians.
fn rotation_delta(before: bevy::math::Quat, after: bevy::math::Quat) -> f32 {
    (after * before.inverse()).to_axis_angle().1
}

/// Returns the angular difference between two parent-relative body rotations.
fn relative_rotation_delta(before: bevy::math::Quat, after: bevy::math::Quat) -> f32 {
    rotation_delta(before, after)
}
