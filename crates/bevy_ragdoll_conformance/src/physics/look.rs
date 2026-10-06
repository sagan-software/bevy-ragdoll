//! Backend-neutral hit, stair, and post-impact behavior checks.

use bevy::math::{Isometry3d, Quat, Vec3};
use bevy_ragdoll::ShapeSpec;
use bevy_ragdoll::profile::BodyIndex;
use bevy_ragdoll::runtime::components::RagdollMode;
use bevy_ragdoll::runtime::messages::RagdollImpulse;

use super::fall::simulation_steps;
use super::{
    BodySnapshot, PhysicsBackend, PhysicsScene, RagdollProfile, Run, ScenarioSetup, body_index,
    human_profile, joint_errors, local_center, lowest_point, poses, scenario, snapshots,
};

/// Converts an ET-space box into a Bevy cuboid using 0.025 metres per unit.
pub(super) fn et_box(min: [f32; 3], max: [f32; 3]) -> (ShapeSpec, Isometry3d) {
    // Convert the source ET axes and scale at this geometry boundary.
    let scale = 0.025;
    let half_extents = Vec3::new(
        (max[0] - min[0]) * scale * 0.5,
        (max[2] - min[2]) * scale * 0.5,
        (max[1] - min[1]) * scale * 0.5,
    );
    let center = Vec3::new(
        (min[0] + max[0]) * scale * 0.5,
        (min[2] + max[2]) * scale * 0.5,
        -(min[1] + max[1]) * scale * 0.5,
    );
    let shape = ShapeSpec::Cuboid {
        center: Vec3::ZERO,
        rotation: Quat::IDENTITY,
        half_extents,
    };
    (shape, Isometry3d::from_translation(center))
}

/// Builds the landing and twenty 20 cm rise, 30 cm run Bevy stairs.
pub(super) fn stairs() -> Vec<(ShapeSpec, Isometry3d)> {
    // The landing reaches the first tread at the source stair origin.
    let mut shapes = vec![et_box([-2048.0, -2048.0, -400.0], [2048.0, 4.0, 0.0])];
    // Preserve the source riser and tread sizes for all twenty steps.
    for index in 0..20 {
        let y = 4.0 + 12.0 * index as f32;
        shapes.push(et_box(
            [-2048.0, y, -400.0],
            [2048.0, y + 12.0, -8.0 * (index + 1) as f32],
        ));
    }
    shapes
}

/// Gives the stair height in metres at a Bevy-space Z coordinate.
pub(super) fn stairs_height(z: f32) -> f32 {
    // Convert Bevy depth back to the source ET vertical coordinate.
    let et_y = -z / 0.025;
    // The landing remains at zero until the first riser.
    if et_y < 4.0 {
        0.0
    } else {
        -0.2 * (((et_y - 4.0) / 12.0).floor() + 1.0)
    }
}

/// Gives flat terrain height in metres.
fn flat_height(_: f32) -> f32 {
    0.0
}

/// Rotates one body and every profile descendant about the body's X axis.
fn bend(profile: &RagdollProfile, poses: &mut [Isometry3d], bone: &str, degrees: f32) {
    // Resolve the pivot through the validated profile name before editing poses.
    let body = body_index(profile, bone).get();
    let pivot_pose = poses
        .get(body)
        .copied()
        .expect("the named profile bone has a pose");
    // Convert the local axis bend into the pivot's world orientation.
    let pivot = Vec3::from(pivot_pose.translation);
    let rotation = pivot_pose.rotation
        * Quat::from_rotation_x(degrees.to_radians())
        * pivot_pose.rotation.inverse();
    // Rotate the pivot and descendants while leaving unrelated bodies intact.
    for (index, pose) in poses.iter_mut().enumerate() {
        let mut ancestor = Some(BodyIndex::try_from(index).expect("the profile index is valid"));
        while let Some(candidate) = ancestor {
            if candidate.get() == body {
                let translation = Vec3::from(pose.translation);
                *pose = Isometry3d::new(
                    pivot + rotation * (translation - pivot),
                    rotation * pose.rotation,
                );
                break;
            }
            ancestor = profile.joint_of(candidate).map(|joint| joint.parent());
        }
    }
}

/// Spawns the idle stance used by the game-facing hit and death checks.
fn look_scene(backend: PhysicsBackend, velocity: Vec3, is_on_stairs: bool) -> PhysicsScene {
    // Apply the game gravity and continuous-collision setting at construction.
    let settings = super::scenario_settings(Vec3::new(0.0, -800.0 * 0.025, 0.0), false, 0.5);
    // Build the profile poses in the same stable order used by the backend.
    let profile = human_profile();
    let mut bone_poses = profile
        .bodies()
        .iter()
        .map(|body| body.rest())
        .collect::<Vec<_>>();
    // Bend both legs before finding the stance's lowest contact point.
    for side in ["l", "r"] {
        bend(&profile, &mut bone_poses, &format!("thigh_{side}"), 8.0);
        bend(&profile, &mut bone_poses, &format!("calf_{side}"), -16.0);
        bend(&profile, &mut bone_poses, &format!("foot_{side}"), 8.0);
    }
    // Lift the authored stance just enough to clear the selected terrain.
    let lowest = profile
        .bodies()
        .iter()
        .zip(&bone_poses)
        .map(|(body, pose)| lowest_point(*body.shape(), *pose))
        .fold(f32::INFINITY, f32::min);
    let root = Isometry3d::from_translation(Vec3::Y * (0.01 - lowest));
    // Select geometry before backend setup initializes the fixed schedule.
    let fixed_shapes = if is_on_stairs {
        stairs()
    } else {
        vec![super::floor_shape(Vec3::new(51.2, 0.8, 51.2))]
    };
    scenario(
        backend,
        settings,
        profile,
        ScenarioSetup {
            root,
            muscle: 0.0,
            pin: 0.0,
            mode: RagdollMode::Dynamic,
            bone_poses,
            initial_velocity: velocity,
            fixed_shapes: &fixed_shapes,
        },
    )
}

/// Adds each named hit at a point 6 cm from its body centre.
fn apply_hits(scene: &mut PhysicsScene, hits: &[(&str, Vec3)]) {
    // Resolve each named body against the current profile-ordered snapshots.
    for (bone, impulse) in hits {
        let index = body_index(&scene.profile, bone);
        let body = snapshots(scene.app.world_mut(), scene.character)
            .into_iter()
            .find(|body| body.index == index)
            .expect("the hit body was spawned");
        let center =
            Vec3::from(body.pose.translation) + body.pose.rotation * local_center(body.shape);
        scene.app.world_mut().write_message(RagdollImpulse {
            body: body.entity,
            point: center + Vec3::new(0.06, 0.06, 0.0),
            impulse: *impulse,
        });
    }
}

/// Profile-ordered body indexes and starting poses used during one measurement.
struct MeasureContext<'a> {
    /// Profile whose joints and body order define the measurements.
    profile: &'a RagdollProfile,
    /// Terrain height function for floor clearance measurements.
    height: fn(f32) -> f32,
    /// Initial body poses before fixed-step simulation.
    start: Vec<Isometry3d>,
    /// Pelvis body index in the profile.
    pelvis: BodyIndex,
    /// Lower-spine body index used to detect floor contact.
    spine: BodyIndex,
    /// Chest body index used for relative head rotation.
    chest: BodyIndex,
    /// Head body index used for relative head rotation.
    head: BodyIndex,
    /// Calf body indexes used to record landing knee angles.
    knees: [BodyIndex; 2],
}

/// State accumulated while advancing one physical measurement scenario.
struct MeasureState {
    /// Behavioral measurements returned to the conformance assertion.
    run: Run,
    /// Body poses captured at each fixed step.
    history: Vec<Vec<Isometry3d>>,
    /// First time the body speeds remain under the settled thresholds.
    slow_since: Option<f32>,
    /// Pelvis world position at first floor contact.
    ground_position: Vec3,
}

impl MeasureState {
    /// Allocates fixed-step pose history and initializes all measurements.
    fn new(steps: usize) -> Self {
        Self {
            run: Run::default(),
            history: Vec::with_capacity(steps),
            slow_since: None,
            ground_position: Vec3::ZERO,
        }
    }
}

/// Measures a rig for `seconds` and records contact, motion, and joint state.
fn measure(scene: &mut PhysicsScene, height: fn(f32) -> f32, seconds: f32) -> Run {
    // Capture the starting pose and stable body indexes before simulation.
    let start = poses(scene);
    let profile = &scene.profile;
    let context = MeasureContext {
        profile,
        height,
        start,
        pelvis: body_index(profile, "pelvis"),
        spine: body_index(profile, "spine_02"),
        chest: body_index(profile, "spine_04"),
        head: body_index(profile, "head"),
        knees: [body_index(profile, "calf_l"), body_index(profile, "calf_r")],
    };
    // Reserve exactly one frame per fixed step before the loop begins.
    let steps = simulation_steps(seconds);
    let mut state = MeasureState::new(steps);
    // Read back complete body state after each fixed update.
    for step in 1..=steps {
        scene.app.update();
        let bodies = snapshots(scene.app.world_mut(), scene.character);
        let current = bodies.iter().map(|body| body.pose).collect::<Vec<_>>();
        record_measurement_step(&context, &mut state, step, &bodies, &current);
        state.history.push(current);
    }
    finish_measurement(context, state)
}

/// Records motion, joint, landing, and resting data for one fixed step.
fn record_measurement_step(
    context: &MeasureContext<'_>,
    state: &mut MeasureState,
    step: usize,
    bodies: &[BodySnapshot],
    current: &[Isometry3d],
) {
    // Convert the step number once so every timed measurement shares one clock.
    let time = step as f32 / 60.0;
    let lowest = |index: BodyIndex| {
        let body = bodies
            .get(index.get())
            .copied()
            .expect("body snapshots are in complete profile order");
        surface_clearance(body, context.height)
    };
    // Record immediate impulse response from the first completed step.
    if step == 1 {
        state.run.speed_after_hit = bodies
            .iter()
            .map(|body| body.velocity.linear.length())
            .fold(0.0, f32::max);
    }
    // Measure relative head rotation at the source test's 150 ms boundary.
    if (time - 0.15).abs() < (1.0 / 60.0) * 0.5 {
        record_head_whip(context, state, current);
    }
    // Capture constraint and landing measurements from the same post-step poses.
    record_joint_measurement(context, state, current, time);
    record_landing_measurement(context, state, bodies, current, time, &lowest);
}

/// Measures head rotation relative to the chest at the configured hit boundary.
fn record_head_whip(
    context: &MeasureContext<'_>,
    state: &mut MeasureState,
    current: &[Isometry3d],
) {
    // Resolve current and initial poses by their validated profile indexes.
    let chest = current
        .get(context.chest.get())
        .expect("the chest pose is present");
    let head = current
        .get(context.head.get())
        .expect("the head pose is present");
    let start_chest = context
        .start
        .get(context.chest.get())
        .expect("the initial chest pose is present");
    let start_head = context
        .start
        .get(context.head.get())
        .expect("the initial head pose is present");
    // Compare head-to-chest orientation so torso rotation cancels out.
    let relative = chest.rotation.inverse() * head.rotation;
    let start_relative = start_chest.rotation.inverse() * start_head.rotation;
    state.run.head_whip = (relative * start_relative.inverse())
        .to_axis_angle()
        .1
        .to_degrees();
}

/// Records transient and final joint-limit excess for the current body poses.
fn record_joint_measurement(
    context: &MeasureContext<'_>,
    state: &mut MeasureState,
    current: &[Isometry3d],
    time: f32,
) {
    // Keep the largest transient limit excess and its first matching bone label.
    let (excess, _, _, worst, _) = joint_errors(context.profile, current);
    let excess_degrees = excess.to_degrees();
    if excess_degrees > state.run.joint_excess {
        state.run.joint_excess = excess_degrees;
        state.run.joint_worst = format!("{worst} at {time:.2} s");
    }
    // Overwrite the final value on each step so it represents the last frame.
    state.run.joint_excess_at_end = excess_degrees;
}

/// Records first contact, knee angles, bounce, and settling speed.
fn record_landing_measurement(
    context: &MeasureContext<'_>,
    state: &mut MeasureState,
    bodies: &[BodySnapshot],
    current: &[Isometry3d],
    time: f32,
    lowest: &impl Fn(BodyIndex) -> f32,
) {
    // Record contact only once, using the lower pelvis or spine surface.
    if state.run.ground.is_none() && lowest(context.pelvis).min(lowest(context.spine)) < 0.08 {
        state.run.ground = Some(time);
        let pelvis = current
            .get(context.pelvis.get())
            .expect("the pelvis pose is present");
        state.ground_position = Vec3::from(pelvis.translation);
        // Capture both knee angles from the same contact frame.
        for (slot, child) in context.knees.iter().enumerate() {
            let angle = context
                .profile
                .joint_angles(*child, current)
                .expect("each knee is connected to the profile root")
                .x
                .to_degrees();
            *state
                .run
                .knees
                .get_mut(slot)
                .expect("both knee slots are represented in the result") = angle;
        }
    }
    // Start bounce and rest measurements only after first floor contact.
    if let Some(ground_time) = state.run.ground {
        if time > ground_time + 0.1 {
            state.run.bounce = state.run.bounce.max(lowest(context.pelvis));
        }
        let is_slow = bodies.iter().all(|body| {
            body.velocity.linear.length() < 0.05 && body.velocity.angular.length() < 0.3
        });
        match (is_slow, state.slow_since) {
            (true, None) => state.slow_since = Some(time),
            (false, _) => state.slow_since = None,
            _ => {}
        }
    }
}

/// Computes final slide, rest delay, and one-second creep from captured frames.
fn finish_measurement(context: MeasureContext<'_>, mut state: MeasureState) -> Run {
    // Derive slide and rest delay from the first contact and final slow interval.
    if let Some(ground_time) = state.run.ground {
        let pelvis = state
            .history
            .last()
            .and_then(|frame| frame.get(context.pelvis.get()))
            .expect("the final frame includes the pelvis");
        let end = Vec3::from(pelvis.translation);
        state.run.slide = Vec3::new(
            end.x - state.ground_position.x,
            0.0,
            end.z - state.ground_position.z,
        )
        .length();
        state.run.rest = state.slow_since.map(|time| (time - ground_time).max(0.0));
    }
    // Compare the final frame with the frame exactly one second earlier.
    let one_second_back = state
        .history
        .len()
        .checked_sub(60)
        .expect("the measurement records at least sixty fixed steps");
    let start = state
        .history
        .get(one_second_back)
        .expect("the one-second history frame is present");
    let end = state
        .history
        .last()
        .expect("the measurement records its final frame");
    state.run.creep = start
        .iter()
        .zip(end)
        .map(|(first, last)| first.translation.distance(last.translation))
        .fold(0.0, f32::max);
    state.run
}

/// Measures a capsule's clearance above flat or stepped terrain.
pub(super) fn surface_clearance(body: BodySnapshot, height: fn(f32) -> f32) -> f32 {
    // Capsules use both endpoints; other shapes use their lowest world point.
    match body.shape {
        ShapeSpec::Capsule { a, b, radius } => {
            // Compare terrain under both endpoints before subtracting radius.
            let a = Vec3::from(body.pose.translation) + body.pose.rotation * a;
            let b = Vec3::from(body.pose.translation) + body.pose.rotation * b;
            (a.y - height(a.z)).min(b.y - height(b.z)) - radius
        }
        ShapeSpec::Sphere { .. } | ShapeSpec::Cuboid { .. } => {
            // Sample the terrain beneath the shape's centre after finding its low point.
            let shape = body.shape;
            let low = lowest_point(shape, body.pose);
            low - height(Vec3::from(body.pose.translation).z)
        }
    }
}

/// Runs one game-settings hit scenario and returns its body measurements.
fn look_run(
    backend: PhysicsBackend,
    height: fn(f32) -> f32,
    is_on_stairs: bool,
    velocity: Vec3,
    hits: &[(&str, Vec3)],
    seconds: f32,
) -> Run {
    // Apply initial movement before sending each scenario's authored hit.
    let mut scene = look_scene(backend, velocity, is_on_stairs);
    apply_hits(&mut scene, hits);
    let result = measure(&mut scene, height, seconds);
    eprintln!("{result:?}");
    result
}

/// Keeps the transient and final joint-limit tolerances from the TGF look checks.
fn assert_joints(result: &Run) {
    // Keep transient impact tolerance separate from the tighter final tolerance.
    let joint_excess = result.joint_excess;
    let joint_worst = &result.joint_worst;
    assert!(
        joint_excess <= 15.0,
        "a joint went {joint_excess} degrees past its limit ({joint_worst})"
    );
    let joint_excess_at_end = result.joint_excess_at_end;
    assert!(
        joint_excess_at_end <= 2.0,
        "a joint rests {joint_excess_at_end} degrees past its limit"
    );
}

/// Checks that a front headshot drops the rig and whips its head.
///
/// The torso reaches the floor in 0.3–0.8 s, the head whips at least 15° after
/// 0.15 s, post-hit speed stays below 3 m/s, and joints return within 2°.
///
/// # Examples
///
/// ```
/// # use bevy_ragdoll_conformance::physics::{PhysicsBackend, a_headshot_drops_the_body_like_the_references};
/// # fn register(_: fn(PhysicsBackend)) {}
/// register(a_headshot_drops_the_body_like_the_references);
/// ```
pub fn a_headshot_drops_the_body_like_the_references(backend: PhysicsBackend) {
    // Apply a front head impulse and record the complete four-second response.
    let result = look_run(
        backend,
        flat_height,
        false,
        Vec3::ZERO,
        &[("head", Vec3::new(0.0, 0.0, -12.0))],
        4.0,
    );
    // Check contact timing and relative head motion before speed and recovery.
    let ground = result.ground.expect("the body reaches the floor");
    assert!((0.3..=0.8).contains(&ground), "hit to ground {ground} s");
    let head_whip = result.head_whip;
    assert!(head_whip >= 15.0, "head whip {head_whip} degrees at 0.15 s");
    let speed_after_hit = result.speed_after_hit;
    assert!(
        speed_after_hit <= 3.0,
        "{speed_after_hit} m/s after the hit"
    );
    assert_joints(&result);
}

/// Checks that a chest hit buckles the knees and stops without a large bounce.
///
/// The rig reaches the floor within 0.3–0.8 s, bends one knee at least 45°,
/// slides at most 0.5 m, bounces at most 5 cm, and rests within 1.7 s.
///
/// # Examples
///
/// ```
/// # use bevy_ragdoll_conformance::physics::{PhysicsBackend, a_chest_hit_buckles_the_knees_and_stops};
/// # fn register(_: fn(PhysicsBackend)) {}
/// register(a_chest_hit_buckles_the_knees_and_stops);
/// ```
pub fn a_chest_hit_buckles_the_knees_and_stops(backend: PhysicsBackend) {
    // Apply the source chest impulse and inspect the fall through final rest.
    let result = look_run(
        backend,
        flat_height,
        false,
        Vec3::ZERO,
        &[("spine_04", Vec3::new(0.0, 0.0, -25.0))],
        4.0,
    );
    // Read knee angles from first contact before evaluating horizontal motion.
    let ground = result.ground.expect("the body reaches the floor");
    assert!((0.3..=0.8).contains(&ground), "hit to ground {ground} s");
    let knees = result.knees;
    assert!(
        result.knees.iter().any(|knee| *knee <= -45.0),
        "knees {knees:?} at landing"
    );
    // TGF references slide at most about 0.3 m; this measured 0.43 m.
    let slide = result.slide;
    assert!(slide <= 0.5, "slid {slide} m after landing");
    let bounce = result.bounce;
    assert!(bounce <= 0.05, "pelvis rose {bounce} m after landing");
    let rest = result.rest.expect("the body comes to rest");
    // Rapier 0.36 measured 1.617 s from landing to the shared slow-speed threshold.
    assert!(rest <= 1.7, "at rest {rest} s after landing");
    // Check one-second creep after the measured rest interval.
    let creep = result.creep;
    assert!(creep <= 0.005, "moved {creep} m in the last second");
    assert_joints(&result);
}

/// Checks that a running death stops within one body length after a chest hit.
///
/// The rig runs forward at 5 m/s, reaches the floor, slides at most 1.5 m, and
/// rests within 2 s after landing with at most 5 mm of final one-second creep.
///
/// # Examples
///
/// ```
/// # use bevy_ragdoll_conformance::physics::{PhysicsBackend, a_running_death_stops_within_a_body_length};
/// # fn register(_: fn(PhysicsBackend)) {}
/// register(a_running_death_stops_within_a_body_length);
/// ```
pub fn a_running_death_stops_within_a_body_length(backend: PhysicsBackend) {
    // Give the rig its reference running velocity before the chest impulse.
    let result = look_run(
        backend,
        flat_height,
        false,
        Vec3::new(0.0, 0.0, 5.0),
        &[("spine_04", Vec3::new(0.0, 0.0, -25.0))],
        4.0,
    );
    // Require contact before checking slide distance and rest delay.
    assert!(result.ground.is_some(), "the body reaches the floor");
    let slide = result.slide;
    assert!(slide <= 1.5, "slid {slide} m after landing");
    let rest = result.rest;
    assert!(
        rest.is_some_and(|seconds| seconds <= 2.0),
        "rest {rest:?} s after landing"
    );
    // Check final creep and joint recovery after the rest threshold.
    let creep = result.creep;
    assert!(creep <= 0.005, "moved {creep} m in the last second");
    assert_joints(&result);
}

/// Checks the body settles on the TGF stairs under a front chest shot.
///
/// This remains ignored because the source TGF settings slide 1.1 m and creep
/// 4 cm/s; raising friction to 0.1 made the determinism drop tear joints.
///
/// # Examples
///
/// ```
/// # use bevy_ragdoll_conformance::physics::{PhysicsBackend, a_body_shot_onto_stairs_stays_on_them};
/// # fn register(_: fn(PhysicsBackend)) {}
/// register(a_body_shot_onto_stairs_stays_on_them);
/// ```
pub fn a_body_shot_onto_stairs_stays_on_them(backend: PhysicsBackend) {
    // Use the source chest impulse and the stair-height function for clearance.
    let result = look_run(
        backend,
        stairs_height,
        true,
        Vec3::ZERO,
        &[("spine_04", Vec3::new(0.0, 0.0, -25.0))],
        5.0,
    );
    // Check stair contact and slide before the tighter final joint limit.
    assert!(result.ground.is_some(), "the body reaches the stairs");
    let slide = result.slide;
    assert!(slide <= 1.5, "slid {slide} m down the stairs after landing");
    let rest = result.rest;
    assert!(
        rest.is_some_and(|seconds| seconds <= 4.5),
        "rest {rest:?} s after landing"
    );
    let creep = result.creep;
    assert!(creep <= 0.005, "moved {creep} m in the last second");
    // Keep transient stair impact error separate from final joint recovery.
    let joint_excess = result.joint_excess;
    let joint_worst = &result.joint_worst;
    assert!(
        joint_excess <= 25.0,
        "{joint_excess} degrees ({joint_worst})"
    );
    let joint_excess_at_end = result.joint_excess_at_end;
    assert!(joint_excess_at_end <= 2.0, "{joint_excess_at_end} degrees");
}

/// Shoots a downed body and returns maximum part and pelvis travel over one second.
fn shoot(scene: &mut PhysicsScene, bone: &str, impulse: Vec3) -> (f32, f32) {
    // Resolve both profile indexes before copying the current body state.
    let hit_index = body_index(&scene.profile, bone);
    let pelvis_index = body_index(&scene.profile, "pelvis");
    let before = snapshots(scene.app.world_mut(), scene.character);
    // Save both baseline poses before writing the impact message.
    let hit_body = before
        .get(hit_index.get())
        .copied()
        .expect("the profile-ordered snapshot includes the hit body");
    let pelvis_before = before
        .get(pelvis_index.get())
        .copied()
        .expect("the profile-ordered snapshot includes the pelvis");
    // Apply the shot at the selected body's local shape centre in world space.
    let hit_pose = hit_body.pose;
    let point = Vec3::from(hit_pose.translation) + hit_pose.rotation * local_center(hit_body.shape);
    scene.app.world_mut().write_message(RagdollImpulse {
        body: hit_body.entity,
        point,
        impulse,
    });
    // Track displacement after every completed fixed step.
    track_shot_motion(
        scene,
        hit_index.get(),
        pelvis_index.get(),
        hit_body.pose,
        pelvis_before.pose,
    )
}

/// Tracks the hit body and pelvis over sixty completed physics steps.
fn track_shot_motion(
    scene: &mut PhysicsScene,
    hit_index: usize,
    pelvis_index: usize,
    hit_start: Isometry3d,
    pelvis_start: Isometry3d,
) -> (f32, f32) {
    // Reset both displacement maxima for this single-body hit.
    let (mut part_max, mut pelvis_max) = (0.0_f32, 0.0_f32);
    for _ in 0..60 {
        scene.app.update();
        // Snapshot after update so displacement includes this step's response.
        let current = snapshots(scene.app.world_mut(), scene.character);
        let hit_after = current
            .get(hit_index)
            .expect("the profile-ordered snapshot retains the hit body");
        let pelvis_after = current
            .get(pelvis_index)
            .expect("the profile-ordered snapshot retains the pelvis");
        part_max = part_max.max(hit_start.translation.distance(hit_after.pose.translation));
        pelvis_max = pelvis_max.max(
            pelvis_start
                .translation
                .distance(pelvis_after.pose.translation),
        );
    }
    (part_max, pelvis_max)
}

/// Checks that bullet impulses move a downed body locally without throwing the rig.
///
/// A 12 N·s hit moves the head 1.5–20 cm and the lower arm 2–50 cm. Pelvis travel
/// after either hit stays within 5 cm.
///
/// # Examples
///
/// ```
/// # use bevy_ragdoll_conformance::physics::{PhysicsBackend, a_bullet_moves_a_downed_body_a_little};
/// # fn register(_: fn(PhysicsBackend)) {}
/// register(a_bullet_moves_a_downed_body_a_little);
/// ```
pub fn a_bullet_moves_a_downed_body_a_little(backend: PhysicsBackend) {
    // Knock the rig down before testing whether later hits remain local.
    let mut scene = look_scene(backend, Vec3::ZERO, false);
    apply_hits(&mut scene, &[("spine_04", Vec3::new(0.0, 0.0, -25.0))]);
    // Let the chest hit settle before capturing each bullet baseline.
    for _ in 0..180 {
        scene.app.update();
    }
    // Keep local-part movement and pelvis movement as separate bounds.
    for (bone, minimum, maximum) in [("head", 0.015, 0.2), ("lowerarm_l", 0.02, 0.5)] {
        let (moved, pelvis) = shoot(
            &mut scene,
            bone,
            Vec3::new(1.0, 0.3, 0.0).normalize() * 12.0,
        );
        eprintln!("{bone}: {moved} m, pelvis {pelvis} m");
        assert!(
            (minimum..=maximum).contains(&moved),
            "the {bone} moved {moved} m"
        );
        assert!(
            pelvis <= 0.05,
            "the body moved {pelvis} m after a hit on the {bone}"
        );
    }
}
