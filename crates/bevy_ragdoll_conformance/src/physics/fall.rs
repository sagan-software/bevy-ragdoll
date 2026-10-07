//! Backend-neutral falling, contact, and joint-bound checks.

use bevy::math::{Isometry3d, Quat, Vec3};
use bevy_ragdoll::runtime::components::RagdollMode;
use bevy_ragdoll::runtime::messages::RagdollImpulse;
use bevy_ragdoll::runtime::settings::RagdollPhysicsSettings;
use std::time::Duration;

use super::{
    Bounds, FALL, PhysicsBackend, PhysicsScene, add_floor, body_index, joint_errors,
    lowest_body_surface, poses, scene, snapshots,
};

/// Measurements accumulated while one ragdoll falls and settles.
struct DropMeasurements {
    /// Whether any body reached the floor threshold.
    has_contact: bool,
    /// Total energy at first floor contact, in joules.
    contact_energy: f32,
    /// Captured body poses in each fixed step.
    frames: Vec<Vec<Isometry3d>>,
    /// Largest joint-limit excess during the fall, in radians.
    worst_angle: f32,
    /// Largest parent-child anchor separation, in metres.
    worst_gap: f32,
    /// Bone name for the largest anchor separation.
    worst_gap_bone: String,
    /// Largest floor penetration, in metres.
    worst_sink: f32,
    /// Joint-limit excess on the final step, in radians.
    final_angle: f32,
}

impl DropMeasurements {
    /// Allocates pose history for the exact number of simulated steps.
    fn new(steps: usize) -> Self {
        Self {
            has_contact: false,
            contact_energy: f32::INFINITY,
            frames: Vec::with_capacity(steps),
            worst_angle: 0.0,
            worst_gap: 0.0,
            worst_gap_bone: String::new(),
            worst_sink: 0.0,
            final_angle: 0.0,
        }
    }
}

/// Converts finite nonnegative seconds to the nearest 60 Hz step count.
pub(super) fn simulation_steps(seconds: f32) -> usize {
    let duration = Duration::try_from_secs_f32(seconds)
        .expect("physics scenarios use finite nonnegative durations");
    let rounded_steps = duration
        .as_nanos()
        .saturating_mul(60)
        .saturating_add(500_000_000)
        / 1_000_000_000;
    usize::try_from(rounded_steps).expect("physics scenario duration fits addressable memory")
}

/// Drops and measures the human rig for one backend-neutral scenario.
fn drop_and_check(
    backend: PhysicsBackend,
    seconds: f32,
    height: f32,
    push: Vec3,
    bounds: Bounds,
) -> Vec<Vec<Isometry3d>> {
    // Start from the authored fall pose so the backend receives exact target transforms.
    let mut scene = scene(
        backend,
        RagdollPhysicsSettings::default(),
        Isometry3d::new(
            Vec3::Y * height,
            Quat::from_rotation_x(-1.2) * Quat::from_rotation_z(0.3),
        ),
        0.0,
        0.0,
        RagdollMode::Dynamic,
    );
    // Add the floor before the first physics step creates backend state.
    add_floor(&mut scene, backend, Vec3::new(51.2, 0.8, 51.2));
    let chest_index = body_index(&scene.profile, "spine_03");
    let chest = snapshots(scene.app.world_mut(), scene.character)
        .into_iter()
        .find(|body| body.index == chest_index)
        .expect("the spine_04 body was spawned");
    // Apply the impulse at the chest centre to avoid adding angular momentum.
    scene.app.world_mut().write_message(RagdollImpulse {
        body: chest.entity,
        point: Vec3::from(chest.pose.translation),
        impulse: push,
    });

    let steps = simulation_steps(seconds);
    let mut measurements = DropMeasurements::new(steps);
    // Record every fixed step before evaluating the final scenario bounds.
    for step in 0..steps {
        scene.app.update();
        measure_drop_step(&mut scene, backend, step, &mut measurements);
    }
    assert_drop_bounds(&measurements, bounds);
    measurements.frames
}

/// Records floor contact, energy, joint error, and body poses for one step.
fn measure_drop_step(
    scene: &mut PhysicsScene,
    backend: PhysicsBackend,
    step: usize,
    measurements: &mut DropMeasurements,
) {
    // Reject invalid geometry before it can contaminate any later measurements.
    let lowest = lowest_body_surface(scene.app.world_mut(), scene.character);
    assert!(
        lowest.is_finite(),
        "the body surface became non-finite at step {step}"
    );
    measurements.worst_sink = measurements.worst_sink.max(-lowest);
    // Capture energy once at first contact, then require it not to rise.
    if !measurements.has_contact && lowest < 0.02 {
        measurements.has_contact = true;
        measurements.contact_energy = (backend.measure_energy)(
            &mut scene.app,
            scene.character,
            RagdollPhysicsSettings::default().gravity,
        );
    }
    if measurements.has_contact {
        let energy = (backend.measure_energy)(
            &mut scene.app,
            scene.character,
            RagdollPhysicsSettings::default().gravity,
        );
        assert!(
            energy <= measurements.contact_energy + 1.0,
            "energy rose after contact: {energy} J > {} J at step {step}",
            measurements.contact_energy
        );
    }
    // Measure constraints from one coherent post-step pose snapshot.
    let current_poses = poses(scene);
    let (angle, final_angle, gap, _, gap_bone) = joint_errors(&scene.profile, &current_poses);
    measurements.worst_angle = measurements.worst_angle.max(angle);
    measurements.final_angle = final_angle;
    if gap > measurements.worst_gap {
        measurements.worst_gap = gap;
        measurements.worst_gap_bone = gap_bone;
    }
    // Retain ordered frames for the final-motion check and repeatability test.
    measurements.frames.push(current_poses);
}

/// Checks the recorded drop against contact, penetration, joint, and gap bounds.
fn assert_drop_bounds(measurements: &DropMeasurements, bounds: Bounds) {
    // Convert angular errors only at the human-readable assertion boundary.
    let worst_angle_degrees = measurements.worst_angle.to_degrees();
    let final_angle_degrees = measurements.final_angle.to_degrees();
    let worst_sink = measurements.worst_sink;
    let worst_gap = measurements.worst_gap;
    let worst_gap_bone = &measurements.worst_gap_bone;
    eprintln!(
        "sink {worst_sink} m, limit overshoot {worst_angle_degrees} deg ({final_angle_degrees} deg at the end), gap {worst_gap} m at {worst_gap_bone}"
    );
    // Check contact and floor clearance before joint behavior.
    assert!(measurements.has_contact, "the ragdoll never landed");
    assert!(
        worst_sink < bounds.sink,
        "sank {worst_sink} m into the floor"
    );
    // Keep transient and final joint limits as separate acceptance bounds.
    assert!(
        measurements.worst_angle < bounds.landing_angle.to_radians(),
        "a joint went {worst_angle_degrees} degrees past its limit"
    );
    // Check parent-child anchor separation after angle limits.
    assert!(
        measurements.final_angle < bounds.final_angle.to_radians(),
        "a joint rests {final_angle_degrees} degrees past its limit"
    );
    assert!(
        worst_gap < bounds.gap,
        "joint {worst_gap_bone} separated {worst_gap} m"
    );
}

/// Drops the rig onto a flat plane and verifies that it lands without separating.
///
/// The rig starts 1.5 m above the floor with a 30 N·s chest push. It must settle
/// within the limits for sink, transient and final joint error, gap, and
/// final-frame motion.
///
/// # Examples
///
/// ```
/// # use bevy_ragdoll_conformance::physics::{PhysicsBackend, a_dropped_ragdoll_lands_and_settles};
/// # fn register(_: fn(PhysicsBackend)) {}
/// register(a_dropped_ragdoll_lands_and_settles);
/// ```
pub fn a_dropped_ragdoll_lands_and_settles(backend: PhysicsBackend) {
    // Reuse the common physics scenario and inspect its final quarter second.
    let frames = drop_and_check(backend, 3.0, 1.5, Vec3::new(0.0, 0.0, 30.0), FALL);
    // Start sixteen fixed frames from the end to measure the last 0.25 seconds.
    let start_index = frames
        .len()
        .checked_sub(16)
        .expect("the drop records at least sixteen frames");
    let start = frames
        .get(start_index)
        .expect("the drop records the frame sixteen steps from the end");
    let end = frames.last().expect("the drop records its final frame");
    let moved = start
        .iter()
        .zip(end)
        .map(|(first, last)| first.translation.distance(last.translation))
        .fold(0.0_f32, f32::max);
    assert!(moved < 0.005, "still moving {moved} m in the last 0.25 s");
}

/// Throws the rig onto the floor and checks the looser explosion bounds.
///
/// A 150 N·s impulse at 3 m height allows 2.5 cm floor sink, 20° transient
/// limit excess, and 3 cm joint separation for one step.
///
/// # Examples
///
/// ```
/// # use bevy_ragdoll_conformance::physics::{PhysicsBackend, a_hard_throw_keeps_the_joints_together};
/// # fn register(_: fn(PhysicsBackend)) {}
/// register(a_hard_throw_keeps_the_joints_together);
/// ```
pub fn a_hard_throw_keeps_the_joints_together(backend: PhysicsBackend) {
    let blast = Bounds {
        sink: 0.025,
        landing_angle: 20.0,
        gap: 0.03,
        ..FALL
    };
    drop_and_check(backend, 2.0, 3.0, Vec3::new(0.0, 40.0, 150.0), blast);
}

/// Repeats identical input and checks exact body-pose determinism.
///
/// Rapier runs serially unless its optional `parallel` feature is enabled; two
/// equal one-second drops must then produce equal profile-ordered pose frames.
///
/// # Examples
///
/// ```
/// # use bevy_ragdoll_conformance::physics::{PhysicsBackend, the_same_input_gives_the_same_output};
/// # fn register(_: fn(PhysicsBackend)) {}
/// register(the_same_input_gives_the_same_output);
/// ```
pub fn the_same_input_gives_the_same_output(backend: PhysicsBackend) {
    // Rapier 0.36 measured a 15.115 mm peak foot_l gap during this one-second drop.
    let bounds = Bounds { gap: 0.016, ..FALL };
    let first = drop_and_check(backend, 1.0, 1.5, Vec3::new(10.0, 0.0, 30.0), bounds);
    let second = drop_and_check(backend, 1.0, 1.5, Vec3::new(10.0, 0.0, 30.0), bounds);
    assert_eq!(first, second);
}
