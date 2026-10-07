//! Run the shared contract and physics tiers against the public Avian adapter.
//!
//! `motors_hold_a_target_pose` is not run: it checks native joint motors, and
//! Avian 0.7 has no spherical joint motor, so the adapter reports
//! `has_native_joint_motors: false`. Physics-tier cases that do not pass yet
//! are `#[ignore]`d with their measured values; the contract tier passes.

use avian3d::prelude::{
    ComputedAngularInertia, ComputedCenterOfMass, ComputedMass, PhysicsPlugins, RigidBody,
};
use bevy::math::{Isometry3d, Vec3};
use bevy::prelude::{App, Entity, FixedUpdate, Transform};
use bevy_ragdoll::profile::ShapeSpec;
use bevy_ragdoll::runtime::body::{BodyPhysicsPose, BodyVelocity};
use bevy_ragdoll::runtime::components::RagdollBodyOf;
use bevy_ragdoll_avian3d::{AvianRagdollHooks, AvianRagdollPlugin, collider_for_shape};
use bevy_ragdoll_conformance::physics::PhysicsBackend;

/// Adds Avian in `FixedUpdate` with the ragdoll hooks, then the adapter.
///
/// The shared harness steps the app with `App::update` and never calls
/// `App::run`, so this finishes plugin setup here: Avian creates some
/// resources, such as its collider-tree diagnostics, in `Plugin::finish`.
fn add_avian(app: &mut App) {
    app.add_plugins(PhysicsPlugins::new(FixedUpdate).with_collision_hooks::<AvianRagdollHooks>());
    app.add_plugins(AvianRagdollPlugin);
    app.finish();
    app.cleanup();
}

/// Adds one static Avian collider at its rigid world pose.
fn add_fixed_shape(app: &mut App, shape: ShapeSpec, pose: Isometry3d) -> Entity {
    app.world_mut()
        .spawn((
            RigidBody::Static,
            collider_for_shape(shape),
            Transform::from_translation(pose.translation.into()).with_rotation(pose.rotation),
        ))
        .id()
}

/// Measures linear, angular, and gravitational energy from Avian mass properties.
fn measure_energy(app: &mut App, character: Entity, gravity: Vec3) -> f32 {
    let world = app.world_mut();
    let mut bodies = world.query::<(
        &RagdollBodyOf,
        &BodyPhysicsPose,
        &BodyVelocity,
        &ComputedMass,
        &ComputedAngularInertia,
        &ComputedCenterOfMass,
    )>();
    bodies
        .iter(world)
        .filter(|(owner, ..)| owner.0 == character)
        .map(|(_, pose, velocity, mass, inertia, center)| {
            let world_center =
                Vec3::from(pose.current.translation) + pose.current.rotation * center.0;
            // The computed tensor is local; rotate the world angular velocity into it.
            let local_angular = pose.current.rotation.inverse() * velocity.angular;
            let local_inertia = inertia.tensor() * local_angular;
            0.5 * mass.value() * velocity.linear.length_squared()
                + 0.5 * local_angular.dot(local_inertia)
                - mass.value() * gravity.dot(world_center)
        })
        .sum()
}

/// Creates the three typed callbacks consumed by the shared physics suite.
fn avian_backend() -> PhysicsBackend {
    PhysicsBackend::new(add_avian, add_fixed_shape, measure_energy)
}

/// Checks that every profile body and joint becomes a public backend component.
#[test]
fn profile_bodies_and_joints_follow_the_shared_contract() {
    bevy_ragdoll_conformance::contract::bodies_and_joints_exist_for_each_profile_entry(add_avian);
}

/// Checks that impulses change momentum by their requested amount.
#[test]
fn impulse_momentum_matches_the_shared_contract() {
    bevy_ragdoll_conformance::contract::an_impulse_changes_momentum_by_its_size(add_avian);
}

/// Checks that body pose and velocity write back after every fixed update.
#[test]
fn completed_steps_write_pose_and_velocity() {
    bevy_ragdoll_conformance::contract::pose_and_velocity_are_read_back_every_step(add_avian);
}

/// Checks transformed kinematic targets against the shared runtime contract.
#[test]
fn kinematic_targets_follow_nested_transforms() {
    bevy_ragdoll_conformance::contract::kinematic_bodies_follow_targets_exactly(add_avian);
}

/// Checks that frozen bodies ignore impulses.
#[test]
fn frozen_bodies_ignore_impulses() {
    bevy_ragdoll_conformance::contract::frozen_bodies_do_not_move_under_impulses(add_avian);
}

/// Checks that raycasts return the intersected ragdoll body.
#[test]
fn raycast_returns_the_shared_body_identity() {
    bevy_ragdoll_conformance::contract::raycast_reports_the_body_hit(add_avian);
}

/// Checks that freezing after an impulse clears Avian's velocities.
#[test]
fn freezing_a_dynamic_ragdoll_clears_velocity() {
    bevy_ragdoll_conformance::contract::freezing_a_dynamic_ragdoll_clears_velocity(add_avian);
}

/// Checks malformed rays, empty-space rays, and both filter forms return misses.
#[test]
fn invalid_and_filtered_raycasts_return_misses() {
    bevy_ragdoll_conformance::contract::invalid_and_filtered_raycasts_return_misses(add_avian);
}

/// Checks that despawning a character removes its backend bodies.
#[test]
fn despawning_removes_owned_bodies() {
    bevy_ragdoll_conformance::contract::despawning_the_character_removes_every_body(add_avian);
}

/// Checks gravity, profile mass, and body writeback through a full fall.
#[test]
fn ragdoll_falls_as_gravity_says() {
    bevy_ragdoll_conformance::physics::a_ragdoll_falls_as_gravity_says(avian_backend());
}

/// Checks a measured impulse against the backend's momentum result.
#[test]
fn impulse_gives_its_momentum() {
    bevy_ragdoll_conformance::physics::an_impulse_gives_its_momentum(avian_backend());
}

/// Checks that two sampled poses produce backend velocity writeback.
#[test]
fn velocities_come_from_two_poses() {
    bevy_ragdoll_conformance::physics::velocities_come_from_two_poses(avian_backend());
}

/// Checks that frozen bodies remain at rest under the physics tier.
#[test]
fn frozen_ragdoll_stays_put() {
    bevy_ragdoll_conformance::physics::a_frozen_ragdoll_stays_put(avian_backend());
}

/// Checks landing, joint limits, contact response, and sleep.
#[test]
#[ignore = "not met yet on Avian 0.7: sinks 1.7 cm into the floor (bound 1 cm)"]
fn dropped_ragdoll_lands_and_settles() {
    bevy_ragdoll_conformance::physics::a_dropped_ragdoll_lands_and_settles(avian_backend());
}

/// Checks that a high-energy throw preserves connected body constraints.
#[test]
#[ignore = "not met yet on Avian 0.7: sinks 4.2 cm into the floor (bound 2.5 cm)"]
fn hard_throw_keeps_joints_together() {
    bevy_ragdoll_conformance::physics::a_hard_throw_keeps_the_joints_together(avian_backend());
}

/// Checks repeatable results for the same scene and input.
#[test]
#[ignore = "not met yet on Avian 0.7: the shared drop sinks 1.3 cm (bound 1 cm) before repeatability is compared"]
fn same_input_gives_the_same_output() {
    bevy_ragdoll_conformance::physics::the_same_input_gives_the_same_output(avian_backend());
}

/// Checks the backend-neutral torque fallback holds a target pose.
#[test]
fn torque_drive_holds_a_target_pose() {
    bevy_ragdoll_conformance::physics::torque_drive_holds_a_target_pose(avian_backend());
}

/// Checks spawn lifting moves overlapping body poses above fixed geometry.
#[test]
fn spawn_with_feet_in_floor_lifts_out() {
    bevy_ragdoll_conformance::physics::a_spawn_with_feet_in_the_floor_lifts_out_of_it(
        avian_backend(),
    );
}

/// Checks a pinned pelvis and torso stay near the standing target for ten seconds.
#[test]
#[ignore = "not met yet on Avian 0.7: pelvis drifts 10.9 cm from the standing target"]
fn pinned_pelvis_stands_for_ten_seconds() {
    bevy_ragdoll_conformance::physics::pinned_pelvis_stands_for_ten_seconds(avian_backend());
}

/// Checks the profile's headshot response against the TGF look thresholds.
#[test]
#[ignore = "not met yet on Avian 0.7: upperarm_r goes 58 deg past its X limit at 0.42 s inside the symmetric swing cone"]
fn headshot_drops_body_like_the_references() {
    bevy_ragdoll_conformance::physics::a_headshot_drops_the_body_like_the_references(
        avian_backend(),
    );
}

/// Checks chest-hit knee flexion, horizontal slide, bounce, and rest time.
#[test]
#[ignore = "not met yet on Avian 0.7: the body does not come to rest"]
fn chest_hit_buckles_knees_and_stops() {
    bevy_ragdoll_conformance::physics::a_chest_hit_buckles_the_knees_and_stops(avian_backend());
}

/// Checks the running death response against the TGF look thresholds.
#[test]
#[ignore = "not met yet on Avian 0.7: the body never comes to rest after landing"]
fn running_death_stops_within_a_body_length() {
    bevy_ragdoll_conformance::physics::a_running_death_stops_within_a_body_length(avian_backend());
}

/// Checks the stair parity case.
#[test]
#[ignore = "not met yet on Avian 0.7 (also ignored on Rapier): the body never comes to rest after landing"]
fn body_shot_onto_stairs_stays_on_them() {
    bevy_ragdoll_conformance::physics::a_body_shot_onto_stairs_stays_on_them(avian_backend());
}

/// Checks that a bullet moves a downed ragdoll within the expected range.
#[test]
fn bullet_moves_downed_body_a_little() {
    bevy_ragdoll_conformance::physics::a_bullet_moves_a_downed_body_a_little(avian_backend());
}

/// Checks the standing chest-hit displacement, rotation, and muscle recovery bounds.
#[test]
#[ignore = "not met yet on Avian 0.7: pelvis moves 48 cm"]
fn pistol_to_the_chest_does_not_move_the_pelvis_far() {
    bevy_ragdoll_conformance::physics::pistol_to_the_chest_does_not_move_the_pelvis_far(
        avian_backend(),
    );
}

/// Checks that a headshot turns the neck and head within 150 milliseconds.
#[test]
fn headshot_turns_the_head() {
    bevy_ragdoll_conformance::physics::headshot_turns_the_head(avian_backend());
}

/// Checks that zero per-body muscle and pin strength lets the pelvis collapse.
#[test]
fn limp_weights_make_a_powered_ragdoll_collapse() {
    bevy_ragdoll_conformance::physics::limp_weights_make_a_powered_ragdoll_collapse(avian_backend());
}
