//! Contract tier shared by every physics backend.

use bevy::prelude::App;
use bevy_ragdoll_conformance::{contract, mock::MockBackendPlugin};

/// Adds the backend under test to a contract app.
fn add_mock_backend(app: &mut App) {
    app.add_plugins(MockBackendPlugin);
}

#[test]
fn bodies_and_joints_exist_for_each_profile_entry() {
    contract::bodies_and_joints_exist_for_each_profile_entry(add_mock_backend);
}

#[test]
fn an_impulse_changes_momentum_by_its_size() {
    contract::an_impulse_changes_momentum_by_its_size(add_mock_backend);
}

#[test]
fn pose_and_velocity_are_read_back_every_step() {
    contract::pose_and_velocity_are_read_back_every_step(add_mock_backend);
}

#[test]
fn kinematic_bodies_follow_targets_exactly() {
    contract::kinematic_bodies_follow_targets_exactly(add_mock_backend);
}

#[test]
fn frozen_bodies_do_not_move_under_impulses() {
    contract::frozen_bodies_do_not_move_under_impulses(add_mock_backend);
}

#[test]
fn raycast_reports_the_body_hit() {
    contract::raycast_reports_the_body_hit(add_mock_backend);
}

#[test]
fn despawning_the_character_removes_every_body() {
    contract::despawning_the_character_removes_every_body(add_mock_backend);
}
