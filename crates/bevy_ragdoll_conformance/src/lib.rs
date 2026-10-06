//! Tools for checking backend-neutral behavior in the `bevy-ragdoll` workspace.
//!
//! The `contract` module builds headless apps and runs the same activation,
//! impulse, pose, kinematic, frozen-body, raycast, and despawn assertions
//! against any backend plugin. The `mock` module supplies a semi-implicit Euler
//! backend for development and tests; it ignores joints and contacts beyond a
//! ground plane, so it does not model connected ragdoll motion.

/// Reusable checks for activation, body state, impulses, queries, and cleanup
/// across backends.
///
/// Each function creates a headless app, adds the core plugin, and invokes the
/// supplied backend plugin function. Use these checks from a backend crate's
/// integration tests to exercise the common component and message contract
/// without importing backend-specific system parameters.
pub mod contract;
/// A deliberately approximate physics backend used by conformance tests and the
/// custom-backend example.
///
/// It integrates dynamic bodies with semi-implicit Euler, follows kinematic
/// targets, applies impulses and drive outputs, clamps bodies to a ground
/// plane, and answers body raycasts. Joint constraints and contact solving
/// remain outside its model, so callers should use a real physics backend when
/// they need connected ragdoll motion or contact response.
pub mod mock;
