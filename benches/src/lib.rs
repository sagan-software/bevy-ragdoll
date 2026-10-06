//! This unpublished workspace crate shares deterministic setup helpers between
//! Criterion targets.
//!
//! The helpers generate seeded profile fixtures and build headless Bevy
//! applications before each measured iteration. `support::PopulationMode`
//! selects capture, passive, powered, or sleeping physics state.
//! `support::BenchmarkSetupError` reports invalid asset resources or profile
//! parent ordering so a benchmark cannot silently measure an incomplete
//! ragdoll population.

/// Profile and application builders shared by the standalone Criterion targets.
///
/// This module loads the checked-in TGF human profile and creates seeded
/// synthetic chains. Its
/// application constructors bind the profile bones before returning an app to a
/// timed benchmark.
/// `core_app` prepares capture and writeback cases; `rapier_app` adds Rapier's
/// fixed step and floor.
/// Both constructors validate profile asset storage and parent-first topology
/// before setup ends.
pub mod support;
