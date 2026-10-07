//! Criterion benchmark groups and their shared deterministic fixtures.
//!
//! Each module except `support` defines one `benches` group. `support` builds
//! seeded profiles and headless apps before Criterion times an iteration.

pub mod capture;
pub mod math;
pub mod plugin_build;
pub mod profile;
pub mod step_rapier3d;
pub mod support;
pub mod writeback;
