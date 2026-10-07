//! Criterion benchmark groups and their shared deterministic fixtures.
//!
//! Each module except `support` defines one `benches` group. `support` builds
//! seeded profiles and headless apps before Criterion times an iteration.

pub(crate) mod capture;
pub(crate) mod math;
pub(crate) mod plugin_build;
pub(crate) mod profile;
pub(crate) mod step_rapier3d;
pub(crate) mod support;
pub(crate) mod writeback;
