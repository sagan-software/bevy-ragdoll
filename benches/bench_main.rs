//! Criterion entry point for every bevy-ragdoll benchmark group.

use criterion::criterion_main;

mod benchmarks;

criterion_main! {
    benchmarks::plugin_build::benches,
    benchmarks::profile::benches,
    benchmarks::math::benches,
    benchmarks::capture::benches,
    benchmarks::writeback::benches,
    benchmarks::step_rapier3d::benches,
}
