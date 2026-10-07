//! Criterion entry point for every bevy-ragdoll benchmark group.
//!
//! Each group lives in `benches/benchmarks/` and measures one runtime stage:
//! profile validation, drive math, target capture, writeback, plugin setup, or
//! a Rapier physics step. Run them with `cargo bench --bench bench_main`, and
//! compare saved baselines with Criterion's `--baseline` option.

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
