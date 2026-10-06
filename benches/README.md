# Stress runs and benchmarks

Run the commands from the workspace root inside `nix develop`. Keep Cargo's
cache and target output on the SD card:

```sh
export CARGO_HOME=/var/mnt/nixsd/Caches/cargo
export CARGO_TARGET_DIR=/var/mnt/nixsd/Build/bevy-ragdoll
export CARGO_BUILD_JOBS=4
nix develop
```

## Stress sweep

The default sweep runs Rapier grids of 8×8, 16×16, 24×24, and 32×32, piles of
32, 64, and 128 characters, and a powered 16×16 grid. It prints skipped
scenarios that need later phases and writes a JSON array under
`$CARGO_TARGET_DIR/stress/`.

```sh
cargo run -p bevy_ragdoll_examples --example ragdoll_stress \
  --profile stress-test --features rapier3d -- --headless --sweep default
```

Run one scenario with a JSON report by adding `--scenario`, its grid or count,
and `--report path.json`. A sweep can read ordered RON rows with
`--sweep path.ron`; each row selects a scenario and can override the backend,
grid, or count.

Compare one run with an existing report array by selecting the same backend,
scenario, grid, seed, and runtime options used by that report:

```sh
cargo run -p bevy_ragdoll_examples --example ragdoll_stress \
  --features rapier3d -- --headless --scenario grid --grid 16x16 \
  --compare benches/baselines/HOSTNAME.json
```

The comparison rejects p95 frame or physics-step regressions above 10 percent,
invalid timing values, missing matching configurations, and unstable bodies.

## Criterion

Run the seeded profile, math, capture, writeback, plugin-build, and Rapier-step
targets. The capture and writeback cases construct their app outside the timed
iteration, bind the human skeleton, then time one Bevy update. Writeback starts
with distinct seeded previous and current physics poses. Rapier cases time 60
fixed steps for limp, powered, and asleep populations.

```sh
cargo bench -p bevy_ragdoll_benches -- --save-baseline phase6
```

Criterion output and intermediate baselines stay under
`$CARGO_TARGET_DIR/criterion`; do not commit those generated files. Record host
details and notable results in `benches/RESULTS.md`, then commit the sweep
report under `benches/baselines/`.

## Profiling

Build the native stress example with the optimized profile before profiling.
`samply` records CPU samples, and `perf` records Linux call stacks:

```sh
cargo build -p bevy_ragdoll_examples --example ragdoll_stress \
  --profile stress-test --features rapier3d
samply record "$CARGO_TARGET_DIR/stress-test/examples/ragdoll_stress" \
  --headless --scenario grid --grid 16x16 --duration 10
perf record --call-graph dwarf -- \
  "$CARGO_TARGET_DIR/stress-test/examples/ragdoll_stress" \
  --headless --scenario grid --grid 16x16 --duration 10
```

Use the `tracy` feature for Bevy's Tracy tracing integration. It also enables
the visible runner because Bevy installs the Tracy tracing plugin through its
default plugin group:

```sh
cargo run -p bevy_ragdoll_examples --example ragdoll_stress \
  --profile stress-test --features rapier3d,tracy -- \
  --scenario grid --grid 16x16 --duration 10
```
