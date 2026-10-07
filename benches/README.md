# Stress runs and benchmarks

Run the commands from the workspace root inside `nix develop`. Keep Cargo's
cache and target output on the SD card:

```sh
export CARGO_HOME=/var/mnt/nixsd/Caches/cargo
export CARGO_TARGET_DIR=/var/mnt/nixsd/Build/bevy-ragdoll
export CARGO_BUILD_JOBS=4
nix develop
```

## Stress runs

`ragdoll_stress` drops ragdolls in a `grid` or a `pile` and reports frame and
fixed-step p50, p95, and maximum times, the body count, and unstable bodies.
Each frame advances one 60 Hz step, so runs are repeatable.

```sh
cargo run --example ragdoll_stress --profile stress-test -- \
  --headless --scenario grid --count 256 --duration 10 --report grid-256.json
```

Run with `--help` for every option. To compare two runs, compare the
`metrics.frame_ms.p95` and `metrics.step_ms.p95` fields of their reports.
`benches/baselines/` holds the older sweep format, which the example no longer
writes.

## Criterion

`benches/bench_main.rs` runs the seeded profile, math, capture, writeback,
plugin-build, and Rapier-step groups from `benches/benchmarks/`. The capture and writeback cases construct their app outside the timed
iteration, bind the human skeleton, then time one Bevy update. The profile
cases use a profile generated from the built-in reference humanoid skeleton. Writeback starts
with distinct seeded previous and current physics poses. Rapier cases time 60
fixed steps for limp, powered, and asleep populations.

```sh
RUSTFLAGS="-C link-arg=-fuse-ld=lld" \
  cargo bench --bench bench_main -- --save-baseline phase6
```

The Nix shell selects `mold` by default. It cannot link Criterion's `alloca`
dependency because that object is LLVM bitcode and mold's GCC plugin does not
claim it. The LLD override in the command selects the compatible linker.

The 60-step Rapier cases use 100 samples for one character and 10 samples for
32, 128, and 512 characters. The larger cases set measurement windows for
their longer physics batches. See [RESULTS.md](RESULTS.md) for the first
machine baseline and recorded Criterion output.

Criterion output and intermediate baselines stay under
`$CARGO_TARGET_DIR/criterion`; do not commit those generated files. Record host
details and notable results in `benches/RESULTS.md`, then commit the sweep
report under `benches/baselines/`.

## Profiling

Build the native stress example with the optimized profile before profiling.
`samply` records CPU samples, and `perf` records Linux call stacks:

```sh
cargo build --example ragdoll_stress --profile stress-test
samply record "$CARGO_TARGET_DIR/stress-test/examples/ragdoll_stress" \
  --headless --scenario grid --grid 16x16 --duration 10
perf record --call-graph dwarf -- \
  "$CARGO_TARGET_DIR/stress-test/examples/ragdoll_stress" \
  --headless --scenario grid --grid 16x16 --duration 10
```

For Tracy, add Bevy's `trace_tracy` feature to the `bevy` dev-dependency
locally; the stress example has no Tracy feature of its own.
