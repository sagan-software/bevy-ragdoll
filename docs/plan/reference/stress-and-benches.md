# Stress example and benchmarks

## The `ragdoll_stress` example

File: `examples/ragdoll_stress.rs`, category "Stress Tests", following
Bevy's `examples/stress_tests/many_foxes.rs` structure (argh `FromArgs`
arguments, `FrameTimeDiagnosticsPlugin`, `PresentMode::AutoNoVsync`,
`WinitSettings::continuous()`). Build it with the `stress-test` profile:

```toml
[profile.stress-test]
inherits = "release"
lto = "fat"
codegen-units = 1
debug = "line-tables-only"
```

Every backend crate is compiled in through example features
(`rapier3d`, `avian3d`, later `jolt`); `--backend` picks which plugin the
app adds. Two engines never run in one process.

### Arguments

```
--backend <rapier3d|avian3d|jolt>     default: the first compiled in
--scenario <pile|grid|wave|powered|balance|shooting|mixed>   default grid
--count <n>                            pile, shooting: number of characters (default 64)
--grid <WxD>                           grid, wave, powered, balance: columns x rows (default 16x16)
--spacing <m>                          grid spacing, default 1.5
--creature <human|fox|raptor|trex|crocodile|rabbit|mixed>   default human
--mode <limp|powered|balance>          drive after the trigger, default limp
--trigger-at <s>                       seconds of standing animation before going ragdoll, default 2
--duration <s>                         seconds measured after the trigger, default 10
--warmup <s>                           seconds discarded before measuring, default 1
--fixed-hz <hz>                        default 60
--substeps <n>                         backend substeps or solver iterations knob, default backend's
--budget <n>                           RagdollBudget::max_dynamic, default unlimited
--seed <u64>                           default 42 (ChaCha8Rng, like bevymark)
--deterministic                        enable the backend's deterministic feature path (phase 15)
--headless                             MinimalPlugins + ScheduleRunnerPlugin, no window
--report <path.json>                   write the run report
--screenshot <path.png>                windowed only: capture at the end
--sweep <file.ron|default>             run a matrix of configurations, one child process each
--compare <baseline.json>              exit 1 if any metric regresses past its threshold
```

### Scenarios

- `pile`: characters spawn in a column above one point, 0.8 m apart
  vertically, already dynamic. Measures contact-heavy stacking.
- `grid`: characters stand animated (idle clip, `RagdollMode::Kinematic`)
  on a flat ground in a W x D grid; at `trigger-at` all switch to `Dynamic`
  at once. Measures the switch spike and the fall.
- `wave`: as grid, but each row switches 0.1 s after the previous one.
- `powered`: every character stays `Dynamic` with muscle 1 and pin 0.5,
  holding the idle animation. Measures drive cost with no falling.
- `balance`: every character runs the puppet state machine of phase 10 and
  receives a random push of 30 to 80 N·s every 2 to 4 s.
- `shooting`: `count` characters in a ring; a hit every 50 ms on a random
  body of a random character (`RagdollHit`, rifle profile).
- `mixed`: half the grid humans, the rest creatures.

The ground is a 200 m x 200 m static cuboid plus a few static boxes and a
ramp so piles interact with geometry.

### Measurements

Collected every frame after warmup:

- frame time (ms): p50, p95, p99, max, from `FrameTimeDiagnosticsPlugin`'s
  history (headless: wall time per `App::update`).
- physics step time (ms per fixed step): an `Instant` recorded by a system
  just before and just after the backend's step set.
- ragdoll core time (ms per frame): `PostUpdate` sets `Bind` through
  `Writeback` and fixed sets `Drive`, `Apply`, `Read`, `AfterStep`, timed the
  same way.
- the spike: the worst frame time within 0.5 s after the trigger.
- counts: characters, bodies, joints, dynamic, sleeping, frozen.
- peak resident memory (Linux `/proc/self/status` `VmHWM`; omitted on other
  platforms).
- simulation health: number of bodies faster than 50 m/s or with NaN
  poses (must be 0).

Windowed runs draw these in a corner overlay (Bevy UI text, updated twice a
second). Keys: `B` restarts the same run on the next compiled backend (the
example re-executes itself with `std::env::current_exe()` and the changed
argument, then exits); `S` cycles scenarios the same way; `Space` triggers
now; `F` toggles freezing all.

### Report format

```json
{
  "schema": 1,
  "git": "<short sha, from build.rs or env>",
  "machine": { "cpu": "...", "cores": 24, "os": "linux", "gpu": "..." },
  "config": { "backend": "rapier3d", "scenario": "grid", "grid": [16, 16], "creature": "human",
              "mode": "limp", "fixed_hz": 60, "substeps": 1, "seed": 42, "headless": true },
  "metrics": { "frame_ms": { "p50": 0.0, "p95": 0.0, "p99": 0.0, "max": 0.0 },
               "step_ms": { "p50": 0.0, "p95": 0.0, "max": 0.0 },
               "core_ms": { "p50": 0.0, "p95": 0.0 },
               "trigger_spike_ms": 0.0,
               "bodies": 0, "joints": 0, "dynamic": 0, "sleeping_end": 0,
               "peak_rss_mb": 0.0, "unstable_bodies": 0 }
}
```

`--sweep default` runs, headless, every compiled backend times
`grid 8x8, 16x16, 24x24, 32x32`, `pile 32, 64, 128`, `powered 16x16`,
`shooting 64`, `balance 8x8`, each in its own child process, writes
`target/stress/<date>-<sha>.json` (an array of reports) and prints a
Markdown summary table to stdout. The sweep prints progress lines so an
agent can watch it.

`--compare` thresholds: p95 frame time and p95 step time regress by more
than 10 %, `unstable_bodies` above 0, or any scenario failing to run.

### Baselines

Commit `benches/baselines/<hostname>.json` from the sweep on the owner's
machine. Phase 6 writes the first one. Later phases rerun the sweep when
they touch the core's fixed systems, writeback, or a backend, and commit
the new baseline in the same commit with the old and new numbers in the
body.

## Criterion benchmarks

Criterion 0.8 benches on the root crate in Criterion's own layout:
`benches/bench_main.rs` registers one `[[bench]] bench_main`,
`harness = false`, and runs one group per module in `benches/benchmarks/`, names
built with a `bench!` macro like Bevy's (`module_path!()` + name). Seeded
inputs only.

- `profile`: `RagdollProfile::new` for the human (16 bodies) and a 64-body
  chain; RON parse of the human profile; `auto::generate` on the UAL
  skeleton.
- `math`: joint angles for 1, 16, 1024 joints; muscle drive values;
  torque drive; pin drive.
- `capture`: target capture for 1, 64, 512 characters (a synthetic bone
  hierarchy in a `World`, no rendering).
- `writeback`: interpolation and writeback for 1, 64, 512 characters.
- `step_rapier3d`, `step_avian3d`: a headless `App` with N limp humans
  falling onto ground, stepping 60 fixed steps; N in 1, 32, 128, 512.
  Separate groups for `powered` (muscle 1, pin 0.5) and `asleep`.
- `balance`: measurements and the stepping planner for 1 and 64 puppets.

Commands:

```
cargo bench --bench bench_main -- --save-baseline main
cargo bench --bench bench_main -- --baseline main
```

Commit nothing from `target/criterion`; record notable numbers in commit
bodies and in `benches/RESULTS.md`, one dated section per phase that ran
them.

## Profiling

Document in `benches/README.md`: `samply record` and `perf` on the stress
example built with `--profile stress-test`; Bevy's `trace_tracy` feature
through an example feature `tracy`. The flake provides `samply`, `perf`
and `tracy`.
