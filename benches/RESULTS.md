# Phase 6 results

Date: 2026-10-06 (Steam Deck local date)

Source revision: `0a2a3d4e8d1249bfe224876f9e2b219774f000ec`

Machine: AMD Custom APU 0405, 8 logical CPUs, Linux.

The saved report is
[`baselines/steamdeck-6dffb94acbac45d38b162ddde8b7d645.json`](baselines/steamdeck-6dffb94acbac45d38b162ddde8b7d645.json).
It uses schema 1 and contains the eight configurations from the default
Rapier sweep. Each run used the human rig, a 10-second sample, a 1-second
warmup, a 60 Hz fixed step, seed 42, and headless mode.

## Stress sweep

All eight runs reported zero unstable bodies. Timing columns show p95
milliseconds. The grid trigger column records the trigger step; pile and
powered runs do not trigger a transition.

| Scenario | Size or count | Characters | Bodies | Frame p95 ms | Step p95 ms | Core p95 ms | Trigger ms | Unstable |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Grid | 8×8 | 64 | 1,024 | 18.402 | 16.925 | 18.345 | 17.186 | 0 |
| Grid | 16×16 | 256 | 4,096 | 97.730 | 90.713 | 97.646 | 86.475 | 0 |
| Grid | 24×24 | 576 | 9,216 | 234.145 | 215.016 | 234.000 | 206.568 | 0 |
| Grid | 32×32 | 1,024 | 16,384 | 431.456 | 390.069 | 431.251 | 383.061 | 0 |
| Pile | 32 | 32 | 512 | 12.511 | 11.222 | 12.461 | 0.000 | 0 |
| Pile | 64 | 64 | 1,024 | 28.619 | 25.130 | 28.566 | 0.000 | 0 |
| Pile | 128 | 128 | 2,048 | 75.363 | 66.419 | 75.298 | 0.000 | 0 |
| Powered | 16×16 | 256 | 4,096 | 97.362 | 90.045 | 97.266 | 0.000 | 0 |

The 16×16 limp grid measured frame p95 97.730 ms, physics-step p95
90.713 ms, core-update p95 97.646 ms, and trigger-step time 86.475 ms.

## Criterion

Command:

```sh
RUSTFLAGS="-C link-arg=-fuse-ld=lld" \
  cargo bench -p bevy_ragdoll_benches -- --save-baseline phase6
```

The Nix shell selects mold by default. The Criterion link failed with mold
because `alloca.o` is LLVM bitcode and mold's selected GCC plugin did not
claim it. The command passed when it selected LLD through `RUSTFLAGS` without
changing Cargo arguments. Criterion data remains under
`$CARGO_TARGET_DIR/criterion`.

The following table reports Criterion's mean point estimate. Capture and
writeback entries time one Bevy update after fixture setup. Math entries show
three input sizes. Profile entries validate or parse the named profile.

| Target | Input | Mean |
|---|---:|---:|
| Plugin build | Required core plugins | 420.930 µs |
| Profile construction | TGF human, 16 bodies | 6.194 µs |
| Profile construction | Seeded chain, 64 bodies | 83.940 µs |
| Profile RON parsing | TGF human, 16 bodies | 270.032 µs |
| Joint angles | 1 | 63.967 ns |
| Joint angles | 16 | 977.574 ns |
| Joint angles | 1,024 | 63.018 µs |
| Muscle drive | 1 | 6.748 ns |
| Muscle drive | 16 | 103.430 ns |
| Muscle drive | 1,024 | 6.596 µs |
| Torque drive | 1 | 49.980 ns |
| Torque drive | 16 | 833.965 ns |
| Torque drive | 1,024 | 56.793 µs |
| Pin drive | 1 | 47.558 ns |
| Pin drive | 16 | 816.084 ns |
| Pin drive | 1,024 | 56.620 µs |
| Target capture | 1 character | 225.755 µs |
| Target capture | 64 characters | 495.282 µs |
| Target capture | 512 characters | 2.429 ms |
| Physics writeback | 1 character | 232.732 µs |
| Physics writeback | 64 characters | 1.123 ms |
| Physics writeback | 512 characters | 8.545 ms |

Each Rapier result measures 60 fixed updates. Values are mean milliseconds
for the full 60-update batch.

| Characters | Limp | Powered | Asleep |
|---:|---:|---:|---:|
| 1 | 16.188 ms | 16.296 ms | 17.659 ms |
| 32 | 460.125 ms | 463.334 ms | 503.836 ms |
| 128 | 1,940.481 ms | 1,948.955 ms | 2,192.522 ms |
| 512 | 8,830.314 ms | 8,762.013 ms | 10,567.717 ms |

The 60-step benchmark uses 100 samples for one character and 10 samples for
32, 128, and 512 characters. Criterion printed four sample-window warnings:
`writeback/physics_writeback/1` estimated 5.942 seconds; asleep Rapier cases
estimated 5.219 seconds at 32 characters, 22.475 seconds at 128, and
108.970 seconds at 512. Each target completed its requested samples, and the
full command exited 0.

## Comparison checks

The stress-test-profile 16×16 run passed against the saved baseline. Candidate
frame p95 was 97.552 ms and step p95 was 90.525 ms, below the saved values.

The default dev-profile run exited 1 because its frame p95 was 112.423 ms.
The baseline came from the `stress-test` profile, so that comparison used
different build profiles. The documented command now uses `--profile
stress-test`.

A copy of the baseline with its 16×16 timings reduced by 20 percent exited 1
as expected. Candidate frame p95 97.535 ms exceeded the 78.184 ms limit, and
candidate step p95 90.491 ms exceeded the 72.570 ms limit.

The CI stress-smoke command and assertion passed. Its four-character pile
reported 64 bodies and zero unstable bodies.

## Screenshot and disk space

The reviewed windowed 16×16 grid screenshot is
[`../docs/screenshots/phase-06-grid-16.png`](../docs/screenshots/phase-06-grid-16.png).
It shows the ragdolls above the ground plane with no body under the floor.

The SD card had at least 72 GiB free during the sweep and Criterion runs,
which stayed above the required 40 GiB reserve.
