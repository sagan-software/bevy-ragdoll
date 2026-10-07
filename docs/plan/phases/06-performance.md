# Phase 6: stress example, benchmarks, first baselines

Goal: the measuring tools exist before the expensive features land. Agents
and the owner can compare backends and catch regressions from here on.

Read first: [../reference/stress-and-benches.md](../reference/stress-and-benches.md)
in full, Bevy's `examples/stress_tests/many_foxes.rs`,
`examples/stress_tests/bevymark.rs` and `benches/` layout in
`~/Code/github.com/bevyengine/bevy`.

## Steps

1. Implement `examples/ragdoll_stress.rs` with every argument in the
   reference. Scenarios that need animation (`grid`, `wave`, `powered`)
   use a procedural idle on the capsule human rig until phase 8 adds the animated
   human: rest pose plus a 0.25 Hz breathing sway of 0.05 rad on the
   spine. `balance` and `shooting` print "needs phase 9/7" and exit 2 until
   those phases enable them; `--creature` accepts only `human` until phase
   11.
2. Draw the human rig's capsules as meshes (one shared capsule mesh per
   shape size, shared material) so windowed runs show bodies.
3. Implement timing systems around the backend's step set and the core
   sets, the report JSON exactly as specified (`schema: 1`), `--sweep`
   with child processes, the Markdown summary, and `--compare`.
4. Put the report and sweep types in `examples/src/stress/` with unit tests:
   percentile maths on known inputs (p50 of 1..=100 is 50.5 or 50 by a
   documented rule; pick nearest-rank and test it), JSON round trip,
   `--compare` threshold edges (exactly 10 % passes, 10.1 % fails).
5. Implement the criterion benches that exist so far (`profile`, `math`,
   `capture`, `writeback`, `step_rapier3d`), each with seeded inputs.
6. Run, on the owner's machine, inside `nix develop`:
   - `cargo run -p bevy_ragdoll_examples --example ragdoll_stress --profile stress-test --features rapier3d -- --headless --sweep default`
   - `cargo bench -p bevy_ragdoll_benches -- --save-baseline phase6`
   Commit `benches/baselines/<hostname>.json` and `benches/RESULTS.md`
   (date, git sha, CPU, the summary table, notable criterion numbers).
7. Add a CI job `stress-smoke` that runs one tiny headless scenario
   (`--scenario pile --count 4 --duration 2`) and checks
   `unstable_bodies == 0`. CI machines are too noisy for timing gates; do
   not compare timings in CI.
8. Write `benches/README.md`: how to run the sweep, compare against a
   baseline, profile with samply and tracy.

## Done when

- `ragdoll_stress` runs windowed and headless for `pile`, `grid`, `wave`
  and `powered` on Rapier; `B` and `S` restart as specified.
- The sweep completes and writes a report; `--compare` against the fresh
  baseline exits 0, and against a baseline edited to be 20 % faster exits
  1.
- Baseline and results are committed. The commit body lists the 16x16 grid
  numbers.
- A windowed `grid 16x16` screenshot after the trigger shows bodies on the
  ground with no body under the floor.
- `PLAN.md` marks phase 6 done.
