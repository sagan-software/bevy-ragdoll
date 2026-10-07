# Phase 3: Rapier powered-ragdoll spike

Goal: prove, before the core API is fixed, that the 16-body human profile on
bevy_rapier3d can follow a moving target pose with joint motors and with
the torque drive, stand with a pelvis pin, and do it cheaply. The owner
reviews the result before phase 4 starts.

Read first: [../reference/physics-api.md](../reference/physics-api.md)
part A, [../reference/algorithms.md](../reference/algorithms.md) sections
1 to 4.

## Steps

1. Add `bevy_rapier3d` to the examples crate behind feature `rapier3d`.
   Create `examples/spike_rapier_powered.rs`. It is throwaway: it may
   build Rapier entities directly from the `RagdollProfile` without the
   core runtime. Phase 5 deletes it.
2. Spawn the human profile standing on a ground cuboid: one entity per
   body (capsule from the profile shape, mass with an inertia floor of
   0.08 m), one `ImpulseJoint` with a `GenericJoint` per joint (locked
   linear axes, per-axis limits, locked `0..0` axes, contacts disabled),
   pairs in `no_contact` filtered with `CollisionGroups` or a hooks
   filter. Use `RapierPhysicsPlugin::in_fixed_schedule()`,
   `TimestepMode::Fixed { dt: 1/60, substeps: 1 }`, `Time::<Fixed>::from_hz(60.0)`,
   `Sleeping::disabled()` on every body.
3. Generate a procedural target pose each fixed step: rest pose plus
   `0.6 * sin(2*PI*0.5*t)` rad on both shoulders' X, `0.4 * sin(...)` on
   the elbows, and a squat cycle of `0.5 * (1 - cos(2*PI*0.25*t))` rad on
   hips and knees (opposite signs as the hinge directions require). Clamp
   to limits.
4. Implement two drive paths selected by `--drive motor|torque`: Rapier
   per-axis motors (algorithms section 2, `AccelerationBased`) and the
   torque drive (section 3) through `ExternalForce.torque` written every
   step. Add `--pin <0..1>` applying section 4 on the pelvis and chest
   only, and `--muscle <0..1>`.
5. Add `--headless --count <n> --seconds <s> --report <json>` that spawns
   n ragdolls 2 m apart and records: mean and max joint angle error
   against the target (degrees), pelvis height drift (m), ms per fixed step
   (mean, p95), and unstable bodies (speed > 50 m/s or NaN).
6. Run the matrix and write `docs/spikes/rapier-powered.md` with a table
   of results and two screenshots (motor and torque, muscle 1, pin 1):
   - `--drive motor` and `--drive torque`, each with muscle 1 and pin
     0, 0.5, 1;
   - muscle 0 (limp drop) as a sanity row;
   - count 1, 32 and 128 headless for the better-looking drive, built with
     `--profile stress-test`;
   - natural frequency 4 Hz and 6 Hz.
7. Commit the spike and the report ("Spike powered ragdolls on Rapier").

## Done when

- The report shows, for at least one drive, mean joint error under 5
  degrees and pelvis drift under 0.05 m over 10 s with pin 1, and no
  unstable bodies in any row. If neither drive gets there, still finish the
  report with the best numbers and the suspected cause.
- `PLAN.md` records the report path.

## Stop and ask

Stop here in every case. Give the owner the report path, the screenshots
with their defects listed, and a recommendation for the motor model the
core should default to. Wait for the owner's go before phase 4.
