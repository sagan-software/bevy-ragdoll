# Phase 12: Avian 3D backend

Goal: `bevy_ragdoll_avian3d` passes the contract and physics tiers and the
balance acceptance tests, and the stress sweep compares it with Rapier.

Read first: [../reference/physics-api.md](../reference/physics-api.md)
part B in full; [../architecture.md](../architecture.md) backend
contract; the Rapier backend as the model to follow.

## Steps

1. Check the current Avian release on crates.io. If a release newer than
   0.7.0 supports Bevy 0.19 and adds spherical joint motors (Avian issue
   #934), stop and ask whether to use it.
2. Write a test first that settles the swing-limit axis question from
   physics-api B5: rotate body 2 about `twist_axis` only and about a
   perpendicular axis only, and record which rotation the swing limit
   stops. Write the result into `docs/plan/reference/physics-api.md`
   (section B5) and into this crate's docs.
3. Crate setup: `avian3d = { workspace = true, default-features = false, features = ["3d", "f32", "parry-f32", "xpbd_joints"] }`,
   optional `parallel`, `enhanced-determinism`, `debug`. Joints are not
   solved without `xpbd_joints`.
4. `AvianRagdollPlugin`: requires `RagdollPlugin` with
   `fixed_schedule = FixedPostUpdate`; orders `Apply` before
   `PhysicsSystems::StepSimulation` and `Read` after
   `PhysicsSystems::Writeback`. Capabilities:
   `native_joint_motors: false`, `asymmetric_swing_limits: false`,
   `deterministic: true` only for same-machine runs until phase 15 proves
   more, `wasm: true`.
5. Bodies: `RigidBody` (immutable; re-insert to change kind), collider from
   the shape (`Collider::capsule_endpoints(radius, a, b)`, `sphere`,
   `cuboid` with full lengths), `Mass`, `AngularInertia` with the floor,
   `CenterOfMass`, `LinearVelocity`, `AngularVelocity`, `LinearDamping`,
   `AngularDamping`, `SweptCcd`, `Friction`, `Restitution`,
   `SleepingDisabled` while driven.
6. Joints: hinges (`twist` and `z` locked) become `RevoluteJoint` with
   `hinge_axis = X`, `angle_limit` and `AngularMotor` (`SpringDamper`
   with the muscle frequency) driven natively; every other joint is a
   `SphericalJoint` with `twist_axis = Y`, twist limits, and the swing cone
   set from the step 2 result (symmetric: the largest of the X and Z
   extents). Each joint entity gets `JointCollisionDisabled`.
7. Torque drive and soft limits come from the core (algorithms sections 3
   and 3's soft-limit paragraph) through `BodyDriveOutput`; apply them with
   `Forces::apply_torque` in `FixedPostUpdate` before the step. Hinges skip
   the core torque because their motor is native: make the capability
   per-joint by adding `JointDriveMode { Native, Torque }` on joint
   entities, set by the backend, read by the core. Update
   architecture.md in the same commit.
8. Contact filter: `CollisionHooks` with `filter_pairs` on `NoContactWith`;
   `ActiveCollisionHooks::FILTER_PAIRS` on ragdoll colliders; document how
   users with their own hooks call `ragdoll_filter_pairs`.
9. Read set from `Position`, `Rotation`, velocities and `Has<Sleeping>`.
   Queries through `SpatialQuery::cast_ray` (map `ColliderOf` to the body)
   and contact pairs for `contacts`.
10. Tune `SubstepCount` and joint compliance until the physics tier
    passes; record the values and the reasons.
11. Add `avian3d` to every example's backend choice and to the stress
    example; add the `step_avian3d` bench.

## Tests

- `tests/conformance.rs`: contract and physics tiers.
- `swing_limit_axis_is_as_documented_in_this_crate` (step 2).
- Balance acceptance tests 1 and 4 to 18 run against Avian through a
  backend-generic harness in `bevy_ragdoll_balance` (refactor the phase 9
  and 10 tests to take the backend as a parameter).

## Done when

- Contract and physics tiers pass. Balance tests that fail on Avian are
  listed in `PLAN.md` with their measured values; none of them may be a
  crash, NaN or explosion.
- Sweep with both backends committed; `benches/RESULTS.md` gains a
  Rapier versus Avian section per scenario, and the README states which
  backend is faster for which scenario, with the numbers.
- `PLAN.md` marks phase 12 done.

## Status (2026-10-07)

`crates/bevy_ragdoll_avian3d` exists and passes the contract tier. The
physics tier does not pass yet. Phase 12 is not done.

Step 1: `avian3d` 0.7.0 is still the newest release and issue #934
(spherical joint motors) is still open, so the plan continues on 0.7.0.

Step 2 result (B5). `joint::tests::swing_limit_axis_is_as_documented_in_this_crate`
pushes a child with a constant torque against a joint to a static parent.
Avian 0.7 measures the swing cone around `twist_axis.any_orthonormal_vector()`,
and its "twist" limit bounds rotation about that same vector. With
`twist_axis = X` that vector is `+Y`. The adapter therefore sets
`twist_axis = X`. The test shows that rotation about frame `+Y` stops at
`twist.max`, rotation about `-Y` stops at `twist.min`, and rotation about X
or Z stops at the swing cone. `revolute_limit_sign_matches_the_profile_x_range`
shows that a `RevoluteJoint` with `hinge_axis = X` stops at the signed X
range. `docs/plan/reference/physics-api.md` B5 still says "verify with a
test"; it was outside this change's file scope.

Deviations from the steps above:

- Schedule: the adapter accepts the `RagdollPlugin` fixed schedule. Apps add
  `PhysicsPlugins::new(FixedUpdate)`, so one binary can hold both adapters
  (step 4 said `FixedPostUpdate`).
- Capabilities: `has_native_joint_motors: false`,
  `has_asymmetric_swing_limits: false`, `is_deterministic: false` (not
  measured; the flag has no "same machine" value), `can_run_on_wasm: true`.
- Joints: all-locked joints are `FixedJoint`; twist and Z locked joints are
  `RevoluteJoint` with exact X limits but no native motor; other joints are
  `SphericalJoint` with a cone at the largest X or Z extent. Every joint is
  driven by the core stable-PD fallback. Step 7's per-joint
  `JointDriveMode` was not added because it changes the core.
- `BodyKind::Fixed` maps to `RigidBody::Kinematic` with zero velocity. Avian
  0.7 panics ("Neither body ... is in an island") when a joint links two
  static bodies, because static bodies have no island. The adapter keeps a
  frozen body's `BodyPhysicsPose` and clears its velocity on readback.
- `solver_iterations` maps to `SubstepCount`. `pgs_iterations`,
  `max_substeps` and `threads` have no Avian equivalent.
- Avian creates its collider-tree diagnostics in `Plugin::finish`. Apps that
  call `App::run` get it; the conformance harness calls `App::update` only,
  so `tests/conformance.rs` calls `app.finish()` and `app.cleanup()`.
- `motors_hold_a_target_pose` is not run, because the adapter has no native
  motors.

Physics-tier cases that fail, with their measured values (all are
`#[ignore]`d in `tests/conformance.rs`; none crashes or produces NaN):

- `dropped_ragdoll_lands_and_settles`: sinks 2.9 cm (bound 1 cm); limit
  overshoot 100 deg, 47 deg at rest (bounds 5 and 2 deg); joint gap 7.5 cm
  at `upperarm_l` (bound 1 cm).
- `hard_throw_keeps_joints_together`: sinks 5.4 cm (bound 2.5 cm); overshoot
  68 deg (bound 20 deg); gap 7.8 cm at `calf_l` (bound 3 cm).
- `same_input_gives_the_same_output`: the shared drop sinks 2.4 cm before
  repeatability is compared.
- `torque_drive_holds_a_target_pose`: `calf_l` ends at -89 deg.
- `pinned_pelvis_stands_for_ten_seconds`: pelvis drifts 12.2 cm.
- `headshot_drops_body_like_the_references`: `hand_l` goes 60 deg past its
  X limit at 0.57 s.
- `chest_hit_buckles_knees_and_stops`: pelvis rises 7.4 cm after landing.
- `running_death_stops_within_a_body_length`,
  `body_shot_onto_stairs_stays_on_them`: the body never rests.
- `bullet_moves_downed_body_a_little`: a head hit moves the body 5.7 cm.
- `pistol_to_the_chest_does_not_move_the_pelvis_far`: pelvis moves 54 cm.

Step 10 tuning tried `SubstepCount` 8, 12 and 30 and `SolverConfig`
(`contact_damping_ratio` 20, `contact_frequency_factor` 2.5,
`max_overlap_solve_speed` 10). More substeps made the drop worse (30
substeps: 4.6 cm sink, 11.8 cm gap, energy rising after contact in the
throw). The joint gaps and limit overshoot point at the joint solve or the
fallback torque, which still needs investigation.

Not done: balance tests against Avian, the `step_avian3d` bench (the bench
fixtures in `benches/benchmarks/support.rs` build Rapier apps only), the
stress sweep, and example backend selection.
