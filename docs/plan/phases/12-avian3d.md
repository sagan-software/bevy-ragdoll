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
- `AvianRagdollSettings::substep_count` (default 20) sets `SubstepCount`.
  `solver_iterations`, `pgs_iterations`, `max_substeps` and `threads` have
  no Avian equivalent.
- Swept CCD is off by default (`AvianRagdollSettings::use_swept_ccd`).
  Avian moves each swept body back to its own time of impact after the
  solve, which separated joints by up to 17 cm on landing. Speculative
  contacts (`SpeculativeMargin` from `soft_ccd_prediction`) stay on.
- Avian creates its collider-tree diagnostics in `Plugin::finish`. Apps that
  call `App::run` get it; the conformance harness calls `App::update` only,
  so `tests/conformance.rs` calls `app.finish()` and `app.cleanup()`.
- `motors_hold_a_target_pose` is not run, because the adapter has no native
  motors.

Second pass (after the generated-humanoid core). A per-step trace of the
drop showed joint gaps under 2 mm in free fall, so the joint frames and
anchors are correct; the gaps appeared only on landing frames. Swept CCD
caused them: with it off, the worst drop gap fell from 3.0 cm to 0.9 cm and
the throw gap from 16.9 cm to 1.8 cm. Raising substeps from 8 to 20 made
`torque_drive_holds_a_target_pose` (knee reached -27.5 deg of -60 deg at 8)
and `bullet_moves_downed_body_a_little` pass. Mass and inertia come from
`Mass` and `AngularInertia` on the body; the impulse-momentum cases confirm
the mass. Stiffer `SolverConfig` contacts (damping 20, frequency factor 4,
overlap speed 10) did not reduce floor sink and broke the knee target, so the
defaults stay.

Physics-tier cases that still fail, with measured values at 20 substeps (all
`#[ignore]`d in `tests/conformance.rs`; none crashes or produces NaN):

- `dropped_ragdoll_lands_and_settles`: sinks 1.7 cm (bound 1 cm).
- `hard_throw_keeps_joints_together`: sinks 4.2 cm (bound 2.5 cm).
- `same_input_gives_the_same_output`: the shared drop sinks 1.3 cm before
  repeatability is compared.
- `pinned_pelvis_stands_for_ten_seconds`: pelvis drifts 10.9 cm.
- `headshot_drops_body_like_the_references`: `upperarm_r` goes 58 deg past
  its X limit at 0.42 s, inside the symmetric swing cone.
- `chest_hit_buckles_knees_and_stops`, `running_death_stops_within_a_body_length`,
  `body_shot_onto_stairs_stays_on_them`: the body does not come to rest.
- `pistol_to_the_chest_does_not_move_the_pelvis_far`: pelvis moves 48 cm.

Floor sink is the remaining contact problem; the limit overshoot and the
pinned and pistol cases follow from the symmetric swing cone plus the weak
core soft-limit torque.

Not done: balance tests against Avian, the `step_avian3d` bench (the bench
fixtures in `benches/benchmarks/support.rs` build Rapier apps only), the
stress sweep, and example backend selection.
