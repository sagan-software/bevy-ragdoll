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
