# Phase 5: Rapier 3D backend and physics conformance

Goal: `bevy_ragdoll_rapier3d`, passing the contract tier and a new physics
tier ported from TGF's physics and look tests.

Read first: [../reference/physics-api.md](../reference/physics-api.md)
part A in full (pitfalls A10 matter), TGF `crates/tgf-ragdoll/src/world.rs`,
`tests/physics.rs`, `tests/look.rs`, `tests/common/mod.rs`,
[../reference/tgf-port-map.md](../reference/tgf-port-map.md), and the
spike report `docs/spikes/rapier-powered.md` with the owner's notes.

## Steps

1. Write the physics tier first in
   `bevy_ragdoll_conformance::physics` as backend-generic functions, then
   the crate `crates/bevy_ragdoll_rapier3d` with
   `tests/conformance.rs` calling both tiers. Confirm they fail.
2. Crate setup: `bevy_rapier3d = { workspace = true, default-features = false, features = ["dim3"] }`
   plus optional features `parallel`, `enhanced-determinism`,
   `debug-render` (forwarding `debug-render-3d`). Never enable `simd8`
   together with `enhanced-determinism`.
3. `RapierRagdollPlugin`: panics with a clear message unless
   `RagdollPlugin` was added first and its fixed schedule is `FixedUpdate`;
   checks that `RapierPhysicsPlugin` is configured `in_fixed_schedule()`
   (document that the user adds `RapierPhysicsPlugin`; the backend does
   not). Orders `RagdollFixedSystems::Apply` before
   `PhysicsSet::SyncBackend` and `Read` after `PhysicsSet::Writeback`.
   Inserts `BackendCapabilities { native_joint_motors: true, asymmetric_swing_limits: true, deterministic: true, wasm: true }`.
4. Body creation (`Added<BodyShape>`): `RigidBody`, the collider from
   `ShapeSpec` (`Collider::capsule(a, b, r)`, `ball`, `cuboid` with half
   extents), `ColliderMassProperties::MassProperties` with the inertia
   floor, `Velocity`, `Damping`, `Ccd`, `SoftCcd`, `Friction`,
   `Restitution`, `Sleeping` thresholds from settings (and
   `Sleeping::disabled()` while muscle or pin is above 0, because motor
   changes do not wake bodies), `ExternalImpulse`, `ExternalForce`,
   `ReadMassProperties`, `ActiveHooks::FILTER_CONTACT_PAIRS`.
5. Joints (`Added<JointToParent>`): `ImpulseJoint::new(parent, TypedJoint::GenericJoint(j))`
   on the child body entity, built as in TGF (`LOCKED_SPHERICAL_AXES`,
   `local_frame1 = frame`, `local_frame2 = identity`, per-axis limits,
   locked `0..0` axes, `set_contacts_enabled(false)`). Leave softness at
   the default unless the physics tier fails; if changed, assign
   `joint.raw.softness`.
6. Contact filter: a `BevyPhysicsHooks` implementation reading
   `NoContactWith` and `RagdollBodyOf` for the two bodies. Document that
   users who already use their own hooks type must call
   `ragdoll_filter_contact_pair` from it; provide that function.
7. Apply set: write `JointDriveTarget` into per-axis motors
   (`set_motor`, `set_motor_max_force`, `AccelerationBased`) only when the
   value changed; write `BodyDriveOutput` pin forces into `ExternalForce`
   (set every step, zero when unused); accumulate `RagdollImpulse` with
   `ExternalImpulse::at_point`; switch `RigidBody` on `BodyKind`
   (kinematic bodies use `KinematicPositionBased` and take the target
   pose).
8. Read set: shift and store `BodyPhysicsPose` from `Transform`, copy
   `Velocity` into `BodyVelocity`, set or remove `BodyAtRest` from
   `Sleeping::sleeping`. Port TGF's forced sleep after
   `force_sleep_after` for limp ragdolls slower than `settle_speed`.
9. `RagdollQuery` through `ReadRapierContext::cast_ray_and_get_normal` and
   the narrow phase's contact pairs for `contacts(body, out)`.
10. Spawn lift (algorithms section 7) with a shape intersection query
    against fixed colliders.
11. Move `minimal`, `from_code`, `from_ron`, `from_gltf_skein` to Rapier;
    delete `spike_rapier_powered` and keep its report.
12. Add CI jobs: `cargo test -p bevy_ragdoll_rapier3d` (already covered by
    workspace tests) and a headless example smoke run
    (`cargo run -p bevy_ragdoll_examples --example minimal --features rapier3d -- --headless --exit-after 3`).

## Physics tier (port; keep TGF thresholds and comments)

- `a_ragdoll_falls_as_gravity_says`, `an_impulse_gives_its_momentum`,
  `a_dropped_ragdoll_lands_and_settles`, `a_hard_throw_keeps_the_joints_together`,
  `the_same_input_gives_the_same_output`, `motors_hold_a_target_pose`,
  `velocities_come_from_two_poses`, `a_frozen_ragdoll_stays_put`,
  `a_spawn_with_feet_in_the_floor_lifts_out_of_it` (from `physics.rs`).
- `a_headshot_drops_the_body_like_the_references`,
  `a_chest_hit_buckles_the_knees_and_stops`,
  `a_running_death_stops_within_a_body_length`,
  `a_bullet_moves_a_downed_body_a_little`, and the ignored
  `a_body_shot_onto_stairs_stays_on_them` (from `look.rs`).
- New: `torque_drive_holds_a_target_pose` (the same case as
  `motors_hold_a_target_pose` with `native_joint_motors` forced off), so
  the Avian path is proven on Rapier first.
- New: `pinned_pelvis_stands_for_ten_seconds`: muscle 1, pin 1 on pelvis
  and chest, idle rest pose; pelvis drift < 0.05 m, mean joint error < 5°.

If a ported threshold fails, compare with TGF's settings first (gravity
20 in the look tests, `ccd: false`, solver iterations 8, PGS 2). Change a
threshold only with the measured value and reason in the test comment and
the commit body.

## Gates

README gates, llvm-cov on the backend crate, the example smoke run, and a
screenshot of `minimal` after landing.

## Done when

- Contract and physics tiers pass on Rapier (the stairs test stays
  ignored with its reason).
- The basic examples run on Rapier and their screenshots match their Check
  lines.
- `PLAN.md` marks phase 5 done.
