# Phase 4: core runtime and mock backend

Goal: the engine-agnostic runtime: binding a skeleton, spawning body
entities on mode changes, capturing targets, computing drives,
interpolating and writing bones back, the budget, and a mock backend that
passes the contract tier of a new conformance suite.

Read first: [../architecture.md](../architecture.md) in full,
[../reference/algorithms.md](../reference/algorithms.md) sections 2 to 7,
the owner's notes on the phase 3 spike in `PLAN.md`. Bevy references:
`examples/animation/animated_mesh.rs` (scene-ready observer),
`examples/ecs/relationships.rs`, `examples/movement/physics_in_fixed_timestep.rs`
in `~/Code/github.com/bevyengine/bevy`.

## Steps

1. Write the failing tests first: `crates/bevy_ragdoll/tests/runtime.rs`
   and the contract tier in the new crate `crates/bevy_ragdoll_conformance`
   (list below). Use a headless `App` (`MinimalPlugins`,
   `TransformPlugin`, `AssetPlugin`, `AnimationPlugin` when needed) and
   `TimeUpdateStrategy::FixedTimesteps(1)` so every `app.update()` runs
   one fixed step.
2. Add the components, resources, messages, events and sets exactly as
   named in the architecture, each in its own module file with docs:
   `components.rs`, `body.rs`, `drive.rs`, `sets.rs`, `messages.rs`,
   `events.rs`, `budget.rs`, `skeleton.rs`, `capture.rs`, `writeback.rs`,
   `backend.rs` (`BackendCapabilities`, `RagdollQuery`), `settings.rs`
   (`RagdollPhysicsSettings` with the defaults in algorithms.md).
3. `RagdollPlugin { fixed_schedule: InternedScheduleLabel }` with
   `Default` (FixedUpdate). It stores the label in a resource that backend
   plugins read, configures the `PostUpdate` sets after `AnimationSystems`
   and before `TransformSystems::Propagate`, and the fixed sets chained in
   architecture order.
4. Binding (`RagdollSystems::Bind`): when a `Ragdoll`'s profile is loaded
   and its skeleton exists, build `SkeletonMap` by matching body bone names
   below the character (breadth-first, parents first; port
   `Skeleton::find`). Trigger it from a `WorldInstanceReady` observer and
   also from a polling system for skeletons built in code. Missing bones
   produce `RagdollError::MissingBone(name)` logged once and the ragdoll
   stays `Animated`.
5. Mode changes: leaving `Animated` spawns body entities (top level) with
   the components of the architecture, positioned at the target pose and
   moving at the target velocity (algorithms section 7). Entering
   `Animated` despawns them. `Kinematic`, `Dynamic`, `Frozen` map to
   `BodyKind`.
6. Capture (algorithms section 5), drive (sections 2 to 4; torques only
   when `native_joint_motors` is false), writeback (section 6) with
   `RagdollBlend`.
7. Budget: port `Pool` to `RagdollBudget`; evict by freezing the oldest
   dynamic ragdoll and trigger `RagdollBudgetEvicted`.
8. Settle: a `Dynamic` ragdoll with muscle 0 whose every body has been
   slower than `settle_speed` (0.5 m/s) for `settle_after` seconds becomes
   `Frozen` if `RagdollPhysicsSettings::freeze_when_settled` is true
   (default false); trigger `RagdollSettled` either way.
9. Mock backend in `bevy_ragdoll_conformance::mock`: `MockBackendPlugin`
   integrates each dynamic body with semi-implicit Euler (gravity, linear
   and angular velocity, impulses, `BodyDriveOutput` forces and torques),
   ignores joints and contacts except a ground plane at y = 0 that stops
   bodies, sets `native_joint_motors: false`, implements `RagdollQuery` by
   ray-capsule tests against body shapes, and writes `BodyPhysicsPose`,
   `BodyVelocity`, `BodyAtRest`. It exists to test the contract, not to
   look like a ragdoll.
10. Contract tier: public functions in `bevy_ragdoll_conformance::contract`
    that take a closure adding a backend plugin to an `App`, so every
    backend runs the same cases from its own `tests/conformance.rs`.
11. Add the shared example helpers (`examples/src/lib.rs`: backend
    selection, camera, ground, overlay, `--headless`, `--exit-after`,
    `--screenshot`) and the `custom_backend` example on the mock backend.
    The mock ignores joints, so its bodies fall apart; that is expected and
    the example says so on screen. The other basic examples arrive in
    phase 5 on Rapier.

## Tests

Runtime (`tests/runtime.rs`, mock backend):

- `binding_finds_every_body_bone` on the human rig scene; and
  `binding_reports_a_missing_bone` on a renamed bone.
- `animated_mode_has_no_body_entities`; `dynamic_mode_spawns_one_entity_per_body`;
  `returning_to_animated_despawns_bodies`.
- `bodies_spawn_at_the_target_pose_with_its_velocity`: a skeleton moved at
  2 m/s for two frames, then switched; body 0 velocity is 2 m/s within
  1 %.
- `capture_reads_animated_locals_without_global_transform`: move the
  character's `Transform` in the same frame; captured skeleton-space poses
  stay equal.
- `writeback_puts_each_body_bone_at_its_body_pose` and
  `bones_without_bodies_keep_their_animated_locals`.
- `blend_zero_shows_the_animation_and_one_shows_physics`, with 0.5 halfway.
- `interpolation_uses_overstep`: poses at overstep 0, 0.5, 1.
- `budget_freezes_the_oldest`: budget 2, three ragdolls turn dynamic in
  order; the first is frozen and the event fires once.
- `drive_values_match_algorithms`: pure-function tests of sections 2 to 4,
  including the stable-PD pendulum and the pin test.
- `drive_clamps_inputs_to_unit_range` for muscle and pin at -1, 0, 1, 2.

Contract tier (`bevy_ragdoll_conformance::contract`, run against the mock):

- `bodies_and_joints_exist_for_each_profile_entry`.
- `an_impulse_changes_momentum_by_its_size`: 10 N·s on a free body changes
  total momentum by 10 ± 0.1.
- `pose_and_velocity_are_read_back_every_step`.
- `kinematic_bodies_follow_targets_exactly` (within 1 mm).
- `frozen_bodies_do_not_move_under_impulses`.
- `raycast_reports_the_body_hit`.
- `despawning_the_character_removes_every_body`.

## Gates

README gates, plus llvm-cov on `bevy_ragdoll` and
`bevy_ragdoll_conformance` (100 % of new lines or written reasons), plus
running each new example with `--exit-after 3 --screenshot` and checking
the image.

## Done when

- All tests pass with the mock backend.
- `custom_backend` runs headless and windowed; its screenshot shows the
  bodies spawned at the rig's rest pose.
- `PLAN.md` marks phase 4 done.
