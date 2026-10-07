# Phase 7: active control and hits

Goal: per-body muscle and pin weights, hits that weaken muscles locally
and recover, and the examples that show them on the capsule human rig with a
procedural target pose.

Read first: [../reference/hit-reaction.md](../reference/hit-reaction.md)
sections 0, 2.1 to 2.4 and 3.6, [../reference/algorithms.md](../reference/algorithms.md)
sections 2, 4 and 8.

## Steps

1. Write failing tests in `crates/bevy_ragdoll/tests/hits.rs` (pure
   systems on the mock backend) and in the physics tier (Rapier) listed
   below.
2. `RagdollBodyWeights`: inserted with all ones when a ragdoll binds;
   drive systems multiply whole-ragdoll muscle and pin by it.
3. Pin targets: `PinTargets` resource or per-ragdoll component listing
   which bodies take pin forces (default: every body; the balance crate
   sets pelvis and chest only). Pin tuning `PinSettings { frequency_hz, damping_ratio, max_force, max_torque, distance_falloff }`
   with the `idle` row of hit-reaction 3.6 as default.
4. Hit module `hit.rs`: `RagdollHit` message, `HitKind` (`Pistol`,
   `Rifle`, `Shotgun`, `Punch`, `Kick`, `Heavy`, `Explosion`, `Custom(f32)`)
   with the impulse table of hit-reaction 2.1 in `HitSettings::default()`.
   Processing order per hit, in `RagdollFixedSystems::Behaviour`:
   impulse clamp along the chain (algorithms section 8) emitting
   `RagdollImpulse` messages; strength drop with falloff (2.3) on
   `RagdollBodyWeights.muscle`, with floors and streak stacking; pin weight
   drop `1 - 0.8 * severity`; record `LastHit` on the character. Recovery
   (2.4) runs every fixed step after `delay`, with `order_delay` per body
   role. Roles come from a `BodyRole` per body: from the profile's bone
   names through the same patterns as algorithms section 9 step 6, or set
   explicitly in the RON spec (`BodySpec::role: Option<BodyRole>`; add the
   optional field now).
5. Velocity clamps while not limp: `max_linear_speed = 10`,
   `max_angular_speed = 20` (2.2 step 4), applied by the core through
   `BodyDriveOutput` limits; backends clamp engine velocity in `Apply` if
   the engine has a native setting.
6. Examples on the human rig (capsule visuals), Rapier backend:
   - `hit_reactions`: the rig stands with muscle 1 and pelvis and chest pin
     1, following the procedural idle. Clicking a capsule sends a
     `RagdollHit` of the selected kind at the clicked point along the
     camera ray. The overlay shows each body's muscle as a bar.
   - `partial_ragdoll`: legs muscle 1 and pin 1, upper body muscle 0.1;
     balls launched at the chest every 2 s.
7. Benchmarks: add hit processing for 1 and 64 ragdolls to `math`.

## Tests

Pure (`tests/hits.rs`):

- `strength_drop_follows_the_falloff_table`: J = 40 on `spine_03` with
  `max_hops = 2`: each body's muscle equals the formula within 1e-5,
  bodies 3 hops away untouched, floors respected.
- `streaks_stack_within_the_window` and `do_not_stack_after_it` (0.3 s
  edge: 0.29 stacks, 0.31 does not).
- `recovery_starts_after_the_delay_and_core_first`: pelvis recovers
  before hands by the `order_delay` difference.
- `impulse_clamp_passes_the_excess_to_the_parent`: 12 N·s on a 0.5 kg
  hand: hand receives 1.5 N·s, the rest goes up the chain, the sum equals 12.
- `hit_kinds_map_to_the_impulse_table`.

Physics tier (Rapier):

- `pistol_to_the_chest_does_not_move_the_pelvis_far`: standing pinned rig,
  J = 12 at the chest from the front: pelvis peak displacement < 0.08 m,
  chest peak rotation ≥ 4°, muscles back to ≥ 0.95 within 1.0 s
  (hit-reaction test 4 without the state machine).
- `headshot_turns_the_head`: J = 12 at the head: neck plus head peak
  rotation ≥ 8° (test 5).
- `limp_weights_make_a_powered_ragdoll_collapse`: muscle weights 0 on every
  body: pelvis height falls below 0.4 m within 1.5 s.

## Done when

- All tests pass on mock and Rapier; coverage gate holds for `hit.rs`.
- Both examples run; a screenshot of `hit_reactions` taken 0.15 s after a
  rifle hit to the chest shows the torso bent away from the camera.
- Stress sweep rerun; baseline updated if any number moved by more than 5 %.
- `PLAN.md` marks phase 7 done.
