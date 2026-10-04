# Phase 9: balance, stepping and flinch

Goal: crate `bevy_ragdoll_balance` with the measurements, pelvis pin,
flinch, recovery stepping and give-up tests, and a puppet state machine
covering `Idle`, `HitReact` and `Stagger`, with `Falling` as a limp
placeholder until phase 10.

Read first: [../reference/hit-reaction.md](../reference/hit-reaction.md)
sections 1.7, 2.5, 3 (all), 7 (Idle, HitReact, Stagger), 8 (tests 1 to 9),
10; [../architecture.md](../architecture.md) (`RagdollTargetAdjust`, crate
`bevy_ragdoll_balance`).

## Steps

1. Create the crate (depends on `bevy_ragdoll` only; dev-depends on
   `bevy_ragdoll_rapier3d` and `bevy_ragdoll_conformance`). Write the
   failing acceptance tests first (list below) in `tests/acceptance.rs`,
   each printing its measured value next to its threshold.
2. `BalanceTuning` with every number from hit-reaction sections 2.5, 3
   and 10 in one `Default`, loadable from RON
   (`assets/characters/ual_human/balance.ron` overrides).
3. `PuppetRig` component, built at bind time from body roles: pelvis,
   chest, head, and per side thigh, calf, foot, upper arm, forearm, hand
   bodies; foot rectangle from hit-reaction 3.2. Fail with a named error
   when a role is missing; the balance layer then stays off for that
   ragdoll.
4. Measuring system (`BalanceSystems::Measure`, first in
   `RagdollFixedSystems::Behaviour`): centre of mass and its velocity from
   `BodyPhysicsPose` and `BodyVelocity`; foot contacts from `RagdollQuery`
   (static contact, normal y > 0.7, foot speed < 0.5 m/s); support polygon
   (convex hull of planted foot corners, monotone chain, deterministic
   order); capture point and margin (signed distance, positive outside);
   chest tilt. Store in `BalanceState`.
5. Pelvis and chest pin through `RagdollBodyWeights.pin` and
   `PinSettings` per state (hit-reaction 3.6 table), including the pin
   weight drop on hits and its recovery.
6. Flinch (2.5): additive rotations into `RagdollTargetAdjust.additive`
   on the spine chain and neck, with the 0.45 s envelope.
7. Two-bone IK (`ik.rs`, pure functions): for root `a`, mid `b`, end `c`
   and target `t` with pole direction `p`: clamp `|t - a|` to
   `0.999 * (|b-a| + |c-b|)`, find the mid angle by the law of cosines,
   rotate the chain so `c` reaches `t` with `b` on the pole side. Unit
   tests: reachable target hit within 1 mm; unreachable target straightens
   toward it; pole flip changes knee side.
8. Stepping (3.4, 3.5): trigger, swing foot choice, target with the
   predicted capture point at landing and the clamps, smoothstep swing with
   0.10 m lift, replanning in the first 60 %, landing rules, swing-leg
   muscle 1, per-step leg weakening after step 2, spine lean and head
   look. Write the swing leg's thigh, calf and foot targets through
   `RagdollTargetAdjust.replace` from the IK solution; the stance foot is
   held by IK at its planted position.
9. Give-up tests (3.7) including the hit-time classification; recovery
   and tidy step (3.8).
10. `PuppetState` enum with data per variant (hit-reaction section 7
    shape) and one system per state. `Falling` in this phase: muscle ramps
    to 0.12 over 0.3 s and stays.
11. Debug gizmos behind feature `debug`: centre of mass, capture point,
    support polygon, planned step and swing path.
12. Example `balance_push` (reference/examples.md).

## Tests (Rapier, UAL human scaled to 70 kg, flat ground, 60 Hz)

Hit-reaction section 8 tests 1, 4, 5, 6, 7, 8 and 9, named
`idle_stands_still_for_ten_seconds`, `pistol_chest_no_step`,
`pistol_head_turns_head`, `punch_chest_recovers_within_two_steps`,
`punch_back_recovers_within_two_steps`,
`kick_side_staggers_without_crossing_feet`,
`thigh_hit_steps_or_bends_that_leg`, plus:

- `capture_point_matches_formula`: a synthetic centre of mass at 1.0 m
  moving 0.5 m/s gives `cp = com + 0.5 / 3.132`.
- `support_polygon_of_two_feet` and `of_one_foot` and `airborne_is_empty`.
- `heavy_hit_classifies_as_falling_at_hit_time`: J = 120.

## Tuning loop

When an acceptance test fails, change only `BalanceTuning` values, one at
a time, recording each run's measured values in `PLAN.md`. After 10 runs
on one test without passing, stop and ask (below).

## Done when

- The listed tests pass three times in a row (Rapier is deterministic for
  a fixed input, so a flaky pass means a bug in system order).
- `balance_push` screenshots at rest, mid-step and after recovery show the
  gizmos and a natural stance; defects listed.
- Stress `balance` scenario enabled; sweep rerun; baseline committed.
- `PLAN.md` marks phase 9 done with the final tuning values.

## Stop and ask

If tests 6 to 9 still fail after the tuning loop, stop with the
measurements and the best tuning so far. The owner decides between more
tuning time, the SIMBICON fallback of hit-reaction 3.5, or moving
stepping to version 2.
