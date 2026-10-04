# Phase 10: falling, getting up, returning, and the showcase

Goal: the full puppet state machine (`Falling`, `Down`, `GettingUp`,
`Returning`, `Turning` added to phase 9's states), the death path, and the
Euphoria-style `puppet_showcase` example.

Read first: [../reference/hit-reaction.md](../reference/hit-reaction.md)
sections 4, 5, 6, 7 (all states), 8 (tests 2, 3, 10 to 18);
[../reference/examples.md](../reference/examples.md) (`puppet_showcase`,
`crowd_brawl`); TGF `crates/tgf-game/src/combat/ragdoll/lifecycle.rs`.

## Steps

1. Write the failing acceptance tests 2, 3 and 10 to 18 first, in
   `tests/acceptance.rs`, with the names below.
2. Falling (4.1, 4.2): muscle ramp to 0.3 over 0.25 s, pins off, brace pose
   by direction, catch-fall reach with two-bone IK on both arms toward the
   predicted contact, arm, torso and leg muscle ratios 1.0, 0.6, 0.4, elbow
   torque at half during the reach.
3. Down (4.3, 4.4): limp at 0.12 over 0.3 s, rest detection with the
   tuning numbers, forced get-up after `force_after`. Wake the ragdoll
   before leaving Down (bevy_rapier motors do not wake sleeping bodies).
4. GettingUp (5.1 to 5.4):
   - face-up or face-down from the chest's forward axis, side tiebreak;
   - face-up: clip `getup_back` (`LayToIdle`);
   - face-down: the free clips have no prone get-up, so roll first: for
     0.6 s set the target to the face-up lying pose (`LayToIdle` frame 0)
     with muscle 0.5 and a pelvis torque about the body's long axis toward
     face-up, capped at 150 N·m; then re-test and continue face-up. If the
     roll fails twice, play `getup_back` from where it lies (log it);
   - measure each get-up clip's frame-0 pelvis offset and head-to-pelvis
     yaw once at load (5.2) and align the character root with them;
   - snapshot and blend over 0.30 s through `RagdollTargetAdjust.replace`
     weights; ramp muscles core first; ramp the get-up pin; protections
     (half `r_max`, double pin regain, ten times knockout distance).
5. Locomotion back to the marker (6.1 to 6.4) as the `PuppetLocomotion`
   trait in the balance crate with a default `ArriveLocomotion`
   implementation: moves the character root (kinematic root, ground by
   raycast through `RagdollQuery` excluding ragdoll bodies), plays `walk`
   at a rate matched to speed, blends to `idle` below 0.15 m/s, leash at
   0.4 m, walking compensation of the step margin.
6. Turning (6.3): the free clips have no turn clips, so turn by rotating
   the root at `clamp(2.5 * e, -1.6, 1.6)` rad/s while idle plays; tidy
   steps happen through the stepping logic. Done at 4 degrees.
7. Death path: `PuppetCommand::Kill` switches to the TGF lifecycle (powered
   fall toward `death` at `fall_strength 1.0` for `fall_ms 1200`, hold at
   0.4, then limp, settle). A killed puppet never gets up.
8. `puppet_showcase` example as specified in
   [../reference/examples.md](../reference/examples.md): yard scene
   (ground, crates, a ramp), three puppets on markers with different
   headings, weapon keys, HUD per puppet (state, muscle bar, steps), gizmo
   toggle `G`, slow motion `T` (0.25 time scale through `Time<Virtual>`).
   Also `crowd_brawl`.
9. Record a 60 s video of the showcase with OBS or Bevy screenshots at
   10 Hz assembled by `ffmpeg` from the flake (add `ffmpeg` to the dev
   shell). Look through it and list defects.

## Tests (names for hit-reaction section 8)

- 2 `random_hits_never_break_the_puppet`, 3 `results_hold_at_120_hz`,
  10 `heavy_front_hit_knocks_down_face_up`, 11 `heavy_back_hit_knocks_down_face_down`,
  12 `forward_fall_lands_on_hands_first`, 13 `get_up_starts_after_rest`,
  14 `get_up_does_not_pop`, 15 `get_up_finishes_standing`,
  16 `get_up_picks_the_clip_for_the_side`, 17 `returns_to_the_marker`,
  18 `a_punch_while_returning_still_arrives`.
- `face_down_rolls_then_gets_up`: start prone; ends standing within
  5.0 s.
- `killed_puppet_stays_down`.

Statistical tests (10, 11, 12) run 10 seeded variations each with the
direction jitter of the reference and assert the count.

## Done when

- All acceptance tests pass three times in a row.
- The showcase video shows, at least once each: flinch, stagger with
  steps, fall with catch-fall, lying, face-up get-up, face-down roll and
  get-up, walking back, turning to heading. List the timestamps in the
  commit body.
- Stress `balance` and `shooting` scenarios run; sweep rerun; baseline
  committed.
- `PLAN.md` marks phase 10 done.

## Stop and ask

Show the owner the video, the timestamps and the list of defects before
phase 11. The owner may ask for a tuning pass first.
