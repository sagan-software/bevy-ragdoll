# Phase 8: the animated human

Goal: a skinned, animated CC0 human with a tuned ragdoll profile, the
animation-driven examples, and the stress example on real animation.

Read first: [../reference/assets.md](../reference/assets.md) sections 1,
3 and 4, [../reference/algorithms.md](../reference/algorithms.md) section 9
step 7 (hinge sign), Bevy's `examples/animation/animated_mesh.rs` and
`examples/stress_tests/many_foxes.rs`.

## Steps

1. Download the Quaternius Universal Animation Library GLBs (CC0). Try
   the official itch.io Standard zips first
   (https://quaternius.itch.io/universal-animation-library and
   `-2`). If the download needs a browser purchase flow, use the CC0
   mirror https://github.com/Dallolz/moorfall-assets files
   `animations/UAL1.glb` and `animations/UAL2.glb` and read its
   `LICENSE.md` first. Record the source URL, date and SHA-256 of each file
   in `assets/CREDITS.md`. Store them as
   `assets/characters/ual_human/ual1.glb` and `ual2.glb`. Both contain the
   65-bone skeleton and a skinned `Mannequin` mesh; clips from either file
   play on the other because the bone name paths match.
2. Write `ual_human.ragdoll.ron` by mapping the TGF profile onto the UAL
   skeleton with a test-only helper (`tests/ual_profile.rs`, ignored test
   `generate_ual_profile` that writes the file):
   - 16 bodies, bone map identical except TGF `spine_04` to UAL
     `spine_03` and `head` to `Head`;
   - keep TGF masses, radii, limits and torques;
   - capsule endpoints from the UAL bone's head to its main child's head,
     inset by the radius, with TGF's capsule length ratio;
   - rest frames from the UAL bind pose.
3. Check every hinge with the rest-pose bend rule and the clips: sample
   `Idle_Loop`, `Walk_Loop`, `Jog_Fwd_Loop`, `Hit_Chest`, `Hit_Head`,
   `Death01`, `LayToIdle`, `Hit_Knockback` every 1/30 s and measure each
   joint's angles against its limits. Widen a limit only to the measured
   maximum plus 5 degrees, and list every widened limit in the commit body.
   If an axis is clearly swapped (the knee's range appears on twist), fix
   the joint frame, not the limits.
4. Clip names: map them once in `assets/characters/ual_human/clips.ron`
   (`idle`, `walk`, `jog`, `hit_chest`, `hit_head`, `death`, `getup_back`
   = `LayToIdle`, `knockback` = `Hit_Knockback`). Clips the free tier lacks
   are procedural in phases 9 and 10: turn in place, stagger, falling brace
   and the face-down get-up (roll to face-up first).
5. Examples on the UAL human (Rapier):
   - `toggle_ragdoll`, `powered_follow`, `death_fall`, `blend_to_animation`
     as described in [../reference/examples.md](../reference/examples.md);
   - `hit_reactions` moves from the TGF rig to the UAL human.
6. Stress example: `--creature human` now means the UAL human playing
   `Idle_Loop` with one shared `AnimationGraph` handle; keep the TGF rig as
   `--creature capsules`. Rerun the sweep and commit the new baseline.

## Tests

- `ual_profile_loads_and_validates`: 16 bodies, 80 kg.
- `ual_knees_bend_backward_and_elbows_forward`: drive each hinge to its
  limit and check the bend direction against the bone's rest-pose bend.
- `ual_clips_stay_within_limits`: the step 3 measurement as a test; no
  excess above 2 degrees after widening.
- Physics tier on Rapier with the UAL profile: `a_dropped_ragdoll_lands_and_settles`
  and `pinned_pelvis_stands_for_ten_seconds` while following `Idle_Loop`
  (mean joint error < 5°).

## Done when

- Tests pass; licences and hashes recorded.
- Each example's screenshot shows its Check line: a skinned mannequin,
  no stretched skin, no bone pop when toggling.
- New sweep baseline committed with the human numbers.
- `PLAN.md` marks phase 8 done and lists the missing clips.

## Stop and ask

If no download path gives a CC0 file with recorded provenance, stop and
ask the owner rather than using another source.
