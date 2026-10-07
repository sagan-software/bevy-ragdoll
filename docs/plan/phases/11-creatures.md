# Phase 11: automatic profiles and creatures

Goal: `auto::generate` turns any skeleton into a working profile, and the
fox, dinosaur, crocodile and rabbit each have an automatic and a tuned
profile.

Read first: [../reference/algorithms.md](../reference/algorithms.md)
section 9, [../reference/assets.md](../reference/assets.md) section 2.

## Steps

1. Download, check licence, hash and record in `assets/CREDITS.md`:
   - fox: Quaternius Fox GLB (CC0) from the poly.pizza URL in
     reference/assets.md; also keep Bevy's `Fox.glb` as `fox_khronos` with
     its required CC-BY 4.0 attribution;
   - dinosaurs: Quaternius Velociraptor and T-Rex GLBs (CC0);
   - crocodile: OpenGameArt `crocodile.7z` (CC0); convert to GLB with
     headless Blender if needed (`blender -b --python-expr` export with
     the glTF exporter), committing only the GLB and noting the
     conversion;
   - rabbit: OpenGameArt `rabbit.blend` (CC0), converted the same way. If
     it has no usable walk or idle clip, use the Quaternius Bunny (CC0,
     biped) and note that it is a biped.
     Inspect each file's bones and clips and add them to
     `reference`-style notes in `assets/creatures/README.md`.
2. Write failing tests in `tests/auto.rs`.
3. `SkeletonView` from a spawned glTF scene (bones by `ChildOf`, rest
   transforms from the skin's inverse bind matrices, `SkinSample` from the
   skinned mesh's positions, joint indices and weights) and from
   `ProfileSpec`-free code.
4. Implement the generator steps 1 to 8 in `auto/` (one file per step
   group: `filter.rs`, `chains.rs`, `shapes.rs`, `roles.rs`, `limits.rs`).
5. `Ragdoll::default()` generates at bind time and caches the
   result as a `RagdollProfile` asset per glTF handle.
6. Example `auto_profile`, then tune each creature with a short RON
   overrides file, `assets/creatures/<name>/<name>.overrides.ron`.
   Tuning targets: lands and settles, legs fold naturally, tails do not
   flip through the body, powered idle holds the pose with mean joint
   error under 8 degrees.
7. Example `creatures` (reference/examples.md); stress `--creature` accepts
   every creature and `mixed`.
8. Balance on creatures is out of scope; creatures use muscle and pin
   only.

## Tests

- `humanoid_auto_profile_is_reasonable` on `assets/rigs/humanoid.glb`
  (UE5 mannequin names): 14 to 18 bodies, mass equals the option, knees
  hinge backward.
- `ual_human_auto_profile_is_reasonable`: same.
- `ignored_bones_never_get_bodies`: twist, leaf, finger and IK bones.
- `hinge_sign_follows_rest_bend`: a synthetic two-bone leg bent forward
  and one bent backward give opposite ranges.
- `tail_chains_are_classified_as_tails` on the raptor.
- Physics tier (Rapier) for each creature, automatic and tuned profile:
  `a_dropped_ragdoll_lands_and_settles` and
  `powered_idle_holds_the_pose`.

## Done when

- Tests pass; every asset has a licence entry and a hash.
- A `creatures` screenshot with all six dropped and settled; defects
  listed.
- Sweep rerun with `--creature mixed`; baseline committed.
- `PLAN.md` marks phase 11 done.
