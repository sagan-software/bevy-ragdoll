# Examples

Every example is one self-contained file in `examples/`, registered on the
root `bevy_ragdoll` crate with Bevy's metadata block:

```toml
[[example]]
name = "hit_reactions"
doc-scrape-examples = true
required-features = ["gltf"]

[package.metadata.example.hit_reactions]
name = "Hit Reactions"
description = "Click a body part to hit it; the character flinches and recovers."
category = "Active"
wasm = true
```

Examples share no helper crate or module. Each file sets up its own app,
backend, scene, and skeleton so it reads as a complete program, like
Bevy's examples. When an example needs a large helper, simplify the
library API instead. The "Check" line names the behaviour to see.

## Basics (phase 4 to 7)

- `minimal`: the reference humanoid skeleton as capsules, dropped as a limp ragdoll on a
  plane. Check: lands, no joint separation, settles.
- `from_code`: a three-body chain built with `ProfileBuilder`. Check: hangs
  from a pinned root and swings.
- `from_gltf`: `assets/rigs/humanoid.glb` with `Ragdoll::default()` and one
  `RagdollBone` override. Check: the generated profile drops and settles.
- `toggle_ragdoll`: an animated character walking in a circle; `R` toggles
  `Dynamic` and back with a blend. Check: no pop entering or leaving.

## Active control (phase 7)

- `powered_follow`: a character playing idle, walk and wave in turn while
  `Dynamic` with muscle 1; sliders for muscle and pin. Check: follows the
  clip; at muscle 0 it collapses.
- `hit_reactions`: click a body part (raycast through `RagdollQuery`); the
  hit body and its neighbours weaken and recover. Pistol, rifle and punch
  buttons. Check: local flinch, recovery within about 1 s.
- `partial_ragdoll`: legs animated (per-body muscle 1, pin 1), upper body
  limp while a ball hits it.
- `death_fall`: plays a death clip with muscle fading to limp. Check:
  powered fall, then lies still.
- `blend_to_animation`: lying ragdoll blends back into a standing idle via
  `RagdollBlend`.

## Balance and puppet (phase 9 and 10)

- `balance_push`: a standing puppet; arrow keys push it from four sides
  with adjustable impulse. Overlay draws centre of mass, capture point,
  support polygon and planned step (gizmos). Check: small pushes recover
  without steps, medium pushes take one to three steps, large pushes fall.
- `puppet_showcase` (the Euphoria-style demo): one or more humans stand on
  markers in a small yard with crates and a ramp. Left click shoots
  (rifle), right click punches (short-range ray from the camera),
  `1`..`6` choose pistol, rifle, shotgun, punch, kick, heavy. HUD shows
  each puppet's state. Puppets flinch, stagger with real steps, fall with
  catch-fall arms, lie, get up (face-up or face-down path), walk back to
  their marker and turn to their stored heading. Check: every acceptance
  test of [hit-reaction.md](hit-reaction.md) section 8 also passes headless
  in `bevy_ragdoll_balance`'s tests, and a 60 s recording of the
  showcase shows each state at least once.
- `crowd_brawl`: 16 puppets on markers; random hits each second. Check:
  every puppet returns to its marker after the hits stop.

## Creatures (phase 11)

- `creatures`: a gallery of human, fox, raptor, trex, crocodile and rabbit
  on pedestals; `1`..`6` drop one; `P` toggles powered idle; `A` toggles
  automatic versus tuned profile. Check: each lands and settles without
  exploding; powered idle holds the pose.
- `auto_profile`: loads any glTF given as an argument, generates a profile,
  draws the capsules, and drops it.

## 2D (phase 14)

- `stick_figure_2d`, `chain_2d`, `creature_2d`: sprite limbs on plain
  `Transform` hierarchies.

## Backends and modes

- `custom_backend`: the mock backend from `bevy_ragdoll_conformance` driving
  a ragdoll, with the code a new engine needs annotated.
- `ragdoll_stress`: see [stress-and-benches.md](stress-and-benches.md).
- `headless_sim`: steps 600 fixed steps of a pile without a window and
  prints a pose hash and timing.
- `determinism_check` (phase 15): runs a scenario twice in one process
  and against `--expect <hash>`; exits 1 on mismatch.
