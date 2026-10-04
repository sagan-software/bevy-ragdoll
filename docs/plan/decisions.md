# Owner decisions and scope

These are settled. Do not reopen them in code; ask the owner if a phase
seems to need a different answer.

## Project

- Repository: public GitHub `sagan-software/bevy-ragdoll`, default branch
  `main`. Local clone: `~/Code/github.com/sagan-software/bevy-ragdoll`.
- Licence: MIT OR Apache-2.0, files `LICENSE-MIT` and `LICENSE-APACHE`.
  Copyright holder: Bill Curry.
- Fresh history. The code is extracted from TGF
  (`~/Code/gitlab.com/liamcurry/tgf`, owner's own code), rewritten to fit
  this crate. TGF itself is not changed by this plan.
- Engine: Bevy 0.19.1. Physics: bevy_rapier3d 0.36.0 (which pins
  rapier3d `=0.35.0-glamx0.2`; do not add rapier3d 0.36, it uses glam
  0.33) and avian3d 0.7.0.
- Backend order: Rapier 3D, then Avian 3D, then Jolt. 2D (Rapier 2D,
  Avian 2D) after Jolt's decision point.
- Performance is a first-class feature: criterion benchmarks, one stress
  example that switches backends and scenarios, committed baselines, and a
  README recommendation backed by measurements.
- Modes: headless (no rendering), deterministic (where the backend
  supports it), WebAssembly (where the backend supports it).

## Scope of version 1

In scope:

- Ragdoll profiles from Blender (Skein components), from RON files, from
  code, and generated automatically from any skeleton.
- Active ragdolls: per-body muscle (joint motor) and pin (world-space)
  drives toward any animation's pose; hits that weaken muscles locally and
  recover; blending between animation and physics; a simulation budget.
- `bevy_ragdoll_balance`: centre of mass, support polygon, capture point,
  pelvis support, recovery stepping, falling with catch-fall, get-up, and
  the puppet state machine of [reference/hit-reaction.md](reference/hit-reaction.md).
- A Euphoria-style showcase: hit a character, see it flinch, stagger, fall,
  get up, walk back to its marker and face its original heading.
- Creatures besides the human: fox, dinosaur, crocodile, rabbit.

Out of scope for version 1:

- Learned controllers (DeepMimic, AMP). The `RagdollTargetPose` component
  leaves room for them.
- An animation graph of our own. `bevy_animation_graph` and Bevy's
  `AnimationGraph` both write bone `Transform`s, which this crate reads.
- Networking, damage and hit regions. Those belong to games.
- Walking controllers that generate a gait from physics alone. Returning
  to the marker uses animation-driven locomotion followed by the powered
  ragdoll.

## Open owner decisions

Ask before settling any of these:

- Jolt: phase 13 ends with a written cost estimate. The owner chooses
  whether to write bindings for Jolt's ragdoll API.
- Buying Quaternius Universal Animation Library Pro for more clips (prone
  get-up, stagger, turns). Until then the free CC0 Standard tier is used
  and missing clips are procedural (phase 8 lists them).
- Publishing to crates.io (phase 16).
