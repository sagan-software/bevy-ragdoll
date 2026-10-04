# What to port from TGF

TGF lives at `~/Code/gitlab.com/liamcurry/tgf` (read only for this plan;
do not edit it). Port means: read the TGF code, then write the new code to
this plan's names and architecture. Copy maths and tuning values exactly;
change structure freely. TGF's own licence does not apply to the port; the
owner relicenses it as MIT OR Apache-2.0.

## Port

- `crates/tgf-ragdoll/src/desc.rs` (354 lines) to `bevy_ragdoll::profile`:
  validation order, `MAX_BODIES = 64`, `REST_CONTACT_MARGIN = 0.01`,
  `segment_distance` (Ericson 5.1.9), `pose_of`, joint frames, joint
  angles, and its unit tests.
- `crates/tgf-rig/src/ragdoll.rs` (181 lines) to `bevy_ragdoll::skein`:
  the `RagdollBody` and `RagdollJoint` Skein components, `AngleRange` in
  degrees for the authoring format, `MAX_ANGLE_DEG`, `problem()`
  validation, and tests. Rename the reflected type path to
  `bevy_ragdoll::skein::RagdollBody` and accept the old path
  `tgf_rig::ragdoll::RagdollBody` when reading glTF extras, so TGF's rig
  GLB loads unchanged.
- `crates/tgf-rig/src/rig.rs` (668 lines), only the parts that read bones,
  `tgf_length` extras and ragdoll capsule nodes from a GLB, to
  `bevy_ragdoll::gltf_rig`. Leave masks, sockets and IK `tgf_follows`
  behind.
- `crates/tgf-ragdoll/src/world.rs` (800 lines) to
  `bevy_ragdoll_rapier3d` and the core: `Settings` and its tuned defaults
  become `RagdollPhysicsSettings` (core) and `RapierRagdollSettings`
  (backend); `body_mass` (inertia floor); the joint build (locked linear
  axes, per-axis limits, locked axes for `0..0`, contacts disabled);
  `apply_motors` (section 2 of [algorithms.md](algorithms.md)); the contact
  filter on `no_contact`; `spawn_lift`; forced sleep after
  `force_sleep_after` when slower than `settle_speed`; `freeze`.
  The interpolation and `advance` logic is replaced by Bevy's fixed
  schedule plus section 6 of algorithms.md.
- `crates/tgf-ragdoll/tests/physics.rs` and `tests/look.rs` to
  `bevy_ragdoll_conformance` physics tier. Replace ET brushes with Bevy
  cuboids (`floor`, `stairs` 0.2 m rise and 0.3 m run). Keep every
  threshold and its comment; convert ET distances with 0.025 m per unit.
  Keep `a_body_shot_onto_stairs_stays_on_them` ignored with its reason.
  Use the game settings (gravity 20 m/s², `ccd: false`) only in the look
  tests, as TGF does; the default gravity elsewhere is 9.81.
- `crates/tgf-ragdoll/benches/pile.rs` to the `step_rapier3d` bench.
- `crates/tgf-game/src/combat/ragdoll/pool.rs` (205 lines) to
  `bevy_ragdoll::budget`.
- `crates/tgf-game/src/combat/ragdoll/lifecycle.rs` (356 lines) to
  `bevy_ragdoll_balance` as the starting point for the death path
  (powered fall, hold, limp, settle) used when a puppet is killed rather
  than knocked down.
- `crates/tgf-game/src/combat/ragdoll/tuning.rs` default values: `fall_strength 1.0`,
  `hold_strength 0.4`, `fall_ms 1200`, `fade_ms 300`, `revive_ms 400`,
  `settle_ms 10000`, `bullet_impulse 12`, `melee_impulse 20`,
  `explosion_impulse 250`, `head_scale 0.6`, `legs_scale 0.8`.
- `crates/tgf-glue/src/ragdoll/skeleton.rs` (280 lines) to
  `bevy_ragdoll::skeleton`: `Skeleton::find` (breadth-first, parents first),
  `ragdoll_locals`, `world_poses`, `blend_locals`, `animated_local`.
- `crates/tgf-glue/src/ragdoll/mod.rs`: `step_blend`, `ReviveBlend`
  (becomes `RagdollBlend` animation), and the `draw_ragdolls` system order
  (after `AnimationSystems`, before `TransformSystems::Propagate`).
- `crates/tgf-glue/src/ragdoll/debug.rs`: capsule gizmos, to a
  `RagdollDebugPlugin` behind a `debug` feature.
- `content/rigs/human/build/human.glb` to
  `assets/rigs/tgf_human/tgf_human.glb` (test rig: 89 UE5-named bones, 16
  capsule bodies totalling 80 kg, no visual mesh). Record it in
  `assets/CREDITS.md` as the owner's own work under MIT OR Apache-2.0.

## Leave behind

- `crates/tgf-ragdoll/src/map.rs` (Enemy Territory brushes), `tgf-collision`,
  `tgf-space`, every ET unit conversion except the look-test geometry.
- Netcode (`body_net.rs`), hit regions, damage, cvars, console commands,
  death clip selection, `BodyPose` replication.
- The dedicated Rapier world and its own thread pool. Backends use the
  engine's Bevy plugin instead.
