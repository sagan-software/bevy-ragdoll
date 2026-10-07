# Architecture

This is the contract every phase builds on. Names here are the names to
use. If an implementation detail forces a change, change this file in the
same commit and say why.

Units: metres, kilograms, seconds, radians. World space is Bevy's (Y up).

## Workspace

```
bevy-ragdoll/
  Cargo.toml                    root crate `bevy_ragdoll` plus [workspace], [workspace.package], [workspace.dependencies], lints
  src/                          core: profiles, binding, drives, writeback, hits, budget
  tests/                        core integration tests
  flake.nix, flake.lock         dev shell (rust, bevy system libs, wasm tools)
  PLAN.md                       progress table copied from this plan's phase list
  README.md, CHANGELOG.md, LICENSE-MIT, LICENSE-APACHE
  crates/
    bevy_ragdoll_conformance/   mock backend + contract and physics test suites
    bevy_ragdoll_rapier3d/      backend on bevy_rapier3d 0.36
    bevy_ragdoll_avian3d/       backend on avian3d 0.7 (phase 12)
    bevy_ragdoll_balance/       measuring, pelvis pin, stepping, puppet state machine
    bevy_ragdoll_rapier2d/      phase 14
    bevy_ragdoll_avian2d/       phase 14
  examples/*.rs                 self-contained examples of the root crate, registered in the root Cargo.toml
  benches/bench_main.rs         Criterion entry point (`[[bench]] bench_main`, harness = false)
  benches/benchmarks/           one module per benchmark group plus `support.rs` fixtures
  assets/                       example assets and CREDITS.md
```

Every crate inherits `version`, `edition = "2024"`, `rust-version`,
`license = "MIT OR Apache-2.0"`, `repository` and `[lints]` from the
workspace. Workspace lints: `missing_docs = "warn"`,
`clippy::missing_docs_in_private_items = "warn"`, `unsafe_code = "forbid"`.

Core crate features: `default = ["3d", "gltf", "serialize"]`; `3d`;
`2d` (phase 14); `gltf` (Skein component types and glTF skeleton
helpers); `serialize` (serde + RON asset loader). The core never depends
on a physics crate.

## Data model (core)

`RagdollProfile` is an `Asset` shared by every ragdoll of one rig. It is
validated at construction and immutable afterwards.

- `RagdollProfile::new(spec: ProfileSpec) -> Result<RagdollProfile, ProfileError>`
  is the only constructor. `ProfileSpec` is the serialisable, unvalidated
  form (what RON files and builders produce).
- `ProfileSpec { bodies: Vec<BodySpec>, joints: Vec<JointSpec> }`.
- `BodySpec { bone: String, shape: ShapeSpec, mass: f32, rest: Isometry3d }`.
  `rest` is the bone's rest transform in skeleton space; it is the body's
  frame, so a body's simulated pose equals its bone's world transform.
- `ShapeSpec::Capsule { a: Vec3, b: Vec3, radius: f32 } | Sphere { center: Vec3, radius: f32 } | Cuboid { center: Vec3, rotation: Quat, half_extents: Vec3 }`,
  in the body frame.
- `JointSpec { child: u8, parent: u8, frame: Isometry3d, limits: JointLimits, max_torque: f32 }`.
  `frame` is the child's rest frame seen from the parent body. The joint
  sits at the child bone's head.
- `JointLimits { x: AngleRange, twist: AngleRange, z: AngleRange }`:
  ranges about the child's rest frame axes X, Y (along the bone: twist)
  and Z. `AngleRange { min, max }` in radians, `min <= 0 <= max`,
  `-PI..=PI`. A locked axis is `0..0`. A hinge locks `twist` and `z`.
- Validated output types: `BodyIndex(u8)`, `Mass` (finite, > 0),
  `AngleRange` (validated), `RagdollProfile { bodies, joints, no_contact: Vec<u64>, total_mass, children: Vec<u64> }`.
  `no_contact` and `children` are derived at construction, never stored in
  RON: joint neighbours plus bodies whose shapes touch at rest
  (port TGF `REST_CONTACT_MARGIN = 0.01` and Ericson's segment distance).
- `ProfileError` variants, checked in this order: `Empty`,
  `TooManyBodies(usize)` (max 64), `NotATree` (parents first, body 0 is
  the root, every other body has exactly one joint), `BadMass { body }`,
  `BadShape { body }`, `BadLimit { joint, axis }`, `BadTorque { joint }`,
  `DuplicateBone(String)`.

Profiles come from four sources (phase 3 and 11):

1. RON asset `*.ragdoll.ron` holding a `ProfileSpec` (`RagdollProfileLoader`).
2. glTF with Skein components `RagdollBody { mass_kg }` and
   `RagdollJoint { limit_x, limit_y, limit_z, torque_nm }` on capsule mesh
   nodes parented to bones (TGF's format; port `tgf-rig`).
3. `ProfileBuilder` in code.
4. `auto::generate(&SkeletonView, &AutoOptions) -> Result<ProfileSpec, AutoError>`.

## Runtime entities

On the character (the entity that owns the skeleton, usually the glTF
scene root):

- `Ragdoll { profile: Handle<RagdollProfile> }` or
  `Ragdoll::auto(AutoOptions)`. `#[require(RagdollMode, RagdollDrive, RagdollBlend)]`.
- `RagdollMode` enum: `Animated` (no physics bodies), `Kinematic` (bodies
  follow the animation exactly; they can be hit and raycast), `Dynamic`
  (simulated, driven by muscle and pin), `Frozen` (bodies static where they
  lie; no simulation cost).
- `RagdollDrive { muscle: f32, pin: f32 }`: whole-ragdoll strengths, 0 to 1.
- `RagdollBodyWeights(Vec<BodyWeights>)`: optional per-body multipliers
  `BodyWeights { muscle, pin }`, index = `BodyIndex`. Hits and the balance
  layer write these.
- `RagdollBlend(f32)`: 0 shows the animation, 1 shows physics. Get-up and
  revive blends animate it.
- `RagdollTargetPose`: per body, the target pose in skeleton space this
  fixed step and the previous one (for target velocity). Written by
  `CaptureTargets` from the animated bones, or by user code instead.
- `RagdollTargetAdjust { replace: Vec<Option<Isometry3d>>, additive: Vec<Quat>, root_offset: Isometry3d }`:
  fixed-step edits on top of the captured target, written in
  `RagdollFixedSystems::Behaviour` (balance stepping IK, flinch, catch-fall
  reach, get-up blend). The drive uses, per body, `replace[i]` if set,
  otherwise the captured pose, then rotates it by `additive[i]` in its
  parent's frame. Cleared by its writer, never by the core.
- `RagdollId(u64)`: assigned at activation from a counter; sorting by it
  gives a deterministic order.
- `SkeletonMap` (private component): bones parents first, each with its
  entity, parent index, optional body, and rest local transform.
- Relationship `RagdollBodies` (target) on the character.

One entity per body, top level (no `ChildOf`), spawned by the core when
the mode leaves `Animated`:

- `RagdollBodyOf(Entity)` (relationship to the character),
  `BodyIndex`, `BodyShape`, `BodyMass { mass, min_inertia_radius }`,
  `Transform` (world), `BodyVelocity { linear, angular }` (initial value
  on spawn, read back each step), `BodyPhysicsPose { previous, current }`
  (written by the backend after each step).
- `BodyDriveOutput { pin_force: Vec3, pin_torque: Vec3, joint_torque: Vec3 }`:
  computed by the core each fixed step; a backend applies the parts it
  does not drive natively.
- `BodyKind` enum mirrored from `RagdollMode`: `Kinematic | Dynamic | Fixed`.
- For every non-root body, `JointToParent { parent: Entity, frame: Isometry3d, limits: JointLimits, max_torque: f32 }`
  and `JointDriveTarget { rotation: Quat, angular_velocity: Vec3, stiffness: f32, damping: f32, max_torque: f32 }`.
- `NoContactWith(u64)`: the profile's mask for this body.
- `BodyAtRest` marker: set by the backend when the engine reports sleep.

## Backend contract

A backend is a Bevy plugin that maps the body entities onto one physics
engine. It must:

1. Require `RagdollPlugin` to be added first; panic with a message naming
   both plugins otherwise.
2. Declare `BackendCapabilities { native_joint_motors: bool, asymmetric_swing_limits: bool, deterministic: bool, wasm: bool }`
   as a resource. The core computes `BodyDriveOutput.joint_torque` only
   when `native_joint_motors` is false, and adds soft-limit torques when
   `asymmetric_swing_limits` is false.
3. On `Added<BodyShape>`: create the engine body and collider with the
   shape, mass and inertia floor (`mass * min_inertia_radius^2` on each
   principal axis), initial velocity, CCD, damping, friction, restitution
   from `RagdollPhysicsSettings`.
4. On `Added<JointToParent>`: create a 3-DOF joint with the limits and
   with contacts between the pair disabled.
5. Exclude body pairs in `NoContactWith` from contact (a contact filter
   hook or collision groups).
6. In `RagdollFixedSystems::Apply`: write `JointDriveTarget` into native
   motors, apply `BodyDriveOutput` forces and torques, apply
   `RagdollImpulse` messages, and switch engine body type on `BodyKind`
   changes. Kinematic bodies take the target pose each step.
7. In `RagdollFixedSystems::Read`, after the engine step: write
   `BodyPhysicsPose`, `BodyVelocity` and `BodyAtRest`.
8. Provide backend-neutral queries through `RagdollRaycast` requests and
   `RagdollRaycastResponse` messages, with `BodyContacts` filled in `Read`.
   The contract runner receives only an `App` and a closure that adds a
   backend plugin, so it cannot name a backend-specific associated
   `SystemParam` type. The message and component boundary lets every backend
   run the same query cases without runtime type registration.
9. Despawn its engine objects when body entities despawn.
10. Pass every test in `bevy_ragdoll_conformance` (contract tier and
    physics tier).

## Schedules and sets

```
PostUpdate:
  AnimationSystems (Bevy)              animation writes bone Transforms
  RagdollSystems::Bind                 build SkeletonMap, spawn/despawn bodies on mode change
  RagdollSystems::CaptureTargets       bones -> RagdollTargetPose (skip if user-owned)
  RagdollSystems::Writeback            interpolated body poses -> bone Transforms, blended
  TransformSystems::Propagate (Bevy)

Fixed schedule (RagdollPlugin::fixed_schedule, default FixedUpdate):
  RagdollFixedSystems::Behaviour       balance crate, user AI; writes drives and weights
  RagdollFixedSystems::Drive           core computes JointDriveTarget, BodyDriveOutput
  RagdollFixedSystems::Apply           backend writes engine state
  (engine step)
  RagdollFixedSystems::Read            backend writes BodyPhysicsPose, BodyVelocity, BodyAtRest
  RagdollFixedSystems::AfterStep       core budget, settle detection, events
```

Each backend orders `Apply` before and `Read` after its engine's step set
(bevy_rapier: `PhysicsSet::SyncBackend` / `PhysicsSet::Writeback` in
`FixedUpdate` with `.in_fixed_schedule()`; Avian: `PhysicsSystems::StepSimulation`
in `FixedPostUpdate`, so the Avian backend sets
`RagdollPlugin::fixed_schedule = FixedPostUpdate`). Target capture runs in
`PostUpdate`, so motor targets lag the animation by one frame; document
it.

## Messages and events

- `RagdollImpulse { body: Entity, point: Vec3, impulse: Vec3 }` (Message).
- `RagdollHit { body: Entity, point: Vec3, impulse: Vec3, kind: HitKind }`
  (Message): impulse plus muscle drop with falloff; see phase 7.
- Observer events: `RagdollActivated`, `RagdollSettled`, `RagdollFrozen`,
  `RagdollBudgetEvicted` (all `EntityEvent` on the character).

## Budget

`RagdollBudget { max_dynamic: usize, policy: EvictPolicy::FreezeOldest }`.
When a ragdoll turns `Dynamic` and the budget is full, the oldest dynamic
ragdoll becomes `Frozen` (port TGF `pool.rs`). Settle detection (port TGF
`force_sleep_after`, `settle_speed`) can freeze a limp ragdoll at rest.

## Crate `bevy_ragdoll_balance`

Builds on the core only, never on a physics crate. Adds the measuring
systems (centre of mass, foot contacts through a `ContactQuery` resource
the backend provides, support polygon, capture point), the pelvis pin, the
stepping controller with two-bone IK on the target pose, and the
`PuppetState` machine with the eight states of
[reference/hit-reaction.md](reference/hit-reaction.md) section 7.
Locomotion back to the marker is example code, exposed through a small
`PuppetLocomotion` trait the showcase implements.

## Invariants

- A profile is valid if and only if `RagdollProfile::new` returned it.
  Nothing else constructs one.
- Body order is the profile's order everywhere: parents before children,
  root first.
- Every drive value is clamped to `0..=1` at the component boundary.
- Body entities exist exactly while `RagdollMode != Animated`.
- The core never reads engine types; a backend never reads animation
  types.
