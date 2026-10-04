# Algorithms

Each section gives the formula, its units, where it runs, and the test
that proves it. TGF line references point into
`~/Code/gitlab.com/liamcurry/tgf`.

## 1. Joint angles

The joint frame is the child's rest frame seen from the parent
(`JointSpec::frame`). For parent pose `P` and child pose `C` (world
isometries):

```
q = normalize( (P.rotation * frame.rotation)^-1 * C.rotation )
if q.w < 0 { q = -q }                       // shortest arc
angles = ( 2*atan2(q.x, q.w), 2*atan2(q.y, q.w), 2*atan2(q.z, q.w) )   // radians about X, Y (twist), Z
```

Port: `crates/tgf-ragdoll/src/desc.rs` `relative_rotation` and
`joint_angles`. At rest every angle is 0. This per-axis measure is exact
for rotation about one axis and approximate for combined rotations.
Rapier 0.35's motor error uses `2*asin(q_err.imag[axis])`
([physics-api.md](physics-api.md) A5); the two agree to first order. Test:
`joint_angles_measure_about_each_axis` (port) plus a round trip for each
single axis at `-PI/2, -0.1, 0, 0.1, PI/2`.

Clamp target angles into the joint's limits before writing motors (TGF
`RagdollWorld::set_target`).

## 2. Muscle drive (joint motors)

Per joint, with muscle `m` in `0..=1` (whole-ragdoll muscle times the
child body's weight), natural frequency `f` (Hz, default 4.0), damping
ratio `zeta` (default 1.0), joint friction `phi` (fraction, default
0.05), friction rate `r` (1/s, default 20.0), torque scale `s` (default
1.0), and the joint's `max_torque` `T` (N·m):

```
omega     = 2*PI*f                              rad/s
stiffness = omega^2 * m                         1/s^2   (acceleration-based)
damping   = 2*zeta*omega*sqrt(m) + r            1/s
limit     = T * s * (phi + m)                   N·m
```

Port: TGF `RagdollWorld::apply_motors`. With no target, `m` counts as 0,
so only friction remains. Rapier: one motor per unlocked angular axis,
`MotorModel::AccelerationBased`, `set_motor(axis, target_angle, target_velocity, stiffness, damping)`,
`set_motor_max_force(axis, limit)`.

Target velocity comes from two consecutive targets (section 5) projected
on each axis of the joint frame.

## 3. Torque drive (backends without spherical motors)

When `BackendCapabilities::native_joint_motors` is false, the core computes
a stable PD torque per joint in world space and the backend applies `+tau`
to the child and `-tau` to the parent:

```
F      = P.rotation * frame.rotation                      joint frame in world
q_rel  = F^-1 * C.rotation                                current relative rotation
q_err  = q_target * q_rel^-1                              shortest arc (flip if w < 0)
e      = F * (axis(q_err) * angle(q_err))                 rad, world
dw     = (w_child - w_parent) - F * w_target              rad/s, world
I      = child inertia about e's axis (use the child's largest principal
         moment if e is near zero)                         kg·m^2
tau    = I * ( stiffness * e - (damping + stiffness*dt) * dw )
tau    = tau * min(1, limit / |tau|)
```

The `stiffness*dt` term is the stable-PD correction (Tan, Liu, Turk 2011):
it keeps high gains stable at 60 Hz. `stiffness`, `damping` and `limit`
are section 2's values, so the same tuning drives both backend kinds.
Test (core, pure function): a single-body pendulum integrated with
semi-implicit Euler at 60 Hz converges to the target within 1 degree in
1 s at `m = 1` and never overshoots by more than 5 degrees.

Soft limits for engines with a symmetric swing cone: if an axis angle
leaves `[min, max]`, add `tau_limit = I * k_lim * excess` about that axis,
`k_lim = (2*PI*8)^2`, opposing the excess, plus the same damping form.

## 4. Pin drive (world space)

Per body with pin weight `p`, target position `x_t`, velocity `v_t`,
rotation `q_t`, angular velocity `w_t` (all world, from the target pose and
the character's world transform), pin frequency `f_p` (Hz), damping ratio
`zeta_p`, maximum force `F_max` (N), torque `T_max` (N·m), distance falloff
`k` (1/m):

```
omega_p = 2*PI*f_p
dist    = |x_t - x|
F       = p * mass * ( omega_p^2 * (x_t - x) + 2*zeta_p*omega_p * (v_t - v) )
F       = F * min(1, p*F_max / (1 + k*dist) / |F|)
e_r     = axis_angle(q_t * q^-1)
tau     = p * I * ( omega_p^2 * e_r + 2*zeta_p*omega_p * (w_t - w) )
tau     = tau * min(1, p*T_max / |tau|)
```

Defaults per state live in [hit-reaction.md](hit-reaction.md) 3.6.
Test: one free body pinned with `p = 1` reaches within 0.01 m of a target
1 m away within 2 s and does not exceed 10 % overshoot.

## 5. Target capture and target velocity

In `PostUpdate` after `AnimationSystems`, for each ragdoll whose target is
captured automatically:

1. Walk `SkeletonMap` parents first. `world[i] = world[parent] * local[i]`
   with `local[i]` the bone's current `Transform` as an `Affine3A`. The
   root's parent world is the character's world transform composed from
   its own `Transform` and its ancestors' `Transform`s (do not read
   `GlobalTransform`; it is one frame stale here).
2. Convert each body bone's world affine to skeleton space (divide by the
   character's world affine), drop scale (`to_scale_rotation_translation`,
   keep rotation and translation).
3. Shift `current` into `previous`, store the new poses in `current`, and
   record the time between them. Target velocity per body:
   `v = (current.t - previous.t) / dt`, `w` from
   `q = current.r * previous.r^-1` as in TGF `BodyState::from_poses`.

## 6. Interpolation and writeback

After each fixed step the backend shifts `BodyPhysicsPose.current` into
`previous` and stores the new pose. In `PostUpdate`:

```
a    = Time<Fixed>::overstep_fraction()          0..1
pose = { t: lerp(prev.t, cur.t, a), r: slerp(prev.r, cur.r, a) }
```

Writeback, parents first (port TGF `crates/tgf-glue/src/ragdoll/skeleton.rs`
`ragdoll_locals` and `animated_local`):

1. For a bone with a body: `world_phys = pose` (body frame equals the bone
   frame). For a bone without one: `world_phys = world_phys[parent] * animated_local`.
2. `local_phys = world_phys[parent]^-1 * world_phys`, keeping the
   animated bone's scale.
3. `local = blend(animated_local, local_phys, RagdollBlend * per-bone weight)`
   with lerp on translation and slerp on rotation.
4. Write `Transform` only if it changed by more than `1e-6`, so
   change detection stays cheap.

## 7. Spawn state from animation

When a ragdoll turns `Dynamic`, each body starts at its target pose with
the target velocity (section 5), so a running character keeps running
into the fall. Port TGF `RagdollWorld::spawn_lift` as an optional
`RagdollPhysicsSettings::max_spawn_lift` (default 0.5 m) that lifts the
whole ragdoll out of static geometry it starts inside; the backend
implements the overlap test through its own shape queries.

## 8. Impulse clamp along the chain

From [hit-reaction.md](hit-reaction.md) 2.2: the hit body takes at most
`m_b * dv_max` (`dv_max = 3.0` m/s) of the impulse; the remainder passes to
its parent at the joint anchor, repeating up to the root, which takes the
rest at its centre. Test: a 12 N·s impulse on a 0.5 kg hand never gives
the hand more than 3.0 m/s plus the parent's induced speed after one
step.

## 9. Automatic profile generation

Input: `SkeletonView { bones: Vec<SkeletonBone { name, parent: Option<usize>, rest_world: Affine3A }>, skin: Option<SkinSample> }`
where `SkinSample` holds vertex positions and per-vertex (bone, weight)
pairs from the skinned mesh in the same space. Output: `ProfileSpec`.

1. Ignore bones whose lowercased name contains any of `twist`, `leaf`,
   `_end`, `ik_`, `ik`, `ctrl`, `socket`, `helper`, `ball` (feet keep
   `foot` only), `finger`, `thumb`, `index`, `middle`, `ring`, `pinky`,
   `metacarpal`, `toe`, `eye`, `jaw`, `ear` (configurable
   `AutoOptions::ignore`).
2. Bone length: distance from the bone's head to its main child's head
   (main child = the kept child farthest away); for leaves, the parent's
   length times 0.5.
3. Merge chains: a kept bone shorter than `AutoOptions::min_length`
   (default 0.06 m times the skeleton's height / 1.8 m) merges into its
   parent's body.
4. Shape: a capsule from head to tail. Radius from `SkinSample` when
   present: the 75th percentile of the distance from each vertex whose
   weight for this bone is at least 0.5 to the segment, clamped to
   `[0.15, 0.6] * length`. Without a skin: `0.22 * length` for limbs and
   tails, `0.45 * length` for torso bones (role step 6).
5. Mass: capsule volume times 985 kg/m^3, then scale every mass so the
   total equals `AutoOptions::total_mass` when it is set.
6. Role by topology, then by name: the spine chain runs from the root to
   the bone with the most kept descendants that has two or more kept
   child chains (shoulders or hips). Chains leaving the pelvis end opposite
   the head are legs, or a tail if the chain has more than three bones and
   no foot-like leaf below the pelvis height. Chains leaving the chest are
   arms; the chain continuing up is neck and head. Name hints (`spine`,
   `neck`, `head`, `arm`, `leg`, `thigh`, `calf`, `shin`, `foot`, `hand`,
   `tail`, `clavicle`, `shoulder`, `hip`, `upperleg`, `lowerleg`) override
   topology when present.
7. Limits and torques by role (radians; torque N·m at 70 kg, scaled
   linearly by total mass):
   - spine and chest: x -0.5..0.5, twist -0.4..0.4, z -0.35..0.35, 300
   - neck and head: x -0.6..0.7, twist -0.7..0.7, z -0.5..0.5, 60
   - upper arm: x -1.5..1.5, twist -1.2..1.2, z -1.4..0.6, 120
   - forearm (hinge): x 0..2.4, locked, locked, 70
   - hand: x -1.0..1.0, twist -0.3..0.3, z -0.5..0.5, 20
   - thigh: x -1.6..0.5, twist -0.6..0.6, z -0.8..0.4, 250
   - shin (hinge): x -2.3..0, locked, locked, 180
   - foot: x -0.6..0.8, twist -0.2..0.2, z -0.3..0.3, 80
   - tail segment: x -0.35..0.35, twist -0.1..0.1, z -0.35..0.35, torque from the mass beyond the joint times 9.81 times the chain length times 0.5
   - quadruped leg segments: as thigh, shin and foot with the hinge sign
     taken from the rest bend direction (positive if the child bends
     forward in the rest pose).
   Hinge sign is never assumed: measure the rest-pose bend between the bone
   and its child and choose the range on the side the joint already bends.
8. Validate with `RagdollProfile::new`; the generator returns its spec and
   any warnings (`AutoWarning::MergedBone`, `AutoWarning::GuessedRole`).

Tests: the TGF human skeleton gives 14 to 18 bodies, total mass equal to
the option, knees as hinges bending backward; each creature in phase 11
gives a profile that passes the physics conformance "drop and settle"
case.
