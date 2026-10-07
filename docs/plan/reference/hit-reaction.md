# Active-ragdoll hit reaction: design for a Bevy 0.19 crate and showcase

Status: research design, 2026-10-03. Target: a humanoid stands at a marker,
reacts physically to shots and punches, staggers or falls, gets up, walks back
and faces its original heading.

Marking rules used below:

- A statement with a URL comes from that fetched source.
- "(from memory)" marks a statement not checked against a fetched source.
- "(design)" marks a value chosen here by calculation or judgement. Tune it
  against the acceptance tests in section 8.

Units: metres, kilograms, seconds, radians unless stated. World is Bevy's
(Y up). The physics step is 60 Hz.

## 0. Where this lands in bevy-ragdoll

The core crate (`bevy_ragdoll`) provides what this design assumes from the
physics side:

1. Per-body muscle strength: `RagdollDrive::muscle` times
   `RagdollBodyWeights[i].muscle`, used by the muscle drive
   ([algorithms.md](algorithms.md) section 2).
2. Per-body pin: `RagdollDrive::pin` times `RagdollBodyWeights[i].pin`
   ([algorithms.md](algorithms.md) section 4). This design pins only the
   pelvis and chest.
3. Contacts per body and raycasts through the backend's `RagdollQuery`
   ([../architecture.md](../architecture.md), backend contract item 8).
4. Body poses and velocities in `BodyPhysicsPose` and `BodyVelocity`, from
   which the balance crate computes the centre of mass.

Everything in sections 2 to 7 is implemented in `bevy_ragdoll_balance`
(phases 9 and 10), except the strength drop and recovery of 2.3 and 2.4,
which live in the core's hit module (phase 7) so games without balance get
flinches too. The motor defaults quoted below (4 Hz, critically damped,
joint friction 0.05) are tuned values from an earlier game project.

## 1. How practical systems are built

### 1.1 Unity PuppetMaster (Root-Motion)

Sources: http://root-motion.com/puppetmasterdox/html/page5.html,
http://root-motion.com/puppetmasterdox/html/class_root_motion_1_1_dynamics_1_1_puppet_master.html,
http://root-motion.com/puppetmasterdox/html/class_root_motion_1_1_dynamics_1_1_behaviour_puppet.html,
https://root-motion.com/puppetmasterdox/html/page10.html.

- Three weights per muscle (body). `pinWeight` pulls the muscle to its
  target's world position "using simple AddForce". `muscleWeight` is the
  normalised strength of the joint drive. `mappingWeight` blends the visible
  character between the animation and the ragdoll pose.
- Defaults: `pinWeight = 1`, `muscleWeight = 1`, `muscleSpring = 100`,
  `muscleDamper = 0`, `pinPow = 4` (slope of the pin curve while
  interpolating), `pinDistanceFalloff = 5` (pin force falls off with
  distance to the target).
- BehaviourPuppet has three states: Puppet (normal, pinned), Unpinned (lost
  balance, "animated physically in muscle space only"), GetUp (transition
  from Unpinned to Puppet).
- A collision or a scripted hit lowers pin weight. `collisionResistance = 3`
  ("smaller value means more unpinning"). Per group, `unpinParents`,
  `unpinChildren` and `unpinGroup` spread the loss along the skeleton.
  `minPinWeight` floors it. `regainPinSpeed = 1` restores it.
- Loss of balance: when a muscle is farther than `knockOutDistance = 1` from
  its target and its pin weight is below `pinWeightThreshold = 1`, the puppet
  is knocked out. On knockout muscles drop to `unpinnedMuscleWeightMlp = 0.3`.
  `maxRigidbodyVelocity = 10` clamps body speed while unpinned.
- Get-up: `canGetUp = true`, wait at least `getUpDelay = 5` s, then until the
  hip speed is below `maxGetUpVelocity = 0.3` m/s. Blend the target from the
  ragdoll pose to the get-up animation over `blendToAnimationTime = 0.2` s.
  Stay in GetUp at least `minGetUpDuration = 1` s. During GetUp, collision
  resistance x2 (`getUpCollisionResistanceMlp`), regain-pin speed x2
  (`getUpRegainPinSpeedMlp`), knockout distance x10
  (`getUpKnockOutDistanceMlp`), so the character does not fall again at once.
  `getUpOffsetProne` and `getUpOffsetSupine` offset the target root from the
  hip bone "if your character slides a bit when starting to get up".
  `IsProne()` decides face down versus face up.
- Troubleshooting from the docs: "snake feet" (never falls) means collision
  resistance is too high. Repeated failed get-ups need higher GetUp
  multipliers. If leg hits never unbalance, raise unpinParents/Children/Group
  for hips, legs and feet, or lower knockOutDistance.

### 1.2 Unreal Physical Animation component

Source: https://docs.unrealengine.com/4.27/en-US/InteractiveExperiences/Physics/PhysicsAssetEditor/HowTo/ApplyPhysicalAnimationProfile/
and the forum thread https://forums.unrealengine.com/t/hit-reaction-with-uphysicalanimationcomponent/96879.

- `ApplyPhysicalAnimationProfileBelow(bone, profile)` sets drive strengths
  for all bodies under a bone. `SetAllBodiesBelowSimulatePhysics(bone, true)`
  makes them simulate. The usual hit reaction sets
  `SetAllBodiesPhysicsBlendWeight(1.0)` on hit and drives it back to 0 with a
  timeline curve.
- A profile holds orientation strength, angular velocity strength, position
  strength and velocity strength per body, plus a max force (from memory).
- Common practice: simulate only from `spine_01` or the hit limb down, keep
  pelvis and feet kinematic for small hits, and blend back over about 0.3 to
  0.6 s (from memory).
- For get-up, Unreal's "Pose Snapshot" captures the ragdoll pose, and the
  anim graph blends from the snapshot to the get-up montage (from memory;
  the 4.27 docs page was behind a Cloudflare challenge).

### 1.3 Overgrowth (David Rosen, GDC 2014)

Source page: https://gdcvault.com/play/1020583/Animation-Bootcamp-An-Indie-Approach
and https://www.wolfire.com/blog/2014/05/gdc-2014-procedural-animation-video/.
The talk covers "simple procedural techniques to achieve interactive and
fluid animations using very few key frames".

From memory: the character is driven by a few key poses blended by physical
quantities (speed, lean from acceleration). Ragdolls use damped springs
toward an animation pose. Get-up chooses a back or front recovery from the
ragdoll's orientation and blends from the ragdoll pose. Hits add a lean or
pose offset to the animation, then the spring system smooths it.

### 1.4 Jolt Ragdoll

Source: https://jrouwe.github.io/JoltPhysics/class_ragdoll.html.

- `DriveToPoseUsingMotors(inPose)` "activat[es] the motors on each
  constraint". The overload `DriveToPoseUsingMotors(inPrevPose, inPose,
  inDeltaTime)` "drives both to target position and velocity": it feeds the
  animation's velocity forward.
- `DriveToPoseUsingKinematics(inPose, inDeltaTime)` sets body velocities so
  the ragdoll reaches the pose in `inDeltaTime`. This is the fully pinned
  mode.

Lesson: feed the target velocity forward, not only the target position, or a
moving animation lags by one spring time constant.

### 1.5 NaturalMotion Euphoria (as exposed in GTA V)

Euphoria is a set of "behaviours ... such as balancing, staggering or
protecting the character" running on top of the game's physics
(https://eprints.bournemouth.ac.uk/25038/1/GREER%2C%20David_DEng._2015.pdf,
https://web.archive.org/web/20100103165025/naturalmotion.com/faq.htm).
GTA V exposes the behaviour messages and their parameters with defaults. The
FiveM source lists them:
https://github.com/citizenfx/fivem/blob/8a2d502a/code/client/clrcore/External/EuphoriaHelpers.cs
(also https://nitanmarcel.github.io/shvdn-docs.github.io/class_g_t_a_1_1_natural_motion_1_1_configure_balance_helper.html).
Euphoria stiffness values are in its own unit, roughly a spring natural
frequency, with damping 1 meaning critical (from memory).

configureBalance (the dynamic balancer used by every standing behaviour):

- `stepHeight = 0.1` m (above 0.2 is high), `legStiffness = 12`.
- `predictionTime = 0.2` s: "amount of time into the future that the
  character tries to step to. bigger values try to recover with fewer,
  bigger steps". `predictionTimeHip = 0.3` s.
- `balanceAbortThreshold = 0.6` (0..1): "when the character gives up and goes
  into a fall. Larger values mean that the balancer can lean more".
- `giveUpHeight = 0.5` m: "height between lowest foot and COM below which
  balancer will give up".
- `maxSteps = 100`, `maxBalanceTime = 50` s, `fallType = RampDownStiffness`,
  `fallMult = 1`.
- `stableLinSpeedThresh = 0.3` m/s and `stableRotSpeedThresh = 0.3` rad/s
  for "successful balance".
- `legsApartRestep = 0.2` m: if legs end up more than hip width + 0.2 apart
  while balanced, take another step. `avoidLeg` with `avoidFootWidth = 0.1`
  stops steps across the stance foot. `stepIfInSupport = true`.
- `resistAcc = 0.5`: "level of cheat force added to character". Euphoria
  also uses external "cheat" forces (the `stayUpright` and `forceLean*`
  messages).
- `backwardsLeanCutoff`: "0.6 is a sensible value" for cutting stay-upright
  forces on a backward lean.

configureBullets and applyBulletImpulse:

- `impulsePeriod = 0.1` s: a bullet impulse is spread over 0.1 s in a
  triangular profile.
- `impulseTorqueScale = 1`; the hit-point impulse equals a centre impulse
  plus a torque.
- `impulseReductionPerShot = 0`, `impulseRecovery = 0`,
  `impulseMinLeakage = 0.2`.
- `doCounterImpulse = false`, `counterImpulseMag = 0.5`: optional counter
  impulse at the pelvis to keep the character in place.
- `impulseSpineAngStart = 0.7`, `impulseSpineAngEnd = 0.2`: scale impulses
  by the dot of hip-to-head with up (a leaning character takes less).
- `impulseAirApplyAbove = 399`: an impulse above this "is a shotgun or
  cannon". This shows game impulses are far above real bullet momentum.
- `applyBulletImpulse.extraShare`: an extra share applied to spine0, which
  "approximates the COM".

shot (the gunshot reaction):

- `bodyStiffness = 11`, `armStiffness = 10`, `neckStiffness = 14`,
  `spineDamping = 1`.
- `loosenessAmount = 1`, `minArmsLooseness = 0.1`, `minLegsLooseness = 0.1`.
- `initialWeaknessZeroDuration = 0`, `initialWeaknessRampDuration = 0.4` s:
  the upper body drops to near zero stiffness and ramps back over 0.4 s.
- `timeBeforeReachForWound = 0.2` s, `reachForWound = true`,
  `grabHoldTime = 2` s.
- Conscious pain: `cpainSmooth2Time = 0.2`, `cpainMag = 1`,
  `cpainSmooth2Zero = 1.5` s.
- `fallingReaction`: 0 Rollup, 1 Catchfall, 2 rollDownStairs, 3 smartFall.

staggerFall:

- `lowerBodyStiffness = 13` falling to `lowerBodyStiffnessEnd = 8`.
- `perStepReduction1 = 0.7`: leg stiffness drops every step "to make the
  character fall over". `stepsTillStartEnd = 2`.
- `predictionTime = 0.1` s, `spineBendMult = 0.4`, `leanInDirMaxF = 0.1`,
  `leanInDirMaxB = 0.3`, `headLookAtVelProb = 1`, `turn2VelProb = 1`.

catchFall: `torsoStiffness = 9`, `legsStiffness = 6`, `armsStiffness = 15`,
`backwardsMinArmOffset = -0.3`, `forwardMaxArmOffset = 0.4`.

bodyWrithe: arm, back and leg stiffness 13, damping 0.5, period 1 s.

upperBodyFlinch: `bodyStiffness = 11`, `backBendAmount = -0.6`,
`noiseScale = 0.1`, hands 0.1 m apart.

### 1.6 Other talks (from memory)

- Michal Mach, "Physics Animation in Uncharted 4" (GDC 2017): powered
  ragdoll with per-body motor strengths driven by animation, hit reactions
  as strength drops plus impulses, blending back to animation (from memory).
- EA and Ubisoft talks on "physical hit reactions" use the same pattern:
  keyframed animation as the target, powered ragdoll that follows it, a
  local strength drop on hit, and full ragdoll only on knockdown (from
  memory). No specific talk was fetched.

### 1.7 The architecture this design adopts

```
            +-------------------+   root yaw/pos   +----------------------+
 Brain ---> | Locomotion / FSM  | ---------------> | Animated target pose |
 (section 7)+-------------------+                  | (Bevy AnimationGraph |
                     |                             |  + additive flinch   |
                     | state params                |  + foot IK targets)  |
                     v                             +----------+-----------+
            +-------------------+                             | target poses
            | Reaction tuning   |   muscle[i], pin[i]         | + velocities
            | (strength map,    | ----------------------------+
            |  balance, pins)   |                             v
            +-------------------+                  +----------------------+
                     ^                             | physics backend      |
                     | CoM, contacts, velocities   | motors + pins +      |
                     +---------------------------- | impulses (Rapier)    |
                                                   +----------+-----------+
                                                              | body poses
                                                              v
                                                   visible skeleton
```

One fixed-step system order per physics tick:

1. Read hits queued this tick; apply impulses (section 2).
2. Measure: CoM, CoM velocity, foot contacts, support polygon, capture
   point, tilt (section 3).
3. Run the state machine (section 7).
4. Update the animated target: state clip, additive flinch, foot IK for a
   recovery step, root motion.
5. Update per-body `muscle[i]` (strength recovery) and `pin[i]`.
6. Write motor targets (position plus feed-forward velocity) and pin forces.
7. Step Rapier.

## 2. Hit reaction

### 2.1 Impulse sizes

Real bullet momentum is small (design arithmetic from typical loads, from
memory for the loads):

- 9 mm, 8 g at 360 m/s: 2.9 kg·m/s.
- 5.56 mm, 4 g at 930 m/s: 3.7 kg·m/s.
- 7.62x51 mm, 9.5 g at 840 m/s: 8.0 kg·m/s.
- 12-gauge 00 buckshot, 35 g total at 400 m/s: 14 kg·m/s.

On a 70 kg body, 3 kg·m/s changes CoM velocity by 0.04 m/s. That is
invisible. Games exaggerate; GTA's configureBullets treats impulses above
399 as "shotgun or cannon" (source in 1.5). Kickback's presets use
`base_impulse` 8 (bullet), 15 (melee), 20 (shotgun), 40 (explosion), each
times a transfer ratio of 0.15 to 1.0
(https://github.com/blugart-dev/kickback/blob/HEAD/addons/kickback/resources/impact_profile.gd).
Those are Godot units on light bodies and do not map directly.

A punch: fist speed 7 to 10 m/s with an effective mass of 2 to 4 kg gives
15 to 40 kg·m/s (from memory; Walilko et al. 2005 measured Olympic boxers).

Design values for the showcase (game impulses, kg·m/s, total for one hit):

```toml
[impulse]          # J applied at the hit point, along the shot direction
pistol   = 12.0    # flinch; one step at most from the chest
rifle    = 20.0    # flinch plus a small step
shotgun  = 60.0    # stagger; spread over 8 pellets of 7.5 each
punch    = 30.0    # flinch and step; head punch twists the neck
kick     = 60.0    # stagger
heavy    = 120.0   # knockdown (bat, shoulder charge)
explosion = 200.0  # knockdown, plus 0.3 * J upward at the pelvis
```

### 2.2 Applying the impulse

1. Find the hit body `b` (ray hit against ragdoll capsules) and the world
   hit point `p`.
2. Clamp per-body velocity change so light bodies do not explode. A 12
   kg·m/s impulse on a 0.5 kg hand is 24 m/s. Rule (design):
   `J_b = min(|J|, m_b * dv_max)` with `dv_max = 3.0` m/s. Apply `J_b` at
   `p` on `b`. Pass the remainder `J - J_b` to the parent body at the
   parent joint anchor, repeating until the pelvis takes the rest at its
   centre.
3. Optional GTA-style spread: apply the impulse as forces over
   `impulse_period = 0.05` s (design; GTA uses 0.1 s) with a triangular
   profile. Use a single impulse first; add spreading only if contacts
   explode.
4. Clamp every body to `max_linear_speed = 10` m/s and
   `max_angular_speed = 20` rad/s while not Falling (PuppetMaster
   `maxRigidbodyVelocity = 10`; kickback `max_linear_velocity = 10`,
   `max_angular_velocity = 20`).
5. Record `hit = { body, point, dir, J, time }` for the state machine.

### 2.3 Strength drop with falloff along the skeleton

Each body has a base muscle strength `base[i]` (1.0 = full motor stiffness)
and a current `muscle[i]`. Kickback's per-bone relative strengths are a good
shape: Hips 0.65, Spine/Chest 0.60, Head 0.35, UpperArm 0.45, LowerArm 0.40,
Hand 0.25, UpperLeg 0.55, LowerLeg 0.45, Foot 0.30
(https://github.com/blugart-dev/kickback/blob/HEAD/addons/kickback/resources/ragdoll_tuning.gd).
In this design `base[i] = 1.0` for every body and the per-joint torque limit
from the rig carries the size difference.

Drop rule (design, after PuppetMaster unpinParents/unpinChildren and
kickback `strength_reduction` / `strength_spread`):

```
severity  s  = clamp(|J| / J_ref, 0, 1)            J_ref = 40 kg·m/s
drop at hit  R0 = r_max * s                         r_max = 0.85
for every body j at hop distance k from the hit body (k <= max_hops):
    a  = a_up   if j is an ancestor of the hit body (toward pelvis)
         a_down otherwise (toward hands, feet, head)
    Rk = R0 * a^k
    muscle[j] = max(floor[j], min(muscle[j], base[j] * (1 - Rk)))
```

Defaults:

```toml
[strength_drop]
j_ref     = 40.0   # kg·m/s that gives the full drop
r_max     = 0.85   # kickback bullet 0.85, melee 0.88, shotgun 0.92
a_up      = 0.5    # toward the pelvis (weaker spread; PuppetMaster unpinParents)
a_down    = 0.7    # toward extremities (PuppetMaster unpinChildren)
max_hops  = 2      # 1 for pistol, 2 for rifle/punch, 3 for shotgun/kick
floor     = { pelvis = 0.15, spine = 0.10, thigh = 0.10, calf = 0.08, foot = 0.05, other = 0.05 }
```

The floors are kickback's `min_strength` values. With `max_hops = 2` a chest
hit weakens chest, spine, neck, both upper arms and the pelvis a little; the
legs keep their strength, so the character stays standing. A thigh hit
weakens thigh, calf, foot and pelvis; with a large J this is what produces a
stumble on that leg.

Hits within `rapid_fire_window = 0.3` s stack: multiply R0 by
`1 + 0.3 * streak` (kickback `hit_streak_multiplier = 0.3`).

### 2.4 Strength recovery

```
after recovery_delay since the last hit:
    muscle[i] = min(base[i], muscle[i] + rate * dt)
```

```toml
[recovery]
delay              = 0.10  # s
rate_idle          = 1.5   # per s; 0.15 -> 1.0 in 0.57 s, near GTA's 0.4 s ramp
rate_stagger       = 0.8   # per s while staggering
order_delay        = { pelvis = 0.0, spine = 0.0, chest = 0.05, thigh = 0.10, calf = 0.15, foot = 0.20, upperarm = 0.20, lowerarm = 0.25, head = 0.25, hand = 0.30 }
```

`order_delay` is kickback's `ramp_delay`: the core recovers first and the
extremities after it.

### 2.5 Additive flinch on the animated target

The impulse moves the bodies; the flinch makes the target pose react too,
so the muscles do not pull the body straight back like a spring toy.

- Six additive clips, or procedural rotations if no clips exist: head,
  chest-front, chest-back, stomach, left side, right side. Pick by hit body
  and the sign of `dot(dir, character_forward)` and
  `dot(dir, character_right)`.
- Procedural fallback (design): rotate the spine bones about
  `axis = normalize(cross(up, dir))` by `angle = 0.35 * s` rad, split
  0.4 / 0.35 / 0.25 over spine_01..spine_03 (or the rig's spine chain).
  For a head hit rotate neck and head by `0.5 * s` rad. Add a yaw twist of
  `0.3 * s * sign(dot(cross(dir, p - chest_centre), up))` on the chest
  (GTA `exagTwistMag = 0.5` relative to lean 1.0).
- Envelope weight `w(t)`: rise linearly over 0.05 s, hold 0.05 s, fall with
  smoothstep over 0.35 s. Total 0.45 s.
- New hit while active: restart the envelope and take the larger amplitude.

## 3. Balance with stepping

### 3.1 Measurements every tick

```
m      = sum(m_i)
com    = sum(m_i * x_i) / m
v_com  = sum(m_i * v_i) / m
ground = height of the ground under the pelvis (ray), or the lowest foot contact
h      = clamp(com.y - ground, 0.6, 1.2)
omega  = sqrt(g / h)                       g = 9.81
cp     = com_xz + v_com_xz / omega         instantaneous capture point
```

The capture point is "a point on the ground where the robot can step to in
order to bring itself to a complete stop"
(https://www.cs.cmu.edu/~cga/legs/Pratt_Goswami_Humanoids2006.pdf). With
the ZMP held at a fixed point `p`, the capture point diverges as
`cp(t) - p = (cp(0) - p) * e^(omega t)`, and `d com/dt = omega (cp - com)`
(https://scaron.info/robotics/capture-point.html).

For a standing human, `h ≈ 1.0` m gives `omega ≈ 3.13` 1/s.

### 3.2 Foot contacts and the support polygon

- A foot is planted when its body has a contact with a static collider whose
  normal has `y > 0.7`, and the foot's speed is below 0.5 m/s.
- Model each foot as a rectangle in the foot frame (from memory for
  anthropometry): heel 0.06 m behind the ankle, toe 0.18 m ahead, half width
  0.05 m. Project the four corners of each planted foot to the ground plane.
- Support polygon = convex hull of all planted corners (0, 4 or 8 points).
  With no planted foot, the character is airborne; skip stepping.
- `margin = signed distance from cp to the polygon` (positive outside).

Kickback, by its own audit, uses only a static estimate ("no extrapolated
CoM or contact support polygon yet"). This design uses the capture point.

### 3.3 What resists small pushes before a step

Three mechanisms act together (design):

1. Ankle strategy: foot friction plus ankle motors hold the body while
   `margin < 0`.
2. Hand of god on the pelvis (3.6).
3. The weakened upper body absorbs part of the impulse as rotation.

### 3.4 When to step

```
trigger a recovery step when all hold:
    state in {HitReact, Stagger, Idle, Returning}
    at least one foot planted, no step in progress
    margin > step_margin                     step_margin = 0.02 m
    time since the last step landed > 0.05 s
```

GTA's balancer also steps if the legs end up too far apart while balanced
(`legsApartRestep = 0.2`). Do the same as a "tidy step" after recovery
(3.8).

### 3.5 Where to step and how

Choose the swing foot:

1. Let `u = normalize(cp - pelvis_xz)` and `r = character right`.
2. If `|dot(u, r)| > 0.5`, step with the foot on that side (cp to the right
   steps with the right foot).
3. Otherwise step with the foot farther from `cp` (GTA
   `alwaysStepWithFarthest`); that foot carries less load.

Choose the target. `s` is the stance foot's ankle on the ground,
`T = swing duration`:

```
T        = clamp(0.20 + 0.15 * |cp - s|, 0.25, 0.40)             s
cp_land  = s + (cp - s) * exp(omega * T)                        predicted cp at landing
target   = cp_land + normalize(v_com_xz) * 0.05                  small overshoot
```

Clamp the target in the stance foot frame (forward x, outward y):

```toml
[step_limits]          # relative to the stance ankle
forward_max   = 0.70
backward_max  = 0.50
outward_max   = 0.50
inward_min    = 0.12   # do not cross the stance foot (GTA avoidFootWidth 0.1)
```

If the unclamped target exceeds the clamp by more than 0.25 m, count the step
as "short". A short step still happens (it slows the fall), and it feeds the
give-up test.

A cheaper alternative that matches GTA: `target = com_xz + v_com_xz *
prediction_time`, `prediction_time = 0.2` s (configureBalance) or 0.1 s
(staggerFall). Use the capture-point form; keep this as a fallback if the
capture point steps look too large.

Swing trajectory for the IK target of the swing ankle (`t` from 0 to `T`):

```
sigma   = t / T
e       = sigma * sigma * (3 - 2 * sigma)                       smoothstep
pos     = lerp(start, target, e) + up * lift * 4 * sigma * (1 - sigma)
lift    = 0.10 m                                                GTA stepHeight 0.1
yaw     = slerp(start_yaw, stance_yaw + clamp(turn, ±0.5 rad), e)
```

- Re-plan the target every tick during the first 60 % of the swing; freeze
  it after that.
- Land when `sigma >= 1`, or when the swing foot gets a ground contact
  after `sigma > 0.6`.
- Solve two-bone IK (thigh, calf) on the animated target pose for the swing
  leg; keep the stance leg's animated pose with its foot pinned in place by
  IK. The upper body keeps playing the state clip plus the flinch.
- Swing-leg muscle = 1.0 for the swing duration. The stance leg muscle uses
  the normal value. GTA staggerFall reduces leg stiffness by
  `perStepReduction1 = 0.7` per step after `stepsTillStartEnd = 2` steps to
  make long staggers end in a fall; this design does the same with factor
  0.85 per step after step 2 (design, gentler).
- Lean the spine of the target toward `v_com` by `0.4 * |v_com|` rad,
  clamped to 0.3 rad (GTA `spineBendMult = 0.4`, `leanInDirMaxB = 0.3`).
- Turn the head toward the velocity direction (GTA `headLookAtVelProb = 1`).

SIMBICON's equivalent foot-placement law is
`theta_d = theta_d0 + c_d * d + c_v * v` on the swing hip, with
`c_d = 0.5`, `c_v = 0.2` and stable ranges `c_d` in [-0.71, 1.4],
`c_v` in [0.03, 0.59] (sagittal)
(https://www.cs.ubc.ca/~van/papers/2007-siggraph-simbicon.pdf). It is the
fallback if IK-target stepping fails, because it needs no IK. The midpoint
of the hips is a "simple and effective proxy" for the CoM (same source).

### 3.6 Hand of god on the pelvis

An external force and torque on the pelvis (and optionally the chest). This
is a cheat, like GTA's `resistAcc` "cheat force" and `stayUpright`.

```
F  = m * (kp_x * (x_target - com_xz) - kd_x * v_com_xz)     horizontal only
F  = F * min(1, F_max / |F|)
tau = I_eff * (kp_r * axis_angle(q_target_pelvis * q_pelvis^-1) - kd_r * w_pelvis)
tau = tau * min(1, tau_max / |tau|)
```

- `x_target`: in Idle and Returning, the animated target's pelvis. In
  Stagger, the centre of the support polygon (or the stance foot) blended
  toward the animated pelvis.
- `kp_x = omega_p^2`, `kd_x = 2 * omega_p` (critically damped) with
  `omega_p = 2 pi * f_p`.
- `I_eff = 10` kg·m² (design; roughly the whole body about the pelvis
  is larger, this keeps the cheat modest).
- Use `pinDistanceFalloff`-style weakening:
  `F_max_eff = F_max / (1 + falloff * dist)`, `falloff = 2` 1/m (design,
  after PuppetMaster `pinDistanceFalloff = 5`).

```toml
# f_p Hz, F_max N, kp_r 1/s^2, kd_r 1/s, tau_max N·m
[pin]
idle        = { f_p = 1.5, F_max = 340.0, kp_r = 150.0, kd_r = 25.0, tau_max = 400.0 }  # 0.5 m g; also Returning, Turning
hit_react   = { f_p = 1.0, F_max = 200.0, kp_r = 100.0, kd_r = 20.0, tau_max = 250.0 }
stagger     = { f_p = 0.8, F_max = 140.0, kp_r = 60.0,  kd_r = 15.0, tau_max = 150.0 }  # 0.2 m g
falling     = { f_p = 0.0, F_max = 0.0,   kp_r = 0.0,   kd_r = 0.0,  tau_max = 0.0 }
getting_up  = { f_p = 2.0, F_max = 820.0, kp_r = 200.0, kd_r = 30.0, tau_max = 500.0 }  # 1.2 m g, all three axes
```

Check: in Stagger, 140 N on 70 kg is 2.0 m/s². It stops 0.5 m/s in 0.25 s
over 0.06 m. It helps but cannot stop a 1.5 m/s push; stepping must do that.

Pin weight after a hit: multiply `F_max` and `tau_max` by `pin_w`, which
drops to `1 - 0.8 * s` on a hit and recovers at 1.0 per s. This is
PuppetMaster's pin loss and `regainPinSpeed = 1`.

### 3.7 Giving up (fall)

Go to Falling when any holds:

```toml
[give_up]
tilt_max_deg        = 50.0  # angle between chest up and world up; GTA impulseSpineAng uses dot 0.7 (45 deg) and 0.2
com_height_min_frac = 0.55  # com height above lowest planted foot < 0.55 * standing (GTA giveUpHeight 0.5 m)
knockout_distance   = 0.60  # pelvis or chest farther than this from its animated target, m (PuppetMaster 1.0)
max_steps           = 4     # steps in one stagger while margin stays > 0
short_steps_max     = 2     # "short" steps (3.5) in one stagger
airborne_max        = 0.30  # s with no planted foot
cp_far              = 1.00  # margin > this, m: no step can save it
```

The design classifies at the moment of the hit too, so a heavy hit does not
wait for the tilt test:

```
dv      = |J| / m
cp_off  = dv / omega
if J >= j_knockdown (100 kg·m/s)            -> Falling immediately
elif cp_off * exp(omega * 0.3) > 0.9 m       -> Falling immediately
else                                         -> HitReact (stepping may follow)
```

At `omega = 3.13`: 30 kg·m/s gives `cp_off = 0.14` m (inside or at the toe,
maybe one step). 60 gives 0.27 m (stagger, 1 to 3 steps). 100 gives 0.46 m;
predicted 1.18 m after 0.3 s, which exceeds 0.9 m, so it falls.

### 3.8 Stagger end and tidy step

- Recovered when `margin < -0.03` m, `|v_com| < 0.3` m/s and pelvis angular
  speed < 0.3 rad/s for 0.3 s (GTA `stableLinSpeedThresh = 0.3`,
  `stableRotSpeedThresh = 0.3`; kickback `balance_recovery_hold_time = 0.5`).
- Then, if the feet are more than hip width + 0.2 m apart, or closer than
  0.08 m, or one foot is more than 0.25 m ahead of the other, take one tidy
  step of that foot to its idle position relative to the pelvis (GTA
  `legsApartRestep = 0.2`).

## 4. Falling and lying

### 4.1 Entering Falling

- Ramp all muscles from their current value to `0.3 * base` over
  `0.25 / fall_mult` s (PuppetMaster `unpinnedMuscleWeightMlp = 0.3`; GTA
  `fallType = RampDownStiffness`, `fallMult = 1`).
- Pins off.
- Keep the body velocity (no reset). Clamp body speed to 10 m/s.
- Animated target: a "falling brace" pose (arms forward-down for a forward
  fall, arms back for a backward fall), plus catch-fall IK.

### 4.2 Catch fall (arms reach toward predicted ground contact)

Per tick while Falling:

```
t_hit   = time for the chest to reach the ground:
          solve chest.y - ground + v_y t - 0.5 g t^2 = 0.15 for t > 0; clamp to [0.05, 0.6]
p_hit   = chest_xz + v_chest_xz * t_hit, at ground height
fall_dir = normalize(v_com_xz) if |v_com_xz| > 0.2 else horizontal part of chest forward or back
hand_l  = p_hit + 0.20 * left  + fall_dir * offset + up * 0.02
hand_r  = p_hit + 0.20 * right + fall_dir * offset + up * 0.02
offset  = 0.4 (forward fall, GTA forwardMaxArmOffset 0.4) or -0.3 (backward, backwardsMinArmOffset -0.3)
```

- Two-bone IK on the target arms toward `hand_l` / `hand_r`, clamped to arm
  reach (0.95 * arm length) from each shoulder.
- Arm muscles 1.0 and torso 0.6, legs 0.4 during the reach (GTA catchFall
  `armsStiffness 15`, `torsoStiffness 9`, `legsStiffness 6`, as ratios
  1.0 / 0.6 / 0.4).
- Kickback reaches for `arm_fall_reach_duration = 0.55` s with strength 0.8.
- Elbows: let the motors bend them on contact; set elbow torque limits low
  (0.5 x) during the reach so the arms absorb impact.

### 4.3 Going limp

When any of pelvis, spine or chest has a ground contact, or 0.8 s after
entering Falling:

- Ramp muscles to `limp = 0.12` over 0.3 s (design; kickback floor values
  0.05 to 0.15).
- Keep a small joint friction (core default `joint_friction = 0.05`) so the
  body does not twitch.
- Optional writhe after 0.5 s: sine targets on spine and limbs with period
  1.0 s and amplitude 0.15 rad (GTA bodyWrithe period 1 s). Off by default.

### 4.4 Rest detection

```toml
[rest]
min_down_time      = 1.0   # s after the first torso ground contact (PuppetMaster getUpDelay 5 is long for a demo)
pelvis_speed_max   = 0.25  # m/s (PuppetMaster maxGetUpVelocity 0.3; kickback settle_linear 0.5)
angular_speed_max  = 0.8   # rad/s, every body (kickback settle_angular 0.3 on the pelvis)
hold_time          = 0.5   # s the two speed tests must hold (kickback settle_duration 0.6)
force_after        = 4.0   # s; then get up if pelvis speed < 0.6 m/s
```

The existing `force_sleep_after = 6.0` and sleep thresholds in
`RagdollPhysicsSettings` must not freeze a ragdoll that is about to get up.
Wake it on entering GettingUp.

## 5. Get-up

### 5.1 Face up or face down

Use the chest (spine_03 or the rig's upper chest) body:

```
f = chest forward axis in world (the direction the character's chest faces)
if dot(f, up) >= 0  -> supine (face up)  -> clip "getup_back"
else                -> prone (face down) -> clip "getup_front"
```

Use the pelvis forward axis as a tiebreak when `|dot(f, up)| < 0.3`
(lying on the side): if the side is ambiguous, first roll toward the nearer
of back or front by setting the target to the lying pose of that clip's
frame 0 for 0.4 s with muscles 0.5 (design). PuppetMaster exposes
`IsProne()` and fires `onGetUpProne` / `onGetUpSupine` (1.1).

### 5.2 Align the root to the ragdoll (no guessing)

Do not assume which way a clip lies. Measure it once per clip, at load:

```
In the clip's frame 0, in the clip root's frame:
    P0 = pelvis position (x, z)
    H0 = flatten(head - pelvis), normalized
    yaw_offset = atan2 of H0 relative to the root forward
```

At runtime:

```
H        = flatten(head_ragdoll - pelvis_ragdoll), normalized
root_yaw = yaw(H) - yaw_offset
root_xz  = pelvis_ragdoll_xz - rotate(root_yaw, P0)
root_y   = ground height under the pelvis (ray down from pelvis + 0.5 m, 2 m long)
```

This replaces PuppetMaster's hand-tuned `getUpOffsetProne` and
`getUpOffsetSupine`. Use the head-pelvis vector, not the pelvis forward
axis, because the head direction is well defined when lying flat.

### 5.3 Blend from the ragdoll pose to the clip

1. Snapshot the ragdoll's local bone rotations (and the pelvis world pose)
   at the transition tick.
2. Set the animated root to `root_xz, root_y, root_yaw` immediately. The
   visible character is the ragdoll, so moving the target root makes no
   visible pop.
3. Play the clip from `t = 0` and set the target pose each tick to
   `slerp(snapshot_local, clip_local(t), w)` per bone, with
   `w = smoothstep(t / blend_time)`, `blend_time = 0.30` s (PuppetMaster
   0.2, kickback `pose_blend_duration = 0.75`).
4. Ramp muscles from `limp` to `base` over 0.5 s with the core first
   (`order_delay` of 2.4). Ramp the getting-up pin from 0 to full over 0.3 s,
   starting 0.1 s in.
5. GettingUp has protection: collision resistance x2 (`r_max` x0.5),
   pin regain x2 and `knockout_distance` x10 (PuppetMaster GetUp
   multipliers). A hit with `J >= 0.5 * j_knockdown` still knocks it down
   again (kickback `recovery_interrupt_threshold = 0.5`).
6. Complete when the clip ends and the pelvis is within 0.10 m and 15° of
   its target, or at `safety_timeout = 3.5` s (kickback 3.5). Minimum time
   in GettingUp 1.0 s (PuppetMaster `minGetUpDuration = 1`).

### 5.4 Pitfalls

- Popping: never write the clip pose directly to the visible skeleton. The
  visible skeleton is always the ragdoll; only the target changes.
- Root snap: set the target root once at entry; never re-align it during
  the clip. Re-aligning makes the pins drag the body sideways.
- Feet sliding at the clip start: the clip's pelvis is not at the
  ragdoll's pelvis if `P0` is measured wrongly or the ground ray hits the
  ragdoll. Exclude ragdoll colliders from the ground ray.
- Motor fight: if the motors are stiff while the snapshot blend runs,
  the body pops toward the clip. Keep muscles below 0.5 during the first
  0.2 s.
- Limbs under the body: the clip's frame 0 assumes free arms. If an arm is
  pinned under the torso, the blend drags it. Mitigation: arm muscles ramp
  0.25 s after the core (order_delay).
- Sleeping bodies: wake the ragdoll first.
- Two get-up clips are the minimum. A side clip is optional.

## 6. Returning to the marker

### 6.1 Locomotion of the animated target

- Clips: idle, walk (in place, no root motion, or root motion stripped),
  turn_left_90, turn_right_90, turn_180 (optional).
- Root moved by code, a kinematic controller on the target root: position
  integrates `v_root`, yaw integrates `w_root`. Ground height by ray.
  Kickback strips root motion and moves the root in the physics step
  (`strip_root_motion = true`).
- Walk clip playback rate = `|v_root| / clip_speed`, clamped to
  [0.5, 1.3]. Below 0.15 m/s blend to idle.

### 6.2 Steering (arrive)

```
d        = marker_xz - root_xz
dist     = |d|
desired_yaw = yaw(d)
yaw_err  = wrap(desired_yaw - root_yaw)
w_root   = clamp(3.0 * yaw_err, -pi, pi)             rad/s
speed    = v_walk * clamp(dist / slow_radius, 0, 1) * clamp(cos(yaw_err), 0, 1)
```

```toml
[return]
v_walk        = 1.4    # m/s; set to the walk clip's speed
slow_radius   = 1.0    # m
arrive_dist   = 0.15   # m; switch to Turning
start_dist    = 0.30   # m; after getting up, skip walking if closer
turn_in_place = 1.2    # rad; if |yaw_err| > this at the start, turn first
```

### 6.3 Turning to the stored heading

- `e = wrap(marker_yaw - root_yaw)`.
- If `|e| > 1.2` rad: play turn_left_90 / turn_right_90 (or turn_180 above
  2.6 rad), apply the clip's yaw delta to the root, repeat.
- Otherwise rotate the root at `w = clamp(2.5 * e, -1.6, 1.6)` rad/s while
  the idle clip plays; a shuffle step comes from the stepping logic (3.4)
  automatically if the feet end up crossed (tidy step).
- Done when `|e| < 4°` (0.07 rad) and the root yaw rate is below 0.1 rad/s.
- Before Idle, nudge the root to the exact marker position at 0.2 m/s if
  the error is below 0.15 m.

### 6.4 How the ragdoll follows locomotion

- Muscles at base, pins at `pin.idle`. The pelvis pin target is the
  animated pelvis, so the body walks with the target.
- Feed the target velocity forward to the motors (Jolt
  `DriveToPoseUsingMotors(prev, pose, dt)`; kickback `spring_feed_forward = 1`).
- Leash: if the ragdoll pelvis is more than 0.4 m behind the target pelvis,
  stop moving the target root until it catches up (ashleve
  MasterController "makes sure static animator can't move too far away
  from ragdoll", https://github.com/ashleve/ActiveRagdoll).
- Hits and collisions while walking go through the same hit path. A
  walking character has `|v_com| ≈ 1.4` m/s already, so its capture point
  is ahead; compute the step trigger margin against the walk's own planned
  footsteps (design): during Returning, add `v_root / omega` to the
  support centre before testing `margin`, so normal walking does not
  trigger recovery steps.
- PuppetMaster lets collision resistance grow with target velocity (its
  `collisionResistance` curve). Here: `j_ref_walking = j_ref * 1.3`.

## 7. State machine

Common data: `hit` queue, `marker = { pos, yaw }` stored at spawn,
`t_state` time in state, `steps` count, tuning tables above.

Global rules:

- Every non-Idle state has a timeout that leads to a safe state.
- Hit handling runs in every state before the state's own tick (2.2 to 2.5).
- `knockdown(J)` means `J >= j_knockdown` or the 3.7 hit-time test fails.

```
Idle
  entry: muscles -> base (rate 1.5/s); pin = pin.idle; target clip = idle; steps = 0
  tick:  stepping logic (3.4) stays active (tidy steps only)
  -> HitReact   on hit and not knockdown(J)
  -> Falling    on hit and knockdown(J)
  -> Stagger    if margin > step_margin (pushed by a body or prop)
  -> Returning  if |root - marker| > 0.30 m or |yaw err| > 10 deg for 0.5 s (drifted)

HitReact
  entry: strength drop (2.3); pin_w drop; start flinch envelope (2.5)
  tick:  recovery at rate_idle after delay
  -> Stagger    if margin > step_margin
  -> Falling    on knockdown(J) or any give-up test (3.7)
  -> Idle       when flinch envelope ended (0.45 s) and min muscle > 0.9 and |v_com| < 0.15
                and root within 0.30 m and 10 deg of marker
  -> Returning  same as Idle but root farther from marker
  timeout 2.0 s -> Returning

Stagger
  entry: pin = pin.stagger; steps = 0; recovery rate = rate_stagger;
         target root follows ragdoll pelvis every tick (root_xz = pelvis_xz - offset)
  tick:  step trigger, step target, swing (3.4, 3.5); leg muscle x0.85 per step after step 2;
         spine lean toward v_com; head look along v_com
  -> Falling    any give-up test (3.7) or knockdown hit
  -> HitReact   recovered (3.8) then tidy step done; HitReact exits immediately to Idle/Returning
  timeout 4.0 s -> Falling

Falling
  entry: muscles ramp to 0.3*base over 0.25 s; pins off; brace pose; catch-fall IK (4.2)
  tick:  update p_hit and hand targets
  -> Down       pelvis, spine or chest touches ground, or t_state > 0.8 s
  timeout 3.0 s -> Down

Down
  entry: muscles ramp to limp (0.12) over 0.3 s; record t_ground
  tick:  rest detection (4.4)
  -> GettingUp  rest detected, or t_state > force_after (4.0 s) and pelvis speed < 0.6
  (hits only add impulses here; no state change)

GettingUp
  entry: wake ragdoll; choose clip (5.1); align root (5.2); snapshot; blend (5.3);
         pin = pin.getting_up ramped; GetUp protection multipliers
  -> Falling    hit with J >= 0.5 * j_knockdown, or pelvis > knockout_distance*10 from target
  -> Returning  clip ended and pelvis within 0.10 m / 15 deg, t_state >= 1.0 s,
                and |root - marker| > start_dist
  -> Turning    same, but already within start_dist of the marker
  timeout 3.5 s -> same exits (force completion)

Returning
  entry: pin = pin.idle; walk/idle blend; steering (6.2)
  -> HitReact / Falling   on hit, as in Idle
  -> Stagger    margin test with walking compensation (6.4)
  -> Turning    dist < arrive_dist
  timeout 20 s -> Turning (log a warning)

Turning
  entry: idle clip; compute e (6.3)
  -> HitReact / Falling   on hit
  -> Idle       |e| < 4 deg, yaw rate < 0.1 rad/s, |root - marker| < 0.15 m
  timeout 6 s -> Idle
```

Implementation shape for Bevy (design): one `ReactionState` component enum
with the eight variants, each carrying its own data (for example
`Stagger { steps: u8, swing: Option<Swing> }`,
`GettingUp { clip: GetUpClip, t: f32 }`). One system per state in
`FixedUpdate`, ordered after "measure" and before "write motors". Tuning in
one `ReactionTuning` struct with `Default` that holds every number in this
document; load it from TOML through `serde`. Where the TOML lives per
character is the owner's decision.

## 8. Acceptance tests

Headless, in `cargo test`, on a flat static ground plane, the human rig,
total mass scaled to 70 kg, 60 Hz. Impulses horizontal, applied at the
chest body's centre unless stated. "Recovered" means state is Idle,
HitReact or Turning with `|v_com| < 0.05` m/s.

Stability:

1. Idle for 10 s with no input: pelvis drift < 0.02 m, max joint angle error
   against the idle pose < 5°, state stays Idle, no step.
2. 20 random hits (random body, direction, J in [5, 150]) over 60 s: no NaN,
   no body faster than 15 m/s, no joint anchor separation > 0.05 m, every
   state exits within its timeout, final state Idle within 25 s of the last
   hit.
3. The same scenario at 120 Hz: pelvis peak displacement for the 30 and 60
   cases differs from 60 Hz by less than 20 %.

Hit reaction:

4. Pistol 12 to the chest from the front: no step, pelvis peak displacement
   < 0.08 m, chest peak rotation ≥ 4°, Idle within 1.0 s.
5. Pistol 12 to the head: neck plus head peak rotation ≥ 8°, no step.
6. Punch 30 to the chest from the front: no fall, at most 2 steps, pelvis
   peak displacement between 0.05 and 0.35 m, recovered within 2.0 s.
7. 30 from behind: at most 2 steps, no fall (backward margin is smaller, so
   a step is expected).
8. Kick 60 to the chest from the side: Stagger with 1 to 4 steps, no foot
   crosses the other (no swing target inside `inward_min`), no fall,
   recovered within 3.0 s.
9. 40 to the stance thigh: at least one step or knee bend of that leg ≥ 10°,
   no fall.

Knockdown:

10. 120 to the chest from the front: state Falling within 0.5 s, Down within
    2.0 s, pelvis height < 0.35 m in Down, supine (face up) in at least 8 of
    10 runs with ±10° direction jitter.
11. 120 from behind: prone (face down) in at least 8 of 10 runs.
12. During Falling with a forward fall, at least one hand body touches the
    ground before the chest does in at least 7 of 10 runs (catch fall).

Get-up:

13. GettingUp starts between 1.0 s and 4.5 s after the first torso ground
    contact.
14. No pop: across the Down to GettingUp transition, no rendered bone moves
    more than 0.05 m between consecutive frames beyond its physical motion
    (compare with the previous frame's velocity times dt).
15. GettingUp completes within 3.5 s. At the end: pelvis height ≥ 0.9 of
    standing height, chest tilt < 10°, both feet planted.
16. The correct clip runs: supine starts getup_back, prone starts
    getup_front (from tests 10 and 11).

Return:

17. After a knockdown that moves the pelvis 1 to 3 m, the character ends in
    Idle with root within 0.20 m and 10° of the marker, within 10 s of the
    get-up ending.
18. A 30 punch during Returning causes HitReact or Stagger and the
    character still arrives (criterion 17) afterwards.

Report for each test the measured value next to the threshold, so tuning
can move numbers without guessing.

## 9. Code worth reading

- ashleve/ActiveRagdoll (MIT, C#, Unity):
  https://github.com/ashleve/ActiveRagdoll. `AnimationFollowing.cs` applies
  PD forces (`PForce = 8`, `DForce = 0.01`, `maxForce = 10`) and joint
  torques (`maxJointTorque = 2000`) toward an animated master.
  `SlaveController.cs` loses strength in contact (`looseStrengthLerp = 1`
  per s) and regains it (`gainStrengthLerp = 0.05` per s), floors at
  `minContactForce = 0.1`, and "dies" for `deadTime = 4` s.
  `MasterController.cs` keeps the master near the ragdoll (the leash).
- bhubbard/active-ragdoll-rs (MIT OR Apache-2.0, Rust, Rapier3D, created
  2026-09-26): https://github.com/bhubbard/active-ragdoll-rs. A Rust port of
  ashleve's design: 11-bone 75 kg humanoid, linear CoM PD plus angular joint
  PD, strength loss and recovery, knockout and revival, a
  `RapierRagdollAdapter`. Closest to this repo's stack; very new and small,
  so read it for structure, not as a tested reference.
- sergioabreu-g/active-ragdolls (Apache-2.0, C#, Unity):
  https://github.com/sergioabreu-g/active-ragdolls. Animated body plus
  physical body with ConfigurableJoint target rotations; a module and
  behaviour split that keeps features independent.
- blugart-dev/kickback (MIT, GDScript, Godot 4.7):
  https://github.com/blugart-dev/kickback. The most relevant reference for
  hit reactions: impact profiles (impulse, transfer, strength reduction,
  spread in hops, recovery rate), per-bone strength and pin maps, staggered
  recovery order, stagger and balance ratios, settle detection, get-up with
  face-up detection and pose blending, foot IK, catch-fall arm reach.
  `spring_resolver.gd` drives bodies by velocity springs with feed-forward
  and "chain consistency". Its own audit says the stumble is "a scripted
  root displacement with foot-IK step targets, not balance-driven
  stepping", so do not copy its stepping.
- CBerry22/Active-Ragdoll---Physics-Animations-in-Godot-4.0 (MIT):
  https://github.com/CBerry22/Active-Ragdoll---Physics-Animations-in-Godot-4.0.
  R3X-G1L6AME5H/Godot-Active-Ragdolls (MIT):
  https://github.com/R3X-G1L6AME5H/Godot-Active-Ragdolls.
  PiCode9560/Godot-4-Active-ragdoll (MIT, Human Fall Flat style):
  https://github.com/PiCode9560/Godot-4-Active-ragdoll. Smaller examples of
  joint-driven following in Godot 4 (not read in detail).
- Vaei/ProcHitReact (MIT, C++, Unreal):
  https://github.com/Vaei/ProcHitReact. Procedural physics hit reactions on
  skeletal meshes using Unreal's physics blend weights (README only seen).
- PuppetMaster docs: http://root-motion.com/puppetmasterdox/html/page10.html
  (BehaviourPuppet) and page5.html (PuppetMaster component). The asset is
  commercial; the docs are the useful part.
- GTA V euphoria parameters with defaults:
  https://github.com/citizenfx/fivem/blob/8a2d502a/code/client/clrcore/External/EuphoriaHelpers.cs.
  A catalogue of balance, shot, stagger, catch-fall and writhe parameters
  as shipped (section 1.5).
- SIMBICON paper:
  https://www.cs.ubc.ca/~van/papers/2007-siggraph-simbicon.pdf. Foot
  placement feedback `c_d = 0.5`, `c_v = 0.2`. JSimbicon (the Java applet
  version) is from memory; no repository was checked.
- Capture point: Pratt et al. 2006,
  https://www.cs.cmu.edu/~cga/legs/Pratt_Goswami_Humanoids2006.pdf, and the
  explainer https://scaron.info/robotics/capture-point.html.
- Jolt Ragdoll API: https://jrouwe.github.io/JoltPhysics/class_ragdoll.html.

## 10. Motor and mass defaults

Segment mass fractions for a 70 kg body (Winter / de Leva, from memory):
head and neck 8.1 %, trunk 49.7 % (split pelvis 14 %, spine 15 %,
chest 20.7 %), upper arm 2.8 %, forearm 1.6 %, hand 0.6 %, thigh 10 %,
shank 4.65 %, foot 1.45 %. Masses: head 5.7, pelvis 9.8, spine 10.5,
chest 14.5, upper arm 2.0, forearm 1.1, hand 0.4, thigh 7.0, shank 3.3,
foot 1.0 kg.

Joint motors (acceleration-based, per axis):

```
omega_m  = 2 pi f,  stiffness = omega_m^2 * muscle,  damping = 2 zeta omega_m * sqrt(muscle)
f = 4 Hz (core default), zeta = 1.0
=> at muscle 1: stiffness 632 s^-2, damping 50.3 s^-1
```

Scaling damping by `sqrt(muscle)` keeps the joint critically damped as
strength drops (design). Torque limits per joint scale with `muscle` too.
Raise `f` to 6 Hz for the legs and pelvis in Idle if the knees buckle under
the body's weight (design); lower to 3 Hz for arms if they look robotic.

Gravity compensation (design): add a feed-forward joint torque for the
stance legs equal to the gravity torque of the mass above each joint
(GTA `opposeGravityLegs = 1`, `opposeGravityAnkles = 1`). Without it the PD
spring sags by `g_torque / stiffness`. If this is too much work, the pelvis
pin's vertical component (`pin.idle` lift, 0.3 m g) is the cheap substitute.

## 11. Order of work for the implementing agent

1. Per-body muscle and pin (core, phase 7), CoM and contacts queries.
   Test 1 (idle stability) must pass before anything else.
2. Impulse application with the per-body velocity clamp; strength drop and
   recovery; flinch. Tests 4, 5.
3. Measurements (3.1, 3.2) with a debug gizmo: CoM, cp, support polygon.
4. Stepping (3.4, 3.5) and hand of god (3.6). Tests 6 to 9.
5. Give-up, Falling, catch fall, Down, rest detection. Tests 10 to 13.
6. Get-up alignment and blend. Tests 14 to 16.
7. Returning and Turning. Tests 17, 18.
8. Robustness tests 2, 3.
