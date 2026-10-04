# Ragdoll physics API cheat sheet: Bevy 0.19.1

Sources read (all paths are crate roots):

- `BR` = `scratchpad/bevy_rapier3d-0.36.0/` (crates.io tarball)
- `R` = `~/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/rapier3d-0.35.0-glamx0.2/`
- `AV` = `~/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/avian3d-0.7.0/`
- `P27` = `.../parry3d-0.27.0/` (Avian's parry; Avian does NOT use parry 0.30)
- Upstream rapier `CHANGELOG.md` (master, fetched 2026-10-03). The crate tarballs ship no changelog.
  The `-glamx0.2` rapier build is a pre-release variant; its source matches the v0.35.0 joint-limit
  changes quoted below (`recentered_angle` exists at `R/src/dynamics/solver/joint_constraint/joint_constraint_helper.rs:470`).

Rules for the implementing agent:

- Every name below exists at the cited line. Anything marked "does not exist" must not be called.
- Items marked "inferred from code" are read from solver source, not from docs. Cover them with a test.
- Both crates use `f32` and Bevy `Vec3`/`Quat` in their public 3D API with the features below.

---

## A. bevy_rapier3d 0.36.0 (rapier3d =0.35.0-glamx0.2)

### A1. Cargo features

`BR/Cargo.toml` `[features]`:

- `default = ["dim3", "async-collider", "debug-render-3d", "picking-backend", "to-bevy-mesh"]`.
- `[lib] required-features = ["dim3"]`. With `default-features = false` you MUST list `"dim3"`, or the lib does not build.
- Headless (no `bevy_render`): `bevy_rapier3d = { version = "0.36", default-features = false, features = ["dim3"] }`.
  `async-collider`, `picking-backend`, `to-bevy-mesh` and `debug-render-3d` all pull `bevy/bevy_render` or `bevy_pbr`.
  `async-collider` also makes `RapierPhysicsPlugin::finish` add `AssetPlugin`, `MeshPlugin` and `WorldSerializationPlugin` if missing (`BR/src/plugin/plugin.rs:343-359`).
- `headless = []` exists but no source file references it (grep of `BR/src` finds no `feature = "headless"`). It is a no-op.
- Determinism: `enhanced-determinism = ["rapier3d/enhanced-determinism"]`.
- Parallel: `parallel = ["rapier3d/parallel"]` (rayon). SIMD: `simd8 = ["rapier3d/simd8"]` (8 lanes, f32, needs AVX2 per changelog).
- Conflict: `simd8` + `enhanced-determinism` is a `compile_error!` (`R/src/lib.rs:18-21`). Upstream changelog v0.35.0: `enhanced-determinism` can now be combined with `parallel`, bitwise identical for any thread count.
- f64: this crate depends only on `rapier3d` (f32) (`BR/Cargo.toml:208-209`, `Real = rapier::math::Real`, `BR/src/lib.rs:45`). No f64 feature exists in bevy_rapier3d.
- wasm: do not enable `parallel` (rayon). No other wasm-specific features exist.
- Bevy dep: `bevy = { version = "0.19.0", default-features = false, features = ["std"] }` (`BR/Cargo.toml:193-196`).

### A2. Plugin setup

- Type: `pub struct RapierPhysicsPlugin<PhysicsHooks = ()>` (`BR/src/plugin/plugin.rs:27`). `NoUserData = ()` (`:21`).
- Default schedule is `PostUpdate` (`:201-215`).
- `.in_fixed_schedule()` puts systems in `FixedUpdate` (`:112`). `.in_schedule(FixedPostUpdate)` for any other label (`:117`).
- `.with_length_unit(f32)` (`:56`), `.with_default_system_setup(bool)` (`:77`), `.with_custom_initialization(RapierContextInitialization)` (`:65`).
- Even in a fixed schedule, `systems::sync_removals` is also added to `PostUpdate` (`:292-297`).
- `TimestepMode` resource (`BR/src/plugin/configuration.rs:15`), default `Variable { max_dt: 1/60, time_scale: 1.0, substeps: 1 }` (`:53-61`).
  Variants: `Fixed { dt: f32, substeps: usize }`, `Variable { max_dt, time_scale, substeps }`, `Interpolated { dt, time_scale, substeps }`.
  The plugin calls `init_resource::<TimestepMode>()` (`plugin.rs:328`) and warns if not `Fixed` in `FixedUpdate` (`:331-339`). Insert the resource before `add_plugins`.
- `Fixed` mode ignores `Time` and steps exactly `dt` per schedule run (`BR/src/plugin/context/mod.rs:809-832`). Set `dt` equal to `Time<Fixed>`'s timestep.
- Substeps: each substep is a full `pipeline.step` with `dt / substeps` (`context/mod.rs:812-831`).
- Gravity and pause live on the `RapierConfiguration` **component** on the context entity, not a resource (`configuration.rs:63-80`):
  `gravity: Vect`, `physics_pipeline_active: bool`. Default gravity `Vec3::Y * -9.81 * length_unit` (`:89-96`).
  Pause: set `physics_pipeline_active = false`; `step_simulation` then skips stepping (`BR/src/plugin/systems/mod.rs:66-81`) and writeback skips (`systems/rigid_body.rs:420-422`).
  Access: `Query<&mut RapierConfiguration, With<DefaultRapierContext>>`.
- Solver tuning: `RapierContextSimulation.integration_parameters: IntegrationParameters` (component, `context/mod.rs:641-656`).
  Fields (`R/src/dynamics/integration_parameters.rs:181-300`, defaults `:380-407`): `num_solver_iterations` (4), `num_internal_pgs_iterations` (1),
  `num_internal_stabilization_iterations` (1), `warmstart_joints` (false), `normalized_max_corrective_velocity` (3.0), `normalized_prediction_distance` (0.02),
  `max_ccd_substeps` (1; 0 disables all CCD), `contact_recycling` (true), `length_unit` (1.0).
  Note: `step_simulation` overwrites `integration_parameters.dt` every step (`context/mod.rs:810`).
- System sets: `pub enum PhysicsSet { SyncBackend, StepSimulation, Writeback }` (`plugin.rs:219-234`), chained and `.before(TransformSystems::Propagate)` (`:301-310`).
  Also `RapierBevyComponentApply` (`:193`) and `RapierTransformPropagateSet` (`:199`), both inside `SyncBackend`.
  Write motor targets / impulses in a system `.before(PhysicsSet::SyncBackend)` in the same schedule.
- Interpolation: `TransformInterpolation { start: Option<Pose>, end: Option<Pose> }` component (`BR/src/dynamics/rigid_body.rs:589`). Used only with `TimestepMode::Interpolated` (`systems/rigid_body.rs:438-450`). With `in_fixed_schedule()` + `Fixed` there is no built-in interpolation.

### A3. Rigid bodies (`BR/src/dynamics/rigid_body.rs`)

- `enum RigidBody { Dynamic (default), Fixed, KinematicPositionBased, KinematicVelocityBased }` (`:34-54`). Mutable component.
- Mass: on the **collider**, `enum ColliderMassProperties { Density(f32), Mass(f32), MassProperties(MassProperties) }`, default `Density(1.0)` (`BR/src/geometry/collider.rs:136-149`).
  On the **body**, `enum AdditionalMassProperties { Mass(f32), MassProperties(MassProperties) }` is added on top (`:159-166`).
  `Mass(m)` scales the collider-derived inertia (doc `:160-162`).
- Principal inertia: `struct MassProperties { local_center_of_mass: Vec3, mass: f32, principal_inertia_local_frame: Quat, principal_inertia: Vec3 }` (`:226-240`).
  Use `ColliderMassProperties::MassProperties(MassProperties { .. })` to replace the collider's mass properties exactly.
- Read total mass: `ReadMassProperties` component, `.get() -> &MassProperties` (`:184-195`). Writeback system `writeback_mass_properties`.
- Damping: `Damping { linear_damping: f32, angular_damping: f32 }` default 0/0 (`:568-583`).
- Initial velocity: `Velocity { linear: Vec3, angular: Vec3 }` (`:88-97`). Read at init (`systems/rigid_body.rs:595-597`), written back each step.
- CCD: `Ccd { enabled: bool }`, `Ccd::enabled()` (`:473-491`). `SoftCcd { prediction: f32 }` (`:505-508`).
- Sleeping: `Sleeping { normalized_linear_threshold: f32, angular_threshold: f32, sleeping: bool }` (`:532-542`), `Sleeping::disabled()` sets thresholds to -1 (`:546-552`).
  Read sleep state: the `sleeping` bool is written back each step if the component is present (`systems/rigid_body.rs:550-557`).
- Per-body solver iterations: `AdditionalSolverIterations(pub usize)` (`:623`). Changelog v0.35.0: it now adds whole substeps for the body's constraint-connected component, "converging much better on high mass ratios".
- `GravityScale(pub f32)` (`:462`), `LockedAxes` (`:281`), `Dominance` (`:513`), `RigidBodyDisabled` (`:610`).
- Friction/restitution are collider components: `Friction { coefficient, combine_rule }`, default 0.5 Average, `Friction::new(f32)` (`BR/src/geometry/collider.rs:154-181`);
  `Restitution { coefficient, combine_rule }`, default 0.0, `Restitution::new(f32)` (`:196-233`).

### A4. Colliders (`BR/src/geometry/collider_impl.rs`)

- `Collider::capsule_y(half_height: f32, radius: f32)` = segment from `-Y*half_height` to `+Y*half_height` plus radius (`:113-116`). `half_height` excludes the hemispheres; total length = `2*half_height + 2*radius`.
  Also `capsule_x` (`:107`), `capsule_z` (`:120`), `capsule(start: Vec3, end: Vec3, radius)` (`:102`).
- `Collider::cuboid(hx, hy, hz)` takes **half**-extents (`:127`). `Collider::ball(radius)` (`:44`). `Collider::compound(Vec<(Vec3, Quat, Collider)>)` (`:35`).
- Groups: `CollisionGroups::new(memberships: Group, filters: Group)` (`BR/src/geometry/collider.rs:392-399`); `Group::GROUP_1..GROUP_32`, `ALL`, `NONE` (`:285-357`).
  Rule: `(a.memberships & b.filters) != 0 && (b.memberships & a.filters) != 0` (doc `:380`). `SolverGroups` same shape (`:419`).
  Changelog v0.35.0: the broad phase now filters by collision groups.
- Hooks: `trait BevyPhysicsHooks: SystemParam + Send + Sync` (`BR/src/pipeline/physics_hooks.rs:84`).
  `fn filter_contact_pair(&self, ctx: PairFilterContextView) -> Option<SolverFlags>` (`:109`); `None` drops the pair; `Some(SolverFlags::COMPUTE_IMPULSES)` keeps it.
  Overriding replaces rapier's default non-dynamic-pair filtering (doc `:96-97`). `ctx.collider1()/collider2() -> Entity`, `rigid_body1()/rigid_body2() -> Option<Entity>` (`:15-42`).
  Activate per collider with component `ActiveHooks::FILTER_CONTACT_PAIRS` (`BR/src/geometry/collider.rs:451-461`).
  Register: `RapierPhysicsPlugin::<MyHooks>::default()` (example `BR/examples/contact_filter3.rs:15-44`).
- Jointed pair contacts: `GenericJoint::set_contacts_enabled(false)` (`BR/src/dynamics/generic_joint.rs:150`); default `contacts_enabled: true` (`R/src/dynamics/joint/generic_joint.rs:320`).
  Note `IntegrationParameters::contact_recycling` doc: "per-step joint-based contact filtering is skipped until the pair moves" (`R/src/dynamics/integration_parameters.rs`, `contact_recycling` field docs). Toggling contacts on an existing joint may lag until the pair moves.
- Changelog v0.35.0: colliders on the same rigid body never form pairs.

### A5. Joints

- Component on the CHILD entity: `ImpulseJoint { parent: Entity, data: TypedJoint }`, `ImpulseJoint::new(parent, impl Into<TypedJoint>)` (`BR/src/dynamics/joint.rs:81-96`).
  `init_joints` inserts it as `impulse_joints.insert(body_of(parent), body_of(this entity or its ancestors), ...)` (`BR/src/plugin/systems/joint.rs:48-71`). So `body1 = parent`, `body2 = child`. Frame 1 is in the parent body's local space.
  Multiple joints on one body: put each `ImpulseJoint` on a child entity of that body (doc `joint.rs:76-79`).
- `MultibodyJoint { parent, data }`, `MultibodyJoint::new(parent, TypedJoint)` (`joint.rs:108-120`); no closed loops (doc `:104-106`).
- `enum TypedJoint { FixedJoint, GenericJoint(GenericJoint), PrismaticJoint, RevoluteJoint, RopeJoint, SphericalJoint, SpringJoint }` (`joint.rs:13-29`); `impl AsMut<GenericJoint>` / `AsRef` (`:31-59`).
  There is no `From<GenericJoint> for TypedJoint`; wrap with `TypedJoint::GenericJoint(j)`.
- `GenericJoint { pub raw: rapier GenericJoint }` (`BR/src/dynamics/generic_joint.rs:15-18`). Methods (all `&mut self -> &mut Self`):
  `set_local_basis1(Quat)` (`:62`), `set_local_basis2` (`:84`), `set_local_anchor1(Vec3)` (`:127`), `set_local_anchor2` (`:139`),
  `set_contacts_enabled(bool)` (`:150`), `set_limits(JointAxis, [f32; 2])` (`:162`), `set_coupled_axes(JointAxesMask)` (`:168`),
  `set_motor_model(JointAxis, MotorModel)` (`:180`), `set_motor_velocity(axis, target_vel, factor)` (`:186`),
  `set_motor_position(axis, target_pos, stiffness, damping)` (`:197`), `set_motor_max_force(axis, f32)` (`:210`),
  `set_motor(axis, target_pos, target_vel, stiffness, damping)` (`:222`). Readers: `limits(axis)`, `motor(axis)`, `motor_model(axis)`.
- `GenericJointBuilder::new(JointAxesMask)` with `local_basis1/2`, `local_anchor1/2`, `limits`, `coupled_axes`, `motor_model`, `motor_velocity`, `motor_position`, `set_motor`, `motor_max_force`, `build()` (`:307-428`).
  The builder has no `contacts_enabled`; call `set_contacts_enabled` on the built joint.
- `SphericalJoint { pub data: GenericJoint }` (`BR/src/dynamics/spherical_joint.rs:11-14`) has anchors, limits and motors but **no basis setter**; use `joint.data.set_local_basis1(..)` or `GenericJointBuilder::new(JointAxesMask::LOCKED_SPHERICAL_AXES)`.
- Axis enums (rapier, re-exported at `BR/src/dynamics/joint.rs:4`): `JointAxis { LinX, LinY, LinZ, AngX, AngY, AngZ }` (`R/src/dynamics/joint/generic_joint.rs:103-119`).
  `JointAxesMask::LOCKED_SPHERICAL_AXES = LIN_X|LIN_Y|LIN_Z` (`:45`), `ANG_X/ANG_Y/ANG_Z` (`:33-37`).
- Local frame: `local_frame1/2: Pose` (anchor = translation, basis = rotation) (`R/.../generic_joint.rs:270-272`). Joint axes `AngX/Y/Z` are the columns of the basis.
- Per-axis angular limits: `set_limits(JointAxis::AngX, [min, max])` sets `limit_axes` bit (`R/.../generic_joint.rs:518-524`).
  Uncoupled angle measure: wrapped joint angle about that basis column, measured from the range centre (`R/src/dynamics/solver/joint_constraint/joint_constraint_helper.rs:470-498`, `:503-563`). Ranges may sit anywhere on the circle; a range wider than a full turn leaves the axis free (changelog v0.35.0; doc `generic_joint.rs:136-139`).
- Coupled (cone) limit: `set_coupled_axes(JointAxesMask::ANG_Y | JointAxesMask::ANG_Z)` + `set_limits(JointAxis::AngY, [lo, hi])`.
  Inferred from code: exactly 2 coupled angular axes are asserted (`joint_constraint_helper.rs:737-739`); the limited quantity is the unsigned angle (>= 0) between the uncoupled axis (here X) of frame 1 and frame 2 (`:740-748`); the limits of the first coupled angular axis are used (`R/src/dynamics/solver/joint_constraint/joint_velocity_constraint.rs:318-331`). Use `[-cone, cone]` or `[0.0, cone]`.
- **Coupled angular motors are not implemented**: `if (motor_axes & coupled_axes) & ANG_AXES != 0 { // TODO: coupled angular motor constraint. }` (`joint_velocity_constraint.rs:224-226`). Motors on coupled angular axes are silently ignored. Use uncoupled per-axis motors.
- Uncoupled angular motor position error is `2*asin(q_err.imag[axis])` (`joint_constraint_helper.rs:583-591`), i.e. per-axis quaternion components, not Euler angles. It is exact for single-axis rotation and approximate for combined rotations.
- Softness/compliance: `raw.softness: SpringCoefficients<f32> { natural_frequency, damping_ratio }`, default `joint_defaults()` = 1.0e6 Hz, ζ 1.0 (`R/src/dynamics/integration_parameters.rs:37-46`, `:78-83`).
  Set with `joint.raw.softness = SpringCoefficients::new(hz, zeta)` (`:50`); path `bevy_rapier3d::rapier::dynamics::SpringCoefficients` (`R/src/dynamics/mod.rs:8`, `BR/src/lib.rs:21`).
  `GenericJoint::set_softness` is `#[must_use]` (`R/.../generic_joint.rs:500`); assign the field to avoid an `unused_must_use` warning under `-D warnings`.
- Limit rows cap correction bias at `max_corrective_velocity` (`joint_constraint_helper.rs:532-540`, changelog v0.35.0).

### A6. Motors

- `JointMotor { target_vel, target_pos, stiffness, damping, max_force (default f32::MAX), impulse, model }` (`R/.../generic_joint.rs:203-231`).
- `MotorModel { AccelerationBased (default), ForceBased }` (`R/src/dynamics/joint/motor_model.rs:25-31`). AccelerationBased scales by mass.
- `set_motor_position` = `set_motor(axis, pos, 0.0, stiffness, damping)`; `set_motor_velocity(axis, vel, factor)` = `set_motor(axis, current target_pos, vel, 0.0, factor)` (`R/.../generic_joint.rs:544-568`). `set_motor` sets the `motor_axes` bit (`:596`).
- Mutate per frame: `Query<&mut ImpulseJoint>`, then `joint.data.as_mut().set_motor_position(JointAxis::AngX, t, k, d);`.
  `apply_joint_user_changes` copies the whole joint on `Changed<ImpulseJoint>` (`BR/src/plugin/systems/joint.rs:118-144`).
  It calls `impulse_joints.get_mut(handle, false)`: **wake_up is false** (`:141`). Changing a motor target does not wake a sleeping ragdoll. Add `Sleeping::disabled()` to motorized bodies, or write `Sleeping { sleeping: false, .. }` (handled at `systems/rigid_body.rs:141-160`).
- Changelog v0.35.0/v0.35.2: 3D motors are on the scalar (non-SIMD) path (`R/.../generic_joint.rs:334-348`), so motorized joints cost more than limit-only joints.

### A7. Forces

- `ExternalImpulse { impulse: Vec3, torque_impulse: Vec3 }` (`BR/src/dynamics/rigid_body.rs:387-396`).
  `ExternalImpulse::at_point(impulse, world_point, world_center_of_mass)` sets `torque_impulse = (point - com) x impulse` (`:406-414`).
  Applied once on change, then reset to zero by the sync system (`systems/rigid_body.rs:326-337`); also applied once at spawn (`:699-729`). Use `+=` to accumulate several hits in one frame.
- `ExternalForce { force: Vec3, torque: Vec3 }` (`:315-324`), `ExternalForce::at_point(..)` (`:334`).
  **Persists**: on change the system calls `reset_forces`, `reset_torques`, then adds the new values (`systems/rigid_body.rs:312-324`); rapier only clears user forces in `reset_forces`/`reset_torques` (`R/src/dynamics/rigid_body.rs:1147-1177`). Set it back to zero yourself. A PD torque driver can write `ExternalForce.torque` every step.
- World COM: `ReadMassProperties.get().local_center_of_mass` transformed by `GlobalTransform`.

### A8. Reading results

- Writeback runs in `PhysicsSet::Writeback` in the plugin schedule: writes `Transform` (parent-relative if `ChildOf`), `Velocity`, `Sleeping` (`systems/rigid_body.rs:403-560`). Only `Transform` is written; `GlobalTransform` updates at `TransformSystems::Propagate` in `PostUpdate`.
- Joint angle: only `RapierContext::impulse_revolute_joint_angle(&self, entity) -> Option<f32>` (`BR/src/plugin/context/systemparams/rapier_context_systemparam.rs:615`), revolute only (`context/mod.rs:613-625`). No spherical/generic relative-rotation getter exists. Compute `(q_parent * basis1)^-1 * (q_child * basis2)` from `Transform`s.

### A9. Raycast

- System param `ReadRapierContext` (`rapier_context_systemparam.rs:18`), `.single() -> Result<RapierContext<'_>>` (`:39`).
- `RapierContext::cast_ray(&self, origin: Vec3, dir: Vec3, max_toi: f32, solid: bool, filter: QueryFilter) -> Option<(Entity, f32)>` (`:311-323`). Hit point = `origin + dir * toi`.
- `cast_ray_and_get_normal(..) -> Option<(Entity, RayIntersection)>` (`:325`); `RayIntersection { time_of_impact, point, normal, .. }` (`BR/src/geometry/mod.rs:46-58`).
- `QueryFilter` (`BR/src/pipeline/query_filter.rs:9-21`): `QueryFilter::new()`, `only_dynamic()` (`:64`), `exclude_fixed()` (`:49`), `.groups(CollisionGroups)` (`:93`), `.exclude_rigid_body(Entity)` (`:105`), `.predicate(&impl Fn(Entity) -> bool)` (`:111`).
- Returned entity is the **collider** entity. If colliders sit on child entities, map to the body with `ChildOf` or `RapierContextColliders::collider_parent` (`context/mod.rs:77`).
- Queries use the broad phase as of the last step (`context/mod.rs:184-260`).

### A10. Pitfalls (rapier/bevy_rapier)

1. Motor target changes do not wake sleeping bodies (A6).
2. Coupled angular motors are a TODO no-op (A5). Coupled angular limits require exactly two coupled angular axes (panic via `assert_eq!` otherwise).
3. Default joint softness is effectively rigid (1e6 Hz). Lower it for stability on extreme mass ratios.
4. `warmstart_joints` defaults to `false`; its doc says enabling it "noticeably improves convergence of stiff joint assemblies" (`integration_parameters.rs`, `warmstart_joints` field).
5. Mass ratios: use `AdditionalSolverIterations` on the torso rather than raising global iterations (changelog v0.35.0).
6. Changelog v0.35.0: contact defaults changed (`normalized_prediction_distance` 0.02, `normalized_max_corrective_velocity` 3.0); bodies capped at 400 m/s and ~45 deg rotation per step unless `set_allow_fast_rotation`.
7. Changelog v0.35.0: NaN bodies are quarantined (rolled back and disabled) instead of corrupting the world.
8. `Fixed { dt }` must match `Time<Fixed>`; otherwise sim speed differs from game time.

### A11. Skeleton (bevy_rapier3d)

```rust
use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use bevy_rapier3d::rapier::dynamics::SpringCoefficients;

#[derive(Component)]
struct UpperArm;
#[derive(Component)]
struct ForeArmJoint; // marker on the entity holding the ImpulseJoint

pub fn plugin(app: &mut App) {
    app.insert_resource(Time::<Fixed>::from_hz(60.0))
        .insert_resource(TimestepMode::Fixed { dt: 1.0 / 60.0, substeps: 1 })
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default().in_fixed_schedule())
        .add_systems(Startup, spawn)
        .add_systems(FixedUpdate, (drive_motor, shoot).before(PhysicsSet::SyncBackend));
}

fn spawn(mut commands: Commands) {
    let half = 0.15; // capsule half-height (segment only)
    let radius = 0.05;
    let parent = commands
        .spawn((
            UpperArm,
            Transform::from_xyz(0.0, 2.0, 0.0),
            RigidBody::Dynamic,
            Collider::capsule_y(half, radius),
            ColliderMassProperties::Mass(2.0),
            Damping { linear_damping: 0.05, angular_damping: 0.5 },
            Velocity::zero(),
            Ccd::enabled(),
            Sleeping::disabled(),
            ReadMassProperties::default(),
            ExternalImpulse::default(),
            CollisionGroups::new(Group::GROUP_2, Group::ALL),
        ))
        .id();

    // 3-DOF joint at the parent's bottom / child's top.
    let mut joint = GenericJointBuilder::new(JointAxesMask::LOCKED_SPHERICAL_AXES)
        .local_anchor1(Vec3::new(0.0, -(half + radius), 0.0))
        .local_anchor2(Vec3::new(0.0, half + radius, 0.0))
        .local_basis1(Quat::IDENTITY)
        .local_basis2(Quat::IDENTITY)
        .limits(JointAxis::AngX, [-0.2, 2.4])
        .limits(JointAxis::AngY, [-0.5, 0.5])
        .limits(JointAxis::AngZ, [-0.3, 0.3])
        .motor_position(JointAxis::AngX, 0.0, 200.0, 20.0)
        .motor_position(JointAxis::AngY, 0.0, 200.0, 20.0)
        .motor_position(JointAxis::AngZ, 0.0, 200.0, 20.0)
        .motor_max_force(JointAxis::AngX, 50.0)
        .motor_max_force(JointAxis::AngY, 50.0)
        .motor_max_force(JointAxis::AngZ, 50.0)
        .build();
    joint.set_contacts_enabled(false);
    joint.raw.softness = SpringCoefficients::new(1.0e4, 1.0);

    commands.spawn((
        ForeArmJoint,
        Transform::from_xyz(0.0, 2.0 - 2.0 * (half + radius), 0.0),
        RigidBody::Dynamic,
        Collider::capsule_y(half, radius),
        ColliderMassProperties::Mass(1.5),
        Velocity::zero(),
        Sleeping::disabled(),
        ReadMassProperties::default(),
        ExternalImpulse::default(),
        CollisionGroups::new(Group::GROUP_2, Group::ALL),
        ImpulseJoint::new(parent, TypedJoint::GenericJoint(joint)),
    ));
}

fn drive_motor(time: Res<Time>, mut joints: Query<&mut ImpulseJoint, With<ForeArmJoint>>) {
    let target = 1.0 + 0.5 * time.elapsed_secs().sin();
    for mut j in &mut joints {
        j.data.as_mut().set_motor_position(JointAxis::AngX, target, 200.0, 20.0);
    }
}

/// `shot` would come from input; a fixed ray keeps the skeleton self-contained.
fn shoot(
    ctx: ReadRapierContext,
    mut bodies: Query<(&GlobalTransform, &ReadMassProperties, &mut ExternalImpulse)>,
) {
    let Ok(ctx) = ctx.single() else { return };
    let (origin, dir) = (Vec3::new(-3.0, 1.8, 0.0), Vec3::X);
    let Some((entity, toi)) = ctx.cast_ray(origin, dir, 100.0, true, QueryFilter::only_dynamic()) else {
        return;
    };
    let point = origin + dir * toi;
    // Collider is on the body entity here; otherwise map collider -> body first.
    if let Ok((gt, mprops, mut imp)) = bodies.get_mut(entity) {
        let com = gt.transform_point(mprops.get().local_center_of_mass);
        *imp += ExternalImpulse::at_point(dir * 5.0, point, com);
    }
}
```

---

## B. avian3d 0.7.0

### B1. Cargo features (`AV/Cargo.toml` `[features]`, table `AV/src/lib.rs:45-65`)

- `default = ["3d", "f32", "parry-f32", "debug-plugin", "xpbd_joints", "parallel", "collider-from-mesh", "bevy_scene", "bevy_picking"]`.
- Headless: `avian3d = { version = "0.7", default-features = false, features = ["3d", "f32", "parry-f32", "xpbd_joints"] }` (add `"parallel"` if wanted).
  `debug-plugin` pulls `bevy/bevy_render` + `bevy_gizmos`; `bevy_picking` pulls `bevy/bevy_picking`.
- **`xpbd_joints` is required for joints to be solved**: `XpbdSolverPlugin` is only added under it (`AV/src/dynamics/solver/mod.rs:79-80`); `JointPlugin` "does not include the actual joint solver" (`AV/src/dynamics/joints/mod.rs:242`).
- f32/f64: exactly one of `f32`/`f64` (`AV/src/lib.rs:464-468`); `default-collider` needs the matching `parry-f32`/`parry-f64` (`:476-492`).
- Determinism: `enhanced-determinism` (libm for bevy_math, bevy_heavy, parry). `simd` = `parry3d/simd-stable`; parry 0.27 makes SIMD + `enhanced-determinism` a `compile_error!` (`P27/src/lib.rs:32-35`).
- `parallel = ["bevy/multi_threaded", "parry3d?/parallel"]`. wasm: drop `parallel` if threads are unavailable (no wasm-specific feature exists).
- Bevy dep: `bevy 0.19.0`, `default-features = false`, `["std", "bevy_log"]` (`AV/Cargo.toml:323-329`).

### B2. Plugin setup

- `PhysicsPlugins` plugin group (`AV/src/lib.rs:681`); `PhysicsPlugins::default()` = `new(FixedPostUpdate)` (`:750-754`); `PhysicsPlugins::new(schedule)` (`:690`); `.with_length_unit(f32)` (`:745`); `.with_collision_hooks::<H>()` (`:701`).
- In a fixed schedule the step uses `Time<Fixed>`'s delta (doc `AV/src/schedule/time.rs:8-9`; `run_physics_schedule` reads `Time` delta, `AV/src/schedule/mod.rs:235-279`). Set the rate with `Time::<Fixed>::from_hz(..)`.
- `SubstepCount(pub u32)` resource, default 6 (`AV/src/dynamics/solver/schedule.rs:185-191`). Each step runs N substeps of `delta / N` (`:194-197`).
- `Gravity(pub Vec3)` resource, default `-9.81 Y` (`AV/src/dynamics/integrator/mod.rs:156-162`); `GravityScale(pub f32)` per body (`AV/src/dynamics/rigid_body/mod.rs:575`).
- Pause: `ResMut<Time<Physics>>` then `.pause()` / `.unpause()` / `.is_paused()` from trait `PhysicsTime` (`AV/src/schedule/time.rs:212-219`, impl `:252-262`). Speed: `with_relative_speed(f32)` (`:167`).
- Outer sets in the plugin schedule: `enum PhysicsSystems { First, Prepare, StepSimulation, Writeback, Last }` (`AV/src/schedule/mod.rs:162-176`), chained before `TransformSystems::Propagate` (`:73-84`).
  Inner `PhysicsSchedule` sets: `enum PhysicsStepSystems { First, BroadPhase, NarrowPhase, Solver, Sleeping, Finalize, Last }` (`:192-214`).
  Systems that write motor targets or forces: run in `FixedUpdate`, or in `FixedPostUpdate` `.before(PhysicsSystems::StepSimulation)`.
- Interpolation: `PhysicsInterpolationPlugin` is in the group with no easing by default (`AV/src/interpolation.rs:183-189`). Per body add `TransformInterpolation` (re-exported `:9-14`), or replace the plugin with `PhysicsInterpolationPlugin::interpolate_all()` (`:195`).

### B3. Rigid bodies

- `enum RigidBody { Dynamic (default), Static, Kinematic }` (`AV/src/dynamics/rigid_body/mod.rs:284-305`). It is `#[component(immutable)]` (`:283`): change type by re-inserting.
  It requires `Position`, `Rotation`, `LinearVelocity`, `AngularVelocity`, `ComputedMass`, `ComputedAngularInertia`, `ComputedCenterOfMass`, ... (`:267-282`).
- Mass (body or collider entity): `Mass(pub f32)` (`.../mass_properties/components/mod.rs:160`); `AngularInertia { principal: Vec3, local_frame: Quat }`, `AngularInertia::new(Vec3)` (`:536`, `:564`), `new_with_local_frame(Vec3, Quat)` (`:611`); `CenterOfMass(pub Vec3)` (`:915`).
  `ColliderDensity(pub f32)` (`.../components/collider.rs:32`). `NoAutoMass` / `NoAutoAngularInertia` / `NoAutoCenterOfMass` stop children contributing (`components/mod.rs:981-1011`).
  If `Mass` is set without `AngularInertia`, the collider inertia is rescaled to the new mass (`.../mass_properties/system_param.rs:175-181`).
  Read totals: `ComputedMass::value()` (`.../components/computed.rs:48`, `:126`), `ComputedAngularInertia` (`:428`), `ComputedCenterOfMass(pub Vec3)` (`:776`).
- Damping: `LinearDamping(pub f32)` (`rigid_body/mod.rs:605`), `AngularDamping(pub f32)` (`:629`). Joint-relative damping: `JointDamping { linear, angular }` on the joint entity (`AV/src/dynamics/joints/mod.rs:611-616`).
- Velocity: `LinearVelocity(pub Vec3)` (`rigid_body/mod.rs:412`), `AngularVelocity(pub Vec3)` (`:543`). Insert at spawn for initial velocity.
- CCD: `SweptCcd { mode: SweepMode, include_dynamic, linear_threshold, angular_threshold }`, `SweptCcd::LINEAR` / `NON_LINEAR` / `default()` (`AV/src/dynamics/ccd/mod.rs:389-437`). `SpeculativeMargin(pub f32)` (`:308`), unbounded by default (doc `:280-285`).
- Sleeping: marker `Sleeping` present while asleep (`AV/src/dynamics/rigid_body/sleeping.rs:61`); read with `Has<Sleeping>`. `SleepingDisabled` (`:70`), `SleepThreshold { linear: 0.15, angular: 0.15 }` (`:84-119`), resource `TimeToSleep` (0.5 s) (`:143`).
  Wakers listed at `:13-27`. Changing a joint's motor fields does **not** wake: joint `Changed<T>` only rebuilds when `body1/body2` change (`AV/src/dynamics/solver/joint_graph/plugin.rs:298-320`). The motor example uses `SleepingDisabled` (`AV/examples/joint_motors_3d.rs:80`).
- Per-body solver iterations: do not exist (no `solver_iterations` symbol in `AV/src`). Only global `SubstepCount`.
- `Friction { dynamic_coefficient, static_coefficient, combine_rule }`, `Friction::new(f32)` (`.../physics_material.rs:137`, `:172`); `Restitution::new(f32)` (`:305`, `:356`).

### B4. Colliders (`AV/src/collision/collider/parry/mod.rs`)

- `Collider::capsule(radius: f32, length: f32)`: Y-axis, `length` is the full segment length excluding hemispheres (from `-Y*length/2` to `+Y*length/2`) (`:788-797`). Argument order is radius first.
  `Collider::capsule_endpoints(radius, a: Vec3, b: Vec3)` (`:800`).
- `Collider::cuboid(x_length, y_length, z_length)` takes **full** lengths (halved internally) (`:747-749`). `Collider::sphere(radius)` (`:725`). `Collider::compound(Vec<(pos, rot, Collider)>)` (`:698`).
- Body lookup from a collider entity: `ColliderOf { body: Entity }` (`AV/src/collision/collider/collider_hierarchy/mod.rs:53-56`).
- Layers: `CollisionLayers::new(memberships: impl Into<LayerMask>, filters: impl Into<LayerMask>)` (`AV/src/collision/collider/layers.rs:403`); `LayerMask(pub u32)` with `From<u32>` (`:86-94`), `ALL/NONE/DEFAULT` (`:117-121`); rule `interacts_with` (`:424-427`), same as rapier.
- Hooks: `trait CollisionHooks: ReadOnlySystemParam + Send + Sync` (`AV/src/collision/hooks.rs:147`):
  `fn filter_pairs(&self, collider1: Entity, collider2: Entity, commands: &mut Commands) -> bool` (`:164`, default `true`),
  `fn modify_contacts(&self, contacts: &mut ContactPair, commands: &mut Commands) -> bool` (`:187`).
  Only called when at least one collider has `ActiveCollisionHooks::FILTER_PAIRS` / `MODIFY_CONTACTS` (`:227-235`; immutable component) and at least one side is non-static and awake (`:158-160`).
  Register with `PhysicsPlugins::default().with_collision_hooks::<MyHooks>()`. Hook system param must be read-only.
- Jointed-pair contacts: put `JointCollisionDisabled` on the **joint entity** (`AV/src/dynamics/joints/mod.rs:536-556`).

### B5. Joints

- Joints are separate entities: `commands.spawn(SphericalJoint::new(body1, body2))` (module doc `AV/src/dynamics/joints/mod.rs:23-37`).
- `SphericalJoint { body1, body2, frame1: JointFrame, frame2: JointFrame, twist_axis: Vec3 (default Y), swing_limit: Option<AngleLimit>, twist_limit: Option<AngleLimit>, point_compliance, swing_compliance, twist_compliance }` (`AV/src/dynamics/joints/spherical.rs:33-61`).
  Builders: `new` (`:75`), `with_twist_axis` (`:97`), `with_local_anchor1/2(Vec3)` (`:130`, `:139`), `with_local_basis1/2(impl Into<Quat>)` (`:159`, `:168`), `with_local_frame1/2(impl Into<Isometry>)` (`:104`, `:111`), `with_anchor(global Vec3)` (`:120`),
  `with_swing_limits(min, max)` (`:275`), `with_twist_limits(min, max)` (`:282`), `with_swing_compliance` (`:309`), `with_twist_compliance` (`:316`), `with_point_compliance` (`:302`).
  Compliance unit: inverse stiffness (N*m/rad for swing/twist). 0 = rigid.
- `AngleLimit { min, max }`, `AngleLimit::new` (`joints/mod.rs:369-405`).
- **`SphericalJoint` has no motor in 0.7.0**: no motor field (`spherical.rs:33-61`); motors exist only on `RevoluteJoint` (`AngularMotor`) and `PrismaticJoint` (`LinearMotor`) (`joints/mod.rs:227`). The crate doc still says motors are unsupported (`AV/src/lib.rs:182`); that line is stale.
- Swing/twist solve (inferred from code, verify with a test): `prepare` builds `swing_axis = twist_axis.any_orthonormal_vector()` and limits the angle between `basis1*swing_axis` and `basis2*swing_axis` (`AV/src/dynamics/solver/xpbd/joints/spherical.rs:76-81`, `:112-151`); twist is measured about `swing_axis1 + swing_axis2` using the twist axes (`:153-210`).
  This uses the perpendicular axis for the swing cone, which differs from the doc ("cone defined by a twist_axis", `spherical.rs:24-26`). Before relying on swing/twist limits, write a test: rotate body2 about `twist_axis` only and check whether the swing limit engages. The swing angle is unsigned (>= 0), so use `with_swing_limits(0.0, cone)`.
- `RevoluteJoint { .., hinge_axis (default Z), angle_limit: Option<AngleLimit>, point_compliance, align_compliance, limit_compliance, motor: AngularMotor }` (`AV/src/dynamics/joints/revolute.rs:48-74`), `with_hinge_axis` (`:112`), `with_angle_limits(min, max)` (`:304`), `with_limit_compliance` (`:342`), `with_motor(AngularMotor)` (`:349`).
- `JointFrame { anchor: JointAnchor, basis: JointBasis }` (`joints/mod.rs:777-785`); `JointAnchor::{Local(Vec3), FromGlobal(Vec3)}` (`:925`), `JointBasis::{Local(Quat), FromGlobal(Quat)}` (`:1002`). Global variants are converted to local on the next step; until then `local_anchor1()` etc. return `None` (`spherical.rs:174-245`).
- `JointDisabled` (`joints/mod.rs:527`), `JointForces::new()` to read force/torque (`:664-690`).

### B6. Motors

- `AngularMotor { enabled, target_velocity, target_position, max_torque, motor_model }` (`AV/src/dynamics/joints/motor.rs:124-135`); `AngularMotor::new(MotorModel)` (`:146`), `with_target_position` (`:183`), `with_target_velocity` (`:176`), `with_max_torque` (`:190`).
- `enum MotorModel { SpringDamper { frequency, damping_ratio }, ForceBased { stiffness, damping }, AccelerationBased { stiffness, damping } }` (`motor.rs:13-88`), default `SpringDamper { 5.0, 1.0 }` (`:97-103`). Docs call SpringDamper "unconditionally stable" (`:14-21`).
- Mutate: `Query<&mut RevoluteJoint>`, then `joint.motor.target_position = x;` (`AV/examples/joint_motors_3d.rs:221-245`).
- For a 3-DOF motorized joint: `SphericalJoint` + PD torque (B7), or a chain of revolute joints through massless helper bodies (not recommended; tiny masses destabilize XPBD).

### B7. Forces

- `Forces` QueryData (`AV/src/dynamics/rigid_body/forces/query_data.rs:107-121`); use `Query<Forces>` (no `&`). Methods come from trait `WriteRigidBodyForces` (`:292`, in prelude):
  `apply_force` (`:300`), `apply_force_at_point(force, world_point)` (`:330`), `apply_torque(Vec3)` (`:358`), `apply_linear_impulse` (`:388`),
  `apply_linear_impulse_at_point(impulse, world_point)` (`:420`), `apply_angular_impulse(Vec3)` (`:450`), `apply_local_torque` (`:374`).
  Reads (`ReadRigidBodyForces`, `:191`): `linear_velocity()`, `angular_velocity()`, `rotation()`, `velocity_at_point(p)` (`:194-269`).
- Impulses change velocity immediately (`:380-399`). Forces/torques are applied over the next physics step and then cleared (doc `:39`).
  A force applied in `Update` reaches only the next fixed step; frames with 0 or 2+ fixed steps break it. Apply per-step drives in `FixedUpdate`.
- Applying wakes the body unless you call `.non_waking()` (`:153`).
- Persistent: `ConstantTorque(pub Vec3)`, `ConstantTorque::new(x, y, z)` (`AV/src/dynamics/rigid_body/forces/mod.rs:317-325`); `ConstantForce` (`:260`); `ConstantLocalTorque` (`:424`). Changing them wakes the body (`AV/src/dynamics/solver/islands/sleeping.rs:545-556`).

### B8. Reading results

- Physics state is `Position(pub Vec3)` (`AV/src/physics_transform/transform.rs:48`) and `Rotation(pub Quat)` (`:745`). `Transform` is written in `PhysicsSystems::Writeback` (`AV/src/physics_transform/mod.rs:116-123`), configurable via `PhysicsTransformConfig` (`:130`).
- Velocities: `LinearVelocity`, `AngularVelocity` components.
- Joint relative rotation: no getter exists. Compute `(rot1 * basis1)^-1 * (rot2 * basis2)` from `Rotation` and `joint.local_basis1()/local_basis2()`.

### B9. Raycast

- `SpatialQuery` system param (`AV/src/spatial_query/system_param.rs:60`).
  `cast_ray(&self, origin: Vec3, direction: Dir3, max_distance: f32, solid: bool, filter: &SpatialQueryFilter) -> Option<RayHitData>` (`:111-120`); `cast_ray_predicate(.., predicate: &dyn Fn(Entity) -> bool)` (`:176`).
- `RayHitData { entity, distance, normal }` (`AV/src/spatial_query/ray_caster.rs:395-404`). Hit point = `origin + *direction * distance`. `entity` is the collider; map to the body with `ColliderOf`.
- `SpatialQueryFilter { mask: LayerMask, excluded_entities }`, `default()`, `from_mask`, `with_excluded_entities` (`AV/src/spatial_query/query_filter.rs:35-89`).

### B10. Pitfalls (Avian)

1. Joints are XPBD (position-based); contacts are impulse-based (`AV/src/dynamics/solver/plugin.rs:36`). Joint stiffness depends on `SubstepCount` (default 6); raise it for long chains.
2. No spherical motor; no per-body iteration control; `RigidBody` is immutable.
3. Motor/joint edits do not wake bodies; use `SleepingDisabled` on ragdoll parts while driven.
4. Swing-limit axis behavior is inferred to differ from docs (B5); test it.
5. `JointCollisionDisabled` must be on the joint entity, not the bodies.
6. `apply_linear_impulse_at_point` uses the cached global COM; after teleporting a body the torque can be huge (doc `query_data.rs:409-418`).
7. `Collider::cuboid` uses full lengths, unlike rapier's half-extents; `Collider::capsule` is `(radius, length)`, unlike rapier's `capsule_y(half_height, radius)`.

### B11. Skeleton (avian3d, PD torque fallback)

```rust
use avian3d::prelude::*;
use bevy::prelude::*;

#[derive(Component)]
struct Drive { parent: Entity, child: Entity, basis: Quat, target: Quat, kp: f32, kd: f32 }

pub fn plugin(app: &mut App) {
    app.insert_resource(Time::<Fixed>::from_hz(60.0))
        .insert_resource(SubstepCount(8))
        .add_plugins(PhysicsPlugins::default()) // FixedPostUpdate
        .add_systems(Startup, spawn)
        .add_systems(FixedUpdate, (pd_drive, shoot));
}

fn spawn(mut commands: Commands) {
    let (radius, length) = (0.05, 0.3); // length = segment, excludes hemispheres
    let half = 0.5 * length + radius;
    let body = |y: f32, mass: f32| {
        (
            RigidBody::Dynamic,
            Collider::capsule(radius, length),
            Mass(mass),
            Transform::from_xyz(0.0, y, 0.0),
            LinearDamping(0.05),
            AngularDamping(0.5),
            SweptCcd::default(),
            SleepingDisabled,
            CollisionLayers::new(0b10, LayerMask::ALL),
        )
    };
    let parent = commands.spawn(body(2.0, 2.0)).id();
    let child = commands.spawn(body(2.0 - 2.0 * half, 1.5)).id();

    commands.spawn((
        SphericalJoint::new(parent, child)
            .with_local_anchor1(Vec3::new(0.0, -half, 0.0))
            .with_local_anchor2(Vec3::new(0.0, half, 0.0))
            .with_local_basis1(Quat::IDENTITY)
            .with_local_basis2(Quat::IDENTITY)
            .with_twist_axis(Vec3::Y)
            .with_swing_limits(0.0, 1.2)  // verify axis semantics (B5)
            .with_twist_limits(-0.5, 0.5),
        JointCollisionDisabled,
        JointDamping { linear: 0.0, angular: 0.2 },
    ));
    commands.spawn(Drive {
        parent, child, basis: Quat::IDENTITY,
        target: Quat::from_rotation_x(0.8), kp: 60.0, kd: 6.0,
    });
}

/// Equal and opposite PD torque in world space; cleared after each physics step.
fn pd_drive(drives: Query<&Drive>, mut bodies: Query<Forces>) {
    for d in &drives {
        let Ok([mut p, mut c]) = bodies.get_many_mut([d.parent, d.child]) else { continue };
        let frame1 = p.rotation().0 * d.basis;
        let rel = frame1.inverse() * c.rotation().0 * d.basis;
        let err_local = d.target * rel.inverse();
        let (axis, mut angle) = err_local.to_axis_angle();
        if angle > core::f32::consts::PI { angle -= core::f32::consts::TAU; }
        let err_world = frame1 * (axis * angle);
        let rel_w = c.angular_velocity() - p.angular_velocity();
        let tau = err_world * d.kp - rel_w * d.kd;
        c.apply_torque(tau);
        p.apply_torque(-tau);
    }
}

fn shoot(spatial: SpatialQuery, colliders: Query<&ColliderOf>, mut bodies: Query<Forces>) {
    let (origin, dir) = (Vec3::new(-3.0, 1.8, 0.0), Dir3::X);
    let Some(hit) = spatial.cast_ray(origin, dir, 100.0, true, &SpatialQueryFilter::default()) else {
        return;
    };
    let point = origin + *dir * hit.distance;
    let body = colliders.get(hit.entity).map_or(hit.entity, |c| c.body);
    if let Ok(mut f) = bodies.get_mut(body) {
        f.apply_linear_impulse_at_point(*dir * 5.0, point);
    }
}
```

Notes on the skeleton:

- `kp`/`kd` here are absolute torque gains (N*m/rad, N*m*s/rad); scale them by the child's inertia for mass-independent tuning.
- `get_many_mut` returns `Result<[Item; 2], QueryEntityError>` in Bevy 0.19; it fails if both entities are the same.
- `to_axis_angle` returns an angle in `[0, 2*pi)`; the wrap keeps the shortest error.
