//! Backend-neutral physical behavior checks for ragdoll physics plugins.
//!
//! Each public check builds a headless app, adds the human profile, and
//! reads only the shared ragdoll body components. The selected backend supplies
//! its plugin and one function that adds a fixed collider from validated shape
//! input, keeping Rapier types out of this conformance crate.

use bevy::asset::{AssetPlugin, Assets};
use bevy::math::{Isometry3d, Quat, Vec3};
use bevy::prelude::{
    AnimationPlugin, App, ChildOf, Entity, MinimalPlugins, Name, Transform, TransformPlugin, World,
};
use bevy::time::{Fixed, Time, TimeUpdateStrategy};
use bevy_ragdoll::profile::BodyIndex;
use bevy_ragdoll::runtime::body::{BodyMass, BodyPhysicsPose, BodyShape, BodyVelocity};
use bevy_ragdoll::runtime::components::{
    Ragdoll, RagdollBodyOf, RagdollDrive, RagdollMode, RagdollTargetPose,
};
use bevy_ragdoll::runtime::messages::RagdollImpulse;
use bevy_ragdoll::runtime::settings::RagdollPhysicsSettings;
use bevy_ragdoll::{RagdollPlugin, RagdollProfile, ShapeSpec};
use std::time::Duration;

mod fall;
mod hits;
mod look;

pub use self::fall::{
    a_dropped_ragdoll_lands_and_settles, a_hard_throw_keeps_the_joints_together,
    the_same_input_gives_the_same_output,
};
pub use self::hits::{
    headshot_turns_the_head, limp_weights_make_a_powered_ragdoll_collapse,
    pistol_to_the_chest_does_not_move_the_pelvis_far,
};
pub use self::look::{
    a_body_shot_onto_stairs_stays_on_them, a_bullet_moves_a_downed_body_a_little,
    a_chest_hit_buckles_the_knees_and_stops, a_headshot_drops_the_body_like_the_references,
    a_running_death_stops_within_a_body_length,
};

use crate::contract::AddBackend;

/// Adds one fixed collider to a conformance app from a local-frame shape and
/// rigid world pose.
///
/// Backends use this seam for floors and stairs. The shape coordinates and
/// dimensions are in metres, and the pose is a rigid transform in world space.
pub type AddFixedShape = fn(&mut App, ShapeSpec, Isometry3d) -> Entity;

/// Measures total kinetic and gravitational potential energy for one ragdoll.
///
/// The callback computes joules from backend-owned mass and inertia data so
/// conformance assertions can include angular as well as linear motion.
pub type MeasureEnergy = fn(&mut App, Entity, Vec3) -> f32;

/// Plugin callbacks used by the shared physics behavior checks.
///
/// The callbacks keep backend setup and fixed scene geometry behind typed
/// function pointers; every assertion reads backend-neutral ragdoll state.
#[derive(Clone, Copy, Debug)]
pub struct PhysicsBackend {
    /// Adds the backend after `RagdollPlugin` has initialized the app.
    add_backend: AddBackend,
    /// Adds a fixed scene collider from a shared shape and pose.
    add_fixed_shape: AddFixedShape,
    /// Measures one ragdoll's total energy in joules.
    measure_energy: MeasureEnergy,
}

impl PhysicsBackend {
    /// Creates the callbacks used by backend-generic physical checks.
    ///
    /// `add_backend` adds a physics plugin and its ragdoll adapter. The second
    /// callback adds fixed collision geometry without exposing backend types to
    /// this crate.
    ///
    /// # Examples
    ///
    /// ```
    /// # use bevy::math::{Isometry3d, Vec3};
    /// # use bevy::prelude::{App, Entity};
    /// # use bevy_ragdoll::ShapeSpec;
    /// # use bevy_ragdoll_conformance::physics::PhysicsBackend;
    /// # fn add_backend(_: &mut App) {}
    /// # fn add_fixed_shape(app: &mut App, _: ShapeSpec, _: Isometry3d) -> Entity {
    /// #     app.world_mut().spawn_empty().id()
    /// # }
    /// # fn measure_energy(_: &mut App, _: Entity, _: Vec3) -> f32 { 0.0 }
    /// let _backend = PhysicsBackend::new(add_backend, add_fixed_shape, measure_energy);
    /// ```
    pub const fn new(
        add_backend: AddBackend,
        add_fixed_shape: AddFixedShape,
        measure_energy: MeasureEnergy,
    ) -> Self {
        Self {
            add_backend,
            add_fixed_shape,
            measure_energy,
        }
    }
}

/// Builds the complete physics baseline with scenario-specific boundary values.
pub(super) const fn scenario_settings(
    gravity: Vec3,
    is_ccd_enabled: bool,
    max_spawn_lift: f32,
) -> RagdollPhysicsSettings {
    RagdollPhysicsSettings {
        gravity,
        max_substeps: 4,
        solver_iterations: 8,
        pgs_iterations: 2,
        linear_damping: 0.0,
        angular_damping: 0.05,
        min_inertia_radius: 0.08,
        friction: 0.7,
        restitution: 0.0,
        is_ccd_enabled,
        soft_ccd_prediction: 0.3,
        motor_frequency_hz: 4.0,
        motor_damping_ratio: 1.0,
        torque_scale: 1.0,
        threads: 4,
        joint_friction: 0.05,
        friction_rate: 20.0,
        pin_frequency_hz: 1.5,
        pin_damping_ratio: 1.0,
        pin_max_force: 340.0,
        pin_max_torque: 400.0,
        pin_distance_falloff: 2.0,
        sleep_linear_threshold: 0.05,
        sleep_angular_threshold: 0.1,
        settle_after: 10.0,
        force_sleep_after: 6.0,
        settle_speed: 0.5,
        max_spawn_lift,
        should_freeze_when_settled: false,
    }
}

/// A headless physics app with one active human ragdoll.
struct PhysicsScene {
    /// App that owns the physics world and runtime components.
    app: App,
    /// Character entity used to filter the scene's body snapshots.
    character: Entity,
    /// Validated profile used to interpret body indexes and joint limits.
    profile: RagdollProfile,
    /// Skeleton entities whose transforms drive target capture.
    bone_entities: Vec<Entity>,
    /// Authored local transforms restored before target-driven test steps.
    bone_targets: Vec<Transform>,
}

/// One body state copied in stable profile order for measurements.
#[derive(Clone, Copy)]
struct BodySnapshot {
    /// Checked index in the human profile.
    index: BodyIndex,
    /// Physics body entity used for impulses and Bevy queries.
    entity: Entity,
    /// World-space rigid pose after the latest completed physics step.
    pose: Isometry3d,
    /// World-space linear and angular velocity after the latest step.
    velocity: BodyVelocity,
    /// Validated body mass in kilograms.
    mass: f32,
    /// Local collision geometry used for centre-of-mass measurements.
    shape: ShapeSpec,
}

/// Root, drive, skeleton, motion, and fixed geometry for one physics scene.
struct ScenarioSetup<'a> {
    /// Character root pose in world space.
    root: Isometry3d,
    /// Unit-range joint muscle strength at spawn.
    muscle: f32,
    /// Unit-range world pin strength at spawn.
    pin: f32,
    /// Character mode after the skeleton binds.
    mode: RagdollMode,
    /// Authored bone poses in profile order.
    bone_poses: Vec<Isometry3d>,
    /// Initial world-space velocity applied through target motion.
    initial_velocity: Vec3,
    /// Fixed geometry spawned before the first physics update.
    fixed_shapes: &'a [(ShapeSpec, Isometry3d)],
}

/// Tolerated floor penetration, limit excess, and constraint separation.
#[derive(Clone, Copy)]
struct Bounds {
    /// Maximum floor penetration in metres.
    sink: f32,
    /// Maximum transient limit excess during landing, in degrees.
    landing_angle: f32,
    /// Maximum final limit excess after settling, in degrees.
    final_angle: f32,
    /// Maximum distance between joint anchors, in metres.
    gap: f32,
}

/// Ordinary fall tolerances copied from the physics checks.
const FALL: Bounds = Bounds {
    sink: 0.01,
    landing_angle: 5.0,
    final_angle: 2.0,
    gap: 0.01,
};

/// Measurements gathered while a body falls, lands, and settles.
#[derive(Debug, Default)]
struct Run {
    /// First time the pelvis or lower spine comes within 8 cm of the floor.
    ground: Option<f32>,
    /// Knee angles in degrees at the first ground contact.
    knees: [f32; 2],
    /// Largest head rotation relative to the chest after a front headshot.
    head_whip: f32,
    /// Horizontal pelvis travel after reaching the floor, in metres.
    slide: f32,
    /// Highest pelvis capsule point after the bounce measurement begins.
    bounce: f32,
    /// Time from ground contact until all bodies stay below the speed bounds.
    rest: Option<f32>,
    /// Largest body displacement over the last second, in metres.
    creep: f32,
    /// Largest body linear speed one step after an impulse, in metres/second.
    speed_after_hit: f32,
    /// Largest joint-limit excess over the run, in degrees.
    joint_excess: f32,
    /// Bone, axis, and time for the largest limit excess.
    joint_worst: String,
    /// Largest joint-limit excess on the final step, in degrees.
    joint_excess_at_end: f32,
}

/// Creates the common headless app and inserts shared settings before backend setup.
fn app(backend: PhysicsBackend, settings: RagdollPhysicsSettings) -> App {
    let mut app = App::new();
    // Register the transform, asset, and animation schedules needed by core binding.
    app.add_plugins((
        MinimalPlugins,
        TransformPlugin,
        AssetPlugin::default(),
        AnimationPlugin,
    ));
    app.insert_resource(Time::<Fixed>::from_hz(60.0));
    app.insert_resource(TimeUpdateStrategy::FixedTimesteps(1));
    // Let each backend read the final scenario settings during plugin construction.
    app.add_plugins(RagdollPlugin::default());
    app.insert_resource(settings);
    (backend.add_backend)(&mut app);
    app
}

/// Generates the human ragdoll profile from the reference humanoid skeleton.
fn human_profile() -> RagdollProfile {
    RagdollProfile::from_skeleton(&bevy_ragdoll::Skeleton::humanoid())
        .expect("the reference humanoid profile validates")
}

/// Converts a rigid isometry to a Bevy transform without introducing scale.
fn transform(pose: Isometry3d) -> Transform {
    Transform {
        translation: pose.translation.into(),
        rotation: pose.rotation,
        scale: Vec3::ONE,
    }
}

/// Spawns the profile's named skeleton and starts its bodies in the requested mode.
fn scene(
    backend: PhysicsBackend,
    settings: RagdollPhysicsSettings,
    root: Isometry3d,
    muscle: f32,
    pin: f32,
    mode: RagdollMode,
) -> PhysicsScene {
    let profile = human_profile();
    let bone_poses = profile.bodies().iter().map(|body| body.rest()).collect();
    scenario(
        backend,
        settings,
        profile,
        ScenarioSetup {
            root,
            muscle,
            pin,
            mode,
            bone_poses,
            initial_velocity: Vec3::ZERO,
            fixed_shapes: &[],
        },
    )
}

/// Builds a profile scene, captures optional initial velocity, and activates one mode.
fn scenario(
    backend: PhysicsBackend,
    settings: RagdollPhysicsSettings,
    profile: RagdollProfile,
    setup: ScenarioSetup<'_>,
) -> PhysicsScene {
    // Destructure one scenario description before configuring the backend app.
    let ScenarioSetup {
        root,
        muscle,
        pin,
        mode,
        bone_poses,
        initial_velocity,
        fixed_shapes,
    } = setup;
    let mut app = app(backend, settings);
    // Spawn the owner before direct-child skeleton bones and fixed geometry.
    let character = spawn_character(&mut app, &profile, root, muscle, pin);
    let bone_entities = attach_bones(
        &mut app,
        backend,
        character,
        &profile,
        bone_poses,
        fixed_shapes,
    );
    // Let binding initialize before target motion and mode changes are applied.
    app.update();
    apply_initial_velocity(&mut app, root, initial_velocity, &bone_entities);
    set_character_mode(&mut app, character, mode);
    app.update();
    // Save authored transforms for tests that restore animation targets each frame.
    let bone_targets = bone_entities
        .iter()
        .map(|entity| {
            app.world()
                .get::<Transform>(*entity)
                .copied()
                .expect("the skeleton bone transform remains available")
        })
        .collect();
    PhysicsScene {
        app,
        character,
        profile,
        bone_entities,
        bone_targets,
    }
}

/// Spawns the animated character root and stores its validated profile handle.
fn spawn_character(
    app: &mut App,
    profile: &RagdollProfile,
    root: Isometry3d,
    muscle: f32,
    pin: f32,
) -> Entity {
    let handle = app
        .world_mut()
        .get_resource_mut::<Assets<RagdollProfile>>()
        .expect("RagdollPlugin initializes profile assets")
        .add(profile.clone());
    app.world_mut()
        .spawn((
            Name::new("human"),
            Ragdoll::new(handle),
            RagdollMode::Animated,
            RagdollDrive::new(muscle, pin),
            transform(root),
        ))
        .id()
}

/// Applies optional world velocity to every local-space animation target.
fn apply_initial_velocity(app: &mut App, root: Isometry3d, velocity: Vec3, bones: &[Entity]) {
    // Avoid target writes and an extra schedule update for stationary scenes.
    if velocity == Vec3::ZERO {
        return;
    }
    // Convert world motion into the character's local target frame.
    let local_velocity = root.rotation.inverse() * velocity;
    for entity in bones {
        if let Some(mut bone) = app.world_mut().get_mut::<Transform>(*entity) {
            bone.translation += local_velocity / 60.0;
        }
    }
    app.update();
}

/// Switches the character root to the requested ragdoll mode.
fn set_character_mode(app: &mut App, character: Entity, mode: RagdollMode) {
    if let Some(mut current_mode) = app.world_mut().get_mut::<RagdollMode>(character) {
        *current_mode = mode;
    }
}

/// Adds the profile skeleton and fixed scene geometry before the first update.
fn attach_bones(
    app: &mut App,
    backend: PhysicsBackend,
    character: Entity,
    profile: &RagdollProfile,
    bone_poses: Vec<Isometry3d>,
    fixed_shapes: &[(ShapeSpec, Isometry3d)],
) -> Vec<Entity> {
    let mut bone_entities = Vec::with_capacity(profile.bodies().len());
    // Direct children preserve each body's skeleton-space rest transform and exact name.
    for (body, pose) in profile.bodies().iter().zip(bone_poses) {
        let bone = app.world_mut().spawn((
            Name::new(body.bone().to_owned()),
            transform(pose),
            ChildOf(character),
        ));
        bone_entities.push(bone.id());
    }
    // Add world geometry before the first fixed schedule initializes physics contexts.
    for (shape, pose) in fixed_shapes {
        (backend.add_fixed_shape)(app, *shape, *pose);
    }
    bone_entities
}

/// Adds one fixed floor from a horizontal cuboid with the requested half extents.
fn add_floor(scene: &mut PhysicsScene, backend: PhysicsBackend, half_extents: Vec3) {
    let (shape, pose) = floor_shape(half_extents);
    (backend.add_fixed_shape)(&mut scene.app, shape, pose);
}

/// Creates a fixed horizontal floor whose top surface lies at Y zero.
fn floor_shape(half_extents: Vec3) -> (ShapeSpec, Isometry3d) {
    let shape = ShapeSpec::Cuboid {
        center: Vec3::ZERO,
        rotation: Quat::IDENTITY,
        half_extents,
    };
    let pose = Isometry3d::from_translation(Vec3::new(0.0, -half_extents.y, 0.0));
    (shape, pose)
}

/// Restores authored bone targets after the previous frame's physics writeback.
fn restore_targets(scene: &mut PhysicsScene) {
    let world = scene.app.world_mut();
    scene
        .bone_entities
        .iter()
        .zip(&scene.bone_targets)
        .filter_map(|(entity, target)| {
            world
                .get_mut::<Transform>(*entity)
                .map(|mut transform| *transform = *target)
        })
        .for_each(drop);
}

/// Copies one character's live backend state and sorts it by profile index.
fn snapshots(world: &mut World, character: Entity) -> Vec<BodySnapshot> {
    let mut query = world.query::<(
        Entity,
        &RagdollBodyOf,
        &BodyIndex,
        &BodyPhysicsPose,
        &BodyVelocity,
        &BodyMass,
        &BodyShape,
    )>();
    let mut bodies = query
        .iter(world)
        .filter(|(_, owner, _, _, _, _, _)| owner.0 == character)
        .map(
            |(entity, _, index, pose, velocity, mass, shape)| BodySnapshot {
                index: *index,
                entity,
                pose: pose.current,
                velocity: *velocity,
                mass: mass.mass,
                shape: shape.0,
            },
        )
        .collect::<Vec<_>>();
    bodies.sort_unstable_by_key(|body| body.index.get());
    bodies
}

/// Finds one profile body by its exact skeleton name.
fn body_index(profile: &RagdollProfile, name: &str) -> BodyIndex {
    profile
        .body_index(name)
        .expect("the human profile contains each named test body")
}

/// Reads complete body poses in profile order after a physics update.
fn poses(scene: &mut PhysicsScene) -> Vec<Isometry3d> {
    // Snapshots are sorted by validated profile index before projecting poses.
    let bodies = snapshots(scene.app.world_mut(), scene.character);
    // A missing body would invalidate every later index-based measurement.
    assert_eq!(bodies.len(), scene.profile.bodies().len());
    bodies.into_iter().map(|body| body.pose).collect()
}

/// Returns the lowest world-space point of one supported profile shape.
fn lowest_point(shape: ShapeSpec, pose: Isometry3d) -> f32 {
    match shape {
        ShapeSpec::Capsule { a, b, radius } => {
            // Transform both capsule endpoints and subtract its spherical radius.
            let a = Vec3::from(pose.translation) + pose.rotation * a;
            let b = Vec3::from(pose.translation) + pose.rotation * b;
            a.y.min(b.y) - radius
        }
        ShapeSpec::Sphere { center, radius } => {
            // Transform the local centre before subtracting its radius.
            let center = Vec3::from(pose.translation) + pose.rotation * center;
            center.y - radius
        }
        ShapeSpec::Cuboid {
            center,
            rotation,
            half_extents,
        } => {
            // Evaluate all eight oriented corners for the rotated cuboid.
            let mut lowest = f32::INFINITY;
            for x in [-1.0, 1.0] {
                for y in [-1.0, 1.0] {
                    for z in [-1.0, 1.0] {
                        let corner = center + rotation * (half_extents * Vec3::new(x, y, z));
                        let world = Vec3::from(pose.translation) + pose.rotation * corner;
                        lowest = lowest.min(world.y);
                    }
                }
            }
            lowest
        }
    }
}

/// Returns the lowest surface point among one ragdoll's body shapes.
fn lowest_body_surface(world: &mut World, character: Entity) -> f32 {
    snapshots(world, character)
        .into_iter()
        .map(|body| lowest_point(body.shape, body.pose))
        .fold(f32::INFINITY, f32::min)
}

/// Measures the largest angular limit excess and parent-child anchor gap.
fn joint_errors(profile: &RagdollProfile, poses: &[Isometry3d]) -> (f32, f32, f32, String, String) {
    // Keep transient angle error, final angle error, and anchor gap independent.
    let (worst_angle, worst) = worst_joint_angle(profile, poses);
    let final_angle = final_joint_angle(profile, poses);
    let (worst_gap, worst_gap_bone) = worst_joint_gap(profile, poses);
    (worst_angle, final_angle, worst_gap, worst, worst_gap_bone)
}

/// Finds the largest limit excess and labels its bone and axis.
fn worst_joint_angle(profile: &RagdollProfile, poses: &[Isometry3d]) -> (f32, String) {
    let (mut worst_angle, mut worst) = (0.0_f32, String::new());
    // Walk profile joints so measurements stay independent of backend ordering.
    for joint in profile.joints() {
        // Missing child angles indicate an incomplete pose set for this joint.
        let Some(angles) = profile.joint_angles(joint.child(), poses) else {
            continue;
        };
        let limits = joint.limits();
        // Compare each Euler component against its matching authored range.
        for (axis, (angle, range)) in [
            (angles.x, limits.x),
            (angles.y, limits.twist),
            (angles.z, limits.z),
        ]
        .into_iter()
        .enumerate()
        {
            let excess = angle_limit_excess(angle, range.min, range.max);
            if excess > worst_angle {
                worst_angle = excess;
                // Resolve the label only when this joint becomes the worst case.
                let Some(body) = profile.bodies().get(joint.child().get()) else {
                    continue;
                };
                let bone = body.bone();
                worst = format!("{bone} axis {axis}");
            }
        }
    }
    (worst_angle, worst)
}

/// Computes angular distance outside one authored joint range in radians.
fn angle_limit_excess(angle: f32, minimum: f32, maximum: f32) -> f32 {
    (angle - maximum).max(minimum - angle).max(0.0)
}

/// Finds the greatest parent-frame anchor separation and its child bone.
fn worst_joint_gap(profile: &RagdollProfile, poses: &[Isometry3d]) -> (f32, String) {
    let (mut worst_gap, mut worst_gap_bone) = (0.0_f32, String::new());
    // Skip incomplete pose sets rather than indexing outside the profile.
    for joint in profile.joints() {
        // The parent, child, and child profile entry must all be present.
        let (Some(parent), Some(child), Some(child_body)) = (
            poses.get(joint.parent().get()),
            poses.get(joint.child().get()),
            profile.bodies().get(joint.child().get()),
        ) else {
            continue;
        };
        // Transform the authored parent frame before measuring its child gap.
        let anchor = *parent * joint.frame();
        let gap = anchor.translation.distance(child.translation);
        if gap > worst_gap {
            worst_gap = gap;
            worst_gap_bone = child_body.bone().to_owned();
        }
    }
    (worst_gap, worst_gap_bone)
}

/// Measures maximum angular limit excess for the supplied final pose set.
fn final_joint_angle(profile: &RagdollProfile, poses: &[Isometry3d]) -> f32 {
    let mut final_angle = 0.0_f32;
    // Invalid or incomplete pose sets contribute no final measurement.
    for joint in profile.joints() {
        // Evaluate all three ranges from this joint's relative pose.
        if let Some(angles) = profile.joint_angles(joint.child(), poses) {
            let limits = joint.limits();
            let ranges = [limits.x, limits.twist, limits.z];
            let values = [angles.x, angles.y, angles.z];
            for (angle, range) in values.into_iter().zip(ranges) {
                final_angle = final_angle.max(angle_limit_excess(angle, range.min, range.max));
            }
        }
    }
    final_angle
}

/// Computes the shape centre in body-local metres.
fn local_center(shape: ShapeSpec) -> Vec3 {
    match shape {
        ShapeSpec::Capsule { a, b, .. } => (a + b) * 0.5,
        ShapeSpec::Sphere { center, .. } | ShapeSpec::Cuboid { center, .. } => center,
    }
}

/// Measures centre of mass, total linear momentum, and mass for one ragdoll.
fn com_and_momentum(world: &mut World, character: Entity) -> (Vec3, Vec3, f32) {
    // Use profile-ordered snapshots so mass and body pose remain paired.
    let bodies = snapshots(world, character);
    let (mut weighted_position, mut momentum, mut total_mass) = (Vec3::ZERO, Vec3::ZERO, 0.0);
    for body in bodies {
        // Weight each shape centre and velocity by the same body's mass.
        let center =
            Vec3::from(body.pose.translation) + body.pose.rotation * local_center(body.shape);
        weighted_position += center * body.mass;
        momentum += body.velocity.linear * body.mass;
        total_mass += body.mass;
    }
    // Divide only the weighted position; total momentum stays in kg·m/s.
    (weighted_position / total_mass, momentum, total_mass)
}

/// Checks free fall at both the default and game gravity values.
///
/// The profile's mass-weighted centre of mass must fall one metre in the
/// analytic free-fall time, within five percent, at 9.81 and 20 m/s².
///
/// # Examples
///
/// ```
/// # use bevy_ragdoll_conformance::physics::{PhysicsBackend, a_ragdoll_falls_as_gravity_says};
/// # fn register(_: fn(PhysicsBackend)) {}
/// register(a_ragdoll_falls_as_gravity_says);
/// ```
pub fn a_ragdoll_falls_as_gravity_says(backend: PhysicsBackend) {
    // Exercise both the crate default and the authored game acceleration.
    for gravity in [9.81_f32, 20.0] {
        let settings = scenario_settings(Vec3::new(0.0, -gravity, 0.0), true, 0.5);
        let mut scene = scene(
            backend,
            settings,
            Isometry3d::from_translation(Vec3::Y * 50.0),
            0.0,
            0.0,
            RagdollMode::Dynamic,
        );
        // Compare mass-centre motion to the analytic one-metre fall time.
        let (start, _, _) = com_and_momentum(scene.app.world_mut(), scene.character);
        let expected = (2.0 / gravity).sqrt();
        let mut elapsed = 0.0;
        // Advance fixed 60 Hz steps until the centre has fallen one metre.
        loop {
            scene.app.update();
            elapsed += 1.0 / 60.0;
            let (current, _, _) = com_and_momentum(scene.app.world_mut(), scene.character);
            if start.y - current.y >= 1.0 {
                break;
            }
            assert!(elapsed < 2.0, "the ragdoll never fell one metre");
        }
        assert!(
            (elapsed - expected).abs() <= expected * 0.05,
            "gravity {gravity} m/s²: one metre in {elapsed} s, expected {expected} s"
        );
    }
}

/// Applies a point impulse and checks that the connected rig gains its momentum.
///
/// The impulse is 20 N·s along +Z in zero gravity. After thirty fixed steps,
/// total momentum must remain within 1 N·s on Z and 0.5 N·s on X and Y.
///
/// # Examples
///
/// ```
/// # use bevy_ragdoll_conformance::physics::{PhysicsBackend, an_impulse_gives_its_momentum};
/// # fn register(_: fn(PhysicsBackend)) {}
/// register(an_impulse_gives_its_momentum);
/// ```
pub fn an_impulse_gives_its_momentum(backend: PhysicsBackend) {
    // Remove gravity so momentum change comes only from the authored impulse.
    let settings = scenario_settings(Vec3::ZERO, true, 0.5);
    let mut scene = scene(
        backend,
        settings,
        Isometry3d::from_translation(Vec3::Y * 50.0),
        0.0,
        0.0,
        RagdollMode::Dynamic,
    );
    // Apply the impulse at the chest centre to avoid adding torque.
    let chest_index = body_index(&scene.profile, "spine_02");
    let chest = snapshots(scene.app.world_mut(), scene.character)
        .into_iter()
        .find(|body| body.index == chest_index)
        .expect("the chest body was spawned");
    let center =
        Vec3::from(chest.pose.translation) + chest.pose.rotation * local_center(chest.shape);
    scene.app.world_mut().write_message(RagdollImpulse {
        body: chest.entity,
        point: center,
        impulse: Vec3::new(0.0, 0.0, 20.0),
    });
    // Allow constraints to distribute the impulse through the connected rig.
    for _ in 0..30 {
        scene.app.update();
    }
    let (_, momentum, _) = com_and_momentum(scene.app.world_mut(), scene.character);
    assert!((momentum.z - 20.0).abs() <= 1.0, "momentum {momentum}");
    assert!(
        momentum.x.abs() < 0.5 && momentum.y.abs() < 0.5,
        "momentum {momentum}"
    );
}

/// Verifies that captured target velocities come from two poses and elapsed time.
///
/// The pelvis target moves 0.1 m and rotates 0.05 rad over 0.01 s. Captured
/// velocity must report 10 m/s on X and 5 rad/s around Y within the source test's
/// tolerances.
///
/// # Examples
///
/// ```
/// # use bevy_ragdoll_conformance::physics::{PhysicsBackend, velocities_come_from_two_poses};
/// # fn register(_: fn(PhysicsBackend)) {}
/// register(velocities_come_from_two_poses);
/// ```
pub fn velocities_come_from_two_poses(backend: PhysicsBackend) {
    // Move and rotate the target over exactly one 100 Hz frame.
    let (mut app, character, pelvis, pelvis_entity) = velocity_capture_scene(backend);
    let mut pelvis_transform = app
        .world_mut()
        .get_mut::<Transform>(pelvis_entity)
        .expect("the pelvis bone has a transform");
    pelvis_transform.translation.x += 0.1;
    pelvis_transform.rotation *= Quat::from_rotation_y(0.05);
    app.update();
    // Read the captured velocity after target-pose processing completes.
    let velocity = app
        .world()
        .get::<RagdollTargetPose>(character)
        .and_then(|targets| targets.velocity(pelvis))
        .expect("target capture reports the pelvis velocity");
    assert!((velocity.linear - Vec3::new(10.0, 0.0, 0.0)).length() < 1e-4);
    assert!((velocity.angular - Vec3::new(0.0, 5.0, 0.0)).length() < 1e-3);
}

/// Builds a 100 Hz animated skeleton used to verify target-velocity capture.
fn velocity_capture_scene(backend: PhysicsBackend) -> (App, Entity, BodyIndex, Entity) {
    // Configure the same 10 ms fixed duration used by the velocity assertion.
    let settings = RagdollPhysicsSettings::default();
    let mut app = app(backend, settings);
    app.insert_resource(Time::<Fixed>::from_hz(100.0));
    app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_millis(
        10,
    )));
    // Add the profile and animated owner before its named skeleton children.
    let profile = human_profile();
    let character = spawn_animated_profile(&mut app, &profile);
    app.update();
    // Resolve the named pelvis only after Bevy has applied child relationships.
    let pelvis = body_index(&profile, "pelvis");
    let pelvis_entity = named_bone_entity(app.world_mut(), "pelvis");
    (app, character, pelvis, pelvis_entity)
}

/// Spawns one animated owner and all profile rest-pose skeleton children.
fn spawn_animated_profile(app: &mut App, profile: &RagdollProfile) -> Entity {
    // Store the validated profile before creating its owner entity.
    let handle = app
        .world_mut()
        .get_resource_mut::<Assets<RagdollProfile>>()
        .expect("RagdollPlugin initializes profile assets")
        .add(profile.clone());
    let character = app
        .world_mut()
        .spawn((
            Name::new("human"),
            Ragdoll::new(handle),
            transform(Isometry3d::IDENTITY),
        ))
        .id();
    // Keep each child transform equal to the profile's authored rest pose.
    for body in profile.bodies() {
        app.world_mut().spawn((
            Name::new(body.bone().to_owned()),
            transform(body.rest()),
            ChildOf(character),
        ));
    }
    character
}

/// Finds a profile skeleton entity by its exact Bevy name.
fn named_bone_entity(world: &mut World, bone_name: &str) -> Entity {
    // Search the completed world after the caller applies its hierarchy update.
    let mut query = world.query::<(Entity, &Name)>();
    query
        .iter(world)
        .find_map(|(entity, name)| (name.as_str() == bone_name).then_some(entity))
        .expect("the named skeleton bone exists")
}

/// Confirms frozen profile bodies keep their initial poses for thirty steps.
///
/// The frozen state holds the pelvis at its authored pose while Rapier
/// advances the fixed schedule.
///
/// # Examples
///
/// ```
/// # use bevy_ragdoll_conformance::physics::{PhysicsBackend, a_frozen_ragdoll_stays_put};
/// # fn register(_: fn(PhysicsBackend)) {}
/// register(a_frozen_ragdoll_stays_put);
/// ```
pub fn a_frozen_ragdoll_stays_put(backend: PhysicsBackend) {
    // Capture the frozen bodies before and after thirty fixed updates.
    let root = Isometry3d::from_translation(Vec3::Y * 10.0);
    let mut scene = scene(
        backend,
        RagdollPhysicsSettings::default(),
        root,
        0.0,
        0.0,
        RagdollMode::Frozen,
    );
    let before = poses(&mut scene);
    // Advance the fixed schedule without rewriting the frozen target transforms.
    for _ in 0..30 {
        scene.app.update();
    }
    let after = poses(&mut scene);
    let pelvis = body_index(&scene.profile, "pelvis").get();
    let before_pelvis = before
        .get(pelvis)
        .expect("the pelvis pose was captured before the fixed steps");
    let after_pelvis = after
        .get(pelvis)
        .expect("the pelvis pose was captured after the fixed steps");
    // Compare both physical stability and the authored pelvis rest pose.
    let rest_pelvis = scene
        .profile
        .rest_poses(root)
        .nth(pelvis)
        .expect("the profile includes a rest pose for the pelvis");
    assert_eq!(after_pelvis, before_pelvis);
    assert_eq!(*before_pelvis, rest_pelvis);
}

/// Checks that native joint motors hold both knees at a 60 degree target.
///
/// Zero gravity and 120 fixed steps match the source test. Each calf remains
/// within five degrees of its target pose.
///
/// # Examples
///
/// ```
/// # use bevy_ragdoll_conformance::physics as ragdoll_physics;
/// # use ragdoll_physics::{PhysicsBackend, motors_hold_a_target_pose};
/// # fn register(_: fn(PhysicsBackend)) {}
/// register(motors_hold_a_target_pose);
/// ```
pub fn motors_hold_a_target_pose(backend: PhysicsBackend) {
    hold_target_pose(backend, true);
}

/// Checks that stable-PD torques hold both knees when native motors are disabled.
///
/// The setup and 60 degree knee targets match [`motors_hold_a_target_pose`],
/// while `BackendCapabilities::has_native_joint_motors` selects the fallback path.
///
/// # Examples
///
/// ```
/// # use bevy_ragdoll_conformance::physics as ragdoll_physics;
/// # use ragdoll_physics::{PhysicsBackend, torque_drive_holds_a_target_pose};
/// # fn register(_: fn(PhysicsBackend)) {}
/// register(torque_drive_holds_a_target_pose);
/// ```
pub fn torque_drive_holds_a_target_pose(backend: PhysicsBackend) {
    hold_target_pose(backend, false);
}

/// Runs the shared knee target test with the selected motor capability.
fn hold_target_pose(backend: PhysicsBackend, has_native_joint_motors: bool) {
    // Disable gravity to isolate pose tracking from falling motion.
    let settings = scenario_settings(Vec3::ZERO, true, 0.5);
    let mut scene = scene(
        backend,
        settings,
        Isometry3d::from_translation(Vec3::Y * 50.0),
        1.0,
        0.0,
        RagdollMode::Dynamic,
    );
    configure_knee_targets(&mut scene, has_native_joint_motors);
    // Restore targets before each update so writeback cannot become the next target.
    for _ in 0..120 {
        restore_targets(&mut scene);
        scene.app.update();
    }
    // Compare final child-relative knee angles with the target bend.
    assert_knee_targets(&mut scene);
}

/// Selects the advertised motor path and bends both calf targets by 60 degrees.
fn configure_knee_targets(scene: &mut PhysicsScene, has_native_joint_motors: bool) {
    // Match backend capability before the fixed schedule selects a drive path.
    scene
        .app
        .world_mut()
        .get_resource_mut::<bevy_ragdoll::runtime::backend::BackendCapabilities>()
        .expect("RagdollPlugin installs backend capabilities")
        .has_native_joint_motors = has_native_joint_motors;
    // Rotate both calf targets while preserving their authored translations.
    for side in ["l", "r"] {
        let index = body_index(&scene.profile, &format!("calf_{side}")).get();
        let entity = scene
            .bone_entities
            .get(index)
            .copied()
            .expect("the calf profile index has a skeleton entity");
        let mut target = scene
            .app
            .world_mut()
            .get_mut::<Transform>(entity)
            .expect("the calf skeleton bone has a transform");
        target.rotation *= Quat::from_rotation_x(-60.0_f32.to_radians());
        *scene
            .bone_targets
            .get_mut(index)
            .expect("the calf profile index has an authored target") = *target;
    }
}

/// Asserts both profile-ordered calf joints remain within five degrees of target.
fn assert_knee_targets(scene: &mut PhysicsScene) {
    // Read the final backend poses after all 120 target-driven steps.
    let current = poses(scene);
    // Compare child-relative angles for each side using the authored target bend.
    for side in ["l", "r"] {
        let name = format!("calf_{side}");
        let index = body_index(&scene.profile, &name);
        let angle = scene
            .profile
            .joint_angles(index, &current)
            .expect("each calf has a profile joint")
            .x
            .to_degrees();
        assert!((angle + 60.0).abs() < 5.0, "{name} at {angle} degrees");
    }
}

/// Checks a 15 cm spawn overlap with a zero, invalid, and enabled lift bound.
///
/// A zero or invalid bound leaves the authored surface below -14 cm. With a
/// 50 cm lift, the first physics result lies between -1 mm and 11 mm, and the
/// pelvis remains above the floor after two seconds.
///
/// # Examples
///
/// ```
/// # use bevy_ragdoll_conformance::physics as ragdoll_physics;
/// # use ragdoll_physics::{PhysicsBackend, a_spawn_with_feet_in_the_floor_lifts_out_of_it};
/// # fn register(_: fn(PhysicsBackend)) {}
/// register(a_spawn_with_feet_in_the_floor_lifts_out_of_it);
/// ```
pub fn a_spawn_with_feet_in_the_floor_lifts_out_of_it(backend: PhysicsBackend) {
    // Zero and non-finite limits must preserve the authored overlap.
    let (unlifted_surface, _) = spawn_overlap_sample(backend, 0.0);
    assert!(unlifted_surface < -0.14, "lowest point {unlifted_surface}");
    let (invalid_bound_surface, _) = spawn_overlap_sample(backend, f32::NAN);
    assert!(
        invalid_bound_surface < -0.14,
        "non-finite lift bound left the surface at {invalid_bound_surface}"
    );
    // A finite positive limit must lift the feet and keep the pelvis supported.
    let (lifted_surface, pelvis_y) = spawn_overlap_sample(backend, 0.5);
    assert!(
        (-0.001..0.011).contains(&lifted_surface),
        "lowest point {lifted_surface}"
    );
    assert!(pelvis_y > 0.0, "pelvis at {pelvis_y}");
}

/// Spawns one overlapping character and measures first-step lift and final pelvis height.
fn spawn_overlap_sample(backend: PhysicsBackend, max_spawn_lift: f32) -> (f32, f32) {
    // Vary only the spawn correction bound for each boundary case.
    let settings = scenario_settings(Vec3::new(0.0, -9.81, 0.0), true, max_spawn_lift);
    let profile = human_profile();
    let bone_poses = profile.bodies().iter().map(|body| body.rest()).collect();
    let floor = floor_shape(Vec3::new(51.2, 0.1, 51.2));
    // The floor exists before the first backend update measures the authored overlap.
    let mut scene = scenario(
        backend,
        settings,
        profile,
        ScenarioSetup {
            root: Isometry3d::from_translation(Vec3::Y * -0.15),
            muscle: 0.0,
            pin: 0.0,
            mode: RagdollMode::Dynamic,
            bone_poses,
            initial_velocity: Vec3::ZERO,
            fixed_shapes: std::slice::from_ref(&floor),
        },
    );
    // Capture the authored overlap before the backend's first correction step.
    let pre_step_surface = lowest_body_surface(scene.app.world_mut(), scene.character);
    scene.app.update();
    let first_step_surface = lowest_body_surface(scene.app.world_mut(), scene.character);
    let initial_surface = if max_spawn_lift > 0.0 {
        first_step_surface
    } else {
        pre_step_surface
    };
    let pelvis_y = pelvis_height_after_steps(&mut scene, 120);
    (initial_surface, pelvis_y)
}

/// Measures the pelvis height after the requested number of fixed updates.
fn pelvis_height_after_steps(scene: &mut PhysicsScene, steps: usize) -> f32 {
    // Advance the same active scene so its original overlap and lift remain measurable.
    for _ in 0..steps {
        scene.app.update();
    }
    // Read the profile-ordered pelvis pose after all fixed updates complete.
    let pelvis = body_index(&scene.profile, "pelvis").get();
    poses(scene)
        .get(pelvis)
        .expect("the pelvis pose is captured")
        .translation
        .y
}

/// Keeps a fully driven pelvis at the idle target for ten seconds.
///
/// Muscle and pin strength are one. Pelvis drift stays below five centimetres,
/// and mean absolute joint angle remains below five degrees.
///
/// # Examples
///
/// ```
/// # use bevy_ragdoll_conformance::physics as ragdoll_physics;
/// # use ragdoll_physics::{PhysicsBackend, pinned_pelvis_stands_for_ten_seconds};
/// # fn register(_: fn(PhysicsBackend)) {}
/// register(pinned_pelvis_stands_for_ten_seconds);
/// ```
pub fn pinned_pelvis_stands_for_ten_seconds(backend: PhysicsBackend) {
    // Spawn the fully driven body at its authored idle pose above a broad floor.
    let mut scene = scene(
        backend,
        RagdollPhysicsSettings::default(),
        Isometry3d::IDENTITY,
        1.0,
        1.0,
        RagdollMode::Dynamic,
    );
    add_floor(&mut scene, backend, Vec3::new(51.2, 0.8, 51.2));
    // Capture the pelvis reference before restoring targets on each step.
    let pelvis_index = body_index(&scene.profile, "pelvis").get();
    let starting = poses(&mut scene)
        .get(pelvis_index)
        .expect("the pelvis pose is captured before the idle run")
        .translation;
    // Run exactly ten seconds at the scenario's 60 Hz fixed rate.
    for _ in 0..600 {
        restore_targets(&mut scene);
        scene.app.update();
    }
    let current = poses(&mut scene);
    let current_pelvis = current
        .get(pelvis_index)
        .expect("the pelvis pose is captured after the idle run");
    let pelvis_drift = current_pelvis.translation.distance(starting);
    assert!(pelvis_drift < 0.05, "pelvis drift {pelvis_drift} m");

    // Average the available joint angles after measuring pelvis drift.
    let (mut joint_error, mut joint_count) = (0.0_f32, 0_usize);
    for joint in scene.profile.joints() {
        if let Some(angles) = scene.profile.joint_angles(joint.child(), &current) {
            joint_error += angles.length();
            joint_count += 1;
        }
    }
    let mean_joint_error = (joint_error / joint_count as f32).to_degrees();
    assert!(
        mean_joint_error < 5.0,
        "mean joint error {mean_joint_error} degrees"
    );
}

#[cfg(test)]
mod tests {
    //! Checks profile-shape measurements and stair coordinate conversion.

    use bevy::math::{Isometry3d, Quat, Vec3};
    use bevy::prelude::Entity;

    use super::fall::simulation_steps;
    use super::look::{et_box, stairs, stairs_height, surface_clearance};
    use super::{BodySnapshot, joint_errors, local_center, lowest_point, scenario_settings};
    use crate::physics::BodyVelocity;
    use bevy_ragdoll::ShapeSpec;
    use bevy_ragdoll::profile::BodyIndex;
    use bevy_ragdoll::runtime::settings::RagdollPhysicsSettings;

    /// Scenario overrides retain every other explicit default.
    #[test]
    fn scenario_settings_keeps_the_complete_human_baseline() {
        // Select non-default gravity, CCD, and spawn lift to exercise each input.
        let gravity = Vec3::new(0.0, -20.0, 0.0);
        let settings = scenario_settings(gravity, false, 0.25);
        let expected = RagdollPhysicsSettings {
            gravity,
            max_substeps: 4,
            solver_iterations: 8,
            pgs_iterations: 2,
            linear_damping: 0.0,
            angular_damping: 0.05,
            min_inertia_radius: 0.08,
            friction: 0.7,
            restitution: 0.0,
            is_ccd_enabled: false,
            soft_ccd_prediction: 0.3,
            motor_frequency_hz: 4.0,
            motor_damping_ratio: 1.0,
            torque_scale: 1.0,
            threads: 4,
            joint_friction: 0.05,
            friction_rate: 20.0,
            pin_frequency_hz: 1.5,
            pin_damping_ratio: 1.0,
            pin_max_force: 340.0,
            pin_max_torque: 400.0,
            pin_distance_falloff: 2.0,
            sleep_linear_threshold: 0.05,
            sleep_angular_threshold: 0.1,
            settle_after: 10.0,
            force_sleep_after: 6.0,
            settle_speed: 0.5,
            max_spawn_lift: 0.25,
            should_freeze_when_settled: false,
        };
        assert_eq!(settings, expected);
    }

    /// Sphere, capsule, and cuboid clearance includes each local shape transform.
    #[test]
    fn lowest_point_handles_every_profile_shape() {
        let pose = Isometry3d::from_translation(Vec3::new(0.0, 10.0, 0.0));
        let capsule = ShapeSpec::Capsule {
            a: Vec3::new(0.0, -0.5, 0.0),
            b: Vec3::new(0.0, 0.5, 0.0),
            radius: 0.25,
        };
        let sphere = ShapeSpec::Sphere {
            center: Vec3::new(0.0, 1.0, 0.0),
            radius: 0.5,
        };
        let cuboid = ShapeSpec::Cuboid {
            center: Vec3::new(0.0, 0.5, 0.0),
            rotation: Quat::IDENTITY,
            half_extents: Vec3::new(1.0, 2.0, 3.0),
        };

        assert_eq!(lowest_point(capsule, pose), 9.25);
        assert_eq!(lowest_point(sphere, pose), 10.5);
        assert_eq!(lowest_point(cuboid, pose), 8.5);
    }

    /// Non-capsule surfaces use their lowest point and the terrain at body Z.
    #[test]
    fn surface_clearance_handles_spheres_and_cuboids() {
        let sphere = ShapeSpec::Sphere {
            center: Vec3::ZERO,
            radius: 0.5,
        };
        let cuboid = ShapeSpec::Cuboid {
            center: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            half_extents: Vec3::new(0.5, 1.0, 0.5),
        };
        let snapshot = |shape| BodySnapshot {
            index: BodyIndex::try_from(0).expect("profile body index zero is valid"),
            entity: Entity::PLACEHOLDER,
            pose: Isometry3d::from_translation(Vec3::new(0.0, 2.0, 0.0)),
            velocity: BodyVelocity::default(),
            mass: 1.0,
            shape,
        };

        assert_eq!(surface_clearance(snapshot(sphere), |_| 0.25), 1.25);
        assert_eq!(surface_clearance(snapshot(cuboid), |_| 0.25), 0.75);
    }

    /// Sphere and cuboid centres remain their authored local-frame centres.
    #[test]
    fn local_center_handles_non_capsule_shapes() {
        let center = Vec3::new(0.1, 0.2, 0.3);
        let sphere = ShapeSpec::Sphere {
            center,
            radius: 0.5,
        };
        let cuboid = ShapeSpec::Cuboid {
            center,
            rotation: Quat::IDENTITY,
            half_extents: Vec3::ONE,
        };

        assert_eq!(local_center(sphere), center);
        assert_eq!(local_center(cuboid), center);
    }

    /// ET bounds convert to metres with reference's XZY coordinate mapping.
    #[test]
    fn et_box_maps_axis_order_and_scale() {
        let (shape, pose) = et_box([0.0, 2.0, -4.0], [8.0, 6.0, 0.0]);
        assert_eq!(
            shape,
            ShapeSpec::Cuboid {
                center: Vec3::ZERO,
                rotation: Quat::IDENTITY,
                half_extents: Vec3::new(0.1, 0.05, 0.05),
            }
        );
        assert_eq!(Vec3::from(pose.translation), Vec3::new(0.1, -0.05, -0.1));
    }

    /// Stair construction has one landing, twenty steps, and exact edge heights.
    #[test]
    fn stairs_have_twenty_steps_and_expected_height_edges() {
        assert_eq!(stairs().len(), 21);
        assert_eq!(stairs_height(0.0), 0.0);
        assert_eq!(stairs_height(-0.1), -0.2);
        assert_eq!(stairs_height(-0.4), -0.4);
    }

    /// Converts common durations to their nearest 60 Hz fixed-step counts.
    #[test]
    fn simulation_steps_uses_sixtieths_of_a_second() {
        assert_eq!(simulation_steps(0.0), 0);
        assert_eq!(simulation_steps(0.25), 15);
        assert_eq!(simulation_steps(0.5), 30);
        assert_eq!(simulation_steps(3.0), 180);
    }

    /// Joint error reporting skips profile joints when the pose slice is incomplete.
    #[test]
    fn joint_errors_skip_missing_child_poses() {
        let profile = super::human_profile();

        assert_eq!(
            joint_errors(&profile, &[]),
            (0.0, 0.0, 0.0, String::new(), String::new())
        );
    }
}
