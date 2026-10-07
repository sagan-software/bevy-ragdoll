//! This module constructs repeatable profile inputs and bound skeletons for
//! benchmark setup.
//!
//! The checked-in TGF human and seeded synthetic chains provide deterministic
//! body and joint data. `core_app` prepares capture and writeback populations,
//! while `rapier_app` adds physics resources and a floor. App constructors
//! reject missing profile storage and invalid joint parent ordering before
//! Criterion starts timing an iteration.

use std::time::Duration;

use bevy::app::{App, ScheduleRunnerPlugin};
use bevy::asset::{AssetPlugin, Assets};
use bevy::math::{Isometry3d, Vec3};
use bevy::prelude::{
    ChildOf, Entity, GlobalTransform, MinimalPlugins, Name, PluginGroup, Transform, With, World,
};
use bevy::time::{Fixed, Time, TimeUpdateStrategy};
use bevy::transform::TransformPlugin;
use bevy_ragdoll::runtime::body::{BodyPhysicsPose, BodyShape};
use bevy_ragdoll::runtime::components::{Ragdoll, RagdollDrive, RagdollMode};
use bevy_ragdoll::runtime::settings::RagdollPhysicsSettings;
use bevy_ragdoll::{
    AngleRange, Body, JointLimits, ProfileBuilder, ProfileSpec, RagdollPlugin, RagdollProfile,
    ShapeSpec,
};
use bevy_ragdoll_rapier3d::{RapierRagdollHooks, RapierRagdollPlugin};
use bevy_rapier3d::plugin::{RapierPhysicsPlugin, TimestepMode};
use bevy_rapier3d::prelude::{Collider, RigidBody, Sleeping};
use bevy_rapier3d::rapier::dynamics::RigidBodyActivation;
use rand_chacha::ChaCha8Rng;
use rand_core::{Rng, SeedableRng};
use thiserror::Error;

/// Seed used to recreate profile dimensions and character positions across
/// separate benchmark runs.
///
/// This fixed value keeps the generated fixture inputs stable for a given code
/// revision, allowing
/// Criterion results to compare equivalent populations instead of measuring new
/// random layouts.
pub const BENCH_SEED: u64 = 42;

/// Selects the rigid-body and ragdoll-drive state applied while a benchmark
/// population is prepared.
///
/// Each mode keeps setup behavior consistent across the core and Rapier
/// targets. The measured
/// operation starts after the app has bound every profile body and applied the
/// selected state.
#[derive(Clone, Copy, Debug)]
pub enum PopulationMode {
    /// Keeps bodies dynamic and disables muscle and pin drive while passive
    /// physics is measured
    /// without changing the seeded population layout used by the other
    /// benchmark modes.
    Limp,
    /// Keeps bodies dynamic and enables full muscle drive with half pin drive
    /// for active cases
    /// while preserving the same seeded character positions as passive physics
    /// runs.
    Powered,
    /// Binds dynamic Rapier bodies, then marks each body asleep before physics
    /// steps are measured
    /// so the benchmark isolates sleeping-body simulation cost at a fixed
    /// timestep.
    Asleep,
    /// Keeps ragdoll bodies kinematic so capture runs while physics writeback
    /// does not move targets
    /// or add backend stepping work to the measured core update.
    Capture,
}

/// Reports setup failures that would otherwise make a benchmark measure an
/// incomplete population.
///
/// The variants distinguish missing Bevy resources from invalid profile indexes
/// and entities that
/// disappear while the setup code applies state. `core_app` and `rapier_app`
/// return this error
/// before a caller starts the timed iteration.
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
pub enum BenchmarkSetupError {
    /// Returned when `RagdollPlugin` has not installed profile asset storage
    /// before setup creates
    /// characters that share one validated profile handle for the measured
    /// benchmark population.
    #[error("profile asset storage is not installed")]
    MissingProfileAssetStorage,
    /// Returned when a joint child index is greater than or equal to the
    /// profile body count, so no
    /// body slot can receive the relationship during benchmark setup.
    #[error("joint child index is outside the profile body array")]
    JointChildIndexOutOfRange,
    /// Returned when a joint parent index is greater than or equal to the
    /// profile body count, so no
    /// profile body can serve as that relationship's parent.
    #[error("joint parent index is outside the profile body array")]
    JointParentIndexOutOfRange,
    /// Returned when a joint points to itself or a body whose profile index
    /// follows its child,
    /// which breaks parent-first entity binding.
    #[error("joint parent must appear before its child in the profile body array")]
    ParentNotFirst,
    /// Returned when separate profile joints assign more than one parent to the
    /// same child body,
    /// which cannot form one parent-first skeleton hierarchy.
    #[error("profile body has more than one parent joint")]
    DuplicateJointChild,
    /// Returned when the checked parent-slot list does not have exactly one
    /// entry for every profile
    /// body before the setup code starts spawning bones.
    #[error("profile parent map length does not match the body array")]
    ParentMapLengthMismatch,
    /// Returned when the profile requires an earlier parent entity but that
    /// entity is absent from
    /// the bone list while setup binds a child body.
    #[error("profile parent bone was not spawned before its child")]
    ParentBoneNotSpawned,
    /// Returned when Bevy cannot mutably fetch a queried body before benchmark
    /// state is inserted.
    /// The source retains whether the entity vanished or its mutable access
    /// aliased another fetch.
    #[error("body entity access failed while benchmark state was being applied: {0}")]
    BodyEntityAccess(#[from] bevy::ecs::world::error::EntityMutableFetchError),
}

/// Loads and validates the checked-in TGF human profile used by benchmark
/// fixtures.
///
/// The checked-in RON file supplies the named 16-body rig used for capture,
/// writeback, and Rapier
/// cases. This helper panics only if that repository fixture stops parsing or
/// fails profile
/// validation.
///
/// # Examples
///
/// ```
/// use bevy_ragdoll_benches::support::human_profile;
///
/// let profile = human_profile();
/// assert_eq!(profile.bodies().len(), 16);
/// ```
pub fn human_profile() -> RagdollProfile {
    let source = include_str!("../../assets/profiles/tgf_human.ragdoll.ron");
    let spec = ron::from_str(source).expect("checked-in TGF profile parses");
    RagdollProfile::new(spec).expect("checked-in TGF profile validates")
}

/// Builds a validated parent-first chain containing between one and 64 profile
/// bodies.
///
/// `seed` fixes each capsule's radius and length, and each body after the root
/// joins its immediate
/// predecessor. The builder rejects zero or more than 64 bodies by panicking
/// during validation.
///
/// # Examples
///
/// ```
/// use bevy_ragdoll_benches::support::chain_profile;
///
/// let profile = chain_profile(8, 42);
/// assert_eq!(profile.bodies().len(), 8);
/// ```
pub fn chain_profile(body_count: usize, seed: u64) -> RagdollProfile {
    RagdollProfile::new(chain_spec(body_count, seed)).expect("seeded chain profile validates")
}

/// Builds seeded authoring data for a parent-first chain with zero through 64
/// bodies.
///
/// The returned `ProfileSpec` preserves the requested body count before profile
/// validation. Values
/// above 64 panic because the profile builder enforces its fixed body limit
/// during generation.
///
/// # Examples
///
/// ```
/// use bevy_ragdoll_benches::support::chain_spec;
///
/// let spec = chain_spec(8, 42);
/// assert_eq!(spec.bodies.len(), 8);
/// ```
pub fn chain_spec(body_count: usize, seed: u64) -> ProfileSpec {
    // Seed one generator so equal arguments produce equal geometry in each benchmark process.
    let mut random = ChaCha8Rng::seed_from_u64(seed);
    let mut builder = ProfileBuilder::default();
    let mut indexes = Vec::with_capacity(body_count);
    let bend = AngleRange {
        min: -0.5,
        max: 0.5,
    };
    let limits = JointLimits {
        x: bend,
        twist: bend,
        z: bend,
    };
    // Add bodies in parent-first order because profile indexes refer to insertion order.
    for index in 0..body_count {
        // Draw dimensions from the seeded stream so body shapes remain stable between runs.
        let radius = 0.08 + next_unit(&mut random).abs() * 0.02;
        let height = 0.35 + next_unit(&mut random).abs() * 0.15;
        let rest_height = height * index as f32;
        let body_index = builder
            .add_body(
                format!("chain_{index}"),
                ShapeSpec::Capsule {
                    a: Vec3::ZERO,
                    b: Vec3::Y * height,
                    radius,
                },
                1.0,
                Isometry3d::from_translation(Vec3::Y * rest_height),
            )
            .expect("64-body profile limit accepts benchmark chains");
        // Connect each new body to its predecessor, leaving only the first body as the root.
        if let Some(parent) = index.checked_sub(1).and_then(|parent| indexes.get(parent)) {
            builder.add_joint(
                body_index,
                *parent,
                Isometry3d::from_translation(Vec3::Y * (height * 0.5)),
                limits,
                40.0,
            );
        }
        indexes.push(body_index);
    }
    builder.into_spec()
}

/// Builds a headless core app with bound profile skeletons and capture systems.
///
/// The app installs the core plugin and binds `character_count` copies before
/// returning. `seed`
/// controls character positions, and `mode` selects capture or writeback
/// preparation. The method
/// returns an error if profile asset storage or the profile's parent-first
/// topology is invalid.
///
/// # Examples
///
/// ```
/// use bevy_ragdoll_benches::support::{BENCH_SEED, PopulationMode, core_app, human_profile};
///
/// let app = core_app(human_profile(), 1, PopulationMode::Capture, BENCH_SEED)?;
/// assert!(app.world().entities().len() > 1);
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub fn core_app(
    profile: RagdollProfile,
    character_count: usize,
    mode: PopulationMode,
    seed: u64,
) -> Result<App, BenchmarkSetupError> {
    // Install shared resources before binding any profile entities.
    let mut app = empty_app();
    add_population(&mut app, profile, character_count, mode, seed)?;
    // Run binding and capture once so measured updates start from a prepared app.
    app.update();
    app.update();
    if matches!(mode, PopulationMode::Limp) {
        prepare_writeback_poses(app.world_mut(), seed);
    }
    Ok(app)
}

/// Builds a headless Rapier app with a bound profile population and floor
/// collider.
///
/// The app uses a fixed 60 Hz timestep and binds every requested character
/// before returning.
/// `mode` selects limp, powered, or sleeping bodies, and `seed` controls their
/// initial layout. The
/// method returns an error if profile storage, parent ordering, or body
/// entities fail validation.
///
/// # Examples
///
/// ```
/// use bevy_ragdoll_benches::support::{BENCH_SEED, PopulationMode, human_profile, rapier_app};
///
/// let app = rapier_app(human_profile(), 1, PopulationMode::Limp, BENCH_SEED)?;
/// assert!(app.world().entities().len() > 1);
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub fn rapier_app(
    profile: RagdollProfile,
    character_count: usize,
    mode: PopulationMode,
    seed: u64,
) -> Result<App, BenchmarkSetupError> {
    // Match every simulation and writeback step to a fixed 60 Hz duration.
    let step = Duration::from_nanos(16_666_667);
    let mut app = empty_app();
    app.insert_resource(Time::<Fixed>::from_hz(60.0))
        .insert_resource(TimeUpdateStrategy::ManualDuration(step))
        .insert_resource(TimestepMode::Fixed {
            dt: step.as_secs_f32(),
            substeps: 1,
        })
        .insert_resource(RagdollPhysicsSettings::default());
    // Install the backend before spawning its fixed floor and ragdoll bodies.
    app.add_plugins(
        RapierPhysicsPlugin::<RapierRagdollHooks<'_, '_>>::default().in_fixed_schedule(),
    );
    app.add_plugins(RapierRagdollPlugin);
    app.world_mut().spawn((
        RigidBody::Fixed,
        Collider::cuboid(100.0, 0.1, 100.0),
        Transform::from_xyz(0.0, -0.1, 0.0),
    ));
    add_population(&mut app, profile, character_count, mode, seed)?;
    // Let backend hooks bind every entity before sleeping state is applied.
    app.update();
    app.update();
    if matches!(mode, PopulationMode::Asleep) {
        mark_bodies_asleep(app.world_mut())?;
    }
    Ok(app)
}

/// Creates Bevy's task, time, asset, transform, and ragdoll resources.
fn empty_app() -> App {
    let mut app = App::new();
    // MinimalPlugins supplies app time and scheduling without opening a window.
    app.add_plugins(MinimalPlugins.set(ScheduleRunnerPlugin::run_loop(Duration::ZERO)));
    // Asset and transform plugins satisfy the core ragdoll binding requirements.
    app.add_plugins((AssetPlugin::default(), TransformPlugin));
    app.add_plugins(RagdollPlugin::default());
    app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_nanos(
        16_666_667,
    )));
    app
}

/// Adds one shared profile asset and a deterministic parent-first character
/// population.
fn add_population(
    app: &mut App,
    profile: RagdollProfile,
    character_count: usize,
    mode: PopulationMode,
    seed: u64,
) -> Result<(), BenchmarkSetupError> {
    // Register one asset so every character uses the same validated profile handle.
    let profile_handle = app
        .world_mut()
        .get_resource_mut::<Assets<RagdollProfile>>()
        .ok_or(BenchmarkSetupError::MissingProfileAssetStorage)?
        .add(profile.clone());
    // Validate every joint index and parent order once before spawning any character.
    let parents = profile_parent_indices(
        profile.bodies().len(),
        profile
            .joints()
            .iter()
            .map(|joint| (joint.child().get(), joint.parent().get())),
    )?;
    // Lay out characters on a square grid while keeping zero-character setups valid.
    let columns = grid_columns(character_count);
    let grid_width = columns as f32;
    let mut random = ChaCha8Rng::seed_from_u64(seed);
    // Spawn roots and then bind bones in the same parent-first order used by profile indexes.
    for index in 0..character_count {
        let transform = character_transform(index, columns, grid_width, mode, &mut random);
        let character = spawn_character_root(app, &profile_handle, index, mode, transform);
        spawn_profile_bones(app.world_mut(), character, &profile, &parents)?;
    }
    Ok(())
}

/// Spawns one named character root with mode-specific drive and ragdoll
/// components.
fn spawn_character_root(
    app: &mut App,
    profile_handle: &bevy::asset::Handle<RagdollProfile>,
    index: usize,
    mode: PopulationMode,
    transform: Transform,
) -> Entity {
    // Resolve drive and integration policy from the selected scenario mode.
    let (ragdoll_mode, drive) = character_physics_state(mode);
    // Keep every profile handle shared while giving each root a stable entity name.
    app.world_mut()
        .spawn((
            Name::new(format!("bench human {index}")),
            Ragdoll::new(profile_handle.clone()),
            ragdoll_mode,
            drive,
            transform,
        ))
        .id()
}

/// Converts joint indexes into a checked parent slot for each profile body.
fn profile_parent_indices(
    body_count: usize,
    joints: impl IntoIterator<Item = (usize, usize)>,
) -> Result<Vec<Option<usize>>, BenchmarkSetupError> {
    // Roots have no incoming joint, so each slot begins empty.
    let mut parents = vec![None; body_count];
    // Validate child and parent bounds before checking their required profile order.
    for (child, parent) in joints {
        if let Some(error) = joint_index_error(child, parent, body_count) {
            return Err(error);
        }
        // Refuse duplicate children so one body cannot acquire conflicting parents.
        let parent_slot = parents
            .get_mut(child)
            .ok_or(BenchmarkSetupError::JointChildIndexOutOfRange)?;
        if parent_slot.is_some() {
            return Err(BenchmarkSetupError::DuplicateJointChild);
        }
        *parent_slot = Some(parent);
    }
    Ok(parents)
}

/// Selects the first invalid relationship category for one profile joint.
fn joint_index_error(
    child: usize,
    parent: usize,
    body_count: usize,
) -> Option<BenchmarkSetupError> {
    match (child < body_count, parent < body_count, parent < child) {
        (false, _, _) => Some(BenchmarkSetupError::JointChildIndexOutOfRange),
        (true, false, _) => Some(BenchmarkSetupError::JointParentIndexOutOfRange),
        (true, true, false) => Some(BenchmarkSetupError::ParentNotFirst),
        (true, true, true) => None,
    }
}

/// Returns the ceiling of the square root, with one column for an empty
/// population.
fn grid_columns(character_count: usize) -> usize {
    // Integer square root avoids float rounding when population sizes grow.
    let root = character_count.isqrt();
    let columns = if root.saturating_mul(root) < character_count {
        root + 1
    } else {
        root
    };
    columns.max(1)
}

/// Places one character on the seeded square grid at its mode-specific starting
/// height.
fn character_transform(
    index: usize,
    columns: usize,
    grid_width: f32,
    mode: PopulationMode,
    random: &mut ChaCha8Rng,
) -> Transform {
    // Convert the linear population index into stable row and column coordinates.
    let row = index / columns;
    let column = index % columns;
    // Apply small seeded jitter so large benchmark grids do not start in exact overlap.
    let x = (column as f32 - (grid_width - 1.0) * 0.5) * 1.7 + next_unit(random) * 0.05;
    let z = (row as f32 - (grid_width - 1.0) * 0.5) * 1.7 + next_unit(random) * 0.05;
    let y = if matches!(mode, PopulationMode::Asleep) {
        0.0
    } else {
        2.0 + next_unit(random) * 0.05
    };
    Transform::from_xyz(x, y, z)
}

/// Selects the root components that match one population mode.
fn character_physics_state(mode: PopulationMode) -> (RagdollMode, RagdollDrive) {
    // Only powered scenes add force input to the character's motor and pin channels.
    let drive = if matches!(mode, PopulationMode::Powered) {
        RagdollDrive::new(1.0, 0.5)
    } else {
        RagdollDrive::new(0.0, 0.0)
    };
    // Capture uses kinematic bodies; simulation modes retain dynamic integration.
    let ragdoll_mode = match mode {
        PopulationMode::Capture => RagdollMode::Kinematic,
        PopulationMode::Limp | PopulationMode::Powered | PopulationMode::Asleep => {
            RagdollMode::Dynamic
        }
    };
    (ragdoll_mode, drive)
}

/// Spawns profile bones after checking every parent is already present.
fn spawn_profile_bones(
    world: &mut World,
    character: Entity,
    profile: &RagdollProfile,
    parents: &[Option<usize>],
) -> Result<(), BenchmarkSetupError> {
    let bodies = profile.bodies();
    if parents.len() != bodies.len() {
        return Err(BenchmarkSetupError::ParentMapLengthMismatch);
    }
    // Store each spawned entity at its profile index for later child relationships.
    let mut bones = Vec::<Entity>::with_capacity(bodies.len());
    // Parent-first ordering makes each valid parent entity available before its child.
    for (body_index, (body, parent_index)) in bodies.iter().zip(parents).enumerate() {
        // Build each bone's parent and local transform from validated profile relationships.
        let (owner, transform) = profile_bone_parent_and_transform(
            profile,
            body,
            character,
            &bones,
            body_index,
            *parent_index,
        )?;
        // Insert the parent before later profile bodies can reference this entity.
        let bone = world
            .spawn((Name::new(body.bone().to_owned()), transform, ChildOf(owner)))
            .id();
        bones.push(bone);
    }
    Ok(())
}

/// Computes one bone's checked owner entity and transform in that parent's
/// local space.
fn profile_bone_parent_and_transform(
    profile: &RagdollProfile,
    body: &Body,
    character: Entity,
    bones: &[Entity],
    body_index: usize,
    parent_index: Option<usize>,
) -> Result<(Entity, Transform), BenchmarkSetupError> {
    match parent_index {
        None => {
            // Attach each profile root directly to its character root.
            let transform = Transform::from_translation(body.rest().translation.into())
                .with_rotation(body.rest().rotation);
            Ok((character, transform))
        }
        Some(parent_index) => {
            // Resolve the parent body before checking order to retain precise index errors.
            let parent_body = profile
                .bodies()
                .get(parent_index)
                .ok_or(BenchmarkSetupError::JointParentIndexOutOfRange)?;
            if parent_index >= body_index {
                return Err(BenchmarkSetupError::ParentNotFirst);
            }
            let parent_entity = parent_bone_entity(bones, parent_index)?;
            // Express the child's rest pose in its parent's local coordinate space.
            let local_rotation = parent_body.rest().rotation.inverse() * body.rest().rotation;
            let local_translation = parent_body.rest().rotation.inverse()
                * (body.rest().translation - parent_body.rest().translation);
            let transform =
                Transform::from_translation(local_translation.into()).with_rotation(local_rotation);
            Ok((parent_entity, transform))
        }
    }
}

/// Resolves the parent bone entity for one child profile body.
fn parent_bone_entity(
    bones: &[Entity],
    parent_index: usize,
) -> Result<Entity, BenchmarkSetupError> {
    bones
        .get(parent_index)
        .copied()
        .ok_or(BenchmarkSetupError::ParentBoneNotSpawned)
}

/// Adds Rapier's requested sleeping state to every bound ragdoll body.
fn mark_bodies_asleep(world: &mut World) -> Result<(), BenchmarkSetupError> {
    // Collect entity IDs first so the query borrow ends before component insertion begins.
    let body_entities = {
        let mut query = world.query_filtered::<Entity, With<BodyShape>>();
        query.iter(world).collect::<Vec<_>>()
    };
    let sleeping = Sleeping {
        normalized_linear_threshold: RigidBodyActivation::default_normalized_linear_threshold(),
        angular_threshold: RigidBodyActivation::default_angular_threshold(),
        sleeping: true,
    };
    // Apply the same explicit sleeping thresholds to every measured body.
    body_entities
        .into_iter()
        .try_for_each(|entity| mark_body_asleep(world, entity, sleeping))
}

/// Applies sleeping state to one entity or reports that the queried entity
/// disappeared.
fn mark_body_asleep(
    world: &mut World,
    entity: Entity,
    sleeping: Sleeping,
) -> Result<(), BenchmarkSetupError> {
    let mut body = world.get_entity_mut(entity)?;
    body.insert(sleeping);
    Ok(())
}

/// Seeds distinct previous and current physics poses before measured writeback.
fn prepare_writeback_poses(world: &mut World, seed: u64) {
    let mut random = ChaCha8Rng::seed_from_u64(seed);
    // Snapshot entity IDs before borrowing the world mutably for pose history updates.
    let body_entities = {
        let mut query = world.query_filtered::<Entity, With<BodyShape>>();
        query.iter(world).collect::<Vec<_>>()
    };
    // Keep previous and current transforms distinct by a reproducible small offset.
    for entity in body_entities {
        let Some(transform) = world
            .get::<GlobalTransform>(entity)
            .copied()
            .map(|global| global.compute_transform())
        else {
            continue;
        };
        let current = Isometry3d::new(transform.translation, transform.rotation);
        let offset = Vec3::new(
            next_unit(&mut random) * 0.01,
            next_unit(&mut random) * 0.01,
            next_unit(&mut random) * 0.01,
        );
        let previous = Isometry3d::new(Vec3::from(current.translation) - offset, current.rotation);
        // Only bound bodies with physics history participate in writeback benchmarks.
        if let Some(mut physics_pose) = world.get_mut::<BodyPhysicsPose>(entity) {
            physics_pose.previous = previous;
            physics_pose.current = current;
        }
    }
}

/// Converts one seeded random word into a signed value in the inclusive unit
/// range.
fn next_unit(random: &mut ChaCha8Rng) -> f32 {
    // Discard the low eight bits so conversion uses the full 24-bit mantissa of `f32`.
    let bits = random.next_u32() >> 8;
    let unit = bits as f32 / 16_777_215.0;
    unit * 2.0 - 1.0
}
