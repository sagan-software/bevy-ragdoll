//! One headless or visible stress application and its measurement systems.

use std::error::Error;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use bevy::app::{
    App, AppExit, First, FixedUpdate, Last, PostUpdate, ScheduleRunnerPlugin, Startup, Update,
};
use bevy::asset::{AssetPlugin, Assets};
#[cfg(feature = "visual")]
use bevy::diagnostic::DiagnosticsStore;
use bevy::diagnostic::FrameTimeDiagnosticsPlugin;
use bevy::ecs::schedule::IntoScheduleConfigs;
use bevy::math::{Isometry3d, Vec3};
use bevy::prelude::{
    ChildOf, Commands, Component, Entity, MessageWriter, MinimalPlugins, Name, PluginGroup, Query,
    Res, ResMut, Resource, Time, Transform, With, World,
};
use bevy::time::{Fixed, TimeUpdateStrategy};
use bevy::transform::TransformPlugin;
#[cfg(feature = "visual")]
use bevy_ragdoll::profile::ShapeSpec;
use bevy_ragdoll::profile::{BodyIndex, RagdollProfile};
use bevy_ragdoll::runtime::body::{BodyAtRest, BodyKind, BodyPhysicsPose, BodyShape, BodyVelocity};
use bevy_ragdoll::runtime::budget::RagdollBudget;
use bevy_ragdoll::runtime::components::{Ragdoll, RagdollBodyOf, RagdollDrive, RagdollMode};
use bevy_ragdoll::runtime::hit::{HitProfile, HitSettings, LastHit};
use bevy_ragdoll::runtime::messages::{HitKind, RagdollHit};
use bevy_ragdoll::runtime::sets::{RagdollFixedSystems, RagdollSystems};
use bevy_ragdoll::{RagdollPlugin, runtime::body::JointToParent};
use bevy_ragdoll_rapier3d::{RapierRagdollHooks, RapierRagdollPlugin};
use bevy_rapier3d::plugin::{PhysicsSet, RapierPhysicsPlugin, TimestepMode};
use bevy_rapier3d::prelude::{Collider, RigidBody};
use clap::Parser;

use super::cli::StressCli;
use super::config::{Backend, Creature, DriveMode, RunConfig, Scenario};
use super::metrics::{
    CoreStatistics, FrameStatistics, Percentile, StepStatistics, StressMetrics,
    nearest_rank_percentile,
};
use super::report::{MachineInfo, StressReport};

/// Runs the stress example, sweep, or baseline comparison selected on the command line.
pub fn run_stress() -> Result<(), Box<dyn Error>> {
    #[cfg(not(target_arch = "wasm32"))]
    let cli = StressCli::parse();
    #[cfg(target_arch = "wasm32")]
    let cli = StressCli::default();

    if let Some(sweep) = cli.sweep.as_ref() {
        return super::sweep::run_sweep(&cli, sweep);
    }

    let config = validate_run(&cli)?;
    let profile = crate::load_profile(crate::ExampleKind::Minimal)?;
    let (report_path, temporary_report) = report_path(&cli)?;
    let app_result = run_app(&cli, config, profile, report_path.as_path());
    let result = match app_result {
        Ok(Some(report)) => write_and_compare(&cli, &report),
        Ok(None) => Ok(()),
        Err(error) => Err(error),
    };
    if temporary_report {
        let _ = std::fs::remove_file(report_path);
    }
    result
}

/// Rejects phase-deferred scenarios, unavailable backends, and incompatible output modes.
fn validate_options(cli: &StressCli, config: &RunConfig) -> Result<(), Box<dyn Error>> {
    match config.backend {
        Backend::Rapier3d => {}
        Backend::Avian3d => return Err("avian3d needs Phase 12".into()),
        Backend::Jolt => return Err("jolt needs Phase 13".into()),
    }
    match config.scenario {
        Scenario::Pile
        | Scenario::Grid
        | Scenario::Wave
        | Scenario::Powered
        | Scenario::Shooting => {}
        Scenario::Balance => return Err("balance needs Phase 9".into()),
        Scenario::Mixed => return Err("mixed needs Phase 11".into()),
    }
    if config.mode == DriveMode::Balance {
        return Err("balance mode needs Phase 9".into());
    }
    if cli.deterministic.is_some() {
        return Err("deterministic mode needs Phase 15".into());
    }
    if cli.screenshot.is_some() && config.headless {
        return Err("screenshots require a visible run".into());
    }
    #[cfg(not(feature = "visual"))]
    if !config.headless {
        return Err("visible stress runs require the `visual` feature".into());
    }
    #[cfg(feature = "visual")]
    if !config.headless
        && cli.screenshot.is_some()
        && cli
            .screenshot
            .as_ref()
            .is_some_and(|p| p.as_os_str().is_empty())
    {
        return Err("screenshot path cannot be empty".into());
    }
    if config.creature != Creature::Human {
        return Err("creature must be human until Phase 11".into());
    }
    Ok(())
}

/// Holds the validated scene inputs shared by startup and measurement systems.
#[derive(Clone, Debug, Resource)]
struct RunScene {
    /// Validated TGF profile used by every spawned character.
    profile: RagdollProfile,
    /// Serialized scenario values used by startup and report collection.
    config: RunConfig,
    /// JSON destination written by the app's final schedule.
    report_path: PathBuf,
    /// Optional PNG destination for a visible run.
    #[cfg(feature = "visual")]
    screenshot: Option<PathBuf>,
}

/// Stores one character's row for wave activation timing.
#[derive(Clone, Copy, Debug, Component)]
struct ScenarioCharacter {
    /// Zero-based grid row used to delay wave activation.
    row: u32,
}

/// Fixed simulation interval between deterministic shooting stress hits.
const SHOOTING_INTERVAL: Duration = Duration::from_millis(50);

/// Cached targets and deterministic clock state for the shooting scenario.
#[derive(Debug, Resource)]
struct ShootingState {
    /// Seeded generator used to choose a character and one profile body.
    random: SplitMix64,
    /// Fixed-step time carried toward the next 50 ms firing interval.
    elapsed: Duration,
    /// Profile-ordered body entities grouped by stable character entity order.
    targets: Option<Vec<Vec<Entity>>>,
    /// Number of scheduled rifle hits since the run began.
    scheduled_hits: usize,
    /// Number of valid hit messages written to Bevy's message queue.
    messages_written: usize,
}

impl ShootingState {
    /// Starts a shooting scenario with a fixed seed and no cached body targets.
    fn new(seed: u64) -> Self {
        Self {
            random: SplitMix64::new(seed),
            elapsed: Duration::ZERO,
            targets: None,
            scheduled_hits: 0,
            messages_written: 0,
        }
    }
}

/// Small deterministic pseudo-random generator for reproducible stress inputs.
#[derive(Debug)]
struct SplitMix64 {
    /// Current state advanced by the generator's fixed arithmetic sequence.
    state: u64,
}

impl SplitMix64 {
    /// Initializes the generator without reserving a special seed value.
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    /// Produces one deterministic 64-bit sample.
    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut value = self.state;
        value = (value ^ (value >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        value = (value ^ (value >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        value ^ (value >> 31)
    }

    /// Chooses an index below `upper_bound`, returning `None` for an empty range.
    fn index(&mut self, upper_bound: usize) -> Option<usize> {
        let upper_bound = u64::try_from(upper_bound).ok()?;
        if upper_bound == 0 {
            return None;
        }
        usize::try_from(self.next_u64() % upper_bound).ok()
    }
}

/// One profile-ordered body row copied while the shooting cache is built.
#[derive(Clone, Copy, Debug)]
struct ShootingBodyRecord {
    /// Character root that owns the physics body.
    character: Entity,
    /// Validated profile index used to stabilize body selection order.
    index: BodyIndex,
    /// Physics entity that receives a selected hit.
    body: Entity,
}

/// Groups a complete body snapshot by character and profile body index.
fn group_shooting_targets(
    mut characters: Vec<Entity>,
    mut bodies: Vec<ShootingBodyRecord>,
    body_count: usize,
) -> Option<Vec<Vec<Entity>>> {
    if characters.is_empty() || body_count == 0 {
        return None;
    }
    let expected_body_count = characters.len().checked_mul(body_count)?;
    if bodies.len() != expected_body_count {
        return None;
    }

    // Sort both dimensions so seeded choices do not depend on ECS query order.
    characters.sort_unstable_by_key(|character| character.to_bits());
    bodies.sort_unstable_by_key(|record| (record.character.to_bits(), record.index.get()));

    // Accept only complete profile-indexed rows for every expected character.
    let mut targets = Vec::with_capacity(characters.len());
    let mut body_offset = 0;
    for character in characters {
        let mut character_bodies = Vec::with_capacity(body_count);
        for index in 0..body_count {
            let record = bodies.get(body_offset)?;
            if record.character != character || record.index.get() != index {
                return None;
            }
            character_bodies.push(record.body);
            body_offset += 1;
        }
        targets.push(character_bodies);
    }
    Some(targets)
}

/// Carries fixed-step fractions and returns the number of due shooting hits.
fn take_due_shots(elapsed: &mut Duration, delta: Duration) -> usize {
    *elapsed = elapsed.saturating_add(delta);
    let mut due = 0;
    while *elapsed >= SHOOTING_INTERVAL {
        *elapsed = elapsed.saturating_sub(SHOOTING_INTERVAL);
        due += 1;
    }
    due
}

/// Associates each named profile bone with its owning character root.
#[cfg(feature = "visual")]
#[derive(Clone, Copy, Debug, Component)]
struct StressBone {
    /// Character whose mode controls procedural idle updates for this bone.
    character: Entity,
}

/// Manual trigger, freeze, restart, and completion state for the current app.
#[derive(Debug, Default, Resource)]
struct RunControl {
    /// Simulation time selected by Space, or the configured trigger time.
    manual_trigger: Option<Duration>,
    /// Time of the first activation trigger in this run.
    triggered_at: Option<Duration>,
    /// Whether all ragdoll modes should be frozen by the F key.
    freeze_all: bool,
    /// Requested child restart from the B or S key.
    #[cfg(feature = "visual")]
    restart: Option<RestartAction>,
    /// Whether the configured measurement window has ended.
    finished: bool,
    /// Whether the final report was written successfully.
    report_written: bool,
    /// Whether report output or screenshot creation failed.
    report_failed: bool,
    /// Whether Bevy has received its application exit message.
    exit_requested: bool,
    /// Simulation time when measurement and playback should end.
    end_at: Option<Duration>,
    /// Simulation time when the requested screenshot observer ran.
    #[cfg(feature = "visual")]
    screenshot_requested_at: Option<Duration>,
    /// Wall-clock time when the screenshot observer was requested.
    #[cfg(feature = "visual")]
    screenshot_started_at: Option<Instant>,
}

/// Key action that restarts the selected stress run in a child process.
#[cfg(feature = "visual")]
#[derive(Clone, Copy, Debug)]
enum RestartAction {
    /// Restart with the next compiled physics backend.
    NextBackend,
    /// Restart with the next Phase 6 scenario.
    NextScenario,
}

/// Wall-clock samples collected around the app and backend schedules.
#[derive(Debug, Default, Resource)]
struct RunMeasurements {
    /// App update wall times in milliseconds after warmup.
    frame_ms: Vec<f64>,
    /// Rapier simulation-set wall times in milliseconds after warmup.
    step_ms: Vec<f64>,
    /// Ragdoll schedule wall times accumulated once per app update.
    core_ms: Vec<f64>,
    /// Current app update's ragdoll schedule duration.
    core_frame: Duration,
    /// Largest app update in the half-second trigger window, in milliseconds.
    trigger_spike_ms: f64,
}

/// Start time shared by the before and after physics-step systems.
#[derive(Debug, Default, Resource)]
struct StepTimer(Option<Instant>);

/// Start time shared by ragdoll core schedule measurement systems.
#[derive(Debug, Default, Resource)]
struct CoreTimer(Option<Instant>);

/// Start time shared by the first and last schedule frame timers.
#[derive(Debug, Default, Resource)]
struct FrameTimer(Option<Instant>);

/// Parsed CLI settings needed to restart a windowed run from its keyboard controls.
#[cfg(feature = "visual")]
#[derive(Clone, Debug, Resource)]
struct RestartInputs {
    /// Typed options preserved when B or S launches the next process.
    cli: StressCli,
}

/// Process-shared flag distinguishing a requested restart from a completed run.
#[cfg(feature = "visual")]
#[derive(Clone, Debug, Resource)]
struct RestartSignal(std::sync::Arc<std::sync::atomic::AtomicBool>);

/// Builds and runs one measured Bevy app, then extracts its completed report.
fn run_app(
    cli: &StressCli,
    config: RunConfig,
    profile: RagdollProfile,
    report_path: &Path,
) -> Result<Option<StressReport>, Box<dyn Error>> {
    #[cfg(not(feature = "visual"))]
    let _ = cli;
    let step = Duration::from_secs_f64(1.0 / f64::from(config.fixed_hz));
    let substeps = usize::try_from(config.substeps.unwrap_or(1))?;
    let mut app = App::new();
    if config.headless {
        app.add_plugins(MinimalPlugins.set(ScheduleRunnerPlugin::run_loop(Duration::ZERO)));
        app.add_plugins((AssetPlugin::default(), TransformPlugin));
    } else {
        #[cfg(feature = "visual")]
        app.add_plugins(visual_plugins());
    }
    app.insert_resource(RunScene {
        profile,
        config: config.clone(),
        report_path: report_path.to_path_buf(),
        #[cfg(feature = "visual")]
        screenshot: cli.screenshot.clone(),
    })
    .insert_resource(Time::<Fixed>::from_hz(f64::from(config.fixed_hz)))
    .insert_resource(TimeUpdateStrategy::ManualDuration(step))
    .insert_resource(TimestepMode::Fixed {
        dt: step.as_secs_f32(),
        substeps,
    })
    .insert_resource(RagdollBudget::new(config.budget.unwrap_or(usize::MAX)))
    .init_resource::<RunControl>()
    .init_resource::<RunMeasurements>()
    .init_resource::<StepTimer>()
    .init_resource::<CoreTimer>()
    .init_resource::<FrameTimer>();
    #[cfg(feature = "visual")]
    app.insert_resource(RestartInputs { cli: cli.clone() });
    #[cfg(feature = "visual")]
    let restart_signal = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    #[cfg(feature = "visual")]
    app.insert_resource(RestartSignal(std::sync::Arc::clone(&restart_signal)));
    app.add_plugins(RagdollPlugin::default());
    app.add_plugins(RapierPhysicsPlugin::<RapierRagdollHooks>::default().in_fixed_schedule());
    app.add_plugins(RapierRagdollPlugin);
    if config.scenario == Scenario::Shooting {
        app.insert_resource(ShootingState::new(config.seed));
        app.add_systems(
            PostUpdate,
            cache_shooting_targets.after(RagdollSystems::Writeback),
        );
        app.add_systems(
            FixedUpdate,
            shoot_ragdolls.before(RagdollFixedSystems::Behaviour),
        );
    }
    app.add_systems(Startup, (spawn_ground, spawn_population));
    app.add_plugins(FrameTimeDiagnosticsPlugin::new(4096));
    app.add_systems(First, start_frame_timer);
    app.add_systems(
        FixedUpdate,
        start_step_timer.before(PhysicsSet::StepSimulation),
    );
    app.add_systems(
        FixedUpdate,
        finish_step_timer.after(PhysicsSet::StepSimulation),
    );
    app.add_systems(
        FixedUpdate,
        start_core_timer.before(RagdollFixedSystems::Drive),
    );
    app.add_systems(
        FixedUpdate,
        finish_core_timer.after(RagdollFixedSystems::AfterStep),
    );
    app.add_systems(
        bevy::app::PostUpdate,
        start_core_timer
            .before(RagdollSystems::Bind)
            .in_set(StressCoreSchedule::VariableStart),
    );
    app.add_systems(
        bevy::app::PostUpdate,
        finish_core_timer
            .after(RagdollSystems::Writeback)
            .in_set(StressCoreSchedule::VariableFinish),
    );
    app.configure_sets(
        bevy::app::PostUpdate,
        (
            StressCoreSchedule::VariableStart,
            StressCoreSchedule::VariableFinish,
        )
            .chain(),
    );
    #[cfg(feature = "visual")]
    if !config.headless {
        install_visual_systems(&mut app);
        app.add_systems(
            Update,
            (
                handle_keyboard,
                update_population_state,
                restart_from_keyboard,
                procedural_idle,
                update_overlay,
            )
                .chain(),
        );
    } else {
        app.add_systems(Update, update_population_state);
    }
    #[cfg(not(feature = "visual"))]
    app.add_systems(Update, update_population_state);
    #[cfg(feature = "visual")]
    app.add_systems(
        Last,
        (request_screenshot, finish_frame_sample, finalize_report).chain(),
    );
    #[cfg(not(feature = "visual"))]
    app.add_systems(Last, (finish_frame_sample, finalize_report).chain());

    let exit = app.run();
    #[cfg(feature = "visual")]
    if restart_signal.load(std::sync::atomic::Ordering::Acquire) {
        return Ok(None);
    }
    if !matches!(exit, AppExit::Success) {
        return Err("stress application exited with an error".into());
    }
    #[cfg(feature = "visual")]
    if let Some(path) = cli.screenshot.as_ref()
        && !path.is_file()
    {
        return Err(format!("screenshot was not written to {}", path.display()).into());
    }
    read_reports(report_path)?
        .into_iter()
        .next()
        .map(Some)
        .ok_or_else(|| "stress application did not write a report".into())
}

/// Adds static floor geometry and simple boxes that stress contact generation.
fn spawn_ground(mut commands: Commands) {
    commands.spawn((
        RigidBody::Fixed,
        Collider::cuboid(100.0, 0.1, 100.0),
        Transform::from_xyz(0.0, -0.1, 0.0),
    ));
    commands.spawn((
        RigidBody::Fixed,
        Collider::cuboid(0.6, 0.4, 0.6),
        Transform::from_xyz(-3.0, 0.4, 1.5),
    ));
    commands.spawn((
        RigidBody::Fixed,
        Collider::cuboid(0.6, 0.4, 0.6),
        Transform::from_xyz(3.0, 0.4, -1.5),
    ));
    commands.spawn((
        RigidBody::Fixed,
        Collider::cuboid(1.5, 0.1, 2.5),
        Transform::from_xyz(5.0, 0.35, 0.0).with_rotation(bevy::math::Quat::from_rotation_z(-0.25)),
    ));
}

/// Creates shared-profile character hierarchies and selects each initial body mode.
fn spawn_population(
    mut commands: Commands,
    mut profiles: ResMut<'_, Assets<RagdollProfile>>,
    scene: Res<'_, RunScene>,
) {
    let profile_handle = profiles.add(scene.profile.clone());
    let [columns, _] = scene.config.grid;
    let initial_mode = match scene.config.scenario {
        Scenario::Pile | Scenario::Powered => RagdollMode::Dynamic,
        Scenario::Grid | Scenario::Wave => RagdollMode::Kinematic,
        Scenario::Balance | Scenario::Shooting | Scenario::Mixed => RagdollMode::Animated,
    };
    for index in 0..scene.config.count {
        let column = u32::try_from(index).unwrap_or(u32::MAX) % columns;
        let row = u32::try_from(index).unwrap_or(u32::MAX) / columns;
        let position = character_position(&scene.config, index, column, row);
        let drive = if scene.config.scenario == Scenario::Powered {
            RagdollDrive::new(1.0, 0.5)
        } else {
            RagdollDrive::new(0.0, 0.0)
        };
        let character = commands
            .spawn((
                Name::new(format!("stress human {index}")),
                Ragdoll::new(profile_handle.clone()),
                initial_mode,
                drive,
                ScenarioCharacter { row },
                Transform::from_translation(position),
            ))
            .id();
        spawn_profile_bones(&mut commands, character, &scene.profile);
    }
}

/// Builds the shooting target cache after each character has all profile bodies.
fn cache_shooting_targets(
    scene: Res<'_, RunScene>,
    mut shooting: ResMut<'_, ShootingState>,
    characters: Query<'_, '_, Entity, (With<Ragdoll>, With<ScenarioCharacter>)>,
    bodies: Query<'_, '_, (Entity, &'static RagdollBodyOf, &'static BodyIndex), With<BodyShape>>,
) {
    if scene.config.scenario != Scenario::Shooting || shooting.targets.is_some() {
        return;
    }
    let characters: Vec<Entity> = characters.iter().collect();
    if characters.len() != scene.config.count {
        return;
    }

    // Copy one bounded body snapshot and sort it once before the stress loop.
    let body_records = bodies
        .iter()
        .map(|(body, owner, index)| ShootingBodyRecord {
            character: owner.0,
            index: *index,
            body,
        })
        .collect();
    shooting.targets =
        group_shooting_targets(characters, body_records, scene.profile.bodies().len());
}

/// Writes one deterministic random rifle hit for each due 50 ms interval.
fn shoot_ragdolls(
    scene: Res<'_, RunScene>,
    control: Res<'_, RunControl>,
    time: Res<'_, Time<Fixed>>,
    settings: Res<'_, HitSettings>,
    mut shooting: ResMut<'_, ShootingState>,
    bodies: Query<'_, '_, &'static BodyPhysicsPose, With<BodyShape>>,
    mut hits: MessageWriter<'_, RagdollHit>,
) {
    if scene.config.scenario != Scenario::Shooting
        || control.triggered_at.is_none()
        || control.finished
    {
        return;
    }
    let Some(magnitude) = settings.impulse_magnitude(HitProfile::Rifle) else {
        return;
    };
    let ShootingState {
        random,
        elapsed,
        targets,
        scheduled_hits,
        messages_written,
    } = &mut *shooting;
    let Some(targets) = targets.as_ref() else {
        return;
    };
    if targets.is_empty() {
        return;
    }

    // Preserve fractional fixed-step time and emit every interval after stalls.
    let due = take_due_shots(elapsed, time.delta());
    *scheduled_hits = scheduled_hits.saturating_add(due);
    for _ in 0..due {
        let Some(character_index) = random.index(targets.len()) else {
            return;
        };
        let Some(character_bodies) = targets.get(character_index) else {
            continue;
        };
        let Some(body_index) = random.index(character_bodies.len()) else {
            continue;
        };
        let Some(body) = character_bodies.get(body_index).copied() else {
            continue;
        };
        let Ok(pose) = bodies.get(body) else {
            continue;
        };
        let point = pose.current.translation.into();
        hits.write(RagdollHit {
            body,
            point,
            impulse: bevy::math::Vec3::NEG_Z * magnitude,
            kind: HitKind::Impact,
        });
        *messages_written = messages_written.saturating_add(1);
    }
}

/// Counts characters whose fixed-step hit processing accepted a shooting event.
fn accepted_shooting_hit_count(world: &mut World) -> usize {
    let mut last_hits = world.query_filtered::<&LastHit, With<Ragdoll>>();
    last_hits
        .iter(world)
        .filter(|last_hit| last_hit.last_at().is_some())
        .count()
}

/// Computes the character root position for the selected pile, grid, or shooting scenario.
fn character_position(config: &RunConfig, index: usize, column: u32, row: u32) -> Vec3 {
    match config.scenario {
        Scenario::Pile => Vec3::new(0.0, 1.5 + index as f32 * 0.8, 0.0),
        Scenario::Grid
        | Scenario::Wave
        | Scenario::Powered
        | Scenario::Balance
        | Scenario::Mixed => {
            let columns = config.grid[0] as f32;
            let rows = config.grid[1] as f32;
            let x = (column as f32 - (columns - 1.0) * 0.5) * config.spacing_m;
            let z = (row as f32 - (rows - 1.0) * 0.5) * config.spacing_m;
            Vec3::new(x, 0.0, z)
        }
        Scenario::Shooting => {
            let count = config.count as f32;
            let angle = std::f32::consts::TAU * index as f32 / count;
            let radius = config.spacing_m * count / std::f32::consts::TAU;
            Vec3::new(radius * angle.cos(), 0.0, radius * angle.sin())
        }
    }
}

/// Builds named profile descendants in parent-first order for runtime binding.
fn spawn_profile_bones(
    commands: &mut Commands<'_, '_>,
    character: Entity,
    profile: &RagdollProfile,
) {
    let bodies = profile.bodies();
    let mut parents = vec![None; bodies.len()];
    for joint in profile.joints() {
        parents[joint.child().get()] = Some(joint.parent().get());
    }
    let mut bones = Vec::<Entity>::with_capacity(bodies.len());
    for (index, body) in bodies.iter().enumerate() {
        let (parent, transform) = parents[index].map_or_else(
            || (character, transform_from_isometry(body.rest())),
            |parent_index| {
                let parent_pose = bodies[parent_index].rest();
                let local_rotation = parent_pose.rotation.inverse() * body.rest().rotation;
                let local_translation = parent_pose.rotation.inverse()
                    * (body.rest().translation - parent_pose.translation);
                (
                    bones[parent_index],
                    Transform::from_translation(local_translation.into())
                        .with_rotation(local_rotation),
                )
            },
        );
        #[cfg(feature = "visual")]
        let bone = commands
            .spawn((
                Name::new(body.bone().to_owned()),
                StressBone { character },
                transform,
                ChildOf(parent),
            ))
            .id();
        #[cfg(not(feature = "visual"))]
        let bone = commands
            .spawn((
                Name::new(body.bone().to_owned()),
                transform,
                ChildOf(parent),
            ))
            .id();
        bones.push(bone);
    }
}

/// Converts a validated rigid profile pose into a Bevy transform.
fn transform_from_isometry(pose: Isometry3d) -> Transform {
    Transform::from_translation(pose.translation.into()).with_rotation(pose.rotation)
}

/// Applies activation timing, drive mode, wave offsets, and total run duration.
fn update_population_state(
    time: Res<'_, Time>,
    scene: Res<'_, RunScene>,
    mut control: ResMut<'_, RunControl>,
    mut time_update: ResMut<'_, TimeUpdateStrategy>,
    mut app_exit: MessageWriter<'_, AppExit>,
    mut characters: Query<'_, '_, (&ScenarioCharacter, &mut RagdollMode, &mut RagdollDrive)>,
) {
    let configured_trigger = match scene.config.scenario {
        Scenario::Pile | Scenario::Powered => Duration::ZERO,
        Scenario::Grid
        | Scenario::Wave
        | Scenario::Balance
        | Scenario::Shooting
        | Scenario::Mixed => Duration::from_secs_f64(scene.config.trigger_at_s),
    };
    let trigger_at = control.manual_trigger.unwrap_or(configured_trigger);
    let wave_tail = if scene.config.scenario == Scenario::Wave {
        Duration::from_secs_f64(f64::from(scene.config.grid[1].saturating_sub(1)) * 0.1)
    } else {
        Duration::ZERO
    };
    let measurement_start =
        Duration::from_secs_f64(scene.config.warmup_s).max(trigger_at.saturating_add(wave_tail));
    let end_at = measurement_start.saturating_add(Duration::from_secs_f64(scene.config.duration_s));
    if time.elapsed() >= trigger_at && control.triggered_at.is_none() {
        control.triggered_at = Some(trigger_at);
    }
    for (character, mut mode, mut drive) in &mut characters {
        let row_trigger = trigger_at.saturating_add(Duration::from_secs_f64(
            if scene.config.scenario == Scenario::Wave {
                f64::from(character.row) * 0.1
            } else {
                0.0
            },
        ));
        let active = matches!(scene.config.scenario, Scenario::Pile | Scenario::Powered)
            || time.elapsed() >= row_trigger;
        let target_mode = if control.freeze_all {
            RagdollMode::Frozen
        } else if active {
            RagdollMode::Dynamic
        } else {
            RagdollMode::Kinematic
        };
        if *mode != target_mode {
            *mode = target_mode;
        }
        if target_mode == RagdollMode::Dynamic {
            let (muscle, pin) = if scene.config.scenario == Scenario::Powered
                || scene.config.mode == DriveMode::Powered
            {
                (1.0, 0.5)
            } else {
                (0.0, 0.0)
            };
            drive.set(muscle, pin);
        }
    }
    control.end_at = Some(end_at);
    control.finished = time.elapsed() >= end_at;
    if control.finished {
        *time_update = TimeUpdateStrategy::ManualDuration(Duration::ZERO);
        #[cfg(feature = "visual")]
        if let Some(started) = control.screenshot_started_at
            && started.elapsed() >= Duration::from_secs(30)
            && scene
                .screenshot
                .as_ref()
                .is_some_and(|path| !path.is_file())
        {
            control.report_failed = true;
        }
        #[cfg(feature = "visual")]
        let screenshot_complete = scene
            .screenshot
            .as_ref()
            .is_none_or(|path| control.screenshot_requested_at.is_some() && path.is_file());
        #[cfg(not(feature = "visual"))]
        let screenshot_complete = true;
        if !control.exit_requested
            && (control.report_written || control.report_failed)
            && (screenshot_complete || control.report_failed)
        {
            app_exit.write(AppExit::Success);
            control.exit_requested = true;
        }
    }
}

/// Records the start of a Rapier fixed simulation step.
fn start_step_timer(mut timer: ResMut<'_, StepTimer>) {
    timer.0 = Some(Instant::now());
}

/// Records one Rapier fixed simulation-step duration when measurement is active.
fn finish_step_timer(
    time: Res<'_, Time>,
    scene: Res<'_, RunScene>,
    control: Res<'_, RunControl>,
    mut timer: ResMut<'_, StepTimer>,
    mut measurements: ResMut<'_, RunMeasurements>,
) {
    let Some(started) = timer.0.take() else {
        return;
    };
    if time.elapsed().as_secs_f64() >= scene.config.warmup_s
        && control
            .end_at
            .is_some_and(|end_at| time.elapsed() <= end_at)
    {
        measurements
            .step_ms
            .push(started.elapsed().as_secs_f64() * 1000.0);
    }
}

/// Records the start of a core schedule interval.
fn start_core_timer(mut timer: ResMut<'_, CoreTimer>) {
    timer.0 = Some(Instant::now());
}

/// Adds one core schedule interval to the current app frame.
fn finish_core_timer(
    mut timer: ResMut<'_, CoreTimer>,
    mut measurements: ResMut<'_, RunMeasurements>,
) {
    if let Some(started) = timer.0.take() {
        measurements.core_frame = measurements.core_frame.saturating_add(started.elapsed());
    }
}

/// Separates the variable-rate and fixed-rate core timer system pairs.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, bevy::prelude::SystemSet)]
enum StressCoreSchedule {
    /// Begins measuring the variable-rate ragdoll sets.
    VariableStart,
    /// Ends measuring the variable-rate ragdoll sets.
    VariableFinish,
}

/// Starts wall-clock measurement for one complete app update.
fn start_frame_timer(mut timer: ResMut<'_, FrameTimer>) {
    timer.0 = Some(Instant::now());
}

/// Records frame, core, and trigger-spike samples after one complete app update.
fn finish_frame_sample(
    time: Res<'_, Time>,
    scene: Res<'_, RunScene>,
    control: Res<'_, RunControl>,
    mut timer: ResMut<'_, FrameTimer>,
    mut measurements: ResMut<'_, RunMeasurements>,
    #[cfg(feature = "visual")] diagnostics: Res<'_, DiagnosticsStore>,
) {
    let Some(started) = timer.0.take() else {
        return;
    };
    let elapsed = time.elapsed();
    let wall_frame_ms = started.elapsed().as_secs_f64() * 1000.0;
    #[cfg(feature = "visual")]
    let frame_ms = if scene.config.headless {
        wall_frame_ms
    } else {
        diagnostics
            .get_measurement(&FrameTimeDiagnosticsPlugin::FRAME_TIME)
            .map(|measurement| measurement.value)
            .unwrap_or(wall_frame_ms)
    };
    #[cfg(not(feature = "visual"))]
    let frame_ms = wall_frame_ms;
    let core_ms = std::mem::take(&mut measurements.core_frame).as_secs_f64() * 1000.0;
    let measuring = !control.report_written
        && elapsed.as_secs_f64() >= scene.config.warmup_s
        && control.end_at.is_some_and(|end_at| elapsed <= end_at);
    if !measuring {
        return;
    }
    measurements.frame_ms.push(frame_ms);
    measurements.core_ms.push(core_ms);
    if control.triggered_at.is_some_and(|trigger| {
        elapsed >= trigger && elapsed <= trigger.saturating_add(Duration::from_millis(500))
    }) {
        measurements.trigger_spike_ms = measurements.trigger_spike_ms.max(frame_ms);
    }
}

/// Writes the final report after the measurement window and screenshot request complete.
fn finalize_report(world: &mut World) {
    let (config, report_path, should_write) = {
        let scene = world.resource::<RunScene>();
        let control = world.resource::<RunControl>();
        (
            scene.config.clone(),
            scene.report_path.clone(),
            control.finished && !control.report_written && !control.report_failed,
        )
    };
    if !should_write {
        return;
    }
    if config.scenario == Scenario::Shooting && accepted_shooting_hit_count(world) == 0 {
        let shooting = world.resource::<ShootingState>();
        let cached_characters = shooting.targets.as_ref().map_or(0, Vec::len);
        let scheduled_hits = shooting.scheduled_hits;
        let messages_written = shooting.messages_written;
        bevy::log::error!(
            cached_characters,
            scheduled_hits,
            messages_written,
            "shooting completed without an accepted rifle hit"
        );
        world.resource_mut::<RunControl>().report_failed = true;
        return;
    }
    let gpu = render_adapter_name(world);
    let metrics = collect_metrics(world);
    let report = StressReport::new(source_revision(), machine_info(gpu), config, metrics);
    match write_json(&report_path, &report) {
        Ok(()) => world.resource_mut::<RunControl>().report_written = true,
        Err(error) => {
            bevy::log::error!(report_error = %error, "could not write stress report");
            world.resource_mut::<RunControl>().report_failed = true;
        }
    }
}

/// Counts final ragdoll state and summarizes all timing samples.
fn collect_metrics(world: &mut bevy::prelude::World) -> StressMetrics {
    let measurements = world.resource::<RunMeasurements>();
    let frame_ms = FrameStatistics {
        p50: nearest_rank_percentile(&measurements.frame_ms, Percentile::P50).unwrap_or(0.0),
        p95: nearest_rank_percentile(&measurements.frame_ms, Percentile::P95).unwrap_or(0.0),
        p99: nearest_rank_percentile(&measurements.frame_ms, Percentile::P99).unwrap_or(0.0),
        max: measurements.frame_ms.iter().copied().fold(0.0, f64::max),
    };
    let step_ms = StepStatistics {
        p50: nearest_rank_percentile(&measurements.step_ms, Percentile::P50).unwrap_or(0.0),
        p95: nearest_rank_percentile(&measurements.step_ms, Percentile::P95).unwrap_or(0.0),
        max: measurements.step_ms.iter().copied().fold(0.0, f64::max),
    };
    let core_ms = CoreStatistics {
        p50: nearest_rank_percentile(&measurements.core_ms, Percentile::P50).unwrap_or(0.0),
        p95: nearest_rank_percentile(&measurements.core_ms, Percentile::P95).unwrap_or(0.0),
    };
    let trigger_spike_ms = measurements.trigger_spike_ms;
    let mut characters = world.query_filtered::<Entity, bevy::prelude::With<Ragdoll>>();
    let character_count = characters.iter(world).count();
    let mut bodies = world.query_filtered::<Entity, bevy::prelude::With<BodyShape>>();
    let body_count = bodies.iter(world).count();
    let mut joints = world.query_filtered::<Entity, bevy::prelude::With<JointToParent>>();
    let joint_count = joints.iter(world).count();
    let mut body_state = world.query_filtered::<(
        &BodyKind,
        Option<&BodyAtRest>,
        &BodyVelocity,
        &BodyPhysicsPose,
    ), bevy::prelude::With<BodyShape>>();
    let mut dynamic = 0;
    let mut sleeping = 0;
    let mut frozen = 0;
    let mut unstable = 0;
    for (kind, at_rest, velocity, pose) in body_state.iter(world) {
        match kind {
            BodyKind::Dynamic => dynamic += 1,
            BodyKind::Kinematic => {}
            BodyKind::Fixed => frozen += 1,
        }
        if at_rest.is_some() {
            sleeping += 1;
        }
        if !velocity.linear.is_finite()
            || velocity.linear.length() > 50.0
            || !pose.current.translation.is_finite()
            || !pose.current.rotation.is_finite()
        {
            unstable += 1;
        }
    }
    StressMetrics {
        frame_ms,
        step_ms,
        core_ms,
        trigger_spike_ms,
        characters: character_count,
        bodies: body_count,
        joints: joint_count,
        dynamic,
        sleeping_end: sleeping,
        frozen_end: frozen,
        peak_rss_mb: peak_rss_megabytes(),
        unstable_bodies: unstable,
    }
}

/// Reads Linux's process high-water RSS and converts kibibytes to mebibytes.
fn peak_rss_megabytes() -> Option<f64> {
    #[cfg(target_os = "linux")]
    {
        let status = std::fs::read_to_string("/proc/self/status").ok()?;
        let kilobytes = status
            .lines()
            .find_map(|line| line.strip_prefix("VmHWM:"))?
            .split_whitespace()
            .next()?
            .parse::<f64>()
            .ok()?;
        Some(kilobytes / 1024.0)
    }
    #[cfg(not(target_os = "linux"))]
    {
        None
    }
}

/// Captures CPU, logical-core, operating-system, and optional adapter details.
fn machine_info(gpu: Option<String>) -> MachineInfo {
    let cpu = std::fs::read_to_string("/proc/cpuinfo")
        .ok()
        .and_then(|input| {
            input.lines().find_map(|line| {
                let (key, value) = line.split_once(':')?;
                matches!(key.trim(), "model name" | "Hardware").then(|| value.trim().to_owned())
            })
        })
        .unwrap_or_else(|| std::env::consts::ARCH.to_owned());
    let cores = std::thread::available_parallelism()
        .map(std::num::NonZeroUsize::get)
        .unwrap_or(1);
    MachineInfo {
        cpu,
        cores,
        os: std::env::consts::OS.to_owned(),
        gpu,
    }
}

/// Returns the short checked-out source revision for reproducible reports.
pub(super) fn source_revision() -> String {
    std::env::var("BEVY_RAGDOLL_GIT_SHA").unwrap_or_else(|_| {
        Command::new("git")
            .args(["rev-parse", "--short", "HEAD"])
            .output()
            .ok()
            .filter(|output| output.status.success())
            .map(|output| String::from_utf8_lossy(&output.stdout).trim().to_owned())
            .filter(|revision| !revision.is_empty())
            .unwrap_or_else(|| "unknown".to_owned())
    })
}

/// Writes a report and checks candidate timings against one report or report array.
fn write_and_compare(cli: &StressCli, report: &StressReport) -> Result<(), Box<dyn Error>> {
    if cli.report.is_none() {
        let report_json = serde_json::to_string_pretty(report)?;
        println!("{report_json}");
    }
    if let Some(path) = cli.compare.as_ref() {
        let baseline = read_reports(path)?;
        let failures = super::compare::compare_reports(std::slice::from_ref(report), &baseline);
        if !failures.is_empty() {
            for failure in failures {
                eprintln!("{failure}");
            }
            return Err("stress report exceeded its baseline comparison".into());
        }
    }
    Ok(())
}

/// Serializes one complete report to its requested JSON path.
pub(super) fn write_json(
    path: &std::path::Path,
    report: &StressReport,
) -> Result<(), Box<dyn Error>> {
    if let Some(parent) = path.parent()
        && !parent.as_os_str().is_empty()
    {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(path, serde_json::to_vec_pretty(report)?)?;
    Ok(())
}

/// Reads either one baseline report or a sweep report array.
pub(super) fn read_reports(path: &std::path::Path) -> Result<Vec<StressReport>, Box<dyn Error>> {
    let bytes = std::fs::read(path)?;
    let value = serde_json::from_slice::<serde_json::Value>(&bytes)?;
    if value.is_array() {
        Ok(serde_json::from_value(value)?)
    } else {
        Ok(vec![serde_json::from_value(value)?])
    }
}

/// Returns a parse or run error when the selected options are not implemented yet.
pub(super) fn validate_run(cli: &StressCli) -> Result<RunConfig, Box<dyn Error>> {
    let config = cli.run_config()?;
    validate_options(cli, &config)?;
    Ok(config)
}

/// Chooses the requested report destination or a target-local temporary JSON file.
fn report_path(cli: &StressCli) -> Result<(PathBuf, bool), Box<dyn Error>> {
    if let Some(path) = cli.report.as_ref() {
        return Ok((path.clone(), false));
    }
    let target = std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("target"));
    let directory = target.join("stress");
    std::fs::create_dir_all(&directory)?;
    let nonce = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|duration| duration.as_nanos())
        .unwrap_or_default();
    let process_id = std::process::id();
    Ok((
        directory.join(format!(".run-{process_id}-{nonce}.json")),
        true,
    ))
}

/// Spawns the next backend or scenario after B or S is pressed.
#[cfg(feature = "visual")]
fn restart_from_keyboard(
    scene: Res<'_, RunScene>,
    inputs: Res<'_, RestartInputs>,
    signal: Res<'_, RestartSignal>,
    mut control: ResMut<'_, RunControl>,
    mut app_exit: MessageWriter<'_, AppExit>,
) {
    let Some(action) = control.restart.take() else {
        return;
    };
    let backends = [Backend::Rapier3d];
    let scenarios = [
        Scenario::Pile,
        Scenario::Grid,
        Scenario::Wave,
        Scenario::Powered,
    ];
    let backend = scene.config.backend;
    let scenario = scene.config.scenario;
    let next_backend = match action {
        RestartAction::NextBackend => {
            let index = backends
                .iter()
                .position(|entry| *entry == backend)
                .unwrap_or(0);
            backends[(index + 1) % backends.len()]
        }
        RestartAction::NextScenario => backend,
    };
    let next_scenario = match action {
        RestartAction::NextBackend => scenario,
        RestartAction::NextScenario => {
            let index = scenarios
                .iter()
                .position(|entry| *entry == scenario)
                .unwrap_or(0);
            scenarios[(index + 1) % scenarios.len()]
        }
    };
    let executable = std::env::current_exe();
    let status = executable.and_then(|path| {
        Command::new(path)
            .args(inputs.cli.restart_arguments(next_backend, next_scenario))
            .spawn()
    });
    match status {
        Ok(_) => {
            signal.0.store(true, std::sync::atomic::Ordering::Release);
            app_exit.write(AppExit::Success);
        }
        Err(error) => {
            bevy::log::error!(restart_error = %error, "could not start the requested stress run")
        }
    }
}

#[cfg(feature = "visual")]
/// Returns an installed visual app group configured for a visible stress window.
fn visual_plugins() -> impl bevy::prelude::PluginGroup {
    use bevy::prelude::DefaultPlugins;
    use bevy::window::{PresentMode, Window, WindowPlugin};

    DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "bevy-ragdoll stress".to_owned(),
            resolution: (1600, 1000).into(),
            present_mode: PresentMode::AutoNoVsync,
            ..Default::default()
        }),
        ..Default::default()
    })
}

#[cfg(feature = "visual")]
/// Returns the active Bevy render adapter name for a visible run.
fn render_adapter_name(world: &World) -> Option<String> {
    world
        .get_resource::<bevy::render::renderer::RenderAdapterInfo>()
        .map(|info| info.name.clone())
}

#[cfg(not(feature = "visual"))]
/// Headless builds have no render adapter.
fn render_adapter_name(_: &World) -> Option<String> {
    None
}

#[cfg(feature = "visual")]
/// Mesh cache keys that share one rendered mesh for each validated shape size.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum MeshKey {
    /// Capsule radius and segment length as exact IEEE-754 bit patterns.
    Capsule {
        /// Capsule radius in metres.
        radius_bits: u32,
        /// Capsule cylinder length in metres.
        length_bits: u32,
    },
    /// Sphere radius as an exact IEEE-754 bit pattern.
    Sphere {
        /// Sphere radius in metres.
        radius_bits: u32,
    },
    /// Cuboid half extents as exact IEEE-754 bit patterns.
    Cuboid {
        /// X half extent in metres.
        x_bits: u32,
        /// Y half extent in metres.
        y_bits: u32,
        /// Z half extent in metres.
        z_bits: u32,
    },
}

#[cfg(feature = "visual")]
/// Shared rendered meshes and one material for every ragdoll body.
#[derive(Debug, Resource)]
struct SharedBodyAssets {
    /// One mesh handle for each distinct capsule, sphere, or cuboid size.
    meshes: std::collections::HashMap<MeshKey, bevy::asset::Handle<bevy::prelude::Mesh>>,
    /// Material shared by all ragdoll body render entities.
    material: bevy::asset::Handle<bevy::prelude::StandardMaterial>,
}

#[cfg(feature = "visual")]
/// Marker for the corner overlay's text entity.
#[derive(Clone, Copy, Debug, Component)]
struct StressOverlay;

#[cfg(feature = "visual")]
/// Adds the camera, light, ground render, shared body material, and HUD.
fn setup_visual_scene(
    mut commands: Commands<'_, '_>,
    scene: Res<'_, RunScene>,
    mut meshes: ResMut<'_, Assets<bevy::prelude::Mesh>>,
    mut materials: ResMut<'_, Assets<bevy::prelude::StandardMaterial>>,
) {
    use bevy::prelude::{
        Camera3d, Color, DirectionalLight, FontSize, Mesh3d, MeshMaterial3d, Meshable, Node,
        Plane3d, PositionType, Text, TextColor, TextFont, px,
    };

    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 22.0, 30.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 14_000.0,
            shadow_maps_enabled: true,
            ..Default::default()
        },
        Transform::from_xyz(-8.0, 16.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    let material = materials.add(bevy::prelude::StandardMaterial {
        base_color: Color::srgb(0.18, 0.62, 0.76),
        metallic: 0.02,
        perceptual_roughness: 0.42,
        ..Default::default()
    });
    commands.insert_resource(SharedBodyAssets {
        meshes: std::collections::HashMap::new(),
        material,
    });
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(200.0, 200.0))),
        MeshMaterial3d(materials.add(bevy::prelude::StandardMaterial {
            base_color: Color::srgb(0.11, 0.15, 0.19),
            perceptual_roughness: 0.92,
            ..Default::default()
        })),
        Transform::from_xyz(0.0, -0.02, 0.0),
    ));
    let controls = "Space trigger  F freeze  B backend  S scenario";
    let scenario = scene.config.scenario;
    let title = format!("{scenario} · {controls}");
    commands.spawn((
        Text::new(title),
        TextFont {
            font_size: FontSize::Px(20.0),
            ..Default::default()
        },
        TextColor(Color::WHITE),
        StressOverlay,
        Node {
            position_type: PositionType::Absolute,
            top: px(18),
            left: px(20),
            ..Default::default()
        },
    ));
}

#[cfg(feature = "visual")]
/// Attaches a cached mesh and shared material to each newly created physics body.
fn create_body_visuals(
    mut commands: Commands<'_, '_>,
    mut meshes: ResMut<'_, Assets<bevy::prelude::Mesh>>,
    mut cache: ResMut<'_, SharedBodyAssets>,
    bodies: Query<'_, '_, (Entity, &BodyShape), bevy::prelude::Added<BodyShape>>,
) {
    use bevy::prelude::{ChildOf, Mesh3d, MeshMaterial3d, Visibility};

    for (body, shape) in &bodies {
        // Give the physics parent the visibility state inherited by its mesh child.
        commands.entity(body).insert(Visibility::Inherited);
        let (key, transform) = shape_mesh_key(&shape.0);
        let mesh_handle = cache.meshes.entry(key).or_insert_with(|| {
            let mesh = mesh_from_key(key);
            meshes.add(mesh)
        });
        let mesh = mesh_handle.clone();
        let material = cache.material.clone();
        commands.spawn((
            Mesh3d(mesh),
            MeshMaterial3d(material),
            Visibility::Inherited,
            transform,
            ChildOf(body),
        ));
    }
}

#[cfg(feature = "visual")]
/// Converts a validated shape into a shared mesh key and its body-local pose.
fn shape_mesh_key(shape: &ShapeSpec) -> (MeshKey, Transform) {
    match shape {
        ShapeSpec::Capsule { a, b, radius } => {
            let segment = *b - *a;
            let orientation = if segment.length_squared() > f32::EPSILON {
                bevy::math::Quat::from_rotation_arc(Vec3::Y, segment.normalize())
            } else {
                bevy::math::Quat::IDENTITY
            };
            (
                MeshKey::Capsule {
                    radius_bits: radius.to_bits(),
                    length_bits: segment.length().to_bits(),
                },
                Transform::from_translation((*a + *b) * 0.5).with_rotation(orientation),
            )
        }
        ShapeSpec::Sphere { center, radius } => (
            MeshKey::Sphere {
                radius_bits: radius.to_bits(),
            },
            Transform::from_translation(*center),
        ),
        ShapeSpec::Cuboid {
            center,
            rotation,
            half_extents,
        } => (
            MeshKey::Cuboid {
                x_bits: half_extents.x.to_bits(),
                y_bits: half_extents.y.to_bits(),
                z_bits: half_extents.z.to_bits(),
            },
            Transform::from_translation(*center).with_rotation(*rotation),
        ),
    }
}

#[cfg(feature = "visual")]
/// Creates one Bevy mesh for each exact validated mesh cache key.
fn mesh_from_key(key: MeshKey) -> bevy::prelude::Mesh {
    use bevy::prelude::{Capsule3d, Cuboid, Mesh, Sphere};

    match key {
        MeshKey::Capsule {
            radius_bits,
            length_bits,
        } => Mesh::from(Capsule3d::new(
            f32::from_bits(radius_bits),
            f32::from_bits(length_bits),
        )),
        MeshKey::Sphere { radius_bits } => Mesh::from(Sphere::new(f32::from_bits(radius_bits))),
        MeshKey::Cuboid {
            x_bits,
            y_bits,
            z_bits,
        } => Mesh::from(Cuboid::from_size(Vec3::new(
            f32::from_bits(x_bits) * 2.0,
            f32::from_bits(y_bits) * 2.0,
            f32::from_bits(z_bits) * 2.0,
        ))),
    }
}

#[cfg(feature = "visual")]
/// Updates standing spine bones with a 0.25 Hz, 0.05 radian idle sway.
fn procedural_idle(
    time: Res<'_, Time>,
    character_modes: Query<'_, '_, &RagdollMode>,
    mut bones: Query<'_, '_, (&Name, &StressBone, &mut Transform)>,
) {
    let phase = std::f32::consts::TAU * 0.25 * time.elapsed_secs();
    for (name, owner, mut transform) in &mut bones {
        if character_modes.get(owner.character) != Ok(&RagdollMode::Kinematic) {
            continue;
        }
        if name.as_str() != "spine_04" {
            continue;
        }
        let amplitude = 0.05;
        transform.rotation = bevy::math::Quat::from_rotation_z(amplitude * phase.sin());
    }
}

#[cfg(feature = "visual")]
/// Handles manual activation, global freeze, backend cycling, and scenario cycling.
fn handle_keyboard(
    keys: Res<'_, bevy::prelude::ButtonInput<bevy::prelude::KeyCode>>,
    time: Res<'_, Time>,
    mut control: ResMut<'_, RunControl>,
) {
    if keys.just_pressed(bevy::prelude::KeyCode::Space) {
        control.manual_trigger = Some(time.elapsed());
    }
    if keys.just_pressed(bevy::prelude::KeyCode::KeyF) {
        control.freeze_all = !control.freeze_all;
    }
    if keys.just_pressed(bevy::prelude::KeyCode::KeyB) {
        control.restart = Some(RestartAction::NextBackend);
    }
    if keys.just_pressed(bevy::prelude::KeyCode::KeyS) {
        control.restart = Some(RestartAction::NextScenario);
    }
}

#[cfg(feature = "visual")]
/// Refreshes visible stress counts and frame p95 context twice per simulated second.
fn update_overlay(
    time: Res<'_, Time>,
    scene: Res<'_, RunScene>,
    control: Res<'_, RunControl>,
    characters: Query<'_, '_, &RagdollMode, bevy::prelude::With<ScenarioCharacter>>,
    bodies: Query<'_, '_, (), bevy::prelude::With<BodyShape>>,
    mut text: Query<'_, '_, &mut bevy::prelude::Text, bevy::prelude::With<StressOverlay>>,
    mut last_update: bevy::prelude::Local<'_, f64>,
) {
    let elapsed = f64::from(time.elapsed_secs());
    if elapsed - *last_update < 0.5 {
        return;
    }
    *last_update = elapsed;
    let active = characters
        .iter()
        .filter(|mode| **mode == RagdollMode::Dynamic)
        .count();
    let body_count = bodies.iter().count();
    let trigger_state = if control.triggered_at.is_some() {
        "triggered"
    } else {
        "standing"
    };
    let scenario = scene.config.scenario;
    let mut status = text.single_mut().expect("stress overlay exists");
    status.0 = format!(
        "{scenario} · {active} dynamic · {body_count} bodies · {trigger_state} · Space trigger  F freeze  B backend  S scenario"
    );
}

#[cfg(feature = "visual")]
/// Requests the final window screenshot after the measurement interval completes.
fn request_screenshot(
    mut commands: Commands<'_, '_>,
    scene: Res<'_, RunScene>,
    time: Res<'_, Time>,
    mut control: ResMut<'_, RunControl>,
) {
    use bevy::render::view::screenshot::{Screenshot, save_to_disk};

    let Some(path) = scene.screenshot.as_ref() else {
        return;
    };
    if !control.finished || control.screenshot_requested_at.is_some() {
        return;
    }
    if let Some(parent) = path.parent()
        && !parent.as_os_str().is_empty()
        && let Err(error) = std::fs::create_dir_all(parent)
    {
        bevy::log::error!(screenshot_directory_error = %error, "could not create screenshot directory");
        control.report_failed = true;
        return;
    }
    // Remove a prior output so file existence confirms this request completed.
    if path.exists()
        && let Err(error) = std::fs::remove_file(path)
    {
        bevy::log::error!(screenshot_remove_error = %error, "could not replace the existing screenshot");
        control.report_failed = true;
        return;
    }
    commands
        .spawn(Screenshot::primary_window())
        .observe(save_to_disk(path.clone()));
    control.screenshot_requested_at = Some(time.elapsed());
    control.screenshot_started_at = Some(Instant::now());
}

#[cfg(feature = "visual")]
/// Adds the visible scene, shared rig material, HUD, controls, and screenshot observer.
fn install_visual_systems(app: &mut App) {
    use bevy::app::{PostUpdate, Startup};
    use bevy::prelude::IntoScheduleConfigs;
    use bevy::transform::TransformSystems;

    app.insert_resource(bevy::winit::WinitSettings::continuous());
    app.add_systems(Startup, setup_visual_scene);
    app.add_systems(
        PostUpdate,
        create_body_visuals
            .after(RagdollSystems::Bind)
            .before(TransformSystems::Propagate),
    );
}

#[cfg(test)]
mod tests {
    use super::{
        Scenario, ShootingBodyRecord, SplitMix64, StressCli, character_position,
        group_shooting_targets, take_due_shots, validate_run,
    };
    use bevy::prelude::Entity;
    use bevy_ragdoll::profile::BodyIndex;
    use clap::Parser;
    use std::time::Duration;

    #[test]
    fn phase7_shooting_scenario_is_available() {
        let cli =
            StressCli::try_parse_from(["ragdoll_stress", "--scenario", "shooting", "--headless"])
                .expect("the shooting stress arguments are valid");

        let config = validate_run(&cli).expect("Phase 7 enables shooting stress runs");

        assert_eq!(config.scenario, Scenario::Shooting);
    }

    #[test]
    fn shooting_targets_are_sorted_by_character_and_profile_index() {
        let first_character = Entity::from_bits(1);
        let second_character = Entity::from_bits(2);
        let first_body = Entity::from_bits(10);
        let second_body = Entity::from_bits(11);
        let third_body = Entity::from_bits(20);
        let fourth_body = Entity::from_bits(21);
        let bodies = vec![
            ShootingBodyRecord {
                character: second_character,
                index: BodyIndex::try_from(1).expect("profile index one is valid"),
                body: fourth_body,
            },
            ShootingBodyRecord {
                character: first_character,
                index: BodyIndex::try_from(1).expect("profile index one is valid"),
                body: second_body,
            },
            ShootingBodyRecord {
                character: second_character,
                index: BodyIndex::try_from(0).expect("profile index zero is valid"),
                body: third_body,
            },
            ShootingBodyRecord {
                character: first_character,
                index: BodyIndex::try_from(0).expect("profile index zero is valid"),
                body: first_body,
            },
        ];

        let targets = group_shooting_targets(vec![second_character, first_character], bodies, 2)
            .expect("complete character body trees are accepted");

        assert_eq!(
            targets,
            [vec![first_body, second_body], vec![third_body, fourth_body]]
        );
    }

    #[test]
    fn shooting_target_cache_waits_for_a_complete_profile() {
        let character = Entity::from_bits(1);
        let body = Entity::from_bits(10);
        let incomplete = vec![ShootingBodyRecord {
            character,
            index: BodyIndex::try_from(1).expect("profile index one is valid"),
            body,
        }];

        assert!(group_shooting_targets(vec![character], incomplete, 2).is_none());
        assert!(group_shooting_targets(vec![character], Vec::new(), 0).is_none());
        assert!(group_shooting_targets(Vec::new(), Vec::new(), 1).is_none());
    }

    #[test]
    fn shooting_target_cache_rejects_duplicate_profile_indexes() {
        let character = Entity::from_bits(1);
        let bodies = vec![
            ShootingBodyRecord {
                character,
                index: BodyIndex::try_from(0).expect("profile index zero is valid"),
                body: Entity::from_bits(10),
            },
            ShootingBodyRecord {
                character,
                index: BodyIndex::try_from(0).expect("profile index zero is valid"),
                body: Entity::from_bits(11),
            },
        ];

        assert!(group_shooting_targets(vec![character], bodies, 2).is_none());
    }

    #[test]
    fn shooting_clock_carries_fixed_step_fractions_and_catches_up() {
        let mut elapsed = Duration::ZERO;

        assert_eq!(take_due_shots(&mut elapsed, Duration::from_millis(49)), 0);
        assert_eq!(elapsed, Duration::from_millis(49));
        assert_eq!(take_due_shots(&mut elapsed, Duration::from_millis(1)), 1);
        assert_eq!(elapsed, Duration::ZERO);
        assert_eq!(take_due_shots(&mut elapsed, Duration::from_millis(120)), 2);
        assert_eq!(elapsed, Duration::from_millis(20));
    }

    #[test]
    fn shooting_random_indexes_are_repeatable_and_bounded() {
        let mut first = SplitMix64::new(42);
        let mut second = SplitMix64::new(42);

        assert_eq!(first.index(0), None);
        assert_eq!(second.index(0), None);
        assert_eq!(SplitMix64::new(42).index(1), Some(0));
        for _ in 0..256 {
            let first_index = first.index(17);
            let second_index = second.index(17);
            assert_eq!(first_index, second_index);
            assert!(first_index.is_some_and(|index| index < 17));
        }
    }

    #[test]
    fn shooting_characters_are_placed_on_a_spacing_scaled_ring() {
        let cli = StressCli::try_parse_from([
            "ragdoll_stress",
            "--scenario",
            "shooting",
            "--count",
            "4",
            "--spacing",
            "2",
            "--headless",
        ])
        .expect("the shooting stress arguments are valid");
        let config = cli
            .run_config()
            .expect("the shooting configuration is valid");
        let radius = config.spacing_m * config.count as f32 / std::f32::consts::TAU;

        for index in 0..config.count {
            let position = character_position(&config, index, 0, 0);
            assert!((position.length() - radius).abs() < 1.0e-5);
        }
    }
}
