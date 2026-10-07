//! Stress test: drops many ragdolls and reports frame and physics-step cost.
//!
//! Each frame advances exactly one 60 Hz fixed step, so runs are repeatable.
//! After the warmup, the app records wall-clock frame and fixed-step times, and
//! at the end it can write a JSON report.
//!
//! ```sh
//! cargo run --release --example ragdoll_stress -- --scenario grid --count 64
//! cargo run --example ragdoll_stress -- --headless --scenario pile --count 4 \
//!   --duration 2 --warmup 0 --report stress-smoke.json
//! ```
//!
//! The report's `metrics.unstable_bodies` counts bodies that move faster than
//! 50 m/s or have a non-finite pose at the end; CI requires zero.

use std::path::PathBuf;
use std::time::Duration;

use bevy::app::ScheduleRunnerPlugin;
use bevy::platform::time::Instant;
use bevy::prelude::*;
use bevy::time::TimeUpdateStrategy;
use bevy_ragdoll::runtime::body::{BodyPhysicsPose, BodyShape, BodyVelocity};
use bevy_ragdoll::runtime::components::{RagdollBodyOf, RagdollDrive, RagdollMode};
use bevy_ragdoll::runtime::sets::RagdollSystems;
use bevy_ragdoll::{Ragdoll, RagdollPlugin, Skeleton};
use bevy_ragdoll_rapier3d::{RapierRagdollHooks, RapierRagdollPlugin};
use bevy_rapier3d::plugin::{RapierPhysicsPlugin, TimestepMode};
use bevy_rapier3d::prelude::{Collider, RigidBody};
use clap::Parser;
use serde::Serialize;

/// Fixed simulation rate in hertz.
const FIXED_HZ: f64 = 60.0;
/// Body speed in metres per second above which a body counts as unstable.
const UNSTABLE_SPEED: f32 = 50.0;

/// Command-line options.
#[derive(Parser, Clone, Serialize)]
struct Args {
    /// Run without a window or renderer.
    #[cfg_attr(
        dylint_lib = "sagan_lints",
        expect(
            bool_name_prefix,
            reason = "clap derives the --headless flag that CI calls from this field name"
        )
    )]
    #[arg(long)]
    headless: bool,
    /// How the ragdolls are placed.
    #[arg(long, value_enum, default_value_t = Scenario::Grid)]
    scenario: Scenario,
    /// Number of ragdolls.
    #[arg(long, default_value_t = 64)]
    count: u16,
    /// Measured simulated seconds after the warmup.
    #[arg(long, default_value_t = 10.0)]
    duration: f64,
    /// Simulated seconds before measuring starts.
    #[arg(long, default_value_t = 1.0)]
    warmup: f64,
    /// Write a JSON report to this path when the run ends.
    #[arg(long)]
    #[serde(skip)]
    report: Option<PathBuf>,
}

/// Ragdoll placement.
#[derive(clap::ValueEnum, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
enum Scenario {
    /// One tall stack that collapses into a contact-heavy heap.
    Pile,
    /// A square grid of ragdolls 1.5 m apart, dropped side by side.
    Grid,
}

/// The JSON report written by `--report`.
#[derive(Serialize)]
struct Report<'a> {
    /// The options that produced this run.
    config: &'a Args,
    /// What the run measured.
    metrics: Metrics,
}

/// Measurements from the timed part of the run.
#[derive(Serialize)]
struct Metrics {
    /// Wall-clock frame time in milliseconds.
    frame_ms: Summary,
    /// Wall-clock fixed-step time (ragdoll runtime plus physics) in milliseconds.
    step_ms: Summary,
    /// Ragdoll physics bodies at the end.
    bodies: usize,
    /// Bodies faster than 50 m/s or with a non-finite pose at the end.
    unstable_bodies: usize,
}

/// Nearest-rank percentiles and maximum of a set of samples.
#[derive(Serialize)]
struct Summary {
    /// Median.
    p50: f64,
    /// 95th percentile.
    p95: f64,
    /// Largest sample.
    max: f64,
}

impl Summary {
    /// Summarizes the samples; all fields are zero when there are none.
    fn new(mut samples: Vec<f64>) -> Self {
        samples.sort_by(f64::total_cmp);
        // Nearest rank: the smallest sample with at least `percent`% of samples at or below it.
        let rank = |percent: usize| {
            let index = (samples.len() * percent).div_ceil(100).saturating_sub(1);
            samples.get(index).copied().unwrap_or_default()
        };
        Self {
            p50: rank(50),
            p95: rank(95),
            max: samples.last().copied().unwrap_or_default(),
        }
    }
}

/// Run options and samples collected so far.
#[cfg_attr(
    dylint_lib = "sagan_lints",
    expect(
        bevy_missing_reflect,
        reason = "the clap Args and Instant fields cannot implement Reflect, and nothing inspects this resource"
    )
)]
#[derive(Resource)]
struct Run {
    /// Command-line options.
    args: Args,
    /// The skeleton every character uses; each `Ragdoll` generates its profile from it.
    skeleton: Skeleton,
    /// Frame times in milliseconds after the warmup.
    frames: Vec<f64>,
    /// Fixed-step times in milliseconds after the warmup.
    steps: Vec<f64>,
    /// Start of the frame or fixed step being timed.
    frame_start: Option<Instant>,
    /// Start of the fixed step being timed.
    step_start: Option<Instant>,
}

impl Run {
    /// Returns whether the warmup has passed at `elapsed` simulated seconds.
    fn is_measuring(&self, elapsed: f64) -> bool {
        elapsed >= self.args.warmup
    }
}

/// Parses options and runs the stress test.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    // The web page has no command line, so it runs with the defaults.
    #[cfg(target_arch = "wasm32")]
    let args = Args::parse_from(["ragdoll_stress"]);
    #[cfg(not(target_arch = "wasm32"))]
    let args = Args::parse();

    let step = Duration::from_secs_f64(1.0 / FIXED_HZ);

    let mut app = App::new();
    if args.headless {
        app.add_plugins((
            MinimalPlugins.set(ScheduleRunnerPlugin::run_loop(Duration::ZERO)),
            AssetPlugin::default(),
            TransformPlugin,
        ));
    } else {
        app.add_plugins(DefaultPlugins)
            .add_systems(Startup, setup_view)
            .add_systems(
                PostUpdate,
                add_body_meshes
                    .after(RagdollSystems::Bind)
                    .before(TransformSystems::Propagate),
            );
    }
    app.insert_resource(Run {
        args,
        skeleton: Skeleton::humanoid(),
        frames: Vec::new(),
        steps: Vec::new(),
        frame_start: None,
        step_start: None,
    })
    // Advance time by exactly one fixed step per frame, however long the frame takes.
    .insert_resource(TimeUpdateStrategy::ManualDuration(step))
    .insert_resource(Time::<Fixed>::from_hz(FIXED_HZ))
    .insert_resource(TimestepMode::Fixed {
        dt: step.as_secs_f32(),
        substeps: 1,
    })
    .add_plugins((
        RagdollPlugin::default(),
        RapierPhysicsPlugin::<RapierRagdollHooks<'_, '_>>::default().in_fixed_schedule(),
        RapierRagdollPlugin,
    ))
    .add_systems(Startup, spawn_ragdolls)
    .add_systems(First, start_frame)
    .add_systems(Last, (finish_frame, finish_run).chain())
    .add_systems(FixedFirst, start_step)
    .add_systems(FixedLast, finish_step);

    match app.run() {
        AppExit::Success => Ok(()),
        AppExit::Error(code) => Err(format!("stress run failed with exit code {code}").into()),
    }
}

/// Spawns the floor and the ragdolls for the selected scenario.
fn spawn_ragdolls(mut commands: Commands<'_, '_>, run: Res<'_, Run>) {
    // A wide static floor so neither scenario falls off the edge.
    commands.spawn((
        RigidBody::Fixed,
        Collider::cuboid(100.0, 0.1, 100.0),
        Transform::from_xyz(0.0, -0.1, 0.0),
    ));
    // The smallest square grid that fits every ragdoll.
    let count = run.args.count;
    let columns = (1..=count)
        .find(|c| u32::from(*c).pow(2) >= u32::from(count))
        .unwrap_or(1);
    // Ragdolls start limp so the run measures contacts and joints, not muscles.
    for index in 0..count {
        let position = match run.args.scenario {
            Scenario::Pile => Vec3::new(0.0, f32::from(index).mul_add(0.8, 0.3), 0.0),
            Scenario::Grid => {
                let offset = (f32::from(columns) - 1.0) * 0.5;
                let (column, row) = (f32::from(index % columns), f32::from(index / columns));
                Vec3::new((column - offset) * 1.5, 0.3, (row - offset) * 1.5)
            }
        };
        // Pile ragdolls lie flat so the stack builds up contacts.
        let rotation = match run.args.scenario {
            Scenario::Pile => {
                Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2)
                    * Quat::from_rotation_z(f32::from(index))
            }
            Scenario::Grid => Quat::IDENTITY,
        };
        let character = commands
            .spawn((
                Ragdoll::default(),
                RagdollMode::Dynamic,
                RagdollDrive::new(0.0, 0.0),
                Transform::from_translation(position).with_rotation(rotation),
            ))
            .id();
        run.skeleton.spawn(&mut commands, character);
    }
}

/// Marks the start of a frame.
fn start_frame(mut run: ResMut<'_, Run>) {
    run.frame_start = Some(Instant::now());
}

/// Records the frame time once the warmup has passed.
fn finish_frame(time: Res<'_, Time>, mut run: ResMut<'_, Run>) {
    if let Some(start) = run.frame_start.take()
        && run.is_measuring(time.elapsed_secs_f64())
    {
        run.frames.push(start.elapsed().as_secs_f64() * 1000.0);
    }
}

/// Marks the start of a fixed step.
fn start_step(mut run: ResMut<'_, Run>) {
    run.step_start = Some(Instant::now());
}

/// Records the fixed-step time once the warmup has passed.
fn finish_step(time: Res<'_, Time>, mut run: ResMut<'_, Run>) {
    if let Some(start) = run.step_start.take()
        && run.is_measuring(time.elapsed_secs_f64())
    {
        run.steps.push(start.elapsed().as_secs_f64() * 1000.0);
    }
}

/// Ends the run after warmup plus duration, prints a summary, and writes the
/// report.
fn finish_run(
    time: Res<'_, Time>,
    mut run: ResMut<'_, Run>,
    bodies: Query<'_, '_, (&BodyPhysicsPose, &BodyVelocity), With<RagdollBodyOf>>,
    mut exit: MessageWriter<'_, AppExit>,
) {
    // Keep running until the warmup and the measured duration have both passed.
    if time.elapsed_secs_f64() < run.args.warmup + run.args.duration {
        return;
    }
    // A NaN pose or an extreme speed means the solver blew up.
    let unstable_bodies = bodies
        .iter()
        .filter(|(pose, velocity)| {
            let is_finite_pose =
                pose.current.translation.is_finite() && pose.current.rotation.is_finite();
            !is_finite_pose || velocity.linear.length() > UNSTABLE_SPEED
        })
        .count();
    let metrics = Metrics {
        frame_ms: Summary::new(std::mem::take(&mut run.frames)),
        step_ms: Summary::new(std::mem::take(&mut run.steps)),
        bodies: bodies.iter().count(),
        unstable_bodies,
    };
    // Print a one-line summary for people watching the terminal.
    let (count, bodies, unstable) = (run.args.count, metrics.bodies, metrics.unstable_bodies);
    let (frame, step) = (metrics.frame_ms.p95, metrics.step_ms.p95);
    println!(
        "{count} ragdolls, {bodies} bodies: frame p95 {frame:.2} ms, step p95 {step:.2} ms, {unstable} unstable"
    );
    // CI reads the JSON report, so a failed write must fail the run.
    let report = Report {
        config: &run.args,
        metrics,
    };
    if let Some(path) = &run.args.report {
        let written = serde_json::to_string_pretty(&report)
            .map_err(std::io::Error::other)
            .and_then(|json| std::fs::write(path, json));
        if let Err(error) = written {
            error!(path = %path.display(), %error, "could not write the stress report");
            exit.write(AppExit::error());
            return;
        }
    }
    exit.write(AppExit::Success);
}

/// Adds a camera and a light for the windowed run.
fn setup_view(mut commands: Commands<'_, '_>) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(10.0, 9.0, 14.0).looking_at(Vec3::new(0.0, 0.5, 0.0), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 10_000.0,
            ..default()
        },
        Transform::from_xyz(-4.0, 8.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

/// Gives each new physics body a mesh matching its collider in the windowed run.
fn add_body_meshes(
    mut commands: Commands<'_, '_>,
    mut meshes: ResMut<'_, Assets<Mesh>>,
    mut material: Local<'_, Option<Handle<StandardMaterial>>>,
    mut materials: ResMut<'_, Assets<StandardMaterial>>,
    bodies: Query<'_, '_, (Entity, &BodyShape), Added<BodyShape>>,
) {
    // One shared material keeps the windowed run cheap to render.
    let material = material
        .get_or_insert_with(|| materials.add(Color::srgb(0.9, 0.6, 0.35)))
        .clone();
    for (entity, shape) in &bodies {
        let (mesh, transform) = shape.0.mesh();
        commands.entity(entity).insert(Visibility::Inherited);
        commands.spawn((
            Mesh3d(meshes.add(mesh)),
            MeshMaterial3d(material.clone()),
            transform,
            ChildOf(entity),
        ));
    }
}
