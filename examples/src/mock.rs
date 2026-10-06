//! Shared visible and headless app setup for every physics backend example.
//!
//! This crate selects the requested backend, builds the example rig, displays
//! its status overlay, and supports headless execution, bounded process
//! duration, and screenshots. The `custom-backend` feature provides the
//! intentionally approximate mock scene; phase-specific Rapier examples use the
//! same startup and screenshot options while supplying their own backend
//! plugin.

#![cfg(feature = "custom-backend")]

use std::path::PathBuf;
use std::str::FromStr;
use std::time::Duration;

use bevy::app::{AppExit, ScheduleRunnerPlugin};
use bevy::camera::RenderTarget;
use bevy::ecs::schedule::IntoScheduleConfigs;
use bevy::math::{Isometry3d, Quat, Vec3};
use bevy::prelude::{
    App, Assets, Camera3d, Capsule3d, ChildOf, ClearColor, Color, Commands, Cuboid, DefaultPlugins,
    DirectionalLight, Entity, FontSize, Image, Mesh, Mesh3d, MeshMaterial3d, Meshable, Name, Node,
    Plane3d, PluginGroup, PositionType, Query, Res, ResMut, Resource, Sphere, StandardMaterial,
    Startup, Text, TextColor, TextFont, Time, Timer, TimerMode, Transform, Update, Visibility,
    With, px,
};
use bevy::render::RenderPlugin;
use bevy::render::render_resource::TextureFormat;
use bevy::render::view::screenshot::{Screenshot, save_to_disk};
use bevy::time::{Fixed, TimeUpdateStrategy};
use bevy::window::{ExitCondition, Window, WindowPlugin};
use bevy::winit::WinitPlugin;
use bevy_ragdoll::profile::{AngleRange, BodyIndex, ProfileBuilder};
use bevy_ragdoll::runtime::body::BodyShape;
use bevy_ragdoll::runtime::components::{Ragdoll, RagdollDrive, RagdollMode};
use bevy_ragdoll::runtime::sets::RagdollSystems;
use bevy_ragdoll::{JointLimits, RagdollPlugin, RagdollProfile, ShapeSpec};
use bevy_ragdoll_conformance::mock::MockBackendPlugin;
use clap::{Parser, ValueEnum};

/// A command-line duration for the example process.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ExitAfter(Duration);

/// Parses positive finite seconds into a standard duration.
impl FromStr for ExitAfter {
    type Err = String;

    /// Converts seconds at the command-line boundary.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        let seconds = input
            .parse::<f64>()
            .map_err(|_| "seconds must be a positive finite number".to_owned())?;
        if !seconds.is_finite() || seconds <= 0.0 {
            return Err("seconds must be a positive finite number".to_owned());
        }
        Duration::try_from_secs_f64(seconds)
            .map(Self)
            .map_err(|_| "seconds exceed the supported duration".to_owned())
    }
}

/// A screenshot path parsed from an optional command-line value.
#[derive(Clone, Debug, PartialEq, Eq)]
struct ScreenshotPath(PathBuf);

/// Parses a path or the default output path marker.
impl FromStr for ScreenshotPath {
    type Err = std::convert::Infallible;

    /// Converts the command-line spelling to a filesystem path.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        let path = if input == "__default_screenshot_path__" {
            default_screenshot_path()
        } else {
            PathBuf::from(input)
        };
        Ok(Self(path))
    }
}

/// A closed headless-rendering mode marker.
#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
enum HeadlessMode {
    /// Render to an image without creating a window.
    Enabled,
}

/// Options shared by visible and headless example runs.
#[derive(Clone, Debug, Resource)]
struct ExampleOptions {
    /// Headless mode marker supplied by the caller.
    headless: Option<HeadlessMode>,
    /// Optional app lifetime limit.
    exit_after: Option<ExitAfter>,
    /// Optional screenshot destination.
    screenshot: Option<PathBuf>,
}

/// The image target used by a headless camera.
#[derive(Clone, Debug, Resource)]
struct CaptureTarget {
    /// Render target image, absent for a primary-window camera.
    image: Option<bevy::asset::Handle<Image>>,
}

/// Marks one render entity created for a physics body.
#[derive(Clone, Copy, Debug, bevy::prelude::Component)]
struct ExampleBodyVisual;

/// Backend selected by this example runner.
#[derive(Clone, Copy, Debug)]
pub enum BackendSelection {
    /// Engine-agnostic mock backend used by phase 4.
    Mock,
}

/// Runs the shared mock-backend scene with command-line controls.
pub fn run_custom_backend() -> Result<(), Box<dyn std::error::Error>> {
    let cli = ExampleCli::parse();
    let options = ExampleOptions {
        headless: cli.headless,
        exit_after: cli.exit_after,
        screenshot: cli.screenshot.map(|path| path.0),
    };
    let screenshot_path = options.screenshot.clone();
    let mut app = App::new();
    app.add_plugins(default_plugins(&options));
    if options.headless.is_some() {
        app.add_plugins(ScheduleRunnerPlugin::run_loop(Duration::from_secs_f64(
            1.0 / 60.0,
        )));
    }
    app.insert_resource(options.clone());
    app.insert_resource(ClearColor(Color::srgb(0.055, 0.075, 0.095)));
    app.insert_resource(Time::<Fixed>::from_hz(60.0));
    app.insert_resource(TimeUpdateStrategy::FixedTimesteps(1));
    app.add_plugins(RagdollPlugin::default());
    add_backend(&mut app, BackendSelection::Mock);
    app.add_systems(Startup, setup_scene);
    app.add_systems(
        bevy::app::PostUpdate,
        create_body_visuals
            .after(RagdollSystems::Bind)
            .before(bevy::transform::TransformSystems::Propagate),
    );
    app.add_systems(Update, (capture_screenshot, exit_after));
    app.run();

    if let Some(path) = screenshot_path
        && !path.is_file()
    {
        return Err(format!("screenshot was not written to {}", path.display()).into());
    }
    Ok(())
}

/// Parses example options with a closed marker for the presence-only mode.
#[derive(Clone, Debug, Parser)]
#[command(about = "Run the mock ragdoll example")]
struct ExampleCli {
    /// Render without a window.
    #[arg(long, value_enum, num_args = 0..=1, default_missing_value = "enabled")]
    headless: Option<HeadlessMode>,
    /// Exit after this many seconds.
    #[arg(long)]
    exit_after: Option<ExitAfter>,
    /// Save a screenshot to PATH, defaulting to the external Cargo target
    /// directory.
    #[arg(long, value_name = "PATH", num_args = 0..=1, default_missing_value = "__default_screenshot_path__")]
    screenshot: Option<ScreenshotPath>,
}

/// Configures visible or windowless Bevy plugins for the selected run mode.
fn default_plugins(options: &ExampleOptions) -> impl PluginGroup {
    let window = if options.headless.is_some() {
        WindowPlugin {
            primary_window: None,
            exit_condition: ExitCondition::DontExit,
            ..Default::default()
        }
    } else {
        WindowPlugin {
            primary_window: Some(Window {
                title: "Bevy ragdoll mock backend".to_owned(),
                resolution: (1280, 720).into(),
                ..Default::default()
            }),
            ..Default::default()
        }
    };
    let plugins = DefaultPlugins.set(window).set(RenderPlugin {
        synchronous_pipeline_compilation: true,
        ..Default::default()
    });
    if options.headless.is_some() {
        plugins.disable::<WinitPlugin>()
    } else {
        plugins
    }
}

/// Adds the selected physics backend to the app.
fn add_backend(app: &mut App, backend: BackendSelection) {
    match backend {
        BackendSelection::Mock => {
            app.add_plugins(MockBackendPlugin);
        }
    }
}

/// Creates the camera, floor, overlay, profile asset, and named skeleton.
fn setup_scene(
    mut commands: Commands,
    options: Res<ExampleOptions>,
    mut images: ResMut<Assets<Image>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut profiles: ResMut<Assets<RagdollProfile>>,
) {
    let target = if options.headless.is_some() {
        let image = Image::new_target_texture(1280, 720, TextureFormat::Rgba8UnormSrgb, None);
        Some(images.add(image))
    } else {
        None
    };
    commands.insert_resource(CaptureTarget {
        image: target.clone(),
    });

    let camera_transform =
        Transform::from_xyz(3.1, 2.7, 5.8).looking_at(Vec3::new(0.0, 1.25, 0.0), Vec3::Y);
    if let Some(image) = target.as_ref() {
        commands.spawn((
            Camera3d::default(),
            RenderTarget::Image(image.clone().into()),
            camera_transform,
        ));
    } else {
        // Keep one window camera so Bevy routes the overlay to the 3D view.
        commands.spawn((Camera3d::default(), camera_transform));
    }

    commands.spawn((
        DirectionalLight {
            illuminance: 12_000.0,
            shadow_maps_enabled: true,
            ..Default::default()
        },
        Transform::from_xyz(-3.0, 6.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(12.0, 12.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.11, 0.15, 0.19),
            perceptual_roughness: 0.9,
            ..Default::default()
        })),
        Transform::from_xyz(0.0, -0.02, 0.0),
    ));
    commands.spawn((
        Text::new("Mock backend: joint constraints are ignored."),
        TextFont {
            font_size: FontSize::Px(22.0),
            ..Default::default()
        },
        TextColor(Color::WHITE),
        Node {
            position_type: PositionType::Absolute,
            top: px(16),
            left: px(18),
            ..Default::default()
        },
    ));

    let profile = build_example_profile();
    let handle = profiles.add(profile);
    let character = commands
        .spawn((
            Ragdoll::new(handle),
            RagdollMode::Dynamic,
            RagdollDrive::new(0.0, 0.0),
            Transform::IDENTITY,
        ))
        .id();
    let pelvis = commands
        .spawn((
            Name::new("pelvis"),
            Transform::from_translation(Vec3::new(0.0, 1.0, 0.0)),
            ChildOf(character),
        ))
        .id();
    let chest = commands
        .spawn((
            Name::new("chest"),
            Transform::from_translation(Vec3::new(0.0, 0.7, 0.0)),
            ChildOf(pelvis),
        ))
        .id();
    commands.spawn((
        Name::new("head"),
        Transform::from_translation(Vec3::new(0.0, 0.65, 0.0)),
        ChildOf(chest),
    ));
}

/// Builds the pelvis, chest, and head profile used by the example.
fn build_example_profile() -> RagdollProfile {
    let mut builder = ProfileBuilder::default();
    let pelvis = builder
        .add_body(
            "pelvis",
            ShapeSpec::Capsule {
                a: Vec3::new(-0.24, 0.0, 0.0),
                b: Vec3::new(0.24, 0.0, 0.0),
                radius: 0.22,
            },
            8.0,
            Isometry3d::from_translation(Vec3::new(0.0, 1.0, 0.0)),
        )
        .expect("the pelvis fits the profile");
    let chest = builder
        .add_body(
            "chest",
            ShapeSpec::Capsule {
                a: Vec3::ZERO,
                b: Vec3::new(0.0, 0.58, 0.0),
                radius: 0.17,
            },
            12.0,
            Isometry3d::from_translation(Vec3::new(0.0, 1.7, 0.0)),
        )
        .expect("the chest fits the profile");
    let head = builder
        .add_body(
            "head",
            ShapeSpec::Sphere {
                center: Vec3::ZERO,
                radius: 0.16,
            },
            4.0,
            Isometry3d::from_translation(Vec3::new(0.0, 2.35, 0.0)),
        )
        .expect("the head fits the profile");
    let bend = AngleRange {
        min: -0.6,
        max: 0.6,
    };
    let limits = JointLimits {
        x: bend,
        twist: bend,
        z: bend,
    };
    builder.add_joint(
        chest,
        pelvis,
        Isometry3d::from_translation(Vec3::new(0.0, 0.7, 0.0)),
        limits,
        80.0,
    );
    builder.add_joint(
        head,
        chest,
        Isometry3d::from_translation(Vec3::new(0.0, 0.65, 0.0)),
        limits,
        25.0,
    );
    builder.build().expect("the example body tree is valid")
}

/// Adds visible meshes as children of newly spawned physics bodies.
fn create_body_visuals(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    bodies: Query<(Entity, &BodyShape, &BodyIndex), bevy::prelude::Added<BodyShape>>,
) {
    for (body, shape, index) in &bodies {
        commands.entity(body).insert(Visibility::Inherited);
        let (mesh, local_transform) = visual_shape(&shape.0);
        let color = match index.get() {
            0 => Color::srgb(0.18, 0.62, 0.76),
            1 => Color::srgb(0.25, 0.78, 0.64),
            _ => Color::srgb(0.92, 0.66, 0.34),
        };
        commands.spawn((
            Mesh3d(meshes.add(mesh)),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                metallic: 0.03,
                perceptual_roughness: 0.45,
                ..Default::default()
            })),
            ExampleBodyVisual,
            local_transform,
            ChildOf(body),
        ));
    }
}

/// Converts profile collision shapes into matching Bevy meshes and local poses.
fn visual_shape(shape: &ShapeSpec) -> (Mesh, Transform) {
    match shape {
        ShapeSpec::Capsule { a, b, radius } => {
            let segment = *b - *a;
            let rotation = if segment.length_squared() > f32::EPSILON {
                Quat::from_rotation_arc(Vec3::Y, segment.normalize())
            } else {
                Quat::IDENTITY
            };
            (
                Mesh::from(Capsule3d::new(*radius, segment.length())),
                Transform::from_translation((*a + *b) * 0.5).with_rotation(rotation),
            )
        }
        ShapeSpec::Sphere { center, radius } => (
            Mesh::from(Sphere::new(*radius)),
            Transform::from_translation(*center),
        ),
        ShapeSpec::Cuboid {
            center,
            rotation,
            half_extents,
        } => (
            Mesh::from(Cuboid::from_size(*half_extents * 2.0)),
            Transform::from_translation(*center).with_rotation(*rotation),
        ),
    }
}

/// Requests one screenshot after every body mesh exists.
fn capture_screenshot(
    mut commands: Commands,
    options: Res<ExampleOptions>,
    capture_target: Res<CaptureTarget>,
    bodies: Query<(), With<BodyShape>>,
    visuals: Query<(), With<ExampleBodyVisual>>,
    mut requested: bevy::prelude::Local<bool>,
    mut ready_frames: bevy::prelude::Local<u8>,
) {
    let Some(path) = options.screenshot.as_ref() else {
        return;
    };
    if *requested || bodies.iter().count() < 3 || visuals.iter().count() < 3 {
        return;
    }
    if *ready_frames < 30 {
        *ready_frames += 1;
        return;
    }
    if let Some(parent) = path.parent()
        && !parent.as_os_str().is_empty()
        && let Err(error) = std::fs::create_dir_all(parent)
    {
        bevy::log::error!(
            screenshot_directory_error = %error,
            "could not create screenshot directory"
        );
        return;
    }
    let screenshot = capture_target
        .image
        .as_ref()
        .map_or_else(Screenshot::primary_window, |image| {
            Screenshot::image(image.clone())
        });
    commands
        .spawn(screenshot)
        .observe(save_to_disk(path.clone()));
    *requested = true;
}

/// Exits after the requested duration while allowing screenshot readback to
/// finish.
fn exit_after(
    options: Res<ExampleOptions>,
    time: Res<Time>,
    mut timer: bevy::prelude::Local<Option<Timer>>,
    mut app_exit: bevy::prelude::MessageWriter<AppExit>,
) {
    let Some(duration) = options.exit_after.map(|exit_after| exit_after.0) else {
        return;
    };
    let timer = timer.get_or_insert_with(|| Timer::new(duration, TimerMode::Once));
    timer.tick(time.delta());
    if timer.is_finished() {
        app_exit.write(AppExit::Success);
    }
}

/// Makes a Cargo-target-local screenshot path when the CLI omits a path.
fn default_screenshot_path() -> PathBuf {
    std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("target"))
        .join("screenshots/custom_backend.png")
}
