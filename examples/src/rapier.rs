//! Shared startup, profile loading, and rendering for the Rapier examples.

use std::error::Error;
use std::path::PathBuf;
use std::str::FromStr;
use std::time::Duration;

use bevy::app::{AppExit, ScheduleRunnerPlugin, Startup, Update};
use bevy::asset::{AssetPlugin, Assets};
#[cfg(feature = "visual")]
use bevy::ecs::schedule::IntoScheduleConfigs;
use bevy::math::{Isometry3d, Vec3};
use bevy::prelude::{
    App, ChildOf, Commands, Entity, MinimalPlugins, Name, PluginGroup, Res, ResMut, Resource, Time,
    Timer, TimerMode, Transform,
};
#[cfg(feature = "visual")]
use bevy::prelude::{Query, With};
use bevy::time::{Fixed, TimeUpdateStrategy};
use bevy::transform::TransformPlugin;
#[cfg(feature = "visual")]
use bevy::transform::TransformSystems;
use bevy_ragdoll::profile::{AngleRange, ProfileBuilder, ProfileSpec, RagdollProfile, ShapeSpec};
#[cfg(feature = "visual")]
use bevy_ragdoll::runtime::body::BodyShape;
use bevy_ragdoll::runtime::components::{
    BodyWeights, Ragdoll, RagdollBodyWeights, RagdollDrive, RagdollMode,
};
#[cfg(feature = "visual")]
use bevy_ragdoll::runtime::sets::RagdollSystems;
use bevy_ragdoll::{JointLimits, RagdollPlugin};
use bevy_ragdoll_rapier3d::{RapierRagdollHooks, RapierRagdollPlugin};
use bevy_rapier3d::plugin::{RapierPhysicsPlugin, TimestepMode};
use bevy_rapier3d::prelude::{Collider, RigidBody};
use clap::{Parser, ValueEnum};

mod active;

#[cfg(feature = "visual")]
use bevy::math::Quat;
#[cfg(feature = "visual")]
use bevy::prelude::{
    Assets as BevyAssets, Camera3d, Capsule3d, ClearColor, Color, Cuboid, DefaultPlugins,
    DirectionalLight, FontSize, Mesh, Mesh3d, MeshMaterial3d, Meshable, Node, Plane3d,
    PositionType, StandardMaterial, Text, TextColor, TextFont, Visibility, px,
};
#[cfg(feature = "visual")]
use bevy::render::view::screenshot::{Screenshot, save_to_disk};
#[cfg(feature = "visual")]
use bevy::window::{Window, WindowPlugin};

/// The closed set of profile and runtime demonstrations in this package.
#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
pub enum ExampleKind {
    /// Drops the credited TGF human profile and checks joint stability.
    Minimal,
    /// Builds a three-body chain in Rust and pins its root body.
    FromCode,
    /// Loads the human profile from RON source data.
    FromRon,
    /// Imports the TGF GLB profile with its Skein annotations.
    FromGltfSkein,
    /// Runs a hit-reactive human ragdoll with local muscle and pin controls.
    HitReactions,
    /// Leaves the upper body loose while the legs follow their target poses.
    PartialRagdoll,
}

impl ExampleKind {
    /// Returns the visible title used by the scene and native window.
    const fn title(self) -> &'static str {
        match self {
            Self::Minimal => "Minimal ragdoll",
            Self::FromCode => "Profile built in Rust",
            Self::FromRon => "Profile loaded from RON",
            Self::FromGltfSkein => "Profile imported from glTF and Skein",
            Self::HitReactions => "Hit reactions",
            Self::PartialRagdoll => "Partial ragdoll",
        }
    }

    /// Returns the documented behavior displayed in the scene.
    #[cfg(feature = "visual")]
    const fn check(self) -> &'static str {
        match self {
            Self::Minimal => "Check: lands, keeps its joints together, and settles.",
            Self::FromCode => "Check: the pinned pelvis holds while the chain swings.",
            Self::FromRon => "Check: a validated human profile loads from RON.",
            Self::FromGltfSkein => "Check: GLB bones and Skein annotations form a rig.",
            Self::HitReactions => "Click the rig to hit it; adjust local muscle and pin strengths.",
            Self::PartialRagdoll => "Check: legs stay driven while the upper body yields.",
        }
    }
}

/// Loads and validates the profile used by one documented example.
///
/// RON and GLB bytes remain borrowed until their parsers create validated data;
/// profiles contain at most the runtime's bounded 64 bodies.
pub fn load_profile(kind: ExampleKind) -> Result<RagdollProfile, Box<dyn Error>> {
    match kind {
        ExampleKind::Minimal => {
            load_ron_profile(include_str!("../../assets/profiles/tgf_human.ragdoll.ron"))
        }
        ExampleKind::FromCode => Ok(build_code_profile()?),
        ExampleKind::FromRon => {
            load_ron_profile(include_str!("../../assets/profiles/human.ragdoll.ron"))
        }
        ExampleKind::FromGltfSkein => {
            let spec =
                ProfileSpec::from_glb(include_bytes!("../../assets/rigs/tgf_human/tgf_human.glb"))?;
            Ok(RagdollProfile::new(spec)?)
        }
        ExampleKind::HitReactions | ExampleKind::PartialRagdoll => {
            let spec =
                ProfileSpec::from_glb(include_bytes!("../../assets/rigs/tgf_human/tgf_human.glb"))?;
            Ok(RagdollProfile::new(spec)?)
        }
    }
}

/// Parses one embedded RON profile and validates its body and joint invariants.
fn load_ron_profile(input: &str) -> Result<RagdollProfile, Box<dyn Error>> {
    let spec = ron::from_str::<ProfileSpec>(input)?;
    Ok(RagdollProfile::new(spec)?)
}

/// Builds the documented pelvis, chest, and head chain through `ProfileBuilder`.
fn build_code_profile() -> Result<RagdollProfile, bevy_ragdoll::ProfileError> {
    let mut builder = ProfileBuilder::default();
    let pelvis = builder.add_body(
        "pelvis",
        ShapeSpec::Capsule {
            a: Vec3::new(-0.24, 0.0, 0.0),
            b: Vec3::new(0.24, 0.0, 0.0),
            radius: 0.2,
        },
        8.0,
        Isometry3d::from_translation(Vec3::new(0.0, 1.0, 0.0)),
    )?;
    let chest = builder.add_body(
        "chest",
        ShapeSpec::Capsule {
            a: Vec3::ZERO,
            b: Vec3::new(0.0, 0.6, 0.0),
            radius: 0.17,
        },
        12.0,
        Isometry3d::from_translation(Vec3::new(0.0, 1.7, 0.0)),
    )?;
    let head = builder.add_body(
        "head",
        ShapeSpec::Sphere {
            center: Vec3::ZERO,
            radius: 0.16,
        },
        4.0,
        Isometry3d::from_translation(Vec3::new(0.0, 2.36, 0.0)),
    )?;
    let bend = AngleRange {
        min: -0.7,
        max: 0.7,
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
        Isometry3d::from_translation(Vec3::new(0.0, 0.66, 0.0)),
        limits,
        25.0,
    );
    builder.build()
}

/// Options parsed from a native example command line.
#[derive(Clone, Debug, Default, Parser)]
#[command(about = "Run one bevy-ragdoll Rapier example")]
struct ExampleCli {
    /// Run a different profile source in the same shared scene runner.
    #[arg(long, value_enum)]
    kind: Option<ExampleKind>,
    /// Run without creating a window.
    #[arg(long, value_enum, num_args = 0..=1, default_missing_value = "enabled")]
    headless: Option<HeadlessMode>,
    /// Exit after this positive finite duration in seconds.
    #[arg(long)]
    exit_after: Option<ExitAfter>,
    /// Save one screenshot to PATH, or use the target-local default path.
    #[arg(
        long,
        value_name = "PATH",
        num_args = 0..=1,
        default_missing_value = "__default_screenshot_path__"
    )]
    screenshot: Option<ScreenshotPath>,
}

/// Marks the presence-only headless mode without a boolean field.
#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
enum HeadlessMode {
    /// Create no window and run the fixed simulation loop.
    Enabled,
}

/// A positive finite application lifetime stored as a standard duration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ExitAfter {
    /// Validated positive process lifetime.
    seconds: Duration,
}

/// Parses positive finite seconds at the command-line boundary.
impl FromStr for ExitAfter {
    type Err = &'static str;

    /// Converts a seconds value to a validated `Duration`.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        let seconds = input
            .parse::<f64>()
            .map_err(|_| "seconds must be a positive finite number")?;
        if !seconds.is_finite() || seconds <= 0.0 {
            return Err("seconds must be a positive finite number");
        }
        Duration::try_from_secs_f64(seconds)
            .map(|seconds| Self { seconds })
            .map_err(|_| "seconds exceed the supported duration")
    }
}

/// A screenshot destination parsed into a filesystem path.
#[derive(Clone, Debug, PartialEq, Eq)]
struct ScreenshotPath {
    /// Final output path after default-marker expansion.
    path: PathBuf,
}

/// Parses a path or the marker that selects the target-local default.
impl FromStr for ScreenshotPath {
    type Err = std::convert::Infallible;

    /// Converts the CLI spelling to its final destination.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        let path = if input == "__default_screenshot_path__" {
            default_screenshot_path()
        } else {
            PathBuf::from(input)
        };
        Ok(Self { path })
    }
}

/// Runtime options shared by visible and headless example modes.
#[derive(Clone, Debug, Default, Resource)]
struct ExampleOptions {
    /// Closed headless-mode marker supplied by the command line.
    headless: Option<HeadlessMode>,
    /// Optional application lifetime limit.
    exit_after: Option<ExitAfter>,
    /// Optional screenshot destination for a visible run.
    screenshot: Option<PathBuf>,
}

/// Profile and example kind inserted after construction for the startup system.
#[derive(Clone, Debug, Resource)]
struct ExampleScene {
    /// Example selection that supplies the title and check line.
    kind: ExampleKind,
    /// Validated profile used by the runtime and scene skeleton.
    profile: RagdollProfile,
}

/// Runs one example with CLI options on native targets and defaults on WebAssembly.
pub fn run_example(kind: ExampleKind) -> Result<(), Box<dyn Error>> {
    #[cfg(not(target_arch = "wasm32"))]
    let cli = ExampleCli::parse();
    #[cfg(target_arch = "wasm32")]
    let cli = ExampleCli::default();
    let kind = cli.kind.unwrap_or(kind);

    let options = ExampleOptions {
        headless: cli.headless,
        exit_after: cli.exit_after,
        screenshot: cli.screenshot.map(|screenshot| screenshot.path),
    };
    #[cfg(not(feature = "visual"))]
    if options.headless.is_none() {
        return Err("visible examples require the `visual` feature".into());
    }
    #[cfg(not(feature = "visual"))]
    if options.screenshot.is_some() {
        return Err("screenshots require the `visual` feature".into());
    }
    #[cfg(feature = "visual")]
    if options.headless.is_some() && options.screenshot.is_some() {
        return Err("headless screenshots are not enabled in this example runner".into());
    }

    let profile = load_profile(kind)?;
    let screenshot_path = options.screenshot.clone();
    let mut app = App::new();
    if options.headless.is_some() {
        app.add_plugins(MinimalPlugins.set(ScheduleRunnerPlugin::run_loop(
            Duration::from_secs_f64(1.0 / 60.0),
        )));
        app.add_plugins((AssetPlugin::default(), TransformPlugin));
    } else {
        #[cfg(feature = "visual")]
        app.add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: kind.title().to_owned(),
                resolution: (1280, 720).into(),
                ..Default::default()
            }),
            ..Default::default()
        }));
    }
    app.insert_resource(options.clone())
        .insert_resource(ExampleScene { kind, profile })
        .insert_resource(Time::<Fixed>::from_hz(60.0))
        .insert_resource(TimeUpdateStrategy::FixedTimesteps(1))
        .insert_resource(TimestepMode::Fixed {
            dt: 1.0 / 60.0,
            substeps: 1,
        });
    #[cfg(feature = "visual")]
    app.insert_resource(ClearColor(Color::srgb(0.055, 0.075, 0.095)));
    app.add_plugins(RagdollPlugin::default());
    app.add_plugins(RapierPhysicsPlugin::<RapierRagdollHooks>::default().in_fixed_schedule());
    app.add_plugins(RapierRagdollPlugin);
    app.add_systems(Startup, (spawn_example, spawn_ground));
    #[cfg(feature = "visual")]
    app.add_systems(Startup, setup_scene_visuals);
    #[cfg(feature = "visual")]
    app.add_systems(
        bevy::app::PostUpdate,
        create_body_visuals
            .after(RagdollSystems::Bind)
            .before(TransformSystems::Propagate),
    );
    app.add_systems(Update, exit_after);
    #[cfg(feature = "visual")]
    app.add_systems(Update, capture_screenshot);
    #[cfg(feature = "visual")]
    active::install(&mut app, kind);
    app.run();

    if let Some(path) = screenshot_path
        && !path.is_file()
    {
        return Err(format!("screenshot was not written to {}", path.display()).into());
    }
    Ok(())
}

/// Adds the fixed collision surface used by every Rapier example.
fn spawn_ground(mut commands: Commands) {
    commands.spawn((
        RigidBody::Fixed,
        Collider::cuboid(6.0, 0.1, 6.0),
        Transform::from_xyz(0.0, -0.1, 0.0),
    ));
}

/// Builds the character skeleton from profile order and derived parent links.
fn spawn_example(
    mut commands: Commands,
    mut profiles: ResMut<Assets<RagdollProfile>>,
    scene: Res<ExampleScene>,
) {
    let profile_handle = profiles.add(scene.profile.clone());
    let mut character = commands.spawn((
        Name::new(scene.kind.title()),
        Ragdoll::new(profile_handle),
        RagdollMode::Dynamic,
        Transform::from_xyz(0.0, 0.35, 0.0),
    ));
    if scene.kind == ExampleKind::FromCode {
        let weights = (0..scene.profile.bodies().len())
            .map(|index| {
                if index == 0 {
                    BodyWeights::new(1.0, 1.0)
                } else {
                    BodyWeights::new(1.0, 0.0)
                }
            })
            .collect();
        character.insert((
            RagdollDrive::new(0.0, 1.0),
            RagdollBodyWeights::new(weights),
        ));
    } else if let Some((weights, pin_targets)) =
        active::character_controls(scene.kind, &scene.profile)
    {
        character.insert((RagdollDrive::new(1.0, 1.0), weights, pin_targets));
    } else {
        character.insert(RagdollDrive::default());
    }
    let character = character.id();

    // Derive one parent slot per body from the validated child-parent joints.
    let bodies = scene.profile.bodies();
    let mut parents = vec![None; bodies.len()];
    for joint in scene.profile.joints() {
        parents[joint.child().get()] = Some(joint.parent().get());
    }

    // Place each named bone at the profile rest pose under its checked parent.
    let mut bones = Vec::<Entity>::with_capacity(bodies.len());
    for (index, body) in bodies.iter().enumerate() {
        let (parent, transform) = parents[index].map_or_else(
            || {
                (
                    character,
                    Transform {
                        translation: body.rest().translation.into(),
                        rotation: body.rest().rotation,
                        ..Default::default()
                    },
                )
            },
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
        let target_bone = active::target_bone(scene.kind, index, body, transform.clone());
        let bone = commands
            .spawn((
                Name::new(body.bone().to_owned()),
                transform,
                ChildOf(parent),
            ))
            .id();
        #[cfg(feature = "visual")]
        if let Some(target_bone) = target_bone {
            commands.entity(bone).insert(target_bone);
        }
        bones.push(bone);
    }
}

/// Configures the camera, ground, directional light, and example label.
#[cfg(feature = "visual")]
fn setup_scene_visuals(
    mut commands: Commands,
    scene: Res<ExampleScene>,
    mut meshes: ResMut<BevyAssets<Mesh>>,
    mut materials: ResMut<BevyAssets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(1.6, 2.0, 2.7).looking_at(Vec3::new(0.0, 1.0, 0.0), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 14_000.0,
            shadow_maps_enabled: true,
            ..Default::default()
        },
        Transform::from_xyz(-4.0, 7.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(12.0, 12.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.11, 0.15, 0.19),
            perceptual_roughness: 0.92,
            ..Default::default()
        })),
        Transform::from_xyz(0.0, -0.02, 0.0),
    ));
    commands.spawn((
        Text::new(format!("{}\n{}", scene.kind.title(), scene.kind.check())),
        TextFont {
            font_size: FontSize::Px(20.0),
            ..Default::default()
        },
        TextColor(Color::WHITE),
        Node {
            position_type: PositionType::Absolute,
            top: px(18),
            left: px(20),
            ..Default::default()
        },
    ));
}

/// Marks one render entity attached to a physics body's collider shape.
#[cfg(feature = "visual")]
#[derive(Clone, Copy, Debug, bevy::prelude::Component)]
struct BodyVisual;

/// Adds one visible mesh to each runtime body after skeleton binding.
#[cfg(feature = "visual")]
fn create_body_visuals(
    mut commands: Commands,
    mut meshes: ResMut<BevyAssets<Mesh>>,
    mut materials: ResMut<BevyAssets<StandardMaterial>>,
    bodies: Query<(Entity, &BodyShape, &bevy_ragdoll::BodyIndex), bevy::prelude::Added<BodyShape>>,
) {
    let palette = [
        Color::srgb(0.18, 0.62, 0.76),
        Color::srgb(0.25, 0.78, 0.64),
        Color::srgb(0.92, 0.66, 0.34),
    ];
    for (body, shape, index) in &bodies {
        let body_index = index.get();
        commands.entity(body).insert((
            Name::new(format!("ragdoll body {body_index}")),
            Visibility::Inherited,
        ));
        let (mesh, local_transform) = visual_shape(&shape.0);
        commands.spawn((
            Mesh3d(meshes.add(mesh)),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: palette[index.get() % palette.len()],
                metallic: 0.02,
                perceptual_roughness: 0.42,
                ..Default::default()
            })),
            BodyVisual,
            local_transform,
            ChildOf(body),
        ));
    }
}

/// Converts a validated collision shape into the corresponding Bevy mesh.
#[cfg(feature = "visual")]
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
            Mesh::from(bevy::prelude::Sphere::new(*radius)),
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

/// Requests one screenshot after the example's visual capture delay.
#[cfg(feature = "visual")]
fn capture_screenshot(
    mut commands: Commands,
    options: Res<ExampleOptions>,
    scene: Res<ExampleScene>,
    time: Res<Time>,
    bodies: Query<(), With<BodyShape>>,
    visuals: Query<(), With<BodyVisual>>,
    mut requested: bevy::prelude::Local<bool>,
) {
    if scene.kind == ExampleKind::HitReactions {
        return;
    }
    let Some(path) = options.screenshot.as_ref() else {
        return;
    };
    let capture_after = if scene.kind == ExampleKind::PartialRagdoll {
        1.5
    } else {
        2.0
    };
    if *requested || time.elapsed_secs() < capture_after || bodies.iter().count() < 3 {
        return;
    }
    if visuals.iter().count() < 3 {
        return;
    }
    if let Some(parent) = path.parent()
        && !parent.as_os_str().is_empty()
        && let Err(error) = std::fs::create_dir_all(parent)
    {
        bevy::log::error!(screenshot_directory_error = %error, "could not create screenshot directory");
        return;
    }
    commands
        .spawn(Screenshot::primary_window())
        .observe(save_to_disk(path.clone()));
    *requested = true;
}

/// Exits after the requested process duration so unattended runs terminate.
fn exit_after(
    options: Res<ExampleOptions>,
    time: Res<Time>,
    mut timer: bevy::prelude::Local<Option<Timer>>,
    mut app_exit: bevy::prelude::MessageWriter<AppExit>,
) {
    let Some(duration) = options.exit_after.map(|exit_after| exit_after.seconds) else {
        return;
    };
    let timer = timer.get_or_insert_with(|| Timer::new(duration, TimerMode::Once));
    timer.tick(time.delta());
    if timer.is_finished() {
        app_exit.write(AppExit::Success);
    }
}

/// Returns a screenshot path under Cargo's configured build directory.
fn default_screenshot_path() -> PathBuf {
    std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("target"))
        .join("screenshots/minimal.png")
}

#[cfg(test)]
/// Tests profile construction and command validation at the examples crate seam.
mod tests {
    use super::{ExampleCli, ExampleKind, ExitAfter, ScreenshotPath, load_profile};
    use clap::Parser;
    use std::path::PathBuf;
    use std::str::FromStr;
    use std::time::Duration;

    /// Every documented source constructs at least one checked profile body.
    #[test]
    fn each_example_profile_loads() {
        for kind in [
            ExampleKind::Minimal,
            ExampleKind::FromCode,
            ExampleKind::FromRon,
            ExampleKind::FromGltfSkein,
            ExampleKind::HitReactions,
            ExampleKind::PartialRagdoll,
        ] {
            assert!(
                !load_profile(kind)
                    .expect("profile validates")
                    .bodies()
                    .is_empty()
            );
        }
    }

    /// The presence-only mode and positive duration parse at the CLI boundary.
    #[test]
    fn headless_cli_accepts_a_positive_duration() {
        let cli = ExampleCli::try_parse_from(["minimal", "--headless", "--exit-after", "3"])
            .expect("valid options parse");
        assert!(cli.headless.is_some());
        assert_eq!(
            cli.exit_after.map(|exit_after| exit_after.seconds),
            Some(Duration::from_secs(3))
        );
    }

    /// The shared runner selects only one of its four documented profile sources.
    #[test]
    fn example_kind_override_uses_the_closed_vocabulary() {
        let cli = ExampleCli::try_parse_from(["minimal", "--kind", "from-code"])
            .expect("closed kind parses");
        assert_eq!(cli.kind, Some(ExampleKind::FromCode));
    }

    /// Zero, non-finite, and malformed durations fail independently.
    #[test]
    fn exit_duration_rejects_invalid_values() {
        for value in ["0", "NaN", "inf", "text"] {
            assert!(ExitAfter::from_str(value).is_err());
        }
    }

    /// Screenshot paths keep supplied spellings and expand the default marker.
    #[test]
    fn screenshot_paths_preserve_explicit_values_and_expand_default() {
        let explicit = ScreenshotPath::from_str("review/minimal.png").expect("path parses");
        assert_eq!(explicit.path, PathBuf::from("review/minimal.png"));

        let default =
            ScreenshotPath::from_str("__default_screenshot_path__").expect("default marker parses");
        assert!(default.path.ends_with("screenshots/minimal.png"));
    }
}

#[cfg(all(test, feature = "visual"))]
/// Tests visual geometry conversion independently from physics contacts.
mod visual_tests {
    use super::visual_shape;
    use bevy::math::Vec3;
    use bevy_ragdoll::ShapeSpec;

    /// Zero-length capsule segments retain an identity rotation.
    #[test]
    fn sphere_like_capsules_use_the_identity_rotation() {
        let shape = ShapeSpec::Capsule {
            a: Vec3::ZERO,
            b: Vec3::ZERO,
            radius: 0.2,
        };
        let (_, transform) = visual_shape(&shape);
        assert_eq!(transform.rotation, bevy::math::Quat::IDENTITY);
    }
}
