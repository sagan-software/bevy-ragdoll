//! Showcase: a stress test and kitchen sink in one scene.
//!
//! Ragdolls rain onto a lit arena. Drag a ragdoll to throw it, click one to hit
//! it with the selected hit profile, and tune the population, muscle strength,
//! gravity, and time scale from the panel while it reports frame rate, fixed
//! step cost, and body count.
//!
//! Controls:
//!
//! - Left drag on a ragdoll: grab and throw it.
//! - Left click on a ragdoll: hit it.
//! - Right drag: orbit the camera. Mouse wheel: zoom.
//!
//! Run with `cargo run --release --example showcase`. Choose a physics backend
//! with `--backend rapier` on native or `?backend=rapier` in the web page URL.

use std::collections::HashMap;

use bevy::app::AnimationSystems;
use bevy::asset::RenderAssetUsages;
use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::light::CascadeShadowConfigBuilder;
use bevy::platform::time::Instant;
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use bevy::window::PrimaryWindow;
use bevy_ragdoll::runtime::body::{BodyMass, BodyPhysicsPose, BodyShape, BodyVelocity};
use bevy_ragdoll::runtime::components::{RagdollBodyOf, RagdollDrive, RagdollMode};
use bevy_ragdoll::runtime::hit::{HitProfile, HitSettings};
use bevy_ragdoll::runtime::messages::{
    HitKind, RagdollHit, RagdollImpulse, RagdollRaycast, RagdollRaycastResponse, RagdollRequestId,
};
use bevy_ragdoll::runtime::sets::{RagdollFixedSystems, RagdollSystems};
use bevy_ragdoll::runtime::settings::RagdollPhysicsSettings;
use bevy_ragdoll::{BodyIndex, Ragdoll, RagdollPlugin, Skeleton};

/// Half the side length of the square arena floor, in metres.
const ARENA_HALF_EXTENT: f32 = 20.0;
/// Checker cells along each floor edge: one per metre of `2 * ARENA_HALF_EXTENT`.
const CHECKER_CELLS: usize = 40;
/// Characters spawned per frame while the population grows, to spread spawn cost.
const SPAWNS_PER_FRAME: usize = 4;
/// Cursor travel in logical pixels that turns a click into a drag.
const DRAG_THRESHOLD_PX: f32 = 6.0;
/// Hit presets the panel cycles through.
const HIT_PROFILES: [HitProfile; 7] = [
    HitProfile::Pistol,
    HitProfile::Rifle,
    HitProfile::Shotgun,
    HitProfile::Punch,
    HitProfile::Kick,
    HitProfile::Heavy,
    HitProfile::Explosion,
];
/// Accent color for panel headings and selected buttons.
const ACCENT: Color = Color::srgb(0.98, 0.62, 0.28);
/// Character colors, assigned round-robin by spawn order.
const PALETTE: [Color; 6] = [
    Color::srgb(0.95, 0.55, 0.30),
    Color::srgb(0.27, 0.70, 0.85),
    Color::srgb(0.40, 0.80, 0.55),
    Color::srgb(0.93, 0.78, 0.35),
    Color::srgb(0.78, 0.48, 0.86),
    Color::srgb(0.90, 0.42, 0.50),
];

/// Physics engines the showcase can run on. Switching restarts the app.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Reflect)]
#[cfg_attr(not(target_arch = "wasm32"), derive(clap::ValueEnum))]
enum Backend {
    /// Rapier through `bevy_ragdoll_rapier3d`.
    Rapier,
    /// Avian through `bevy_ragdoll_avian3d`.
    Avian,
}

impl Backend {
    /// Every backend, in panel order.
    const ALL: [Self; 2] = [Self::Rapier, Self::Avian];

    /// Returns the name used on the command line, in the page URL, and in the panel.
    const fn name(self) -> &'static str {
        match self {
            Self::Rapier => "rapier",
            Self::Avian => "avian",
        }
    }

    /// Reads `--backend NAME` on native or `?backend=NAME` on the web, defaulting to
    /// Rapier.
    fn from_environment() -> Self {
        #[cfg(not(target_arch = "wasm32"))]
        {
            /// Command-line options for the showcase.
            #[derive(clap::Parser)]
            struct Args {
                /// Physics backend to run.
                #[arg(long, value_enum, default_value_t = Backend::Rapier)]
                backend: Backend,
            }
            <Args as clap::Parser>::parse().backend
        }
        #[cfg(target_arch = "wasm32")]
        {
            let search = web_sys::window()
                .and_then(|window| window.location().search().ok())
                .unwrap_or_default();
            Self::ALL
                .into_iter()
                .find(|backend| search.contains(&format!("backend={}", backend.name())))
                .unwrap_or(Self::Rapier)
        }
    }

    /// Restarts the showcase on `self`: reloads the page on the web, re-executes on
    /// native.
    fn restart_into(self) {
        let name = self.name();
        #[cfg(target_arch = "wasm32")]
        if let Some(window) = web_sys::window() {
            let _ = window.location().set_search(&format!("backend={name}"));
        }
        #[cfg(not(target_arch = "wasm32"))]
        if let Ok(exe) = std::env::current_exe()
            && std::process::Command::new(exe)
                .args(["--backend", name])
                .spawn()
                .is_ok()
        {
            std::process::exit(0);
        }
    }

    /// Adds the physics engine and its ragdoll adapter, both stepping at 60 Hz.
    fn add_plugins(self, app: &mut App) {
        match self {
            Self::Rapier => {
                use bevy_ragdoll_rapier3d::{RapierRagdollHooks, RapierRagdollPlugin};
                use bevy_rapier3d::plugin::{RapierPhysicsPlugin, TimestepMode};
                app.insert_resource(TimestepMode::Fixed {
                    dt: 1.0 / 60.0,
                    substeps: 1,
                })
                .add_plugins((
                    RapierPhysicsPlugin::<RapierRagdollHooks<'_, '_>>::default()
                        .in_fixed_schedule(),
                    RapierRagdollPlugin,
                ));
            }
            Self::Avian => {
                use avian3d::prelude::PhysicsPlugins;
                use bevy_ragdoll_avian3d::{AvianRagdollHooks, AvianRagdollPlugin};
                app.add_plugins((
                    PhysicsPlugins::new(FixedUpdate)
                        .with_collision_hooks::<AvianRagdollHooks<'_, '_>>(),
                    AvianRagdollPlugin,
                ));
            }
        }
    }

    /// Adds a fixed box collider with the given half extents in metres to `entity`.
    fn insert_static_box(self, entity: &mut EntityCommands<'_>, half_extents: Vec3) {
        match self {
            Self::Rapier => {
                use bevy_rapier3d::prelude::{Collider, RigidBody};
                entity.insert((
                    RigidBody::Fixed,
                    Collider::cuboid(half_extents.x, half_extents.y, half_extents.z),
                ));
            }
            Self::Avian => {
                use avian3d::prelude::RigidBody;
                use bevy_ragdoll::ShapeSpec;
                entity.insert((
                    RigidBody::Static,
                    bevy_ragdoll_avian3d::collider_for_shape(ShapeSpec::Cuboid {
                        center: Vec3::ZERO,
                        rotation: Quat::IDENTITY,
                        half_extents,
                    }),
                ));
            }
        }
    }
}

/// The running backend.
#[derive(Resource, Clone, Copy, Reflect)]
struct ActiveBackend(Backend);

/// Skeletons the crowd is built from. Each `Ragdoll` generates its profile from its
/// skeleton.
#[derive(Resource, Reflect)]
struct Rigs {
    /// The reference humanoid, spawned from code.
    #[reflect(ignore)]
    humanoid: Skeleton,
    /// Skinned glTF creatures with no authored ragdoll data: a quadruped and a
    /// seven-legged alien.
    creatures: [Handle<WorldAsset>; 2],
}

/// Values the panel edits.
#[derive(Resource, Reflect)]
struct Params {
    /// Target number of ragdolls.
    count: usize,
    /// Joint motor multiplier from 0 (limp) to 1 (holds the rest pose).
    muscle: f32,
    /// Downward gravity in metres per second squared.
    gravity: f32,
    /// Virtual time speed multiplier.
    time_scale: f32,
    /// Position in `HIT_PROFILES` of the hit applied by a click.
    hit: usize,
    /// Whether every fourth and fifth ragdoll is a glTF creature instead of a humanoid.
    has_creatures: bool,
}

impl Default for Params {
    fn default() -> Self {
        Self {
            count: 24,
            muscle: 0.3,
            gravity: 9.81,
            time_scale: 1.0,
            hit: 2,
            has_creatures: true,
        }
    }
}

impl Params {
    /// Returns the hit preset a click applies.
    fn hit_profile(&self) -> HitProfile {
        HIT_PROFILES
            .get(self.hit)
            .copied()
            .unwrap_or(HitProfile::Shotgun)
    }
}

/// A panel value the user can step up or down.
#[derive(Clone, Copy, PartialEq, Eq, Reflect)]
enum Param {
    /// `Params::count`.
    Count,
    /// `Params::muscle`.
    Muscle,
    /// `Params::gravity`.
    Gravity,
    /// `Params::time_scale`.
    TimeScale,
    /// `Params::hit`.
    Hit,
    /// `Params::has_creatures`.
    Creatures,
}

impl Param {
    /// Every parameter, in panel order.
    const ALL: [Self; 6] = [
        Self::Count,
        Self::Muscle,
        Self::Gravity,
        Self::TimeScale,
        Self::Hit,
        Self::Creatures,
    ];

    /// Returns the panel label.
    const fn label(self) -> &'static str {
        match self {
            Self::Count => "Ragdolls",
            Self::Muscle => "Muscle",
            Self::Gravity => "Gravity",
            Self::TimeScale => "Time scale",
            Self::Hit => "Hit",
            Self::Creatures => "Creatures",
        }
    }

    /// Moves the value one step in the direction of `sign` and keeps it in range.
    fn step(self, params: &mut Params, sign: f32) {
        match self {
            Self::Count => {
                let step = if params.count >= 64 { 16 } else { 4 };
                params.count = if sign > 0.0 {
                    (params.count + step).min(256)
                } else {
                    params.count.saturating_sub(step).max(1)
                };
            }
            Self::Muscle => params.muscle = 0.1f32.mul_add(sign, params.muscle).clamp(0.0, 1.0),
            Self::Gravity => params.gravity = (params.gravity + sign).clamp(0.0, 30.0),
            Self::TimeScale => {
                params.time_scale = 0.1f32.mul_add(sign, params.time_scale).clamp(0.1, 2.0);
            }
            Self::Hit => {
                let len = HIT_PROFILES.len();
                params.hit = (params.hit + if sign > 0.0 { 1 } else { len - 1 }) % len;
            }
            Self::Creatures => params.has_creatures = !params.has_creatures,
        }
    }

    /// Formats the current value for the panel.
    fn value(self, params: &Params) -> String {
        match self {
            Self::Count => params.count.to_string(),
            Self::Muscle => format!("{:.0}%", params.muscle * 100.0),
            Self::Gravity => format!("{:.1} m/s2", params.gravity),
            Self::TimeScale => format!("{:.1}x", params.time_scale),
            Self::Hit => format!("{:?}", params.hit_profile()),
            Self::Creatures => if params.has_creatures { "Mixed" } else { "Off" }.to_owned(),
        }
    }
}

/// A panel button and what it does.
#[derive(Component, Clone, Copy, Reflect)]
enum Action {
    /// Steps a parameter down (`-1`) or up (`+1`).
    Step(Param, i8),
    /// Restarts on another backend.
    UseBackend(Backend),
    /// Despawns every ragdoll so the population respawns from the sky.
    Reset,
    /// Hits every ragdoll outward from the arena centre.
    Explode,
}

/// Marks the text showing one parameter's value.
#[derive(Component, Reflect)]
struct ParamValue(Param);

/// Marks the performance readout text.
#[derive(Component, Reflect)]
struct MetricsText;

/// Marks a character root and records its spawn order for coloring.
#[derive(Component, Reflect)]
struct Character(usize);

/// A skeleton bone and its local rest rotation, restored each frame as the drive
/// target.
#[derive(Component, Reflect)]
struct RestRotation(Quat);

/// Remaining seconds of a body's hit highlight; holds the body's normal material.
#[derive(Component, Reflect)]
struct HitFlash {
    /// Seconds left before the normal material returns.
    remaining: f32,
    /// Material to restore.
    normal: Handle<StandardMaterial>,
}

/// Meshes and materials shared by every ragdoll.
#[derive(Resource, Default, Reflect)]
struct BodyAssets {
    /// One mesh per distinct collider shape, keyed by its debug text, built on first
    /// use.
    meshes: HashMap<String, (Handle<Mesh>, Transform)>,
    /// One material per palette color.
    materials: Vec<Handle<StandardMaterial>>,
    /// Bright material shown briefly on a hit body.
    flash: Handle<StandardMaterial>,
}

/// Camera orbit around a focus point.
#[derive(Resource, Reflect)]
struct Orbit {
    /// Rotation about the vertical axis, in radians.
    yaw: f32,
    /// Elevation above the horizon, in radians.
    pitch: f32,
    /// Distance from the focus, in metres.
    distance: f32,
}

/// Mouse state for click-to-hit and drag-to-throw.
#[derive(Resource, Default, Reflect)]
struct PointerState {
    /// Cursor position when the left button went down.
    press: Option<Vec2>,
    /// Identity of the newest ray request.
    next_request: u64,
    /// Ray request sent on the latest press, waiting for its response.
    pending: Option<u64>,
    /// The body under the cursor at press time.
    target: Option<Target>,
    /// Whether the cursor moved far enough to turn the press into a drag.
    is_dragging: bool,
}

/// A body picked by the cursor.
#[derive(Clone, Copy, Reflect)]
struct Target {
    /// The picked ragdoll body.
    body: Entity,
    /// Picked point in the body's local frame, in metres.
    local_point: Vec3,
    /// Distance from the camera to the picked point, in metres.
    depth: f32,
    /// Ray direction at press time, used as the hit direction.
    direction: Vec3,
    /// World point the grabbed point is pulled toward, in metres.
    goal: Vec3,
}

/// Fixed-step timing, accumulated over the frames between panel updates.
#[derive(Resource, Default, Reflect)]
struct StepTimer {
    /// Start of the fixed step in progress.
    #[reflect(ignore)]
    started: Option<Instant>,
    /// Summed fixed-step seconds since the last readout.
    total: f32,
    /// Fixed steps since the last readout.
    steps: u16,
    /// Smoothed milliseconds per fixed step.
    average_ms: f32,
}

/// Loads the profile, picks a backend, and runs the showcase.
fn main() -> AppExit {
    let backend = Backend::from_environment();

    let mut app = App::new();
    // Web servers answer 404 for the `.meta` files Bevy probes by default; the rigs have none.
    let assets = AssetPlugin {
        meta_check: bevy::asset::AssetMetaCheck::Never,
        ..default()
    };
    app.add_plugins(DefaultPlugins.set(assets).set(WindowPlugin {
        primary_window: Some(Window {
            title: "bevy_ragdoll showcase".into(),
            resolution: (1440, 860).into(),
            fit_canvas_to_parent: true,
            ..default()
        }),
        ..default()
    }))
    .add_plugins((
        RagdollPlugin::default(),
        FrameTimeDiagnosticsPlugin::default(),
    ))
    .insert_resource(Time::<Fixed>::from_hz(60.0))
    .insert_resource(ClearColor(Color::srgb(0.06, 0.07, 0.09)))
    .insert_resource(GlobalAmbientLight {
        color: Color::srgb(0.7, 0.8, 1.0),
        brightness: 350.0,
        ..default()
    })
    .insert_resource(ActiveBackend(backend))
    .insert_resource(Orbit {
        yaw: 0.6,
        pitch: 0.38,
        distance: 17.0,
    })
    .init_resource::<Params>()
    .init_resource::<PointerState>()
    .init_resource::<StepTimer>()
    .init_resource::<BodyAssets>();
    backend.add_plugins(&mut app);
    // Registration makes the example's state visible to reflection tools such as inspectors.
    app.register_type::<ActiveBackend>()
        .register_type::<Rigs>()
        .register_type::<Params>()
        .register_type::<Action>()
        .register_type::<ParamValue>()
        .register_type::<MetricsText>()
        .register_type::<Character>()
        .register_type::<RestRotation>()
        .register_type::<HitFlash>()
        .register_type::<BodyAssets>()
        .register_type::<Orbit>()
        .register_type::<PointerState>()
        .register_type::<StepTimer>();
    app.add_systems(Startup, (setup_scene, setup_assets, load_rigs, spawn_panel))
        .add_systems(
            Update,
            (
                handle_buttons,
                apply_params,
                sync_population,
                orbit_camera,
                (press_pointer, read_pick, release_pointer).chain(),
                fade_flashes,
                update_panel,
            ),
        )
        .add_systems(
            PostUpdate,
            (
                restore_rest_pose
                    .after(AnimationSystems)
                    .before(RagdollSystems::CaptureTargets),
                add_body_meshes
                    .after(RagdollSystems::Bind)
                    .before(TransformSystems::Propagate),
            ),
        )
        .add_systems(FixedFirst, start_step_timer)
        .add_systems(FixedLast, finish_step_timer)
        .add_systems(
            FixedUpdate,
            pull_grabbed_body.before(RagdollFixedSystems::Behaviour),
        )
        .run()
}

/// Builds the humanoid skeleton and starts loading the glTF creatures.
fn load_rigs(mut commands: Commands<'_, '_>, assets: Res<'_, AssetServer>) {
    let creature =
        |name: &str| assets.load(GltfAssetLabel::Scene(0).from_asset(format!("rigs/{name}.glb")));
    commands.insert_resource(Rigs {
        humanoid: Skeleton::humanoid(),
        creatures: [creature("quadruped"), creature("alien")],
    });
}

/// Spawns the camera, lights, checkered floor, and a few obstacles.
fn setup_scene(
    mut commands: Commands<'_, '_>,
    backend: Res<'_, ActiveBackend>,
    mut meshes: ResMut<'_, Assets<Mesh>>,
    mut materials: ResMut<'_, Assets<StandardMaterial>>,
    mut images: ResMut<'_, Assets<Image>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::default(),
        DistanceFog {
            color: Color::srgb(0.06, 0.07, 0.09),
            falloff: FogFalloff::Linear {
                start: 30.0,
                end: 90.0,
            },
            ..default()
        },
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 9_000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        CascadeShadowConfigBuilder {
            maximum_distance: 60.0,
            first_cascade_far_bound: 12.0,
            ..default()
        }
        .build(),
        Transform::from_xyz(-8.0, 14.0, 6.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // The floor and obstacles carry the backend's static colliders.
    let floor_half = Vec3::new(ARENA_HALF_EXTENT, 0.25, ARENA_HALF_EXTENT);
    let mut floor = commands.spawn((
        Mesh3d(meshes.add(Cuboid::from_size(floor_half * 2.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color_texture: Some(images.add(checker_image())),
            perceptual_roughness: 0.9,
            ..default()
        })),
        Transform::from_xyz(0.0, -0.25, 0.0),
    ));
    backend.0.insert_static_box(&mut floor, floor_half);

    let obstacle = materials.add(StandardMaterial {
        base_color: Color::srgb(0.22, 0.25, 0.30),
        perceptual_roughness: 0.6,
        ..default()
    });
    let obstacles = [
        (
            Vec3::new(1.0, 0.5, 1.0),
            Transform::from_xyz(-5.0, 0.5, -3.0),
        ),
        (Vec3::new(0.6, 1.0, 0.6), Transform::from_xyz(4.0, 1.0, 4.0)),
        (
            Vec3::new(3.0, 0.15, 1.5),
            Transform::from_xyz(6.0, 0.9, -5.0).with_rotation(Quat::from_rotation_z(0.3)),
        ),
    ];
    for (half_extents, transform) in obstacles {
        let mut entity = commands.spawn((
            Mesh3d(meshes.add(Cuboid::from_size(half_extents * 2.0))),
            MeshMaterial3d(obstacle.clone()),
            transform,
        ));
        backend.0.insert_static_box(&mut entity, half_extents);
    }
}

/// Builds a two-tone checker texture with one-metre cells across the floor.
fn checker_image() -> Image {
    let cells = CHECKER_CELLS;
    let cell_px = 16;
    let size = cells * cell_px;
    let mut data = Vec::with_capacity(size * size * 4);
    for y in 0..size {
        for x in 0..size {
            let is_light_cell = (x / cell_px + y / cell_px) % 2 == 0;
            let value = if is_light_cell { 58 } else { 46 };
            data.extend_from_slice(&[value, value + 4, value + 10, 255]);
        }
    }
    let edge_px = u32::try_from(size).expect("the checker fits in u32");
    Image::new(
        Extent3d {
            width: edge_px,
            height: edge_px,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        data,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::RENDER_WORLD,
    )
}

/// Creates the shared character and hit-flash materials.
fn setup_assets(
    mut assets: ResMut<'_, BodyAssets>,
    mut materials: ResMut<'_, Assets<StandardMaterial>>,
) {
    assets.materials = PALETTE
        .iter()
        .map(|&color| {
            materials.add(StandardMaterial {
                base_color: color,
                perceptual_roughness: 0.45,
                ..default()
            })
        })
        .collect();
    assets.flash = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        emissive: LinearRgba::rgb(6.0, 4.0, 2.0),
        ..default()
    });
}

/// Spawns or despawns characters until the population matches `Params::count`.
///
/// Characters that fall off the arena are despawned and replaced.
#[expect(
    clippy::cast_precision_loss,
    reason = "population and spawn counts stay far below 2^24"
)]
fn sync_population(
    mut commands: Commands<'_, '_>,
    params: Res<'_, Params>,
    rigs: Option<Res<'_, Rigs>>,
    mut spawned: Local<'_, usize>,
    characters: Query<'_, '_, (Entity, &GlobalTransform), With<Character>>,
) {
    let mut alive = 0;
    for (entity, transform) in &characters {
        if transform.translation().y < -20.0 || alive >= params.count {
            commands.entity(entity).despawn();
        } else {
            alive += 1;
        }
    }
    let Some(rigs) = rigs else { return };
    let radius = (params.count as f32).sqrt().mul_add(0.9, 2.0);
    for _ in alive..params.count.min(alive + SPAWNS_PER_FRAME) {
        let index = *spawned;
        *spawned += 1;
        // A golden-angle spiral spreads drops evenly without a random number generator.
        let angle = index as f32 * 2.399_963;
        let distance = radius * ((index % 64) as f32 / 64.0).sqrt();
        let position = Vec3::new(
            angle.cos() * distance,
            ((index % 5) as f32).mul_add(0.9, 1.0),
            angle.sin() * distance,
        );
        spawn_character(&mut commands, &rigs, &params, index, position, angle);
    }
}

/// Spawns one dynamic character: a humanoid from code or, when enabled, a glTF
/// creature.
///
/// `Ragdoll::default()` generates the ragdoll profile from whatever skeleton is
/// under it.
fn spawn_character(
    commands: &mut Commands<'_, '_>,
    rigs: &Rigs,
    params: &Params,
    index: usize,
    position: Vec3,
    yaw: f32,
) {
    let mut character = commands.spawn((
        Name::new(format!("ragdoll {index}")),
        Character(index),
        Ragdoll::default(),
        RagdollMode::Dynamic,
        RagdollDrive::new(params.muscle, 0.0),
        Transform::from_translation(position).with_rotation(Quat::from_rotation_y(yaw)),
    ));
    let creature = params
        .has_creatures
        .then(|| rigs.creatures.get((index % 5).checked_sub(3)?))
        .flatten();
    if let Some(scene) = creature {
        character.insert(WorldAssetRoot(scene.clone()));
    } else {
        let character = character.id();
        rigs.humanoid.spawn(commands, character);
    }
}

/// Filter for named child entities (skeleton bones) whose rest rotation is not yet
/// recorded.
type NewBone = (With<Name>, With<ChildOf>, Without<RestRotation>);

/// Puts each bone back at its rest rotation so the muscles pull toward the rest
/// pose.
///
/// Writeback copies physics poses into the bones after capture, so without this
/// the captured target would equal the current pose and the muscles would idle.
fn restore_rest_pose(
    mut commands: Commands<'_, '_>,
    new_bones: Query<'_, '_, (Entity, &Transform), NewBone>,
    mut bones: Query<'_, '_, (&RestRotation, &mut Transform)>,
) {
    // Named children are skeleton bones, from code or from a glTF scene; remember their spawn pose.
    for (entity, transform) in &new_bones {
        commands
            .entity(entity)
            .insert(RestRotation(transform.rotation));
    }
    for (rest, mut transform) in &mut bones {
        transform.rotation = rest.0;
    }
}

/// Gives each new physics body a shared mesh matching its collider, in its
/// character's color.
fn add_body_meshes(
    mut commands: Commands<'_, '_>,
    mut assets: ResMut<'_, BodyAssets>,
    mut meshes: ResMut<'_, Assets<Mesh>>,
    bodies: Query<'_, '_, (Entity, &BodyShape, &RagdollBodyOf), Added<BodyShape>>,
    characters: Query<'_, '_, &Character>,
) {
    for (entity, shape, owner) in &bodies {
        let (mesh, transform) = assets
            .meshes
            .entry(format!("{:?}", shape.0))
            .or_insert_with(|| {
                let (mesh, transform) = shape.0.mesh();
                (meshes.add(mesh), transform)
            })
            .clone();
        let color = characters.get(owner.0).map_or(0, |character| character.0);
        let material = assets
            .materials
            .iter()
            .cycle()
            .nth(color)
            .cloned()
            .unwrap_or_default();
        commands.entity(entity).insert(Visibility::Inherited);
        commands.spawn((
            Mesh3d(mesh),
            MeshMaterial3d(material),
            transform,
            ChildOf(entity),
        ));
    }
}

/// Spawns the control panel: metrics, parameter steppers, backend choice, and
/// actions.
fn spawn_panel(
    mut commands: Commands<'_, '_>,
    backend: Res<'_, ActiveBackend>,
    params: Res<'_, Params>,
) {
    let panel = commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                top: px(14),
                left: px(14),
                width: px(310),
                flex_direction: FlexDirection::Column,
                row_gap: px(6),
                padding: UiRect::all(px(14)),
                border_radius: BorderRadius::all(px(8)),
                ..default()
            },
            BackgroundColor(Color::srgba(0.04, 0.05, 0.07, 0.86)),
        ))
        .id();
    commands.spawn((text("bevy_ragdoll", 18.0, ACCENT), ChildOf(panel)));
    commands.spawn((
        text("", 13.0, Color::srgb(0.82, 0.86, 0.9)),
        MetricsText,
        ChildOf(panel),
    ));
    for param in Param::ALL {
        let row = commands.spawn((row_node(), ChildOf(panel))).id();
        commands.spawn((
            text(param.label(), 14.0, Color::WHITE),
            Node {
                flex_grow: 1.0,
                ..default()
            },
            ChildOf(row),
        ));
        button(&mut commands, row, "-", Action::Step(param, -1), false);
        commands.spawn((
            text(&param.value(&params), 14.0, Color::WHITE),
            Node {
                width: px(100),
                justify_content: JustifyContent::Center,
                ..default()
            },
            TextLayout::justify(Justify::Center),
            ParamValue(param),
            ChildOf(row),
        ));
        button(&mut commands, row, "+", Action::Step(param, 1), false);
    }
    let row = commands.spawn((row_node(), ChildOf(panel))).id();
    commands.spawn((
        text("Backend", 14.0, Color::WHITE),
        Node {
            flex_grow: 1.0,
            ..default()
        },
        ChildOf(row),
    ));
    for option in Backend::ALL {
        let is_selected = option == backend.0;
        button(
            &mut commands,
            row,
            option.name(),
            Action::UseBackend(option),
            is_selected,
        );
    }
    let row = commands.spawn((row_node(), ChildOf(panel))).id();
    button(&mut commands, row, "Explode", Action::Explode, false);
    button(&mut commands, row, "Reset", Action::Reset, false);

    commands.spawn((
        text(
            "Drag: throw   Click: hit   Right-drag: orbit   Wheel: zoom",
            13.0,
            Color::srgba(1.0, 1.0, 1.0, 0.7),
        ),
        Node {
            position_type: PositionType::Absolute,
            bottom: px(14),
            left: px(14),
            ..default()
        },
    ));
}

/// Returns a horizontal panel row.
fn row_node() -> Node {
    Node {
        align_items: AlignItems::Center,
        column_gap: px(6),
        ..default()
    }
}

/// Returns a text bundle with the given size and color.
fn text(value: &str, size: f32, color: Color) -> impl Bundle {
    (
        Text::new(value),
        TextFont {
            font_size: FontSize::Px(size),
            ..default()
        },
        TextColor(color),
    )
}

/// Spawns a labelled button in `row` that performs `action` when pressed.
fn button(
    commands: &mut Commands<'_, '_>,
    row: Entity,
    label: &str,
    action: Action,
    is_selected: bool,
) {
    let background = if is_selected {
        ACCENT.with_alpha(0.35)
    } else {
        Color::srgba(1.0, 1.0, 1.0, 0.08)
    };
    let button = commands
        .spawn((
            Button,
            action,
            Node {
                min_width: px(26),
                padding: UiRect::axes(px(8), px(3)),
                justify_content: JustifyContent::Center,
                border_radius: BorderRadius::all(px(5)),
                ..default()
            },
            BackgroundColor(background),
            ChildOf(row),
        ))
        .id();
    commands.spawn((text(label, 14.0, Color::WHITE), ChildOf(button)));
}

/// Applies pressed panel buttons.
fn handle_buttons(
    mut commands: Commands<'_, '_>,
    buttons: Query<'_, '_, (&Interaction, &Action), Changed<Interaction>>,
    backend: Res<'_, ActiveBackend>,
    settings: Res<'_, HitSettings>,
    mut params: ResMut<'_, Params>,
    characters: Query<'_, '_, Entity, With<Character>>,
    bodies: Query<'_, '_, (Entity, &BodyIndex, &GlobalTransform), With<RagdollBodyOf>>,
    mut hits: MessageWriter<'_, RagdollHit>,
) {
    for (interaction, action) in &buttons {
        if *interaction != Interaction::Pressed {
            continue;
        }
        match *action {
            Action::Step(param, sign) => param.step(&mut params, f32::from(sign)),
            Action::UseBackend(choice) if choice != backend.0 => choice.restart_into(),
            Action::UseBackend(_) => {}
            Action::Reset => characters
                .iter()
                .for_each(|entity| commands.entity(entity).despawn()),
            Action::Explode => {
                let magnitude = settings
                    .impulse_magnitude(HitProfile::Explosion)
                    .unwrap_or_default();
                // Hit each pelvis (body 0) outward and upward from the arena centre.
                for (body, _, transform) in bodies.iter().filter(|(_, index, _)| index.get() == 0) {
                    let point = transform.translation();
                    let outward = (point * Vec3::new(1.0, 0.0, 1.0)).normalize_or(Vec3::X);
                    hits.write(RagdollHit {
                        body,
                        point,
                        impulse: (outward + Vec3::Y * 1.2).normalize() * magnitude,
                        kind: HitKind::Environmental,
                    });
                }
            }
        }
    }
}

/// Copies changed parameters to the drive, physics settings, and virtual clock.
fn apply_params(
    params: Res<'_, Params>,
    mut settings: ResMut<'_, RagdollPhysicsSettings>,
    mut time: ResMut<'_, Time<Virtual>>,
    mut drives: Query<'_, '_, &mut RagdollDrive>,
) {
    if !params.is_changed() {
        return;
    }
    for mut drive in &mut drives {
        *drive = RagdollDrive::new(params.muscle, 0.0);
    }
    // The backend adapter copies this gravity to its engine every fixed step.
    settings.gravity = Vec3::NEG_Y * params.gravity;
    time.set_relative_speed(params.time_scale);
}

/// Orbits the camera on right drag and zooms on the mouse wheel.
fn orbit_camera(
    buttons: Res<'_, ButtonInput<MouseButton>>,
    motion: Res<'_, AccumulatedMouseMotion>,
    scroll: Res<'_, AccumulatedMouseScroll>,
    mut orbit: ResMut<'_, Orbit>,
    mut cameras: Query<'_, '_, &mut Transform, With<Camera3d>>,
) {
    if buttons.pressed(MouseButton::Right) {
        orbit.yaw = motion.delta.x.mul_add(-0.005, orbit.yaw);
        orbit.pitch = motion.delta.y.mul_add(0.005, orbit.pitch).clamp(0.05, 1.45);
    }
    orbit.distance = (orbit.distance * scroll.delta.y.mul_add(-0.08, 1.0)).clamp(4.0, 60.0);
    let focus = Vec3::new(0.0, 1.0, 0.0);
    let offset = Quat::from_euler(EulerRot::YXZ, orbit.yaw, -orbit.pitch, 0.0)
        * Vec3::new(0.0, 0.0, orbit.distance);
    for mut transform in &mut cameras {
        *transform = Transform::from_translation(focus + offset).looking_at(focus, Vec3::Y);
    }
}

/// Returns the camera ray under the cursor.
fn cursor_ray(
    windows: &Query<'_, '_, &Window, With<PrimaryWindow>>,
    cameras: &Query<'_, '_, (&Camera, &GlobalTransform)>,
) -> Option<(Vec2, Ray3d)> {
    let cursor = windows.single().ok()?.cursor_position()?;
    let (camera, transform) = cameras.single().ok()?;
    Some((cursor, camera.viewport_to_world(transform, cursor).ok()?))
}

/// Sends a backend ray on left press, and moves the grab goal while dragging.
fn press_pointer(
    buttons: Res<'_, ButtonInput<MouseButton>>,
    interactions: Query<'_, '_, &Interaction>,
    windows: Query<'_, '_, &Window, With<PrimaryWindow>>,
    cameras: Query<'_, '_, (&Camera, &GlobalTransform)>,
    mut pointer: ResMut<'_, PointerState>,
    mut rays: MessageWriter<'_, RagdollRaycast>,
) {
    let Some((cursor, ray)) = cursor_ray(&windows, &cameras) else {
        return;
    };
    let is_over_panel = interactions.iter().any(|i| *i != Interaction::None);
    if buttons.just_pressed(MouseButton::Left) && !is_over_panel {
        let id = pointer.next_request;
        pointer.next_request = id.wrapping_add(1);
        pointer.press = Some(cursor);
        pointer.pending = Some(id);
        pointer.target = None;
        pointer.is_dragging = false;
        rays.write(RagdollRaycast {
            request_id: RagdollRequestId::new(id),
            origin: ray.origin,
            direction: *ray.direction,
            max_distance: 200.0,
            filter: None,
        });
    }
    if let Some(press) = pointer.press
        && buttons.pressed(MouseButton::Left)
    {
        pointer.is_dragging |= press.distance(cursor) > DRAG_THRESHOLD_PX;
        if let Some(target) = pointer.target.as_mut() {
            target.goal = ray.origin + *ray.direction * target.depth;
        }
    }
}

/// Records the ragdoll body hit by the press ray, if any.
fn read_pick(
    mut responses: MessageReader<'_, '_, RagdollRaycastResponse>,
    mut pointer: ResMut<'_, PointerState>,
    bodies: Query<'_, '_, &GlobalTransform>,
    cameras: Query<'_, '_, &GlobalTransform, With<Camera3d>>,
) {
    for response in responses.read() {
        if pointer.pending != Some(response.request_id.get()) {
            continue;
        }
        pointer.pending = None;
        let Some(hit) = response.hit else { continue };
        let (Some(body), Ok(camera)) = (hit.body, cameras.single()) else {
            continue;
        };
        let Ok(body_transform) = bodies.get(body) else {
            continue;
        };
        let camera = camera.translation();
        pointer.target = Some(Target {
            body,
            local_point: body_transform
                .affine()
                .inverse()
                .transform_point3(hit.point),
            depth: camera.distance(hit.point),
            direction: (hit.point - camera).normalize_or(Vec3::NEG_Z),
            goal: hit.point,
        });
    }
}

/// On left release, hits the picked body if the cursor did not drag; a drag just
/// lets go.
fn release_pointer(
    mut commands: Commands<'_, '_>,
    buttons: Res<'_, ButtonInput<MouseButton>>,
    params: Res<'_, Params>,
    settings: Res<'_, HitSettings>,
    assets: Res<'_, BodyAssets>,
    mut pointer: ResMut<'_, PointerState>,
    bodies: Query<'_, '_, (&GlobalTransform, &Children)>,
    visuals: Query<'_, '_, &MeshMaterial3d<StandardMaterial>, Without<HitFlash>>,
    mut hits: MessageWriter<'_, RagdollHit>,
) {
    if !buttons.just_released(MouseButton::Left) {
        return;
    }
    pointer.press = None;
    let Some(target) = pointer.target.take() else {
        return;
    };
    if pointer.is_dragging {
        return;
    }
    let Ok((transform, children)) = bodies.get(target.body) else {
        return;
    };
    let magnitude = settings
        .impulse_magnitude(params.hit_profile())
        .unwrap_or_default();
    hits.write(RagdollHit {
        body: target.body,
        point: transform.transform_point(target.local_point),
        impulse: target.direction * magnitude,
        kind: HitKind::Impact,
    });
    // Flash the hit body so the reaction is easy to spot among many ragdolls.
    for child in children {
        if let Ok(material) = visuals.get(*child) {
            commands.entity(*child).insert((
                HitFlash {
                    remaining: 0.25,
                    normal: material.0.clone(),
                },
                MeshMaterial3d(assets.flash.clone()),
            ));
        }
    }
}

/// Restores each flashed body's material once its highlight expires.
fn fade_flashes(
    mut commands: Commands<'_, '_>,
    time: Res<'_, Time<Real>>,
    mut flashes: Query<'_, '_, (Entity, &mut HitFlash)>,
) {
    for (entity, mut flash) in &mut flashes {
        flash.remaining -= time.delta_secs();
        if flash.remaining <= 0.0 {
            commands
                .entity(entity)
                .insert(MeshMaterial3d(flash.normal.clone()))
                .remove::<HitFlash>();
        }
    }
}

/// Pulls the dragged body toward the cursor with a velocity-matching impulse each
/// step.
///
/// The impulse also carries most of the character's weight, so the grabbed
/// ragdoll dangles instead of stretching one limb. Releasing keeps its velocity,
/// which throws it.
fn pull_grabbed_body(
    time: Res<'_, Time<Fixed>>,
    pointer: Res<'_, PointerState>,
    settings: Res<'_, RagdollPhysicsSettings>,
    bodies: Query<'_, '_, (&BodyPhysicsPose, &BodyVelocity, &BodyMass, &RagdollBodyOf)>,
    masses: Query<'_, '_, (&BodyMass, &RagdollBodyOf)>,
    mut impulses: MessageWriter<'_, RagdollImpulse>,
) {
    let Some(target) = pointer.target.filter(|_| pointer.is_dragging) else {
        return;
    };
    let Ok((pose, velocity, mass, owner)) = bodies.get(target.body) else {
        return;
    };
    let character_mass: f32 = masses
        .iter()
        .filter(|(_, of)| of.0 == owner.0)
        .map(|(mass, _)| mass.mass)
        .sum();
    let point: Vec3 = pose.current.transform_point(target.local_point).into();
    let desired = ((target.goal - point) * 10.0).clamp_length_max(25.0);
    let dt = time.delta_secs();
    let impulse = (desired - velocity.linear) * mass.mass * 0.8
        - settings.gravity * character_mass * dt * 0.9;
    impulses.write(RagdollImpulse {
        body: target.body,
        point,
        impulse,
    });
}

/// Marks the start of a fixed step.
fn start_step_timer(mut timer: ResMut<'_, StepTimer>) {
    timer.started = Some(Instant::now());
}

/// Adds the finished fixed step's wall-clock duration to the running total.
fn finish_step_timer(mut timer: ResMut<'_, StepTimer>) {
    if let Some(started) = timer.started.take() {
        timer.total += started.elapsed().as_secs_f32();
        timer.steps = timer.steps.saturating_add(1);
    }
}

/// Refreshes parameter values every frame and the metrics twice a second.
fn update_panel(
    params: Res<'_, Params>,
    real: Res<'_, Time<Real>>,
    diagnostics: Res<'_, DiagnosticsStore>,
    backend: Res<'_, ActiveBackend>,
    mut timer: ResMut<'_, StepTimer>,
    mut since_refresh: Local<'_, f32>,
    characters: Query<'_, '_, (), With<Character>>,
    bodies: Query<'_, '_, (), With<RagdollBodyOf>>,
    mut values: Query<'_, '_, (&ParamValue, &mut Text), Without<MetricsText>>,
    mut metrics: Query<'_, '_, &mut Text, With<MetricsText>>,
) {
    if params.is_changed() {
        for (value, mut text) in &mut values {
            **text = value.0.value(&params);
        }
    }
    *since_refresh += real.delta_secs();
    if *since_refresh < 0.5 {
        return;
    }
    *since_refresh = 0.0;
    if timer.steps > 0 {
        timer.average_ms = timer.total * 1000.0 / f32::from(timer.steps);
    }
    (timer.total, timer.steps) = (0.0, 0);
    let fps = diagnostics
        .get(&FrameTimeDiagnosticsPlugin::FPS)
        .and_then(bevy::diagnostic::Diagnostic::smoothed)
        .unwrap_or_default();
    let (ragdolls, body_count) = (characters.iter().count(), bodies.iter().count());
    let (step, backend) = (timer.average_ms, backend.0.name());
    for mut text in &mut metrics {
        **text = format!(
            "{fps:.0} FPS   step {step:.2} ms\n{ragdolls} ragdolls   {body_count} bodies   {backend}"
        );
    }
}
