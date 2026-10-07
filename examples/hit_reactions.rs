//! Hit reactions on an actively driven ragdoll.
//!
//! The rig is the reference humanoid skeleton with a generated profile.
//! Every body follows a procedural idle pose at full muscle strength, and
//! the pelvis and chest are pinned to their animated targets. A rifle hit
//! lands on the chest shortly after startup.
//!
//! Controls:
//!
//! - Left click: hit the body under the cursor.
//! - `1` to `7`: select pistol, rifle, shotgun, punch, kick, heavy, or explosion.
//! - `-` and `=`: decrease or increase the hit impulse.
//! - `M` and `P`: edit muscle or pin strength.
//! - `Tab`: select the next body. Pin editing skips unpinned bodies.
//! - Up and Down: raise or lower the selected strength.

use std::collections::HashMap;

use bevy::app::AnimationSystems;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_ragdoll::runtime::backend::RayHit;
use bevy_ragdoll::runtime::components::{
    BodyWeights, Ragdoll, RagdollBodyOf, RagdollBodyWeights, RagdollDrive, RagdollMode,
};
use bevy_ragdoll::runtime::hit::{HitProfile, HitSettings};
use bevy_ragdoll::runtime::messages::{
    HitKind, RagdollHit, RagdollRaycast, RagdollRaycastResponse, RagdollRequestId,
};
use bevy_ragdoll::runtime::pin::PinTargets;
use bevy_ragdoll::runtime::sets::RagdollSystems;
use bevy_ragdoll::{Body, BodyIndex, BodyRole, RagdollDebugPlugin, RagdollPlugin, RagdollProfile};
use bevy_ragdoll_rapier3d::{RapierRagdollHooks, RapierRagdollPlugin};
use bevy_rapier3d::plugin::{RapierPhysicsPlugin, TimestepMode};
use bevy_rapier3d::prelude::{Collider, RigidBody};

/// Hit presets selected by the number keys.
const PRESETS: [(KeyCode, HitProfile); 7] = [
    (KeyCode::Digit1, HitProfile::Pistol),
    (KeyCode::Digit2, HitProfile::Rifle),
    (KeyCode::Digit3, HitProfile::Shotgun),
    (KeyCode::Digit4, HitProfile::Punch),
    (KeyCode::Digit5, HitProfile::Kick),
    (KeyCode::Digit6, HitProfile::Heavy),
    (KeyCode::Digit7, HitProfile::Explosion),
];

/// Bar color for bodies at or above 30% muscle strength.
const STRONG: Color = Color::srgb(0.24, 0.79, 0.71);
/// Bar color for bodies below 30% muscle strength.
const WEAK: Color = Color::srgb(0.93, 0.57, 0.3);

/// Loads the rig profile and runs the windowed example.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let profile = RagdollProfile::from_skeleton(&bevy_ragdoll::Skeleton::humanoid())?;
    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "Hit reactions".into(),
            resolution: (1280, 720).into(),
            ..default()
        }),
        ..default()
    }));
    add_ragdoll_physics(&mut app);
    app.insert_resource(ClearColor(Color::srgb(0.055, 0.075, 0.095)))
        .insert_resource(Rig(profile))
        .init_resource::<Controls>();
    add_example_systems(&mut app);
    app.run();
    Ok(())
}

/// Adds the ragdoll runtime and Rapier, both stepping in `FixedUpdate` at 60 Hz.
fn add_ragdoll_physics(app: &mut App) {
    app.add_plugins((
        RagdollPlugin::default(),
        RapierPhysicsPlugin::<RapierRagdollHooks<'_, '_>>::default().in_fixed_schedule(),
        RapierRagdollPlugin,
        RagdollDebugPlugin,
    ));
    app.insert_resource(Time::<Fixed>::from_hz(60.0))
        .insert_resource(TimestepMode::Fixed {
            dt: 1.0 / 60.0,
            substeps: 1,
        });
}

/// Registers reflected state and adds the scene, idle animation, and input systems.
#[cfg_attr(
    dylint_lib = "sagan_lints",
    expect(
        bevy_disallow_update_schedule,
        reason = "input handling and UI react once per rendered frame, which is what Update is for"
    )
)]
fn add_example_systems(app: &mut App) {
    app.register_type::<Rig>()
        .register_type::<IdleTarget>()
        .register_type::<Controls>()
        .register_type::<MuscleBar>()
        .register_type::<Readout>();
    app.add_systems(Startup, (setup_scene, spawn_ragdoll, spawn_hud))
        .add_systems(
            PostUpdate,
            animate_idle_targets
                .after(AnimationSystems)
                .before(RagdollSystems::CaptureTargets),
        )
        .add_systems(
            Update,
            (
                adjust_controls,
                request_mouse_hit,
                apply_ray_hits,
                rifle_demo,
                update_hud,
            )
                .chain(),
        );
}

/// The validated profile shared by the ragdoll, HUD, and hit systems.
#[derive(Resource, Reflect)]
struct Rig(RagdollProfile);

/// A skeleton bone that sways around its rest rotation to give the drive a target.
#[derive(Component, Reflect)]
struct IdleTarget {
    /// Profile position, used to offset the sway phase.
    index: usize,
    /// Anatomical role, used to choose the sway amplitude.
    role: BodyRole,
    /// Local rest rotation from the profile.
    rest_rotation: Quat,
}

/// The strength channel edited by the arrow keys.
#[derive(Clone, Copy, PartialEq, Eq, Reflect)]
enum Channel {
    /// The joint motor multiplier.
    Muscle,
    /// The pin-force multiplier.
    Pin,
}

/// The user's current hit and strength selections.
#[derive(Resource, Reflect)]
struct Controls {
    /// Preset selected by the number keys.
    profile: HitProfile,
    /// Impulse in kg*m/s that replaces the preset value after `-` or `=`.
    magnitude_override: Option<f32>,
    /// Profile position of the body selected for strength edits.
    body: usize,
    /// Strength channel changed by the arrow keys.
    channel: Channel,
    /// Identity for the next ray request.
    next_request_id: RagdollRequestId,
    /// Impulse to apply when the ray response with this identity arrives.
    pending: HashMap<RagdollRequestId, Vec3>,
}

impl Default for Controls {
    fn default() -> Self {
        Self {
            profile: HitProfile::Rifle,
            magnitude_override: None,
            body: 0,
            channel: Channel::Muscle,
            next_request_id: RagdollRequestId::new(0),
            pending: HashMap::new(),
        }
    }
}

impl Controls {
    /// Returns the selected impulse magnitude in kg*m/s.
    fn magnitude(&self, settings: &HitSettings) -> f32 {
        self.magnitude_override
            .or_else(|| settings.impulse_magnitude(self.profile))
            .unwrap_or_default()
    }
}

/// Marks the HUD bar that shows one body's muscle strength.
#[derive(Component, Reflect)]
struct MuscleBar(usize);

/// Marks the HUD text that shows the selected hit and strength.
#[derive(Component, Clone, Copy, Default, Reflect)]
struct Readout;

/// Converts a profile position to a body index.
fn body_index(position: usize) -> BodyIndex {
    BodyIndex::try_from(position).expect("validated profiles fit the body limit")
}

/// Returns whether a body is pinned: the pelvis and chest.
const fn is_core(body: &Body) -> bool {
    matches!(body.role(), BodyRole::Pelvis | BodyRole::Chest)
}

/// Starts every body at full strength and pins only the core bodies.
fn ragdoll_controls(profile: &RagdollProfile) -> (RagdollBodyWeights, PinTargets) {
    let bodies = profile.bodies();
    let weights = RagdollBodyWeights::new(vec![BodyWeights::new(1.0, 1.0); bodies.len()]);
    let pins = PinTargets::only(
        bodies
            .iter()
            .enumerate()
            .filter(|(_, body)| is_core(body))
            .map(|(position, _)| body_index(position)),
    );
    (weights, pins)
}

/// Returns the next body after `current`; pin editing skips bodies without pin
/// targets.
fn next_body(current: usize, count: usize, channel: Channel, pins: PinTargets) -> usize {
    (1..=count)
        .map(|offset| (current + offset) % count)
        .find(|&candidate| channel == Channel::Muscle || pins.is_targeted(body_index(candidate)))
        .unwrap_or(current)
}

/// Spawns the camera, light, ground, and title.
fn setup_scene(
    mut commands: Commands<'_, '_>,
    mut meshes: ResMut<'_, Assets<Mesh>>,
    mut materials: ResMut<'_, Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(1.6, 2.0, 2.7).looking_at(Vec3::new(0.0, 1.0, 0.0), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 14_000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(-4.0, 7.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        RigidBody::Fixed,
        Collider::cuboid(6.0, 0.1, 6.0),
        Mesh3d(meshes.add(Cuboid::new(12.0, 0.2, 12.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.11, 0.15, 0.19),
            perceptual_roughness: 0.92,
            ..default()
        })),
        Transform::from_xyz(0.0, -0.1, 0.0),
    ));
    commands.spawn((
        label(
            "Hit reactions\nClick the rig to hit it; adjust local muscle and pin strengths.",
            20.0,
            Color::WHITE,
        ),
        Node {
            position_type: PositionType::Absolute,
            top: px(18),
            left: px(20),
            ..default()
        },
    ));
}

/// Spawns the ragdoll root and one bone per profile body at its rest pose.
fn spawn_ragdoll(
    mut commands: Commands<'_, '_>,
    mut profiles: ResMut<'_, Assets<RagdollProfile>>,
    rig: Res<'_, Rig>,
) {
    let profile = &rig.0;
    // Full strength everywhere, with only the core bodies pinned.
    let (weights, pins) = ragdoll_controls(profile);
    let character = commands
        .spawn((
            Name::new("Hit reactions"),
            Ragdoll::new(profiles.add(profile.clone())),
            RagdollMode::Dynamic,
            RagdollDrive::new(1.0, 1.0),
            weights,
            pins,
            Transform::from_xyz(0.0, 0.35, 0.0),
        ))
        .id();

    // Bodies without a joint are roots and hang from the character entity.
    let bodies = profile.bodies();

    // Profiles list parents before children, so each parent bone already exists.
    let mut bones = Vec::with_capacity(bodies.len());
    for (index, body) in bodies.iter().enumerate() {
        let (parent, transform) = parent_and_local_rest(profile, &bones, body, character);
        let target = IdleTarget {
            index,
            role: body.role(),
            rest_rotation: transform.rotation,
        };
        let bone = commands
            .spawn((
                Name::new(body.bone().to_owned()),
                target,
                transform,
                ChildOf(parent),
            ))
            .id();
        bones.push(bone);
    }
}

/// Returns the bone entity a body's bone hangs from and its rest pose relative to it.
///
/// Root bodies hang from the character entity and keep their character-space rest.
fn parent_and_local_rest(
    profile: &RagdollProfile,
    bones: &[Entity],
    body: &Body,
    character: Entity,
) -> (Entity, Transform) {
    let rest = body.rest();
    // The parent's bone entity and rest pose, when this body has a parent.
    let parent = profile
        .joint_of(body.index())
        .map(|joint| joint.parent().get())
        .and_then(|parent| Some((*bones.get(parent)?, profile.bodies().get(parent)?.rest())));
    let Some((parent, parent_rest)) = parent else {
        let local =
            Transform::from_translation(rest.translation.into()).with_rotation(rest.rotation);
        return (character, local);
    };
    // Express the rest pose in the parent's frame.
    let inverse = parent_rest.rotation.inverse();
    let offset = inverse * (rest.translation - parent_rest.translation);
    let local = Transform::from_translation(offset.into()).with_rotation(inverse * rest.rotation);
    (parent, local)
}

/// Sways each bone before the runtime captures its world pose as a drive target.
fn animate_idle_targets(
    time: Res<'_, Time>,
    mut bones: Query<'_, '_, (&IdleTarget, &mut Transform)>,
) {
    // Limbs sway more than the core so the drive has visible work to do.
    for (bone, mut transform) in &mut bones {
        let phase = f32::from(u8::try_from(bone.index % 9).unwrap_or_default()) * 0.47;
        let amplitude = match bone.role {
            BodyRole::Pelvis | BodyRole::Thigh | BodyRole::Calf | BodyRole::Foot => 0.018,
            BodyRole::Spine | BodyRole::Chest => 0.035,
            BodyRole::UpperArm | BodyRole::LowerArm | BodyRole::Hand => 0.055,
            BodyRole::Head | BodyRole::Neck => 0.022,
            BodyRole::Tail | BodyRole::Other => 0.03,
        };
        let sway = f32::mul_add(time.elapsed_secs(), 0.82, phase).sin() * amplitude;
        transform.rotation = bone.rest_rotation * Quat::from_rotation_z(sway);
    }
}

/// Returns a text bundle with the given size and color.
fn label(text: &str, size: f32, color: Color) -> impl Bundle {
    (
        Text::new(text),
        TextFont {
            font_size: FontSize::Px(size),
            ..default()
        },
        TextColor(color),
    )
}

/// Spawns the panel with the hit readout, one muscle bar per body, and key help.
fn spawn_hud(mut commands: Commands<'_, '_>, rig: Res<'_, Rig>) {
    // A dark panel in the top-right corner holds the readout and one bar per body.
    let panel = commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                top: px(16),
                right: px(16),
                width: px(292),
                flex_direction: FlexDirection::Column,
                row_gap: px(6),
                padding: UiRect::all(px(12)),
                ..default()
            },
            BackgroundColor(Color::srgba(0.035, 0.055, 0.07, 0.88)),
        ))
        .id();
    commands.spawn((
        label("Hit profile", 16.0, Color::srgb(0.49, 0.86, 0.89)),
        ChildOf(panel),
    ));
    commands.spawn((label("", 13.0, Color::WHITE), Readout, ChildOf(panel)));
    // One labelled strength bar per profile body.
    for (index, body) in rig.0.bodies().iter().enumerate() {
        let row = commands
            .spawn((
                Node {
                    height: px(13),
                    column_gap: px(7),
                    align_items: AlignItems::Center,
                    ..default()
                },
                ChildOf(panel),
            ))
            .id();
        commands.spawn((
            label(body.bone(), 10.0, Color::srgb(0.83, 0.89, 0.9)),
            Node {
                width: px(116),
                ..default()
            },
            ChildOf(row),
        ));
        let track = commands
            .spawn((
                Node {
                    width: px(142),
                    height: px(7),
                    ..default()
                },
                BackgroundColor(Color::srgb(0.12, 0.18, 0.21)),
                ChildOf(row),
            ))
            .id();
        commands.spawn((
            Node {
                width: percent(100.0),
                height: percent(100.0),
                ..default()
            },
            BackgroundColor(STRONG),
            MuscleBar(index),
            ChildOf(track),
        ));
    }
    // Key help sits at the bottom of the panel.
    commands.spawn((
        label(
            "1-7 select hit | click rig | M muscle | P pin\nTab next body | Up/Down strength | -/= impulse",
            11.0,
            Color::srgb(0.78, 0.85, 0.86),
        ),
        ChildOf(panel),
    ));
}

/// Applies key presses to the hit selection and the selected body's strengths.
fn adjust_controls(
    keys: Res<'_, ButtonInput<KeyCode>>,
    settings: Res<'_, HitSettings>,
    rig: Res<'_, Rig>,
    mut controls: ResMut<'_, Controls>,
    mut ragdolls: Query<'_, '_, (&mut RagdollBodyWeights, &PinTargets)>,
) {
    // Number keys pick a preset and clear any custom impulse.
    for (key, profile) in PRESETS {
        if keys.just_pressed(key) {
            controls.profile = profile;
            controls.magnitude_override = None;
        }
    }
    // Minus and equals nudge the impulse in 1 kg*m/s steps.
    for (key, delta) in [(KeyCode::Minus, -1.0), (KeyCode::Equal, 1.0)] {
        if keys.just_pressed(key) {
            let magnitude = controls.magnitude(&settings) + delta;
            controls.magnitude_override = Some(magnitude.clamp(0.0, 300.0));
        }
    }
    // M and P choose which strength the arrow keys edit.
    if keys.just_pressed(KeyCode::KeyM) {
        controls.channel = Channel::Muscle;
    }
    if keys.just_pressed(KeyCode::KeyP) {
        controls.channel = Channel::Pin;
    }
    // The remaining keys edit the single ragdoll's weights.
    let Ok((mut weights, pins)) = ragdolls.single_mut() else {
        return;
    };
    if keys.just_pressed(KeyCode::Tab) {
        controls.body = next_body(controls.body, rig.0.bodies().len(), controls.channel, *pins);
    }

    // Arrow keys change the selected strength by 0.1.
    let step = if keys.just_pressed(KeyCode::ArrowUp) {
        0.1
    } else if keys.just_pressed(KeyCode::ArrowDown) {
        -0.1
    } else {
        return;
    };
    // Write back both values so the untouched channel keeps its strength.
    let current = weights.get(controls.body).unwrap_or_default();
    let (mut muscle, mut pin) = (current.muscle(), current.pin());
    match controls.channel {
        Channel::Muscle => muscle = (muscle + step).clamp(0.0, 1.0),
        Channel::Pin => pin = (pin + step).clamp(0.0, 1.0),
    }
    weights.set(body_index(controls.body), BodyWeights::new(muscle, pin));
}

/// Sends a backend ray from the cursor on left click and remembers its impulse.
fn request_mouse_hit(
    buttons: Res<'_, ButtonInput<MouseButton>>,
    windows: Query<'_, '_, &Window, With<PrimaryWindow>>,
    cameras: Query<'_, '_, (&Camera, &GlobalTransform)>,
    settings: Res<'_, HitSettings>,
    mut controls: ResMut<'_, Controls>,
    mut requests: MessageWriter<'_, RagdollRaycast>,
) {
    // Only a fresh left click starts a hit.
    if !buttons.just_pressed(MouseButton::Left) {
        return;
    }
    let (Ok(window), Ok((camera, camera_transform))) = (windows.single(), cameras.single()) else {
        return;
    };
    // No cursor over the window means there is nothing to aim at.
    let Some(ray) = window
        .cursor_position()
        .and_then(|cursor| camera.viewport_to_world(camera_transform, cursor).ok())
    else {
        return;
    };
    // Remember the impulse until the backend answers this ray.
    let id = controls.next_request_id;
    controls.next_request_id = RagdollRequestId::new(id.get().wrapping_add(1));
    let impulse = *ray.direction * controls.magnitude(&settings);
    controls.pending.insert(id, impulse);
    requests.write(RagdollRaycast {
        request_id: id,
        origin: ray.origin,
        direction: *ray.direction,
        max_distance: 100.0,
        filter: None,
    });
}

/// Turns each ray response that hit a ragdoll body into a hit message.
fn apply_ray_hits(
    mut responses: MessageReader<'_, '_, RagdollRaycastResponse>,
    mut controls: ResMut<'_, Controls>,
    mut hits: MessageWriter<'_, RagdollHit>,
) {
    // Responses without a pending impulse belong to another sender.
    for response in responses.read() {
        let Some(impulse) = controls.pending.remove(&response.request_id) else {
            continue;
        };
        if let Some(RayHit {
            body: Some(body),
            point,
            ..
        }) = response.hit
        {
            hits.write(RagdollHit {
                body,
                point,
                impulse,
                kind: HitKind::Impact,
            });
        }
    }
}

/// Hits the chest once with a rifle impulse aimed away from the camera.
fn rifle_demo(
    time: Res<'_, Time>,
    mut done: Local<'_, bool>,
    rig: Res<'_, Rig>,
    settings: Res<'_, HitSettings>,
    cameras: Query<'_, '_, &GlobalTransform, With<Camera>>,
    bodies: Query<'_, '_, (Entity, &BodyIndex, &GlobalTransform), With<RagdollBodyOf>>,
    mut hits: MessageWriter<'_, RagdollHit>,
) {
    // Fire once, shortly after the ragdoll has settled into its idle pose.
    if *done || time.elapsed_secs() < 0.85 {
        return;
    }
    let chest = rig.0.body_with_role(BodyRole::Chest).map(BodyIndex::get);
    let Some((body, _, transform)) = bodies
        .iter()
        .find(|(_, index, _)| Some(index.get()) == chest)
    else {
        return;
    };
    // Aim from the camera through the chest so the hit pushes it away from view.
    let point = transform.translation();
    let direction = cameras.iter().next().map_or(Vec3::NEG_Z, |camera| {
        (point - camera.translation()).normalize_or_zero()
    });
    hits.write(RagdollHit {
        body,
        point,
        impulse: direction
            * settings
                .impulse_magnitude(HitProfile::Rifle)
                .unwrap_or_default(),
        kind: HitKind::Impact,
    });
    // Never fire the demo hit again.
    *done = true;
}

/// Updates the muscle bars and the selected hit and body readout.
fn update_hud(
    controls: Res<'_, Controls>,
    settings: Res<'_, HitSettings>,
    rig: Res<'_, Rig>,
    ragdolls: Query<'_, '_, &RagdollBodyWeights>,
    mut bars: Query<'_, '_, (&MuscleBar, &mut Node, &mut BackgroundColor)>,
    mut readouts: Query<'_, '_, &mut Text, With<Readout>>,
) {
    let Ok(weights) = ragdolls.single() else {
        return;
    };
    // Bars turn orange once a body drops below 30% muscle.
    for (bar, mut node, mut color) in &mut bars {
        let muscle = weights.get(bar.0).unwrap_or_default().muscle();
        node.width = percent(muscle * 100.0);
        color.0 = if muscle < 0.3 { WEAK } else { STRONG };
    }
    let Some(body) = rig.0.bodies().get(controls.body) else {
        return;
    };
    // The readout shows the selected body's edited channel.
    let selected = weights.get(controls.body).unwrap_or_default();
    let (channel, value) = match controls.channel {
        Channel::Muscle => ("muscle", selected.muscle()),
        Channel::Pin => ("pin", selected.pin()),
    };
    // And the current hit preset with its impulse.
    let profile = controls.profile;
    let magnitude = controls.magnitude(&settings);
    // Bind the values first so the format string can name them.
    let bone = body.bone();
    for mut text in &mut readouts {
        **text = format!("{profile:?}: {magnitude:.0} kg*m/s\n{bone}: {channel} {value:.2}");
    }
}

#[cfg(test)]
/// Tests the pin and selection rules against the reference humanoid.
mod tests {
    use super::*;

    /// Generates the reference humanoid profile.
    fn profile() -> RagdollProfile {
        RagdollProfile::from_skeleton(&bevy_ragdoll::Skeleton::humanoid())
            .expect("profile validates")
    }

    /// Every body starts at full strength, and the pelvis and chest are pinned.
    #[test]
    fn core_bodies_are_pinned_at_full_strength() {
        let profile = profile();
        let (weights, pins) = ragdoll_controls(&profile);
        let bodies = profile.bodies();
        for position in 0..bodies.len() {
            let weight = weights.get(position).expect("weights cover every body");
            assert_eq!((weight.muscle(), weight.pin()), (1.0, 1.0));
        }
        let pelvis = bodies
            .iter()
            .position(|body| body.role() == BodyRole::Pelvis);
        let spine = bodies
            .iter()
            .position(|body| body.role() == BodyRole::Chest);
        for position in [pelvis, spine] {
            assert!(pins.is_targeted(body_index(position.expect("the humanoid has the body"))));
        }
    }

    /// Pin editing stays on the only pinned body instead of selecting an unpinned one.
    #[test]
    fn pin_selection_cycles_only_targeted_bodies() {
        let pins = PinTargets::only([body_index(2)]);
        assert_eq!(next_body(2, 5, Channel::Pin, pins), 2);
        assert_eq!(next_body(0, 5, Channel::Pin, pins), 2);
        assert_eq!(next_body(4, 5, Channel::Muscle, pins), 0);
    }
}
