//! Active hit, pin, idle-target, and partial-control demonstrations.

#[cfg(feature = "visual")]
use bevy::prelude::Transform;
use bevy_ragdoll::profile::{BodyIndex, BodyRole, RagdollProfile};
use bevy_ragdoll::runtime::components::{BodyWeights, RagdollBodyWeights};
use bevy_ragdoll::runtime::pin::PinTargets;

use super::ExampleKind;

/// Builds the per-body strengths and eligible pin targets for an active example.
pub(super) fn character_controls(
    kind: ExampleKind,
    profile: &RagdollProfile,
) -> Option<(RagdollBodyWeights, PinTargets)> {
    if !is_active_control(kind) {
        return None;
    }

    let mut weights = Vec::with_capacity(profile.bodies().len());
    let mut targets = PinTargets::none();
    for (position, body) in profile.bodies().iter().enumerate() {
        let index = BodyIndex::try_from(position).expect("validated profiles fit the body limit");
        let leg = matches!(
            body.role(),
            BodyRole::Thigh | BodyRole::Calf | BodyRole::Foot
        );
        let supported_lower_body = leg || body.role() == BodyRole::Pelvis;
        match kind {
            ExampleKind::HitReactions => {
                if matches!(body.role(), BodyRole::Pelvis | BodyRole::Chest)
                    || is_tgf_chest_proxy(body.bone())
                {
                    targets.set(index, true);
                }
                weights.push(BodyWeights::new(1.0, 1.0));
            }
            ExampleKind::PartialRagdoll => {
                if supported_lower_body {
                    targets.set(index, true);
                }
                weights.push(if supported_lower_body {
                    BodyWeights::new(1.0, 1.0)
                } else {
                    BodyWeights::new(0.1, 0.0)
                });
            }
            _ => return None,
        }
    }

    Some((RagdollBodyWeights::new(weights), targets))
}

/// Returns whether a profile is used by the active-control demonstrations.
const fn is_active_control(kind: ExampleKind) -> bool {
    matches!(
        kind,
        ExampleKind::HitReactions | ExampleKind::PartialRagdoll
    )
}

/// The TGF human profile uses `spine_04` as its upper torso segment.
fn is_tgf_chest_proxy(bone: &str) -> bool {
    bone.eq_ignore_ascii_case("spine_04")
}

/// Maps numeric example keys to their seven documented hit presets.
#[cfg(feature = "visual")]
fn profile_for_slot(slot: usize) -> HitProfile {
    match slot {
        0 => HitProfile::Pistol,
        1 => HitProfile::Rifle,
        2 => HitProfile::Shotgun,
        3 => HitProfile::Punch,
        4 => HitProfile::Kick,
        5 => HitProfile::Heavy,
        _ => HitProfile::Explosion,
    }
}

/// Marks an animated source bone and stores its local rest rotation.
#[cfg(feature = "visual")]
#[derive(bevy::prelude::Component, Clone, Copy, Debug)]
pub(super) struct TargetBone {
    /// Stable profile position used to vary the procedural idle phase.
    index: BodyIndex,
    /// Resolved anatomical role used to choose a small local sway.
    role: BodyRole,
    /// Source skeleton's authored local orientation before procedural motion.
    rest_rotation: bevy::math::Quat,
}

/// Creates a target marker only for the two active-control demonstrations.
#[cfg(feature = "visual")]
pub(super) fn target_bone(
    kind: ExampleKind,
    position: usize,
    body: &bevy_ragdoll::Body,
    local_transform: Transform,
) -> Option<TargetBone> {
    is_active_control(kind).then(|| TargetBone {
        index: BodyIndex::try_from(position).expect("validated profiles fit the body limit"),
        role: body.role(),
        rest_rotation: local_transform.rotation,
    })
}

#[cfg(feature = "visual")]
mod visual {
    use std::collections::HashMap;
    use std::time::Duration;

    use bevy::app::{AnimationSystems, App, PostUpdate, Startup, Update};
    use bevy::ecs::schedule::IntoScheduleConfigs;
    use bevy::math::Quat;
    use bevy::prelude::{
        AlignItems, BackgroundColor, ButtonInput, Camera, ChildOf, Commands, Component, Entity,
        FlexDirection, GlobalTransform, KeyCode, MessageReader, MessageWriter, Node, PositionType,
        Query, Res, ResMut, Resource, Time, Timer, TimerMode, Transform, UiRect, Vec3, Window,
        With, default, percent, px,
    };
    use bevy::render::view::screenshot::{Screenshot, save_to_disk};
    use bevy::time::Fixed;
    use bevy_ragdoll::profile::{BodyIndex, BodyRole};
    use bevy_ragdoll::runtime::backend::RayHit;
    use bevy_ragdoll::runtime::components::{
        BodyWeights, Ragdoll, RagdollBodyOf, RagdollBodyWeights,
    };
    use bevy_ragdoll::runtime::hit::{HitProfile, HitSettings};
    use bevy_ragdoll::runtime::messages::{
        HitKind, RagdollHit, RagdollRaycast, RagdollRaycastResponse, RagdollRequestId,
    };
    use bevy_ragdoll::runtime::pin::PinTargets;
    use bevy_ragdoll::runtime::sets::RagdollSystems;
    use bevy_rapier3d::prelude::{Collider, Damping, Restitution, RigidBody, Velocity};

    use super::{TargetBone, is_active_control, is_tgf_chest_proxy, profile_for_slot};
    use crate::rapier::{ExampleKind, ExampleOptions, ExampleScene};

    /// Selects one muscle or pin channel for a profile body.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
    enum WeightChannel {
        /// Edit the body's joint motor multiplier.
        #[default]
        Muscle,
        /// Edit the body's pin-force multiplier.
        Pin,
    }

    /// Stores the user's current hit, body, and strength selections.
    #[derive(Resource)]
    struct ControlState {
        /// Preset selected by the numeric keys.
        profile: HitProfile,
        /// Optional absolute magnitude that replaces the preset value.
        magnitude_override: Option<f32>,
        /// Profile body currently selected for strength edits.
        body_index: usize,
        /// Strength channel changed by the arrow keys.
        channel: WeightChannel,
        /// Eligible bodies whose world-space animation targets receive pins.
        pin_targets: PinTargets,
        /// Next caller-owned identity used by an asynchronous ray request.
        next_request_id: u64,
        /// Direction and impulse magnitude stored until a ray response arrives.
        pending_rays: HashMap<u64, PendingRay>,
        /// Fixed example time when its first rifle shot was queued.
        rifle_hit_at: Option<Duration>,
        /// Whether the initial rifle demonstration has run.
        rifle_hit_sent: bool,
        /// Whether the requested post-hit screenshot has been queued.
        screenshot_requested: bool,
    }

    impl Default for ControlState {
        fn default() -> Self {
            Self {
                profile: HitProfile::Rifle,
                magnitude_override: None,
                body_index: 0,
                channel: WeightChannel::Muscle,
                pin_targets: PinTargets::none(),
                next_request_id: 0,
                pending_rays: HashMap::new(),
                rifle_hit_at: None,
                rifle_hit_sent: false,
                screenshot_requested: false,
            }
        }
    }

    /// Holds input data until the backend answers the matching ray request.
    #[derive(Clone, Copy, Debug)]
    struct PendingRay {
        /// World-space unit ray direction used to aim the hit impulse.
        direction: Vec3,
        /// Magnitude captured when the user clicked.
        magnitude: f32,
    }

    /// Identifies a muscle bar that follows one profile body's current value.
    #[derive(Component, Clone, Copy, Debug)]
    struct MuscleBar(BodyIndex);

    /// Marks the text entity that displays the current editable parameters.
    #[derive(Component)]
    struct ControlReadout;

    /// Marks visible bodies used by the partial-ragdoll impact launcher.
    #[derive(Component)]
    struct ImpactBall {
        /// Maximum visible lifetime before the projectile is removed.
        lifetime: Timer,
    }

    /// Controls the repeating impact timer and whether launching is paused.
    #[derive(Resource)]
    struct ImpactLauncher {
        /// Time between projectile launches while the example is running.
        timer: Timer,
        /// Whether the space bar currently pauses projectile launches.
        paused: bool,
    }

    impl Default for ImpactLauncher {
        fn default() -> Self {
            Self {
                timer: Timer::from_seconds(2.0, TimerMode::Repeating),
                paused: false,
            }
        }
    }

    /// Adds procedural targets and the controls needed by the selected example.
    pub(super) fn install(app: &mut App, kind: ExampleKind) {
        if !is_active_control(kind) {
            return;
        }
        app.add_systems(
            PostUpdate,
            animate_idle_targets
                .after(AnimationSystems)
                .before(RagdollSystems::CaptureTargets),
        );
        if kind == ExampleKind::HitReactions {
            app.init_resource::<ControlState>()
                .add_systems(Startup, spawn_hit_controls)
                .add_systems(
                    Update,
                    (
                        adjust_hit_controls,
                        request_mouse_hit,
                        read_hit_responses,
                        queue_rifle_demo,
                        update_hit_hud,
                        capture_hit_screenshot,
                    )
                        .chain(),
                );
        } else {
            app.init_resource::<ImpactLauncher>()
                .add_systems(Startup, spawn_partial_controls)
                .add_systems(
                    Update,
                    (
                        toggle_projectile_launcher,
                        launch_projectile,
                        age_projectiles,
                    )
                        .chain(),
                );
        }
    }

    /// Animates source bone transforms before target capture samples the skeleton.
    fn animate_idle_targets(
        time: Res<'_, Time>,
        mut bones: Query<'_, '_, (&TargetBone, &mut Transform)>,
    ) {
        let seconds = time.elapsed_secs();
        for (bone, mut transform) in &mut bones {
            let phase = (bone.index.get() % 9) as f32 * 0.47;
            let amplitude = match bone.role {
                BodyRole::Pelvis | BodyRole::Thigh | BodyRole::Calf | BodyRole::Foot => 0.018,
                BodyRole::Spine | BodyRole::Chest => 0.035,
                BodyRole::UpperArm | BodyRole::LowerArm | BodyRole::Hand => 0.055,
                BodyRole::Head | BodyRole::Neck => 0.022,
                BodyRole::Tail | BodyRole::Other => 0.03,
            };
            let sway = (seconds * 0.82 + phase).sin() * amplitude;
            transform.rotation = bone.rest_rotation * Quat::from_rotation_z(sway);
        }
    }

    /// Adds the hit preset selector, strength bars, and keyboard help.
    fn spawn_hit_controls(mut commands: Commands<'_, '_>, scene: Res<'_, ExampleScene>) {
        commands.insert_resource(ControlState {
            pin_targets: hit_pin_targets(&scene),
            ..ControlState::default()
        });
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
                BackgroundColor(bevy::prelude::Color::srgba(0.035, 0.055, 0.07, 0.88)),
            ))
            .id();
        spawn_text(
            &mut commands,
            panel,
            "Hit profile",
            16.0,
            bevy::prelude::Color::srgb(0.49, 0.86, 0.89),
        );
        commands.spawn((
            bevy::prelude::Text::new("Rifle 20 kg*m/s"),
            bevy::prelude::TextFont {
                font_size: bevy::prelude::FontSize::Px(13.0),
                ..default()
            },
            bevy::prelude::TextColor(bevy::prelude::Color::WHITE),
            ControlReadout,
            ChildOf(panel),
        ));
        for (position, body) in scene.profile.bodies().iter().enumerate() {
            spawn_muscle_bar(&mut commands, panel, body.bone(), position);
        }
        spawn_text(
            &mut commands,
            panel,
            "1-7 select hit | click rig | M muscle | P pin\nTab next body | Up/Down strength | -/= impulse",
            11.0,
            bevy::prelude::Color::srgb(0.78, 0.85, 0.86),
        );
    }

    /// Creates one labeled horizontal muscle bar for a profile body.
    fn spawn_muscle_bar(
        commands: &mut Commands<'_, '_>,
        panel: Entity,
        bone: &str,
        position: usize,
    ) {
        let index = BodyIndex::try_from(position).expect("validated profiles fit the body limit");
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
            bevy::prelude::Text::new(bone.to_owned()),
            bevy::prelude::TextFont {
                font_size: bevy::prelude::FontSize::Px(10.0),
                ..default()
            },
            bevy::prelude::TextColor(bevy::prelude::Color::srgb(0.83, 0.89, 0.9)),
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
                BackgroundColor(bevy::prelude::Color::srgb(0.12, 0.18, 0.21)),
                ChildOf(row),
            ))
            .id();
        commands.spawn((
            Node {
                width: percent(100.0),
                height: percent(100.0),
                ..default()
            },
            BackgroundColor(bevy::prelude::Color::srgb(0.24, 0.79, 0.71)),
            MuscleBar(index),
            ChildOf(track),
        ));
    }

    /// Adds one label to a UI panel with the selected palette and text size.
    fn spawn_text(
        commands: &mut Commands<'_, '_>,
        parent: Entity,
        value: &str,
        font_size: f32,
        color: bevy::prelude::Color,
    ) {
        commands.spawn((
            bevy::prelude::Text::new(value.to_owned()),
            bevy::prelude::TextFont {
                font_size: bevy::prelude::FontSize::Px(font_size),
                ..default()
            },
            bevy::prelude::TextColor(color),
            ChildOf(parent),
        ));
    }

    /// Selects the pelvis and chest, using the TGF spine proxy when needed.
    fn hit_pin_targets(scene: &ExampleScene) -> PinTargets {
        let mut targets = PinTargets::none();
        for (position, body) in scene.profile.bodies().iter().enumerate() {
            if matches!(body.role(), BodyRole::Pelvis | BodyRole::Chest)
                || is_tgf_chest_proxy(body.bone())
            {
                let index =
                    BodyIndex::try_from(position).expect("validated profiles fit the body limit");
                targets.set(index, true);
            }
        }
        targets
    }

    /// Handles preset selection, body selection, and local strength adjustment.
    fn adjust_hit_controls(
        keys: Res<'_, ButtonInput<KeyCode>>,
        settings: Res<'_, HitSettings>,
        scene: Res<'_, ExampleScene>,
        mut state: ResMut<'_, ControlState>,
        mut characters: Query<'_, '_, &mut RagdollBodyWeights, With<Ragdoll>>,
    ) {
        set_preset(&keys, &settings, &mut state);
        if keys.just_pressed(KeyCode::KeyM) {
            state.channel = WeightChannel::Muscle;
        }
        if keys.just_pressed(KeyCode::KeyP) {
            state.channel = WeightChannel::Pin;
        }
        if keys.just_pressed(KeyCode::Tab) {
            select_next_body(&scene, &mut state);
        }
        if keys.just_pressed(KeyCode::Minus) {
            adjust_hit_magnitude(-1.0, &settings, &mut state);
        }
        if keys.just_pressed(KeyCode::Equal) {
            adjust_hit_magnitude(1.0, &settings, &mut state);
        }
        let step = if keys.just_pressed(KeyCode::ArrowUp) {
            Some(0.1)
        } else if keys.just_pressed(KeyCode::ArrowDown) {
            Some(-0.1)
        } else {
            None
        };
        let Some(step) = step else {
            return;
        };
        if state.body_index >= scene.profile.bodies().len() {
            return;
        }
        let index = BodyIndex::try_from(state.body_index)
            .expect("the selected index came from the validated profile");
        let Ok(mut weights) = characters.single_mut() else {
            return;
        };
        let current = weights.get(state.body_index).unwrap_or_default();
        let (muscle, pin) = match state.channel {
            WeightChannel::Muscle => ((current.muscle() + step).clamp(0.0, 1.0), current.pin()),
            WeightChannel::Pin => (current.muscle(), (current.pin() + step).clamp(0.0, 1.0)),
        };
        weights.set(index, BodyWeights::new(muscle, pin));
    }

    /// Selects a named preset when a numeric key is pressed.
    fn set_preset(keys: &ButtonInput<KeyCode>, settings: &HitSettings, state: &mut ControlState) {
        let selection = [
            (KeyCode::Digit1, 0),
            (KeyCode::Digit2, 1),
            (KeyCode::Digit3, 2),
            (KeyCode::Digit4, 3),
            (KeyCode::Digit5, 4),
            (KeyCode::Digit6, 5),
            (KeyCode::Digit7, 6),
        ];
        for (key, slot) in selection {
            if keys.just_pressed(key) {
                state.profile = profile_for_slot(slot);
                state.magnitude_override = None;
            }
        }
        if state
            .magnitude_override
            .is_some_and(|value| !value.is_finite() || value < 0.0)
        {
            state.magnitude_override = settings.impulse_magnitude(state.profile);
        }
    }

    /// Moves to the next body, restricting pin edits to bodies with pin targets.
    fn select_next_body(scene: &ExampleScene, state: &mut ControlState) {
        let body_count = scene.profile.bodies().len();
        if body_count == 0 {
            state.body_index = 0;
            return;
        }
        for offset in 1..=body_count {
            let candidate = (state.body_index + offset) % body_count;
            let index = BodyIndex::try_from(candidate)
                .expect("the profile body count is validated below the body limit");
            if state.channel == WeightChannel::Muscle || state.pin_targets.is_targeted(index) {
                state.body_index = candidate;
                return;
            }
        }
    }

    /// Applies an absolute magnitude override to the selected preset.
    fn adjust_hit_magnitude(delta: f32, settings: &HitSettings, state: &mut ControlState) {
        let current = state
            .magnitude_override
            .or_else(|| settings.impulse_magnitude(state.profile))
            .unwrap_or_default();
        state.magnitude_override = Some((current + delta).clamp(0.0, 300.0));
    }

    /// Sends a correlated backend ray request from the primary-window cursor.
    fn request_mouse_hit(
        buttons: Res<'_, ButtonInput<bevy::prelude::MouseButton>>,
        windows: Query<'_, '_, &Window, With<bevy::window::PrimaryWindow>>,
        cameras: Query<'_, '_, (&Camera, &GlobalTransform)>,
        settings: Res<'_, HitSettings>,
        mut state: ResMut<'_, ControlState>,
        mut requests: MessageWriter<'_, RagdollRaycast>,
    ) {
        if !buttons.just_pressed(bevy::prelude::MouseButton::Left) {
            return;
        }
        let Ok(window) = windows.single() else {
            return;
        };
        let Some(cursor) = window.cursor_position() else {
            return;
        };
        let Ok((camera, transform)) = cameras.single() else {
            return;
        };
        let Ok(ray) = camera.viewport_to_world(transform, cursor) else {
            return;
        };
        let Some(magnitude) = selected_magnitude(&settings, &state) else {
            return;
        };
        let request_id = state.next_request_id;
        state.next_request_id = state.next_request_id.wrapping_add(1);
        state.pending_rays.insert(
            request_id,
            PendingRay {
                direction: ray.direction.as_vec3(),
                magnitude,
            },
        );
        requests.write(RagdollRaycast {
            request_id: RagdollRequestId::new(request_id),
            origin: ray.origin,
            direction: ray.direction.as_vec3(),
            max_distance: 100.0,
            filter: None,
        });
    }

    /// Converts matching backend ray results into public hit messages.
    fn read_hit_responses(
        mut responses: MessageReader<'_, '_, RagdollRaycastResponse>,
        mut state: ResMut<'_, ControlState>,
        mut hits: MessageWriter<'_, RagdollHit>,
    ) {
        for response in responses.read() {
            let Some(pending) = state.pending_rays.remove(&response.request_id.get()) else {
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
                    impulse: pending.direction * pending.magnitude,
                    kind: HitKind::Impact,
                });
            }
        }
    }

    /// Queues one rifle hit at the TGF upper torso for the timed screenshot.
    fn queue_rifle_demo(
        time: Res<'_, Time<Fixed>>,
        scene: Res<'_, ExampleScene>,
        settings: Res<'_, HitSettings>,
        cameras: Query<'_, '_, &GlobalTransform, With<Camera>>,
        bodies: Query<'_, '_, (Entity, &BodyIndex, &GlobalTransform), With<RagdollBodyOf>>,
        mut state: ResMut<'_, ControlState>,
        mut hits: MessageWriter<'_, RagdollHit>,
    ) {
        if state.rifle_hit_sent || time.elapsed() < Duration::from_millis(850) {
            return;
        }
        let chest_position = chest_body_index(&scene).and_then(|index| {
            bodies
                .iter()
                .find(|(_, body_index, _)| body_index.get() == index)
                .map(|(entity, _, transform)| (entity, transform.translation()))
        });
        let Some((body, point)) = chest_position else {
            return;
        };
        let camera_position = cameras.iter().next().map(GlobalTransform::translation);
        let direction = camera_position.map_or(Vec3::NEG_Z, |position| {
            (point - position).normalize_or_zero()
        });
        let Some(magnitude) = settings.impulse_magnitude(HitProfile::Rifle) else {
            return;
        };
        hits.write(RagdollHit {
            body,
            point,
            impulse: direction * magnitude,
            kind: HitKind::Impact,
        });
        state.rifle_hit_at = Some(time.elapsed());
        state.rifle_hit_sent = true;
    }

    /// Finds the profile body representing the TGF upper torso.
    fn chest_body_index(scene: &ExampleScene) -> Option<usize> {
        scene
            .profile
            .bodies()
            .iter()
            .position(|body| body.role() == BodyRole::Chest)
            .or_else(|| {
                scene
                    .profile
                    .bodies()
                    .iter()
                    .position(|body| is_tgf_chest_proxy(body.bone()))
            })
    }

    /// Returns the selected finite hit magnitude from its preset or override.
    fn selected_magnitude(settings: &HitSettings, state: &ControlState) -> Option<f32> {
        state
            .magnitude_override
            .or_else(|| settings.impulse_magnitude(state.profile))
    }

    /// Updates strength bars and the currently selected profile and body values.
    fn update_hit_hud(
        state: Res<'_, ControlState>,
        settings: Res<'_, HitSettings>,
        scene: Res<'_, ExampleScene>,
        characters: Query<'_, '_, &RagdollBodyWeights, With<Ragdoll>>,
        mut bars: Query<'_, '_, (&MuscleBar, &mut Node, &mut BackgroundColor)>,
        mut readouts: Query<'_, '_, &mut bevy::prelude::Text, With<ControlReadout>>,
    ) {
        let Ok(weights) = characters.single() else {
            return;
        };
        for (bar, mut node, mut color) in &mut bars {
            let muscle = weights.get(bar.0.get()).unwrap_or_default().muscle();
            node.width = percent(muscle * 100.0);
            color.0 = if muscle < 0.3 {
                bevy::prelude::Color::srgb(0.93, 0.57, 0.3)
            } else {
                bevy::prelude::Color::srgb(0.24, 0.79, 0.71)
            };
        }
        let Some(body) = scene.profile.bodies().get(state.body_index) else {
            return;
        };
        let weights = weights.get(state.body_index).unwrap_or_default();
        let magnitude = selected_magnitude(&settings, &state).unwrap_or_default();
        let channel_value = match state.channel {
            WeightChannel::Muscle => weights.muscle(),
            WeightChannel::Pin => weights.pin(),
        };
        let channel = match state.channel {
            WeightChannel::Muscle => "muscle",
            WeightChannel::Pin => "pin",
        };
        for mut text in &mut readouts {
            **text = format!(
                "{}: {magnitude:.0} kg*m/s\n{}: {channel} {channel_value:.2}",
                profile_name(state.profile),
                body.bone(),
            );
        }
    }

    /// Returns the visible short name for one hit profile.
    fn profile_name(profile: HitProfile) -> &'static str {
        match profile {
            HitProfile::Pistol => "Pistol",
            HitProfile::Rifle => "Rifle",
            HitProfile::Shotgun => "Shotgun",
            HitProfile::Punch => "Punch",
            HitProfile::Kick => "Kick",
            HitProfile::Heavy => "Heavy",
            HitProfile::Explosion => "Explosion",
            HitProfile::Custom(_) => "Custom",
        }
    }

    /// Saves the requested screenshot 150 ms after the scripted rifle hit.
    fn capture_hit_screenshot(
        mut commands: Commands<'_, '_>,
        options: Res<'_, ExampleOptions>,
        time: Res<'_, Time<Fixed>>,
        mut state: ResMut<'_, ControlState>,
    ) {
        let Some(path) = options.screenshot.as_ref() else {
            return;
        };
        let Some(hit_at) = state.rifle_hit_at else {
            return;
        };
        if state.screenshot_requested
            || time.elapsed().saturating_sub(hit_at) < Duration::from_millis(150)
        {
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
        state.screenshot_requested = true;
    }

    /// Adds the partial-ragdoll impact launcher status text.
    fn spawn_partial_controls(mut commands: Commands<'_, '_>) {
        commands.spawn((
            bevy::prelude::Text::new("Partial ragdoll\nSpace pauses or resumes chest impacts"),
            bevy::prelude::TextFont {
                font_size: bevy::prelude::FontSize::Px(15.0),
                ..default()
            },
            bevy::prelude::TextColor(bevy::prelude::Color::srgb(0.78, 0.85, 0.86)),
            Node {
                position_type: PositionType::Absolute,
                right: px(16),
                top: px(16),
                padding: UiRect::all(px(10)),
                ..default()
            },
            BackgroundColor(bevy::prelude::Color::srgba(0.035, 0.055, 0.07, 0.88)),
        ));
    }

    /// Toggles timed impacts with the space bar.
    fn toggle_projectile_launcher(
        keys: Res<'_, ButtonInput<KeyCode>>,
        mut launcher: ResMut<'_, ImpactLauncher>,
    ) {
        if keys.just_pressed(KeyCode::Space) {
            launcher.paused = !launcher.paused;
        }
    }

    /// Launches one visible Rapier ball from the human chest every two seconds.
    fn launch_projectile(
        mut commands: Commands<'_, '_>,
        scene: Res<'_, ExampleScene>,
        time: Res<'_, Time>,
        mut launcher: ResMut<'_, ImpactLauncher>,
        bodies: Query<'_, '_, (&BodyIndex, &GlobalTransform), With<RagdollBodyOf>>,
        mut meshes: ResMut<'_, bevy::prelude::Assets<bevy::prelude::Mesh>>,
        mut materials: ResMut<'_, bevy::prelude::Assets<bevy::prelude::StandardMaterial>>,
    ) {
        if launcher.paused {
            return;
        }
        launcher.timer.tick(time.delta());
        if !launcher.timer.just_finished() {
            return;
        }
        let Some(index) = chest_body_index(&scene) else {
            return;
        };
        let Some((_, chest)) = bodies
            .iter()
            .find(|(body_index, _)| body_index.get() == index)
        else {
            return;
        };
        let target = chest.translation();
        let launch_direction = Vec3::new(0.0, 0.04, -1.0).normalize();
        let origin = target - launch_direction * 1.15;
        commands.spawn((
            RigidBody::Dynamic,
            Collider::ball(0.11),
            Velocity {
                linear: launch_direction * 7.0,
                angular: Vec3::ZERO,
            },
            Damping {
                linear_damping: 0.04,
                angular_damping: 0.04,
            },
            Restitution::coefficient(0.35),
            Transform::from_translation(origin),
            bevy::prelude::Mesh3d(meshes.add(bevy::prelude::Sphere::new(0.11))),
            bevy::prelude::MeshMaterial3d(materials.add(bevy::prelude::StandardMaterial {
                base_color: bevy::prelude::Color::srgb(0.93, 0.57, 0.3),
                metallic: 0.18,
                perceptual_roughness: 0.32,
                ..default()
            })),
            ImpactBall {
                lifetime: Timer::from_seconds(4.0, TimerMode::Once),
            },
        ));
    }

    /// Removes impact balls after their lifetime or when they fall below the floor.
    fn age_projectiles(
        mut commands: Commands<'_, '_>,
        time: Res<'_, Time>,
        mut balls: Query<'_, '_, (Entity, &mut ImpactBall, &Transform)>,
    ) {
        for (entity, mut ball, transform) in &mut balls {
            ball.lifetime.tick(time.delta());
            if ball.lifetime.is_finished() || transform.translation.y < -2.0 {
                commands.entity(entity).despawn();
            }
        }
    }

    #[cfg(test)]
    mod tests {
        use super::{
            ControlState, WeightChannel, adjust_hit_magnitude, profile_for_slot, profile_name,
            select_next_body, set_preset,
        };
        use crate::rapier::{ExampleKind, load_profile};
        use bevy::prelude::{ButtonInput, KeyCode};
        use bevy_ragdoll::runtime::hit::{HitProfile, HitSettings};
        use bevy_ragdoll::runtime::pin::PinTargets;

        /// Numeric slots expose all seven named hit presets in plan order.
        #[test]
        fn hit_slots_select_the_documented_profiles() {
            let expected = [
                "Pistol",
                "Rifle",
                "Shotgun",
                "Punch",
                "Kick",
                "Heavy",
                "Explosion",
            ];
            for (slot, name) in expected.into_iter().enumerate() {
                assert_eq!(profile_name(profile_for_slot(slot)), name);
            }
            assert_eq!(profile_for_slot(6), HitProfile::Explosion);
        }

        /// Pin editing skips bodies outside the profile's configured pin mask.
        #[test]
        fn pin_selection_cycles_only_targeted_bodies() {
            let scene = crate::rapier::ExampleScene {
                kind: ExampleKind::HitReactions,
                profile: load_profile(ExampleKind::HitReactions).expect("TGF profile validates"),
            };
            let pelvis = scene
                .profile
                .bodies()
                .iter()
                .position(|body| body.role() == bevy_ragdoll::BodyRole::Pelvis)
                .expect("TGF rig has a pelvis");
            let mut targets = PinTargets::none();
            targets.set(
                bevy_ragdoll::BodyIndex::try_from(pelvis).expect("profile index is valid"),
                true,
            );
            let mut state = ControlState {
                body_index: pelvis,
                channel: WeightChannel::Pin,
                pin_targets: targets,
                ..ControlState::default()
            };
            select_next_body(&scene, &mut state);
            assert_eq!(state.body_index, pelvis);
        }

        /// Hit strength and magnitude edits remain bounded at both ends.
        #[test]
        fn hit_adjustments_clamp_strength_and_magnitude() {
            let settings = HitSettings::default();
            let mut state = ControlState::default();
            adjust_hit_magnitude(-1000.0, &settings, &mut state);
            assert_eq!(state.magnitude_override, Some(0.0));
            adjust_hit_magnitude(1000.0, &settings, &mut state);
            assert_eq!(state.magnitude_override, Some(300.0));

            let mut keys = ButtonInput::<KeyCode>::default();
            keys.press(KeyCode::Digit6);
            set_preset(&keys, &settings, &mut state);
            assert_eq!(state.profile, HitProfile::Heavy);
            assert_eq!(state.magnitude_override, None);
            assert_eq!(settings.impulse_magnitude(state.profile), Some(120.0));
        }
    }
}

#[cfg(feature = "visual")]
pub(super) fn install(app: &mut bevy::prelude::App, kind: ExampleKind) {
    visual::install(app, kind);
}

#[cfg(test)]
mod tests {
    use super::character_controls;
    use crate::rapier::{ExampleKind, load_profile};
    use bevy_ragdoll::profile::BodyRole;

    /// Hit reactions start at full strength and pin the pelvis and torso proxy.
    #[test]
    fn hit_example_pins_core_bodies_and_keeps_full_strength() {
        let profile = load_profile(ExampleKind::HitReactions).expect("TGF rig validates");
        let (weights, targets) = character_controls(ExampleKind::HitReactions, &profile)
            .expect("hit example has active controls");
        let pelvis = profile
            .bodies()
            .iter()
            .position(|body| body.role() == BodyRole::Pelvis)
            .expect("TGF rig has a pelvis");
        let pelvis = bevy_ragdoll::BodyIndex::try_from(pelvis).expect("profile index is valid");
        assert!(targets.is_targeted(pelvis));
        assert!(profile.bodies().iter().enumerate().all(|(position, _)| {
            let weight = weights
                .get(position)
                .expect("profile weights are initialized");
            weight.muscle() == 1.0 && weight.pin() == 1.0
        }));
        assert!(profile.bodies().iter().enumerate().any(|(position, body)| {
            body.bone().eq_ignore_ascii_case("spine_04")
                && targets.is_targeted(
                    bevy_ragdoll::BodyIndex::try_from(position).expect("profile index is valid"),
                )
        }));
    }

    /// The partial example pins its pelvis and legs, then weakens upper bodies.
    #[test]
    fn partial_example_keeps_leg_targets_and_releases_upper_bodies() {
        let profile = load_profile(ExampleKind::PartialRagdoll).expect("TGF rig validates");
        let (weights, targets) = character_controls(ExampleKind::PartialRagdoll, &profile)
            .expect("partial example has active controls");
        let mut leg_count = 0;
        for (position, body) in profile.bodies().iter().enumerate() {
            let index = bevy_ragdoll::BodyIndex::try_from(position)
                .expect("profile body position is validated");
            let weight = weights
                .get(position)
                .expect("profile weights are initialized");
            let is_leg = matches!(
                body.role(),
                BodyRole::Thigh | BodyRole::Calf | BodyRole::Foot
            );
            let supported = is_leg || body.role() == BodyRole::Pelvis;
            if supported {
                if is_leg {
                    leg_count += 1;
                }
                assert!(targets.is_targeted(index));
                assert_eq!((weight.muscle(), weight.pin()), (1.0, 1.0));
            } else {
                assert!(!targets.is_targeted(index));
                assert_eq!((weight.muscle(), weight.pin()), (0.1, 0.0));
            }
        }
        assert!(leg_count >= 4, "the TGF profile includes both legs");
    }
}
