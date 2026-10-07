//! Joint motors, world-space pins, and backend fallback drive calculations.
//!
//! The fixed drive stage reads animation targets and current body state, then
//! writes force outputs before backend application. Native-motor backends
//! receive typed joint targets; other backends receive equal and opposite
//! stable-PD torques with optional soft-limit restoration. Pure helpers expose
//! the same equations for focused tests and backend adapters.

use std::collections::HashMap;

use bevy::math::{Isometry3d, Quat, Vec3};
use bevy::prelude::{Entity, World};
use bevy::time::{Fixed, Time};

use crate::profile::{BodyIndex, JointLimits};

use super::backend::BackendCapabilities;
use super::body::{
    BodyDriveOutput, BodyMass, BodyPhysicsPose, BodyVelocity, JointDriveTarget, JointToParent,
};
use super::components::{
    Ragdoll, RagdollBodies, RagdollBodyWeights, RagdollDrive, RagdollMode, RagdollTargetPose,
};
use super::pin::{PinSettings, PinTargets};
use super::settings::RagdollPhysicsSettings;

/// Acceleration-based motor gains and torque limit for one profile joint.
///
/// The core scales stiffness and damping by whole-ragdoll and per-body muscle
/// strength, then preserves authored maximum torque through configured torque
/// scale and friction terms. Backends map these values to their native
/// acceleration-based motor model.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct JointMotorValues {
    /// Angular motor stiffness in inverse seconds squared after frequency and
    /// muscle scaling. The core or backend reads this member during the drive
    /// stage and follows its documented physical units.
    pub stiffness: f32,
    /// Angular motor damping in inverse seconds, including configured joint
    /// friction rate. The core or backend reads this member during the drive
    /// stage and follows its documented physical units.
    pub damping: f32,
    /// Maximum motor torque in newton metres after profile and backend settings
    /// are applied. The core or backend reads this member during the drive
    /// stage and follows its documented physical units.
    pub max_torque: f32,
}

/// World-space pin force and torque calculated for one ragdoll body.
///
/// Force uses newtons and torque uses newton metres. Both outputs are finite
/// and limited by `RagdollPhysicsSettings`, with strength and distance falloff
/// applied before the backend step.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PinDriveOutput {
    /// World-space force in newtons pulling the body toward its captured
    /// animated target pose. The core or backend reads this member during the
    /// drive stage and follows its documented physical units.
    pub force: Vec3,
    /// World-space torque in newton metres rotating the body toward its
    /// captured animated target. The core or backend reads this member during
    /// the drive stage and follows its documented physical units.
    pub torque: Vec3,
}

/// Computes acceleration-based motor values from muscle strength and an
/// authored torque limit.
///
/// Non-finite muscle or torque input becomes zero, finite strengths are clamped
/// to `0..=1`, and settings supply frequency, damping, friction, and torque
/// scaling.
///
/// # Examples
///
/// ```
/// use bevy_ragdoll::runtime::drive::joint_motor_values; use
/// bevy_ragdoll::runtime::settings::RagdollPhysicsSettings;
///
/// let settings = RagdollPhysicsSettings::default(); let motor =
/// joint_motor_values(1.0, 8.0, &settings); assert!((motor.max_torque -
/// 8.4).abs() < 1.0e-5);
/// ```
#[must_use]
pub fn joint_motor_values(
    muscle: f32,
    joint_max_torque: f32,
    settings: &RagdollPhysicsSettings,
) -> JointMotorValues {
    // Clamp profile-facing strengths and torque limits before deriving acceleration gains.
    let muscle = if muscle.is_finite() {
        muscle.clamp(0.0, 1.0)
    } else {
        0.0
    };
    let joint_max_torque = if joint_max_torque.is_finite() {
        joint_max_torque.max(0.0)
    } else {
        0.0
    };
    // Convert natural frequency to angular frequency before calculating damped gains.
    let omega = std::f32::consts::TAU * settings.motor_frequency_hz;
    JointMotorValues {
        stiffness: omega * omega * muscle,
        damping: 2.0 * settings.motor_damping_ratio * omega * muscle.sqrt()
            + settings.friction_rate,
        max_torque: joint_max_torque * settings.torque_scale * (settings.joint_friction + muscle),
    }
}

/// Interpolates translation and rotation between two poses surrounding a fixed
/// physics step.
///
/// The weight is clamped to `0..=1`, translation is linear, and quaternion
/// rotation follows the shortest path without scale extrapolation.
///
/// # Examples
///
/// ```
/// use bevy::math::{Isometry3d, Vec3}; use
/// bevy_ragdoll::runtime::drive::interpolate_pose;
///
/// let end = Isometry3d::from_translation(Vec3::X);
/// assert_eq!(interpolate_pose(Isometry3d::IDENTITY, end, 1.0), end);
/// ```
#[must_use]
pub fn interpolate_pose(previous: Isometry3d, current: Isometry3d, alpha: f32) -> Isometry3d {
    // Clamp render overstep so callers never extrapolate beyond completed physics states.
    let alpha = alpha.clamp(0.0, 1.0);
    Isometry3d::new(
        previous.translation.lerp(current.translation, alpha),
        previous.rotation.slerp(current.rotation, alpha),
    )
}

/// Returns the shortest-arc axis-angle rotation vector from `current` to
/// `target`.
///
/// The vector direction is the rotation axis and its magnitude is radians;
/// negating an equivalent target quaternion does not change the selected
/// shortest arc.
///
/// # Examples
///
/// ```
/// use bevy::math::{Quat, Vec3}; use
/// bevy_ragdoll::runtime::drive::rotation_error;
///
/// assert!(rotation_error(Quat::IDENTITY, Quat::from_rotation_z(0.5))
/// .abs_diff_eq(Vec3::Z * 0.5, 1.0e-6));
/// ```
#[must_use]
pub fn rotation_error(current: Quat, target: Quat) -> Vec3 {
    // Normalize the quaternion representative to its positive-w shortest-arc form.
    let mut error = target * current.inverse();
    if error.w < 0.0 {
        error = -error;
    }
    let (axis, angle) = error.to_axis_angle();
    axis * angle
}

/// Inputs for one critically damped world-space pin force and torque
/// calculation.
///
/// Poses use world-space isometries, velocities use metres and radians per
/// second, and mass uses kilograms. The runtime constructs this value from one
/// body and its captured target before calling [`pin_drive`], keeping related
/// physical inputs together at the API boundary.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PinDriveInput {
    /// Current rigid body pose in world coordinates after the latest completed
    /// backend step. The core or backend reads this member during the drive
    /// stage and follows its documented physical units.
    pub current_pose: Isometry3d,
    /// Target rigid body pose in world coordinates, composed from the latest
    /// animated skeleton capture. The core or backend reads this member during
    /// the drive stage and follows its documented physical units.
    pub target_pose: Isometry3d,
    /// Current world-space linear and angular body velocity reported by the
    /// backend after stepping. The core or backend reads this member during the
    /// drive stage and follows its documented physical units.
    pub current_velocity: BodyVelocity,
    /// Target world-space linear and angular velocity derived from the two
    /// latest animation captures. The core or backend reads this member during
    /// the drive stage and follows its documented physical units.
    pub target_velocity: BodyVelocity,
    /// Body mass in kilograms used to convert desired linear acceleration into
    /// backend force units. The core or backend reads this member during the
    /// drive stage and follows its documented physical units.
    pub mass: f32,
    /// Scalar inertia estimate in kilogram metres squared used to convert
    /// angular acceleration into torque. The core or backend reads this member
    /// during the drive stage and follows its documented physical units.
    pub inertia: f32,
    /// Pin strength multiplier clamped to the inclusive unit interval before
    /// force and torque scaling. The core or backend reads this member during
    /// the drive stage and follows its documented physical units.
    pub strength: f32,
}

/// Computes a critically damped pin force and torque with configured limits.
///
/// Invalid strength, mass, or inertia values become zero before the controller
/// applies the configured frequency, damping ratio, force cap, torque cap, and
/// distance falloff.
///
/// # Examples
///
/// ```
/// use bevy::math::Isometry3d; use bevy_ragdoll::runtime::body::BodyVelocity;
/// use bevy_ragdoll::runtime::drive::{pin_drive, PinDriveInput}; use
/// bevy_ragdoll::runtime::settings::RagdollPhysicsSettings;
///
/// let input = PinDriveInput { current_pose: Isometry3d::IDENTITY, target_pose:
/// Isometry3d::IDENTITY, current_velocity: BodyVelocity::default(),
/// target_velocity: BodyVelocity::default(), mass: 1.0, inertia: 1.0, strength:
/// 1.0, }; let output = pin_drive(input, &RagdollPhysicsSettings::default());
/// assert_eq!(output.force.length(), 0.0);
/// ```
#[must_use]
pub fn pin_drive(input: PinDriveInput, settings: &RagdollPhysicsSettings) -> PinDriveOutput {
    // Sanitize physical scalars before they enter acceleration and force calculations.
    let strength = clamp_unit(input.strength);
    let mass = finite_nonnegative(input.mass);
    let inertia = finite_nonnegative(input.inertia);
    let omega = std::f32::consts::TAU * settings.pin_frequency_hz;
    // Calculate translation correction and apply distance-dependent force limits.
    let position_error = Vec3::from(input.target_pose.translation - input.current_pose.translation);
    let distance = position_error.length();
    let linear_acceleration = omega * omega * position_error
        + 2.0
            * settings.pin_damping_ratio
            * omega
            * (input.target_velocity.linear - input.current_velocity.linear);
    let force = limit_vector(
        strength * mass * linear_acceleration,
        strength * finite_nonnegative(settings.pin_max_force)
            / (1.0 + finite_nonnegative(settings.pin_distance_falloff) * distance),
    );

    // Calculate angular correction independently so force and torque limits stay separate.
    let rotation_error = rotation_error(input.current_pose.rotation, input.target_pose.rotation);
    let angular_acceleration = omega * omega * rotation_error
        + 2.0
            * settings.pin_damping_ratio
            * omega
            * (input.target_velocity.angular - input.current_velocity.angular);
    let torque = limit_vector(
        strength * inertia * angular_acceleration,
        strength * finite_nonnegative(settings.pin_max_torque),
    );
    PinDriveOutput { force, torque }
}

/// Inputs for a stable-PD joint torque calculation in the parent constraint
/// frame.
///
/// Rotations and angular velocities describe the child relative to its parent,
/// while inertia is a scalar estimate in kilogram metres squared and the step
/// duration is measured in seconds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StablePdInput {
    /// World rotation of the parent joint frame used to rotate local angular
    /// error and torque. The core or backend reads this member during the drive
    /// stage and follows its documented physical units.
    pub frame_rotation: Quat,
    /// Current child orientation relative to the parent's validated joint frame
    /// after the backend step. The core or backend reads this member during the
    /// drive stage and follows its documented physical units.
    pub current_relative_rotation: Quat,
    /// Desired child orientation relative to the parent's joint frame from
    /// captured target poses. The core or backend reads this member during the
    /// drive stage and follows its documented physical units.
    pub target_relative_rotation: Quat,
    /// Current child-minus-parent angular velocity in world coordinates,
    /// measured in radians per second. The core or backend reads this member
    /// during the drive stage and follows its documented physical units.
    pub relative_angular_velocity: Vec3,
    /// Desired child-relative angular velocity in the target parent joint
    /// frame, measured in radians per second. The core or backend reads this
    /// member during the drive stage and follows its documented physical units.
    pub target_angular_velocity: Vec3,
    /// Scalar child inertia estimate in kilogram metres squared about the
    /// correction axis. The core or backend reads this member during the drive
    /// stage and follows its documented physical units.
    pub inertia: f32,
    /// Positive duration of the upcoming backend fixed integration step,
    /// measured in seconds. The core or backend reads this member during the
    /// drive stage and follows its documented physical units.
    pub delta_seconds: f32,
}

/// Per-body state gathered once before pin and joint calculations.
#[derive(Clone, Copy, Debug, PartialEq)]
struct BodyDriveState {
    /// Physics entity receiving this body's output and joint target.
    entity: Entity,
    /// Current animation target in world space, absent when capture has no
    /// indexed pose.
    target_pose: Option<Isometry3d>,
    /// World-space target velocity derived from animation capture.
    target_velocity: BodyVelocity,
    /// Current rigid physics pose, defaulting to identity when backend state is
    /// absent.
    current_pose: Isometry3d,
    /// Current world-space backend velocity, defaulting to zero when not yet
    /// reported.
    current_velocity: BodyVelocity,
    /// Body mass in kilograms used by the pin controller.
    mass: f32,
    /// Scalar inertia estimate in kilogram metres squared used by both
    /// controllers.
    inertia: f32,
    /// Force and torque values accumulated for this body's next backend step.
    output: BodyDriveOutput,
}

/// Character-wide inputs shared while each body drive state is gathered.
struct BodyDriveContext<'a> {
    /// Character pose applied to every captured local target.
    character_pose: Isometry3d,
    /// Whole-character motor and pin strengths.
    drive: RagdollDrive,
    /// Profile-ordered per-body muscle and pin strengths.
    weights: &'a RagdollBodyWeights,
    /// Captured local poses and velocities for the character.
    targets: &'a RagdollTargetPose,
    /// Body indexes that receive world-space pin output.
    pin_targets: PinTargets,
    /// Character-specific pin-controller settings after overrides.
    settings: RagdollPhysicsSettings,
    /// Whether any non-limp body enables the shared velocity limits.
    has_velocity_limits: bool,
}

/// Shared controls for calculating every joint on one dynamic character.
struct JointDriveConfig<'a> {
    /// Whole-character drive strengths applied before per-body multipliers.
    drive: RagdollDrive,
    /// Profile-ordered strength overrides used by each child body.
    weights: &'a RagdollBodyWeights,
    /// Motor frequency, damping, friction, and torque scaling for the fixed
    /// step.
    settings: RagdollPhysicsSettings,
    /// Backend feature predicates selecting native motors or fallback torque.
    capabilities: BackendCapabilities,
    /// Positive duration of the current fixed physics step in seconds.
    delta_seconds: f32,
}

/// Computes stable-PD joint torque in world space and enforces the torque
/// limit.
///
/// A non-finite or nonpositive step duration returns zero torque; otherwise the
/// result is clamped to the motor's finite nonnegative torque limit after
/// stable-PD acceleration correction.
///
/// # Examples
///
/// ```
/// use bevy::math::{Quat, Vec3}; use
/// bevy_ragdoll::runtime::drive::{stable_pd_torque, JointMotorValues,
/// StablePdInput};
///
/// let input = StablePdInput { frame_rotation: Quat::IDENTITY,
/// current_relative_rotation: Quat::IDENTITY, target_relative_rotation:
/// Quat::from_rotation_z(0.5), relative_angular_velocity: Vec3::ZERO,
/// target_angular_velocity: Vec3::ZERO, inertia: 1.0, delta_seconds: 1.0 /
/// 60.0, }; let motor = JointMotorValues { stiffness: 1.0, damping: 0.0,
/// max_torque: 2.0 }; assert!(stable_pd_torque(input, motor).z > 0.0);
/// ```
#[must_use]
pub fn stable_pd_torque(input: StablePdInput, motor: JointMotorValues) -> Vec3 {
    // Reject invalid fixed-step durations before evaluating the stable-PD denominator.
    if !input.delta_seconds.is_finite() || input.delta_seconds <= 0.0 {
        return Vec3::ZERO;
    }
    // Transform rotation and velocity errors from the parent joint frame into world coordinates.
    let error_world = input.frame_rotation
        * rotation_error(
            input.current_relative_rotation,
            input.target_relative_rotation,
        );
    let target_velocity_world = input.frame_rotation * input.target_angular_velocity;
    let velocity_error = input.relative_angular_velocity - target_velocity_world;
    let torque = finite_nonnegative(input.inertia)
        * (motor.stiffness * error_world
            - (motor.damping + motor.stiffness * input.delta_seconds) * velocity_error);
    limit_vector(torque, finite_nonnegative(motor.max_torque))
}

/// Computes per-body pin and joint drive outputs before the backend step.
pub(crate) fn drive(world: &mut World) {
    // Missing fixed time or settings prevents a valid force calculation for this stage.
    let Some(delta_seconds) = world.get_resource::<Time<Fixed>>().map(Time::delta_secs) else {
        return;
    };
    let Some(settings) = world.get_resource::<RagdollPhysicsSettings>().copied() else {
        return;
    };
    // Backends without an explicit capability resource use the portable fallback path.
    let capabilities = world
        .get_resource::<BackendCapabilities>()
        .copied()
        .unwrap_or_default();
    // Snapshot characters before writing body output components.
    let characters = {
        let mut query = world.query_filtered::<Entity, bevy::prelude::With<Ragdoll>>();
        query.iter(world).collect::<Vec<_>>()
    };
    for character in characters {
        drive_character(world, character, settings, capabilities, delta_seconds);
    }
}

/// Gathers a character's body inputs in validated profile order.
fn gather_body_drive_states(
    world: &World,
    character: Entity,
    entities: &[Entity],
    settings: RagdollPhysicsSettings,
) -> (RagdollDrive, RagdollBodyWeights, Vec<BodyDriveState>) {
    // Snapshot character-wide controls before any body component is mutated.
    let character_pose = super::writeback::world_pose(world, character);
    let drive = world
        .get::<RagdollDrive>(character)
        .copied()
        .unwrap_or_default();
    let weights = world
        .get::<RagdollBodyWeights>(character)
        .cloned()
        .unwrap_or_default();
    let targets = world
        .get::<RagdollTargetPose>(character)
        .cloned()
        .unwrap_or_default();
    let pin_targets = world
        .get::<PinTargets>(character)
        .copied()
        .unwrap_or_default();
    // Apply character overrides before computing any body pin outputs.
    let mut pin_controller_settings = settings;
    if let Some(pin_settings) = world.get::<PinSettings>(character).copied() {
        pin_settings.override_shared(&mut pin_controller_settings);
    }
    let has_velocity_limits = drive.muscle() > 0.0
        && entities.iter().any(|entity| {
            world
                .get::<BodyIndex>(*entity)
                .and_then(|index| weights.get(index.get()))
                .map_or(1.0, super::components::BodyWeights::muscle)
                > 0.0
        });

    // Resolve each body's target, velocity, mass, and pin output once per fixed step.
    let context = BodyDriveContext {
        character_pose,
        drive,
        weights: &weights,
        targets: &targets,
        pin_targets,
        settings: pin_controller_settings,
        has_velocity_limits,
    };
    let states = entities
        .iter()
        .copied()
        .map(|entity| body_drive_state(world, entity, &context))
        .collect();
    (drive, weights, states)
}

/// Gathers one character's body state, computes joint targets, and writes backend outputs.
fn drive_character(
    world: &mut World,
    character: Entity,
    settings: RagdollPhysicsSettings,
    capabilities: BackendCapabilities,
    delta_seconds: f32,
) {
    // Only dynamic ragdolls receive active pin and joint drive calculations.
    if world.get::<RagdollMode>(character) != Some(&RagdollMode::Dynamic) {
        return;
    }
    let Some(entities) = ordered_body_entities(world, character) else {
        return;
    };

    let (drive, weights, mut states) =
        gather_body_drive_states(world, character, &entities, settings);

    // Resolve joint parents once, then apply fallback torques symmetrically to both bodies.
    calculate_joint_targets(
        world,
        &entities,
        &mut states,
        JointDriveConfig {
            drive,
            weights: &weights,
            settings,
            capabilities,
            delta_seconds,
        },
    );

    // Publish complete accumulated outputs after every joint has contributed its torque.
    for state in states {
        if let Ok(mut entity) = world.get_entity_mut(state.entity) {
            entity.insert(state.output);
        }
    }
}

/// Returns a profile-index-ordered body list, or `None` when the character has
/// no bodies.
fn ordered_body_entities(world: &World, character: Entity) -> Option<Vec<Entity>> {
    // Copy entity identifiers before sorting so no world borrow overlaps later component writes.
    let mut entities = world
        .get::<RagdollBodies>(character)?
        .iter()
        .collect::<Vec<_>>();
    entities.sort_unstable_by_key(|entity| {
        world
            .get::<BodyIndex>(*entity)
            .map_or(usize::MAX, |index| index.get())
    });
    if entities.is_empty() {
        None
    } else {
        Some(entities)
    }
}

/// Resolves captured and backend state and calculates the body's world-space
/// pin output.
fn body_drive_state(
    world: &World,
    entity: Entity,
    context: &BodyDriveContext<'_>,
) -> BodyDriveState {
    // Convert captured target pose and velocity into world coordinates when the body index exists.
    let body_index = world.get::<BodyIndex>(entity).copied();
    let target_pose = body_index
        .and_then(|index| context.targets.current_pose(index))
        .map(|target| context.character_pose * target);
    let target_velocity = body_index
        .and_then(|index| context.targets.velocity(index))
        .map(|velocity| BodyVelocity {
            linear: context.character_pose.rotation * velocity.linear,
            angular: context.character_pose.rotation * velocity.angular,
        })
        .unwrap_or_default();

    // Derive scalar inertia from validated mass and the backend's minimum radius.
    let mass = world.get::<BodyMass>(entity).copied().unwrap_or(BodyMass {
        mass: 1.0,
        min_inertia_radius: 1.0,
    });
    let inertia = (mass.mass * mass.min_inertia_radius.powi(2)).max(1.0e-6);
    let current_pose = world
        .get::<BodyPhysicsPose>(entity)
        .map_or(Isometry3d::IDENTITY, |pose| pose.current);
    let current_velocity = world
        .get::<BodyVelocity>(entity)
        .copied()
        .unwrap_or_default();
    let mut state = BodyDriveState {
        entity,
        target_pose,
        target_velocity,
        current_pose,
        current_velocity,
        mass: mass.mass,
        inertia,
        output: BodyDriveOutput::default(),
    };
    state.output = body_pin_output(
        state,
        body_index,
        context.drive,
        context.weights,
        context.pin_targets,
        context.settings,
        context.has_velocity_limits,
    );
    state
}

/// Computes this body's pin force and torque from its gathered state and
/// strength overrides.
fn body_pin_output(
    state: BodyDriveState,
    body_index: Option<BodyIndex>,
    drive: RagdollDrive,
    weights: &RagdollBodyWeights,
    pin_targets: PinTargets,
    settings: RagdollPhysicsSettings,
    has_velocity_limits: bool,
) -> BodyDriveOutput {
    // Speed limits apply to every body until the whole character is limp.
    let mut output = BodyDriveOutput {
        pin_force: Vec3::ZERO,
        pin_torque: Vec3::ZERO,
        joint_torque: Vec3::ZERO,
        max_linear_speed: has_velocity_limits.then_some(10.0),
        max_angular_speed: has_velocity_limits.then_some(20.0),
    };
    // A missing target keeps the body output at zero until capture provides its indexed pose.
    let Some(target_pose) = state.target_pose else {
        return output;
    };
    if !body_index.is_some_and(|index| pin_targets.is_targeted(index)) {
        return output;
    }
    // Scale pin strength independently for this checked profile body position.
    let pin_strength = drive.pin()
        * body_index
            .and_then(|index| weights.get(index.get()))
            .map_or(1.0, super::components::BodyWeights::pin);
    // Apply the resolved per-body strength with the checked controller settings.
    let pin = pin_drive(
        PinDriveInput {
            current_pose: state.current_pose,
            target_pose,
            current_velocity: state.current_velocity,
            target_velocity: state.target_velocity,
            mass: state.mass,
            inertia: state.inertia,
            strength: pin_strength,
        },
        &settings,
    );
    output.pin_force = pin.force;
    output.pin_torque = pin.torque;
    output
}

/// Computes motor targets and fallback torques for every available parent-child
/// pair.
fn calculate_joint_targets(
    world: &mut World,
    entities: &[Entity],
    states: &mut [BodyDriveState],
    config: JointDriveConfig<'_>,
) {
    // Build one bounded entity lookup so each joint resolves its parent in constant expected time.
    let entity_indexes = entities
        .iter()
        .enumerate()
        .map(|(index, entity)| (*entity, index))
        .collect::<HashMap<_, _>>();

    for (child_position, child_entity) in entities.iter().copied().enumerate() {
        calculate_child_joint_target(
            world,
            child_entity,
            child_position,
            &entity_indexes,
            states,
            &config,
        );
    }
}

/// Current and target child motion expressed in parent joint frames.
#[derive(Clone, Copy, Debug)]
struct JointFrameState {
    /// Parent joint frame in world coordinates for torque rotation.
    current_frame: Quat,
    /// Current child orientation relative to the current parent joint frame.
    current_relative: Quat,
    /// Target child orientation relative to the target parent joint frame.
    target_relative: Quat,
    /// Current child-minus-parent angular velocity in world coordinates.
    current_relative_velocity: Vec3,
    /// Target angular velocity expressed in the target parent joint frame.
    target_relative_velocity: Vec3,
}

/// Derives current and target child motion in the profile joint's parent frame.
fn joint_frame_state(
    joint: JointToParent,
    basis: Quat,
    parent: BodyDriveState,
    child: BodyDriveState,
    parent_target: Isometry3d,
    child_target: Isometry3d,
) -> JointFrameState {
    // Compose the authored joint frame separately with current and target parent poses.
    // The basis turns both frames onto the joint's limit axes.
    let current_frame = parent.current_pose.rotation * joint.frame.rotation * basis;
    let target_frame = parent_target.rotation * joint.frame.rotation * basis;
    JointFrameState {
        current_frame,
        current_relative: current_frame.inverse() * child.current_pose.rotation * basis,
        target_relative: target_frame.inverse() * child_target.rotation * basis,
        current_relative_velocity: child.current_velocity.angular - parent.current_velocity.angular,
        target_relative_velocity: target_frame.inverse()
            * (child.target_velocity.angular - parent.target_velocity.angular),
    }
}

/// Computes stable-PD and soft-limit torque for a backend without native motors.
fn fallback_joint_torque(
    joint: JointToParent,
    frames: JointFrameState,
    child_inertia: f32,
    delta_seconds: f32,
    motor: JointMotorValues,
    has_asymmetric_swing_limits: bool,
) -> Vec3 {
    // Apply stable-PD feedback before soft-limit restoration in the same joint frame.
    let mut torque = stable_pd_torque(
        StablePdInput {
            frame_rotation: frames.current_frame,
            current_relative_rotation: frames.current_relative,
            target_relative_rotation: frames.target_relative,
            relative_angular_velocity: frames.current_relative_velocity,
            target_angular_velocity: frames.target_relative_velocity,
            inertia: child_inertia,
            delta_seconds,
        },
        motor,
    );
    if !has_asymmetric_swing_limits {
        torque += soft_limit_torque(
            frames.current_frame,
            frames.current_relative,
            joint.limits,
            child_inertia,
        );
    }
    torque
}

/// Computes one child target and adds fallback torque to its bodies when required.
fn calculate_child_joint_target(
    world: &mut World,
    child_entity: Entity,
    child_position: usize,
    entity_indexes: &HashMap<Entity, usize>,
    states: &mut [BodyDriveState],
    config: &JointDriveConfig<'_>,
) {
    // Resolve only joints whose parent and both captured target poses are available.
    let Some(joint) = world.get::<JointToParent>(child_entity).copied() else {
        return;
    };
    let Some(parent_position) = entity_indexes.get(&joint.parent).copied() else {
        return;
    };
    let (Some(parent), Some(child)) = (
        states.get(parent_position).copied(),
        states.get(child_position).copied(),
    ) else {
        return;
    };
    let (Some(parent_target), Some(child_target)) = (parent.target_pose, child.target_pose) else {
        return;
    };

    // Express current and target rotation and velocity in the authored parent joint frame.
    let basis = world
        .get::<super::body::JointBasis>(child_entity)
        .map_or(Quat::IDENTITY, |basis| basis.0);
    let frames = joint_frame_state(joint, basis, parent, child, parent_target, child_target);

    // Combine whole-character muscle strength with the checked child-body override.
    let muscle = config.drive.muscle()
        * world
            .get::<BodyIndex>(child_entity)
            .and_then(|index| config.weights.get(index.get()))
            .map_or(1.0, super::components::BodyWeights::muscle);
    let motor = joint_motor_values(muscle, joint.max_torque, &config.settings);

    // Publish the native target before calculating any backend fallback torque.
    if let Ok(mut entity) = world.get_entity_mut(child_entity) {
        entity.insert(JointDriveTarget {
            rotation: frames.target_relative,
            angular_velocity: frames.target_relative_velocity,
            stiffness: motor.stiffness,
            damping: motor.damping,
            max_torque: motor.max_torque,
        });
    }

    // Supply equal and opposite fallback torque when the backend lacks native joint motors.
    if !config.capabilities.has_native_joint_motors {
        let torque = fallback_joint_torque(
            joint,
            frames,
            child.inertia,
            config.delta_seconds,
            motor,
            config.capabilities.has_asymmetric_swing_limits,
        );
        if let Some(child_state) = states.get_mut(child_position) {
            child_state.output.joint_torque += torque;
        }
        if let Some(parent_state) = states.get_mut(parent_position) {
            parent_state.output.joint_torque -= torque;
        }
    }
}

/// Adds angular restoring torque when a backend cannot enforce asymmetric
/// limits.
fn soft_limit_torque(
    frame_rotation: Quat,
    relative_rotation: Quat,
    limits: JointLimits,
    inertia: f32,
) -> Vec3 {
    // Use the positive-w quaternion representative so equivalent rotations produce the same axes.
    let mut rotation = relative_rotation;
    if rotation.w < 0.0 {
        rotation = -rotation;
    }
    let x_angle = 2.0 * rotation.x.atan2(rotation.w);
    let y_angle = 2.0 * rotation.y.atan2(rotation.w);
    let z_angle = 2.0 * rotation.z.atan2(rotation.w);
    let stiffness = (std::f32::consts::TAU * 8.0).powi(2);
    // Compute each authored range independently, preserving X bend, Y twist, and Z bend order.
    let local_torque = Vec3::new(
        inertia * stiffness * angular_limit_excess(x_angle, limits.x),
        inertia * stiffness * angular_limit_excess(y_angle, limits.twist),
        inertia * stiffness * angular_limit_excess(z_angle, limits.z),
    );
    frame_rotation * local_torque
}

/// Returns the signed amount by which one angle lies outside its inclusive
/// range.
fn angular_limit_excess(angle: f32, range: crate::profile::AngleRange) -> f32 {
    // Preserve the configured boundary exactly and restore toward its nearest endpoint.
    if angle < range.min {
        range.min - angle
    } else if angle > range.max {
        range.max - angle
    } else {
        0.0
    }
}

/// Maps invalid input to zero and clamps finite strength to the unit interval.
const fn clamp_unit(value: f32) -> f32 {
    // Keep strengths finite before they scale any force or torque calculation.
    if value.is_finite() {
        value.clamp(0.0, 1.0)
    } else {
        0.0
    }
}

/// Maps negative and non-finite physical magnitudes to zero.
const fn finite_nonnegative(value: f32) -> f32 {
    // Convert invalid physical magnitudes to zero before they reach backend outputs.
    if value.is_finite() {
        value.max(0.0)
    } else {
        0.0
    }
}

/// Limits a vector's magnitude without changing its direction.
fn limit_vector(value: Vec3, max_magnitude: f32) -> Vec3 {
    // Preserve direction when clamping a finite vector to its configured magnitude.
    let magnitude = value.length();
    if magnitude.is_finite() && magnitude > max_magnitude && magnitude > f32::EPSILON {
        value * (max_magnitude / magnitude)
    } else if magnitude.is_finite() {
        value
    } else {
        Vec3::ZERO
    }
}

#[cfg(test)]
mod tests {
    //! Numerical checks for the two core feedback drives.

    use std::collections::HashMap;

    use bevy::asset::Handle;
    use bevy::math::{Isometry3d, Quat, Vec3};
    use bevy::prelude::{Entity, World};
    use bevy::time::{Fixed, Time};

    use crate::profile::{AngleRange, BodyIndex, JointLimits, RagdollProfile};
    use crate::runtime::backend::BackendCapabilities;
    use crate::runtime::body::{BodyDriveOutput, BodyVelocity, JointDriveTarget, JointToParent};
    use crate::runtime::components::{
        Ragdoll, RagdollBodies, RagdollBodyOf, RagdollBodyWeights, RagdollDrive, RagdollMode,
        RagdollTargetPose,
    };
    use crate::runtime::pin::{PinSettings, PinTargets};
    use crate::runtime::settings::RagdollPhysicsSettings;

    use super::{
        BodyDriveState, JointDriveConfig, PinDriveInput, StablePdInput, body_pin_output,
        calculate_child_joint_target, clamp_unit, drive, finite_nonnegative, joint_motor_values,
        limit_vector, pin_drive, rotation_error, soft_limit_torque, stable_pd_torque,
    };

    /// Creates the resources and one active character required by `drive`.
    fn drive_world() -> (World, Entity) {
        let mut world = World::new();
        // Drive reads settings, capabilities, and the fixed step; all three are present here.
        world.insert_resource(RagdollPhysicsSettings::default());
        world.insert_resource(BackendCapabilities::default());
        world.insert_resource(Time::<Fixed>::from_hz(60.0));
        // One dynamic character with no bodies; each test adds the bodies it needs.
        let character = world
            .spawn((
                Ragdoll::new(Handle::<RagdollProfile>::default()),
                RagdollMode::Dynamic,
            ))
            .id();
        (world, character)
    }

    /// Drive returns without a fixed clock or physics settings resource.
    #[test]
    fn drive_requires_fixed_time_and_settings() {
        // Without Time<Fixed>, drive has no step length and must return early.
        let mut no_clock = World::new();
        drive(&mut no_clock);

        // With a clock but no RagdollPhysicsSettings it must also return early.
        let mut no_settings = World::new();
        no_settings.insert_resource(Time::<Fixed>::from_hz(60.0));
        drive(&mut no_settings);
    }

    /// Applies pin masks and per-character controller values to body outputs.
    #[test]
    fn body_pin_output_respects_targets_and_settings() {
        // With no pin targets the body gets no pin force but keeps its speed caps.
        let settings = RagdollPhysicsSettings::default();
        let unpinned = far_target_pin_output(PinTargets::none(), settings);
        assert_eq!(
            (unpinned.pin_force, unpinned.max_linear_speed),
            (Vec3::ZERO, Some(10.0))
        );

        // A 0.5 N force cap must bound the pin even though the target is far away.
        let defaults = PinSettings::default();
        let limited_settings = PinSettings::new(
            defaults.frequency_hz(),
            defaults.damping_ratio(),
            0.5,
            defaults.max_torque(),
            0.0,
        );
        let mut physics_settings = settings;
        limited_settings.override_shared(&mut physics_settings);
        let pinned = far_target_pin_output(PinTargets::all(), physics_settings);
        // The capped force and the angular speed cap both reach the output.
        let is_capped = (pinned.pin_force.length() - 0.5).abs() < 1.0e-5;
        assert_eq!((is_capped, pinned.max_angular_speed), (true, Some(20.0)));
    }

    /// Computes the pin output of one body ten metres from its target.
    fn far_target_pin_output(
        targets: PinTargets,
        settings: RagdollPhysicsSettings,
    ) -> BodyDriveOutput {
        let state = BodyDriveState {
            entity: Entity::PLACEHOLDER,
            target_pose: Some(Isometry3d::from_translation(Vec3::X * 10.0)),
            target_velocity: BodyVelocity::default(),
            current_pose: Isometry3d::IDENTITY,
            current_velocity: BodyVelocity::default(),
            mass: 1.0,
            inertia: 1.0,
            output: BodyDriveOutput::default(),
        };
        body_pin_output(
            state,
            Some(body_index(0)),
            RagdollDrive::default(),
            &RagdollBodyWeights::default(),
            targets,
            settings,
            true,
        )
    }

    /// Reads a character's pin settings before computing every body output.
    #[test]
    fn drive_applies_per_character_pin_settings() {
        let (mut world, character) = drive_world();
        // A distant target that a zero-force pin setting must ignore.
        let mut targets = RagdollTargetPose::default();
        targets.record(
            vec![Isometry3d::from_translation(Vec3::X * 10.0)],
            vec![BodyVelocity::default()],
        );
        world
            .get_entity_mut(character)
            .unwrap()
            .insert((PinSettings::new(1.5, 1.0, 0.0, 400.0, 2.0), targets));
        let body = world.spawn((RagdollBodyOf(character), body_index(0))).id();

        // The character's own PinSettings override the shared defaults.
        drive(&mut world);

        let output = world
            .get::<BodyDriveOutput>(body)
            .expect("dynamic bodies receive a drive output");
        assert_eq!(output.pin_force, Vec3::ZERO);
    }

    /// A joint index that does not match the gathered state vector is skipped.
    #[test]
    fn joint_drive_skips_inconsistent_body_state_indexes() {
        // The child's parent is known, but the child has no drive state slot.
        let mut world = World::new();
        let parent = world.spawn_empty().id();
        let child = world
            .spawn(JointToParent {
                parent,
                frame: Isometry3d::IDENTITY,
                limits: locked_limits(),
                max_torque: 10.0,
            })
            .id();
        let mut states = [BodyDriveState {
            entity: parent,
            target_pose: Some(Isometry3d::IDENTITY),
            target_velocity: BodyVelocity::default(),
            current_pose: Isometry3d::IDENTITY,
            current_velocity: BodyVelocity::default(),
            mass: 1.0,
            inertia: 1.0,
            output: BodyDriveOutput::default(),
        }];
        // Only the parent is indexed, so the child lookup fails.
        let entity_indexes = HashMap::from([(parent, 0)]);
        let weights = RagdollBodyWeights::default();
        let config = JointDriveConfig {
            drive: RagdollDrive::default(),
            weights: &weights,
            settings: RagdollPhysicsSettings::default(),
            capabilities: BackendCapabilities::default(),
            delta_seconds: 1.0 / 60.0,
        };

        // The joint is skipped without writing a target or touching the parent output.
        calculate_child_joint_target(&mut world, child, 1, &entity_indexes, &mut states, &config);

        assert!(world.get::<JointDriveTarget>(child).is_none());
        assert_eq!(
            states
                .first()
                .expect("the parent drive state exists")
                .output,
            BodyDriveOutput::default()
        );
    }

    /// A free unit-inertia pendulum converges without exceeding the allowed
    /// overshoot.
    #[test]
    fn stable_pd_pendulum_converges_with_bounded_overshoot() {
        // A unit-inertia pendulum driven toward 0.3 rad for one second at 60 Hz.
        let settings = RagdollPhysicsSettings::default();
        let motor = joint_motor_values(1.0, 10.0, &settings);
        let target = Quat::from_rotation_z(0.3);
        let mut rotation = Quat::IDENTITY;
        let mut angular_velocity = Vec3::ZERO;
        let mut maximum_angle: f32 = 0.0;
        let delta_seconds = 1.0 / 60.0;

        // Integrate explicitly so the test checks the controller, not an engine.
        for _step in 0..60 {
            let torque = stable_pd_torque(
                StablePdInput {
                    frame_rotation: Quat::IDENTITY,
                    current_relative_rotation: rotation,
                    target_relative_rotation: target,
                    relative_angular_velocity: angular_velocity,
                    target_angular_velocity: Vec3::ZERO,
                    inertia: 1.0,
                    delta_seconds,
                },
                motor,
            );
            angular_velocity += torque * delta_seconds;
            rotation =
                (Quat::from_scaled_axis(angular_velocity * delta_seconds) * rotation).normalize();
            maximum_angle = maximum_angle.max(rotation.to_axis_angle().1);
        }

        // It must settle within 1 degree and overshoot by at most 5 degrees.
        assert!(rotation.angle_between(target).to_degrees() < 1.0);
        assert!(maximum_angle <= 0.3 + 5.0_f32.to_radians());
    }

    /// A free unit-mass body reaches a one-metre pin target without excess
    /// overshoot.
    #[test]
    fn pin_reaches_target_without_excess_overshoot() {
        // A unit mass pulled one metre along X for two seconds at 60 Hz.
        let settings = RagdollPhysicsSettings::default();
        let mut position = Vec3::ZERO;
        let mut velocity = Vec3::ZERO;
        let target = Isometry3d::from_translation(Vec3::X);
        let delta_seconds = 1.0 / 60.0;
        let mut maximum_x: f32 = 0.0;

        // Integrate explicitly so the test checks the pin controller alone.
        for _step in 0..120 {
            let output = pin_drive(
                PinDriveInput {
                    current_pose: Isometry3d::from_translation(position),
                    target_pose: target,
                    current_velocity: BodyVelocity {
                        linear: velocity,
                        angular: Vec3::ZERO,
                    },
                    target_velocity: BodyVelocity::default(),
                    mass: 1.0,
                    inertia: 1.0,
                    strength: 1.0,
                },
                &settings,
            );
            velocity += output.force * delta_seconds;
            position += velocity * delta_seconds;
            maximum_x = maximum_x.max(position.x);
        }

        // It must arrive within 1 cm and overshoot by at most 10 cm.
        assert!((position.x - 1.0).abs() < 0.01);
        assert!(maximum_x <= 1.1);
    }

    /// Invalid motor inputs become zero while finite inputs stay clamped.
    #[test]
    fn joint_motor_values_sanitize_non_finite_inputs() {
        let motor = joint_motor_values(f32::NAN, f32::INFINITY, &RagdollPhysicsSettings::default());

        assert_eq!(motor.stiffness, 0.0);
        assert_eq!(motor.damping, 20.0);
        assert_eq!(motor.max_torque, 0.0);
    }

    /// Quaternion sign changes preserve the same shortest-arc rotation vector.
    #[test]
    fn rotation_error_uses_the_shortest_quaternion_arc() {
        let rotation = Quat::from_rotation_z(0.5);

        assert!(rotation_error(Quat::IDENTITY, -rotation).abs_diff_eq(Vec3::Z * 0.5, 1.0e-6));
    }

    /// A zero or invalid step duration cannot produce stable-PD torque.
    #[test]
    fn stable_pd_torque_rejects_invalid_step_durations() {
        let motor = joint_motor_values(1.0, 10.0, &RagdollPhysicsSettings::default());

        // Collect every invalid step duration that still produced torque.
        let mismatches = [0.0, -1.0, f32::INFINITY, f32::NAN]
            .into_iter()
            .filter(|&delta_seconds| {
                stable_pd_torque(
                    StablePdInput {
                        frame_rotation: Quat::IDENTITY,
                        current_relative_rotation: Quat::IDENTITY,
                        target_relative_rotation: Quat::from_rotation_x(0.5),
                        relative_angular_velocity: Vec3::ZERO,
                        target_angular_velocity: Vec3::ZERO,
                        inertia: 1.0,
                        delta_seconds,
                    },
                    motor,
                ) != Vec3::ZERO
            })
            .collect::<Vec<_>>();
        assert_eq!(mismatches.len(), 0, "{mismatches:?}");
    }

    /// Soft joint limits restore both ends of each configured interval.
    #[test]
    fn soft_limit_torque_restores_beyond_both_limits() {
        // The same 0.1 rad limit on every axis.
        let range = AngleRange {
            min: -0.1,
            max: 0.1,
        };
        let limits = JointLimits {
            x: range,
            twist: range,
            z: range,
        };
        // Rotations past each end and one inside the range.
        let upper = soft_limit_torque(Quat::IDENTITY, Quat::from_rotation_x(0.2), limits, 1.0);
        let lower = soft_limit_torque(Quat::IDENTITY, -Quat::from_rotation_x(-0.2), limits, 1.0);
        let inside = soft_limit_torque(Quat::IDENTITY, Quat::from_rotation_x(0.05), limits, 1.0);

        // Torque pushes back toward the range from either side and is zero inside it.
        assert!(upper.x < 0.0);
        assert!(lower.x > 0.0);
        assert_eq!(inside, Vec3::ZERO);
    }

    /// Numeric guards clamp strength and physical magnitudes before vector
    /// limiting.
    #[test]
    fn numeric_drive_helpers_handle_finite_and_invalid_values() {
        // Unit clamping maps out-of-range and NaN values into 0..=1.
        assert_eq!([2.0, -1.0, f32::NAN].map(clamp_unit), [1.0, 0.0, 0.0]);
        // Negative and infinite magnitudes become zero.
        assert_eq!(
            [2.0, -1.0, f32::INFINITY].map(finite_nonnegative),
            [2.0, 0.0, 0.0]
        );
        // Vectors are capped by length, and an overflowing length becomes zero.
        assert_eq!(
            [Vec3::X * 2.0, Vec3::X * 0.5, Vec3::splat(f32::MAX)]
                .map(|vector| limit_vector(vector, 1.0)),
            [Vec3::X, Vec3::X * 0.5, Vec3::ZERO]
        );
    }

    /// Drive skips inactive characters and characters without dynamic bodies.
    #[test]
    fn drive_skips_inactive_and_empty_characters() {
        let (mut world, _) = drive_world();
        // An Animated character and a Dynamic character without bodies.
        let animated = world
            .spawn((
                Ragdoll::new(Handle::<RagdollProfile>::default()),
                RagdollMode::Animated,
            ))
            .id();
        let empty = world
            .spawn((
                Ragdoll::new(Handle::<RagdollProfile>::default()),
                RagdollMode::Dynamic,
                RagdollBodies::default(),
            ))
            .id();

        // Neither has anything to drive, so neither gains a target pose.
        drive(&mut world);

        assert!(world.get::<RagdollTargetPose>(animated).is_none());
        assert!(world.get::<RagdollTargetPose>(empty).is_none());
    }

    /// Dynamic drive handles missing bodies, indices, targets, and joint
    /// parents.
    #[test]
    fn drive_skips_bodies_without_required_identity_or_target_data() {
        let (mut world, character) = drive_world();
        // One body without an index and one whose index has no recorded target.
        let missing_index = world.spawn(RagdollBodyOf(character)).id();
        let missing_target = world.spawn((RagdollBodyOf(character), body_index(0))).id();
        world
            .get_entity_mut(character)
            .unwrap()
            .insert(RagdollTargetPose::default());

        // Drive must not pin either body but still apply the character's speed caps.
        drive(&mut world);

        // Both bodies get no pin force and the character's speed caps.
        let outputs = [missing_index, missing_target].map(|body| {
            let output = world
                .get::<BodyDriveOutput>(body)
                .expect("an unpinned body still receives its active character limit");
            (
                output.pin_force,
                output.max_linear_speed,
                output.max_angular_speed,
            )
        });
        assert_eq!(outputs, [(Vec3::ZERO, Some(10.0), Some(20.0)); 2]);
    }

    /// Joint drive skips missing parents and targets without writing motor
    /// state.
    #[test]
    fn drive_skips_joints_without_parent_or_target_pose() {
        let (mut world, character) = drive_world();
        // A valid parent, a child whose target pose is missing, and a child with no parent entity.
        let parent = world.spawn((RagdollBodyOf(character), body_index(0))).id();
        let child = spawn_jointed_body(&mut world, character, 1, parent);
        let missing_parent = spawn_jointed_body(&mut world, character, 2, Entity::PLACEHOLDER);
        // Only body 0 has a target, so neither joint can compute one.
        let mut targets = RagdollTargetPose::default();
        targets.record(vec![Isometry3d::IDENTITY], vec![BodyVelocity::default()]);
        world.get_entity_mut(character).unwrap().insert(targets);

        drive(&mut world);

        // Both joints are skipped while the parent body is still driven.
        let has_target =
            [child, missing_parent].map(|body| world.get::<JointDriveTarget>(body).is_some());
        assert_eq!(has_target, [false, false]);
        assert!(world.get::<BodyDriveOutput>(parent).is_some());
    }

    /// Native joint motors receive targets without fallback torque writes.
    #[test]
    fn drive_omits_fallback_torque_for_native_joint_motors() {
        let (mut world, character) = drive_world();
        // A backend with native motors applies joint targets itself.
        world
            .get_resource_mut::<BackendCapabilities>()
            .unwrap()
            .has_native_joint_motors = true;
        let parent = world.spawn((RagdollBodyOf(character), body_index(0))).id();
        let child = spawn_jointed_body(&mut world, character, 1, parent);
        // Body 1's target is rotated, so the joint has real work to do.
        let mut targets = RagdollTargetPose::default();
        targets.record(
            vec![
                Isometry3d::IDENTITY,
                Isometry3d::new(Vec3::ZERO, Quat::from_rotation_x(0.3)),
            ],
            vec![BodyVelocity::default(); 2],
        );
        world.get_entity_mut(character).unwrap().insert(targets);

        drive(&mut world);

        // The joint target is written but no fallback torque reaches the parent.
        assert!(world.get::<JointDriveTarget>(child).is_some());
        let torques = [parent, child].map(|body| {
            world
                .get::<BodyDriveOutput>(body)
                .expect("each body receives a drive output")
                .joint_torque
        });
        assert_eq!(torques, [Vec3::ZERO; 2]);
    }

    /// Spawns body `index` of `character` with a locked joint to `parent`.
    fn spawn_jointed_body(
        world: &mut World,
        character: Entity,
        index: usize,
        parent: Entity,
    ) -> Entity {
        world
            .spawn((
                RagdollBodyOf(character),
                body_index(index),
                JointToParent {
                    parent,
                    frame: Isometry3d::IDENTITY,
                    limits: locked_limits(),
                    max_torque: 10.0,
                },
            ))
            .id()
    }

    /// Converts a test body index into the validated profile index type.
    fn body_index(value: usize) -> BodyIndex {
        BodyIndex::try_from(value).expect("test body indices are in range")
    }

    /// Builds zero-width angular limits for drive branch tests.
    fn locked_limits() -> JointLimits {
        let locked = AngleRange { min: 0.0, max: 0.0 };
        JointLimits {
            x: locked,
            twist: locked,
            z: locked,
        }
    }
}
