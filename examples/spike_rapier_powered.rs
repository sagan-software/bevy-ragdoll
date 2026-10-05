//! A throwaway Rapier powered-ragdoll spike and its command-line interface.

use bevy::app::FixedUpdate;
#[cfg(feature = "visual")]
use bevy::app::{AppExit, Startup, Update};
use bevy::ecs::schedule::IntoScheduleConfigs;
use bevy::ecs::system::SystemParam;
use bevy::math::{Isometry3d, Quat, Vec3};
use bevy::prelude::{App, MinimalPlugins, Resource};
#[cfg(feature = "visual")]
use bevy::prelude::{
    Assets, Camera3d, Capsule3d, Color, Commands, Cuboid, DefaultPlugins, DirectionalLight, Mesh,
    Mesh3d, MeshMaterial3d, Meshable, MessageWriter, Plane3d, PluginGroup, Res, ResMut, Sphere,
    StandardMaterial, Window, WindowPlugin,
};
use bevy::prelude::{Component, Entity, Transform, World};
#[cfg(feature = "visual")]
use bevy::render::view::screenshot::{Screenshot, save_to_disk};
use bevy::time::{Fixed, Time, TimeUpdateStrategy};
use bevy_ragdoll::{Body, BodyIndex, Joint, JointLimits, ProfileSpec, RagdollProfile, ShapeSpec};
use bevy_rapier3d::prelude::{
    ActiveHooks, BevyPhysicsHooks, Collider, ColliderMassProperties, Damping, ExternalForce,
    Friction, GenericJoint, GenericJointBuilder, ImpulseJoint, JointAxesMask, JointAxis,
    MassProperties, PhysicsSet, RapierPhysicsPlugin, RigidBody, Sleeping, SolverFlags,
    TimestepMode, TypedJoint, Velocity,
};
use clap::{Parser, ValueEnum};
use serde::Serialize;
use std::collections::HashSet;
use std::fmt;
use std::fs;
use std::io;
use std::num::NonZeroUsize;
use std::path::PathBuf;
use std::str::FromStr;
use std::time::{Duration, Instant};

/// The profile asset used to exercise the imported TGF human rig.
const TGF_PROFILE_RON: &str = include_str!("../assets/profiles/tgf_human.ragdoll.ron");

/// A command-line number failed to meet its domain constraint.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ValueParseError(&'static str);

/// Displays the constraint that rejected a command-line number.
impl fmt::Display for ValueParseError {
    /// Writes the stable CLI validation message.
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(self.0)
    }
}

/// Exposes CLI validation failures to Clap's value parser.
impl std::error::Error for ValueParseError {}

/// Parses finite values before applying a domain-specific constraint.
fn parse_finite_f32(input: &str) -> Result<f32, ValueParseError> {
    let value = input
        .parse::<f32>()
        .map_err(|_| ValueParseError("expected a finite number"))?;
    if value.is_finite() {
        Ok(value)
    } else {
        Err(ValueParseError("expected a finite number"))
    }
}

/// A normalized pelvis and chest pin weight in the closed interval `[0, 1]`.
#[derive(Clone, Copy, Debug, PartialEq)]
struct PinStrength(f32);

/// Parses a finite pin weight and rejects values outside `[0, 1]`.
impl FromStr for PinStrength {
    type Err = ValueParseError;

    /// Converts the command-line spelling to a validated pin weight.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        let value = parse_finite_f32(input)?;
        if (0.0..=1.0).contains(&value) {
            Ok(Self(value))
        } else {
            Err(ValueParseError("pin strength must be between 0 and 1"))
        }
    }
}

/// Formats a pin weight for Clap's default-value support.
impl fmt::Display for PinStrength {
    /// Writes the normalized value.
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(formatter)
    }
}

/// A normalized actuator strength in the closed interval `[0, 1]`.
#[derive(Clone, Copy, Debug, PartialEq)]
struct MuscleStrength(f32);

/// Parses a finite muscle strength and rejects values outside `[0, 1]`.
impl FromStr for MuscleStrength {
    type Err = ValueParseError;

    /// Converts the command-line spelling to a validated muscle strength.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        let value = parse_finite_f32(input)?;
        if (0.0..=1.0).contains(&value) {
            Ok(Self(value))
        } else {
            Err(ValueParseError("muscle strength must be between 0 and 1"))
        }
    }
}

/// Formats a muscle strength for Clap's default-value support.
impl fmt::Display for MuscleStrength {
    /// Writes the normalized value.
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(formatter)
    }
}

/// A ragdoll count between one and the measured limit of 128.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct RagdollCount(NonZeroUsize);

/// Parses a positive ragdoll count up to the supported stress limit.
impl FromStr for RagdollCount {
    type Err = ValueParseError;

    /// Converts the command-line spelling to a validated ragdoll count.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        let value = input
            .parse::<usize>()
            .map_err(|_| ValueParseError("count must be an integer from 1 to 128"))?;
        if value > 128 {
            return Err(ValueParseError("count must be an integer from 1 to 128"));
        }
        NonZeroUsize::new(value)
            .map(Self)
            .ok_or(ValueParseError("count must be an integer from 1 to 128"))
    }
}

/// Formats a ragdoll count for Clap's default-value support.
impl fmt::Display for RagdollCount {
    /// Writes the positive count.
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(formatter)
    }
}

/// A positive finite simulation duration stored as a standard duration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct SimulationDuration(Duration);

/// Parses positive finite seconds and stores the result as `Duration`.
impl FromStr for SimulationDuration {
    type Err = ValueParseError;

    /// Converts seconds at the CLI boundary to a validated duration.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        let seconds = input
            .parse::<f64>()
            .map_err(|_| ValueParseError("seconds must be a positive finite number"))?;
        if !seconds.is_finite() || seconds <= 0.0 {
            return Err(ValueParseError("seconds must be a positive finite number"));
        }
        Duration::try_from_secs_f64(seconds)
            .map(Self)
            .map_err(|_| ValueParseError("seconds exceed the supported duration"))
    }
}

/// Formats a simulation duration as seconds for Clap's default-value support.
impl fmt::Display for SimulationDuration {
    /// Writes seconds with enough precision to preserve the stored duration.
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.as_secs_f64().fmt(formatter)
    }
}

/// A positive finite actuator frequency in hertz.
#[derive(Clone, Copy, Debug, PartialEq)]
struct NaturalFrequency(f32);

/// Parses a positive finite frequency in hertz.
impl FromStr for NaturalFrequency {
    type Err = ValueParseError;

    /// Converts the command-line spelling to a validated frequency.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        let value = parse_finite_f32(input)?;
        if value > 0.0 {
            Ok(Self(value))
        } else {
            Err(ValueParseError("frequency must be greater than zero"))
        }
    }
}

/// Formats a frequency for Clap's default-value support.
impl fmt::Display for NaturalFrequency {
    /// Writes the frequency in hertz.
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(formatter)
    }
}

/// The closed actuator paths compared by the spike.
#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
enum DriveMode {
    /// Rapier's acceleration-based angular joint motors.
    Motor,
    /// The spike's stable-PD torque system.
    Torque,
}

/// Marks the headless simulation mode when `--headless` is present.
#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
enum Headless {
    /// Run without a rendered window.
    Enabled,
}

/// Command-line arguments accepted by the spike executable.
#[derive(Debug, Parser)]
#[command(name = "spike_rapier_powered")]
struct Args {
    /// Selects Rapier's motor or the explicit torque driver.
    #[arg(long, value_enum, default_value_t = DriveMode::Motor)]
    drive: DriveMode,

    /// Run the fixed-step simulation without a rendered window.
    #[arg(
        long,
        value_enum,
        num_args = 0..=1,
        default_missing_value = "enabled",
        require_equals = true
    )]
    headless: Option<Headless>,

    /// Applies the pin model to the pelvis and chest.
    #[arg(long, default_value = "0")]
    pin: PinStrength,

    /// Scales actuator stiffness and torque limits.
    #[arg(long, default_value = "1")]
    muscle: MuscleStrength,

    /// Number of ragdolls to spawn, limited to the 128-body stress run.
    #[arg(long, default_value = "1")]
    count: RagdollCount,

    /// Duration of the fixed-step simulation in seconds.
    #[arg(long = "seconds", default_value = "10")]
    duration: SimulationDuration,

    /// Actuator natural frequency in hertz.
    #[arg(long, default_value = "4")]
    frequency: NaturalFrequency,

    /// JSON result path, required for headless runs.
    #[arg(long)]
    report: Option<PathBuf>,

    /// PNG screenshot path for a visible run.
    #[arg(long)]
    screenshot: Option<PathBuf>,
}

/// The execution path and its matching output configuration.
#[derive(Clone, Debug, PartialEq, Eq)]
enum RunMode {
    /// Run fixed steps without a window and write the metrics JSON.
    Headless {
        /// Destination file for the headless metrics report.
        report: PathBuf,
    },
    /// Run with a window and optionally save a screenshot.
    Visual {
        /// Optional destination file for a visible-run screenshot.
        screenshot: Option<PathBuf>,
    },
}

/// A headless or visible CLI configuration has incompatible output options.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ConfigError {
    /// A headless run omitted its report file.
    HeadlessRequiresReport,
    /// A visible run supplied a headless report file.
    ReportRequiresHeadless,
    /// A headless run supplied a visible screenshot file.
    ScreenshotRequiresVisual,
}

/// Displays the configuration policy that rejected the command line.
impl fmt::Display for ConfigError {
    /// Writes the stable mode configuration error.
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::HeadlessRequiresReport => "--headless requires --report <json>",
            Self::ReportRequiresHeadless => "--report requires --headless",
            Self::ScreenshotRequiresVisual => "--screenshot cannot be used with --headless",
        };
        formatter.write_str(message)
    }
}

/// Exposes a machine-readable category for CLI configuration failures.
impl std::error::Error for ConfigError {}

/// Validated CLI options after mode-specific output policy checks.
#[derive(Clone, Debug, PartialEq)]
struct RunConfig {
    /// Whether to run headless or with a rendered window.
    mode: RunMode,
    /// Joint motor or stable-PD torque driver.
    drive: DriveMode,
    /// Pin weight for pelvis and chest.
    pin: PinStrength,
    /// Whole-ragdoll muscle strength.
    muscle: MuscleStrength,
    /// Number of ragdoll instances.
    count: RagdollCount,
    /// Requested simulation duration.
    duration: SimulationDuration,
    /// Joint motor natural frequency.
    frequency: NaturalFrequency,
}

/// Checks output paths against the selected execution mode.
impl TryFrom<Args> for RunConfig {
    type Error = ConfigError;

    /// Converts parsed values to a mode-valid run configuration.
    fn try_from(args: Args) -> Result<Self, Self::Error> {
        // Select the output contract before moving each validated CLI field.
        let mode = match args.headless {
            Some(Headless::Enabled) => {
                let report = args.report.ok_or(ConfigError::HeadlessRequiresReport)?;
                if args.screenshot.is_some() {
                    return Err(ConfigError::ScreenshotRequiresVisual);
                }
                RunMode::Headless { report }
            }
            None => {
                if args.report.is_some() {
                    return Err(ConfigError::ReportRequiresHeadless);
                }
                RunMode::Visual {
                    screenshot: args.screenshot,
                }
            }
        };
        Ok(Self {
            mode,
            drive: args.drive,
            pin: args.pin,
            muscle: args.muscle,
            count: args.count,
            duration: args.duration,
            frequency: args.frequency,
        })
    }
}

/// Parses and validates the bundled TGF human profile.
fn load_tgf_profile() -> Result<RagdollProfile, Box<dyn std::error::Error>> {
    // Decode the checked RON document into its untrusted authoring model.
    let spec = ron::from_str::<ProfileSpec>(TGF_PROFILE_RON)?;
    // Validate body geometry, relationships, masses, transforms, and joint limits.
    Ok(RagdollProfile::new(spec)?)
}

/// Computes the bounded single-axis animation target for one profile bone.
fn target_angle(bone: &str, time_s: f64, limits: JointLimits) -> Vec3 {
    // The arms use a 0.5 Hz sine, while the legs use a 0.25 Hz squat cycle.
    let arm_sine = (std::f64::consts::TAU * 0.5 * time_s).sin() as f32;
    let squat = (0.5 * (1.0 - (std::f64::consts::TAU * 0.25 * time_s).cos())) as f32;
    let x = match bone {
        "upperarm_l" | "upperarm_r" => 0.6 * arm_sine,
        "lowerarm_l" | "lowerarm_r" => 0.4 * arm_sine,
        "thigh_l" | "thigh_r" => squat,
        "calf_l" | "calf_r" => -squat,
        _ => 0.0,
    };
    Vec3::new(
        x.clamp(limits.x.min, limits.x.max),
        0.0_f32.clamp(limits.twist.min, limits.twist.max),
        0.0_f32.clamp(limits.z.min, limits.z.max),
    )
}

/// Builds target body transforms in the profile's parent-first order.
fn target_poses(profile: &RagdollProfile, root: Isometry3d, time_s: f64) -> Vec<Isometry3d> {
    // Start from the profile's validated rest transforms under the instance root.
    let mut poses = Vec::with_capacity(profile.bodies().len());
    target_poses_into(profile, root, time_s, &mut poses);
    poses
}

/// Reuses caller capacity while deriving parent-first target poses.
fn target_poses_into(
    profile: &RagdollProfile,
    root: Isometry3d,
    time_s: f64,
    poses: &mut Vec<Isometry3d>,
) {
    poses.clear();
    poses.extend(profile.rest_poses(root));
    // Apply each target after its parent pose is available in parent-first order.
    for joint in profile.joints() {
        let child = joint.child().get();
        let parent = joint.parent().get();
        let limits = joint.limits();
        let angle = target_angle(profile.bodies()[child].bone(), time_s, limits);
        poses[child] = poses[parent]
            * joint.frame()
            * Isometry3d::new(Vec3::ZERO, Quat::from_rotation_x(angle.x));
    }
}

/// Identifies a body and its same-ragdoll contact exclusions to the hook filter.
#[derive(Clone, Copy, Debug, Component)]
struct RagdollMember {
    /// Entity used as the identity for the ragdoll containing this body.
    root: Entity,
    /// Checked profile index used for mask lookup.
    body: BodyIndex,
    /// Bodies excluded from contact with this body by profile index.
    no_contact_mask: u64,
}

/// The pose and world velocities requested for one body by the animation.
#[derive(Clone, Copy, Debug, Component)]
struct TargetPose {
    /// Desired rigid pose in world coordinates.
    pose: Isometry3d,
    /// Desired linear velocity in metres per second.
    linear_velocity: Vec3,
    /// Desired angular velocity in radians per second.
    angular_velocity: Vec3,
}

/// The bounded joint target and local angular velocity for one child body.
#[derive(Clone, Copy, Debug, Component)]
struct JointTarget {
    /// Desired X bend, Y twist, and Z bend angles in radians.
    angles: Vec3,
    /// Target angle derivative in radians per second for each joint axis.
    angular_velocity: Vec3,
}

/// Principal inertia and body mass used by the explicit torque driver.
#[derive(Clone, Copy, Debug, Component)]
struct BodyInertia {
    /// Validated body mass in kilograms.
    mass: f32,
    /// Principal moments in kilogram metres squared.
    principal: Vec3,
    /// Rotation from the body frame into the principal inertia frame.
    frame: Quat,
}

/// Entity mapping returned after one profile instance has been spawned.
struct SpawnedRagdoll {
    /// One dynamic body entity per profile body, in profile order.
    bodies: Vec<Entity>,
}

/// Returns the profile collision shape as a Rapier collider.
fn collider_for_body(body: &Body) -> Collider {
    match body.shape() {
        ShapeSpec::Capsule { a, b, radius } => Collider::capsule(*a, *b, *radius),
        ShapeSpec::Sphere { center, radius } => {
            Collider::compound(vec![(*center, Quat::IDENTITY, Collider::ball(*radius))])
        }
        ShapeSpec::Cuboid {
            center,
            rotation,
            half_extents,
        } => Collider::compound(vec![(
            *center,
            *rotation,
            Collider::cuboid(half_extents.x, half_extents.y, half_extents.z),
        )]),
    }
}

/// Computes profile mass properties and applies TGF's 0.08 metre inertia floor.
fn body_mass_properties(collider: &Collider, mass: f32) -> (MassProperties, BodyInertia) {
    // Give the authored collider its profile mass before flooring thin principal axes.
    let mut raw = collider.raw.mass_properties(1.0);
    raw.set_mass(mass, true);
    let floor = Vec3::splat(mass * 0.08_f32 * 0.08_f32);
    let principal = raw.principal_inertia().max(floor);
    let frame = raw.principal_inertia_local_frame;
    let raw = bevy_rapier3d::rapier::dynamics::MassProperties::with_principal_inertia_frame(
        raw.local_com,
        mass,
        principal,
        frame,
    );
    (
        MassProperties::from_rapier(raw),
        BodyInertia {
            mass,
            principal,
            frame,
        },
    )
}

/// Builds the Rapier joint frame, limits, locked axes, and disabled contact pair.
fn rapier_joint(joint: &Joint) -> GenericJoint {
    let limits = joint.limits();
    let mut locked_axes = JointAxesMask::LOCKED_SPHERICAL_AXES;
    if limits.x.is_locked() {
        locked_axes |= JointAxesMask::ANG_X;
    }
    if limits.twist.is_locked() {
        locked_axes |= JointAxesMask::ANG_Y;
    }
    if limits.z.is_locked() {
        locked_axes |= JointAxesMask::ANG_Z;
    }
    let frame = joint.frame();
    let mut builder = GenericJointBuilder::new(locked_axes)
        .local_anchor1(frame.translation.into())
        .local_basis1(frame.rotation)
        .local_anchor2(Vec3::ZERO)
        .local_basis2(Quat::IDENTITY);
    for (axis, range) in [
        (JointAxis::AngX, limits.x),
        (JointAxis::AngY, limits.twist),
        (JointAxis::AngZ, limits.z),
    ] {
        if !range.is_locked() {
            builder = builder.limits(axis, [range.min, range.max]);
        }
    }
    let mut data = builder.build();
    data.set_contacts_enabled(false);
    data
}

/// Spawns one profile instance with fixed-rate actuator targets and no-contact masks.
fn spawn_ragdoll(
    world: &mut World,
    profile: &RagdollProfile,
    root_pose: Isometry3d,
) -> SpawnedRagdoll {
    let poses = target_poses(profile, root_pose, 0.0);
    let mut bodies = Vec::with_capacity(profile.bodies().len());
    for _ in profile.bodies() {
        bodies.push(world.spawn_empty().id());
    }
    let root = bodies[0];
    // Create body entities in validated profile order so every index stays direct.
    for (position, body) in profile.bodies().iter().enumerate() {
        let collider = collider_for_body(body);
        let (mass_properties, inertia) = body_mass_properties(&collider, body.mass().kilograms());
        let pose = poses[position];
        let member = RagdollMember {
            root,
            body: body.index(),
            no_contact_mask: profile.no_contact_masks()[position],
        };
        world.entity_mut(bodies[position]).insert((
            RigidBody::Dynamic,
            collider,
            ColliderMassProperties::MassProperties(mass_properties),
            Damping {
                linear_damping: 0.0,
                angular_damping: 0.05,
            },
            ExternalForce::default(),
            Sleeping::disabled(),
            ActiveHooks::FILTER_CONTACT_PAIRS,
            member,
            inertia,
            TargetPose {
                pose,
                linear_velocity: Vec3::ZERO,
                angular_velocity: Vec3::ZERO,
            },
            Transform::from_translation(pose.translation.into()).with_rotation(pose.rotation),
            Velocity::default(),
        ));
    }
    // Attach one GenericJoint to each child entity after all parent entities exist.
    for joint in profile.joints() {
        let child = joint.child().get();
        let parent = joint.parent().get();
        let target = target_angle(profile.bodies()[child].bone(), 0.0, joint.limits());
        world.entity_mut(bodies[child]).insert((
            ImpulseJoint::new(
                bodies[parent],
                TypedJoint::GenericJoint(rapier_joint(joint)),
            ),
            JointTarget {
                angles: target,
                angular_velocity: Vec3::ZERO,
            },
        ));
    }
    SpawnedRagdoll { bodies }
}

/// Filters profile contact pairs while preserving all other dynamic contacts.
#[derive(SystemParam)]
struct ContactFilter<'w, 's> {
    /// Reads profile body membership for each candidate collider pair.
    members: bevy::prelude::Query<'w, 's, &'static RagdollMember>,
}

/// Checks profile exclusions for bodies that belong to one ragdoll instance.
fn contact_pair_excluded(first: &RagdollMember, second: &RagdollMember) -> bool {
    if first.root != second.root {
        return false;
    }
    let first_excludes_second = first.no_contact_mask & (1_u64 << second.body.get()) != 0;
    let second_excludes_first = second.no_contact_mask & (1_u64 << first.body.get()) != 0;
    first_excludes_second || second_excludes_first
}

/// Applies profile no-contact masks to same-ragdoll collider pairs.
impl BevyPhysicsHooks for ContactFilter<'_, '_> {
    /// Rejects only same-instance pairs present in the profile's symmetric mask.
    fn filter_contact_pair(
        &self,
        context: bevy_rapier3d::pipeline::PairFilterContextView<'_>,
    ) -> Option<SolverFlags> {
        let first = self.members.get(context.collider1()).ok();
        let second = self.members.get(context.collider2()).ok();
        if let (Some(first), Some(second)) = (first, second)
            && contact_pair_excluded(first, second)
        {
            return None;
        }
        Some(SolverFlags::COMPUTE_IMPULSES)
    }
}

/// Motor and pin parameters applied to one simulation.
#[derive(Clone, Copy, Debug)]
struct ControlSettings {
    /// Selected angular motor or explicit torque controller.
    drive: DriveMode,
    /// Pin force and torque weight in `[0, 1]`.
    pin: PinStrength,
    /// Motor strength in `[0, 1]`.
    muscle: MuscleStrength,
    /// Natural frequency in hertz.
    frequency: NaturalFrequency,
}

/// Entity and reusable pose buffers for one ragdoll instance.
struct RagdollInstance {
    /// Spawned bodies in parent-first profile order.
    bodies: Vec<Entity>,
    /// Root pose used to regenerate target poses.
    root_pose: Isometry3d,
    /// Reusable capacity for the current target pose pass.
    target_scratch: Vec<Isometry3d>,
    /// Reusable capacity for measuring current body poses.
    current_poses: Vec<Isometry3d>,
}

/// State and measurements shared by the fixed-step simulation systems.
#[derive(Resource)]
struct Simulation {
    /// Validated TGF human profile used to create bodies and drive targets.
    profile: RagdollProfile,
    /// Ragdoll instances in their stable spawn order.
    instances: Vec<RagdollInstance>,
    /// Controller values used by every instance.
    settings: ControlSettings,
    /// Fixed simulation time elapsed since the first target pose.
    elapsed: Duration,
    /// Profile positions for pelvis and chest pin application.
    pin_bodies: [BodyIndex; 2],
    /// Joint-angle error accumulated across all axes and fixed steps, radians.
    angle_error_sum: f64,
    /// Number of scalar joint-angle error samples.
    angle_error_samples: u64,
    /// Largest scalar joint-angle error observed, radians.
    max_angle_error: f32,
    /// Largest absolute pelvis height difference from its target, metres.
    max_pelvis_drift: f32,
    /// Simulated time when the largest pelvis height difference occurred, seconds.
    max_pelvis_drift_at_s: f64,
    /// Largest pin force magnitude applied to either pinned body, newtons.
    max_pin_force: f32,
    /// Pelvis height after the latest completed Rapier step, metres.
    final_pelvis_height: f32,
    /// Duration of each Rapier `StepSimulation` set, milliseconds.
    step_milliseconds: Vec<f64>,
    /// Bodies that exceeded 50 m/s or produced a non-finite pose or velocity.
    unstable_bodies: HashSet<Entity>,
    /// Start time captured immediately before the Rapier step set.
    step_started: Option<Instant>,
}

/// Serialized configuration fields for one headless run.
#[derive(Serialize)]
struct ReportConfiguration {
    /// Selected motor or torque driver.
    drive: &'static str,
    /// Pelvis and chest pin weight.
    pin: f32,
    /// Whole-ragdoll muscle strength.
    muscle: f32,
    /// Number of spawned ragdolls.
    count: usize,
    /// Requested duration in seconds.
    seconds: f64,
    /// Actuator natural frequency in hertz.
    natural_frequency_hz: f32,
    /// Rapier fixed-step frequency in hertz.
    fixed_frequency_hz: f64,
}

/// Serialized aggregate measurements from a headless run.
#[derive(Serialize)]
struct ReportMetrics {
    /// Number of completed Rapier fixed steps.
    fixed_steps: usize,
    /// Mean absolute scalar joint-angle error in degrees, or null without valid samples.
    mean_joint_angle_error_deg: Option<f64>,
    /// Largest absolute scalar joint-angle error in degrees, or null without valid samples.
    max_joint_angle_error_deg: Option<f64>,
    /// Largest pelvis target-height difference, metres.
    max_pelvis_height_drift_m: f32,
    /// Simulated time when the largest pelvis target-height difference occurred, seconds.
    max_pelvis_height_drift_at_s: f64,
    /// Largest requested pin force magnitude, newtons.
    max_pin_force_n: f32,
    /// Pelvis height after the final fixed step, metres.
    final_pelvis_height_m: f32,
    /// Mean time spent inside Rapier's step set per fixed step, milliseconds.
    mean_step_ms: Option<f64>,
    /// Nearest-rank 95th percentile Rapier step time, milliseconds.
    p95_step_ms: Option<f64>,
    /// Number of distinct bodies that exceeded the instability threshold.
    unstable_bodies: usize,
}

/// Versioned JSON report for one deterministic headless configuration.
#[derive(Serialize)]
struct SpikeReport {
    /// Schema version for the report structure.
    schema_version: u8,
    /// Parameters used to produce the measurements.
    configuration: ReportConfiguration,
    /// Measurements collected after each Rapier step.
    metrics: ReportMetrics,
}

/// Builds a one-ragdoll or stress-count Rapier app and its fixed simulation schedule.
fn build_headless_app(profile: RagdollProfile, config: &RunConfig) -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins)
        .add_plugins(bevy::transform::TransformPlugin)
        .insert_resource(Time::<Fixed>::from_hz(60.0))
        .insert_resource(TimeUpdateStrategy::FixedTimesteps(1))
        .insert_resource(TimestepMode::Fixed {
            dt: 1.0 / 60.0,
            substeps: 1,
        })
        .add_plugins(RapierPhysicsPlugin::<ContactFilter>::default().in_fixed_schedule());
    configure_simulation_systems(&mut app);
    initialize_simulation(&mut app, profile, config);
    app
}

/// Orders target updates, actuation, Rapier stepping, and metric collection.
fn configure_simulation_systems(app: &mut App) {
    app.add_systems(
        FixedUpdate,
        (
            update_targets.before(PhysicsSet::SyncBackend),
            update_motor_drive
                .after(update_targets)
                .before(PhysicsSet::SyncBackend),
            reset_external_forces
                .after(update_motor_drive)
                .before(PhysicsSet::SyncBackend),
            apply_torque_drive
                .after(reset_external_forces)
                .before(PhysicsSet::SyncBackend),
            apply_pins
                .after(apply_torque_drive)
                .before(PhysicsSet::SyncBackend),
            begin_step_timer
                .after(PhysicsSet::SyncBackend)
                .before(PhysicsSet::StepSimulation),
            end_step_timer
                .after(PhysicsSet::StepSimulation)
                .before(PhysicsSet::Writeback),
            record_metrics.after(PhysicsSet::Writeback),
        ),
    );
}

/// Spawns the ground and profile bodies, then installs their run state.
fn initialize_simulation(app: &mut App, profile: RagdollProfile, config: &RunConfig) {
    let count = config.count.0.get();
    spawn_ground(app.world_mut(), count as f32 + 5.0);
    let mut instances = Vec::with_capacity(count);
    for index in 0..count {
        let x = (index as f32 - (count as f32 - 1.0) * 0.5) * 2.0;
        let root_pose = Isometry3d::from_xyz(x, 0.0, 0.0);
        let spawned = spawn_ragdoll(app.world_mut(), &profile, root_pose);
        let mut current_poses = Vec::with_capacity(profile.bodies().len());
        target_poses_into(&profile, root_pose, 0.0, &mut current_poses);
        instances.push(RagdollInstance {
            bodies: spawned.bodies,
            root_pose,
            target_scratch: Vec::with_capacity(profile.bodies().len()),
            current_poses,
        });
    }
    let pin_bodies = [
        profile
            .body_index("pelvis")
            .expect("TGF profile has a pelvis"),
        profile
            .body_index("spine_04")
            .expect("TGF profile has a chest body"),
    ];
    app.insert_resource(Simulation {
        profile,
        instances,
        settings: ControlSettings {
            drive: config.drive,
            pin: config.pin,
            muscle: config.muscle,
            frequency: config.frequency,
        },
        elapsed: Duration::ZERO,
        pin_bodies,
        angle_error_sum: 0.0,
        angle_error_samples: 0,
        max_angle_error: 0.0,
        max_pelvis_drift: 0.0,
        max_pelvis_drift_at_s: 0.0,
        max_pin_force: 0.0,
        final_pelvis_height: 0.0,
        step_milliseconds: Vec::new(),
        unstable_bodies: HashSet::new(),
        step_started: None,
    });
}

/// Adds one static floor with the requested half-width in metres.
fn spawn_ground(world: &mut World, half_width: f32) {
    world.spawn((
        RigidBody::Fixed,
        Collider::cuboid(half_width, 0.1, 20.0),
        Friction::coefficient(0.7),
        Transform::from_xyz(0.0, -0.1, 0.0),
    ));
}

/// Advances target poses and finite-difference velocities before Rapier synchronization.
fn update_targets(
    fixed_time: bevy::prelude::Res<'_, Time<Fixed>>,
    mut simulation: bevy::prelude::ResMut<'_, Simulation>,
    mut poses: bevy::prelude::Query<'_, '_, &mut TargetPose>,
    mut joint_targets: bevy::prelude::Query<'_, '_, &mut JointTarget>,
) {
    let dt = fixed_time.delta().as_secs_f32();
    let next_elapsed = simulation.elapsed + fixed_time.delta();
    let target_time = next_elapsed.as_secs_f64();
    {
        let Simulation {
            profile, instances, ..
        } = &mut *simulation;
        // Reuse per-instance buffers and derive body velocity from consecutive targets.
        for instance in instances {
            target_poses_into(
                profile,
                instance.root_pose,
                target_time,
                &mut instance.target_scratch,
            );
            for (position, entity) in instance.bodies.iter().copied().enumerate() {
                let next_pose = instance.target_scratch[position];
                let mut target = poses
                    .get_mut(entity)
                    .expect("spawned body keeps its target pose component");
                let previous_pose = target.pose;
                let previous_translation: Vec3 = previous_pose.translation.into();
                let next_translation: Vec3 = next_pose.translation.into();
                target.linear_velocity = (next_translation - previous_translation) / dt;
                target.angular_velocity =
                    angular_velocity_between(previous_pose.rotation, next_pose.rotation, dt);
                target.pose = next_pose;
            }
            // Derive each motor velocity from the previous and current bounded angles.
            for joint in profile.joints() {
                let child = joint.child().get();
                let target_angle =
                    target_angle(profile.bodies()[child].bone(), target_time, joint.limits());
                let entity = instance.bodies[child];
                let mut target = joint_targets
                    .get_mut(entity)
                    .expect("profile joint child keeps its target component");
                target.angular_velocity = (target_angle - target.angles) / dt;
                target.angles = target_angle;
            }
        }
    }
    simulation.elapsed = next_elapsed;
}

/// Applies acceleration-based Rapier motors to every unlocked angular joint axis.
fn update_motor_drive(
    simulation: bevy::prelude::Res<'_, Simulation>,
    mut joints: bevy::prelude::Query<'_, '_, &mut ImpulseJoint>,
    targets: bevy::prelude::Query<'_, '_, &JointTarget>,
) {
    if simulation.settings.drive != DriveMode::Motor {
        return;
    }
    let omega = std::f32::consts::TAU * simulation.settings.frequency.0;
    let muscle = simulation.settings.muscle.0;
    let stiffness = omega * omega * muscle;
    let damping = 2.0 * omega * muscle.sqrt() + 20.0;
    // A zero-strength motor keeps only the profile-scaled friction torque.
    for instance in &simulation.instances {
        for profile_joint in simulation.profile.joints() {
            let child = profile_joint.child().get();
            let entity = instance.bodies[child];
            let target = targets
                .get(entity)
                .expect("profile joint child keeps its target component");
            let mut impulse_joint = joints
                .get_mut(entity)
                .expect("profile joint child keeps its Rapier joint");
            let data = impulse_joint.data.as_mut();
            for (axis, range, position, velocity) in [
                (
                    JointAxis::AngX,
                    profile_joint.limits().x,
                    target.angles.x,
                    target.angular_velocity.x,
                ),
                (
                    JointAxis::AngY,
                    profile_joint.limits().twist,
                    target.angles.y,
                    target.angular_velocity.y,
                ),
                (
                    JointAxis::AngZ,
                    profile_joint.limits().z,
                    target.angles.z,
                    target.angular_velocity.z,
                ),
            ] {
                if range.is_locked() {
                    continue;
                }
                let max_force = profile_joint.max_torque() * (0.05 + muscle);
                data.set_motor_model(axis, bevy_rapier3d::prelude::MotorModel::AccelerationBased)
                    .set_motor(axis, position, velocity, stiffness, damping)
                    .set_motor_max_force(axis, max_force);
            }
        }
    }
}

/// Returns angular velocity from two consecutive world rotations.
fn angular_velocity_between(previous: Quat, current: Quat, dt: f32) -> Vec3 {
    let delta = current * previous.inverse();
    let (axis, angle) = delta.to_axis_angle();
    axis * (angle / dt)
}

/// Clears persistent Rapier external forces before each torque or pin update.
fn reset_external_forces(
    simulation: bevy::prelude::Res<'_, Simulation>,
    mut forces: bevy::prelude::Query<'_, '_, &mut ExternalForce>,
) {
    if simulation.settings.drive != DriveMode::Torque && simulation.settings.pin.0 == 0.0 {
        return;
    }
    for instance in &simulation.instances {
        for entity in &instance.bodies {
            let mut external = forces
                .get_mut(*entity)
                .expect("spawned body keeps its external force component");
            external.force = Vec3::ZERO;
            external.torque = Vec3::ZERO;
        }
    }
}

/// Applies child and parent torques using the stable-PD profile formula.
fn apply_torque_drive(
    mut simulation: bevy::prelude::ResMut<'_, Simulation>,
    states: bevy::prelude::Query<'_, '_, (&Transform, &Velocity, &BodyInertia)>,
    targets: bevy::prelude::Query<'_, '_, &JointTarget>,
    mut forces: bevy::prelude::Query<'_, '_, &mut ExternalForce>,
    fixed_time: bevy::prelude::Res<'_, Time<Fixed>>,
) {
    if simulation.settings.drive != DriveMode::Torque {
        return;
    }
    let dt = fixed_time.delta_secs();
    let omega = std::f32::consts::TAU * simulation.settings.frequency.0;
    let muscle = simulation.settings.muscle.0;
    let stiffness = omega * omega * muscle;
    let damping = 2.0 * omega * muscle.sqrt() + 20.0;
    let Simulation {
        profile, instances, ..
    } = &mut *simulation;
    for instance in instances {
        // Reuse current-pose storage while resolving every joint from one coherent world sample.
        for (position, entity) in instance.bodies.iter().copied().enumerate() {
            let (transform, _, _) = states
                .get(entity)
                .expect("spawned body keeps its transform and velocity");
            instance.current_poses[position] =
                Isometry3d::new(transform.translation, transform.rotation);
        }
        for joint in profile.joints() {
            let parent_index = joint.parent().get();
            let child_index = joint.child().get();
            let parent_entity = instance.bodies[parent_index];
            let child_entity = instance.bodies[child_index];
            let target = targets
                .get(child_entity)
                .expect("profile joint child keeps its target component");
            let (parent_transform, parent_velocity, _) = states
                .get(parent_entity)
                .expect("profile joint parent keeps its transform and velocity");
            let (child_transform, child_velocity, child_inertia) = states
                .get(child_entity)
                .expect("profile joint child keeps its transform and velocity");
            let frame_rotation = parent_transform.rotation * joint.frame().rotation;
            let current_relative = frame_rotation.inverse() * child_transform.rotation;
            let desired_relative = Quat::from_rotation_x(target.angles.x);
            let mut error_rotation = desired_relative * current_relative.inverse();
            if error_rotation.w < 0.0 {
                error_rotation = -error_rotation;
            }
            let (error_axis, error_angle) = error_rotation.to_axis_angle();
            let error_world = frame_rotation * (error_axis * error_angle);
            let target_angular_velocity = frame_rotation * target.angular_velocity;
            let angular_velocity_error =
                child_velocity.angular - parent_velocity.angular - target_angular_velocity;
            let inertia = if error_world.length_squared() <= 1.0e-8 {
                child_inertia.principal.max_element()
            } else {
                inertia_along_world_axis(*child_inertia, child_transform.rotation, error_world)
            };
            let torque = inertia
                * (stiffness * error_world - (damping + stiffness * dt) * angular_velocity_error);
            let torque_limit = joint.max_torque() * (0.05 + muscle);
            let torque = clamp_vector_magnitude(torque, torque_limit);
            forces
                .get_mut(child_entity)
                .expect("profile joint child keeps its external force component")
                .torque += torque;
            forces
                .get_mut(parent_entity)
                .expect("profile joint parent keeps its external force component")
                .torque -= torque;
        }
    }
}

/// Computes the child's scalar principal inertia about a world-space axis.
fn inertia_along_world_axis(inertia: BodyInertia, body_rotation: Quat, axis: Vec3) -> f32 {
    let principal_rotation = body_rotation * inertia.frame;
    let local_axis = principal_rotation.inverse() * axis.normalize();
    (inertia.principal * local_axis).dot(local_axis)
}

/// Multiplies a world-space angular acceleration by the full principal inertia tensor.
fn world_inertia_times(inertia: BodyInertia, body_rotation: Quat, vector: Vec3) -> Vec3 {
    let principal_rotation = body_rotation * inertia.frame;
    let local_vector = principal_rotation.inverse() * vector;
    principal_rotation * (inertia.principal * local_vector)
}

/// Limits a finite vector's length while preserving its direction.
fn clamp_vector_magnitude(vector: Vec3, max_length: f32) -> Vec3 {
    let length = vector.length();
    if !length.is_finite() || !max_length.is_finite() {
        return Vec3::ZERO;
    }
    if length > max_length && length > 0.0 {
        vector * (max_length / length)
    } else {
        vector
    }
}

/// Applies the position and rotation pin model to pelvis and chest bodies.
fn apply_pins(
    mut simulation: bevy::prelude::ResMut<'_, Simulation>,
    states: bevy::prelude::Query<'_, '_, (&Transform, &Velocity, &BodyInertia, &TargetPose)>,
    mut forces: bevy::prelude::Query<'_, '_, &mut ExternalForce>,
) {
    let pin = simulation.settings.pin.0;
    if pin == 0.0 {
        return;
    }
    let pin_frequency = 1.5_f32;
    let omega = std::f32::consts::TAU * pin_frequency;
    let force_limit = 340.0_f32;
    let torque_limit = 400.0_f32;
    let Simulation {
        instances,
        pin_bodies,
        max_pin_force,
        ..
    } = &mut *simulation;
    for instance in instances {
        for body_index in *pin_bodies {
            let entity = instance.bodies[body_index.get()];
            let (transform, velocity, inertia, target) = states
                .get(entity)
                .expect("pelvis and chest keep target and physics components");
            let target_position: Vec3 = target.pose.translation.into();
            let position_error = target_position - transform.translation;
            let distance = position_error.length();
            let position_acceleration = omega * omega * position_error
                + 2.0 * omega * (target.linear_velocity - velocity.linear);
            let force = inertia.mass * pin * position_acceleration;
            let max_force = pin * force_limit / (1.0 + 2.0 * distance);
            let force = clamp_vector_magnitude(force, max_force);
            *max_pin_force = max_pin_force.max(force.length());

            let mut rotation_error = target.pose.rotation * transform.rotation.inverse();
            if rotation_error.w < 0.0 {
                rotation_error = -rotation_error;
            }
            let (rotation_axis, rotation_angle) = rotation_error.to_axis_angle();
            let rotation_error = rotation_axis * rotation_angle;
            let angular_acceleration = omega * omega * rotation_error
                + 2.0 * omega * (target.angular_velocity - velocity.angular);
            let torque =
                pin * world_inertia_times(*inertia, transform.rotation, angular_acceleration);
            let torque = clamp_vector_magnitude(torque, pin * torque_limit);
            let mut external = forces
                .get_mut(entity)
                .expect("pelvis and chest keep their external force component");
            external.force += force;
            external.torque += torque;
        }
    }
}

/// Starts wall-clock timing immediately before Rapier's `StepSimulation` set.
fn begin_step_timer(mut simulation: bevy::prelude::ResMut<'_, Simulation>) {
    simulation.step_started = Some(Instant::now());
}

/// Records the wall-clock time spent in Rapier's `StepSimulation` set.
fn end_step_timer(mut simulation: bevy::prelude::ResMut<'_, Simulation>) {
    let started = simulation
        .step_started
        .take()
        .expect("each Rapier step has a preceding timer start");
    simulation
        .step_milliseconds
        .push(started.elapsed().as_secs_f64() * 1000.0);
}

/// Converts a Bevy transform into the rigid pose used by the profile angle API.
fn rigid_pose(transform: &Transform) -> Isometry3d {
    Isometry3d::new(transform.translation, transform.rotation)
}

/// Samples joint errors, pelvis drift, and unstable bodies after Rapier writeback.
fn record_metrics(
    mut simulation: bevy::prelude::ResMut<'_, Simulation>,
    states: bevy::prelude::Query<'_, '_, (&Transform, &Velocity, &TargetPose)>,
    joint_targets: bevy::prelude::Query<'_, '_, &JointTarget>,
) {
    let Simulation {
        profile,
        instances,
        pin_bodies,
        angle_error_sum,
        angle_error_samples,
        max_angle_error,
        max_pelvis_drift,
        max_pelvis_drift_at_s,
        final_pelvis_height,
        unstable_bodies,
        elapsed,
        ..
    } = &mut *simulation;
    for instance in instances {
        // Capture all current poses before measuring the child joints.
        for (position, entity) in instance.bodies.iter().copied().enumerate() {
            let (transform, velocity, target) = states
                .get(entity)
                .expect("spawned body keeps its writeback components");
            instance.current_poses[position] = rigid_pose(transform);
            let speed = velocity.linear.length();
            let finite = transform.translation.is_finite()
                && transform.rotation.is_finite()
                && velocity.linear.is_finite()
                && velocity.angular.is_finite()
                && speed.is_finite();
            if !finite || speed > 50.0 {
                unstable_bodies.insert(entity);
            }
            if position == pin_bodies[0].get() {
                *final_pelvis_height = transform.translation.y;
                let target_y = target.pose.translation.y;
                let drift = (transform.translation.y - target_y).abs();
                if drift.is_finite() && drift > *max_pelvis_drift {
                    *max_pelvis_drift = drift;
                    *max_pelvis_drift_at_s = elapsed.as_secs_f64();
                }
            }
        }
        // Count each axis independently so the mean has a stable scalar denominator.
        for joint in profile.joints() {
            let child = joint.child().get();
            let Some(angles) = profile.joint_angles(joint.child(), &instance.current_poses) else {
                continue;
            };
            let target = joint_targets
                .get(instance.bodies[child])
                .expect("profile joint child keeps its target component");
            for error in [
                (target.angles.x - angles.x).abs(),
                (target.angles.y - angles.y).abs(),
                (target.angles.z - angles.z).abs(),
            ] {
                if error.is_finite() {
                    *angle_error_sum += f64::from(error);
                    *angle_error_samples += 1;
                    *max_angle_error = max_angle_error.max(error);
                }
            }
        }
    }
}

/// Computes the ceiling number of fixed steps needed to cover a positive duration.
fn fixed_step_count(duration: Duration) -> u128 {
    let step = Duration::from_secs_f64(1.0 / 60.0).as_nanos();
    duration.as_nanos().div_ceil(step)
}

/// Runs the real fixed schedule without rendering and returns aggregated measurements.
fn run_headless_simulation(
    profile: RagdollProfile,
    config: &RunConfig,
) -> Result<SpikeReport, Box<dyn std::error::Error>> {
    if !matches!(config.mode, RunMode::Headless { .. }) {
        return Err(io::Error::other("headless simulation requires --headless").into());
    }
    let mut app = build_headless_app(profile, config);
    let expected_steps = fixed_step_count(config.duration.0);
    let mut updates = 0_u128;
    while (app.world().resource::<Simulation>().step_milliseconds.len() as u128) < expected_steps
        && updates < expected_steps + 2
    {
        app.update();
        updates += 1;
    }
    let actual_steps = app.world().resource::<Simulation>().step_milliseconds.len() as u128;
    if actual_steps != expected_steps {
        return Err(io::Error::other(format!(
            "fixed schedule ran {actual_steps} steps, expected {expected_steps}"
        ))
        .into());
    }
    Ok(make_report(app.world().resource::<Simulation>(), config))
}

/// Builds the serialized report from finite accumulated values.
fn make_report(simulation: &Simulation, config: &RunConfig) -> SpikeReport {
    let mean_error = (simulation.angle_error_samples > 0).then(|| {
        simulation.angle_error_sum / simulation.angle_error_samples as f64
            * (180.0 / std::f64::consts::PI)
    });
    let max_error = (simulation.angle_error_samples > 0)
        .then(|| f64::from(simulation.max_angle_error) * (180.0 / std::f64::consts::PI));
    let mean_step = (!simulation.step_milliseconds.is_empty()).then(|| {
        simulation.step_milliseconds.iter().sum::<f64>() / simulation.step_milliseconds.len() as f64
    });
    let mut sorted_steps = simulation.step_milliseconds.clone();
    sorted_steps.sort_by(f64::total_cmp);
    let p95_step = (!sorted_steps.is_empty()).then(|| {
        let rank = sorted_steps.len().saturating_mul(95).div_ceil(100);
        sorted_steps[rank.saturating_sub(1)]
    });
    SpikeReport {
        schema_version: 1,
        configuration: ReportConfiguration {
            drive: match config.drive {
                DriveMode::Motor => "motor",
                DriveMode::Torque => "torque",
            },
            pin: config.pin.0,
            muscle: config.muscle.0,
            count: config.count.0.get(),
            seconds: config.duration.0.as_secs_f64(),
            natural_frequency_hz: config.frequency.0,
            fixed_frequency_hz: 60.0,
        },
        metrics: ReportMetrics {
            fixed_steps: simulation.step_milliseconds.len(),
            mean_joint_angle_error_deg: mean_error,
            max_joint_angle_error_deg: max_error,
            max_pelvis_height_drift_m: simulation.max_pelvis_drift,
            max_pelvis_height_drift_at_s: simulation.max_pelvis_drift_at_s,
            max_pin_force_n: simulation.max_pin_force,
            final_pelvis_height_m: simulation.final_pelvis_height,
            mean_step_ms: mean_step,
            p95_step_ms: p95_step,
            unstable_bodies: simulation.unstable_bodies.len(),
        },
    }
}

/// Writes pretty JSON and creates its parent directory when needed.
fn write_report(
    path: &std::path::Path,
    report: &SpikeReport,
) -> Result<(), Box<dyn std::error::Error>> {
    if let Some(parent) = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        fs::create_dir_all(parent)?;
    }
    let mut json = serde_json::to_vec_pretty(report)?;
    json.push(b'\n');
    fs::write(path, json)?;
    Ok(())
}

/// Parses, validates, and runs one powered-ragdoll configuration.
fn main() {
    if let Err(error) = run(Args::parse()) {
        eprintln!("{error}");
        std::process::exit(2);
    }
}

/// Runs the headless report path or routes visible execution to the renderer.
fn run(args: Args) -> Result<(), Box<dyn std::error::Error>> {
    let config = RunConfig::try_from(args)?;
    let profile = load_tgf_profile()?;
    match &config.mode {
        RunMode::Headless { report } => {
            let result = run_headless_simulation(profile, &config)?;
            write_report(report, &result)?;
            let report_path = report.display();
            println!("Wrote {report_path}");
            Ok(())
        }
        RunMode::Visual { .. } => {
            #[cfg(feature = "visual")]
            {
                run_visual(profile, config)
            }
            #[cfg(not(feature = "visual"))]
            {
                Err(io::Error::other("visible mode requires the `visual` feature").into())
            }
        }
    }
}

/// Runtime controls for a visible simulation and its optional screenshot.
#[cfg(feature = "visual")]
#[derive(Resource)]
struct VisualRun {
    /// Requested fixed simulation duration.
    duration: Duration,
    /// Optional path for a screenshot taken halfway through the run.
    screenshot: Option<PathBuf>,
    /// Whether the screenshot request has been submitted to Bevy.
    screenshot_requested: bool,
    /// Wall-clock time when the screenshot request was submitted.
    screenshot_requested_at: Option<Instant>,
    /// Whether screenshot capture timed out before producing a file.
    screenshot_timed_out: bool,
}

/// Associates a standalone mesh entity with its dynamic body and profile-local shape pose.
#[cfg(feature = "visual")]
#[derive(Component)]
struct VisualFollower {
    /// Dynamic body whose Rapier transform drives this mesh.
    body: Entity,
    /// Shape offset and orientation relative to the body frame.
    local_transform: Transform,
}

/// Builds the rendered Rapier application with geometry for the profile shapes.
#[cfg(feature = "visual")]
fn run_visual(
    profile: RagdollProfile,
    config: RunConfig,
) -> Result<(), Box<dyn std::error::Error>> {
    let screenshot = match &config.mode {
        RunMode::Visual { screenshot } => screenshot.clone(),
        RunMode::Headless { .. } => {
            return Err(io::Error::other("visible simulation requires visual mode").into());
        }
    };
    let screenshot_output = screenshot.clone();
    if let Some(path) = &screenshot {
        if let Some(parent) = path
            .parent()
            .filter(|parent| !parent.as_os_str().is_empty())
        {
            fs::create_dir_all(parent)?;
        }
        if path.exists() {
            fs::remove_file(path)?;
        }
    }

    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "Bevy Ragdoll powered simulation".to_owned(),
            resolution: (1280, 800).into(),
            ..Default::default()
        }),
        ..Default::default()
    }))
    .insert_resource(Time::<Fixed>::from_hz(60.0))
    .insert_resource(TimestepMode::Fixed {
        dt: 1.0 / 60.0,
        substeps: 1,
    })
    .add_plugins(RapierPhysicsPlugin::<ContactFilter>::default().in_fixed_schedule());
    configure_simulation_systems(&mut app);
    initialize_simulation(&mut app, profile, &config);
    app.insert_resource(VisualRun {
        duration: config.duration.0,
        screenshot,
        screenshot_requested: false,
        screenshot_requested_at: None,
        screenshot_timed_out: false,
    })
    .add_systems(Startup, setup_visual_scene)
    .add_systems(Update, (sync_visual_followers, visual_lifecycle).chain());
    app.run();

    if let Some(path) = screenshot_output.as_ref() {
        if !path.is_file() {
            return Err(io::Error::other(format!(
                "screenshot was not written to {}",
                path.display()
            ))
            .into());
        }
        let screenshot_path = path.display();
        println!("Wrote screenshot {screenshot_path}");
    }
    Ok(())
}

/// Creates the camera, lighting, floor, and profile-shape meshes.
#[cfg(feature = "visual")]
fn setup_visual_scene(
    mut commands: Commands,
    simulation: Res<'_, Simulation>,
    mut meshes: ResMut<'_, Assets<Mesh>>,
    mut materials: ResMut<'_, Assets<StandardMaterial>>,
) {
    let floor_mesh = meshes.add(Plane3d::default().mesh().size(400.0, 400.0));
    let floor_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.11, 0.15, 0.19),
        perceptual_roughness: 0.88,
        ..Default::default()
    });
    commands.spawn((
        Mesh3d(floor_mesh),
        MeshMaterial3d(floor_material),
        Transform::from_xyz(0.0, -0.001, 0.0),
    ));

    let body_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.18, 0.62, 0.76),
        metallic: 0.04,
        perceptual_roughness: 0.42,
        ..Default::default()
    });
    let head_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.92, 0.66, 0.34),
        metallic: 0.02,
        perceptual_roughness: 0.48,
        ..Default::default()
    });
    // Keep render meshes independent and copy each body's current world pose per frame.
    for instance in &simulation.instances {
        for (position, entity) in instance.bodies.iter().copied().enumerate() {
            let body = &simulation.profile.bodies()[position];
            let (mesh, transform) = visual_shape(body.shape());
            let material = if body.bone() == "head" {
                head_material.clone()
            } else {
                body_material.clone()
            };
            commands.spawn((
                Mesh3d(meshes.add(mesh)),
                MeshMaterial3d(material),
                VisualFollower {
                    body: entity,
                    local_transform: transform,
                },
                Transform::IDENTITY,
            ));
        }
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
        Camera3d::default(),
        Transform::from_xyz(1.8, 2.2, 4.7).looking_at(Vec3::new(0.0, 1.0, 0.0), Vec3::Y),
    ));
}

/// Applies each body pose to its independent render mesh before transform propagation.
#[cfg(feature = "visual")]
fn sync_visual_followers(
    bodies: bevy::prelude::Query<'_, '_, &Transform, bevy::prelude::With<RagdollMember>>,
    mut followers: bevy::prelude::Query<
        '_,
        '_,
        (&VisualFollower, &mut Transform),
        bevy::prelude::Without<RagdollMember>,
    >,
) {
    for (follower, mut visual_transform) in &mut followers {
        if let Ok(body_transform) = bodies.get(follower.body) {
            *visual_transform = body_transform.mul_transform(follower.local_transform);
        }
    }
}

/// Converts each profile collision shape to a matching visible child mesh.
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

/// Captures the requested mid-run image and exits after the run and capture finish.
#[cfg(feature = "visual")]
fn visual_lifecycle(
    mut commands: Commands,
    simulation: Res<'_, Simulation>,
    mut visual: ResMut<'_, VisualRun>,
    mut exit: MessageWriter<'_, AppExit>,
) {
    let screenshot_path = visual.screenshot.clone();
    if let Some(path) = screenshot_path.as_ref() {
        if !visual.screenshot_requested && simulation.elapsed >= visual.duration / 2 {
            commands
                .spawn(Screenshot::primary_window())
                .observe(save_to_disk(path.clone()));
            visual.screenshot_requested = true;
            visual.screenshot_requested_at = Some(Instant::now());
        }
        if let Some(requested_at) = visual.screenshot_requested_at {
            visual.screenshot_timed_out =
                !path.is_file() && requested_at.elapsed() >= Duration::from_secs(10);
        }
    }
    let duration_complete = simulation.elapsed >= visual.duration;
    let screenshot_complete = visual.screenshot.as_ref().is_none_or(|path| path.is_file());
    if duration_complete && (screenshot_complete || visual.screenshot_timed_out) {
        exit.write(AppExit::Success);
    }
}

#[cfg(test)]
mod tests {
    use super::{
        Args, BodyInertia, DriveMode, RagdollMember, contact_pair_excluded, load_tgf_profile,
        spawn_ragdoll, target_poses,
    };
    use bevy::math::Isometry3d;
    use bevy::prelude::{Transform, World};
    use bevy_rapier3d::prelude::{
        ActiveHooks, ColliderMassProperties, ImpulseJoint, JointAxesMask, RigidBody, Sleeping,
    };
    use clap::Parser;

    /// Accepts each supported drive name and rejects an unknown mode.
    #[test]
    fn drive_argument_has_two_closed_modes() {
        let motor = Args::try_parse_from(["spike", "--drive", "motor"])
            .expect("motor is a supported drive mode");
        assert_eq!(motor.drive, DriveMode::Motor);

        let torque = Args::try_parse_from(["spike", "--drive", "torque"])
            .expect("torque is a supported drive mode");
        assert_eq!(torque.drive, DriveMode::Torque);

        assert!(Args::try_parse_from(["spike", "--drive", "unknown"]).is_err());
    }

    /// Keeps headless selection as a closed presence marker, not a boolean.
    #[test]
    fn headless_argument_is_an_optional_closed_marker() {
        let visual = Args::try_parse_from(["spike"]).expect("default arguments parse");
        assert_eq!(visual.headless, None);

        let headless = Args::try_parse_from(["spike", "--headless"]).expect("headless flag parses");
        assert_eq!(headless.headless, Some(super::Headless::Enabled));
    }

    /// Requires a report path for headless runs and reserves screenshots for visual runs.
    #[test]
    fn execution_mode_requires_its_output_path() {
        let headless = Args::try_parse_from([
            "spike",
            "--headless",
            "--report",
            "results/run.json",
            "--count",
            "32",
        ])
        .expect("headless options parse");
        let config = super::RunConfig::try_from(headless).expect("report is required and present");
        assert_eq!(config.count.0.get(), 32);
        assert_eq!(
            config.mode,
            super::RunMode::Headless {
                report: "results/run.json".into()
            }
        );

        for arguments in [
            vec!["spike", "--headless"],
            vec!["spike", "--report", "results/run.json"],
            vec!["spike", "--headless", "--screenshot", "frame.png"],
        ] {
            let args = Args::try_parse_from(arguments).expect("option syntax is valid");
            assert!(super::RunConfig::try_from(args).is_err());
        }

        let visual = Args::try_parse_from(["spike", "--screenshot", "frame.png"])
            .expect("visual screenshot path parses");
        let config = super::RunConfig::try_from(visual).expect("visual mode accepts screenshot");
        assert_eq!(
            config.mode,
            super::RunMode::Visual {
                screenshot: Some("frame.png".into())
            }
        );
    }

    /// Runs the real fixed schedule and returns finite headless measurements.
    #[test]
    fn headless_run_steps_rapier_and_collects_metrics() {
        let args = Args::try_parse_from([
            "spike",
            "--headless",
            "--report",
            "unused.json",
            "--seconds",
            "0.05",
            "--pin",
            "1",
        ])
        .expect("headless arguments parse");
        let config = super::RunConfig::try_from(args).expect("headless report path is present");
        let profile = load_tgf_profile().expect("embedded TGF profile is valid");
        let report = super::run_headless_simulation(profile, &config)
            .expect("Rapier fixed-step simulation completes");

        assert_eq!(report.configuration.count, 1);
        assert_eq!(report.metrics.fixed_steps, 3);
        assert!(report.metrics.mean_step_ms.is_some_and(f64::is_finite));
        assert!(report.metrics.p95_step_ms.is_some_and(f64::is_finite));
        assert!(
            report
                .metrics
                .mean_joint_angle_error_deg
                .is_some_and(f64::is_finite)
        );
        assert!(report.metrics.max_pelvis_height_drift_m.is_finite());

        let process_id = std::process::id();
        let report_path =
            std::env::temp_dir().join(format!("bevy-ragdoll-spike-{process_id}.json"));
        super::write_report(&report_path, &report).expect("report JSON writes to disk");
        let bytes = std::fs::read(&report_path).expect("report JSON is readable");
        let json: serde_json::Value = serde_json::from_slice(&bytes).expect("report JSON is valid");
        assert_eq!(json["schema_version"], 1);
        assert_eq!(json["metrics"]["fixed_steps"], 3);
        std::fs::remove_file(report_path).expect("temporary report is removed");
    }

    /// Validates simulation strengths, count, duration and frequency at CLI ingress.
    #[test]
    fn simulation_arguments_use_validated_domain_types() {
        let defaults = Args::try_parse_from(["spike"]).expect("default arguments parse");
        assert_eq!(defaults.pin, super::PinStrength(0.0));
        assert_eq!(defaults.muscle, super::MuscleStrength(1.0));
        assert_eq!(defaults.count.0.get(), 1);
        assert_eq!(defaults.duration.0.as_secs(), 10);
        assert_eq!(defaults.frequency, super::NaturalFrequency(4.0));

        let args = Args::try_parse_from([
            "spike",
            "--pin",
            "0.5",
            "--muscle",
            "0.25",
            "--count",
            "32",
            "--seconds",
            "10",
            "--frequency",
            "6",
        ])
        .expect("valid simulation configuration parses");
        assert_eq!(args.pin, super::PinStrength(0.5));
        assert_eq!(args.muscle, super::MuscleStrength(0.25));
        assert_eq!(args.count.0.get(), 32);
        assert_eq!(args.duration.0.as_secs(), 10);
        assert_eq!(args.frequency, super::NaturalFrequency(6.0));

        let endpoints = Args::try_parse_from([
            "spike",
            "--pin=0",
            "--muscle=1",
            "--count=128",
            "--seconds=0.001",
            "--frequency=0.25",
        ])
        .expect("closed interval endpoints and positive minima parse");
        assert_eq!(endpoints.pin, super::PinStrength(0.0));
        assert_eq!(endpoints.muscle, super::MuscleStrength(1.0));
        assert_eq!(endpoints.count.0.get(), 128);
        assert_eq!(endpoints.duration.0, std::time::Duration::from_millis(1));
        assert_eq!(endpoints.frequency, super::NaturalFrequency(0.25));

        for (argument, value) in [
            ("--pin", "-0.1"),
            ("--pin", "1.1"),
            ("--pin", "NaN"),
            ("--pin", "inf"),
            ("--muscle", "-0.1"),
            ("--muscle", "1.1"),
            ("--muscle", "NaN"),
            ("--muscle", "inf"),
            ("--count", "0"),
            ("--count", "129"),
            ("--count", "2.5"),
            ("--seconds", "0"),
            ("--seconds", "-1"),
            ("--seconds", "NaN"),
            ("--seconds", "1e9999"),
            ("--frequency", "0"),
            ("--frequency", "-1"),
            ("--frequency", "NaN"),
            ("--frequency", "inf"),
        ] {
            let option = format!("{argument}={value}");
            assert!(
                Args::try_parse_from(["spike", option.as_str()]).is_err(),
                "accepted {option}"
            );
        }
    }

    /// Loads the checked TGF profile and measures the procedural motion in its joint frames.
    #[test]
    fn target_pose_matches_arm_and_squat_cycles() {
        let profile = load_tgf_profile().expect("embedded TGF profile is valid");
        assert_eq!(profile.bodies().len(), 16);
        assert_eq!(profile.joints().len(), 15);

        let arm_targets = target_poses(&profile, Isometry3d::IDENTITY, 0.5);
        for bone in ["upperarm_l", "upperarm_r"] {
            let child = profile.body_index(bone).expect("arm body exists");
            let angles = profile
                .joint_angles(child, &arm_targets)
                .expect("arm target has valid joint poses");
            assert!((angles.x - 0.6).abs() < 1.0e-4, "{bone}: {angles}");
        }
        for bone in ["lowerarm_l", "lowerarm_r"] {
            let child = profile.body_index(bone).expect("elbow body exists");
            let angles = profile
                .joint_angles(child, &arm_targets)
                .expect("elbow target has valid joint poses");
            assert!((angles.x - 0.4).abs() < 1.0e-4, "{bone}: {angles}");
        }

        let squat_targets = target_poses(&profile, Isometry3d::IDENTITY, 2.0);
        for bone in ["thigh_l", "thigh_r"] {
            let child = profile.body_index(bone).expect("hip body exists");
            let angles = profile
                .joint_angles(child, &squat_targets)
                .expect("hip target has valid joint poses");
            assert!((angles.x - 1.0).abs() < 1.0e-4, "{bone}: {angles}");
        }
        for bone in ["calf_l", "calf_r"] {
            let child = profile.body_index(bone).expect("knee body exists");
            let angles = profile
                .joint_angles(child, &squat_targets)
                .expect("knee target has valid joint poses");
            assert!((angles.x + 1.0).abs() < 1.0e-4, "{bone}: {angles}");
        }
    }

    /// Creates one dynamic body per profile entry and one locked-linear joint per edge.
    #[test]
    fn spawn_builds_rapier_bodies_and_joints() {
        let profile = load_tgf_profile().expect("embedded TGF profile is valid");
        let mut world = World::new();
        let spawned = spawn_ragdoll(&mut world, &profile, Isometry3d::IDENTITY);

        assert_eq!(spawned.bodies.len(), profile.bodies().len());
        let mut body_query = world.query::<(
            &RigidBody,
            &super::RagdollMember,
            &Sleeping,
            &ActiveHooks,
            &ColliderMassProperties,
            &BodyInertia,
        )>();
        let bodies = body_query.iter(&world).collect::<Vec<_>>();
        assert_eq!(bodies.len(), 16);
        assert!(bodies.iter().all(|(body, _, sleeping, hooks, _, inertia)| {
            **body == RigidBody::Dynamic
                && !sleeping.sleeping
                && hooks.contains(ActiveHooks::FILTER_CONTACT_PAIRS)
                && inertia.principal.min_element() >= inertia.mass * 0.08_f32 * 0.08_f32
        }));

        let mut joint_query = world.query::<&ImpulseJoint>();
        let joints = joint_query.iter(&world).collect::<Vec<_>>();
        assert_eq!(joints.len(), 15);
        assert!(joints.iter().all(|joint| {
            let data = joint.data.as_ref();
            !data.contacts_enabled()
                && data
                    .locked_axes()
                    .contains(JointAxesMask::LOCKED_SPHERICAL_AXES)
        }));
    }

    /// Applies profile masks only to excluded pairs within one ragdoll instance.
    #[test]
    fn contact_filter_preserves_cross_instance_pairs() {
        let mut world = World::new();
        let root = world.spawn_empty().id();
        let other_root = world.spawn_empty().id();
        let first_index = super::BodyIndex::try_from(0).expect("body zero is in range");
        let second_index = super::BodyIndex::try_from(1).expect("body one is in range");
        let first = RagdollMember {
            root,
            body: first_index,
            no_contact_mask: 1_u64 << second_index.get(),
        };
        let excluded = RagdollMember {
            root,
            body: second_index,
            no_contact_mask: 0,
        };
        let cross_instance = RagdollMember {
            root: other_root,
            ..excluded
        };

        assert!(contact_pair_excluded(&first, &excluded));
        assert!(!contact_pair_excluded(&first, &cross_instance));
    }

    /// Builds render meshes and local transforms for each profile shape variant.
    #[cfg(feature = "visual")]
    #[test]
    fn visual_shape_covers_capsules_spheres_and_cuboids() {
        let capsule_shape = super::ShapeSpec::Capsule {
            a: bevy::math::Vec3::new(1.0, 0.0, 0.0),
            b: bevy::math::Vec3::new(1.0, 2.0, 0.0),
            radius: 0.25,
        };
        let (capsule, capsule_transform) = super::visual_shape(&capsule_shape);
        assert!(
            capsule
                .attribute(bevy::prelude::Mesh::ATTRIBUTE_POSITION)
                .is_some()
        );
        assert_eq!(
            capsule_transform.translation,
            bevy::math::Vec3::new(1.0, 1.0, 0.0)
        );

        let sphere_shape = super::ShapeSpec::Sphere {
            center: bevy::math::Vec3::new(4.0, 2.0, 1.0),
            radius: 0.5,
        };
        let (sphere, sphere_transform) = super::visual_shape(&sphere_shape);
        assert!(
            sphere
                .attribute(bevy::prelude::Mesh::ATTRIBUTE_POSITION)
                .is_some()
        );
        assert_eq!(
            sphere_transform.translation,
            bevy::math::Vec3::new(4.0, 2.0, 1.0)
        );

        let cuboid_rotation = bevy::math::Quat::from_rotation_y(0.5);
        let cuboid_shape = super::ShapeSpec::Cuboid {
            center: bevy::math::Vec3::new(0.0, 1.0, 2.0),
            rotation: cuboid_rotation,
            half_extents: bevy::math::Vec3::splat(0.25),
        };
        let (cuboid, cuboid_transform) = super::visual_shape(&cuboid_shape);
        assert!(
            cuboid
                .attribute(bevy::prelude::Mesh::ATTRIBUTE_POSITION)
                .is_some()
        );
        assert_eq!(
            cuboid_transform.translation,
            bevy::math::Vec3::new(0.0, 1.0, 2.0)
        );
        assert_eq!(cuboid_transform.rotation, cuboid_rotation);
    }

    /// Copies body poses to visual meshes and tolerates a body removed before its mesh.
    #[cfg(feature = "visual")]
    #[test]
    fn visual_followers_track_live_bodies_and_skip_missing_bodies() {
        let mut app = bevy::prelude::App::new();
        app.add_plugins(bevy::prelude::MinimalPlugins)
            .add_systems(bevy::app::Update, super::sync_visual_followers);

        let body = app.world_mut().spawn_empty().id();
        let body_transform = Transform::from_xyz(2.0, 1.0, 0.0).with_rotation(
            bevy::math::Quat::from_rotation_y(std::f32::consts::FRAC_PI_2),
        );
        let body_index = super::BodyIndex::try_from(0).expect("body zero is in range");
        app.world_mut().entity_mut(body).insert((
            super::RagdollMember {
                root: body,
                body: body_index,
                no_contact_mask: 0,
            },
            body_transform,
        ));

        let local_transform = Transform::from_xyz(1.0, 2.0, 0.0);
        let visual = app
            .world_mut()
            .spawn((
                super::VisualFollower {
                    body,
                    local_transform,
                },
                Transform::IDENTITY,
            ))
            .id();
        let missing_body = app.world_mut().spawn_empty().id();
        assert!(app.world_mut().despawn(missing_body));
        let stale_visual_transform = Transform::from_xyz(7.0, 8.0, 9.0);
        let stale_visual = app
            .world_mut()
            .spawn((
                super::VisualFollower {
                    body: missing_body,
                    local_transform: Transform::IDENTITY,
                },
                stale_visual_transform,
            ))
            .id();

        app.update();

        let world_transform = app
            .world()
            .entity(visual)
            .get::<Transform>()
            .expect("visual mesh keeps its transform");
        assert!(
            world_transform
                .translation
                .abs_diff_eq(bevy::math::Vec3::new(2.0, 3.0, -1.0), 1.0e-5)
        );
        assert_eq!(
            world_transform.rotation,
            body_transform.rotation * local_transform.rotation
        );
        assert_eq!(
            app.world()
                .entity(stale_visual)
                .get::<Transform>()
                .expect("stale visual keeps its transform"),
            &stale_visual_transform
        );
    }
}
