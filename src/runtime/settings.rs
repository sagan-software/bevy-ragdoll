//! Shared physics and actuator parameters copied from an earlier game project
//! configuration.
//!
//! The runtime exposes these values to backends and uses motor, pin, and settle
//! settings directly. Defaults preserve that tuning where the project scope
//! requires parity, while every field remains editable before play. Values use
//! SI units except dimensionless coefficients, counts, and frequencies;
//! adapters should not silently reinterpret the documented units.

use bevy::math::Vec3;
use bevy::prelude::Resource;

/// Shared physics and drive tuning passed from the core runtime to a backend.
///
/// Defaults mirror that project's gravity, solver, damping, collision, motor,
/// pin, and settling configuration. The core sanitizes selected force inputs, but
/// backends remain responsible for validating values they pass to
/// engine-specific APIs and for preserving the stated SI unit conventions.
#[derive(Clone, Copy, Debug, PartialEq, Resource, bevy::prelude::Reflect)]
pub struct RagdollPhysicsSettings {
    /// World acceleration vector in metres per second squared; the default
    /// points down the Y axis. The core or backend adapter reads this field at
    /// its matching physics stage and preserves the stated units.
    pub gravity: Vec3,
    /// Upper bound on fixed physics substeps processed during one rendered
    /// update, defaulting to four. The core or backend adapter reads this field
    /// at its matching physics stage and preserves the stated units.
    pub max_substeps: usize,
    /// Number of constraint solver iterations performed by the backend on each
    /// physics step. The core or backend adapter reads this field at its
    /// matching physics stage and preserves the stated units.
    pub solver_iterations: usize,
    /// Number of projected Gauss-Seidel iterations requested for backend
    /// constraint solving. The core or backend adapter reads this field at its
    /// matching physics stage and preserves the stated units.
    pub pgs_iterations: usize,
    /// Fractional linear velocity damping applied per second; zero disables
    /// this damping path. The core or backend adapter reads this field at its
    /// matching physics stage and preserves the stated units.
    pub linear_damping: f32,
    /// Fractional angular velocity damping applied per second; the default
    /// value is `0.05`. The core or backend adapter reads this field at its
    /// matching physics stage and preserves the stated units.
    pub angular_damping: f32,
    /// Lower bound in metres for principal inertia radius estimation on every
    /// spawned body. The core or backend adapter reads this field at its
    /// matching physics stage and preserves the stated units.
    pub min_inertia_radius: f32,
    /// Dimensionless contact friction coefficient supplied to backend material
    /// construction. The core or backend adapter reads this field at its
    /// matching physics stage and preserves the stated units.
    pub friction: f32,
    /// Dimensionless contact restitution coefficient supplied to backend
    /// material construction. The core or backend adapter reads this field at
    /// its matching physics stage and preserves the stated units.
    pub restitution: f32,
    /// Whether the backend enables continuous collision detection for moving
    /// ragdoll bodies. The core or backend adapter reads this field at its
    /// matching physics stage and preserves the stated units.
    pub is_ccd_enabled: bool,
    /// Prediction distance in metres used by speculative or soft continuous
    /// collision algorithms. The core or backend adapter reads this field at
    /// its matching physics stage and preserves the stated units.
    pub soft_ccd_prediction: f32,
    /// Native joint motor natural frequency in hertz, converted to angular
    /// frequency by the core. The core or backend adapter reads this field at
    /// its matching physics stage and preserves the stated units.
    pub motor_frequency_hz: f32,
    /// Dimensionless damping ratio used with the joint motor natural frequency
    /// and muscle strength. The core or backend adapter reads this field at its
    /// matching physics stage and preserves the stated units.
    pub motor_damping_ratio: f32,
    /// Dimensionless multiplier applied to each authored maximum joint torque
    /// before clamping. The core or backend adapter reads this field at its
    /// matching physics stage and preserves the stated units.
    pub torque_scale: f32,
    /// Worker thread count requested by backends that expose configurable
    /// physics parallelism. The core or backend adapter reads this field at its
    /// matching physics stage and preserves the stated units.
    pub threads: usize,
    /// Dimensionless friction fraction retained by joints when muscle strength
    /// reaches zero. The core or backend adapter reads this field at its
    /// matching physics stage and preserves the stated units.
    pub joint_friction: f32,
    /// Joint friction damping rate in inverse seconds, added to motor damping
    /// at every strength. The core or backend adapter reads this field at its
    /// matching physics stage and preserves the stated units.
    pub friction_rate: f32,
    /// World-space pin spring natural frequency in hertz for position and
    /// rotation correction. The core or backend adapter reads this field at its
    /// matching physics stage and preserves the stated units.
    pub pin_frequency_hz: f32,
    /// Dimensionless damping ratio used by the critically damped world-space
    /// pin controller. The core or backend adapter reads this field at its
    /// matching physics stage and preserves the stated units.
    pub pin_damping_ratio: f32,
    /// Maximum world-space pin force in newtons before distance falloff and
    /// strength scaling. The core or backend adapter reads this field at its
    /// matching physics stage and preserves the stated units.
    pub pin_max_force: f32,
    /// Maximum world-space pin torque in newton metres before muscle or pin
    /// strength scaling. The core or backend adapter reads this field at its
    /// matching physics stage and preserves the stated units.
    pub pin_max_torque: f32,
    /// Inverse-metre coefficient that reduces maximum pin force as target
    /// distance increases. The core or backend adapter reads this field at its
    /// matching physics stage and preserves the stated units.
    pub pin_distance_falloff: f32,
    /// Linear speed threshold in metres per second that a backend may use to
    /// put bodies to sleep. The core or backend adapter reads this field at its
    /// matching physics stage and preserves the stated units.
    pub sleep_linear_threshold: f32,
    /// Angular speed threshold in radians per second that a backend may use to
    /// put bodies to sleep. The core or backend adapter reads this field at its
    /// matching physics stage and preserves the stated units.
    pub sleep_angular_threshold: f32,
    /// Required duration in seconds for every body to remain below
    /// `settle_speed` before an event. The core or backend adapter reads this
    /// field at its matching physics stage and preserves the stated units.
    pub settle_after: f32,
    /// Duration in seconds before a zero-muscle ragdoll may be forced to sleep
    /// by a backend. The core or backend adapter reads this field at its
    /// matching physics stage and preserves the stated units.
    pub force_sleep_after: f32,
    /// Maximum linear or angular-combined body speed in metres per second
    /// accepted as settled. The core or backend adapter reads this field at its
    /// matching physics stage and preserves the stated units.
    pub settle_speed: f32,
    /// Maximum upward correction in metres when spawning bodies from an
    /// overlapping target pose. The core or backend adapter reads this field at
    /// its matching physics stage and preserves the stated units.
    pub max_spawn_lift: f32,
    /// Whether completing settle detection also changes the whole ragdoll to
    /// frozen mode. The core or backend adapter reads this field at its
    /// matching physics stage and preserves the stated units.
    pub should_freeze_when_settled: bool,
}

impl Default for RagdollPhysicsSettings {
    fn default() -> Self {
        Self {
            gravity: Vec3::new(0.0, -9.81, 0.0),
            max_substeps: 4,
            solver_iterations: 8,
            pgs_iterations: 2,
            linear_damping: 0.0,
            angular_damping: 0.05,
            min_inertia_radius: 0.08,
            friction: 0.7,
            restitution: 0.0,
            is_ccd_enabled: true,
            soft_ccd_prediction: 0.3,
            motor_frequency_hz: 4.0,
            motor_damping_ratio: 1.0,
            torque_scale: 1.0,
            threads: 4,
            joint_friction: 0.05,
            friction_rate: 20.0,
            pin_frequency_hz: 1.5,
            pin_damping_ratio: 1.0,
            pin_max_force: 340.0,
            pin_max_torque: 400.0,
            pin_distance_falloff: 2.0,
            sleep_linear_threshold: 0.05,
            sleep_angular_threshold: 0.1,
            settle_after: 10.0,
            force_sleep_after: 6.0,
            settle_speed: 0.5,
            max_spawn_lift: 0.5,
            should_freeze_when_settled: false,
        }
    }
}
