//! Public schedule sets for variable-rate animation and fixed-rate physics
//! work.
//!
//! The plugin orders variable-rate binding, capture, and writeback around Bevy
//! animation and transform propagation. It orders fixed-rate behavior, drive,
//! backend apply, backend read, and after-step bookkeeping inside the selected
//! fixed schedule. Backend plugins attach systems to these sets to place
//! physics writes and reads around their own integration stage.

use bevy::prelude::SystemSet;

/// Variable-rate ragdoll systems ordered after animation and before transform
/// propagation.
///
/// These sets run once per rendered update, allowing the runtime to capture
/// animated targets and blend interpolated physics poses without changing the
/// backend's fixed-step integration rate.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, SystemSet)]
pub enum RagdollSystems {
    /// Bind validated profile names to skeleton bones and create or remove
    /// bodies after mode changes. This stage resolves loaded profiles and
    /// creates body relationships before capture or backend-facing fixed work
    /// begins.
    Bind,
    /// Capture the latest animated local transforms and derive target
    /// velocities for fixed-step drive. This stage records animated transforms
    /// and derives target velocities for following drive calculations.
    CaptureTargets,
    /// Blend fixed-step-interpolated physics poses into local animated bone
    /// transforms. This render schedule stage blends interpolated physics poses
    /// before Bevy propagates global transforms.
    Writeback,
}

/// Fixed-rate ragdoll systems ordered before, around, and after the backend's
/// physics step.
///
/// Backends write body state in `Apply`, integrate their world, and publish
/// updated state in `Read`. Core systems compute drive targets earlier and
/// handle settling or budget eviction afterward.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, SystemSet)]
pub enum RagdollFixedSystems {
    /// User behavior, balance, and target-adjustment systems that run before
    /// core drive calculation. Application systems write pose overrides here
    /// before core target-driven muscle and pin calculations begin.
    Behaviour,
    /// Core calculation of pin forces, native motor targets, and fallback joint
    /// torques. The core publishes per-body pin outputs and joint targets here
    /// before the backend applies forces.
    Drive,
    /// Backend writes of force, target, and body state immediately before
    /// physics integration. The backend consumes targets and impulses during
    /// this stage before integrating its physics world.
    Apply,
    /// Backend reads of completed poses, velocities, contacts, and sleep state
    /// after integration. Backends publish simulated poses, velocities,
    /// contacts, and sleep markers after completing integration.
    Read,
    /// Core budget eviction and settle checks after the backend publishes each
    /// completed step. Core bookkeeping updates settle state and enforces
    /// budgets after backend state has been read.
    AfterStep,
}
