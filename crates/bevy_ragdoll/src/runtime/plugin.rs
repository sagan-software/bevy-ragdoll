//! Bevy plugin setup for profile assets, runtime resources, reflection, and
//! schedules.
//!
//! Add [`RagdollPlugin`] after the Bevy asset and animation plugins, then add
//! one physics backend. The plugin initializes shared profile and budget
//! resources, registers public runtime state with reflection, and orders
//! variable-rate capture and writeback around animation and transform
//! propagation. It does not create a physics world or choose a backend
//! implementation.

use crate::profile::RagdollProfile;
use bevy::app::{AnimationSystems, FixedUpdate};
use bevy::asset::AssetApp;
use bevy::ecs::schedule::{InternedScheduleLabel, IntoScheduleConfigs, ScheduleLabel};
use bevy::prelude::{App, Plugin, PostUpdate, Reflect, Resource};
use bevy::transform::TransformSystems;

use super::backend::{BackendCapabilities, BodyContact, BodyContacts, RayHit};
use super::body::{
    BodyAtRest, BodyDriveOutput, BodyKind, BodyMass, BodyPhysicsPose, BodyShape, BodyVelocity,
    JointDriveTarget, JointToParent, NoContactWith,
};
use super::budget::{EvictPolicy, RagdollBudget};
use super::components::{
    BodyWeights, Ragdoll, RagdollBlend, RagdollBodies, RagdollBodyOf, RagdollBodyWeights,
    RagdollDrive, RagdollId, RagdollMode, RagdollTargetAdjust, RagdollTargetPose,
};
use super::events::{RagdollActivated, RagdollBudgetEvicted, RagdollFrozen, RagdollSettled};
use super::messages::{
    HitKind, RagdollHit, RagdollImpulse, RagdollRaycast, RagdollRaycastResponse, RagdollRequestId,
};
use super::sets::{RagdollFixedSystems, RagdollSystems};
use super::settings::RagdollPhysicsSettings;
use super::skeleton::{self, RagdollError, RagdollIdCounter};

/// Configures profile loading, reflection, and runtime system ordering.
///
/// Add one instance before spawning characters with [`Ragdoll`]. The plugin
/// initializes the profile asset, shared settings, budget, messages, and
/// schedules. A physics backend must be added separately because the core
/// runtime does not create or step physics worlds.
#[derive(Clone, Copy, Debug)]
pub struct RagdollPlugin {
    /// Fixed schedule that runs drive, backend apply and read, and after-step
    /// runtime systems. The runtime uses this label to order core and backend
    /// systems around fixed simulation stages.
    pub fixed_schedule: InternedScheduleLabel,
}

impl Default for RagdollPlugin {
    fn default() -> Self {
        Self {
            fixed_schedule: FixedUpdate.intern(),
        }
    }
}

/// Resource recording the fixed schedule selected by [`RagdollPlugin`].
///
/// Backend plugins read this label during their build step and attach apply and
/// read systems to the corresponding [`RagdollFixedSystems`] sets. The resource
/// prevents each backend from guessing which fixed schedule the application
/// selected.
#[derive(Clone, Copy, Debug, Resource, Reflect)]
#[reflect(from_reflect = false)]
pub struct RagdollFixedSchedule {
    /// Interned Bevy label used by runtime and backend fixed systems for this
    /// application. Backend plugins read this interned label during setup to
    /// order their apply and read systems.
    #[reflect(ignore)]
    pub label: InternedScheduleLabel,
}

impl Plugin for RagdollPlugin {
    fn build(&self, app: &mut App) {
        // Initialize resources and reflection before systems that consume their schedule state.
        initialize_resources(app, self.fixed_schedule);
        initialize_messages(app);
        register_reflected_types(app);
        configure_system_sets(app, self.fixed_schedule);
        install_runtime_systems(app, self.fixed_schedule);
    }
}

/// Initializes resources whose values must exist before runtime systems
/// execute.
fn initialize_resources(app: &mut App, fixed_schedule: InternedScheduleLabel) {
    // The plugin records its schedule before backends inspect the configuration.
    app.insert_resource(RagdollFixedSchedule {
        label: fixed_schedule,
    });
    app.init_asset::<RagdollProfile>()
        .init_resource::<RagdollPhysicsSettings>()
        .init_resource::<RagdollBudget>()
        .init_resource::<RagdollIdCounter>();
    #[cfg(feature = "serialize")]
    app.init_asset_loader::<crate::profile::RagdollProfileLoader>();
}

/// Registers request and response message types with Bevy's world.
fn initialize_messages(app: &mut App) {
    // Every request has one matching response message type and a typed request identity.
    app.add_message::<RagdollImpulse>()
        .add_message::<RagdollHit>()
        .add_message::<RagdollRaycast>()
        .add_message::<RagdollRaycastResponse>();
}

/// Registers user-facing runtime state with Bevy reflection.
fn register_reflected_types(app: &mut App) {
    // Keep registration stages focused so each domain has one visible reflection boundary.
    register_profile_types(app);
    register_backend_types(app);
    register_runtime_types(app);
}

/// Registers profile assets and Skein authoring annotations for editor
/// inspection.
///
/// The profile entries are registered before runtime components that reference
/// profile body indexes or shape data, preserving nested reflection support for
/// editor tools.
fn register_profile_types(app: &mut App) {
    // Register referenced profile values before components that hold the asset handle.
    app.register_type::<RagdollProfile>()
        .register_type::<crate::profile::Body>()
        .register_type::<crate::profile::Joint>()
        .register_type::<crate::profile::ShapeSpec>()
        .register_type::<crate::profile::JointLimits>()
        .register_type::<crate::profile::Mass>()
        .register_type::<crate::profile::BodyIndex>()
        .register_type::<crate::skein::RagdollBody>()
        .register_type::<crate::skein::AngleRange>()
        .register_type::<crate::skein::RagdollJoint>();
}

/// Registers backend-neutral body and capability values for backend plugin
/// inspection.
///
/// These types describe data exchanged around the physics step and do not
/// depend on one engine's component or constraint representation.
fn register_backend_types(app: &mut App) {
    // Keep body state and physics result types available in Bevy's reflection registry.
    app.register_type::<RayHit>()
        .register_type::<BodyContact>()
        .register_type::<BackendCapabilities>()
        .register_type::<BodyContacts>()
        .register_type::<BodyShape>()
        .register_type::<BodyMass>()
        .register_type::<BodyVelocity>()
        .register_type::<BodyPhysicsPose>()
        .register_type::<BodyDriveOutput>()
        .register_type::<BodyKind>()
        .register_type::<JointToParent>()
        .register_type::<JointDriveTarget>()
        .register_type::<NoContactWith>()
        .register_type::<BodyAtRest>();
}

/// Registers character state, messages, events, settings, and schedule
/// resources.
///
/// Applications can inspect these values through Bevy reflection without
/// exposing the runtime's private skeleton maps or internal settle bookkeeping.
fn register_runtime_types(app: &mut App) {
    // Register character components before their interaction messages and observer events.
    app.register_type::<EvictPolicy>()
        .register_type::<RagdollBudget>()
        .register_type::<Ragdoll>()
        .register_type::<RagdollMode>()
        .register_type::<RagdollDrive>()
        .register_type::<BodyWeights>()
        .register_type::<RagdollBodyWeights>()
        .register_type::<RagdollBlend>()
        .register_type::<RagdollTargetPose>()
        .register_type::<RagdollTargetAdjust>()
        .register_type::<RagdollId>()
        .register_type::<RagdollBodyOf>()
        .register_type::<RagdollBodies>()
        .register_type::<RagdollError>()
        .register_type::<RagdollFixedSchedule>();
    register_interaction_types(app);
}

/// Registers hit, query, event, and settings values used by application systems.
fn register_interaction_types(app: &mut App) {
    // Keep message and event reflection available to inspectors and observer tooling.
    app.register_type::<RagdollPhysicsSettings>()
        .register_type::<HitKind>()
        .register_type::<RagdollImpulse>()
        .register_type::<RagdollHit>()
        .register_type::<RagdollRaycast>()
        .register_type::<RagdollRaycastResponse>()
        .register_type::<RagdollRequestId>()
        .register_type::<RagdollActivated>()
        .register_type::<RagdollSettled>()
        .register_type::<RagdollFrozen>()
        .register_type::<RagdollBudgetEvicted>();
}

/// Places variable-rate and fixed-rate sets around Bevy animation and physics
/// work.
fn configure_system_sets(app: &mut App, fixed_schedule: InternedScheduleLabel) {
    // Animation capture follows animation, and local writeback precedes global propagation.
    app.configure_sets(
        PostUpdate,
        (
            RagdollSystems::Bind,
            RagdollSystems::CaptureTargets,
            RagdollSystems::Writeback,
        )
            .chain()
            .after(AnimationSystems)
            .before(TransformSystems::Propagate),
    );
    // The backend owns the physics step between the core Apply and Read sets.
    app.configure_sets(
        fixed_schedule,
        (
            RagdollFixedSystems::Behaviour,
            RagdollFixedSystems::Drive,
            RagdollFixedSystems::Apply,
            RagdollFixedSystems::Read,
            RagdollFixedSystems::AfterStep,
        )
            .chain(),
    );
}

/// Installs core systems in the schedules selected by the runtime contract.
fn install_runtime_systems(app: &mut App, fixed_schedule: InternedScheduleLabel) {
    // Binding builds body entities before target capture and writeback can query them.
    app.add_systems(
        PostUpdate,
        skeleton::bind_and_sync.in_set(RagdollSystems::Bind),
    );
    app.add_systems(
        PostUpdate,
        super::capture::capture_targets.in_set(RagdollSystems::CaptureTargets),
    );
    app.add_systems(
        PostUpdate,
        super::writeback::writeback.in_set(RagdollSystems::Writeback),
    );
    app.add_systems(
        fixed_schedule,
        super::drive::drive.in_set(RagdollFixedSystems::Drive),
    );
    app.add_systems(
        fixed_schedule,
        skeleton::update_settle_state.in_set(RagdollFixedSystems::AfterStep),
    );
}
