//! Public character state, target history, drive controls, and body
//! relationships.
//!
//! Insert [`Ragdoll`] on an animated character and change [`RagdollMode`] to
//! move between animation, kinematic following, dynamic simulation, and
//! freezing. Drive and blend strengths are checked at their setters and remain
//! in `0..=1`; target poses and body relationships are managed by runtime
//! systems. Backends can query body relationships without owning character-side
//! profile data.

use bevy::math::{Isometry3d, Quat};
use bevy::prelude::{Component, Entity};

use crate::profile::{BodyIndex, RagdollProfile};

/// A finite unit-range strength used for muscle, pin, and body weights.
#[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd, bevy::prelude::Reflect)]
struct Strength(f32);

impl Strength {
    /// Clamps finite input and maps non-finite input to zero.
    fn clamped(value: f32) -> Self {
        if value.is_finite() {
            Self(value.clamp(0.0, 1.0))
        } else {
            Self::default()
        }
    }

    /// Returns the value in `0..=1`.
    const fn get(self) -> f32 {
        self.0
    }
}

/// Marks a character whose descendants form a ragdoll skeleton.
///
/// `Ragdoll::default()` generates the profile from the skeleton once its bones
/// exist, for example after a glTF scene spawns. Inserting this component also
/// inserts its mode, drive, blend, per-body strengths, and hit-history
/// defaults when the character does not already have them.
#[derive(Component, Clone, Debug, Default, PartialEq, bevy::prelude::Reflect)]
#[require(
    RagdollMode,
    RagdollDrive,
    RagdollBlend,
    RagdollBodyWeights,
    super::hit::LastHit
)]
pub struct Ragdoll {
    /// Profile used for body creation. `None` makes the runtime generate one
    /// from the skeleton under the character and store the new handle here.
    pub profile: Option<bevy::asset::Handle<RagdollProfile>>,
    /// Total mass in kilograms for a generated profile; `None` derives each
    /// body's mass from its collider volume and a uniform density.
    pub mass: Option<crate::profile::Mass>,
    /// Sparse per-bone overrides for a generated profile, usually loaded from a
    /// `.ragdoll.ron` file; `None` keeps every generated value.
    pub overrides: Option<bevy::asset::Handle<crate::auto::RagdollOverrides>>,
}

impl Ragdoll {
    /// Uses a prebuilt profile instead of generating one from the skeleton, for
    /// rigs whose bodies were authored in code or loaded from an asset.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy::asset::Handle;
    /// use bevy_ragdoll::{Ragdoll, RagdollProfile};
    ///
    /// let ragdoll = Ragdoll::new(Handle::<RagdollProfile>::default());
    /// assert!(ragdoll.profile.is_some());
    /// ```
    #[must_use]
    pub const fn new(profile: bevy::asset::Handle<RagdollProfile>) -> Self {
        Self {
            profile: Some(profile),
            mass: None,
            overrides: None,
        }
    }
}

/// The character's closed animation and physics state, changed through Bevy's
/// component storage.
///
/// `Animated` owns no physics bodies; `Kinematic` follows captured targets;
/// `Dynamic` integrates backend forces; and `Frozen` retains the last pose. The
/// runtime creates and removes body entities at these transitions, so systems
/// should change the mode rather than editing body kinds directly.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq, bevy::prelude::Reflect)]
pub enum RagdollMode {
    /// Animation drives all bones and the runtime keeps no physics body
    /// entities for this character.
    /// The character follows animation, owns no physics body entities, and
    /// remains the fallback after binding failure.
    #[default]
    Animated,
    /// Physics bodies follow animation targets without force integration and
    /// remain queryable. The runtime creates queryable bodies that follow
    /// animation targets without integrating forces or impulses.
    Kinematic,
    /// Bodies integrate backend forces and impulses while drive systems compute
    /// targets each step. The backend integrates body forces while core drive
    /// systems write joint targets and pin outputs each step.
    Dynamic,
    /// Bodies retain their last physics poses and ignore dynamic integration
    /// until mode changes. The runtime keeps body entities and their last poses
    /// while the backend disables dynamic integration.
    Frozen,
}

/// Whole-ragdoll muscle and world-space pin strengths used by the fixed drive
/// system.
///
/// Both values are finite and clamped to `0..=1`; non-finite inputs become
/// zero. Muscle strength scales joint motors, while pin strength scales
/// root-target forces and torques independently.
#[derive(Component, Clone, Copy, Debug, PartialEq, bevy::prelude::Reflect)]
pub struct RagdollDrive {
    /// Strength of joint motors in `0..=1`.
    muscle: Strength,
    /// Strength of the pelvis and chest pin in `0..=1`.
    pin: Strength,
}

impl Default for RagdollDrive {
    fn default() -> Self {
        Self::new(1.0, 1.0)
    }
}

impl RagdollDrive {
    /// Creates whole-ragdoll muscle and pin strengths, clamping finite values
    /// to `0..=1`.
    ///
    /// Non-finite inputs become zero before drive systems use them, so invalid
    /// floats cannot create unbounded forces. The muscle and pin channels
    /// remain independent after construction.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::components::RagdollDrive;
    ///
    /// let drive = RagdollDrive::new(1.5, f32::NAN); assert_eq!(drive.muscle(),
    /// 1.0); assert_eq!(drive.pin(), 0.0);
    /// ```
    #[must_use]
    pub fn new(muscle: f32, pin: f32) -> Self {
        Self {
            muscle: Strength::clamped(muscle),
            pin: Strength::clamped(pin),
        }
    }

    /// Returns the finite muscle strength stored in the unit interval.
    ///
    /// Drive systems multiply this value by per-body overrides before computing
    /// motor gains, and callers can inspect it without changing the component's
    /// validated state.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::components::RagdollDrive;
    ///
    /// assert_eq!(RagdollDrive::default().muscle(), 1.0);
    /// ```
    #[must_use]
    pub const fn muscle(self) -> f32 {
        self.muscle.get()
    }

    /// Returns the finite pin strength stored in the unit interval.
    ///
    /// The fixed drive system uses this value for target-position force and
    /// rotation torque, while per-body pin overrides can reduce the
    /// contribution for selected profile bodies.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::components::RagdollDrive;
    ///
    /// assert_eq!(RagdollDrive::default().pin(), 1.0);
    /// ```
    #[must_use]
    pub const fn pin(self) -> f32 {
        self.pin.get()
    }

    /// Replaces both strengths using the same finite unit-range validation as
    /// construction.
    ///
    /// The assignments take effect on the next fixed drive calculation, and
    /// non-finite values become zero instead of propagating invalid numbers
    /// into force or torque calculations.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::components::RagdollDrive;
    ///
    /// let mut drive = RagdollDrive::default(); drive.set(0.25, 0.75);
    /// assert_eq!(drive.muscle(), 0.25);
    /// ```
    pub fn set(&mut self, muscle: f32, pin: f32) {
        self.muscle = Strength::clamped(muscle);
        self.pin = Strength::clamped(pin);
    }
}

/// Per-body multipliers for muscle and pin strengths, indexed in profile body
/// order.
///
/// Each channel is finite and clamped to `0..=1`, with non-finite input mapped
/// to zero. Missing entries use the character-wide strength unchanged, which
/// lets callers override only selected joints or pins without allocating values
/// for every profile body.
#[derive(Clone, Copy, Debug, PartialEq, bevy::prelude::Reflect)]
pub struct BodyWeights {
    /// Muscle multiplier in `0..=1`.
    muscle: Strength,
    /// Pin multiplier in `0..=1`.
    pin: Strength,
}

impl Default for BodyWeights {
    fn default() -> Self {
        Self::new(1.0, 1.0)
    }
}

impl BodyWeights {
    /// Creates per-body muscle and pin multipliers with finite unit-range
    /// validation.
    ///
    /// Non-finite values become zero, and finite values are clamped to `0..=1`.
    /// The returned value can be placed in [`RagdollBodyWeights`] and indexed
    /// by validated profile body position.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::components::BodyWeights;
    ///
    /// let weights = BodyWeights::new(0.5, 1.5); assert_eq!(weights.pin(),
    /// 1.0);
    /// ```
    #[must_use]
    pub fn new(muscle: f32, pin: f32) -> Self {
        Self {
            muscle: Strength::clamped(muscle),
            pin: Strength::clamped(pin),
        }
    }

    /// Returns this body's finite muscle multiplier in `0..=1`.
    ///
    /// The fixed drive system multiplies the character-wide muscle strength by
    /// this value before calculating this body's joint motor gains.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::components::BodyWeights;
    ///
    /// assert_eq!(BodyWeights::default().muscle(), 1.0);
    /// ```
    #[must_use]
    pub const fn muscle(self) -> f32 {
        self.muscle.get()
    }

    /// Returns this body's finite pin multiplier in `0..=1`.
    ///
    /// The fixed drive system multiplies the character-wide pin strength by
    /// this value before applying forces and torques toward the animated
    /// target.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::components::BodyWeights;
    ///
    /// assert_eq!(BodyWeights::default().pin(), 1.0);
    /// ```
    #[must_use]
    pub const fn pin(self) -> f32 {
        self.pin.get()
    }
}

/// Optional body-specific multipliers indexed by [`crate::profile::BodyIndex`].
///
/// Entries use the profile's stable parent-first body order. If a profile body
/// has no override, the runtime uses the whole-ragdoll muscle and pin values
/// without allocating a default entry.
#[derive(Component, Clone, Debug, Default, PartialEq, bevy::prelude::Reflect)]
pub struct RagdollBodyWeights {
    /// Current multipliers in parent-first profile body order.
    weights: Vec<BodyWeights>,
    /// Authored multipliers restored by hit recovery in profile body order.
    base_weights: Vec<BodyWeights>,
}

impl RagdollBodyWeights {
    /// Stores per-body multipliers in the validated profile's parent-first
    /// order.
    ///
    /// The vector may be shorter than the profile; omitted entries inherit
    /// whole-ragdoll strengths. Extra entries are retained but never read
    /// because no validated body index refers to them.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::components::{BodyWeights,
    /// RagdollBodyWeights};
    ///
    /// let weights = RagdollBodyWeights::new(vec![BodyWeights::default()]);
    /// assert_eq!(weights.get(0), Some(BodyWeights::default()));
    /// ```
    #[must_use]
    pub fn new(weights: Vec<BodyWeights>) -> Self {
        Self {
            base_weights: weights.clone(),
            weights,
        }
    }

    /// Extends current and authored values to cover every validated profile body.
    ///
    /// Existing overrides remain unchanged. Missing body positions inherit the
    /// full-strength default used by the whole-ragdoll drive.
    pub(crate) fn initialize_profile_bodies(&mut self, body_count: usize) {
        self.weights
            .resize(body_count.max(self.weights.len()), BodyWeights::default());
        self.base_weights.resize(
            body_count.max(self.base_weights.len()),
            BodyWeights::default(),
        );
    }

    /// Replaces one body's authored baseline and its current multipliers.
    ///
    /// Hit recovery returns to this value. Missing earlier positions are filled
    /// with full-strength defaults, and positions beyond the active profile
    /// remain unused by runtime body indexes.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::{BodyIndex, runtime::components::{BodyWeights, RagdollBodyWeights}};
    ///
    /// let mut weights = RagdollBodyWeights::default();
    /// let index = BodyIndex::try_from(1).expect("one is within the profile limit");
    /// weights.set(index, BodyWeights::default());
    /// assert_eq!(weights.get(1), Some(BodyWeights::default()));
    /// ```
    pub fn set(&mut self, index: BodyIndex, weights: BodyWeights) {
        let position = index.get();
        let required_len = position + 1;
        // Extend both vectors before replacing the checked profile position.
        if self.weights.len() < required_len {
            self.weights.resize(required_len, BodyWeights::default());
        }
        if self.base_weights.len() < required_len {
            self.base_weights
                .resize(required_len, BodyWeights::default());
        }
        // Store the authored value as both the initial strength and recovery target.
        if let Some(current) = self.weights.get_mut(position) {
            *current = weights;
        }
        if let Some(base) = self.base_weights.get_mut(position) {
            *base = weights;
        }
    }

    /// Returns the optional multiplier stored for a zero-based profile body
    /// position.
    ///
    /// `None` means the body inherits whole-ragdoll strength, while `Some`
    /// supplies explicit muscle and pin multipliers. This lookup does not
    /// allocate and returns a copied value.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::components::{BodyWeights,
    /// RagdollBodyWeights};
    ///
    /// let weights = RagdollBodyWeights::new(vec![BodyWeights::default()]);
    /// assert_eq!(weights.get(1), None);
    /// ```
    #[must_use]
    pub fn get(&self, index: usize) -> Option<BodyWeights> {
        self.weights.get(index).copied()
    }

    /// Returns one body's authored recovery target, defaulting to full strength.
    pub(crate) fn base(&self, index: usize) -> BodyWeights {
        self.base_weights.get(index).copied().unwrap_or_default()
    }

    /// Replaces current strengths for one checked profile body, filling omitted
    /// earlier entries with full-strength defaults.
    pub(crate) fn set_current(&mut self, index: BodyIndex, weights: BodyWeights) {
        let position = index.get();
        let required_len = position + 1;
        // Keep current and authored vectors aligned for later hit recovery.
        if self.weights.len() < required_len {
            self.weights.resize(required_len, BodyWeights::default());
        }
        if self.base_weights.len() < required_len {
            self.base_weights
                .resize(required_len, BodyWeights::default());
        }
        // Replace current values without changing the authored recovery target.
        if let Some(current) = self.weights.get_mut(position) {
            *current = weights;
        }
    }
}

impl AsRef<[BodyWeights]> for RagdollBodyWeights {
    fn as_ref(&self) -> &[BodyWeights] {
        &self.weights
    }
}

/// The animation-to-physics display blend used while writing body poses into
/// skeleton bones.
///
/// A value of zero preserves the animated local transform, one uses the physics
/// pose, and intermediate values interpolate translation and rotation. Setters
/// clamp finite values and map non-finite input to zero before writeback reads
/// the component.
#[derive(Component, Clone, Copy, Debug, PartialEq, bevy::prelude::Reflect)]
pub struct RagdollBlend {
    /// Physics contribution in `0..=1`.
    weight: Strength,
}

impl Default for RagdollBlend {
    fn default() -> Self {
        Self::new(1.0)
    }
}

impl RagdollBlend {
    /// Creates a physics blend after clamping finite input to the unit
    /// interval.
    ///
    /// Non-finite input becomes zero, leaving the rendered skeleton fully
    /// animation-driven until a later valid value is supplied.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::components::RagdollBlend;
    ///
    /// assert_eq!(RagdollBlend::new(1.5).get(), 1.0);
    /// ```
    #[must_use]
    pub fn new(weight: f32) -> Self {
        Self {
            weight: Strength::clamped(weight),
        }
    }

    /// Returns the finite physics contribution stored in `0..=1`.
    ///
    /// Writeback multiplies this value by the corresponding per-body muscle
    /// override before interpolating each body-backed bone toward its physics
    /// pose.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::components::RagdollBlend;
    ///
    /// assert_eq!(RagdollBlend::default().get(), 1.0);
    /// ```
    #[must_use]
    pub const fn get(self) -> f32 {
        self.weight.get()
    }

    /// Replaces the blend using the same finite unit-range validation as
    /// construction.
    ///
    /// The next writeback reads the updated value; non-finite input becomes
    /// zero and finite values outside the interval are clamped before skeleton
    /// transforms change.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::components::RagdollBlend;
    ///
    /// let mut blend = RagdollBlend::default(); blend.set(0.5);
    /// assert_eq!(blend.get(), 0.5);
    /// ```
    pub fn set(&mut self, weight: f32) {
        self.weight = Strength::clamped(weight);
    }
}

/// Previous and current skeleton-space targets with velocities derived during
/// animation capture.
///
/// The runtime updates this component after animation systems and reads it from
/// the fixed drive stage. Public accessors require a checked profile body index
/// and return `None` when capture has not supplied that body, preserving the
/// profile's parent-first ordering.
#[derive(Component, Clone, Debug, Default, PartialEq, bevy::prelude::Reflect)]
pub struct RagdollTargetPose {
    /// Target poses from the preceding capture in profile body order.
    previous: Vec<Isometry3d>,
    /// Target poses from the latest capture in profile body order.
    current: Vec<Isometry3d>,
    /// Velocities derived from the last two captures in profile body order.
    velocities: Vec<super::body::BodyVelocity>,
}

impl RagdollTargetPose {
    /// Returns the latest captured skeleton-space target for the checked
    /// profile body index.
    ///
    /// The pose is absent until capture supplies that index, and returned
    /// isometries preserve the target's skeleton-space origin and orientation
    /// without a global-transform round trip.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::profile::BodyIndex; use
    /// bevy_ragdoll::runtime::components::RagdollTargetPose;
    ///
    /// let targets = RagdollTargetPose::default(); let index =
    /// BodyIndex::try_from(0).expect("zero is a valid body index");
    /// assert_eq!(targets.current_pose(index), None);
    /// ```
    #[must_use]
    pub fn current_pose(&self, index: BodyIndex) -> Option<Isometry3d> {
        self.current.get(index.get()).copied()
    }

    /// Returns the preceding captured skeleton-space target for the checked
    /// body index.
    ///
    /// Capture keeps the prior frame's pose for interpolation and velocity
    /// derivation; `None` indicates that no prior entry exists for this profile
    /// position.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::profile::BodyIndex; use
    /// bevy_ragdoll::runtime::components::RagdollTargetPose;
    ///
    /// let targets = RagdollTargetPose::default(); let index =
    /// BodyIndex::try_from(0).expect("zero is a valid body index");
    /// assert_eq!(targets.previous_pose(index), None);
    /// ```
    #[must_use]
    pub fn previous_pose(&self, index: BodyIndex) -> Option<Isometry3d> {
        self.previous.get(index.get()).copied()
    }

    /// Returns the captured linear and angular velocity for the checked profile
    /// body index.
    ///
    /// Velocities are expressed in skeleton coordinates before drive transforms
    /// them into world space. `None` means the capture history does not contain
    /// this body position.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::profile::BodyIndex; use
    /// bevy_ragdoll::runtime::components::RagdollTargetPose;
    ///
    /// let targets = RagdollTargetPose::default(); let index =
    /// BodyIndex::try_from(0).expect("zero is a valid body index");
    /// assert_eq!(targets.velocity(index), None);
    /// ```
    #[must_use]
    pub fn velocity(&self, index: BodyIndex) -> Option<super::body::BodyVelocity> {
        self.velocities.get(index.get()).copied()
    }

    /// Borrows current poses in profile order for body construction.
    pub(crate) fn current(&self) -> &[Isometry3d] {
        &self.current
    }

    /// Borrows captured velocities in profile order for body construction.
    pub(crate) fn velocities(&self) -> &[super::body::BodyVelocity] {
        &self.velocities
    }

    /// Replaces the capture history after validating all pose and velocity
    /// values.
    pub(crate) fn record(
        &mut self,
        poses: Vec<Isometry3d>,
        velocities: Vec<super::body::BodyVelocity>,
    ) {
        // Retain the prior pose frame so the drive can derive animation target velocity.
        if self.current.is_empty() {
            self.previous.clone_from(&poses);
        } else {
            self.previous.clone_from(&self.current);
        }
        self.current = poses;
        self.velocities = velocities;
    }
}

/// Fixed-step edits that replace or offset captured target poses before drive
/// calculation.
///
/// Replacement poses, additive rotations, and root offset use profile body
/// order and parent-frame composition. Behavior systems update this component
/// in the `Behaviour` set before the core drive stage consumes adjusted
/// targets.
#[derive(Component, Clone, Debug, PartialEq, bevy::prelude::Reflect)]
pub struct RagdollTargetAdjust {
    /// Optional replacement pose by body index.
    replace: Vec<Option<Isometry3d>>,
    /// Additive parent-frame rotation by body index.
    additive: Vec<Quat>,
    /// Root transform composed after body target adjustments.
    root_offset: Isometry3d,
}

impl Default for RagdollTargetAdjust {
    fn default() -> Self {
        Self {
            replace: Vec::new(),
            additive: Vec::new(),
            root_offset: Isometry3d::IDENTITY,
        }
    }
}

/// A stable character identity assigned once when the runtime successfully
/// binds its skeleton.
///
/// The monotonically increasing value orders activations and remains attached
/// to the character while body entities are created, frozen, or removed. It is
/// generated by the plugin resource and is not intended for application
/// construction or persistence across world reloads.
#[derive(
    Component, Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, bevy::prelude::Reflect,
)]
pub struct RagdollId(u64);

impl RagdollId {
    /// Returns the internal ordering value used only by runtime budget
    /// bookkeeping.
    pub(crate) const fn get(self) -> u64 {
        self.0
    }

    /// Creates an identity from the plugin's monotonic counter.
    pub(crate) const fn new(value: u64) -> Self {
        Self(value)
    }
}

/// Relationship component on a physics body that identifies its owning
/// character.
///
/// Bevy maintains the reciprocal [`RagdollBodies`] relationship as body
/// entities are spawned and removed. The body entity remains a top-level
/// physics object, while this component lets backend systems find the character
/// without traversing its animated skeleton hierarchy.
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq, bevy::prelude::Reflect)]
#[relationship(relationship_target = RagdollBodies)]
pub struct RagdollBodyOf(pub Entity);

/// Relationship target listing the physics body entities owned by one
/// character.
///
/// Bevy updates this collection through [`RagdollBodyOf`], including linked
/// despawn behavior. The iteration order is Bevy relationship storage order and
/// is not a substitute for profile body order; use each entity's
/// [`crate::profile::BodyIndex`] for stable indexing.
#[derive(Component, Clone, Debug, Default, PartialEq, Eq, bevy::prelude::Reflect)]
#[relationship_target(relationship = RagdollBodyOf, linked_spawn)]
pub struct RagdollBodies(Vec<Entity>);

impl RagdollBodies {
    /// Iterates related body entities in Bevy's stored relationship order
    /// without allocation.
    ///
    /// The iterator yields copied entity identifiers and borrows the
    /// relationship collection for its lifetime. This order is storage-defined,
    /// so callers needing profile order should inspect each body's validated
    /// [`crate::profile::BodyIndex`].
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::components::RagdollBodies;
    ///
    /// assert_eq!(RagdollBodies::default().iter().count(), 0);
    /// ```
    pub fn iter(&self) -> impl Iterator<Item = Entity> + '_ {
        self.0.iter().copied()
    }

    /// Returns the number of body entities related to this character.
    ///
    /// The count reflects Bevy's current relationship storage and is zero for
    /// an animated character before activation or after all linked bodies are
    /// removed.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::components::RagdollBodies;
    ///
    /// assert_eq!(RagdollBodies::default().len(), 0);
    /// ```
    #[must_use]
    pub const fn len(&self) -> usize {
        self.0.len()
    }

    /// Returns whether Bevy currently stores no physics bodies for this
    /// character.
    ///
    /// The result is true before the runtime activates a ragdoll and after
    /// linked body despawn has completed; it does not report whether the
    /// profile itself contains body definitions.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::components::RagdollBodies;
    ///
    /// assert!(RagdollBodies::default().is_empty());
    /// ```
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use bevy::asset::Handle;
    use bevy::math::{Isometry3d, Quat, Vec3};
    use bevy::prelude::{Entity, World};

    use crate::profile::{BodyIndex, RagdollProfile};

    use super::{
        BodyWeights, Ragdoll, RagdollBlend, RagdollBodies, RagdollBodyWeights, RagdollDrive,
        RagdollTargetAdjust, RagdollTargetPose,
    };

    #[test]
    fn component_accessors_keep_clamped_values_and_target_history() {
        let mut drive = RagdollDrive::new(0.25, 0.75);
        drive.set(0.5, 0.0);
        assert_eq!(drive.muscle(), 0.5);
        assert_eq!(drive.pin(), 0.0);

        let default_weights = BodyWeights::default();
        assert_eq!(default_weights.muscle(), 1.0);
        assert_eq!(default_weights.pin(), 1.0);
        let overrides = RagdollBodyWeights::new(vec![BodyWeights::new(0.25, 0.75)]);
        assert_eq!(overrides.as_ref(), &[BodyWeights::new(0.25, 0.75)]);
        assert_eq!(overrides.get(1), None);

        let mut blend = RagdollBlend::default();
        blend.set(0.25);
        assert_eq!(blend.get(), 0.25);

        let index = BodyIndex::try_from(0_usize).expect("the first body index is valid");
        let first = Isometry3d::IDENTITY;
        let second = Isometry3d::from_translation(Vec3::X);
        let velocity = super::super::body::BodyVelocity {
            linear: Vec3::Y,
            angular: Vec3::Z,
        };
        let mut targets = RagdollTargetPose::default();
        assert_eq!(targets.current_pose(index), None);
        assert_eq!(targets.previous_pose(index), None);
        assert_eq!(targets.velocity(index), None);
        targets.record(vec![first], vec![velocity]);
        targets.record(vec![second], vec![velocity]);
        assert_eq!(targets.previous_pose(index), Some(first));
        assert_eq!(targets.current_pose(index), Some(second));
        assert_eq!(targets.velocity(index), Some(velocity));

        let adjustments = RagdollTargetAdjust::default();
        assert_eq!(adjustments.replace, []);
        assert_eq!(adjustments.additive, []);
        assert_eq!(adjustments.root_offset, Isometry3d::IDENTITY);
        assert_eq!(Quat::IDENTITY, adjustments.root_offset.rotation);
    }

    /// Fills bound body positions without replacing authored or extra weights.
    #[test]
    fn profile_weight_initialization_preserves_authored_entries() {
        let authored = BodyWeights::new(0.25, 0.5);
        let mut weights = RagdollBodyWeights::new(vec![authored]);

        weights.initialize_profile_bodies(3);

        assert_eq!(
            weights.as_ref(),
            &[authored, BodyWeights::default(), BodyWeights::default()]
        );
        assert_eq!(weights.base(0), authored);
        assert_eq!(weights.base(1), BodyWeights::default());
        let third_index = BodyIndex::try_from(2).expect("the third body index is valid");
        let extra = BodyWeights::new(0.75, 0.5);
        weights.set(third_index, extra);
        weights.initialize_profile_bodies(1);

        assert_eq!(weights.as_ref(), &[authored, BodyWeights::default(), extra]);
        assert_eq!(weights.base(2), extra);
    }

    /// Creates required control state before binding can spawn physics bodies.
    #[test]
    fn ragdoll_requires_weights_and_hit_history_at_spawn() {
        let mut world = World::new();
        let character = world
            .spawn(Ragdoll::new(Handle::<RagdollProfile>::default()))
            .id();

        assert!(world.get::<RagdollBodyWeights>(character).is_some());
        assert_eq!(
            world
                .get::<super::super::hit::LastHit>(character)
                .and_then(|last_hit| last_hit.last_at()),
            None
        );
    }

    #[test]
    fn body_relationship_target_reports_empty_and_populated_states() {
        let empty = RagdollBodies::default();
        assert_eq!(empty.len(), 0);
        assert_eq!(empty.len(), 0);
        assert_eq!(empty.iter().count(), 0);

        let populated = RagdollBodies(vec![Entity::PLACEHOLDER]);
        assert!(!populated.is_empty());
        assert_eq!(populated.len(), 1);
        assert_eq!(populated.iter().collect::<Vec<_>>(), [Entity::PLACEHOLDER]);
    }
}
