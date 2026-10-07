//! Hit profiles and default impulse magnitudes for active ragdoll reactions.
//!
//! [`HitKind`](super::messages::HitKind) classifies the source of a hit.
//! [`HitProfile`] selects a showcase impulse magnitude while
//! [`super::messages::RagdollHit`] retains its world-space impulse vector.
//! Hit processing validates vectors and body identity before changing muscle,
//! pin, recovery, or backend impulse state. The module also exposes fixed-clock
//! hit history so applications can inspect rapid-fire scaling and recovery time.

use bevy::prelude::Reflect;
use std::time::Duration;

use crate::profile::{BodyIndex, BodyRole, MAX_BODIES};

use super::body::JointToParent;
use super::components::{
    BodyWeights, RagdollBodies, RagdollBodyOf, RagdollBodyWeights, RagdollMode,
};
use super::messages::{RagdollHit, RagdollImpulse};

/// Body components needed to gather one character's indexed joint tree.
type HitBodyData = (
    bevy::prelude::Entity,
    &'static BodyIndex,
    &'static BodyRole,
    &'static RagdollBodyOf,
    Option<&'static JointToParent>,
    Option<&'static super::body::BodyMass>,
    Option<&'static super::body::BodyPhysicsPose>,
);

/// Body data copied from one shared query row before tree construction.
#[derive(Clone, Copy)]
struct HitBodyRecord {
    /// ECS entity carrying the checked body components.
    entity: bevy::prelude::Entity,
    /// Validated profile position carried by the body entity.
    index: BodyIndex,
    /// Anatomical role used for hit floors and recovery order.
    role: BodyRole,
    /// Character that owns this body entity.
    owner: bevy::prelude::Entity,
    /// Optional parent body entity from the profile joint.
    parent: Option<bevy::prelude::Entity>,
    /// Optional parent-joint frame used to move the remaining impulse.
    joint_frame: Option<bevy::math::Isometry3d>,
    /// Body mass in kilograms, or zero when backend data is absent.
    mass: f32,
    /// Current backend pose when the body has a published physics pose.
    pose: Option<bevy::math::Isometry3d>,
}

/// Profile-ordered entity and physics data used while impulse moves up a body tree.
struct HitImpulseTree {
    /// Number of populated profile positions in the character's body tree.
    body_count: usize,
    /// Body entity at each validated profile position.
    body_entities: [Option<bevy::prelude::Entity>; MAX_BODIES],
    /// Checked index at each body position, absent for empty or malformed slots.
    indexes: [Option<BodyIndex>; MAX_BODIES],
    /// Parent body entity retained until all profile positions are collected.
    parent_entities: [Option<bevy::prelude::Entity>; MAX_BODIES],
    /// Anatomical role at each populated profile position.
    roles: [Option<BodyRole>; MAX_BODIES],
    /// Parent position for each body with a matching parent entity.
    parents: [Option<usize>; MAX_BODIES],
    /// Parent-joint frame for each non-root body.
    joint_frames: [Option<bevy::math::Isometry3d>; MAX_BODIES],
    /// Mass in kilograms at each profile position.
    masses: [f32; MAX_BODIES],
    /// Current backend pose for each available body.
    poses: [Option<bevy::math::Isometry3d>; MAX_BODIES],
}

impl HitImpulseTree {
    /// Copies one character's bounded body data and resolves parent positions.
    fn collect(
        character: bevy::prelude::Entity,
        related_bodies: &RagdollBodies,
        bodies: &bevy::prelude::Query<'_, '_, HitBodyData>,
    ) -> Self {
        let mut tree = Self {
            body_count: 0,
            body_entities: [None; MAX_BODIES],
            indexes: [None; MAX_BODIES],
            parent_entities: [None; MAX_BODIES],
            roles: [None; MAX_BODIES],
            parents: [None; MAX_BODIES],
            joint_frames: [None; MAX_BODIES],
            masses: [0.0; MAX_BODIES],
            poses: [None; MAX_BODIES],
        };

        // Copy matching body values into fixed profile-ordered storage.
        for body in related_bodies.iter() {
            let Ok((entity, index, role, owner, joint, mass, pose)) = bodies.get(body) else {
                continue;
            };
            tree.insert_body(
                character,
                HitBodyRecord {
                    entity,
                    index: *index,
                    role: *role,
                    owner: owner.0,
                    parent: joint.map(|joint| joint.parent),
                    joint_frame: joint.map(|joint| joint.frame),
                    mass: mass.map_or(0.0, |mass| mass.mass),
                    pose: pose.map(|pose| pose.current),
                },
            );
        }

        // Resolve entity relationships after every possible parent slot exists.
        tree.resolve_parent_indexes();
        tree
    }

    /// Inserts one same-owner record when its reflected index stays in bounds.
    fn insert_body(&mut self, character: bevy::prelude::Entity, record: HitBodyRecord) {
        // Reject foreign rows and corrupted reflected indexes before any slot access.
        if record.owner != character || record.index.get() >= MAX_BODIES {
            return;
        }
        let position = record.index.get();
        // Zip equal-sized arrays so every accepted profile position has complete backing storage.
        let slots = self
            .body_entities
            .get_mut(position)
            .zip(self.indexes.get_mut(position))
            .zip(self.parent_entities.get_mut(position))
            .zip(self.roles.get_mut(position))
            .zip(self.parents.get_mut(position))
            .zip(self.joint_frames.get_mut(position))
            .zip(self.masses.get_mut(position))
            .zip(self.poses.get_mut(position));
        if let Some((
            (
                (
                    ((((entity_slot, index_slot), parent_slot), role_slot), parent_index_slot),
                    frame_slot,
                ),
                mass_slot,
            ),
            pose_slot,
        )) = slots
        {
            // Store identity and policy data at the validated profile position.
            *entity_slot = Some(record.entity);
            *index_slot = Some(record.index);
            *parent_slot = record.parent;
            *role_slot = Some(record.role);
            *parent_index_slot = None;
            *frame_slot = record.joint_frame;
            *mass_slot = record.mass;
            *pose_slot = record.pose;
            self.body_count = self.body_count.max(position + 1);
        }
    }

    /// Resolves parent entity references into profile positions after collection.
    fn resolve_parent_indexes(&mut self) {
        // Visit only the populated prefix so parent search remains bounded by profile size.
        for (position, parent_entity) in self
            .parent_entities
            .iter()
            .take(self.body_count)
            .enumerate()
        {
            // Ignore roots and missing parent entities without changing the tree.
            let Some(parent_entity) = parent_entity else {
                continue;
            };
            let parent_position = self
                .body_entities
                .iter()
                .take(self.body_count)
                .position(|entity| *entity == Some(*parent_entity));
            if let Some(parent_slot) = self.parents.get_mut(position) {
                *parent_slot = parent_position;
            }
        }
    }

    /// Returns the indexed body entity when the requested position is populated.
    fn body(&self, position: usize) -> Option<bevy::prelude::Entity> {
        self.body_entities.get(position).copied().flatten()
    }

    /// Returns the parent position stored at one bounded profile slot.
    fn parent(&self, position: usize) -> Option<usize> {
        self.parents.get(position).copied().flatten()
    }

    /// Returns whether a hit's body entity occupies its checked profile index.
    fn has_body_at_index(&self, index: BodyIndex, entity: bevy::prelude::Entity) -> bool {
        self.body(index.get()) == Some(entity) && index.get() < self.body_count
    }
}

/// Reference impulse in kilogram metres per second for full hit severity.
const REFERENCE_IMPULSE: f32 = 40.0;

/// Maximum fractional muscle reduction at full hit severity.
const MAXIMUM_STRENGTH_DROP: f32 = 0.85;

/// Fractional pin reduction applied per unit hit severity.
const PIN_DROP_PER_SEVERITY: f32 = 0.8;

/// Time before a hit strength can begin recovering.
const RECOVERY_DELAY: Duration = Duration::from_millis(100);

/// Full-strength muscle recovery rate in normalized units per second.
const MUSCLE_RECOVERY_RATE: f32 = 1.5;

/// Full-strength pin recovery rate in normalized units per second.
const PIN_RECOVERY_RATE: f32 = 1.0;

/// Rapid-fire interval in which hits increase the drop streak.
const RAPID_FIRE_WINDOW: Duration = Duration::from_millis(300);

/// Increase in the initial muscle drop for each hit within the rapid-fire window.
const STREAK_DROP_MULTIPLIER: f32 = 0.3;

/// Stores fixed-clock timing and rapid-fire state for one character. The
/// component lets hit processing scale consecutive strength drops and delay
/// recovery after accepted messages.
#[derive(bevy::prelude::Component, Clone, Copy, Debug, Default, PartialEq, Reflect)]
pub struct LastHit {
    /// Fixed-clock time at which the most recent accepted hit was processed.
    last_at: Option<Duration>,
    /// Number of hits after the first hit in the current rapid-fire streak.
    streak: u32,
}

impl LastHit {
    /// Returns the number of accepted hits after the first hit in the current
    /// rapid-fire window. Later hits use this count to increase muscle-strength
    /// reduction before the window expires.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::hit::LastHit;
    ///
    /// assert_eq!(LastHit::default().streak(), 0);
    /// ```
    pub const fn streak(self) -> u32 {
        self.streak
    }

    /// Returns the fixed-clock elapsed time recorded for the most recently
    /// accepted hit. `None` means this character has not received an accepted
    /// hit during its current ragdoll session.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::hit::LastHit;
    ///
    /// assert_eq!(LastHit::default().last_at(), None);
    /// ```
    pub const fn last_at(self) -> Option<Duration> {
        self.last_at
    }

    /// Records a validated hit and advances or resets the rapid-fire streak.
    fn record(&mut self, now: Duration) {
        self.streak = match self.last_at {
            Some(last) if now.saturating_sub(last) <= RAPID_FIRE_WINDOW => {
                self.streak.saturating_add(1)
            }
            _ => 0,
        };
        self.last_at = Some(now);
    }
}

/// A showcase attack profile whose magnitude comes from [`HitSettings`].
///
/// Custom magnitudes are expressed in kilogram metres per second. The public
/// payload remains an `f32`, so [`HitSettings::impulse_magnitude`] rejects
/// negative and non-finite custom values at the boundary.
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
pub enum HitProfile {
    /// A light firearm hit with the default 12 kg·m/s impulse. It is the
    /// smallest preset and suits localized recoil responses when an application
    /// selects a pistol reaction.
    Pistol,
    /// A medium firearm hit with the default 20 kg·m/s impulse. It produces a
    /// larger response than pistol while remaining below shotgun strength when
    /// an application selects rifle.
    Rifle,
    /// A close-range firearm hit with the default 60 kg·m/s impulse. It reaches
    /// farther through the body tree than pistol or rifle when an application
    /// selects a shotgun reaction.
    Shotgun,
    /// An unarmed upper-body hit with the default 30 kg·m/s impulse. It uses a
    /// smaller response than a kick when an application selects a punch for
    /// close contact.
    Punch,
    /// An unarmed lower-body hit with the default 60 kg·m/s impulse. It reaches
    /// farther through the body tree than a punch when an application selects
    /// a kick.
    Kick,
    /// A high-energy collision with the default 120 kg·m/s impulse. It affects
    /// more nearby bodies than named light attacks when an application selects
    /// a heavy reaction.
    Heavy,
    /// A wide-area blast with the default 200 kg·m/s impulse. It uses the
    /// largest preset when an application selects an explosion reaction for
    /// nearby bodies.
    Explosion,
    /// An application-selected impulse magnitude in kilogram metres per
    /// second, accepted only when finite and nonnegative. The value is returned
    /// unchanged, including zero, so callers can disable the preset impulse
    /// without changing policy.
    Custom(f32),
}

/// Default impulse magnitudes for the hit profiles in kilogram metres per
/// second.
///
/// The table follows hit-reaction design section 2.1. Applications multiply a
/// normalized world-space direction by the selected magnitude when creating a
/// [`super::messages::RagdollHit`].
#[derive(Clone, Copy, Debug, PartialEq, bevy::prelude::Resource, Reflect)]
pub struct HitSettings {
    /// Default impulse for [`HitProfile::Pistol`] in kilogram metres per second.
    pistol_impulse: f32,
    /// Default impulse for [`HitProfile::Rifle`] in kilogram metres per second.
    rifle_impulse: f32,
    /// Default impulse for [`HitProfile::Shotgun`] in kilogram metres per second.
    shotgun_impulse: f32,
    /// Default impulse for [`HitProfile::Punch`] in kilogram metres per second.
    punch_impulse: f32,
    /// Default impulse for [`HitProfile::Kick`] in kilogram metres per second.
    kick_impulse: f32,
    /// Default impulse for [`HitProfile::Heavy`] in kilogram metres per second.
    heavy_impulse: f32,
    /// Default impulse for [`HitProfile::Explosion`] in kilogram metres per second.
    explosion_impulse: f32,
}

impl Default for HitSettings {
    fn default() -> Self {
        Self {
            pistol_impulse: 12.0,
            rifle_impulse: 20.0,
            shotgun_impulse: 60.0,
            punch_impulse: 30.0,
            kick_impulse: 60.0,
            heavy_impulse: 120.0,
            explosion_impulse: 200.0,
        }
    }
}

impl HitSettings {
    /// Returns the configured preset magnitude, or `None` for malformed custom
    /// magnitudes.
    ///
    /// Named profiles use their configured defaults. Custom values pass this
    /// boundary only when finite and nonnegative; zero is valid and represents
    /// no impulse.
    ///
    /// # Examples
    ///
    /// ```
    /// use bevy_ragdoll::runtime::hit::{HitProfile, HitSettings};
    ///
    /// let settings = HitSettings::default();
    /// assert_eq!(settings.impulse_magnitude(HitProfile::Rifle), Some(20.0));
    /// assert_eq!(settings.impulse_magnitude(HitProfile::Custom(f32::NAN)), None);
    /// ```
    pub fn impulse_magnitude(&self, profile: HitProfile) -> Option<f32> {
        let magnitude = match profile {
            HitProfile::Pistol => self.pistol_impulse,
            HitProfile::Rifle => self.rifle_impulse,
            HitProfile::Shotgun => self.shotgun_impulse,
            HitProfile::Punch => self.punch_impulse,
            HitProfile::Kick => self.kick_impulse,
            HitProfile::Heavy => self.heavy_impulse,
            HitProfile::Explosion => self.explosion_impulse,
            HitProfile::Custom(value) => value,
        };
        (magnitude.is_finite() && magnitude >= 0.0).then_some(magnitude)
    }
}

/// Applies each accepted hit's local weight change and forwards its impulse to
/// the backend before the current physics step.
///
/// The system ignores malformed vectors, stale body entities, and characters
/// that are animated or frozen. A valid hit lowers the addressed body's
/// current muscle and pin multipliers, then distributes a velocity-limited
/// impulse up the character's joint chain.
pub(crate) fn process_hits(
    mut hits: bevy::prelude::MessageReader<'_, '_, RagdollHit>,
    mut impulses: bevy::prelude::MessageWriter<'_, RagdollImpulse>,
    bodies: bevy::prelude::Query<'_, '_, HitBodyData>,
    mut characters: bevy::prelude::Query<
        '_,
        '_,
        (
            &RagdollMode,
            &mut RagdollBodyWeights,
            &mut LastHit,
            &RagdollBodies,
        ),
    >,
    fixed_time: bevy::prelude::Res<'_, bevy::time::Time<bevy::time::Fixed>>,
) {
    // Read each message once and preserve its order for per-character streak updates.
    for hit in hits.read() {
        process_hit(
            hit,
            &bodies,
            characters.reborrow(),
            fixed_time.elapsed(),
            &mut impulses,
        );
    }
}

/// Hit values copied only after vector and body identity validation succeeds.
struct ValidatedHit {
    /// Addressed body entity that receives the first impulse share.
    body: bevy::prelude::Entity,
    /// Checked profile position for the addressed body.
    index: BodyIndex,
    /// Character whose state and body tree receive the hit response.
    character: bevy::prelude::Entity,
    /// World-space contact point in metres.
    point: bevy::math::Vec3,
    /// World-space impulse vector in kilogram metres per second.
    impulse: bevy::math::Vec3,
    /// Finite positive impulse magnitude in kilogram metres per second.
    magnitude: f32,
}

/// Validates one message and copies the fields needed by the response stages.
fn validate_hit(
    hit: &RagdollHit,
    bodies: &bevy::prelude::Query<'_, '_, HitBodyData>,
) -> Option<ValidatedHit> {
    // Reject non-finite points and vectors before any character state is borrowed.
    let magnitude = hit.impulse.length();
    if !hit.point.is_finite()
        || !hit.impulse.is_finite()
        || !magnitude.is_finite()
        || magnitude <= 0.0
    {
        return None;
    }

    // Require a live body with a checked index and character owner.
    let Ok((_, index, _, owner, _, _, _)) = bodies.get(hit.body) else {
        return None;
    };
    Some(ValidatedHit {
        body: hit.body,
        index: *index,
        character: owner.0,
        point: hit.point,
        impulse: hit.impulse,
        magnitude,
    })
}

/// Applies one validated hit after checking owner state and indexed body identity.
fn process_hit(
    hit: &RagdollHit,
    bodies: &bevy::prelude::Query<'_, '_, HitBodyData>,
    mut characters: bevy::prelude::Query<
        '_,
        '_,
        (
            &RagdollMode,
            &mut RagdollBodyWeights,
            &mut LastHit,
            &RagdollBodies,
        ),
    >,
    now: Duration,
    impulses: &mut bevy::prelude::MessageWriter<'_, RagdollImpulse>,
) {
    // Stop when message validation, owner state, or body-tree identity fails.
    let Some(hit) = validate_hit(hit, bodies) else {
        return;
    };
    let Ok((mode, mut weights, mut last_hit, related_bodies)) = characters.get_mut(hit.character)
    else {
        return;
    };
    if !matches!(*mode, RagdollMode::Kinematic | RagdollMode::Dynamic) {
        return;
    }

    // Build only this character's bounded profile tree without temporary maps.
    let tree = HitImpulseTree::collect(hit.character, related_bodies, bodies);
    if !tree.has_body_at_index(hit.index, hit.body) {
        return;
    }

    // Record the accepted hit before applying streak-scaled muscle recovery data.
    last_hit.record(now);
    apply_strength_drop(
        &mut weights,
        &tree,
        hit.index,
        hit.magnitude,
        last_hit.streak(),
    );
    distribute_impulse(hit.point, hit.impulse, hit.index.get(), &tree, impulses);
}

/// Applies role floors and graph falloff to every affected profile body.
fn apply_strength_drop(
    weights: &mut RagdollBodyWeights,
    tree: &HitImpulseTree,
    target: BodyIndex,
    magnitude: f32,
    streak: u32,
) {
    // Normalize severity and rapid-fire stacking before traversing body slots.
    let severity = (magnitude / REFERENCE_IMPULSE).clamp(0.0, 1.0);
    let drop_streak = 1.0 + STREAK_DROP_MULTIPLIER * streak as f32;
    let hops_limit = maximum_hops(magnitude);

    // Visit only slots that contain a checked body index.
    for (position, body_index) in tree.indexes.iter().enumerate() {
        let Some(body_index) = body_index else {
            continue;
        };
        let Some((hops, falloff)) = hop_falloff(position, target.get(), tree) else {
            continue;
        };
        if hops > hops_limit {
            continue;
        }

        // Apply the role floor after the body-specific hit falloff.
        let role = tree
            .roles
            .get(position)
            .copied()
            .flatten()
            .unwrap_or_default();
        let current = weights.get(position).unwrap_or_default();
        let base = weights.base(position);
        let drop = MAXIMUM_STRENGTH_DROP * severity * drop_streak * falloff;
        let muscle = role
            .muscle_floor()
            .max(current.muscle().min(base.muscle() * (1.0 - drop)));
        // Reduce pin only at the directly addressed body position.
        let pin = if *body_index == target {
            current.pin() * (1.0 - PIN_DROP_PER_SEVERITY * severity)
        } else {
            current.pin()
        };

        // Store current values while preserving the authored recovery baseline.
        weights.set_current(*body_index, BodyWeights::new(muscle, pin));
    }
}

/// Remaining impulse and world-space application point while walking to the root.
struct PendingImpulse {
    /// Current body position in profile order.
    current: usize,
    /// Impulse vector not yet delivered to a body.
    remaining: bevy::math::Vec3,
    /// World-space point where the current body's share is applied.
    point: bevy::math::Vec3,
}

/// One bounded transfer result produced while walking the parent chain.
enum ImpulseStep {
    /// Stop because a required indexed body or finite impulse is unavailable.
    Stop,
    /// Deliver one body share and continue to the next parent.
    Continue(RagdollImpulse),
    /// Deliver one body share and finish the transfer.
    Finish(RagdollImpulse),
    /// Deliver one body share and the remaining impulse to the root.
    FinishAtRoot(RagdollImpulse, RagdollImpulse),
}

/// Splits an impulse at 3 m/s per non-root body and passes the remainder to
/// each parent joint anchor, with the root receiving the final remainder.
fn distribute_impulse(
    hit_point: bevy::math::Vec3,
    impulse: bevy::math::Vec3,
    target: usize,
    tree: &HitImpulseTree,
    impulses: &mut bevy::prelude::MessageWriter<'_, RagdollImpulse>,
) {
    // Keep traversal bounded by the profile's fixed maximum body count.
    let mut pending = PendingImpulse {
        current: target,
        remaining: impulse,
        point: hit_point,
    };
    for _ in 0..tree.body_count {
        // Each step either emits a body share, finishes, or reports missing data.
        match next_impulse_step(tree, &mut pending) {
            ImpulseStep::Stop => break,
            ImpulseStep::Continue(body_share) => {
                impulses.write(body_share);
            }
            ImpulseStep::Finish(body_share) => {
                impulses.write(body_share);
                break;
            }
            ImpulseStep::FinishAtRoot(body_share, root_share) => {
                impulses.write(body_share);
                impulses.write(root_share);
                break;
            }
        }
    }
}

/// Creates the next root or non-root impulse transfer without indexing arrays.
fn next_impulse_step(tree: &HitImpulseTree, pending: &mut PendingImpulse) -> ImpulseStep {
    // Stop when reflection or backend changes leave no body at the current slot.
    let Some(body) = tree.body(pending.current) else {
        return ImpulseStep::Stop;
    };

    // A root receives every remaining unit of impulse at its current centre.
    let Some(parent) = tree.parent(pending.current) else {
        return ImpulseStep::Finish(RagdollImpulse {
            body,
            point: tree
                .poses
                .get(pending.current)
                .copied()
                .flatten()
                .map_or(pending.point, |pose| pose.translation.into()),
            impulse: pending.remaining,
        });
    };

    // Pass non-root bodies through the mass-limited transfer calculation.
    child_impulse_step(tree, pending, body, parent)
}

/// Limits one non-root body share and advances the remainder toward its parent.
fn child_impulse_step(
    tree: &HitImpulseTree,
    pending: &mut PendingImpulse,
    body: bevy::prelude::Entity,
    parent: usize,
) -> ImpulseStep {
    // Reject a zero or invalid remainder before dividing by its magnitude.
    let magnitude = pending.remaining.length();
    if !magnitude.is_finite() || magnitude == 0.0 {
        return ImpulseStep::Stop;
    }

    // Clamp the body share using the 3 m/s cap represented as mass times speed.
    let maximum_impulse = tree.masses.get(pending.current).copied().unwrap_or(0.0) * 3.0;
    let delivered = if magnitude > maximum_impulse {
        pending.remaining * (maximum_impulse / magnitude)
    } else {
        pending.remaining
    };
    let body_share = RagdollImpulse {
        body,
        point: pending.point,
        impulse: delivered,
    };
    pending.remaining -= delivered;

    // Finish after this share when no impulse remains or parent data is incomplete.
    if pending.remaining == bevy::math::Vec3::ZERO {
        return ImpulseStep::Finish(body_share);
    }
    let parent_data = tree
        .body(parent)
        .zip(tree.poses.get(parent).copied().flatten())
        .zip(tree.joint_frames.get(pending.current).copied().flatten());
    let Some(((parent_body, parent_pose), frame)) = parent_data else {
        return ImpulseStep::Finish(body_share);
    };

    // Move the remaining impulse to the parent joint frame before the next step.
    pending.point = (parent_pose * frame).translation.into();
    pending.current = parent;
    if tree.parent(parent).is_none() {
        let root_share = RagdollImpulse {
            body: parent_body,
            point: parent_pose.translation.into(),
            impulse: pending.remaining,
        };
        return ImpulseStep::FinishAtRoot(body_share, root_share);
    }
    ImpulseStep::Continue(body_share)
}

/// Returns hop count and direction-specific falloff when `candidate` is an
/// ancestor or descendant of `target` within the profile's bounded tree.
fn hop_falloff(candidate: usize, target: usize, tree: &HitImpulseTree) -> Option<(usize, f32)> {
    // Keep the addressed body at full influence without traversing its parents.
    if candidate == target {
        return Some((0, 1.0));
    }

    // Search upward from the target for candidates that are ancestors.
    let ancestor = tree.parent(target);
    for (hops, position) in (1..=MAX_BODIES).zip(std::iter::successors(ancestor, |position| {
        tree.parent(*position)
    })) {
        if position == candidate {
            return Some((hops, 0.5_f32.powi(hops as i32)));
        }
    }

    // Search upward from each candidate for descendants of the target.
    let first_parent = tree.parent(candidate);
    for (hops, parent) in (1..=MAX_BODIES).zip(std::iter::successors(first_parent, |parent| {
        tree.parent(*parent)
    })) {
        if parent == target {
            return Some((hops, 0.7_f32.powi(hops as i32)));
        }
    }
    // Return no falloff for siblings, unrelated branches, or malformed cycles.
    None
}

/// Returns the local graph radius selected by the validated impulse magnitude.
fn maximum_hops(magnitude: f32) -> usize {
    if magnitude <= 12.0 {
        1
    } else if magnitude <= 40.0 {
        2
    } else {
        3
    }
}

/// Restores hit-reduced body strengths after the core and role recovery delays.
pub(crate) fn recover_strengths(
    mut characters: bevy::prelude::Query<
        '_,
        '_,
        (
            bevy::prelude::Entity,
            &RagdollMode,
            &mut RagdollBodyWeights,
            &LastHit,
            &RagdollBodies,
        ),
    >,
    bodies: bevy::prelude::Query<'_, '_, (&BodyIndex, &BodyRole, &RagdollBodyOf)>,
    fixed_time: bevy::prelude::Res<'_, bevy::time::Time<bevy::time::Fixed>>,
) {
    // Use one fixed-clock snapshot so every character observes the same recovery instant.
    let now = fixed_time.elapsed();
    let dt = fixed_time.delta_secs();
    // Recover only characters that remain active in the physics runtime.
    for (character, mode, mut weights, last_hit, related_bodies) in &mut characters {
        if !matches!(*mode, RagdollMode::Kinematic | RagdollMode::Dynamic) {
            continue;
        }
        let Some(last_at) = last_hit.last_at() else {
            continue;
        };
        // Each body uses its role-specific delay after the character's core delay.
        for body in related_bodies.iter() {
            let Ok((index, role, owner)) = bodies.get(body) else {
                continue;
            };
            if owner.0 != character {
                continue;
            }
            let recovery_at = last_at + RECOVERY_DELAY + role.recovery_order_delay();
            if now < recovery_at {
                continue;
            }
            let base = weights.base(index.get());
            let current = weights.get(index.get()).unwrap_or_default();
            let muscle = (current.muscle() + MUSCLE_RECOVERY_RATE * dt).min(base.muscle());
            let pin = (current.pin() + PIN_RECOVERY_RATE * dt).min(base.pin());
            // Clamp recovery to authored strengths and publish it at the checked body index.
            weights.set_current(*index, BodyWeights::new(muscle, pin));
        }
    }
}

#[cfg(test)]
mod tests {
    //! Private hit-policy boundaries not repeated at the public message seam.

    use std::time::Duration;

    use bevy::prelude::{App, Fixed, Time, Update};
    use bevy::reflect::tuple_struct::GetTupleStructField;

    use crate::profile::{BodyIndex, BodyRole, MAX_BODIES};
    use crate::runtime::body::JointToParent;
    use crate::runtime::components::{
        BodyWeights, RagdollBodies, RagdollBodyOf, RagdollBodyWeights, RagdollMode,
    };
    use crate::runtime::messages::{HitKind, RagdollHit, RagdollImpulse};

    use super::{
        HitBodyData, HitImpulseTree, LastHit, distribute_impulse, hop_falloff, maximum_hops,
    };

    /// Selects the local graph radius at each documented impulse boundary.
    #[test]
    fn maximum_hops_changes_at_twelve_and_forty() {
        let above_twelve = f32::from_bits(12.0_f32.to_bits() + 1);
        let above_forty = f32::from_bits(40.0_f32.to_bits() + 1);

        assert_eq!(maximum_hops(12.0), 1);
        assert_eq!(maximum_hops(above_twelve), 2);
        assert_eq!(maximum_hops(40.0), 2);
        assert_eq!(maximum_hops(above_forty), 3);
    }

    /// Keeps ancestor and descendant falloff distinct and sibling falloff absent.
    #[test]
    fn hop_falloff_respects_tree_direction_and_siblings() {
        let mut tree = empty_tree();
        tree.body_count = 4;
        tree.parents[1] = Some(0);
        tree.parents[2] = Some(1);
        tree.parents[3] = Some(1);

        assert_eq!(hop_falloff(2, 2, &tree), Some((0, 1.0)));
        assert_eq!(hop_falloff(0, 2, &tree), Some((2, 0.25)));
        assert_eq!(hop_falloff(3, 1, &tree), Some((1, 0.7)));
        assert_eq!(hop_falloff(3, 2, &tree), None);
    }

    /// Creates an empty bounded tree for private traversal tests.
    fn empty_tree() -> HitImpulseTree {
        HitImpulseTree {
            body_count: 0,
            body_entities: [None; MAX_BODIES],
            indexes: [None; MAX_BODIES],
            parent_entities: [None; MAX_BODIES],
            roles: [None; MAX_BODIES],
            parents: [None; MAX_BODIES],
            joint_frames: [None; MAX_BODIES],
            masses: [0.0; MAX_BODIES],
            poses: [None; MAX_BODIES],
        }
    }

    /// Collects only the body's owner's relationship members in profile order.
    #[test]
    fn hit_tree_collects_related_bodies_without_foreign_owners() {
        let mut world = bevy::prelude::World::new();
        let first_owner = world.spawn(RagdollBodies::default()).id();
        let second_owner = world.spawn(RagdollBodies::default()).id();
        let first_body = world
            .spawn((
                BodyIndex::try_from(0).expect("zero is a valid body index"),
                BodyRole::Pelvis,
                RagdollBodyOf(first_owner),
            ))
            .id();
        let second_body = world
            .spawn((
                BodyIndex::try_from(1).expect("one is a valid body index"),
                BodyRole::Spine,
                RagdollBodyOf(first_owner),
                JointToParent {
                    parent: first_body,
                    frame: bevy::math::Isometry3d::IDENTITY,
                    limits: crate::profile::JointLimits {
                        x: crate::profile::AngleRange { min: 0.0, max: 0.0 },
                        twist: crate::profile::AngleRange { min: 0.0, max: 0.0 },
                        z: crate::profile::AngleRange { min: 0.0, max: 0.0 },
                    },
                    max_torque: 1.0,
                },
            ))
            .id();
        let foreign_body = world
            .spawn((
                BodyIndex::try_from(0).expect("zero is a valid body index"),
                BodyRole::Pelvis,
                RagdollBodyOf(second_owner),
            ))
            .id();
        let related_bodies = world
            .get::<RagdollBodies>(first_owner)
            .expect("the first owner's relationship is populated")
            .clone();
        let mut body_query_state = world.query::<HitBodyData>();
        let body_query = body_query_state.query(&world);

        let tree = HitImpulseTree::collect(first_owner, &related_bodies, &body_query);

        assert_eq!(tree.body_count, 2);
        assert_eq!(tree.body(0), Some(first_body));
        assert_eq!(tree.body(1), Some(second_body));
        assert_ne!(tree.body(0), Some(foreign_body));
    }

    /// Saturates rapid-fire hit counts instead of wrapping after `u32::MAX`.
    #[test]
    fn rapid_fire_streak_saturates_at_its_integer_limit() {
        let mut last_hit = LastHit {
            last_at: Some(Duration::ZERO),
            streak: u32::MAX,
        };

        last_hit.record(Duration::from_millis(1));

        assert_eq!(last_hit.streak(), u32::MAX);
    }

    /// Skips hit state changes when the body owner lacks required character state.
    #[test]
    fn process_hits_skips_a_body_with_missing_character_state() {
        let mut app = App::new();
        app.add_message::<RagdollHit>()
            .add_message::<RagdollImpulse>()
            .insert_resource(Time::<Fixed>::from_hz(60.0))
            .add_systems(Update, super::process_hits);

        let owner = app
            .world_mut()
            .spawn((RagdollBodyWeights::default(), LastHit::default()))
            .id();
        let body = app
            .world_mut()
            .spawn((
                BodyIndex::try_from(0).expect("zero is a valid body index"),
                BodyRole::Spine,
                RagdollBodyOf(owner),
            ))
            .id();
        app.world_mut().write_message(RagdollHit {
            body,
            point: bevy::math::Vec3::ZERO,
            impulse: bevy::math::Vec3::X * 20.0,
            kind: HitKind::Impact,
        });

        app.update();

        assert_eq!(
            app.world()
                .get::<LastHit>(owner)
                .expect("owner retains its hit history")
                .last_at(),
            None
        );
        assert_eq!(
            app.world()
                .get::<RagdollBodyWeights>(owner)
                .expect("owner retains its body weights")
                .as_ref(),
            &[]
        );
        assert!(
            app.world()
                .resource::<bevy::ecs::message::Messages<RagdollImpulse>>()
                .iter_current_update_messages()
                .next()
                .is_none()
        );
    }

    /// Ignores an out-of-range index introduced through mutable reflection.
    #[test]
    fn process_hits_skips_an_out_of_range_reflected_body_index() {
        let mut app = App::new();
        app.add_message::<RagdollHit>()
            .add_message::<RagdollImpulse>()
            .insert_resource(Time::<Fixed>::from_hz(60.0))
            .add_systems(Update, super::process_hits);

        let owner = app
            .world_mut()
            .spawn((
                RagdollMode::Dynamic,
                RagdollBodyWeights::new(vec![BodyWeights::default()]),
                LastHit::default(),
            ))
            .id();
        let body = app
            .world_mut()
            .spawn((
                BodyIndex::try_from(0).expect("zero is a valid body index"),
                BodyRole::Spine,
                RagdollBodyOf(owner),
            ))
            .id();
        {
            let mut index = app
                .world_mut()
                .get_mut::<BodyIndex>(body)
                .expect("the hit body has an index");
            *index
                .get_field_mut::<u8>(0)
                .expect("reflection exposes the tuple field") = u8::MAX;
        }
        app.world_mut().write_message(RagdollHit {
            body,
            point: bevy::math::Vec3::ZERO,
            impulse: bevy::math::Vec3::X * 20.0,
            kind: HitKind::Impact,
        });

        app.update();

        assert_eq!(
            app.world()
                .get::<LastHit>(owner)
                .expect("owner retains its hit history")
                .last_at(),
            None
        );
        assert_eq!(
            app.world()
                .get::<RagdollBodyWeights>(owner)
                .expect("owner retains its body weights")
                .as_ref(),
            &[BodyWeights::default()]
        );
        assert!(
            app.world()
                .resource::<bevy::ecs::message::Messages<RagdollImpulse>>()
                .iter_current_update_messages()
                .next()
                .is_none()
        );
    }

    /// Rejects a hit when another body occupies its validated index.
    #[test]
    fn process_hits_skips_duplicate_body_indexes() {
        let mut app = App::new();
        app.add_message::<RagdollHit>()
            .add_message::<RagdollImpulse>()
            .insert_resource(Time::<Fixed>::from_hz(60.0))
            .add_systems(Update, super::process_hits);

        let owner = app
            .world_mut()
            .spawn((
                RagdollMode::Dynamic,
                RagdollBodyWeights::new(vec![BodyWeights::default()]),
                LastHit::default(),
            ))
            .id();
        let hit_body = app
            .world_mut()
            .spawn((
                BodyIndex::try_from(0).expect("zero is a valid body index"),
                BodyRole::Spine,
                RagdollBodyOf(owner),
            ))
            .id();
        app.world_mut().spawn((
            BodyIndex::try_from(0).expect("zero is a valid body index"),
            BodyRole::Spine,
            RagdollBodyOf(owner),
        ));
        app.world_mut().write_message(RagdollHit {
            body: hit_body,
            point: bevy::math::Vec3::ZERO,
            impulse: bevy::math::Vec3::X * 20.0,
            kind: HitKind::Impact,
        });

        app.update();

        assert_eq!(
            app.world()
                .get::<LastHit>(owner)
                .expect("owner retains its hit history")
                .last_at(),
            None
        );
        assert_eq!(
            app.world()
                .get::<RagdollBodyWeights>(owner)
                .expect("owner retains its body weights")
                .as_ref(),
            &[BodyWeights::default()]
        );
        assert!(
            app.world()
                .resource::<bevy::ecs::message::Messages<RagdollImpulse>>()
                .iter_current_update_messages()
                .next()
                .is_none()
        );
    }

    /// Stops safely when impulse-tree data or the remaining vector is invalid.
    #[test]
    fn distribute_impulse_stops_on_missing_tree_data() {
        let mut app = App::new();
        app.add_message::<RagdollImpulse>().add_systems(
            Update,
            |mut impulses: bevy::ecs::message::MessageWriter<RagdollImpulse>| {
                let mut tree = HitImpulseTree {
                    masses: [1.0; MAX_BODIES],
                    body_count: 1,
                    ..empty_tree()
                };
                tree.parents[0] = Some(1);

                distribute_impulse(
                    bevy::math::Vec3::ZERO,
                    bevy::math::Vec3::X * 10.0,
                    0,
                    &tree,
                    &mut impulses,
                );

                tree.body_entities[0] = Some(bevy::prelude::Entity::PLACEHOLDER);
                distribute_impulse(
                    bevy::math::Vec3::ZERO,
                    bevy::math::Vec3::ZERO,
                    0,
                    &tree,
                    &mut impulses,
                );
                distribute_impulse(
                    bevy::math::Vec3::ZERO,
                    bevy::math::Vec3::splat(f32::INFINITY),
                    0,
                    &tree,
                    &mut impulses,
                );

                tree.body_count = 2;
                distribute_impulse(
                    bevy::math::Vec3::ZERO,
                    bevy::math::Vec3::X * 10.0,
                    0,
                    &tree,
                    &mut impulses,
                );

                tree.body_entities[1] = Some(bevy::prelude::Entity::PLACEHOLDER);
                distribute_impulse(
                    bevy::math::Vec3::ZERO,
                    bevy::math::Vec3::X * 10.0,
                    0,
                    &tree,
                    &mut impulses,
                );

                tree.poses[1] = Some(bevy::math::Isometry3d::IDENTITY);
                distribute_impulse(
                    bevy::math::Vec3::ZERO,
                    bevy::math::Vec3::X * 10.0,
                    0,
                    &tree,
                    &mut impulses,
                );

                tree.joint_frames[0] = Some(bevy::math::Isometry3d::IDENTITY);
                distribute_impulse(
                    bevy::math::Vec3::ZERO,
                    bevy::math::Vec3::X * 10.0,
                    0,
                    &tree,
                    &mut impulses,
                );
            },
        );

        app.update();

        let outputs = app
            .world()
            .resource::<bevy::ecs::message::Messages<RagdollImpulse>>()
            .iter_current_update_messages()
            .collect::<Vec<_>>();
        assert_eq!(
            outputs
                .iter()
                .map(|output| output.impulse)
                .collect::<Vec<_>>(),
            vec![
                bevy::math::Vec3::X * 3.0,
                bevy::math::Vec3::X * 3.0,
                bevy::math::Vec3::X * 3.0,
                bevy::math::Vec3::X * 3.0,
                bevy::math::Vec3::X * 7.0,
            ]
        );
    }
}
