//! Bind validated profiles to skeleton descendants and synchronize physics body
//! entities.
//!
//! Binding walks a character hierarchy breadth-first so every parent precedes
//! its children and profile indexes map to deterministic nodes. Mode changes
//! create or remove top-level body entities, while fixed-step bookkeeping
//! enforces dynamic budgets and limp-settle policy. All world lookups are
//! fallible because entities, assets, and resources can disappear between
//! stages.

use std::collections::{HashMap, VecDeque};

use bevy::asset::Assets;
use bevy::math::Isometry3d;
use bevy::prelude::{Children, Component, Entity, Name, Resource, Transform, World};
use bevy::time::{Fixed, Time};

use crate::profile::{Body, RagdollProfile};

use super::body::{
    BodyAtRest, BodyDriveOutput, BodyKind, BodyMass, BodyPhysicsPose, BodyShape, BodyVelocity,
    JointDriveTarget, JointToParent, NoContactWith,
};
use super::budget::RagdollBudget;
use super::components::{
    Ragdoll, RagdollBodies, RagdollDrive, RagdollId, RagdollMode, RagdollTargetPose,
};
use super::events::{RagdollActivated, RagdollBudgetEvicted, RagdollFrozen, RagdollSettled};
use super::settings::RagdollPhysicsSettings;

/// A skeleton binding or profile activation failure stored on the affected
/// character.
///
/// Missing bones leave the character animated and keep body entities absent,
/// allowing applications to repair the hierarchy or display a useful
/// diagnostic. Identity exhaustion also blocks activation rather than creating
/// duplicate deterministic age values.
#[derive(Component, Clone, Debug, PartialEq, Eq, thiserror::Error, bevy::prelude::Reflect)]
pub enum RagdollError {
    /// A required profile bone name was not found among descendants of the
    /// character entity.
    #[error("ragdoll profile bone `{0}` was not found in the skeleton")]
    /// Activation stores the exact profile name so an application can repair
    /// its skeleton before retrying.
    MissingBone(String),
    /// The monotonic identity counter reached its maximum and cannot assign a
    /// unique character ID.
    #[error("ragdoll identity space is exhausted")]
    /// Activation keeps the character Animated because a wrapped ID would break
    /// oldest-first eviction order.
    IdentityExhausted,
    /// The skeleton could not produce a valid generated profile.
    #[error("ragdoll profile generation failed: {0}")]
    InvalidSkeleton(String),
}

/// Next monotonic ragdoll identity used for deterministic age ordering.
#[derive(Resource, Default, bevy::prelude::Reflect)]
pub(crate) struct RagdollIdCounter {
    /// Identity assigned to the next activated character.
    next: u64,
}

/// Duration and event state for limp ragdoll settling.
#[derive(Component, Clone, Copy, Debug, Default, bevy::prelude::Reflect)]
struct SettleProgress {
    /// Seconds for which every body has stayed below the settle speed.
    seconds: f32,
    /// Whether this ragdoll has already emitted its settle event.
    emitted: bool,
}

/// Parent-first skeleton nodes captured when the profile binds.
#[derive(Component, Clone, Debug, Default, bevy::prelude::Reflect)]
pub(crate) struct SkeletonMap {
    /// Skeleton nodes in breadth-first parent-first order.
    pub(crate) bones: Vec<SkeletonBone>,
    /// Profile-body index to skeleton node index.
    pub(crate) body_to_bone: Vec<usize>,
}

/// One skeleton node and its optional matching profile body.
#[derive(Clone, Debug, bevy::prelude::Reflect)]
pub(crate) struct SkeletonBone {
    /// Skeleton entity for this node.
    pub(crate) entity: Entity,
    /// Parent node index, absent for the character root.
    pub(crate) parent: Option<usize>,
    /// Profile body position when this bone has collision geometry.
    pub(crate) body: Option<usize>,
    /// Authored local rest transform for this bone.
    pub(crate) rest_local: Transform,
}

/// Resolves ready profiles, builds each skeleton map, and synchronizes body
/// entities.
///
/// Binding walks each skeleton once in O(nodes) time and stores O(nodes) map
/// state. Mode synchronization visits only that ragdoll's profile bodies, and
/// missing world state is skipped rather than unwrapped so despawned or
/// partially initialized characters remain safe.
pub(crate) fn bind_and_sync(world: &mut World) {
    // Snapshot characters because binding and mode changes mutate components during this system.
    let characters = {
        let mut query = world.query_filtered::<Entity, bevy::prelude::With<Ragdoll>>();
        query.iter(world).collect::<Vec<_>>()
    };

    for character in characters {
        synchronize_character(world, character);
    }

    // Enforce the limit after every mode transition so activation order stays deterministic.
    enforce_budget(world);
}

/// Binds one loaded profile and makes its requested body mode match the
/// character components.
fn synchronize_character(world: &mut World, character: Entity) {
    // Preserve the first binding failure until the application changes or removes it.
    if world.get::<RagdollError>(character).is_some() {
        return;
    }
    let Some(profile_handle) = resolve_profile(world, character) else {
        return;
    };
    // Read mode and existing relationships before borrowing an asset for possible construction.
    let mode = world
        .get::<RagdollMode>(character)
        .copied()
        .unwrap_or_default();
    let is_bound = world.get::<SkeletonMap>(character).is_some();
    let has_bodies = world
        .get::<RagdollBodies>(character)
        .is_some_and(|bodies| !bodies.is_empty());

    // Load a profile only when binding or creating a body set needs authored data.
    let needs_profile = !is_bound || (mode != RagdollMode::Animated && !has_bodies);
    let profile = loaded_profile(world, &profile_handle, needs_profile);

    // Bind animated characters as soon as their profile asset becomes available.
    if ensure_character_binding(world, character, is_bound, profile.as_ref()).is_none() {
        return;
    }
    synchronize_body_mode(world, character, mode, has_bodies, profile.as_ref());
}

/// Returns the character's profile, generating it from the skeleton first
/// when the [`Ragdoll`] names none.
///
/// Generation waits until the character has descendants and its overrides
/// asset, if any, has loaded. A skeleton that yields no valid profile stores
/// [`RagdollError::InvalidSkeleton`].
fn resolve_profile(
    world: &mut World,
    character: Entity,
) -> Option<bevy::asset::Handle<RagdollProfile>> {
    let ragdoll = world.get::<Ragdoll>(character)?;
    if let Some(profile) = &ragdoll.profile {
        return Some(profile.clone());
    }
    let mass = ragdoll.mass;
    let overrides = match &ragdoll.overrides {
        Some(handle) => Some(
            world
                .get_resource::<Assets<crate::auto::RagdollOverrides>>()?
                .get(handle)?
                .clone(),
        ),
        None => None,
    };
    let mut skeleton = crate::auto::skeleton_from_world(world, character, overrides.as_ref())?;
    skeleton.mass = mass.or_else(|| {
        overrides
            .and_then(|overrides| overrides.mass)
            .and_then(|kilograms| crate::profile::Mass::try_from(kilograms).ok())
    });
    let profile = match RagdollProfile::from_skeleton(&skeleton) {
        Ok(profile) => profile,
        Err(error) => {
            bevy::log::warn!(%error, "ragdoll profile generation failed");
            if let Ok(mut entity) = world.get_entity_mut(character) {
                entity.insert(RagdollError::InvalidSkeleton(error.to_string()));
            }
            return None;
        }
    };
    let handle = world
        .get_resource_mut::<Assets<RagdollProfile>>()?
        .add(profile);
    world.get_mut::<Ragdoll>(character)?.profile = Some(handle.clone());
    Some(handle)
}

/// Ensures the profile is bound before the requested mode can create body entities.
fn ensure_character_binding(
    world: &mut World,
    character: Entity,
    is_bound: bool,
    profile: Option<&RagdollProfile>,
) -> Option<()> {
    if is_bound {
        return Some(());
    }
    let profile = profile?;
    bind_character(world, character, profile)
}

/// Creates, updates, or removes physics bodies to match the requested character mode.
fn synchronize_body_mode(
    world: &mut World,
    character: Entity,
    mode: RagdollMode,
    has_bodies: bool,
    profile: Option<&RagdollProfile>,
) {
    // Animated mode owns no physics bodies, while retaining its successful skeleton binding.
    if mode == RagdollMode::Animated {
        despawn_bodies(world, character);
        return;
    }
    // Assign stable age before activation so budget eviction can sort deterministically.
    if assign_ragdoll_id(world, character).is_none() {
        return;
    }
    // Existing bodies only need their backend motion state updated on a mode transition.
    if has_bodies {
        update_body_kinds(world, character, mode);
    } else if let Some(profile) = profile {
        spawn_bodies(world, character, profile, mode);
    }
}

/// Returns a cloned profile only when the current mode needs profile
/// construction data.
fn loaded_profile(
    world: &World,
    handle: &bevy::asset::Handle<RagdollProfile>,
    should_load: bool,
) -> Option<RagdollProfile> {
    // Avoid cloning loaded profile data on unchanged animated and already-active characters.
    if !should_load {
        return None;
    }
    world
        .get_resource::<Assets<RagdollProfile>>()
        .and_then(|assets| assets.get(handle))
        .cloned()
}

/// Builds and installs a skeleton map, then initializes its target components.
fn bind_character(world: &mut World, character: Entity, profile: &RagdollProfile) -> Option<()> {
    // Reject missing bones before attaching partial binding state to the character.
    let map = build_skeleton_map(world, character, profile)?;
    if let Ok(mut entity) = world.get_entity_mut(character) {
        entity.insert(map);
    } else {
        return None;
    }
    // Install capture defaults only after the complete parent-first map is attached.
    initialize_target_components(world, character, profile.bodies().len());
    Some(())
}

/// Initializes target history and body overrides after skeleton mapping
/// succeeds.
fn initialize_target_components(world: &mut World, character: Entity, body_count: usize) {
    // Capture an initial pose so a first fixed step never reads an empty target history.
    if world.get::<RagdollTargetPose>(character).is_none() {
        let Some(map) = world.get::<SkeletonMap>(character).cloned() else {
            return;
        };
        let target_poses = super::capture::capture_poses(world, &map);
        let mut targets = RagdollTargetPose::default();
        targets.record(
            target_poses.clone(),
            vec![BodyVelocity::default(); target_poses.len()],
        );
        if let Ok(mut entity) = world.get_entity_mut(character) {
            entity.insert(targets);
        }
    }

    // All successful bindings capture animation unless the character loses its marker with despawn.
    if let Ok(mut entity) = world.get_entity_mut(character) {
        entity.insert(super::capture::AutoCapture);
    }

    // Extend the required component in place without moving the character to another table.
    if let Some(mut weights) = world.get_mut::<super::components::RagdollBodyWeights>(character) {
        weights.initialize_profile_bodies(body_count);
    }
}

/// Assigns one stable identity when a character first activates.
fn assign_ragdoll_id(world: &mut World, character: Entity) -> Option<()> {
    // Preserve the original identity across later mode changes and body recreation.
    if world.get::<RagdollId>(character).is_some() {
        return Some(());
    }
    // Reserve a value only while the counter exists and can advance without wrapping.
    let id = {
        let mut counter = world.get_resource_mut::<RagdollIdCounter>()?;
        counter.next.checked_add(1).map(|next| {
            let id = RagdollId::new(counter.next);
            counter.next = next;
            id
        })
    };
    let Some(id) = id else {
        // Prevent ambiguous activation when no unique identity remains.
        if let Ok(mut entity) = world.get_entity_mut(character) {
            entity.insert((RagdollError::IdentityExhausted, RagdollMode::Animated));
        }
        return None;
    };
    // A character may have despawned since the binding query snapshot was made.
    if let Ok(mut entity) = world.get_entity_mut(character) {
        entity.insert(id);
        Some(())
    } else {
        None
    }
}

/// Freezes the oldest dynamic ragdolls until the configured budget is met.
fn enforce_budget(world: &mut World) {
    // A removed budget resource disables eviction without terminating the runtime schedule.
    let Some(budget) = world.get_resource::<RagdollBudget>().copied() else {
        return;
    };
    if budget.max_dynamic == usize::MAX {
        return;
    }
    let mut dynamic = {
        let mut query =
            world.query_filtered::<(Entity, &RagdollId), bevy::prelude::With<Ragdoll>>();
        query
            .iter(world)
            .filter(|(entity, _)| world.get::<RagdollMode>(*entity) == Some(&RagdollMode::Dynamic))
            .map(|(entity, id)| (entity, id.get()))
            .collect::<Vec<_>>()
    };
    // Stable IDs preserve activation order when the same set exceeds the budget again.
    dynamic.sort_unstable_by_key(|(_, id)| *id);

    // Freeze only the excess prefix, which contains the oldest active identities.
    let evictions = dynamic.len().saturating_sub(budget.max_dynamic);
    for (character, _) in dynamic.into_iter().take(evictions) {
        if let Some(mut mode) = world.get_mut::<RagdollMode>(character) {
            *mode = RagdollMode::Frozen;
        }
        update_body_kinds(world, character, RagdollMode::Frozen);
        // Trigger after mode and body kind agree so observers see completed eviction state.
        world.trigger(RagdollBudgetEvicted { entity: character });
    }
}

/// Tracks limp low-speed time and emits one settle event per active ragdoll.
pub(crate) fn update_settle_state(world: &mut World) {
    // Missing clock or settings resources make this fixed-step evaluation inapplicable.
    let Some(delta_seconds) = world.get_resource::<Time<Fixed>>().map(Time::delta_secs) else {
        return;
    };
    let Some(settings) = world.get_resource::<RagdollPhysicsSettings>().copied() else {
        return;
    };
    // Snapshot entities before mode, progress, and observer state are changed.
    let characters = {
        let mut query = world.query_filtered::<Entity, bevy::prelude::With<Ragdoll>>();
        query.iter(world).collect::<Vec<_>>()
    };

    for character in characters {
        update_character_settle(world, character, settings, delta_seconds);
    }
}

/// Updates one character's low-speed duration and applies the configured settle
/// transition.
fn update_character_settle(
    world: &mut World,
    character: Entity,
    settings: RagdollPhysicsSettings,
    delta_seconds: f32,
) {
    // Only zero-muscle dynamic characters participate in limp settling.
    let is_limp_dynamic = world.get::<RagdollMode>(character) == Some(&RagdollMode::Dynamic)
        && world
            .get::<RagdollDrive>(character)
            .is_some_and(|drive| drive.muscle() == 0.0);
    if !is_limp_dynamic {
        if let Ok(mut entity) = world.get_entity_mut(character) {
            entity.remove::<SettleProgress>();
        }
        return;
    }

    // Every related body's linear speed must be below the threshold for the full interval.
    let all_slow = all_body_linear_speeds_below(world, character, settings.settle_speed);
    let progress = world
        .get::<SettleProgress>(character)
        .copied()
        .unwrap_or_default();
    let mut progress = advance_settle_progress(progress, all_slow, delta_seconds);

    // Emit once at the interval boundary, then optionally freeze every body.
    if !progress.emitted && progress.seconds >= settings.settle_after {
        progress.emitted = true;
        complete_settle_transition(world, character, settings.should_freeze_when_settled);
    }

    // Store progress only if the observer did not despawn the character.
    if let Ok(mut entity) = world.get_entity_mut(character) {
        entity.insert(progress);
    }
}

/// Adds valid low-speed time or resets progress when any body exceeds the
/// settle threshold.
fn advance_settle_progress(
    mut progress: SettleProgress,
    all_slow: bool,
    delta_seconds: f32,
) -> SettleProgress {
    // Invalid frame durations do not add time; motion above threshold resets accumulated time.
    if all_slow && delta_seconds.is_finite() && delta_seconds > 0.0 {
        progress.seconds += delta_seconds;
    } else if !all_slow {
        progress.seconds = 0.0;
    }
    progress
}

/// Triggers the settle event and optionally changes the character's body kinds
/// to fixed.
fn complete_settle_transition(world: &mut World, character: Entity, should_freeze: bool) {
    // Notify observers before changing body kinds so they see the completed settle transition.
    world.trigger(RagdollSettled { entity: character });
    if should_freeze {
        if let Some(mut mode) = world.get_mut::<RagdollMode>(character) {
            *mode = RagdollMode::Frozen;
        }
        update_body_kinds(world, character, RagdollMode::Frozen);
    }
}

/// Reports whether a character has at least one body and every body is below a
/// linear-speed limit.
fn all_body_linear_speeds_below(world: &World, character: Entity, threshold: f32) -> bool {
    let Some(bodies) = world.get::<RagdollBodies>(character) else {
        return false;
    };
    let mut has_body = false;
    // Require a complete body set so absent physics data cannot imply that settling succeeded.
    for body in bodies.iter() {
        // Missing backend velocity means this body has not established a settled state.
        has_body = true;
        let body_is_slow = world
            .get::<BodyVelocity>(body)
            .is_some_and(|velocity| velocity.linear.length() < threshold);
        if !body_is_slow {
            return false;
        }
    }
    has_body
}

/// Builds the breadth-first map and rejects the first profile name that is
/// absent.
fn build_skeleton_map(
    world: &mut World,
    character: Entity,
    profile: &RagdollProfile,
) -> Option<SkeletonMap> {
    // Resolve profile names once so each visited skeleton node needs one hash lookup.
    let body_by_name = profile
        .bodies()
        .iter()
        .enumerate()
        .map(|(index, body)| (body.bone().to_owned(), index))
        .collect::<HashMap<_, _>>();
    let (bones, body_to_bone) = walk_skeleton(world, character, &body_by_name)?;

    // Report the first missing body in validated profile order and keep the character animated.
    if let Some(missing) = body_to_bone.iter().position(|index| *index == usize::MAX) {
        let name = profile.bodies().get(missing)?.bone().to_owned();
        bevy::log::warn!(
            missing_bone = %name,
            "ragdoll activation failed: skeleton bone is missing"
        );
        if let Ok(mut entity) = world.get_entity_mut(character) {
            entity.insert((RagdollError::MissingBone(name), RagdollMode::Animated));
        }
        return None;
    }

    Some(SkeletonMap {
        bones,
        body_to_bone,
    })
}

/// Records one visited node and retains its earliest profile-body match.
fn record_skeleton_node(
    world: &World,
    entity: Entity,
    parent: Option<usize>,
    body_by_name: &HashMap<String, usize>,
    bones: &mut Vec<SkeletonBone>,
    body_to_bone: &mut [usize],
) -> usize {
    let index = bones.len();
    // Preserve local rest transforms for nodes without an explicit transform.
    let rest_local = world.get::<Transform>(entity).copied().unwrap_or_default();
    // Match exact names and retain the earliest breadth-first body occurrence.
    let body = world
        .get::<Name>(entity)
        .and_then(|name| body_by_name.get(name.as_str()).copied());
    if let Some(slot) = body.and_then(|body_index| body_to_bone.get_mut(body_index)) {
        *slot = (*slot).min(index);
    }
    bones.push(SkeletonBone {
        entity,
        parent,
        body,
        rest_local,
    });
    index
}

/// Visits skeleton nodes breadth-first and records each profile body's first match.
fn walk_skeleton(
    world: &World,
    character: Entity,
    body_by_name: &HashMap<String, usize>,
) -> Option<(Vec<SkeletonBone>, Vec<usize>)> {
    // Queue children in their stored order so each parent precedes its descendants.
    let children = world.get::<Children>(character)?;
    let mut queue = children
        .iter()
        .copied()
        .map(|entity| (entity, None))
        .collect::<VecDeque<_>>();
    if queue.is_empty() {
        return None;
    }

    // Store one mapping slot per validated profile body and preserve first matching names.
    let mut bones = Vec::new();
    let mut body_to_bone = vec![usize::MAX; body_by_name.len()];
    // Process queued parents before their children so stored indexes remain parent-first.
    while let Some((entity, parent)) = queue.pop_front() {
        let index = record_skeleton_node(
            world,
            entity,
            parent,
            body_by_name,
            &mut bones,
            &mut body_to_bone,
        );
        if let Some(children) = world.get::<Children>(entity) {
            queue.extend(children.iter().copied().map(|child| (child, Some(index))));
        }
    }

    Some((bones, body_to_bone))
}

/// Values shared by every profile body created during one activation transition.
struct BodySpawnContext<'a> {
    /// Character entity linked from each newly created physics body.
    character: Entity,
    /// Physics values copied into body components during spawn.
    settings: RagdollPhysicsSettings,
    /// Character world pose used to convert target poses into world coordinates.
    character_pose: Isometry3d,
    /// Captured target poses in validated profile body order.
    target_poses: &'a [Isometry3d],
    /// Captured target velocities in validated profile body order.
    target_velocities: &'a [BodyVelocity],
    /// Backend motion state selected from the requested character mode.
    kind: BodyKind,
}

/// Reads one activation's target poses and target velocities in profile order.
fn initial_body_targets(
    world: &World,
    character: Entity,
    profile: &RagdollProfile,
) -> (Vec<Isometry3d>, Vec<BodyVelocity>) {
    // Use captured poses when available and validated rest poses before the first capture.
    let targets = world.get::<RagdollTargetPose>(character);
    let poses = targets
        .map(|target| target.current().to_vec())
        .unwrap_or_else(|| profile.bodies().iter().map(|body| body.rest()).collect());
    // Missing target history has no measured motion, so new bodies begin without velocity.
    let velocities = targets
        .map(|target| target.velocities().to_vec())
        .unwrap_or_default();
    (poses, velocities)
}

/// Creates top-level physics body entities in parent-first profile order.
fn spawn_bodies(world: &mut World, character: Entity, profile: &RagdollProfile, mode: RagdollMode) {
    // A missing settings resource prevents construction because each body needs its inertia floor.
    let Some(settings) = world.get_resource::<RagdollPhysicsSettings>().copied() else {
        return;
    };
    // Capture the character pose once so every profile-local target uses the same world frame.
    let character_pose = super::writeback::world_pose(world, character);
    let (target_poses, target_velocities) = initial_body_targets(world, character, profile);
    // Share the common world frame and captured arrays across every spawned profile body.
    let context = BodySpawnContext {
        character,
        settings,
        character_pose,
        target_poses: &target_poses,
        target_velocities: &target_velocities,
        kind: body_kind(mode),
    };
    // Create all bodies before attaching joints so profile indexes resolve to entity IDs.
    let entities = spawn_profile_bodies(world, profile, &context);

    // Attach constraints only after every parent and child entity has been created.
    for joint in profile.joints() {
        let (Some(child), Some(parent)) = (
            entities.get(joint.child().get()).copied(),
            entities.get(joint.parent().get()).copied(),
        ) else {
            continue;
        };
        if let Ok(mut entity) = world.get_entity_mut(child) {
            entity.insert((
                JointToParent {
                    parent,
                    frame: joint.frame(),
                    limits: joint.limits(),
                    max_torque: joint.max_torque(),
                },
                JointDriveTarget::default(),
            ));
            if joint.basis() != bevy::math::Quat::IDENTITY {
                entity.insert(super::body::JointBasis(joint.basis()));
            }
        }
    }

    // Notify observers only after bodies, relationships, and joints have been installed.
    world.trigger(RagdollActivated { entity: character });
}

/// Spawns one physics entity for each validated profile body.
fn spawn_profile_bodies(
    world: &mut World,
    profile: &RagdollProfile,
    context: &BodySpawnContext<'_>,
) -> Vec<Entity> {
    // Reserve only the bounded profile body count, which cannot exceed sixty-four entries.
    let mut entities = Vec::with_capacity(profile.bodies().len());

    for (index, body) in profile.bodies().iter().enumerate() {
        // Preserve the profile's symmetric exclusion mask for every body collider.
        let contact_mask = profile
            .no_contact_masks()
            .get(index)
            .copied()
            .unwrap_or_default();
        // Insert the complete shared body contract before any backend queries can observe it.
        entities.push(
            world
                .spawn(body_spawn_bundle(body, index, contact_mask, context))
                .id(),
        );
    }

    entities
}

/// Builds the complete backend-neutral component bundle for one profile body.
fn body_spawn_bundle(
    body: &Body,
    index: usize,
    contact_mask: u64,
    context: &BodySpawnContext<'_>,
) -> impl bevy::ecs::bundle::Bundle {
    // Use the indexed animated target when present and the validated rest pose otherwise.
    let target_pose = context
        .target_poses
        .get(index)
        .copied()
        .unwrap_or_else(|| body.rest());
    let pose = context.character_pose * target_pose;
    // Rotate captured skeleton-space velocity into the character's world frame.
    let target_velocity = context
        .target_velocities
        .get(index)
        .copied()
        .unwrap_or_default();
    let velocity = BodyVelocity {
        linear: context.character_pose.rotation * target_velocity.linear,
        angular: context.character_pose.rotation * target_velocity.angular,
    };
    (
        body.index(),
        body.role(),
        super::components::RagdollBodyOf(context.character),
        BodyShape(*body.shape()),
        BodyMass {
            mass: body.mass().kilograms(),
            min_inertia_radius: context.settings.min_inertia_radius,
        },
        context.kind,
        velocity,
        BodyPhysicsPose {
            previous: pose,
            current: pose,
        },
        BodyDriveOutput::default(),
        NoContactWith(contact_mask),
        Transform::from_translation(pose.translation.into()).with_rotation(pose.rotation),
    )
}

/// Removes body entities and their relationship target from a character.
fn despawn_bodies(world: &mut World, character: Entity) {
    let bodies = world
        .get::<RagdollBodies>(character)
        .map(|bodies| bodies.iter().collect::<Vec<_>>())
        .unwrap_or_default();
    for body in bodies {
        world.despawn(body);
    }
}

/// Applies the character's mode to its existing body entities.
fn update_body_kinds(world: &mut World, character: Entity, mode: RagdollMode) {
    // Resolve one backend state so every related body receives the same transition.
    let kind = body_kind(mode);
    let bodies = world
        .get::<RagdollBodies>(character)
        .map(|bodies| bodies.iter().collect::<Vec<_>>())
        .unwrap_or_default();
    // Emit a freeze event only when at least one body changes from a movable kind.
    let has_body_to_freeze = kind == BodyKind::Fixed
        && bodies.iter().any(|body| {
            world
                .get::<BodyKind>(*body)
                .is_some_and(|current| *current != BodyKind::Fixed)
        });
    for body in bodies {
        // Update backend motion state before observers receive the character event.
        if let Some(mut body_kind) = world.get_mut::<BodyKind>(body) {
            *body_kind = kind;
        }
        // Clear stale sleep state whenever dynamic simulation is not the active mode.
        if mode != RagdollMode::Dynamic
            && let Ok(mut entity) = world.get_entity_mut(body)
        {
            entity.remove::<BodyAtRest>();
        }
    }
    // Notify only after every body's backend state matches the character mode.
    if has_body_to_freeze {
        world.trigger(RagdollFrozen { entity: character });
    }
}

/// Maps character mode to the backend's body kind.
const fn body_kind(mode: RagdollMode) -> BodyKind {
    match mode {
        RagdollMode::Animated => BodyKind::Fixed,
        RagdollMode::Kinematic => BodyKind::Kinematic,
        RagdollMode::Dynamic => BodyKind::Dynamic,
        RagdollMode::Frozen => BodyKind::Fixed,
    }
}

#[cfg(test)]
mod tests {
    //! Checks every conversion from character mode to backend body kind.

    use std::collections::HashMap;

    use bevy::math::{Isometry3d, Vec3};
    use bevy::prelude::{Children, Entity, Transform, World};
    use bevy::time::{Fixed, Time};

    use crate::profile::{ProfileBuilder, RagdollProfile, ShapeSpec};
    use crate::runtime::body::{BodyKind, BodyShape};
    use crate::runtime::capture::AutoCapture;
    use crate::runtime::components::{RagdollBodies, RagdollBodyWeights, RagdollTargetPose};
    use crate::runtime::components::{RagdollId, RagdollMode};
    use crate::runtime::skeleton::{RagdollError, RagdollIdCounter};

    use super::{
        SkeletonBone, SkeletonMap, all_body_linear_speeds_below, assign_ragdoll_id, body_kind,
        enforce_budget, initialize_target_components, spawn_bodies, synchronize_body_mode,
        synchronize_character, update_settle_state, walk_skeleton,
    };

    /// Builds a valid one-body profile for early-spawn checks.
    fn single_body_profile() -> RagdollProfile {
        let mut builder = ProfileBuilder::default();
        builder
            .add_body(
                "root",
                ShapeSpec::Sphere {
                    center: Vec3::ZERO,
                    radius: 0.1,
                },
                1.0,
                Isometry3d::IDENTITY,
            )
            .expect("the single body index fits the profile");
        builder
            .build()
            .expect("a single root body is a valid profile")
    }

    /// Synchronization skips a character that disappeared from its query snapshot.
    #[test]
    fn synchronization_skips_a_missing_character() {
        synchronize_character(&mut World::new(), Entity::PLACEHOLDER);
    }

    /// Body activation stays animated when no unique identity remains.
    #[test]
    fn body_mode_skips_activation_after_identity_exhaustion() {
        let mut world = World::new();
        world.insert_resource(RagdollIdCounter { next: u64::MAX });
        let character = world.spawn(RagdollMode::Dynamic).id();

        synchronize_body_mode(&mut world, character, RagdollMode::Dynamic, false, None);

        assert_eq!(
            world.get::<RagdollError>(character),
            Some(&RagdollError::IdentityExhausted)
        );
        assert_eq!(
            world.get::<RagdollMode>(character),
            Some(&RagdollMode::Animated)
        );
        assert!(world.get::<RagdollBodies>(character).is_none());
    }

    /// Target initialization does nothing until binding installs its skeleton map.
    #[test]
    fn target_initialization_skips_a_missing_skeleton_map() {
        let mut world = World::new();
        let character = world.spawn_empty().id();

        initialize_target_components(&mut world, character, 1);

        assert!(world.get::<RagdollTargetPose>(character).is_none());
    }

    /// Binding restores a removed target component from the mapped skeleton pose.
    #[test]
    fn target_initialization_captures_initial_pose() {
        let mut world = World::new();
        let bone_pose = Transform::from_xyz(1.0, 2.0, 3.0);
        let bone = world.spawn(bone_pose).id();
        let character = world
            .spawn((
                SkeletonMap {
                    bones: vec![SkeletonBone {
                        entity: bone,
                        parent: None,
                        body: Some(0),
                        rest_local: Transform::IDENTITY,
                    }],
                    body_to_bone: vec![0],
                },
                RagdollBodyWeights::default(),
            ))
            .id();

        initialize_target_components(&mut world, character, 1);

        let body_index = crate::profile::BodyIndex::try_from(0)
            .expect("zero is a valid index for the mapped skeleton body");
        assert_eq!(
            world
                .get::<RagdollTargetPose>(character)
                .and_then(|targets| targets.current_pose(body_index)),
            Some(Isometry3d::from_translation(Vec3::new(1.0, 2.0, 3.0)))
        );
        assert!(world.get::<AutoCapture>(character).is_some());
        assert!(world.get::<RagdollBodyWeights>(character).is_some());
    }

    /// Binding keeps a caller-supplied target pose instead of recapturing it.
    #[test]
    fn target_initialization_preserves_existing_pose() {
        let mut world = World::new();
        let bone = world.spawn(Transform::IDENTITY).id();
        let supplied_pose = Isometry3d::from_translation(Vec3::new(4.0, 5.0, 6.0));
        let mut targets = RagdollTargetPose::default();
        targets.record(vec![supplied_pose], vec![Default::default()]);
        let character = world
            .spawn((
                SkeletonMap {
                    bones: vec![SkeletonBone {
                        entity: bone,
                        parent: None,
                        body: Some(0),
                        rest_local: Transform::IDENTITY,
                    }],
                    body_to_bone: vec![0],
                },
                targets,
                RagdollBodyWeights::default(),
            ))
            .id();

        initialize_target_components(&mut world, character, 1);

        let body_index = crate::profile::BodyIndex::try_from(0)
            .expect("zero is a valid index for the mapped skeleton body");
        assert_eq!(
            world
                .get::<RagdollTargetPose>(character)
                .and_then(|targets| targets.current_pose(body_index)),
            Some(supplied_pose)
        );
        assert!(world.get::<AutoCapture>(character).is_some());
        assert!(world.get::<RagdollBodyWeights>(character).is_some());
    }

    /// A disappeared entity cannot receive a stable identity after counter reservation.
    #[test]
    fn identity_assignment_skips_a_missing_entity() {
        let mut world = World::new();
        world.insert_resource(RagdollIdCounter::default());

        assert_eq!(assign_ragdoll_id(&mut world, Entity::PLACEHOLDER), None);
        assert_eq!(world.resource::<RagdollIdCounter>().next, 1);
    }

    /// Budget enforcement does nothing after its resource has been removed.
    #[test]
    fn budget_enforcement_skips_a_missing_budget() {
        enforce_budget(&mut World::new());
    }

    /// Settle tracking skips without its fixed clock or physics settings.
    #[test]
    fn settle_tracking_requires_fixed_time_and_settings() {
        let mut no_clock = World::new();
        update_settle_state(&mut no_clock);

        let mut no_settings = World::new();
        no_settings.insert_resource(Time::<Fixed>::from_hz(60.0));
        update_settle_state(&mut no_settings);
    }

    /// Characters without body relationships cannot be considered settled.
    #[test]
    fn settle_check_requires_a_body_relationship() {
        let mut world = World::new();
        let character = world.spawn_empty().id();

        assert!(!all_body_linear_speeds_below(&world, character, 0.1));
    }

    /// An empty relationship target cannot produce a skeleton map.
    #[test]
    fn skeleton_walk_rejects_an_empty_children_list() {
        let mut world = World::new();
        let character = world.spawn(Children::default()).id();

        assert!(walk_skeleton(&world, character, &HashMap::new()).is_none());
    }

    /// Body spawning skips after its physics settings resource has been removed.
    #[test]
    fn body_spawn_requires_physics_settings() {
        let mut world = World::new();
        let character = world.spawn_empty().id();
        let profile = single_body_profile();

        spawn_bodies(&mut world, character, &profile, RagdollMode::Dynamic);

        let mut bodies = world.query::<&BodyShape>();
        assert_eq!(bodies.iter(&world).count(), 0);
    }

    /// Keeps the character animated when the next stable activation identity would wrap.
    #[test]
    fn identity_exhaustion_keeps_character_animated() {
        let mut world = World::new();
        world.insert_resource(RagdollIdCounter { next: u64::MAX });
        let character = world.spawn(RagdollMode::Dynamic).id();

        assert_eq!(assign_ragdoll_id(&mut world, character), None);
        assert_eq!(
            world.get::<RagdollError>(character),
            Some(&RagdollError::IdentityExhausted)
        );
        assert_eq!(
            world.get::<RagdollMode>(character),
            Some(&RagdollMode::Animated)
        );
        assert!(world.get::<RagdollId>(character).is_none());
    }

    /// Animated and frozen modes map to fixed bodies, with active modes
    /// preserved.
    #[test]
    fn body_kind_covers_every_ragdoll_mode() {
        assert_eq!(body_kind(RagdollMode::Animated), BodyKind::Fixed);
        assert_eq!(body_kind(RagdollMode::Kinematic), BodyKind::Kinematic);
        assert_eq!(body_kind(RagdollMode::Dynamic), BodyKind::Dynamic);
        assert_eq!(body_kind(RagdollMode::Frozen), BodyKind::Fixed);
    }
}
