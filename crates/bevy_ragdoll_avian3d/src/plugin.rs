//! Validate plugin order and place Avian adapter systems around the physics step.

use std::collections::HashMap;

use avian3d::prelude::{Gravity, PhysicsSystems, SleepBody, SubstepCount};
use bevy::app::{App, Plugin};
use bevy::ecs::change_detection::DetectChangesMut;
use bevy::math::Vec3;
use bevy::prelude::{
    Command, Commands, Entity, IntoScheduleConfigs, Query, Res, ResMut, Resource, With, World,
};
use bevy::time::{Fixed, Time};
use bevy_ragdoll::RagdollPlugin;
use bevy_ragdoll::runtime::RagdollFixedSchedule;
use bevy_ragdoll::runtime::backend::BackendCapabilities;
use bevy_ragdoll::runtime::body::BodyVelocity;
use bevy_ragdoll::runtime::components::{Ragdoll, RagdollBodyOf, RagdollDrive};
use bevy_ragdoll::runtime::sets::RagdollFixedSystems;
use bevy_ragdoll::runtime::settings::RagdollPhysicsSettings;

use crate::settings::AvianRagdollSettings;
use crate::{body, joint, query};

/// Capabilities of the Avian adapter.
///
/// Avian 0.7 has no spherical joint motors (issue #934) and one symmetric
/// swing cone, so the core drives joints with stable-PD torque and enforces the
/// asymmetric X and Z ranges with soft-limit torque. Determinism has not been
/// measured yet.
pub const AVIAN_CAPABILITIES: BackendCapabilities = BackendCapabilities {
    has_native_joint_motors: false,
    has_asymmetric_swing_limits: false,
    is_deterministic: false,
    can_run_on_wasm: true,
};

/// Connects the backend-neutral ragdoll runtime to a user-owned Avian world.
///
/// Add [`bevy_ragdoll::RagdollPlugin`] first, then Avian's `PhysicsPlugins`
/// in the same schedule as `RagdollPlugin` (`FixedUpdate` by default) with
/// [`crate::AvianRagdollHooks`], then this plugin:
///
/// ```no_run
/// use avian3d::prelude::PhysicsPlugins;
/// use bevy::prelude::{App, FixedUpdate};
/// use bevy_ragdoll::RagdollPlugin;
/// use bevy_ragdoll_avian3d::{AvianRagdollHooks, AvianRagdollPlugin};
///
/// App::new().add_plugins((
///     RagdollPlugin::default(),
///     PhysicsPlugins::new(FixedUpdate).with_collision_hooks::<AvianRagdollHooks>(),
///     AvianRagdollPlugin,
/// ));
/// ```
///
/// The adapter copies `RagdollPhysicsSettings::gravity` to `Gravity` and
/// [`crate::AvianRagdollSettings::substep_count`] to Avian's `SubstepCount`.
/// `solver_iterations`, `pgs_iterations`, `max_substeps` and `threads` have
/// no Avian equivalent.
#[derive(Clone, Copy, Debug)]
pub struct AvianRagdollPlugin;

impl Plugin for AvianRagdollPlugin {
    fn build(&self, app: &mut App) {
        let fixed_schedule = validate_plugin_order(app);
        app.init_resource::<AvianSleepTimers>()
            .init_resource::<AvianRagdollSettings>()
            .insert_resource(AVIAN_CAPABILITIES);
        app.add_systems(
            fixed_schedule,
            (
                apply_avian_settings,
                body::create_avian_bodies,
                joint::create_avian_joints,
                body::apply_body_kinds_and_sleeping,
                body::apply_kinematic_targets,
                body::apply_body_forces,
                body::apply_impulses,
            )
                .chain()
                .in_set(RagdollFixedSystems::Apply)
                .before(PhysicsSystems::First),
        );
        app.add_systems(
            fixed_schedule,
            (
                query::read_body_pose,
                query::read_body_motion,
                query::read_ragdoll_queries,
                force_sleep_limp_ragdolls,
            )
                .chain()
                .in_set(RagdollFixedSystems::Read)
                .after(PhysicsSystems::Last),
        );
    }
}

/// Elapsed fixed-step age per character for TGF's delayed limp-body sleep.
#[derive(Resource, Default)]
struct AvianSleepTimers {
    /// Active ragdoll age in seconds, keyed by the owning character entity.
    timers: HashMap<Entity, f32>,
}

/// Checks the core plugin and that Avian steps in the ragdoll fixed schedule,
/// then returns that schedule.
fn validate_plugin_order(
    app: &App,
) -> bevy::ecs::intern::Interned<dyn bevy::ecs::schedule::ScheduleLabel> {
    assert!(
        app.is_plugin_added::<RagdollPlugin>(),
        "AvianRagdollPlugin requires RagdollPlugin to be added first"
    );
    let fixed_schedule = app
        .world()
        .get_resource::<RagdollFixedSchedule>()
        .expect("RagdollPlugin installs its fixed schedule resource")
        .label;
    // Avian steps with the delta of the schedule it runs in, so no timestep check is needed.
    let avian_in_schedule = app.get_schedule(fixed_schedule).is_some_and(|schedule| {
        schedule
            .graph()
            .system_sets
            .contains(PhysicsSystems::StepSimulation)
    });
    assert!(
        avian_in_schedule,
        "AvianRagdollPlugin requires PhysicsPlugins::new(<RagdollPlugin fixed schedule>) to be added first"
    );
    fixed_schedule
}

/// Copies shared gravity and solver settings into Avian resources.
fn apply_avian_settings(
    settings: Res<'_, RagdollPhysicsSettings>,
    avian_settings: Res<'_, AvianRagdollSettings>,
    mut gravity: ResMut<'_, Gravity>,
    mut substeps: ResMut<'_, SubstepCount>,
) {
    let target_gravity = if settings.gravity.is_finite() {
        settings.gravity
    } else {
        Vec3::ZERO
    };
    // Write only on change so Avian's change detection stays quiet.
    if gravity.0 != target_gravity {
        gravity.0 = target_gravity;
    }
    substeps.set_if_neq(SubstepCount(avian_settings.substep_count.max(1)));
}

/// Forces a body to sleep and ignores bodies that have no island yet.
struct TrySleepBody(Entity);

impl Command for TrySleepBody {
    type Out = ();

    fn apply(self, world: &mut World) {
        // A body without an island cannot sleep yet; the next step retries.
        let _ignored = SleepBody(self.0).apply(world);
    }
}

/// Sleeps a limp ragdoll after its configured age when every body is slow.
fn force_sleep_limp_ragdolls(
    mut commands: Commands<'_, '_>,
    time: Res<'_, Time<Fixed>>,
    settings: Res<'_, RagdollPhysicsSettings>,
    mut timers: ResMut<'_, AvianSleepTimers>,
    characters: Query<'_, '_, (Entity, &RagdollDrive), With<Ragdoll>>,
    bodies: Query<'_, '_, (Entity, &BodyVelocity, &RagdollBodyOf)>,
) {
    timers
        .timers
        .retain(|character, _| characters.contains(*character));
    for (character, drive) in &characters {
        let mut owned = bodies
            .iter()
            .filter(|(_, _, owner)| owner.0 == character)
            .peekable();
        if owned.peek().is_none() {
            timers.timers.remove(&character);
            continue;
        }
        let every_body_is_slow = owned
            .clone()
            .all(|(_, velocity, _)| velocity.linear.length() < settings.settle_speed);
        let age = timers.timers.entry(character).or_default();
        *age += time.delta_secs();
        if should_force_sleep(*drive, every_body_is_slow, *age, &settings) {
            // Sleeping one body sleeps its whole island; queue each to cover split islands.
            for (body, ..) in owned {
                commands.queue(TrySleepBody(body));
            }
        }
    }
}

/// Applies delayed sleep only to limp, settled ragdolls whose timer has elapsed.
fn should_force_sleep(
    drive: RagdollDrive,
    every_body_is_slow: bool,
    age: f32,
    settings: &RagdollPhysicsSettings,
) -> bool {
    drive.muscle() <= 0.0
        && drive.pin() <= 0.0
        && every_body_is_slow
        && settings.force_sleep_after > 0.0
        && age >= settings.force_sleep_after
}

#[cfg(test)]
mod tests {
    //! Checks plugin validation, settings sanitization, and limp sleep.

    use std::time::Duration;

    use avian3d::prelude::{Gravity, PhysicsPlugins, SubstepCount};
    use bevy::asset::{AssetPlugin, Handle};
    use bevy::ecs::schedule::Schedule;
    use bevy::prelude::{
        AnimationPlugin, App, Command, FixedPostUpdate, MinimalPlugins, TransformPlugin, World,
    };
    use bevy::time::{Fixed, Time};
    use bevy_ragdoll::runtime::body::BodyVelocity;
    use bevy_ragdoll::runtime::components::{Ragdoll, RagdollBodyOf, RagdollDrive};
    use bevy_ragdoll::runtime::settings::RagdollPhysicsSettings;
    use bevy_ragdoll::{RagdollPlugin, RagdollProfile};

    use super::{
        AvianRagdollPlugin, AvianRagdollSettings, AvianSleepTimers, TrySleepBody,
        apply_avian_settings, force_sleep_limp_ragdolls, should_force_sleep,
    };

    /// Builds the headless core stack.
    fn core_app() -> App {
        let mut app = App::new();
        app.add_plugins((
            MinimalPlugins,
            TransformPlugin,
            AssetPlugin::default(),
            AnimationPlugin,
        ));
        app.add_plugins(RagdollPlugin::default());
        app
    }

    /// The adapter needs the core plugin.
    #[test]
    #[should_panic(expected = "AvianRagdollPlugin requires RagdollPlugin to be added first")]
    fn plugin_requires_the_core_plugin() {
        App::new().add_plugins(AvianRagdollPlugin);
    }

    /// Avian must step in the ragdoll fixed schedule.
    #[test]
    #[should_panic(expected = "AvianRagdollPlugin requires PhysicsPlugins::new")]
    fn plugin_rejects_avian_in_another_schedule() {
        let mut app = core_app();
        app.add_plugins(PhysicsPlugins::new(FixedPostUpdate));
        app.add_plugins(AvianRagdollPlugin);
    }

    /// Non-finite gravity becomes zero and a zero substep count becomes one.
    #[test]
    fn settings_sanitize_gravity_and_map_substeps() {
        let mut world = World::new();
        world.insert_resource(RagdollPhysicsSettings {
            gravity: bevy::math::Vec3::splat(f32::NAN),
            ..Default::default()
        });
        world.insert_resource(AvianRagdollSettings {
            substep_count: 0,
            ..Default::default()
        });
        world.insert_resource(Gravity::default());
        world.insert_resource(SubstepCount(6));
        let mut schedule = Schedule::default();
        schedule.add_systems(apply_avian_settings);
        schedule.run(&mut world);

        assert_eq!(world.resource::<Gravity>().0, bevy::math::Vec3::ZERO);
        assert_eq!(world.resource::<SubstepCount>().0, 1);
    }

    /// Limp slow owners pass the sleep gate; driven, moving, young or disabled ones do not.
    #[test]
    fn sleep_gate_requires_limp_slow_and_old_enough() {
        let settings = RagdollPhysicsSettings {
            force_sleep_after: 1.0,
            ..Default::default()
        };
        let limp = RagdollDrive::new(0.0, 0.0);
        assert!(should_force_sleep(limp, true, 1.0, &settings));
        assert!(!should_force_sleep(
            RagdollDrive::new(1.0, 0.0),
            true,
            1.0,
            &settings
        ));
        assert!(!should_force_sleep(
            RagdollDrive::new(0.0, 1.0),
            true,
            1.0,
            &settings
        ));
        assert!(!should_force_sleep(limp, false, 1.0, &settings));
        assert!(!should_force_sleep(limp, true, 0.5, &settings));
        let disabled = RagdollPhysicsSettings {
            force_sleep_after: 0.0,
            ..settings
        };
        assert!(!should_force_sleep(limp, true, 1.0, &disabled));
    }

    /// Timers age live owners with bodies and drop owners without bodies or ragdolls.
    #[test]
    fn sleep_timers_track_only_live_owners_with_bodies() {
        let mut world = World::new();
        let mut fixed_time = Time::<Fixed>::from_hz(60.0);
        fixed_time.advance_by(Duration::from_millis(20));
        world.insert_resource(fixed_time);
        world.insert_resource(RagdollPhysicsSettings {
            settle_speed: 0.1,
            force_sleep_after: 0.01,
            ..Default::default()
        });
        let with_bodies = world
            .spawn((
                Ragdoll::new(Handle::<RagdollProfile>::default()),
                RagdollDrive::new(0.0, 0.0),
            ))
            .id();
        let without_bodies = world
            .spawn((
                Ragdoll::new(Handle::<RagdollProfile>::default()),
                RagdollDrive::new(0.0, 0.0),
            ))
            .id();
        let removed = world.spawn_empty().id();
        let mut timers = AvianSleepTimers::default();
        timers.timers.insert(without_bodies, 5.0);
        timers.timers.insert(removed, 5.0);
        world.insert_resource(timers);
        world.spawn((RagdollBodyOf(with_bodies), BodyVelocity::default()));
        let mut schedule = Schedule::default();
        schedule.add_systems(force_sleep_limp_ragdolls);

        schedule.run(&mut world);

        let timers = &world.resource::<AvianSleepTimers>().timers;
        assert!(timers.contains_key(&with_bodies));
        assert!(!timers.contains_key(&without_bodies));
        assert!(!timers.contains_key(&removed));
    }

    /// Sleeping an entity that is not a body does nothing and does not panic.
    #[test]
    fn sleeping_a_non_body_is_ignored() {
        let mut world = World::new();
        let entity = world.spawn_empty().id();
        TrySleepBody(entity).apply(&mut world);
        assert!(world.get_entity(entity).is_ok());
    }
}
