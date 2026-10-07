//! Validate plugin order and place Rapier adapter systems around fixed simulation.

#[cfg(feature = "parallel")]
use bevy::app::Startup;
use bevy::app::{App, FixedUpdate, Plugin};
use bevy::ecs::schedule::ScheduleLabel;
use bevy::prelude::{IntoScheduleConfigs, Resource};
use bevy::time::{Fixed, Time};
use bevy_ragdoll::RagdollPlugin;
use bevy_ragdoll::runtime::RagdollFixedSchedule;
use bevy_ragdoll::runtime::backend::BackendCapabilities;
use bevy_ragdoll::runtime::sets::RagdollFixedSystems;
use bevy_ragdoll::runtime::settings::RagdollPhysicsSettings;
use bevy_rapier3d::plugin::{PhysicsSet, TimestepMode};

use crate::body;
use crate::joint;
use crate::query;
use crate::settings::RapierRagdollSettings;

/// Connects the backend-neutral ragdoll runtime to a user-owned Rapier plugin.
///
/// Add [`bevy_ragdoll::RagdollPlugin`] first, then a
/// `RapierPhysicsPlugin` configured with [`crate::RapierRagdollHooks`] and
/// `.in_fixed_schedule()`, then this plugin. The adapter does not create or
/// own Rapier's physics world.
#[derive(Clone, Copy, Debug)]
pub struct RapierRagdollPlugin;

impl Plugin for RapierRagdollPlugin {
    fn build(&self, app: &mut App) {
        // Validate all schedule ownership before inserting adapter state or systems.
        validate_plugin_order(app);
        app.init_resource::<RapierRagdollSettings>()
            .init_resource::<RapierSleepTimers>()
            .register_type::<RapierRagdollSettings>()
            .register_type::<RapierSleepTimers>()
            .insert_resource(BackendCapabilities {
                has_native_joint_motors: true,
                has_asymmetric_swing_limits: true,
                is_deterministic: true,
                can_run_on_wasm: true,
            });
        #[cfg(feature = "parallel")]
        app.add_systems(Startup, configure_rapier_thread_pool);
        // The preceding validation proves the schedule resource exists.
        let fixed_schedule = app
            .world()
            .get_resource::<RagdollFixedSchedule>()
            .expect("validated ragdoll fixed schedule remains installed")
            .label;
        app.add_systems(
            fixed_schedule,
            (
                apply_rapier_settings,
                body::create_rapier_bodies,
                joint::create_rapier_joints,
                body::apply_body_kinds_and_sleeping,
                body::apply_kinematic_targets,
                body::apply_body_forces,
                body::apply_impulses,
                joint::apply_joint_motors,
            )
                .chain()
                .in_set(RagdollFixedSystems::Apply)
                .before(PhysicsSet::SyncBackend),
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
                .after(PhysicsSet::Writeback),
        );
    }
}

/// Gives each default Rapier context the configured dedicated worker pool.
#[cfg(feature = "parallel")]
fn configure_rapier_thread_pool(
    settings: bevy::prelude::Res<'_, RagdollPhysicsSettings>,
    mut simulations: bevy::prelude::Query<
        '_,
        '_,
        &mut bevy_rapier3d::plugin::context::RapierContextSimulation,
        bevy::prelude::With<bevy_rapier3d::plugin::context::DefaultRapierContext>,
    >,
) {
    for mut simulation in &mut simulations {
        if let Err(error) = simulation
            .pipeline
            .configure_thread_pool(settings.threads.max(1))
        {
            bevy::log::warn!(error = %error, "could not configure the Rapier worker pool");
        }
    }
}

/// Elapsed fixed-step age used to apply the delayed limp-body sleep policy.
#[derive(Resource, Default, bevy::prelude::Reflect)]
struct RapierSleepTimers {
    /// Active ragdoll age in seconds, keyed by the owning character entity.
    timers: std::collections::HashMap<bevy::prelude::Entity, f32>,
}

/// Checks the required core plugin, fixed schedule, Rapier schedule, and timestep.
fn validate_plugin_order(app: &App) {
    // The shared runtime must own the fixed schedule before the Rapier adapter starts.
    assert!(
        app.is_plugin_added::<RagdollPlugin>(),
        "RapierRagdollPlugin requires RagdollPlugin to be added first"
    );
    let fixed_schedule = app
        .world()
        .get_resource::<RagdollFixedSchedule>()
        .expect("RagdollPlugin installs its fixed schedule resource");
    assert_eq!(
        fixed_schedule.label,
        FixedUpdate.intern(),
        "RapierRagdollPlugin requires RagdollPlugin to use FixedUpdate"
    );
    let rapier_is_fixed = app.get_schedule(FixedUpdate).is_some_and(|schedule| {
        schedule
            .graph()
            .system_sets
            .contains(PhysicsSet::StepSimulation)
    });
    assert!(
        rapier_is_fixed,
        "RapierRagdollPlugin requires the user to add RapierPhysicsPlugin with in_fixed_schedule() first"
    );
    // Rapier and Bevy must advance with the same fixed delta for deterministic readback.
    let timestep = app
        .world()
        .get_resource::<TimestepMode>()
        .expect("RapierPhysicsPlugin initializes TimestepMode");
    assert!(
        matches!(timestep, TimestepMode::Fixed { .. }),
        "RapierRagdollPlugin requires TimestepMode::Fixed in FixedUpdate"
    );
    if let TimestepMode::Fixed { dt, .. } = timestep {
        let fixed_time = app
            .world()
            .get_resource::<Time<Fixed>>()
            .expect("the Bevy fixed schedule has a Time<Fixed> resource")
            .timestep()
            .as_secs_f32();
        assert!(
            (*dt - fixed_time).abs() <= 1.0e-6,
            "Rapier TimestepMode::Fixed dt must match Time<Fixed>"
        );
    }
}

/// Copies backend-neutral solver settings to each default Rapier context.
fn apply_rapier_settings(
    ragdoll_settings: bevy::prelude::Res<'_, RagdollPhysicsSettings>,
    rapier_settings: bevy::prelude::Res<'_, RapierRagdollSettings>,
    mut configurations: bevy::prelude::Query<
        '_,
        '_,
        &mut bevy_rapier3d::prelude::RapierConfiguration,
        bevy::prelude::With<bevy_rapier3d::plugin::context::DefaultRapierContext>,
    >,
    mut simulations: bevy::prelude::Query<
        '_,
        '_,
        &mut bevy_rapier3d::plugin::context::RapierContextSimulation,
        bevy::prelude::With<bevy_rapier3d::plugin::context::DefaultRapierContext>,
    >,
) {
    // Use finite gravity and solver settings from the shared physics profile.
    let gravity = if ragdoll_settings.gravity.is_finite() {
        ragdoll_settings.gravity
    } else {
        bevy::math::Vec3::ZERO
    };
    for mut configuration in &mut configurations {
        configuration.gravity = gravity;
    }
    // Copy Rapier-only CCD settings alongside the shared solver iteration counts.
    for mut simulation in &mut simulations {
        simulation.integration_parameters.num_solver_iterations =
            ragdoll_settings.solver_iterations.max(1);
        simulation
            .integration_parameters
            .num_internal_pgs_iterations = ragdoll_settings.pgs_iterations;
        simulation.integration_parameters.max_ccd_substeps = rapier_settings.max_ccd_substeps;
    }
}

/// Sleeps a limp ragdoll after its configured age when every body is slow.
fn force_sleep_limp_ragdolls(
    time: bevy::prelude::Res<'_, Time<Fixed>>,
    settings: bevy::prelude::Res<'_, RagdollPhysicsSettings>,
    mut timers: bevy::prelude::ResMut<'_, RapierSleepTimers>,
    mut live_characters: bevy::prelude::Local<'_, std::collections::HashSet<bevy::prelude::Entity>>,
    characters: bevy::prelude::Query<
        '_,
        '_,
        (
            bevy::prelude::Entity,
            &bevy_ragdoll::runtime::components::RagdollDrive,
        ),
        bevy::prelude::With<bevy_ragdoll::runtime::components::Ragdoll>,
    >,
    body_speeds: bevy::prelude::Query<
        '_,
        '_,
        (
            &bevy_ragdoll::runtime::body::BodyVelocity,
            &bevy_ragdoll::runtime::components::RagdollBodyOf,
        ),
    >,
    mut body_sleep: bevy::prelude::Query<
        '_,
        '_,
        (
            &bevy_ragdoll::runtime::components::RagdollBodyOf,
            &mut bevy_rapier3d::prelude::Sleeping,
        ),
    >,
) {
    // Retain timers only while their owning character still has a ragdoll component.
    live_characters.clear();
    live_characters.extend(characters.iter().map(|(entity, _)| entity));
    timers
        .timers
        .retain(|entity, _| live_characters.contains(entity));
    for (character, drive) in &characters {
        // Remove stale ages when no body belonging to this character remains.
        let Some(every_body_is_slow) =
            body_speeds_are_slow_for_owner(character, &body_speeds, settings.settle_speed)
        else {
            timers.timers.remove(&character);
            continue;
        };
        // Age every live ragdoll in fixed time, including driven or moving ragdolls.
        let age = timers.timers.entry(character).or_default();
        *age += time.delta_secs();
        if should_force_sleep(*drive, every_body_is_slow, *age, *settings) {
            // Mark only this owner's bodies so other ragdolls keep their sleep state.
            body_sleep
                .iter_mut()
                .filter(|(owner, _)| owner.0 == character)
                .for_each(|(_, mut sleeping)| sleeping.sleeping = true);
        }
    }
}

/// Returns whether an owner has bodies and every body's linear speed is below the threshold.
fn body_speeds_are_slow_for_owner(
    character: bevy::prelude::Entity,
    body_speeds: &bevy::prelude::Query<
        '_,
        '_,
        (
            &bevy_ragdoll::runtime::body::BodyVelocity,
            &bevy_ragdoll::runtime::components::RagdollBodyOf,
        ),
    >,
    settle_speed: f32,
) -> Option<bool> {
    // Ignore velocity samples belonging to other ragdoll owners.
    let mut has_bodies = false;
    // Require every matching body to remain below the settle-speed threshold.
    let mut every_body_is_slow = true;
    for (velocity, owner) in body_speeds {
        if owner.0 != character {
            continue;
        }
        has_bodies = true;
        every_body_is_slow &= velocity.linear.length() < settle_speed;
    }
    has_bodies.then_some(every_body_is_slow)
}

/// Applies delayed sleep only to limp, settled ragdolls whose timer has elapsed.
fn should_force_sleep(
    drive: bevy_ragdoll::runtime::components::RagdollDrive,
    every_body_is_slow: bool,
    age: f32,
    settings: RagdollPhysicsSettings,
) -> bool {
    drive.muscle() <= 0.0
        && drive.pin() <= 0.0
        && every_body_is_slow
        && settings.force_sleep_after > 0.0
        && age >= settings.force_sleep_after
}

#[cfg(test)]
mod tests {
    //! Checks plugin timestep validation, gravity sanitization, and limp sleep.

    use std::time::Duration;

    use bevy::asset::{AssetPlugin, Handle};
    use bevy::prelude::{AnimationPlugin, App, MinimalPlugins, TransformPlugin};
    use bevy::time::{Fixed, Time, TimeUpdateStrategy};
    use bevy::{ecs::schedule::Schedule, prelude::World};
    use bevy_ragdoll::runtime::body::BodyVelocity;
    use bevy_ragdoll::runtime::components::{Ragdoll, RagdollBodyOf, RagdollDrive};
    use bevy_ragdoll::runtime::settings::RagdollPhysicsSettings;
    use bevy_ragdoll::{RagdollPlugin, RagdollProfile};
    use bevy_rapier3d::plugin::context::DefaultRapierContext;
    use bevy_rapier3d::plugin::{RapierPhysicsPlugin, TimestepMode};
    use bevy_rapier3d::prelude::{RapierConfiguration, Sleeping};

    use super::{
        RapierRagdollPlugin, RapierSleepTimers, apply_rapier_settings, force_sleep_limp_ragdolls,
    };

    /// Builds the headless plugin stack with one caller-selected Rapier timestep.
    fn app_with_timestep(timestep: TimestepMode) -> App {
        let mut app = App::new();
        app.add_plugins((
            MinimalPlugins,
            TransformPlugin,
            AssetPlugin::default(),
            AnimationPlugin,
        ));
        app.insert_resource(Time::<Fixed>::from_hz(60.0));
        app.insert_resource(TimeUpdateStrategy::FixedTimesteps(1));
        app.add_plugins(RagdollPlugin::default());
        app.insert_resource(timestep);
        app.add_plugins(
            RapierPhysicsPlugin::<crate::RapierRagdollHooks<'static, 'static>>::default()
                .in_fixed_schedule(),
        );
        app
    }

    /// A variable Rapier step is rejected even when Rapier uses `FixedUpdate`.
    #[test]
    #[should_panic(expected = "RapierRagdollPlugin requires TimestepMode::Fixed in FixedUpdate")]
    fn plugin_rejects_variable_timestep_mode() {
        let mut app = app_with_timestep(TimestepMode::Variable {
            max_dt: 1.0 / 60.0,
            time_scale: 1.0,
            substeps: 1,
        });

        app.add_plugins(RapierRagdollPlugin);
    }

    /// A fixed Rapier interval must match Bevy's fixed clock interval.
    #[test]
    #[should_panic(expected = "Rapier TimestepMode::Fixed dt must match Time<Fixed>")]
    fn plugin_rejects_mismatched_fixed_timestep() {
        let mut app = app_with_timestep(TimestepMode::Fixed {
            dt: 1.0 / 30.0,
            substeps: 1,
        });

        app.add_plugins(RapierRagdollPlugin);
    }

    /// Non-finite shared gravity writes a zero vector to Rapier contexts.
    #[test]
    fn non_finite_gravity_is_replaced_with_zero() {
        let mut app = app_with_timestep(TimestepMode::Fixed {
            dt: 1.0 / 60.0,
            substeps: 1,
        });
        app.add_plugins(RapierRagdollPlugin);
        app.update();
        if let Some(mut settings) = app.world_mut().get_resource_mut::<RagdollPhysicsSettings>() {
            settings.gravity = bevy::math::Vec3::splat(f32::NAN);
        }
        let mut schedule = Schedule::default();
        schedule.add_systems(apply_rapier_settings);
        schedule.run(app.world_mut());

        let world = app.world_mut();
        let mut configurations = world
            .query_filtered::<&RapierConfiguration, bevy::prelude::With<DefaultRapierContext>>();
        let gravity = configurations
            .iter(world)
            .next()
            .expect("the Rapier plugin creates its default context")
            .gravity;

        assert_eq!(gravity, bevy::math::Vec3::ZERO);
    }

    /// Limp slow bodies sleep after the configured age without affecting another owner.
    #[test]
    fn force_sleep_only_marks_the_slow_unpinned_owner() {
        let mut world = World::new();
        let mut fixed_time = Time::<Fixed>::from_hz(60.0);
        fixed_time.advance_by(Duration::from_millis(17));
        world.insert_resource(fixed_time);
        let base = RagdollPhysicsSettings::default();
        world.insert_resource(RagdollPhysicsSettings {
            settle_speed: 0.1,
            force_sleep_after: 0.01,
            ..base
        });
        world.insert_resource(RapierSleepTimers::default());

        let sleeping_owner = world
            .spawn((
                Ragdoll::new(Handle::<RagdollProfile>::default()),
                RagdollDrive::new(0.0, 0.0),
            ))
            .id();
        let moving_owner = world
            .spawn((
                Ragdoll::new(Handle::<RagdollProfile>::default()),
                RagdollDrive::new(0.0, 0.0),
            ))
            .id();
        let sleeping_body = world
            .spawn((
                RagdollBodyOf(sleeping_owner),
                BodyVelocity::default(),
                Sleeping::disabled(),
            ))
            .id();
        let moving_body = world
            .spawn((
                RagdollBodyOf(moving_owner),
                BodyVelocity {
                    linear: bevy::math::Vec3::X,
                    angular: bevy::math::Vec3::ZERO,
                },
                Sleeping::disabled(),
            ))
            .id();
        let mut schedule = Schedule::default();
        schedule.add_systems(force_sleep_limp_ragdolls);

        schedule.run(&mut world);

        assert!(world.get::<Sleeping>(sleeping_body).unwrap().sleeping);
        assert!(!world.get::<Sleeping>(moving_body).unwrap().sleeping);
    }
}
