//! Criterion targets for joint angles, motor values, torque, and pin math.

use std::hint::black_box;
use std::time::Duration;

use bevy::app::FixedUpdate;
use bevy::math::{Isometry3d, Quat, Vec3};
use bevy::prelude::{Entity, With, World};
use bevy_ragdoll::runtime::body::{BodyShape, BodyVelocity};
use bevy_ragdoll::runtime::components::RagdollBodyOf;
use bevy_ragdoll::runtime::drive::{
    PinDriveInput, StablePdInput, joint_motor_values, pin_drive, stable_pd_torque,
};
use bevy_ragdoll::runtime::messages::{HitKind, RagdollHit};
use bevy_ragdoll::runtime::settings::RagdollPhysicsSettings;
use criterion::{Criterion, criterion_group};
use rand_chacha::ChaCha8Rng;
use rand_core::{Rng, SeedableRng};

use super::support::{BENCH_SEED, PopulationMode, chain_profile, core_app, human_profile};

/// Measures deterministic loops of joint-angle, motor, torque, and pin math.
fn math_benchmarks(criterion: &mut Criterion) {
    let profile = chain_profile(64, BENCH_SEED);
    let poses = profile.rest_poses(Isometry3d::IDENTITY).collect::<Vec<_>>();
    let joint_children = profile
        .joints()
        .iter()
        .map(bevy_ragdoll::Joint::child)
        .collect::<Vec<_>>();
    let settings = RagdollPhysicsSettings::default();
    let mut random = ChaCha8Rng::seed_from_u64(BENCH_SEED);

    for evaluation_count in [1, 16, 1024] {
        let angle_children = (0..evaluation_count)
            .filter_map(|_| {
                let offset =
                    usize::try_from(random.next_u32()).unwrap_or_default() % joint_children.len();
                joint_children.get(offset).copied()
            })
            .collect::<Vec<_>>();
        let inputs = (0..evaluation_count)
            .map(|_| {
                let angle = next_angle(&mut random);
                let muscle = f32::midpoint(angle, 1.0);
                let motor = joint_motor_values(muscle, 40.0, &settings);
                let target_rotation = Quat::from_rotation_y(angle);
                let torque = StablePdInput {
                    frame_rotation: Quat::IDENTITY,
                    current_relative_rotation: Quat::IDENTITY,
                    target_relative_rotation: target_rotation,
                    relative_angular_velocity: Vec3::ZERO,
                    target_angular_velocity: Vec3::Y * angle,
                    inertia: 0.2 + muscle,
                    delta_seconds: 1.0 / 60.0,
                };
                let pin = PinDriveInput {
                    current_pose: Isometry3d::IDENTITY,
                    target_pose: Isometry3d::new(Vec3::new(angle, 0.2, -angle), target_rotation),
                    current_velocity: BodyVelocity::default(),
                    target_velocity: BodyVelocity {
                        linear: Vec3::X * angle,
                        angular: Vec3::Y * angle,
                    },
                    mass: 4.0,
                    inertia: 0.4,
                    strength: muscle,
                };
                (angle, muscle, motor, torque, pin)
            })
            .collect::<Vec<_>>();

        let angle_name = format!("math/joint_angles/{evaluation_count}");
        criterion.bench_function(&angle_name, |bencher| {
            bencher.iter(|| {
                for child in &angle_children {
                    black_box(profile.joint_angles(*child, &poses));
                }
            });
        });
        let motor_name = format!("math/muscle_drive/{evaluation_count}");
        criterion.bench_function(&motor_name, |bencher| {
            bencher.iter(|| {
                for (_, muscle, _, _, _) in &inputs {
                    black_box(joint_motor_values(*muscle, 40.0, &settings));
                }
            });
        });
        let torque_name = format!("math/torque_drive/{evaluation_count}");
        criterion.bench_function(&torque_name, |bencher| {
            bencher.iter(|| {
                for (_, _, motor, torque, _) in &inputs {
                    black_box(stable_pd_torque(*torque, *motor));
                }
            });
        });
        let pin_name = format!("math/pin_drive/{evaluation_count}");
        criterion.bench_function(&pin_name, |bencher| {
            bencher.iter(|| {
                for (_, _, _, _, pin) in &inputs {
                    black_box(pin_drive(*pin, &settings));
                }
            });
        });
    }

    let human = human_profile();
    let mut group = criterion.benchmark_group("math/hit_processing");
    group.measurement_time(Duration::from_secs(5));
    for character_count in [1, 64] {
        let mut app = core_app(
            human.clone(),
            character_count,
            PopulationMode::Capture,
            BENCH_SEED,
        )
        .expect("hit benchmark setup must bind the complete character population");
        let targets = first_body_per_character(app.world_mut());
        assert_eq!(targets.len(), character_count);
        let benchmark_name = format!("{character_count}_ragdolls");
        group.bench_function(&benchmark_name, |bencher| {
            bencher.iter(|| {
                for body in &targets {
                    app.world_mut().write_message(RagdollHit {
                        body: *body,
                        point: Vec3::new(0.0, 1.0, 0.0),
                        impulse: Vec3::X * 40.0,
                        kind: HitKind::Impact,
                    });
                }
                app.world_mut()
                    .try_run_schedule(FixedUpdate)
                    .expect("the ragdoll plugin installs FixedUpdate");
                black_box(targets.len());
            });
        });
    }
}

/// Collects one stable root-body entity for each bound character.
fn first_body_per_character(world: &mut World) -> Vec<Entity> {
    let mut query = world
        .query_filtered::<(Entity, &RagdollBodyOf, &bevy_ragdoll::BodyIndex), With<BodyShape>>();
    let mut targets = query
        .iter(world)
        .filter(|(_, _, index)| index.get() == 0)
        .map(|(body, owner, _)| (owner.0, body))
        .collect::<Vec<_>>();
    targets.sort_unstable_by_key(|(character, _)| character.to_bits());
    targets.into_iter().map(|(_, body)| body).collect()
}

/// Derives one deterministic angle in the inclusive half-radian range.
#[expect(
    clippy::cast_precision_loss,
    reason = "a 24-bit integer converts to f32 exactly"
)]
fn next_angle(random: &mut ChaCha8Rng) -> f32 {
    let bits = random.next_u32() >> 8;
    let unit = bits as f32 / 16_777_215.0;
    unit.mul_add(2.0, -1.0) * 0.5
}

criterion_group!(benches, math_benchmarks);
