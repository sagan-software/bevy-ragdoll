//! Criterion targets for joint angles, motor values, torque, and pin math.

use std::hint::black_box;

use bevy::math::{Isometry3d, Quat, Vec3};
use bevy_ragdoll::runtime::body::BodyVelocity;
use bevy_ragdoll::runtime::drive::{
    PinDriveInput, StablePdInput, joint_motor_values, pin_drive, stable_pd_torque,
};
use bevy_ragdoll::runtime::settings::RagdollPhysicsSettings;
use criterion::{Criterion, criterion_group, criterion_main};
use rand_chacha::ChaCha8Rng;
use rand_core::{RngCore, SeedableRng};

use bevy_ragdoll_benches::support::{BENCH_SEED, chain_profile};

/// Measures deterministic loops of joint-angle, motor, torque, and pin math.
fn math_benchmarks(criterion: &mut Criterion) {
    let profile = chain_profile(64, BENCH_SEED);
    let poses = profile.rest_poses(Isometry3d::IDENTITY).collect::<Vec<_>>();
    let joint_children = profile
        .joints()
        .iter()
        .map(|joint| joint.child())
        .collect::<Vec<_>>();
    let settings = RagdollPhysicsSettings::default();
    let mut random = ChaCha8Rng::seed_from_u64(BENCH_SEED);

    for evaluation_count in [1, 16, 1024] {
        let angle_children = (0..evaluation_count)
            .map(|_| {
                let offset =
                    usize::try_from(random.next_u32()).unwrap_or_default() % joint_children.len();
                joint_children[offset]
            })
            .collect::<Vec<_>>();
        let inputs = (0..evaluation_count)
            .map(|_| {
                let angle = next_angle(&mut random);
                let muscle = (angle + 1.0) * 0.5;
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
}

/// Derives one deterministic angle in the inclusive half-radian range.
fn next_angle(random: &mut ChaCha8Rng) -> f32 {
    let bits = random.next_u32() >> 8;
    let unit = bits as f32 / 16_777_215.0;
    (unit * 2.0 - 1.0) * 0.5
}

criterion_group!(benches, math_benchmarks);
criterion_main!(benches);
