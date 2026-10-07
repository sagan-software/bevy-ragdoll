//! Criterion targets for 60 fixed Rapier steps across ragdoll populations.

use std::hint::black_box;
use std::time::Duration;

use criterion::{BatchSize, Criterion, criterion_group};

use super::support::{BENCH_SEED, PopulationMode, human_profile, rapier_app};

/// Measures 60 fixed steps for limp, powered, and asleep TGF populations.
fn rapier_step_benchmarks(criterion: &mut Criterion) {
    let profile = human_profile();
    let mut group = criterion.benchmark_group("step_rapier3d");
    for mode in [
        PopulationMode::Limp,
        PopulationMode::Powered,
        PopulationMode::Asleep,
    ] {
        let mode_name = match mode {
            PopulationMode::Limp => "limp",
            PopulationMode::Powered => "powered",
            PopulationMode::Asleep => "asleep",
            PopulationMode::Capture => "capture",
        };
        for character_count in [1, 32, 128, 512] {
            // When physics batches dominate runtime, keep 10 samples and budget one 60-step batch per sample.
            let (sample_size, measurement_time) = match character_count {
                32 => (10, Duration::from_secs(5)),
                128 => (10, Duration::from_millis(20_100)),
                512 => (10, Duration::from_millis(91_700)),
                _ => (100, Duration::from_secs(5)),
            };
            group
                .sample_size(sample_size)
                .measurement_time(measurement_time);
            let benchmark_name = format!("{mode_name}/{character_count}_characters/60_steps");
            group.bench_function(&benchmark_name, |bencher| {
                bencher.iter_batched(
                    || {
                        rapier_app(profile.clone(), character_count, mode, BENCH_SEED)
                            .expect("Rapier benchmark setup must produce the complete fixture")
                    },
                    |mut app| {
                        for _ in 0..60 {
                            app.update();
                        }
                        black_box(app.world().entities().len());
                    },
                    BatchSize::SmallInput,
                );
            });
        }
    }
    group.finish();
}

criterion_group!(benches, rapier_step_benchmarks);
