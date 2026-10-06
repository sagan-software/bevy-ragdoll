//! Criterion targets for 60 fixed Rapier steps across ragdoll populations.

use std::hint::black_box;

use criterion::{BatchSize, Criterion, criterion_group, criterion_main};

use bevy_ragdoll_benches::support::{BENCH_SEED, PopulationMode, human_profile, rapier_app};

/// Measures 60 fixed steps for limp, powered, and asleep TGF populations.
fn rapier_step_benchmarks(criterion: &mut Criterion) {
    let profile = human_profile();
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
            let benchmark_name =
                format!("step_rapier3d/{mode_name}/{character_count}_characters/60_steps");
            criterion.bench_function(&benchmark_name, |bencher| {
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
}

criterion_group!(benches, rapier_step_benchmarks);
criterion_main!(benches);
