//! Criterion targets for target capture on synthetic character populations.

use std::hint::black_box;

use criterion::{BatchSize, Criterion, criterion_group, criterion_main};

use bevy_ragdoll_benches::support::{BENCH_SEED, PopulationMode, core_app, human_profile};

/// Measures one core app update with target capture for each population size.
fn capture_benchmarks(criterion: &mut Criterion) {
    let profile = human_profile();
    for character_count in [1, 64, 512] {
        let benchmark_name = format!("capture/target_capture/{character_count}");
        criterion.bench_function(&benchmark_name, |bencher| {
            bencher.iter_batched(
                || {
                    core_app(
                        profile.clone(),
                        character_count,
                        PopulationMode::Capture,
                        BENCH_SEED,
                    )
                    .expect("capture benchmark app setup must produce the complete fixture")
                },
                |mut app| {
                    app.update();
                    black_box(app.world().entities().len());
                },
                BatchSize::SmallInput,
            );
        });
    }
}

criterion_group!(benches, capture_benchmarks);
criterion_main!(benches);
