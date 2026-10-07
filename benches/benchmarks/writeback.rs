//! Criterion targets for physics-to-skeleton writeback populations.

use std::hint::black_box;

use criterion::{BatchSize, Criterion, criterion_group};

use super::support::{BENCH_SEED, PopulationMode, core_app, human_profile};

/// Measures one core app update with writeback for each population size.
fn writeback_benchmarks(criterion: &mut Criterion) {
    let profile = human_profile();
    for character_count in [1, 64, 512] {
        let benchmark_name = format!("writeback/physics_writeback/{character_count}");
        criterion.bench_function(&benchmark_name, |bencher| {
            bencher.iter_batched(
                || {
                    core_app(
                        profile.clone(),
                        character_count,
                        PopulationMode::Limp,
                        BENCH_SEED,
                    )
                    .expect("writeback benchmark app setup must produce the complete fixture")
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

criterion_group!(benches, writeback_benchmarks);
