//! Criterion targets for target capture on synthetic character populations.

use std::hint::black_box;
use std::time::Duration;

use criterion::{BatchSize, Criterion, criterion_group};

use super::support::{BENCH_SEED, PopulationMode, core_app, human_profile};

/// Measures one core app update with target capture for each population size.
fn capture_benchmarks(criterion: &mut Criterion) {
    let profile = human_profile();
    // Longer measurement time steadies the 512-character case.
    let mut group = criterion.benchmark_group("capture/target_capture");
    group.measurement_time(Duration::from_secs(6));
    // Scale the population to show how capture cost grows per character.
    for character_count in [1, 64, 512] {
        let benchmark_name = character_count.to_string();
        group.bench_function(&benchmark_name, |bencher| {
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
    group.finish();
}

criterion_group!(benches, capture_benchmarks);
