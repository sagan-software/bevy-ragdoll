//! Criterion targets for profile validation and generation.

use std::hint::black_box;

use bevy_ragdoll::RagdollProfile;
use criterion::{BatchSize, Criterion, criterion_group};

use super::support::{BENCH_SEED, chain_spec, human_profile};

/// Measures human and chain profile validation plus human profile generation.
fn profile_benchmarks(criterion: &mut Criterion) {
    let _human = human_profile();
    // Build inputs outside the timed closures so only validation and generation are measured.
    let skeleton = bevy_ragdoll::Skeleton::humanoid();
    let human_spec = bevy_ragdoll::ProfileSpec::from(&skeleton);
    let chain_spec = chain_spec(64, BENCH_SEED);

    // Validate a realistic humanoid and a worst-case 64-body chain.
    criterion.bench_function("profile/new/human_16", |bencher| {
        bencher.iter_batched(
            || human_spec.clone(),
            |spec| black_box(RagdollProfile::new(spec).expect("human profile validates")),
            BatchSize::SmallInput,
        );
    });
    criterion.bench_function("profile/new/chain_64", |bencher| {
        bencher.iter_batched(
            || chain_spec.clone(),
            |spec| black_box(RagdollProfile::new(spec).expect("chain profile validates")),
            BatchSize::SmallInput,
        );
    });
    criterion.bench_function("profile/generate/human_16", |bencher| {
        bencher.iter(|| {
            black_box(
                RagdollProfile::from_skeleton(black_box(&skeleton))
                    .expect("human profile generates"),
            )
        });
    });
}

criterion_group!(benches, profile_benchmarks);
