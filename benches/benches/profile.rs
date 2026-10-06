//! Criterion targets for profile validation and source parsing.

use std::hint::black_box;

use bevy_ragdoll::RagdollProfile;
use criterion::{BatchSize, Criterion, criterion_group, criterion_main};

use bevy_ragdoll_benches::support::{BENCH_SEED, chain_spec, human_profile};

/// Measures human and chain profile construction plus the human RON parser.
fn profile_benchmarks(criterion: &mut Criterion) {
    let _human = human_profile();
    let human_spec = ron::from_str::<bevy_ragdoll::ProfileSpec>(include_str!(
        "../../assets/profiles/tgf_human.ragdoll.ron"
    ))
    .expect("checked-in human RON parses");
    let chain_spec = chain_spec(64, BENCH_SEED);

    criterion.bench_function("profile/new/tgf_human_16", |bencher| {
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
    criterion.bench_function("profile/ron/tgf_human_16", |bencher| {
        bencher.iter(|| {
            let parsed = ron::from_str::<bevy_ragdoll::ProfileSpec>(black_box(include_str!(
                "../../assets/profiles/tgf_human.ragdoll.ron"
            )))
            .expect("checked-in human RON parses");
            black_box(parsed);
        });
    });
}

criterion_group!(benches, profile_benchmarks);
criterion_main!(benches);
