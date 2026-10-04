//! Criterion benchmark for building an app with the ragdoll plugin.

use bevy::app::App;
use bevy_ragdoll::RagdollPlugin;
use criterion::{Criterion, criterion_group, criterion_main};
use std::hint::black_box;

/// Measures app construction with [`RagdollPlugin`].
fn plugin_build_time(criterion: &mut Criterion) {
    criterion.bench_function("ragdoll_plugin_build", |bencher| {
        bencher.iter(|| {
            let mut app = App::new();
            app.add_plugins(RagdollPlugin);
            let _ = black_box(app);
        });
    });
}

criterion_group!(benches, plugin_build_time);
criterion_main!(benches);
