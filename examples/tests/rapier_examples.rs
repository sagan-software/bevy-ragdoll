//! Public checks for the profile sources used by the Rapier examples.

#![cfg(feature = "rapier3d")]

use bevy_ragdoll_examples::{ExampleKind, load_profile};

/// Every documented Rapier example constructs a validated nonempty profile.
#[test]
fn each_basic_example_loads_a_valid_profile() {
    for kind in [
        ExampleKind::Minimal,
        ExampleKind::FromCode,
        ExampleKind::FromRon,
        ExampleKind::FromGltfSkein,
    ] {
        let profile = load_profile(kind).expect("example profile should validate");
        assert!(!profile.bodies().is_empty());
    }
}

/// The code example retains the documented three-body chain.
#[test]
fn code_example_builds_a_three_body_chain() {
    let profile = load_profile(ExampleKind::FromCode).expect("code profile should validate");
    assert_eq!(profile.bodies().len(), 3);
    assert_eq!(profile.joints().len(), 2);
}
