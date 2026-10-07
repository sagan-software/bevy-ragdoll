# Phase 2: the ragdoll profile

Goal: `RagdollProfile`, its validation, its builder and its RON form, all
tested without a physics engine.

Read first: [../architecture.md](../architecture.md) (Data model),
[../reference/algorithms.md](../reference/algorithms.md) section 1.

Profiles come from automatic generation (phase 11), `ProfileBuilder`, or a
`ProfileSpec` in RON. There are no authored full-profile files and no
Skein-specific components. Sparse per-bone overrides (a reflected
`RagdollBone` component or a short RON overrides file) apply on top of a
generated profile.

## Steps

1. Write the failing integration tests in `tests/profile.rs` (list below).
   Run `cargo test --test profile` and keep the failure output in the
   commit body.
2. Create the modules, one primary type per file, each starting with a
   `//!` comment: `profile/mod.rs` (`RagdollProfile`, `BodyIndex`),
   `profile/spec.rs` (`ProfileSpec`, `BodySpec`, `JointSpec`,
   `ShapeSpec`), `profile/limits.rs` (`AngleRange`, `JointLimits`),
   `profile/error.rs` (`ProfileError`), `profile/builder.rs`
   (`ProfileBuilder`), `profile/geometry` (`segment_distance`, private).
   Keep `lib.rs` a facade of private modules and re-exports.
3. Implement validation in the order of `ProfileError` in the
   architecture. Derive `no_contact` and `children` masks.
4. Register reflected types in `RagdollPlugin`.
5. Add internal `#[cfg(test)]` modules for private branches the public
   tests do not reach (segment distance degenerate cases, quaternion sign
   flip).

## Tests (`tests/profile.rs` unless noted)

- `human_has_sixteen_bodies_and_eighty_kilograms`: the profile generated
  from the built-in reference humanoid skeleton; root is `pelvis`; total
  mass 80.0.
- `knees_are_hinges_that_bend_backward` and `hips_bend_forward`.
- `each_error_variant_is_reported`: one spec per `ProfileError` variant,
  each changing exactly one field of a valid spec, asserting the exact
  variant. Also a spec with two problems asserts the earlier variant in
  validation order.
- `no_contact_holds_neighbours_and_touching_capsules`: joint pairs and the
  pairs closer than 0.01 m at rest are set; a far pair is clear.
- `ron_round_trip_is_exact`: spec to RON to spec equals.
- `builder_matches_spec`: a 3-body chain from `ProfileBuilder` equals the
  hand-written spec.
- `joint_angles_round_trip_each_axis` (algorithms.md section 1).

## Gates

The README gates, plus
`nix develop -c cargo llvm-cov --summary-only`: every new production file
at 100 % line coverage, or a written reason per gap in the commit body.

## Done when

- Every test above passes; the coverage gate holds.
- `cargo doc --no-deps` builds with no warnings.
- `PLAN.md` marks phase 2 done.
