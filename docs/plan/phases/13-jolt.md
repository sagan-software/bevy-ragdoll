# Phase 13: Jolt feasibility

Goal: a written, measured decision basis for a Jolt backend. This phase
writes no backend unless the owner says so afterwards.

Known facts (checked 2026-10-03):

- Jolt 5.6.0 has `Ragdoll`, `RagdollSettings`, `SwingTwistConstraint` with
  motors, `DriveToPoseUsingMotors` and `DriveToPoseUsingKinematics`.
- The Rust bindings `joltc-sys` and `rolt` 0.3.1 bind Jolt 5.0.0, were
  last released in May 2024, and expose no constraints in the release;
  `main` of SecondHalfGames/JoltC adds Hinge and SixDOF with motors but no
  SwingTwist, Ragdoll or Skeleton.
- They build C++ with CMake and bindgen (needs libclang) and will not
  build for `wasm32-unknown-unknown`.
- No Bevy integration exists.

## Steps

1. Recheck the facts above on crates.io and GitHub (SecondHalfGames
   JoltC and jolt-rust, any newer Jolt binding crates, any Bevy Jolt
   plugin). Bound each request to 30 s.
2. Build `rolt` from `main` in a scratch crate outside the workspace inside
   `nix develop` (add `cmake` and `llvmPackages.libclang` to the shell for
   this). Record whether it builds and how long.
3. Prototype in the scratch crate: two capsules joined by a SixDOF
   constraint with angular limits and motors, stepped 600 times; and, if
   SixDOF suffices, the TGF human from its RON profile in a Jolt
   `PhysicsSystem` with motors toward the rest pose, measuring ms per step
   for 1, 32 and 128 ragdolls.
4. Write `docs/jolt-feasibility.md`: what builds, what the bindings lack
   for ragdolls (SwingTwist, Ragdoll, collision groups per pair), the
   C++ wrapper code that would need to be written upstream in JoltC and
   its size estimate, the measured step cost against the Rapier and Avian
   baselines for the same counts, the wasm consequence, the maintenance
   risk, and a recommendation.

## Done when

- `docs/jolt-feasibility.md` is committed with measurements or the exact
  build failure output.

## Stop and ask

Stop. The owner decides whether to fund JoltC work, use SixDOF only,
wait, or drop Jolt. Do not start a `bevy_ragdoll_jolt` crate without that
decision.
