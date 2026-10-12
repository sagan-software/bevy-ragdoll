# Changelog

All notable changes to this project are recorded here.

## Unreleased

- Recognize 3ds Max Biped style humanoid bone names, with a rig prefix and a
  middle side marker such as `Bip01 L Thigh`.
- Pick each humanoid limb chain from candidates that descend from one
  another, so helper bones named like limbs, such as IK targets, no longer
  hide the limb.
- Generate skinned skeletons from the bind pose in the skin's inverse bind
  matrices rather than from the current animated pose.
- Add `RagdollPhysicsSettings::soft_limit_frequency_hz`; zero turns off the
  fallback soft-limit torque.
- Avian: map joints with only a Z range to hinges about Z, and turn new
  bodies inside their joint limits before the first step, so a ragdoll that
  activates from an animated pose no longer launches.
- Create the initial workspace.
- Make `bevy_ragdoll` the root crate, replace the examples crate with
  standalone examples, and move benches to one Criterion `bench_main` target.
