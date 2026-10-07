# bevy-ragdoll implementation plan

This plan builds `bevy-ragdoll`: an open-source Bevy 0.19 crate family for
active ragdolls on any skeleton, with physics-engine backends, a balance
and hit-reaction layer, benchmarks, a stress-test example and a
Euphoria-style showcase. An implementing agent follows it phase by phase.
The owner is Bill Curry (also known as Liam Curry).

## How to use this plan

0. Phase 1 copies this plan into the repository as `docs/plan/`. After
   phase 1, read and update that copy.
1. Read [decisions.md](decisions.md) and [architecture.md](architecture.md)
   once, in full, before phase 1. They are the contract every phase builds
   on.
2. Do the phases in order. Open only the current phase file. Each phase
   file lists its inputs, its numbered steps, its tests, its gates and its
   done criteria.
3. A phase is done only when every item under "Done when" holds and every
   gate command passed after the last edit. Record the result in the
   repository's `PLAN.md` progress table (phase 1 creates it) in the same
   commit that completes the phase.
4. When a phase says "Stop and ask", stop, write what you found in
   `PLAN.md`, and report to the owner. Do not guess past a stop.

## Phases

1. [phases/01-repository.md](phases/01-repository.md): Public repo, workspace, toolchain, CI.
2. [phases/02-profile.md](phases/02-profile.md): `RagdollProfile` data model, RON loader, glTF import.
3. [phases/03-spike.md](phases/03-spike.md): Rapier powered-ragdoll spike; owner review.
4. [phases/04-core-runtime.md](phases/04-core-runtime.md): Skeleton binding, targets, writeback, mock backend.
5. [phases/05-rapier3d.md](phases/05-rapier3d.md): Rapier 3D backend, physics conformance suite.
6. [phases/06-performance.md](phases/06-performance.md): Stress example, criterion benches, first baselines.
7. [phases/07-active-control.md](phases/07-active-control.md): Muscle and pin drives, hits, blending, examples.
8. [phases/08-human-assets.md](phases/08-human-assets.md): Animated humanoid character and its profile.
9. [phases/09-balance.md](phases/09-balance.md): `bevy_ragdoll_balance`: measuring, pelvis pin, stepping.
10. [phases/10-puppet-showcase.md](phases/10-puppet-showcase.md): Fall, get-up, return; Euphoria-style showcase.
11. [phases/11-creatures.md](phases/11-creatures.md): Automatic profiles; fox, dinosaur, crocodile, rabbit.
12. [phases/12-avian3d.md](phases/12-avian3d.md): Avian 3D backend.
13. [phases/13-jolt.md](phases/13-jolt.md): Jolt feasibility; owner decision.
14. [phases/14-2d.md](phases/14-2d.md): 2D core, Rapier 2D and Avian 2D backends.
15. [phases/15-determinism-wasm.md](phases/15-determinism-wasm.md): Deterministic mode, WebAssembly, showcase site.
16. [phases/16-release.md](phases/16-release.md): Documentation pass and release candidate.

## Reference

Open these when a phase points at them:

- [reference/physics-api.md](reference/physics-api.md): exact bevy_rapier3d
  0.36 and avian3d 0.7 APIs with source citations and pitfalls.
- [reference/algorithms.md](reference/algorithms.md): joint-angle maths,
  drives, interpolation, writeback, automatic profile generation.
- [reference/hit-reaction.md](reference/hit-reaction.md): hit reaction,
  balance, stepping, falling, get-up, returning, state machine, acceptance
  numbers.
- [reference/stress-and-benches.md](reference/stress-and-benches.md): the
  stress example's command line, scenarios, metrics, report format, and the
  criterion benchmark list.
- [reference/examples.md](reference/examples.md): every example, what it
  shows, and how to check it.
- [reference/assets.md](reference/assets.md): licensed asset sources,
  download URLs, bone names, clips, attribution.

## Rules for every phase

- Work in `~/Code/github.com/sagan-software/bevy-ragdoll`. Build and run
  everything inside its `nix develop` shell (phase 1 creates `flake.nix`).
  The host's rustup linker is broken on this machine.
- Write the failing test first, then the code (the `tdd` skill). Each
  changed behaviour gets a test. Aim for full coverage of changed lines; a
  gap needs a written reason in the commit body.
- Make one described change per commit. The subject names the change; the
  body says what was verified and how. Commit as the configured Git
  identity (Bill Curry). Add no co-author, AI attribution or trailer lines.
- Gates after every change, inside `nix develop`:
  `cargo fmt --all -- --check`,
  `cargo clippy --workspace --all-targets -- -D warnings`,
  `cargo test --workspace`. A phase may add gates; run those too. A
  warning counts as a failure.
- Push to `origin main` after the gates pass:
  `GIT_SSH_COMMAND='ssh -F /dev/null -o BatchMode=yes' git push origin main`.
  Never force-push.
- Document every public item and every private item with `///` or `//!`.
  Put a guide comment before each nontrivial maths, ordering or policy
  stage.
- Use Bevy 0.19 idioms: small components, small systems, public
  `SystemSet` enums, `Message` for buffered streams, observer `Event`s for
  one-off notifications, relationships for owner links, `#[require]` for
  required components, `Reflect` on every public component and resource.
- Keep tuning numbers in one `Default` impl per feature, loadable from RON.
  Units go in field names or docs: metres, kilograms, seconds, radians,
  newtons, newton-metres.
- For anything visible, run the example, take a screenshot (Bevy's
  `Screenshot` or the example's `--screenshot` flag), look at it at full
  resolution, and list its defects in the commit body.
- Use `liamc-lints` on changed Rust and Markdown before handing a phase
  over; fix every finding in changed lines.
- Third-party content needs a recorded licence in `assets/CREDITS.md`
  before it is committed. Only CC0, CC-BY (with attribution), MIT or
  Apache-2.0 content may be committed.
