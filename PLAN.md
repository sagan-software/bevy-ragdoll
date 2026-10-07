# Implementation progress

| Complete | Phase | Evidence |
| --- | --- | --- |
| [x] | 1. Repository, toolchain and CI | Local workspace and gates complete. The public GitHub repository exists, and `main` has been pushed. |
| [x] | 2. Profile data model and import | Profile model, RON loader, Skein components and TGF GLB import implemented; all phase 2 gates pass. Coverage gaps are documented below and in the commit body. |
| [x] | 3. Rapier powered-ragdoll spike | [Report](docs/spikes/rapier-powered.md); checks and coverage gap recorded below. |
| [x] | 4. Core runtime | Runtime, mock backend, conformance tests and screenshots complete. Coverage gaps and reasons are recorded below. |
| [x] | 5. Rapier 3D backend | Workspace gates, backend coverage, headless smoke run, and reviewed screenshots pass. Coverage gaps are recorded below. |
| [x] | 6. Performance | The [stress baseline and Criterion results](benches/RESULTS.md) are committed; the default sweep, comparison boundaries, and benchmark gates pass. The all-features Clippy dependency error is recorded below. |
| [ ] | 7. Active control | Implementation, workspace tests, strict Clippy, and formatting pass. Fresh coverage, stress and Criterion results, and Rapier-feature example tests remain. See Phase 7 evidence below. |
| [ ] | 8. Human assets | |
| [ ] | 9. Balance | |
| [ ] | 10. Puppet showcase | |
| [ ] | 11. Creatures | |
| [ ] | 12. Avian 3D backend | |
| [ ] | 13. Jolt feasibility | |
| [ ] | 14. 2D | |
| [ ] | 15. Determinism, WebAssembly and showcase site | |
| [ ] | 16. Release | |

## Notes

- Phase 1 Cargo gates used `CARGO_HOME=/var/mnt/nixsd/Caches/cargo`,
  `CARGO_TARGET_DIR=/var/mnt/nixsd/Build/bevy-ragdoll`, and
  `CARGO_BUILD_JOBS=4`.
- `nix develop -c cargo --version` passed with Cargo 1.99.0
  (2026-08-27). `nix develop -c rustc --version` passed with rustc 1.99.0
  (2026-09-28).
- `nix develop -c cargo fmt --all -- --check` passed.
- `nix develop -c cargo clippy --workspace --all-targets -- -D warnings`
  passed.
- `nix develop -c cargo test --workspace` passed. Both libraries have zero
  unit tests; the phase 1 usage example is ignored until phase 4.
- All four phase 1 gates were rerun after the source and README edits with the
  stated SD-card environment and exited 0. Nix printed
  `error (ignored): opening file "/etc/nix/sentry-endpoint": Permission denied`;
  Cargo produced no warning diagnostics, and Cargo Deny completed without
  warnings.
- `nix develop -c cargo deny init` generated `deny.toml`. The owner approved
  adding `MIT-0` on 2026-10-04 because Bevy 0.19.1 depends on `encase`,
  `encase_derive`, and `encase_derive_impl` 0.12.1. The project license is
  `MIT OR Apache-2.0`.
- The first `nix develop -c cargo deny check` exited 4 on those `MIT-0`
  dependencies. The approved allowlist includes `MIT-0`; `bans.skip` records
  eight duplicate package versions in Bevy 0.19.1 and Criterion 0.8's
  dependency graph. The final Cargo Deny check passed without warnings.
- The Dylints checkout is
  `/var/mnt/nixsd/Code/github.com/sagan-software/dylints`, remote
  `https://github.com/sagan-software/dylints`, commit
  `2b4f80e272c413eda75de2cd0ce4fa9c7e713677`. Run its bundled Rust linter
  from that checkout with:

  ```sh
  PRIVATE_LINTS_ROOT=/var/mnt/nixsd/Code/github.com/sagan-software/dylints \
  SAGAN_LINTS_CACHE_DIR=/var/mnt/nixsd/Caches/dylints \
  CARGO_HOME=/var/mnt/nixsd/Caches/cargo \
  CARGO_TARGET_DIR=/var/mnt/nixsd/Build/bevy-ragdoll \
  CARGO_BUILD_JOBS=4 \
  nix develop /var/mnt/nixsd/Code/github.com/sagan-software/dylints -c \
    nix run /var/mnt/nixsd/Code/github.com/sagan-software/dylints -- \
      --repo /var/mnt/nixsd/Code/github.com/sagan-software/bevy-ragdoll \
      --fast
  ```

- The Dylints `sagan-lints` command passed after documenting the API and
  deriving `Clone`, `Copy`, and `Debug`. Its first run reported the intentionally
  pinned `bevy_transform` dependency as unused; `use bevy_transform as _;`
  records that direct dependency.
- Dylints includes the `rumdl_doc_comments` Rust lint. Standalone Markdown uses
  `rumdl` 0.2.55, installed under the SD-card Cargo home. Its first check found
  eleven MD013 line-length findings in `README.md` and `PLAN.md`. After
  wrapping those files to 80 columns, the Markdown check passed on all four
  files.
- The anonymous GitHub page check for `sagan-software/bevy-ragdoll` returned
  404. `gh auth status --hostname github.com` confirmed that no host is logged
  in. `gh repo view sagan-software/bevy-ragdoll` could not run without
  authentication. The owner later deferred repository creation, authentication,
  push, and CI until after the local implementation.

## Phase 2 evidence

- The public integration test gate first ran on the phase 1 baseline in a
  temporary worktree with the GLB fixture present:
  `nix develop -c cargo test -p bevy_ragdoll --test profile` exited 101.
  Rust reported unresolved `bevy_ragdoll::skein` and profile re-exports,
  missing `ron` and `serde_json` dev crates, and missing `JointAxis`.
  The captured compiler output reported 16 previous errors. The temporary
  worktree was removed after recording this baseline failure.
- These exact commands pass with the listed Cargo environment:

  ```sh
  nix develop -c cargo fmt --all -- --check
  nix develop -c cargo test --workspace
  nix develop -c cargo clippy --workspace --all-targets -- -D warnings
  nix develop -c cargo deny check
  nix develop -c cargo doc -p bevy_ragdoll --no-deps
  ```

  Cargo output and build artifacts are on the SD card.
  The test run passed 25 unit tests, 15 integration tests, and 39 doctests;
  one asset-generator test and one root doctest are intentionally ignored.
- The Dylints `sagan-lints --fast` run passes with the exact command above.
  This standalone Markdown command passes:

  ```sh
  PATH=/var/mnt/nixsd/Caches/cargo/bin:$PATH \
    rumdl check --no-config README.md CHANGELOG.md PLAN.md assets/CREDITS.md
  ```

  Dylints ran from the corrected GitHub checkout at commit
  `2b4f80e272c413eda75de2cd0ce4fa9c7e713677`; `rumdl` is version 0.2.55.
- The GLB importer follows the [Khronos glTF 2.0.1 specification, sections
  3.2 and 4.4](https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html).
  It accepts container version 2, exact `asset.version` `2.0`, and absent or
  exact `asset.minVersion` `2.0`. It rejects other minimum versions. It accepts
  `extensionsRequired` only when absent; it rejects non-arrays, empty arrays,
  non-string entries, and every nonempty list because this importer implements
  no extensions. It ignores `extensionsUsed` and optional extension members.
  Tests cover malformed version metadata, both minimum-version restrictions,
  optional extensions, required extensions, and GLB chunk ordering.
- `nix develop -c cargo llvm-cov -p bevy_ragdoll --summary-only` exits 0.
  Its line coverage is `gltf/rig/container.rs` 96.39%, `gltf/rig/mod.rs`
  98.96%, `profile/mod.rs` 99.09%, and `skein.rs` 93.02%; all other production
  files are at 100% line coverage.
- The coverage report lists `container.rs` lines 159 and 162 as uncovered.
  They are the `u32` to `usize` conversion failure arm, which cannot occur on
  this 64-bit target. The report lists 11 missed lines in `gltf/rig/mod.rs`
  and 3 in `profile/mod.rs`, but its `Uncovered Lines` section gives no source
  locations for them. LLVM warns that 11 functions have mismatched data, so
  these summary gaps cannot be targeted with additional behavioral tests from
  this report. `skein.rs` misses lines 23, 60 and 126, which hold derives for
  reflected components; its test covers reflected defaults, registration and
  component access, but derive attributes have no executable source counter.
  `profile/geometry/tests.rs:274` is test-only fallback panic code, and Rust
  standard-library coverage is outside this crate. LLVM does not map the
  reported misses in `gltf/rig/mod.rs` and `profile/mod.rs` to source lines, so
  this report cannot identify behavior-specific tests for those gaps.
- The refreshed coverage build used the configured SD-card cache and target
  directories. It passed with 79 GiB free. Nix printed the ignored
  `/etc/nix/sentry-endpoint` permission warning; Cargo emitted no warning
  diagnostics.

## Phase 3 evidence

- Report: [docs/spikes/rapier-powered.md](docs/spikes/rapier-powered.md).
  It links both actual Bevy window screenshots and the JSON reports for all
  matrix runs. The report recommends `AccelerationBased` joint motors.
- The phase's owner-review checkpoint was waived by the owner's instruction
  to continue through all phases. The local implementation continues to phase
  4; GitHub operations remain deferred.
- `cargo fmt --all -- --check` passed. `cargo test --workspace` passed 25
  library unit tests, 15 profile integration tests, and 39 doctests. One
  integration test and one doctest remain intentionally ignored.
- `cargo test -p bevy_ragdoll_examples --example spike_rapier_powered
  --features visual` passed all 10 example tests. The real visible run
  produced the screenshots listed in the report.
- `cargo build -p bevy_ragdoll_examples --example spike_rapier_powered
  --features visual` passed after the final source edits.
- `cargo clippy --workspace --all-targets --all-features -- -D warnings`
  passed. Its first run found two undocumented private enum fields and one
  collapsible conditional; all three findings were fixed before rerunning.
- `cargo deny check` passed all four checks. `cargo doc -p bevy_ragdoll
  --no-deps` passed. Dylints `sagan-lints --fast` passed after the shared
  Rapier workspace dependency disabled default features, matching the example
  dependency declaration.
- `rumdl check --no-config README.md CHANGELOG.md PLAN.md assets/CREDITS.md
  docs/spikes/rapier-powered.md` passed after the report was written.
- The stress build command
  `cargo build -p bevy_ragdoll_examples --example spike_rapier_powered
  --features rapier3d --profile stress-test` passed. The 1, 32, and 128
  ragdoll runs each completed 600 steps with zero unstable bodies. The 128
  ragdoll p95 was 10.6003 ms on this machine.
- `cargo llvm-cov -p bevy_ragdoll_examples --example
  spike_rapier_powered --features visual --summary-only` passed all 10 tests.
  Before the coverage build, the SD card had 61 GiB free. Its report listed
  core-library files but omitted `examples/spike_rapier_powered.rs`; the
  changed example has no measured line or branch coverage. This remains an
  unresolved coverage gap. The report's 49.20% region and 51.98% line totals
  cover only the files LLVM included and do not measure the example.
- No matrix row met both the mean joint-error and pelvis-drift thresholds.
  The best mean error was 3.270 degrees at count 128. The best pelvis drift
  was 0.1936 m in the single-ragdoll 6 Hz motor run.

## Phase 4 evidence

- The phase 4 invariant scratchpad is at
  `/tmp/bevy-ragdoll-phase04-invariants.md`.
- The required failing public runtime test ran first:
  `cargo test -p bevy_ragdoll --test runtime` exited 101 because runtime
  components, backend modules, and the configurable `RagdollPlugin` do not
  exist yet. This is the expected baseline failure.
- `cargo test -p bevy_ragdoll_conformance --test conformance` exited 101
  because the conformance crate has no `contract` or `mock` module yet. This
  is the expected baseline failure.
- `cargo fmt --all -- --check` passed after the runtime tests were added.
  The first check requested one import-order change. `cargo fmt --all` fixed
  it, and the exact check passed.
- `cargo test --workspace` passed 61 library unit tests, 15 profile tests,
  21 runtime integration tests, 8 conformance unit tests, and 7 conformance
  tests. The profile suite has one intentional ignored asset-generator test.
  Rustdoc passed 69 core examples and 8 conformance examples; one core
  doctest remains intentionally ignored.
- `cargo clippy --workspace --all-targets -- -D warnings` and
  `cargo clippy --all-targets --all-features -- -D warnings` passed.
  The first Clippy run found three test-code issues. The mock plugin's unit
  constructor, a `Copy` clone, and a nested conditional were fixed before
  both exact gates passed.
- `cargo deny check` passed advisories, bans, licenses, and sources.
  `cargo doc -p bevy_ragdoll --no-deps` passed.
- Dylints from
  `/var/mnt/nixsd/Code/github.com/sagan-software/dylints` passed `--fast` in
  132.73 seconds. The full logs are in
  `/var/mnt/nixsd/Caches/dylints/bevy-ragdoll-current`.
- The coverage gate passed:
  `cargo llvm-cov -p bevy_ragdoll -p bevy_ragdoll_conformance --text
  --show-missing-lines --output-path
  /var/mnt/nixsd/Build/bevy-ragdoll/phase4-coverage.txt`.
  It passed all 61 library unit, 15 profile, 21 runtime, 8 conformance
  unit, and 7 conformance tests. The target-pose capture and preservation
  branches both have direct unit tests.
- Remaining coverage lines have these reasons. Rust derive attributes in
  profile, runtime, and Skein modules emit generated code without executable
  source lines. `std::thread_local!` misses are outside this crate.
  `gltf/rig/container.rs:159,162` convert `u32` to `usize`; failure is
  unreachable on this 64-bit target. `profile/geometry/tests.rs:274` is a
  test-only fallback panic. `runtime/capture.rs:46,49` cannot occur after
  the query has required those components and the exclusive world borrow
  begins. `runtime/skeleton.rs:209` cannot occur after successful map
  construction while this system holds exclusive world access.
  `runtime/skeleton.rs:586` cannot occur because profile validation checks
  every joint index and body spawning returns one entity per body.
- The phase text names a `RagdollQuery` trait. The architecture instead
  defines backend-neutral raycast messages and `BodyContacts`, which the
  contract runner can use without a backend-specific system parameter type.
  The runtime follows the architecture contract; no query trait is needed.
- `cargo build -p bevy_ragdoll_examples --example custom_backend
  --features custom-backend` passed. The SD card had 83 GiB free before
  the build.
- Both `custom_backend` runs passed with `--exit-after 3 --screenshot`.
  The headless run used:

  ```sh
  nix develop -c bash -c '
    export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/radeon_icd.x86_64.json
    exec /lib64/ld-linux-x86-64.so.2 \
      --library-path "${LD_LIBRARY_PATH}:/usr/lib:/usr/lib64" \
      /var/mnt/nixsd/Build/bevy-ragdoll/debug/examples/custom_backend \
      --headless --exit-after 3 \
      --screenshot /var/mnt/nixsd/Build/bevy-ragdoll/screenshots/custom_backend_headless.png
  '
  ```

  The windowed run used:

  ```sh
  nix develop -c bash -c '
    export DISPLAY=:0 XAUTHORITY=/run/user/1000/xauth_wOWDgV \
      VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/radeon_icd.x86_64.json
    exec /lib64/ld-linux-x86-64.so.2 \
      --library-path "${LD_LIBRARY_PATH}:/usr/lib:/usr/lib64" \
      /var/mnt/nixsd/Build/bevy-ragdoll/debug/examples/custom_backend \
      --exit-after 3 \
      --screenshot /var/mnt/nixsd/Build/bevy-ragdoll/screenshots/custom_backend_windowed.png
  '
  ```

  The headless screenshot is
  [docs/screenshots/phase-04-custom-backend-headless.png](docs/screenshots/phase-04-custom-backend-headless.png).
  The windowed screenshot is
  [docs/screenshots/phase-04-custom-backend-windowed.png](docs/screenshots/phase-04-custom-backend-windowed.png).
  I reviewed both images. Each shows the pelvis, chest, and head shapes at
  the authored rest pose. The windowed capture also shows the mock-backend
  label.
- Runtime commands used the system Radeon ICD and host loader because the
  Nix Mesa selector reported a missing `GLIBC_ABI_GNU2_TLS` symbol against
  the system glibc. RADV VANGOGH rendered both images, and both runs exited
  successfully.

## Phase 5 evidence

- `nix develop -c cargo fmt --all -- --check` passed.
- `nix develop -c cargo test --workspace` passed all workspace unit,
  integration, and documentation tests: 164 unit and integration tests and
  97 documentation tests passed. The asset-generator doctest and the stairs
  physics test remain ignored for the documented reasons.
- `nix develop -c cargo test -p bevy_ragdoll_examples --features rapier3d`
  passed all 7 feature-gated example tests.
- `nix develop -c cargo clippy --workspace --all-targets -- -D warnings`
  passed. `nix develop -c cargo deny check` passed advisories, bans, licenses,
  and sources.
- `nix develop -c cargo llvm-cov -p bevy_ragdoll_rapier3d --summary-only`
  passed 20 backend unit tests and 24 physics integration tests, with one
  ignored stairs case. The backend line summary is 97.74% (25 of 1,105 lines
  missed). LLVM reported two functions with mismatched coverage data.
- The report maps these uncovered backend lines: `body.rs:156` skips an
  added shape without the required body components; `body.rs:189` handles an
  owner with no spawn-lift entry; `body.rs:606,632` are fallback panics after
  shape assertions; `contact.rs:69,71` reject static-static pairs;
  `query.rs:60,91,107` handle a missing Rapier context, a missing collider
  mapping, and a pair endpoint mismatch. `plugin.rs:156` is a line-counter
  miss in fixed-schedule validation. `joint.rs:13`, `plugin.rs:104`, and
  `settings.rs:10` are declaration or derive lines. LLVM did not map one
  missed `shape.rs` line or all remaining missed lines because of its
  mismatched-function warning. The defensive missing-component and missing
  context paths, impossible pair mismatch, and expected-shape fallback panics
  have no direct behavioral tests.
- The conformance source lines in the report are not a complete conformance
  crate coverage run. This command selected the Rapier crate. The stair helper
  lines remain unexecuted because the stairs test is ignored. With
  `joint_friction` 0.05, the body slides 1.1 m and still moves at 4 cm/s
  4.5 s after landing. At 0.1, it stops but tears joints in the determinism
  drop test.
- The coverage command does not measure `bevy_ragdoll_examples`. The 7
  profile and CLI tests pass, and the four examples have reviewed screenshots,
  but no example line-coverage report was collected. A separate instrumented
  visual build was skipped to protect the SD-card free-space reserve.
- `nix develop -c cargo run -p bevy_ragdoll_examples --example minimal
  --features rapier3d -- --headless --exit-after 3` passed. I reviewed the
  rendered screenshots [minimal](docs/screenshots/phase-05-minimal.png),
  [windowed minimal](docs/screenshots/phase-05-minimal-windowed.png),
  [from code](docs/screenshots/phase-05-from-code.png),
  [from RON](docs/screenshots/phase-05-from-ron.png), and
  [from glTF Skein](docs/screenshots/phase-05-from-gltf-skein.png). Each shows
  the ragdoll at or after landing with the corresponding example label.
- `nix develop -c cargo doc --workspace --no-deps` passed without warnings
  after the broken `AddBackend` intra-doc link was changed to its crate path.
- The final Markdown lint passed:

  ```sh
  PATH=/var/mnt/nixsd/Caches/cargo/bin:$PATH \
    rumdl check --no-config README.md CHANGELOG.md PLAN.md \
      assets/CREDITS.md docs/spikes/rapier-powered.md
  ```

- The exact Dylints `sagan-lints --fast` command passed after the final Rust
  source edit. Its logs are in
  `/var/mnt/nixsd/Caches/dylints/bevy-ragdoll`. GitHub creation,
  authentication, and push remain deferred.

## Phase 6 evidence

- The default stress sweep passed with the planned Rapier grid, pile, and
  powered configurations:

  ```sh
  nix develop -c cargo run -p bevy_ragdoll_examples \
    --example ragdoll_stress --profile stress-test --features rapier3d -- \
    --headless --sweep default
  ```

  It wrote eight schema-one reports to
  `/var/mnt/nixsd/Build/bevy-ragdoll/stress/2026-10-06-0a2a3d4.json`.
  Every row reported zero unstable bodies. The committed host baseline is
  `benches/baselines/steamdeck-6dffb94acbac45d38b162ddde8b7d645.json`.
  `benches/RESULTS.md` records the report table, host details, and Criterion
  measurements.
- The matched 16×16 comparison passed with `--profile stress-test`. Candidate
  frame and step p95 were 97.552 ms and 90.525 ms, below the baseline values
  97.730 ms and 90.713 ms. The first dev-profile comparison exited 1 because
  that profile exceeded the stress-test baseline by more than 10 percent.
  The README now uses the baseline's profile.
- The comparison against a temporary baseline with 20 percent lower 16×16
  timings exited 1 as expected. It reported frame p95 97.535 ms over the
  78.184 ms limit and step p95 90.491 ms over the 72.570 ms limit.
- The CI stress-smoke command and its JSON assertion passed. Four characters
  spawned 64 bodies and reported zero unstable bodies.
- The exact Criterion command passed with LLD selected for linking:

  ```sh
  CARGO_HOME=/var/mnt/nixsd/Caches/cargo \
  CARGO_TARGET_DIR=/var/mnt/nixsd/Build/bevy-ragdoll \
  CARGO_BUILD_JOBS=4 \
  nix develop -c env 'RUSTFLAGS=-C link-arg=-fuse-ld=lld' \
    cargo bench -p bevy_ragdoll_benches -- --save-baseline phase6
  ```

  The unmodified Nix `mold` selection failed to link `alloca.o`, which is LLVM
  bitcode. The LLD run exited 0 and saved the Criterion baseline under the
  SD-card target directory. Criterion printed four sample-window warnings
  for writeback of one character and asleep Rapier cases of 32, 128, and 512
  characters. Each target completed its requested samples; the full command
  exited 0. The sample windows and mean estimates are recorded in
  `benches/RESULTS.md`.
- These gates passed after the final source and documentation edits:

  ```sh
  nix develop -c cargo fmt --all -- --check
  nix develop -c cargo test --workspace
  nix develop -c cargo clippy --workspace --all-targets --locked -- \
    -D warnings
  nix develop -c cargo deny check
  nix develop -c cargo doc --workspace --no-deps
  ```

- Dylints at commit `2b4f80e272c413eda75de2cd0ce4fa9c7e713677` passed its
  bundled runner. Its first direct invocation lacked `cc`; the Nix shell
  supplied the linker. Its next invocation required `PRIVATE_LINTS_ROOT`.
  Setting that variable to the local Dylints checkout produced this passing
  command:

  ```sh
  PRIVATE_LINTS_ROOT=/var/mnt/nixsd/Code/github.com/sagan-software/dylints \
  SAGAN_LINTS_CACHE_DIR=/var/mnt/nixsd/Caches/dylints/phase6 \
  CARGO_HOME=/var/mnt/nixsd/Caches/cargo \
  CARGO_TARGET_DIR=/var/mnt/nixsd/Build/bevy-ragdoll \
  CARGO_BUILD_JOBS=4 \
  nix develop /var/mnt/nixsd/Code/github.com/sagan-software/dylints -c \
    nix run /var/mnt/nixsd/Code/github.com/sagan-software/dylints -- \
      --repo /var/mnt/nixsd/Code/github.com/sagan-software/bevy-ragdoll \
      --fast
  ```

  Dylints strict Clippy and `cargo check` both exited 0.
- The fallback command
  `nix develop -c cargo clippy --all-targets --all-features -- -D warnings`
  remains blocked in Bevy 0.19.1 `bevy_reflect`. Rapier enhanced determinism
  selects scalar math, and Bevy's `BVec3A` and `BVec4A` Serde registrations
  fail because those implementations are unavailable in that feature graph.
  The errors originate in the dependency before workspace crates are linted.
- No LLVM coverage report was collected for the new Criterion targets or the
  stress runner. Criterion and the exact stress sweep exercised those paths,
  but those runs are not line-coverage evidence. An instrumented workspace
  build was not attempted because its additional size was not bounded below
  the remaining SD-card space above the 40 GiB reserve.
- I reviewed
  [the windowed 16×16 grid screenshot](docs/screenshots/phase-06-grid-16.png).
  The ragdolls stand above the ground plane, with no body below the floor.
  At least 72 GiB remained free on the SD card during the sweep and benchmark
  runs.

## Phase 7 evidence

- The relationship-based hit-tree regression test was added before changing
  collection. Its first focused build failed because `HitImpulseTree::collect`
  still accepted only two arguments. After the collector switched to the
  character's `RagdollBodies` relationship, the focused test passed.
- `cargo test --locked -p bevy_ragdoll` passed 89 tests; one test was ignored.
  `cargo test --locked --workspace` passed after the collector and recovery
  changes, including the workspace doctests.
- `cargo clippy --locked --workspace --all-targets -- -D warnings` and
  `cargo clippy --locked -p bevy_ragdoll_examples --all-targets --features
  rapier3d -- -D warnings` passed.
- The final `cargo fmt --all -- --check` command passed after Nix's SD-card
  read completed. `git diff --check` passed.
- `cargo test --locked -p bevy_ragdoll_examples --features rapier3d` was
  stopped with exit 130 at 41 GiB free while several example linkers were
  active. The test suite did not finish. No files were deleted.
- Before the relationship-based collector change, the hit benchmark measured
  3.366 µs for one ragdoll and 529.642 µs for 64 ragdolls. Criterion reported
  local comparisons of +10.7% and +59.2%, but those saved comparisons have no
  source revision and are not a Phase 7 baseline. Rerun the benchmark against
  the final source revision. The pre-optimization command was:

  ```sh
  nix develop --command env \
    CARGO_HOME=/var/mnt/nixsd/Caches/cargo \
    CARGO_TARGET_DIR=/var/mnt/nixsd/Build/bevy-ragdoll \
    CARGO_BUILD_JOBS=4 \
    'RUSTFLAGS=-C link-arg=-fuse-ld=lld' \
    cargo bench --locked -p bevy_ragdoll_benches --bench math -- \
      math/hit_processing --noplot --sample-size 10 --measurement-time 5
  ```

- The prior `llvm-cov` report predates the collector and recovery changes. Its
  only reported core misses were derive-generated lines 222, 278, and 321;
  Rapier conformance line 144 was failure-only panic formatting. This report
  does not cover the latest code. Rerun coverage after space permits.
- A four-character headless shooting smoke run passed before the collector
  change with 64 bodies and zero unstable bodies. Its command was:

  ```sh
  nix develop --command env \
    CARGO_HOME=/var/mnt/nixsd/Caches/cargo \
    CARGO_TARGET_DIR=/var/mnt/nixsd/Build/bevy-ragdoll \
    CARGO_BUILD_JOBS=4 \
    cargo run --locked -p bevy_ragdoll_examples \
      --example ragdoll_stress --features rapier3d -- \
      --headless --scenario shooting --count 4 --trigger-at 0 \
      --duration 0.5 --warmup 0 \
      --report /var/mnt/nixsd/Build/bevy-ragdoll/stress/phase7-shooting-final-smoke.json
  ```

  Rerun it and the default sweep command below against the final source
  revision:

  ```sh
  nix develop --command env \
    CARGO_HOME=/var/mnt/nixsd/Caches/cargo \
    CARGO_TARGET_DIR=/var/mnt/nixsd/Build/bevy-ragdoll \
    CARGO_BUILD_JOBS=4 \
    cargo run -p bevy_ragdoll_examples --example ragdoll_stress \
      --profile stress-test --features rapier3d -- --headless --sweep default
  ```

- I reviewed the Phase 7
  [hit-reactions screenshot](docs/screenshots/phase-07-hit-reactions.png) and
  [partial-ragdoll screenshot](docs/screenshots/phase-07-partial-ragdoll.png).
  The first shows the torso leaning after a rifle hit; the second shows the
  upper body yielding while the legs remain controlled.
- Dylints, Cargo Deny, Cargo documentation, Criterion, and LLVM coverage passed
  or ran before the final collector change. Rerun applicable gates before
  marking Phase 7 complete.
