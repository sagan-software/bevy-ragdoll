# Implementation progress

| Complete | Phase | Evidence |
| --- | --- | --- |
| [x] | 1. Repository, toolchain and CI | Local workspace and gates complete. GitHub creation, authentication, push and CI are deferred at the owner's direction. |
| [x] | 2. Profile data model and import | Profile model, RON loader, Skein components and TGF GLB import implemented; all phase 2 gates pass. Coverage gaps are documented below and in the commit body. |
| [ ] | 3. Rapier powered-ragdoll spike | |
| [ ] | 4. Core runtime | |
| [ ] | 5. Rapier 3D backend | |
| [ ] | 6. Performance | |
| [ ] | 7. Active control | |
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
  `/home/deck/Code/github.com/sagan-software/dylints`, remote
  `https://github.com/sagan-software/dylints`, commit
  `2b4f80e272c413eda75de2cd0ce4fa9c7e713677`. Run the Rust linter from that
  checkout with:

  ```sh
  CARGO_HOME=/var/mnt/nixsd/Caches/cargo \
  CARGO_TARGET_DIR=/var/mnt/nixsd/Build/bevy-ragdoll \
  CARGO_BUILD_JOBS=4 \
  SAGAN_LINTS_CACHE_DIR=/var/mnt/nixsd/Caches/dylints \
  nix develop -c cargo run --bin sagan-lints -- \
    --repo /var/mnt/nixsd/Code/github.com/sagan-software/bevy-ragdoll \
    --fast \
    --target-dir /var/mnt/nixsd/Build/bevy-ragdoll/dylints-target \
    --log-dir /var/mnt/nixsd/Caches/dylints/bevy-ragdoll
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
