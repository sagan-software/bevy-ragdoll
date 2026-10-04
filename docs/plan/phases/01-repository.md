# Phase 1: repository, toolchain and CI

Goal: a public, empty-but-building workspace with a dev shell and green
CI.

Read first: [../decisions.md](../decisions.md),
[../architecture.md](../architecture.md) (Workspace section), and TGF's
`~/Code/gitlab.com/liamcurry/tgf/flake.nix` for the Bevy system-library
setup this machine needs.

## Steps

1. Create the GitHub repository (the owner approved this):
   `gh repo create sagan-software/bevy-ragdoll --public --description "Active ragdolls for Bevy on any skeleton and physics engine"`.
   Clone it to `~/Code/github.com/sagan-software/bevy-ragdoll` over SSH
   (`git@github.com:sagan-software/bevy-ragdoll.git`). Check
   `git config user.name` is `Bill Curry` and `user.email` is set; stop if
   not.
2. Write `flake.nix` with one dev shell:
   - Rust from `oxalica/rust-overlay`: stable, at least 1.95 (Bevy 0.19's
     MSRV), extensions `rust-src clippy rustfmt rust-analyzer
     llvm-tools-preview`, targets `wasm32-unknown-unknown wasm32-wasip1`.
   - Bevy libraries as in TGF's flake: `pkg-config alsa-lib udev
     vulkan-loader libxkbcommon wayland` and the X11 libraries, with
     `LD_LIBRARY_PATH` set for the runtime-loaded ones.
   - Tools: `cargo-llvm-cov cargo-deny critcmp samply perf tracy wasmtime
     wasm-bindgen-cli mold clang`.
   - `RUSTFLAGS` for mold only on Linux x86_64.
   Run `nix develop -c cargo --version` and `nix develop -c rustc --version`.
3. Write the root `Cargo.toml`: `[workspace]` with `resolver = "3"`,
   members `crates/bevy_ragdoll`, `examples`, `benches`;
   `[workspace.package]` (`version = "0.1.0-dev"`, `edition = "2024"`,
   `rust-version = "1.95"`, `license = "MIT OR Apache-2.0"`,
   `repository = "https://github.com/sagan-software/bevy-ragdoll"`);
   `[workspace.dependencies]` pinning `bevy = { version = "=0.19.1", default-features = false }`,
   `bevy_rapier3d = "=0.36.0"`, `avian3d = "=0.7.0"`, `serde`, `ron`,
   `thiserror`, `criterion = "0.8"`, `argh`, `rand_chacha`;
   `[workspace.lints]` from [../architecture.md](../architecture.md);
   `[profile.dev] opt-level = 1` and `[profile.dev.package."*"] opt-level = 3`
   (physics in debug builds is otherwise too slow to test);
   `[profile.stress-test]` from [../reference/stress-and-benches.md](../reference/stress-and-benches.md).
4. Create `crates/bevy_ragdoll` with `src/lib.rs`: crate docs (what it is,
   a usage sketch marked `ignore` until phase 4), and an empty
   `RagdollPlugin` that registers nothing yet. Verify Bevy 0.19's feature
   names in `~/Code/github.com/bevyengine/bevy/Cargo.toml` before choosing
   the core's bevy features (at least `std`, the asset, transform,
   animation and log crates; no rendering).
5. Create `examples/` (crate `bevy_ragdoll_examples`, `publish = false`,
   `src/lib.rs` empty with docs) and `benches/` (crate
   `bevy_ragdoll_benches`, `publish = false`, one placeholder bench that
   measures `RagdollPlugin` build time so the harness runs).
6. Add `LICENSE-MIT` (copyright 2026 Bill Curry), `LICENSE-APACHE` (the
   full Apache 2.0 text from https://www.apache.org/licenses/LICENSE-2.0.txt),
   `README.md` (one paragraph, the licence section Bevy crates use, and
   "status: in development"), `CHANGELOG.md`, `.gitignore` (`/target`,
   `/result`, `.direnv`), `assets/CREDITS.md` (empty table of contents),
   and `PLAN.md` with the phase list from this plan's README as a
   checklist plus a "Notes" section.
7. Add `deny.toml` from `cargo deny init`; allow MIT, Apache-2.0,
   Apache-2.0 WITH LLVM-exception, BSD-2-Clause, BSD-3-Clause, ISC, Zlib,
   Unicode-3.0, CC0-1.0, MPL-2.0, BSL-1.0. Stop and ask if a dependency
   needs anything else.
8. Add `.github/workflows/ci.yml`, on push and pull request, Ubuntu,
   `dtolnay/rust-toolchain@stable` with clippy and rustfmt,
   `Swatinem/rust-cache`, apt packages `libasound2-dev libudev-dev
   libwayland-dev libxkbcommon-dev`. Jobs: `fmt`
   (`cargo fmt --all -- --check`), `clippy`
   (`cargo clippy --workspace --all-targets --locked -- -D warnings`),
   `test` (`cargo test --workspace --locked`), `deny`
   (`EmbarkStudios/cargo-deny-action`). Later phases add jobs.
9. Copy this whole plan directory (`/mnt/expansion/Work/bevy-ragdoll-plan`)
   into the repository as `docs/plan/`. From now on, use and update that
   copy; links inside it are relative and keep working.
10. Commit ("Create the bevy-ragdoll workspace"), push, and watch CI with
   `gh run watch`.

## Gates

- `nix develop -c cargo fmt --all -- --check`
- `nix develop -c cargo clippy --workspace --all-targets -- -D warnings`
- `nix develop -c cargo test --workspace`
- `nix develop -c cargo deny check`
- The CI run for the pushed commit is green.

## Done when

- `https://github.com/sagan-software/bevy-ragdoll` is public with both
  licence files.
- All gates pass locally and in CI.
- `PLAN.md` marks phase 1 done with the CI run URL.
