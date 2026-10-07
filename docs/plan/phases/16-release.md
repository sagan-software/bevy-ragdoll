# Phase 16: documentation and release candidate

Goal: documentation a new user can follow, and a release candidate the
owner can publish.

## Steps

1. README: what the crate does (one paragraph), a screenshot or GIF from
   the showcase, a quick start for Rapier and for Avian (each a complete
   `main.rs` that compiles as a doctest or example), a list of backends and
   modes with their measured status, the performance summary
   from `benches/RESULTS.md` with the backend recommendation per scenario,
   the Bevy version table, the licence section, and asset credits.
2. Guide pages in `docs/`: automatic profiles, per-bone overrides (with
   `RagdollBone` in Blender through Skein, or a RON overrides file), active control (muscle, pin, blend), hits, balance
   and the puppet, writing a backend (from the mock backend and the
   contract), determinism, performance and profiling.
3. Every public item documented with an example where useful; run
   `cargo doc --workspace --no-deps` with `RUSTDOCFLAGS="-D warnings"`.
4. `CHANGELOG.md` entry for 0.1.0. Set `version = "0.1.0"` in the
   workspace. Run `cargo publish --dry-run` for each publishable crate in
   dependency order.
5. Final sweep and criterion run; commit results.
6. Tag `v0.1.0-rc.1` (annotated) and push the tag.

## Done when

- Docs build without warnings; README quick starts compile.
- Dry-run publishes pass.
- `PLAN.md` marks phase 16 done.

## Stop and ask

Publishing to crates.io needs the owner's explicit go. Report the
release candidate tag, the dry-run output and the README link.
