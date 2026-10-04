# Phase 15: deterministic mode, WebAssembly and the showcase site

Goal: a tested deterministic mode for the backends that support it, every
non-Jolt example running in a browser, and a GitHub Pages site with the
examples.

Read first: [../reference/physics-api.md](../reference/physics-api.md)
A1 and B1 (determinism and wasm features), Bevy's `examples/wasm/` and
`tools/build-wasm-example` in `~/Code/github.com/bevyengine/bevy`.

## Deterministic mode

1. Feature `deterministic` on each backend crate forwards the engine's
   `enhanced-determinism` and turns off `simd8`/`simd`. On Rapier it may
   keep `parallel` (rapier 0.35 is bitwise identical across thread
   counts); on Avian, test before claiming the same.
2. Core rules under `deterministic` (always true, documented): ragdolls
   are processed in `RagdollId` order, bodies in profile order; no
   `HashMap` iteration in fixed systems; no wall-clock time; `bevy_math`
   with `libm`.
3. `determinism_check` example: runs a fixed scenario (`pile 16` plus 20
   seeded hits, 600 fixed steps) and prints a SHA-256 of all body poses
   as little-endian `f32` bits in profile order.
4. Tests: same process twice gives the same hash; Rapier with
   `parallel` at 1 and 8 threads gives the same hash.
5. CI: a matrix job runs `determinism_check` on `ubuntu-latest`
   (x86_64), `ubuntu-24.04-arm` (aarch64) and under `wasmtime` for
   `wasm32-wasip1` (headless, no window), and a final job compares the
   three hashes. If Bevy's `MinimalPlugins` app does not build for
   `wasm32-wasip1`, run the wasm check in headless Chromium from the
   phase's site build instead and record the method. Rapier must match
   across all three. For Avian, record
   whether it matches; if not, set its `deterministic` capability to
   same-machine only and document it.

## WebAssembly

1. Build examples for `wasm32-unknown-unknown` with `wasm-bindgen-cli`
   matching the `wasm-bindgen` version in `Cargo.lock` (pin it in the
   flake). Disable `parallel` features for wasm.
2. `argh::from_env()` panics on wasm: use `Args::from_args(&[], &[])` on
   wasm as Bevy's `many_foxes` does.
3. Add an `xtask`-free build: a `cargo run -p bevy_ragdoll_examples --bin build_site`
   binary that builds the listed examples and writes `site/` with an
   `index.html` per example and a gallery page. Rust only; no shell or
   Node scripts.
4. GitHub Actions workflow `pages.yml` builds the site on pushes to `main`
   and deploys to GitHub Pages. Ask the owner before enabling Pages in the
   repository settings.

## Done when

- Determinism tests pass; the CI hash comparison is green for Rapier.
- `hit_reactions`, `puppet_showcase`, `creatures` and `ragdoll_stress`
  (small default) run in Chromium and Firefox from the built site; a
  screenshot of each in the built-in browser.
- `PLAN.md` marks phase 15 done.
