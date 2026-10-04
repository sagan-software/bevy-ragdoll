# Implementation progress

| Complete | Phase | Evidence |
| --- | --- | --- |
| [ ] | 1. Repository, toolchain and CI | Cargo, Dylints, and Markdown gates pass. GitHub creation, push and CI remain incomplete. |
| [ ] | 2. Profile data model and import | |
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
- Dylints came from
  `https://github.com/sagan-software/dylints` at commit
  `483b64d83e38352994d509eacf4a56db1892f1a3`. Run the Rust linter from that
  checkout with:

  ```sh
  CARGO_HOME=/var/mnt/nixsd/Caches/cargo \
  CARGO_TARGET_DIR=/var/mnt/nixsd/Build/bevy-ragdoll \
  CARGO_BUILD_JOBS=4 \
  SAGAN_LINTS_CACHE_DIR=/var/mnt/nixsd/Caches/dylints \
  nix develop -c cargo run --bin sagan-lints -- \
    --repo /home/deck/Code/github.com/sagan-software/bevy-ragdoll \
    --fast \
    --target-dir /var/mnt/nixsd/Build/bevy-ragdoll/dylints-target \
    --log-dir /var/mnt/nixsd/Caches/dylints/bevy-ragdoll
  ```

- The Dylints `sagan-lints` command passed after documenting the API and
  deriving `Clone`, `Copy`, and `Debug`. Its first run reported the intentionally
  pinned `bevy_transform` dependency as unused; `use bevy_transform as _;`
  records that direct dependency.
- Dylints pins `rumdl` 0.2.55. Its first check found eleven MD013 line-length
  findings in `README.md` and `PLAN.md`. After wrapping those files to 80
  columns, `rumdl check --no-config README.md CHANGELOG.md PLAN.md
  assets/CREDITS.md` passed with no findings in four files.
- The anonymous GitHub page check for `sagan-software/bevy-ragdoll` returned
  404. `gh auth status --hostname github.com` confirmed that no host is logged
  in. `gh repo view sagan-software/bevy-ragdoll` could not run without
  authentication. Repository creation, push, and CI remain incomplete.
