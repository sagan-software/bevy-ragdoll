{
  description = "Development environment and CI checks for bevy-ragdoll";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-26.05";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    crane.url = "github:ipetkov/crane";
    treefmt-nix = {
      url = "github:numtide/treefmt-nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    # RustSec database for the offline cargo-deny advisories check. CI passes
    # `--override-input advisory-db github:rustsec/advisory-db` to check the
    # latest advisories without updating flake.lock.
    advisory-db = {
      url = "github:rustsec/advisory-db";
      flake = false;
    };
    # Sagan Dylint libraries, driver, and the nightly toolchain they need.
    # Keep this revision equal to `workspace.metadata.dylint` in Cargo.toml.
    dylints.url = "github:sagan-software/dylints/v0.2.1";
  };

  outputs =
    {
      self,
      nixpkgs,
      rust-overlay,
      crane,
      treefmt-nix,
      advisory-db,
      dylints,
      ...
    }:
    let
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "x86_64-darwin"
        "aarch64-darwin"
      ];
      forAllSystems = nixpkgs.lib.genAttrs systems;
      perSystem =
        system:
        let
          pkgs = import nixpkgs {
            inherit system;
            overlays = [ rust-overlay.overlays.default ];
          };
          inherit (pkgs) lib;
          inherit (pkgs.stdenv.hostPlatform) isLinux isx86_64;

          # rust-toolchain.toml pins the stable toolchain; Dylint alone uses nightly.
          rust = pkgs.rust-bin.fromRustupToolchainFile ./rust-toolchain.toml;
          # The development shell adds rust-analyzer and the WASI target.
          devRust = rust.override {
            extensions = [
              "rust-src"
              "clippy"
              "rustfmt"
              "rust-analyzer"
              "llvm-tools-preview"
            ];
            targets = [
              "wasm32-unknown-unknown"
              "wasm32-wasip1"
            ];
          };
          craneLib = (crane.mkLib pkgs).overrideToolchain rust;

          # Bevy links audio, input, windowing, and graphics libraries on Linux.
          runtimeLibraries = lib.optionals isLinux (
            with pkgs;
            [
              vulkan-loader
              libxkbcommon
              wayland
              libx11
              libxcursor
              libxi
              libxrandr
              alsa-lib
              udev
              libGL
            ]
          );
          buildLibraries = lib.optionals isLinux (
            with pkgs;
            [
              alsa-lib
              udev
              wayland
              libxkbcommon
            ]
          );

          # Cargo inputs only, so documentation and CI edits do not rebuild Rust.
          # Paths are optional to survive workspace layout changes.
          src = lib.fileset.toSource {
            root = ./.;
            fileset = lib.fileset.unions (
              [
                (craneLib.fileset.commonCargoSources ./.)
                (lib.fileset.fileFilter (file: file.hasExt "ron") ./.)
              ]
              ++ map lib.fileset.maybeMissing [
                ./assets
                ./benches
                ./crates
                ./examples
                ./src
                ./tests
                ./README.md
              ]
            );
          };

          manifest = builtins.fromTOML (builtins.readFile ./Cargo.toml);
          commonArgs = {
            inherit src;
            pname = "bevy-ragdoll";
            version = manifest.workspace.package.version or "0.0.0";
            strictDeps = true;
            nativeBuildInputs = [ pkgs.pkg-config ];
            buildInputs = buildLibraries;
            # CI checks use the dev profile, like local `cargo test`.
            CARGO_PROFILE = "dev";
            cargoExtraArgs = "--locked --workspace";
          };
          cargoArtifacts = craneLib.buildDepsOnly commonArgs;

          treefmt = treefmt-nix.lib.evalModule pkgs ./treefmt.nix;

          # wasm-bindgen-cli must equal the `wasm-bindgen` crate version in Cargo.lock.
          # After a lock bump, update the version and both hashes; the assert names the new version.
          wasmBindgenVersion = "0.2.129";
          lockedWasmBindgen =
            (lib.findFirst (package: package.name == "wasm-bindgen") {
              version = "missing";
            } (builtins.fromTOML (builtins.readFile ./Cargo.lock)).package).version;
          wasmBindgenCli =
            assert lib.assertMsg (
              lockedWasmBindgen == wasmBindgenVersion
            ) "flake.nix wasm-bindgen-cli ${wasmBindgenVersion} does not match Cargo.lock ${lockedWasmBindgen}";
            pkgs.buildWasmBindgenCli rec {
              src = pkgs.fetchCrate {
                pname = "wasm-bindgen-cli";
                version = wasmBindgenVersion;
                hash = "sha256-pcecKQd7E8Opw6bkFoE569epUi7gh5qpQF1e5PJY6V8=";
              };
              cargoDeps = pkgs.rustPlatform.fetchCargoVendor {
                inherit src;
                inherit (src) pname version;
                hash = "sha256-vmUrWVU7kPJJxO5qIVeAkwQyWDELO1Z4Z5gitz2kco8=";
              };
            };

          # Dylint runs the workspace through the nightly compiler that built the lints.
          dylintSupported = dylints.packages ? ${system};
          dylintCargoMetadata = manifest.workspace.metadata.dylint.libraries or [ ];
          dylintCheck =
            assert lib.all (
              entry:
              entry.git == "https://github.com/sagan-software/dylints"
              && entry.rev or null == dylints.rev
              && entry.pattern or [ ] == [ "lints" ]
            ) dylintCargoMetadata;
            craneLib.mkCargoDerivation (
              commonArgs
              // {
                pnameSuffix = "-dylint";
                cargoArtifacts = null;
                doInstallCargoArtifacts = false;
                buildPhaseCargoCommand = ''
                  # Put the pinned nightly, rustup shim, cargo-dylint, and driver on PATH.
                  ${dylints.devShells.${system}.default.shellHook}
                  # The hook links lint libraries with dylint-link; this check links nothing new.
                  unset ${
                    "CARGO_TARGET_"
                    + lib.toUpper (lib.replaceStrings [ "-" ] [ "_" ] pkgs.stdenv.hostPlatform.rust.rustcTarget)
                    + "_LINKER"
                  }
                  export DYLINT_RUSTFLAGS="-D warnings"
                  library=$(echo ${dylints.packages.${system}.lint-libraries}/lib/libsagan_lints@*)
                  cargo dylint --no-metadata --no-build --lib-path "$library" \
                    --workspace -- --all-targets --locked
                '';
                installPhaseCommand = "mkdir -p $out";
              }
            );

          coverage = craneLib.mkCargoDerivation (
            commonArgs
            // {
              pnameSuffix = "-coverage";
              # Instrumented builds cannot reuse the uninstrumented dependency artifacts.
              cargoArtifacts = null;
              doInstallCargoArtifacts = false;
              nativeBuildInputs = commonArgs.nativeBuildInputs ++ [
                pkgs.cargo-llvm-cov
                pkgs.jq
              ];
              buildPhaseCargoCommand = ''
                cargo llvm-cov --no-report --locked --workspace
              '';
              installPhaseCommand = ''
                mkdir -p "$out"
                cargo llvm-cov report --lcov --output-path "$out/lcov.info"
                cargo llvm-cov report --html --output-dir "$out"
                cargo llvm-cov report --json --summary-only --output-path "$out/summary.json"
                # Shields.io endpoint badge: https://shields.io/badges/endpoint-badge
                jq '.data[0].totals.lines.percent as $p | {
                  schemaVersion: 1,
                  label: "coverage",
                  message: "\($p * 10 | round / 10)%",
                  color: (if $p >= 90 then "brightgreen" elif $p >= 75 then "yellow" else "red" end)
                }' "$out/summary.json" > "$out/badge.json"
              '';
            }
          );
        in
        {
          checks = {
            fmt = treefmt.config.build.check self;
            clippy = craneLib.cargoClippy (
              commonArgs
              // {
                inherit cargoArtifacts;
                cargoClippyExtraArgs = "--all-targets -- -D warnings";
              }
            );
            test = craneLib.cargoTest (commonArgs // { inherit cargoArtifacts; });
            deny = craneLib.cargoDeny (
              commonArgs
              // {
                cargoExtraArgs = "";
                cargoDenyChecks = "bans licenses sources";
              }
            );
            deny-advisories = craneLib.cargoDeny (
              commonArgs
              // {
                cargoExtraArgs = "";
                cargoDenyChecks = "--disable-fetch advisories";
                nativeBuildInputs = commonArgs.nativeBuildInputs ++ [ pkgs.git ];
                # cargo-deny reads the default database from this fixed directory name
                # and asks git for the checkout's HEAD and FETCH_HEAD timestamps.
                preBuild = ''
                  db="$CARGO_HOME/advisory-dbs/advisory-db-3157b0e258782691"
                  mkdir -p "$(dirname "$db")"
                  cp -r ${advisory-db} "$db"
                  chmod -R u+w "$db"
                  export GIT_AUTHOR_DATE="@${toString advisory-db.lastModified} +0000"
                  export GIT_COMMITTER_DATE="$GIT_AUTHOR_DATE"
                  git -C "$db" init --quiet
                  git -C "$db" add --all
                  git -C "$db" -c user.name=nix -c user.email=nix@localhost \
                    commit --quiet --message ${advisory-db.rev}
                  git -C "$db" rev-parse HEAD > "$db/.git/FETCH_HEAD"
                  touch --date="@${toString advisory-db.lastModified}" "$db/.git/FETCH_HEAD"
                '';
              }
            );
          }
          // lib.optionalAttrs dylintSupported { dylint = dylintCheck; };

          packages = { inherit coverage; };

          formatter = treefmt.config.build.wrapper;

          devShells.default = pkgs.mkShell {
            packages = [
              devRust
              pkgs.pkg-config
              pkgs.cargo-llvm-cov
              pkgs.cargo-deny
              pkgs.critcmp
              pkgs.samply
              pkgs.tracy
              pkgs.wasmtime
              # `web/build.sh` builds the WebAssembly example gallery with these.
              wasmBindgenCli
              pkgs.binaryen
              pkgs.python3
              pkgs.clang
              treefmt.config.build.wrapper
            ]
            ++ lib.optionals isLinux [ pkgs.perf ]
            ++ lib.optionals (isLinux && isx86_64) [ pkgs.mold ];
            buildInputs = buildLibraries ++ runtimeLibraries;
            LD_LIBRARY_PATH = lib.makeLibraryPath runtimeLibraries;
            RUSTFLAGS = lib.optionalString (isLinux && isx86_64) "-C link-arg=-fuse-ld=mold";
          };
        };
      outputs = forAllSystems perSystem;
    in
    {
      checks = forAllSystems (system: outputs.${system}.checks);
      packages = forAllSystems (system: outputs.${system}.packages);
      formatter = forAllSystems (system: outputs.${system}.formatter);
      devShells = forAllSystems (system: outputs.${system}.devShells);
    };
}
