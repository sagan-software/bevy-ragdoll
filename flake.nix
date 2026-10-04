{
  description = "Development environment for bevy-ragdoll";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.11";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    { nixpkgs, rust-overlay, ... }:
    let
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "x86_64-darwin"
        "aarch64-darwin"
      ];
      forAllSystems = nixpkgs.lib.genAttrs systems;
      runtimeLibraries = pkgs:
        pkgs.lib.optionals pkgs.stdenv.isLinux (with pkgs; [
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
        ]);
    in
    {
      devShells = forAllSystems (system:
        let
          pkgs = import nixpkgs {
            inherit system;
            overlays = [ rust-overlay.overlays.default ];
          };
          rust = pkgs.rust-bin.stable.latest.default.override {
            extensions = [
              "rust-src"
              "clippy"
              "rustfmt"
              "rust-analyzer"
              "llvm-tools-preview"
            ];
            targets = [ "wasm32-unknown-unknown" "wasm32-wasip1" ];
          };
          linuxBuildLibraries = pkgs.lib.optionals pkgs.stdenv.isLinux (with pkgs; [
            alsa-lib.dev
            udev.dev
            wayland.dev
            libxkbcommon.dev
          ]);
          linuxTools = pkgs.lib.optionals pkgs.stdenv.isLinux [ pkgs.perf ];
          moldTools = pkgs.lib.optionals
            (pkgs.stdenv.isLinux && pkgs.stdenv.hostPlatform.isx86_64)
            [ pkgs.mold ];
        in
        {
          default = pkgs.mkShell {
            packages = [
              rust
              pkgs.pkg-config
              pkgs.cargo-llvm-cov
              pkgs.cargo-deny
              pkgs.critcmp
              pkgs.samply
              pkgs.tracy
              pkgs.wasmtime
              pkgs.wasm-bindgen-cli
              pkgs.clang
            ] ++ linuxTools ++ moldTools;
            buildInputs = linuxBuildLibraries ++ runtimeLibraries pkgs;
            LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath (runtimeLibraries pkgs);
            RUSTFLAGS = pkgs.lib.optionalString
              (pkgs.stdenv.isLinux && pkgs.stdenv.hostPlatform.isx86_64)
              "-C link-arg=-fuse-ld=mold";
          };
        });
    };
}
