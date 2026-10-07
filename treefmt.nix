# Formatter configuration shared by `nix fmt` and the `fmt` flake check.
# No RON formatter exists in nixpkgs, so `.ron` files are not formatted.
{ lib, ... }:
{
  projectRootFile = "flake.nix";

  programs = {
    nixfmt.enable = true;

    rustfmt.enable = true;

    taplo = {
      enable = true;
      settings.formatting = {
        # Keep hand-written array layout and the blank lines between tables.
        array_auto_collapse = false;
        array_auto_expand = false;
        allowed_blank_lines = 1;
        indent_string = "    ";
      };
    };

    # Prettier formats Markdown and YAML only; generated JSON reports keep their layout.
    prettier = {
      enable = true;
      includes = lib.mkForce [
        "*.md"
        "*.yaml"
        "*.yml"
      ];
      settings = {
        # Keep author line breaks and leave code blocks untouched.
        proseWrap = "preserve";
        embeddedLanguageFormatting = "off";
      };
    };
  };

  settings.global.excludes = [
    ".claude/**"
    ".direnv/**"
    "target/**"
    "result*"
  ];
}
