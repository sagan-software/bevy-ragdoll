#!/usr/bin/env python3
"""Builds the WebAssembly example gallery into a static site.

Run inside the dev shell, which provides cargo, wasm-bindgen, and wasm-opt:

    nix develop -c python3 web/build.py [--out dist] [--coverage result] [--example showcase]

Every `[package.metadata.example.NAME]` entry in Cargo.toml with `wasm = true`
becomes one page at `examples/NAME/`. `--coverage DIR` copies a
`nix build .#coverage` result to `coverage/`, including the Shields.io
`badge.json` endpoint that the README badge reads.
"""

import argparse
import html
import os
import shutil
import subprocess
import tomllib
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
WEB = ROOT / "web"
TARGET = "wasm32-unknown-unknown"
PROFILE = "wasm-release"
# Runtime assets; Bevy on the web fetches them from `assets/` beside the page.
ASSET_DIRS = ["rigs"]
# The showcase leads the gallery; other categories follow in Cargo.toml order.
FEATURED_CATEGORY = "Showcase"


def wasm_examples() -> list[tuple[str, dict]]:
    """Returns (name, metadata) for each example marked `wasm = true`, in Cargo.toml order."""
    manifest = tomllib.loads((ROOT / "Cargo.toml").read_text())
    examples = manifest["package"]["metadata"]["example"]
    return [(name, meta) for name, meta in examples.items() if meta.get("wasm")]


def build_wasm(names: list[str], out: Path) -> dict[str, int]:
    """Compiles, binds, and optimizes each example; returns the shipped .wasm size in bytes."""
    env = dict(os.environ)
    # The dev shell links native code with mold, which rust-lld rejects for wasm32.
    env["RUSTFLAGS"] = ""
    # Cargo.toml's size-optimized profile; wasm-opt then shrinks the bound module further.
    command = ["cargo", "build", "--profile", PROFILE, "--locked", "--target", TARGET]
    for name in names:
        command += ["--example", name]
    subprocess.run(command, cwd=ROOT, env=env, check=True)

    target_dir = Path(env.get("CARGO_TARGET_DIR", ROOT / "target"))

    def bind(name: str) -> int:
        """Generates the JS bindings for one example and optimizes its module."""
        page = out / "examples" / name
        page.mkdir(parents=True, exist_ok=True)
        wasm = target_dir / TARGET / PROFILE / "examples" / f"{name}.wasm"
        subprocess.run(
            ["wasm-bindgen", "--target", "web", "--no-typescript", "--out-dir", page, "--out-name", name, wasm],
            check=True,
        )
        bound = page / f"{name}_bg.wasm"
        subprocess.run(["wasm-opt", "-Os", "--all-features", bound, "-o", bound], check=True)
        return bound.stat().st_size

    # wasm-opt is single-threaded per module; run the examples side by side.
    with ThreadPoolExecutor(max_workers=os.cpu_count()) as pool:
        return dict(zip(names, pool.map(bind, names)))


def megabytes(size: int) -> str:
    """Formats a byte count as megabytes for the loading message."""
    return f"{size / 1_000_000:.0f} MB"


def tile(name: str, meta: dict, out: Path) -> str:
    """Returns one gallery tile linking to the example page."""
    thumbnail = next((WEB / "thumbnails").glob(f"{name}.*"), None)
    title = html.escape(meta["name"])
    if thumbnail:
        shutil.copy(thumbnail, out / "thumbnails" / thumbnail.name)
        image = f'<img src="thumbnails/{thumbnail.name}" alt="" loading="lazy" />'
    else:
        image = f'<div class="placeholder">{title}</div>'
    cta = '<span class="cta">Open demo</span>' if meta["category"] == FEATURED_CATEGORY else ""
    return (
        f'        <a class="tile" href="examples/{name}/">{image}'
        f'<div class="tile-text"><h3>{title}</h3><p>{html.escape(meta["description"])}</p>{cta}</div></a>'
    )


def write_site(examples: list[tuple[str, dict]], sizes: dict[str, int], out: Path) -> None:
    """Writes the gallery index, one page per example, and the stylesheet."""
    (out / "thumbnails").mkdir(parents=True, exist_ok=True)
    shutil.copy(WEB / "style.css", out / "style.css")
    (out / ".nojekyll").write_text("")

    categories: dict[str, list[str]] = {}
    for name, meta in examples:
        categories.setdefault(meta["category"], []).append(tile(name, meta, out))
    order = sorted(categories, key=lambda category: category != FEATURED_CATEGORY)
    sections = []
    for category in order:
        grid = "grid featured" if category == FEATURED_CATEGORY else "grid"
        heading = "" if category == FEATURED_CATEGORY else f"      <h2>{html.escape(category)}</h2>\n"
        sections.append(f'{heading}      <div class="{grid}">\n' + "\n".join(categories[category]) + "\n      </div>")
    index = (WEB / "index.html").read_text().replace("{{sections}}", "\n".join(sections))
    (out / "index.html").write_text(index)

    template = (WEB / "example.html").read_text()
    for name, meta in examples:
        page = (
            template.replace("{{id}}", name)
            .replace("{{name}}", html.escape(meta["name"]))
            .replace("{{description}}", html.escape(meta["description"]))
            .replace("{{size}}", megabytes(sizes.get(name, 0)))
        )
        (out / "examples" / name).mkdir(parents=True, exist_ok=True)
        (out / "examples" / name / "index.html").write_text(page)
        for directory in ASSET_DIRS:
            shutil.copytree(ROOT / "assets" / directory, out / "examples" / name / "assets" / directory, dirs_exist_ok=True)


def main() -> None:
    """Parses options and builds the site."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=ROOT / "dist", help="output directory (default: dist)")
    parser.add_argument("--coverage", type=Path, help="`nix build .#coverage` result to publish at coverage/")
    parser.add_argument("--example", action="append", help="build only this example (repeatable)")
    parser.add_argument("--skip-wasm", action="store_true", help="regenerate HTML only, keeping built modules")
    args = parser.parse_args()

    examples = wasm_examples()
    if args.example:
        examples = [(name, meta) for name, meta in examples if name in args.example]
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)

    if args.skip_wasm:
        sizes = {
            name: (out / "examples" / name / f"{name}_bg.wasm").stat().st_size
            for name, _ in examples
            if (out / "examples" / name / f"{name}_bg.wasm").exists()
        }
    else:
        sizes = build_wasm([name for name, _ in examples], out)
    write_site(examples, sizes, out)

    if args.coverage:
        destination = out / "coverage"
        shutil.rmtree(destination, ignore_errors=True)
        shutil.copytree(args.coverage, destination, symlinks=False)
        # Nix store copies are read-only; make the tree removable on the next build.
        for path in [destination, *destination.rglob("*")]:
            path.chmod(path.stat().st_mode | 0o200)

    for name, size in sizes.items():
        print(f"{name}: {size:,} bytes")


if __name__ == "__main__":
    main()
