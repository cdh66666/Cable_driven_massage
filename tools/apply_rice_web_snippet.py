#!/usr/bin/env python3
"""Replace riceEarStrokes() in src/main.cpp with a generated web snippet."""

from __future__ import annotations

import argparse
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--snippet", required=True, help="generated *_web.js file")
    parser.add_argument("--main", default="src/main.cpp")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def find_function_bounds(text: str, name: str) -> tuple[int, int]:
    marker = f"function {name}("
    start = text.find(marker)
    if start < 0:
        raise SystemExit(f"Cannot find {marker}")

    brace = text.find("{", start)
    if brace < 0:
        raise SystemExit(f"Cannot find opening brace for {name}")

    depth = 0
    in_quote: str | None = None
    escape = False
    for idx in range(brace, len(text)):
        ch = text[idx]
        if in_quote:
            if escape:
                escape = False
            elif ch == "\\":
                escape = True
            elif ch == in_quote:
                in_quote = None
            continue
        if ch in ("'", '"', "`"):
            in_quote = ch
        elif ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return start, idx + 1

    raise SystemExit(f"Cannot find closing brace for {name}")


def main() -> None:
    args = parse_args()
    main_path = Path(args.main)
    snippet_path = Path(args.snippet)

    text = main_path.read_text(encoding="utf-8")
    snippet = snippet_path.read_text(encoding="utf-8").strip()
    if not snippet.startswith("function riceEarStrokes()"):
        raise SystemExit("Snippet must define function riceEarStrokes()")

    start, end = find_function_bounds(text, "riceEarStrokes")
    next_text = text[:start] + snippet + text[end:]
    if args.dry_run:
        print(f"would replace bytes {start}..{end} in {main_path}")
        return
    main_path.write_text(next_text, encoding="utf-8")
    print(f"updated {main_path} from {snippet_path}")


if __name__ == "__main__":
    main()
