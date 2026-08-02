#!/usr/bin/env python3
"""Validate the METABEGIN headers in our rpi-image-gen layer files.

Every line in a layer's METABEGIN block starts with `#`, so it reads like a
comment block. It is not. It is DEB822 -- `Key: Value`, with continuation
lines indented by exactly one space -- and rpi-image-gen parses it.

The failure mode is why this script exists, because it is silent. Adding one
prose sentence between two fields produced:

    Invalid DEB822 format: line 'Provides `rpi-device` ONLY, matching...'
    appears to be a continuation but is not indented.
    Layer 'overboard-pi5' not found

Note the second line. A malformed header does NOT stop the build where the
mistake is; it makes the layer *invisible*, and the build then fails later
complaining that a layer is missing. The error points at the config that
referenced it, not at the file that is broken. That is a bad half-hour, and
it costs a full image build to discover.

Cheap to check here, in seconds, with no container and no network.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
IMAGE_TREE = REPO / "scripts" / "pi" / "image"

REQUIRED = ["X-Env-Layer-Name", "X-Env-Layer-Category", "X-Env-Layer-Version"]

# `Key: Value`. Underscores are allowed even though strict DEB822 field names
# do not use them: rpi-image-gen's own stock layers carry
# `X-Env-Var-storage_type`, and a validator that rejects the upstream format
# is a validator that gets deleted.
FIELD = re.compile(r"^([A-Za-z][A-Za-z0-9_-]*):\s?(.*)$")


def check_file(path: Path) -> list[str]:
    text = path.read_text()
    rel = path.relative_to(REPO)
    problems: list[str] = []

    if "# METABEGIN" not in text:
        # Not every YAML in the tree is a layer -- config/overboard.yaml is a
        # plain config and has no header. Silence is correct there.
        return []

    if "# METAEND" not in text:
        return [f"{rel}: has METABEGIN but no METAEND"]

    block = text.split("# METABEGIN", 1)[1].split("# METAEND", 1)[0]

    seen: list[str] = []
    for i, raw in enumerate(block.splitlines(), 1):
        if not raw.strip():
            continue
        if not raw.startswith("#"):
            problems.append(f"{rel}: line {i} of the header does not start with '#': {raw!r}")
            continue
        # The prefix is '# ' -- hash AND one space -- so a continuation reads
        # '#  ' with TWO spaces. Strip the hash, then the single comment
        # space; whatever indentation remains is real DEB822 indentation.
        #
        # Getting this wrong is how the first version of this script reported
        # every field in every file as a continuation: it stripped only the
        # hash, so `# X-Env-Layer-Name: ...` became ` X-Env-Layer-Name: ...`,
        # which begins with a space and therefore looked like one.
        line = raw[1:]
        if line.startswith(" "):
            line = line[1:]
        if not line.strip():
            continue  # '#' alone is a paragraph separator, which is fine
        if line.startswith((" ", "\t")):
            if not seen:
                problems.append(
                    f"{rel}: header begins with a continuation line, which "
                    f"continues nothing: {line.strip()!r}"
                )
            continue  # a valid continuation of the previous field
        m = FIELD.match(line.strip())
        if not m:
            problems.append(
                f"{rel}: header line is neither 'Key: Value' nor an indented "
                f"continuation: {line.strip()!r}\n"
                f"      Prose belongs BELOW METAEND. Left here it makes the "
                f"layer invisible, and the build fails elsewhere reporting the "
                f"layer as not found."
            )
            continue
        seen.append(m.group(1))

    for field in REQUIRED:
        if field not in seen:
            problems.append(f"{rel}: header is missing {field}")

    return problems


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--tree", type=Path, default=IMAGE_TREE)
    args = ap.parse_args()

    files = sorted(args.tree.rglob("*.yaml"))
    if not files:
        print(f"error: no layer files under {args.tree}", file=sys.stderr)
        return 1

    problems: list[str] = []
    checked = 0
    for f in files:
        found = check_file(f)
        if "# METABEGIN" in f.read_text():
            checked += 1
        problems.extend(found)

    if problems:
        print("LAYER HEADER CHECK FAILED\n")
        for p in problems:
            print(f"  x {p}")
        return 1

    print(f"layer header check: {checked} layer file(s) parse as valid DEB822")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
