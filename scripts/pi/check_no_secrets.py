#!/usr/bin/env python3
"""Fail the build if a credential ever enters the repository.

Both repositories are public and the Pi image is published to a public
release. "We will be careful" is not a control when the cost of one slip is a
home Wi-Fi passphrase in a public git history that cannot be rewritten out of
every clone.

Why this does NOT scan for high-entropy strings
-----------------------------------------------
The obvious implementation -- flag any 64-hex or high-entropy blob -- is the
wrong one here, and it is worth saying why so nobody "improves" it back.

``Cargo.lock`` is full of 64-hex SHA256 checksums. So is any lockfile, and so
are the kernel checksums this project deliberately records in
``scripts/pi/verify_kernel.sh``'s log. A generic entropy scanner fires on all
of them, every run. A check that cries wolf gets an allowlist, then a bigger
allowlist, then ``|| true``, and this repository has already written down what
that costs: "a routine override is a check that has stopped meaning anything"
(``.github/workflows/ci.yml``).

So this looks for the specific shapes a real leak takes -- a private key
header, a populated PSK field, a filled-in passphrase variable, a file that
should never have been added -- and stays quiet otherwise. Narrow and trusted
beats broad and ignored.

Runs over tracked files only (``git ls-files``): the working tree may contain
build output, a virtualenv, or the operator's own scratch files, and none of
those can reach the public history.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

# Files that must never be tracked, matched on basename or glob.
FORBIDDEN_NAMES = [
    "pi-secrets.env",
    "*.nmconnection",
    "overboard-authorized_keys",
    "overboard-userconf",
    "authorized_keys",
    "id_rsa",
    "id_dsa",
    "id_ecdsa",
    "id_ed25519",
    "*.pem",
    "*.p12",
    "*.pfx",
    "wpa_supplicant.conf",
    "custom.toml",
    "firstrun.sh",
]

# .gitignore must carry these, so the accident is prevented as well as
# detected. Detection alone means the leak already happened locally and is one
# `git push` from being permanent.
REQUIRED_GITIGNORE = [
    "pi-secrets.env",
    "*.nmconnection",
    "overboard-authorized_keys",
    "overboard-userconf",
]

# Placeholder values the example file is allowed to contain. Anything else in
# a passphrase field is treated as real.
PLACEHOLDER = re.compile(
    r"^(|your-[a-z0-9-]*|<[^>]*>|xxx+|changeme|placeholder|example|\.\.\.)$", re.I
)

CONTENT_RULES: list[tuple[str, re.Pattern[str], str]] = [
    (
        "private key",
        re.compile(r"-----BEGIN (RSA |OPENSSH |EC |DSA |PGP )?PRIVATE KEY-----"),
        "a private key block",
    ),
    (
        "wpa psk",
        # A populated NetworkManager psk= field, in the keyfile form it
        # actually takes: one literal value, alone, to end of line.
        #
        # Same tightening as the passphrase rule below and for the same
        # reason -- the loose version matched `psk = wpa_psk(ssid,
        # passphrase)`, which is a function call, not a key. Requiring the
        # value to end the line and to contain only literal characters
        # excludes every call and subscript expression without needing to
        # know anything about the language it appears in.
        re.compile(
            r"""^[ \t]*psk[ \t]*=[ \t]*["']?([A-Za-z0-9._@#$%^&*+/!-]{8,})["']?[ \t]*$""",
            re.M,
        ),
        "a populated NetworkManager psk= field",
    ),
    (
        "wpa passphrase",
        # Anchored to end of line, and the value may only contain characters a
        # passphrase LITERAL contains -- no parentheses, commas, braces or
        # dots-with-calls.
        #
        # The loose version of this (`=\s*(\S+)`) flagged
        # `passphrase = secrets.get(f"WIFI_{i}_PASSPHRASE", "")` in this
        # project's own generator: ordinary Python, no credential. That is
        # precisely the false positive this whole module is built to avoid,
        # and it came from the pattern reading an ASSIGNMENT rather than a
        # config VALUE. 8 is the WPA minimum passphrase length.
        re.compile(
            r"""^[ \t]*(?:wpa_)?(?:passphrase|WIFI_\d+_PASSPHRASE)[ \t]*=[ \t]*"""
            r"""["']?([A-Za-z0-9._@#$%^&*+/!-]{8,})["']?[ \t]*$""",
            re.M | re.I,
        ),
        "a filled-in Wi-Fi passphrase",
    ),
]

# This file names every pattern it hunts for, and the example file exists to
# show the shape of a secrets file. Both would otherwise flag themselves.
SELF_EXEMPT = {
    "scripts/pi/check_no_secrets.py",
    "scripts/pi/pi-secrets.example.env",
    "tests/test_check_no_secrets.py",
}


def tracked_files(root: Path) -> list[str]:
    out = subprocess.run(
        ["git", "ls-files"], cwd=root, capture_output=True, text=True, check=True
    )
    return [line for line in out.stdout.splitlines() if line]


def check_names(paths: list[str]) -> list[str]:
    problems = []
    for rel in paths:
        name = Path(rel).name
        for pattern in FORBIDDEN_NAMES:
            if Path(name).match(pattern):
                problems.append(
                    f"{rel}: filename matches '{pattern}' and must never be tracked"
                )
                break
    return problems


def check_content(root: Path, paths: list[str]) -> list[str]:
    problems = []
    for rel in paths:
        if rel in SELF_EXEMPT:
            continue
        p = root / rel
        try:
            text = p.read_text(encoding="utf-8")
        except (UnicodeDecodeError, OSError):
            # Binaries and unreadable files: nothing to match, and guessing an
            # encoding to scan them would only invent findings.
            continue
        for label, pattern, description in CONTENT_RULES:
            for m in pattern.finditer(text):
                captured = m.group(1) if m.groups() else ""
                if captured and PLACEHOLDER.match(captured.strip("\"'")):
                    continue
                line = text[: m.start()].count("\n") + 1
                problems.append(f"{rel}:{line}: {description} ({label})")
    return problems


def check_gitignore(root: Path) -> list[str]:
    gi = root / ".gitignore"
    if not gi.exists():
        return ["no .gitignore at the repository root"]
    body = gi.read_text()
    return [
        f".gitignore does not cover '{entry}'"
        for entry in REQUIRED_GITIGNORE
        if entry not in body
    ]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    args = ap.parse_args()

    paths = tracked_files(args.root)
    problems = (
        check_names(paths)
        + check_content(args.root, paths)
        + check_gitignore(args.root)
    )

    if problems:
        print("SECRET GUARD FAILED\n")
        for p in problems:
            print(f"  {p}")
        print(
            "\nDo not 'fix' this by editing the pattern list. If a credential is"
            "\nin a commit, it is in the history: rotate it, then remove it."
            "\nThis repository is public and the image is published publicly."
        )
        return 1

    print(f"secret guard: {len(paths)} tracked files clean")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
