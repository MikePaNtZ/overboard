"""Acceptance gate for the Pi credential guard (issue #182, I4).

Both repositories are public and the Pi image is published to a public
release, so `scripts/pi/check_no_secrets.py` is the control that stops a
credential entering history. A guard nobody has tried to fool is not a
control, so these tests plant each leak shape it claims to catch.

The most load-bearing test here is the one that asserts a Cargo.lock-style
SHA256 does NOT fire. That is the whole design: the guard is deliberately
narrow so it stays trusted. A change that makes it broad enough to flag a
checksum would, on this repository, flag hundreds -- and a check that cries
wolf gets an allowlist, then `|| true`, then nothing.
"""

import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts" / "pi"))

from check_no_secrets import (  # noqa: E402
    REQUIRED_GITIGNORE,
    check_content,
    check_gitignore,
    check_names,
)


@pytest.fixture
def repo(tmp_path):
    """A throwaway git repo with a conforming .gitignore."""
    subprocess.run(["git", "init", "-q"], cwd=tmp_path, check=True)
    (tmp_path / ".gitignore").write_text("\n".join(REQUIRED_GITIGNORE) + "\n")
    return tmp_path


def plant(repo, rel, text):
    p = repo / rel
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(text)
    return [rel]


# ---------------------------------------------------------------------------
# It catches what it claims to catch.
# ---------------------------------------------------------------------------


def test_a_private_key_block_is_caught(repo):
    paths = plant(
        repo,
        "notes.md",
        "here is the key\n-----BEGIN OPENSSH PRIVATE KEY-----\nb3BlbnNz\n",
    )
    problems = check_content(repo, paths)
    assert problems, "a private key block must be caught"
    assert "private key" in problems[0]


def test_a_populated_psk_field_is_caught(repo):
    paths = plant(
        repo,
        "wifi.nmconnection.bak",
        "[wifi-security]\nkey-mgmt=wpa-psk\npsk=8f4c2a1b9e7d6503f1a2b3c4d5e6f708\n",
    )
    problems = check_content(repo, paths)
    assert problems and "psk" in problems[0]


def test_a_filled_in_passphrase_is_caught(repo):
    paths = plant(repo, "secrets.env", 'WIFI_1_PASSPHRASE="correct-horse-battery"\n')
    problems = check_content(repo, paths)
    assert problems and "passphrase" in problems[0]


def test_a_credential_filename_is_caught_even_when_empty(repo):
    # The filename rule is independent of content: an empty
    # `home.nmconnection` is still a file that should never be tracked, and
    # tomorrow it will not be empty.
    problems = check_names(["config/home.nmconnection"])
    assert problems and "nmconnection" in problems[0]

    assert check_names(["deploy/id_ed25519"])
    assert check_names(["certs/server.pem"])


def test_a_missing_gitignore_entry_is_caught(repo):
    (repo / ".gitignore").write_text("/target/\n")
    problems = check_gitignore(repo)
    assert len(problems) == len(REQUIRED_GITIGNORE)


def test_no_gitignore_at_all_is_caught(tmp_path):
    assert check_gitignore(tmp_path)


# ---------------------------------------------------------------------------
# It stays quiet on everything else. These matter as much as the catches:
# the guard's value depends on nobody wanting to switch it off.
# ---------------------------------------------------------------------------


def test_a_cargo_lock_checksum_does_not_fire(repo):
    """The reason this guard is not an entropy scanner.

    64 hex characters is a WPA PSK. It is also every SHA256 in every lockfile
    in this repository, and the kernel checksum verify_kernel.sh prints on
    purpose. Flagging the shape rather than the context would make this
    unusable on day one.
    """
    paths = plant(
        repo,
        "Cargo.lock",
        '[[package]]\nname = "libc"\n'
        'checksum = "a2b3c4d5e6f708192a3b4c5d6e7f80912a3b4c5d6e7f80912a3b4c5d6e7f8091"\n',
    )
    assert check_content(repo, paths) == []


def test_the_example_secrets_file_does_not_fire(repo):
    """Placeholders are not secrets.

    The committed example has to show the shape of a real secrets file. If it
    tripped the guard, the guard would be disabled within a day.
    """
    paths = plant(
        repo,
        "example.env",
        'WIFI_1_SSID="your-home-ssid"\nWIFI_1_PASSPHRASE="your-home-passphrase"\n',
    )
    assert check_content(repo, paths) == []


def test_an_empty_password_field_does_not_fire(repo):
    paths = plant(repo, "example.env", 'PI_PASSWORD=""\nWIFI_1_PASSPHRASE=""\n')
    assert check_content(repo, paths) == []


def test_a_psk_template_placeholder_does_not_fire(repo):
    """The generator's own f-string template must not flag itself."""
    paths = plant(repo, "gen.py", 'profile = f"""\npsk={net["psk"]}\n"""\n')
    assert check_content(repo, paths) == []


def test_python_code_that_merely_mentions_a_passphrase_does_not_fire(repo):
    """Regression: the guard flagged this project's own generator.

    `passphrase = secrets.get(f"WIFI_{i}_PASSPHRASE", "")` is ordinary Python
    with no credential in it, and the original pattern read the ASSIGNMENT
    rather than a config VALUE. Exactly the false positive this module exists
    to avoid, in the first file it was pointed at.
    """
    paths = plant(
        repo,
        "gen.py",
        'passphrase = secrets.get(f"WIFI_{i}_PASSPHRASE", "")\n'
        'if not passphrase:\n'
        '    raise ConfigError("empty")\n'
        'psk = wpa_psk(ssid, passphrase)\n',
    )
    assert check_content(repo, paths) == []


def test_a_real_passphrase_still_fires_after_the_tightening(repo):
    """The tightening must not have disarmed the rule it was narrowing."""
    for line in (
        'WIFI_1_PASSPHRASE="correct-horse-battery"',
        "WIFI_2_PASSPHRASE=hunter2hunter2",
        'wpa_passphrase="s3cret-workshop-key"',
    ):
        assert check_content(repo, plant(repo, "leak.env", line + "\n")), line


def test_a_binary_file_is_skipped_rather_than_guessed_at(repo):
    p = repo / "blob.bin"
    p.write_bytes(bytes(range(256)))
    assert check_content(repo, ["blob.bin"]) == []


# ---------------------------------------------------------------------------
# End to end, against the real repository.
# ---------------------------------------------------------------------------


def test_this_repository_is_clean():
    """The guard passes on the tree as committed.

    A red here is not a flaky test. It means a credential is in the working
    tree, and the correct response is to rotate it -- never to adjust the
    patterns until it goes green.
    """
    result = subprocess.run(
        [sys.executable, str(REPO / "scripts" / "pi" / "check_no_secrets.py")],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
