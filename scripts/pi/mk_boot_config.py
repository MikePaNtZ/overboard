#!/usr/bin/env python3
"""Turn a local secrets file into the files that go on the card's boot partition.

Run by ``flash_pi.sh`` at flash time. Never run in CI, never run by an agent,
and its input never enters the repository.

Why the boot partition and nothing else
---------------------------------------
The card has two partitions: a FAT32 boot partition and an ext4 rootfs. macOS
cannot mount ext4 without third-party kernel extensions, and the CEO flashes
from a Mac. So every byte of configuration this script produces has to land on
the FAT32 side, and something on the Pi has to install it at first boot --
``overboard-firstboot``, shipped in the image (I1) from
scripts/pi/image/layer/overboard-base.rootfs-overlay/usr/local/sbin/.

That rules out writing NetworkManager profiles directly to
``/etc/NetworkManager/system-connections/``, which is the obvious approach and
is unavailable to us.

It also rules IN a better shape than the one Raspberry Pi Imager uses. Imager
hooks ``firstrun.sh`` from ``cmdline.txt`` and has the script edit
``cmdline.txt`` back out again at the end -- a boot-critical file mutated by a
script running during that same boot. Since we build the image (I1), the
installer can be a plain systemd oneshot that reads a directory, and
``cmdline.txt`` is never touched.

What the PSK derivation does and does not buy
---------------------------------------------
Passphrases are converted to their 256-bit WPA-PSK before they are written,
where the passphrase allows it. This does NOT protect the card: the PSK is
exactly as good as the passphrase for joining that SSID, so anyone holding the
card can still join the network.

What it does prevent is the passphrase leaking in cleartext to be tried
somewhere else -- and a home Wi-Fi passphrase is precisely the kind of secret
that gets reused. That is a real and narrow benefit, and overstating it would
be worse than not doing it.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import re
import shlex
import subprocess
import sys
from pathlib import Path

# NetworkManager refuses to load a connection profile that is group- or
# world-readable, so this is functional, not decorative.
NM_PROFILE_MODE = 0o600

MAX_WIFI_NETWORKS = 16


class ConfigError(Exception):
    """A problem with the operator's secrets file, not a bug."""


def load_secrets(path: Path) -> dict[str, str]:
    """Read a shell-style env file without executing it.

    Deliberately not ``source``d through a shell. The file holds the operator's
    home Wi-Fi passphrase; sourcing it would execute whatever a stray backtick
    or ``$(...)`` happened to contain, which is a needless amount of trust to
    place in a file that only ever needs to yield key/value pairs.

    ``$HOME`` and ``~`` are expanded, because ``SSH_PUBKEY_FILE`` is a path and
    writing it out in full is the kind of friction that ends with someone
    pasting a private key in instead.
    """
    if not path.exists():
        raise ConfigError(
            f"no secrets file at {path}\n"
            f"  cp scripts/pi/pi-secrets.example.env {path}\n"
            f"  chmod 600 {path}\n"
            f"then fill it in. It must live outside the repository."
        )

    mode = path.stat().st_mode & 0o777
    if mode & 0o077:
        raise ConfigError(
            f"{path} is mode {mode:o}, readable beyond its owner. "
            f"Run: chmod 600 {path}"
        )

    out: dict[str, str] = {}
    for lineno, raw in enumerate(path.read_text().splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            raise ConfigError(f"{path}:{lineno}: not a KEY=VALUE line: {raw!r}")
        key, _, value = line.partition("=")
        key = key.strip()
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", key):
            raise ConfigError(f"{path}:{lineno}: not a valid variable name: {key!r}")
        # shlex handles the quoting the example file uses; expandvars/expanduser
        # then make $HOME and ~ work in path values.
        try:
            parts = shlex.split(value.strip(), comments=True)
        except ValueError as e:
            raise ConfigError(f"{path}:{lineno}: unbalanced quotes: {e}") from e
        out[key] = os.path.expanduser(os.path.expandvars("".join(parts)))
    return out


def wpa_psk(ssid: str, passphrase: str) -> str:
    """The 256-bit WPA-PSK for this SSID and passphrase, as 64 hex characters.

    A passphrase that is already 64 hex characters IS a PSK and is passed
    through -- deriving a PSK from a PSK would produce a key that joins
    nothing.

    WPA-PSK is only defined for passphrases of 8..63 characters. Outside that
    range the passphrase is returned unchanged and NetworkManager is left to
    reject it, which gives a better error than anything invented here: a
    5-character passphrase is a typo, and silently accepting it would put a
    card in the field that never associates.
    """
    if re.fullmatch(r"[0-9a-fA-F]{64}", passphrase):
        return passphrase.lower()
    if not 8 <= len(passphrase) <= 63:
        return passphrase
    return hashlib.pbkdf2_hmac(
        "sha1", passphrase.encode("utf-8"), ssid.encode("utf-8"), 4096, 32
    ).hex()


def collect_networks(secrets: dict[str, str]) -> list[dict[str, object]]:
    """Gather WIFI_<n>_* blocks, in the order they will be tried.

    Stops at the first missing index rather than scanning to a limit, so
    deleting a block from the middle of the file is a visible mistake (the
    ones after it vanish) instead of a silent renumbering.
    """
    nets: list[dict[str, object]] = []
    for i in range(1, MAX_WIFI_NETWORKS + 1):
        ssid = secrets.get(f"WIFI_{i}_SSID", "").strip()
        if not ssid:
            break
        passphrase = secrets.get(f"WIFI_{i}_PASSPHRASE", "")
        if not passphrase:
            raise ConfigError(f"WIFI_{i}_SSID is set but WIFI_{i}_PASSPHRASE is empty")
        if ssid.startswith("your-"):
            raise ConfigError(
                f"WIFI_{i}_SSID is still the example placeholder ({ssid!r}). "
                "Fill in the real values before flashing."
            )
        try:
            priority = int(secrets.get(f"WIFI_{i}_PRIORITY", str(100 - i)))
        except ValueError as e:
            raise ConfigError(f"WIFI_{i}_PRIORITY must be an integer: {e}") from e
        nets.append(
            {
                "ssid": ssid,
                "psk": wpa_psk(ssid, passphrase),
                "priority": priority,
                "hidden": secrets.get(f"WIFI_{i}_HIDDEN", "false").lower() == "true",
            }
        )
    if not nets:
        raise ConfigError("no WIFI_1_SSID set - the Pi would boot with no network")
    return nets


def nm_profile(net: dict[str, object], index: int) -> str:
    """One NetworkManager keyfile connection profile."""
    ssid = net["ssid"]
    hidden = "\nhidden=true" if net["hidden"] else ""
    return f"""# Generated by scripts/pi/mk_boot_config.py at flash time.
# Not from the repository, and never committed to it.
[connection]
id=overboard-wifi-{index}
type=wifi
autoconnect=true
autoconnect-priority={net["priority"]}

[wifi]
mode=infrastructure
ssid={ssid}{hidden}

[wifi-security]
key-mgmt=wpa-psk
psk={net["psk"]}

[ipv4]
method=auto

[ipv6]
method=auto
addr-gen-mode=default
"""


def hash_password(password: str) -> str:
    """Hash a console password with openssl, or refuse.

    No fallback to a weaker algorithm and no writing it in the clear. If
    ``openssl`` is missing, the honest outcome is to say so and let the
    operator leave ``PI_PASSWORD`` empty -- key-only SSH is the default and
    works without this.
    """
    try:
        out = subprocess.run(
            ["openssl", "passwd", "-6", "-stdin"],
            input=password.encode(),
            capture_output=True,
            check=True,
        )
    except FileNotFoundError as e:
        raise ConfigError(
            "PI_PASSWORD is set but `openssl` was not found, so it cannot be "
            "hashed. Install openssl, or leave PI_PASSWORD empty and use "
            "key-only SSH."
        ) from e
    except subprocess.CalledProcessError as e:
        raise ConfigError(f"openssl passwd failed: {e.stderr.decode().strip()}") from e
    return out.stdout.decode().strip()


def build(secrets: dict[str, str], outdir: Path) -> list[Path]:
    """Write the boot-partition payload. Returns the files written."""
    written: list[Path] = []

    netdir = outdir / "overboard-net.d"
    netdir.mkdir(parents=True, exist_ok=True)
    for i, net in enumerate(collect_networks(secrets), 1):
        p = netdir / f"overboard-wifi-{i}.nmconnection"
        p.write_text(nm_profile(net, i))
        p.chmod(NM_PROFILE_MODE)
        written.append(p)

    username = secrets.get("PI_USERNAME", "").strip()
    if not username:
        raise ConfigError("PI_USERNAME is empty - the image ships with no default user")

    pubkey_path = secrets.get("SSH_PUBKEY_FILE", "").strip()
    if not pubkey_path:
        raise ConfigError("SSH_PUBKEY_FILE is empty - the Pi would be unreachable")
    pubkey = Path(pubkey_path)
    if not pubkey.exists():
        raise ConfigError(f"SSH_PUBKEY_FILE does not exist: {pubkey}")
    key_text = pubkey.read_text().strip()
    # The failure this catches is real and expensive: pointing at id_ed25519
    # instead of id_ed25519.pub would copy a PRIVATE key onto a partition that
    # any machine can read.
    if "PRIVATE KEY" in key_text or not re.match(r"(ssh|ecdsa|sk-)", key_text):
        raise ConfigError(
            f"{pubkey} does not look like an SSH PUBLIC key. Point "
            f"SSH_PUBKEY_FILE at the .pub file, never at a private key."
        )

    p = outdir / "overboard-authorized_keys"
    p.write_text(key_text + "\n")
    written.append(p)

    conf = [
        f"hostname={secrets.get('PI_HOSTNAME', 'overboard')}",
        f"username={username}",
        f"wifi_country={secrets.get('WIFI_COUNTRY', 'US')}",
        f"ssh_password_auth={secrets.get('SSH_PASSWORD_AUTH', 'false')}",
    ]
    password = secrets.get("PI_PASSWORD", "")
    if password:
        conf.append(f"password_hash={hash_password(password)}")

    p = outdir / "overboard-userconf"
    p.write_text("\n".join(conf) + "\n")
    p.chmod(NM_PROFILE_MODE)
    written.append(p)

    return written


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument(
        "--secrets",
        type=Path,
        default=Path.home() / ".overboard" / "pi-secrets.env",
        help="local secrets file (default: ~/.overboard/pi-secrets.env)",
    )
    ap.add_argument(
        "--out", type=Path, required=True, help="staging directory to write into"
    )
    args = ap.parse_args()

    try:
        secrets = load_secrets(args.secrets)
        written = build(secrets, args.out)
    except ConfigError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    # Filenames only. Printing a path is navigation; printing content would
    # put a PSK in a terminal scrollback and, on CI, in a log.
    print(f"wrote {len(written)} files to {args.out}:")
    for p in written:
        print(f"    {p.relative_to(args.out)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
