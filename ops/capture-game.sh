#!/usr/bin/env bash
# Launch the Unreal client, drive it with the REAL sim-host, and capture frames.
#
#   ops/capture-game.sh [MAP] [OUT_PREFIX]
#
#   MAP          OB_City (default) or OB_Main. OB_Main loads in ~40s with no
#                third-party content and is the right level for transform and
#                handedness checks. OB_City pulls ~1.5GB of City Park and can
#                take two minutes or more.
#   OUT_PREFIX   defaults to /tmp/obcap
#
# Why this exists: verifying the game by hand meant fixed `sleep` guesses, and
# they were wrong every time the level size changed -- a capture taken while UE
# was still loading shows the desktop, not the game, and reads as "it's broken"
# rather than "it wasn't up yet". This waits for a real readiness MARKER in the
# log instead of guessing, so it works on a 40s map and a 3-minute map without
# retuning. Three separate capture attempts were lost to that before this
# existed.
#
# Requires (both are one-time macOS grants, already given on this machine):
#   * Screen Recording      -- for `screencapture`
#   * Automation/System Events -- to raise the UE window, since a capture of a
#     window that is behind the browser is a capture of the browser.
set -uo pipefail

MAP="${1:-OB_City}"
OUT="${2:-/tmp/obcap}"

GAME_DIR="${OVERBOARD_GAME_DIR:-$HOME/projects/overboard-game}"
UE_BIN="${UE_BIN:-/Users/Shared/Epic Games/UE_5.7/Engine/Binaries/Mac/UnrealEditor}"
SIM_HOST="${SIM_HOST:-/tmp/coo-verify-target/release/sim-host}"
SEND_INPUT="${SEND_INPUT:-/tmp/coo-verify-target/release/send-input}"
MUJOCO_DIR="${MUJOCO_DIR:-$HOME/projects/overboard/.venv/lib/python3.14/site-packages/mujoco}"
UE_LOG="$HOME/Library/Logs/Unreal Engine/OverboardGameEditor/OverboardGame.log"

# The world is up when the board actor's UDP client binds 9601. We detect that
# by TRYING TO BIND IT OURSELVES: if the bind fails with EADDRINUSE, the game
# has it and the world is live.
#
# This replaced polling the UE log for a readiness line, which does not work:
# UE buffers its log and a hard kill loses the tail, so the marker for a run
# that genuinely succeeded may never be written. Three capture attempts were
# lost to that, and the failure mode is maximally confusing -- the scene is
# visibly up on screen while the script insists it is not ready.
#
# `lsof -iUDP:9601` is NOT a substitute; it returns nothing here even when the
# port is definitely bound. Attempting the bind is the only check that agrees
# with reality.
READY_TIMEOUT_S="${READY_TIMEOUT_S:-300}"

port_taken() {
  python3 - <<'PY'
import socket, sys
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    s.bind(("127.0.0.1", 9601)); sys.exit(1)   # free -> not ready
except OSError:
    sys.exit(0)                                # in use -> world is up
finally:
    s.close()
PY
}

export PATH="/usr/bin:/bin:/usr/sbin:/sbin:$PATH"
export DYLD_LIBRARY_PATH="$MUJOCO_DIR:$MUJOCO_DIR/lib:${DYLD_LIBRARY_PATH:-}"

for f in "$UE_BIN" "$SIM_HOST" "$SEND_INPUT"; do
  [ -x "$f" ] || { echo "MISSING: $f"; exit 1; }
done

cleanup() {
  pkill -f "UnrealEditor.*OverboardGame" 2>/dev/null
  pkill -f "release/sim-host"            2>/dev/null
  pkill -f "release/send-input"          2>/dev/null
}
trap cleanup EXIT
cleanup; sleep 1
rm -f "$OUT"_*.png

# Read the log only from here on. It is appended across runs, so a stale marker
# from the PREVIOUS launch would otherwise report ready instantly -- which looks
# exactly like a fast load and produces a black frame.
LOG_START=0
[ -f "$UE_LOG" ] && LOG_START=$(wc -c < "$UE_LOG" | tr -d ' ')

echo "launching $MAP ..."
"$UE_BIN" "$GAME_DIR/OverboardGame.uproject" "/Game/Maps/$MAP" \
  -game -windowed -resx=1280 -resy=720 -nosplash >"/tmp/obcap-ue.log" 2>&1 &

waited=0
ready=0
while [ "$waited" -lt "$READY_TIMEOUT_S" ]; do
  if port_taken; then echo "world up after ${waited}s"; ready=1; break; fi
  pgrep -f "UnrealEditor.*OverboardGame" >/dev/null || { echo "FAILED: UE exited during load"; tail -5 /tmp/obcap-ue.log; exit 1; }
  sleep 2; waited=$((waited + 2))
done
[ "$ready" -eq 1 ] || { echo "FAILED: 9601 never bound after ${READY_TIMEOUT_S}s"; exit 1; }

# Raise the window. A capture of an occluded window is a capture of whatever is
# in front of it, and that failure is silent -- you get a valid PNG of the wrong
# thing, which is worse than no PNG at all.
osascript -e 'tell application "System Events" to set frontmost of first process whose name contains "Unreal" to true' 2>/dev/null
sleep 3
screencapture -x "${OUT}_0_loaded.png"

"$SIM_HOST" --duration-secs 24 >/tmp/obcap-host.log 2>&1 &
sleep 1
"$SEND_INPUT" >/tmp/obcap-input.log 2>&1 &

# Offsets track send-input's fixed schedule: settle 0-1.5, accelerate 1.5-4.5,
# coast 4.5-6.5, reverse 6.5-10, coast 10-12, lean+steer turn 12-14, release 14+.
shoot() { sleep "$1"; screencapture -x "${OUT}_$2.png"; echo "  captured $2"; }
shoot 3  1_accel
shoot 4  2_coast
shoot 4  3_reverse
shoot 3  4_preturn
shoot 3  5_turn
shoot 2  6_turnend

echo "--- host ---";  tail -1 /tmp/obcap-host.log
echo "--- frames ---"; ls -1 "${OUT}"_*.png
