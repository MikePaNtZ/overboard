#!/usr/bin/env bash
# Drive the board yourself. Starts the real physics host, launches the game,
# hands you the keyboard.
#
#     ops/drive-game.sh [MAP] [MINUTES]
#
#     MAP      OB_City (default) or OB_Main
#     MINUTES  how long the physics host stays up (default 15)
#
# Ctrl-C, or just quit the game window, when you are done.
#
#     W / Up        lean forward   -- accelerate
#     S / Down      lean back      -- slow, then reverse
#     A / Left      lean + steer left
#     D / Right     lean + steer right
#     Space         arm            R  reset
#
# There is NO scripted input here: sim-host runs the real 500 Hz control law
# against MuJoCo and waits for whatever you send it. Nothing drives the board
# but you.
#
# Why a script: the two halves must be started in the right order and the game
# has to be in the foreground to take keystrokes. Doing it by hand means a
# window that silently has no focus, or a host that is not listening yet, and
# both look identical to "it is broken".
set -uo pipefail

MAP="${1:-OB_City}"
MINUTES="${2:-15}"

GAME_DIR="${OVERBOARD_GAME_DIR:-$HOME/projects/overboard-game}"
UE_BIN="${UE_BIN:-/Users/Shared/Epic Games/UE_5.7/Engine/Binaries/Mac/UnrealEditor}"
SIM_HOST="${SIM_HOST:-/tmp/coo-verify-target/release/sim-host}"
MUJOCO_DIR="${MUJOCO_DIR:-$HOME/projects/overboard/.venv/lib/python3.14/site-packages/mujoco}"

export PATH="/usr/bin:/bin:/usr/sbin:/sbin:$PATH"
export DYLD_LIBRARY_PATH="$MUJOCO_DIR:$MUJOCO_DIR/lib:${DYLD_LIBRARY_PATH:-}"

for f in "$UE_BIN" "$SIM_HOST"; do
  [ -x "$f" ] || { echo "MISSING: $f"; exit 1; }
done

cleanup() {
  pkill -f "UnrealEditor.*OverboardGame" 2>/dev/null
  pkill -f "release/sim-host"            2>/dev/null
  pkill -f "release/send-input"          2>/dev/null
}
trap cleanup EXIT
cleanup; sleep 1

# The world is live once the board actor binds 9601. Poll by trying to bind it
# ourselves -- see ops/capture-game.sh for why the UE log is not usable here.
port_taken() {
  python3 - <<'PY'
import socket, sys
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    s.bind(("127.0.0.1", 9601)); sys.exit(1)
except OSError:
    sys.exit(0)
finally:
    s.close()
PY
}

echo "launching $MAP ..."
"$UE_BIN" "$GAME_DIR/OverboardGame.uproject" "/Game/Maps/$MAP" \
  -game -windowed -resx=1600 -resy=900 -nosplash >/tmp/drive-ue.log 2>&1 &

waited=0
while [ "$waited" -lt 300 ]; do
  port_taken && break
  pgrep -f "UnrealEditor.*OverboardGame" >/dev/null || { echo "FAILED: UE exited during load"; tail -5 /tmp/drive-ue.log; exit 1; }
  sleep 2; waited=$((waited + 2))
done
echo "world up after ${waited}s"

# Focus, or the keystrokes go to whatever was in front.
osascript -e 'tell application "System Events" to set frontmost of first process whose name contains "Unreal" to true' 2>/dev/null
sleep 2

SECS=$((MINUTES * 60))
"$SIM_HOST" --duration-secs "$SECS" >/tmp/drive-host.log 2>&1 &

cat <<'KEYS'

  ─────────────────────────────────────────────
   CLICK THE GAME WINDOW FIRST, then drive:

     W / Up      lean forward  (accelerate)
     S / Down    lean back     (slow, reverse)
     A / Left    carve left
     D / Right   carve right
     Space       arm       R   reset

   Lean and hold. Releasing coasts -- it does not brake.
  ─────────────────────────────────────────────

KEYS
echo "physics host up for ${MINUTES} min. Ctrl-C here, or quit the game, when done."
wait
