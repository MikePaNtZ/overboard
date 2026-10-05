#!/bin/bash
# Authority sweep: hold a speed on one constant grade, down and up, at several
# motor current limits. The board's own outer speed loop (--speed-hold) holds
# the speed; the rider is passive (all-zero schedule), so the result is the
# board's authority, not rider skill. Free-run, so it is fast.
# One line per run in OUT/summary.txt.
#   sim/carve/authority_sweep.sh OUT_DIR [VREF]
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
. "$HERE/env.sh"
OUT="$1"; V="${2:-3.0}"
mkdir -p "$OUT/courses" "$OUT/runs"
GRADES="${GRADES:-4 6 8 10 12 15}"
CURRENTS="${CURRENTS:-40 60 80}"
printf "0,60,0,0,0,passive rider\n" > "$OUT/passive.csv"
for dir in down up; do
  for g in $GRADES; do
    C="$OUT/courses/${dir}_$g"
    if [ ! -f "$C/course_hfield.bin" ]; then
      if [ "$dir" = down ]; then
        $PY "$HERE/course.py" "$C" --preset steady_down --descent "$g" --r-sag 100 --r-crest 100 > /dev/null
      else
        $PY "$HERE/course.py" "$C" --preset steady_up --climb "$g" --r-sag 100 --r-crest 100 > /dev/null
      fi
    fi
    for amps in $CURRENTS; do
      N="${dir}_${g}_${amps}A"
      R="$OUT/runs/$N"
      $PY "$HERE/record.py" 9711 "$R.npz" 120 > /dev/null 2>&1 &
      REC=$!
      sleep 1
      $SIMHOST --lean-steer --estimator-aiding grade-aware --max-current "$amps" --speed-hold "$V" \
        --spawn-x 88 --terrain "$C/course_hfield.bin" --schedule-csv "$OUT/passive.csv" \
        --duration-secs 45 --free-run \
        --state-out-addr 127.0.0.1:9711 --input-in-addr 127.0.0.1:9712 --stats-path none \
        --trace-csv "$R.csv" > "$R.host.txt" 2>&1
      wait $REC
      $PY "$HERE/authority_sum.py" "$N" "$R" "$amps" >> "$OUT/summary.txt"
      tail -1 "$OUT/summary.txt"
    done
  done
done
