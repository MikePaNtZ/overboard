# Downhill carve on the City Park loop road

Working scripts from the first carve session (2026-10-03). They still use `/tmp` paths; make
them path-relative before you depend on them.

| Script | What it does |
|---|---|
| `park_dump_full.py` | Unreal editor python: dumps every hard-surface triangle in OB_City to `/tmp/parkprobe` |
| `park_grade_survey.py` | Rasterises the dump; finds 6–8 % grade candidates; draws the park grade map |
| `park_rasterize_carve.py` | Heightmap centred on the carve start (UE −47725, −30575, ZCM cm; yaw −37.6°), 150 m, 5 cm posts |
| `merge.py` | Removes paint overlays (< 3 cm above wide road triangles); kerbs stay |
| `rider.py` | Closed-loop rider: soft speed hold + pure pursuit on a three-carve line; records the wire track |
| `rider_sine.py` | The earlier sine-weave rider (rejected: too many turns) |
| `record.py` | Records `sim-host` state packets (wire v3, 104 B) to `.npz` |
| `model.py` | Builds the same terrain-spliced MuJoCo model `sim-host` builds; `render=True` adds visual extras |
| `render_mj.py` | Replays a track in MuJoCo and encodes an mp4 (trailing camera) |
| `analyse2.py` | Path-in-lane plot and time series |
| `env.sh` | PATH, `MUJOCO_DIR`, `DYLD_LIBRARY_PATH`, `SIMHOST` |

Run one carve:

```bash
. sim/carve/env.sh
$PY sim/carve/rider.py out.npz 4.0 &
sleep 3
$SIMHOST --terrain <dir>/carve_hfield.bin --duration-secs 24 \
  --state-out-addr 127.0.0.1:9711 --input-in-addr 127.0.0.1:9712 --stats-path none --trace-csv trace.csv
```

Data for the published runs (tracks, traces, terrain) is in
`overboard-viz/out/carve-lab/data/` (gitignored).
