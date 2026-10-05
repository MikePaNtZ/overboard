"""Convert the game's course elements (overboard-game tools/play/elements/
<course>.json) into the obstacle CSV that sim-host reads (--obstacles).

    python3 sim/carve/obstacles.py ELEMENTS.json > obstacles.csv

Only physical obstacles are kept (cones and debris); gates, flags and zones
are drawn by the game and are not bodies. No board physics here: sim-host
makes the bodies, the game draws them at the same place.

One CSV row per obstacle, in the MuJoCo frame. The full format has 11 columns:

    type,id,x_m,y_m,lx_m,ly_m,lz_m,yaw_deg,pitch_deg,roll_deg,z_m

lx/ly/lz are full sizes; a cone gets its standard size. Columns 9-11
(pitch_deg, roll_deg, z_m) are optional and may each be empty; a row with 8
columns is still valid. z_m is the absolute world z of the box BOTTOM before
rotation; absent or empty, the bottom sits on the terrain at (x, y). Pitch and
roll apply to the `box` and `debris` types only. Types: `cone`, `debris` and
the generic grey `box` (ramps, planks, kerbs). This converter keeps writing 8
columns; sim-host reads all 11.
"""
import json
import sys

CONE_SIZE_M = (0.30, 0.30, 0.45)    # a traffic cone: 0.3 m base, 0.45 m tall


def main():
    d = json.load(open(sys.argv[1]))
    x0 = float(d['start_x_m'])
    print('# type,id,x_m,y_m,lx_m,ly_m,lz_m,yaw_deg  (from ' + sys.argv[1] + ')')
    for e in d['elements']:
        if e.get('type') == 'cone':
            lx, ly, lz = CONE_SIZE_M
        elif e.get('type') == 'debris':
            lx, ly, lz = e['size_m']
        else:
            continue
        x = x0 - float(e['s'])
        print(f"{e['type']},{e.get('id', '')},{x:.3f},{float(e['y']):.3f},{lx},{ly},{lz},{float(e.get('yaw_deg', 0.0))}")


if __name__ == '__main__':
    main()
