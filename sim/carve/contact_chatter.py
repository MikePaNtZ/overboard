"""Wheel contact chatter: a 0.1454 m sphere wheel under an 80 kg frame, rolled
at constant speed on a flat plane and on a flat MuJoCo heightfield.

The frame slides in x and z only (no pitch), so its accelerometer shows the
contact force alone. A plane gives one steady contact. A heightfield gives
0-6 contacts that change at every grid edge, and the specific force swings
by several m/s^2, with lift-off. That chatter reached the board's IMU and
biased the pitch estimate, which is why the Monte Carlo rides a plane.

    $PY sim/carve/contact_chatter.py OUT.csv
"""
import csv
import struct
import sys
import tempfile
from pathlib import Path

import mujoco
import numpy as np

R = 0.1454


def run(ground, v, spacing=0.05):
    n = int(round(20 / spacing)) + 1
    tmp = Path(tempfile.mkdtemp())
    hf = tmp / f"h_{spacing}.bin"  # unique name: MuJoCo caches assets by path
    data = np.zeros((n, n), np.float32)
    data[0, 0] = 1.0  # the elevation scale needs a range; one corner post
    hf.write_bytes(struct.pack('<ii', n, n) + data.tobytes())
    g = ('<geom type="hfield" hfield="h"/>' if ground == 'hfield'
         else '<geom type="plane" size="0 0 1"/>')
    xml = f"""<mujoco><option timestep="0.002"/>
    <asset><hfield name="h" file="{hf}" size="10 10 1 1"/></asset>
    <worldbody>{g}
      <body pos="-8.013 0.017 {R}"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 0 1"/>
        <inertial pos="0 0 0" mass="80" diaginertia="5 5 5"/><site name="imu"/>
        <body><joint type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="3" diaginertia="0.03 0.03 0.03"/>
          <geom type="sphere" size="{R}" condim="3" friction="0.8 0.005 0.0001"/></body></body></worldbody>
    <sensor><accelerometer site="imu"/></sensor></mujoco>"""
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    d.qvel[0], d.qvel[2] = v, v / R
    fz, nc = [], []
    for k in range(1500):
        mujoco.mj_step(m, d)
        if k > 250:
            fz.append(d.sensordata[2])
            nc.append(d.ncon)
    return np.array(fz), np.array(nc)


def main():
    out = Path(sys.argv[1])
    rows = []
    for ground in ('plane', 'hfield'):
        for v in (0.5, 1, 2, 3, 4, 5, 6):
            fz, nc = run(ground, v)
            rows.append(dict(ground=ground, v_m_s=v, fz_mean=round(fz.mean(), 3),
                             fz_sd=round(fz.std(), 3), fz_min=round(fz.min(), 2),
                             fz_max=round(fz.max(), 2), contacts_min=int(nc.min()),
                             contacts_max=int(nc.max())))
            print(rows[-1])
    with open(out, 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


if __name__ == '__main__':
    main()
