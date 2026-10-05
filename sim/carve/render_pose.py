"""Render a sim-host `--pose-out` log in MuJoCo, low resolution.

    python render_pose.py POSE.csv OUT.mp4 [--title TEXT] [--side] [--t0 S] [--t1 S]

Replays the poses sim-host computed, with the model sim-host used (named in
the log's first line). Nothing here steps the physics, with one exception
that the video labels: after a `--rider-reacts` dismount the rider's step-off
is DRAWN (a 0.8 s move to stand beside the board), because the rider model
has no legs to step with. A fall (`--tumble`) is not drawn: MuJoCo computes
the rider's tumble and slide.
"""
import argparse, subprocess
import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont

ap = argparse.ArgumentParser()
ap.add_argument('pose')
ap.add_argument('out')
ap.add_argument('--title', default='')
ap.add_argument('--side', action='store_true', help='camera beside the board, not behind it')
ap.add_argument('--az', type=float, help='camera azimuth from the travel direction, deg (behind 180, side 90 or -90)')
ap.add_argument('--dist', type=float, help='camera distance, m')
ap.add_argument('--t0', type=float, default=0.0)
ap.add_argument('--t1', type=float, default=1e9)
ap.add_argument('--hold', type=float, default=1.0, help='hold the last frame, s')
a = ap.parse_args()

W, H, FPS = 960, 540, 30
model_path = open(a.pose).readline().split('=', 1)[1].strip()
rows = np.loadtxt(a.pose, delimiter=',', comments='#')
t, ev, spd, amps = rows[:, 0], rows[:, 1].astype(int), rows[:, 2], rows[:, 3]
rider, qpos = rows[:, 4:11], rows[:, 11:]

M = mujoco.MjModel.from_xml_path(model_path)
M.vis.global_.offwidth, M.vis.global_.offheight = W, H
# Clip planes are fractions of the model extent (about 750 m with a kerb), so
# set them in metres: near 0.05 m, far 150 m.
M.vis.map.znear = 0.05 / M.stat.extent
M.vis.map.zfar = 150 / M.stat.extent
D = mujoco.MjData(M)
# The smooth-contact plate under the tyre is a physics helper, not scenery.
_plate = mujoco.mj_name2id(M, mujoco.mjtObj.mjOBJ_BODY, 'wheel_ground')
if _plate >= 0:
    M.geom_rgba[M.geom_bodyid == _plate, 3] = 0.0
adr = lambda j: M.jnt_qposadr[mujoco.mj_name2id(M, mujoco.mjtObj.mjOBJ_JOINT, j)]
free_a, hip_a = adr('rider_free_j'), adr('rider_hip')
frame_b = mujoco.mj_name2id(M, mujoco.mjtObj.mjOBJ_BODY, 'frame')
assert qpos.shape[1] == M.nq, (qpos.shape, M.nq)

r = mujoco.Renderer(M, H, W)
cam = mujoco.MjvCamera()
cam.type = mujoco.mjtCamera.mjCAMERA_FREE
opt = mujoco.MjvOption()
opt.geomgroup[1] = 0          # the ballast sphere: the tumble rider is drawn instead
opt.geomgroup[5] = 0          # the old board visuals

ts = np.arange(max(a.t0, t[0]), min(a.t1, t[-1]), 1 / FPS)
idx = np.clip(np.searchsorted(t, ts), 0, len(t) - 1)
# Camera target: the board until the fall, then midway to the rider.
bx, by = qpos[idx, 0], qpos[idx, 1]
rx, ry = rider[idx, 0], rider[idx, 1]
free = (ev[idx] & 1) == 1
cx, cy = np.where(free, 0.5 * (bx + rx), bx), np.where(free, 0.5 * (by + ry), by)
k = 20
sm = lambda x: np.convolve(np.concatenate([np.full(k, x[0]), x, np.full(k, x[-1])]),
                           np.ones(2 * k + 1) / (2 * k + 1), 'valid')
cx, cy = sm(cx), sm(cy)
hx, hy = np.gradient(sm(bx)), np.gradient(sm(by))
head = np.degrees(np.arctan2(hy, hx))
moving = np.hypot(hx, hy) * FPS > 0.3
for j in range(1, len(head)):           # hold the heading when stopped or after the fall
    if not moving[j] or (ev[idx[j]] & 5):
        head[j] = head[j - 1]
head = np.degrees(np.unwrap(np.radians(head)))

def step_off_pose(i, since):
    """Drawn step-off: from the ballast to standing beside the board."""
    s = min(1.0, since / 0.8)
    s = s * s * (3 - 2 * s)
    i = i_dis   # the rider steps off where they were, not where the board goes
    p0 = rider[i, :3]
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, qpos[i, 3:7])
    R = R.reshape(3, 3)
    right = R[:, 1] * np.array([1, 1, 0])
    right /= np.linalg.norm(right) + 1e-9
    ground_z = qpos[i, 2] - 0.146
    p1 = np.array([p0[0], p0[1], ground_z + 0.69]) + 0.5 * right
    yaw = np.arctan2(R[1, 0], R[0, 0])
    q1 = np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])
    return (1 - s) * p0 + s * p1, q1 if s > 0.5 else rider[i, 3:7]

font = ImageFont.load_default(size=22)
small = ImageFont.load_default(size=16)
proc = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                         '-s', f'{W}x{H}', '-r', str(FPS), '-i', '-', '-c:v', 'libx264',
                         '-pix_fmt', 'yuv420p', '-crf', '22', '-movflags', '+faststart', a.out],
                        stdin=subprocess.PIPE)
dis_t = t[np.argmax((ev & 2) == 2)] if (ev & 2).any() else None
i_dis = int(np.argmax((ev & 2) == 2))
fall_t = t[np.argmax((ev & 4) == 4)] if (ev & 4).any() else None
n_hold = int(a.hold * FPS)
for j, i in enumerate(list(idx) + [idx[-1]] * n_hold):
    jj = min(j, len(ts) - 1)
    D.qpos[:] = qpos[i]
    mujoco.mj_kinematics(M, D)
    if not ev[i] & 1:
        if dis_t is not None and t[i] >= dis_t:
            p, q = step_off_pose(i, t[i] - dis_t)
        else:
            p, q = rider[i, :3], rider[i, 3:7]
        D.qpos[free_a:free_a + 7] = [*p, *q]
        D.qpos[hip_a:hip_a + 4] = [1, 0, 0, 0]
    mujoco.mj_forward(M, D)
    cam.lookat[:] = [cx[jj], cy[jj], qpos[i, 2] + 0.35]
    cam.distance = a.dist or (4.2 if a.side else 5.0)
    cam.elevation = -10 if a.side else -16
    cam.azimuth = head[jj] + (a.az if a.az is not None else (90 if a.side else 180))
    r.update_scene(D, cam, opt)
    img = Image.fromarray(r.render())
    g = ImageDraw.Draw(img)
    g.text((16, 12), a.title, font=font, fill=(255, 255, 255), stroke_width=2, stroke_fill=(0, 0, 0))
    g.text((16, 42), f"t {t[i]:5.2f} s   {abs(spd[i]) * 2.237:4.1f} mph (wheel)   motor {amps[i]:+5.0f} A",
           font=small, fill=(255, 255, 255), stroke_width=2, stroke_fill=(0, 0, 0))
    note = ''
    if fall_t is not None and t[i] >= fall_t:
        note = f"FALL at {fall_t:.2f} s: motor cut, MuJoCo computes the rider's tumble"
    if dis_t is not None and t[i] >= dis_t:
        note = f"STEP-OFF at {dis_t:.2f} s: motor cut (the step-off is drawn, not simulated)"
    if note:
        g.text((16, H - 34), note, font=small, fill=(255, 210, 120), stroke_width=2, stroke_fill=(0, 0, 0))
    g.text((W - 250, H - 26), 'MuJoCo, low resolution', font=small, fill=(220, 220, 220),
           stroke_width=2, stroke_fill=(0, 0, 0))
    proc.stdin.write(img.tobytes())
proc.stdin.close()
proc.wait()
print('rendered', len(idx) + n_hold, 'frames to', a.out)
