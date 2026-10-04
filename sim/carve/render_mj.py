"""Render a recorded carve track in MuJoCo: same model, same park terrain.

    python render_mj.py TRACK.npz OUT.mp4 [t0] [t1] [--still FRAME.png]

Replays recorded poses only. Nothing here steps the physics.
"""
import subprocess, sys
import numpy as np
import mujoco
sys.path.insert(0, '/tmp/carve')
from model import build, set_state

track, out = sys.argv[1], sys.argv[2]
t0 = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
t1 = float(sys.argv[4]) if len(sys.argv) > 4 else 1e9
still = sys.argv[sys.argv.index('--still') + 1] if '--still' in sys.argv else None
W, H, FPS = 1280, 720, 30

VIS = '''
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.55 0.72 0.92" rgb2="0.95 0.97 1.0" width="512" height="512"/>
'''
M = build('/tmp/carve_terrain_v2/carve_hfield.bin', visual_extra=VIS, render=True)
M.vis.global_.offwidth, M.vis.global_.offheight = W, H
M.vis.quality.shadowsize = 4096
M.vis.map.zfar = 400
M.vis.headlight.ambient[:] = [0.35, 0.35, 0.35]
M.vis.headlight.diffuse[:] = [0.6, 0.6, 0.6]
D = mujoco.MjData(M)
d = np.load(track)
r = mujoco.Renderer(M, H, W)
cam = mujoco.MjvCamera()
cam.type = mujoco.mjtCamera.mjCAMERA_FREE
opt = mujoco.MjvOption()

ts = np.arange(max(t0, d['t'][0]), min(t1, d['t'][-1]), 1 / FPS)
# smoothed chase target: follow the board's travel direction, not its wiggle
px, py = np.interp(ts, d['t'], d['px']), np.interp(ts, d['t'], d['py'])
k = 45
pad = lambda a: np.concatenate([np.full(k, a[0]), a, np.full(k, a[-1])])
sm = lambda a: np.convolve(pad(a), np.ones(2 * k + 1) / (2 * k + 1), 'valid')
hx, hy = np.gradient(sm(px)), np.gradient(sm(py))
head = np.degrees(np.arctan2(hy, hx))

frames = [ts[len(ts) // 2]] if still else ts
proc = None
if not still:
    proc = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                             '-s', f'{W}x{H}', '-r', str(FPS), '-i', '-', '-c:v', 'libx264',
                             '-pix_fmt', 'yuv420p', '-crf', '18', out], stdin=subprocess.PIPE)
for n, t in enumerate(frames):
    i = int(np.searchsorted(d['t'], t))
    set_state(M, D, d, i)
    mujoco.mj_forward(M, D)
    j = int(np.searchsorted(ts, t))
    cam.lookat[:] = [sm(px)[j], sm(py)[j], D.qpos[2] + 0.5]
    cam.distance = 9.0
    cam.elevation = -24
    cam.azimuth = head[j] + 180 + 8  # behind the smoothed line, so each carve reads as a sweep across the road
    r.update_scene(D, cam, opt)
    img = r.render()
    if still:
        from PIL import Image
        Image.fromarray(img).save(still)
    else:
        proc.stdin.write(img.tobytes())
if proc:
    proc.stdin.close()
    proc.wait()
print('rendered', len(frames), 'frames')
