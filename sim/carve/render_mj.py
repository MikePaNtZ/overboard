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
W, H, FPS = 1920, 1080, 30

VIS = '''
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.46 0.62 0.84" rgb2="0.86 0.91 0.97" width="512" height="512"/>
'''
M = build('/tmp/carve_terrain_v2/carve_hfield.bin', visual_extra=VIS, render=True)
M.vis.global_.offwidth, M.vis.global_.offheight = W, H
M.vis.quality.shadowsize = 4096
# map distances are multiples of stat.extent (~213 m here), so use fractions.
EXT = M.stat.extent
M.vis.map.zfar = 80 / EXT          # clip at ~80 m
M.vis.map.znear = 0.5 / EXT
# Distance fog, sky-matched, hides the heightfield "wall" beyond the kerbs:
# it fades from ~5 m to ~22 m, so past the kerb only sky reads.
M.vis.map.fogstart = 5 / EXT
M.vis.map.fogend = 22 / EXT
M.vis.rgba.fog[:] = [0.66, 0.75, 0.86, 1.0]
# The directional sun (model.py) carries the scene; keep the headlight low.
M.vis.headlight.ambient[:] = [0.28, 0.29, 0.31]
M.vis.headlight.diffuse[:] = [0.25, 0.25, 0.25]
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
spx, spy = sm(px), sm(py)
hx, hy = np.gradient(spx), np.gradient(spy)
hn = np.hypot(hx, hy) + 1e-9
ux, uy = hx / hn, hy / hn           # unit travel direction (for the look-ahead)
head = np.degrees(np.arctan2(hy, hx))
# Freeze the camera yaw at the fall: after a crash the board tumbles, so the
# heading goes wild. Hold the last clean heading (and look-ahead) from there.
fb = ((d['flags'].astype(int) >> 2) & 1) if 'flags' in d else np.zeros(len(d['t']), int)
fall_t = float(d['t'][np.argmax(fb)]) if fb.any() else None
if fall_t is not None:
    jf = int(np.searchsorted(ts, fall_t))
    if 0 <= jf < len(head):
        head[jf:] = head[jf]
        ux[jf:], uy[jf:] = ux[jf], uy[jf]

frames = [ts[len(ts) // 2]] if still else ts
proc = None
if not still:
    proc = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                             '-s', f'{W}x{H}', '-r', str(FPS), '-i', '-', '-c:v', 'libx264',
                             '-pix_fmt', 'yuv420p', '-crf', '20', '-g', '30',
                             '-movflags', '+faststart', out], stdin=subprocess.PIPE)
for n, t in enumerate(frames):
    i = int(np.searchsorted(d['t'], t))
    set_state(M, D, d, i)
    mujoco.mj_forward(M, D)
    j = int(np.searchsorted(ts, t)); j = min(j, len(spx) - 1)
    # Close, low chase (~3 m back, ~1.2 m up), looking ~2.5 m ahead down the line.
    cam.lookat[:] = [spx[j] + 2.5 * ux[j], spy[j] + 2.5 * uy[j], D.qpos[2] + 0.6]
    cam.distance = 5.5
    cam.elevation = -14
    jl = min(j + 9, len(head) - 1)                 # slight lead into the turn
    cam.azimuth = 0.7 * head[j] + 0.3 * head[jl] + 180 + 6
    r.update_scene(D, cam, opt)
    r.scene.flags[int(mujoco.mjtRndFlag.mjRND_FOG)] = 1
    r.scene.flags[int(mujoco.mjtRndFlag.mjRND_SHADOW)] = 1
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
