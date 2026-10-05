"""Build the same terrain-spliced rider model sim-host builds, in Python.

`render=True` adds VISUAL-ONLY extras for replay videos: an asphalt grid
material and a stick-figure rider proxy on the ballast body. The ballast has
an explicit <inertial>, so the extra geoms change no mass property, and a
replay never steps the physics anyway.
"""
import json, re, numpy as np, mujoco
REPO = '/Users/mike/projects/overboard-carve'
FIGURE = '''
          <geom name="fig_torso" type="capsule" fromto="0 0 -0.05 0 0.02 0.38" size="0.11" rgba="0.15 0.35 0.65 1" contype="0" conaffinity="0"/>
          <geom name="fig_shoulders" type="capsule" fromto="-0.19 0.02 0.38 0.19 0.02 0.38" size="0.055" rgba="0.15 0.35 0.65 1" contype="0" conaffinity="0"/>
          <geom name="fig_head" type="sphere" pos="0 0.03 0.56" size="0.105" rgba="0.85 0.68 0.55 1" contype="0" conaffinity="0"/>
          <geom name="fig_helmet" type="sphere" pos="0 0.02 0.59" size="0.112" rgba="0.95 0.55 0.10 1" contype="0" conaffinity="0"/>
          <geom name="fig_thigh_f" type="capsule" fromto="-0.10 0 -0.08 -0.19 0.07 -0.38" size="0.07" rgba="0.12 0.12 0.14 1" contype="0" conaffinity="0"/>
          <geom name="fig_shin_f" type="capsule" fromto="-0.19 0.07 -0.38 -0.23 0 -0.66" size="0.055" rgba="0.12 0.12 0.14 1" contype="0" conaffinity="0"/>
          <geom name="fig_thigh_r" type="capsule" fromto="0.10 0 -0.08 0.19 0.07 -0.38" size="0.07" rgba="0.12 0.12 0.14 1" contype="0" conaffinity="0"/>
          <geom name="fig_shin_r" type="capsule" fromto="0.19 0.07 -0.38 0.23 0 -0.66" size="0.055" rgba="0.12 0.12 0.14 1" contype="0" conaffinity="0"/>
          <geom name="fig_arm_f" type="capsule" fromto="-0.19 0.02 0.38 -0.36 0.08 0.10" size="0.045" rgba="0.85 0.68 0.55 1" contype="0" conaffinity="0"/>
          <geom name="fig_arm_r" type="capsule" fromto="0.19 0.02 0.38 0.34 0.10 0.12" size="0.045" rgba="0.85 0.68 0.55 1" contype="0" conaffinity="0"/>
'''
ASPHALT = ('<texture name="grid" type="2d" builtin="checker" rgb1="0.185 0.195 0.205" rgb2="0.165 0.175 0.185" '
           'mark="edge" markrgb="0.26 0.27 0.28" width="512" height="512"/>')

# A directional sun with shadows, plus the rider-lean body (see build()).
SUN = ('<light name="sun" directional="true" castshadow="true" pos="0 0 25" '
       'dir="0.35 0.25 -1" diffuse="0.95 0.92 0.85" specular="0.25 0.25 0.22"/>')


def build(hfield='/tmp/carve_terrain_v2/carve_hfield.bin', visual_extra='', render=False):
    xml = open(f'{REPO}/sim/models/overboard_rider.xml').read()
    m = json.load(open(hfield.rsplit('/', 1)[0] + '/metadata.json'))
    he = m['half_extent_m']; zmin, zmax = m['z_min_m'], m['z_max_m']
    h = np.load(hfield.replace('_hfield.bin', '_height.npy')); c = (h.shape[0] - 1) // 2; z0 = float(h[c, c])
    xml = xml.replace('</asset>', f'<hfield name="citypark" file="{hfield}" size="{he} {he} {zmax-zmin} 2.0"/>\n{visual_extra}</asset>')
    xml = xml.replace('<geom name="ground" type="plane" size="20 20 0.1" material="ground_mat"/>',
        f'<geom name="ground" type="hfield" hfield="citypark" pos="0 0 {zmin}" material="ground_mat" condim="3" friction="0.8 0.005 0.0001"/>')
    xml = xml.replace('<body name="frame" pos="0 0 0.1454">', f'<body name="frame" pos="0 0 {0.1454+z0+0.005}">')
    if render:
        xml = re.sub(r'<texture name="grid"[^>]*/>', ASPHALT, xml, flags=re.S)
        xml = xml.replace('texrepeat="160 160"', f'texrepeat="{2*he} {2*he}"')  # 0.5 m squares over the 2*he field
        # Directional sun with shadows.
        xml = xml.replace('<worldbody>', '<worldbody>\n        ' + SUN, 1)
        # Rider figure on a visual-only roll joint, so lean reads as a body tilt
        # (replay sets fig_roll from rider_lat; it changes no mass property).
        leaned = ('<body name="fig_lean" pos="0 0 0">'
                  '<joint name="fig_roll" type="hinge" axis="1 0 0" pos="0 0 -0.66" '
                  'limited="false" damping="0" stiffness="0"/>'
                  '<inertial pos="0 0 0" mass="1e-6" diaginertia="1e-8 1e-8 1e-8"/>'
                  + FIGURE + '</body>')
        xml = re.sub(r'(<geom name="ballast_mass_geom"[^>]*/>)', r'<!-- \1 -->' + leaned, xml, flags=re.S)
    xml = re.sub(r'(file|mesh)dir="([^/"][^"]*)"', lambda mm: f'{mm.group(1)}dir="{REPO}/sim/models/{mm.group(2)}"', xml)
    xml = re.sub(r'file="(meshes/[^"]+)"', lambda mm: f'file="{REPO}/sim/models/{mm.group(1)}"', xml)
    return mujoco.MjModel.from_xml_string(xml)


def set_state(M, D, d, i):
    D.qpos[:7] = [d['px'][i], d['py'][i], d['pz'][i], d['qw'][i], d['qx'][i], d['qy'][i], d['qz'][i]]
    for jn, key in (('wheel_hinge', 'wheel_angle'), ('ballast_fa', 'rider_fa'), ('ballast_lat', 'rider_lat')):
        D.qpos[M.joint(jn).qposadr[0]] = d[key][i]
    # Visual rider lean: roll the figure ~6 rad per metre of lateral slide.
    try:
        D.qpos[M.joint('fig_roll').qposadr[0]] = -float(d['rider_lat'][i]) * 6.0
    except (KeyError, Exception):
        pass
