"""Pose the existing complete bimanual ROV; preserve its native arm mounts."""
from pathlib import Path
import json
import sys
import numpy as np
from scipy.spatial.transform import Rotation
from ship_stern_asset import blade_surface, blade_width

ROOT = Path(__file__).resolve().parents[1]
ROV = ROOT.parent/'viscous-rov-model'
sys.path.insert(0, str(ROV/'scripts'))
from bimanual_common import inverse, fk, NAMES
OUT = ROOT/'cinematic/rov_ship'; OUT.mkdir(parents=True, exist_ok=True)
END, FPS = 480, 24
U, EDGE = .68, .994
V = EDGE-.19/blade_width(U)
PI, BI = 0, 2


def smooth(a):
    a = np.clip(a, 0, 1)
    return a*a*a*(10+a*(-15+6*a))


def point(u, v):
    return tuple(np.array(x, dtype=float) for x in blade_surface(PI, BI, u, v))


C, N = point(U, V)
A, AN = point(U, EDGE)
forward = -N
left = np.cross([0, 0, 1], forward); left /= np.linalg.norm(left)
up = np.cross(forward, left)
B = np.stack([forward, left, up], axis=1)
base = C-B@np.array([.53, -.04, -.02])
vehicle = np.eye(4); vehicle[:3, :3] = B; vehicle[:3, 3] = base
# A diagonal edge pinch permits the existing short arm to reach without remounting.
theta = np.radians(60)
grip_z = np.array([np.cos(theta), -np.sin(theta), 0])
grip_x = np.array([np.sin(theta), np.cos(theta), 0])
grip_R = np.stack([grip_x, np.cross(grip_z, grip_x), grip_z], axis=1)
strokes = [(150, .655, V), (218, .705, V), (238, .705, V-.029),
           (306, .655, V-.029), (326, .655, V-.058), (394, .697, V-.058)]


def uv_at(frame):
    for (a, u0, v0), (b, u1, v1) in zip(strokes, strokes[1:]):
        if frame <= b:
            t = smooth((frame-a)/(b-a))
            return u0+(u1-u0)*t, v0+(v1-v0)*t
    return strokes[-1][1:]


seeds = {'left': [0, .8, -1, 1.2, 1, 0], 'right': [1, .7, -1.1, 1.4, -1, 0]}
records, errors = [], []
for f in range(1, END+1):
    P, normal = point(*uv_at(f))
    separation = .035*(1-smooth((f-114)/36))+.06*smooth((f-402)/42)
    tool_pos = P+normal*(.073+separation)
    tool_z = -normal; tool_x = -left.copy(); tool_x -= tool_x.dot(tool_z)*tool_z
    tool_x /= np.linalg.norm(tool_x)
    tool_R = np.stack([tool_x, np.cross(tool_z, tool_x), tool_z], axis=1)
    anchor = B.T@(A-base)
    anchor -= np.array([.020, 0, .005])*(1-smooth((f-80)/32))
    targets = {'left': (anchor, grip_R), 'right': (B.T@(tool_pos-base), B.T@tool_R)}
    joints = {}
    for side, (pos, rot) in targets.items():
        q, error = inverse(side, pos, rot, seeds[side])
        if error > 1e-5:
            raise RuntimeError((f, side, error, pos.tolist()))
        seeds[side] = q
        joints.update(zip(NAMES[side], q.tolist())); errors.append(float(error))
    joints['left_gripper_left_joint'] = .022-(.022-.004)*smooth((f-110)/20)
    joints['right_gripper_left_joint'] = .012
    records.append({'frame': f, 'joints': joints, 'pad': P.tolist(), 'normal': normal.tolist(), 'uv': list(uv_at(f))})

data = {'fps': FPS, 'end': END, 'frames': records, 'rov_transform': vehicle.tolist(),
        'source_rig': str(ROV/'blender/viscous_rov.blend'), 'source_urdf': str(ROV/'model/viscous_rov.urdf'),
        'work': {'prop_index': PI, 'blade_index': BI, 'u': U, 'v': V,
                 'center': C.tolist(), 'normal': N.tolist(), 'left': left.tolist(), 'up': up.tolist(),
                 'anchor': A.tolist(), 'anchor_normal': AN.tolist()},
        'contact_frames': [150, 394], 'pad_radius': .047,
        'max_ik_residual': max(errors),
        'note': 'Native mounted ROV rig; cinematic joint animation, not underwater dynamics.'}
(OUT/'motion.json').write_text(json.dumps(data, separators=(',', ':')))
print('Native ROV motion baked:', len(records), 'frames; max IK residual', max(errors))
