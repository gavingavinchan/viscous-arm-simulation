"""Bake the existing arm's kinematics for a cinematic (not dynamics) animation."""
import json
import numpy as np
from scipy.spatial.transform import Rotation
from kinematics import ROOT, fk, inverse

OUT = ROOT / 'cinematic'
OUT.mkdir(exist_ok=True)
FPS, END = 24, 432


def smooth(a):
    a = np.clip(a, 0, 1)
    return a*a*a*(10+a*(-15+6*a))


def surface(x, z):
    return np.array([x, .13+.25*x, z])


normal = np.array([-.25, 1., 0.])
normal /= np.linalg.norm(normal)
across = np.array([-normal[1], normal[0], 0.])
scrub_rot = np.stack([across, np.cross(normal, across), normal], axis=1)
grip_rot = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
bases = [np.array([-.28, -.05, .16]), np.array([.17, -.26, .12])]
mounts = [np.eye(3), Rotation.from_euler('z', np.pi/2).as_matrix()]
# Three overlapping strokes on the downward blade.
strokes = [(100, .012, .365), (168, .012, .475),
           (188, .051, .475), (256, .051, .365),
           (276, .087, .365), (344, .087, .455)]


def pad_point(frame):
    if frame <= strokes[0][0]:
        return surface(*strokes[0][1:])
    for (a, x0, z0), (b, x1, z1) in zip(strokes, strokes[1:]):
        if frame <= b:
            u = smooth((frame-a)/(b-a))
            return surface(x0+(x1-x0)*u, z0+(z1-z0)*u)
    return surface(*strokes[-1][1:])


records = []
seeds = [np.array([-1.3, -1.3, .8, .65, 1.3, 0]),
         np.array([-.6, -1.4, .8, .75, .35, 0])]
max_residual = 0
grasp = surface(-.058, .41)
for f in range(1, END+1):
    grip_p = grasp - np.array([.012, .045, 0])*(1-smooth((f-12)/55))
    point = pad_point(f)
    separation = .060*(1-smooth((f-62)/38)) + .075*smooth((f-352)/42)
    scrub_p = point-normal*(.073+separation)
    arms = []
    for i, (p, rot) in enumerate([(grip_p, grip_rot), (scrub_p, scrub_rot)]):
        T = np.eye(4)
        T[:3, :3] = mounts[i].T @ rot
        T[:3, 3] = mounts[i].T @ (p-bases[i])
        q, info = inverse(T, seeds[i])
        max_residual = max(max_residual, info['residual_norm'])
        if info['residual_norm'] > 1e-5:
            raise RuntimeError((f, i, info))
        seeds[i] = q
        jaw = .020-(.020-.0035)*smooth((f-66)/22) if i == 0 else .012
        frames = fk([*q, jaw])
        B = np.eye(4); B[:3, :3] = mounts[i]; B[:3, 3] = bases[i]
        arms.append({k: (B@v).tolist() for k, v in frames.items()})
    records.append({'frame': f, 'arms': arms, 'pad': point.tolist()})

data = {'fps': FPS, 'end': END, 'frames': records,
        'max_ik_residual': max_residual, 'bases': [p.tolist() for p in bases],
        'contact_frames': [100, 344], 'pad_radius': .047,
        'note': 'Kinematic art animation using CAD joints; no underwater dynamics or contact-force claims.'}
(OUT/'motion.json').write_text(json.dumps(data, separators=(',', ':')))
print(f'Baked {END} frames; maximum IK residual {max_residual:.3g}')
