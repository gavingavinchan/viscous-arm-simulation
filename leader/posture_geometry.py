"""Compare corresponding URDF link postures. Model angles only, never motors.

The CAD bases share their planar X/Z convention. Align upper-arm/forearm
directions and the two wrist axes, without scaling either link's length.
Roll zero is a model-body convention, not an alignment of handle and jaws.
"""
import hashlib
import json
import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
FOLLOWER_NAMES = ('shoulder_pan', 'shoulder_lift', 'elbow_flex',
                  'wrist_flex', 'wrist_yaw', 'wrist_roll')
LEADER_NAMES = tuple(f'joint{i}' for i in range(1, 7))


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def unit(v):
    length = math.sqrt(dot(v, v))
    if length < 1e-12:
        raise ValueError('Degenerate direction')
    return tuple(x/length for x in v)


def matvec(m, v):
    return tuple(dot(row, v) for row in m)


def matmul(a, b):
    return tuple(tuple(dot(row, col) for col in zip(*b)) for row in a)


IDENTITY = ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))


def rotation(axis, angle):
    x, y, z = unit(axis)
    c, s, t = math.cos(angle), math.sin(angle), 1-math.cos(angle)
    return ((t*x*x+c, t*x*y-s*z, t*x*z+s*y),
            (t*x*y+s*z, t*y*y+c, t*y*z-s*x),
            (t*x*z-s*y, t*y*z+s*x, t*z*z+c))


def vector(node, attr, default='0 0 0'):
    return tuple(map(float, node.get(attr, default).split())) if node is not None else tuple(map(float, default.split()))


class Chain:
    def __init__(self, path, names):
        self.path, self.names = Path(path), names
        root = ET.parse(self.path).getroot()
        self.nodes = root.findall('joint')
        children = {j.find('child').get('link') for j in self.nodes}
        roots = {link.get('name') for link in root.findall('link')} - children
        if len(roots) != 1:
            raise ValueError('Expected rooted URDF')
        self.root = roots.pop()
        self.by_name = {j.get('name'): j for j in self.nodes}

    def fk(self, degrees):
        values = dict(zip(self.names, degrees))
        if len(degrees) != 6 or not all(math.isfinite(v) for v in degrees):
            raise ValueError('Expected six finite model angles')
        frames = {self.root: (IDENTITY, (0., 0., 0.))}
        result, pending = {}, list(self.nodes)
        while pending:
            ready = [j for j in pending if j.find('parent').get('link') in frames]
            if not ready:
                raise ValueError('Invalid URDF tree')
            for j in ready:
                parent_r, parent_p = frames[j.find('parent').get('link')]
                origin = j.find('origin')
                r, p, y = vector(origin, 'rpy')
                local_r = matmul(rotation((0, 0, 1), y), matmul(rotation((0, 1, 0), p), rotation((1, 0, 0), r)))
                world_r = matmul(parent_r, local_r)
                offset = matvec(parent_r, vector(origin, 'xyz'))
                position = tuple(a+b for a, b in zip(parent_p, offset))
                axis = vector(j.find('axis'), 'xyz', '1 0 0')
                name = j.get('name')
                result[name] = {'position': position, 'axis': matvec(world_r, axis)}
                value = values.get(name, 0.)
                if j.get('type') == 'revolute':
                    world_r = matmul(world_r, rotation(axis, math.radians(value)))
                elif j.get('type') not in ('fixed', 'prismatic'):
                    raise ValueError('Unsupported joint type')
                # Fingers remain at their CAD datum; not part of this arm comparison.
                frames[j.find('child').get('link')] = world_r, position
                pending.remove(j)
        return result


def direction(snapshot, first, second):
    return unit(tuple(b-a for a, b in zip(snapshot[first]['position'], snapshot[second]['position'])))


def signed_angle(a, b, axis):
    return math.degrees(math.atan2(dot(unit(axis), cross(unit(a), unit(b))), dot(unit(a), unit(b))))


def derive():
    leader = Chain(ROOT/'leader/model/star_arm_102_hd.urdf', LEADER_NAMES)
    follower = Chain(ROOT/'model/viscous_arm.urdf', FOLLOWER_NAMES)
    l, f = leader.fk([0]*6), follower.fk([0]*6)
    pitch = f['shoulder_lift']['axis']
    upper = signed_angle(direction(f, 'shoulder_lift', 'elbow_flex'), direction(l, 'joint2', 'joint3'), pitch)
    forearm = signed_angle(direction(f, 'elbow_flex', 'wrist_flex'), direction(l, 'joint3', 'joint4'), pitch)
    wrist = signed_angle(f['wrist_yaw']['axis'], l['joint5']['axis'], pitch)
    offsets = [0., upper, forearm-upper, wrist-forearm, 0., 0.]
    aligned = follower.fk(offsets)
    signs = [1. if dot(aligned[fn]['axis'], l[ln]['axis']) > 0 else -1.
             for fn, ln in zip(FOLLOWER_NAMES, LEADER_NAMES)]
    return leader, follower, signs, offsets


def angular_error(a, b):
    return math.degrees(math.atan2(math.sqrt(dot(cross(unit(a), unit(b)), cross(unit(a), unit(b)))), dot(unit(a), unit(b))))


def report():
    leader, follower, signs, offsets = derive()
    rng = random.Random(102)
    poses = [[0.]*6, [25., 55., -95., 22., -28., 15.]] + [[rng.uniform(-75, 75) for _ in range(6)] for _ in range(100)]
    worst_axis = worst_upper = worst_forearm = 0.
    for q in poses:
        l = leader.fk(q)
        f = follower.fk([s*v+b for s, v, b in zip(signs, q, offsets)])
        for i, (fn, ln) in enumerate(zip(FOLLOWER_NAMES, LEADER_NAMES)):
            worst_axis = max(worst_axis, angular_error(tuple(signs[i]*v for v in f[fn]['axis']), l[ln]['axis']))
        worst_upper = max(worst_upper, angular_error(direction(f, 'shoulder_lift', 'elbow_flex'), direction(l, 'joint2', 'joint3')))
        worst_forearm = max(worst_forearm, angular_error(direction(f, 'elbow_flex', 'wrist_flex'), direction(l, 'joint3', 'joint4')))
    return {'scope': 'URDF geometry only; no encoder conversion, motor commands, collision or hardware-limit validation',
            'base_alignment': 'CAD base X/Y directions aligned; no world-position or tool-pose matching',
            'roll_datum': 'Model body roll zero retained; handle and jaw tool frames are different',
            'physically_verified': False, 'poses_checked': len(poses),
            'formula': 'follower_CAD_deg = direction * leader_CAD_deg + offset_deg',
            'rules': {n: {'direction': s, 'offset_deg': b} for n, s, b in zip(FOLLOWER_NAMES, signs, offsets)},
            'max_axis_direction_error_deg': worst_axis, 'max_upper_arm_direction_error_deg': worst_upper,
            'max_forearm_direction_error_deg': worst_forearm,
            'urdf_sha256': {str(c.path.relative_to(ROOT)): hashlib.sha256(c.path.read_bytes()).hexdigest() for c in (leader, follower)}}


if __name__ == '__main__':
    result = report()
    path = ROOT/'leader/inspection/posture_geometry_validation.json'
    path.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
