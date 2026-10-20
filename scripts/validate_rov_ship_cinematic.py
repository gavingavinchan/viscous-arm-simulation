"""Check that the film retains the real ROV geometry and mounting arrangement."""
from pathlib import Path
import hashlib
import json
import sys
import bpy
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ROV = ROOT.parent/'viscous-rov-model'
OUT = ROOT/'cinematic/rov_ship'


def snapshot():
    meshes = {}
    for obj in bpy.context.scene.objects:
        if obj.type == 'MESH':
            coords = np.empty(len(obj.data.vertices)*3, dtype=np.float32)
            obj.data.vertices.foreach_get('co', coords)
            meshes[obj.name] = {'vertices': len(obj.data.vertices), 'polygons': len(obj.data.polygons),
                                'sha256': hashlib.sha256(coords.tobytes()).hexdigest(),
                                'parent': obj.parent.name if obj.parent else None}
    # matrix_basis stores the original local input; matrix_local can pick up
    # float32 roundoff after translating the complete rig into ship coordinates.
    mounts = {n: np.asarray(bpy.data.objects[n].matrix_basis).copy()
              for n in ['left_mount_origin', 'right_mount_origin']}
    return meshes, mounts


bpy.ops.wm.open_mainfile(filepath=str(ROV/'blender/viscous_rov.blend'))
original, mounts = snapshot()
bpy.ops.wm.open_mainfile(filepath=str(OUT/'viscous_rov_ship_cleaning.blend'))
current, current_mounts = snapshot()
diffs = [name for name, data in original.items() if current.get(name) != data]
mount_error = max(float(np.max(np.abs(mounts[n]-current_mounts[n]))) for n in mounts)
rudders = [o.name for o in bpy.context.scene.objects if 'rudder foil' in o.name]
props = [o.name for o in bpy.context.scene.objects if 'cast bronze hub' in o.name]
scale_error = float(np.max(np.abs(np.asarray(bpy.data.objects['rov_base'].scale)-1)))
sys.path.insert(0, str(ROV/'scripts'))
from bimanual_common import JD, fk
motion = json.loads((OUT/'motion.json').read_text())
limit_error = 0
contact_error = 0
grip_poses = []
M = np.array(motion['rov_transform'])
for record in motion['frames']:
    for name, q in record['joints'].items():
        limit = JD[name].find('limit')
        limit_error = max(limit_error, float(limit.get('lower'))-q, q-float(limit.get('upper')))
    if 150 <= record['frame'] <= 394:
        q = [record['joints']['right_'+n] for n in ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_yaw', 'wrist_roll']]
        T = M@fk('right', q)
        pad = T[:3, 3]+T[:3, 2]*.073
        contact_error = max(contact_error, float(np.linalg.norm(pad-record['pad'])))
    if record['frame'] >= 130:
        q = [record['joints']['left_'+n] for n in ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_yaw', 'wrist_roll']]
        grip_poses.append(fk('left', q))
drift = max(float(np.linalg.norm(T-grip_poses[0])) for T in grip_poses)
report = {'source_rig': str(ROV/'blender/viscous_rov.blend'), 'original_mesh_instances': len(original),
          'mesh_or_parent_differences': diffs, 'max_mount_basis_difference': mount_error,
          'propellers': props, 'rudder_foils': rudders, 'vehicle_scale_error': scale_error,
          'max_joint_limit_excursion': limit_error, 'max_pad_target_error_m': contact_error,
          'anchor_drift_after_grasp': drift, 'max_ik_residual': motion['max_ik_residual']}
report['passed'] = not diffs and mount_error < 1e-9 and len(rudders) == 1 and len(props) == 2 and scale_error < 1e-6 and limit_error < 1e-8 and contact_error < 1e-6 and drift < 1e-8
(OUT/'asset_review.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
assert report['passed'], report
