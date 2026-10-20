"""Validate embedded URDF rig, mimic, level framing and hero image bounds."""
from pathlib import Path
import argparse,json,hashlib
import bpy
import numpy as np
from mathutils import Vector
from bpy_extras.object_utils import world_to_camera_view
from kinematics import fk,NAMES,JOINTS
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--finish',choices=['silver','black'],default='silver');a=p.parse_args();out=ROOT/'cinematic/launch'/a.finish
bpy.ops.wm.open_mainfile(filepath=str(out/'viscous_launch.blend'));s=bpy.context.scene;ctrl=bpy.data.objects['POSE | animate these URDF joint values'];motion=json.loads((out/'motion.json').read_text())
max_fk=0.;max_mimic=0.;max_center=0.;max_roll=0.;q_all=[];hero=[];cameras=[]
for f in range(1,769):
 s.frame_set(f);bpy.context.view_layer.update();q=np.array([ctrl[n] for n in NAMES]);q_all.append(q);cameras.append(np.array(s.camera.matrix_world).ravel().tolist()+[s.camera.data.lens])
 for n,v in zip(NAMES,q):assert JOINTS[n]['lower']-1e-7<=v<=JOINTS[n]['upper']+1e-7,(f,n,v)
 truth=fk(q)
 for name,T in truth.items():max_fk=max(max_fk,float(np.max(np.abs(np.array(bpy.data.objects[name].matrix_world)-T))))
 l=bpy.data.objects['gripper_left'].location.y;r=bpy.data.objects['gripper_right'].location.y;max_mimic=max(max_mimic,abs(l+r))
 if 97<=f<=192:
  c=bpy.data.objects['02_grip'];tcp=Vector(truth['tool0'][:3,3]);p0=world_to_camera_view(s,c,tcp);p1=world_to_camera_view(s,c,tcp+Vector(truth['tool0'][:3,0])*.02)
  max_center=max(max_center,abs(p0.x-.5),abs(p0.y-.5));max_roll=max(max_roll,abs(p1.y-p0.y))
 if f>=625 and f%6==0:
  points=[world_to_camera_view(s,s.camera,o.matrix_world@Vector(v)) for o in bpy.data.collections['01 | Viscous — original CAD'].objects for v in o.bound_box]
  hero.append({'frame':f,'xmin':min(p.x for p in points),'xmax':max(p.x for p in points),'ymin':min(p.y for p in points),'ymax':max(p.y for p in points)})
assert max_fk<2e-6,max_fk
assert max_mimic<1e-7,max_mimic
assert max_center<1e-5,max_center
assert max_roll<1e-5,max_roll
assert all(min(x['xmin'],x['ymin'])>.03 and max(x['xmax'],x['ymax'])<.97 for x in hero),hero
speed=np.max(np.abs(np.diff(q_all,axis=0))*24,axis=0)
assert all(v<=JOINTS[n].get('velocity',2 if i<6 else .05)+1e-5 for i,(n,v) in enumerate(zip(NAMES,speed))),speed
geometry=hashlib.sha256()
for o in sorted(bpy.data.collections['01 | Viscous — original CAD'].objects,key=lambda x:x.name):
 geometry.update(o.name.encode());arr=np.empty(len(o.data.vertices)*3,dtype=np.float32);o.data.vertices.foreach_get('co',arr);geometry.update(arr.tobytes())
report={'rig_animation_sha256':hashlib.sha256(np.array(q_all).tobytes()).hexdigest(),'camera_animation_sha256':hashlib.sha256(np.array(cameras).tobytes()).hexdigest(),'geometry_sha256':geometry.hexdigest(),'passed':True,'frames_checked':768,'max_fk_matrix_error':max_fk,'max_mimic_m':max_mimic,'front_center_error_normalized':max_center,'front_horizontal_error_normalized':max_roll,'maximum_joint_speed':dict(zip(NAMES,speed.tolist())),'hero_bounds':hero,'visual_parts':len(motion['materials']),'note':'Kinematic/visual validation only; no physical contact or dynamics guarantee.'}
(out/'validation_report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
