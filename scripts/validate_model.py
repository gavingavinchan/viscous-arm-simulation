from pathlib import Path
import os,json,xml.etree.ElementTree as ET
os.environ.setdefault('MUJOCO_GL','egl')
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from PIL import Image
from kinematics import fk,inverse,MODEL,NAMES,JOINTS,ROOT
OUT=ROOT/'inspection';OUT.mkdir(exist_ok=True)
m=mujoco.MjModel.from_xml_path(str(ROOT/'model/viscous_arm.xml'));d=mujoco.MjData(m)
rng=np.random.default_rng(98741)
report={'scope':'Offline numerical/model validation only; does not validate real motor calibration or physical parameters','mass_kg':float(m.body_mass.sum()),'nq':m.nq,'nu':m.nu}
# XML references must exist and tree must have a single root and no repeated child.
r=ET.parse(ROOT/'model/viscous_arm.urdf').getroot();links={l.get('name') for l in r.findall('link')};child=[j.find('child').get('link') for j in r.findall('joint')]
assert len(set(child))==len(child);assert links-set(child)=={'base_link'}
for mesh in r.findall('.//mesh'):assert (ROOT/'model'/mesh.get('filename')).is_file()
report['urdf_tree_and_assets']='pass'
# Independent scripted URDF-style FK versus actual MuJoCo compiled body frames.
maxpos=maxrot=0.
for k in range(100):
 q=np.array([rng.uniform(JOINTS[n]['lower']+.001,JOINTS[n]['upper']-.001) for n in NAMES])
 d.qpos[:6]=q[:6];d.qpos[6:]=q[6];mujoco.mj_forward(m,d);frames=fk(q)
 for name,T in frames.items():
  b=m.body(name).id;maxpos=max(maxpos,float(np.linalg.norm(T[:3,3]-d.xpos[b])));maxrot=max(maxrot,float(np.max(np.abs(T[:3,:3]-d.xmat[b].reshape(3,3)))))
assert maxpos<1e-8 and maxrot<1e-8
report['fk_100_poses']={'pass':True,'max_position_error_m':maxpos,'max_rotation_matrix_error':maxrot}
ik=[]
for k in range(12):
 q=np.array([rng.uniform(.1*JOINTS[n]['lower'],.65*JOINTS[n]['upper']) if n!='shoulder_lift' else rng.uniform(-2.2,-.4) for n in NAMES[:6]])
 q[2]=rng.uniform(.4,2.6);target=fk(q)['tool0'];sol,info=inverse(target,q+rng.normal(0,.10,6));actual=fk(sol)['tool0']
 pe=float(np.linalg.norm(actual[:3,3]-target[:3,3]));re=float(Rotation.from_matrix(actual[:3,:3].T@target[:3,:3]).magnitude());ik.append({'position_error_m':pe,'orientation_error_rad':re,**info})
assert max(x['position_error_m'] for x in ik)<1e-5 and max(x['orientation_error_rad'] for x in ik)<1e-5
report['ik_12_local_reachable_targets']={'pass':True,'max_position_error_m':max(x['position_error_m'] for x in ik),'max_orientation_error_rad':max(x['orientation_error_rad'] for x in ik),'note':'Local IK with perturbed known solution; not a global reachability guarantee'}
# CAD-zero and demo poses: contact geometry and static loads.
poses={'cad_zero':[0,0,0,0,0,0,0,0],'inspection_pose':[0,-.9,1.3,-.5,.1,0,.018,.018],'extended_pose':[0,-1.4,.8,.2,0,0,.035,.035]}
report['poses']={}
for name,q in poses.items():
 mujoco.mj_resetData(m,d);d.qpos[:]=q;mujoco.mj_forward(m,d)
 contacts=[{'a':m.geom(c.geom1).name,'b':m.geom(c.geom2).name,'penetration_m':float(min(0,c.dist))} for c in d.contact if c.dist<-.0001]
 report['poses'][name]={'contacts':contacts,'gravity_torque_Nm':d.qfrc_bias[:6].tolist(),'over_nominal_5Nm':bool(np.max(np.abs(d.qfrc_bias[:6]))>5)}
# Bounded-control dry-air smoke test in known clear inspection pose.
mujoco.mj_resetData(m,d);target=np.array(poses['inspection_pose']);d.qpos[:]=target;mujoco.mj_forward(m,d)
maxerr=0.;maxcoupling=0.;maxctrl=0.;saturated=0
for step in range(2000):
 desired=d.qfrc_bias[:6]+18*(target[:6]-d.qpos[:6])-1.2*d.qvel[:6]
 d.ctrl[:6]=np.clip(desired,-5,5);saturated+=int(np.any(np.abs(desired)>5))
 d.ctrl[6]=np.clip(d.qfrc_bias[6]+d.qfrc_bias[7]+300*(target[6]-d.qpos[6])-4*d.qvel[6],-10,10)
 mujoco.mj_step(m,d)
 assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()
 maxerr=max(maxerr,float(np.max(np.abs(d.qpos[:6]-target[:6]))));maxcoupling=max(maxcoupling,float(abs(d.qpos[6]-d.qpos[7])));maxctrl=max(maxctrl,float(np.max(np.abs(d.ctrl[:6]))))
report['two_second_hold']={'finite':True,'max_joint_error_rad':maxerr,'max_gripper_mimic_error_m':maxcoupling,'max_arm_command_Nm':maxctrl,'saturation_steps':saturated,'warning_counts':d.warning.number.tolist(),'note':'Gravity feedforward uses this same approximate model; numerical check, not hardware identification'}
assert maxerr<.01 and maxcoupling<1e-4
# Render on the Linux GPU through EGL; visual meshes group2, collisions hiddengroup3.
renderer=mujoco.Renderer(m,height=900,width=1200);cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE;cam.lookat[:]=[.02,0,.22];cam.distance=.92;cam.azimuth=135;cam.elevation=-24
opt=mujoco.MjvOption();opt.geomgroup[3]=0
for name,q in poses.items():
 mujoco.mj_resetData(m,d);d.qpos[:]=q;mujoco.mj_forward(m,d);renderer.update_scene(d,cam,scene_option=opt);Image.fromarray(renderer.render()).save(OUT/(name+'.png'))
renderer.close();report['renders']='3 PNGs generated on Linux via MuJoCo EGL'
(OUT/'validation_report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
