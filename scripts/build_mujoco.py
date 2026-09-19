"""Standalone dry-air MuJoCo model, bounded torque motors, explicit assumptions."""
from pathlib import Path
import json,xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'model';m=json.loads((OUT/'model_manifest.json').read_text())
f=lambda v:' '.join(f'{x:.12g}' for x in v)
r=ET.Element('mujoco',model=m['robot']);ET.SubElement(r,'compiler',angle='radian',meshdir='meshes',autolimits='true',inertiafromgeom='false',fusestatic='false')
ET.SubElement(r,'option',timestep='.001',gravity='0 0 -9.81',integrator='implicitfast',iterations='80')
ET.SubElement(r,'size',njmax='3000',nconmax='1000')
vis=ET.SubElement(r,'visual');ET.SubElement(vis,'global',offwidth='1400',offheight='1000');ET.SubElement(vis,'headlight',diffuse='.7 .7 .7',ambient='.3 .3 .3')
default=ET.SubElement(r,'default');ET.SubElement(default,'joint',damping='0',armature='0');ET.SubElement(default,'geom',friction='.6 .005 .0001',solref='.005 1',solimp='.95 .99 .001',margin='0')
asset=ET.SubElement(r,'asset');world=ET.SubElement(r,'worldbody')
ET.SubElement(world,'light',pos='1 -1 2',dir='-1 1 -2',directional='true')
ET.SubElement(world,'geom',name='floor',type='plane',size='2 2 .01',rgba='.16 .19 .23 1',contype='1',conaffinity='1')
children={}
for j in m['joints']:children.setdefault(j['parent'],[]).append(j)
bodies={}
def add_link(name,parent,j=None):
 b=ET.SubElement(parent,'body',name=name,pos=f(j['xyz']) if j else '0 0 0');bodies[name]=b
 if j and 'rpy' in j:
  xyzw=Rotation.from_euler('xyz',j['rpy']).as_quat();b.set('quat',f([xyzw[3],*xyzw[:3]]))
 if name in m['links']:
  d=m['links'][name];I=np.array(d['inertia_kg_m2']);ET.SubElement(b,'inertial',mass=str(d['mass_kg']),pos=f(d['com_m']),fullinertia=f([I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]]))
  if j and j['type']!='fixed':ET.SubElement(b,'joint',name=j['name'],type='hinge' if j['type']=='revolute' else 'slide',axis=f(j['axis']),range=f([j['lower'],j['upper']]))
  for v in d['visuals']:
   mn=f"visual_{v['part']:03d}";ET.SubElement(asset,'mesh',name=mn,file=Path(v['mesh']).name)
   ET.SubElement(b,'geom',name=mn,type='mesh',mesh=mn,rgba=f(v['rgba']),contype='0',conaffinity='0',group='2',density='0')
  for c in d['collisions']:
   mn=f"collision_{c['part']:03d}";ET.SubElement(asset,'mesh',name=mn,file=Path(c['mesh']).name)
   ET.SubElement(b,'geom',name=mn,type='mesh',mesh=mn,rgba='1 .4 .05 .2',contype='1',conaffinity='1',group='3',density='0')
 else:ET.SubElement(b,'site',name='tool_center',size='.004',rgba='0.1 0.8 0.8 1')
 for jj in children.get(name,[]):add_link(jj['child'],b,jj)
add_link('base_link',world)
contact=ET.SubElement(r,'contact')
# Adjacent parts contact at bearings/interfaces by construction; suppress parent/child pairs explicitly.
for j in m['joints']:
 if j['child'] in m['links']:ET.SubElement(contact,'exclude',body1=j['parent'],body2=j['child'])
eq=ET.SubElement(r,'equality');ET.SubElement(eq,'joint',name='symmetric_gripper',joint1='gripper_right_joint',joint2='gripper_left_joint',polycoef='0 1 0 0 0',solref='.002 1')
act=ET.SubElement(r,'actuator')
for j in m['joints']:
 if j['type']=='fixed' or 'mimic' in j:continue
 # Direct torque/force, not a fabricated position-servo identification.
 cap=j['effort'];ET.SubElement(act,'motor',name=j['name']+'_drive',joint=j['name'],gear='1',ctrllimited='true',ctrlrange=f([-cap,cap]))
key=ET.SubElement(r,'keyframe');ET.SubElement(key,'key',name='cad_zero',qpos='0 0 0 0 0 0 0 0')
ET.SubElement(key,'key',name='inspection_pose',qpos='0 -.9 1.3 -.5 .1 0 .018 .018')
ET.indent(r);ET.ElementTree(r).write(OUT/'viscous_arm.xml',encoding='utf-8',xml_declaration=True)
print(OUT/'viscous_arm.xml')
