"""Build a reproducible CAD-zero Viscous model. Does not access robot hardware."""
from pathlib import Path
from importlib.metadata import version
import json,copy,hashlib,xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
from collision_geometry import collision_hulls, SETTINGS as COLLISION_SETTINGS
from outline_collision import outline_hulls
ROOT=Path(__file__).resolve().parents[1]; OUT=ROOT/'model'; OUT.mkdir(exist_ok=True)
(OUT/'meshes').mkdir(exist_ok=True)
parts=json.loads((ROOT/'inspection/parts.json').read_text())
R=np.array([[0.,-1.,0.],[0.,0.,-1.],[1.,0.,0.]]) # CAD X-up mm -> robot Z-up m
MOTORS=[0,18,19,20,49,53,58]
STEEL=[1,7,12,26,27,29,30,33,34,36,37,25,31,38,40,41]
INTERNAL=[21,22,23,26,27,28,33,34,35,40]
# Bearing races are lumped onto one link; motor solid is housing + equivalent rotor.
GROUPS={
 'base_link':[0,1,2,14,15,16],
 'yaw_link':[3,4,5,6,7,19],
 'upper_arm_link':[8,9,10,11,12,18],
 'forearm_link':[13,17,58]+list(range(60,70)),
 'wrist_pitch_link':[53,54,55,56,57],
 'wrist_yaw_link':[49,50,51,52,59],
 'wrist_roll_link':[20,21,22,23,24,25,26,27,28,33,34,35,40,46,47,48,70,71],
 'gripper_left':[29,30,31,32,41,43],
 'gripper_right':[36,37,38,39,42],
}
assert sorted(sum(GROUPS.values(),[]))==[i for i in range(len(parts)) if i not in [44,45]], 'Every CAD solid must be assigned exactly once'
def cyl(i):return max(parts[i]['cylinders'],key=lambda x:x['area'])
def plane_center(i):
 c=cyl(i);p=np.array(c['p']);a=np.array(c['axis']);return p-a*(p[2]/a[2])
j1=np.array([57.4,0,0]);j2=plane_center(19);j3=plane_center(18);j4=plane_center(58)
j5=np.array(cyl(53)['p'])-np.array(cyl(53)['axis'])*40.9
j6=np.array(parts[48]['cylinders'][0]['p'])
PIVOTS={'base_link':np.zeros(3),'yaw_link':j1,'upper_arm_link':j2,'forearm_link':j3,'wrist_pitch_link':j4,'wrist_yaw_link':j5,'wrist_roll_link':j6,'gripper_left':j6,'gripper_right':j6}
axes=[[-1,0,0],[0,0,1],[0,0,1],[0,0,1],(-np.array(cyl(53)['axis'])).tolist(),cyl(49)['axis']]
names=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_yaw','wrist_roll']
chain=list(GROUPS)[:7]
# Captured physical ranges from Gavin's Aug28 record, provisionally zero-offset.
# Expand only to include CAD zero; final calibrated mapping requires physical pose readings.
recorded=[[-3.235,2.227],[-3.078,-.042],[.05,4.383],[-2.109,1.606],[-1.556,1.505],[-2.641,2.68]]
limits=[[min(x,0),max(y,0)] for x,y in recorded]
J=[]
for k,name in enumerate(names):
 parent,child=chain[k:k+2]
 J.append(dict(name=name,type='revolute',parent=parent,child=child,xyz=(R@(PIVOTS[child]-PIVOTS[parent])*.001).tolist(),axis=(R@np.array(axes[k])).tolist(),lower=limits[k][0],upper=limits[k][1],effort=5.,velocity=2.,source_motor_id=[0,19,18,58,53,49][k]))
for side,axis in [('left',[0,0,-1]),('right',[0,0,1])]:
 J.append(dict(name=f'gripper_{side}_joint',type='prismatic',parent='wrist_roll_link',child=f'gripper_{side}',xyz=[0,0,0],axis=(R@np.array(axis)).tolist(),lower=0.,upper=.0524125,effort=10.,velocity=.05,**({'mimic':'gripper_left_joint'} if side=='right' else {})))
# CAD fingers nearly touch, remaining gap about 0.25 mm; zero = as-exported, not exact contact.
GRASP_CAD=np.mean([parts[42]['center_mm'],parts[43]['center_mm']],axis=0)
# Locate inner opposing fingertip surfaces geometrically below after meshes load.
color_palette={'base_link':[.20,.24,.28,1],'yaw_link':[.70,.74,.77,1],'upper_arm_link':[.69,.74,.77,1],'forearm_link':[.67,.73,.77,1],'wrist_pitch_link':[.66,.72,.76,1],'wrist_yaw_link':[.65,.70,.74,1],'wrist_roll_link':[.65,.71,.76,1],'gripper_left':[.16,.20,.23,1],'gripper_right':[.16,.20,.23,1]}
# Material overrides are task inputs, never hidden constants.
def default_material(i):
 if i in MOTORS:return {'mass_kg':.310,'basis':'RS00 nominal manufacturer mass; solid-normalized COM/inertia approximation'}
 if i in STEEL:return {'density_kg_m3':7850,'basis':'steel estimate for bearings/rail/blocks/bolts; verify assembled mass'}
 if i in [42,43,44,45]:return {'density_kg_m3':1200,'basis':'provisional polymer effective density; finger/loop material and infill unconfirmed'}
 if i==71:return {'mass_kg':.030,'basis':'camera provisional mass, not measured; override when weighed'}
 if i==70:return {'density_kg_m3':1200,'basis':'camera bracket provisional polymer; material unconfirmed'}
 return {'density_kg_m3':2700,'basis':'CNC aluminium, likely6061 per Gavin; nonstructural metal assignment provisional'}
overridepath=ROOT/'materials.json'
if overridepath.exists():overrides=json.loads(overridepath.read_text())
else:
 overrides={str(i):default_material(i) for i in range(len(parts))};overridepath.write_text(json.dumps(overrides,indent=2))
assemblies=COLLISION_SETTINGS.get('assemblies',{})
assembly_members={}
for name,spec in assemblies.items():
 assert set(spec['parts'])<=set(GROUPS[spec['link']]), 'Collision assemblies cannot cross a moving joint'
 for i in spec['parts']:
  assert i not in assembly_members and i not in INTERNAL and i not in STEEL
  assembly_members[i]=name
def export_hulls(hulls,stem,metadata):
 result=[]
 for piece,hull in enumerate(hulls):
  # Triangulate on the float32 STL grid after 1 micrometre vertex cleanup.
  hull=trimesh.convex.convex_hull(hull.vertices.round(6).astype(np.float32).astype(float))
  suffix=f'_{piece:02d}' if len(hulls)>1 else ''
  cf=f'meshes/{stem}{suffix}.stl';hull.export(OUT/cf)
  result.append({**metadata,'piece':piece,'mesh':cf})
 return result
linkdata={};audit=[];allmeshes={}
for link,ids in GROUPS.items():
 mass=0;first=np.zeros(3);Iorigin=np.zeros((3,3));visual=[];collisions=[]
 for i in ids:
  p=parts[i];spec=overrides.get(str(i),default_material(i));vol=p['volume_mm3'];m=spec.get('mass_kg',spec.get('density_kg_m3',2700)*vol*1e-9)
  com=R@(np.array(p['center_mm'])-PIVOTS[link])*.001
  inertia=R@np.array(p['inertia_unit_density_mm5'])@R.T*(m/vol)*1e-6
  mass+=m;first+=m*com;Iorigin+=inertia+m*((com@com)*np.eye(3)-np.outer(com,com))
  mesh=trimesh.load_mesh(ROOT/'inspection'/f'part_{i:03d}.stl');v=(R@(mesh.vertices-PIVOTS[link]).T).T*.001;mesh.vertices=v
  allmeshes[i]=mesh
  file=f'meshes/part_{i:03d}.stl';mesh.export(OUT/file)
  col=[.12,.15,.18,1] if i in MOTORS else color_palette[link]
  if i in STEEL:col=[.38,.42,.45,1]
  if i==71:col=[.10,.13,.16,1]
  if i not in INTERNAL:visual.append({'part':i,'mesh':file,'rgba':col})
  # Separate convex pieces preserve large openings in selected concave solids.
  # Tiny fasteners/bearings are visually represented but excluded from contact geometry.
  if i not in INTERNAL and i not in STEEL and i not in assembly_members:
   # Outline extraction and the comparison image use exactly the exported CAD
   # vertices, including STL processing, to share the same projection frame.
   collision_source=trimesh.load_mesh(OUT/file) if str(i) in COLLISION_SETTINGS.get('outline_parts',{}) else mesh
   hulls=collision_hulls(collision_source,i)
   collisions.extend(export_hulls(hulls,f'collision_{i:03d}',{'part':i}))
  audit.append({'part':i,'name':p['name'],'link':link,'mass_kg':m,'volume_mm3':vol,'material':spec,'visual':i not in INTERNAL,'collision':i not in INTERNAL and i not in STEEL})
  if i in assembly_members:audit[-1]['collision_assembly']=assembly_members[i]
 for name,spec in assemblies.items():
  if spec['link']!=link:continue
  source=trimesh.util.concatenate([trimesh.load_mesh(OUT/f'meshes/part_{i:03d}.stl') for i in spec['parts']])
  hulls=outline_hulls(source,boundary_cleanup_m=spec['boundary_cleanup_m'],frame=(source.bounds.mean(axis=0),np.asarray(spec['profile_basis'])),outward_margin_m=spec['outward_margin_m'])
  assert len(hulls)<=spec['max_pieces']
  collisions.extend(export_hulls(hulls,f'collision_{name}',{'part':spec['parts'][0],'parts':spec['parts'],'assembly':name}))
  print(f"Collision assembly {name}: {len(spec['parts'])} CAD solids -> {len(hulls)} convex pieces",flush=True)
 com=first/mass;I=Iorigin-mass*((com@com)*np.eye(3)-np.outer(com,com))
 assert np.linalg.eigvalsh(I).min()>0
 linkdata[link]={'mass_kg':mass,'com_m':com.tolist(),'inertia_kg_m2':I.tolist(),'visuals':visual,'collisions':collisions,'cad_pivot_mm':PIVOTS[link].tolist()}
# TCP at area-weighted centre of opposing inner finger pad faces; midpoint between jaws.
tool_axis=R@(-np.array(cyl(20)['axis']));tool_axis/=np.linalg.norm(tool_axis)
padcenters=[]
for i in [42,43]:
 mesh=allmeshes[i];centers=mesh.triangles_center+(R@j6*.001)
 # Inner faces run along the jaw-opening Y direction and lie closest to the mid-plane.
 normals=mesh.face_normals;areas=mesh.area_faces
 mask=(np.abs(normals[:,1])>.995)&(np.abs(centers[:,1])<.001)
 if not np.any(mask): raise RuntimeError('Unable to identify jaw inner pad')
 padcenters.append(np.average(centers[mask],axis=0,weights=areas[mask]))
toolpoint=np.mean(padcenters,axis=0)
# +X opens left jaw, +Z points toward fingertips, +Y completes right-handed frame.
tool_x=np.array([0.,1.,0.]);tool_y=np.cross(tool_axis,tool_x);tool_y/=np.linalg.norm(tool_y)
tool_x=np.cross(tool_y,tool_axis);tool_R=np.column_stack([tool_x,tool_y,tool_axis])
tool_rpy=Rotation.from_matrix(tool_R).as_euler('xyz').tolist()
J.append(dict(name='tool_fixed',type='fixed',parent='wrist_roll_link',child='tool0',xyz=(toolpoint-R@j6*.001).tolist(),rpy=tool_rpy,axis=[0,0,1]))
robot=ET.Element('robot',name='viscous_arm_dry_cad_v1')
robot.append(ET.Comment('CAD-zero model. Encoder calibration, gripper mapping and estimated physical properties require validation. No hardware control configuration.'))
fmt=lambda v:' '.join(f'{float(x):.10g}' for x in v)
for link,data in linkdata.items():
 l=ET.SubElement(robot,'link',name=link);inert=ET.SubElement(l,'inertial');ET.SubElement(inert,'origin',xyz=fmt(data['com_m']),rpy='0 0 0');ET.SubElement(inert,'mass',value=f"{data['mass_kg']:.10g}")
 I=np.array(data['inertia_kg_m2']);ET.SubElement(inert,'inertia',**{k:f'{I[a,b]:.12g}' for k,a,b in [('ixx',0,0),('ixy',0,1),('ixz',0,2),('iyy',1,1),('iyz',1,2),('izz',2,2)]})
 for typ,key in [('visual','visuals'),('collision','collisions')]:
  for v in data[key]:
   shape_name=Path(v['mesh']).stem if typ=='collision' else f"part_{v['part']:03d}"
   el=ET.SubElement(l,typ,name=shape_name);g=ET.SubElement(el,'geometry');ET.SubElement(g,'mesh',filename=v['mesh'])
   if typ=='visual':mat=ET.SubElement(el,'material',name=f"material_{v['part']}");ET.SubElement(mat,'color',rgba=fmt(v['rgba']))
ET.SubElement(robot,'link',name='tool0')
for j in J:
 el=ET.SubElement(robot,'joint',name=j['name'],type=j['type']);ET.SubElement(el,'parent',link=j['parent']);ET.SubElement(el,'child',link=j['child']);ET.SubElement(el,'origin',xyz=fmt(j['xyz']),rpy=fmt(j.get('rpy',[0,0,0])))
 if j['type']!='fixed':
  ET.SubElement(el,'axis',xyz=fmt(j['axis']));ET.SubElement(el,'limit',**{k:str(j[k]) for k in ['lower','upper','effort','velocity']})
  # Zero damping/friction = unidentified ideal joint, not an invented measured parameter.
  ET.SubElement(el,'dynamics',damping='0',friction='0')
  if 'mimic' in j:ET.SubElement(el,'mimic',joint=j['mimic'],multiplier='1',offset='0')
ET.indent(robot);ET.ElementTree(robot).write(OUT/'viscous_arm.urdf',encoding='utf-8',xml_declaration=True)
meta={'robot':'viscous_arm_dry_cad_v1','status':'geometry-derived model; physical calibration pending','units':'metres, kilograms, radians','coordinate_mapping':'robot XYZ=(-CAD Y,-CAD Z,CAD X); CAD mm -> m; base underside Z=0','mesh_tessellation':{'linear_deflection_mm':.08,'angular_deflection_rad':.25,'relative':False},'source_sha256':hashlib.sha256((ROOT/'source/2026-09-18/maker arm cnc装配.step').read_bytes()).hexdigest(),'excluded_cad_parts':[44,45],'excluded_reason':'Gavin confirmed real arm has no finger loops, 2026-09-18','tool_frame':{'origin':'area-weighted midpoint of opposing inner finger pad faces','axes':'+X jaw opening; +Z approach toward fingertips; +Y right-handed'},'cad_zero_is_encoder_zero':False,'encoder_offset_status':'unmeasured; never command hardware with this model yet','joint_motor_mapping':dict(zip(names,range(1,7))),'recorded_motor_ranges_rad':dict(zip(names,recorded)),'joint_ranges_status':'provisionally assumes zero offset; widened to include CAD zero. Not calibrated limits.','link_spacings_mm':{'J2_J3':float(np.linalg.norm(j3-j2)),'J3_J4':float(np.linalg.norm(j4-j3))},'total_mass_kg':sum(x['mass_kg'] for x in linkdata.values()),'links':linkdata,'joints':J,'parts':audit,'simulator':{'engine':'MuJoCo','model':'viscous_arm.xml','gravity_m_s2':[0,0,-9.81],'velocity_caps_enforced':False,'joint_friction_damping_armature':'zero/unidentified','contact_friction':[.6,.005,.0001],'contact_friction_status':'generic numerical default, unmeasured'},'gripper':{'model':'symmetric sliders; internal crank/rod geometry omitted, its mass/inertia retained in wrist at CAD pose','per_jaw_max_travel_m':.0524125,'range_basis':'Maker reference only; physical Viscous gap not remeasured','actuator_generalized_force_N':10,'symmetric_contact_force_per_jaw_N_approx':5,'force_status':'simulation placeholder, not motor specification','motor_to_gap':'unidentified nonlinear transmission; no encoder conversion provided'},'assumptions':['Seven nominal 310g RS00 solids with uniform effective-density inertias; internal rotor/transmission inertia unidentified','CNC structural aluminium at2700kg/m3; 6061 alloy provisional','Bearing/rail steel7850kg/m3 provisional','PLA+ fingers42/43 and camera bracket70: nominal solid density1240kg/m3 times user-assumed30%effective solid fraction=372kg/m3; uniform effective-density inertia approximation. Finger loops44/45 physically absent and excluded','Camera mass30g placeholder','All model arm effort limits5Nm nominal datasheet rated, not thermal guarantee; simulation velocity2rad/s is a conservative chosen cap','Cables, omitted fasteners, waterproofing additions and payload not estimated without evidence','Per-solid convex collision approximation; holes/concavities can be conservative','Motor axis positive signs inherited from reference convention, physical signs not independently checked','Reference pose is source CAD assembly, not measured encoder zero']}
meta['collision_geometry']={**COLLISION_SETTINGS,'coacd_version':version('coacd'),'shapely_version':version('shapely'),'export_grid_m':1e-6,'surface_coverage':'Outline parts preserve the filled broad-view CAD projection inside the full 3D convex envelope. Other compounds restore sampled source surface coverage after simplification.'}
meta['assumptions']=[a.replace('Per-solid convex collision approximation; holes/concavities can be conservative','J2-J3 and J3-J4 use solid assembly collision envelopes with plate-pair cavities filled; rail base and fingers retain filled-outline envelopes. CAD visuals and measured/estimated mass properties remain separate from collision filling.') for a in meta['assumptions']]
(OUT/'model_manifest.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2));print(json.dumps({'mass_kg':meta['total_mass_kg'],'spacing_mm':meta['link_spacings_mm'],'links':{k:round(v['mass_kg'],4) for k,v in linkdata.items()}},indent=2))
# Remove only obsolete generated collision assets after the model is written.
used={c['mesh'] for d in linkdata.values() for c in d['collisions']}
for path in (OUT/'meshes').glob('collision_*.stl'):
 if str(path.relative_to(OUT)) not in used:path.unlink()
