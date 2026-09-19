'''Contact-only dry-air pick/place demonstration. No hardware access.'''
from pathlib import Path
import os,sys,json,math,xml.etree.ElementTree as ET
os.environ.setdefault('MUJOCO_GL','egl')
import numpy as np,mujoco
from PIL import Image,ImageDraw,ImageFont
from kinematics import ROOT,inverse
OUT=ROOT/'demo';OUT.mkdir(exist_ok=True)
RENDER='--render' in sys.argv
f=lambda a:' '.join(str(float(x)) for x in a)
pick=np.array([.17,.04]);drop=np.array([.12,-.12]);platform=.025
rot=np.array([[0.,1,0],[1,0,0],[0,0,-1]])
r=ET.parse(ROOT/'model/viscous_arm.xml').getroot()
r.remove(r.find('keyframe'))
r.find('compiler').set('meshdir','../model/meshes')
r.find('option').set('cone','elliptic')
r.find('option').set('iterations','100')
r.find('visual/global').set('offwidth','1280');r.find('visual/global').set('offheight','960')
w=r.find('worldbody')
w.find("geom[@name='floor']").set('rgba','.08 .11 .15 1')
ET.SubElement(w,'light',name='fill',pos='-.5 -1 1',dir='.5 1 -1',diffuse='.55 .6 .7')
ET.SubElement(w,'geom',name='pickup_platform',type='box',pos=f([*pick,platform/2]),size=f([.045,.038,platform/2]),rgba='.22 .29 .36 1',friction='.8 .005 .0001')
b=ET.SubElement(w,'body',name='cube',pos=f([*pick,platform+.0101]))
ET.SubElement(b,'freejoint',name='cube_free')
ET.SubElement(b,'inertial',pos='0 0 0',mass='.01',diaginertia='6.666666667e-7 6.666666667e-7 6.666666667e-7')
ET.SubElement(b,'geom',name='cube_geom',type='box',size='.01 .01 .01',mass='.01',rgba='1 .32 .075 1',friction='.8 .01 .001',condim='4',solref='.004 1',solimp='.95 .99 .001')
c=ET.SubElement(w,'body',name='cup',pos=f([*drop,0]))
ET.SubElement(c,'geom',name='cup_bottom',type='cylinder',pos='0 0 .003',size='.041 .003',rgba='.14 .58 .63 1')
for i in range(32):
 a=2*np.pi*i/32;rr=.038
 ET.SubElement(c,'geom',name='cup_wall_'+str(i),type='box',pos=f([rr*np.cos(a),rr*np.sin(a),.025]),size=f([.0035,.0041,.025]),euler=f([0,0,a]),rgba='.18 .66 .70 .85',friction='.6 .005 .0001')
# Implicitly integrated position servos avoid unstable explicit velocity feedback.
KP=np.array([90.,90.,70.,35.,20.,15.,3000.])
KV=np.array([5.,5.,3.,1.5,.7,.5,12.])
for i,act in enumerate(r.find('actuator')):
 act.tag='position'
 for key in ['gear','ctrllimited','ctrlrange']:act.attrib.pop(key,None)
 act.set('kp',str(KP[i]));act.set('kv',str(KV[i]))
 act.set('forcelimited','true');act.set('forcerange','-5 5' if i<6 else '-10 10')
scene=OUT/'pick_place_scene.xml';ET.indent(r);ET.ElementTree(r).write(scene,encoding='utf-8',xml_declaration=True)
m=mujoco.MjModel.from_xml_path(str(scene));d=mujoco.MjData(m)
waypoints=[
 ('Ready',1.0,[*pick,.100],.020),
 ('Approach cube',3.0,[*pick,.062],.020),
 ('Close fingers',1.6,[*pick,.062],.0085),
 ('Lift cube',3.0,[*pick,.100],.0085),
 ('Transfer to cup',4.0,[*drop,.100],.0085),
 ('Lower above cup',2.2,[*drop,.090],.0085),
 ('Release cube',1.8,[*drop,.090],.024),
 ('Retract',2.8,[*drop,.110],.024),
 ('Cube in cup',2.0,[.11,-.075,.110],.024)]
T=np.eye(4);T[:3,:3]=rot;T[:3,3]=waypoints[0][2]
q,info=inverse(T,[0,-1.9,2,-1.5,0,0]);assert info['residual_norm']<1e-5,info
initial=q.copy();samples=[];t=0.;prev=np.array(waypoints[0][2]);prevgrip=waypoints[0][3]
for label,duration,pos,grip in waypoints:
 pos=np.array(pos)
 for s in np.linspace(0,1,round(duration*50)+1):
  a=s*s*s*(10+s*(-15+6*s));T[:3,3]=prev+(pos-prev)*a
  q,info=inverse(T,q)
  if info['residual_norm']>1e-5:raise RuntimeError((label,s,T[:3,3],info))
  samples.append((t+s*duration,np.r_[q,prevgrip+(grip-prevgrip)*a],label))
 t+=duration;prev=pos;prevgrip=grip
traj={round(x[0],8):(x[1],x[2]) for x in samples};times=np.array(sorted(traj));qs=np.array([traj[x][0] for x in times]);labels=[traj[x][1] for x in times]
d.qpos[:6]=initial;d.qpos[6:8]=.020;mujoco.mj_forward(m,d)
cube=m.body('cube').id;tool=m.body('tool0').id
renderer=None;writer=None
if RENDER:
 import imageio_ffmpeg
 renderer=mujoco.Renderer(m,height=960,width=1280)
 camera=mujoco.MjvCamera();camera.lookat[:]=[.11,-.015,.145];camera.distance=.77;camera.azimuth=132;camera.elevation=-24
 options=mujoco.MjvOption();options.geomgroup[3]=0;options.sitegroup[:]=0
 writer=imageio_ffmpeg.write_frames(str(OUT/'viscous_pick_place_20mm.mp4'),(1280,960),fps=30,codec='libx264',quality=8,pix_fmt_in='rgb24',pix_fmt_out='yuv420p',output_params=['-movflags','+faststart'])
 writer.send(None)
 font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',20)
 small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',15)
logs=[];stage_records={};max_ctrl=0.;max_cubez=0.;maxerr=0.;nextframe=0.;badcontacts={};bilateral=0;carried_bilateral=0;maxspeed=0.;maxlimit=0.;states=[]
for step in range(round(t/m.opt.timestep)):
 now=d.time;idx=min(max(np.searchsorted(times,now,side='right')-1,0),len(times)-2)
 a=np.clip((now-times[idx])/(times[idx+1]-times[idx]),0,1)
 target=qs[idx]*(1-a)+qs[idx+1]*a
 velocity=(qs[idx+1]-qs[idx])/(times[idx+1]-times[idx]);label=labels[idx]
 d.ctrl[:6]=target[:6]+(d.qfrc_bias[:6]+KV[:6]*velocity[:6])/KP[:6]
 d.ctrl[6]=target[6]+(d.qfrc_bias[6]+d.qfrc_bias[7]+KV[6]*velocity[6])/KP[6]
 mujoco.mj_step(m,d)
 max_ctrl=max(max_ctrl,float(np.max(np.abs(d.actuator_force[:6]))));max_cubez=max(max_cubez,float(d.xpos[cube,2]));maxerr=max(maxerr,float(np.max(np.abs(target[:6]-d.qpos[:6]))))
 maxspeed=max(maxspeed,float(np.max(np.abs(d.qvel[:6]))))
 maxlimit=max(maxlimit,float(np.max(np.maximum(m.jnt_range[:8,0]-d.qpos[:8],d.qpos[:8]-m.jnt_range[:8,1]))))
 touch=set()
 for contact in d.contact:
  n1=m.geom(contact.geom1).name;n2=m.geom(contact.geom2).name
  if 'cube_geom' in (n1,n2):
   other=n2 if n1=='cube_geom' else n1;touch.add(other)
   if other.startswith('collision_') and other not in ['collision_042','collision_043']:badcontacts['cube / '+other]=max(badcontacts.get('cube / '+other,0),float(-contact.dist))
  elif contact.dist<-.0005 and ('collision_' in n1 or 'collision_' in n2):
   pair=' / '.join(sorted([n1,n2]));badcontacts[pair]=max(badcontacts.get(pair,0),float(-contact.dist))
 bilateral+=int('collision_042' in touch and 'collision_043' in touch)
 carried_bilateral+=int('collision_042' in touch and 'collision_043' in touch and d.xpos[cube,2]>.05)
 if step%100==0:
  rec={'time':round(now,3),'stage':label,'cube_xyz':d.xpos[cube].tolist(),'tool_xyz':d.xpos[tool].tolist(),'jaw_q':d.qpos[6:8].tolist(),'cube_contacts':sorted(touch),'joint_error':float(np.max(np.abs(target[:6]-d.qpos[:6])))}
  logs.append(rec);stage_records[label]=rec
  states.append(np.r_[now,d.qpos.copy(),d.qvel.copy()])
 if RENDER and now+1e-8>=nextframe:
  if now>17.0:
   blend=min((now-17)/3,1);blend=blend*blend*(3-2*blend)
   camera.elevation=-24-24*blend
  renderer.update_scene(d,camera,scene_option=options);im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im)
  draw.rectangle([0,0,1280,84],fill=(15,23,32));draw.text((28,16),'VISCOUS ARM  /  MuJoCo pick & place',font=font,fill=(225,240,241));draw.text((28,49),'20 mm cube · 10 g demo mass · dry air · contact-driven grasp',font=small,fill=(151,185,193))
  draw.rectangle([0,910,1280,960],fill=(15,23,32));draw.text((28,923),label,font=font,fill=(126,224,208));draw.text((755,927),'Simulation · provisional masses / friction / calibration',font=small,fill=(178,191,202))
  if any(abs(now-t)<.0005 for t in [0,5.6,9.,14.8,17.2,21.3]):im.save(OUT/('frame_%05.1f.png'%now))
  writer.send(np.asarray(im));nextframe+=1/30
 if not np.isfinite(d.qpos).all():raise RuntimeError('Nonfinite simulation')
if writer:writer.close();renderer.close()
final=d.xpos[cube].copy();inside=bool(np.linalg.norm(final[:2]-drop)<.020 and .012<final[2]<.025)
cube_speed=float(np.linalg.norm(d.qvel[8:11]));cube_spin=float(np.linalg.norm(d.qvel[11:14]))
valid=bool(inside and max_cubez>.065 and carried_bilateral>1000 and not badcontacts and not np.any(d.warning.number) and cube_speed<.001 and cube_spin<.01 and maxspeed<2 and maxlimit<.001)
np.savez_compressed(OUT/'simulation_states.npz',states=np.array(states),nq=m.nq,nv=m.nv)
report={'success':valid,'final_cube_speed_m_s':cube_speed,'final_cube_angular_speed_rad_s':cube_spin,'max_arm_speed_rad_s':maxspeed,'max_joint_limit_excursion':maxlimit,'carried_bilateral_contact_steps':carried_bilateral,'cube_side_m':.020,'cube_mass_kg':.010,'cube_final_xyz':final.tolist(),'inside_cup':inside,'max_cube_height_m':max_cubez,'bilateral_contact_steps':bilateral,'max_arm_command_Nm':max_ctrl,'max_arm_tracking_error_rad':maxerr,'unexpected_contacts':badcontacts,'warning_counts':d.warning.number.tolist(),'duration_seconds':t,'rendered':RENDER,'stage_end_states':stage_records,'assumptions':['Bounded position servos with model gravity feedforward for this demo.','Friction coefficient .8 for demo cube; contact forces resolve physically.','No weld, mocap parenting, cube position resets, or object teleporting.','Original arm torque limits +/-5 Nm, gripper generalized force +/-10 N.','Robot masses and calibration remain provisional.']}
(OUT/'pick_place_report.json').write_text(json.dumps(report,indent=2));(OUT/'trajectory_log.json').write_text(json.dumps(logs,indent=2));print(json.dumps(report,indent=2),flush=True)

if not valid:raise RuntimeError('Pick/place validation failed; see report')
