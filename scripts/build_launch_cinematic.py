"""Single CAD arm launch film. Run using .blender-venv/bin/python.
Shared staging/edit/rig; --finish changes only structural metal and light power.
"""
from pathlib import Path
import argparse, json, math, struct, subprocess, xml.etree.ElementTree as ET
import bpy
import faulthandler
faulthandler.enable()
import numpy as np
from mathutils import Matrix, Vector, Euler
from kinematics import fk, NAMES
ROOT=Path(__file__).resolve().parents[1]
FPS=24
SHOTS=[('01_surface','Form, before function.',1,96),('02_grip','Precision in motion.',97,192),('03_base','A grounded beginning.',193,264),('04_upper','J2 — J3',265,336),('05_forearm','J3 — J4',337,408),('06_motor','RS00',409,480),('07_wrist','Every degree considered.',481,552),('08_fingers','A considered touch.',553,624),('09_hero','Viscous',625,768)]
POSE=np.array([-.25,-1.15,1.65,-.45,.18,0,.012])

def gpu(scene):
    prefs=bpy.context.preferences.addons['cycles'].preferences
    prefs.compute_device_type='OPTIX'; prefs.get_devices()
    for d in prefs.devices: d.use=d.type=='OPTIX'
    scene.cycles.device='GPU' if any(d.use for d in prefs.devices) else 'CPU'
    return [d.name for d in prefs.devices if d.use]

def material(name,color,metal,rough,texture=False):
    m=bpy.data.materials.new(name);m.use_nodes=True;m.diffuse_color=(*color,1)
    n=m.node_tree.nodes;l=m.node_tree.links;b=n.get('Principled BSDF')
    b.inputs['Base Color'].default_value=(*color,1);b.inputs['Metallic'].default_value=metal;b.inputs['Roughness'].default_value=rough
    if texture:
        t=n.new('ShaderNodeTexNoise');t.inputs['Scale'].default_value=1600;t.inputs['Detail'].default_value=2
        bump=n.new('ShaderNodeBump');bump.inputs['Strength'].default_value=.1;bump.inputs['Distance'].default_value=.000012
        l.new(t.outputs['Fac'],bump.inputs['Height']);l.new(bump.outputs[0],b.inputs['Normal'])
        b.inputs['Anisotropic'].default_value=.22
    return m

def empty(name,collection):
    o=bpy.data.objects.new(name,None);collection.objects.link(o);o.empty_display_size=.018;return o

def look(o,point,up='Y'):
    o.rotation_euler=(Vector(point)-o.location).to_track_quat('-Z',up).to_euler()

def main():
    p=argparse.ArgumentParser();p.add_argument('--finish',choices=['silver','black'],default='silver');args=p.parse_args()
    out=ROOT/'cinematic/launch'/args.finish;out.mkdir(parents=True,exist_ok=True)
    silver=ROOT/'cinematic/launch/silver'
    if args.finish=='black' and (silver/'viscous_launch.blend').exists():
        bpy.ops.wm.open_mainfile(filepath=str(silver/'viscous_launch.blend'))
        m=bpy.data.materials['FINISH | silver'];m.name='FINISH | black';m.diffuse_color=(.025,.028,.032,1)
        bs=m.node_tree.nodes.get('Principled BSDF');bs.inputs['Base Color'].default_value=(.025,.028,.032,1);bs.inputs['Roughness'].default_value=.33
        for light in bpy.data.lights:light.energy*=3.0 if light.name.startswith('Key') else (2.5 if light.name.startswith('Front') else 1.7)
        bpy.data.objects['POSE | animate these URDF joint values']['finish']='black'
        bpy.ops.wm.save_as_mainfile(filepath=str(out/'viscous_launch.blend'))
        motion=json.loads((silver/'motion.json').read_text());motion['finish']='black'
        for part in motion['materials']:
            if part['material']=='FINISH | silver':part['material']='FINISH | black'
        (out/'motion.json').write_text(json.dumps(motion,indent=2)+'\n')
        print('BUILT black from shared silver scene',flush=True)
        return
    bpy.ops.wm.read_factory_settings(use_empty=True);s=bpy.context.scene
    s.render.engine='CYCLES';s.cycles.samples=48;s.cycles.use_denoising=True;s.cycles.max_bounces=6;s.cycles.use_adaptive_sampling=True;s.cycles.adaptive_threshold=.025
    s.render.resolution_x=1920;s.render.resolution_y=1080;s.render.resolution_percentage=100;s.render.fps=FPS;s.frame_start=1;s.frame_end=768
    s.render.image_settings.file_format='PNG';s.render.image_settings.color_mode='RGB';s.render.image_settings.compression=15
    s.render.use_persistent_data=True;s.render.film_transparent=False;s.render.use_motion_blur=True;s.render.motion_blur_shutter=.25
    s.view_settings.view_transform='AgX';s.view_settings.look='AgX - Medium High Contrast';s.view_settings.exposure=0
    s.world=bpy.data.worlds.new('Near-black seamless studio');s.world.use_nodes=True
    s.world.node_tree.nodes['Background'].inputs[0].default_value=(.025,.028,.032,1);s.world.node_tree.nodes['Background'].inputs[1].default_value=.16
    device=gpu(s)
    metal=material('FINISH | '+args.finish,(.57,.59,.61) if args.finish=='silver' else (.025,.028,.032),.92,.29 if args.finish=='silver' else .33,True)
    motor=material('RS00 | graphite satin',(.052,.058,.065),.72,.3,True)
    pla=material('PLA+ | soft charcoal',(.008,.010,.012),0,.55,True)
    steel=material('Steel | guides and bearings',(.32,.35,.38),.92,.23)
    lens=material('Camera | optical glass',(.007,.012,.018),.35,.12)
    manifest=json.loads((ROOT/'model/model_manifest.json').read_text());parts={x['part']:x for x in manifest['parts']}
    urdf=ET.parse(ROOT/'model/viscous_arm.urdf').getroot()
    col=bpy.data.collections.new('01 | Viscous — original CAD');s.collection.children.link(col)
    rigcol=bpy.data.collections.new('02 | URDF articulation');s.collection.children.link(rigcol)
    control=empty('POSE | animate these URDF joint values',rigcol);control['finish']=args.finish
    roots={l.get('name'):empty(l.get('name'),rigcol) for l in urdf.findall('link')}
    bindings=[]
    for j in urdf.findall('joint'):
        name=j.get('name');kind=j.get('type');o=empty('ORIGIN | '+name,rigcol);o.parent=roots[j.find('parent').get('link')]
        origin=j.find('origin');o.location=[float(x) for x in origin.get('xyz','0 0 0').split()];o.rotation_euler=[float(x) for x in origin.get('rpy','0 0 0').split()]
        child=roots[j.find('child').get('link')];child.parent=o
        if kind=='fixed':continue
        limit=j.find('limit');lo=float(limit.get('lower'));hi=float(limit.get('upper'));axis=[float(x) for x in j.find('axis').get('xyz').split()]
        mimic=j.find('mimic');master=mimic.get('joint') if mimic is not None else name
        if mimic is None:
            control[name]=0.;control.id_properties_ui(name).update(min=lo,max=hi,soft_min=lo,soft_max=hi,description='URDF '+kind+'; radians or metres')
        path='rotation_axis_angle' if kind=='revolute' else 'location'
        indices=[0] if kind=='revolute' else range(3)
        if kind=='revolute':child.rotation_mode='AXIS_ANGLE';child.rotation_axis_angle=(0,*axis)
        for idx in indices:
            drv=child.driver_add(path,idx).driver;v=drv.variables.new();v.name='q';v.targets[0].id=control;v.targets[0].data_path='["'+master+'"]'
            mult=float(mimic.get('multiplier','1')) if mimic is not None else 1.;offset=float(mimic.get('offset','0')) if mimic is not None else 0.
            drv.expression=f'(min({hi},max({lo},q*{mult}+{offset})))'+(f'*{axis[idx]}' if kind=='prismatic' else '')
        bindings.append({'name':name,'master':master,'axis':axis,'lower':lo,'upper':hi})
    assignments=[]
    for link in urdf.findall('link'):
        print('IMPORT',link.get('name'),flush=True)
        for v in link.findall('visual'):
            filename=v.find('geometry/mesh').get('filename');raw=(ROOT/'model'/filename).read_bytes();count=struct.unpack_from('<I',raw,80)[0]
            arr=np.frombuffer(raw,dtype=np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')]),count=count,offset=84)
            verts,inv=np.unique(arr['v'].reshape(-1,3),axis=0,return_inverse=True);mesh=bpy.data.meshes.new(v.get('name'));mesh.from_pydata(verts.tolist(),[],inv.reshape(-1,3).tolist());mesh.update()
            o=bpy.data.objects.new(v.get('name'),mesh);col.objects.link(o);o.parent=roots[link.get('name')]
            origin=v.find('origin')
            if origin is not None:o.location=[float(x) for x in origin.get('xyz','0 0 0').split()];o.rotation_euler=[float(x) for x in origin.get('rpy','0 0 0').split()]
            part=parts[int(v.get('name').split('_')[-1])];density=part['material'].get('density_kg_m3',0)
            mat=motor if 'RS00' in part['material'].get('basis','') else (metal if density==2700 else (steel if density==7850 else (lens if 'camera' in part['name'].lower() and density==0 else pla)))
            mesh.materials.append(mat)
            # Smooth only tessellated curved surfaces, retaining sharp CAD edges.
            for poly in mesh.polygons:poly.use_smooth=True
            edgefaces={}
            for poly in mesh.polygons:
                for e in poly.edge_keys:edgefaces.setdefault(tuple(sorted(e)),[]).append(poly)
            for e in mesh.edges:
                faces=edgefaces.get(tuple(sorted(e.vertices)),[])
                e.use_edge_sharp=len(faces)!=2 or faces[0].normal.dot(faces[-1].normal)<.85
            assignments.append({'part':part['part'],'material':mat.name,'link':link.get('name')})
    # Kinematic keys: fixed beauty pose for component studies, articulated hero finale.
    keys=[(1,POSE),(96,POSE),(97,POSE.copy()),(116,POSE.copy()),(144,POSE.copy()),(164,POSE.copy()),(192,POSE.copy()),(193,POSE),(624,POSE),(625,POSE),(696,np.array([.04,-1.28,1.86,-.6,-.10,.22,.02])),(744,np.array([.15,-1.24,1.8,-.52,-.12,.32,.012])),(768,np.array([.15,-1.24,1.8,-.52,-.12,.32,.012]))]
    for f,q in keys:
        q=q.copy()
        if f in (1,96,97,116,192,193):q[6]=.001
        if f in (144,164):q[6]=.038
        for name,value in zip(NAMES,q):control[name]=float(value);control.keyframe_insert(data_path='["'+name+'"]',frame=f)
    for fc in control.animation_data.action.fcurves:
        for k in fc.keyframe_points:k.handle_left_type=k.handle_right_type='AUTO_CLAMPED'
    T=fk(POSE);P={k:Vector(t[:3,3]) for k,t in T.items()}
    cams=bpy.data.collections.new('03 | Nine-shot edit');s.collection.children.link(cams)
    lights=bpy.data.collections.new('04 | Softbox studio');s.collection.children.link(lights)
    def area(name,loc,target,power,size,size_y,color):
        d=bpy.data.lights.new(name,'AREA');o=bpy.data.objects.new(name,d);lights.objects.link(o);o.location=loc;look(o,target);d.energy=power*((3.0 if name.startswith('Key') else (2.5 if name.startswith('Front') else 1.7)) if args.finish=='black' else 1);d.shape='RECTANGLE';d.size=size;d.size_y=size_y;d.color=color
    area('Key | tall silk',(.15,-.48,.6),(.05,0,.22),6,.32,.65,(.94,.97,1))
    area('Rim | left blade',(-.34,.13,.44),(.04,0,.25),22,.075,.62,(.83,.90,1))
    area('Rim | right blade',(.48,.22,.48),(.08,0,.25),26,.09,.52,(1,.94,.88))
    area('Top | broad feather',(.03,.05,.85),(.05,0,.2),4,.55,.3,(1,1,1))
    area('Front | low fill',(.55,-.65,.28),(.1,0,.24),1.5,.45,.45,(1,1,1))
    # No horizon: the tiny shadow anchors the base on an effectively black infinity floor.
    floor=material('Stage | near-black',(.0018,.002,.0025),0,1)
    floor.node_tree.nodes['Principled BSDF'].inputs['Specular IOR Level'].default_value=0
    bpy.ops.mesh.primitive_plane_add(size=200,location=(0,0,-.001));bpy.context.object.name='Infinite black stage';bpy.context.object.data.materials.append(floor)
    base=Vector((0,0,.065));upper=(P['upper_arm_link']+P['forearm_link'])/2;fore=(P['forearm_link']+P['wrist_pitch_link'])/2
    wrist=(P['wrist_pitch_link']+P['wrist_roll_link'])/2;grip=P['tool0']
    specs=[(upper+Vector((.018,-.005,.045)),upper+Vector((-.018,.0,-.045)),Vector((.045,-.16,.025)),Vector((.052,-.16,.015)),85,8),
           (grip,grip,None,None,65,8),
           (base,base+Vector((0,0,.005)),Vector((.25,-.38,.18)),Vector((.22,-.38,.16)),65,6.3),
           (upper,upper,Vector((.20,-.39,.12)),Vector((.16,-.4,.1)),65,6.3),
           (fore,fore,Vector((.18,-.38,.12)),Vector((.14,-.39,.1)),70,6.3),
           (P['forearm_link'],P['forearm_link'],Vector((.10,.27,.04)),Vector((.075,.27,.045)),85,8),
           (wrist,wrist,Vector((.20,-.32,.10)),Vector((.17,-.33,.085)),75,6.3),
           (grip-Vector((.025,0,0)),grip-Vector((.025,0,0)),Vector((.26,-.29,.12)),Vector((.24,-.30,.10)),70,6.3),
           (Vector((.075,0,.24)),Vector((.08,0,.24)),Vector((.64,-1.1,.39)),Vector((.50,-1.13,.35)),46,9)]
    shotlist=[]
    for (slug,title,start,end),spec in zip(SHOTS,specs):
        a,b,da,db,lensmm,fstop=spec;d=bpy.data.cameras.new(slug);c=bpy.data.objects.new(slug,d);cams.objects.link(c);d.lens=lensmm;d.clip_start=.005;d.clip_end=250;d.dof.use_dof=True;d.dof.aperture_fstop=fstop
        focus=empty('FOCUS | '+slug,cams);d.dof.focus_object=focus
        for f,target,delta in [(start,a,da),(end,b,db)]:
            if slug=='02_grip':
                # Tool +X is jaw-opening direction: exact screen-horizontal; +Y screen-up.
                R=Matrix(T['tool0'][:3,:3]);c.location=grip+R@Vector((0,0,.52));c.rotation_euler=R.to_euler()
            else:c.location=target+delta;look(c,target)
            c.keyframe_insert('location',frame=f);c.keyframe_insert('rotation_euler',frame=f);focus.location=target+(delta.normalized()*.028 if delta is not None and slug=='01_surface' else Vector((0,0,0)));focus.keyframe_insert('location',frame=f)
        mark=s.timeline_markers.new(slug,frame=start);mark.camera=c
        shotlist.append({'id':slug,'title':title,'start':start,'end':end,'review':(start+end)//2})
    # Packed transparent title art composited after optical depth of field.
    subprocess.run([str(ROOT/'.venv/bin/python'),str(ROOT/'scripts/launch_titles.py')],check=True)
    s.use_nodes=True;nodes=s.node_tree.nodes;nodes.clear();links=s.node_tree.links
    render=nodes.new('CompositorNodeRLayers');last=render.outputs['Image']
    for i,(_,title,start,end) in enumerate(SHOTS):
        if i==0:continue
        node=nodes.new('CompositorNodeImage');node.image=bpy.data.images.load(str(ROOT/f'cinematic/launch/titles/{i+1:02d}.png'));node.label='EDIT SOURCE: scripts/launch_titles.py | '+title
        scale=nodes.new('CompositorNodeScale');scale.space='RENDER_SIZE';links.new(node.outputs['Image'],scale.inputs['Image'])
        mix=nodes.new('CompositorNodeAlphaOver');links.new(last,mix.inputs[1]);links.new(scale.outputs['Image'],mix.inputs[2])
        for f,value in [(1,0),(start-1,0),(start,1),(end,1),(end+1,0)]:mix.inputs[0].default_value=value;mix.inputs[0].keyframe_insert('default_value',frame=f)
        last=mix.outputs[0]
    for curve in s.node_tree.animation_data.action.fcurves:
        for key in curve.keyframe_points:key.interpolation='CONSTANT'
    output=nodes.new('CompositorNodeComposite');links.new(last,output.inputs[0])
    s.camera=bpy.data.objects['01_surface'];s.frame_set(700)
    s['README']='Animate POSE custom properties (radians/metres). URDF axis drivers clamp limits. Right jaw mimics left. Timeline markers select cameras. Source: model/viscous_arm.urdf.'
    bpy.ops.file.pack_all();bpy.ops.wm.save_as_mainfile(filepath=str(out/'viscous_launch.blend'))
    (out/'motion.json').write_text(json.dumps({'fps':24,'end':768,'duration':32,'finish':args.finish,'source_urdf':'model/viscous_arm.urdf','device':device,'shots':shotlist,'joints':bindings,'materials':assignments,'note':'CAD-derived kinematic film; uncalibrated hardware limits. No dynamics or hardware control.'},indent=2)+'\n')
    print('BUILT',out,flush=True)
if __name__=='__main__':main()
