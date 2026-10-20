"""Reference-informed twin-screw ship stern. Units are metres; forward is +Y.

Import build_ship_stern. Executing this file creates a ship-only review scene.
This is a cinematic asset, not a naval-architecture or manufacturing model.
"""
import math
import random
from pathlib import Path
import bpy
from mathutils import Vector


def _mat(name, color, metallic=0, roughness=.5):
    m = bpy.data.materials.new(name); m.use_nodes = True
    p = m.node_tree.nodes.get('Principled BSDF')
    p.inputs['Base Color'].default_value = (*color, 1)
    p.inputs['Metallic'].default_value = metallic
    p.inputs['Roughness'].default_value = roughness
    m.diffuse_color = (*color, 1)
    return m


def _surface_material(name, colors, metallic, scale, roughness):
    m = _mat(name, colors[0], metallic, roughness)
    n=m.node_tree.nodes; l=m.node_tree.links; p=n.get('Principled BSDF')
    t=n.new('ShaderNodeTexNoise'); t.inputs['Scale'].default_value=scale
    t.inputs['Detail'].default_value=4; t.inputs['Roughness'].default_value=.75
    tex=n.new('ShaderNodeTexCoord'); l.new(tex.outputs['Object'], t.inputs['Vector'])
    ramp=n.new('ShaderNodeValToRGB'); ramp.color_ramp.elements[0].position=.23
    ramp.color_ramp.elements[0].color=(*colors[0],1)
    ramp.color_ramp.elements[1].position=.76; ramp.color_ramp.elements[1].color=(*colors[1],1)
    l.new(t.outputs['Fac'],ramp.inputs[0]); l.new(ramp.outputs[0],p.inputs['Base Color'])
    b=n.new('ShaderNodeBump'); b.inputs['Strength'].default_value=.24; b.inputs['Distance'].default_value=.014
    l.new(t.outputs['Fac'],b.inputs['Height']); l.new(b.outputs[0],p.inputs['Normal'])
    return m


def blade_width(u, prop_diameter=4.6):
    """Full chord length in metres at normalized radial station u."""
    u=max(0,min(1,u))
    return (prop_diameter/4.6)*(.36+1.28*math.sin(math.pi*(u**.83))**.75)*math.sqrt(max(.001,1-u**12))


def blade_surface(prop_index, blade_index, u, v, origin=(0,0,0), prop_diameter=4.6, prop_spacing=7.6):
    """Pure geometry query; no scene or meshes need to exist. Returns point, normal.

    u=0..1 root-to-tip; v=0..1 across chord. Face is the aft (-Y) face.
    """
    scale=prop_diameter/4.6
    def p(uu,vv):
        uu=max(0,min(1,uu));vv=max(0,min(1,vv))
        hand=1 if prop_index==0 else -1
        cx=(-1 if prop_index==0 else 1)*prop_spacing/2
        r=(.42+1.88*uu)*scale;chord=blade_width(uu,prop_diameter)
        tangent=(vv-.5)*chord
        angle=.12+blade_index*math.tau/5+hand*(.10+.48*uu*uu)
        pitch=hand*(.88-.34*uu)
        yy=tangent*math.sin(pitch)+.07*uu*uu*scale
        tang=tangent*math.cos(pitch)
        thickness=(.075*(1-uu)+.019)*math.sin(math.pi*vv)**.65*scale
        return Vector(origin)+Vector((cx+r*math.sin(angle)+tang*math.cos(angle),yy-thickness,r*math.cos(angle)-tang*math.sin(angle)))
    point=p(u,v)
    du=p(min(.99999,u+.0001),v)-p(max(.00001,u-.0001),v)
    dv=p(u,min(.99999,v+.0001))-p(u,max(.00001,v-.0001))
    normal=du.cross(dv).normalized()
    if normal.y>0:normal=-normal
    return point,normal


def build_ship_stern(origin=(0,0,0), prop_diameter=4.6, prop_spacing=7.6, collection_name='SHIP | large twin-screw stern', barnacles=True):
    """Return ship objects plus callable blade_surface(prop_index, blade_index, u, v).

    u in [0,1] is root-to-tip, v in [0,1] crosses the blade chord.
    Surface callable returns (world_point, outward_aft_normal). Cleaning point is
    port propeller, blade 3, u=.65, v about .17 (near edge). Propeller centres: x=+-spacing/2,y=z=0.
    All dimensions scale from a 4.6m propeller. No ROV or environment is created.
    """
    scale=prop_diameter/4.6; off=Vector(origin)
    coll=bpy.data.collections.new(collection_name); bpy.context.scene.collection.children.link(coll)
    def world(p): return off+Vector(p)*scale
    def mesh(name, verts, faces, mat, smooth=True):
        data=bpy.data.meshes.new(name); data.from_pydata([world(v) for v in verts], [], faces); data.update()
        ob=bpy.data.objects.new(name,data); coll.objects.link(ob); data.materials.append(mat)
        if smooth:
            for p in data.polygons:p.use_smooth=True
        return ob
    def relink(ob):
        for c in list(ob.users_collection):c.objects.unlink(ob)
        coll.objects.link(ob)
    def cylinder(name,a,b,r,mat,vertices=64):
        a,b=world(a),world(b); d=b-a
        bpy.ops.mesh.primitive_cylinder_add(vertices=vertices,radius=r*scale,depth=d.length,location=(a+b)/2)
        ob=bpy.context.object; ob.name=name; relink(ob); ob.rotation_euler=d.to_track_quat('Z','Y').to_euler()
        ob.data.materials.append(mat)
        for p in ob.data.polygons:p.use_smooth=True
        bevel=ob.modifiers.new('Cast edge radii','BEVEL'); bevel.width=.025*scale; bevel.segments=3
        ob.modifiers.new('Weighted normals','WEIGHTED_NORMAL')
        return ob
    def tube(name,points,r,mat):
        cu=bpy.data.curves.new(name,'CURVE');cu.dimensions='3D';cu.bevel_depth=r*scale;cu.bevel_resolution=2
        sp=cu.splines.new('POLY');sp.points.add(len(points)-1)
        for p,co in zip(sp.points,points):p.co=(*world(co),1)
        ob=bpy.data.objects.new(name,cu);coll.objects.link(ob);cu.materials.append(mat);return ob
    red=_surface_material('SHIP | worn oxide antifouling',[(.045,.011,.009),(.27,.067,.035)],.1,1.8,.7)
    bronze=_surface_material('SHIP | bronze with mottled patina',[(.09,.11,.044),(.46,.28,.091)],.7,4.8,.39)
    clean=_surface_material('SHIP | freshly exposed propeller bronze',[(.22,.12,.034),(.53,.33,.105)],.85,35,.29)
    steel=_mat('SHIP | shaft steel',(.18,.23,.24),.78,.3)
    seam=_mat('SHIP | welded plating edges',(.085,.025,.017),.3,.67)
    shell=_surface_material('SHIP | small calcareous growth',[(.22,.25,.16),(.57,.52,.35)],0,55,.9)
    # Continuous rounded hull sections, from the raised transom into full draft.
    sections=[(-5.2,8.7,4.25,12.5),(-4.9,9.25,4.0,12.8),(-3,10.3,3.55,13),
              (0,11.25,2.88,13),(3,11.8,1.5,13),(6,12.1,-.1,13),
              (10,12.35,-1.65,13),(15,12.45,-2.4,13),(25,12.45,-2.55,13),
              (42,12.45,-2.55,13),(60,12.45,-2.55,13)]
    rings=96; verts=[]
    def ring_point(section,t):
        y,w,bottom,top=section
        # Rounded bilge and near-vertical upper sides, not a cuboid platform.
        x=w*math.sin(t)
        z=(top+bottom)/2-(top-bottom)/2*math.copysign(abs(math.cos(t))**.72,math.cos(t))
        return (x,y,z)
    for sec in sections:
        verts.extend(ring_point(sec,2*math.pi*j/rings) for j in range(rings))
    faces=[]
    for i in range(len(sections)-1):
        for j in range(rings):faces.append((i*rings+j,i*rings+(j+1)%rings,(i+1)*rings+(j+1)%rings,(i+1)*rings+j))
    faces += [tuple(reversed(range(rings))),tuple((len(sections)-1)*rings+j for j in range(rings))]
    hull=mesh('SHIP / continuous curved stern shell',verts,faces,red)
    mod=hull.modifiers.new('Faired steel hull','SUBSURF');mod.levels=2;mod.render_levels=2
    # Long seams, drawn into hull curvature; welds provide human-scale visual detail.
    for j,t in enumerate([]):
        pts=[ring_point(s,t) for s in sections]
        tube(f'SHIP / longitudinal weld {j}',pts,.009,seam)
    # Centreline skeg grows into hull forward, with curved tapered aft face.
    sv=[(-.12,-2,3.2),(.12,-2,3.2),(-.40,4,1.1),(.40,4,1.1),(-.48,10,-1.3),(.48,10,-1.3),
        (-.11,-1,-.9),(.11,-1,-.9),(-.30,4,-2.3),(.30,4,-2.3),(-.43,10,-2.6),(.43,10,-2.6)]
    sf=[(0,2,8,6),(2,4,10,8),(1,7,9,3),(3,9,11,5),(0,1,3,2),(2,3,5,4),(6,8,9,7),(8,10,11,9),(0,6,7,1),(4,5,11,10)]
    skeg=mesh('SHIP / central stern skeg',sv,sf,red)
    b=skeg.modifiers.new('Rounded welded skeg','BEVEL');b.width=.12;b.segments=4
    skeg.modifiers.new('Skeg normals','WEIGHTED_NORMAL')
    centres=[(-prop_spacing/(2*scale),0,0),(prop_spacing/(2*scale),0,0)]
    blades=[];props=[]
    def surface_local(pi,bi,u,v,side=-1):
        cx,_,_=centres[pi];hand=1 if pi==0 else -1
        u=max(0,min(1,u));r=.42+1.88*u
        # Wide swept marine blades, narrowing smoothly into a rounded tip.
        chord=(.36+1.28*math.sin(math.pi*(u**.83))**.75)*math.sqrt(max(.001,1-u**12))
        tangent=(v-.5)*chord
        angle=.12+bi*math.tau/5+hand*(.10+.48*u*u)
        pitch=hand*(.88-.34*u)
        yy=tangent*math.sin(pitch)+.07*u*u
        tang=tangent*math.cos(pitch)
        thickness=(.075*(1-u)+.019)*math.sin(math.pi*v)**.65
        # Aft (-Y) blade face; thick aerofoil root tapers to thin leading/trailing edges.
        return Vector((cx+r*math.sin(angle)+tang*math.cos(angle),yy+side*thickness, r*math.cos(angle)-tang*math.sin(angle)))
    def blade_surface(pi,bi,u,v):
        p=surface_local(pi,bi,u,v)
        du=surface_local(pi,bi,min(.99999,u+.0001),v)-surface_local(pi,bi,max(.00001,u-.0001),v)
        dv=surface_local(pi,bi,u,min(.99999,v+.0001))-surface_local(pi,bi,u,max(.00001,v-.0001))
        norm=du.cross(dv).normalized()
        if norm.y>0:norm=-norm
        return world(p),norm
    for pi,(cx,_,_) in enumerate(centres):
        label='PORT' if pi==0 else 'STARBOARD'
        cylinder(f'SHIP / {label} propeller shaft',(cx,.2,0),(cx,7.2,.34),.19,steel)
        # Streamlined bearing housing and a pair of brackets reach into the hull.
        cylinder(f'SHIP / {label} stern shaft bearing',(cx,.65,.04),(cx,2.2,.12),.31,red)
        for j,sgn in enumerate([-1,1]):
            a=Vector((cx,1.5,.08));end=Vector((cx+sgn*1.25,3.1,2.0))
            axis=(end-a).normalized();cross=Vector((0,1,0));perp=axis.cross(cross).normalized()
            vv=[]
            for q,w in [(a,.17),(end,.23)]:
                for dy,dx in [(-.55,-1),(.55,-1),(.55,1),(-.55,1)]:vv.append(q+cross*dy+perp*w*dx)
            bracket=mesh(f'SHIP / {label} shaft A bracket {j}',vv,[(0,1,2,3),(4,7,6,5),(0,4,5,1),(1,5,6,2),(2,6,7,3),(3,7,4,0)],red)
            b=bracket.modifiers.new('Foil edge radii','BEVEL');b.width=.11;b.segments=3
            bracket.modifiers.new('Bracket normals','WEIGHTED_NORMAL')
        # Rounded hydrodynamic hub with aft cap.
        hubverts=[];hubfaces=[];profile=[(-.75,.09),(-.64,.26),(-.40,.40),(.12,.43),(.48,.34),(.56,.21)]
        for yy,rr in profile:
            for j in range(64):hubverts.append((cx+rr*math.sin(j*math.tau/64),yy,rr*math.cos(j*math.tau/64)))
        for k in range(len(profile)-1):
            for j in range(64):hubfaces.append((k*64+j,k*64+(j+1)%64,(k+1)*64+(j+1)%64,(k+1)*64+j))
        hubfaces +=[tuple(reversed(range(64))),tuple((len(profile)-1)*64+j for j in range(64))]
        hub=mesh(f'SHIP / {label} cast bronze hub',hubverts,hubfaces,bronze);props.append(hub)
        for bi in range(5):
            nu,nv=42,22;vv=[];ff=[]
            for side in [-1,1]:
                for iu in range(nu+1):
                    for iv in range(nv+1):vv.append(surface_local(pi,bi,iu/nu,iv/nv,side))
            layer=(nu+1)*(nv+1)
            for side in range(2):
                for iu in range(nu):
                    for iv in range(nv):
                        k=side*layer+iu*(nv+1)+iv;quad=(k,k+nv+1,k+nv+2,k+1)
                        ff.append(quad if side==0 else tuple(reversed(quad)))
            perimeter=list(range(nv+1))+[iu*(nv+1)+nv for iu in range(1,nu+1)]+[nu*(nv+1)+iv for iv in reversed(range(nv))]+[iu*(nv+1) for iu in reversed(range(1,nu))]
            for k,j in zip(perimeter,perimeter[1:]+perimeter[:1]):ff.append((k,j,j+layer,k+layer))
            blade=mesh(f'SHIP / {label} swept bronze blade {bi+1}',vv,ff,clean if pi==0 and bi==3 else bronze);blades.append(blade)
    # User-selected single centreline rudder: both screw faces remain unobstructed.
    # Its leading edge follows the aft end of the centreline skeg, and the
    # continuous stock rises into the hull above the foil.
    rv=[];count=48
    for z,chord,leading in [(-2.10,2.75,-2.08),(3.65,3.05,-2.08)]:
        for j in range(count):
            th=j*math.tau/count
            yy=leading-chord*(.5+.5*math.cos(th))
            xx=.28*math.sin(th)*(0.78+.22*math.cos(th))
            rv.append((xx,yy,z))
    rf=[(j,(j+1)%count,count+(j+1)%count,count+j) for j in range(count)]
    rf += [tuple(reversed(range(count))),tuple(count+j for j in range(count))]
    rudder=mesh('SHIP / CENTER single rudder foil',rv,rf,red)
    cylinder('SHIP / CENTER rudder stock',(0,-2.70,3.3),(0,-2.70,5.0),.28,steel)
    cylinder('SHIP / CENTER upper stock bearing',(0,-2.70,3.8),(0,-2.70,4.35),.37,red)
    for zz in [-.55,2.2]:
        cylinder(f'SHIP / CENTER rudder pintle {zz}',(0,-2.13,zz-.16),(0,-2.13,zz+.16),.24,red)
    # Sparse real shell geometry on the broad bronze blades; the close-up work
    # patch is left to the shot builder so it can animate removal precisely.
    rng=random.Random(205)
    if barnacles:
        bv=[];bf=[]
        for pi in range(2):
            for bi in range(5):
                for k in range(70):
                    u=rng.uniform(.16,.94);v=rng.uniform(.08,.92)
                    if pi==0 and bi==3:continue
                    p,n=blade_surface(pi,bi,u,v);p=(p-off)/scale
                    rr=rng.uniform(.010,.031);h=rr*rng.uniform(.6,1.05)
                    ax=n.cross(Vector((1,0,0))).normalized();ay=n.cross(ax).normalized();start=len(bv)
                    for rad,hei in [(rr,0),(rr*.48,h),(rr*.21,h*.87)]:
                        for j in range(7):bv.append(p+ax*math.cos(j*math.tau/7)*rad+ay*math.sin(j*math.tau/7)*rad+n*hei)
                    for layer in range(2):
                        for j in range(7):bf.append((start+layer*7+j,start+layer*7+(j+1)%7,start+(layer+1)*7+(j+1)%7,start+(layer+1)*7+j))
        mesh('SHIP / scattered attached barnacle shells',bv,bf,shell,False)
    u=.65
    chord=blade_width(u,prop_diameter)
    edge_v=.006;pad_v=edge_v+.19/chord
    point,normal=blade_surface(0,3,u,pad_v)
    anchor,anchor_normal=blade_surface(0,3,u,edge_v)
    tangent=(blade_surface(0,3,u,pad_v+.001)[0]-blade_surface(0,3,u,pad_v-.001)[0]).normalized()
    radial=(blade_surface(0,3,u+.001,pad_v)[0]-blade_surface(0,3,u-.001,pad_v)[0]).normalized()
    work_patch={'center':point,'normal':normal,'tangent':tangent,'radial':radial,
                'anchor_point':anchor,'anchor_normal':anchor_normal,'pad_point':point,
                'prop_index':0,'blade_index':3,'u':u,'v':pad_v,'edge_v':edge_v,'blade_width':chord,
                'anchor_thickness':2*(.075*(1-u)+.019)*math.sin(math.pi*edge_v)**.65*scale}
    return {'collection':coll,'hull':hull,'rudder':rudder,'propellers':props,'blades':blades,'materials':{'bronze':bronze,'clean_bronze':clean,'hull':red,'shell':shell},'prop_centres':[world(p) for p in centres], 'prop_radius':prop_diameter/2,'cleaning_point':point,'cleaning_normal':normal,'cleaning_blade':blades[3], 'blade_surface':blade_surface,'cleaning_parameters':(0,3,u,pad_v),'work_patch':work_patch,'blade_width':lambda u:blade_width(u,prop_diameter)}


def preview():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    ship=build_ship_stern();scene=bpy.context.scene
    scene.render.engine='CYCLES';scene.cycles.samples=48;scene.cycles.use_denoising=True
    try:
        prefs=bpy.context.preferences.addons['cycles'].preferences;prefs.compute_device_type='OPTIX';prefs.get_devices()
        for d in prefs.devices:d.use=d.type=='OPTIX'
        scene.cycles.device='GPU'
    except Exception:pass
    scene.world=bpy.data.worlds.new('Underwater blue ambience');scene.world.use_nodes=True
    scene.world.node_tree.nodes['Background'].inputs[0].default_value=(.04,.13,.18,1)
    scene.world.node_tree.nodes['Background'].inputs[1].default_value=.35
    def light(name,loc,power,color,size,target):
        data=bpy.data.lights.new(name,'AREA');data.energy=power;data.color=color;data.shape='DISK';data.size=size
        ob=bpy.data.objects.new(name,data);scene.collection.objects.link(ob);ob.location=loc;ob.rotation_euler=(Vector(target)-ob.location).to_track_quat('-Z','Y').to_euler()
    light('Soft aft water light',(0,-12,7),6000,(.40,.73,1),11,(0,0,1))
    light('Warm grazing inspection illumination',(-7,-6,1),2500,(1,.77,.43),5,(-3.8,0,0))
    light('Port rim',(-12,3,3),5500,(.17,.65,.8),7,(0,0,1))
    light('Starboard rim',(12,1,5),6000,(.35,.64,.74),7,(3.8,0,0))
    data=bpy.data.cameras.new('Aft three-quarter review');cam=bpy.data.objects.new('Aft three-quarter review',data);scene.collection.objects.link(cam)
    cam.location=(-11,-20,-2.8);target=Vector((0,.3,2.2));cam.rotation_euler=(target-cam.location).to_track_quat('-Z','Y').to_euler();data.lens=31;scene.camera=cam
    scene.render.resolution_x=1600;scene.render.resolution_y=1000;scene.render.resolution_percentage=100
    scene.view_settings.view_transform='AgX';scene.view_settings.look='AgX - Medium High Contrast';scene.view_settings.exposure=-.15
    out=Path(__file__).resolve().parents[1]/'cinematic';out.mkdir(exist_ok=True)
    scene.render.filepath=str(out/'ship_stern_preview.png')
    bpy.ops.wm.save_as_mainfile(filepath=str(out/'ship_stern_asset.blend'))
    print('SHIP cleaning point',tuple(ship['cleaning_point']),'normal',tuple(ship['cleaning_normal']),flush=True)
    bpy.ops.render.render(write_still=True)
    for suffix,location,lens in [('aft',(-2,-22,-3.3),34),('outboard',(-15,-16,-2.8),31)]:
        cam.location=location;cam.rotation_euler=(Vector((0,.3,2.3))-cam.location).to_track_quat('-Z','Y').to_euler();data.lens=lens
        scene.render.filepath=str(out/f'ship_stern_{suffix}_preview.png')
        bpy.ops.render.render(write_still=True)

if __name__=='__main__':preview()
