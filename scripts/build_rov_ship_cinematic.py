"""Website film using the actual saved BlueROV2 Heavy bimanual Blender rig."""
from pathlib import Path
import argparse
import json
import math
import random
import bpy
import numpy as np
from mathutils import Matrix, Vector
from ship_stern_asset import build_ship_stern, blade_surface

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'cinematic/rov_ship'
motion = json.loads((OUT/'motion.json').read_text())
p = argparse.ArgumentParser(); p.add_argument('--preview', type=int, default=250)
args = p.parse_args()
bpy.ops.wm.open_mainfile(filepath=motion['source_rig'])
scene = bpy.context.scene
# The existing rig is the source of the entire ROV, its joints, and mount geometry.
for o in list(scene.objects):
    if o.type in {'CAMERA', 'LIGHT'}: bpy.data.objects.remove(o, do_unlink=True)
for marker in list(scene.timeline_markers): scene.timeline_markers.remove(marker)
rov = bpy.data.objects['rov_base']; rov.matrix_world = Matrix(motion['rov_transform'])
scene.render.fps = motion['fps']; scene.frame_start = 1; scene.frame_end = motion['end']
for record in motion['frames'][::2]+[motion['frames'][-1]]:
    for joint, q in record['joints'].items():
        ob = bpy.data.objects[joint+'_origin']; ob['q'] = q; ob.keyframe_insert('["q"]', frame=record['frame'])
for ob in scene.objects:
    if 'rpm' in ob: ob['rpm'] = 48 if 'left' in ob.name else -48
scene.render.engine = 'CYCLES'; scene.cycles.samples = 32; scene.cycles.use_denoising = True
scene.cycles.max_bounces = 6; scene.render.use_persistent_data = True
prefs = bpy.context.preferences.addons['cycles'].preferences
prefs.compute_device_type = 'OPTIX'; prefs.get_devices()
for d in prefs.devices: d.use = d.type == 'OPTIX'
scene.cycles.device = 'GPU'
scene.render.resolution_x = 1280; scene.render.resolution_y = 720; scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = 'PNG'; scene.render.film_transparent = False
scene.view_settings.view_transform = 'AgX'; scene.view_settings.look = 'AgX - Medium High Contrast'
scene.view_settings.exposure = -.25
scene.world = bpy.data.worlds.new('Open blue water'); scene.world.use_nodes = True
scene.world.node_tree.nodes['Background'].inputs[0].default_value = (.025, .078, .11, 1)
scene.world.node_tree.nodes['Background'].inputs[1].default_value = .22


def mat(name, color, metal=0, rough=.4):
    m = bpy.data.materials.new(name); m.use_nodes = True
    bs = m.node_tree.nodes['Principled BSDF']; bs.inputs['Base Color'].default_value = (*color, 1)
    bs.inputs['Metallic'].default_value = metal; bs.inputs['Roughness'].default_value = rough
    return m


def cylinder(name, radius, depth, material, parent=None, loc=(0, 0, 0)):
    bpy.ops.mesh.primitive_cylinder_add(vertices=64, radius=radius, depth=depth)
    o = bpy.context.object; o.name = name; o.parent = parent; o.location = loc; o.data.materials.append(material)
    for face in o.data.polygons: face.use_smooth = True
    bevel = o.modifiers.new('Machined edge radius', 'BEVEL'); bevel.width = .001; bevel.segments = 3
    o.modifiers.new('Weighted normals', 'WEIGHTED_NORMAL')
    return o


def tube(name, points, radius, material):
    c = bpy.data.curves.new(name, 'CURVE'); c.dimensions = '3D'; c.bevel_depth = radius; c.bevel_resolution = 3
    s = c.splines.new('BEZIER'); s.bezier_points.add(len(points)-1)
    for p, co in zip(s.bezier_points, points): p.co = co; p.handle_left_type = p.handle_right_type = 'AUTO'
    o = bpy.data.objects.new(name, c); scene.collection.objects.link(o); c.materials.append(material)
    return o


def light(name, position, target, power, color, size):
    d = bpy.data.lights.new(name, 'AREA'); d.energy = power; d.color = color; d.shape = 'DISK'; d.size = size
    o = bpy.data.objects.new(name, d); scene.collection.objects.link(o); o.location = position
    o.rotation_euler = (Vector(target)-o.location).to_track_quat('-Z', 'Y').to_euler()
    return o


C = Vector(motion['work']['center']); N = Vector(motion['work']['normal'])
L = Vector(motion['work']['left']); UP = Vector(motion['work']['up'])
ship = build_ship_stern(barnacles=False)
pi, bi = motion['work']['prop_index'], motion['work']['blade_index']
target_blade = ship['blades'][pi*5+bi]
# Restore uniform worn metal on the asset's default work blade; our accessible
# work point is selected from the actual mounted ROV's reach instead.
for blade in ship['blades']:
    blade.data.materials.clear(); blade.data.materials.append(ship['materials']['bronze'])

# A true gripped tool attachment; the ROV geometry and mounts are untouched.
dark = mat('Tool / graphite drive', (.012, .022, .03), .65, .29)
orange = mat('Tool / orange motor shell', (.56, .11, .025), .45, .31)
rubber = mat('Tool / rubber handle', (.007, .011, .014), .05, .8)
abrasive = mat('Tool / burgundy abrasive fibres', (.11, .014, .025), 0, .87)
nt = abrasive.node_tree; tex = nt.nodes.new('ShaderNodeTexNoise'); tex.inputs['Scale'].default_value = 170
bump = nt.nodes.new('ShaderNodeBump'); bump.inputs['Strength'].default_value = .7; bump.inputs['Distance'].default_value = .003
nt.links.new(tex.outputs['Fac'], bump.inputs['Height']); nt.links.new(bump.outputs[0], nt.nodes['Principled BSDF'].inputs['Normal'])
tool = bpy.data.objects['right_tool0']
cylinder('Polisher / gripped handle', .012, .058, rubber, tool, (0, 0, -.009))
cylinder('Polisher / sealed rotary drive', .024, .035, orange, tool, (0, 0, .028))
cylinder('Polisher / spindle', .009, .015, dark, tool, (0, 0, .049))
disc = cylinder('Polisher / backing disc', .047, .010, dark, tool, (0, 0, .061))
cylinder('Polisher / flat abrasive pad', .047, .014, abrasive, disc, (0, 0, .009))
for a in range(6):
    theta = a*math.tau/6
    mark = cylinder('Polisher / rotating orange marker', .003, .002, orange, disc, (.034*math.cos(theta), .034*math.sin(theta), -.006))
for f, phase in [(1, 0), (130, 0), (150, 8), (394, 8+244*.68), (423, 8+244*.68+8), (480, 8+244*.68+8)]:
    disc.rotation_euler[2] = phase; disc.keyframe_insert('rotation_euler', frame=f)

# The same material covers the work blade; the contact footprint reveals bronze.
film = mat('Working blade / accumulated cleaning reveal', (.06, .08, .022), .1, .8)
nodes, links = film.node_tree.nodes, film.node_tree.links; bs = nodes['Principled BSDF']
geo = nodes.new('ShaderNodeNewGeometry'); noise = nodes.new('ShaderNodeTexNoise'); noise.inputs['Scale'].default_value = 160
noise.inputs['Detail'].default_value = 3; links.new(geo.outputs['Position'], noise.inputs['Vector'])
ramp = nodes.new('ShaderNodeValToRGB'); ramp.color_ramp.elements[0].color = (.018, .028, .009, 1)
ramp.color_ramp.elements[1].color = (.17, .17, .055, 1); links.new(noise.outputs['Fac'], ramp.inputs[0])
attr = nodes.new('ShaderNodeAttribute'); attr.attribute_name = 'clean_at'
clock = nodes.new('ShaderNodeValue')
for f in [1, 480]: clock.outputs[0].default_value = f; clock.outputs[0].keyframe_insert('default_value', frame=f)
sub = nodes.new('ShaderNodeMath'); sub.operation = 'SUBTRACT'; links.new(clock.outputs[0], sub.inputs[0]); links.new(attr.outputs['Fac'], sub.inputs[1])
reveal = nodes.new('ShaderNodeMapRange'); reveal.inputs['From Min'].default_value = -1; reveal.inputs['From Max'].default_value = 3
links.new(sub.outputs[0], reveal.inputs[0])
mix = nodes.new('ShaderNodeMixRGB'); mix.inputs[2].default_value = (.48, .29, .087, 1)
links.new(reveal.outputs[0], mix.inputs[0]); links.new(ramp.outputs[0], mix.inputs[1]); links.new(mix.outputs[0], bs.inputs['Base Color'])
for socket, low, high in [('Roughness', .85, .23), ('Metallic', .13, .85)]:
    n = nodes.new('ShaderNodeMapRange'); n.inputs['To Min'].default_value = low; n.inputs['To Max'].default_value = high
    links.new(reveal.outputs[0], n.inputs[0]); links.new(n.outputs[0], bs.inputs[socket])
bump = nodes.new('ShaderNodeBump'); bump.inputs['Distance'].default_value = .0015
links.new(noise.outputs['Fac'], bump.inputs['Height']); links.new(bump.outputs[0], bs.inputs['Normal'])
n = nodes.new('ShaderNodeMapRange'); n.inputs['To Min'].default_value = .45; n.inputs['To Max'].default_value = .015
links.new(reveal.outputs[0], n.inputs[0]); links.new(n.outputs[0], bump.inputs['Strength'])
path = [(r['frame'], np.array(r['pad'])) for r in motion['frames'] if 150 <= r['frame'] <= 394]


def contact_frame(position, radius=.047):
    p = np.asarray(position)
    for f, point in path:
        if np.linalg.norm(p-point) < radius: return f
    return 10000


# Dense geometry only around the actual small work area on the 4.6m propeller.
us = sorted(set(np.linspace(0, 1, 90).tolist()+np.linspace(.625, .735, 150).tolist()))
vs = sorted(set(np.linspace(0, 1, 90).tolist()+np.linspace(.76, .98, 180).tolist()))
verts, faces = [], []
for u in us:
    for v in vs:
        point, normal = blade_surface(pi, bi, u, v); verts.append(tuple(point+normal*.0005))
for i in range(len(us)-1):
    for j in range(len(vs)-1):
        a = i*len(vs)+j; faces.append((a, a+len(vs), a+len(vs)+1, a+1))
positions = np.asarray(verts); times = np.full(len(verts), 10000., dtype=np.float32)
for f, point in path:
    delta = positions-point; hit = (np.einsum('ij,ij->i', delta, delta) < .047**2) & (times == 10000); times[hit] = f
mesh = bpy.data.meshes.new('Working blade / dense surface'); mesh.from_pydata(verts, [], faces); mesh.update()
mesh.attributes.new('clean_at', 'FLOAT', 'POINT').data.foreach_set('value', times)
face = bpy.data.objects.new('Working blade / fouled surface', mesh); scene.collection.objects.link(face); mesh.materials.append(film)
for poly in mesh.polygons: poly.use_smooth = True

# Hollow shell forms. Static shells are combined; removable patch shells animate.
shell_mat = mat('Barnacle / calcified shell', (.36, .34, .235), .05, .85)
shell_hole = mat('Barnacle / dark opening', (.03, .037, .014), 0, .95)
shell_verts, shell_faces = [], []
for rad, height in [(1, 0), (.85, .3), (.45, .9), (.23, .87), (.22, .35)]:
    for j in range(9):
        a = j*math.tau/9; rr = rad*(1+.1*math.sin(2.7*j)); shell_verts.append((rr*math.cos(a), rr*math.sin(a), height))
for k in range(4):
    for j in range(9): shell_faces.append((k*9+j, k*9+(j+1)%9, (k+1)*9+(j+1)%9, (k+1)*9+j))
shell_faces.append(tuple(range(36, 45)))
sm = bpy.data.meshes.new('Hollow barnacle'); sm.from_pydata(shell_verts, [], shell_faces); sm.materials.append(shell_mat); sm.materials.append(shell_hole)
for poly in sm.polygons:
    if poly.index >= 27: poly.material_index = 1
rng = random.Random(720)
static_verts, static_faces = [], []
for prop in range(2):
    for blade in range(5):
        samples = [(rng.uniform(.12, .96), rng.uniform(.03, .97), rng.uniform(.009, .024)) for _ in range(110)]
        if (prop, blade) == (pi, bi):
            samples += [(rng.uniform(.625, .735), rng.uniform(.77, .98), rng.uniform(.0035, .009)) for _ in range(160)]
        for u, v, radius in samples:
            point, normal = blade_surface(prop, blade, u, v)
            rotation = normal.to_track_quat('Z', 'Y')
            f = contact_frame(point, .044) if (prop, blade) == (pi, bi) else 10000
            if f < 1000:
                o = bpy.data.objects.new('Growth / removed under pad', sm); scene.collection.objects.link(o)
                o.location = point; o.rotation_mode = 'QUATERNION'; o.rotation_quaternion = rotation; o.scale = (radius,)*3
                o.keyframe_insert('scale', frame=f-2); o.scale = (.000001,)*3; o.keyframe_insert('scale', frame=f+3)
                flake = bpy.data.objects.new('Debris / drifting fragment', sm); scene.collection.objects.link(flake)
                flake.location = point; flake.rotation_mode = 'QUATERNION'; flake.rotation_quaternion = rotation; flake.scale = (.000001,)*3
                flake.keyframe_insert('scale', frame=f); flake.scale = (radius*.3,)*3; flake.keyframe_insert('scale', frame=f+3)
                flake.keyframe_insert('location', frame=f)
                flake.location += normal*.03+Vector((rng.uniform(-.04, .04), -.03, -.08))
                flake.keyframe_insert('location', frame=f+40); flake.scale = (.000001,)*3; flake.keyframe_insert('scale', frame=f+44)
            else:
                start = len(static_verts)
                static_verts.extend(tuple(point+rotation@Vector(vv)*radius) for vv in shell_verts)
                static_faces.extend(tuple(start+j for j in ff) for ff in shell_faces)
sm_static = bpy.data.meshes.new('Ship / fixed marine growth'); sm_static.from_pydata(static_verts, [], static_faces)
o = bpy.data.objects.new('Ship / fixed marine growth', sm_static); scene.collection.objects.link(o); sm_static.materials.append(shell_mat); sm_static.materials.append(shell_hole)
for poly in sm_static.polygons:
    if poly.index % len(shell_faces) >= 27: poly.material_index = 1

# Tether exits the real vehicle's rear, without inventing a carrier or table.
M = Matrix(motion['rov_transform'])
tether_start = M@Vector((-.27, .04, .04))
tube('ROV / trailing tether', [tether_start, tether_start+N*.5-UP*.12, tether_start+N*1.5+L*.3,
                             tether_start+N*3+L*1.1+UP*.8, tether_start+N*8+L*3+UP*4], .005, rubber)
light('Ocean / broad overhead', (-3, -7, 10), (0, 0, 0), 2200, (.32, .66, .85), 10)
light('Ship / cool port rim', (-13, 4, 4), (-3, 0, 0), 2500, (.10, .56, .73), 7)
light('Ship / starboard rim', (12, 1, 4), (4, 0, 0), 2500, (.27, .59, .74), 7)
light('Ship / aft soft fill', (0, -10, 2), (0, 0, 1), 1500, (.55, .74, .83), 8)
light('ROV / aluminium key', C+N*1.2+L*.8+UP*1.7, C+N*.35, 90, (.65, .85, 1), 1.7)
light('ROV / soft face fill', C+N*1.5-L*.8+UP*.3, C, 32, (1, .84, .64), 1.0)
strip = light('Bronze / polishing reflection', C+N*.8-L*.8-UP*.4, C, 12, (1, .86, .58), .18)
strip.data.shape = 'RECTANGLE'; strip.data.size = .1; strip.data.size_y = .6
for y in [-.2, .2]:
    light('ROV / mounted work light', M@Vector((.28, y, .12)), C, 5, (.65, .86, 1), .04)

# Subtle water volume. No floor, presentation platform, or invented vehicle body.
volume = bpy.data.materials.new('Water / blue extinction'); volume.use_nodes = True
nn = volume.node_tree.nodes; nn.clear(); output = nn.new('ShaderNodeOutputMaterial'); shader = nn.new('ShaderNodeVolumePrincipled')
shader.inputs['Density'].default_value = .018; shader.inputs['Color'].default_value = (.12, .35, .44, 1)
shader.inputs['Anisotropy'].default_value = .25; volume.node_tree.links.new(shader.outputs['Volume'], output.inputs['Volume'])
bpy.ops.mesh.primitive_cube_add(size=1, location=(0, 5, 0)); water = bpy.context.object; water.name = 'Open water volume'; water.scale = (90, 120, 70); water.data.materials.append(volume)


def camera(name, shots, focus=None, fstop=7):
    d = bpy.data.cameras.new(name); o = bpy.data.objects.new(name, d); scene.collection.objects.link(o)
    d.clip_start = .02; d.clip_end = 300; d.dof.use_dof = focus is not None; d.dof.aperture_fstop = fstop
    if focus is not None:
        target = bpy.data.objects.new(name+' focus', None); scene.collection.objects.link(target); target.location = focus; d.dof.focus_object = target
    for f, pos, target, lens in shots:
        o.location = pos; o.rotation_euler = (Vector(target)-o.location).to_track_quat('-Z', 'Y').to_euler(); d.lens = lens
        o.keyframe_insert('location', frame=f); o.keyframe_insert('rotation_euler', frame=f); d.keyframe_insert('lens', frame=f)
    return o


wide = camera('CAMERA / twin screws and central rudder', [(1, (-1.6, -20, -2.6), (0, 0, 1.7), 31), (96, (-2.6, -18, -2.6), (-.3, 0, 1.4), 32)])
hero_target = C+N*.24+UP*.12
hero = camera('CAMERA / existing ROV at work', [(97, C+N*2.0+L*1.9+UP*.85, hero_target, 43),
              (210, C+N*1.7+L*1.65+UP*.7, hero_target, 43)], C+N*.20, 7)
close = camera('CAMERA / abrasive contact detail', [(211, C+N*1.15+L*1.25+UP*.45, C+N*.12, 47),
               (394, C+N*1.07+L*1.15+UP*.43, C+N*.10, 47)], C+N*.08, 8)
reveal = camera('CAMERA / ROV and cleaned bronze', [(395, C+N*1.5+L*1.6+UP*.65, hero_target, 43),
                (480, C+N*2.0+L*2.0+UP*.9, hero_target, 40)], C+N*.2, 8)
for f, cam in [(1, wide), (97, hero), (211, close), (395, reveal)]:
    marker = scene.timeline_markers.new(cam.name, frame=f); marker.camera = cam
scene.camera = wide
for action in bpy.data.actions:
    for layer in action.layers:
        for strip in layer.strips:
            for bag in strip.channelbags:
                for curve in bag.fcurves:
                    for k in curve.keyframe_points: k.interpolation = 'LINEAR'
scene['source_rov'] = motion['source_rig']
scene['design'] = 'Actual BlueROV2 Heavy bimanual CAD rig; large twin-screw ship with one centerline rudder.'
scene.frame_set(args.preview)
scene.render.filepath = str(OUT/f'preview_{args.preview:04d}.png')
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'viscous_rov_ship_cleaning.blend'), compress=True)
bpy.ops.render.render(write_still=True)
print('Native ROV ship scene saved', flush=True)
