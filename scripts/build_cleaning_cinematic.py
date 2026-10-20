"""Build the website film from the existing CAD meshes and baked URDF motion.

Run with Blender 4.5's Python module. All geometry is embedded in the .blend.
"""
from pathlib import Path
import argparse
import json
import math
import random
import struct
import xml.etree.ElementTree as ET
import bpy
from mathutils import Matrix, Vector
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'cinematic'
MOTION = json.loads((OUT/'motion.json').read_text())
random.seed(23)
parser = argparse.ArgumentParser()
parser.add_argument('--preview', type=int, default=0)
parser.add_argument('--render', action='store_true')
parser.add_argument('--width', type=int, default=1280)
args = parser.parse_args()
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.render.engine = 'CYCLES'
scene.cycles.samples = 32
scene.cycles.use_denoising = True
scene.cycles.max_bounces = 6
scene.cycles.transparent_max_bounces = 4
try:
    prefs = bpy.context.preferences.addons['cycles'].preferences
    prefs.compute_device_type = 'OPTIX'
    prefs.get_devices()
    for device in prefs.devices:
        device.use = device.type == 'OPTIX'
    if any(d.type == 'OPTIX' for d in prefs.devices):
        scene.cycles.device = 'GPU'
except Exception as exc:
    print('Cycles device setup:', exc)
scene.render.resolution_x = args.width
scene.render.resolution_y = round(args.width*9/16)
scene.render.resolution_percentage = 100
scene.render.fps = MOTION['fps']
scene.frame_start, scene.frame_end = 1, MOTION['end']
scene.render.image_settings.file_format = 'PNG'
scene.render.film_transparent = False
scene.view_settings.view_transform = 'AgX'
scene.view_settings.look = 'AgX - Medium High Contrast'
scene.view_settings.exposure = -.45
scene.world = bpy.data.worlds.new('Deep teal water')
scene.world.use_nodes = True
scene.world.node_tree.nodes['Background'].inputs[0].default_value = (.045, .12, .16, 1)
scene.world.node_tree.nodes['Background'].inputs[1].default_value = .18


def material(name, color, metal=0, rough=.4):
    m = bpy.data.materials.new(name); m.diffuse_color = (*color, 1); m.use_nodes = True
    bs = m.node_tree.nodes.get('Principled BSDF')
    bs.inputs['Base Color'].default_value = (*color, 1)
    bs.inputs['Metallic'].default_value = metal
    bs.inputs['Roughness'].default_value = rough
    return m


alloy = material('Satin machined aluminium', (.47, .57, .62), .8, .27)
dark = material('Graphite anodised housings', (.018, .034, .044), .65, .3)
black = material('Rubber and printed gripper pads', (.009, .018, .021), .05, .52)
orange = material('Burnt orange ceramic panels', (.64, .16, .037), .48, .31)
steel = material('Exposed stainless fasteners', (.23, .29, .32), .85, .23)
bronze = material('Propeller bronze', (.42, .24, .075), .85, .25)
pad_mat = material('Burgundy nonwoven abrasive', (.115, .016, .027), .05, .9)
shell = material('Calcareous barnacle shells', (.47, .43, .30), .08, .82)
shell_inner = material('Barnacle openings', (.055, .065, .036), .0, .95)
growth = material('Olive marine biofilm', (.044, .069, .022), .06, .9)


def texture_bump(mat, scale, strength, distance):
    n = mat.node_tree.nodes; l = mat.node_tree.links
    noise = n.new('ShaderNodeTexNoise'); noise.inputs['Scale'].default_value = scale
    noise.inputs['Detail'].default_value = 3
    bump = n.new('ShaderNodeBump'); bump.inputs['Strength'].default_value = strength
    bump.inputs['Distance'].default_value = distance
    l.new(noise.outputs['Fac'], bump.inputs['Height'])
    l.new(bump.outputs[0], n.get('Principled BSDF').inputs['Normal'])
    return noise


texture_bump(alloy, 350, .14, .00025)
texture_bump(pad_mat, 180, .8, .004)
texture_bump(shell, 80, .7, .001)
texture_bump(bronze, 240, .15, .0003)


def assign(obj, mat):
    obj.data.materials.append(mat)
    return obj


def bevel(obj, width=.005, segments=3):
    mod = obj.modifiers.new('Soft manufactured edges', 'BEVEL')
    mod.width, mod.segments = width, segments
    mod = obj.modifiers.new('Weighted corner normals', 'WEIGHTED_NORMAL')
    return obj


def cube(name, loc, scale, mat, rounding=.006):
    bpy.ops.mesh.primitive_cube_add(size=1, location=loc)
    obj = bpy.context.object; obj.name = name; obj.dimensions = scale
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    assign(obj, mat)
    if rounding: bevel(obj, rounding)
    return obj


def cylinder(name, loc, radius, depth, mat, direction=(0, 0, 1), vertices=64):
    bpy.ops.mesh.primitive_cylinder_add(vertices=vertices, radius=radius, depth=depth, location=loc)
    obj = bpy.context.object; obj.name = name
    obj.rotation_euler = Vector(direction).to_track_quat('Z', 'Y').to_euler()
    assign(obj, mat); bevel(obj, min(.002, depth/6), 2)
    for p in obj.data.polygons: p.use_smooth = True
    return obj


def sphere(name, loc, scale, mat, segments=24):
    bpy.ops.mesh.primitive_uv_sphere_add(segments=segments, ring_count=12, radius=1, location=loc)
    o = bpy.context.object; o.name = name; o.scale = scale; assign(o, mat)
    for p in o.data.polygons: p.use_smooth = True
    return o


def tube(name, points, radius, mat):
    c = bpy.data.curves.new(name, 'CURVE'); c.dimensions = '3D'; c.resolution_u = 24
    c.bevel_depth = radius; c.bevel_resolution = 3
    s = c.splines.new('BEZIER'); s.bezier_points.add(len(points)-1)
    for p, co in zip(s.bezier_points, points):
        p.co = co; p.handle_left_type = p.handle_right_type = 'AUTO'
    o = bpy.data.objects.new(name, c); scene.collection.objects.link(o); assign(o, mat)
    return o


def key_transform(obj, matrix, frame):
    obj.matrix_world = Matrix(matrix)
    obj.keyframe_insert('location', frame=frame)
    obj.rotation_mode = 'QUATERNION'
    obj.keyframe_insert('rotation_quaternion', frame=frame)


# Preserve actual per-link visual meshes, including the CAD finger geometry.
urdf = ET.parse(ROOT/'model/viscous_arm.urdf').getroot()
arm_roots = []
mesh_cache = {}
for arm_index, role in enumerate(['ANCHOR', 'POLISH']):
    roots = {}
    collection = bpy.data.collections.new(f'{role} | existing Viscous Arm CAD')
    scene.collection.children.link(collection)
    for link in urdf.findall('link'):
        name = link.get('name')
        root = bpy.data.objects.new(f'{role} / {name}', None); collection.objects.link(root)
        roots[name] = root
        for visual in link.findall('visual'):
            filename = visual.find('geometry/mesh').get('filename')
            if filename not in mesh_cache:
                raw = (ROOT/'model'/filename).read_bytes(); count = struct.unpack_from('<I', raw, 80)[0]
                arr = np.frombuffer(raw, dtype=np.dtype([('normal', '<f4', (3,)), ('v', '<f4', (3, 3)), ('attr', '<u2')]), count=count, offset=84)
                verts, inv = np.unique(arr['v'].reshape(-1, 3), axis=0, return_inverse=True)
                mesh = bpy.data.meshes.new(Path(filename).stem)
                mesh.from_pydata(verts.tolist(), [], inv.reshape(-1, 3).tolist()); mesh.update()
                rgba = [float(v) for v in visual.find('material/color').get('rgba').split()]
                mat = alloy if max(rgba[:3]) > .5 else (black if name.startswith('gripper_') else dark)
                mesh.materials.append(mat)
                # Keep planar CAD faces crisp; fine tessellation already describes curved surfaces.
                mesh_cache[filename] = mesh
            obj = bpy.data.objects.new(f'{role} / {visual.get("name")}', mesh_cache[filename])
            collection.objects.link(obj); obj.parent = root
            origin = visual.find('origin')
            if origin is not None:
                obj.location = [float(x) for x in origin.get('xyz', '0 0 0').split()]
                obj.rotation_euler = [float(x) for x in origin.get('rpy', '0 0 0').split()]
    for record in MOTION['frames'][::2] + [MOTION['frames'][-1]]:
        for name, matrix in record['arms'][arm_index].items():
            key_transform(roots[name], matrix, record['frame'])
    arm_roots.append(roots)

# Compact concept carrier. The two existing arms remain the hero assets.
cube('ROV / pressure chassis', (-.055, -.20, .045), (.61, .40, .15), dark, .035)
cube('ROV / upper orange fairing', (-.055, -.20, .119), (.57, .36, .035), orange, .026)
cube('ROV / central service cover', (-.03, -.20, .143), (.22, .29, .024), dark, .013)
for i, base in enumerate(MOTION['bases']):
    cube(f'ROV / arm pedestal {i}', (base[0], base[1], base[2]-.014), (.135, .12, .028), steel, .006)
    for dx in [-.047, .047]:
        for dy in [-.04, .04]:
            cylinder('Captive pedestal screw', (base[0]+dx, base[1]+dy, base[2]+.001), .004, .004, steel, vertices=6)
for x in [-.36, .25]:
    for y in [-.34, -.06]:
        cylinder('ROV / shrouded vertical thruster', (x, y, .045), .061, .084, dark)
        cylinder('ROV / thruster inset', (x, y, .089), .049, .004, black)
        cylinder('ROV / thruster spinner', (x, y, .096), .013, .013, orange)
        for a in range(4):
            blade = cube('ROV / impeller vane', (x, y, .095), (.080, .010, .005), steel, .002)
            blade.rotation_euler[2] = a*math.pi/4
for x in [-.31, .21]:
    tube('ROV / protective skid', [(x, -.4, .065), (x, -.42, -.035), (x, -.02, -.035), (x, .01, .07)], .012, steel)
    tube('ROV / lifting rail', [(x, -.36, .12), (x, -.34, .21), (x, -.12, .21), (x, -.10, .12)], .008, dark)
tube('Umbilical leaving frame', [(-.09, -.40, .04), (-.16, -.63, .0), (-.40, -.9, .12), (-.9, -1.25, .6), (-1.4, -1.4, 1.3)], .009, black)

led = material('Cool white work lamps', (.65, .9, 1), .2, .2)
led.node_tree.nodes['Principled BSDF'].inputs['Emission Color'].default_value = (.58, .88, 1, 1)
led.node_tree.nodes['Principled BSDF'].inputs['Emission Strength'].default_value = 5
for x in [-.24, .13]:
    cylinder('ROV / lamp bezel', (x, .008, .062), .027, .04, steel, (0, 1, .1))
    cylinder('ROV / lamp lens', (x, .03, .064), .022, .003, led, (0, 1, .1))

# A gripped polisher handle, a stationary motor, and an independently rotating disc.
tool = arm_roots[1]['tool0']
def local_cylinder(name, z, radius, depth, mat):
    o = cylinder(name, (0, 0, z), radius, depth, mat)
    o.parent = tool
    return o
local_cylinder('Polisher / gripped handle', -.005, .012, .066, black)
local_cylinder('Polisher / sealed drive housing', .030, .024, .036, orange)
local_cylinder('Polisher / spindle', .052, .009, .015, steel)
disc = local_cylinder('Polisher / spinning backing plate', .062, .047, .009, dark)
pad = cylinder('Polisher / flat abrasive pad', (0, 0, .007), .047, .013, pad_mat)
pad.parent = disc
for a in range(6):
    angle = a*math.tau/6
    mark = cube('Polisher / backing plate radial slot', (.032*math.cos(angle), .032*math.sin(angle), -.006), (.015, .004, .002), orange, .001)
    mark.rotation_euler[2] = angle; mark.parent = disc
for f, angle in [(1, 0), (82, 0), (100, 20), (344, 20+244*.70), (367, 20+244*.70+8), (432, 20+244*.70+8)]:
    disc.rotation_euler[2] = angle; disc.keyframe_insert('rotation_euler', frame=f)

# Swept four-blade ship propeller. Individual blades have actual thickness.
center_z = .76
path = [(r['frame'], np.array(r['pad'])) for r in MOTION['frames'] if 100 <= r['frame'] <= 344]
def clean_frame(p, radius=.047):
    for f, point in path:
        if np.linalg.norm(np.array(p)-point) < radius:
            return float(f)
    return 10000.


def blade_point(r, t, angle):
    # Lower blade occupies x/z plane with a 14-degree pitch at its cleaning face.
    return (t*math.cos(angle)+r*math.sin(angle), .13+.25*t,
            center_z-r*math.cos(angle)+t*math.sin(angle))


def blade_width(r):
    u = (r-.085)/.47
    return .119*max(0, math.sin(math.pi*u))**.65+.010


film = material('Bronze under marine growth / animated scrub reveal', (.15, .13, .035), .6, .6)
nodes = film.node_tree.nodes; links = film.node_tree.links; bs = nodes.get('Principled BSDF')
noise = texture_bump(film, 190, .55, .0015)
ramp = nodes.new('ShaderNodeValToRGB')
ramp.color_ramp.elements[0].position = .23; ramp.color_ramp.elements[0].color = (.015, .029, .009, 1)
ramp.color_ramp.elements[1].position = .8; ramp.color_ramp.elements[1].color = (.19, .18, .055, 1)
links.new(noise.outputs['Fac'], ramp.inputs[0])
attr = nodes.new('ShaderNodeAttribute'); attr.attribute_name = 'clean_at'
clock = nodes.new('ShaderNodeValue'); clock.name = 'Cleaning timeline'
for f in [1, 432]:
    clock.outputs[0].default_value = f; clock.outputs[0].keyframe_insert('default_value', frame=f)
sub = nodes.new('ShaderNodeMath'); sub.operation = 'SUBTRACT'
links.new(clock.outputs[0], sub.inputs[0]); links.new(attr.outputs['Fac'], sub.inputs[1])
remap = nodes.new('ShaderNodeMapRange'); remap.inputs['From Min'].default_value = -2; remap.inputs['From Max'].default_value = 3
links.new(sub.outputs[0], remap.inputs[0])
mix = nodes.new('ShaderNodeMixRGB'); mix.inputs[2].default_value = (.49, .30, .095, 1)
links.new(remap.outputs[0], mix.inputs[0]); links.new(ramp.outputs[0], mix.inputs[1]); links.new(mix.outputs[0], bs.inputs['Base Color'])
rough = nodes.new('ShaderNodeMapRange'); rough.inputs['To Min'].default_value = .86; rough.inputs['To Max'].default_value = .24
links.new(remap.outputs[0], rough.inputs[0]); links.new(rough.outputs[0], bs.inputs['Roughness'])
metal = nodes.new('ShaderNodeMapRange'); metal.inputs['To Min'].default_value = .15; metal.inputs['To Max'].default_value = .83
links.new(remap.outputs[0], metal.inputs[0]); links.new(metal.outputs[0], bs.inputs['Metallic'])
bump_amount = nodes.new('ShaderNodeMapRange'); bump_amount.inputs['To Min'].default_value = .55; bump_amount.inputs['To Max'].default_value = .015
links.new(remap.outputs[0], bump_amount.inputs[0])
links.new(bump_amount.outputs[0], next(n for n in nodes if n.type == 'BUMP').inputs['Strength'])

for b in range(4):
    angle = b*math.tau/4
    verts, faces, times = [], [], []
    nr, nt = (300, 180) if b == 0 else (100, 40)
    for i in range(nr+1):
        r = .085+.47*i/nr; width = blade_width(r); sweep = .046*math.sin((r-.085)/.47*math.pi*.8)
        for j in range(nt+1):
            t = sweep+width*(j/nt*2-1)
            p = blade_point(r, t, angle); verts.append(p)
    times = np.full(len(verts), 10000., dtype=np.float32)
    if b == 0:
        positions = np.asarray(verts)
        for frame, point in path:
            delta = positions-point
            hit = (np.einsum('ij,ij->i', delta, delta) < .047**2) & (times == 10000)
            times[hit] = frame
    for i in range(nr):
        for j in range(nt):
            a = i*(nt+1)+j; faces.append((a, a+1, a+nt+2, a+nt+1))
    mesh = bpy.data.meshes.new(f'Propeller blade {b+1}'); mesh.from_pydata(verts, [], faces); mesh.update()
    attribute = mesh.attributes.new('clean_at', 'FLOAT', 'POINT')
    attribute.data.foreach_set('value', times)
    blade = bpy.data.objects.new(f'Propeller / blade {b+1}', mesh); scene.collection.objects.link(blade); assign(blade, film)
    solid = blade.modifiers.new('Cast blade thickness', 'SOLIDIFY'); solid.thickness = .008
    solid.offset = 1
    for p in mesh.polygons: p.use_smooth = True

# Sculpted shell mesh shared by barnacles; the central hole stays dark.
vs, fs = [], []
for ring, (radius, height) in enumerate([(1, 0), (.86, .30), (.47, .92), (.24, .90), (.22, .46)]):
    for j in range(10):
        a = j*math.tau/10; rr = radius*(1+.13*math.sin(j*2.3))
        vs.append((rr*math.cos(a), rr*math.sin(a), height))
for ring in range(4):
    for j in range(10): fs.append((ring*10+j, ring*10+(j+1)%10, (ring+1)*10+(j+1)%10, (ring+1)*10+j))
fs.append(tuple(range(40, 50)))
mesh = bpy.data.meshes.new('Ribbed hollow barnacle'); mesh.from_pydata(vs, [], fs); mesh.materials.append(shell); mesh.materials.append(shell_inner)
for p in mesh.polygons:
    if p.index >= 30: p.material_index = 1
for b in range(4):
    angle = b*math.tau/4
    for j in range(155):
        r = random.uniform(.14, .535); t = .046*math.sin((r-.085)/.47*math.pi*.8)+random.uniform(-.88, .88)*blade_width(r)
        p = np.array(blade_point(r, t, angle)); p[1] -= .003
        o = bpy.data.objects.new('Marine growth / barnacle', mesh); scene.collection.objects.link(o); o.location = p
        o.rotation_mode = 'QUATERNION'
        o.rotation_quaternion = Vector((.25*math.cos(angle), -1, .25*math.sin(angle))).to_track_quat('Z', 'Y')
        radius = random.uniform(.003, .009); o.scale = (radius, radius, radius*random.uniform(.7, 1.2))
        f = clean_frame(p, .045) if b == 0 else 10000
        if f < 1000:
            o.keyframe_insert('scale', frame=max(1, f-3)); o.scale = (.00001,)*3; o.keyframe_insert('scale', frame=f+4)
            # Small flakes peel away from the rotating pad and drift behind its path.
            flake = bpy.data.objects.new('Scrub debris / drifting shell fragment', mesh); scene.collection.objects.link(flake)
            flake.location = p; flake.rotation_euler = o.rotation_euler; flake.scale = (.00001,)*3
            flake.keyframe_insert('scale', frame=f-1); flake.scale = (radius*.35,)*3; flake.keyframe_insert('scale', frame=f+2)
            flake.keyframe_insert('location', frame=f)
            flake.location += Vector((random.uniform(-.09, .09), random.uniform(-.075, -.025), random.uniform(-.08, -.02)))
            flake.keyframe_insert('location', frame=f+45); flake.scale = (.00001,)*3; flake.keyframe_insert('scale', frame=f+48)

cylinder('Propeller / hub barrel', (0, .195, center_z), .107, .22, bronze, (0, 1, 0))
sphere('Propeller / rounded hub cap', (0, .063, center_z), (.101, .055, .101), bronze)
cylinder('Propeller / shaft', (0, .64, center_z), .050, .75, steel, (0, 1, 0))
for j in range(8):
    a = j*math.tau/8
    cylinder('Propeller / hub bolt', (.075*math.cos(a), .043, center_z+.075*math.sin(a)), .007, .008, steel, (0, 1, 0), 6)

# Hull silhouette and rudder provide scale without crowding the two arms.
hull = material('Ship / deep petrol antifouling paint', (.013, .040, .047), .25, .62)
sphere('Ship / stern hull', (0, 1.20, 2.08), (1.7, 1.9, .61), hull, 64)
cube('Ship / shaft support', (0, .73, 1.14), (.08, .22, .70), hull, .025)
cube('Ship / rudder', (-.03, .89, .59), (.07, .40, .76), hull, .035)
sand = material('Distant seabed', (.019, .045, .046), .0, .95)
texture_bump(sand, 8, .65, .07)
# Kept editable, but hidden for an open-water hero background without a horizon.
seabed = cube('Seafloor / optional distant bed', (0, 0, -.41), (200, 200, .1), sand, 0)
seabed.hide_render = True

# Fine suspended particles are sparse and softly out of focus.
dust = material('Water particulates', (.15, .27, .25), .0, .7)
bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=1, radius=1)
dust_mesh = bpy.context.object.data; bpy.data.objects.remove(bpy.context.object, do_unlink=True); dust_mesh.materials.append(dust)
for j in range(170):
    o = bpy.data.objects.new('Water / suspended particulate', dust_mesh); scene.collection.objects.link(o)
    o.location = (random.uniform(-1.1, 1.1), random.uniform(-.8, .6), random.uniform(-.15, 1.35))
    o.scale = (random.uniform(.0005, .0018),)*3
    o.keyframe_insert('location', frame=1); o.location += Vector((.025, -.008, .04)); o.keyframe_insert('location', frame=432)

volume = bpy.data.materials.new('Water / subtle cyan extinction'); volume.use_nodes = True
nodes = volume.node_tree.nodes; nodes.clear(); output = nodes.new('ShaderNodeOutputMaterial')
v = nodes.new('ShaderNodeVolumePrincipled'); v.inputs['Density'].default_value = .025
v.inputs['Color'].default_value = (.17, .43, .48, 1); v.inputs['Anisotropy'].default_value = .25
volume.node_tree.links.new(v.outputs['Volume'], output.inputs['Volume'])
cube('Water volume', (0, 0, 1), (12, 12, 8), volume, 0)


def light(name, loc, target, color, power, size):
    data = bpy.data.lights.new(name, 'AREA'); data.energy = power; data.color = color; data.shape = 'DISK'; data.size = size
    obj = bpy.data.objects.new(name, data); scene.collection.objects.link(obj); obj.location = loc
    obj.rotation_euler = (Vector(target)-obj.location).to_track_quat('-Z', 'Y').to_euler()
    return obj


light('Surface / broad cool key', (.4, -.35, 2.5), (0, .1, .4), (.56, .82, 1), 65, 2.0)
light('Port / cyan rim', (-1.1, .65, 1.05), (0, -.05, .4), (.13, .70, .85), 40, 1.1)
light('Camera / softbox reflection', (.7, -1.0, .9), (0, .05, .4), (1, .86, .68), 20, .85)
light('Bronze / grazing edge', (.75, .50, .55), (0, .12, .45), (.9, .70, .40), 15, .6)
strip = light('Polished bronze / reflected light strip', (.07, -.70, .18), (.04, .13, .42), (1, .89, .69), 8, .36)
strip.data.shape = 'RECTANGLE'; strip.data.size = .045; strip.data.size_y = .48
for x in [-.24, .13]: light('ROV / working beam', (x, .035, .08), (0, .13, .43), (.67, .87, 1), 2, .06)

# Continuous camera choreography: establish, inspect the contact, reveal.
bpy.ops.object.camera_add()
camera = bpy.context.object; camera.name = 'CAMERA / website hero'; scene.camera = camera
camera.data.lens = 43; camera.data.sensor_width = 36
camera.data.clip_start = .01; camera.data.clip_end = 100
camera.data.dof.use_dof = True; camera.data.dof.aperture_fstop = 5.6
focus = bpy.data.objects.new('Camera focus / working plane', None); scene.collection.objects.link(focus); camera.data.dof.focus_object = focus
shots = [(1, (1.35, -2.45, 1.15), (0, .08, .60), 34),
         (84, (.91, -1.40, .86), (0, .07, .49), 45),
         (170, (.64, -1.04, .69), (.01, .07, .44), 48),
         (280, (.55, -.92, .64), (.005, .08, .44), 48),
         (350, (.65, -1.04, .74), (.0, .07, .48), 46),
         (432, (1.20, -2.30, 1.10), (0, .08, .60), 34)]
for f, pos, target, lens in shots:
    camera.location = pos; camera.rotation_euler = (Vector(target)-camera.location).to_track_quat('-Z', 'Y').to_euler()
    camera.keyframe_insert('location', frame=f); camera.keyframe_insert('rotation_euler', frame=f)
    camera.data.lens = lens; camera.data.keyframe_insert('lens', frame=f)
    focus.location = target; focus.keyframe_insert('location', frame=f)

# Every numeric timeline is sampled motion or a deliberate steady transition.
for action in bpy.data.actions:
    # Blender 4.5 actions store curves under slots/channelbags.
    for layer in action.layers:
        for strip in layer.strips:
            for bag in strip.channelbags:
                for curve in bag.fcurves:
                    for key in curve.keyframe_points:
                        key.interpolation = 'LINEAR' if action != camera.animation_data.action else 'BEZIER'

for f, name in [(1, 'ESTABLISH'), (66, 'ANCHOR'), (100, 'CONTACT'), (188, 'SECOND PASS'), (276, 'FINAL PASS'), (352, 'RETRACT'), (394, 'REVEAL')]:
    scene.timeline_markers.new(name, frame=f)
scene['asset_provenance'] = 'Arm meshes and transforms from model/viscous_arm.urdf. Surrounding scene procedurally authored.'
scene['purpose'] = 'Website concept film. Kinematic animation, not validated underwater dynamics.'
scene.frame_set(args.preview or 240)
scene.render.filepath = str(OUT/'frames'/'frame_')
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'viscous_propeller_cleaning.blend'))
if args.preview:
    scene.render.filepath = str(OUT/f'preview_{args.preview:04d}.png')
    bpy.ops.render.render(write_still=True)
if args.render:
    (OUT/'frames').mkdir(exist_ok=True)
    scene.render.filepath = str(OUT/'frames'/'frame_')
    bpy.ops.render.render(animation=True)
print('Saved cinematic scene', flush=True)
