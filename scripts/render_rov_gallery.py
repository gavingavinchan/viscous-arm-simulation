"""Render selectable website stills from the actual ROV and current ship scene."""
from pathlib import Path
import argparse
import json
import math
import bpy
from mathutils import Vector
from bpy_extras.object_utils import world_to_camera_view

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'cinematic/gallery'
OUT.mkdir(exist_ok=True)
p = argparse.ArgumentParser()
p.add_argument('--draft', action='store_true')
p.add_argument('--ids', nargs='*')
p.add_argument('--force', action='store_true')
args = p.parse_args()
bpy.ops.wm.open_mainfile(filepath=str(ROOT/'cinematic/rov_ship/viscous_rov_ship_cleaning.blend'))
scene = bpy.context.scene
for marker in list(scene.timeline_markers): scene.timeline_markers.remove(marker)
prefs = bpy.context.preferences.addons['cycles'].preferences
prefs.compute_device_type = 'OPTIX'; prefs.get_devices()
for device in prefs.devices: device.use = device.type == 'OPTIX'
scene.cycles.device = 'GPU'
scene.cycles.samples = 24 if args.draft else 64
scene.cycles.use_denoising = True
scene.render.use_persistent_data = True
scene.render.image_settings.file_format = 'PNG'
scene.render.resolution_percentage = 100
scene.render.film_transparent = False
rov = bpy.data.objects['rov_base']
M = rov.matrix_world.copy(); R = M.to_3x3()


def belongs_to_rov(obj):
    while obj:
        if obj == rov: return True
        obj = obj.parent
    return False


rov_meshes = [o for o in scene.objects if o.type == 'MESH' and belongs_to_rov(o)]
environment = [o for o in scene.objects if o.type in {'MESH', 'CURVE'} and not belongs_to_rov(o)
               and o.name != 'Open water volume']
original_visibility = {o.name: o.hide_render for o in environment}
cam_data = bpy.data.cameras.new('Gallery camera')
cam = bpy.data.objects.new('Gallery camera', cam_data); scene.collection.objects.link(cam)
cam_data.clip_start = .01; cam_data.clip_end = 300
cam_data.dof.use_dof = True; cam_data.dof.aperture_fstop = 9
focus = bpy.data.objects.new('Gallery focus', None); scene.collection.objects.link(focus)
cam_data.dof.focus_object = focus
fill_data = bpy.data.lights.new('Gallery / soft camera fill', 'AREA')
fill_data.energy = 28; fill_data.color = (.66, .84, 1); fill_data.shape = 'DISK'; fill_data.size = 1.4
fill = bpy.data.objects.new('Gallery / soft camera fill', fill_data); scene.collection.objects.link(fill)

shots = []


def shot(number, title, group, direction, **kwargs):
    shots.append(dict(id=f'{number:02d}', title=title, group=group, direction=direction,
                      frame=kwargs.pop('frame', 275), **kwargs))


# Genuine camera moves around the model, not alternate crops of video frames.
shot(1, 'Front port quarter', 'At work', (1.15, 1.9, .6))
shot(2, 'Front port, low angle', 'At work', (1.0, 1.8, -.65))
shot(3, 'Port side profile', 'At work', (0, 2.2, .18))
shot(4, 'High port quarter', 'At work', (-.6, 1.8, 1.0))
shot(5, 'Rear port quarter', 'At work', (-1.8, 1.5, .55))
shot(6, 'Low rear view', 'At work', (-2.3, -.15, -.65))
shot(7, 'Rear starboard quarter', 'At work', (-1.5, -1.8, .6))
shot(8, 'Starboard side profile', 'At work', (0, -2.2, .2))
shot(9, 'Low starboard angle', 'At work', (-.4, -1.8, -.7))
shot(10, 'High starboard overview', 'At work', (-.6, -1.3, 1.0))
shot(11, 'Starboard underside', 'At work', (-1, -1.7, -1.35))
shot(12, 'Below the arms', 'At work', (.0, .5, -2.0))
shot(13, 'Hero / room for text on left', 'At work', (.7, 1.8, .55), shift=-.12, fit=False, distance=2.2, lens=37)
shot(14, 'Hero / room for text on right', 'At work', (-1.6, -1.8, .45), shift=.12, fit=False, distance=2.1, lens=37)
shot(15, 'Gripper on the blade edge', 'Details', (.7, 1.4, .2), target=(.535, .16, -.05), distance=.66, lens=57, fit=False)
shot(16, 'Rotating pad / port side', 'Details', (-.8, 1.25, .4), target=(.49, -.075, .03), distance=.70, lens=57, fit=False)
shot(17, 'Rotating pad / starboard side', 'Details', (-.55, -1.2, -.25), target=(.49, -.075, .03), distance=.72, lens=57, fit=False)
shot(18, 'Both arms / low front angle', 'Details', (1.1, 1.3, -.5), target=(.40, .01, .02), distance=1.22, lens=45, fit=False)
shot(19, 'Twin screws and centre rudder', 'Ship context', (0, 0, 0), world_position=(-1.6, -20, -2.6), world_target=(0, 0, 1.7), lens=31, fit=False)
shot(20, 'ROV and complete port propeller', 'Ship context', (0, 0, 0), world_position=(-7.8, -9, -.5), world_target=(-3.8, 0, -.15), lens=38, fit=False)
shot(21, 'Port stern quarter', 'Ship context', (0, 0, 0), world_position=(-12, -16, -3.2), world_target=(-1, 0, .8), lens=32, fit=False)
shot(22, 'Below the large propellers', 'Ship context', (0, 0, 0), world_position=(-3, -16, -7), world_target=(0, 0, .35), lens=33, fit=False)
shot(23, 'ROV / front', 'ROV only', (2.2, 0, .2))
shot(24, 'ROV / front port quarter', 'ROV only', (1.8, 1.8, .7))
shot(25, 'ROV / front starboard quarter', 'ROV only', (1.8, -1.8, .7))
shot(26, 'ROV / port profile', 'ROV only', (.05, 2.5, .12))
shot(27, 'ROV / starboard profile', 'ROV only', (.05, -2.5, .12))
shot(28, 'ROV / rear', 'ROV only', (-2.4, 0, .2))
shot(29, 'ROV / overhead', 'ROV only', (.03, .03, 2.5))
shot(30, 'ROV / underside', 'ROV only', (.15, .05, -2.5))
shot(31, 'ROV / portrait composition', 'ROV only', (1.6, 1.8, .75), size=(1440, 1800), lens=50, zoom=.84)
shot(32, 'ROV / square composition', 'ROV only', (1.8, -1.8, .7), size=(1800, 1800), lens=48, zoom=.85)
shot(33, 'Cleaned bronze / wider reveal', 'At work', (.75, 1.65, .45), frame=460)
shot(34, 'ROV / wide website banner', 'ROV only', (1.5, 1.8, .55), size=(2560, 1080), shift=-.18, fit=False, distance=2.3, lens=40)


def camera_pose(position, target):
    cam.location = position
    cam.rotation_euler = (target-position).to_track_quat('-Z', 'Y').to_euler()
    bpy.context.view_layer.update()


manifest = []
for s in shots:
    full_size = s.get('size', (1920, 1080))
    manifest.append({**s, 'width': full_size[0], 'height': full_size[1],
                     'image': f'full/rov_{s["id"]}.jpg', 'png': f'png/rov_{s["id"]}.png',
                     'thumbnail': f'thumbs/rov_{s["id"]}.jpg'})
(OUT/'shots.json').write_text(json.dumps(manifest, indent=2)+'\n')

for s in shots:
    if args.ids and s['id'] not in args.ids: continue
    folder = OUT/('draft' if args.draft else 'png'); folder.mkdir(exist_ok=True)
    output = folder/f'rov_{s["id"]}.png'
    if output.exists() and not args.force: continue
    scene.frame_set(s['frame']); scene.camera = cam
    product = s['group'] == 'ROV only'
    for ob in environment: ob.hide_render = True if product else original_visibility[ob.name]
    scene.render.resolution_x, scene.render.resolution_y = s.get('size', (1920, 1080))
    if args.draft:
        factor = 960/scene.render.resolution_x
        scene.render.resolution_x = 960; scene.render.resolution_y = round(scene.render.resolution_y*factor)
    cam_data.lens = s.get('lens', 45)
    cam_data.shift_x = s.get('shift', 0)
    target = Vector(s['world_target']) if 'world_target' in s else M@Vector(s.get('target', (.15, 0, -.015)))
    if 'world_position' in s:
        position = Vector(s['world_position'])
    else:
        direction = (R@Vector(s['direction'])).normalized()
        distance = s.get('distance', 1.9)
        if s.get('fit', True):
            deps = bpy.context.evaluated_depsgraph_get()
            bounds = [ob.evaluated_get(deps).matrix_world@Vector(corner) for ob in rov_meshes for corner in ob.bound_box]
            margin = s.get('margin', .095)
            # Find closest whole-vehicle framing, preserving margin and text space.
            distance = .65
            for _ in range(70):
                camera_pose(target+direction*distance, target)
                projected = [world_to_camera_view(scene, cam, v) for v in bounds]
                if all(margin < p.x < 1-margin and margin < p.y < 1-margin and p.z > 0 for p in projected): break
                distance *= 1.045
        position = target+direction*distance*s.get('zoom', 1)
    camera_pose(position, target); focus.location = target
    cam_data.dof.use_dof = s['group'] != 'Ship context'
    fill.hide_render = s['group'] == 'Ship context'
    fill.location = position+R@Vector((0, .35, .7))
    fill.rotation_euler = (target-fill.location).to_track_quat('-Z', 'Y').to_euler()
    scene.render.filepath = str(output)
    bpy.ops.render.render(write_still=True)
    print(f'SHOT {s["id"]}: {s["title"]} / {tuple(position)}', flush=True)
print('Gallery render complete', flush=True)
