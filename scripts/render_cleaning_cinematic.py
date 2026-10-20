"""Render the saved scene with the available GPU, resumably."""
from pathlib import Path
import argparse
import bpy

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'cinematic'
p = argparse.ArgumentParser()
p.add_argument('--frames', nargs='*', type=int)
p.add_argument('--width', type=int, default=1280)
p.add_argument('--samples', type=int, default=32)
p.add_argument('--native-rov', action='store_true', help='Render the corrected complete ROV / large ship scene')
args = p.parse_args()
if args.native_rov:
    OUT = OUT/'rov_ship'
filename = 'viscous_rov_ship_cleaning.blend' if args.native_rov else 'viscous_propeller_cleaning.blend'
bpy.ops.wm.open_mainfile(filepath=str(OUT/filename))
scene = bpy.context.scene
prefs = bpy.context.preferences.addons['cycles'].preferences
prefs.compute_device_type = 'OPTIX'; prefs.get_devices()
for d in prefs.devices: d.use = d.type == 'OPTIX'
scene.cycles.device = 'GPU' if any(d.type == 'OPTIX' for d in prefs.devices) else 'CPU'
scene.cycles.samples = args.samples
scene.render.use_persistent_data = True
scene.render.resolution_x = args.width
scene.render.resolution_y = round(args.width*9/16)
frames = args.frames or range(scene.frame_start, scene.frame_end+1)
(OUT/'frames').mkdir(exist_ok=True)
for f in frames:
    output = OUT/(f'preview_{f:04d}.png' if args.frames else f'frames/frame_{f:04d}.png')
    if output.exists() and not args.frames:
        continue
    scene.frame_set(f); scene.render.filepath = str(output)
    bpy.ops.render.render(write_still=True)
    print(f'COMPLETE {f}', flush=True)
