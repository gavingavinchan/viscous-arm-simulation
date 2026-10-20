"""OptiX numbered PNG renderer. Selected reviews never replace master frames."""
from pathlib import Path
import argparse,json,time
import bpy
from build_launch_cinematic import gpu
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--finish',choices=['silver','black'],default='silver');p.add_argument('--frames',nargs='+',type=int);p.add_argument('--review',action='store_true');p.add_argument('--width',type=int,default=1920);p.add_argument('--samples',type=int,default=48);p.add_argument('--force',action='store_true');a=p.parse_args()
out=ROOT/'cinematic/launch'/a.finish;motion=json.loads((out/'motion.json').read_text());bpy.ops.wm.open_mainfile(filepath=str(out/'viscous_launch.blend'));s=bpy.context.scene;gpu(s);s.cycles.samples=a.samples;s.cycles.denoising_use_gpu=True;s.render.resolution_x=a.width;s.render.resolution_y=round(a.width*9/16);s.render.use_persistent_data=True
review=a.review or a.frames;folder=out/('review' if review else 'frames');folder.mkdir(exist_ok=True)
frames=a.frames or ([x['review'] for x in motion['shots']]+[116,152,188,625,768] if a.review else range(1,motion['end']+1))
signature={'width':a.width,'samples':a.samples,'gpu_denoise':True,'blend_mtime_ns':(out/'viscous_launch.blend').stat().st_mtime_ns}
if not review:
    setting=folder/'settings.json'
    if setting.exists() and json.loads(setting.read_text())!=signature and not a.force:raise RuntimeError('Render settings/scene changed; use a clean frames folder or --force.')
    if a.force:
        for old in folder.glob('frame_*.png'):old.unlink()
    setting.write_text(json.dumps(signature,indent=2))
for f in frames:
    path=folder/f'frame_{f:04d}.png'
    if path.exists() and not review and not a.force:
        with path.open('rb') as check:
            check.seek(max(0,path.stat().st_size-12));complete=check.read()==bytes.fromhex('0000000049454e44ae426082')
        if complete:continue
    t=time.monotonic();s.frame_set(f);s.render.filepath=str(path.with_suffix('.part.png'));bpy.ops.render.render(write_still=True);path.with_suffix('.part.png').replace(path);print(f'COMPLETE {a.finish} {f} {time.monotonic()-t:.2f}s',flush=True)
