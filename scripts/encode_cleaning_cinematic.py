"""Encode the rendered film and a smaller website version; verify all frames exist."""
from pathlib import Path
import json
import shutil
import subprocess
import argparse
import imageio_ffmpeg

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'cinematic'
parser = argparse.ArgumentParser()
parser.add_argument('--native-rov', action='store_true')
args = parser.parse_args()
if args.native_rov:
    OUT = OUT/'rov_ship'
motion = json.loads((OUT/'motion.json').read_text())
expected = [OUT/'frames'/f'frame_{f:04d}.png' for f in range(1, motion['end']+1)]
missing = [str(p) for p in expected if not p.is_file()]
if missing:
    raise RuntimeError(f'{len(missing)} missing frames, beginning with {missing[0]}')
ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
source = ['-framerate', str(motion['fps']), '-start_number', '1', '-i', str(OUT/'frames/frame_%04d.png')]
color = ['-pix_fmt', 'yuv420p', '-color_primaries', 'bt709', '-color_trc', 'bt709', '-colorspace', 'bt709']


def run(args):
    subprocess.run([ffmpeg, '-hide_banner', '-loglevel', 'warning', '-y', *args], check=True)


run([*source, '-c:v', 'libx264', '-preset', 'slow', '-crf', '18', *color,
     '-movflags', '+faststart', '-an', str(OUT/'viscous_propeller_cleaning.mp4')])
run([*source, '-vf', 'scale=960:540', '-c:v', 'libx264', '-preset', 'slow', '-crf', '24', *color,
     '-movflags', '+faststart', '-an', str(OUT/'viscous_propeller_cleaning_web.mp4')])
poster = OUT/'frames/frame_0140.png' if args.native_rov else expected[0]
run(['-i', str(poster), '-frames:v', '1', '-q:v', '2', '-update', '1', str(OUT/'poster.jpg')])
shutil.copyfile(OUT/('frames/frame_0250.png' if args.native_rov else 'frames/frame_0240.png'), OUT/'detail.png')

report = {'duration_seconds': motion['end']/motion['fps'], 'fps': motion['fps'],
          'rendered_frames': len(expected),
          'arm_source': motion.get('source_urdf', 'model/viscous_arm.urdf'), 'max_ik_residual': motion['max_ik_residual'],
          'physics_scope': motion['note'], 'outputs': {}}
for filename in ['viscous_propeller_cleaning.mp4', 'viscous_propeller_cleaning_web.mp4']:
    path = OUT/filename
    # Decode every frame to catch truncated or corrupt output.
    subprocess.run([ffmpeg, '-v', 'error', '-i', str(path), '-f', 'null', '-'], check=True)
    reader = imageio_ffmpeg.read_frames(str(path)); metadata = next(reader); reader.close()
    report['outputs'][filename] = {'bytes': path.stat().st_size, 'size': metadata['size'],
                                   'fps': metadata['fps'], 'duration': metadata['duration'],
                                   'full_decode_passed': True}
(OUT/'render_report.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
