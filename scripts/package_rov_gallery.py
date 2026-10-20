"""Package Blender still renders for quick browser review and full-size downloads."""
from pathlib import Path
import json
import math
import zipfile
from PIL import Image, ImageOps, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'cinematic/gallery'
shots = json.loads((OUT/'shots.json').read_text())
for name in ['full', 'thumbs']: (OUT/name).mkdir(exist_ok=True)
for shot in shots:
    source = OUT/shot['png']
    if not source.is_file(): raise RuntimeError(f'Missing render {source}')
    with Image.open(source) as original:
        original.load()
        assert original.size == (shot['width'], shot['height']), (shot['id'], original.size)
        rgb = original.convert('RGB')
        rgb.save(OUT/shot['image'], quality=94, subsampling=0, optimize=True)
        thumb = ImageOps.contain(rgb, (640, 400))
        thumb.save(OUT/shot['thumbnail'], quality=86, optimize=True)
    shot['jpg_bytes'] = (OUT/shot['image']).stat().st_size
    shot['png_bytes'] = source.stat().st_size
(OUT/'shots.json').write_text(json.dumps(shots, indent=2)+'\n')

cols, width, height = 4, 480, 302
sheet = Image.new('RGB', (cols*width, math.ceil(len(shots)/cols)*height), '#06191e')
draw = ImageDraw.Draw(sheet)
font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 16)
for i, shot in enumerate(shots):
    with Image.open(OUT/shot['image']) as im: tile = ImageOps.contain(im, (width, 270))
    x, y = i % cols*width, i//cols*height
    sheet.paste(tile, (x+(width-tile.width)//2, y+(270-tile.height)//2))
    draw.text((x+9, y+278), shot['id']+'  '+shot['title'], fill='#e5f0ef', font=font)
sheet.save(OUT/'contact_sheet.jpg', quality=93, optimize=True)
notes = '\n'.join(f"{s['id']} — {s['title']} — {s['group']} — {s['width']}x{s['height']}" for s in shots)+'\n'
(OUT/'shot_list.txt').write_text(notes)
with zipfile.ZipFile(OUT/'viscous_rov_stills_jpg.zip', 'w', compression=zipfile.ZIP_STORED) as archive:
    for shot in shots: archive.write(OUT/shot['image'], arcname=Path(shot['image']).name)
    archive.write(OUT/'shot_list.txt', arcname='shot_list.txt')
report = {'shot_count': len(shots), 'groups': {group: sum(s['group']==group for s in shots) for group in sorted({s['group'] for s in shots})},
          'png_and_jpg_dimensions_verified': True, 'jpeg_zip_bytes': (OUT/'viscous_rov_stills_jpg.zip').stat().st_size,
          'source_scene': 'cinematic/rov_ship/viscous_rov_ship_cleaning.blend', 'video_changed': False}
(OUT/'gallery_report.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
