"""Generate editable-source title artwork, packed into the Blender compositor."""
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1];out=ROOT/'cinematic/launch/titles';out.mkdir(parents=True,exist_ok=True)
font='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
labels=['','Precision in motion.','A grounded beginning.','J2 — J3','J3 — J4','RS00','Every degree considered.','A considered touch.','Viscous']
for i,label in enumerate(labels):
 im=Image.new('RGBA',(1920,1080));d=ImageDraw.Draw(im)
 if label:
  if i==8:
   d.text((160,480),label,font=ImageFont.truetype(font,82),fill=(237,239,242,255),anchor='lm')
   d.text((160,560),'The art of articulation.',font=ImageFont.truetype(font,25),fill=(170,176,185,255),anchor='lm')
  else:d.text((96,1010),label,font=ImageFont.truetype(font,27),fill=(210,215,223,255),anchor='lm')
 im.save(out/f'{i+1:02d}.png')
