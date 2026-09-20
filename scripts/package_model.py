from pathlib import Path
import hashlib,json,zipfile,xml.etree.ElementTree as ET
R=Path(__file__).resolve().parents[1]
files={R/'README.md',R/'materials.json',R/'collision_settings.json',R/'requirements-lock.txt'}
for folder in ['source','reference','viewer','scripts']:
 files.update(p for p in (R/folder).rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix not in ['.log','.pyc'])
files.update((R/'inspection').glob('*.json'))
files.update((R/'inspection').glob('*.png'))
files.update((R/'model').glob('*.json'))
for name in ['viscous_arm.urdf','viscous_arm.xml']:
 p=R/'model'/name;files.add(p);t=ET.parse(p)
 if name.endswith('urdf'):
  meshes=[p.parent/e.attrib['filename'] for e in t.findall('.//mesh')]
 else:
  meshes=[p.parent/'meshes'/e.attrib['file'] for e in t.findall('./asset/mesh')]
 for m in meshes:
  assert m.is_file(),m
  assert m.name not in ['part_044.stl','part_045.stl','collision_044.stl','collision_045.stl']
 files.update(meshes)
manifest={str(p.relative_to(R)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}
(R/'package_checksums.json').write_text(json.dumps(manifest,indent=2)+'\n');files.add(R/'package_checksums.json')
out=R/'viscous-arm-dry-air-v1.zip'
with zipfile.ZipFile(out,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 for p in sorted(files):z.write(p,'viscous-arm-model/'+str(p.relative_to(R)))
with zipfile.ZipFile(out) as z:
 assert z.testzip() is None
 for name,digest in manifest.items():assert hashlib.sha256(z.read('viscous-arm-model/'+name)).hexdigest()==digest
print(json.dumps({'archive':str(out),'files':len(files),'bytes':out.stat().st_size,'sha256':hashlib.sha256(out.read_bytes()).hexdigest(),'mesh_reference_closure':'PASS','archive_integrity':'PASS'},indent=2))
