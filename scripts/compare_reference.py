from pathlib import Path
import json, numpy as np, trimesh
ROOT=Path(__file__).resolve().parents[1]
parts=json.loads((ROOT/'inspection/parts.json').read_text())
ref=[]
for p in sorted((ROOT/'reference/maker-urdf/meshes').glob('*.stl')):
 m=trimesh.load_mesh(p)
 ref.append({'name':p.name,'volume':abs(m.volume),'extents':sorted(m.extents.tolist()),'center':m.center_mass.tolist()})
for p in parts:
 b=np.array(p['bbox_mm']);ext=sorted((b[3:]-b[:3]).tolist());vol=p['volume_mm3']
 matches=sorted(ref,key=lambda r:abs(r['volume']-vol)/max(vol,1))[:3]
 print(p['id'],p['name'].split('/')[-2], round(vol,1),'ref',[(r['name'],round(r['volume'],1),round(abs(r['volume']-vol)/max(vol,1),4)) for r in matches])
(ROOT/'inspection/reference_meshes.json').write_text(json.dumps(ref,indent=2))
