from pathlib import Path
import json, cadquery as cq
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
ROOT=Path(__file__).resolve().parents[1]
p=ROOT/'source/2026-09-18/maker arm cnc装配.step'
a=cq.Assembly.importStep(str(p))
rows=[]
flat=[]
for component,(shape,name,loc,color) in enumerate(a):
 for solid_index,solid in enumerate(shape.Solids()):
  flat.append((solid,name+f"/solid_{solid_index}",loc,color,component))
for idx,(shape,name,loc,color,component) in enumerate(flat):
 s=shape.moved(loc); b=s.BoundingBox(); g=GProp_GProps();BRepGProp.VolumeProperties_s(s.wrapped,g)
 m=g.MatrixOfInertia();c=g.CentreOfMass(); cyl=[]
 for f in s.Faces():
  ad=BRepAdaptor_Surface(f.wrapped)
  if ad.GetType()==GeomAbs_Cylinder:
   cy=ad.Cylinder(); ax=cy.Axis();v=ax.Direction();q=ax.Location()
   if cy.Radius()>10:cyl.append({'r':round(cy.Radius(),5),'p':[q.X(),q.Y(),q.Z()],'axis':[v.X(),v.Y(),v.Z()],'area':f.Area()})
 row={'id':idx,'component_id':component,'name':name,'volume_mm3':g.Mass(),'center_mm':[c.X(),c.Y(),c.Z()],'bbox_mm':[b.xmin,b.ymin,b.zmin,b.xmax,b.ymax,b.zmax],'inertia_unit_density_mm5':[[m.Value(i,j) for j in range(1,4)] for i in range(1,4)],'cylinders':cyl,'color':color.toTuple() if color else None,'solid_count':len(s.Solids())}
 rows.append(row)
 out=ROOT/'inspection'/f'part_{idx:03d}.stl';s.exportStl(str(out),tolerance=.08,angularTolerance=.25,relative=False)
 print(idx,name,'vol',round(g.Mass(),2),'center',[round(x,2) for x in row['center_mm']],'bbox',[round(x,2) for x in row['bbox_mm']])
(ROOT/'inspection/parts.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2))
print('PARTS',len(rows))
