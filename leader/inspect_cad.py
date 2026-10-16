"""Inventory the pinned Star Arm 102-HD STEP assembly, preserving placements.

Run with ../.venv/bin/python inspect_cad.py from this directory, or from the
repository root with .venv/bin/python leader/inspect_cad.py.
"""
import hashlib
import json
from pathlib import Path

import cadquery as cq
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepGProp import BRepGProp
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.GProp import GProp_GProps

ROOT = Path(__file__).resolve().parent


def main():
    source = ROOT / "source/hd_assembly.STEP"
    intake = json.loads((ROOT / "source/intake.json").read_text())
    expected = next(f["sha256"] for f in intake["files"] if f["file"] == source.name)
    assert hashlib.sha256(source.read_bytes()).hexdigest() == expected
    assembly = cq.Assembly.importStep(str(source))
    output = ROOT / "inspection"
    output.mkdir(exist_ok=True)
    rows = []
    for component, (shape, name, loc, color) in enumerate(assembly):
        solids = shape.Solids()
        # The UC-01 terminal block is a surface model, not a solid. Preserve it
        # for display, without inventing volume or mass properties.
        for solid_index, original in enumerate(solids or [shape]):
            solid = original.moved(loc)
            index = len(rows)
            box = solid.BoundingBox()
            props = GProp_GProps()
            BRepGProp.VolumeProperties_s(solid.wrapped, props)
            center, inertia = props.CentreOfMass(), props.MatrixOfInertia()
            cylinders = []
            for face in solid.Faces():
                surface = BRepAdaptor_Surface(face.wrapped)
                if surface.GetType() != GeomAbs_Cylinder:
                    continue
                cylinder = surface.Cylinder()
                if cylinder.Radius() < 1.0:
                    continue
                axis = cylinder.Axis()
                p, d = axis.Location(), axis.Direction()
                face_center = face.Center()
                projections = [
                    (v.Center().x - p.X()) * d.X()
                    + (v.Center().y - p.Y()) * d.Y()
                    + (v.Center().z - p.Z()) * d.Z()
                    for v in face.Vertices()
                ]
                cylinders.append({
                    "radius_mm": cylinder.Radius(),
                    "point_mm": [p.X(), p.Y(), p.Z()],
                    "axis": [d.X(), d.Y(), d.Z()],
                    "area_mm2": face.Area(),
                    "face_center_mm": list(face_center.toTuple()),
                    "axial_bounds_mm": [min(projections), max(projections)] if projections else None,
                })
            row = {
                "id": index, "component_id": component,
                "name": name, "solid_index": solid_index,
                "is_solid": bool(solids),
                "volume_mm3": props.Mass() if solids else None,
                "center_mm": [center.X(), center.Y(), center.Z()],
                "bbox_mm": [box.xmin, box.ymin, box.zmin, box.xmax, box.ymax, box.zmax],
                "inertia_unit_density_mm5": [
                    [inertia.Value(i, j) for j in range(1, 4)] for i in range(1, 4)
                ] if solids else None,
                "color": color.toTuple() if color else None,
                "cylinders": cylinders,
            }
            rows.append(row)
            solid.exportStl(str(output / f"part_{index:03d}.stl"),
                            tolerance=.05, angularTolerance=.2, relative=False)
            print(index, name, solid_index,
                  [round(x, 3) for x in row["center_mm"]], flush=True)
    (output / "parts.json").write_text(json.dumps({
        "source_sha256": expected, "units": "mm", "parts": rows,
    }, indent=2) + "\n")
    print("Parts:", len(rows), "Solids:", sum(p["is_solid"] for p in rows))


if __name__ == "__main__":
    main()
