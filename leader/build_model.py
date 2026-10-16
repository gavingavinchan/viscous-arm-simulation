"""Build a CAD-derived, kinematics-only Star Arm 102-HD URDF.

All dimensions come from the pinned STEP inventory. This script never opens
hardware. Inertia, actuator ratings, and encoder calibration are deliberately
not inferred from the appearance of a servo or a CAD solid's material.
"""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

ROOT = Path(__file__).resolve().parent
# Right-handed: CAD Y is up; place the base origin on J1's axis at the floor.
R = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
BASE_CAD_MM = np.array([0., 0., -7.5])
MOTOR_COMPONENTS = {0, 1, 2, 3, 14, 15, 16}
# Output shafts/horns move with the child; servo cases and electronics do not.
# J5 is mounted in reverse: its shaft is held by link4, its case by link5.
OUTPUTS = {0: {4, 13, 14}, 1: {4, 13, 14}, 2: {4, 13, 14},
           14: {4, 13}, 16: {3, 13}, 15: {3, 13}}
COMPONENT_OWNER = {4: "link4", 5: "link5", 6: "link6", 7: "link7_left",
                   8: "link7_right", 9: "link5", 10: "link1", 11: "link2",
                   12: "base_link", 13: "link3", 29: "link1"}
# Preview ranges only. J2/J3 interval placement assumes the exported folded
# reference; measured raw-servo limits cannot be applied without zero mapping.
PREVIEW_LIMITS_DEG = [(-110, 110), (0, 180), (-270, 0), (-90, 90), (-65, 65), (-150, 150)]
ALIASES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll"]


def fmt(value):
    return " ".join(f"{float(x):.12g}" for x in np.atleast_1d(value))


def robot_point(point):
    return R @ (np.array(point) - BASE_CAD_MM) * .001


def owner(part):
    component, solid = part["component_id"], part["solid_index"]
    if component in {0, 1, 2}:
        number = component + 1
        return f"link{number + (solid in OUTPUTS[component])}"
    if component == 3:
        return "link4" if solid == 3 else "link5"
    if component == 14:
        return "link1" if solid in OUTPUTS[14] else "base_link"
    if component == 16:
        return "link6" if solid in OUTPUTS[16] else "link5"
    if component == 15:
        return "link7_left" if solid in OUTPUTS[15] else "link6"
    # Rear J1 output horn is grouped inside the base STEP component.
    if component == 12 and solid == 4:
        return "link1"
    if 17 <= component <= 28:
        return "base_link"  # Fixed UC-01 board, connectors and components.
    return COMPONENT_OWNER[component]


def cylinder(part, radius, expected_axis):
    expected = np.array(expected_axis, dtype=float)
    candidates = [c for c in part["cylinders"]
                  if abs(c["radius_mm"] - radius) < 1e-5
                  and abs(np.dot(c["axis"], expected)) > .999999]
    assert candidates, (part["id"], radius, expected_axis)
    return max(candidates, key=lambda c: c["area_mm2"])


def line_at(cyl, coordinate, value, direction):
    point, axis = np.array(cyl["point_mm"]), np.array(cyl["axis"])
    point = point + axis * ((value - point[coordinate]) / axis[coordinate])
    axis *= np.sign(axis @ direction)
    return point, axis


def main():
    inventory = json.loads((ROOT / "inspection/parts.json").read_text())
    parts = inventory["parts"]
    source_hash = hashlib.sha256((ROOT / "source/hd_assembly.STEP").read_bytes()).hexdigest()
    assert inventory["source_sha256"] == source_hash, "Re-extract changed STEP first"
    assert len(parts) == 220 and sum(p["is_solid"] for p in parts) == 219
    lookup = {(p["component_id"], p["solid_index"]): p for p in parts}
    # Each selector uses an analytic cylindrical shaft, not a tessellated mesh.
    selectors = [(14, 4, 3., [0, -1, 0], 1, 45.496),
                 (0, 4, 3., [-1, 0, 0], 0, 0.),
                 (1, 4, 3., [-1, 0, 0], 0, 0.),
                 (2, 4, 3., [-1, 0, 0], 0, 0.),
                 (3, 3, 3., [0, -1, 0], 1, None),
                 (16, 3, 3., [0, 0, 1], 2, -86.696)]
    joints, evidence, pivots = [], [], {"base_link": np.zeros(3)}
    for number, (component, solid, radius, direction, coord, value) in enumerate(selectors, 1):
        part = lookup[component, solid]
        surface = cylinder(part, radius, direction)
        if value is None:
            value = evidence[3]["pivot_cad_mm"][1]  # J4 height: origin chosen along J5 axis.
        point, axis = line_at(surface, coord, value, direction)
        link, parent = f"link{number}", "base_link" if number == 1 else f"link{number-1}"
        pivots[link] = robot_point(point)
        lower, upper = np.deg2rad(PREVIEW_LIMITS_DEG[number-1])
        joints.append({"name": f"joint{number}", "type": "revolute", "parent": parent,
                       "child": link, "xyz": (pivots[link] - pivots[parent]).tolist(),
                       "axis": (R @ axis).tolist(), "lower": lower, "upper": upper})
        evidence.append({"joint": f"joint{number}", "servo_id": number-1,
                         "motor_alias": ALIASES[number-1], "shaft_part_id": part["id"],
                         "shaft_component": component, "shaft_solid": solid,
                         "cylinder_radius_mm": radius, "pivot_cad_mm": point.tolist(),
                         "axis_cad": axis.tolist(), "origin_coordinate": coord,
                         "origin_coordinate_value_mm": value})
    # Left ring is driven by servo 6. Its shaft is the authoritative axis;
    # the imported ring bore is displaced 0.15 mm from it in the source CAD.
    left = cylinder(lookup[15, 3], 3., [0, 0, 1])
    right = cylinder(lookup[8, 0], 9.75, [0, 0, 1])
    for side, surface, multiplier in [("left", left, 1), ("right", right, -1)]:
        point, axis = line_at(surface, 2, -143.939056191365, np.array([0., 0., 1.]))
        link = f"link7_{side}"
        pivots[link] = robot_point(point)
        joint = {"name": f"joint7_{side}", "type": "revolute", "parent": "link6",
                 "child": link, "xyz": (pivots[link] - pivots["link6"]).tolist(),
                 "axis": (R @ axis).tolist(), "lower": 0. if side == "left" else -np.pi/2,
                 "upper": np.pi/2 if side == "left" else 0.}
        if side == "right":
            joint["mimic"] = {"joint": "joint7_left", "multiplier": multiplier, "offset": 0.}
        joints.append(joint)
        evidence.append({"joint": joint["name"], "pivot_cad_mm": point.tolist(),
                         "axis_cad": axis.tolist(), "shaft_part_id": lookup[15, 3]["id"] if side == "left" else lookup[8, 0]["id"],
                         "cylinder_radius_mm": 3. if side == "left" else 9.75})
    # Handle reference: midpoint of the axial extent of the 16-mm grip cylinder.
    grip = cylinder(lookup[6, 1], 16., [0., -.984807753, .173648178])
    z_cad = np.array(grip["axis"])
    if z_cad[1] > 0:
        z_cad *= -1
    t = np.mean(grip["axial_bounds_mm"])
    grip_point = np.array(grip["point_mm"]) + np.array(grip["axis"]) * t
    x_cad = np.array([1., 0., 0.])
    y_cad = np.cross(z_cad, x_cad)
    tool_rotation = R @ np.column_stack([x_cad, y_cad, z_cad])
    pivots["tool0"] = robot_point(grip_point)
    joints.append({"name": "handle_reference", "type": "fixed", "parent": "link6",
                   "child": "tool0", "xyz": (pivots["tool0"] - pivots["link6"]).tolist(),
                   "rpy": Rotation.from_matrix(tool_rotation).as_euler("xyz").tolist()})
    output = ROOT / "model"
    (output / "meshes").mkdir(parents=True, exist_ok=True)
    robot = ET.Element("robot", name="star_arm_102_hd")
    robot.append(ET.Comment("CAD-derived kinematic model. Preview limits only; no hardware zero calibration or identified dynamics."))
    links = {name: ET.SubElement(robot, "link", name=name) for name in pivots}
    for joint in joints:
        node = ET.SubElement(robot, "joint", name=joint["name"], type=joint["type"])
        ET.SubElement(node, "parent", link=joint["parent"])
        ET.SubElement(node, "child", link=joint["child"])
        ET.SubElement(node, "origin", xyz=fmt(joint["xyz"]), rpy=fmt(joint.get("rpy", [0, 0, 0])))
        if joint["type"] == "revolute":
            ET.SubElement(node, "axis", xyz=fmt(joint["axis"]))
            # URDF requires effort/velocity fields. These are chosen preview
            # metadata, not sustainable actuator ratings or a control policy.
            ET.SubElement(node, "limit", lower=fmt(joint["lower"]), upper=fmt(joint["upper"]), effort="1", velocity="1")
        if "mimic" in joint:
            ET.SubElement(node, "mimic", **{k: str(v) for k, v in joint["mimic"].items()})
    part_map = []
    for part in parts:
        name = owner(part)
        mesh = trimesh.load_mesh(ROOT / f"inspection/part_{part['id']:03d}.stl", process=False)
        mesh.vertices = (mesh.vertices - BASE_CAD_MM) @ R.T * .001 - pivots[name]
        filename = f"meshes/part_{part['id']:03d}.stl"
        mesh.export(output / filename)
        visual = ET.SubElement(links[name], "visual", name=f"part_{part['id']:03d}")
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "mesh", filename=filename)
        material = ET.SubElement(visual, "material", name=f"part_{part['id']:03d}")
        # Use the CAD component color if present. Electronics/servo housings
        # are dark; structural plastics and gears remain distinguishable.
        color = part.get("color")
        if color is None:
            color = [.20, .23, .26, 1.] if part["component_id"] in MOTOR_COMPONENTS else [.82, .83, .86, 1.]
        ET.SubElement(material, "color", rgba=fmt(color))
        contact = part["is_solid"] and part["component_id"] < 17 and part["volume_mm3"] > 500
        if contact:
            collision = ET.SubElement(links[name], "collision", name=f"part_{part['id']:03d}")
            ET.SubElement(ET.SubElement(collision, "geometry"), "mesh", filename=filename)
        part_map.append({"id": part["id"], "component_id": part["component_id"],
                         "solid_index": part["solid_index"], "link": name,
                         "mesh": filename, "collision": contact,
                         "mesh_sha256": hashlib.sha256((output / filename).read_bytes()).hexdigest()})
    ET.indent(robot, space="  ")
    ET.ElementTree(robot).write(output / "star_arm_102_hd.urdf", encoding="utf-8", xml_declaration=True)
    manifest = {
        "name": "Star Arm 102-HD", "scope": "offline geometry and kinematics",
        "source_sha256": source_hash, "coordinate_rotation": R.tolist(),
        "base_origin_cad_mm": BASE_CAD_MM.tolist(), "units": "metres and radians",
        "zero_convention": "Exported STEP pose; physical encoder signs and offsets unverified",
        "joint_evidence": evidence, "joints": joints,
        "link_origins_world_m": {name: value.tolist() for name, value in pivots.items()},
        "tool0": {"point_cad_mm": grip_point.tolist(), "rotation_cad": np.column_stack([x_cad, y_cad, z_cad]).tolist(),
                  "definition": "Midpoint of 16-mm-radius grip cylinder axial extent; +Z down handle, +X along CAD +X"},
        "parts": part_map,
        "assumptions": [
            "Standard manufacturer handle and finger rings, confirmed by Gavin.",
            "All 219 solids and one terminal-block surface occurrence are retained exactly once.",
            "J5's shaft is attached to link4; its case and downstream wrist are attached to link5.",
            "Equal opposing ring-gear rotation is a kinematic coupling; backlash and compliance are not modeled.",
            "Left ring bore is offset 0.15 mm laterally from its servo shaft in the supplied CAD; assembled geometry is preserved, joint axis follows servo shaft.",
            "CAD axis signs follow the earlier reference convention; they are not measured servo signs.",
            "Preview limits use nominal manufacturer travel; J2/J3 interval placement assumes folded CAD zero. These are not hardware soft limits.",
            "Effort=1 Nm and velocity=1 rad/s are required URDF preview metadata, not identified actuator ratings.",
            "No inertial properties are supplied: LD/HD masses and inertias need identification before dynamics or gravity compensation.",
            "Collision elements use selected CAD surface meshes; physics engines may convexify them. No contact-dynamics validation is claimed.",
            "Cables and the HD button assembly are absent from this STEP and are not invented.",
        ],
    }
    (output / "model_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Built {len(links)} links, 6 arm joints, coupled rings, {len(parts)} part occurrences")


if __name__ == "__main__":
    main()
