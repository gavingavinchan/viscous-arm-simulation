"""Check emitted URDF geometry and FK independently of its builder.

Uses raw CAD cylinders + spatial Rodrigues rotations, a separate URDF FK, and
MuJoCo's URDF importer. The temporary MuJoCo test copy has synthetic inertias
solely so that the engine accepts a kinematic model; no dynamics are tested.
"""
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import trimesh

from kinematics import fk, NAMES

ROOT = Path(__file__).resolve().parent
URDF = ROOT / "model/star_arm_102_hd.urdf"


def rotation(axis, angle):
    x, y, z = axis
    cross = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + np.sin(angle) * cross + (1-np.cos(angle)) * cross @ cross


def axis_motion(point, axis, angle):
    transform = np.eye(4)
    transform[:3, :3] = rotation(axis, angle)
    transform[:3, 3] = point - transform[:3, :3] @ point
    return transform


def main():
    report_path = ROOT / "inspection/validation_report.json"
    report_path.unlink(missing_ok=True)
    model = json.loads((ROOT / "model/model_manifest.json").read_text())
    inventory = json.loads((ROOT / "inspection/parts.json").read_text())
    parts = inventory["parts"]
    root = ET.parse(URDF).getroot()
    link_names = [l.get("name") for l in root.findall("link")]
    joints = {j.get("name"): j for j in root.findall("joint")}
    assert len(link_names) == len(set(link_names)) == 10
    assert len(joints) == 9
    assert all(joints[f"joint{i}"].get("type") == "revolute" for i in range(1, 7))
    assert joints["joint7_right"].find("mimic").attrib == {"joint": "joint7_left", "multiplier": "-1", "offset": "0.0"}
    children = [j.find("child").get("link") for j in joints.values()]
    assert len(set(children)) == len(children)
    assert set(link_names) - set(children) == {"base_link"}
    zero_frames = fk(np.zeros(7))
    assert set(zero_frames) == set(link_names)
    assert len(model["parts"]) == 220 == len(parts)
    assert sorted(p["id"] for p in model["parts"]) == list(range(220))
    assert sum(p["is_solid"] for p in parts) == 219
    # Independent coordinate convention, specified in the engineering notes.
    R = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
    base = np.array([0., 0., -7.5])
    assert np.linalg.det(R) == 1.
    source_hash = hashlib.sha256((ROOT / "source/hd_assembly.STEP").read_bytes()).hexdigest()
    assert source_hash == inventory["source_sha256"] == model["source_sha256"]
    for record in json.loads((ROOT / "source/intake.json").read_text())["files"]:
        assert hashlib.sha256((ROOT / "source" / record["file"]).read_bytes()).hexdigest() == record["sha256"]
    asset_error, triangles, surface_parts = 0., 0, 0
    assignments = {p["id"]: p for p in model["parts"]}
    for link in root.findall("link"):
        for element in list(link.findall("visual")) + list(link.findall("collision")):
            mesh_node = element.find("geometry/mesh")
            assert mesh_node is not None
            path = (URDF.parent / mesh_node.get("filename")).resolve()
            assert path.is_relative_to(URDF.parent.resolve()) and path.is_file()
            assert mesh_node.get("scale") is None
        for element in link.findall("visual"):
            index = int(element.get("name").split("_")[-1])
            assignment = assignments[index]
            assert assignment["link"] == link.get("name")
            path = URDF.parent / element.find("geometry/mesh").get("filename")
            assert hashlib.sha256(path.read_bytes()).hexdigest() == assignment["mesh_sha256"]
            mesh = trimesh.load_mesh(path, process=False)
            raw = trimesh.load_mesh(ROOT / f"inspection/part_{index:03d}.stl", process=False)
            assert np.isfinite(mesh.vertices).all() and len(mesh.faces) > 0
            assert np.array_equal(mesh.faces, raw.faces)
            world = trimesh.transform_points(mesh.vertices, zero_frames[link.get("name")])
            expected = (raw.vertices - base) @ R.T * .001
            asset_error = max(asset_error, float(np.max(np.linalg.norm(world - expected, axis=1))))
            triangles += len(mesh.faces)
            surface_parts += not parts[index]["is_solid"]
    assert asset_error < 5e-8, asset_error
    assert surface_parts == 1
    # Verify axis lines using a SECOND CAD component at each mechanical joint.
    corroborating = {"joint1": (70, 37.5), "joint2": (66, 5.), "joint3": (82, 5.),
                     "joint4": (58, 12.5), "joint5": (58, 11.), "joint6": (61, 12.),
                     "joint7_right": (61, 1.6)}
    axis_checks = []
    for evidence in model["joint_evidence"]:
        name = evidence["joint"]
        transform = zero_frames[joints[name].find("child").get("link")]
        axis = np.fromstring(joints[name].find("axis").get("xyz"), sep=" ")
        assert abs(np.linalg.norm(axis)-1) < 1e-12
        point = evidence["pivot_cad_mm"]
        expected_axis = R @ np.array(evidence["axis_cad"])
        assert np.linalg.norm(axis - expected_axis) < 1e-10
        assert np.linalg.norm(transform[:3, 3] - R @ (np.array(point)-base) * .001) < 1e-10
        if name not in corroborating:
            continue
        part_id, radius = corroborating[name]
        candidates = [c for c in parts[part_id]["cylinders"]
                      if abs(c["radius_mm"]-radius) < 1e-5
                      and abs(np.dot(c["axis"], evidence["axis_cad"])) > .999999]
        distance = min(np.linalg.norm(np.cross(np.array(c["point_mm"])-point, evidence["axis_cad"])) for c in candidates)
        assert distance < 1e-7, (name, distance)
        axis_checks.append({"joint": name, "corroborating_part": part_id,
                            "radius_mm": radius, "axis_line_error_mm": float(distance)})
    # Compile a fresh copy of the actual emitted URDF, not a separate MJCF
    # generated from the same manifest. Synthetic inertias are test-only.
    test = copy.deepcopy(root)
    extension = ET.SubElement(test, "mujoco")
    ET.SubElement(extension, "compiler", fusestatic="false", discardvisual="false")
    for link in test.findall("link"):
        for tag in ("visual", "collision"):
            for element in link.findall(tag):
                link.remove(element)
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "mass", value="1")
        ET.SubElement(inertial, "inertia", ixx="0.01", iyy="0.01", izz="0.01", ixy="0", ixz="0", iyz="0")
    engine = mujoco.MjModel.from_xml_string(ET.tostring(test, encoding="unicode"))
    state = mujoco.MjData(engine)
    engine_joint_addresses = {name: int(engine.jnt_qposadr[mujoco.mj_name2id(engine, mujoco.mjtObj.mjOBJ_JOINT, name)])
                              for name, node in joints.items() if node.get("type") == "revolute"}
    evidence = {e["joint"]: e for e in model["joint_evidence"]}
    rng = np.random.default_rng(102)
    max_fk_error, max_rotation_error, max_engine_error = 0., 0., 0.
    sampled_ranks = []
    samples = [np.zeros(7)] + [rng.uniform([-.8, .2, -2., -.8, -.8, -1., 0.], [.8, 2., -.1, .8, .8, 1., .8]) for _ in range(100)]
    for q in samples:
        values = dict(zip(NAMES, q))
        values["joint7_right"] = -values["joint7_left"]
        frames = fk(q)
        for name, address in engine_joint_addresses.items():
            state.qpos[address] = values[name]
        mujoco.mj_forward(engine, state)
        # Spatial screw rotations about the RAW CAD axes, independent of
        # URDF parent/child origins, RPY construction and link frames.
        spatial = np.eye(4)
        expected_frames = {"base_link": np.eye(4)}
        for number in range(1, 7):
            e = evidence[f"joint{number}"]
            p = R @ (np.array(e["pivot_cad_mm"])-base) * .001
            axis = R @ np.array(e["axis_cad"])
            spatial = spatial @ axis_motion(p, axis, q[number-1])
            expected_frames[f"link{number}"] = spatial @ zero_frames[f"link{number}"]
        for side, sign in [("left", 1), ("right", -1)]:
            e = evidence[f"joint7_{side}"]
            p = R @ (np.array(e["pivot_cad_mm"])-base) * .001
            a = R @ np.array(e["axis_cad"])
            expected_frames[f"link7_{side}"] = spatial @ axis_motion(p, a, q[6]*sign) @ zero_frames[f"link7_{side}"]
        expected_frames["tool0"] = spatial @ zero_frames["tool0"]
        for name, actual in frames.items():
            expected = expected_frames[name]
            max_fk_error = max(max_fk_error, float(np.linalg.norm(actual[:3, 3]-expected[:3, 3])))
            max_rotation_error = max(max_rotation_error, float(np.max(np.abs(actual[:3, :3]-expected[:3, :3]))))
            body_id = mujoco.mj_name2id(engine, mujoco.mjtObj.mjOBJ_BODY, name)
            assert body_id >= 0
            max_engine_error = max(max_engine_error, float(np.max(np.abs(state.xpos[body_id]-actual[:3, 3]))),
                                   float(np.max(np.abs(state.xmat[body_id].reshape(3, 3)-actual[:3, :3]))))
        # A six-axis wrist must affect a six-dimensional hand pose in ordinary
        # configurations; a permanently fixed J5 would fail this check.
        if np.any(q[:6]):
            columns = []
            for number in range(1, 7):
                frame = frames[f"link{number}"]
                axis = frame[:3, :3] @ np.fromstring(joints[f"joint{number}"].find("axis").get("xyz"), sep=" ")
                columns.append(np.r_[np.cross(axis, frames["tool0"][:3, 3]-frame[:3, 3]), axis])
            sampled_ranks.append(int(np.linalg.matrix_rank(np.array(columns).T, tol=1e-9)))
    assert max_fk_error < 1e-9 and max_rotation_error < 1e-9 and max_engine_error < 1e-9
    assert min(sampled_ranks) == 6
    # Explicit J5 regression: upstream stays fixed, downstream rotates about
    # the true CAD axis; hand position and orientation both respond.
    q = np.zeros(7)
    q[4] = np.deg2rad(30.)
    turned = fk(q)
    assert np.allclose(turned["link4"], zero_frames["link4"])
    j5_shift = float(np.linalg.norm(turned["tool0"][:3, 3]-zero_frames["tool0"][:3, 3]))
    assert j5_shift > .02
    assert not np.allclose(turned["tool0"][:3, :3], zero_frames["tool0"][:3, :3])
    # Coupled rings must open apart, and do not move the fixed handle frame.
    q = np.zeros(7)
    q[6] = np.deg2rad(25.)
    opened = fk(q)
    left_point, right_point = np.array(parts[62]["center_mm"]), np.array(parts[63]["center_mm"])
    distances = []
    for frames in [zero_frames, opened]:
        centers = []
        for name, p in [("link7_left", left_point), ("link7_right", right_point)]:
            rest_world = R @ (p-base) * .001
            local = np.linalg.inv(zero_frames[name]) @ np.r_[rest_world, 1.]
            centers.append((frames[name] @ local)[:3])
        distances.append(float(np.linalg.norm(centers[0]-centers[1])))
    assert distances[1] > distances[0] + .02
    assert np.allclose(opened["tool0"], zero_frames["tool0"])
    report = {
        "passed": True, "urdf_sha256": hashlib.sha256(URDF.read_bytes()).hexdigest(),
        "manifest_sha256": hashlib.sha256((ROOT / "model/model_manifest.json").read_bytes()).hexdigest(),
        "source_sha256": source_hash, "links": 10, "arm_dof": 6, "independent_dof": 7,
        "solid_occurrences": 219, "surface_occurrences": 1, "triangles": triangles,
        "max_mesh_cad_zero_error_m": asset_error, "corroborating_axes": axis_checks,
        "configurations_checked": len(samples), "max_spatial_fk_position_error_m": max_fk_error,
        "max_spatial_fk_rotation_matrix_error": max_rotation_error,
        "max_mujoco_urdf_fk_error": max_engine_error, "minimum_sampled_hand_jacobian_rank": min(sampled_ranks),
        "j5_30deg_hand_displacement_m": j5_shift, "ring_com_separation_m_at_0_and_25deg": distances,
        "limits": "Preview only; encoder zeros, signs and physical bounds are unverified",
        "dynamics": "Not tested; temporary MuJoCo copy uses synthetic inertias for FK only",
    }
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
