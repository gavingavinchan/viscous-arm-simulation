"""Offline FK for the emitted leader URDF; radians, no encoder conversion."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

URDF = Path(__file__).resolve().parent / "model/star_arm_102_hd.urdf"
NAMES = [f"joint{i}" for i in range(1, 7)] + ["joint7_left"]


def vector(node, attribute, default="0 0 0"):
    return np.fromstring(node.get(attribute, default), sep=" ") if node is not None else np.fromstring(default, sep=" ")


def fk(q, path=URDF):
    """Return world transforms for all links for six arm angles + ring opening.

    q is a length-7 array or a dictionary keyed by independent URDF joint name.
    The opposite ring follows its URDF mimic relation. No range clamping occurs:
    the supplied limits are preview metadata, not calibrated hardware bounds.
    """
    if isinstance(q, dict):
        if set(q) - set(NAMES):
            raise ValueError(f"Unknown independent joints: {set(q) - set(NAMES)}")
        values = {name: float(q.get(name, 0.)) for name in NAMES}
    else:
        angles = np.asarray(q, dtype=float)
        if angles.shape != (7,):
            raise ValueError("Expected six arm angles and one ring angle")
        values = dict(zip(NAMES, angles))
    if not all(np.isfinite(v) for v in values.values()):
        raise ValueError("Joint angles must be finite")
    root = ET.parse(path).getroot()
    nodes = {j.get("name"): j for j in root.findall("joint")}

    def value(name, seen=frozenset()):
        if name in seen:
            raise ValueError("Mimic cycle")
        mimic = nodes[name].find("mimic")
        if mimic is None:
            return values.get(name, 0.)
        return value(mimic.get("joint"), seen | {name}) * float(mimic.get("multiplier", 1)) + float(mimic.get("offset", 0))

    frames = {"base_link": np.eye(4)}
    pending = list(nodes.values())
    while pending:
        ready = [j for j in pending if j.find("parent").get("link") in frames]
        if not ready:
            raise ValueError("URDF is not a rooted tree")
        for joint in ready:
            transform = np.eye(4)
            transform[:3, 3] = vector(joint.find("origin"), "xyz")
            transform[:3, :3] = Rotation.from_euler("xyz", vector(joint.find("origin"), "rpy")).as_matrix()
            motion = np.eye(4)
            if joint.get("type") == "revolute":
                axis = vector(joint.find("axis"), "xyz", "1 0 0")
                motion[:3, :3] = Rotation.from_rotvec(axis * value(joint.get("name"))).as_matrix()
            elif joint.get("type") != "fixed":
                raise ValueError(f"Unsupported joint type: {joint.get('type')}")
            frames[joint.find("child").get("link")] = frames[joint.find("parent").get("link")] @ transform @ motion
            pending.remove(joint)
    return frames


def handle_pose(q):
    """Pose at the geometric grip center; not a measured human-hand TCP."""
    return fk(q)["tool0"]
