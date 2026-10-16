"""Offline Cartesian pose retargeting between the leader and Viscous URDFs.

Inputs and outputs are MODEL joint angles in radians, never encoder values.
Encoder sign/zero calibration must be established separately before integrating
this with a controller. This module has no device or motor interfaces.
"""
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent.parent


def numbers(element, attribute, default="0 0 0"):
    return np.fromstring(element.get(attribute, default) if element is not None else default, sep=" ")


class Chain:
    """Read joint transforms from the actual URDF, including its tool frame."""

    def __init__(self, urdf, names, tool="tool0"):
        self.path = Path(urdf)
        self.names = tuple(names)
        root = ET.parse(self.path).getroot()
        self.joints = {}
        for element in root.findall("joint"):
            name, kind = element.get("name"), element.get("type")
            if kind not in {"fixed", "revolute", "prismatic"}:
                raise ValueError(f"Unsupported joint type {kind}")
            origin = np.eye(4)
            origin[:3, 3] = numbers(element.find("origin"), "xyz")
            origin[:3, :3] = Rotation.from_euler("xyz", numbers(element.find("origin"), "rpy")).as_matrix()
            axis = numbers(element.find("axis"), "xyz", "1 0 0")
            if kind != "fixed" and not np.isclose(np.linalg.norm(axis), 1.):
                raise ValueError(f"Invalid axis in {name}")
            self.joints[name] = {
                "parent": element.find("parent").get("link"),
                "child": element.find("child").get("link"),
                "origin": origin, "kind": kind, "axis": axis,
                "mimic": element.find("mimic"), "limit": element.find("limit"),
            }
        all_links = {l.get("name") for l in root.findall("link")}
        roots = all_links - {j["child"] for j in self.joints.values()}
        if len(roots) != 1 or tool not in all_links:
            raise ValueError("Expected one rooted URDF tree with a tool frame")
        self.root, self.tool = next(iter(roots)), tool
        self.bounds = np.array([
            [float(self.joints[n]["limit"].get("lower")), float(self.joints[n]["limit"].get("upper"))]
            for n in self.names
        ])

    def pose(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (len(self.names),) or not np.isfinite(q).all():
            raise ValueError("Expected finite model joint angles in radians")
        values = dict(zip(self.names, q))

        def value(name, seen=frozenset()):
            if name in seen:
                raise ValueError("Mimic cycle")
            mimic = self.joints[name]["mimic"]
            if mimic is None:
                return values.get(name, 0.)
            return value(mimic.get("joint"), seen | {name}) * float(mimic.get("multiplier", 1)) + float(mimic.get("offset", 0))

        frames, pending = {self.root: np.eye(4)}, dict(self.joints)
        while pending:
            ready = [name for name, j in pending.items() if j["parent"] in frames]
            if not ready:
                raise ValueError("URDF contains a disconnected tree or cycle")
            for name in ready:
                joint = pending.pop(name)
                motion = np.eye(4)
                if joint["kind"] == "revolute":
                    motion[:3, :3] = Rotation.from_rotvec(joint["axis"] * value(name)).as_matrix()
                elif joint["kind"] == "prismatic":
                    motion[:3, 3] = joint["axis"] * value(name)
                frames[joint["child"]] = frames[joint["parent"]] @ joint["origin"] @ motion
        return frames[self.tool]


@dataclass(frozen=True)
class Solution:
    angles_rad: np.ndarray
    target: np.ndarray
    position_error_m: float
    orientation_error_rad: float


class PoseRejected(ValueError):
    """An unreachable or discontinuous target must not become a motor command."""


class PoseMapper:
    """Match motion about corresponding hand poses, using follower IK.

    Reference poses align the two hand coordinate frames. Translation scaling
    is explicit; rotation is preserved. Limits are model bounds unless the
    caller supplies independently calibrated bounds in MODEL coordinates.
    No physical accuracy, collision clearance or real-time speed is implied.
    """

    def __init__(self, leader_reference, follower_reference, translation_scale=1.,
                 follower_bounds=None, max_joint_step_rad=.35):
        self.leader = Chain(ROOT / "leader/model/star_arm_102_hd.urdf", [f"joint{i}" for i in range(1, 7)])
        self.follower = Chain(ROOT / "model/viscous_arm.urdf", [
            "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll",
        ])
        if not np.isfinite(translation_scale) or translation_scale <= 0:
            raise ValueError("Translation scale must be finite and positive")
        if not np.isfinite(max_joint_step_rad) or max_joint_step_rad <= 0:
            raise ValueError("Joint step bound must be finite and positive")
        self.scale, self.max_step = float(translation_scale), float(max_joint_step_rad)
        self.bounds = self.follower.bounds.copy() if follower_bounds is None else np.asarray(follower_bounds, dtype=float)
        if self.bounds.shape != (6, 2) or not np.isfinite(self.bounds).all() or np.any(self.bounds[:, 0] >= self.bounds[:, 1]):
            raise ValueError("Expected six finite increasing follower angle bounds")
        self.leader_reference = self.leader.pose(leader_reference)
        self.follower_reference = self.follower.pose(follower_reference)
        self._validate_seed(follower_reference)

    def _validate_seed(self, seed):
        seed = np.asarray(seed, dtype=float)
        if seed.shape != (6,) or not np.isfinite(seed).all() or np.any(seed < self.bounds[:, 0]) or np.any(seed > self.bounds[:, 1]):
            raise ValueError("Seed must be within the supplied model angle bounds")
        return seed

    def target(self, leader_angles):
        relative = np.linalg.inv(self.leader_reference) @ self.leader.pose(leader_angles)
        relative[:3, 3] *= self.scale
        return self.follower_reference @ relative

    def map(self, leader_angles, follower_seed):
        seed = self._validate_seed(follower_seed)
        target = self.target(leader_angles)

        def residual(q):
            actual = self.follower.pose(q)
            return np.r_[actual[:3, 3] - target[:3, 3],
                         .15 * Rotation.from_matrix(target[:3, :3].T @ actual[:3, :3]).as_rotvec()]

        solution = least_squares(residual, seed, bounds=(self.bounds[:, 0], self.bounds[:, 1]),
                                 xtol=1e-11, ftol=1e-11, gtol=1e-11, max_nfev=150)
        actual = self.follower.pose(solution.x)
        position_error = float(np.linalg.norm(actual[:3, 3] - target[:3, 3]))
        orientation_error = float(Rotation.from_matrix(target[:3, :3].T @ actual[:3, :3]).magnitude())
        if not solution.success or position_error > 1e-4 or orientation_error > 1e-3:
            raise PoseRejected(f"Unreachable pose: {position_error*1000:.3f} mm, {np.rad2deg(orientation_error):.3f} deg residual")
        if np.max(np.abs(solution.x - seed)) > self.max_step:
            raise PoseRejected("Target requires a joint step larger than the configured bound")
        return Solution(solution.x.copy(), target, position_error, orientation_error)
