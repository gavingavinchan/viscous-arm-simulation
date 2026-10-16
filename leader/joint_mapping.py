"""Offline leader-encoder to follower-angle mapping for joint-posture control.

Six arm rotations have unit angular gain, with explicit reference anchors and
direction signs. The gripper uses separate closed/open control endpoints.
This module has no hardware interfaces. Draft profiles require an explicit
allow_unverified=True, and their output remains labelled unverified.
"""
import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
              "wrist_flex", "wrist_yaw", "wrist_roll")
ALL_JOINTS = (*ARM_JOINTS, "gripper")


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def unwrap(raw_deg, center_deg):
    """Choose a full-turn branch using the LEADER frame, never follower limits."""
    raw_deg = finite(raw_deg, "raw angle")
    center_deg = finite(center_deg, "unwrap center")
    relative = (raw_deg - center_deg + 180.) % 360. - 180.
    if math.isclose(abs(relative), 180., abs_tol=1e-9, rel_tol=0):
        raise ValueError("Leader angle is at an ambiguous full-turn window boundary")
    return center_deg + relative


@dataclass(frozen=True)
class JointTarget:
    leader_unwrapped_deg: float
    requested_deg: float
    target_deg: float
    limited: bool
    limit_reasons: tuple[str, ...]


@dataclass(frozen=True)
class ArmRule:
    direction: float
    leader_reference_deg: float
    follower_reference_deg: float
    leader_unwrap_center_deg: float
    follower_limits_deg: tuple[float, float]

    def __post_init__(self):
        for name in ("direction", "leader_reference_deg", "follower_reference_deg", "leader_unwrap_center_deg"):
            finite(getattr(self, name), name)
        if self.direction not in (-1., 1.):
            raise ValueError("Arm posture mapping requires direction +1 or -1, with no travel scaling")
        validate_limits(self.follower_limits_deg)
        if abs(self.leader_reference_deg - self.leader_unwrap_center_deg) >= 180.:
            raise ValueError("Leader reference must lie inside its unwrap window")

    def map(self, raw_deg):
        angle = unwrap(raw_deg, self.leader_unwrap_center_deg)
        requested = self.follower_reference_deg + self.direction * (angle - self.leader_reference_deg)
        target = min(self.follower_limits_deg[1], max(self.follower_limits_deg[0], requested))
        limited = target != requested
        return JointTarget(angle, requested, target, limited,
                           ("follower_joint_limit",) if limited else ())


def validate_limits(bounds):
    if not isinstance(bounds, (list, tuple)) or len(bounds) != 2:
        raise ValueError("Expected two follower joint limits")
    lo, hi = (finite(v, "follower limit") for v in bounds)
    if lo >= hi:
        raise ValueError("Follower limits must be increasing")


@dataclass(frozen=True)
class GripperRule:
    leader_closed_deg: float
    leader_open_deg: float
    follower_closed_deg: float
    follower_open_deg: float
    leader_unwrap_center_deg: float
    follower_limits_deg: tuple[float, float]

    def __post_init__(self):
        for name in ("leader_closed_deg", "leader_open_deg", "follower_closed_deg", "follower_open_deg", "leader_unwrap_center_deg"):
            finite(getattr(self, name), name)
        validate_limits(self.follower_limits_deg)
        if not 0 < abs(self.leader_open_deg - self.leader_closed_deg) < 360.:
            raise ValueError("Leader gripper endpoints must be distinct and less than a turn apart")
        for value in (self.leader_closed_deg, self.leader_open_deg):
            if abs(value - self.leader_unwrap_center_deg) >= 180.:
                raise ValueError("Gripper endpoints must lie inside the leader unwrap window")
        if self.follower_closed_deg == self.follower_open_deg:
            raise ValueError("Follower gripper endpoints must be distinct")
        if any(not self.follower_limits_deg[0] <= v <= self.follower_limits_deg[1]
               for v in (self.follower_closed_deg, self.follower_open_deg)):
            raise ValueError("Follower gripper endpoints must lie within its limits")

    def map(self, raw_deg):
        angle = unwrap(raw_deg, self.leader_unwrap_center_deg)
        fraction = (angle - self.leader_closed_deg) / (self.leader_open_deg - self.leader_closed_deg)
        span = self.follower_open_deg - self.follower_closed_deg
        requested = self.follower_closed_deg + fraction * span
        clipped_fraction = min(1., max(0., fraction))
        target = self.follower_closed_deg + clipped_fraction * span
        target = min(self.follower_limits_deg[1], max(self.follower_limits_deg[0], target))
        reasons = ("gripper_control_endpoint",) if fraction != clipped_fraction else ()
        return JointTarget(angle, requested, target, bool(reasons), reasons)


class JointMapper:
    def __init__(self, profile, *, allow_unverified=False):
        if profile.get("schema_version") != 1 or profile.get("angle_unit") != "degrees" or profile.get("mapping_mode") != "joint_posture":
            raise ValueError("Expected a version-1 joint_posture profile in degrees")
        self.unit_id = profile["unit_id"]
        status = profile.get("verification", {}).get("status")
        if status not in {"draft", "physically_verified"}:
            raise ValueError("Profile verification status is missing or invalid")
        self.physically_verified = status == "physically_verified"
        if not self.physically_verified and not allow_unverified:
            raise ValueError("Draft mapping has unverified physical anchors; use allow_unverified=True for offline evaluation only")
        if set(profile["arm_joints"]) != set(ARM_JOINTS):
            raise ValueError("Expected exactly six named arm joint rules")
        self.rules = {name: ArmRule(**profile["arm_joints"][name]) for name in ARM_JOINTS}
        self.gripper = GripperRule(**profile["gripper"])

    @classmethod
    def from_file(cls, path, *, allow_unverified=False):
        return cls(json.loads(Path(path).read_text()), allow_unverified=allow_unverified)

    def map(self, leader_degrees):
        if set(leader_degrees) != set(ALL_JOINTS):
            raise ValueError("Expected exactly seven named leader encoder angles")
        targets = {name: rule.map(leader_degrees[name]) for name, rule in self.rules.items()}
        targets["gripper"] = self.gripper.map(leader_degrees["gripper"])
        return {
            "unit_id": self.unit_id,
            "angle_unit": "degrees",
            "physically_verified": self.physically_verified,
            "targets": {name: asdict(target) for name, target in targets.items()},
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--leader-degrees", type=Path, required=True,
                        help="JSON object with seven named raw leader angles")
    parser.add_argument("--offline", action="store_true", help="Allow an unverified draft for offline calculation")
    args = parser.parse_args()
    mapper = JointMapper.from_file(args.profile, allow_unverified=args.offline)
    print(json.dumps(mapper.map(json.loads(args.leader_degrees.read_text())), indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
