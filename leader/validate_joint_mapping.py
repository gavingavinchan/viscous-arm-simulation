"""Run offline joint mapping regressions and save a hash-bound audit report."""
import hashlib
import json
from pathlib import Path
import unittest

from leader.joint_mapping import ARM_JOINTS, ArmRule, JointMapper

ROOT = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    output = ROOT / "inspection/joint_mapping_validation.json"
    output.unlink(missing_ok=True)
    suite = unittest.defaultTestLoader.loadTestsFromName("leader.test_joint_mapping")
    result = unittest.TextTestRunner(verbosity=1).run(suite)
    if not result.wasSuccessful():
        raise SystemExit(1)
    inputs_path = ROOT / "mapping/inputs.json"
    inputs = json.loads(inputs_path.read_text())
    units = {}
    for unit in ("viscous_01", "viscous_waterproof_01"):
        path = ROOT / "mapping/profiles" / f"{unit}.json"
        profile = json.loads(path.read_text())
        mapper = JointMapper(profile, allow_unverified=True)
        increments = {}
        for name in ARM_JOINTS:
            rule = mapper.rules[name]
            center = rule.leader_unwrap_center_deg
            change = rule.map(center+10.).requested_deg-rule.map(center).requested_deg
            increments[name] = {"leader_change_deg": 10., "requested_follower_change_deg": change}
        units[unit] = {
            "profile_sha256": sha(path), "physically_verified": mapper.physically_verified,
            "ten_degree_changes": increments,
            "gripper_closed_deg": mapper.gripper.map(mapper.gripper.leader_closed_deg).target_deg,
            "gripper_open_deg": mapper.gripper.map(mapper.gripper.leader_open_deg).target_deg,
        }
        assert profile["source_inputs_sha256"] == sha(inputs_path)
    waterproof_path = ROOT / "mapping/profiles/viscous_waterproof_01.json"
    waterproof = json.loads(waterproof_path.read_text())
    pan = ArmRule(**waterproof["arm_joints"]["shoulder_pan"])
    raw = -107.1
    old_scale = inputs["waterproof_profile"]["leader"]["joint_directions"]["shoulder_pan"]
    old_window_center = sum(pan.follower_limits_deg)/(2.*old_scale)
    old_unwrapped = raw-round((raw-old_window_center)/360.)*360.
    old_target = min(pan.follower_limits_deg[1], max(pan.follower_limits_deg[0], old_unwrapped*old_scale))
    report = {
        "passed": True, "tests_run": result.testsRun, "mode": "joint_posture",
        "validation_scope": "Offline calculations under explicit draft reference assumptions; no physical motion or live profile changes",
        "input_sha256": sha(inputs_path),
        "implementation_sha256": sha(ROOT / "joint_mapping.py"),
        "tests_sha256": sha(ROOT / "test_joint_mapping.py"),
        "units": units,
        "waterproof_j1_regression": {
            "raw_leader_deg": raw, "neutral_requested_deg": pan.map(0.).requested_deg,
            "corrected_follower_limits_deg": list(pan.follower_limits_deg),
            "old_algorithm_if_only_limits_were_shifted": {"raw_unwrapped_deg": old_unwrapped, "target_deg": old_target},
            "new_independent_leader_window": {"raw_unwrapped_deg": pan.map(raw).leader_unwrapped_deg, "target_deg": pan.map(raw).target_deg},
        },
        "next_session": "Check per-joint matching-pose anchors, directions, raw unwrap windows and each gripper's endpoints before live integration",
    }
    output.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(output)


if __name__ == "__main__":
    main()
