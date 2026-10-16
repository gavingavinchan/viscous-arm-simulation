"""Rebuild offline joint-posture drafts from the preserved commissioned records."""
import hashlib
import json
import math
from pathlib import Path

from leader.joint_mapping import ARM_JOINTS

ROOT = Path(__file__).resolve().parent


def prepare_profiles(inputs):
    captured = {j["servo"]: j for j in inputs["leader_range_capture"]["joints"]}
    profiles = {}
    for unit in ("viscous_01", "viscous_waterproof_01"):
        waterproof = unit == "viscous_waterproof_01"
        limits = inputs["waterproof_profile"]["follower"]["joint_limits"] if waterproof else inputs["legacy_follower_limits_deg"]
        limits = {name: list(bounds) for name, bounds in limits.items()}
        if waterproof:
            # Old = new - shift, so re-express limits as new = old + shift.
            shift = -inputs["waterproof_j1_rezero"]["reading_before_old_frame_deg"]
            limits["shoulder_pan"] = [value + shift for value in limits["shoulder_pan"]]
            recorded = inputs["waterproof_j1_rezero"]["captured_stops_new_frame_deg"]["backed_off_soft_limits_shifted"]
            if any(abs(actual-expected) > .005 for actual, expected in zip(limits["shoulder_pan"], recorded)):
                raise ValueError("Waterproof J1 input frame does not match its re-zero record; refusing a possible double shift")
        rules = {}
        for servo, name in enumerate(ARM_JOINTS):
            capture = captured[servo]
            # Pan's origin was subsequently changed; center the draft unwrap
            # window on its matched neutral instead of borrowing old endpoints.
            center = 0. if servo == 0 else (capture["min_deg"] + capture["max_deg"]) / 2.
            rules[name] = {
                "direction": math.copysign(1., inputs["legacy_leader_directions"][name]),
                "leader_reference_deg": 0.,
                "follower_reference_deg": 0.,
                "leader_unwrap_center_deg": center,
                "follower_limits_deg": limits[name],
            }
        travel = captured[6]["travel_deg"]
        profiles[unit] = {
            "schema_version": 1, "unit_id": unit, "angle_unit": "degrees",
            "mapping_mode": "joint_posture",
            "verification": {
                "status": "draft",
                "arm_anchors": "Zero-to-zero assumptions only; no fresh matched-pose measurements. Replace each leader/follower reference with a checked corresponding pose.",
                "directions": "Signs retained from the existing commissioned mapping; not freshly verified against the URDFs.",
                "unwrap_windows": "Independent leader-frame windows. Non-pan centers use historical 2026-08-29 capture; pan uses neutral 0 after origin changes. Recheck in current leader zero frame.",
                "gripper": "Historical follower closed/open limits; leader closed=0 and open=archived travel are assumptions requiring endpoint verification. Control interpolation is not a transmission or jaw-gap calibration.",
                "follower_frame": "ROV-mounted J1 zero, with shifted J1 limits; neutral remains zero" if waterproof else "Original non-waterproof unit's commissioned frame",
            },
            "arm_joints": rules,
            "gripper": {
                "leader_closed_deg": 0., "leader_open_deg": travel,
                "follower_closed_deg": limits["gripper"][1] if waterproof else inputs["legacy_gripper_closed_offset_deg"],
                "follower_open_deg": limits["gripper"][0],
                "leader_unwrap_center_deg": travel / 2.,
                "follower_limits_deg": limits["gripper"],
            },
        }
    return profiles


def main():
    source = ROOT / "mapping/inputs.json"
    profiles = prepare_profiles(json.loads(source.read_text()))
    output = ROOT / "mapping/profiles"
    output.mkdir(exist_ok=True)
    for unit, profile in profiles.items():
        profile["source_inputs_sha256"] = hashlib.sha256(source.read_bytes()).hexdigest()
        (output / f"{unit}.json").write_text(json.dumps(profile, indent=2) + "\n")
        print(unit, "draft joint-posture profile")


if __name__ == "__main__":
    main()
