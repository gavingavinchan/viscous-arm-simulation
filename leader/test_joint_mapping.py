"""Joint-posture mapping regressions using the two historical arm configurations."""
import copy
import json
import math
from pathlib import Path
import unittest

from leader.joint_mapping import ARM_JOINTS, ALL_JOINTS, ArmRule, GripperRule, JointMapper, unwrap
from leader.prepare_joint_profiles import prepare_profiles

ROOT = Path(__file__).resolve().parent


def profile(unit="viscous_01"):
    return json.loads((ROOT / "mapping/profiles" / f"{unit}.json").read_text())


class JointMappingTests(unittest.TestCase):
    def test_reference_anchors_and_unit_gain_for_both_directions(self):
        for direction in (-1., 1.):
            rule = ArmRule(direction, 12., 20., 0., (-100., 100.))
            self.assertEqual(rule.map(12.).target_deg, 20.)
            self.assertEqual(rule.map(17.).target_deg, 20. + direction*5.)
            self.assertEqual(rule.map(17.+720.).target_deg, rule.map(17.).target_deg)

    def test_limits_report_requested_and_applied_angles(self):
        target = ArmRule(1., 0., 0., 0., (-30., 100.)).map(-80.)
        self.assertEqual(target.requested_deg, -80.)
        self.assertEqual(target.target_deg, -30.)
        self.assertTrue(target.limited)
        self.assertEqual(target.limit_reasons, ("follower_joint_limit",))

    def test_raw_unwrap_is_independent_of_follower_limits(self):
        for bounds in ((-185., 128.), (-32., 282.), (0., 100.)):
            target = ArmRule(1., 0., 0., 0., bounds).map(252.9)
            self.assertAlmostEqual(target.leader_unwrapped_deg, -107.1)

    def test_waterproof_pan_rezero_uses_new_limits_without_moving_neutral(self):
        inputs = json.loads((ROOT / "mapping/inputs.json").read_text())
        old = inputs["waterproof_profile"]["follower"]["joint_limits"]["shoulder_pan"]
        p = profile("viscous_waterproof_01")
        rule = ArmRule(**p["arm_joints"]["shoulder_pan"])
        self.assertAlmostEqual(rule.follower_limits_deg[0], old[0]+94.48)
        self.assertAlmostEqual(rule.follower_limits_deg[1], old[1]+94.48)
        self.assertEqual(rule.map(0.).target_deg, 0.)
        for raw in (-107.1, 252.9, -467.1):
            target = rule.map(raw)
            self.assertLess(target.requested_deg, 0.)
            self.assertAlmostEqual(target.target_deg, -32.37285584196426)

    def test_elbow_whole_turn_reading_does_not_flip_to_other_stop(self):
        rule = ArmRule(**profile()["arm_joints"]["elbow_flex"])
        for raw in (-270.5, 89.5, 449.5):
            self.assertEqual(rule.map(raw).target_deg, rule.follower_limits_deg[1])

    def test_both_profiles_use_signed_unit_gain_for_every_arm_joint(self):
        for unit in ("viscous_01", "viscous_waterproof_01"):
            p = profile(unit)
            for name in ARM_JOINTS:
                rule = ArmRule(**p["arm_joints"][name])
                first = rule.map(rule.leader_unwrap_center_deg)
                second = rule.map(rule.leader_unwrap_center_deg+10.)
                self.assertAlmostEqual(second.requested_deg-first.requested_deg, rule.direction*10.)

    def test_all_captured_raw_sweeps_are_monotonic_and_whole_turn_invariant(self):
        inputs = json.loads((ROOT / "mapping/inputs.json").read_text())
        for unit in ("viscous_01", "viscous_waterproof_01"):
            p = profile(unit)
            for servo, name in enumerate(ARM_JOINTS):
                rule = ArmRule(**p["arm_joints"][name])
                capture = inputs["leader_range_capture"]["joints"][servo]
                previous = None
                for i in range(501):
                    raw = capture["min_deg"] + i/500.*capture["travel_deg"]
                    target = rule.map(raw)
                    self.assertLessEqual(rule.follower_limits_deg[0], target.target_deg)
                    self.assertLessEqual(target.target_deg, rule.follower_limits_deg[1])
                    if previous is not None:
                        self.assertGreaterEqual(rule.direction*(target.target_deg-previous), -1e-9)
                    previous = target.target_deg
                    for turns in (-2, -1, 1, 2):
                        self.assertAlmostEqual(rule.map(raw+360.*turns).target_deg, target.target_deg, places=9)

    def test_draft_profile_refuses_default_loading_and_labels_offline_results(self):
        for unit in ("viscous_01", "viscous_waterproof_01"):
            with self.assertRaisesRegex(ValueError, "Draft"):
                JointMapper(profile(unit))
            result = JointMapper(profile(unit), allow_unverified=True).map(dict.fromkeys(ALL_JOINTS, 0.))
            self.assertFalse(result["physically_verified"])
            self.assertEqual(set(result["targets"]), set(ALL_JOINTS))

    def test_backed_off_limits_are_visible_at_assumed_zero(self):
        mapper = JointMapper(profile(), allow_unverified=True)
        result = mapper.map(dict.fromkeys(ALL_JOINTS, 0.))["targets"]
        for name in ("shoulder_lift", "elbow_flex"):
            self.assertEqual(result[name]["requested_deg"], 0.)
            self.assertTrue(result[name]["limited"])

    def test_gripper_endpoints_and_halfway_match_each_unit(self):
        closed_values = {"viscous_01": -33., "viscous_waterproof_01": -2.9220847551671985}
        for unit, closed in closed_values.items():
            rule = GripperRule(**profile(unit)["gripper"])
            self.assertAlmostEqual(rule.map(rule.leader_closed_deg).target_deg, closed)
            self.assertAlmostEqual(rule.map(rule.leader_open_deg).target_deg, rule.follower_open_deg)
            halfway = (rule.leader_closed_deg+rule.leader_open_deg)/2.
            self.assertAlmostEqual(rule.map(halfway).target_deg, (closed+rule.follower_open_deg)/2.)
            self.assertAlmostEqual(rule.map(halfway+720.).target_deg, rule.map(halfway).target_deg)
            self.assertTrue(rule.map(rule.leader_open_deg+5.).limited)

    def test_input_names_and_nonfinite_values_refuse(self):
        mapper = JointMapper(profile(), allow_unverified=True)
        with self.assertRaisesRegex(ValueError, "seven"):
            mapper.map({"joint1": 10.})
        for bad in (math.nan, math.inf, True, "1.0"):
            values = dict.fromkeys(ALL_JOINTS, 0.)
            values["wrist_yaw"] = bad
            with self.assertRaises(ValueError):
                mapper.map(values)

    def test_bad_rules_and_ambiguous_wrap_boundary_refuse(self):
        for direction in (0., .9, 1.44, math.nan):
            with self.assertRaises(ValueError):
                ArmRule(direction, 0., 0., 0., (-100., 100.))
        with self.assertRaises(ValueError):
            ArmRule(1., 0., 0., 0., (100., -100.))
        with self.assertRaises(ValueError):
            ArmRule(1., 200., 0., 0., (-100., 100.))
        for value in (-180., 180., 540.):
            with self.assertRaisesRegex(ValueError, "ambiguous"):
                unwrap(value, 0.)

    def test_profile_schema_and_missing_joint_refuse(self):
        for key, value in (("angle_unit", "radians"), ("mapping_mode", "cartesian"), ("schema_version", 2)):
            p = profile()
            p[key] = value
            with self.assertRaises(ValueError):
                JointMapper(p, allow_unverified=True)
        p = profile()
        del p["arm_joints"]["wrist_yaw"]
        with self.assertRaisesRegex(ValueError, "six"):
            JointMapper(p, allow_unverified=True)

    def test_profile_generation_reproduces_checked_in_rules(self):
        generated = prepare_profiles(json.loads((ROOT / "mapping/inputs.json").read_text()))
        for unit, p in generated.items():
            actual = profile(unit)
            for key, value in p.items():
                self.assertEqual(value, actual[key])

    def test_generation_refuses_already_shifted_or_wrong_frame_inputs(self):
        inputs = json.loads((ROOT / "mapping/inputs.json").read_text())
        limits = inputs["waterproof_profile"]["follower"]["joint_limits"]
        limits["shoulder_pan"] = [value+94.48 for value in limits["shoulder_pan"]]
        with self.assertRaisesRegex(ValueError, "double shift"):
            prepare_profiles(inputs)

    def test_gripper_bad_or_out_of_limit_endpoints_refuse(self):
        for changes in ({"leader_open_deg": 0.}, {"follower_closed_deg": 100.}, {"leader_open_deg": 400.}):
            fields = copy.deepcopy(profile()["gripper"])
            fields.update(changes)
            with self.assertRaises(ValueError):
                GripperRule(**fields)


if __name__ == "__main__":
    unittest.main()
