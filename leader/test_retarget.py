"""Offline geometry tests; run with .venv/bin/python -m unittest leader.test_retarget."""
import unittest

import numpy as np

from leader.kinematics import fk
from leader.retarget import PoseMapper, PoseRejected


class RetargetTests(unittest.TestCase):
    def setUp(self):
        self.leader_reference = np.array([0., .8, -1.4, .2, 0., .2])
        self.follower_reference = np.array([0., -.6, 1.2, .1, 0., .1])
        self.mapper = PoseMapper(self.leader_reference, self.follower_reference)

    def test_reference_pose_has_no_jump(self):
        solution = self.mapper.map(self.leader_reference, self.follower_reference)
        np.testing.assert_allclose(solution.angles_rad, self.follower_reference, atol=1e-9)

    def test_leader_tool_frame_agrees_with_validated_fk(self):
        np.testing.assert_allclose(self.mapper.leader.pose(self.leader_reference),
                                   fk(np.r_[self.leader_reference, .3])["tool0"], atol=1e-12)

    def test_100_small_cartesian_motions_match_position_and_orientation(self):
        seed = self.follower_reference.copy()
        for i in range(100):
            q = self.leader_reference + np.array([
                .02*np.sin(i*.2), .02*np.cos(i*.2), .015*np.sin(i*.3),
                .01*np.cos(i*.3), .01*np.sin(i*.3), .01*np.cos(i*.2),
            ])
            solution = self.mapper.map(q, seed)
            self.assertLess(solution.position_error_m, 1e-4)
            self.assertLess(solution.orientation_error_rad, 1e-3)
            self.assertTrue(np.all(solution.angles_rad >= self.mapper.bounds[:, 0]))
            self.assertTrue(np.all(solution.angles_rad <= self.mapper.bounds[:, 1]))
            seed = solution.angles_rad

    def test_translation_scaling_does_not_scale_rotation(self):
        mapper = PoseMapper(self.leader_reference, self.follower_reference, translation_scale=1.5)
        q = self.leader_reference.copy()
        q[4] += .05
        relative1 = np.linalg.inv(self.mapper.follower_reference) @ self.mapper.target(q)
        relative2 = np.linalg.inv(mapper.follower_reference) @ mapper.target(q)
        np.testing.assert_allclose(relative2[:3, 3], 1.5*relative1[:3, 3], atol=1e-12)
        np.testing.assert_allclose(relative2[:3, :3], relative1[:3, :3], atol=1e-12)

    def test_unreachable_target_is_rejected_instead_of_returning_clipped_angles(self):
        mapper = PoseMapper(self.leader_reference, self.follower_reference, translation_scale=1000.)
        q = self.leader_reference.copy()
        q[0] += .2
        with self.assertRaisesRegex(PoseRejected, "Unreachable pose"):
            mapper.map(q, self.follower_reference)

    def test_large_joint_step_is_rejected(self):
        mapper = PoseMapper(self.leader_reference, self.follower_reference, max_joint_step_rad=1e-6)
        q = self.leader_reference.copy()
        q[4] += .01
        with self.assertRaisesRegex(PoseRejected, "joint step"):
            mapper.map(q, self.follower_reference)

    def test_bad_seed_and_nonfinite_inputs_refuse(self):
        with self.assertRaisesRegex(ValueError, "Seed"):
            self.mapper.map(self.leader_reference, np.full(6, 100.))
        with self.assertRaisesRegex(ValueError, "finite"):
            self.mapper.target(np.full(6, np.nan))

    def test_calibrated_bounds_are_used_instead_of_preview_limits(self):
        bounds = np.column_stack([self.follower_reference-.00001, self.follower_reference+.00001])
        mapper = PoseMapper(self.leader_reference, self.follower_reference, follower_bounds=bounds)
        q = self.leader_reference.copy()
        q[4] += .1
        with self.assertRaises(PoseRejected):
            mapper.map(q, self.follower_reference)


if __name__ == "__main__":
    unittest.main()
