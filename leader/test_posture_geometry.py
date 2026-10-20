"""Geometric posture checks against both emitted URDFs, without hardware."""
import unittest

from leader.posture_geometry import derive, report


class PostureGeometryTests(unittest.TestCase):
    def test_corresponding_segments_and_axes_agree_across_joint_poses(self):
        result = report()
        self.assertEqual(result['poses_checked'], 102)
        for key in ('max_axis_direction_error_deg', 'max_upper_arm_direction_error_deg',
                    'max_forearm_direction_error_deg'):
            self.assertLess(result[key], 1e-7, key)
        self.assertFalse(result['physically_verified'])

    def test_cad_zero_is_not_mistaken_for_matching_link_posture(self):
        _, _, signs, offsets = derive()
        self.assertEqual(signs, [1., -1., -1., -1., 1., 1.])
        self.assertGreater(abs(offsets[2]), 9.)
        self.assertGreater(abs(offsets[3]), 19.)

    def test_invalid_model_angles_refuse(self):
        leader, follower, _, _ = derive()
        for chain in (leader, follower):
            for invalid in ([0]*5, [0]*7, [0]*5+[float('nan')]):
                with self.assertRaises(ValueError):
                    chain.fk(invalid)


if __name__ == '__main__':
    unittest.main()
