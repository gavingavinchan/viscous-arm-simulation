"""Wire-level whitelist and reply matching for the sensor-only capture path."""
import struct
import unittest

from leader.capture_pair import read_request, decode_reply, mit_feedback_request, decode_mit_reply


class CapturePairTests(unittest.TestCase):
    def test_mit_feedback_packets_never_enable_torque_and_have_zero_gains(self):
        for motor in range(1, 8):
            packet = mit_feedback_request(motor)
            self.assertEqual(packet[:5], f't{motor:03X}8'.encode())
            data = bytes.fromhex(packet[5:-1].decode())
            self.assertEqual(len(data), 8)
            self.assertEqual(((data[3] & 15) << 8) | data[4], 0)  # Kp
            self.assertEqual((data[5] << 4) | (data[6] >> 4), 0)  # Kd
            self.assertNotEqual(data[:7], bytes([255])*7)
        with self.assertRaises(ValueError):
            mit_feedback_request(8)

    def test_mit_feedback_requires_correct_joint_and_full_encoder_frame(self):
        frame = b't0FD8017FFF7FF7FF00D2'
        decoded = decode_mit_reply(frame, 1)
        self.assertIsNotNone(decoded)
        self.assertAlmostEqual(decoded['degrees'], 0, delta=.02)
        self.assertEqual(decoded['temperature_c'], 21)
        self.assertIsNone(decode_mit_reply(frame, 2))
        self.assertIsNone(decode_mit_reply(b't0FD50100000000', 1))
        self.assertIsNone(decode_mit_reply(frame.replace(b't0FD', b'T0FD'), 1))

    def test_transmit_can_only_read_the_two_position_parameters(self):
        for motor in range(1, 8):
            for parameter in (0x3016, 0x7019):
                request = read_request(motor, parameter)
                self.assertEqual(request[:10], f'T1100FD{motor:02X}8'.encode())
                self.assertEqual(bytes.fromhex(request[10:-1].decode()), struct.pack('<HHL', parameter, 0, 0))
        for motor, parameter in ((0, 0x3016), (8, 0x3016), (1, 0x7005)):
            with self.assertRaises(ValueError):
                read_request(motor, parameter)

    def test_reply_must_match_motor_parameter_and_host(self):
        payload = struct.pack('<HHf', 0x7019, 0, 1.25).hex().upper()
        reply = f'T110001FD8{payload}'.encode()
        self.assertEqual(decode_reply(reply, 1, 0x7019), 1.25)
        self.assertIsNone(decode_reply(reply, 2, 0x7019))
        self.assertIsNone(decode_reply(reply, 1, 0x3016))
        self.assertIsNone(decode_reply(reply.replace(b'01FD8', b'01FE8'), 1, 0x7019))
        self.assertIsNone(decode_reply(b't0FD50100000000', 1, 0x7019))
        nan = f'T110001FD8{struct.pack("<HHf", 0x7019, 0, float("nan")).hex()}'.encode()
        self.assertIsNone(decode_reply(nan, 1, 0x7019))


if __name__ == '__main__':
    unittest.main()
