"""Capture raw Star and RobStride parameters without configuring either arm.

Run with the Mac Lab Python environment. Follower TX is restricted to native
READ_PARAMETER (17), 0x3016 and 0x7019. Leader calls read_angle only: no origin
reset, multi-turn reset, unlock, configure, enable, disable or fault clear.
The default native path never falls back. An explicit mit-feedback mode can
request feedback with a non-enabling zero-gain packet, but requires supported,
torque-off hardware. This second mode is a zero-effort command, not a passive
parameter read. Neither mode calls driver connect() or sends torque enable.
"""
import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import struct
import subprocess
import time
import urllib.request

NAMES = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_yaw', 'wrist_roll', 'gripper')
PARAMETERS = {0x3016: 'measured_position', 0x7019: 'mechanical_position'}
ROOT = Path(__file__).resolve().parents[1]


def read_request(motor, parameter):
    if motor not in range(1, 8) or parameter not in PARAMETERS:
        raise ValueError('Only commissioned IDs and position read parameters are permitted')
    identifier = (17 << 24) | (0xFD << 8) | motor
    return f'T{identifier:08X}8{struct.pack("<HHL", parameter, 0, 0).hex().upper()}\r'.encode()


def decode_reply(line, motor, parameter):
    if not line.startswith(b'T') or len(line) != 26:
        return None
    try:
        identifier = int(line[1:9], 16)
        data = bytes.fromhex(line[10:].decode())
        if (int(line[9:10], 16) != 8 or identifier >> 24 != 17
                or identifier & 255 != 0xFD or (identifier >> 8) & 255 != motor
                or int.from_bytes(data[:2], 'little') != parameter):
            return None
        value = struct.unpack('<f', data[4:8])[0]
        return value if math.isfinite(value) else None
    except (ValueError, struct.error, UnicodeDecodeError):
        return None


def mit_feedback_request(motor):
    if motor not in range(1, 8):
        raise ValueError('Only commissioned motor IDs are permitted')
    # Standard MIT packet: target/velocity/torque zero, Kp=Kd=0. Integer
    # quantization uses the same midpoints as the commissioned RS00 driver.
    # No enable (FC), disable (FD), zero (FE) or fault-clear (FB) command.
    return f't{motor:03X}87FFF7FF0000007FF\r'.encode()


def decode_mit_reply(line, motor):
    if not line.startswith(b't0FD8') or len(line) != 21:
        return None
    try:
        data = bytes.fromhex(line[5:].decode())
        if len(data) != 8 or data[0] != motor:
            return None
        position = -12.57 + int.from_bytes(data[1:3], 'big')*25.14/65535
        velocity = -33 + ((data[3] << 4) | (data[4] >> 4))*66/4095
        torque = -14 + (((data[4] & 15) << 8) | data[5])*28/4095
        return {'radians': position, 'degrees': math.degrees(position),
                'velocity_rad_s': velocity, 'torque_nm': torque,
                'temperature_c': int.from_bytes(data[6:8], 'big')/10}
    except (ValueError, UnicodeDecodeError):
        return None


def idle(api):
    with urllib.request.urlopen(api.rstrip('/')+'/sessions/current', timeout=3) as response:
        state = json.load(response)
    if state.get('session'):
        raise RuntimeError('An active Lab session owns hardware; use its normal GUI stop first')


def unowned(ports):
    paths = set(ports)
    for port in ports:
        if not Path(port).exists():
            raise RuntimeError(f'USB port missing: {port}')
        if port.startswith('/dev/cu.'):
            paths.add(port.replace('/dev/cu.', '/dev/tty.', 1))
    result = subprocess.run(['lsof', '-t', *sorted(p for p in paths if Path(p).exists())], text=True, capture_output=True)
    if result.returncode not in (0, 1):
        raise RuntimeError('Cannot check USB ownership: '+result.stderr)
    if result.stdout.strip():
        raise RuntimeError('A process already owns a selected USB port: '+result.stdout.strip())


class FollowerReader:
    def __init__(self, port, frames):
        import serial
        self.frames = frames
        self.serial = serial.Serial(port, baudrate=115200, timeout=.01, write_timeout=.3, exclusive=True)
        self.buffer = bytearray()
        # These are CAN adapter channel/bitrate commands, never motor commands.
        self.serial.write(b'\rC\r'); self.serial.flush(); time.sleep(.05)
        self.serial.reset_input_buffer()
        self.serial.write(b'S8\rO\r'); self.serial.flush(); time.sleep(.05)
        reply = self.serial.read(64)
        if b'\a' in reply:
            self.close()
            raise RuntimeError('CAN adapter rejected channel setup')

    def read(self, motor, parameter):
        request = read_request(motor, parameter)
        self.frames.append({'direction': 'tx', 'at': time.time(), 'slcan': request.decode().strip()})
        self.serial.write(request); self.serial.flush()
        deadline = time.monotonic()+.35
        while time.monotonic() < deadline:
            self.buffer.extend(self.serial.read(512))
            while b'\r' in self.buffer:
                end = self.buffer.index(b'\r')
                line = bytes(self.buffer[:end]); del self.buffer[:end+1]
                if line:
                    self.frames.append({'direction': 'rx', 'at': time.time(), 'slcan': line.decode(errors='replace')})
                value = decode_reply(line, motor, parameter)
                if value is not None:
                    return {'radians': value, 'degrees': math.degrees(value), 'received_at': time.time()}
        raise RuntimeError(f'No native position-parameter reply from motor {motor}, 0x{parameter:04X}. No torque or mode-changing fallback was attempted.')

    def close(self):
        try:
            self.serial.write(b'C\r'); self.serial.flush()
        finally:
            self.serial.close()

    def read_mit_feedback(self, motor):
        # Discard pre-existing queued feedback so it cannot masquerade as the
        # response to this request. This path is exclusively for torque-off,
        # manually supported hardware, never a concurrently controlled bus.
        self.serial.reset_input_buffer(); self.buffer.clear()
        request = mit_feedback_request(motor)
        self.frames.append({'direction': 'tx', 'at': time.time(), 'slcan': request.decode().strip()})
        self.serial.write(request); self.serial.flush()
        deadline = time.monotonic()+.35
        while time.monotonic() < deadline:
            self.buffer.extend(self.serial.read(512))
            while b'\r' in self.buffer:
                end = self.buffer.index(b'\r')
                line = bytes(self.buffer[:end]); del self.buffer[:end+1]
                if line:
                    self.frames.append({'direction': 'rx', 'at': time.time(), 'slcan': line.decode(errors='replace')})
                value = decode_mit_reply(line, motor)
                if value is not None:
                    return {**value, 'received_at': time.time()}
        raise RuntimeError(f'No fresh MIT feedback from motor {motor}; no enable was sent')


def capture(args):
    from motorbridge_smart_servo import FashionStarServo
    if args.protocol == 'mit-feedback' and not args.supported_torque_off:
        raise ValueError('MIT feedback requests require supported, torque-off arms; explicitly select --supported-torque-off')
    home = Path.home()/'.makermods/makermodslab-viscous'
    robot_path = home/'robots'/f'{args.unit}.json'
    robot = json.loads(robot_path.read_text())
    if robot.get('arm_type') != 'viscous' or robot.get('mode') != 'single':
        raise ValueError('Expected a single commissioned Viscous record')
    leader_port = args.leader_port or robot['leader_port']
    follower_port = args.follower_port or robot['follower_port']
    if leader_port == follower_port:
        raise ValueError('Leader and follower ports must differ')
    idle(args.api); unowned((leader_port, follower_port))
    now = datetime.now().astimezone()
    path = ROOT/'leader/mapping/captures'/f'{args.unit}-{now:%Y%m%d-%H%M%S}.json'
    path.parent.mkdir(parents=True, exist_ok=True)
    record = {'record_type': 'paired_encoder_capture', 'protocol': args.protocol, 'created_at': now.isoformat(),
              'unit': args.unit, 'pose_description': args.pose, 'physically_matched': False,
              'source': 'Fresh leader raw read_angle; follower native parameters or explicit non-enabling zero-gain MIT feedback requests; sequential, not simultaneous',
              'frame_caution': 'Raw follower values retain their reported turn branch. Native parameters are not automatically equated to MIT coordinates. No automatic fitting or branch selection.',
              'leader_port': leader_port, 'follower_port': follower_port,
              'motor_configuration_changed': False, 'samples': [], 'frames': [], 'complete': False,
              'source_sha256': {str(robot_path): hashlib.sha256(robot_path.read_bytes()).hexdigest()}}
    for config_path in (home/'viscous_profiles'/f'{robot["follower_config"]}.json',
                        Path.home()/'.cache/huggingface/lerobot/calibration/robots/viscous_follower'/f'{robot["follower_config"]}.json',
                        Path.home()/'.cache/huggingface/lerobot/calibration/teleoperators/rebot_102_leader'/f'{robot["leader_config"]}.json'):
        record['source_sha256'][str(config_path)] = hashlib.sha256(config_path.read_bytes()).hexdigest()
    follower = leader = None
    try:
        follower = FollowerReader(follower_port, record['frames'])
        leader = FashionStarServo(leader_port, baudrate=1_000_000)
        for _ in range(args.samples):
            idle(args.api)
            sample = {'started_at': time.time(), 'leader': {}, 'follower': {}}
            record['samples'].append(sample)
            for i, name in enumerate(NAMES):
                attempts = []
                for _attempt in range(3):
                    reading = leader.read_angle(i, multi_turn=True)
                    attempts.append({'raw_deg': reading.raw_deg, 'reliable': reading.reliable,
                                     'received_at': time.time()})
                    if reading.reliable and math.isfinite(reading.raw_deg):
                        break
                    time.sleep(.05)
                sample['leader'][name] = {'raw_deg': reading.raw_deg, 'received_at': time.time(),
                                          'reliable': reading.reliable, 'attempts': attempts}
                if not reading.reliable or not math.isfinite(reading.raw_deg):
                    raise RuntimeError(f'No reliable leader reading for {name}')
                if args.protocol == 'mit-feedback':
                    sample['follower'][name] = {'mit_feedback': follower.read_mit_feedback(i+1)}
                else:
                    sample['follower'][name] = {label: follower.read(i+1, parameter) for parameter, label in PARAMETERS.items()}
            sample['finished_at'] = time.time()
        record['complete'] = True
        record['leader_median_raw_deg'] = {name: statistics.median(s['leader'][name]['raw_deg'] for s in record['samples']) for name in NAMES}
    except Exception as exc:
        record['error'] = str(exc)
        raise
    finally:
        try:
            if leader is not None:
                leader.close()
        finally:
            try:
                if follower is not None:
                    follower.close()
            finally:
                record['finished_at'] = datetime.now().astimezone().isoformat()
                path.write_text(json.dumps(record, indent=2)+'\n')
                print(f'Evidence: {path}', flush=True)
    print(json.dumps({key: value for key, value in record.items() if key not in ('samples', 'frames')}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--unit', default='viscous_waterproof_01', choices=('viscous_waterproof_01', 'viscous_01'))
    parser.add_argument('--leader-port')
    parser.add_argument('--follower-port')
    parser.add_argument('--api', default='http://127.0.0.1:8000/api/v1')
    parser.add_argument('--pose', required=True, help='Describe the rough posture and which joints are being matched')
    parser.add_argument('--samples', type=int, choices=range(1, 6), default=3)
    parser.add_argument('--protocol', choices=('native', 'mit-feedback'), default='native')
    parser.add_argument('--supported-torque-off', action='store_true', help='Arms have been supported and torque is off, as required for zero-gain MIT feedback requests')
    capture(parser.parse_args())
