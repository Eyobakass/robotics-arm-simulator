"""Quick integration test — verifies all core systems without VPython."""
import sys
import os
import socket
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from robot.config_loader import load_config
from robot.robot_model import RobotModel
from kinematics.forward_kinematics import compute_forward_kinematics
from network.packet_parser import parse_packet
from network.udp_server import UDPServer

# Change to project root for config path
os.chdir(os.path.join(os.path.dirname(__file__), '..'))

print("=" * 50)
print("Integration Test — Core Systems")
print("=" * 50)

# 1. Config loading
config = load_config('config/demo_arm.json')
print(f"[OK] Config: {config.name}, {len(config.joints)} joints")

# 2. Robot model
robot = RobotModel(config)
print(f"[OK] Model: {robot}")

# 3. FK at zero angles
positions, orientations = compute_forward_kinematics(robot)
print(f"[OK] FK at zero: end effector at "
      f"[{positions[-1][0]:.2f}, {positions[-1][1]:.2f}, {positions[-1][2]:.2f}]")

# 4. Packet parsing
pkt = b'{"robot":"demo_arm","timestamp":123,"joints":{"base_rotation":45,"shoulder":30,"elbow":-20}}'
update = parse_packet(pkt)
print(f"[OK] Parsed: {update.joints}")

# 5. Apply update + FK
robot.apply_update(update)
positions2, _ = compute_forward_kinematics(robot)
print(f"[OK] FK after update: end effector at "
      f"[{positions2[-1][0]:.2f}, {positions2[-1][1]:.2f}, {positions2[-1][2]:.2f}]")

# 6. Malformed packet handling
bad_packets = [
    b'not json',
    b'{"robot":"arm"}',
    b'\xff\xfe\xfd',
    b'',
    b'{"robot":"arm","joints":{"j1":"text"}}',
]
for bp in bad_packets:
    assert parse_packet(bp) is None
print(f"[OK] All {len(bad_packets)} malformed packets rejected")

# 7. UDP round-trip
server = UDPServer('127.0.0.1', 19876)
server.start()
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(pkt, ('127.0.0.1', 19876))
time.sleep(0.3)
received = server.get_latest_update()
server.stop()
sock.close()
assert received is not None
assert received.joints['base_rotation'] == 45.0
print(f"[OK] UDP round-trip: sent -> received {received.joints}")

# 8. Joint clamping
robot.joints[1].set_angle(200)  # shoulder max is 90
assert robot.joints[1].current_angle == 90.0
print(f"[OK] Joint clamping: 200 deg -> {robot.joints[1].current_angle} deg")

print()
print("=" * 50)
print("ALL CORE SYSTEMS VERIFIED SUCCESSFULLY")
print("=" * 50)
