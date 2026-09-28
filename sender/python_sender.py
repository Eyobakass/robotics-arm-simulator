"""
Python UDP sender — simulates a C embedded controller.

Sends animated joint-angle messages over UDP using the same
JSON wire protocol that a C application would use.

Provides multiple animation modes:
    - wave:   Smooth sinusoidal motion on all joints
    - sweep:  Sequential joint sweeps
    - random: Random angle changes

Usage:
    python sender/python_sender.py                  # default wave animation
    python sender/python_sender.py --mode sweep
    python sender/python_sender.py --host 127.0.0.1 --port 9999
"""

import argparse
import json
import math
import random
import socket
import sys
import time


def create_packet(robot_name: str, joints: dict) -> bytes:
    """
    Create a UDP packet in the simulator's JSON wire protocol.

    This is exactly what a C application would produce with sprintf():
        sprintf(buf, "{\"robot\":\"%s\",\"timestamp\":%ld,\"joints\":{...}}", ...);

    Args:
        robot_name: Name of the target robot configuration.
        joints: Dict of {joint_name: angle_degrees}.

    Returns:
        UTF-8 encoded JSON bytes ready for sendto().
    """
    message = {
        "robot": robot_name,
        "timestamp": int(time.time()),
        "joints": joints,
    }
    return json.dumps(message).encode('utf-8')


def wave_animation(t: float) -> dict:
    """
    Smooth sinusoidal motion — all joints move with different
    frequencies and phases, producing a natural wave-like motion.
    """
    return {
        "base_rotation": 45.0 * math.sin(t * 0.5),
        "shoulder":      30.0 * math.sin(t * 0.7 + 1.0),
        "elbow":         50.0 * math.sin(t * 1.1 + 2.0),
    }


def sweep_animation(t: float) -> dict:
    """
    Sequential sweeps — each joint sweeps through its range one at a time.
    """
    cycle = t % 9.0  # 3 seconds per joint

    if cycle < 3.0:
        phase = cycle / 3.0
        return {
            "base_rotation": 120.0 * math.sin(phase * math.pi * 2),
            "shoulder": 0.0,
            "elbow": 0.0,
        }
    elif cycle < 6.0:
        phase = (cycle - 3.0) / 3.0
        return {
            "base_rotation": 0.0,
            "shoulder": 60.0 * math.sin(phase * math.pi * 2),
            "elbow": 0.0,
        }
    else:
        phase = (cycle - 6.0) / 3.0
        return {
            "base_rotation": 0.0,
            "shoulder": 0.0,
            "elbow": 90.0 * math.sin(phase * math.pi * 2),
        }


# Persistent state for random animation smoothing
_random_targets = {"base_rotation": 0.0, "shoulder": 0.0, "elbow": 0.0}
_random_current = {"base_rotation": 0.0, "shoulder": 0.0, "elbow": 0.0}
_random_last_change = 0.0


def random_animation(t: float) -> dict:
    """
    Random target angles with smooth interpolation.
    Picks new random targets every 2 seconds and smoothly moves toward them.
    """
    global _random_targets, _random_current, _random_last_change

    if t - _random_last_change > 2.0:
        _random_targets = {
            "base_rotation": random.uniform(-120, 120),
            "shoulder": random.uniform(-60, 60),
            "elbow": random.uniform(-90, 90),
        }
        _random_last_change = t

    # Smooth interpolation toward targets
    alpha = 0.05  # smoothing factor
    for name in _random_current:
        _random_current[name] += alpha * (_random_targets[name] - _random_current[name])

    return dict(_random_current)


ANIMATIONS = {
    'wave': wave_animation,
    'sweep': sweep_animation,
    'random': random_animation,
}


def main():
    parser = argparse.ArgumentParser(
        description='UDP Sender — simulates a C embedded controller sending '
                    'joint angles to the robotics simulator.'
    )
    parser.add_argument('--host', default='127.0.0.1',
                        help='Target host (default: 127.0.0.1)')
    parser.add_argument('--port', '-p', type=int, default=9999,
                        help='Target UDP port (default: 9999)')
    parser.add_argument('--robot', default='demo_arm',
                        help='Robot name in packets (default: demo_arm)')
    parser.add_argument('--rate', type=float, default=30.0,
                        help='Packets per second (default: 30)')
    parser.add_argument('--mode', choices=list(ANIMATIONS.keys()),
                        default='wave',
                        help='Animation mode (default: wave)')
    args = parser.parse_args()

    animation_fn = ANIMATIONS[args.mode]
    interval = 1.0 / args.rate

    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"╔══════════════════════════════════════════════════╗")
    print(f"║  UDP Joint Sender                                ║")
    print(f"║  Target:    {args.host}:{args.port:<24}  ║")
    print(f"║  Robot:     {args.robot:<35}  ║")
    print(f"║  Mode:      {args.mode:<35}  ║")
    print(f"║  Rate:      {args.rate:.0f} Hz{' ' * 32}  ║")
    print(f"╚══════════════════════════════════════════════════╝")
    print(f"\nSending joint angles... Press Ctrl+C to stop.\n")

    packet_count = 0
    start_time = time.time()

    try:
        while True:
            t = time.time() - start_time
            joints = animation_fn(t)

            packet = create_packet(args.robot, joints)
            sock.sendto(packet, (args.host, args.port))

            packet_count += 1
            if packet_count % int(args.rate) == 0:
                angle_str = "  ".join(
                    f"{k}: {v:+7.1f}°" for k, v in joints.items()
                )
                print(f"[{t:6.1f}s] {angle_str}")

            time.sleep(interval)

    except KeyboardInterrupt:
        elapsed = time.time() - start_time
        print(f"\n\nSent {packet_count} packets in {elapsed:.1f}s "
              f"({packet_count / max(elapsed, 0.001):.1f} pkt/s)")

    finally:
        sock.close()


if __name__ == '__main__':
    main()
