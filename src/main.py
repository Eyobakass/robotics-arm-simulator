"""
Robotics Simulator — Main Entry Point

Loads a JSON robot configuration, starts the UDP server,
creates the VPython 3D scene, and runs the real-time update loop.

Usage:
    python src/main.py                          # default config
    python src/main.py --config config/demo_arm.json
    python src/main.py --port 9999
"""

import argparse
import logging
import os
import sys
import signal

# Ensure src/ is on the Python path so submodule imports work
SRC_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SRC_DIR)
sys.path.insert(0, SRC_DIR)
os.chdir(PROJECT_DIR)

import vpython as vp

from robot.config_loader import load_config
from robot.robot_model import RobotModel
from kinematics.forward_kinematics import compute_forward_kinematics
from network.udp_server import UDPServer
from visualization.vpython_renderer import VPythonRenderer


def parse_args():
    parser = argparse.ArgumentParser(
        description='Robotics Arm Simulator — receives joint angles via UDP '
                    'and visualizes the arm in 3D.'
    )
    parser.add_argument(
        '--config', '-c',
        default='config/demo_arm.json',
        help='Path to robot JSON configuration file (default: config/demo_arm.json)',
    )
    parser.add_argument(
        '--host',
        default='127.0.0.1',
        help='UDP listen address (default: 127.0.0.1)',
    )
    parser.add_argument(
        '--port', '-p',
        type=int,
        default=9999,
        help='UDP listen port (default: 9999)',
    )
    parser.add_argument(
        '--fps',
        type=int,
        default=60,
        help='Visualization frame rate (default: 60)',
    )
    return parser.parse_args()


def main():
    args = parse_args()

    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    )
    logger = logging.getLogger('simulator')

    # ── Load Configuration ──────────────────────────────────────────
    logger.info(f"Loading robot config: {args.config}")
    try:
        config = load_config(args.config)
    except (FileNotFoundError, ValueError) as e:
        logger.error(f"Failed to load config: {e}")
        sys.exit(1)

    logger.info(f"Robot: {config.name} — {config.description}")
    logger.info(f"Joints: {[j['name'] for j in config.joints]}")

    # ── Build Robot Model ───────────────────────────────────────────
    robot = RobotModel(config)

    # ── Create 3D Visualization ─────────────────────────────────────
    logger.info("Initializing VPython 3D visualization...")
    renderer = VPythonRenderer(robot)
    renderer.set_udp_port(args.port)

    # Initial FK computation and render
    positions, orientations = compute_forward_kinematics(robot)
    renderer.update(positions, orientations)

    # ── Start UDP Server ────────────────────────────────────────────
    server = UDPServer(host=args.host, port=args.port)
    server.start()

    logger.info(f"Listening for joint updates on UDP {args.host}:{args.port}")
    logger.info("Send joint angles to see the arm move!")
    logger.info("Press Ctrl+C in this terminal to exit.\n")

    # ── Handle Ctrl+C ───────────────────────────────────────────────
    shutdown_requested = False

    def signal_handler(sig, frame):
        nonlocal shutdown_requested
        shutdown_requested = True

    signal.signal(signal.SIGINT, signal_handler)

    # ── Main Loop ───────────────────────────────────────────────────
    frame_count = 0
    try:
        while not shutdown_requested:
            vp.rate(args.fps)

            # Check for new joint data from UDP
            update = server.get_latest_update()
            if update is not None:
                robot.apply_update(update)

                # Recompute FK and update visualization
                positions, orientations = compute_forward_kinematics(robot)
                renderer.update(positions, orientations)

                # Update status display
                angles = robot.get_joint_angles()
                angle_str = "  ".join(
                    f"{name}: {angle:+.1f}°" for name, angle in angles.items()
                )
                renderer.update_status(angle_str)

                if frame_count % 60 == 0:
                    logger.info(f"Joint angles: {angles}")
                frame_count += 1

    except KeyboardInterrupt:
        pass
    finally:
        logger.info("Shutting down...")
        server.stop()
        logger.info("Goodbye!")


if __name__ == '__main__':
    main()
