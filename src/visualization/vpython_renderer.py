"""
VPython 3D renderer for the robotic arm simulator.

Renders the robot as articulated 3D geometry:
    - Box for the base platform
    - Sphere at each joint
    - Cylinder for each link
    - Cone for the end effector
    - Grid floor and axis indicators
    - Start/Stop Movement button for built-in demo animation

The renderer creates VPython objects once on initialization,
then updates their positions/orientations each frame based on
forward kinematics output.
"""

import json
import math
import socket
import threading
import time
from typing import List

import numpy as np

try:
    import vpython as vp
except ImportError:
    raise ImportError(
        "vpython is required for 3D visualization. "
        "Install it with: pip install vpython"
    )

from robot.robot_model import RobotModel


class VPythonRenderer:
    """
    Real-time 3D renderer using VPython.

    Creates a browser-based 3D scene showing the robot arm,
    floor grid, and coordinate axes.
    """

    def __init__(self, robot: RobotModel):
        """
        Initialize the VPython scene and create all 3D objects.

        Args:
            robot: The RobotModel to visualize.
        """
        self._robot = robot

        # Compute total arm reach for sensible camera framing
        total_reach = sum(j.link_length for j in robot.joints) + robot.base_height
        half_reach = total_reach / 2.0

        # Set up the VPython scene
        self._scene = vp.canvas(
            title=(
                '<div style="background:linear-gradient(135deg, #1e1e24 0%, #2b2b36 100%); padding:15px 20px; border-radius:8px; margin-bottom:10px; font-family:Segoe UI, sans-serif; border-left: 5px solid #00ff88; box-shadow: 0 4px 6px rgba(0,0,0,0.3);">'
                '<h2 style="margin:0; color:#fff; letter-spacing: 1px;">'
                '  <span style="font-size:24px; margin-right:8px;">🤖</span> Real-Time Robotics Arm Simulator'
                '</h2>'
                f'<div style="color:#00ff88; margin-top:6px; font-size:14px; font-weight:bold;">● LIVE <span style="color:#aab; font-weight:normal; margin-left:8px;">Model: {robot.name} &mdash; {robot.description}</span></div>'
                '</div>'
            ),
            width=1000,
            height=540,
            center=vp.vector(0, half_reach, 0),
            background=vp.vector(0.08, 0.08, 0.11),
        )

        # Lock the view so it doesn't jump around
        self._scene.autoscale = False
        self._scene.range = total_reach * 1.1

        # Set a sensible camera angle (looking from front-right, slightly above)
        self._scene.forward = vp.vector(-1, -0.5, -1).norm()
        self._scene.up = vp.vector(0, 1, 0)

        # Lighting
        self._scene.lights = []
        vp.distant_light(direction=vp.vector(1, 1, 1), color=vp.vector(0.8, 0.8, 0.8))
        vp.distant_light(direction=vp.vector(-1, 0.5, -1), color=vp.vector(0.3, 0.3, 0.4))

        # Caption under the scene with controls help
        self._scene.caption = (
            '<div style="background:#1e1e24; padding:12px 20px; border-radius:8px; margin-top:10px; font-family:Segoe UI, sans-serif; color:#888; font-size:13px; box-shadow: 0 4px 6px rgba(0,0,0,0.3);">'
            '<b><span style="color:#aaa;">🖱️ Camera Controls:</span></b> &nbsp; '
            'Rotate: <span style="color:#fff;">Right-drag</span> &nbsp;|&nbsp; '
            'Zoom: <span style="color:#fff;">Scroll</span> &nbsp;|&nbsp; '
            'Pan: <span style="color:#fff;">Shift+drag</span>'
            '</div>'
        )

        # Create static scene elements
        self._create_floor()
        self._create_axes()

        # Create robot visual elements
        self._create_base()
        self._create_joints_and_links()
        self._create_end_effector()

        # Status label (attached to the scene, not the world)
        self._label = vp.label(
            pos=vp.vector(0, -0.3, 0),
            text='Waiting for UDP data...',
            height=12,
            color=vp.vector(0.9, 0.9, 0.9),
            box=False,
            opacity=0,
            line=False,
        )

        # ── Built-in demo animation ────────────────────────────────
        self._anim_running = False
        self._anim_thread = None
        self._udp_port = 9999  # default, updated by set_udp_port()

        self._scene.append_to_caption(
            '<div style="background:linear-gradient(135deg, #2b2b36 0%, #1e1e24 100%); padding:15px 20px; border-radius:8px; margin-top:10px; font-family:Segoe UI, sans-serif; box-shadow: 0 4px 6px rgba(0,0,0,0.3); display:inline-block; border-left: 5px solid #3498db;">'
            '<span style="color:#fff; font-size:15px; font-weight:bold; margin-right:20px;">⚙️ Control Panel</span>'
        )
        self._start_btn = vp.button(
            text='<b>&nbsp;&nbsp;▶ START MOVEMENT&nbsp;&nbsp;</b>',
            bind=self._on_button_click,
            background=vp.color.green,
            color=vp.color.white
        )
        self._scene.append_to_caption('&nbsp;&nbsp;&nbsp;&nbsp;<span style="color:#aaa; font-size:14px; font-weight:bold;">Animation Mode:</span> ')
        self._mode_menu = vp.menu(
            choices=['wave', 'sweep', 'random'],
            bind=lambda m: None,  # just stores selection
        )
        self._scene.append_to_caption('</div>\n')

    def set_udp_port(self, port: int):
        """Set the UDP port the built-in sender targets."""
        self._udp_port = port

    def _on_button_click(self, btn):
        """Toggle the built-in animation on/off."""
        if not self._anim_running:
            self._anim_running = True
            btn.text = '<b>&nbsp;&nbsp;■ STOP MOVEMENT&nbsp;&nbsp;</b>'
            try: btn.background = vp.color.red
            except Exception: pass
            
            mode = self._mode_menu.selected
            self._anim_thread = threading.Thread(
                target=self._run_animation, args=(mode,), daemon=True
            )
            self._anim_thread.start()
        else:
            self._anim_running = False
            btn.text = '<b>&nbsp;&nbsp;▶ START MOVEMENT&nbsp;&nbsp;</b>'
            try: btn.background = vp.color.green
            except Exception: pass

    def _run_animation(self, mode: str):
        """Send animated joint packets to the local UDP server."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        t = 0.0
        dt = 1.0 / 30.0  # 30 Hz
        try:
            while self._anim_running:
                if mode == 'sweep':
                    joints = self._sweep(t)
                elif mode == 'random':
                    joints = self._random(t)
                else:
                    joints = self._wave(t)

                msg = json.dumps({
                    "robot": self._robot.name,
                    "timestamp": int(time.time()),
                    "joints": joints,
                }).encode('utf-8')
                sock.sendto(msg, ('127.0.0.1', self._udp_port))
                time.sleep(dt)
                t += dt
        finally:
            sock.close()

    # ── Animation functions (same math as python_sender.py) ─────

    @staticmethod
    def _wave(t):
        return {
            "base_rotation": 45.0 * math.sin(t * 0.5),
            "shoulder":      30.0 * math.sin(t * 0.7 + 1.0),
            "elbow":         50.0 * math.sin(t * 1.1 + 2.0),
        }

    @staticmethod
    def _sweep(t):
        cycle = t % 9.0
        if cycle < 3.0:
            return {"base_rotation": 120.0 * math.sin((cycle / 3.0) * math.pi * 2),
                    "shoulder": 0.0, "elbow": 0.0}
        elif cycle < 6.0:
            return {"base_rotation": 0.0,
                    "shoulder": 60.0 * math.sin(((cycle - 3.0) / 3.0) * math.pi * 2),
                    "elbow": 0.0}
        else:
            return {"base_rotation": 0.0, "shoulder": 0.0,
                    "elbow": 90.0 * math.sin(((cycle - 6.0) / 3.0) * math.pi * 2)}

    _rand_targets = {"base_rotation": 0.0, "shoulder": 0.0, "elbow": 0.0}
    _rand_current = {"base_rotation": 0.0, "shoulder": 0.0, "elbow": 0.0}
    _rand_last = 0.0

    def _random(self, t):
        import random
        if t - self._rand_last > 2.0:
            self._rand_targets = {
                "base_rotation": random.uniform(-120, 120),
                "shoulder": random.uniform(-60, 60),
                "elbow": random.uniform(-90, 90),
            }
            self._rand_last = t
        for k in self._rand_current:
            self._rand_current[k] += 0.05 * (self._rand_targets[k] - self._rand_current[k])
        return dict(self._rand_current)

    def _create_floor(self):
        """Draw a grid floor at y=0."""
        floor_size = 6
        floor_half = floor_size / 2

        # Main floor surface
        vp.box(
            pos=vp.vector(0, -0.025, 0),
            size=vp.vector(floor_size, 0.01, floor_size),
            color=vp.vector(0.22, 0.22, 0.25),
        )

        # Grid lines — thinner, subtler
        grid_color = vp.vector(0.30, 0.30, 0.33)
        for i in range(-floor_size // 2, floor_size // 2 + 1):
            vp.curve(
                pos=[
                    vp.vector(i, -0.019, -floor_half),
                    vp.vector(i, -0.019, floor_half),
                ],
                color=grid_color,
                radius=0.003,
            )
            vp.curve(
                pos=[
                    vp.vector(-floor_half, -0.019, i),
                    vp.vector(floor_half, -0.019, i),
                ],
                color=grid_color,
                radius=0.003,
            )

    def _create_axes(self):
        """Draw XYZ axis indicators at the origin."""
        axis_len = 0.5
        shaft = 0.015
        vp.arrow(
            pos=vp.vector(0, 0.005, 0),
            axis=vp.vector(axis_len, 0, 0),
            color=vp.vector(0.9, 0.2, 0.2),
            shaftwidth=shaft,
        )
        vp.arrow(
            pos=vp.vector(0, 0.005, 0),
            axis=vp.vector(0, axis_len, 0),
            color=vp.vector(0.2, 0.9, 0.2),
            shaftwidth=shaft,
        )
        vp.arrow(
            pos=vp.vector(0, 0.005, 0),
            axis=vp.vector(0, 0, axis_len),
            color=vp.vector(0.2, 0.4, 0.9),
            shaftwidth=shaft,
        )

        # Axis labels
        label_offset = 0.12
        vp.label(pos=vp.vector(axis_len + label_offset, 0, 0),
                 text='X', height=10, color=vp.vector(0.9, 0.2, 0.2),
                 box=False, opacity=0, line=False)
        vp.label(pos=vp.vector(0, axis_len + label_offset, 0),
                 text='Y', height=10, color=vp.vector(0.2, 0.9, 0.2),
                 box=False, opacity=0, line=False)
        vp.label(pos=vp.vector(0, 0, axis_len + label_offset),
                 text='Z', height=10, color=vp.vector(0.2, 0.4, 0.9),
                 box=False, opacity=0, line=False)

    def _create_base(self):
        """Create the base platform box."""
        r = self._robot
        bc = r.base_color
        self._base_box = vp.box(
            pos=vp.vector(
                r.base_position[0],
                r.base_position[1] + r.base_height / 2,
                r.base_position[2],
            ),
            size=vp.vector(r.base_width, r.base_height, r.base_depth),
            color=vp.vector(bc[0], bc[1], bc[2]),
        )

    def _create_joints_and_links(self):
        """Create sphere + cylinder for each joint/link pair."""
        self._joint_spheres = []
        self._link_cylinders = []

        for joint in self._robot.joints:
            jc = joint.joint_color
            sphere = vp.sphere(
                pos=vp.vector(0, 0, 0),
                radius=joint.joint_radius,
                color=vp.vector(jc[0], jc[1], jc[2]),
                shininess=0.6,
            )
            self._joint_spheres.append(sphere)

            lc = joint.link_color
            cyl = vp.cylinder(
                pos=vp.vector(0, 0, 0),
                axis=vp.vector(0, 1, 0),
                radius=joint.link_radius,
                color=vp.vector(lc[0], lc[1], lc[2]),
            )
            self._link_cylinders.append(cyl)

    def _create_end_effector(self):
        """Create the end effector cone."""
        ee = self._robot.end_effector_config
        ec = ee['color']
        self._end_effector = vp.cone(
            pos=vp.vector(0, 0, 0),
            axis=vp.vector(0, ee['length'], 0),
            radius=ee['radius'],
            color=vp.vector(ec[0], ec[1], ec[2]),
        )

    def update(self, positions: List[np.ndarray],
               orientations: List[np.ndarray]):
        """
        Update all 3D objects based on forward kinematics results.

        Args:
            positions: List of N+1 position vectors from FK.
            orientations: List of N+1 rotation matrices from FK.
        """
        num_joints = len(self._robot.joints)

        for i in range(num_joints):
            p_start = positions[i]
            p_end = positions[i + 1]

            # Update joint sphere position
            self._joint_spheres[i].pos = vp.vector(
                float(p_start[0]), float(p_start[1]), float(p_start[2])
            )

            # Update link cylinder: from joint to next position
            link_vec = p_end - p_start
            self._link_cylinders[i].pos = vp.vector(
                float(p_start[0]), float(p_start[1]), float(p_start[2])
            )
            self._link_cylinders[i].axis = vp.vector(
                float(link_vec[0]), float(link_vec[1]), float(link_vec[2])
            )

        # Update end effector at the last position
        ee_pos = positions[-1]
        ee_rot = orientations[-1]
        # End effector extends in the local +Y direction
        ee_cfg = self._robot.end_effector_config
        ee_dir = ee_rot @ np.array([0.0, ee_cfg['length'], 0.0])
        self._end_effector.pos = vp.vector(
            float(ee_pos[0]), float(ee_pos[1]), float(ee_pos[2])
        )
        self._end_effector.axis = vp.vector(
            float(ee_dir[0]), float(ee_dir[1]), float(ee_dir[2])
        )

    def update_status(self, text: str):
        """Update the status label text."""
        self._label.text = text
