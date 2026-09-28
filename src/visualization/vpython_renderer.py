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


# ── Palette (GitHub Dark) ──────────────────────────────────────────────────────
_BG        = '#0d1117'
_PANEL     = '#161b22'
_BORDER    = '#30363d'
_TEXT      = '#e6edf3'
_MUTED     = '#8b949e'
_BLUE      = '#58a6ff'
_GREEN     = '#3fb950'
_RED       = '#f85149'
_ORANGE    = '#d29922'
_MONO      = "Consolas, 'Courier New', monospace"
_SANS      = "'Segoe UI', Arial, sans-serif"


def _pill(text, bg, fg='#fff'):
    return (
        f'<span style="background:{bg};color:{fg};font-size:10px;font-weight:700;'
        f'border-radius:20px;padding:2px 8px;letter-spacing:0.4px;'
        f'font-family:{_SANS};">{text}</span>'
    )


def _tag(label, value, vc=_TEXT):
    return (
        f'<span style="color:{_MUTED};font-size:11px;">{label}&nbsp;</span>'
        f'<span style="color:{vc};font-size:11px;font-weight:600;">{value}</span>'
    )


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
        half_reach  = total_reach / 2.0
        dof         = len(robot.joints)
        joint_names = [j.name for j in robot.joints]

        # ── Title bar ────────────────────────────────────────────────────────
        title_html = (
            f'<div style="background:{_PANEL};border-bottom:1px solid {_BORDER};'
            f'padding:11px 18px;font-family:{_SANS};">'

            # Row 1 – app name + badges
            f'<div style="display:flex;align-items:center;gap:10px;margin-bottom:5px;">'
            f'<span style="color:{_TEXT};font-size:16px;font-weight:600;letter-spacing:0.2px;">'
            f'Robotic Arm Simulator</span>'
            + _pill('LIVE', _GREEN)
            + _pill(f'{dof}-DOF', '#1f6feb')
            + f'</div>'

            # Row 2 – technical metadata strip
            f'<div style="display:flex;gap:20px;">'
            + _tag('model:', robot.name, _BLUE)
            + '&nbsp;&nbsp;'
            + _tag('desc:', robot.description)
            + '&nbsp;&nbsp;'
            + _tag('joints:', ', '.join(joint_names))
            + f'</div>'

            f'</div>'
        )

        # ── Canvas ───────────────────────────────────────────────────────────
        self._scene = vp.canvas(
            title=title_html,
            width=1000,
            height=530,
            center=vp.vector(0, half_reach, 0),
            background=vp.vector(0.05, 0.05, 0.07),
        )

        self._scene.autoscale = False
        self._scene.range     = total_reach * 1.1
        self._scene.forward   = vp.vector(-1, -0.5, -1).norm()
        self._scene.up        = vp.vector(0, 1, 0)

        self._scene.lights = []
        vp.distant_light(direction=vp.vector(1, 1, 1),    color=vp.vector(0.85, 0.85, 0.85))
        vp.distant_light(direction=vp.vector(-1, 0.5, -1), color=vp.vector(0.25, 0.28, 0.35))

        # ── 3-D objects ──────────────────────────────────────────────────────
        self._create_floor()
        self._create_axes()
        self._create_base()
        self._create_joints_and_links()
        self._create_end_effector()

        # In-scene status label (shows packet info near the base)
        self._label = vp.label(
            pos=vp.vector(0, -0.35, 0),
            text='waiting for data',
            height=11,
            color=vp.vector(0.55, 0.65, 0.75),
            box=False, opacity=0, line=False,
        )

        # ── Built-in demo animation state ────────────────────────────────────
        self._anim_running = False
        self._anim_thread  = None
        self._udp_port     = 9999

        # ── Caption / control panel ──────────────────────────────────────────
        #
        # Layout (all in one dark panel below canvas):
        #
        #  [RUN]   Mode: [wave v]   |  Status: ---   Pkts: ---
        #  base_rotation: ---   shoulder: ---   elbow: ---
        #  Rotate: right-drag | Zoom: scroll | Pan: shift+drag
        #
        # wtext fields are used for the updatable parts.

        # -- Row 0: section header
        self._scene.caption = (
            f'<div style="background:{_PANEL};border:1px solid {_BORDER};'
            f'border-radius:6px;margin-top:8px;font-family:{_SANS};overflow:hidden;">'

            # Header strip
            f'<div style="background:{_BG};padding:6px 16px;border-bottom:1px solid {_BORDER};">'
            f'<span style="color:{_MUTED};font-size:11px;letter-spacing:0.8px;font-weight:600;">'
            f'CONTROL PANEL</span>'
            f'</div>'

            # Row 1: button + mode + live status
            f'<div style="padding:10px 16px;border-bottom:1px solid {_BORDER};'
            f'display:flex;align-items:center;gap:10px;">'
        )

        # Button (inserted inline into the div)
        self._start_btn = vp.button(
            text='  Run  ',
            bind=self._on_button_click,
            background=vp.color.green,
            color=vp.color.white,
        )

        self._scene.append_to_caption(
            f'&nbsp;&nbsp;'
            f'<span style="color:{_MUTED};font-size:12px;">Mode:</span>&nbsp;'
        )

        self._mode_menu = vp.menu(
            choices=['wave', 'sweep', 'random'],
            bind=lambda m: None,
        )

        self._scene.append_to_caption(
            f'&nbsp;&nbsp;&nbsp;'
            f'<span style="color:{_BORDER};font-size:14px;">|</span>&nbsp;&nbsp;&nbsp;'
            f'<span style="color:{_MUTED};font-size:12px;">Status:</span>&nbsp;'
        )
        self._status_wt = vp.wtext(
            text=f'<span style="color:{_MUTED};font-family:{_MONO};font-size:12px;">idle</span>'
        )

        self._scene.append_to_caption(
            f'&nbsp;&nbsp;&nbsp;'
            f'<span style="color:{_BORDER};font-size:14px;">|</span>&nbsp;&nbsp;&nbsp;'
            f'<span style="color:{_MUTED};font-size:12px;">Packets:</span>&nbsp;'
        )
        self._pkt_wt = vp.wtext(
            text=f'<span style="color:{_MUTED};font-family:{_MONO};font-size:12px;">0</span>'
        )
        self._pkt_count = 0

        self._scene.append_to_caption(
            f'</div>'   # close row 1

            # Row 2: live joint angles
            f'<div style="padding:8px 16px;border-bottom:1px solid {_BORDER};'
            f'background:{_BG};">'
            f'<span style="color:{_MUTED};font-size:11px;letter-spacing:0.6px;'
            f'font-weight:600;margin-right:12px;">JOINTS</span>'
        )

        # One wtext per joint for live angle display
        self._joint_wtexts = []
        for i, j in enumerate(robot.joints):
            if i > 0:
                self._scene.append_to_caption(
                    f'<span style="color:{_BORDER};margin:0 10px;">|</span>'
                )
            self._scene.append_to_caption(
                f'<span style="color:{_MUTED};font-size:11px;font-family:{_MONO};">'
                f'{j.name}:&nbsp;</span>'
            )
            wt = vp.wtext(
                text=f'<span style="color:{_TEXT};font-family:{_MONO};font-size:11px;"> 0.0&deg;</span>'
            )
            self._joint_wtexts.append(wt)

        self._scene.append_to_caption(
            f'</div>'   # close row 2

            # Row 3: camera controls
            f'<div style="padding:7px 16px;">'
            f'<span style="color:{_MUTED};font-size:11px;">'
            f'Rotate: <span style="color:{_TEXT};">right-drag</span>'
            f'&nbsp;&nbsp;/&nbsp;&nbsp;'
            f'Zoom: <span style="color:{_TEXT};">scroll</span>'
            f'&nbsp;&nbsp;/&nbsp;&nbsp;'
            f'Pan: <span style="color:{_TEXT};">shift + drag</span>'
            f'</span>'
            f'</div>'

            f'</div>'   # close outer panel
        )

    # ── Public API ────────────────────────────────────────────────────────────

    def set_udp_port(self, port: int):
        """Set the UDP port the built-in sender targets."""
        self._udp_port = port

    # ── Button / animation ────────────────────────────────────────────────────

    def _on_button_click(self, btn):
        """Toggle the built-in animation on/off."""
        if not self._anim_running:
            self._anim_running = True
            btn.text = '  Stop  '
            try:
                btn.background = vp.color.red
            except Exception:
                pass
            mode = self._mode_menu.selected
            self._anim_thread = threading.Thread(
                target=self._run_animation, args=(mode,), daemon=True
            )
            self._anim_thread.start()
        else:
            self._anim_running = False
            btn.text = '  Run  '
            try:
                btn.background = vp.color.green
            except Exception:
                pass

    def _run_animation(self, mode: str):
        """Send animated joint packets to the local UDP server."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        t  = 0.0
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
                    "robot":     self._robot.name,
                    "timestamp": int(time.time()),
                    "joints":    joints,
                }).encode('utf-8')
                sock.sendto(msg, ('127.0.0.1', self._udp_port))
                time.sleep(dt)
                t += dt
        finally:
            sock.close()

    # ── Animation functions (same math as python_sender.py) ──────────────────

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
    _rand_last    = 0.0

    def _random(self, t):
        import random
        if t - self._rand_last > 2.0:
            self._rand_targets = {
                "base_rotation": random.uniform(-120, 120),
                "shoulder":      random.uniform(-60,  60),
                "elbow":         random.uniform(-90,  90),
            }
            self._rand_last = t
        for k in self._rand_current:
            self._rand_current[k] += 0.05 * (self._rand_targets[k] - self._rand_current[k])
        return dict(self._rand_current)

    # ── Scene geometry ────────────────────────────────────────────────────────

    def _create_floor(self):
        """Draw a grid floor at y=0."""
        floor_size = 6
        floor_half = floor_size / 2

        vp.box(
            pos=vp.vector(0, -0.025, 0),
            size=vp.vector(floor_size, 0.01, floor_size),
            color=vp.vector(0.18, 0.18, 0.21),
        )

        grid_color = vp.vector(0.26, 0.26, 0.30)
        for i in range(-floor_size // 2, floor_size // 2 + 1):
            vp.curve(
                pos=[vp.vector(i, -0.019, -floor_half),
                     vp.vector(i, -0.019,  floor_half)],
                color=grid_color, radius=0.003,
            )
            vp.curve(
                pos=[vp.vector(-floor_half, -0.019, i),
                     vp.vector( floor_half, -0.019, i)],
                color=grid_color, radius=0.003,
            )

    def _create_axes(self):
        """Draw XYZ axis indicators at the origin."""
        axis_len = 0.5
        shaft    = 0.015
        vp.arrow(pos=vp.vector(0, 0.005, 0), axis=vp.vector(axis_len, 0, 0),
                 color=vp.vector(0.9, 0.2, 0.2), shaftwidth=shaft)
        vp.arrow(pos=vp.vector(0, 0.005, 0), axis=vp.vector(0, axis_len, 0),
                 color=vp.vector(0.2, 0.85, 0.2), shaftwidth=shaft)
        vp.arrow(pos=vp.vector(0, 0.005, 0), axis=vp.vector(0, 0, axis_len),
                 color=vp.vector(0.2, 0.45, 0.95), shaftwidth=shaft)

        offset = 0.12
        vp.label(pos=vp.vector(axis_len + offset, 0, 0), text='X', height=10,
                 color=vp.vector(0.9, 0.2, 0.2), box=False, opacity=0, line=False)
        vp.label(pos=vp.vector(0, axis_len + offset, 0), text='Y', height=10,
                 color=vp.vector(0.2, 0.85, 0.2), box=False, opacity=0, line=False)
        vp.label(pos=vp.vector(0, 0, axis_len + offset), text='Z', height=10,
                 color=vp.vector(0.2, 0.45, 0.95), box=False, opacity=0, line=False)

    def _create_base(self):
        """Create the base platform box."""
        r  = self._robot
        bc = r.base_color
        self._base_box = vp.box(
            pos=vp.vector(r.base_position[0],
                          r.base_position[1] + r.base_height / 2,
                          r.base_position[2]),
            size=vp.vector(r.base_width, r.base_height, r.base_depth),
            color=vp.vector(bc[0], bc[1], bc[2]),
        )

    def _create_joints_and_links(self):
        """Create sphere + cylinder for each joint/link pair."""
        self._joint_spheres   = []
        self._link_cylinders  = []

        for joint in self._robot.joints:
            jc     = joint.joint_color
            sphere = vp.sphere(
                pos=vp.vector(0, 0, 0),
                radius=joint.joint_radius,
                color=vp.vector(jc[0], jc[1], jc[2]),
                shininess=0.7,
            )
            self._joint_spheres.append(sphere)

            lc  = joint.link_color
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

    # ── Per-frame update ──────────────────────────────────────────────────────

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
            p_end   = positions[i + 1]

            self._joint_spheres[i].pos = vp.vector(
                float(p_start[0]), float(p_start[1]), float(p_start[2])
            )

            link_vec = p_end - p_start
            self._link_cylinders[i].pos = vp.vector(
                float(p_start[0]), float(p_start[1]), float(p_start[2])
            )
            self._link_cylinders[i].axis = vp.vector(
                float(link_vec[0]), float(link_vec[1]), float(link_vec[2])
            )

            # Update live joint angle display
            angle = self._robot.joints[i].angle
            color = _GREEN if abs(angle) < 5 else _ORANGE if abs(angle) > 70 else _TEXT
            self._joint_wtexts[i].text = (
                f'<span style="color:{color};font-family:{_MONO};'
                f'font-size:11px;">{angle:+.1f}&deg;</span>'
            )

        # Update end effector
        ee_pos = positions[-1]
        ee_rot = orientations[-1]
        ee_cfg = self._robot.end_effector_config
        ee_dir = ee_rot @ np.array([0.0, ee_cfg['length'], 0.0])
        self._end_effector.pos = vp.vector(
            float(ee_pos[0]), float(ee_pos[1]), float(ee_pos[2])
        )
        self._end_effector.axis = vp.vector(
            float(ee_dir[0]), float(ee_dir[1]), float(ee_dir[2])
        )

    def update_status(self, text: str):
        """Update the status line (called from main loop)."""
        # Update 3D scene label
        self._label.text = text

        # Update packet count and status wtext in caption
        self._pkt_count += 1
        mode  = self._mode_menu.selected if self._anim_running else 'external'
        state = self._anim_running

        status_color = _GREEN if state else _MUTED
        status_text  = f'running ({mode})' if state else 'receiving'

        self._status_wt.text = (
            f'<span style="color:{status_color};font-family:{_MONO};'
            f'font-size:12px;">{status_text}</span>'
        )
        self._pkt_wt.text = (
            f'<span style="color:{_BLUE};font-family:{_MONO};'
            f'font-size:12px;">{self._pkt_count}</span>'
        )
