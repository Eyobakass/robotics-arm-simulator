"""
Tests for the RobotModel class.

Tests cover:
    - Building from config
    - Joint lookup
    - Applying updates
    - Unknown joint handling
    - Angle clamping through the model
"""

import sys
import os
import json
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from robot.config_loader import load_config_from_string
from robot.robot_model import RobotModel
from network.packet_parser import JointUpdate


def _make_model(joints=None):
    """Helper to build a RobotModel from joint definitions."""
    if joints is None:
        joints = [
            {"name": "j1", "axis": "y", "link_length": 1.5,
             "min_angle": -180, "max_angle": 180, "initial_angle": 0},
            {"name": "j2", "axis": "x", "link_length": 1.0,
             "min_angle": -90, "max_angle": 90, "initial_angle": 10},
        ]
    config_dict = {"name": "test_arm", "joints": joints}
    config = load_config_from_string(json.dumps(config_dict))
    return RobotModel(config)


class TestRobotModel(unittest.TestCase):
    """Test the RobotModel class."""

    def test_build_from_config(self):
        """Model should build correctly from config."""
        model = _make_model()
        self.assertEqual(model.name, "test_arm")
        self.assertEqual(len(model.joints), 2)

    def test_joint_initial_angles(self):
        """Joints should have correct initial angles."""
        model = _make_model()
        angles = model.get_joint_angles()
        self.assertAlmostEqual(angles['j1'], 0.0)
        self.assertAlmostEqual(angles['j2'], 10.0)

    def test_get_joint_by_name(self):
        """Should find joints by name."""
        model = _make_model()
        j = model.get_joint_by_name('j1')
        self.assertIsNotNone(j)
        self.assertEqual(j.name, 'j1')

    def test_get_joint_by_name_not_found(self):
        """Missing joint name should return None."""
        model = _make_model()
        j = model.get_joint_by_name('nonexistent')
        self.assertIsNone(j)

    def test_apply_update(self):
        """apply_update should set joint angles."""
        model = _make_model()
        update = JointUpdate(
            robot="test_arm",
            timestamp=0,
            joints={"j1": 45.0, "j2": -30.0},
        )
        applied = model.apply_update(update)

        self.assertAlmostEqual(model.joints[0].current_angle, 45.0)
        self.assertAlmostEqual(model.joints[1].current_angle, -30.0)
        self.assertAlmostEqual(applied['j1'], 45.0)
        self.assertAlmostEqual(applied['j2'], -30.0)

    def test_apply_update_clamps_angles(self):
        """Angles exceeding limits should be clamped."""
        model = _make_model()
        update = JointUpdate(
            robot="test_arm",
            timestamp=0,
            joints={"j2": 200.0},  # j2 max is 90
        )
        applied = model.apply_update(update)

        self.assertAlmostEqual(model.joints[1].current_angle, 90.0)
        self.assertAlmostEqual(applied['j2'], 90.0)

    def test_apply_update_unknown_joints_ignored(self):
        """Unknown joint names should be silently ignored."""
        model = _make_model()
        update = JointUpdate(
            robot="test_arm",
            timestamp=0,
            joints={"unknown_joint": 45.0, "j1": 20.0},
        )
        applied = model.apply_update(update)

        self.assertNotIn('unknown_joint', applied)
        self.assertIn('j1', applied)
        self.assertAlmostEqual(applied['j1'], 20.0)

    def test_apply_update_partial(self):
        """Updating only some joints should leave others unchanged."""
        model = _make_model()
        update = JointUpdate(
            robot="test_arm",
            timestamp=0,
            joints={"j1": 60.0},  # only update j1
        )
        model.apply_update(update)

        self.assertAlmostEqual(model.joints[0].current_angle, 60.0)
        self.assertAlmostEqual(model.joints[1].current_angle, 10.0)  # unchanged

    def test_base_properties(self):
        """Base geometry properties should be accessible."""
        model = _make_model()
        self.assertIsInstance(model.base_position, list)
        self.assertGreater(model.base_height, 0)

    def test_end_effector_config(self):
        """End effector config should be accessible."""
        model = _make_model()
        ee = model.end_effector_config
        self.assertIn('radius', ee)
        self.assertIn('length', ee)

    def test_repr(self):
        """repr should be informative."""
        model = _make_model()
        r = repr(model)
        self.assertIn('test_arm', r)
        self.assertIn('j1', r)


if __name__ == '__main__':
    unittest.main()
