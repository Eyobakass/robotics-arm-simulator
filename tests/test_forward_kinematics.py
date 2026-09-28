"""
Tests for the forward kinematics engine.

Tests verify correct positions and orientations for known
configurations using manual trigonometric calculations.
"""

import sys
import os
import math
import unittest
import json

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from robot.config_loader import load_config_from_string
from robot.robot_model import RobotModel
from kinematics.forward_kinematics import (
    compute_forward_kinematics,
    rotation_matrix_x,
    rotation_matrix_y,
    rotation_matrix_z,
    get_rotation_matrix,
)


def _make_robot(joints_data, base_height=0.3):
    """Helper to create a RobotModel from joint definitions."""
    config_dict = {
        "name": "test",
        "base": {"height": base_height},
        "joints": joints_data,
    }
    config = load_config_from_string(json.dumps(config_dict))
    return RobotModel(config)


class TestRotationMatrices(unittest.TestCase):
    """Test individual rotation matrix functions."""

    def test_rx_zero(self):
        """Rx(0) should be the identity."""
        R = rotation_matrix_x(0.0)
        np.testing.assert_array_almost_equal(R, np.eye(3))

    def test_ry_zero(self):
        """Ry(0) should be the identity."""
        R = rotation_matrix_y(0.0)
        np.testing.assert_array_almost_equal(R, np.eye(3))

    def test_rz_zero(self):
        """Rz(0) should be the identity."""
        R = rotation_matrix_z(0.0)
        np.testing.assert_array_almost_equal(R, np.eye(3))

    def test_rx_90(self):
        """Rx(90°) should rotate Y-axis into Z-axis."""
        R = rotation_matrix_x(math.radians(90))
        # Y unit vector -> Z unit vector
        result = R @ np.array([0, 1, 0])
        np.testing.assert_array_almost_equal(result, [0, 0, 1])

    def test_ry_90(self):
        """Ry(90°) should rotate Z-axis into X-axis."""
        R = rotation_matrix_y(math.radians(90))
        result = R @ np.array([0, 0, 1])
        np.testing.assert_array_almost_equal(result, [1, 0, 0], decimal=10)

    def test_rz_90(self):
        """Rz(90°) should rotate X-axis into Y-axis."""
        R = rotation_matrix_z(math.radians(90))
        result = R @ np.array([1, 0, 0])
        np.testing.assert_array_almost_equal(result, [0, 1, 0])

    def test_rotation_determinant(self):
        """All rotation matrices should have determinant 1."""
        for angle in [0, 30, 45, 90, 180, -45]:
            rad = math.radians(angle)
            for fn in [rotation_matrix_x, rotation_matrix_y, rotation_matrix_z]:
                R = fn(rad)
                self.assertAlmostEqual(np.linalg.det(R), 1.0, places=10)

    def test_rotation_orthogonality(self):
        """R @ R^T should equal identity for any rotation matrix."""
        R = rotation_matrix_x(math.radians(37))
        np.testing.assert_array_almost_equal(R @ R.T, np.eye(3))

    def test_get_rotation_matrix_dispatch(self):
        """get_rotation_matrix should dispatch to correct function."""
        for axis in ['x', 'y', 'z']:
            R = get_rotation_matrix(axis, 45.0)
            self.assertEqual(R.shape, (3, 3))
            self.assertAlmostEqual(np.linalg.det(R), 1.0)

    def test_get_rotation_matrix_invalid_axis(self):
        """Invalid axis should raise ValueError."""
        with self.assertRaises(ValueError):
            get_rotation_matrix('w', 45.0)


class TestForwardKinematics(unittest.TestCase):
    """Test the full FK computation."""

    def test_single_joint_zero_angle(self):
        """Single joint at 0°: link should extend straight up."""
        robot = _make_robot([
            {"name": "j1", "axis": "y", "link_length": 2.0},
        ], base_height=0.0)

        positions, orientations = compute_forward_kinematics(robot)

        # positions[0] = base top = [0, 0, 0]
        # positions[1] = end of link = [0, 2, 0]
        self.assertEqual(len(positions), 2)
        np.testing.assert_array_almost_equal(positions[0], [0, 0, 0])
        np.testing.assert_array_almost_equal(positions[1], [0, 2, 0])

    def test_all_zeros_straight_up(self):
        """3-joint arm at all-zero angles: should extend straight up."""
        robot = _make_robot([
            {"name": "j1", "axis": "y", "link_length": 1.0},
            {"name": "j2", "axis": "x", "link_length": 1.0},
            {"name": "j3", "axis": "x", "link_length": 1.0},
        ], base_height=0.5)

        positions, orientations = compute_forward_kinematics(robot)

        self.assertEqual(len(positions), 4)
        # Base top
        np.testing.assert_array_almost_equal(positions[0], [0, 0.5, 0])
        # End of link 1
        np.testing.assert_array_almost_equal(positions[1], [0, 1.5, 0])
        # End of link 2
        np.testing.assert_array_almost_equal(positions[2], [0, 2.5, 0])
        # End of link 3 (end effector)
        np.testing.assert_array_almost_equal(positions[3], [0, 3.5, 0])

    def test_shoulder_90_degrees(self):
        """
        Shoulder (X-axis) at 90°: second link should point along +Z.

        Joint 1 (Y, 0°): link goes up [0, 1, 0] -> pos = [0, 1, 0]
        Joint 2 (X, 90°): Rx(90) @ [0, 1, 0] = [0, 0, 1]
                           -> pos = [0, 1, 0] + [0, 0, 1] = [0, 1, 1]
        """
        robot = _make_robot([
            {"name": "j1", "axis": "y", "link_length": 1.0},
            {"name": "j2", "axis": "x", "link_length": 1.0,
             "initial_angle": 90.0},
        ], base_height=0.0)

        positions, _ = compute_forward_kinematics(robot)

        np.testing.assert_array_almost_equal(positions[0], [0, 0, 0])
        np.testing.assert_array_almost_equal(positions[1], [0, 1, 0])
        np.testing.assert_array_almost_equal(positions[2], [0, 1, 1])

    def test_base_rotation_90(self):
        """
        Base rotation (Y, 90°) alone: link still goes straight up.

        Ry(90) @ [0, L, 0] = [0, L, 0] (Y rotation doesn't affect Y vector).
        """
        robot = _make_robot([
            {"name": "j1", "axis": "y", "link_length": 2.0,
             "initial_angle": 90.0},
        ], base_height=0.0)

        positions, _ = compute_forward_kinematics(robot)

        np.testing.assert_array_almost_equal(positions[0], [0, 0, 0])
        np.testing.assert_array_almost_equal(positions[1], [0, 2, 0])

    def test_base_rotation_affects_shoulder(self):
        """
        Base rotation 90° + shoulder 90°:
        Link 1: Ry(90) @ [0,1,0] = [0,1,0]
        Link 2: Ry(90) @ Rx(90) @ [0,1,0]
               = Ry(90) @ [0,0,1] = [1,0,0]
        """
        robot = _make_robot([
            {"name": "base", "axis": "y", "link_length": 1.0,
             "initial_angle": 90.0},
            {"name": "shoulder", "axis": "x", "link_length": 1.0,
             "initial_angle": 90.0},
        ], base_height=0.0)

        positions, _ = compute_forward_kinematics(robot)

        np.testing.assert_array_almost_equal(positions[0], [0, 0, 0])
        np.testing.assert_array_almost_equal(positions[1], [0, 1, 0])
        np.testing.assert_array_almost_equal(positions[2], [1, 1, 0])

    def test_orientations_are_rotation_matrices(self):
        """All returned orientations should be valid rotation matrices."""
        robot = _make_robot([
            {"name": "j1", "axis": "y", "link_length": 1.0,
             "initial_angle": 30.0},
            {"name": "j2", "axis": "x", "link_length": 1.0,
             "initial_angle": 45.0},
        ], base_height=0.0)

        _, orientations = compute_forward_kinematics(robot)

        for R in orientations:
            # Determinant should be 1
            self.assertAlmostEqual(np.linalg.det(R), 1.0, places=10)
            # R @ R^T should be identity
            np.testing.assert_array_almost_equal(R @ R.T, np.eye(3))

    def test_dynamic_angle_update(self):
        """FK should reflect joint angle changes after set_angle()."""
        robot = _make_robot([
            {"name": "j1", "axis": "y", "link_length": 1.0},
            {"name": "j2", "axis": "x", "link_length": 1.0},
        ], base_height=0.0)

        # Initially straight up
        pos1, _ = compute_forward_kinematics(robot)
        np.testing.assert_array_almost_equal(pos1[2], [0, 2, 0])

        # Now bend the shoulder
        robot.joints[1].set_angle(90.0)
        pos2, _ = compute_forward_kinematics(robot)
        np.testing.assert_array_almost_equal(pos2[2], [0, 1, 1])


if __name__ == '__main__':
    unittest.main()
