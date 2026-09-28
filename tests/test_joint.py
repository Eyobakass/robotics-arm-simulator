"""
Tests for the Joint class.

Tests cover:
    - Initialization
    - Angle clamping
    - Limit validation
    - Invalid axis
    - Type checking
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from robot.joint import Joint


class TestJoint(unittest.TestCase):
    """Test the Joint class."""

    def _make_joint(self, **kwargs):
        """Helper to create a joint with sensible defaults."""
        defaults = {
            'name': 'test_joint',
            'axis': 'y',
            'link_length': 1.0,
            'min_angle': -90.0,
            'max_angle': 90.0,
            'initial_angle': 0.0,
        }
        defaults.update(kwargs)
        return Joint(**defaults)

    def test_basic_initialization(self):
        """Joint should initialize with correct values."""
        j = self._make_joint(name='shoulder', axis='x', link_length=1.5)
        self.assertEqual(j.name, 'shoulder')
        self.assertEqual(j.axis, 'x')
        self.assertEqual(j.link_length, 1.5)
        self.assertAlmostEqual(j.current_angle, 0.0)

    def test_initial_angle_within_limits(self):
        """Initial angle within limits should be set exactly."""
        j = self._make_joint(initial_angle=45.0)
        self.assertAlmostEqual(j.current_angle, 45.0)

    def test_initial_angle_clamped(self):
        """Initial angle outside limits should be clamped."""
        j = self._make_joint(initial_angle=180.0, max_angle=90.0)
        self.assertAlmostEqual(j.current_angle, 90.0)

    def test_set_angle_within_limits(self):
        """Setting an angle within limits should work."""
        j = self._make_joint()
        result = j.set_angle(45.0)
        self.assertAlmostEqual(result, 45.0)
        self.assertAlmostEqual(j.current_angle, 45.0)

    def test_set_angle_above_max(self):
        """Angles above max should be clamped."""
        j = self._make_joint(max_angle=90.0)
        result = j.set_angle(120.0)
        self.assertAlmostEqual(result, 90.0)
        self.assertAlmostEqual(j.current_angle, 90.0)

    def test_set_angle_below_min(self):
        """Angles below min should be clamped."""
        j = self._make_joint(min_angle=-90.0)
        result = j.set_angle(-120.0)
        self.assertAlmostEqual(result, -90.0)
        self.assertAlmostEqual(j.current_angle, -90.0)

    def test_set_angle_exact_min(self):
        """Exact min angle should be accepted."""
        j = self._make_joint(min_angle=-90.0)
        result = j.set_angle(-90.0)
        self.assertAlmostEqual(result, -90.0)

    def test_set_angle_exact_max(self):
        """Exact max angle should be accepted."""
        j = self._make_joint(max_angle=90.0)
        result = j.set_angle(90.0)
        self.assertAlmostEqual(result, 90.0)

    def test_set_angle_zero(self):
        """Zero angle should always work."""
        j = self._make_joint()
        result = j.set_angle(0.0)
        self.assertAlmostEqual(result, 0.0)

    def test_set_angle_type_error(self):
        """Non-numeric angle should raise TypeError."""
        j = self._make_joint()
        with self.assertRaises(TypeError):
            j.set_angle("hello")

    def test_invalid_axis(self):
        """Invalid axis should raise ValueError."""
        with self.assertRaises(ValueError):
            self._make_joint(axis='w')

    def test_invalid_min_max(self):
        """min_angle > max_angle should raise ValueError."""
        with self.assertRaises(ValueError):
            self._make_joint(min_angle=90.0, max_angle=-90.0)

    def test_negative_link_length(self):
        """Negative link_length should raise ValueError."""
        with self.assertRaises(ValueError):
            self._make_joint(link_length=-1.0)

    def test_zero_link_length(self):
        """Zero link_length should be valid (e.g., for pure rotation joints)."""
        j = self._make_joint(link_length=0.0)
        self.assertEqual(j.link_length, 0.0)

    def test_clamp_angle(self):
        """clamp_angle should work without changing current angle."""
        j = self._make_joint(min_angle=-45.0, max_angle=45.0)
        j.set_angle(10.0)
        clamped = j.clamp_angle(100.0)
        self.assertAlmostEqual(clamped, 45.0)
        # Current angle should be unchanged
        self.assertAlmostEqual(j.current_angle, 10.0)

    def test_visual_defaults(self):
        """Default visual properties should be set."""
        j = self._make_joint()
        self.assertIsInstance(j.link_color, list)
        self.assertIsInstance(j.joint_color, list)
        self.assertGreater(j.link_radius, 0)
        self.assertGreater(j.joint_radius, 0)

    def test_repr(self):
        """repr should include useful info."""
        j = self._make_joint(name='elbow')
        r = repr(j)
        self.assertIn('elbow', r)
        self.assertIn('Joint', r)


if __name__ == '__main__':
    unittest.main()
