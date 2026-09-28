"""
Tests for the UDP packet parser.

Tests cover:
    - Valid JSON packets
    - Missing fields
    - Malformed JSON
    - Invalid UTF-8
    - Non-numeric joint values
    - Empty packets
    - Edge cases
"""

import sys
import os
import unittest

# Add src/ to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from network.packet_parser import parse_packet, JointUpdate


class TestParsePacket(unittest.TestCase):
    """Test the parse_packet function."""

    def test_valid_packet(self):
        """A well-formed packet should parse correctly."""
        data = b'{"robot":"demo_arm","timestamp":123456,"joints":{"base_rotation":30.0,"shoulder":45.0}}'
        result = parse_packet(data)
        self.assertIsNotNone(result)
        self.assertEqual(result.robot, "demo_arm")
        self.assertEqual(result.timestamp, 123456)
        self.assertAlmostEqual(result.joints["base_rotation"], 30.0)
        self.assertAlmostEqual(result.joints["shoulder"], 45.0)

    def test_valid_packet_with_integers(self):
        """Integer joint values should be accepted and converted to float."""
        data = b'{"robot":"arm","timestamp":0,"joints":{"j1":45}}'
        result = parse_packet(data)
        self.assertIsNotNone(result)
        self.assertIsInstance(result.joints["j1"], float)
        self.assertAlmostEqual(result.joints["j1"], 45.0)

    def test_valid_packet_negative_angles(self):
        """Negative angles should be accepted."""
        data = b'{"robot":"arm","timestamp":0,"joints":{"j1":-90.5}}'
        result = parse_packet(data)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.joints["j1"], -90.5)

    def test_missing_robot_field(self):
        """Missing 'robot' field should use empty string default."""
        data = b'{"timestamp":0,"joints":{"j1":10.0}}'
        result = parse_packet(data)
        self.assertIsNotNone(result)
        self.assertEqual(result.robot, "")

    def test_missing_timestamp(self):
        """Missing 'timestamp' should default to 0."""
        data = b'{"robot":"arm","joints":{"j1":10.0}}'
        result = parse_packet(data)
        self.assertIsNotNone(result)
        self.assertEqual(result.timestamp, 0)

    def test_empty_joints(self):
        """Empty joints dict should return None (no valid joints)."""
        data = b'{"robot":"arm","timestamp":0,"joints":{}}'
        result = parse_packet(data)
        self.assertIsNone(result)

    def test_missing_joints_field(self):
        """Missing 'joints' field should return None."""
        data = b'{"robot":"arm","timestamp":0}'
        result = parse_packet(data)
        self.assertIsNone(result)

    def test_malformed_json(self):
        """Invalid JSON should return None."""
        data = b'{"robot": "arm", INVALID}'
        result = parse_packet(data)
        self.assertIsNone(result)

    def test_not_json_object(self):
        """A JSON array (not object) should return None."""
        data = b'[1, 2, 3]'
        result = parse_packet(data)
        self.assertIsNone(result)

    def test_invalid_utf8(self):
        """Invalid UTF-8 bytes should return None."""
        data = b'\xff\xfe\xfd'
        result = parse_packet(data)
        self.assertIsNone(result)

    def test_empty_bytes(self):
        """Empty bytes should return None."""
        data = b''
        result = parse_packet(data)
        self.assertIsNone(result)

    def test_non_numeric_joint_values_skipped(self):
        """Non-numeric joint values should be silently skipped."""
        data = b'{"robot":"arm","timestamp":0,"joints":{"j1":"hello","j2":45.0}}'
        result = parse_packet(data)
        self.assertIsNotNone(result)
        self.assertNotIn("j1", result.joints)
        self.assertAlmostEqual(result.joints["j2"], 45.0)

    def test_boolean_joint_values_skipped(self):
        """Boolean values should not be treated as numbers."""
        data = b'{"robot":"arm","timestamp":0,"joints":{"j1":true,"j2":10.0}}'
        result = parse_packet(data)
        self.assertIsNotNone(result)
        self.assertNotIn("j1", result.joints)

    def test_joints_not_dict(self):
        """If joints is not a dict, return None."""
        data = b'{"robot":"arm","timestamp":0,"joints":[1,2,3]}'
        result = parse_packet(data)
        self.assertIsNone(result)

    def test_whitespace_padding(self):
        """Whitespace around JSON should be tolerated."""
        data = b'  \n {"robot":"arm","timestamp":0,"joints":{"j1":1.0}}  \n'
        result = parse_packet(data)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.joints["j1"], 1.0)

    def test_all_joints_non_numeric(self):
        """If all joint values are non-numeric, return None."""
        data = b'{"robot":"arm","timestamp":0,"joints":{"j1":"bad","j2":null}}'
        result = parse_packet(data)
        self.assertIsNone(result)


if __name__ == '__main__':
    unittest.main()
