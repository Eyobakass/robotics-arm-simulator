"""
Tests for the configuration loader.

Tests cover:
    - Loading valid JSON configuration
    - Missing required fields
    - Duplicate joint names
    - Invalid axes
    - Default values
    - Loading from string
"""

import sys
import os
import json
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from robot.config_loader import load_config, load_config_from_string, RobotConfig


class TestConfigLoader(unittest.TestCase):
    """Test the configuration loader."""

    VALID_CONFIG = {
        "name": "test_arm",
        "description": "Test robot",
        "base": {
            "position": [0.0, 0.0, 0.0],
            "width": 1.0,
            "height": 0.3,
            "depth": 1.0,
            "color": [0.5, 0.5, 0.5],
        },
        "joints": [
            {
                "name": "j1",
                "axis": "y",
                "link_length": 1.0,
                "min_angle": -180.0,
                "max_angle": 180.0,
                "initial_angle": 0.0,
            },
            {
                "name": "j2",
                "axis": "x",
                "link_length": 0.8,
                "min_angle": -90.0,
                "max_angle": 90.0,
                "initial_angle": 0.0,
            },
        ],
        "end_effector": {
            "radius": 0.1,
            "length": 0.2,
            "color": [0.0, 1.0, 0.0],
        },
    }

    def test_load_valid_config_from_string(self):
        """Valid JSON should parse into a RobotConfig."""
        config = load_config_from_string(json.dumps(self.VALID_CONFIG))
        self.assertIsInstance(config, RobotConfig)
        self.assertEqual(config.name, "test_arm")
        self.assertEqual(len(config.joints), 2)
        self.assertEqual(config.joints[0]['name'], 'j1')
        self.assertEqual(config.joints[1]['axis'], 'x')

    def test_load_valid_config_from_file(self):
        """Valid JSON file should load correctly."""
        with tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False
        ) as f:
            json.dump(self.VALID_CONFIG, f)
            f.flush()
            config = load_config(f.name)

        os.unlink(f.name)
        self.assertEqual(config.name, "test_arm")
        self.assertEqual(len(config.joints), 2)

    def test_file_not_found(self):
        """Missing file should raise FileNotFoundError."""
        with self.assertRaises(FileNotFoundError):
            load_config("/nonexistent/path/robot.json")

    def test_missing_name(self):
        """Config without 'name' should raise ValueError."""
        bad = {"joints": [{"name": "j1", "axis": "x", "link_length": 1.0}]}
        with self.assertRaises(ValueError):
            load_config_from_string(json.dumps(bad))

    def test_missing_joints(self):
        """Config without 'joints' should raise ValueError."""
        bad = {"name": "arm"}
        with self.assertRaises(ValueError):
            load_config_from_string(json.dumps(bad))

    def test_empty_joints_list(self):
        """Empty joints list should raise ValueError."""
        bad = {"name": "arm", "joints": []}
        with self.assertRaises(ValueError):
            load_config_from_string(json.dumps(bad))

    def test_duplicate_joint_names(self):
        """Duplicate joint names should raise ValueError."""
        bad = {
            "name": "arm",
            "joints": [
                {"name": "j1", "axis": "x", "link_length": 1.0},
                {"name": "j1", "axis": "y", "link_length": 0.5},
            ],
        }
        with self.assertRaises(ValueError):
            load_config_from_string(json.dumps(bad))

    def test_invalid_joint_axis(self):
        """Invalid axis should raise ValueError."""
        bad = {
            "name": "arm",
            "joints": [
                {"name": "j1", "axis": "w", "link_length": 1.0},
            ],
        }
        with self.assertRaises(ValueError):
            load_config_from_string(json.dumps(bad))

    def test_negative_link_length(self):
        """Negative link_length should raise ValueError."""
        bad = {
            "name": "arm",
            "joints": [
                {"name": "j1", "axis": "x", "link_length": -1.0},
            ],
        }
        with self.assertRaises(ValueError):
            load_config_from_string(json.dumps(bad))

    def test_defaults_applied(self):
        """Missing optional fields should use defaults."""
        minimal = {
            "name": "arm",
            "joints": [
                {"name": "j1", "axis": "x", "link_length": 1.0},
            ],
        }
        config = load_config_from_string(json.dumps(minimal))
        self.assertEqual(config.description, '')
        self.assertEqual(config.base_position, [0.0, 0.0, 0.0])
        self.assertEqual(config.base_height, 0.3)
        # Joint defaults
        j = config.joints[0]
        self.assertEqual(j['min_angle'], -180.0)
        self.assertEqual(j['max_angle'], 180.0)
        self.assertEqual(j['initial_angle'], 0.0)

    def test_base_color_loaded(self):
        """Base color from config should be loaded."""
        config = load_config_from_string(json.dumps(self.VALID_CONFIG))
        self.assertEqual(config.base_color, [0.5, 0.5, 0.5])

    def test_end_effector_defaults(self):
        """Missing end_effector should use defaults."""
        minimal = {
            "name": "arm",
            "joints": [{"name": "j1", "axis": "x", "link_length": 1.0}],
        }
        config = load_config_from_string(json.dumps(minimal))
        self.assertIn('radius', config.end_effector)
        self.assertIn('length', config.end_effector)

    def test_invalid_json(self):
        """Invalid JSON string should raise json.JSONDecodeError."""
        with self.assertRaises(json.JSONDecodeError):
            load_config_from_string("{not valid json}")

    def test_load_actual_demo_config(self):
        """The actual demo_arm.json should load without errors."""
        config_path = os.path.join(
            os.path.dirname(__file__), '..', 'config', 'demo_arm.json'
        )
        if os.path.exists(config_path):
            config = load_config(config_path)
            self.assertEqual(config.name, "demo_arm")
            self.assertEqual(len(config.joints), 3)


if __name__ == '__main__':
    unittest.main()
