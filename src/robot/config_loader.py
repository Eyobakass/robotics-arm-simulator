"""
Configuration loader for JSON robot definition files.

Loads and validates robot geometry, joint definitions, and visual
properties from JSON configuration files. This makes the simulator
data-driven: swap the JSON file to simulate a different robot.
"""

import json
import os
from dataclasses import dataclass, field
from typing import List, Dict, Any


@dataclass
class RobotConfig:
    """Parsed and validated robot configuration."""
    name: str
    description: str
    base_position: List[float]
    base_width: float
    base_height: float
    base_depth: float
    base_color: List[float]
    joints: List[Dict[str, Any]]
    end_effector: Dict[str, Any]


def load_config(filepath: str) -> RobotConfig:
    """
    Load a robot configuration from a JSON file.

    Args:
        filepath: Path to the JSON configuration file.

    Returns:
        A validated RobotConfig dataclass.

    Raises:
        FileNotFoundError: If the config file doesn't exist.
        ValueError: If the config is missing required fields or has invalid values.
        json.JSONDecodeError: If the file contains invalid JSON.
    """
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Config file not found: {filepath}")

    with open(filepath, 'r', encoding='utf-8') as f:
        data = json.load(f)

    return _parse_config(data)


def load_config_from_string(json_string: str) -> RobotConfig:
    """
    Load a robot configuration from a JSON string.

    Useful for testing without filesystem access.

    Args:
        json_string: JSON string containing robot configuration.

    Returns:
        A validated RobotConfig dataclass.
    """
    data = json.loads(json_string)
    return _parse_config(data)


def _parse_config(data: dict) -> RobotConfig:
    """Parse and validate a configuration dictionary."""
    # Validate required top-level fields
    _require_fields(data, ['name', 'joints'], 'robot config')

    # Parse base configuration with defaults
    base = data.get('base', {})
    base_position = base.get('position', [0.0, 0.0, 0.0])
    base_width = float(base.get('width', 1.0))
    base_height = float(base.get('height', 0.3))
    base_depth = float(base.get('depth', 1.0))
    base_color = base.get('color', [0.4, 0.4, 0.45])

    # Validate and parse joints
    raw_joints = data['joints']
    if not isinstance(raw_joints, list) or len(raw_joints) == 0:
        raise ValueError("Config must define at least one joint")

    joints = []
    joint_names = set()
    for i, jdata in enumerate(raw_joints):
        joint = _parse_joint(jdata, i)
        if joint['name'] in joint_names:
            raise ValueError(f"Duplicate joint name: '{joint['name']}'")
        joint_names.add(joint['name'])
        joints.append(joint)

    # Parse end effector with defaults
    ee = data.get('end_effector', {})
    end_effector = {
        'radius': float(ee.get('radius', 0.10)),
        'length': float(ee.get('length', 0.20)),
        'color': ee.get('color', [0.0, 0.8, 0.2]),
    }

    return RobotConfig(
        name=str(data['name']),
        description=str(data.get('description', '')),
        base_position=[float(x) for x in base_position],
        base_width=base_width,
        base_height=base_height,
        base_depth=base_depth,
        base_color=[float(c) for c in base_color],
        joints=joints,
        end_effector=end_effector,
    )


def _parse_joint(jdata: dict, index: int) -> dict:
    """Parse and validate a single joint definition."""
    _require_fields(jdata, ['name', 'axis', 'link_length'], f'joint[{index}]')

    axis = str(jdata['axis']).lower()
    if axis not in ('x', 'y', 'z'):
        raise ValueError(
            f"Joint '{jdata['name']}' axis must be 'x', 'y', or 'z', got '{axis}'"
        )

    link_length = float(jdata['link_length'])
    if link_length < 0:
        raise ValueError(
            f"Joint '{jdata['name']}' link_length must be >= 0"
        )

    return {
        'name': str(jdata['name']),
        'axis': axis,
        'link_length': link_length,
        'min_angle': float(jdata.get('min_angle', -180.0)),
        'max_angle': float(jdata.get('max_angle', 180.0)),
        'initial_angle': float(jdata.get('initial_angle', 0.0)),
        'link_radius': float(jdata.get('link_radius', 0.05)),
        'joint_radius': float(jdata.get('joint_radius', 0.08)),
        'link_color': jdata.get('link_color', [0.7, 0.7, 0.8]),
        'joint_color': jdata.get('joint_color', [1.0, 0.5, 0.0]),
    }


def _require_fields(data: dict, fields: list, context: str):
    """Check that all required fields are present."""
    for field_name in fields:
        if field_name not in data:
            raise ValueError(f"Missing required field '{field_name}' in {context}")
