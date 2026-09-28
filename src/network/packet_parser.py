"""
UDP packet parser for the robotics simulator protocol.

Wire Protocol (JSON over UDP):
    Each UDP datagram contains a UTF-8 encoded JSON object:

    {
        "robot": "demo_arm",            // robot config name
        "timestamp": 1695900000,        // epoch timestamp (integer)
        "joints": {                     // joint-name → angle (degrees)
            "base_rotation": 30.0,
            "shoulder": 45.0,
            "elbow": -20.0
        }
    }

    This format is C-compatible: a C application can construct this
    string with sprintf() and send it with sendto().

    Maximum datagram size: 4096 bytes.
"""

import json
from dataclasses import dataclass
from typing import Dict, Optional


@dataclass
class JointUpdate:
    """Parsed joint-angle update from a UDP packet."""
    robot: str
    timestamp: int
    joints: Dict[str, float]


def parse_packet(data: bytes) -> Optional[JointUpdate]:
    """
    Parse raw UDP bytes into a JointUpdate.

    Handles:
        - Invalid UTF-8 encoding
        - Malformed JSON
        - Missing fields (uses defaults)
        - Non-numeric joint values (skipped)

    Args:
        data: Raw bytes received from the UDP socket.

    Returns:
        A JointUpdate if parsing succeeds, None otherwise.
    """
    try:
        # Decode UTF-8 bytes to string
        text = data.decode('utf-8').strip()
        if not text:
            return None

        # Parse JSON
        msg = json.loads(text)

        if not isinstance(msg, dict):
            return None

        # Extract fields with safe defaults
        robot = str(msg.get('robot', ''))
        timestamp = int(msg.get('timestamp', 0))
        raw_joints = msg.get('joints', {})

        if not isinstance(raw_joints, dict):
            return None

        # Validate each joint value is a number
        joints: Dict[str, float] = {}
        for name, angle in raw_joints.items():
            if isinstance(angle, (int, float)) and not isinstance(angle, bool):
                joints[str(name)] = float(angle)
            # Non-numeric values are silently skipped

        if not joints:
            return None

        return JointUpdate(
            robot=robot,
            timestamp=timestamp,
            joints=joints,
        )

    except (UnicodeDecodeError, json.JSONDecodeError, ValueError, TypeError):
        return None
