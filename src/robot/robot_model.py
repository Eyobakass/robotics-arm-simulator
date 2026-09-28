"""
Robot model that assembles joints into a complete articulated arm.

The RobotModel owns a list of Joint objects and provides methods
to update joint angles (from UDP messages) and query the current state.
"""

from typing import Dict, List, Optional

from robot.joint import Joint
from robot.config_loader import RobotConfig


class RobotModel:
    """
    Complete robot arm model built from a JSON configuration.

    Holds the kinematic chain (ordered list of joints) and base geometry.
    """

    def __init__(self, config: RobotConfig):
        """
        Build a robot model from a parsed configuration.

        Args:
            config: A validated RobotConfig from the config loader.
        """
        self._name = config.name
        self._description = config.description
        self._base_position = list(config.base_position)
        self._base_width = config.base_width
        self._base_height = config.base_height
        self._base_depth = config.base_depth
        self._base_color = list(config.base_color)
        self._end_effector_config = dict(config.end_effector)

        # Build Joint objects from config dictionaries
        self._joints: List[Joint] = []
        self._joint_map: Dict[str, Joint] = {}
        for jdata in config.joints:
            joint = Joint(
                name=jdata['name'],
                axis=jdata['axis'],
                link_length=jdata['link_length'],
                min_angle=jdata['min_angle'],
                max_angle=jdata['max_angle'],
                initial_angle=jdata['initial_angle'],
                link_radius=jdata['link_radius'],
                joint_radius=jdata['joint_radius'],
                link_color=jdata['link_color'],
                joint_color=jdata['joint_color'],
            )
            self._joints.append(joint)
            self._joint_map[joint.name] = joint

    @property
    def name(self) -> str:
        return self._name

    @property
    def description(self) -> str:
        return self._description

    @property
    def joints(self) -> List[Joint]:
        return self._joints

    @property
    def base_position(self) -> List[float]:
        return self._base_position

    @property
    def base_height(self) -> float:
        return self._base_height

    @property
    def base_width(self) -> float:
        return self._base_width

    @property
    def base_depth(self) -> float:
        return self._base_depth

    @property
    def base_color(self) -> List[float]:
        return self._base_color

    @property
    def end_effector_config(self) -> dict:
        return self._end_effector_config

    def get_joint_by_name(self, name: str) -> Optional[Joint]:
        """Look up a joint by name. Returns None if not found."""
        return self._joint_map.get(name)

    def get_joint_angles(self) -> Dict[str, float]:
        """Return a dict of {joint_name: current_angle_degrees}."""
        return {j.name: j.current_angle for j in self._joints}

    def apply_update(self, update) -> Dict[str, float]:
        """
        Apply a JointUpdate to this robot model.

        Only updates joints that exist in the model. Angles are clamped
        to each joint's limits. Unknown joint names are silently ignored.

        Args:
            update: A JointUpdate with a joints dict of {name: angle}.

        Returns:
            Dict of {joint_name: actual_angle} for all joints that were updated.
        """
        applied = {}
        for joint_name, angle in update.joints.items():
            joint = self._joint_map.get(joint_name)
            if joint is not None:
                actual = joint.set_angle(angle)
                applied[joint_name] = actual
        return applied

    def __repr__(self) -> str:
        joint_info = ", ".join(
            f"{j.name}={j.current_angle:.1f}°" for j in self._joints
        )
        return f"RobotModel(name='{self._name}', joints=[{joint_info}])"
