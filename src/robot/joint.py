"""
Joint model for a single revolute joint in the robotic arm.

Each joint has a rotation axis, angular limits, and visual properties.
Joint angles are specified in degrees for human readability,
and converted to radians only when needed for kinematics.
"""


class Joint:
    """Represents a single revolute joint with limits and visual properties."""

    def __init__(self, name: str, axis: str, link_length: float,
                 min_angle: float, max_angle: float, initial_angle: float,
                 link_radius: float = 0.05, joint_radius: float = 0.08,
                 link_color: list = None, joint_color: list = None):
        """
        Initialize a Joint.

        Args:
            name: Unique identifier for this joint (e.g., "shoulder").
            axis: Rotation axis - "x", "y", or "z".
            link_length: Length of the link attached to this joint.
            min_angle: Minimum allowed angle in degrees.
            max_angle: Maximum allowed angle in degrees.
            initial_angle: Starting angle in degrees.
            link_radius: Visual radius of the cylindrical link.
            joint_radius: Visual radius of the joint sphere.
            link_color: RGB color [r, g, b] for the link (0.0-1.0).
            joint_color: RGB color [r, g, b] for the joint (0.0-1.0).
        """
        if axis not in ('x', 'y', 'z'):
            raise ValueError(f"Joint axis must be 'x', 'y', or 'z', got '{axis}'")
        if min_angle > max_angle:
            raise ValueError(
                f"min_angle ({min_angle}) must be <= max_angle ({max_angle})"
            )
        if link_length < 0:
            raise ValueError(f"link_length must be >= 0, got {link_length}")

        self.name = name
        self.axis = axis
        self.link_length = link_length
        self.min_angle = min_angle
        self.max_angle = max_angle
        self._current_angle = self.clamp_angle(initial_angle)

        # Visual properties
        self.link_radius = link_radius
        self.joint_radius = joint_radius
        self.link_color = link_color if link_color else [0.7, 0.7, 0.8]
        self.joint_color = joint_color if joint_color else [1.0, 0.5, 0.0]

    @property
    def current_angle(self) -> float:
        """Current angle in degrees."""
        return self._current_angle

    def set_angle(self, angle: float) -> float:
        """
        Set the joint angle, clamping to limits.

        Args:
            angle: Desired angle in degrees.

        Returns:
            The actual angle after clamping.
        """
        if not isinstance(angle, (int, float)):
            raise TypeError(f"Angle must be a number, got {type(angle).__name__}")
        self._current_angle = self.clamp_angle(float(angle))
        return self._current_angle

    def clamp_angle(self, angle: float) -> float:
        """Clamp an angle to this joint's limits."""
        return max(self.min_angle, min(self.max_angle, float(angle)))

    def __repr__(self) -> str:
        return (
            f"Joint(name='{self.name}', axis='{self.axis}', "
            f"angle={self._current_angle:.1f}°, "
            f"limits=[{self.min_angle:.1f}, {self.max_angle:.1f}])"
        )
