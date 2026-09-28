"""
Forward kinematics engine for serial robotic arm chains.

Computes the 3D position and orientation of each joint and link
by chaining rotation matrices along the kinematic chain.

Convention:
    - Each link extends along the local +Y axis (upward when arm is straight).
    - Joint rotations are applied around the joint's specified axis (x, y, or z)
      in the current local frame.
    - The cumulative rotation matrix tracks the orientation of each link frame.

Mathematics:
    For a joint rotating by angle θ around axis A:

    Rx(θ) = [[1,    0,      0   ],      Rotation around X axis
              [0,  cos(θ), -sin(θ)],
              [0,  sin(θ),  cos(θ)]]

    Ry(θ) = [[ cos(θ), 0, sin(θ)],      Rotation around Y axis
              [   0,    1,   0   ],
              [-sin(θ), 0, cos(θ)]]

    Rz(θ) = [[cos(θ), -sin(θ), 0],      Rotation around Z axis
              [sin(θ),  cos(θ), 0],
              [  0,       0,    1]]

    Position of link i endpoint:
        p_i = p_{i-1} + R_cumulative · [0, link_length, 0]

    Where R_cumulative = R_0 · R_1 · ... · R_i
"""

import math
from typing import List, Tuple

import numpy as np

from robot.robot_model import RobotModel


def rotation_matrix_x(theta_rad: float) -> np.ndarray:
    """3x3 rotation matrix around the X axis."""
    c, s = math.cos(theta_rad), math.sin(theta_rad)
    return np.array([
        [1.0, 0.0, 0.0],
        [0.0,   c,  -s],
        [0.0,   s,   c],
    ])


def rotation_matrix_y(theta_rad: float) -> np.ndarray:
    """3x3 rotation matrix around the Y axis."""
    c, s = math.cos(theta_rad), math.sin(theta_rad)
    return np.array([
        [ c,  0.0,  s],
        [0.0, 1.0, 0.0],
        [-s,  0.0,  c],
    ])


def rotation_matrix_z(theta_rad: float) -> np.ndarray:
    """3x3 rotation matrix around the Z axis."""
    c, s = math.cos(theta_rad), math.sin(theta_rad)
    return np.array([
        [ c,  -s, 0.0],
        [ s,   c, 0.0],
        [0.0, 0.0, 1.0],
    ])


def get_rotation_matrix(axis: str, angle_deg: float) -> np.ndarray:
    """
    Get a rotation matrix for a given axis and angle.

    Args:
        axis: "x", "y", or "z".
        angle_deg: Rotation angle in degrees.

    Returns:
        3x3 numpy rotation matrix.
    """
    theta = math.radians(angle_deg)
    if axis == 'x':
        return rotation_matrix_x(theta)
    elif axis == 'y':
        return rotation_matrix_y(theta)
    elif axis == 'z':
        return rotation_matrix_z(theta)
    else:
        raise ValueError(f"Unknown axis: '{axis}'")


def compute_forward_kinematics(
    robot: RobotModel,
) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    """
    Compute forward kinematics for the entire robot arm.

    Walks the kinematic chain from the base, applying each joint's
    rotation and translating along each link to find all positions.

    Args:
        robot: A RobotModel with joints and base geometry.

    Returns:
        A tuple of (positions, orientations):
            positions:    List of (N+1) 3D position vectors (numpy arrays).
                          positions[0] = top of base (first joint location).
                          positions[i+1] = end of link i (next joint location).
                          positions[N] = end effector position.
            orientations: List of (N+1) 3x3 rotation matrices.
                          orientations[i] = cumulative orientation at position i.
    """
    # Starting position: top of the base
    base = np.array(robot.base_position, dtype=float)
    base_top = base + np.array([0.0, robot.base_height, 0.0])

    positions = [base_top.copy()]
    orientations = [np.eye(3)]

    current_pos = base_top.copy()
    current_rot = np.eye(3)

    # Walk the kinematic chain
    for joint in robot.joints:
        # Apply this joint's rotation to the cumulative frame
        R_joint = get_rotation_matrix(joint.axis, joint.current_angle)
        current_rot = current_rot @ R_joint

        # The link extends along the local +Y axis
        local_link = np.array([0.0, joint.link_length, 0.0])
        world_link = current_rot @ local_link

        # Move to the end of this link
        current_pos = current_pos + world_link

        positions.append(current_pos.copy())
        orientations.append(current_rot.copy())

    return positions, orientations
