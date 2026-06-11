"""Shared Spot definitions: conf vector layout, camera params."""

from __future__ import annotations

import numpy as np

ROBOT_NAME = "spot"

MUJOCO_PORT = 5558
DRAKE_PORT = 5559

ARM_JOINT_NAMES = ("arm_sh0", "arm_sh1", "arm_el0", "arm_el1", "arm_wr0", "arm_wr1")
GRIPPER_JOINT_NAME = "arm_f1x"

# Layout of the canonical 10-element Spot conf vector:
# [base x, y, theta, arm(6), gripper(1)].
chain_slices: dict[str, slice] = dict(
    base=slice(0, 3),
    right=slice(3, 9),
    right_gripper=slice(9, 10),
)

# Gripper jaw length: opening = -sin(angle) * length.
GRIPPER_JAW_LENGTH = 0.12


def gripper_angle_to_opening(angle: float) -> float:
    return -np.sin(angle) * GRIPPER_JAW_LENGTH


def gripper_opening_to_angle(opening: float) -> float:
    return -np.arcsin(np.clip(opening / GRIPPER_JAW_LENGTH, -1.0, 1.0))


def parse_robot_vector(robot_vector: np.ndarray) -> dict[str, np.ndarray]:
    """Canonical 10-vector → chain dict; gripper angle reduced to an opening."""
    conf = {ch: np.asarray(robot_vector[sl]) for ch, sl in chain_slices.items()}
    conf["right_gripper"] = np.array(
        [gripper_angle_to_opening(float(conf["right_gripper"][0]))]
    )
    return conf


# (FOV degrees, height, width, min_depth, max_depth)
CAMERA_PARAMS = (60.0, 480, 640, 0.05, 3.0)
CAMERA_NAMES = ["hand_camera"]
