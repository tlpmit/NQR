"""Shared RBY1 ("rainbow") definitions: conf vector layout, camera params."""

from __future__ import annotations

import numpy as np

ROBOT_NAME = "rainbow"

MUJOCO_PORT = 5556
DRAKE_PORT = 5557   # different from MuJoCo's to allow both to run

# x-offset of the wheel-axle midpoint in the base body frame.
AXLE_X_OFFSET: float = 0.228

# Layout of the canonical 31-element RBY1 conf vector
# (base pose is the wheel-axle midpoint [x, y, theta]).
chain_slices: dict[str, slice] = dict(
    base=slice(0, 3),
    wheels=slice(3, 5),
    torso=slice(5, 11),
    right=slice(11, 18),
    right_gripper=slice(18, 20),
    left=slice(20, 27),
    left_gripper=slice(27, 29),
    head=slice(29, 31),
)


def parse_robot_vector(robot_vector: np.ndarray) -> dict[str, np.ndarray]:
    """Canonical 31-vector → chain dict; grippers reduced to a single opening."""
    conf = {ch: robot_vector[sl] for ch, sl in chain_slices.items() if ch != "wheels"}
    for gripper in ('right_gripper', 'left_gripper'):
        fp = conf[gripper]
        conf[gripper] = np.array([abs(fp[0] - fp[1])])
    return conf


# (FOV degrees, height, width, min_depth, max_depth)
CAMERA_PARAMS = (60.0, 720, 1280, 0.05, 3.0)
CAMERA_NAMES = ["head_camera"]
