"""Rby1MujocoVirtualMoman — RBY1 virtual robot backed by the MuJoCo sim server."""

from __future__ import annotations

import sys

from qr_robots.common import rby1
from qr_robots.common.zmq_virtual_moman import ZmqVirtualMoman


class Rby1MujocoVirtualMoman(ZmqVirtualMoman):
    def __init__(self, objects: dict | None = None, model_dir: str = ".",
                 mode: str = "headless", port: int = rby1.MUJOCO_PORT,
                 host: str | None = None):
        super().__init__(
            robot_name=rby1.ROBOT_NAME,
            server_module="qr_robots.mujoco.rby1.sim",
            port=port,
            parse_conf=rby1.parse_robot_vector,
            arm_sides=("right", "left"),
            gripper_sides=("right", "left"),
            has_torso=True,
            has_head=True,
            camera_names=tuple(rby1.CAMERA_NAMES),
            camera_params=rby1.CAMERA_PARAMS,
            objects=objects,
            model_dir=model_dir,
            mode=mode,
            # MuJoCo's passive viewer needs mjpython on macOS.
            use_mjpython=(mode == "display" and sys.platform == "darwin"),
            host=host,
        )
