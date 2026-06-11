"""Rby1DrakeVirtualMoman — RBY1 virtual robot backed by the Drake sim server.

Drop-in replacement for Rby1MujocoVirtualMoman: same wire protocol, same conf
format, different physics backend.  Scene objects must be URDF/SDF files
(Drake cannot load MJCF objects).
"""

from __future__ import annotations

from qr_robots.common import rby1
from qr_robots.common.zmq_virtual_moman import ZmqVirtualMoman


class Rby1DrakeVirtualMoman(ZmqVirtualMoman):
    def __init__(self, objects: dict | None = None, model_dir: str = ".",
                 mode: str = "headless", port: int = rby1.DRAKE_PORT,
                 host: str | None = None):
        super().__init__(
            robot_name=rby1.ROBOT_NAME,
            server_module="qr_robots.drake.rby1.sim",
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
            host=host,
        )
