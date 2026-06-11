"""SpotDrakeVirtualMoman — Spot virtual robot backed by the Drake sim server."""

from __future__ import annotations

from qr_robots.common import spot
from qr_robots.common.zmq_virtual_moman import ZmqVirtualMoman


class SpotDrakeVirtualMoman(ZmqVirtualMoman):
    def __init__(self, objects: dict | None = None, model_dir: str = ".",
                 mode: str = "headless", port: int = spot.DRAKE_PORT,
                 host: str | None = None):
        super().__init__(
            robot_name=spot.ROBOT_NAME,
            server_module="qr_robots.drake.spot.sim",
            port=port,
            parse_conf=spot.parse_robot_vector,
            arm_sides=("right",),
            gripper_sides=("right",),
            has_torso=False,
            has_head=False,
            camera_names=tuple(spot.CAMERA_NAMES),
            camera_params=spot.CAMERA_PARAMS,
            objects=objects,
            model_dir=model_dir,
            mode=mode,
            host=host,
        )
