"""Virtual robots backed by the kinematic (Pinocchio) sim server.

Drop-in replacements for the MuJoCo/Drake virtual robots: cameras are
rendered through a MuJoCo render bridge synced to the kinematic state (same
image/intrinsics/extrinsics formats as the dynamics backends), gripper
close/open models grasping by attaching/detaching objects, and the client can
query collisions via robot.client.check_collisions().
"""

from __future__ import annotations

from qr_kinsim.sim import RBY1_KINSIM_PORT, SPOT_KINSIM_PORT
from qr_robots.common import rby1, spot
from qr_robots.common.zmq_virtual_moman import ZmqVirtualMoman


class Rby1KinsimVirtualMoman(ZmqVirtualMoman):
    def __init__(self, objects: dict | None = None, model_dir: str = ".",
                 mode: str = "headless", port: int = RBY1_KINSIM_PORT,
                 host: str | None = None):
        super().__init__(
            robot_name=rby1.ROBOT_NAME,
            server_module="qr_kinsim.sim",
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
            server_extra_args=["--robot", "rby1"],
        )


class SpotKinsimVirtualMoman(ZmqVirtualMoman):
    def __init__(self, objects: dict | None = None, model_dir: str = ".",
                 mode: str = "headless", port: int = SPOT_KINSIM_PORT,
                 host: str | None = None):
        super().__init__(
            robot_name=spot.ROBOT_NAME,
            server_module="qr_kinsim.sim",
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
            server_extra_args=["--robot", "spot"],
        )
