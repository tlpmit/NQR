"""
ZmqVirtualMoman — a VirtualMomanModule backed by a sim server subprocess.

All concrete sim-backed robots (RBY1/Spot × MuJoCo/Drake/kinematic) are thin
configurations of this class: they choose the server module, port, chain
layout, and conf parsing.
"""

from __future__ import annotations

from typing import Callable, Optional

import numpy as np

from qr_api.virtual_moman_interfaces import VirtualMomanModule
from qr_robots.common.launch import connect_client, shutdown_server, start_sim_server
from qr_robots.common.zmq_robot_modules import (
    ZmqBaseControlModule,
    ZmqCameraModule,
    ZmqChainControlModule,
    ZmqGripperControlModule,
    ZmqRobotStateModule,
    ZmqWorldStateModule,
)
from qr_utils.colors import DEFAULT_COLORS


class ZmqVirtualMoman(VirtualMomanModule):
    def __init__(
        self,
        robot_name: str,
        server_module: str,
        port: int,
        parse_conf: Callable[[np.ndarray], dict],
        arm_sides: tuple[str, ...] = ("right", "left"),
        gripper_sides: tuple[str, ...] = ("right", "left"),
        has_torso: bool = True,
        has_head: bool = True,
        camera_names: tuple[str, ...] = (),
        camera_params=None,
        objects: dict | None = None,
        model_dir: str = ".",
        mode: str = "headless",
        use_mjpython: bool = False,
        host: Optional[str] = None,
        server_extra_args: Optional[list[str]] = None,
    ):
        """
        Args:
            robot_name: Canonical robot name (e.g. "rainbow", "spot").
            server_module: Python module to launch as the sim server
                (e.g. "qr_robots.mujoco.rby1.sim").  Ignored when *host* is
                given, in which case we connect to an already-running server.
            port: ZMQ port for the sim server.
            parse_conf: Maps the robot's raw conf vector to a chain dict.
            objects: Scene-object dict passed to the sim server (see
                qr_robots.mujoco.sim_base for the format).
            mode: "headless" or "display".
            use_mjpython: Launch the server with mjpython (required for MuJoCo
                display mode on macOS).
        """
        self.real_robot = False
        self.robot_name = robot_name

        self.server = None
        if host is None:
            print(f"Starting {server_module} server …")
            self.server = start_sim_server(
                server_module,
                objects=objects,
                model_dir=model_dir,
                mode=mode,
                port=port,
                use_mjpython=use_mjpython,
                extra_args=server_extra_args,
            )
        self.client = connect_client(port)

        super().__init__(
            robot_state_module=ZmqRobotStateModule(
                robot_name, self.client, parse_conf
            ),
            world_state_module=ZmqWorldStateModule(
                robot_name, self.client, parse_conf
            ),
            robot_odometry_module=None,
            camera_modules={
                name: ZmqCameraModule(
                    name, self.client,
                    camera_params=camera_params,
                    color_profile=DEFAULT_COLORS,
                )
                for name in camera_names
            },
            base_controller=ZmqBaseControlModule(self.client),
            head_controller=(
                ZmqChainControlModule("head", self.client) if has_head else None
            ),
            torso_controller=(
                ZmqChainControlModule("torso", self.client) if has_torso else None
            ),
            arm_controllers={
                side: ZmqChainControlModule(f"{side}_arm", self.client)
                for side in arm_sides
            },
            gripper_controllers={
                f"{side}_gripper": ZmqGripperControlModule(
                    f"{side}_gripper", self.client
                )
                for side in gripper_sides
            },
        )

    def shutdown(self):
        print(f"\nStopping {self.robot_name} sim server …")
        shutdown_server(self.client, self.server)
        print("Done.")
