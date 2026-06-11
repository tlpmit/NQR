"""
Sensor and control modules for ZMQ-backed virtual robots.

All simulation backends (MuJoCo, Drake, kinematic) speak the same wire
protocol via :class:`qr_robots.common.zmq_sim.SimClient`, so a single set of
control/sensor module implementations serves every robot.  Per-robot
differences (chain layout, conf parsing, camera names) are injected through
constructor arguments.
"""

from __future__ import annotations

import time
from typing import Callable, Iterable

import numpy as np

from qr_api.control_interfaces import (
    ChainControlModule,
    GripperControlModule,
    MonitoredExecutionFuture,
)
from qr_api.control_typing import ChainCommand, JointPositionCommand
from qr_api.perc_typing import CalibratedRGBDObservation
from qr_api.sensor_interfaces import CameraModule, RobotStateModule, WorldStateModule
from qr_api.sensor_typing import RobotStateInput
from qr_robots.common.zmq_sim import SimClient


class ZmqMonitoredExecutionFuture(MonitoredExecutionFuture):
    def wait_until_done(self):
        return

    def get_observations(self):
        return


def _trajectory_from_commands(commands: Iterable[ChainCommand]) -> list:
    """
    Build the wire-format trajectory from JointPositionCommands.

    When commands carry positive durations the result is a list of
    (arrival_time, position) pairs so the sim honors the caller's timing;
    otherwise it is a plain list of positions and the sim picks the timing.
    """
    joint_commands = list(commands)
    if not joint_commands:
        raise ValueError("No joint commands provided.")
    assert isinstance(joint_commands[0], JointPositionCommand), (
        "Only JointPositionCommand is supported."
    )
    durations = [getattr(jc, "duration", None) for jc in joint_commands]
    if all(d is not None and d > 0 for d in durations):
        t = 0.0
        pairs = []
        for jc in joint_commands:
            t += float(jc.duration)
            pairs.append((t, jc.target_position))
        return pairs
    return [jc.target_position for jc in joint_commands]


class ZmqChainControlModule(ChainControlModule):
    """Drives one chain (arm/torso/head) through joint waypoints."""

    def __init__(self, chain_name: str, client: SimClient,
                 waypoint_dt: float | None = None):
        super().__init__()
        self.chain_name = chain_name
        self._client = client
        self._waypoint_dt = waypoint_dt

    def control_blocking(
        self, commands: Iterable[ChainCommand]
    ) -> ZmqMonitoredExecutionFuture:
        trajectory = _trajectory_from_commands(commands)
        self._client.execute_chain_trajectory(
            self.chain_name, trajectory, self._waypoint_dt
        )
        return ZmqMonitoredExecutionFuture.make_from_success()


class ZmqBaseControlModule(ChainControlModule):
    """Drives the mobile base through [x, y, theta] waypoints."""

    def __init__(self, client: SimClient, waypoint_dt: float | None = None):
        super().__init__()
        self.chain_name = "base"
        self._client = client
        self._waypoint_dt = waypoint_dt

    def control_blocking(
        self, commands: Iterable[ChainCommand]
    ) -> ZmqMonitoredExecutionFuture:
        trajectory = _trajectory_from_commands(commands)
        self._client.execute_base_trajectory(trajectory, self._waypoint_dt)
        return ZmqMonitoredExecutionFuture.make_from_success()


class ZmqGripperControlModule(GripperControlModule):
    """
    Sends open/close (or fractional opening) commands to one gripper.
    The command is 'open' or 'close' so the simulator knows whether to attach
    or detach the object from the gripper (kinematic backends).
    """

    def __init__(self, gripper_name: str, client: SimClient):
        super().__init__()
        self.gripper_name = gripper_name
        self._client = client

    def control_blocking(
        self, command: str, opening_cm: float
    ) -> ZmqMonitoredExecutionFuture:
        side = "left" if "left" in self.gripper_name else "right"
        self._client.execute_gripper_command(side, command)
        return ZmqMonitoredExecutionFuture.make_from_success()


class ZmqCameraModule(CameraModule):
    """RGB-D(+label) camera; intrinsics and extrinsics come from the sim."""

    def __init__(self, camera_name: str, client: SimClient,
                 camera_params=None, color_profile=None):
        super().__init__(
            camera_name, color_profile=color_profile, camera_params=camera_params
        )
        self.camera_name = camera_name
        self._client = client

    def sense(self) -> CalibratedRGBDObservation:
        rgb_image, depth_image, label_image = self._client.get_camera_image(
            self.camera_name
        )
        intrinsics = self._client.get_camera_intrinsics(self.camera_name)
        extrinsics = self._client.get_camera_extrinsics(self.camera_name)
        im = CalibratedRGBDObservation(
            rgb_image, depth_image, intrinsics, extrinsics,
            label_image=label_image,
        )
        im.camera_params = self.camera_params
        return im


class ZmqRobotStateModule(RobotStateModule):
    """
    Reports joint positions as a chain-name → array dict.  *parse_conf* maps
    the robot's raw conf vector to that dict (per-robot format).
    """

    def __init__(self, robot_identifier: str, client: SimClient,
                 parse_conf: Callable[[np.ndarray], dict]):
        super().__init__(robot_identifier)
        self._client = client
        self._parse_conf = parse_conf

    def sense(self) -> RobotStateInput:
        conf = self._parse_conf(self._client.get_robot_conf())
        return RobotStateInput(joint_positions=conf, timestamp=time.time())


class ZmqWorldStateModule(WorldStateModule):
    """Perfect-perception world state: object poses plus the parsed robot conf."""

    def __init__(self, robot_identifier: str, client: SimClient,
                 parse_conf: Callable[[np.ndarray], dict]):
        super().__init__(robot_identifier)
        self._client = client
        self._parse_conf = parse_conf

    def sense(self) -> dict:
        world = self._client.get_world_state()
        state = {k: v for k, v in world.items() if k != "robot"}
        state["conf"] = self._parse_conf(world["robot"])
        return state
