from typing import Iterable, Union

import numpy as np

from qr_api.control_interfaces import ControlModuleBase, MonitoredExecutionFuture
from qr_api.control_typing import CartesianPoseCommand, JointPositionCommand
from qr_api.perc_typing import CalibratedRGBDObservation, M4Transformation
from qr_api.sensor_interfaces import (
    CameraModule,
    RobotOdometryModule,
    RobotStateModule,
    WorldStateModule,
)
from qr_api.sensor_typing import Observation, ObservationSequence
from qr_api.virtual_robot_interfaces import VirtualRobotModule


class VirtualMomanModule(VirtualRobotModule):
    def __init__(
        self,
        robot_state_module: RobotStateModule,
        world_state_module: Union[WorldStateModule, None] = None,
        robot_odometry_module: Union[RobotOdometryModule, None] = None,
        camera_modules: dict[str, CameraModule] = {},
        base_controller: Union[ControlModuleBase, None] = None,
        head_controller: Union[ControlModuleBase, None] = None,
        torso_controller: Union[ControlModuleBase, None] = None,
        arm_controllers: dict[str, ControlModuleBase] = {},
        arm_cartesian_controllers: dict[str, ControlModuleBase] = {},
        gripper_controllers: dict[str, ControlModuleBase] = {},
    ):
        super().__init__(
            sensor_modules={
                "robot_state": robot_state_module,
                "robot_odometry": robot_odometry_module,
                "world_state": world_state_module,
            }
            | camera_modules,
            control_modules={
                "base_controller": base_controller,
                "head_controller": head_controller,
                "torso_controller": torso_controller,
            }
            | arm_controllers
            | arm_cartesian_controllers
            | gripper_controllers,
        )
        self.robot_state_module = robot_state_module
        self.world_state_module = world_state_module
        self.robot_odometry_module = robot_odometry_module
        self.camera_modules = camera_modules
        self.base_controller = base_controller
        self.head_controller = head_controller
        self.torso_controller = torso_controller
        self.arm_controllers = arm_controllers
        self.arm_cartesian_controllers = arm_cartesian_controllers
        self.gripper_controllers = gripper_controllers

    def get_cameras(self) -> list[CameraModule]:
        """
        Returns:
            list[CameraModule]: list of camera modules on the robot.
        """
        return list(self.camera_modules.values())

    def get_camera_names(self) -> list[str]:
        """
        Returns:
            list[str]: list of camera names on the robot.
        """
        return list(self.camera_modules.keys())

    def get_observation(self) -> Observation:
        """
        Returns:
            Observation: the current observation of the robot.
            An alternative to more fine-grained individual observation captures
        """
        return {
            "robot_state": self.robot_state_module.sense(),
            "robot_odometry": self.robot_odometry_module.sense()
            if self.robot_odometry_module
            else None,
            "images": [camera.sense() for camera in self.camera_modules.values()],
        }

    def get_num_arms(self) -> int:
        """
        Returns:
            int: number of arms on the robot.
        """
        return len(self.arm_controllers)

    def get_robot_conf(self) -> dict[str, np.ndarray]:
        """
        Returns:
            JointConfiguration: the current configuration of the robot.
        """
        return self.robot_state_module.sense().joint_positions

    def get_robot_base_pose(self) -> M4Transformation:
        """
        Returns:
            M4Transformation: the current pose of the robot base.
        """
        return self.robot_odometry_module.sense().pose

    def get_camera_image(self, camera_name: str) -> CalibratedRGBDObservation:
        """
        Args:
            camera_name (str): name of the camera.

        Returns:
            CalibratedRGBDObservation: the current image from the specified camera.
        """
        if camera_name not in self.camera_modules:
            raise ValueError(f"Camera {camera_name} not found.")
        return self.camera_modules[camera_name].sense()

    def get_world_state(self):
        if self.world_state_module:
            return self.world_state_module.sense()
        else:
            raise ValueError("World state sensing is not supported")

    def async_follow_joint_trajectory_stiff(
        self, chain_dict_commands: Iterable[dict[str, JointPositionCommand]]
    ) -> MonitoredExecutionFuture:
        """
        Asynchronous command to arm to stiffly follow the provided joint trajectory.

        Returns:
            MonitoredExecutionFuture: A monitor that can be used to check the status of the
                command.
        """

        # Assumes that for each chain name in each joint_conf, we have a controller with that name

        return self.control(chain_dict_commands, blocking=True)

    def sync_follow_joint_trajectory_stiff(
        self,
        chain_dict_commands: Iterable[dict[str, JointPositionCommand]],
    ) -> ObservationSequence:
        """
        Synchronously command arm to stiffly follow the provided joint trajectory.

        Returns:
            ObservationSequence: Sequence of observations collected during execution.
        """
        monitor = self.async_follow_joint_trajectory_stiff(chain_dict_commands)
        monitor.wait_until_done()
        return monitor.get_observations()

    def async_follow_joint_trajectory_compliant(
        self, chain_dict_commands: Iterable[dict[str, JointPositionCommand]]
    ) -> MonitoredExecutionFuture:
        """
        Asynchronous command to arm to follow the provided joint trajectory with compliance.

        Returns:
            MonitoredExecutionFuture: A monitor that can be used to check the status of the command.
        """
        raise NotImplementedError("Call self.control() to implement this method.")

    def sync_follow_joint_trajectory_compliant(
        self,
        chain_dict_commands: Iterable[dict[str, JointPositionCommand]],
    ) -> ObservationSequence:
        """
        Synchronously command arm to follow the provided joint trajectory with compliance.

        Returns:
            ObservationSequence: Sequence of observations collected during execution.
        """
        monitor = self.async_follow_joint_trajectory_compliant(chain_dict_commands)
        monitor.wait_until_done()
        return monitor.get_observations()

    def async_follow_cartesian_trajectory_stiff(
        self, eef_poses: list[Iterable[dict[str, CartesianPoseCommand]]]
    ) -> MonitoredExecutionFuture:
        """
        Asynchronously command arm to stiffly follow the provided end-effector trajectory.

        Args:
            eef_poses (list[Iterable[dict[str, M4Transformation]]]): Desired end-effector pose trajectory,
                specified as iterables of 4x4 SE(3) matrices in "global" coordinatea

        Returns:
            MonitoredExecutionFuture: A monitor that can be used to check the status of the command.
        """
        return self.control(eef_poses, blocking=True)

    def sync_follow_cartesian_trajectory_stiff(
        self,
        eef_poses: list[Iterable[dict[str, CartesianPoseCommand]]],
    ) -> ObservationSequence:
        """
        Synchronously command arm to stiffly follow the provided end-effector trajectory.

        Args:
            eef_poses (list[Iterable[dict[str, M4Transformation]]]): Desired end-effector pose trajectory,
                specified as iterables of 4x4 SE(3) matrices in "global" coordinates
            times (list[float]): Position timestamps.

        Returns:
            ObservationSequence: Sequence of observations collected during execution.
        """
        monitor = self.async_follow_cartesian_trajectory_stiff(eef_poses)
        monitor.wait_until_done()
        return monitor.get_observations()

    def async_follow_eef_trajectory_stiff(
        self,
        eef_poses: list[dict[str, M4Transformation]],
        gripper_positions: list[dict[str, float]],
        times: list[float],
    ) -> MonitoredExecutionFuture:
        """
        Asynchronously command arm to stiffly follow the provided end-effector trajectory.

        Args:
            eef_poses (list[dict[str, M4Transformation]]): A mapping from chain to the desired
                end-effector pose, as specified as iterables of 4x4 SE(3) matrices relative to the base link
                of the chain.
            gripper_positions (list[dict[str, float]]): A mapping from the gripper to the position
            of the gripper jaw.
            times (list[float]): Position timestamps.

        Returns:
            MonitoredExecutionFuture: A monitor that can be used to check the status of the command.
        """
        raise NotImplementedError("Call self.control() to implement this method.")

    def sync_follow_eef_trajectory_stiff(
        self,
        eef_poses: list[dict[str, M4Transformation]],
        gripper_positions: list[dict[str, float]],
        times: list[float],
    ) -> ObservationSequence:
        """
        Synchronously command arm to stiffly follow the provided end-effector trajectory.

        Args:
            eef_poses (list[dict[str, M4Transformation]]): A mapping from chain to the desired
                end-effector pose, as specified as iterables of 4x4 SE(3) matrices relative to the base link
                of the chain.
            gripper_positions (list[dict[str, float]]): A mapping from the gripper to the position
            of the gripper jaw.
            times (list[float]): Position timestamps.

        Returns:
            ObservationSequence: Sequence of observations collected during execution.
        """
        monitor = self.async_follow_eef_trajectory_stiff(
            eef_poses, gripper_positions, times
        )
        monitor.wait_until_done()
        return monitor.get_observations()

    def async_follow_eef_trajectory_compliant(
        self,
        eef_poses: list[dict[str, M4Transformation]],
        gripper_positions: list[dict[str, float]],
        times: list[float],
    ) -> MonitoredExecutionFuture:
        """
        Asynchronously command arm to compliantly follow the provided end-effector trajectory.

        Args:
            eef_poses (list[dict[str, M4Transformation]]): A mapping from chain to the desired
                end-effector pose, as specified as iterables of 4x4 SE(3) matrices relative to the base link
                of the chain.
            gripper_positions (list[dict[str, float]]): A mapping from the gripper to the position
            of the gripper jaw.
            times (list[float]): Position timestamps.

        Returns:
            MonitoredExecutionFuture: A monitor that can be used to check the status of the command.
        """
        raise NotImplementedError("Call self.control() to implement this method.")

    def sync_follow_eef_trajectory_compliant(
        self,
        eef_poses: list[dict[str, M4Transformation]],
        gripper_positions: list[dict[str, float]],
        times: list[float],
    ) -> ObservationSequence:
        """
        Synchronously command arm to compliantly follow the provided end-effector trajectory.

        Args:
            eef_poses (list[dict[str, M4Transformation]]): A mapping from chain to the desired
                end-effector pose, as specified as iterables of 4x4 SE(3) matrices relative to the base link
                of the chain.
            gripper_positions (list[dict[str, float]]): A mapping from the gripper to the position
            of the gripper jaw.
            times (list[float]): Position timestamps.

        Returns:
            ObservationSequence: Sequence of observations collected during execution.
        """
        monitor = self.async_follow_eef_trajectory_compliant(
            eef_poses, gripper_positions, times
        )
        monitor.wait_until_done()
        return monitor.get_observations()

    # Handle this like a black-box skill, because
    # - It needs special handling in simulation
    # - On a real robot, we might want to execute some more specialized closed-loop policy

    def sync_execute_gripper_command(
        self, gripper_name: "str", command: "str", opening_cm: float
    ):
        """
        Asynchronously command gripper to open/close.

        Args:
            command (str): Command to open or close the gripper.
            opening_cm (float): Distance to open the gripper in cm.

        Returns:
            MonitoredExecutionFuture: A monitor that can be used to check the status of the command.
        """
        return self.gripper_controllers[gripper_name].control_blocking(
            command, opening_cm
        )

    def async_execute_gripper_command(
        self, gripper_name: "str", command: "str", opening_cm: float
    ) -> MonitoredExecutionFuture:
        """
        Asynchronously command gripper to open/close.

        Args:
            command (str): Command to open or close the gripper.
            opening_cm (float): Distance to open the gripper in cm.

        Returns:
            MonitoredExecutionFuture: A monitor that can be used to check the status of the command.
        """
        return self.gripper_controllers[gripper_name].control(command, opening_cm)

    def async_follow_base_trajectory(
        self,
        base_commands: Iterable[JointPositionCommand],
    ) -> MonitoredExecutionFuture:
        """
        Command robot base to follow a trajectory of X, Y, yaw coordinates relative
        to the base's current pose.

        Returns:
            MonitoredExecutionFuture: A future instance that returns observations when command execution is complete.
        """
        cmds = []

        for _base_cmd in base_commands:
            cmds.append({"base_controller": _base_cmd})

        return self.control(cmds, blocking=True)

    def sync_follow_base_trajectory(
        self,
        base_commands: Iterable[JointPositionCommand],
    ) -> ObservationSequence:
        future = self.async_follow_base_trajectory(base_commands)
        future.wait_until_done()
        return future.get_observations()

    def sync_follow_head_trajectory(
        self,
        head_commands: Iterable[JointPositionCommand],
    ) -> ObservationSequence:
        """
        Command robot head to follow a trajectory of pan, tilt coordinates

        Args:
            head_positions (list[np.ndarray]): list of pan, tilt coordinates
            times (list[float]): Position timestamps.

        Returns:
            ObservationSequence: Sequence of observations collected during execution.
        """
        cmds = []

        for _head_cmd in head_commands:
            cmds.append({"head_controller": _head_cmd})

        future = self.control(cmds, blocking=True)
        future.wait_until_done()

    def sync_follow_torso_trajectory(
        self,
        torso_commands: Iterable[JointPositionCommand],
    ) -> ObservationSequence:
        """
        Command robot head to follow a trajectory of torsocoordinates

        Args:
            torso_positions (list[np.ndarray]): list of torso coordinates
            times (list[float]): Position timestamps.

        Returns:
            ObservationSequence: Sequence of observations collected during execution.
        """
        cmds = []

        for _torso_cmd in torso_commands:
            cmds.append({"torso_controller": _torso_cmd})

        future = self.control(cmds, blocking=True)
        future.wait_until_done()

    def move_camera_to_pose(self, pose: M4Transformation) -> ObservationSequence:
        """
        Move camera to provided pose.

        Args:
            pose (M4Transformation): Desired camera frame relative to robot torso link
                (must be kinematically feasible).

        Returns:
            ObservationSequence: Image captured after camera reaches designated pose.
        """
        raise NotImplementedError("Call self.control() to implement this method.")

    def record_joint_trajectory(
        self, state_hz: float, camera_hz: float, time: float
    ) -> ObservationSequence:
        """
        Passively collect robot state at specified frequencies/length (good for teleoperation use).

        Args:
            state_hz (float): Rate at which to collect robot kinematic state.
            camera_hz (float): Rate at which to collect camera observations.
            time (float): Length of time to record sensorimotor data.

        Returns:
            ObservationSequence: All sensorimotor experiences collected by the robot.
        """
        raise NotImplementedError
