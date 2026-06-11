from typing import Optional, Iterable
from qr_core.module import QRModule
from qr_api.control_typing import RobotBaseCommand, ChainCommand, GripperCommand, ControlCommandBase, VirtualRobotControlContext


class MonitoredExecutionFuture(object):
    # TODO(Jiayuan Mao @ 2025/03/31): monitored execution goes here.
    @classmethod
    def make_from_success(cls):
        return cls()

    @classmethod
    def make_from_failure(cls, error: Exception):
        raise cls()

    @classmethod
    def make_from_control_loop(cls, control_loop, callbacks: Optional[list[callable]] = None):
        def run(self):
            for _ in control_loop:
                for callback in callbacks:
                    callback()
        return cls()


class ControlModuleBase(QRModule):
    """A module that can control the virtual robot, either a physical robot or a simulated robot.
    For a detailed explanation of the control module, please refer to the documentation of the class :class:`~qr_api.virtual_robot_interfaces.VirtualRobotModule`."""

    def set_control(self, ctx: VirtualRobotControlContext, command: ControlCommandBase):
        """Set the control command to the robot.

        Args:
            ctx: The context of the control module.
            command: The command to control the robot.
        """
        raise NotImplementedError(f"ControlModuleBase does not implement control method. Please implement it in the derived class.")

    def step_control(self, ctx: VirtualRobotControlContext):
        """Actually step the robot based on the saved control commands."""
        pass

    def control_blocking(self, commands: Iterable[ControlCommandBase]) -> MonitoredExecutionFuture:
        """Control the robot with a set of blocking commands. The default implementation is to call set_control for each command in the iterable."""
        ctx = VirtualRobotControlContext()
        for command in commands:
            self.set_control(ctx, command)
        return MonitoredExecutionFuture.make_from_success()


class RobotBaseControlModule(ControlModuleBase):
    def set_control(self, ctx: VirtualRobotControlContext, command: RobotBaseCommand) -> MonitoredExecutionFuture:
        raise NotImplementedError(f"RobotBaseControlModule does not implement control method. Please implement it in the derived class.")

    def control_blocking(self, commands: Iterable[RobotBaseCommand]) -> MonitoredExecutionFuture:
        return super().control_blocking(commands)


class ChainControlModule(ControlModuleBase):
    def set_control(self, ctx: VirtualRobotControlContext, command: ChainCommand) -> MonitoredExecutionFuture:
        raise NotImplementedError(f"ChainControlModule does not implement control method. Please implement it in the derived class.")

    def control_blocking(self, commands: Iterable[ChainCommand]) -> MonitoredExecutionFuture:
        return super().control_blocking(commands)


class GripperControlModule(ControlModuleBase):
    def set_control(self, ctx: VirtualRobotControlContext, command: GripperCommand) -> MonitoredExecutionFuture:
        raise NotImplementedError(f"GripperControlModule does not implement control method. Please implement it in the derived class.")

    def control_blocking(self, commands: Iterable[GripperCommand]) -> MonitoredExecutionFuture:
        return super().control_blocking(commands)
