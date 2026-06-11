import numpy as np
from typing import Optional, TYPE_CHECKING
from dataclasses import dataclass

if TYPE_CHECKING:
    from qr_api.control_interfaces import ControlModuleBase
    from qr_api.perc_typing import M4Transformation

from qr_api.perc_typing import M4Transformation

class ControlCommandBase(object):
    """Base class for control commands."""


@dataclass
class ChainCommand(ControlCommandBase):
    """Joint command for the robot arm."""

    duration: float


@dataclass
class JointSpaceChainCommand(ChainCommand):
    """Joint space command for the robot arm."""

    # TODO(Jiayuan Mao @ 2025/03/31): add joint space command, joint angles / velocities / torques


@dataclass
class JointPositionCommand(JointSpaceChainCommand):
    """Joint position command for the robot arm."""
    target_position: np.ndarray

@dataclass
class RobotConfigurationCommand(ControlCommandBase):
    """Robot configuration command for multiple robot chains"""
    target_configuration: dict[str, JointPositionCommand]

@dataclass
class CartesianSpaceChainCommand(ChainCommand):
    """Cartesian space command for the robot arm."""

    # TODO(Jiayuan Mao @ 2025/03/31): add cartesian space command, M4Transformation / velocities / force / compliance

@dataclass
class CartesianPoseCommand(CartesianSpaceChainCommand):
    """Cartesian pose command for the robot arm."""
    target_pose: M4Transformation
    target_joint_position: np.ndarray

@dataclass
class CartesianPositionCommand(CartesianSpaceChainCommand):
    """Cartesian position command for a chain. The base transformation
    is the base link of the chain."""
    target_pose: M4Transformation


@dataclass
class GripperCommand(ControlCommandBase):
    """Gripper command for the robot arm."""


@dataclass
class GripperPositionCommand(GripperCommand):
    target_width: float
    target_force: float


@dataclass
class GripperStateCommand(GripperCommand):
    """Gripper state command for the robot arm."""

    close: bool


@dataclass
class RobotBaseCommand(ControlCommandBase):
    """Robot base command for the robot arm."""

    # TODO(Jiayuan Mao @ 2025/03/31): add robot base command, XY-Theta, XYZ-Theta


@dataclass
class RobotBaseXYThetaCommand(RobotBaseCommand):
    """An XY-theta command for the robot base."""
    x: float
    y: float
    theta: float

@dataclass
class RobotBaseXYThetaDirCommand(RobotBaseCommand):
    """An XY-theta command for the robot base."""
    x: float
    y: float
    theta: float
    dir: float

class VirtualRobotControlContext(object):
    def __init__(self, controller_modules: Optional[list['ControlModuleBase']] = None):
        self.controller_modules = controller_modules
