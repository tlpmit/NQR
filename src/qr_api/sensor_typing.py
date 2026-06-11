from typing import Dict

from dataclasses import dataclass, field

import numpy as np

from qr_api.perc_typing import M4Transformation
from qr_core.event import QREvent


@dataclass
class RGBDImageInput(object):
    """An RGBD image input for an agent."""

    camera_name: str

    color_image: np.ndarray

    depth_image: np.ndarray

    camera_intrinsics: np.ndarray

    camera_extrinsics: np.ndarray
    """The camera extrinsics matrix (transform points from world to camera). AKA the pose of the camera in the world."""

    timestamp: float


@dataclass
class RobotStateInput(object):
    """A robot state input for an agent."""

    joint_positions: Dict[str, np.ndarray]

    timestamp: float

    joint_velocities: np.ndarray = None

    joint_torques: np.ndarray = None

    operating_states: dict = None



@dataclass
class RobotOdometryInput(object):
    """The odometry of the robot."""

    pose: M4Transformation

    timestamp: float


@dataclass
class RGBDImageReceivedEvent(QREvent):
    images: list[RGBDImageInput]


@dataclass
class RobotStateReceivedEvent(QREvent):
    states: list[RobotStateInput]


@dataclass
class RobotOdometryReceivedEvent(QREvent):
    odometry: list[RobotOdometryInput]


@dataclass
class Observation(object):
    fields: dict[str, any] = field(default_factory=dict)


ObservationSequence = list[Observation]


@dataclass
class LanguageInstructionInput(object):
    """A language instruction for an agent."""

    text: str

    timestamp: float


@dataclass
class HumanInstructionReceivedEvent(QREvent):
    instructions: list[LanguageInstructionInput]