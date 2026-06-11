from abc import ABC, abstractmethod
from typing import Optional, Tuple
import numpy as np

Pose = np.ndarray
Timestamp = int


class OdometryInterface(ABC):
    @abstractmethod
    def get_current_pose(self) -> Tuple[Timestamp, Pose]:
        raise NotImplementedError


class LocalizationInterface(ABC):
    @abstractmethod
    def __init__(self, robot, config, *args, **kwargs):
        self.robot = robot
        self.config = config

    @abstractmethod
    def get_odometry_pose(self, time: Optional[float] = None) -> Pose:
        """Get pose of the lidar in the odometry frame.

        The odometry frame is not corrected for loop closure, so it
        will drift. However, it will not jump
        """
        raise NotImplementedError

    @abstractmethod
    def get_map_pose(self, time: Optional[float] = None) -> Pose:
        """Get pose of the lidar in the map frame.

        The map frame *may* be corrected for loop closure, so it
        may drift less than the odometry pose. It may jump
        discontinuously
        """
        raise NotImplementedError

    @abstractmethod
    def reset_pose(self):
        """Reset the odometry_pose and map_pose to identity"""
        raise NotImplementedError
