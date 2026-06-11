from typing import Any
from qr_api.sensor_typing import (
    HumanInstructionReceivedEvent, RobotOdometryInput, RobotStateInput, RGBDImageInput, RGBDImageReceivedEvent,
    RobotStateReceivedEvent, RobotOdometryReceivedEvent
)
from qr_core.module import QRModule


class InputModule(QRModule):
    """A module that provides sensory inputs to the robot."""

    EVENTS_RECEIVES = {}
    EVENTS_GENERATES = [RGBDImageReceivedEvent, HumanInstructionReceivedEvent, RobotStateReceivedEvent]
    QUERIES_RECEIVES = {}
    QUERIES_GENERATES = []


class SensorModuleBase(InputModule):
    def sense(self) -> Any:
        """Get the sensor data from the module."""
        raise NotImplementedError("This method should be implemented by subclasses.")


class CameraModule(SensorModuleBase):
    def __init__(self, camera_identifier, camera_intrinsics=None, color_profile=None,
                 camera_params=None):
        super().__init__()
        self.camera_identifier = camera_identifier
        self.camera_intrinsics = camera_intrinsics
        self.camera_params = camera_params if camera_params else (60.0, 480, 640, 0.05, 3.0)

        lo_val = 0.5 * 255
        hi_val = 0.9 * 255
        MIDDLING_COLORS = {
            'red': (hi_val, lo_val, lo_val, 1),
            'magenta' : (hi_val, lo_val, hi_val, 1),
            'cyan' : (lo_val, hi_val, hi_val, 1),
            'green': (lo_val, hi_val, lo_val, 1),
            'blue': (lo_val, lo_val, hi_val, 1),
            'yellow' : (hi_val, hi_val, lo_val, 1),
            'brown' : (128, 64, 25, 1),
            'grey' : (128, 128, 128, 1) }
        self.color_profile = color_profile if color_profile else MIDDLING_COLORS

    def sense(self) -> RGBDImageInput:
        raise NotImplementedError("This method should be implemented by subclasses.")

    def emit_image_received_event(self, image: RGBDImageInput):
        """Emit an event when an image is received."""
        self.emit_event(RGBDImageReceivedEvent([image]))


class RobotStateModule(SensorModuleBase):
    def __init__(self, robot_identifier):
        super().__init__()
        self.robot_identifier = robot_identifier

    def sense(self) -> RobotStateInput:
        raise NotImplementedError("This method should be implemented by subclasses.")

    def emit_state_received_event(self, state: RobotStateInput):
        """Emit an event when the robot state is received."""
        self.emit_event(RobotStateReceivedEvent([state]))

# To simulate perfect perception
class WorldStateModule(SensorModuleBase):
    def __init__(self, robot_identifier):
        super().__init__()
        self.robot_identifier = robot_identifier

    def sense(self) -> dict[str, any]:
        raise NotImplementedError("This method should be implemented by subclasses.")

    def emit_state_received_event(self, state):
        pass


class RobotOdometryModule(SensorModuleBase):
    def __init__(self, robot_identifier):
        super().__init__()
        self.robot_identifier = robot_identifier

    def reset_odometry(self):
        """Reset the odometry of the robot."""
        raise NotImplementedError("This method should be implemented by subclasses.")

    def sense(self) -> RobotOdometryInput:
        raise NotImplementedError("This method should be implemented by subclasses.")

    def emit_odometry_received_event(self, odometry: RobotOdometryInput):
        """Emit an event when odometry is received."""
        self.emit_event(RobotOdometryReceivedEvent([odometry]))
