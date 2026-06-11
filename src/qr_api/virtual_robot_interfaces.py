from typing import Optional, Iterable
from qr_api.control_interfaces import ControlModuleBase, MonitoredExecutionFuture
from qr_api.control_typing import ControlCommandBase, VirtualRobotControlContext
from qr_api.sensor_interfaces import SensorModuleBase
from qr_api.sensor_typing import Observation


class VirtualRobotModule(object):
    """A virtual robot module that contains sensor and control modules.

    Each sensor module implements a sense() method that returns a value of a custom type. The :meth:`sense` method is called to collect data from multiple sensor modules.

    Each control module implements an interface that control the robot. There are two ways to control the robot:

    - Asynchronous control: each control module implements a set_control method that takes a control command and a context. The VirtualRobot module implements a step_control method.
        The context is used to store the state of the control module, which is passed to the step_control method. Roughly, this set_control method should send some control commands
        to the robot, but not execute them immediately. After all control commands for all modules (e.g., different joints) are set, the step_control method is called to execute the commands.
        In implementation, the step_control method will first call the step_control method of each control module, and then call the _step_control method of the virtual robot module.

        This design can be used to implement multiple types of control methods.

        - PyBullet Simulation:
            The set_control method sets the joint commands for all joints. The step_control method is called to step the simulation (p.stepSimulation()).
        - Franka Robots:
            The set_control method sends the joint commands to multiple robots simultaneously. The step_control method waits 1/fps seconds.
        - RBY1A Robots:
            The context is used to store a WholeBodyControl command. The set_control method sets the corresponding part of the command.
            The step_control method actually sends the compiled command to the robot.


    - Synchronous control: each control module implements a control_blocking method that takes a list of control commands and executes them immediately. Return after all commands are executed.
        Most robot interfaces implement this method.

    The VirtualRobotModule class is a base class for all robot interfaces. It contains a high-level API :meth:`control` that does the following:

    - If the commands list contains the command to a single control module, and blocking is True, it will call the control_blocking method of the control module.
    - If the commands list contains the command to multiple control modules, and blocking is True, it will execute the control loop in the main thread and return after all commands are executed.
    - If the commands list contains the command to multiple control modules, and blocking is False, it will execute the control loop in a separate thread and return immediately.

    .. note::

        - Other types of sensors: LIDAR, IMU, GPS.
        - Other types of control modules: joint impedance, cartesian impedance
        - Async execution of the control loop (aka the implementation of the MonitoredExecutionFuture class) is not implemented yet.
        - Time annotation for the trajectories.
        - Safety monitoring is not implemented yet.
        - Error handling is not implemented yet.
    """

    def __init__(self, sensor_modules: Optional[dict[str, SensorModuleBase]] = None, control_modules: Optional[dict[str, ControlModuleBase]] = None):
        """Initialize the virtual robot module with sensor and control modules."""
        self.sensor_modules = sensor_modules if sensor_modules is not None else dict()
        self.control_modules = control_modules if control_modules is not None else dict()

    def add_sensor_modules(self, **kwargs):
        self.sensor_modules.update(kwargs)

    def add_control_modules(self, **kwargs):
        self.control_modules.update(kwargs)

    def get_sensor_module(self, name: str) -> SensorModuleBase:
        """Get a sensor module by name."""
        if name not in self.sensor_modules:
            raise ValueError(f"Sensor module {name} not found.")
        return self.sensor_modules[name]

    def get_control_module(self, name: str) -> ControlModuleBase:
        """Get a control module by name."""
        if name not in self.control_modules:
            raise ValueError(f"Control module {name} not found.")
        return self.control_modules[name]

    def set_control(self, commands: dict[str, ControlCommandBase]) -> VirtualRobotControlContext:
        """Send control commands to the robot."""
        ctx = self._make_control_context()
        for name, command in commands.items():
            control_module = self.get_control_module(name)
            control_module.set_control(ctx, command)
            ctx.controller_modules.append(control_module)
        return ctx

    def step_control(self, ctx: VirtualRobotControlContext):
        for module in ctx.controller_modules:
            module.step_control(ctx)
        self._step_control(ctx)

    def _make_control_context(self) -> VirtualRobotControlContext:
        """Create a control context for the robot."""
        ctx = VirtualRobotControlContext(list())
        return ctx

    def _step_control(self, ctx: VirtualRobotControlContext):
        raise NotImplementedError("The step function is not implemented in the base class. Please implement it in the derived class.")

    def control(self, commands: Iterable[dict[str, ControlCommandBase]], blocking: bool = True) -> MonitoredExecutionFuture:
        """Send control trajectory commands to the robot."""
        commands = list(commands)

        if len(commands) == 0:
            raise ValueError("The commands list is empty.")

        # NOTE: control_blocking, at the moment, creates a default command context,
        # and we may want a robot-specific one.    
        if all(len(command) == 1 for command in commands) and blocking:
            # If there is only a single command.
            # Todo check the command is the same.
            module_name = list(commands[0].keys())[0]
            module = self.get_control_module(module_name)
            commands = [command[module_name] for command in commands]
            return module.control_blocking(commands)

        if blocking:
            return self._control_loop(commands)
        else:
            raise NotImplementedError('Non-blocking control is not implemented yet.')

    def _control_loop(self, commands: Iterable[dict[str, ControlCommandBase]]) -> MonitoredExecutionFuture:
        # TODO(Jiayuan Mao @ 2025/04/1): implement error handling and safety monitoring.
        for command in commands:
            ctx = self.set_control(command)
            self.step_control(ctx)
        return MonitoredExecutionFuture.make_from_success()

    def sense(self, sensor_names: Optional[list[str]] = None) -> Observation:
        """Get sensor data from the robot."""
        if sensor_names is None:
            sensor_names = list(self.sensor_modules.keys())
        rv = {}
        for name in sensor_names:
            sensor_module = self.get_sensor_module(name)
            if sensor_module is None:
                continue
            rv[name] = sensor_module.sense()
        return Observation(rv)
    
    def shutdown(self):
        pass
