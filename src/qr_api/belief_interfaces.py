from enum import Enum
from typing import Callable

from qr_api.belief_typing import (
    HumanInstructionTimeRangeQueryService,
    HumanInstructionUpdateEvent,
    ObjectMemoryFindQueryService,
    ObjectMemoryTrackQueryService,
    ObjectMemoryUntrackQueryService,
    SpatialMemoryPointcloudQueryService,
    SpatialMemoryRegionQueryService,
)
from qr_api.policy_typing import Action, ActionRV, TransitionPrediction
from qr_api.sensor_typing import (
    HumanInstructionReceivedEvent,
    Observation,
    RGBDImageReceivedEvent,
    RobotStateReceivedEvent,
)
from qr_core.module import QRModule


class MemoryModuleHookType(Enum):
    BEFORE_RESET = "before_reset"
    AFTER_RESET = "after_reset"
    BEFORE_UPDATE = "before_update"
    AFTER_UPDATE = "after_update"


class MemoryModuleBase(QRModule):
    """A base class for memory modules that store the robot's knowledge. Each memory
    module has two main operations:

    - reset: Reset the memory module with the given initial observation.
    - update: Update the memory module with the given observation after taking an action.

    In the main loop, the belief module roughly follows the following steps:

    .. code-block:: python

        initial_observation = ...
        belief_module.reset(initial_observation)
        while True:
            action = ...
            action_rv = action.execute()
            observation = ...
            belief_module.update(action, action_rv, observation)

    The belief module also supports registering hooks to the memory module. Hooks are
    callbacks that are executed before and after reset and update operations.
    For example, a hook can be used to update the simulation state based on the object
    memory update (e.g., set the object poses).
    To register a hook, create a subclass of `BeliefModuleHook` and implement the hook
    methods, or directly use the `register_hook` method.

    For all implementations of this class, please implement the :meth:`_reset` and
      :meth:`_update` methods.
    """

    def __init__(self):
        super().__init__()
        self._on_update_hooks = dict()

    _on_update_hooks: dict[MemoryModuleHookType, list[Callable]]

    def register_hook(self, hook_type: MemoryModuleHookType, hook: Callable):
        """Register a hook to the memory module.

        Args:
            hook_type: The hook type. It must be one of the values in
            `MemoryModuleHookType`.
            hook: The hook function. For both `reset` and `update` hooks, the function
            should have the signature `def hook(observation: Observation)`.
        """
        if hook_type not in self._on_update_hooks:
            self._on_update_hooks[hook_type] = []
        self._on_update_hooks[hook_type].append(hook)

    def unregister_hook(self, hook_type: MemoryModuleHookType, hook: Callable):
        """Unregister a hook from the memory module."""
        if hook_type in self._on_update_hooks:
            self._on_update_hooks[hook_type].remove(hook)

    def _run_hooks(self, hook_type: MemoryModuleHookType, *args, **kwargs):
        if hook_type in self._on_update_hooks:
            for hook in self._on_update_hooks[hook_type]:
                hook(*args, **kwargs)

    def reset(self, observation: Observation):
        self._run_hooks(MemoryModuleHookType.BEFORE_RESET, observation)
        self._reset(observation)
        self._run_hooks(MemoryModuleHookType.AFTER_RESET, observation)

    def _reset(self, observation: Observation):
        raise NotImplementedError

    def update(
        self,
        action: Action,
        actionrv: ActionRV,
        transition_prediction: TransitionPrediction,
        observation: Observation,
    ):
        self._run_hooks(MemoryModuleHookType.BEFORE_UPDATE, observation)
        self._update(action, actionrv, transition_prediction, observation)
        self._run_hooks(MemoryModuleHookType.AFTER_UPDATE, observation)

    def _update(
        self,
        action: Action,
        actionrv: ActionRV,
        transition_prediction: TransitionPrediction,
        observation: Observation,
    ):
        raise NotImplementedError


class SpatialMemoryModule(MemoryModuleBase):
    EVENTS_RECEIVES = {
        "rgbd_input": RGBDImageReceivedEvent,
        "robot_state": RobotStateReceivedEvent,
    }
    EVENTS_GENERATES = []
    QUERIES_RECEIVES = {
        "region": SpatialMemoryRegionQueryService,
        "pointcloud": SpatialMemoryPointcloudQueryService,
    }
    QUERIES_GENERATES = []

    def _handle_event_rgbd_input(self, event: RGBDImageReceivedEvent):
        pass

    def _handle_event_robot_state(self, event: RobotStateReceivedEvent):
        pass

    def query_region(
        self, query: SpatialMemoryRegionQueryService.Query
    ) -> SpatialMemoryRegionQueryService.Response:
        raise NotImplementedError

    def query_pointcloud(
        self, query: SpatialMemoryPointcloudQueryService.Query
    ) -> SpatialMemoryPointcloudQueryService.Response:
        raise NotImplementedError


class ObjectMemoryModule(MemoryModuleBase):
    EVENTS_RECEIVES = {
        "rgbd_input": RGBDImageReceivedEvent,
        "robot_state": RobotStateReceivedEvent,
    }
    EVENTS_GENERATES = []
    QUERIES_RECEIVES = {
        "find": ObjectMemoryFindQueryService,
        "track": ObjectMemoryTrackQueryService,
        "untrack": ObjectMemoryUntrackQueryService,
    }
    QUERIES_GENERATES = []

    def _handle_event_rgbd_input(self, event: RGBDImageReceivedEvent):
        pass

    def _handle_event_robot_state(self, event: RobotStateReceivedEvent):
        pass

    def query_find(
        self, query: ObjectMemoryFindQueryService.Query
    ) -> ObjectMemoryFindQueryService.Response:
        raise NotImplementedError

    def query_track(
        self, query: ObjectMemoryTrackQueryService.Query
    ) -> ObjectMemoryTrackQueryService.Response:
        raise NotImplementedError

    def query_untrack(
        self, query: ObjectMemoryUntrackQueryService.Query
    ) -> ObjectMemoryUntrackQueryService.Response:
        raise NotImplementedError


class RobotMemoryModule(MemoryModuleBase):
    EVENTS_RECEIVES = {
        "robot_state": RobotStateReceivedEvent,
    }
    EVENTS_GENERATES = []
    QUERIES_RECEIVES = {}
    QUERIES_GENERATES = []

    def _handle_event_robot_state(self, event: RobotStateReceivedEvent):
        pass


class AgentMemoryModule(MemoryModuleBase):
    """A module to manage the agent's memory."""

    EVENTS_RECEIVES = {"human_instruction": HumanInstructionReceivedEvent}
    EVENTS_GENERATES = [HumanInstructionUpdateEvent]
    QUERIES_RECEIVES = {
        "human_instruction_time_range": HumanInstructionTimeRangeQueryService
    }
    QUERIES_GENERATES = []

    def _handle_event_human_instruction(self, event: HumanInstructionReceivedEvent):
        """Handle a human instruction."""
        pass

    def query_human_instruction_time_range(
        self, query: HumanInstructionTimeRangeQueryService.Query
    ) -> HumanInstructionTimeRangeQueryService.Response:
        """Query the human instructions within a time range."""
        raise NotImplementedError


class GoalMemoryModule(MemoryModuleBase):
    """A module to manage memory about the agent's goals."""

    pass


class BeliefModuleHook(QRModule):
    """A hook module for registering callbacks to various memory submodules in a
    BeliefModule.

    This module allows defining custom hooks that are executed before and after key
    operations
    (reset and update) on spatial, object, robot_memory and agent memory submodules of a
    given `BeliefModule`.

    Hooks are registered for the following events:
    - `on_{spatial_memory, object_memory, agent_memory, robot_memory}_before_reset`
    - `on_{spatial_memory, object_memory, agent_memory, robot_memory}_after_reset`
    - `on_{spatial_memory, object_memory, agent_memory, robot_memory}_before_update`
    - `on_{spatial_memory, object_memory, agent_memory, robot_memory}_after_update`

    If the current instance has a method matching the hook naming convention
    (`on_<memory_name>_<hook_type>`), it is registered to the respective memory submodule.

    Example:

        .. code-block:: python

            class MyBeliefModuleHook(BeliefModuleHook):
                def on_spatial_memory_before_reset(self, spatial_memory:
                    SpatialMemoryModule):
                    pass

            belief_module = BeliefModule(...)
            hook_module = MyBeliefModuleHook()
            belief_module.register_hook(hook_module)

    """

    def register(self, belief_module: "BeliefModule"):
        for name in ["spatial_memory", "object_memory", "agent_memory", "robot_memory"]:
            for hook_type in (
                "before_reset",
                "after_reset",
                "before_update",
                "after_update",
            ):
                hook_name = f"on_{name}_{hook_type}"
                if hasattr(self, hook_name):
                    submodule: MemoryModuleBase = getattr(belief_module, name)
                    hook = getattr(self, hook_name)
                    submodule.register_hook(
                        getattr(MemoryModuleHookType, hook_type.upper()), hook
                    )


class BeliefModule(QRModule):
    """The high-level belief module that manages the robot's belief states. It consists
    of four submodules:

    - spatial_memory: A spatial memory module that stores the robot's spatial knowledge
    such as occupancy grid and odometry.
    - object_memory: An object memory module that stores the robot's object knowledge
    such as object poses and properties.
    - agent_memory: An agent memory module that stores the robot's knowledge about
    other agents, such as human goals and instructions.
    - goal_memory: An memory module that stores the robot's knowledge about
    its goals
    """

    def __init__(
        self,
        sensory_interfaces: dict[str, QRModule],
        spatial_memory: SpatialMemoryModule,
        object_memory: ObjectMemoryModule,
        agent_memory: AgentMemoryModule,
        goal_memory: AgentMemoryModule,
        robot_memory: RobotMemoryModule,
    ):
        super().__init__()
        self._sensory_interfaces = sensory_interfaces
        self._spatial_memory = spatial_memory
        self._object_memory = object_memory
        self._agent_memory = agent_memory
        self._goal_memory = goal_memory
        self._robot_memory = robot_memory
        self.action_history = []
        self.observation_history = []

    @property
    def sensory_interfaces(self) -> dict[str, QRModule]:
        return self._sensory_interfaces

    @property
    def spatial_memory(self) -> SpatialMemoryModule:
        return self._spatial_memory

    @property
    def object_memory(self) -> ObjectMemoryModule:
        return self._object_memory

    @property
    def agent_memory(self) -> AgentMemoryModule:
        return self._agent_memory

    @property
    def goal_memory(self) -> GoalMemoryModule:
        return self._goal_memory

    @property
    def robot_memory(self) -> RobotMemoryModule:
        return self._robot_memory

    def reset(self, observation: Observation):
        self.robot_memory.reset(observation)
        self.spatial_memory.reset(observation)
        self.object_memory.reset(observation)
        self.agent_memory.reset(observation)
        self.goal_memory.reset(observation)

    def update(self, action: Action, action_rv: ActionRV, observation: Observation):
        self.action_history.append((action, action_rv))
        self.observation_history.append(observation)
        action.update_belief_before(self, action_rv, observation)
        transition_prediction = action.transition_prediction(self, action_rv)
        self.robot_memory.update(action, action_rv, transition_prediction, observation)
        self.spatial_memory.update(
            action, action_rv, transition_prediction, observation
        )
        self.object_memory.update(action, action_rv, transition_prediction, observation)
        self.agent_memory.update(action, action_rv, transition_prediction, observation)
        self.goal_memory.update(action, action_rv, transition_prediction, observation)

    def register_hook_module(self, hook_module: BeliefModuleHook):
        hook_module.register(self)
