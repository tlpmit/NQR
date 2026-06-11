import threading

from qr_api.belief_interfaces import (
    AgentMemoryModule,
    ObjectMemoryModule,
    SpatialMemoryModule,
)
from qr_api.belief_typing import HumanInstructionTimeRangeQueryService
from qr_api.perc_interfaces import ObjectBasedScenePerceptionFunction
from qr_api.policy_typing import Action, ActionRV, TransitionPrediction
from qr_api.sensor_typing import HumanInstructionReceivedEvent, Observation


class DummySpatialMemoryModule(SpatialMemoryModule):
    def __init__(self):
        super().__init__()

    def _reset(self, observation: Observation):
        pass

    def _update(self, observation: Observation):
        pass


class DummyObjectMemoryModule(ObjectMemoryModule):
    def __init__(self, rgbd_module, robot_state_module):
        super().__init__()
        self.rgbd_module = rgbd_module
        self.robot_state_module = robot_state_module

    def _reset(self, observation: Observation):
        pass

    def _update(self, observation: Observation):
        pass


class ObjectPerceptionOnlyMemoryModule(ObjectMemoryModule):
    def __init__(self, object_perception_function: ObjectBasedScenePerceptionFunction):
        super().__init__()
        self.object_perception_function = object_perception_function

    def _reset(self, observation: Observation):
        pass

    def _update(self, observation: Observation):
        print("Calling the perception pipline on the observation!")
        result = self.object_perception_function.forward(observation["images"])
        pass


class DummyAgentMemoryModule(AgentMemoryModule):
    def __init__(self, human_instruction_module):
        super().__init__()
        self.human_instruction_module = human_instruction_module

    def _reset(self, observation: Observation):
        pass

    def _update(
        self,
        action: Action,
        actionrv: ActionRV,
        transition_prediction: TransitionPrediction,
        observation: Observation,
    ):
        pass


class SimpleAgentMemoryModule(AgentMemoryModule):
    """A module to manage the agent's memory. This module only keeps the latest human instruction."""

    def __init__(self):
        super().__init__()
        self.current_human_instruction = None
        self.mutex = threading.Lock()
        self.condition = threading.Condition(self.mutex)
        self.subscribe_to_event(
            HumanInstructionReceivedEvent, self._handle_event_human_instruction
        )

    def _handle_event_human_instruction(self, event: HumanInstructionReceivedEvent):
        """Handle a human instruction."""
        with self.mutex:
            self.current_human_instruction = event.instructions[-1]
            self.condition.notify_all()

    def wait_for_human_instruction(self):
        """Wait for a human instruction."""
        with self.mutex:
            while self.current_human_instruction is None:
                self.condition.wait()
            return self.current_human_instruction

    def query_human_instruction_time_range(
        self, query: HumanInstructionTimeRangeQueryService.Query
    ) -> HumanInstructionTimeRangeQueryService.Response:
        """Query the human instructions within a time range."""
        with self.mutex:
            if query.wait:
                while empty := self._is_instruction_empty(query.timestamp_range):
                    self.condition.wait()
            else:
                empty = self._is_instruction_empty(query.timestamp_range)
            if empty:
                return HumanInstructionTimeRangeQueryService.Response([])
            return HumanInstructionTimeRangeQueryService.Response(
                [self.current_human_instruction]
            )

    def _is_instruction_empty(self, timestamp_range):
        if self.current_human_instruction is None:
            return True
        if (
            timestamp_range[0] > self.current_human_instruction.timestamp
            or timestamp_range[1] < self.current_human_instruction.timestamp
        ):
            return True
        return False

    def _reset(self, observation: Observation):
        pass

    def _update(self, observation: Observation):
        pass


class DummyBeliefModule:
    def reset(self, observation):
        pass

    def update(self, action, action_rv, observation):
        pass
