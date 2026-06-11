from qr_api.belief_interfaces import BeliefModule
from qr_api.control_interfaces import ControlModuleBase
from qr_api.policy_typing import Action, ActionRV, PlanFinishedException
from qr_api.sensor_typing import Observation
from qr_api.policy_interfaces import PolicyModule


class DummyAction(Action):
    def __init__(self, string: str):
        self.string = string

    def execute(self, belief_module: BeliefModule, controller_interfaces: dict[str, ControlModuleBase]) -> ActionRV:
        print(f'Executing action: {self.string}')
        return ActionRV()


class DummyPolicyModule(PolicyModule):
    _actions: list[Action]
    _action_index: int

    def reset(self, observation: Observation):
        self._actions = [DummyAction('dummy-action-1'), DummyAction('dummy-action-2')]
        self._action_index = 0

    def step(self) -> Action:
        if self._action_index >= len(self._actions):
            raise PlanFinishedException()
        action = self._actions[self._action_index]
        self._action_index += 1
        return action
