from qr_core.module import QRModule
from qr_api.sensor_typing import Observation
from qr_api.policy_typing import Action, ActionRV
from qr_api.belief_interfaces import BeliefModule
from qr_api.control_interfaces import ControlModuleBase


class PolicyModule(QRModule):
    def __init__(self, belief_module: BeliefModule, control_interfaces: dict[str, ControlModuleBase]):
        super().__init__()
        self._belief_module = belief_module
        self._control_modules = control_interfaces

    @property
    def belief_module(self) -> BeliefModule:
        return self._belief_module

    @property
    def control_modules(self) -> dict[str, ControlModuleBase]:
        return self._control_modules

    def reset(self, observation: Observation):
        pass

    def step(self) -> Action:
        pass

    def update(self, action: Action, action_rv: ActionRV, observation: Observation):
        pass

    def get_observation_reset(self) -> Observation:
        return Observation()

    def get_observation_step(self) -> Observation:
        return Observation()
    
    def shutdown(self):
        """
        Clean up resources, if necessary.
        This method can be overridden by subclasses to implement specific cleanup logic.
        """
        pass

