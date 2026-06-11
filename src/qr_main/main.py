from qr_api.control_interfaces import ControlModuleBase
from qr_core.module import QRModule
from qr_api.policy_typing import PlanFailedException, PlanFinishedException
from qr_api.sensor_typing import Observation
from qr_api.belief_interfaces import BeliefModule
from qr_api.sensor_interfaces import InputModule
from qr_api.policy_interfaces import PolicyModule


class QRMain(QRModule):
    def __init__(self, sensory_interfaces: dict[str, InputModule], control_interfaces: dict[str, ControlModuleBase]):
        super().__init__()
        self._sensor_interfaces = sensory_interfaces
        self._control_interfaces = control_interfaces
        self._belief_module = None
        self._policy_module = None

    @property
    def sensor_modules(self) -> dict[str, InputModule]:
        """A mapping from the name of the sensory interfaces to the sensory interfaces."""
        return self._sensor_interfaces

    @property
    def control_modules(self) -> dict[str, ControlModuleBase]:
        """A mapping from the name of the control interfaces to the control interfaces."""
        return self._control_interfaces

    @property
    def belief_module(self) -> BeliefModule:
        assert self._belief_module is not None, 'The belief module is not set.'
        return self._belief_module

    @property
    def policy_module(self) -> PolicyModule:
        assert self._policy_module is not None, 'The policy module is not set.'
        return self._policy_module

    def set_belief_module(self, belief_module: BeliefModule):
        self._belief_module = belief_module

    def set_policy_module(self, policy_module: PolicyModule):
        self._policy_module = policy_module

    def main(self):
        assert self._belief_module is not None, 'The belief module is not set.'
        assert self._policy_module is not None, 'The policy module is not set.'

        try:

            observation = self._policy_module.get_observation_reset()
            self.belief_module.reset(observation)

            try:
                self.policy_module.reset(observation)
            except PlanFailedException as e:
                raise e

            # Main loop
            while True:
                try:
                    action = self.policy_module.step()
                except PlanFinishedException:
                    return
                except PlanFailedException as e:
                    raise e
                except KeyboardInterrupt:
                    return

                action_rv = action.execute(self._belief_module, self.control_modules)
                observation = self.policy_module.get_observation_step()
                self.belief_module.update(action, action_rv, observation)
                self.policy_module.update(action, action_rv, observation)
        except Exception as e:
            print(e)
            pass
        finally:
            self.policy_module.shutdown()

class QRMainDebug(QRModule):
    def __init__(self, sensory_interfaces: dict[str, InputModule], control_interfaces: dict[str, ControlModuleBase]):
        super().__init__()
        self._sensor_interfaces = sensory_interfaces
        self._control_interfaces = control_interfaces
        self._belief_module = None
        self._policy_module = None

    @property
    def sensor_modules(self) -> dict[str, InputModule]:
        """A mapping from the name of the sensory interfaces to the sensory interfaces."""
        return self._sensor_interfaces

    @property
    def control_modules(self) -> dict[str, ControlModuleBase]:
        """A mapping from the name of the control interfaces to the control interfaces."""
        return self._control_interfaces

    @property
    def belief_module(self) -> BeliefModule:
        assert self._belief_module is not None, 'The belief module is not set.'
        return self._belief_module

    @property
    def policy_module(self) -> PolicyModule:
        assert self._policy_module is not None, 'The policy module is not set.'
        return self._policy_module

    def set_belief_module(self, belief_module: BeliefModule):
        self._belief_module = belief_module

    def set_policy_module(self, policy_module: PolicyModule):
        self._policy_module = policy_module

    def main(self):
        assert self._belief_module is not None, 'The belief module is not set.'
        assert self._policy_module is not None, 'The policy module is not set.'

        observation = self._policy_module.get_observation_reset()
        self.belief_module.reset(observation)

        self.policy_module.reset(observation)

        # Main loop
        while True:

            action = self.policy_module.step()

            action_rv = action.execute(self._belief_module, self.control_modules)
            observation = self.policy_module.get_observation_step()
            self.belief_module.update(action, action_rv, observation)
            self.policy_module.update(action, action_rv, observation)
