from typing import TYPE_CHECKING, Any
from dataclasses import dataclass

from qr_api.sensor_typing import Observation

if TYPE_CHECKING:
    from qr_api.belief_interfaces import BeliefModule
    from qr_api.control_interfaces import ControlModuleBase
else:
    BeliefModule = 'BeliefModule'
    ControlModuleBase = 'ControlModule'


class ActionRV(object):
    pass


class ActionExecutionException(Exception):
    pass


class PlanFinishedException(Exception):
    pass


class PlanFailedException(Exception):
    pass

@dataclass
class ObjectTransitionPrediction(object):
    """An object and its predicted state change based on this action.
    This is used to update the belief module.
    """
    object_name: str
    # Assume the belief module knows how to interpret these updates
    # They will generally be something like {'pose' : (new_pose, delta)}
    attribute_updates: dict[str, Any]

@dataclass  
class TransitionPrediction(object):
    """A list of objects and predicted state changes based on this action.
    This is used to update the belief module.
    Could potentially also make predictions about robot state, but we assume for now that we can observe it well.
    """
    object_predictions: list[ObjectTransitionPrediction]  

class Action(object):
    """The basic Action class.

    An action implements the following methods:

    - `execute(belief_module, controller_interfaces) -> ActionRV`: execute the action.
    - `transition_prediction(belief_module, action_rv) -> TransitionPrediction`: predict the state change of the world based on this action.
    """

    def execute(self, belief_module: BeliefModule, controller_interfaces: dict[str, ControlModuleBase]) -> ActionRV:
        pass

    def transition_prediction(self, belief_module: BeliefModule, action_rv: ActionRV) -> TransitionPrediction:
        """Predict the state change of the world based on this action.
        This is used to update the belief module.
        """
        return TransitionPrediction(object_predictions=[])
    
    # Given the transition prediction, we may not need these methods
    def update_belief_before(self, belief_module: BeliefModule, action_rv: ActionRV, observation: Observation):
        pass
    def update_belief_after(self, belief_module: BeliefModule, action_rv: ActionRV, observation: Observation):
        pass
