from typing import Any

from qr_api.perc_typing import ObjectDetection
from qr_api.belief_interfaces import ObjectMemoryModule
from qr_api.obm_typing import ObjectBelief
from qr_api.policy_typing import ObjectTransitionPrediction

class OBMMatchScoreFunction(object):
    def __call__(self, hypothesis: ObjectBelief, detection: ObjectDetection) -> float:
        raise NotImplementedError()

class OBMMergeFunction(object):
    def __call__(self, hypothesis: ObjectBelief, detection: ObjectDetection) -> ObjectBelief:
        """
        Merges a hypothesis with a detection.
        """
        raise NotImplementedError()
    
class OBMInitBeliefFunction(object):
    def __call__(self, detection: ObjectDetection) -> ObjectBelief:
        """
        Initializes a new hypothesis from a detection.
        """
        raise NotImplementedError()

class OBMObjectTransitionUpdateFunction(object):
    """
    Update the belief about a hypothsis based on a prediction about its transition
    Works by side-effect on
    """
    def __call__(self, hypothesis: ObjectBelief, transition_prediction: ObjectTransitionPrediction):
        raise NotImplementedError()