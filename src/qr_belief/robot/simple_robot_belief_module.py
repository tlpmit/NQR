from qr_api.belief_interfaces import RobotMemoryModule
from qr_api.policy_typing import Action, ActionRV, TransitionPrediction
from qr_api.sensor_typing import Observation
from qr_utils.traceFile import tr, tr_a


class SimpleRobotMemoryModule(RobotMemoryModule):
    def __init__(self):
        super().__init__()
        self._robot_conf = None
        self._holding = {}  # dict from hand to (obj, grasp)

    def _reset(self, observation: Observation):
        self._robot_conf = observation["conf"]
        self._holding = {}

    # TODO: Handle gripper width in a more subtle way
    def _update(
        self,
        action: Action,
        actionrv: ActionRV,
        transition_prediction: TransitionPrediction,
        observation: Observation,
    ):
        self._robot_conf = observation["conf"]
        # Update holding based on the predictions
        for obj_prediction in transition_prediction.object_predictions:
            obj = obj_prediction.object_name
            if "pose" in obj_prediction.attribute_updates:
                # We are not holding this object
                to_delete = []
                for hand, (held_obj, grasp) in self._holding.items():
                    if obj == held_obj:
                        to_delete.append(hand)
                for hand in to_delete:
                    del self._holding[hand]
            if "in_hand" in obj_prediction.attribute_updates:
                # We are holding this object
                (hand, grasp) = obj_prediction.attribute_updates["in_hand"]
                if self.holding(hand) is not None:
                    if (
                        grasp is None
                    ):  # another way of saying we are not holding anything
                        del self._holding[hand]
                        continue
                    # We think we are are already holding something in this hand
                    (held_obj, _held_grasp) = self.holding(hand)
                    tr_a(
                        "Just picked up",
                        obj,
                        "with",
                        hand,
                        "but already thought we were holding",
                        held_obj,
                    )
                self._holding[hand] = (obj, grasp)

        # But, really, trust what our observation tells us
        gripper_empty_threshold = 0.005
        # The old repo's conf was a Roboverse Conf (with .value/.get_grip);
        # the new virtual robots report a plain chain-name → array dict.
        conf_dict = (
            self._robot_conf.value
            if hasattr(self._robot_conf, "value")
            else self._robot_conf
        )
        hands = set()
        if "left_gripper" in conf_dict.keys():
            hands.add("left")
        if "right_gripper" in conf_dict.keys():
            hands.add("right")

        def grip(hand):
            if hasattr(self._robot_conf, "get_grip"):
                return self._robot_conf.get_grip(hand)
            return float(conf_dict[f"{hand}_gripper"][0])

        for hand in hands:
            gripper_empty = grip(hand) < gripper_empty_threshold
            if gripper_empty and self.holding(hand) is not None:
                (obj, _grasp) = self.holding(hand)
                print(f"!! We seem to have droppeed {obj}!!")
                del self._holding[hand]

        self.print()

    @property
    def robot_conf(self):
        return self._robot_conf

    def holding(self, hand=None):
        if hand is None:
            return self._holding
        if hand not in self._holding:
            return None
        else:
            return self._holding[hand]

    def get_held_objects(self):
        return [obj for obj, grasp in self._holding.items()]

    def print(self, tag="terminal"):
        tr(tag, "\n==== Robot memory module state ====")
        tr(tag, "Holding:", self._holding)
        tr(tag, "Robot:", self.robot_conf)
        tr(tag, "===================================")
