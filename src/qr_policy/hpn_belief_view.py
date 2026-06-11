from functools import reduce
from typing import Sequence, Tuple

from Domains.btamp.btamp_utils import condition_vals
from RetroPlan.planner import State
from Roboverse import Pose6
from Roboverse.physical_world.physical import Physical

from qr_api.belief_interfaces import BeliefModule
from qr_api.planning_interfaces import BeliefViewForPlanner
from qr_utils.traceFile import tr, tr_a

# Some of the VV methods should potentially be in here, too


class BeliefViewForHPN(BeliefViewForPlanner, State):
    """
    A class to represent a belief view from the planner's perspective.
    It provides methods to access the belief state and its properties.
    """

    def __init__(self, QR_bel: BeliefModule, phys: Physical):
        self._QR_bel = QR_bel
        self._phys = phys  # A concrete physical state, roughly the ML configuration
        self._local_vv = None  # A local copy of vv used only for debugging
        # Note that we will store the cache of fluent satisfaction values here
        State.__init__(self)

    @property
    def phys(self) -> Physical:
        return self._phys

    def __copy__(self) -> "BeliefViewForHPN":
        """
        Returns a copy of the belief view, but *** without the QR bel ***
        """
        return BeliefViewForHPN(None, self._phys.copy())

    def get_obj_pose_prob(self, obj_name: str, pose: Pose6, delta: float) -> float:
        """
        Returns the probability of an object being within delta a specific pose.
        """
        # For now, just making the interfaces look probabilistic!
        # TODO fix these!
        true_pose = self.get_phys_obj_pose(obj_name)
        is_near = pose.near(true_pose, delta, 2 * delta)
        return 0.99 if is_near else 0.01

    def get_obj_descr_prob(self, obj: str, descr: str):
        obm = self.QR_bel.object_memory
        if obj not in obm.memory:
            tr_a(f"Object {obj} not found in object memory.")
            return 0.0
        obj_bel = obm.memory[obj]
        propkey = ("has_description", descr)
        if obj_bel.has_feature(propkey):
            return obj_bel.get_feature(propkey)
        tr("Asked for object description but no precomputed result")
        return 0

    def get_obj_shape_prob(self, obj: str):
        """
        Returns a probability-like number indicating the certainty we have about the
        shape of the object.
        For now, assume that surfaces have known shape (we don't really care about
        their full extent) and otherwise, if curtailed is False, then we know it.
        """
        if self.QR_bel is None:
            return 0.99
        obm = self.QR_bel.object_memory
        if obj not in obm.memory:
            # Check in phys in case it's a built-in object
            if self.phys.get_body(obj) is not None:
                return 0.99
            return 0.0
        obj_bel = obm.memory[obj]
        if obj_bel.get_feature("category", False) == "surface":
            return 0.99
        if obj_bel.get_feature("curtailed", False):
            return 0.01
        return 0.99

    def get_phys_obj_pose(self, obj_name: str) -> Pose6:
        return self.phys.get_body_pose(obj_name)

    def get_phys_object_names(self) -> list[str]:
        return self.phys.get_bnames()

    def get_phys_permanent_object_names(self) -> list[str]:
        return self.phys.permanent_bnames

    def get_obj_held_prob(self, obj_name: str) -> float:
        return 0.99 if self.phys.get_obj_held_prob(obj_name) else 0.01

    def get_phys_obj_held(self, obj_name: str) -> bool:
        """
        Returns whether an object is being held in the physical state.

        :param obj_name: Name of the object.
        :return: True if the object is held, False otherwise.
        """
        return self.phys.get_obj_held(obj_name)

    def get_available_prob(self, effector_name):
        """
        Returns the probability of an effector being available (for picking or sensing).

        :param obj_name: Name of the object.
        :return: Probability of the object being available.
        """
        return 0.99 if self.phys.is_available(effector_name) else 0.01

    def get_sensor_names(self) -> list[str]:
        """
        Returns a list of sensor names in the physical state.

        :return: List of sensor names.
        """
        return self.phys.robot.get_sensor_names()

    def get_holding_prob(self, obj_name, hand, grasp, delta) -> float:
        """
        Returns the probability of an object being held by a specific hand with a specific grasp.

        :param obj_name: Name of the object.
        :param hand: Name of the hand.
        :param grasp: Name of the grasp.
        :param delta: Tolerance for the holding check.
        :return: Probability of the object being held by the specified hand with the specified grasp.
        """
        return 0.99 if self.phys.is_holding(obj_name, hand, grasp) else 0.01

    def get_phys_legal_path(
        self, path_prog, perm_only=False, lazy=True, ignore_shadows=False
    ) -> bool:
        """
        Checks if a path program is legal in the physical state.

        :param lazy: If True, only check the start and end configurations, otherwise find whole path and side-effect the path_prog

        """
        if not lazy:
            # Finds a path and side effects the path argument
            return self.phys.cc.legal_path_exists(
                path_prog,
                self.phys.get_attached(),
                perm_only=perm_only,
                ignore_shadows=ignore_shadows,
            )
        else:
            # Just check the end points
            return self.phys.cc.legal_path_key(
                path_prog.path_key, perm_only, ignore_shadows=ignore_shadows
            )

    def get_legal_path_prob(
        self, path_prog, perm_only=False, ignore_shadows=False
    ) -> float:
        """
        Returns the probability of a path program being legal in the physical state.

        :param lazy: If True, only check the start and end configurations, otherwise find whole path and side-effect the path_prog
        :return: Probability of the path program being legal.
        """
        return (
            0.99
            if self.get_phys_legal_path(
                path_prog, perm_only=perm_only, lazy=True, ignore_shadows=ignore_shadows
            )
            else 0.01
        )

    def get_phys_path_viols(
        self,
        path_prog,
        perm_only=False,
        only_one=False,
        ignore_finger_shadow_collisions=False,
    ) -> Tuple[set[str], set[str]]:
        """
        Returns a list of violations for a path program in the physical state.

        :param path_prog: Path program to check.
        :return: List of violations.

        :param lazy: If True, only check the start and end configurations, otherwise find whole path and side-effect the path_prog
        """
        return self.phys.cc.path_violations(
            path_prog,
            only_one=only_one,
            perm_only=perm_only,
            ignore_finger_shadow_collisions=ignore_finger_shadow_collisions,
        )

    def get_obj_attr_prob(
        self, obj_name: str, attr_name: str, attr_value: any
    ) -> float:
        if attr_name == "graspable":
            return self.get_graspable_prob(obj_name)
        return (
            0.99 if self.phys.test_body_attr(obj_name, attr_name, attr_value) else 0.01
        )

    def get_graspable_prob(self, obj: str) -> float:
        if self.QR_bel is None:
            return 0.99
        obm = self.QR_bel.object_memory
        if obj not in obm.memory:
            tr_a(f"Object {obj} not found in object memory.")
            # Assumes that built-in objects are not graspable
            return 0.0
        obj_bel = obm.memory[obj]
        if obj_bel.get_feature("category", False) == "surface":
            return 0.01
        if obj_bel.get_feature("curtailed", False):
            return 0.5
        return 0.99 if self.phys.get_body_attr(obj, "graspable") else 0.01

    def get_available_prob(self, hand):
        if self.phys.get_attached_body_name(hand):
            return 0.01
        if hand in self.phys.robot.hands:
            return 0.99 if self.phys.is_gripper_open(hand) else 0.01
        elif self.phys.robot.get_sensor(hand):
            return 0.99
        else:
            assert None, f"What is {hand=}?"

    def get_obj_ml_attr(self, obj, attr):
        return self.phys.get_body_attr(obj, attr)

    # For backward compatibility so domain constraints apply in both fully observed and
    # belief cases
    def get_object_attr(self, obj, attr):
        return self.get_obj_ml_attr(obj, attr)

    def get_objects_ml_with_attr(self, attr_name: str, attr_value: any) -> list[str]:
        """
        Returns a list of objects that whose most likely value for a given attribute matches the specified value.
        """
        return self.phys.bnames_with_attr_value(attr_name, attr_value)

    def get_objects_ml_satisfying(self, expr: Sequence) -> list[str]:
        """
        Returns a list of objects that satisfy the given expression.

        :param expr: Expression to check.
        :return: List of objects that satisfy the expression.
        """
        if expr[0] == "and":
            return reduce(
                set.intersection, [self.get_objects_ml_satisfying(e) for e in expr[1:]]
            )
        elif expr[0] == "or":
            return reduce(
                set.union, [self.get_objects_ml_satisfying(e) for e in expr[1:]]
            )
        else:
            (attr, value) = expr
            return self.get_objects_ml_with_attr(attr, value)

    def get_vv_regions(self):
        return self.get_vv().vv_regions

    def get_vv(self):
        if self._local_vv:  # Used only for debugging
            return self._local_vv
        return self.QR_bel.spatial_memory.vv

    def condition(self, conds, assignment=None):
        """
        Conditions the belief view on the given conditions.

        :param conds: Conditions to apply.
        :param assignment: Optional assignment for the conditions.
        :return: A new belief view conditioned on the given conditions.
        """
        vals = condition_vals(self, conds.apply_bindings(assignment))
        new_phys = self.phys.condition_from_vals(vals)
        if new_phys is None:
            tr("log", f"Conditioning on {conds} failed.")
            return None
        return BeliefViewForHPN(None, new_phys)
