from typing import Tuple
from abc import ABC, abstractmethod
from qr_api.belief_interfaces import BeliefModule
# Some of the VV methods should be in here, too

class BeliefViewForPlanner:
    """
    A class to represent a belief view from the planner's perspective.
    It provides methods to access the belief state and its properties.
    """

    def __init__(self, QR_bel: BeliefModule):
        self._QR_bel = QR_bel

    @property
    def QR_bel(self) -> BeliefModule:
        return self._QR_bel
    
    @abstractmethod
    def get_obj_pose_prob(self, obj_name, pose, delta) -> float:
        pass

    @abstractmethod
    def get_phys_obj_pose(self, obj_name):
        pass

    @abstractmethod
    def get_phys_object_names(self) -> list[str]:
        pass
    
    @abstractmethod
    def get_phys_permanent_object_names(self) -> list[str]:
        pass
    
    @abstractmethod
    def get_obj_held_prob(self, obj_name):
        """
        Returns the probability of an object being held.
        """
        pass
    
    @abstractmethod
    def get_phys_obj_held(self, obj_name : str) -> bool:
        """
        Returns whether an object is being held in the physical state.
        """
        pass
    
    @abstractmethod
    def get_available_prob(self, effector_name) -> float:
        """
        Returns the probability of an effector being available (for picking or sensing).
        """
        pass
    
    @abstractmethod
    def get_sensor_names(self) -> list[str]:
        """
        Returns a list of sensor names for this robot.
        """
        pass
    
    @abstractmethod
    def get_holding_prob(self, obj_name, hand, grasp, delta) -> float:
        """
        Returns the probability of an object being held by a specific hand with a specific grasp.
        
        :param obj_name: Name of the object.
        :param hand: Name of the hand.
        :param grasp: Name of the grasp.
        :param delta: Tolerance for the holding check.
        :return: Probability of the object being held by the specified hand with the specified grasp.
        """
        pass
    
    @abstractmethod
    def phys_legal_path(self, path_prog, perm_only = False, lazy = True)  -> bool:
        """
        Checks if a path program is legal in the physical state.
        
        :param path_prog: Path program to check.
        :return: True if the path program is legal, False otherwise.

        :param lazy: If True, only check the start and end configurations, otherwise find whole path and side-effect the path_prog

        """
        pass

    @abstractmethod
    def get_phys_path_viols(self, path_prog, perm_only=False, only_one=False,
                       ignore_finger_shadow_collisions=False) -> Tuple[set[str], set[str]]:
        """
        Returns a list of violations for a path program in the physical state.
        
        :param path_prog: Path program to check.
        :return: List of violations.
        
        :param lazy: If True, only check the start and end configurations, otherwise find whole path and side-effect the path_prog
        """
        pass
    
    @abstractmethod
    def get_obj_attr_prob(self, obj_name, attr_name, attr_value) -> float:
        """
        Returns the probability of an object having a specific attribute value.
        
        :param obj_name: Name of the object.
        :param attr_name: Name of the attribute.
        :param attr_value: Value of the attribute to check.
        :return: Probability of the object having the specified attribute value.
        """
        pass
    
    @abstractmethod
    def get_obj_ml_attr(self, obj, attr):
        """
        Returns the most likely value of an attribute for a given object.
        
        :param obj: Name of the object.
        :param attr: Name of the attribute.
        :return: Most likely value of the attribute for the specified object.
        """
        pass

    @abstractmethod
    def get_objects_ml_with_attr(self, attr_name: str, attr_value: any) -> list[str]:
        """
        Returns a list of objects that whose most likely value for a given attribute matches the specified value.
        
        :param attr_name: Name of the attribute.
        :param attr_value: Value of the attribute to match.
        :return: List of object names that match the specified attribute value.
        """
        pass