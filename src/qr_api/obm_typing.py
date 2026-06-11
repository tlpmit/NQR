import copy
from dataclasses import dataclass, field
from typing import Any, Dict, List

import numpy as np

import qr_api.perc_typing as T
from qr_api.policy_typing import ObjectTransitionPrediction
from qr_utils.traceFile import tr

UNSET = object()

# trace tag is 'obm'

# Todo
# - in_hand should be a probability
# - pose confidence delta should at least have different values for translation and rotation


@dataclass
class ObjectBelief(object):
    name: str
    """A unique human-readable name for the object."""

    m4_pose: T.M4Transformation
    """Object pose in world frame as a 4x4 numpy array."""

    mesh: T.UncertainObjectMesh
    """An instance of UncertainObjectMesh in its own local frame.  May be non-convex"""

    features: Dict[str, Any] = field(default_factory=dict)
    """A dictionary mapping names of other attributes to values."""

    cached_properties: Dict[str, Any] = field(default_factory=dict)
    """A dictionary of cached properties."""

    in_hand: bool = False
    """Is the object currently being held?"""

    existence_confidence: float = 0.7
    """The confidence in the object's existence."""

    pose_confidence_delta: float = 0.01
    """The size of a 99% confidence interval on the object pose."""

    detections: List[T.ObjectDetection] = field(default_factory=list)

    def __copy__(self):
        oc = self.__class__.__new__(self.__class__)
        oc.__dict__.update(self.__dict__)

        oc.mesh = copy.copy(self.mesh)
        oc.features = copy.copy(self.features)
        oc.cached_properties = copy.copy(self.cached_properties)
        return oc

    def get_feature(self, name: str, default: Any = UNSET) -> Any:
        if name in self.features:
            return self.features[name]
        if default is not UNSET:
            return default
        raise KeyError(
            f"Feature '{name}' not found. Available features: {list(self.features.keys())}"
        )

    def has_feature(self, name: str) -> bool:
        return name in self.features

    def put_feature(self, name: str, feature: Any):
        self.features[name] = feature

    def get_cached_property(self, name: str, default: Any = UNSET) -> Any:
        if name in self.cached_properties:
            tr(("obm_detail", "log"), f"Using cached property {name}")
            return self.cached_properties[name]
        if default is not UNSET:
            return default
        raise KeyError(
            f"Property '{name}' not found. Available properties: {list(self.cached_properties.keys())}"  # noqa: E501
        )

    def has_cached_property(self, name: str) -> bool:
        return name in self.cached_properties

    def put_cached_property(self, name: str, value: Any):
        self.cached_properties[name] = value

    @property
    def X_world_obj(self) -> T.M4Transformation:
        """The pose of the object in the world frame."""
        return self.m4_pose

    @property
    def X_obj_world(self) -> T.M4Transformation:
        """The pose of the object in the world frame."""
        return np.linalg.inv(self.m4_pose)

    @property
    def mesh_worldframe(self) -> T.UncertainObjectMesh:
        """Meshes in the world frame. Note that this function can not be cached because
        the pose can change."""
        return self.mesh.apply_transform(self.m4_pose)

    def print(self, tag: str = ("obm", "log")):
        if self.m4_pose is not None:
            rt = self.m4_pose
            translation = rt[:3, 3].tolist()
            rotation = rt[:3, :3].tolist()
            tr(
                tag,
                f"{self.name}: existence_confidence: {self.existence_confidence} in_hand: {self.in_hand}",  # noqa: E501
            )
            tr(tag, f"    pose: {translation}, {rotation}")
            lbs = self.mesh.bounds[0] + translation
            ubs = self.mesh.bounds[1] + translation
            tr(tag, "    pcd/aabb/lower:", lbs)
            tr(tag, "    pcd/aabb/upper:", ubs)
        else:
            tr(("obm", "log"), f"{self.name} (id={self.identifier})\nno pose")
        for k, v in self.features.items():
            tr(("obm", "log"), f"    feature/{k}: {v}")

    def to_o3d(self, false_color: bool = True) -> "open3d.geometry.TriangleMesh":
        """Convert the object to an open3d mesh"""
        if false_color:
            rgb = np.random.randint(0, 255, size=(3,), dtype=np.uint8)
        else:
            rgb = self.get_feature(
                "rgb_average", np.array([100, 100, 100], dtype=np.uint8)
            )
        return self.mesh_worldframe.to_o3d(rgb)

    def transition_update(self, prediction: ObjectTransitionPrediction):
        """Update the object belief based on a transition prediction."""
        raise NotImplementedError("This method should be implemented in subclasses.")


@dataclass
class ObjectMemory(object):
    objects: Dict[str, ObjectBelief] = field(default_factory=dict)
    """dictionary from internal names to instances of :class:`ObjectState`"""

    def __getitem__(self, name: str) -> ObjectBelief:
        # Hack to make running from pkl work more often, when the object name is not the same
        if name not in self.objects:
            name_prefix = name.split("_")[0]
            for obj_name in self.objects:
                if obj_name.split("_")[0] == name_prefix:
                    tr(
                        "terminal",
                        f"Warning: Object '{name}' not found, using '{obj_name}' instead.",
                    )
                    return self.objects[obj_name]
            raise KeyError(f"Object '{name}' not found in memory.")
        # Return the object if it exists
        return self.objects[name]

    def __setitem__(self, name: str, obj: ObjectBelief):
        self.objects[name] = obj

    def __contains__(self, name: str) -> bool:
        return name in self.objects

    def add_hypothesis(self, obj: ObjectBelief):
        """Add a new object hypothesis to the memory."""
        if obj.name in self.objects:
            raise KeyError(f"Object '{obj.name}' already exists in memory.")
        self.objects[obj.name] = obj

    def remove_hypothesis(self, name: str):
        """Remove an object hypothesis from the memory."""
        if name not in self.objects:
            raise KeyError(f"Object '{name}' not found in memory.")
        del self.objects[name]

    def delete_low_likelihood_hypotheses(self, threshold: float):
        """Delete hypotheses with low likelihood."""
        to_delete = [
            name
            for name, obj in self.objects.items()
            if obj.existence_confidence < threshold
        ]
        for name in to_delete:
            del self.objects[name]

    def get_objects(self) -> List[ObjectBelief]:
        """Get a list of all objects in memory."""
        return list(self.objects.values())

    def get_ml_objects(self) -> List[ObjectBelief]:
        """Get max likelihood list of objects in memory."""
        from qr_belief.obm.ml_object_set import get_ml_object_set

        return get_ml_object_set([copy.copy(o) for o in self.objects.values()])

    def print(self, tag=("obm", "log")):
        tr(tag, "\n*** OBM State ***")
        for o in self.get_objects():
            o.print(tag)

    def show(self, false_color: bool = True):
        """Show the current state of the object memory."""
        tr(
            "terminal",
            f"Showing OBM state {'in false color' if false_color else 'in perceived colors'}",
        )
        from qr_utils.pcd_utilities import open3d_show

        meshes = [o.to_o3d(false_color=false_color) for o in self.get_objects()]
        open3d_show(meshes)

    def get_cached_object_property(
        self, object_name: str, property_name: str, default: Any = UNSET
    ) -> Any:
        """Get a cached property of an object."""
        if object_name not in self.objects:
            if default is not UNSET:
                return default
            raise KeyError(f"Object '{object_name}' not found in memory.")
        return self.objects[object_name].get_cached_property(property_name, default)

    def put_cached_object_property(
        self, object_name: str, property_name: str, value: Any
    ):
        """Set a cached property of an object."""
        if object_name not in self.objects:
            raise KeyError(f"Object '{object_name}' not found in memory.")
        self.objects[object_name].put_cached_property(property_name, value)

    def has_cached_object_property(self, object_name: str, property_name: str) -> bool:
        """Check if an object has a cached property."""
        if object_name not in self.objects:
            return False
        return self.objects[object_name].has_cached_property(property_name)
