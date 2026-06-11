from dataclasses import dataclass, field
from functools import cached_property
from typing import TYPE_CHECKING, Any, Dict, List, Optional, Tuple

import numpy as np

from qr_utils.pcd_utilities import (
    open3d_show,
    point_cloud_from_depth_image_camera_frame,
    point_cloud_from_depth_image_world_frame,
    recenter_vertices,
)

if TYPE_CHECKING:
    import open3d
    import trimesh

UNSET = object()


RGBImage = np.ndarray
DepthImage = np.ndarray
RGBDImage = np.ndarray

M3Rotaion = np.ndarray  # Rotation matrix
M4Transformation = np.ndarray  # Transformation matrix
SurfaceEquation = np.ndarray
ImageMask = np.ndarray
PointCloudMask = np.ndarray


@dataclass
class Surface(object):
    """A surface in 3D space. It is segmented from a 3D point cloud."""

    surface_eqn: SurfaceEquation
    """A 4x1 array representing the equation of the surface in the form ax + by + cz + d = 0."""

    surface_mask: PointCloudMask
    """A (N) binary mask indicating the points that belong to the surface."""


SurfaceList = list[Surface]

Segmentation = np.ndarray
SegmentationList = list[np.ndarray]  # shape (N)
NumpyPointCloud = np.ndarray
NumpyPointCloudList = list[np.ndarray]
PointCloud = NumpyPointCloud

# May not need these
TrimeshPointCloud = "trimesh.PointCloud"
TrimeshPointCloudList = list[TrimeshPointCloud]
Open3DPointCloud = "open3d.geometry.PointCloud"
Open3DPointCloudList = list[Open3DPointCloud]

ImageFeature = np.ndarray
PointCloudFeature = np.ndarray
ObjectFeature = np.ndarray
ObjectFeatureList = list[np.ndarray]


@dataclass
class CalibratedRGBDObservation(object):
    """A data class that contains the raw input data for the perception pipeline."""

    rgb_image: RGBImage
    """The RGB image of the scene."""

    depth_image: DepthImage
    """The depth image of the scene."""

    camera_intrinsics: np.ndarray
    """The camera intrinsics matrix."""

    camera_extrinsics: np.ndarray
    """The camera extrinsics matrix (transform points from world to camera)."""

    camera_params: Optional[tuple] = None
    """Optional camera parameters, such as field of view, resolution, etc.  Not all derivable from intrinsics."""

    label_image: Optional[np.ndarray] = None
    """When using simulated perception, we can cheat and produce an image of object labels"""

    exclusion_bbox: Optional[np.ndarray] = None
    """Exclude point cloud points inside this box (in world coordinates)"""

    _point_cloud_world_frame: Optional[np.ndarray] = None

    # @property
    # def default_depth_image(self):
    #     return self.depth_image

    @property
    def camera_pose(self):
        return np.linalg.inv(self.camera_extrinsics)

    @property
    def point_cloud_world_frame(self) -> NumpyPointCloud:
        """The point cloud in world frame. It is represented as a Nx3 array. The entries are ordered in the same way as image.flatten()."""
        if self._point_cloud_world_frame is None:
            self._point_cloud_world_frame = point_cloud_from_depth_image_world_frame(
                self.depth_image, self.camera_intrinsics, self.camera_extrinsics
            )
            if self.exclusion_bbox is not None:
                lower, upper = self.exclusion_bbox
                points = self._point_cloud_world_frame
                inside_mask = np.all((points >= lower) & (points <= upper), axis=1)
                self._point_cloud_world_frame[inside_mask] = np.nan
        return self._point_cloud_world_frame

    @point_cloud_world_frame.setter
    def point_cloud_world_frame(self, value):
        self._point_cloud_world_frame = value

    @property
    def point_cloud(self):
        """An alias for :attr:`point_cloud_world_frame`."""
        return self.point_cloud_world_frame

    @cached_property
    def point_cloud_camera_frame(self) -> NumpyPointCloud:
        """The point cloud in camera frame. It is represented as a Nx3 array. The entries are ordered in the same way as image.flatten()."""
        return point_cloud_from_depth_image_camera_frame(
            self.depth_image, self.camera_intrinsics
        )

    def show_rgb(self):
        import matplotlib.pyplot as plt

        plt.imshow(self.rgb_image)
        plt.show()

    def show_depth(self):
        import matplotlib.pyplot as plt

        plt.imshow(self.depth_image)
        plt.show()


CalibratedRGBDObservationList = list[CalibratedRGBDObservation]


@dataclass
class SceneRepresentation(object):
    """ "A data class that contains the raw input data for the perception pipeline and scene-level features."""

    calibrated_rgbds: tuple[CalibratedRGBDObservation, ...]
    """The raw input data for the perception pipeline."""

    image_features: Dict[str, ImageFeature] = field(default_factory=dict)
    """A dictionary of image features. The keys are the feature names and the values are the feature arrays."""

    point_cloud_features: Dict[str, PointCloudFeature] = field(default_factory=dict)
    """A dictionary of point cloud features. The keys are the feature names and the values are the feature arrays."""

    @classmethod
    def from_calibrated_rgbds(
        cls, calibrated_rgbds: tuple[CalibratedRGBDObservation, ...]
    ):
        return cls(tuple(calibrated_rgbds))

    @classmethod
    def from_single_calibrated_rgbd(cls, calibrated_rgbd: CalibratedRGBDObservation):
        return cls((calibrated_rgbd,))

    def put_image_feature(self, name: str, feature: ImageFeature):
        self.image_features[name] = feature

    def put_point_cloud_feature(self, name: str, feature: PointCloudFeature):
        self.point_cloud_features[name] = feature

    def has_image_feature(self, name: str) -> bool:
        return name in self.image_features

    def has_point_cloud_feature(self, name: str) -> bool:
        return name in self.point_cloud_features

    def get_image_feature(self, name: str, default: Any = UNSET) -> ImageFeature:
        if name not in self.image_features:
            if default is not UNSET:
                return default
            raise KeyError(
                f"Image feature '{name}' not found. Available features: {list(self.image_features.keys())}"
            )
        return self.image_features[name]

    def get_point_cloud_feature(
        self, name: str, default: Any = UNSET
    ) -> PointCloudFeature:
        if name not in self.point_cloud_features:
            if default is not UNSET:
                return default
            raise KeyError(
                f"Point cloud feature '{name}' not found. Available features: {list(self.point_cloud_features.keys())}"
            )
        return self.point_cloud_features[name]


@dataclass
class UncertainObjectMesh(object):
    """A data class with basic mesh representation and binary certainty on vertices"""

    vertices: np.ndarray[np.float32]
    """The vertices of the mesh. It is represented as a Nx3 array."""
    faces: np.ndarray[np.int32]
    """The faces of the mesh. It is represented as a Mx3 array."""
    certainty: np.ndarray[np.bool_]
    """The certainty of the vertices. It is represented as a Nx1 array, indicating whether the vertex was observed or made up"""

    def to_o3d(
        self, rgb: Optional[np.ndarray] = None
    ) -> "open3d.geometry.TriangleMesh":
        """Convert the mesh to Open3D format."""
        import open3d

        if rgb is None:
            rgb = np.array([100, 100, 100])
        mesh = open3d.geometry.TriangleMesh()
        mesh.vertices = open3d.utility.Vector3dVector(self.vertices)
        mesh.triangles = open3d.utility.Vector3iVector(self.faces)
        mesh.vertex_colors = open3d.utility.Vector3dVector(
            np.ones((self.vertices.shape[0], 3)) * rgb / 255.0
        )
        # TODO: color uncertain vertices black
        return mesh

    def to_trimesh(self) -> "trimesh.Trimesh":
        import trimesh

        mesh = trimesh.Trimesh(vertices=self.vertices, faces=self.faces)
        return mesh

    def centered(self) -> Tuple["UncertainObjectMesh", np.ndarray, np.ndarray]:
        centered_vertices, uncenter_tform, center_tform = recenter_vertices(
            self.vertices
        )
        return (
            UncertainObjectMesh(centered_vertices, self.faces, self.certainty),
            uncenter_tform,
            center_tform,
        )

    def apply_transform(self, transform: M4Transformation) -> "UncertainObjectMesh":
        """Apply a transformation to the mesh."""
        import trimesh

        mesh = trimesh.Trimesh(vertices=self.vertices, faces=self.faces)
        transformed_mesh = mesh.apply_transform(transform)
        return UncertainObjectMesh(
            transformed_mesh.vertices, transformed_mesh.faces, self.certainty
        )

    @cached_property
    def bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get the bounds of the mesh."""
        import trimesh

        mesh = trimesh.Trimesh(vertices=self.vertices, faces=self.faces)
        return mesh.bounds


@dataclass
class ObjectDetection(object):
    """A data class that contains the geometry of an object."""

    scene: SceneRepresentation
    """The scene representation where the object belongs to."""

    segmentation_masks: tuple[tuple[CalibratedRGBDObservation, ImageMask], ...]
    """The segmentation mask of the object. It is represented as a list of (image, 
    binary mask) pairs.
        """

    _mesh: UncertainObjectMesh = None

    @property
    def mesh(self):
        return self._mesh

    @mesh.setter
    def mesh(self, value):
        if self._mesh is None or value is not None:
            self._mesh = value
        else:
            print("Setting detection mesh with None")
            pass

    features: Dict[str, Any] = field(default_factory=dict)
    """A dictionary of object features. The keys are the feature names and the values are the feature values.
        This can be used for values (like rgb_average or whether it's a partial view) and feature maps (like clip) """

    @cached_property
    def point_cloud(self) -> NumpyPointCloud:
        """The point cloud of the object. It is represented as a Nx3 array."""
        # Assume a single image
        assert len(self.segmentation_masks) == 1, (
            "Object detection should have a single segmentation mask"
        )
        assert len(self.segmentation_masks[0]) == 2, (
            "Segmentation mask should be a tuple of (image, mask)"
        )
        im, mask = self.segmentation_masks[0]
        obj_pcd = im.point_cloud_world_frame[mask.flatten()]
        # Remove "zero" points from object point cloud
        obj_pcd = obj_pcd[np.logical_not(np.all(obj_pcd == 0.0, axis=1))]
        return obj_pcd

    def has_feature(self, name: str) -> bool:
        return name in self.features

    def get_feature(self, name: str, default: Any = None) -> ObjectFeature:
        if name not in self.features:
            if default is not UNSET:
                return default
            raise KeyError(
                f"Feature '{name}' not found. Available features: {list(self.features.keys())}"
            )
        return self.features[name]

    def put_feature(self, name: str, feature: ObjectFeature):
        self.features[name] = feature

    def to_o3d(self, false_color: bool = False) -> "open3d.geometry.TriangleMesh":
        """Convert the mesh to Open3D format."""
        if false_color:
            rgb = np.random.randint(0, 255, size=(3,))
        else:
            rgb = self.features.get(
                "rgb_average", np.array([100, 100, 100], dtype=np.uint8)
            )
        if self.mesh is None:
            pass
        return self.mesh.to_o3d(rgb)

    @cached_property
    def descriptive_string(self) -> str:
        """A descriptive name for the object, if we have one"""
        from qr_utils.colors import get_best_color_name

        color = self.features.get("rgb_average", None)
        category = self.features.get("category", None)
        color_str = get_best_color_name(color) if color is not None else ""
        category_str = category if category is not None else "thing"
        return f"{color_str}_{category_str}"


ObjectDetectionList = list[ObjectDetection]


@dataclass
class SegmentationHypothesis(object):
    object_hypotheses: ObjectDetectionList
    probabilty: float


UncertainObjectDetection = List[SegmentationHypothesis]


@dataclass
class ObjectCentricSceneRepresentation(object):
    scene_representation: SceneRepresentation
    """The scene representation."""

    object_detections: List[ObjectDetection] = field(default_factory=list)
    """A list of object representations."""

    def add_object_detection(self, object_detection: ObjectDetection):
        self.object_detections.append(object_detection)

    def get_object_detection(self, index: int) -> ObjectDetection:
        return self.object_detections[index]

    @classmethod
    def from_scene_representation(cls, scene_representation: SceneRepresentation):
        return cls(scene_representation)


# Uncos-style output: a list of certain objects and a list of uncertain regions, each of
#  which has multiple segmentaiton hypotheses
@dataclass
class UncertainMaskOutput(object):
    certain_segments: List[ImageMask] = field(default_factory=list)
    uncertain_segments: List[Tuple[List[ImageMask], float]] = field(
        default_factory=list
    )


def prod_shape(shape):
    return np.prod(shape)


# Uncertain segmentation structure with object detection instances
@dataclass
class UncertainObjectSegmentation(object):
    certain_object_detections: List[ObjectDetection] = field(default_factory=list)
    """A list of object representations."""

    uncertain_object_detections: List[UncertainObjectDetection] = field(
        default_factory=list
    )
    """A list of uncertain object representations."""

    def show(self, title: str = "", false_color: bool = False):
        print(title)
        ml_objects = self.get_ml_object_segmentation()
        # print(f"Displaying {[o.descriptive_string for o in ml_objects]}")
        meshes = [h.to_o3d(false_color=false_color) for h in ml_objects if h.mesh]
        open3d_show(meshes)

    def get_ml_object_segmentation(self):
        """Get the ML object segmentation. This is a list of object detections."""
        result = self.certain_object_detections.copy()
        for h in self.uncertain_object_detections:
            result.extend(h.object_hypotheses)
        assert all(det.mesh is not None for det in result)
        return result


@dataclass
class UncertainObjectCentricSceneRepresentation(object):
    scene_representation: SceneRepresentation
    """The scene representation."""

    object_detections: UncertainObjectSegmentation
    """An uncertain object segmentation."""

    def get_ml_object_scene_representation(self) -> ObjectCentricSceneRepresentation:
        return ObjectCentricSceneRepresentation(
            self.scene_representation,
            self.object_detections.get_ml_object_segmentation(),
        )
