import numpy as np
from dataclasses import dataclass, field

from qr_api.sensor_typing import RGBDImageInput, LanguageInstructionInput
from qr_core.event import QREvent
from qr_core.query import QRQueryService
from qr_core.typing import Pose


class HumanInstructionTimeRangeQueryService(QRQueryService):
    """A service to query the human instructions."""

    @dataclass
    class Query(object):
        timestamp_range: tuple[float, float] = (0, float('inf'))
        wait: bool = False

    @dataclass
    class Response(object):
        instructions: list[LanguageInstructionInput]


@dataclass
class HumanInstructionUpdateEvent(QREvent):
    """Human instructions have been updated."""

    instructions: list[LanguageInstructionInput]


@dataclass
class Region(object):
    """Region is a representation of a chunk of the 3D space. Currently represented as an oriented bounding box."""

    size: np.ndarray

    pose: Pose


@dataclass
class RegionPointcloud(object):
    """A point cloud that is associated with a region."""

    region: Region

    point_cloud: np.ndarray


@dataclass
class ObjectMask(object):
    """A mask for an object in the 3D space."""

    mask: np.ndarray

    image: RGBDImageInput


@dataclass
class ObjectRecord(object):
    """An object in the 3D space."""

    identifier: str
    """A unique identifier for the object memory module to reference this object."""

    query: str
    """The query that generated this object record."""

    associated_masks: list[ObjectMask]
    """The masks associated with this object."""

    point_cloud: np.ndarray

    completed_point_cloud: np.ndarray

    image_features: dict = field(default_factory=dict)


@dataclass
class ObjectTrajectory(object):
    """A trajectory of an object in the 3D space."""

    identifier: str
    """The identifier of the object."""

    trajectory: list[tuple[float, Pose]]
    """The trajectory of the object. (timestamp, pose)"""


class SpatialMemoryRegionQueryService(QRQueryService):
    """A service to query the spatial memory."""

    @dataclass
    class Query(object):
        """A query to find regions in the 3D space."""

        query: str
        """The query to find the region."""


    @dataclass
    class Response(object):
        """A result of a query to find regions in the 3D space."""

        regions: list[Region]
        """The regions that were found."""


class SpatialMemoryPointcloudQueryService(QRQueryService):
    """A service to query the spatial memory."""

    @dataclass
    class Query(object):
        """A query to find point clouds in the 3D space."""

        regions: list[Region]
        """The query to find the point cloud."""


    @dataclass
    class Response(object):
        """A result of a query to find point clouds in the 3D space."""

        point_clouds: list[RegionPointcloud]
        """The point clouds that were found."""


class ObjectMemoryFindQueryService(object):
    """A service to find objects in the 3D space."""

    @dataclass
    class Query(object):
        """A query to find objects in the 3D space."""

        query: str
        """The query to find the object."""

    @dataclass
    class Response(object):
        """A result of a query to find objects in the 3D space."""

        objects: list[ObjectRecord]
        """The objects that were found."""


class ObjectMemoryTrackQueryService(object):
    """A service to track objects in the 3D space."""

    @dataclass
    class Query(object):
        """A query to track objects in the 3D space."""

        identifier: list[str]
        """The query to track a list of objects."""


    @dataclass
    class Response(object):
        """A result of a query to track objects in the 3D space."""

        trajectories: list[ObjectTrajectory]


class ObjectMemoryUntrackQueryService(object):
    """A service to untrack objects in the 3D space."""

    @dataclass
    class Query(object):
        """A query to untrack objects in the 3D space."""

        identifier: list[str]
        """The query to untrack a list of objects."""


    @dataclass
    class Response(object):
        """A result of a query to untrack objects in the 3D space."""

        success: bool
        """Whether the untracking was successful."""
