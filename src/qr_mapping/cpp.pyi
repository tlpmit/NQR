"""
qr_mapping C++ bindings
"""
from __future__ import annotations
import numpy
import pydrake.geometry
__all__ = ['VoxelGridConfig', 'VoxelGridEstimator']
class VoxelGridConfig:
    cuda_device: int
    filled_color: pydrake.geometry.Rgba
    grid_resolution: float
    max_depth: float
    max_threads: int
    min_depth: float
    num_cameras_seen_free: int
    opencl_device_index: int
    opencl_platform_index: int
    outlier_points_threshold: float
    percent_seen_free: float
    pitch_origin: float
    roll_origin: float
    step_size_multiplier: float
    unknown_color: pydrake.geometry.Rgba
    voxelizer_option: str
    x_origin: float
    x_size: float
    y_origin: float
    y_size: float
    yaw_origin: float
    z_origin: float
    z_size: float
    def __init__(self) -> None:
        ...
class VoxelGridEstimator:
    def GetOccupancy(self) -> numpy.ndarray[numpy.uint8[..., ..., ...]]:
        ...
    def GetVoxelCenters(self) -> numpy.ndarray[numpy.float64[..., ..., ..., ...]]:
        ...
    def UpdateFromDepthImage(self, depth_image: ..., intrinsics: numpy.ndarray[numpy.float64[3, 3]], extrinsics: numpy.ndarray[numpy.float64[4, 4]], color_image: ... = None, max_depth_points_to_publish: int = -1, viz_point_size: float = 0.001) -> None:
        ...
    def __init__(self, config: VoxelGridConfig, meshcat: pydrake.geometry.Meshcat) -> None:
        ...
