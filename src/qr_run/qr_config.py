from dataclasses import dataclass, field
from typing import Union
import numpy as np

@dataclass
class HPNConfig:
    run_trajopt: bool = False
    """Whether to run trajectory optimization or not."""
    overrides: list = field(default_factory=list)
    """A list of overrides for the HPN configuration."""
    debug_tags: str = "full"
    """HPN debug-tag set name (see Domains.fully_observed_tamp.set_vars)."""
    debug_level: int = 1
    """HPN debug level; 0 disables the interactive plan_fail pauses."""
    log_level: int = 7
    """HPN log level."""
    interactive: bool = True
    """If False, clear HPN's interactive pause tags (headless runs)."""

@dataclass
class SpatialMemConfig:
    voxel_grid_resolution: float = 0.025
    """The resolution of the voxel grid used for spatial memory."""

@dataclass
class PerceptionConfig:
    min_z_for_planes: float = 0.025
    """Points below this z are ignored."""

@dataclass  
class QRSystemConfig:
    segmentation_method: str = 'uncos_service'
    """The segmentation method to use. Options are 'uncos_service', 'sim'."""
    
    completion_method: str = 'projection'
    """The completion method to use. Options are 'projection', 'box', 'scene_complete'."""

    use_eye_extrusion: bool = True
    """Whether to use eye extrusion in projection complettion."""

    eye_extrusion_distance: float = 0.03
    """The distance for eye extrusion in projection completion."""

    depth_method: str = 'raw'
    """The depyj method to use. Options are 'raw', 'DAV2' """

    policy_module: str = 'hpn_btamp'
    """The policy module to use. Options are 'crow_tamp', 'hpn_btamp', 'hpn_tamp'"""

    hpn_params: HPNConfig = field(default_factory=HPNConfig)
    """Parameters for the HPN policy module."""

    perception_params: PerceptionConfig = field(default_factory=PerceptionConfig)
    """Parameters for the perception modules."""

    spatial_mem_params: SpatialMemConfig = field(default_factory=SpatialMemConfig)
    """Parameters for the spatial memory module."""

    write_to_pkl_path: Union[str, None] = None
    """The path to the pkl file to write to. If None, will not write to pkl."""

    run_from_pkl_path: Union[str, None] = None
    """The path to the pkl file to run from. If None, will not run from pkl."""

    run_from_pkl_n_iterations: Union[int, None] = None

    display_perception: bool = False
    """Whether to display intermediate steps in the perception process."""

    display_obm_state: bool = False
    "Whether to show the OBM belief state after update in separate window."

    terminal_tags: tuple = ()
    """A tuple of tags to use for terminal messages. Default is empty tuple."""

    pause_tags: tuple = ()
    """A tuple of tags to use for pausing the execution. Default is empty tuple."""

    display_large_objects: bool = False
    """Whether to display large objects in the perception display."""
    display_small_objects: bool = False
    """Whether to display small objects in the perception display."""

    display_raw_images: bool = False
    """Whether to display raw RGB and depth images in the perception display."""

    max_depth_threshold: Union[float, None] = None
    """Maximum depth threshold for depth filtering. If None, no max threshold is applied."""
    min_depth_threshold: Union[float, None] = None
    """Minimum depth threshold for depth filtering. If None, no min threshold is applied."""
    nan_removal_radius: int = 2
    """Radius for removing NaN values (pixels). 0 disables removal."""

    invalid_image_region_mask: Union[np.array, None] = None
