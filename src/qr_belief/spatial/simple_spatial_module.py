"""
Spatial memory: ternary voxel occupancy (occupied / free / unknown-"shadow")
maintained from depth images via the qr_mapping C++ voxel grid.

Port of the old repo's OldHPNSpatialMemoryModule with three substitutions:
- VV_Regions (HPN's region abstraction over the voxel grid, used by the HPN
  policy) is not ported yet; VV.vv_regions is None until the policy port.
- bboxGrow (Roboverse) is inlined.
- update_costmap (Roboverse raster costmap, config-gated off in the old repo)
  is a no-op stub.
The `phys`-based shadow-carving methods take any world object exposing the
same interface (bodies with planes()/bbox(); the kinematic world can provide
this later) — they are only exercised by the policy.
"""

import copy

import numpy as np
from scipy.ndimage import binary_erosion
from termcolor import cprint

from qr_api.belief_interfaces import SpatialMemoryModule
from qr_api.perc_typing import CalibratedRGBDObservation
from qr_api.policy_typing import Action, ActionRV, TransitionPrediction
from qr_api.sensor_typing import Observation
from qr_mapping.cpp import VoxelGridConfig, VoxelGridEstimator
from qr_utils.cog_utils import get_problem_property


def bboxGrow(bbox, delta):
    """Grow an axis-aligned bbox ((min, max) rows) by delta on all sides.
    (Inlined from Roboverse.utils.bbox.)"""
    bbox = np.asarray(bbox, dtype=float)
    return np.vstack([bbox[0] - delta, bbox[1] + delta])


class VV:  # ViewVolume
    ID = 0

    def __init__(self, voxel_grid_config, voxel_grid, voxel_grid_aux0, voxel_grid_aux1):
        # We will assume that voxel grids are axis aligned
        self.voxel_grid: VoxelGridEstimator = voxel_grid
        self.voxel_grid_aux0: VoxelGridEstimator = voxel_grid_aux0
        self.voxel_grid_aux1: VoxelGridEstimator = voxel_grid_aux1
        self.voxel_grid_config: VoxelGridConfig = voxel_grid_config
        self.voxel_grid_res = voxel_grid_config.grid_resolution
        self.voxel_centers = self.voxel_grid.GetVoxelCenters()
        self.voxel_grid_dims = self.voxel_centers.shape[:3]
        self.voxel_occupancy = None  # filled during update
        self.voxel_shadows_bin = None  # filled from voxel occupancy
        self.voxel_occupied_bin = None
        self.voxel_junk_bin = None
        self.voxel_junk_ignore_bin = None
        VV.ID += 1
        self.vv_id = VV.ID
        # This should probably live in the HPN state somewhere
        from Domains.btamp.visible_voxel import VV_Regions

        self.vv_regions = VV_Regions(self)

    def pickle_data(self):
        cp = copy.copy(self)
        cp.voxel_grid = None
        cp.voxel_grid_aux0 = None
        cp.voxel_grid_aux1 = None
        cp.voxel_grid_config = None
        cp.vv_regions = copy.copy(self.vv_regions)
        cp.vv_regions.fixed_shadow_regions_by_name = {}
        cp.vv_regions.bbox_regions_by_name = {}
        cp.vv_regions.vv = None
        return cp

    def update_voxel_shadows(self):
        self.voxel_grid_centers = self.voxel_grid.GetVoxelCenters()
        self.voxel_occupancy_aux0 = self.voxel_grid_aux0.GetOccupancy()
        self.voxel_occupancy_aux1 = self.voxel_grid_aux1.GetOccupancy()
        self.voxel_shadows_bin = self.voxel_occupancy_aux0 == 255
        # Free cells in the aux occupancy are not shadowed
        aux_empty = self.voxel_occupancy_aux1 == 0
        aux_nonzero_count = np.count_nonzero(aux_empty)
        if aux_nonzero_count:
            self.voxel_shadows_bin[aux_empty] = False
        # self.voxel_shadows_bin = self.erode_xy(self.voxel_shadows_bin, radius=1)
        self.voxel_occupied_bin = self.voxel_occupancy_aux0 == 1
        self.voxel_junk_bin = self.voxel_occupancy_aux0 == 1

        VV.ID += 1
        self.vv_id = VV.ID

    @staticmethod
    def erode_xy(binary_xyz, radius=1, z_axis=2):
        """
        Morphologically erode a 3D binary array in x,y only (no erosion along z).

        Parameters
        ----------
        binary_xyz : ndarray (bool or 0/1)
            3D binary array, e.g. shape (nx, ny, nz) or (nz, ny, nx), etc.
        radius : int
            Radius (in pixels) of erosion in x,y.
        z_axis : int
            Index of the z axis in binary_xyz (0, 1, or 2).

        Returns
        -------
        eroded : ndarray (bool)
            Eroded binary volume, same shape as input.
        """
        vol = np.asarray(binary_xyz).astype(bool)

        # Move z axis to axis 0 so volume has shape (Z, Y, X)
        vol = np.moveaxis(vol, z_axis, 0)

        # Build a 2D disk structuring element in XY
        yy, xx = np.ogrid[-radius : radius + 1, -radius : radius + 1]
        disk = (xx * xx + yy * yy) <= radius * radius  # shape (2r+1, 2r+1)

        # Make it 3D with thickness 1 in Z
        selem = disk[np.newaxis, :, :]  # shape (1, 2r+1, 2r+1)

        # Erode: acts only within each XY slice
        eroded = binary_erosion(vol, structure=selem)

        # Move axes back so shape matches input
        eroded = np.moveaxis(eroded, 0, z_axis)
        return eroded

    def shadow_shell_bin(vv, phys, mult):  # vv is self
        if vv.voxel_shadows_bin is None:
            # Not initialized yet
            return
        all_voxels = vv.voxel_centers.reshape(-1, 3)
        N = len(all_voxels)
        all_inside_bin = np.zeros(N, dtype=bool)

        for name in phys._bodies:
            if name.startswith("wall"):
                continue
            body = phys._bodies[name]
            trans = phys._body_trans.get(name)
            if trans is None:
                attached = phys.get_attached()
                for h in attached:
                    if attached[h]:  # can be None
                        (b, t, _) = attached[h]
                        if b.name == name:
                            conf = phys.get_conf()
                            trans = conf.get_end_trans(h).compose(t)
                            break
            assert trans is not None
            res = vv.voxel_grid_res
            bbox = bboxGrow(body.bbox(trans), mult * res)
            pcd_ind = np.where(
                np.all((all_voxels <= bbox[1]) & (all_voxels >= bbox[0]), axis=1)
            )[0]
            if pcd_ind.shape[0] == 0:
                continue
            planes = body.planes(trans).copy()
            pcd = all_voxels[pcd_ind]
            points = np.hstack([pcd, np.ones((pcd.shape[0], 1))])
            # Note we are growing the object here by using res instead of eps
            displace_uniformly = False
            if displace_uniformly:
                cpoints_bin = np.all(np.dot(planes, points.T) < mult * res, axis=0)
            else:
                # Don't displace the bottom planes
                for i in range(len(planes)):
                    if -planes[i, 2] < 0.95:
                        planes[i, 3] -= mult * res
                    else:
                        planes[i, 3] -= 0.5 * mult * res
                cpoints_bin = np.all(np.dot(planes, points.T) <= 0.0, axis=0)
            pcd_cpoints_ind = pcd_ind[cpoints_bin]
            inside_bin = np.isin(np.arange(N), pcd_cpoints_ind)
            all_inside_bin = np.logical_or(inside_bin, all_inside_bin)

        all_inside_bin_vox = all_inside_bin.reshape(vv.voxel_shadows_bin.shape)
        return all_inside_bin_vox

    def update_shadows_and_junk(vv, phys, mult=2):
        all_inside_bin_vox = vv.shadow_shell_bin(phys, mult=mult)
        if all_inside_bin_vox is None:
            return
        # Remove the inside of objects from the shadow BIN
        vv.voxel_shadows_bin[all_inside_bin_vox] = False
        # Remove the inside of objects from the junk BIN
        vv.voxel_junk_bin[all_inside_bin_vox] = False
        if vv.voxel_junk_ignore_bin is None:
            vv.voxel_junk_ignore_bin = np.zeros(vv.voxel_junk_bin.shape, dtype=bool)
        vv.voxel_junk_ignore_bin[all_inside_bin_vox] = True
        vv.voxel_junk_bin = vv.voxel_junk_bin & (~vv.voxel_junk_ignore_bin)
        vv.voxel_junk_bin[np.where(vv.voxel_grid_centers[:, :, :, 2] < 0.1)] = False
        # The inside of objects is occupied
        vv.voxel_occupied_bin[all_inside_bin_vox] = True
        vv.voxel_occupied_bin = vv.voxel_occupied_bin & (~vv.voxel_junk_ignore_bin)
        update_costmap(vv, phys)

    def grow_shadows(vv, phys, mult=4):
        all_inside_bin_vox = vv.shadow_shell_bin(phys, mult=mult)
        if all_inside_bin_vox is None:
            return
        vv.update_voxel_shadows()
        vv.update_shadows_and_junk(phys, mult=2)


class OldHPNSpatialMemoryModule(SpatialMemoryModule):
    def __init__(
        self,
        static_domain_info=None,
        drake_meshcat=None,
        drake_meshcat_aux=None,
        voxel_grid_resolution=0.02,
        workspace=None,
        shadow_extents=None,
        shadow_pose=None,
    ):
        """
        The grid bounds come either from a static problem description
        (static_domain_info, as in the old repo) or directly from the
        workspace/shadow_extents/shadow_pose arrays.
        """
        super().__init__()
        if static_domain_info is not None:
            shadow_extents = get_problem_property(static_domain_info, "shadow-extents")
            shadow_pose = get_problem_property(static_domain_info, "shadow-pose")
            workspace = get_problem_property(static_domain_info, "workspace")
        shadow_extents = np.asarray(shadow_extents, dtype=float)
        shadow_pose = np.asarray(shadow_pose, dtype=float)
        workspace = np.asarray(workspace, dtype=float)
        assert shadow_pose[2:].sum() == 0.0, "Only the xy shadow pose can be set"
        # Make sure the shadow range in inside the workspace
        shadow_pose[:3] = np.clip(shadow_pose[:3], workspace[0], workspace[1])
        shadow_posed_extent = np.clip(
            shadow_pose[:3] + shadow_extents, workspace[0], workspace[1]
        )
        shadow_extents = shadow_posed_extent - shadow_pose[:3]

        if drake_meshcat is None:
            from pydrake.all import Meshcat

            drake_meshcat = Meshcat()
        if drake_meshcat_aux is None:
            from pydrake.all import Meshcat

            drake_meshcat_aux = (Meshcat(), Meshcat())

        self._vv = None
        self.initialize_vv_map(
            shadow_extents,
            shadow_pose,
            drake_meshcat,
            drake_meshcat_aux,
            voxel_grid_resolution,
        )

    @property
    def vv(self):
        return self._vv

    def _reset(self, observation: Observation):
        pass

    def _update(
        self,
        action: Action,
        actionrv: ActionRV,
        transition_prediction: TransitionPrediction,
        observation: Observation,
    ):
        images = observation["images"]
        if images:
            # Incorporate the images into VoxelGrid
            for image in images:
                self.UpdateVoxelGrid(image)
        self.vv.update_voxel_shadows()

    def initialize_vv_map(
        self,
        shadow_extents,
        shadow_pose,
        drake_meshcat,
        drake_meshcat_aux,
        voxel_grid_resolution,
    ):
        from pydrake.all import Rgba

        # Create voxel grid config
        voxel_grid_config = VoxelGridConfig()
        voxel_grid_config.x_size = shadow_extents[0]
        voxel_grid_config.y_size = shadow_extents[1]
        voxel_grid_config.z_size = shadow_extents[2]
        voxel_grid_config.x_origin = shadow_pose[0]
        voxel_grid_config.y_origin = shadow_pose[1]
        voxel_grid_config.z_origin = 0.0
        voxel_grid_config.filled_color = Rgba(1.0, 0.0, 0.0, 1.0)
        voxel_grid_config.unknown_color = Rgba(0.0, 0.0, 0.0, 0.5)
        voxel_grid_config.grid_resolution = voxel_grid_resolution
        voxel_grid_config.max_threads = 1
        # Create voxel grid
        voxel_grid = VoxelGridEstimator(voxel_grid_config, drake_meshcat)
        voxel_grid_aux0 = VoxelGridEstimator(voxel_grid_config, drake_meshcat_aux[0])
        voxel_grid_aux1 = VoxelGridEstimator(voxel_grid_config, drake_meshcat_aux[1])
        drake_meshcat.SetProperty("/voxel_grid/pointcloud", "visible", True)
        drake_meshcat_aux[0].SetProperty("/voxel_grid/pointcloud", "visible", True)
        drake_meshcat_aux[1].SetProperty("/voxel_grid/pointcloud", "visible", True)

        self._vv = VV(voxel_grid_config, voxel_grid, voxel_grid_aux0, voxel_grid_aux1)

    def UpdateVoxelGrid(
        self,
        obs: CalibratedRGBDObservation,
        print_stats: bool = False,
        **kwargs,
    ):
        max_depth = 3.0

        from pydrake.all import ImageDepth32F, ImageRgb8U

        def PrintNumpyStats(arr: np.ndarray, prefix=""):
            cprint(
                f"{prefix}: ["
                f"Occupied: {(arr == 1).mean() * 100.0:.1f}%, "
                f"Free: {(arr == 0).mean() * 100.0:.1f}%, "
                f"Unknown: {(arr == 255).mean() * 100.0:.1f}%]",
                color="yellow",
                attrs=["bold"],
            )

        def toImageDepth32F(obs, no_max=False, only_max=False):
            depth_image = obs.depth_image
            height, width = depth_image.shape
            if depth_image.max() >= max_depth and (no_max or only_max):
                depth_image = depth_image.copy()
                if no_max:
                    ext = np.where(depth_image >= max_depth)
                    depth_image[ext[0], ext[1]] = np.nan
                elif only_max:
                    ext = np.where(depth_image < max_depth)
                    depth_image[ext[0], ext[1]] = np.nan
            image = ImageDepth32F(width=width, height=height)
            src_image = depth_image[..., np.newaxis]  # (H, W, 1)
            np.copyto(
                image.mutable_data, np.clip(src_image, a_min=0.0, a_max=max_depth)
            )
            return image

        def toImageRgb8U(obs):
            height, width, _ = obs.rgb_image.shape
            image = ImageRgb8U(width=width, height=height)
            np.copyto(image.mutable_data, obs.rgb_image)
            return image

        arr = self.vv.voxel_grid.GetOccupancy()
        if print_stats:
            PrintNumpyStats(arr, prefix="Before")

        # Add default params if not provided by user
        kwargs.setdefault("max_depth_points_to_publish", 20000)
        kwargs.setdefault("viz_point_size", 0.02)

        # Everything in here
        self.vv.voxel_grid.UpdateFromDepthImage(
            toImageDepth32F(obs),
            obs.camera_intrinsics,
            obs.camera_extrinsics,
            toImageRgb8U(obs),
            **kwargs,
        )

        # The filled and empty voxels here should be accurate, but some
        # empty cells are missing because the max_depth readings are left out.
        self.vv.voxel_grid_aux0.UpdateFromDepthImage(
            toImageDepth32F(obs, no_max=True),
            obs.camera_intrinsics,
            obs.camera_extrinsics,
            toImageRgb8U(obs),
            **kwargs,
        )
        # The empty cells in this array are the missing ones.
        if obs.depth_image.max() >= max_depth:
            self.vv.voxel_grid_aux1.UpdateFromDepthImage(
                toImageDepth32F(obs, only_max=True),
                obs.camera_intrinsics,
                obs.camera_extrinsics,
                toImageRgb8U(obs),
                **kwargs,
            )

        if print_stats:
            arr = self.vv.voxel_grid.GetOccupancy()
            PrintNumpyStats(arr, prefix=" After")


def update_costmap(vv, phys):
    import Domains.tamp.configuration as tamp_config
    from Roboverse.planning.raster_map import Costmap
    from Roboverse.skrobot.shared import Trans

    if not tamp_config.use_costmap:
        return
    if phys.costmap is None:
        phys.costmap = Costmap()
        robot_body = phys.robot.make_robot_body(phys.robot.get_default_skrobot_conf())
        robot_body_bbox = robot_body.bbox(Trans((0, 0, 0, 0)))
        robot_dims = [robot_body_bbox[1][i] - robot_body_bbox[0][i] for i in range(3)]
        sensor = phys.robot.get_sensors()[0]
        (sensor_fov, _, _, _, sensor_max_d) = sensor.params
        # Limit the simulated sensor cone, so we don't rely on marginal views
        sensor_LWH = (
            sensor_max_d - 1.0,
            sensor_max_d * np.tan(np.radians(sensor_fov / 2)) * 0.5,
            sensor_max_d * np.tan(np.radians(sensor_fov / 2)) * 0.5,
        )
        sensor_trans = phys.robot.get_sensor_matrix(
            sensor.link, phys.robot.get_default_skrobot_conf()
        )
        sensor_height = sensor_trans[2, 3]
        # For Spot
        if phys.robot.name == "Spot":
            sensor_origin = (robot_dims[0] / 2, 0, sensor_height)
        elif phys.robot.name == "Rainbow":
            sensor_origin = (0, 0, sensor_height)
        else:
            raise NotImplementedError

        phys.costmap.initialize(
            phys.robot.workspace,
            (vv.voxel_grid_config.x_origin, vv.voxel_grid_config.y_origin),
            vv.voxel_shadows_bin.shape,
            vv.voxel_grid_res,
            robot_dims,
            sensor_LWH,
            sensor_origin,
            theta_list=tamp_config.THETA_LIST_DEG,
            pan_list=tamp_config.PAN_LIST_DEG,
            tilt_list=tamp_config.TILT_LIST_DEG,
        )
    # HACK - keep track of the most recent update, so we can
    # update the costmap on demand.
    phys.costmap.latest_vv = vv
