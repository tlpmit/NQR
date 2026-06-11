import matplotlib.pyplot as plt
import numpy as np
import trimesh

from qr_api.perc_interfaces import UncertainShapeCompletionFunction
from qr_api.perc_typing import (
    CalibratedRGBDObservation,
    ObjectDetection,
    UncertainObjectMesh,
)
from qr_utils.pcd_utilities import point_cloud_from_depth_image_camera_frame
from qr_utils.traceFile import tr


class ProjectionCompletion(UncertainShapeCompletionFunction):
    def __init__(
        self,
        use_eye_extrusion: bool = False,
        eye_extrusion_distance: float = 0.025,
        depth_method="raw",
        display: bool = False,
    ):
        super().__init__()
        self.use_eye_extrusion = use_eye_extrusion
        self.eye_extrusion_distance = eye_extrusion_distance
        if depth_method == "DAV2":
            from qr_perception.modules.DAV2_depth import DAV2DepthEstimator

            self.DAV2 = DAV2DepthEstimator(n_erosion_iterations=6, display=display)
        else:
            self.DAV2 = None
        self.display = display

    def forward(
        self, detection: ObjectDetection, observation: CalibratedRGBDObservation
    ) -> UncertainObjectMesh | None:
        """Complete the shape of the object."""
        if len(detection.point_cloud) == 0:
            return None

        # Get the args
        rgb_image = observation.rgb_image
        intrinsics = observation.camera_intrinsics
        extrinsics = observation.camera_extrinsics
        extrinsics_inv = np.linalg.inv(extrinsics)
        seg_mask = detection.segmentation_masks[0][1]
        if self.display:
            masked_rgb_image = np.ones_like(rgb_image) * 255
            masked_rgb_image[seg_mask] = rgb_image[seg_mask]
            plt.imshow(masked_rgb_image)
            plt.show()

        if self.DAV2:
            scaled_computed_depth_image = self.DAV2.forward(
                detection.segmentation_masks[0][1], observation
            )
        else:
            scaled_computed_depth_image = None

        if scaled_computed_depth_image is not None:
            computed_pcd = point_cloud_from_depth_image_camera_frame(
                scaled_computed_depth_image, intrinsics
            )
            rotation, translation = extrinsics_inv[:3, :3], extrinsics_inv[:3, 3]
            world_computed_pcd = rotation.dot(computed_pcd.T).T + translation
            vertices = world_computed_pcd[seg_mask.reshape(-1)]
            if len(vertices) == 0:
                return None
            min_z, max_z = (
                np.percentile(vertices[:, 2], 1),
                np.percentile(vertices[:, 2], 99),
            )
        else:
            world_computed_pcd = None
            vertices = detection.point_cloud
            clean_points = vertices[~np.isnan(vertices).any(axis=1)]
            if len(clean_points) <= 10:
                return None
            vertices = clean_points
            # These filtering methods remove too much (esp with realsense), but without them we end up
            # with bad outliers
            # TODO: improve
            # name = input('Name to save to: ')
            # if name:
            #     np.save(f"{name}.npy", vertices)
            vertices = filter_streamers(vertices)
            vertices = filter_dbscan(vertices)
            if len(vertices) == 0:
                return None
            min_z, max_z = (
                np.percentile(vertices[:, 2], 1),
                np.percentile(vertices[:, 2], 99),
            )

        # Slightly fatten thin sheets
        if max_z - min_z < 0.01:
            min_z = max_z - 0.01
            #    return None

        if self.use_eye_extrusion:
            eye_point = extrinsics_inv[:3, 3]
            eye_extrusion = vertices.copy()
            for i in range(len(vertices)):
                v = vertices[i] - eye_point
                dv = v / np.linalg.norm(v)
                nv = vertices[i] + self.eye_extrusion_distance * dv
                if nv[2] < min_z:
                    continue
                eye_extrusion[i] = nv
            vertices = np.vstack([eye_extrusion, vertices])

        downward_extrusion = vertices.copy()
        downward_extrusion[:, 2] = min_z
        upward_extrusion = vertices.copy()
        upward_extrusion[:, 2] = max_z
        all_extrusion = np.vstack([downward_extrusion, upward_extrusion])
        mesh = trimesh.convex.convex_hull(all_extrusion)
        extents = mesh.extents
        if extents.min() < 0.01:
            return None
        tr('log', 'mesh extents', extents)
        if mesh is None:
            tr(("completion", "log"), "Discarding detection; it is too thin")
            return None

        completed_vertices = mesh.vertices
        completed_faces = mesh.faces
        certainty = compute_certainty(vertices, completed_vertices)

        if self.display:
            from qr_utils.pcd_utilities import visualize_pointclouds_colors

            if world_computed_pcd is not None:
                visualize_pointclouds_colors(
                    [detection.point_cloud, all_extrusion, vertices],
                    [
                        np.array([255, 0, 0]),
                        np.array([0, 255, 0]),
                        np.array([0, 0, 255]),
                    ],
                )
            else:
                visualize_pointclouds_colors(
                    [detection.point_cloud, all_extrusion],
                    [np.array([255, 0, 0]), np.array([0, 255, 0])],
                )

        detection.mesh = UncertainObjectMesh(
            completed_vertices, completed_faces, certainty
        )


def compute_certainty(original, completed, eps=0.01):
    import scipy.spatial as sp

    tA = sp.KDTree(original)
    tB = sp.KDTree(completed)
    neighbors = tB.query_ball_tree(tA, r=eps)
    # indices of points in B (completed) with no near neighbors in A (original)
    indices = [i for i, n in enumerate(neighbors) if not n]
    certainty = np.ones(len(completed))
    certainty[indices] = 0.0
    return certainty


def filter_streamers(points, resolution=0.01, display=False):
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    # Define the grid edges based on min/max and resolution
    x_min, x_max = np.min(x), np.max(x)
    y_min, y_max = np.min(y), np.max(y)
    z_min, z_max = np.percentile(z, 0), np.percentile(z, 99)

    x_edges = np.arange(x_min, x_max + resolution, resolution)
    y_edges = np.arange(y_min, y_max + resolution, resolution)

    # Create a 2D histogram (grid of counts)
    grid, _, _ = np.histogram2d(x, y, bins=[x_edges, y_edges])

    counts = grid.flatten()
    counts = counts[counts > 0]

    q3 = np.percentile(counts, 75)

    delete = []
    rows, cols = np.where((grid > 0) & (grid < q3))
    for i in range(len(rows)):
        x = x_edges[rows[i]]
        y = y_edges[cols[i]]
        ind = np.where(
            (points[:, 0] >= x)
            & (points[:, 0] < x + resolution)
            & (points[:, 1] >= y)
            & (points[:, 1] < y + resolution)
        )
        zq2 = np.percentile(points[ind[0], 2], 50)
        if zq2 < 0.5 * (z_min + z_max):
            delete.extend(ind[0].tolist())

    result = np.delete(points, delete, axis=0)
    result_grid, _, _ = np.histogram2d(
        result[:, 0], result[:, 1], bins=[x_edges, y_edges]
    )

    if display:
        print(grid)
        print(result_grid)

    return result


def filter_dbscan(points, eps=0.02, min_points=100, display=False):
    pcd = numpy_to_PCD(points)
    group_indices = np.asarray(pcd.cluster_dbscan(eps=eps, min_points=min_points))
    groups = np.unique(group_indices)
    group_counts = {}
    for group_id in groups:
        group_counts[group_id] = len(np.where(group_indices == group_id)[0])
    max_group_count = max(group_counts.values())
    max_group = next(
        group_id for group_id, count in group_counts.items() if count == max_group_count
    )
    delete = np.array([], dtype=int)
    for group_id in groups:
        if group_id != max_group:
            d = np.where(group_indices == group_id)[0]
            delete = np.hstack([delete, d])
            n_ignore = len(delete)
            tr(("completion", "log"), f"Ignoring {n_ignore} points from point cloud")
    points = np.delete(points, delete, axis=0)
    return points


def numpy_to_PCD(xyz):
    """convert numpy ndarray to open3D point cloud

    Args:
        xyz (ndarray):

    Returns:
        [open3d.geometry.PointCloud]:
    """

    import open3d as o3d

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    return pcd
