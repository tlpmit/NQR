import time

import matplotlib.pyplot as plt
import numpy as np
import trimesh

from qr_api.perc_interfaces import LargeObjectDetectionFunction
from qr_api.perc_typing import (
    CalibratedRGBDObservation,
    ObjectDetection,
    ObjectDetectionList,
    SceneRepresentation,
    UncertainObjectMesh,
)
from qr_utils.traceFile import tr, tr_a

# Debug tag: 'planes'


class SurfaceFinder(LargeObjectDetectionFunction):
    def __init__(
        self,
        min_ratio=0.05,
        voxel_size=0.01,
        iterations=1000,
        min_surface_dim=0.25,
        min_z_for_planes=0.025,
        horizontal_tolerance_deg=15,
        display: bool = False,
    ):
        self.min_ratio = min_ratio
        self.voxel_size = voxel_size
        self.iterations = iterations
        self.min_surface_dim = min_surface_dim
        self.min_z_for_planes = min_z_for_planes
        self.horizontal_tolerance_deg = horizontal_tolerance_deg
        self.display = display

    def forward(
        self, scene: SceneRepresentation, image: CalibratedRGBDObservation
    ) -> ObjectDetectionList:
        from qr_utils.pcd_utilities import dbscan_clusters_from_dense_pcd

        plane_list = horizontal_planes(
            image,
            self.iterations,
            self.voxel_size,
            0.01,
            self.horizontal_tolerance_deg,
            display=self.display,
        )

        detections = []
        for plane_eq, pcd_points, im_indices in plane_list:
            tr("planes", "plane_eq", plane_eq.tolist())
            normal = plane_eq[:3]
            if (
                normal[2] <= np.cos(np.deg2rad(self.horizontal_tolerance_deg))
                or -plane_eq[3] < self.min_z_for_planes
                or np.all(plane_eq == 0.0)
            ):
                # Keep only horizontal planes, above the floor
                continue
            # There could be multiple clusters in the plane
            pcd = numpy_to_PCD(pcd_points)

            # group_indices = np.asarray(pcd.cluster_dbscan(eps=2*self.threshold, min_points=100))
            # groups = np.unique(group_indices)
            # for group_id in groups:
            #     if len(groups) > 1 and group_id == -1:
            #         n_ignore = np.where(group_indices == group_id)[0].shape[0]
            #         tr('planes', f'Ignoring {n_ignore} points from plane')
            #         continue
            #     group_id_indices = np.where(group_indices == group_id)[0]

            groups, labels = dbscan_clusters_from_dense_pcd(
                pcd, self.voxel_size, 10 * self.voxel_size, 100
            )
            for group_id_indices in groups:
                group_points = pcd_points[group_id_indices]
                group_points[:, 2] = -plane_eq[3]

                surface_bbox = bbox(group_points)
                surface_dims = surface_bbox[1, :] - surface_bbox[0, :]
                if max(surface_dims[0], surface_dims[1]) < self.min_surface_dim:
                    tr(
                        "planes",
                        f"Ignoring segment at height {plane_eq[3]} with dims {surface_dims[:2]}",
                    )
                    continue

                group_im_indices = im_indices[group_id_indices]
                group_points_extruded = np.vstack(
                    [group_points, group_points - 0.025 * normal]
                )
                mesh = trimesh.convex.convex_hull(group_points_extruded)
                umesh = UncertainObjectMesh(
                    mesh.vertices, mesh.faces, np.ones(mesh.vertices.shape[0])
                )
                seg_mask = image_mask(image, group_im_indices)

                if umesh is None:
                    pass

                detection = ObjectDetection(
                    scene,
                    ((image, seg_mask),),
                    _mesh=umesh,
                    features={"category": "surface", "plane_eq": plane_eq},
                )
                detections.append(detection)

        return detections


def horizontal_planes_ransac(
    image: CalibratedRGBDObservation,
    iterations: int,
    threshold: float,
    min_ratio: float,
    display: bool = False,
):
    points = image.point_cloud
    N = len(points)
    image_indices = np.arange(N)

    plane_list = []
    count = 0
    while count < (1 - min_ratio) * N:
        w, index = plane_regression(
            target, threshold=threshold, init_n=3, iter=iterations
        )

        if np.count_nonzero(index) == 0:
            break

        count += len(index)
        plane_list.append((w, target[index], image_indices[index]))

        if display:
            from qr_utils.pcd_utilities import visualize_pointclouds_colors

            tr_a("plane_eq", w)
            image_mask(image, N, image_indices[index])
            visualize_pointclouds_colors(
                [target, target[index]], [np.array([0, 255, 0]), np.array([255, 0, 0])]
            )

        target = np.delete(target, index, axis=0)
        image_indices = np.delete(image_indices, index)
    return plane_list


def horizontal_planes(
    image: CalibratedRGBDObservation,
    iterations: int,
    threshold: float,
    delta: float,
    horizontal_tolerance_deg: float,
    display: bool = False,
):
    import open3d as o3d

    points = image.point_cloud
    N = len(points)
    image_indices = np.arange(N)

    dense = False
    voxel_size = 0.01
    thr = np.cos(np.deg2rad(horizontal_tolerance_deg))
    start = time.time()
    PCD = numpy_to_PCD(points)
    if dense:
        PCD.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.025, max_nn=30)
        )
        PCD.orient_normals_towards_camera_location(image.camera_pose[:3, 3])
        normals = np.asarray(PCD.normals)
        up = np.where(normals[:, 2] > thr)[0]
    else:
        (downPCD, _, trace) = PCD.voxel_down_sample_and_trace(
            voxel_size,
            PCD.get_min_bound() + voxel_size / 2,
            PCD.get_max_bound() - voxel_size / 2,
        )
        downPCD.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=4 * voxel_size, max_nn=30
            )
        )
        downPCD.orient_normals_towards_camera_location(image.camera_pose[:3, 3])
        down_normals = np.asarray(downPCD.normals)
        down_up = np.where(down_normals[:, 2] > thr)[0]
        if len(down_up) == 0:
            tr("log", "No horizontal points found")
            return []
        up = np.hstack([trace[i] for i in down_up])
        pass
    tr("planes", "Plane fitting time =", time.time() - start)
    up_points = points[up]
    up_im_indices = image_indices[up]
    heights = points[up, 2]
    heights.sort()
    clusters = cluster_values(heights, delta)
    plane_list = []

    for height in clusters:
        # Find points near the plane
        h_index = np.where(
            (up_points[:, 2] < height + delta) & (up_points[:, 2] > height - delta)
        )[0]
        h_points = up_points[h_index]
        h_im_indices = up_im_indices[h_index]
        plane_eq, index = np.array([0.0, 0.0, 1.0, -height]), slice(None, None)
        # plane_eq, index = plane_regression(
        #     h_points, threshold=threshold, init_n=3, iter=iterations)
        if np.all(plane_eq == 0.0):
            continue
        plane_list.append((plane_eq, h_points[index], h_im_indices[index]))

        if display:
            from qr_utils.pcd_utilities import visualize_pointclouds_colors

            tr_a("plane_eq", plane_eq)
            image_mask(image, h_im_indices[index])
            visualize_pointclouds_colors(
                [points, h_points[index]],
                [np.array([0, 255, 0]), np.array([255, 0, 0])],
            )
    return plane_list


# Make sure that the clusters (avg +- delta) don't overlap.
def cluster_values(heights, delta):
    def add_cluster(avg, n):
        # Test for overlap
        if clusters and avg - delta < clusters[-1][0] + delta:
            c1, n1 = clusters[-1]
            c2, n2 = avg, n
            clusters[-1] = ((c1 * n1 + c2 * n2) / (n1 + n2), n1 + n2)
        else:
            clusters.append([avg, n])

    clusters = []
    sum = heights[0]
    n = 1
    for i in range(1, len(heights)):
        avg = sum / n
        if abs(heights[i] - avg) <= delta:
            sum += heights[i]
            n += 1
        else:
            add_cluster(avg, n)
            sum = heights[i]
            n = 1
    if n > 1:
        add_cluster(avg, n)
    tr("planes", "height clusters", clusters)
    return [avg for (avg, n) in clusters if n > 1000]


def image_mask(image, indices, debug=False):
    N = len(image.point_cloud)
    mask = np.zeros(N, bool)
    mask[indices] = True
    seg_mask = mask.reshape(image.rgb_image.shape[:-1])
    if debug:
        # Show the masked rgb_image
        mask_3d = np.stack([seg_mask] * 3, axis=-1)
        masked_rgb_image = image.rgb_image * mask_3d.astype(np.uint8)
        plt.imshow(masked_rgb_image)
        plt.show()

    return mask


def plane_regression(points, threshold=0.01, init_n=3, iter=1000):
    """plane regression using ransac

    Args:
        points (ndarray): N x3 point clouds
        threshold (float, optional): distance threshold. Defaults to 0.003.
        init_n (int, optional): Number of initial points to be considered inliers in each iteration
        iter (int, optional): number of iteration. Defaults to 1000.

    Returns:
        [ndarray, List]: 4 x 1 plane equation weights, List of plane point index
    """

    pcd = numpy_to_PCD(points)

    w, index = pcd.segment_plane(threshold, init_n, iter)

    return w, index


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


def bbox(points):
    """Bounding box of point set

    Args:
        points (ndarray): N x3 point clouds

    Returns:
        [ndarray]: 2 x 3
    """
    a = np.zeros((2, 3))
    a[0, :] = np.min(points, axis=0)
    a[1, :] = np.max(points, axis=0)
    return a
