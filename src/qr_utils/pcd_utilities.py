from typing import List, Optional, Tuple, Union

import matplotlib.pyplot as plt
import numpy as np
import shapely
import trimesh
from trimesh.transformations import rotation_matrix

from qr_utils.clustering import downsample_cluster, geometric_cluster_filter

# Find oriented bounding box that tries to minimize distance of points to sides.
# alternative strategy:
#     pc_tri.vertices.bounding_box_oriented

# from sys import platform
# if platform == "linux" or platform == "linux2":
PERC_VIEWER = None
DISPLAY_TRIMESH_OR_O3D = "o3d"


def obb_from_points(
    pc_tri, downsample=False, downsample_num_pt=1000, surface_eqn=None, verbose=False
):
    """
    downsample: If set True, downsample the point cloud during pre-processing for faster inference
    """
    points = pc_tri.vertices
    if downsample and len(points) > downsample_num_pt:
        points = downsample_cluster(points, downsample_num_pt)
    npoints = len(points)
    ones_arr = np.ones((npoints, 1))
    points_hom = np.concatenate((points, ones_arr), axis=1)
    ch = trimesh.convex.convex_hull(points)
    if surface_eqn is not None:
        # Support surface for scene
        neg_surface_eqn = -surface_eqn
        neg_dist = neg_surface_eqn[3]
        # Find the support surface for this cluster
        face_normals = neg_surface_eqn[:3].reshape(1, 3)
        point_dists = face_normals @ points.T
        neg_dist = -np.max(point_dists)
        centroid = points.mean(axis=0)
        dist = np.dot(face_normals[0], centroid) + neg_dist
        face_origins = np.array([centroid - dist * face_normals[0]])
        # Quick and dirty approx assuming normal is along z
        # centroid[2] = -surface_eqn[3]
        face_sorted = [0]
    else:
        # This is still needed for the table body
        face_normals = ch.face_normals
        face_origins = ch.triangles_center
        face_dists = np.sum(face_normals * face_origins, axis=1)
        face_planes = np.hstack([face_normals, -face_dists.reshape(-1, 1)])
        face_point_dists = face_planes @ points_hom.T
        face_avg_dists = np.sum(face_point_dists, axis=1) / npoints
        for facet in ch.facets:
            for face in facet[1:]:
                face_avg_dists[face] = -100
        face_sorted = np.argsort(-face_avg_dists)[:10]

    # lpk terrible hack: assume horizontal rest face at min vertex
    face_normals = [np.array([0, 0, 1])]
    centroid = points.mean(axis=0)
    centroid[2] = np.min(points[:, 2])
    face_origins = [centroid]
    face_sorted = [0]

    best_score = 1e6
    best_tform = None
    best_bbox = None

    z_step_size = np.pi / 180  # was np.pi/90

    for idx in face_sorted:
        tform_plane_to_xy = trimesh.points.plane_transform(
            origin=face_origins[idx], normal=face_normals[idx]
        )
        for z_ang in np.arange(-np.pi, np.pi, z_step_size):
            z_rot = rotation_matrix(z_ang, [0, 0, 1])
            tform_rot = np.dot(tform_plane_to_xy, z_rot)
            tform_rot_pc_hom = tform_rot.dot(points_hom.T)
            tform_rot_pc = tform_rot_pc_hom[:3].T
            tform_rot_pc_tri = trimesh.PointCloud(tform_rot_pc)
            tform_rot_bbox = tform_rot_pc_tri.bounding_box
            score = box_score(tform_rot_pc, tform_rot_bbox.bounds)
            if score < best_score:
                best_score = score
                best_tform = tform_rot
                best_bbox = tform_rot_bbox
    obb = None
    if best_bbox:
        # Newer trimesh doesn't allow apply_tranform to a Box primitive directly
        # Note that the vertices of a box are NOT in the local coordinate frame.
        if True:
            print(f"Best bbox bounds {best_bbox.bounds.tolist()}")
            print(f"Best bbox extents {best_bbox.extents.tolist()}")

        obb = trimesh.convex.convex_hull(best_bbox.vertices).apply_transform(
            np.linalg.inv(best_tform)
        )

        if (
            np.min(obb.vertices[:, 2]) < np.min(points[:, 2]) - 0.001
            or np.max(obb.vertices[:, 2]) > np.max(points[:, 2]) + 0.001
        ):
            print(
                f"Obb z goes beyond points: {np.min(obb.vertices[:, 2])} {np.min(points[:, 2])} {np.max(obb.vertices[:, 2])} {np.max(points[:, 2])}"
            )
            pass
    return obb


def box_score(pc, bounds):
    # Use entropy as a measure where p is normalized distance to the box bound.
    # Halfway between the bounds is the worst score.
    n = len(pc)
    H = 100 * np.ones(n)
    eps = 1e-6
    for i in range(3):
        ext = bounds[1, i] - bounds[0, i]
        # Make sure p is in bounds to compute log
        p = np.minimum(1 - eps, np.maximum(eps, np.abs(pc[:, i] - bounds[0, i]) / ext))
        h = -p * np.log(p) - (1 - p) * np.log(1 - p)
        # Keep the minimum for each point
        H = np.minimum(H, h)
    # Average distance
    return np.sum(H) / n


def support_boxes(detections, surface_object, offset=1e-3, draw=False):
    trim_surfaces = False
    first_pass_boxes = []
    first_pass_xy_polys = []
    n = len(detections)
    for i, obj in enumerate(detections):
        # print(f"Object {i}  Surface {obj.is_surface}")
        filtered_pc = obj.filtered_point_cloud
        pc = filtered_pc.vertices
        if filtered_pc is None or pc.shape[0] == 0:
            first_pass_boxes.append(None)
            first_pass_xy_polys.append(None)
            continue
        box = np.vstack([np.amin(pc, axis=0), np.amax(pc, axis=0)])
        poly = shapely.MultiPoint(pc[:, :2]).convex_hull
        # New hack: use the z value of the plane equation as the top
        # of the table box
        # if obj.is_surface:
        #     surface_z = -surface_object.surface_eqn[-1]
        #     if box[1][2] > surface_z:
        #         if True:
        #             print(f'Surface object top {box[1][2]} is above plane z {surface_z}.  Moving it down')
        #         box[1][2] = surface_z + .005
        #         if box[0][2] > box[1][2]:
        #             if True:
        #                 print(f'Surface object bottom {box[0][2]} is above plane z {surface_z}.  Moving it down')
        #             box[0][2] = surface_z - 0.01
        if draw:
            print(box)
        first_pass_boxes.append(box)
        first_pass_xy_polys.append(poly)

    boxes = []
    for i in range(n):
        if first_pass_boxes[i] is None:
            boxes.append(None)
            continue
        box_i = first_pass_boxes[i].copy()
        poly_i = first_pass_xy_polys[i]
        if trim_surfaces and detections[i].is_surface:
            # LPK : added trimming top of surface box based on other objects
            # But usually counterproductive (ha)
            min_z_above_surf = float("inf")
            for j in range(n):
                if j == i:
                    continue
                poly_j = first_pass_xy_polys[j]
                overlap_poly = poly_i.intersection(poly_j)
                overlap_frac = overlap_poly.area / min(poly_i.area, poly_j.area)
                if overlap_frac > 0.33:
                    min_z_above_surf = min(
                        min_z_above_surf, first_pass_boxes[j][0, 2] - offset
                    )
            if min_z_above_surf < box_i[1, 2] and True:
                print(f"Trimming surface from {box_i[1, 2]} to {min_z_above_surf}")
            box_i[1, 2] = min(box_i[1, 2], min_z_above_surf)
            boxes.append(box_i)
            continue
        for j in range(len(first_pass_boxes)):
            if j == i:
                continue
            box_j = first_pass_boxes[j]
            poly_j = first_pass_xy_polys[j]
            if draw:
                print("Intersects:", poly_i.intersects(poly_j))
                plt.plot(*poly_i.exterior.xy)
                plt.plot(*poly_j.exterior.xy)
                plt.show()
            overlap_poly = poly_i.intersection(poly_j)
            overlap_frac = (
                overlap_poly.area / min(poly_i.area, poly_j.area)
                if overlap_poly
                else 0.0
            )
            if overlap_frac > 0.1:
                # print(f"Object {i} overlaps object {j}")
                if box_i[0, 2] <= box_j[1, 2] and box_i[1, 2] >= box_j[1, 2]:
                    # print(f'Object {i} should be above object {j}')
                    # Bottom of box_i below top of box_j
                    # This gives preference to supporting box
                    if box_i[1, 2] <= box_j[1, 2]:
                        print(f"Object {i} included in object {j}")
                    box_i[0, 2] = box_j[1, 2] + offset
                    if draw:
                        print(f"Increased box {i} lower bound to {box_i[0, 2]}")
        boxes.append(box_i)
    return boxes


# If biggest_only is set, return only the biggest cluster, otherwise return
# the union of all clusters bigger than min_points
def cluster_filter(
    pc_detection: np.ndarray, biggest_only=False, min_points=30, display=False, **kwargs
):
    indices = np.array([], dtype=int)
    print("Cluster sizes", end=" ")
    for cluster_pointidx in geometric_cluster_filter(pc_detection, **kwargs):
        if len(cluster_pointidx) < min_points:
            continue
        print(len(cluster_pointidx), end=" ")
        if biggest_only:
            indices = cluster_pointidx
            break
        indices = np.concatenate((indices, cluster_pointidx))

    if display:
        print("Filtered point cloud size", len(indices))
        if len(indices) > 0:
            show_completion(pc_detection, pc_detection[indices])
        else:
            print("No points remaining after filtering")

    return np.take(pc_detection, indices, axis=0)


def filter_above_surface(point_cloud: np.ndarray, surface, min_z=1e-3):
    # TODO: check is contained in aabb2d
    point_cloud_rel, tform_mat4_pc2surface = get_points_relative_to_support_surface(
        point_cloud, surface
    )
    return point_cloud[point_cloud_rel[..., 2] >= min_z], tform_mat4_pc2surface


def get_points_relative_to_support_surface(point_cloud: np.ndarray, surface_eqn):
    tform_plane_to_xy = trimesh.points.plane_transform(
        origin=np.array([0, 0, -surface_eqn[3, 0]]), normal=surface_eqn[:3, 0]
    )
    point_cloud_tformed = tform_pointcloud(tform_plane_to_xy, point_cloud)
    return point_cloud_tformed, tform_plane_to_xy


def tform_pointcloud(tform_mat4, point_cloud: np.ndarray) -> np.ndarray:
    point_cloud = dim3_to_homogeneous_coord(point_cloud)
    tformed_pc = tform_mat4.dot(point_cloud.T)
    tformed_pc = (tformed_pc / tformed_pc[-1])[:3].T
    return tformed_pc


def dim3_to_homogeneous_coord(pointcloud_xyz: np.ndarray) -> np.ndarray:
    """
    Args: N x 3 Point Cloud
    Returns: N x 4 Point Cloud [XYZW]. Last dim fills with 1
    """
    if pointcloud_xyz.shape[1] == 3:
        ones_arr = np.ones((pointcloud_xyz.shape[0], 1))
        return np.concatenate((pointcloud_xyz, ones_arr), axis=1)
    else:
        return pointcloud_xyz


def get_convex_mesh_tri(pcd, rgb=None):
    try:
        hull = trimesh.convex.convex_hull(pcd)
    except Exception:
        return None
    if rgb is not None:
        (r, g, b) = rgb
        hull.visual.face_colors[:] = np.array([r, g, b, 200])
        hull.visual.vertex_colors[:] = np.array([r, g, b, 200])
    return hull


def planes_from_mesh(mesh):
    normals = mesh.facets_normal
    centers = mesh.facets_origin
    offsets = -(np.sum(normals * centers, axis=1).reshape((-1, 1)))
    planes = np.hstack([normals, offsets])
    return planes


def not_collision(ch1, ch2):
    # Conservative quick-reject before the exact cdd hull intersection.
    # (The old repo used Roboverse's GJK here; disjoint AABBs are a weaker
    # but dependency-free sufficient condition for no collision.)
    return bool(
        np.any(ch1.bounds[0] > ch2.bounds[1] + 1e-3)
        or np.any(ch2.bounds[0] > ch1.bounds[1] + 1e-3)
    )


def intersect_pcd_hulls(pcd1, pcd2):
    ch1 = get_convex_mesh_tri(pcd1)
    ch2 = get_convex_mesh_tri(pcd2)
    if not (ch1 and ch2) or not_collision(ch1, ch2):
        return None
    if ch1.facets_normal.shape[0] == 0 or ch2.facets_normal.shape[0] == 0:
        return None
    planes = np.vstack([planes_from_mesh(ch1), planes_from_mesh(ch2)])
    isect = verts_from_planes(planes)
    if isect is None:
        return None
    return get_convex_mesh_tri(isect)


def union_pcd_hull(pcd1, pcd2):
    return get_convex_mesh_tri(np.vstack([pcd1, pcd2]))


def IoU(pcd1, pcd2):
    Ih = intersect_pcd_hulls(pcd1, pcd2)
    if Ih is None:
        return 0.0
    I = Ih.volume
    U = union_pcd_hull(pcd1, pcd2).volume
    if U < 1.0e-6:
        return 0.0
    return I / U


def IoM(pcd1, pcd2):
    Ih = intersect_pcd_hulls(pcd1, pcd2)
    if not Ih:
        return 0
    I = Ih.volume
    if I < 1e-6:
        return 0.0
    ch1 = get_convex_mesh_tri(pcd1)
    ch2 = get_convex_mesh_tri(pcd2)
    M = min(ch1.volume, ch2.volume)
    if M < 1e-6:
        return 0.0
        pass
    return min(I / M, 1.0)


def inflate(pcd, delta):
    ctr = np.mean(pcd, axis=0)
    pcd_vectors = pcd - ctr
    pcd_norms = np.linalg.norm(pcd_vectors, axis=1)
    pcd_factors = (pcd_norms + delta) / pcd_norms
    new = ctr + (pcd_vectors * pcd_factors[:, np.newaxis])
    return new


# # For older versions of cdd
def verts_from_planes_og(planes):
    import cdd

    flip_plane_mat = np.hstack([-planes[:, 3:], -planes[:, :3]])
    mat = cdd.Matrix(flip_plane_mat.tolist(), number_type="fraction")
    mat.rep_type = cdd.RepType.INEQUALITY
    poly = cdd.Polyhedron(mat)
    ext = poly.get_generators()
    v_lists = [r[1:] for r in ext[: ext.row_size] if r[0] == 1]
    if len(v_lists) > 4:
        return np.array(v_lists, dtype=np.double)


# # For latest cdd (version 3.0)
def verts_from_planes_cdd(planes):
    import cdd

    # The old Matrix(...) is not matrix_from_array(...)
    # and old Polyhedron(...) is polyhedron_from_matrix(...)
    if not hasattr(cdd, "matrix_from_array"):
        return verts_from_planes_og(planes)
    flip_plane_mat = np.hstack([-planes[:, 3:], -planes[:, :3]])
    mat = cdd.matrix_from_array(
        flip_plane_mat.tolist(), rep_type=cdd.RepType.INEQUALITY
    )
    poly = cdd.polyhedron_from_matrix(mat)
    ext = cdd.copy_generators(poly)
    v_lists = [r[1:] for r in ext.array if r[0] == 1]
    if len(v_lists) > 4:
        return np.array(v_lists, dtype=np.double)


def verts_from_planes(planes):
    """Vertices of the polytope {x : n·x + d <= 0 for each plane [n, d]}.

    Implemented with scipy (Chebyshev-center LP + HalfspaceIntersection);
    the old repo used pycddlib, which has no binary wheels — fall back to it
    if installed.  Returns None when the intersection is empty/degenerate
    (matching the old behavior of requiring > 4 vertices).
    """
    try:
        import cdd  # noqa: F401

        return verts_from_planes_cdd(planes)
    except ImportError:
        pass

    from scipy.optimize import linprog
    from scipy.spatial import HalfspaceIntersection, QhullError

    A = planes[:, :3]
    b = planes[:, 3]
    norms = np.linalg.norm(A, axis=1)
    keep = norms > 1e-12
    A, b, norms = A[keep], b[keep], norms[keep]
    if len(A) < 4:
        return None

    # Chebyshev center: maximize r s.t. A x + r·||a_i|| <= -b.
    res = linprog(
        c=[0.0, 0.0, 0.0, -1.0],
        A_ub=np.hstack([A, norms[:, None]]),
        b_ub=-b,
        bounds=[(None, None)] * 3 + [(0, None)],
        method="highs",
    )
    if not res.success or res.x[3] < 1e-9:
        return None   # empty (or measure-zero) intersection

    try:
        hs = HalfspaceIntersection(np.hstack([A, b[:, None]]), res.x[:3])
    except (QhullError, ValueError):
        return None
    verts = np.asarray(hs.intersections, dtype=np.double)
    verts = verts[np.isfinite(verts).all(axis=1)]
    if len(verts) > 4:
        return verts


def point_cloud_from_depth_image_world_frame(
    depth_image: np.ndarray,
    camera_intrinsics: np.ndarray,
    T_world2dep: np.ndarray,
    max_depth_threshold: Optional[float] = None,
    min_depth_threshold: Optional[float] = None,
) -> np.ndarray:
    """Compute the point cloud in world frame from depth image."""
    point_cloud_cameraframe = point_cloud_from_depth_image_camera_frame(
        depth_image, camera_intrinsics, remove_invalid_points=False
    )

    T_dep2world = np.linalg.inv(T_world2dep)
    rotation, translation = T_dep2world[:3, :3], T_dep2world[:3, 3]
    world_pcd = rotation.dot(point_cloud_cameraframe.T).T + translation

    # set degenerate points (depth reading invalid) to [0,0,0]
    # they were set to 0, 0, 0 in the camera frame cloud
    world_pcd[(point_cloud_cameraframe == 0).all(axis=1)] = 0

    # move out-of-range points to 0,0,0
    if max_depth_threshold is not None:
        world_pcd[depth_image.reshape(-1) > max_depth_threshold] = np.array(
            [0.0, 0.0, 0.0]
        )
    if min_depth_threshold is not None:
        world_pcd[depth_image.reshape(-1) < min_depth_threshold] = np.array(
            [0.0, 0.0, 0.0]
        )

    # from qr_utils.pcd_utilities import visualize_pointclouds_colors
    # visualize_pointclouds_colors([world_pcd], [np.array([255, 0, 0])])

    return world_pcd


def dbscan_clusters_from_dense_pcd(
    pcd,
    voxel_size: float,
    eps: float,
    min_points: int,
    print_progress: bool = False,
):
    """
    Returns:
      clusters_orig: list[np.ndarray]
          clusters_orig[k] is a 1D array of ORIGINAL point indices belonging to cluster k (k=0..K-1).
      labels_orig: np.ndarray shape (N,)
          Label per ORIGINAL point index (−1 for noise / not in any cluster).
    """
    import open3d as o3d

    if not isinstance(pcd, o3d.geometry.PointCloud):
        raise TypeError("pcd must be an open3d.geometry.PointCloud")

    # 1) Downsample with trace (maps each downsampled point -> indices in original pcd)
    min_b = pcd.get_min_bound()
    max_b = pcd.get_max_bound()
    pcd_ds, _, trace = pcd.voxel_down_sample_and_trace(
        voxel_size=voxel_size,
        min_bound=min_b,
        max_bound=max_b,
        approximate_class=False,
    )
    # 'trace' is a Python list of arrays/lists of original point indices, length == len(pcd_ds.points)

    # 2) DBSCAN on the downsampled cloud
    labels_ds = np.array(
        pcd_ds.cluster_dbscan(
            eps=eps, min_points=min_points, print_progress=print_progress
        ),
        dtype=int,
    )
    n_ds = len(labels_ds)
    n_clusters = (labels_ds.max() + 1) if n_ds > 0 else 0

    # 3) Map cluster labels back to ORIGINAL indices
    N = np.asarray(pcd.points).shape[0]
    labels_orig = -np.ones(N, dtype=int)  # default: noise
    clusters_orig = [set() for _ in range(n_clusters)]

    for i_ds, lab in enumerate(labels_ds):
        if lab < 0:
            continue  # noise voxel
        orig_ids = np.asarray(trace[i_ds], dtype=int)
        clusters_orig[lab].update(orig_ids)

    # finalize: sets -> sorted arrays; fill labels_orig
    for lab, idx_set in enumerate(clusters_orig):
        arr = np.fromiter(idx_set, dtype=int)
        arr.sort()
        clusters_orig[lab] = arr
        labels_orig[arr] = lab

    return clusters_orig, labels_orig


def point_cloud_from_depth_image_camera_frame(
    depth_image: np.ndarray, camera_intrinsics: np.ndarray, remove_invalid_points=False
) -> np.ndarray:
    """Compute the point cloud in camera frame from depth image."""

    height, width = depth_image.shape
    xmap, ymap = np.meshgrid(np.arange(width), np.arange(height))
    homogenous_coord = np.concatenate(
        (xmap.reshape(1, -1), ymap.reshape(1, -1), np.ones((1, height * width)))
    )
    rays = np.linalg.inv(camera_intrinsics).dot(homogenous_coord)
    camera_pcd = depth_image.reshape(1, height * width) * rays
    camera_pcd = camera_pcd.transpose(1, 0).reshape(-1, 3)

    depth_image_linear = depth_image.reshape(-1)
    nan_idx = np.where(np.isnan(depth_image_linear))[0]
    camera_pcd[nan_idx] = np.array([0.0, 0.0, 0.0])

    return camera_pcd


def np_to_trimesh(points, colors=None):
    assert points.shape[-1] == 3
    points = points.reshape(-1, 3)
    if colors is None:
        pcd = trimesh.points.PointCloud(
            points, colors=np.tile(np.array([0, 0, 255]), (points.shape[0], 1))
        )
    else:
        pcd = trimesh.points.PointCloud(points, colors)
    return pcd


def np_to_o3d(points, colors=None):
    import open3d as o3d

    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    if colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(colors / 255)
    return pcd


def trimesh_to_o3d(point_cloud):
    return np_to_o3d(np.asarray(point_cloud.vertices))


def o3d_to_trimesh(point_cloud):
    return np_to_trimesh(np.asarray(point_cloud.points))


def o3d_to_np(point_cloud):
    return np.asarray(point_cloud.points)


def recenter_pointcloud(
    point_cloud: trimesh.points.PointCloud,
) -> Tuple[trimesh.points.PointCloud, np.ndarray, np.ndarray]:
    centering = np.eye(4)
    uncentering = np.eye(4)
    bounds = point_cloud.bounds
    center = 0.5 * (bounds[0] + bounds[1])
    centering[:3, 3] = -center
    uncentering[:3, 3] = center
    point_cloud = point_cloud.copy()
    point_cloud = point_cloud.apply_transform(centering)  # apply_transform is in-place
    return point_cloud, uncentering, centering


def recenter_vertices(
    vertices: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    centering = np.eye(4)
    uncentering = np.eye(4)
    bounds = vertices.min(axis=0), vertices.max(axis=0)
    center = 0.5 * (bounds[0] + bounds[1])
    centering[:3, 3] = -center
    uncentering[:3, 3] = center
    return vertices - center, uncentering, centering


# Each value of mask_or_color_list can be:
# None, an N x 3 array of rgb values, or an N array of Booleans
# If colors, then we color the corresponding pointcloud with them
# If a mask, then we color the positive points red and the others black
# Using multiple point_clouds with multiple masks is pretty much guaranteed to be confusing
def visualize_pointclouds(
    point_cloud_list: List[np.ndarray],
    mask_or_color_list: List[Union[np.ndarray, None]],
    downsample_voxel_size=None,
    message=None,
):
    if message:
        print("Visualizing meshes: ", message)
    pcds = []
    if mask_or_color_list is None:
        mask_or_color_list = [None] * len(point_cloud_list)
    for point_cloud, mask_or_color in zip(point_cloud_list, mask_or_color_list):
        if mask_or_color is None:
            colors = None
        elif len(mask_or_color.shape) == 2:
            # color N x 3 array
            colors = mask_or_color
            if colors.max() <= 1:
                colors *= 255
        else:
            # mask N array
            colors = np.zeros((point_cloud.shape[0], 3))
            colors[mask_or_color] = np.array([255.0, 0, 0])

        clean_points = ~np.isnan(point_cloud).any(axis=1)
        point_cloud = point_cloud[clean_points]
        colors = colors[clean_points]

        if DISPLAY_TRIMESH_OR_O3D == "trimesh":
            pcd_trimesh = np_to_trimesh(point_cloud, colors)
            pcds.append(pcd_trimesh)
        else:
            pcd_o3d = np_to_o3d(point_cloud, colors)
            if downsample_voxel_size is not None:
                pcd_o3d = pcd_o3d.voxel_down_sample(downsample_voxel_size)
            pcds.append(pcd_o3d)
    if DISPLAY_TRIMESH_OR_O3D == "trimesh":
        trimesh_show(pcds)
    else:
        open3d_show(pcds)


def visualize_pointclouds_colors(
    point_cloud_list: List[np.ndarray],
    color_list: List[np.ndarray],
    downsample_voxel_size=None,
    message=None,
):
    if message:
        print("Visualizing meshes: ", message)
    pcds = []
    for point_cloud, color in zip(point_cloud_list, color_list):
        colors = np.zeros((point_cloud.shape[0], 3))
        colors[:, :] = color

        if DISPLAY_TRIMESH_OR_O3D == "trimesh":
            pcd_trimesh = np_to_trimesh(point_cloud, colors)
            pcds.append(pcd_trimesh)
        else:
            pcd_o3d = np_to_o3d(point_cloud, colors)
            if downsample_voxel_size is not None:
                pcd_o3d = pcd_o3d.voxel_down_sample(downsample_voxel_size)
            pcds.append(pcd_o3d)
    if DISPLAY_TRIMESH_OR_O3D == "trimesh":
        trimesh_show(pcds)
    else:
        open3d_show(pcds)


def visualize_pointcloud(
    point_cloud: np.ndarray,
    mask_or_color: Union[np.ndarray, None] = None,
    downsample_voxel_size=0.001,
    message=None,
):
    visualize_pointclouds(
        [point_cloud], [mask_or_color], downsample_voxel_size, message=message
    )


def visualize_meshes(meshes, colors=None, message=None):
    if message:
        print("Visualizing meshes: ", message)

    if not meshes:
        return
    if DISPLAY_TRIMESH_OR_O3D == "trimesh":
        trimesh_show(meshes)
    else:
        o3d_meshes = [
            mesh_trimesh_to_o3d(mesh, color)
            for (mesh, color) in zip(meshes, colors)
            if mesh
        ]
        open3d_show(o3d_meshes)


def visualize_labeled_np_pcd(pcd, labels):
    cm = plt.get_cmap("rainbow")
    print([(k, np.count_nonzero(labels == k)) for k in np.unique(labels)])
    NUM_COLORS = labels.max() + 1
    colors = cm(labels / NUM_COLORS) * 255
    colors[labels == -1] = np.array([255, 0, 0, 1])  # not in a cluster
    visualize_pointcloud(pcd, colors[:, :3])


def visualize_labeled_o3d_pcd(pcd, labels):
    import open3d as o3d

    cm = plt.get_cmap("rainbow")
    print([(k, np.count_nonzero(labels == k)) for k in np.unique(labels)])
    NUM_COLORS = labels.max() + 1
    colors = cm(labels / NUM_COLORS)
    colors[labels == -1] = np.array([0.01, 0.01, 0.01, 1.0])  # not in a cluster
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    open3d_show([pcd])


def open3d_show(stuff):
    import open3d as o3d

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    for thing in stuff:
        vis.add_geometry(thing)
    vis.run()


def trimesh_show(stuff):
    global PERC_VIEWER
    from qr_utils.perc_viewer import ViewerPerc

    if PERC_VIEWER is None:
        PERC_VIEWER = ViewerPerc(scale=2)
    PERC_VIEWER.clear()
    PERC_VIEWER.add(stuff)


def mesh_trimesh_to_o3d(tr_mesh, color=None):
    import open3d as o3d

    o3d_mesh = o3d.geometry.TriangleMesh.create_box()
    o3d_mesh.triangles = o3d.utility.Vector3iVector(np.asarray(tr_mesh.faces))
    o3d_mesh.vertices = o3d.utility.Vector3dVector(np.asarray(tr_mesh.vertices).copy())
    o3d_mesh.paint_uniform_color(color / 255)
    return o3d_mesh


def tidy_point_cloud_wrt_box(trimesh_point_cloud, box, is_surface, display=False):
    vertices = np.asarray(trimesh_point_cloud.vertices)
    zlo, zhi = box[:, 2]
    vertices_within_z_box = vertices[(vertices[:, 2] >= zlo) & (vertices[:, 2] <= zhi)]
    if vertices_within_z_box.shape[0] < 10:
        print("Too few points remaining above surface.  Discarding")
        return None
    pc_within_z_tri = trimesh.points.PointCloud(vertices_within_z_box)

    if display:
        print("After z filter", pc_within_z_tri.vertices.shape[0])
        show_completion(trimesh_point_cloud.vertices, pc_within_z_tri.vertices)

    if (
        not pc_within_z_tri
        or pc_within_z_tri.vertices.shape[0] == 0
        or (np.prod(pc_within_z_tri.extents) < 1e-8 and not is_surface)
    ):
        print("Point cloud empty or small volume")
        return None

    return pc_within_z_tri


# Arguments are trimesh point clouds
def show_completion(start_points, completed_points, message=None):
    completed_color = np.array([255.0, 0, 0])
    start_color = np.array([0, 255, 0])
    completed_n = len(completed_points)
    start_n = len(start_points)
    completed_colors = np.tile(completed_color, reps=[completed_n, 1])
    start_colors = np.tile(start_color, reps=[start_n, 1])
    visualize_pointclouds(
        [start_points, completed_points],
        [start_colors, completed_colors],
        message=message,
    )


def shrink_mask(mask, depth, thr=0.02, offset=3):
    from scipy import ndimage

    struct2 = np.ones((7, 7))
    mask = mask.reshape(depth.shape)
    zmask = np.logical_and(mask, depth == 0.0)
    offsets_2 = [(u, v) for u in range(-offset, offset + 1) for v in range(-3, 4)]

    NU, NV = depth.shape
    for u, v in zip(*np.where(np.logical_and(mask, depth))):
        max_diff = 0
        d_uv = depth[u, v]
        for du, dv in offsets_2:
            if 0 <= u + du < NU and 0 <= v + dv < NV:
                d = depth[u + du, v + dv]
                if d:
                    max_diff = max(max_diff, abs(d_uv - d))
                    if max_diff > thr:
                        zmask[u, v] = True
                        break

    zmask_dil = ndimage.binary_dilation(zmask, structure=struct2)
    nmask = np.logical_and(mask, np.logical_not(zmask_dil))

    # plt.figure('Depth')
    # plt.imshow(np.minimum(1, depth), interpolation='none')
    # plt.show(block=False)
    # plt.figure('Mask')
    # plt.imshow(mask, interpolation='none')
    # plt.show(block=False)
    # plt.figure('ZMask')
    # plt.imshow(nmask, interpolation='none')
    # plt.show(block=True)
    return nmask.reshape(-1)


# Point cloud dilation/erosion from https://github.com/jbalado/Tutorial-MM/


def make_SE(d=0.005):
    # SE definition
    SE = np.array(
        [[0, 0, 0], [0, 0, -d], [0, 0, d], [0, -d, 0], [0, d, 0], [-d, 0, 0], [d, 0, 0]]
    )
    return SE, d


def mm_dilate(input_data, SE=None, d=None):
    import open3d as o3d

    if SE is None:
        SE, d = make_SE()
    dilated_data = input_data[:, 0:3]

    for i in range(1, SE.shape[0]):
        # Move point i from the SE to the whole input cloud
        translated_SE = input_data[:, 0:3] + SE[i, :]

        # Convert to SE traslated to cloud-object
        pcd_tSE = o3d.geometry.PointCloud()
        pcd_tSE.points = o3d.utility.Vector3dVector(translated_SE)

        # Convert concatenated point cloud to cloud-object
        pcd_dil = o3d.geometry.PointCloud()
        pcd_dil.points = o3d.utility.Vector3dVector(dilated_data)

        # Calculate distances between clouds
        dist_pcd_tSE_2_pcd_dil = pcd_tSE.compute_point_cloud_distance(pcd_dil)
        dist_pcd_tSE_2_pcd_dil = np.asarray(dist_pcd_tSE_2_pcd_dil)

        # Checking the existence of nearby points
        idx_add = dist_pcd_tSE_2_pcd_dil > d / 2

        # Adding new points to the dilated cloud
        dilated_data = np.vstack((dilated_data, translated_SE[idx_add, 0:3]))

    return dilated_data


def mm_erode(input_data, SE=None, d=None):
    import open3d as o3d

    if SE is None:
        SE, d = make_SE()
    # Convert the input points to point cloud-object
    pcd_in = o3d.geometry.PointCloud()
    pcd_in.points = o3d.utility.Vector3dVector(input_data[:, 0:3])

    # Generate indices of points to keep
    idx_remain = np.ones(input_data.shape[0], dtype=bool)

    for i in range(1, SE.shape[0]):
        # Move point i from the SE to the whole cloud
        translated_SE = input_data[:, 0:3] + SE[i, :]

        # Convert to SE moved to point cloud-object
        pcd_tSE = o3d.geometry.PointCloud()
        pcd_tSE.points = o3d.utility.Vector3dVector(translated_SE)

        # Calculate distances between clouds
        dist_pcd_tSE_2_pcd_in = pcd_tSE.compute_point_cloud_distance(pcd_in)
        dist_pcd_tSE_2_pcd_in = np.asarray(dist_pcd_tSE_2_pcd_in)

        # Filtering by distances of points in the input cloud that match points in the SE
        idx_aux = dist_pcd_tSE_2_pcd_in < d

        # Combination with the index list of input data
        idx_remain = idx_remain * idx_aux

    # Selection of the output points according to the indexes of the points to be preserved.
    output_data = input_data[idx_remain, 0:3]

    return output_data


"""The Intersection Over Union (IoU) for 3D oriented bounding boxes."""


def z_iou(o1, o2):
    z_range_1 = o1.mesh_worldframe.bounds[:, 2]
    z_range_2 = o2.mesh_worldframe.bounds[:, 2]
    z_overlap = min(z_range_1[1], z_range_2[1]) - max(z_range_1[0], z_range_2[0])
    z_union = max(z_range_1[1], z_range_2[1]) - min(z_range_1[0], z_range_2[0])
    return z_overlap / z_union


# Return the score and the object with the maximum intersection over minimum
def max_iom(obj_mesh_verts, list_of_obj_mesh_verts):
    best = 0.0
    best_det = None
    for i, ho in enumerate(list_of_obj_mesh_verts):
        iom = IoM(obj_mesh_verts, ho)
        if iom > best:
            best = iom
            best_det = i
    return best, best_det
