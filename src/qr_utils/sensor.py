import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix

from qr_utils.raster_scan import Raster
from qr_utils.traceFile import tr, tr_a

# def camera_params_from_intrinsics_matrix(intrinsic_matrix, zlo=0.05, zhi=3.0):
#     """
#     Extract camera parameters from the intrinsic matrix.

#     Args:
#         intrinsic_matrix (np.ndarray): The intrinsic matrix of the camera.

#     Returns:
#         tuple: A tuple containing the camera parameters:
#             - fov_deg_width (float): Field of view in degrees (width).
#             - res_height_pix (int): Height of the image in pixels.
#             - res_width_pix (int): Width of the image in pixels.
#             - zlo (float): Near clipping plane distance.
#             - zhi (float): Far clipping plane distance.
#     """
#     fov_deg_width = 2 * np.arctan(intrinsic_matrix[0, 2] / intrinsic_matrix[1, 1]) * 180 / np.pi
#     res_height_pix = intrinsic_matrix[1, 2] * 2
#     res_width_pix = intrinsic_matrix[0, 2] * 2
#     return fov_deg_width, res_height_pix, res_width_pix, zlo, zhi

# def intrinsics_matrix_from_camera_params(params):
#     fov_deg_width, res_height_pix, res_width_pix = params[:3]
#     pcp_x = res_width_pix//2
#     focal_length_x = pcp_x/np.tan(np.deg2rad(fov_deg_width/2))
#     pcp_y = res_height_pix//2
#     k = np.zeros((3,3), dtype=np.float32)
#     k[2,2] = 1.
#     k[0, 0] = focal_length_x
#     k[0, 2] = pcp_x
#     k[1,1] = focal_length_x
#     k[1, 2] = pcp_y
#     return k


class Sensor:
    sensor_count = 0

    def __init__(
        self,
        link: str,
        link_to_sensor: np.ndarray,
        intrinsics: np.ndarray,
        params: tuple,
        color_profile: dict = None,
    ):
        self.link = link
        self.link_to_sensor = link_to_sensor
        self._raster = None
        self.intrinsics = intrinsics
        self.params = params
        self.color_profile = color_profile

    def raster(self):
        if self._raster is None:
            self._raster = Raster(self.params)
        return self._raster

    def colliding_objects_to_point(self, trans, point, meshes):
        if not isinstance(trans, np.ndarray):
            trans = trans.T()
        eye = trans[:3, 3]
        vector = point - eye
        mag = np.linalg.norm(vector)
        vector /= mag
        origins = np.array([eye])
        vectors = np.array([vector])
        colliders = []
        for name, mesh in meshes.items():
            if mesh is None:
                continue
            points, _, _ = mesh.ray.intersects_location(
                origins, vectors, multiple_hits=False
            )
            if len(points) > 0 and np.linalg.norm(points[0] - eye) <= mag:
                colliders.append(name)
        return set(colliders)

    def trace_image(self, trans, umeshes=None, multiplier=1.0, debug=True):
        return Sensor.trace_image_raw(
            trans, self.intrinsics, self.params, umeshes, multiplier, debug
        )

    @staticmethod
    def trace_image_raw(
        trans,
        camera_intrinsics,
        camera_params,
        umeshes=None,
        multiplier=1.0,
        debug=True,
    ):
        tr("sensor", "Generating image...", end=" ")
        (fov_deg_width, res_height_pix, res_width_pix, zlo, zhi) = camera_params
        if umeshes and not isinstance(list(umeshes.values())[0], trimesh.Trimesh):
            meshes = {n: m.to_trimesh() for n, m in umeshes.items()}
        else:
            meshes = umeshes
        scene = trimesh.scene.scene.Scene(geometry=meshes.values())
        scene.camera_transform = trans
        aspect_ratio = res_height_pix / res_width_pix
        # array shape
        scene.camera.resolution = np.array([res_height_pix, res_width_pix])
        scene.camera.fov = np.array([fov_deg_width, fov_deg_width * aspect_ratio])
        height = int(res_height_pix)
        width = int(res_width_pix)
        # convert the camera to rays with one ray per pixel
        origins, tri_vectors, tri_pixel_ray = scene.camera_rays()
        xmap, ymap = np.meshgrid(np.arange(res_width_pix), np.arange(res_height_pix))
        height, width = xmap.shape
        homogenous_coord = np.concatenate(
            (xmap.reshape(1, -1), ymap.reshape(1, -1), np.ones((1, height * width)))
        )
        vectors = np.linalg.inv(camera_intrinsics).dot(homogenous_coord).T
        norms = np.linalg.norm(vectors, axis=1)
        vectors = vectors / norms[:, None]
        rot = trans[:3, :3]
        # Note that multiplier is usually -1
        vectors = multiplier * np.dot(rot, vectors.T).T
        pixel_ray = homogenous_coord.T[:, :2]

        obj_id_map = np.full(scene.camera.resolution, 0, dtype=int)
        depth_map = np.full(scene.camera.resolution, zhi, dtype=float)
        obj_pixel_counts = {}
        vis_pixel_counts = {}
        obj_index = {}
        use_trimesh = False  # Trimesh ray tracing is slow

        for i, (name, mesh) in enumerate(meshes.items()):
            if mesh is None:
                continue
            # do the actual ray-mesh queries
            # start = time.time()
            if use_trimesh:
                points, index_ray, _ = mesh.ray.intersects_location(
                    origins, vectors, multiple_hits=False
                )
            else:
                from ncollpyde import Volume

                volume = Volume(mesh.vertices, mesh.faces)
                targets = origins + 3 * vectors
                index_ray, points, bfs = volume.intersections(origins, targets, True)
                index_ray = index_ray[~bfs]
                points = points[~bfs]

            if points.shape[0] == 0.0:
                obj_pixel_counts[name] = 0
                continue
            # for each hit, find the distance along its vector
            # you could also do this against the single camera Z vector
            depth = trimesh.util.diagonal_dot(
                points - origins[index_ray], vectors[index_ray]
            )
            depth /= norms[index_ray]
            obj_pixel_counts[name] = depth.shape[0]
            pixels = pixel_ray[index_ray].astype(np.int32)
            obj_depth_map = np.full(scene.camera.resolution, zhi, dtype=float)
            # Note flip of X and Y
            obj_depth_map[pixels[:, 1], pixels[:, 0]] = depth

            idx = 0 if ("floor" in name or "table" in name) else i + 1
            obj_index[name] = idx
            obj_id_map = np.where(obj_depth_map < depth_map, idx, obj_id_map)
            depth_map = np.minimum(obj_depth_map, depth_map)

        for name, i in obj_index.items():
            if i >= 0:
                vis_pixel_counts[name] = np.count_nonzero(obj_id_map == i)

        if debug:
            # from matplotlib import pyplot as plt
            # plt.imshow(depth_map, cmap='jet')
            for name, i in obj_index.items():
                if i >= 0:
                    tr(
                        "sensor",
                        name,
                        "obj_count",
                        obj_pixel_counts[name],
                        "vis_count",
                        vis_pixel_counts[name],
                    )
            # plt.show()

        tr("sensor", "... finished generating image")
        return obj_id_map, depth_map, obj_index, obj_pixel_counts, vis_pixel_counts

    def count_occluders(self, trans, target_mesh, meshes, display=False):
        if target_mesh is None:
            return {"world": 100}, 100
        # display = True
        raster = self.raster()
        raster.reset()
        trans = np.dot(trans, rotation_matrix(np.pi, (1, 0, 0)))
        trans = np.linalg.inv(trans)
        mesh_c = target_mesh.copy().apply_transform(trans).convex_hull
        raster.update(mesh_c, 1)
        total = raster.countId(1)
        raster.save()
        # Count the pixels blocked by occluders
        mesh_i = 2
        occluders = {}
        for name, mesh in meshes.items():
            mesh_c = mesh.copy().apply_transform(trans).convex_hull
            raster.update(mesh_c, mesh_i)
            occ = raster.countIdVsSaved(mesh_i, 1)
            if occ:
                occluders[name] = occ
                assert occ <= total
            if not display:
                # Skip the raster.revert() to get full picture.
                raster.revert()
            mesh_i += 1

        if display:
            tr_a(f"{occluders=} {total=}")
            from matplotlib import pyplot as plt

            data = np.reshape(
                raster.frameBuffer, (raster.imageHeight, raster.imageWidth)
            )
            plt.imshow(data, interpolation="nearest")
            plt.show()
        return occluders, total


def project_points_to_image(points_3d, K, R, t):
    """
    Projects 3D points into 2D pixel coordinates using camera intrinsics and extrinsics.

    Parameters:
    - points_3d: np.ndarray of shape (N, 3)
    - K: np.ndarray of shape (3, 3) - intrinsic matrix
    - R: np.ndarray of shape (3, 3) - rotation matrix (world to camera)
    - t: np.ndarray of shape (3,)   - translation vector (world to camera)

    Returns:
    - pixels_2d: np.ndarray of shape (N, 2) - 2D pixel coordinates
    """
    # Ensure inputs are numpy arrays
    points_3d = np.asarray(points_3d)
    R = np.asarray(R)
    t = np.asarray(t).reshape(3, 1)

    # Convert to shape (3, N)
    points_3d_h = np.hstack([points_3d, np.ones((points_3d.shape[0], 1))])  # (N, 4)

    # Create the 3x4 extrinsic matrix
    extrinsic = np.hstack((R, t))  # (3, 4)

    # Project to camera coordinates
    camera_coords = extrinsic @ points_3d_h.T  # shape: (3, N)

    # Project to pixel coordinates
    pixel_homog = K @ camera_coords  # shape: (3, N)

    # Normalize by the third coordinate
    pixel_coords = pixel_homog[:2, :] / pixel_homog[2, :]

    return pixel_coords.T  # shape: (N, 2)
