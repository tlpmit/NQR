from typing import Dict, List

import networkx as nx
import numpy as np
import shapely
import trimesh
from Roboverse.utils.shapely_geometry import Point, Polygon
from Utils.miscUtil import dict_add

import qr_api.obm_typing as O
import qr_api.perc_typing as T
from qr_utils.pcd_utilities import max_iom, z_iou
from qr_utils.traceFile import tr, tr_a


# Return a set of objects that do not overlap (much)
# In case of conflict, prefer the object with higher existence probability
def get_ml_object_set(all_objects: List[O.ObjectBelief]) -> List[O.ObjectBelief]:
    min_confidence = 0.5  # Don't take any objects with confidence less than this
    allowable_total_overlap = 0.05  # LPK Just decreased from 0.1
    allowable_z_overlap = 0.35
    objects = [o for o in all_objects if o.existence_confidence > min_confidence]
    sorted_objs = sorted(objects, key=lambda x: x.existence_confidence, reverse=True)
    result = []
    for obj in sorted_objs:
        is_surface = obj.get_feature("category", False) == "surface"
        max_iom_val, other_obj = max_iom(obj, result)
        if (
            is_surface
            or (max_iom_val < allowable_total_overlap)
            or (max_iom_val < 0.001)
        ):
            result.append(obj)
        elif other_obj is not None and z_iou(obj, other_obj) < allowable_z_overlap:
            # There is substantial overlap, but it is in the z direction
            result.append(obj)
        else:
            tr(
                ("mpp", "log"),
                f"Omitting {obj.name} from ml object set due to overlap with {other_obj.name}",
            )
    return make_physically_plausible(result)


bot = 0
top = 1
eps = 0.002


def make_physically_plausible(objects: List[O.ObjectBelief]) -> List[O.ObjectBelief]:
    placed_mesh_dict = {}
    placed_mesh_polys_dict = {}
    for obj in objects:
        placed_mesh = obj.mesh.apply_transform(obj.m4_pose)
        bounds = placed_mesh.bounds
        placed_mesh_dict[obj.name] = placed_mesh
        placed_mesh_polys_dict[obj.name] = Polygon(
            sh_poly=shapely.MultiPoint(placed_mesh.vertices[:, :2]).convex_hull,
            z_range=(bounds[0, 2], bounds[1, 2]),
        )
    supports, is_supported_by = find_supports(placed_mesh_dict, placed_mesh_polys_dict)
    # Get delta_z for top and bottom of each object.  Get a dict of (bot_dz, top_dz) for each object
    delta_z_dict = find_delta_z(supports, is_supported_by, placed_mesh_dict)
    # Update the z ranges for polys, to avoid false z collisions
    update_z_ranges(placed_mesh_polys_dict, delta_z_dict)
    # Find the xy displacement to separate colliding objects
    # delta_xy_dict = find_delta_xy(placed_mesh_polys_dict)
    delta_xy_dict = {}
    # Extend the mesh and derive the new poses
    # Make all the changes by side-effects
    adjusted_objects = []
    for o in objects:
        tr(("mpp", "log"), "operating on object", o.name)
        if o.name in delta_z_dict:
            mesh_adjust_z(o, placed_mesh_dict[o.name], delta_z_dict[o.name])
        mesh_adjust_xy(o, placed_mesh, delta_xy_dict)
        adjusted_objects.append(o)
    return adjusted_objects


# Note that this is broken if there are multiple tables
def find_delta_z(
    supports: Dict[str, List[str]],
    is_supported_by: Dict[str, List[str]],
    placed_mesh_dict: Dict[str, T.UncertainObjectMesh],
) -> Dict[str, List[float]]:
    # bot_dz, top_dz
    delta_z = {x: [0.0, 0.0] for x in placed_mesh_dict}
    if len(placed_mesh_dict) <= 1:
        return delta_z
    # Unsupported objects
    roots = [x for x, sup in is_supported_by.items() if not sup]
    support_graph = nx.DiGraph()
    for support, supported_set in supports.items():
        for supported in supported_set:
            support_graph.add_edge(support, supported)
    valid = set(roots)
    for x in roots:
        if x not in support_graph:
            continue
        for support, supported in nx.bfs_edges(support_graph, x):
            valid.add(supported)
            support_top_z = placed_mesh_dict[support].bounds[1, 2]
            supported_bot_z = placed_mesh_dict[supported].bounds[0, 2]
            # if support_top_z is bigger, we need to increase supported_top_z
            diff_z = support_top_z - supported_bot_z

            if abs(diff_z) > 0.025:
                tr(
                    ("mpp", "log"),
                    f"make_physically_plausible, skipping support {diff_z=}",
                )
                continue

            supported_dz = delta_z[supported]
            support_dz = delta_z[support]
            if support in roots:
                supported_dz[bot] = diff_z
            else:
                if diff_z >= 0:
                    supported_dz[bot] = diff_z
                else:  # diff_z is negative
                    supported_dz[bot] = 0.5 * diff_z
                    support_dz[top] = -0.5 * diff_z
    for o, dz in delta_z.items():
        tr(("mpp", "log"), f"{o=} {dz=}")
    return delta_z


def update_z_ranges(
    placed_mesh_polys_dict: Dict[str, Polygon], delta_z_dict: Dict[str, List[float]]
):
    for obj_name, dz in delta_z_dict.items():
        if dz[bot] or dz[top]:
            z_range = placed_mesh_polys_dict[obj_name].z_range
            new_z_range = [z_range[bot] + dz[bot] + eps, z_range[top] + dz[top] - eps]
            # lpk : dealing with very thin objects
            if new_z_range[top] < new_z_range[bot]:
                # make them at least eps thick
                new_z_range[top] = new_z_range[bot] + eps
            placed_mesh_polys_dict[obj_name].z_range = new_z_range


# Displace colliding objects on the basis of their polygons
# TODO: Generalize to union of convex polyhedra
def find_delta_xy(placed_mesh_polys_dict: Dict[str, Polygon]) -> Dict[tuple, tuple]:
    delta_xy = {}
    if len(placed_mesh_polys_dict) <= 1:
        return delta_xy
    polys = {}
    for obj_name_1, poly_1 in placed_mesh_polys_dict.items():
        polys[obj_name_1] = poly_1
        for obj_name_2, poly_2 in placed_mesh_polys_dict.items():
            if obj_name_1 == obj_name_2:
                continue
            polys[obj_name_2] = poly_2
            pair = (obj_name_1, obj_name_2)
            pair_rev = (obj_name_2, obj_name_1)
            if pair in delta_xy or pair_rev in delta_xy:
                continue
            zr1 = polys[pair[0]].z_range
            zr2 = polys[pair[1]].z_range
            overlap = min(zr1[1], zr2[1]) - max(zr1[0], zr2[0])
            if overlap > 0.01 and poly_1.collides(poly_2):
                # TODO: choose which cut would be most conservative
                sep = xy_separation(
                    placed_mesh_polys_dict[pair[0]], placed_mesh_polys_dict[pair[1]]
                )
                if sep is not None:
                    tr("log", f"XY cut from {pair[0]} for {pair[1]} by {sep}")
                    delta_xy[pair] = sep
                else:
                    tr("log", f"XY cut from {pair[0]} for {pair[1]} was Npne")
    return delta_xy


def xy_separation(poly_B: Polygon, poly_A: Polygon):
    CO = poly_B.cspace_poly(poly_A)
    # poly_A is in absolute coordinates, so ref is (0,0)
    P0 = Point(0, 0)
    if CO.contains(P0, edges=True):
        snorm = None
        sd = None
        for norm, d in CO.get_line_eqs():
            assert d <= 0.0  # since we're inside CO
            if sd is None or abs(d) < sd:
                sd = abs(d)
                snorm = norm
        sd += 0.005
        return snorm.x, snorm.y, sd
    # assert None, 'Should not happen'
    return


def mesh_adjust_xy(
    o: O.ObjectBelief, placed_mesh: T.UncertainObjectMesh, delta_xy: Dict[tuple, tuple]
):
    changed = False
    pairs = [pair for pair in delta_xy if o == pair[1]]
    points = placed_mesh.vertices
    for pair in pairs:
        # Move along direction (sep_x, sep_y) by distace sep_d to separate
        sep_x, sep_y, sep_d = delta_xy[pair]
        plane = np.array([sep_x, sep_y, 0.0])
        dists = np.dot(plane, points.T)
        min_d = np.min(dists)
        max_d = np.max(dists)
        if sep_d >= max_d - min_d:
            tr_a("Separation exceeds width of object")
            return
        plane_3d = np.array([sep_x, sep_y, 0.0, -(min_d + sep_d)])
        placed_mesh = slice_mesh(placed_mesh, plane_3d)
        changed = True
    if changed:
        o.mesh = placed_mesh.apply_transform(np.linalg.inv(o.m4_pose))
        tr(("mpp", "log"), "mesh_adjust_xy Final")
        tr(("mpp", "log"), placed_mesh.bounds)
    return o


def mesh_adjust_z(
    o: O.ObjectBelief, placed_mesh: T.UncertainObjectMesh, delta_z: Dict[tuple, tuple]
):
    if not any(delta_z):
        return o
    bounds = placed_mesh.bounds
    tr(("mpp", "log"), "mesh_adjust_z Initial", delta_z)
    tr(("mpp", "log"), bounds)
    bot_dz = delta_z[bot] + eps
    top_dz = delta_z[top] - eps

    # lpk trying to handle very thin objects
    real_z_bounds = bounds[:, 2]
    proposed_z_bounds = real_z_bounds + np.array([bot_dz, top_dz])
    if proposed_z_bounds[1] < proposed_z_bounds[0] + eps:
        # make them at least eps thick
        proposed_z_bounds[1] = proposed_z_bounds[0] + eps
        top_dz = proposed_z_bounds[1] - real_z_bounds[1]
        tr_a("New top dz for thin object", top_dz)

    if top_dz == 0 and bot_dz == 0:
        return o

    if bot_dz > 0.0:
        placed_mesh = shave_z(placed_mesh, bounds[0, 2], bot_dz)
    elif bot_dz < 0:
        placed_mesh = grow_z(placed_mesh, bounds[0, 2], bot_dz)

    if top_dz > 0:
        placed_mesh = grow_z(placed_mesh, bounds[1, 2], top_dz)
    elif top_dz < 0:
        placed_mesh = shave_z(placed_mesh, bounds[1, 2], top_dz)

    o.mesh = placed_mesh.apply_transform(np.linalg.inv(o.m4_pose))
    tr(("mpp", "log"), "mesh_adjust_xy Final")
    tr(("mpp", "log"), placed_mesh.bounds)
    return o


def shave_z(placed_mesh: T.UncertainObjectMesh, z: float, delta_z: float):
    if abs(delta_z) < 0.001:
        return placed_mesh
    if delta_z > 0:
        plane_3d = np.array([0.0, 0.0, 1.0, -(z + delta_z)])
    else:
        plane_3d = np.array([0.0, 0.0, -1.0, (z + delta_z)])
    return slice_mesh(placed_mesh, plane_3d)


def grow_z(placed_mesh, z, delta_z):
    tr(("mpp", "log"), "grow_z is not implemented", placed_mesh.bounds, z, delta_z)
    return placed_mesh


def find_supports(
    placed_mesh_dict: Dict[str, T.UncertainObjectMesh],
    placed_mesh_polys_dict: Dict[str, Polygon],
):
    def obj_above(obj_name_1, obj_name_2):
        if obj_name_1 == obj_name_2:
            return False
        bounds_1 = placed_mesh_dict[obj_name_1].bounds
        bounds_2 = placed_mesh_dict[obj_name_2].bounds
        frac = 0.5
        if bounds_2[1, 2] > (bounds_1[1, 2] * (1 - frac) + bounds_1[0, 2] * frac):
            return False
        poly_1 = placed_mesh_polys_dict[obj_name_1]
        poly_2 = placed_mesh_polys_dict[obj_name_2]
        overlap_poly = poly_1.sh_polygon.intersection(poly_2.sh_polygon)
        overlap_frac = overlap_poly.area / min(
            poly_1.sh_polygon.area, poly_2.sh_polygon.area
        )
        return overlap_frac > 0.1

    supports = {obj_name: set() for obj_name in placed_mesh_dict}
    is_supported_by = {obj_name: set() for obj_name in placed_mesh_dict}
    # Collect support relations and nedded offsets
    for obj_name_1 in placed_mesh_dict:
        for obj_name_2 in placed_mesh_dict:
            if obj_above(obj_name_1, obj_name_2) and not [
                obj_name_3
                for obj_name_3 in placed_mesh_dict
                if obj_above(obj_name_3, obj_name_2)
                and obj_above(obj_name_1, obj_name_3)
            ]:
                dict_add(supports, obj_name_2, obj_name_1)
                dict_add(is_supported_by, obj_name_1, obj_name_2)
    return supports, is_supported_by


def slice_mesh(uncertain_mesh: T.UncertainObjectMesh, plane: np.ndarray):
    whole = trimesh.Trimesh(uncertain_mesh.vertices, uncertain_mesh.faces)
    normal = plane[:3]
    d = plane[3]
    try:
        sliced_mesh = trimesh.intersections.slice_mesh_plane(
            whole, normal, -d * normal
        ).convex_hull
        return T.UncertainObjectMesh(
            sliced_mesh.vertices, sliced_mesh.faces, np.ones(len(sliced_mesh.vertices))
        )
    except Exception:
        return uncertain_mesh
