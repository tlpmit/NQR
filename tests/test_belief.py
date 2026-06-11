"""
End-to-end tests of the BeliefModule against simulator ground truth:
sim camera → perception/OBM (object memory) + qr_mapping voxel grid
(spatial memory) + robot memory, all driven through BeliefModule.update.
"""

import numpy as np
import pytest

from qr_api.perc_typing import CalibratedRGBDObservation
from qr_robots.common import rby1
from tests.conftest import SIM_MODE, sim_fixture

BLOCK_POS = np.array([0.65, 0.0, 0.79])   # 8 cm cube from block.xml
TABLE_TOP_Z = 0.75


@pytest.fixture(scope="module")
def sim(test_scene_dir):
    from qr_robots.mujoco.rby1.sim import make_sim

    sim = make_sim(test_scene_dir, {
        "table": {"file": "table.xml", "fixed": True},
        "block": {"file": "block.xml"},
    }, mode=SIM_MODE)
    sim.point_head_at(BLOCK_POS)
    yield from sim_fixture(sim, "belief")


def observe(sim):
    rgb, depth, label = sim.get_camera_image("head_camera")
    im = CalibratedRGBDObservation(
        rgb, depth,
        sim.get_camera_intrinsics("head_camera"),
        sim.get_camera_extrinsics("head_camera"),
        label_image=label,
    )
    im.camera_params = (60.0, 720, 1280, 0.05, 3.0)
    return {"images": [im], "conf": rby1.parse_robot_vector(sim.get_robot_conf())}


@pytest.fixture(scope="module")
def belief(sim):
    from qr_api.policy_typing import Action
    from qr_belief.builder import make_sim_belief_module

    belief = make_sim_belief_module(
        workspace=((-1.0, -1.5, 0.0), (2.0, 1.5, 1.5)),
        shadow_extents=(3.0, 3.0, 1.2),
        shadow_pose=(-1.0, -1.5, 0.0, 0.0, 0.0, 0.0),
        voxel_grid_resolution=0.04,
    )
    belief.reset(observe(sim))
    belief.update(Action(), None, observe(sim))
    return belief


def vox_index(vv, p):
    centers = vv.voxel_centers
    flat = np.linalg.norm(centers.reshape(-1, 3) - np.asarray(p), axis=1).argmin()
    return tuple(int(i) for i in np.unravel_index(flat, centers.shape[:3]))


def test_spatial_memory_occupancy_matches_scene(belief):
    vv = belief.spatial_memory.vv

    # Visible block faces are occupied (front face toward the robot, top face).
    assert vv.voxel_occupied_bin[vox_index(vv, BLOCK_POS + [-0.04, 0, 0])]
    assert vv.voxel_occupied_bin[vox_index(vv, BLOCK_POS + [0, 0, 0.04])]

    # The visible table top is solidly occupied: sample a band of points.
    xs = np.linspace(0.45, 1.1, 8)
    hits = sum(
        bool(vv.voxel_occupied_bin[vox_index(vv, [x, y, TABLE_TOP_Z - 0.01])])
        for x in xs for y in (-0.2, 0.0, 0.2)
    )
    assert hits / 24 > 0.5

    # The block interior is occluded by its own surface → shadow (unknown).
    assert vv.voxel_shadows_bin[vox_index(vv, BLOCK_POS - [0, 0, 0.01])]

    # Under the table is occluded → shadow.
    assert vv.voxel_shadows_bin[vox_index(vv, [0.9, 0.0, 0.4])]

    # Air between the camera and the table was seen through → free.
    air = vox_index(vv, [0.4, 0.0, 1.1])
    assert not vv.voxel_occupied_bin[air] and not vv.voxel_shadows_bin[air]


def test_object_memory_matches_world(belief, sim):
    objects = belief.object_memory.memory.objects
    cats = [h.get_feature("category", "thing") for h in objects.values()]
    assert sorted(cats) == ["surface", "thing"]

    block = next(h for h in objects.values()
                 if h.get_feature("category", "thing") == "thing")
    center = block.mesh_worldframe.to_trimesh().bounds.mean(axis=0)
    assert np.linalg.norm(center - sim.get_world_state()["block"][:3]) < 0.02


def test_robot_memory_matches_conf(belief, sim):
    conf = rby1.parse_robot_vector(sim.get_robot_conf())
    np.testing.assert_allclose(
        belief.robot_memory.robot_conf["base"], conf["base"], atol=1e-9
    )
    assert belief.robot_memory.holding() == {}


def test_spatial_memory_updates_on_object_motion(belief, sim):
    from qr_api.policy_typing import Action

    vv = belief.spatial_memory.vv
    old_front = vox_index(vv, BLOCK_POS + [-0.04, 0, 0])
    assert vv.voxel_occupied_bin[old_front]

    # Move the block away and re-observe: the rays now pass through the old
    # location and carve it free.
    sim.set_object_pose("block", BLOCK_POS + [0.25, -0.3, 0.0], [1, 0, 0, 0])
    sim.step(100)
    belief.update(Action(), None, observe(sim))

    assert not vv.voxel_occupied_bin[old_front]
    new_top = vox_index(vv, BLOCK_POS + [0.25, -0.3, 0.04])
    assert vv.voxel_occupied_bin[new_top]

    # Put it back for any later tests.
    sim.set_object_pose("block", BLOCK_POS, [1, 0, 0, 0])
    sim.step(100)
    belief.update(Action(), None, observe(sim))
