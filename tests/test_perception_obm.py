"""
End-to-end tests of the perception pipeline + OBM against simulator ground
truth: sim camera → SimSegmenter/ProjectionCompletion/SurfaceFinder → OBM,
checking that the object memory closely matches the actual world state.
"""

import numpy as np
import pytest

from qr_api.perc_typing import CalibratedRGBDObservation
from tests.conftest import SIM_MODE, sim_fixture

# Ground truth from the scene files (block.xml, blue_block.xml, table.xml).
RED_POS = np.array([0.65, 0.0, 0.79])
BLUE_POS = np.array([0.95, -0.2, 0.78])
TABLE_TOP_Z = 0.75
CENTER_TOL = 0.02  # m
EXTENT_TOL = 0.015  # m


@pytest.fixture(scope="module")
def sim(test_scene_dir):
    from qr_robots.mujoco.rby1.sim import make_sim

    sim = make_sim(
        test_scene_dir,
        {
            "table": {"file": "table.xml", "fixed": True},
            "block": {"file": "block.xml"},
            "blue_block": {"file": "blue_block.xml"},
        },
        mode=SIM_MODE,
    )
    sim.point_head_at([0.8, -0.1, 0.79])
    yield from sim_fixture(sim, "perception obm")


def observe(sim):
    rgb, depth, label = sim.get_camera_image("head_camera")
    im = CalibratedRGBDObservation(
        rgb,
        depth,
        sim.get_camera_intrinsics("head_camera"),
        sim.get_camera_extrinsics("head_camera"),
        label_image=label,
    )
    im.camera_params = (60.0, 720, 1280, 0.05, 3.0)
    return {"images": [im]}


def reset_scene(sim):
    sim.set_object_pose("block", RED_POS, [1, 0, 0, 0])
    sim.set_object_pose("blue_block", BLUE_POS, [1, 0, 0, 0])
    sim.step(100)


def make_obm():
    from qr_perception.pipelines import make_sim_obm

    return make_sim_obm()


def hyp_center(h):
    return h.mesh_worldframe.to_trimesh().bounds.mean(axis=0)


def things_and_surfaces(memory):
    things = {
        k: h
        for k, h in memory.objects.items()
        if h.get_feature("category", "thing") == "thing"
    }
    surfaces = {
        k: h
        for k, h in memory.objects.items()
        if h.get_feature("category", "thing") in ("surface", "table")
    }
    return things, surfaces


def find_by_color(things, channel):
    """The hypothesis whose rgb_average is dominated by the given channel."""
    return max(things.values(), key=lambda h: h.get_feature("rgb_average")[channel])


def test_perception_detections_match_ground_truth(sim):
    from qr_perception.pipelines import make_sim_perception_pipeline

    reset_scene(sim)
    pipeline = make_sim_perception_pipeline()
    scene = pipeline.forward(observe(sim)["images"])
    detections = scene.get_ml_object_scene_representation().object_detections

    surfaces = [
        d for d in detections if d.get_feature("category", "thing") == "surface"
    ]
    things = [d for d in detections if d.get_feature("category", "thing") == "thing"]
    assert len(surfaces) == 1
    assert len(things) == 2

    # The detected support surface sits at the table-top height.
    table = surfaces[0].mesh.to_trimesh()
    assert table.bounds[1, 2] == pytest.approx(TABLE_TOP_Z, abs=0.02)

    # Each block detection is at its ground-truth pose with correct extents.
    for gt_pos, gt_size in ((RED_POS, 0.08), (BLUE_POS, 0.06)):
        mesh = min(
            (d.mesh.to_trimesh() for d in things),
            key=lambda m: np.linalg.norm(m.bounds.mean(axis=0) - gt_pos),
        )
        assert np.linalg.norm(mesh.bounds.mean(axis=0) - gt_pos) < CENTER_TOL
        assert np.abs(mesh.extents - gt_size).max() < EXTENT_TOL


def test_obm_matches_world_state(sim):
    reset_scene(sim)
    obm = make_obm()
    obm.observation_update(observe(sim))

    things, surfaces = things_and_surfaces(obm.memory)
    assert len(surfaces) == 1 and len(things) == 2

    world = sim.get_world_state()
    red = find_by_color(things, 0)
    blue = find_by_color(things, 2)
    assert np.linalg.norm(hyp_center(red) - world["block"][:3]) < CENTER_TOL
    assert np.linalg.norm(hyp_center(blue) - world["blue_block"][:3]) < CENTER_TOL
    assert np.abs(red.mesh_worldframe.to_trimesh().extents - 0.08).max() < EXTENT_TOL
    assert np.abs(blue.mesh_worldframe.to_trimesh().extents - 0.06).max() < EXTENT_TOL


def test_obm_reobservation_is_stable(sim):
    reset_scene(sim)
    obm = make_obm()
    obm.observation_update(observe(sim))
    n_objects = len(obm.memory.objects)
    confs = {k: h.existence_confidence for k, h in obm.memory.objects.items()}

    obm.observation_update(observe(sim))
    # Re-observing the same scene must not spawn duplicates, and matched
    # hypotheses become more confident.
    assert len(obm.memory.objects) == n_objects
    assert set(obm.memory.objects) == set(confs)
    things, _ = things_and_surfaces(obm.memory)
    for k, h in things.items():
        assert h.existence_confidence > confs[k]


def test_obm_tracks_small_motion(sim):
    reset_scene(sim)
    obm = make_obm()
    obm.observation_update(observe(sim))
    names_before = set(obm.memory.objects)

    # Nudge the red block by less than its size: the detection still overlaps
    # the hypothesis, so it must be matched and updated, not duplicated.
    new_pos = RED_POS + [0.03, 0.02, 0.0]
    sim.set_object_pose("block", new_pos, [1, 0, 0, 0])
    sim.step(100)
    obm.observation_update(observe(sim))

    assert set(obm.memory.objects) == names_before
    things, _ = things_and_surfaces(obm.memory)
    red = find_by_color(things, 0)
    world = sim.get_world_state()
    assert np.linalg.norm(hyp_center(red) - world["block"][:3]) < CENTER_TOL


def test_obm_teleported_object(sim):
    reset_scene(sim)
    obm = make_obm()
    obm.observation_update(observe(sim))
    things_before, _ = things_and_surfaces(obm.memory)
    blue_before = find_by_color(things_before, 2)
    conf_before = blue_before.existence_confidence

    # Teleport the blue block far (no overlap): the OBM creates a fresh
    # hypothesis at the new pose and loses confidence in the stale one
    # (it was unmatched but predicted visible).
    new_pos = BLUE_POS + [-0.25, 0.35, 0.0]
    sim.set_object_pose("blue_block", new_pos, [1, 0, 0, 0])
    sim.step(100)
    obm.observation_update(observe(sim))

    things, _ = things_and_surfaces(obm.memory)
    world = sim.get_world_state()
    dists = [
        np.linalg.norm(hyp_center(h) - world["blue_block"][:3]) for h in things.values()
    ]
    assert min(dists) < CENTER_TOL

    stale = obm.memory.objects.get(blue_before.name)
    if stale is not None:
        assert stale.existence_confidence < conf_before


# Base poses (x, y, theta) the robot teleports to between observations. The
# first is the fixture's original pose; the others step around the scene so
# the head sees the blocks from genuinely different angles.
VIEWPOINTS = [
    (0.0, 0.0, 0.0),
    (-0.10, 0.20, -0.20),
    (-0.10, -0.20, 0.20),
]


def _set_base(sim, x, y, theta):
    conf = sim.get_robot_conf()
    conf[:3] = [x, y, theta]
    sim.set_robot_conf(conf)
    sim.step(20)


def _observe_from(sim, viewpoints, head_target):
    """Teleport through each base pose, point the head at head_target, and
    return the per-viewpoint snapshot of OBM hypothesis confidences."""
    obm = make_obm()
    history = []
    for x, y, theta in viewpoints:
        _set_base(sim, x, y, theta)
        sim.point_head_at(head_target)
        obm.observation_update(observe(sim))
        history.append(
            {k: h.existence_confidence for k, h in obm.memory.objects.items()}
        )
    return obm, history


def test_obm_reobservation_from_multiple_viewpoints(sim):
    """Same scene viewed from several base poses: OBM keeps a single
    hypothesis per object and matched thing confidences don't decrease."""
    reset_scene(sim)
    obm, history = _observe_from(sim, VIEWPOINTS, [0.8, -0.1, 0.79])

    things, surfaces = things_and_surfaces(obm.memory)
    assert len(surfaces) == 1 and len(things) == 2

    keys0 = set(history[0])
    for confs in history[1:]:
        assert set(confs) == keys0
    for k in things:
        for prev, curr in zip(history, history[1:]):
            assert curr[k] >= prev[k]


# Extra red blocks for the 4-object scene. Both reuse block.xml — the sim
# attaches them under distinct body prefixes so the duplicates are safe.
RED_POS_3 = np.array([0.85, 0.20, 0.79])
RED_POS_4 = np.array([1.15, 0.10, 0.79])
MULTI_HEAD_TARGET = [0.90, 0.0, 0.79]


@pytest.fixture(scope="module")
def sim_multi(test_scene_dir):
    from qr_robots.mujoco.rby1.sim import make_sim

    sim = make_sim(
        test_scene_dir,
        {
            "table": {"file": "table.xml", "fixed": True},
            "block": {"file": "block.xml"},
            "blue_block": {"file": "blue_block.xml"},
            "block_3": {"file": "block.xml"},
            "block_4": {"file": "block.xml"},
        },
        mode=SIM_MODE,
    )
    sim.point_head_at(MULTI_HEAD_TARGET)
    yield from sim_fixture(sim, "perception obm (multi-object)")


def reset_multi_scene(sim):
    sim.set_object_pose("block", RED_POS, [1, 0, 0, 0])
    sim.set_object_pose("blue_block", BLUE_POS, [1, 0, 0, 0])
    sim.set_object_pose("block_3", RED_POS_3, [1, 0, 0, 0])
    sim.set_object_pose("block_4", RED_POS_4, [1, 0, 0, 0])
    sim.step(100)


def test_obm_reobservation_from_multiple_viewpoints_multi_object(sim_multi):
    """4-block scene observed from several base poses: still one hypothesis
    per object across viewpoints, no duplicates spawned."""
    reset_multi_scene(sim_multi)
    obm, history = _observe_from(sim_multi, VIEWPOINTS, MULTI_HEAD_TARGET)

    things, surfaces = things_and_surfaces(obm.memory)
    assert len(surfaces) == 1 and len(things) == 4

    keys0 = set(history[0])
    for confs in history[1:]:
        assert set(confs) == keys0
    for k in things:
        for prev, curr in zip(history, history[1:]):
            assert curr[k] >= prev[k]


@pytest.fixture(scope="module")
def drake_sim(test_scene_dir):
    from qr_robots.drake.rby1.sim import make_sim

    sim = make_sim(
        test_scene_dir,
        {
            "table": {"file": "table.urdf", "fixed": True, "pos": [0.9, 0, 0]},
            "block": {"file": "block.urdf", "pos": [0.65, 0, 0.81]},
        },
        mode=SIM_MODE,
    )
    sim.point_head_at([0.65, 0.0, 0.8])
    yield from sim_fixture(sim, "perception obm (drake)")


def test_perception_on_drake_images(drake_sim):
    """Same pipeline, Drake backend: the cross-backend image conventions
    (labels, extrinsics) feed perception identically."""
    from qr_perception.pipelines import make_sim_perception_pipeline

    obs = observe(drake_sim)
    obs["images"][0].depth_image = obs["images"][0].depth_image.astype(np.float64)
    detections = (
        make_sim_perception_pipeline()
        .forward(obs["images"])
        .get_ml_object_scene_representation()
        .object_detections
    )
    surfaces = [
        d for d in detections if d.get_feature("category", "thing") == "surface"
    ]
    things = [d for d in detections if d.get_feature("category", "thing") == "thing"]
    assert len(surfaces) == 1 and len(things) == 1

    world = drake_sim.get_world_state()
    mesh = things[0].mesh.to_trimesh()
    assert np.linalg.norm(mesh.bounds.mean(axis=0) - world["block"][:3]) < 0.02
    assert np.abs(mesh.extents[:2] - 0.08).max() < EXTENT_TOL


def test_obm_on_kinsim_images(mode="headless"):
    """Kinematic sim images (rendered via the MuJoCo bridge) feed the same
    pipeline; the OBM matches the kinematic world state."""
    from qr_kinsim.sim import KinematicSim

    sim = KinematicSim(
        "rby1",
        objects={
            "table": {
                "shape": {"box": [1.2, 0.8, 0.04]},
                "pos": [0.9, 0, 0.73],
                "fixed": True,
                "color": [0.6, 0.45, 0.3, 1.0],
            },
            "block": {
                "shape": {"box": [0.08, 0.08, 0.08]},
                "pos": [0.65, 0, 0.79],
                "color": [0.9, 0.2, 0.1, 1.0],
            },
        },
        mode=mode,
    )
    sim.execute_chain_trajectory("head", [[0.0, 0.7]])

    obm = make_obm()
    obm.observation_update(observe(sim))
    things, surfaces = things_and_surfaces(obm.memory)
    assert len(surfaces) == 1 and len(things) == 1

    world = sim.get_world_state()
    block = next(iter(things.values()))
    assert np.linalg.norm(hyp_center(block) - world["block"][:3]) < CENTER_TOL
    sim.close()
