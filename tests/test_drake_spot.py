import numpy as np
import pytest

from qr_robots.common import spot
from tests.conftest import SIM_MODE, URDF_OBJECTS, sim_fixture


@pytest.fixture(scope="module")
def sim(test_scene_dir):
    from qr_robots.drake.spot.sim import make_sim

    objects = dict(URDF_OBJECTS)
    # Move the scene out of Spot's footprint (its body is long).
    objects["table"] = {**objects["table"], "pos": [1.4, 0, 0]}
    objects["block"] = {**objects["block"], "pos": [1.15, 0, 0.81]}
    sim = make_sim(test_scene_dir, objects, mode=SIM_MODE)
    url = sim.meshcat.web_url() if sim.meshcat else None
    yield from sim_fixture(sim, "drake spot", url)


def test_conf_layout(sim):
    conf = sim.get_robot_conf()
    assert len(conf) == 10
    parsed = spot.parse_robot_vector(conf)
    assert set(parsed) == {"base", "right", "right_gripper"}


def test_arm_trajectory(sim):
    target = np.array([0.0, -1.6, 1.2, 0.0, 0.3, 0.0])
    assert sim.execute_chain_trajectory("right_arm", [target])
    conf = sim.get_robot_conf()
    assert np.abs(conf[spot.chain_slices["right"]] - target).max() < 0.1


def test_base_trajectory(sim):
    assert sim.execute_base_trajectory([0.3, 0.2, 0.3])
    base = sim.get_robot_conf()[:3]
    assert np.abs(base - [0.3, 0.2, 0.3]).max() < 0.05


def test_gripper(sim):
    sim.execute_gripper_command("right", "open", settle_secs=1.0)
    assert sim.get_robot_conf()[9] < -1.2
    sim.execute_gripper_command("right", "close", settle_secs=1.0)
    assert sim.get_robot_conf()[9] > -0.3


def test_cameras(sim):
    rgb, depth, label = sim.get_camera_image("hand_camera")
    assert rgb.shape[2] == 3
    X = sim.get_camera_extrinsics("hand_camera")
    assert np.linalg.det(X[:3, :3]) == pytest.approx(1.0, abs=1e-6)
