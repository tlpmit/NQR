import numpy as np
import pytest

from qr_robots.common import rby1
from tests.conftest import SIM_MODE, URDF_OBJECTS, sim_fixture


@pytest.fixture(scope="module")
def sim(test_scene_dir):
    from qr_robots.drake.rby1.sim import make_sim

    sim = make_sim(test_scene_dir, dict(URDF_OBJECTS), mode=SIM_MODE)
    url = sim.meshcat.web_url() if sim.meshcat else None
    yield from sim_fixture(sim, "drake rby1", url)


def test_conf_layout(sim):
    conf = sim.get_robot_conf()
    assert len(conf) == 31
    parsed = rby1.parse_robot_vector(conf)
    assert len(parsed["right"]) == 7 and len(parsed["torso"]) == 6


def test_world_state(sim):
    state = sim.get_world_state()
    assert set(state) == {"table", "block", "robot"}
    assert state["block"][2] == pytest.approx(0.81, abs=0.05)


def test_chain_trajectory(sim):
    target = np.array([0.2, -0.3, 0.0, -1.0, 0.0, 0.5, 0.0])
    assert sim.execute_chain_trajectory("right_arm", [target])
    conf = sim.get_robot_conf()
    assert np.abs(conf[rby1.chain_slices["right"]] - target).max() < 0.05


def test_base_trajectory(sim):
    # Drive away from the table (the base is force-controlled, so driving
    # into furniture genuinely blocks it).
    assert sim.execute_base_trajectory([-0.2, 0.3, 0.5])
    base = sim.get_robot_conf()[:3]
    assert np.abs(base - [-0.2, 0.3, 0.5]).max() < 0.03


def test_gripper(sim):
    sim.execute_gripper_command("right", "open", settle_secs=0.5)
    opening = rby1.parse_robot_vector(sim.get_robot_conf())["right_gripper"][0]
    assert opening > 0.05


def test_camera(sim):
    rgb, depth, label = sim.get_camera_image("head_camera")
    assert rgb.shape[2] == 3
    K = sim.get_camera_intrinsics("head_camera")
    assert K[0, 0] > 0
    X = sim.get_camera_extrinsics("head_camera")
    assert np.linalg.det(X[:3, :3]) == pytest.approx(1.0, abs=1e-6)


def test_point_head_at(sim):
    assert sim.point_head_at([0.65, 0.0, 0.8])
