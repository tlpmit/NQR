import numpy as np
import pytest

from qr_robots.common import rby1
from tests.conftest import MJCF_OBJECTS, SIM_MODE, sim_fixture


@pytest.fixture(scope="module")
def sim(test_scene_dir):
    from qr_robots.mujoco.rby1.sim import make_sim

    yield from sim_fixture(
        make_sim(test_scene_dir, dict(MJCF_OBJECTS), mode=SIM_MODE),
        "mujoco rby1",
    )


def test_conf_layout(sim):
    conf = sim.get_robot_conf()
    assert len(conf) == 31
    parsed = rby1.parse_robot_vector(conf)
    assert set(parsed) == {
        "base", "torso", "right", "right_gripper", "left", "left_gripper", "head"
    }
    assert len(parsed["right"]) == 7
    assert sorted(sim.chains) == ["base", "head", "left_arm", "right_arm", "torso"]


def test_world_state(sim):
    state = sim.get_world_state()
    assert set(state) == {"table", "block", "robot"}
    # block.xml places the cube on the table top
    assert state["block"][2] == pytest.approx(0.79, abs=0.02)


def test_chain_trajectory(sim):
    target = np.array([0.2, -0.3, 0.0, -1.0, 0.0, 0.5, 0.0])
    assert sim.execute_chain_trajectory("right_arm", [target], waypoint_dt=0.5)
    conf = sim.get_robot_conf()
    assert np.abs(conf[rby1.chain_slices["right"]] - target).max() < 0.05


def test_base_trajectory(sim):
    assert sim.execute_base_trajectory([0.3, 0.1, 0.2], waypoint_dt=0.5)
    base = sim.get_robot_conf()[:3]
    assert np.abs(base - [0.3, 0.1, 0.2]).max() < 0.02


def test_gripper(sim):
    sim.execute_gripper_command("right", "open", settle_secs=0.3)
    opening = rby1.parse_robot_vector(sim.get_robot_conf())["right_gripper"][0]
    assert opening > 0.05
    sim.execute_gripper_command("right", "close", settle_secs=0.3)
    closed = rby1.parse_robot_vector(sim.get_robot_conf())["right_gripper"][0]
    assert closed < opening


def test_camera(sim):
    rgb, depth, label = sim.get_camera_image("head_camera")
    assert rgb.shape == (720, 1280, 3) and rgb.dtype == np.uint8
    assert depth.shape == (720, 1280)
    assert label.shape == (720, 1280)
    K = sim.get_camera_intrinsics("head_camera")
    assert K.shape == (3, 3) and K[0, 0] > 0
    X = sim.get_camera_extrinsics("head_camera")
    assert X.shape == (4, 4)
    assert np.linalg.det(X[:3, :3]) == pytest.approx(1.0, abs=1e-6)


def test_point_head_at(sim):
    assert sim.point_head_at([0.65, 0.0, 0.79])


def test_set_robot_conf(sim):
    conf = sim.get_robot_conf().copy()
    conf[:3] = [1.0, -0.5, 0.7]
    sim.set_robot_conf(conf)
    assert np.abs(sim.get_robot_conf()[:3] - [1.0, -0.5, 0.7]).max() < 1e-6
