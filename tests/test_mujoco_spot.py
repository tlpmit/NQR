import numpy as np
import pytest

from qr_robots.common import spot
from tests.conftest import SIM_MODE, sim_fixture


@pytest.fixture(scope="module")
def sim(test_scene_dir):
    from qr_robots.mujoco.spot.sim import make_sim

    # Shift the scene beyond the arm's reach: the outstretched gripper tip
    # sits at x ≈ 1.2, and at smaller offsets it touches the table-top edge
    # at startup.
    objects = {
        "table": {"file": "table.xml", "fixed": True, "pos": [1.1, 0, 0]},
        "block": {"file": "block.xml", "pos": [1.1, 0, 0]},
    }
    yield from sim_fixture(
        make_sim(test_scene_dir, objects, mode=SIM_MODE), "mujoco spot"
    )


def test_conf_layout(sim):
    conf = sim.get_robot_conf()
    assert len(conf) == 10
    parsed = spot.parse_robot_vector(conf)
    assert set(parsed) == {"base", "right", "right_gripper"}
    assert sorted(sim.chains) == ["base", "right_arm"]


def test_objects_rest_on_floor_and_table(sim):
    sim.step(300)
    state = sim.get_world_state()
    assert state["block"][2] == pytest.approx(0.79, abs=0.02)


def test_arm_trajectory(sim):
    target = np.array([0.0, -1.6, 1.2, 0.0, 0.3, 0.0])
    assert sim.execute_chain_trajectory("right_arm", [target], waypoint_dt=0.5)
    conf = sim.get_robot_conf()
    assert np.abs(conf[spot.chain_slices["right"]] - target).max() < 0.1


def test_base_trajectory(sim):
    # Drive away from the table (the scene is in front of the robot).
    assert sim.execute_base_trajectory([-0.3, 0.4, 0.5], waypoint_dt=1.0)
    base = sim.get_robot_conf()[:3]
    assert np.abs(base - [-0.3, 0.4, 0.5]).max() < 0.05


def test_gripper(sim):
    sim.execute_gripper_command("right", "open", settle_secs=0.5)
    assert sim.get_robot_conf()[9] < -1.2   # open ≈ -1.57
    sim.execute_gripper_command("right", "close", settle_secs=0.5)
    assert sim.get_robot_conf()[9] > -0.3   # closed ≈ 0


def test_camera(sim):
    rgb, depth, label = sim.get_camera_image("hand_camera")
    assert rgb.shape == (480, 640, 3)
    X = sim.get_camera_extrinsics("hand_camera")
    assert np.linalg.det(X[:3, :3]) == pytest.approx(1.0, abs=1e-6)
