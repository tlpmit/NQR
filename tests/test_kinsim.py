import numpy as np
import pytest

from qr_robots.common import rby1, spot
from tests.conftest import KINSIM_OBJECTS, SIM_MODE, sim_fixture


@pytest.fixture()
def rby1_sim():
    from qr_kinsim.sim import KinematicSim

    sim = KinematicSim("rby1", objects=dict(KINSIM_OBJECTS), mode=SIM_MODE)
    yield from sim_fixture(sim, "kinsim rby1", sim.world.meshcat_url)


@pytest.fixture()
def spot_sim():
    from qr_kinsim.sim import KinematicSim

    sim = KinematicSim("spot", objects=dict(KINSIM_OBJECTS), mode=SIM_MODE)
    yield from sim_fixture(sim, "kinsim spot", sim.world.meshcat_url)


def test_rby1_conf_roundtrip(rby1_sim):
    conf = rby1_sim.get_robot_conf()
    assert len(conf) == 31
    conf[rby1.chain_slices["right"]] = [0.2, -0.3, 0.0, -1.0, 0.0, 0.5, 0.0]
    conf[:3] = [0.4, 0.2, 0.6]
    rby1_sim.set_robot_conf(conf)
    out = rby1_sim.get_robot_conf()
    assert np.abs(out[:3] - [0.4, 0.2, 0.6]).max() < 1e-9
    assert np.abs(out[rby1.chain_slices["right"]] - conf[rby1.chain_slices["right"]]).max() < 1e-9


def test_rby1_trajectories_are_exact(rby1_sim):
    target = np.array([0.2, -0.3, 0.0, -1.0, 0.0, 0.5, 0.0])
    rby1_sim.execute_chain_trajectory("right_arm", [target])
    assert np.abs(
        rby1_sim.get_robot_conf()[rby1.chain_slices["right"]] - target
    ).max() < 1e-9
    rby1_sim.execute_base_trajectory([0.3, 0.1, 0.2])
    assert np.abs(rby1_sim.get_robot_conf()[:3] - [0.3, 0.1, 0.2]).max() < 1e-9


def test_collision_semantics(rby1_sim):
    # Resting contact (block exactly on table) is not a collision.
    assert rby1_sim.check_collisions() == []
    # Penetration is.
    rby1_sim.set_object_pose("block", [0.65, 0, 0.73])
    assert ("table", "block") in rby1_sim.check_collisions()
    # Robot driven into the table collides with it.
    rby1_sim.set_object_pose("block", [0.65, 0, 0.79])
    rby1_sim.execute_base_trajectory([0.9, 0.0, 0.0])
    cols = rby1_sim.check_collisions()
    assert any("table" in pair for pair in cols)


def test_grasp_attach_detach(rby1_sim):
    # Put the block into the right gripper, close to grasp it.
    ee = rby1_sim.get_frame_pose("ee_right")
    rby1_sim.set_object_pose("block", ee[:3, 3].tolist())
    rby1_sim.execute_gripper_command("right", "close")
    assert rby1_sim.world.attached_objects() == ["block"]

    # The attached object follows the robot.
    before = rby1_sim.get_world_state()["block"][:3]
    rby1_sim.execute_base_trajectory([0.5, 0.2, 0.0])
    after = rby1_sim.get_world_state()["block"][:3]
    assert np.abs(after - before).max() > 0.1

    # Opening releases it at its current pose.
    rby1_sim.execute_gripper_command("right", "open")
    assert rby1_sim.world.attached_objects() == []
    released = rby1_sim.get_world_state()["block"][:3]
    assert np.abs(released - after).max() < 1e-9


def test_no_grasp_when_out_of_reach(rby1_sim):
    rby1_sim.execute_gripper_command("right", "close")
    assert rby1_sim.world.attached_objects() == []


def test_spot_conf_and_motion(spot_sim):
    conf = spot_sim.get_robot_conf()
    assert len(conf) == 10
    target = np.array([0.0, -1.6, 1.2, 0.0, 0.3, 0.0])
    spot_sim.execute_chain_trajectory("right_arm", [target])
    assert np.abs(
        spot_sim.get_robot_conf()[spot.chain_slices["right"]] - target
    ).max() < 1e-9
    spot_sim.execute_base_trajectory([0.4, 0.2, 0.3])
    assert np.abs(spot_sim.get_robot_conf()[:3] - [0.4, 0.2, 0.3]).max() < 1e-9


def test_world_state_format(rby1_sim):
    state = rby1_sim.get_world_state()
    assert set(state) == {"table", "block", "robot"}
    assert len(state["block"]) == 7   # x y z qw qx qy qz
    assert state["block"][3:] == pytest.approx([1, 0, 0, 0], abs=1e-9)


def test_camera_render_bridge(rby1_sim):
    """Kinsim cameras come from the MuJoCo render bridge and match the
    dynamics backends' formats."""
    rgb, depth, label = rby1_sim.get_camera_image("head_camera")
    assert rgb.shape == (720, 1280, 3) and rgb.dtype == np.uint8
    assert depth.shape == (720, 1280)
    assert label.shape == (720, 1280)
    K = rby1_sim.get_camera_intrinsics("head_camera")
    assert K[0, 0] > 0
    X = rby1_sim.get_camera_extrinsics("head_camera")
    assert np.linalg.det(X[:3, :3]) == pytest.approx(1.0, abs=1e-6)

    # The render tracks the kinematic state: tilt the head down so the red
    # block is in view, then move the block away and re-render.
    rby1_sim.execute_chain_trajectory("head", [[0.0, 0.6]])
    rgb1, _, _ = rby1_sim.get_camera_image("head_camera")
    red1 = int(((rgb1[:, :, 0] > 150) & (rgb1[:, :, 1] < 80)).sum())
    rby1_sim.set_object_pose("block", [0.65, 0, 2.5])
    rgb2, _, _ = rby1_sim.get_camera_image("head_camera")
    red2 = int(((rgb2[:, :, 0] > 150) & (rgb2[:, :, 1] < 80)).sum())
    assert red1 > 100 and red2 < red1 / 10
    rby1_sim.close()
