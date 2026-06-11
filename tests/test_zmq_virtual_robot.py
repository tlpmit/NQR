"""End-to-end tests of the ZMQ virtual robots (server subprocess + client).

Uses the MuJoCo RBY1 backend and the kinematic Spot backend (both cheap to
start); the same ZmqVirtualMoman/SimClient path covers all other backends.
"""

import numpy as np
import pytest

from qr_api.control_typing import JointPositionCommand
from tests.conftest import KINSIM_OBJECTS, MJCF_OBJECTS, SIM_MODE, sim_fixture


@pytest.fixture(scope="module")
def rby1_robot(test_scene_dir):
    from qr_robots.mujoco.rby1.virtual_robot import Rby1MujocoVirtualMoman

    robot = Rby1MujocoVirtualMoman(
        objects=dict(MJCF_OBJECTS), model_dir=test_scene_dir, port=6601,
        mode=SIM_MODE,
    )
    yield from sim_fixture(robot, "zmq mujoco rby1")
    robot.shutdown()


@pytest.fixture(scope="module")
def spot_kinsim_robot(test_scene_dir):
    from qr_kinsim.virtual_robot import SpotKinsimVirtualMoman

    robot = SpotKinsimVirtualMoman(
        objects=dict(KINSIM_OBJECTS), model_dir=test_scene_dir, port=6602,
        mode=SIM_MODE,
    )
    yield from sim_fixture(robot, "zmq kinsim spot")
    robot.shutdown()


def test_sense(rby1_robot):
    obs = rby1_robot.sense()
    assert set(obs.fields) == {"robot_state", "world_state", "head_camera"}
    rs = obs.fields["robot_state"]
    assert set(rs.joint_positions) == {
        "base", "torso", "right", "right_gripper", "left", "left_gripper", "head"
    }
    im = obs.fields["head_camera"]
    assert im.rgb_image.shape == (720, 1280, 3)
    assert im.camera_extrinsics.shape == (4, 4)
    assert im.label_image is not None


def test_control_blocking_arm(rby1_robot):
    target = np.array([0.2, -0.3, 0.0, -1.0, 0.0, 0.5, 0.0])
    rby1_robot.arm_controllers["right"].control_blocking(
        [JointPositionCommand(duration=0.5, target_position=target)]
    )
    got = rby1_robot.get_robot_conf()["right"]
    assert np.abs(got - target).max() < 0.05


def test_high_level_control(rby1_robot):
    target = np.zeros(7)
    rby1_robot.control(
        [{"right": JointPositionCommand(duration=0.5, target_position=target)}]
    )
    assert np.abs(rby1_robot.get_robot_conf()["right"] - target).max() < 0.05


def test_base_and_gripper(rby1_robot):
    rby1_robot.base_controller.control_blocking(
        [JointPositionCommand(duration=1.0, target_position=np.array([0.2, 0.1, 0.1]))]
    )
    assert np.abs(rby1_robot.get_robot_conf()["base"] - [0.2, 0.1, 0.1]).max() < 0.02
    rby1_robot.gripper_controllers["right_gripper"].control_blocking("open", 10.0)
    assert rby1_robot.get_robot_conf()["right_gripper"][0] > 0.05


def test_world_state(rby1_robot):
    state = rby1_robot.get_world_state()
    assert set(state) == {"table", "block", "conf"}


def test_kinsim_spot_over_zmq(spot_kinsim_robot):
    robot = spot_kinsim_robot
    conf = robot.get_robot_conf()
    assert set(conf) == {"base", "right", "right_gripper"}

    target = np.array([0.0, -1.6, 1.2, 0.0, 0.3, 0.0])
    robot.arm_controllers["right"].control_blocking(
        [JointPositionCommand(duration=0.5, target_position=target)]
    )
    assert np.abs(robot.get_robot_conf()["right"] - target).max() < 1e-6

    # Kinematic extras exposed through the client.
    assert isinstance(robot.client.check_collisions(), list)
    state = robot.get_world_state()
    assert set(state) == {"table", "block", "conf"}
