"""
Plug-compatibility tests for the NQR + HPN combined system.

These exercise the full assembly path of qr_run.run — QDDL world → HPN
Roboverse scene → NQR virtual robot → NQR belief → HPN policy module — up to
policy construction. Full planning runs are exercised via the scripts in
/tmp (see qr_run.run.run_QR_main); they are too slow and HPN-state-dependent
for the default test suite.
"""

import numpy as np
import pytest

pytest.importorskip("RetroPlan", reason="HPN sibling checkout not installed")


@pytest.fixture(scope="module")
def problem():
    from qr_problem_defs.misc_problems import ruby_pick_place_move_base

    ruby_pick_place_move_base.virtual_robot = "Ruby_Kinsim"
    return ruby_pick_place_move_base


def test_qddl_world_to_scene(problem):
    import tempfile

    from qr_run.virtual_robots import build_qddl_world, write_scene_objects

    world = build_qddl_world(problem, problem.robot)
    bodies = sorted(world.phys.get_bodies())
    # problem_green_on_book: a table with a book and two colored spam cans
    assert "table" in bodies and "book" in bodies
    assert "shpam1" in bodies and "shpam2" in bodies

    objects = write_scene_objects(world.phys, tempfile.mkdtemp(prefix="qr_scene_"))
    assert any(name.startswith("book") for name in objects)
    assert any(name.startswith("table") for name in objects)


def test_combined_construction(problem):
    """Virtual robot + belief + HPN policy all construct and interoperate."""
    from qr_main.main_virtual_robot import QRMainVR
    from qr_run.run import (
        construct_belief_module,
        construct_policy_module,
        get_virtual_robot,
        make_belief_meshcats,
    )
    from qr_run.standard_configs import adjust_config_for_robot, make_sim_config

    robot = get_virtual_robot(problem.virtual_robot, problem.robot, problem=problem)
    try:
        cfg = adjust_config_for_robot(
            problem, make_sim_config(interactive=False, debug_level=0)
        )
        qr_main = QRMainVR(robot, cfg)
        m, a0, a1 = make_belief_meshcats(problem.partially_observed)
        belief = construct_belief_module(problem, cfg, qr_main, m, (a0, a1))
        policy = construct_policy_module(problem, cfg, belief, robot, m)

        # The robot starts at the planner's expected conf (HPN world conf).
        sk_conf = policy.planning_bel.phys.get_conf().value
        v_conf = robot.get_robot_conf()
        assert np.abs(np.asarray(v_conf["torso"]) - np.asarray(sk_conf["torso"])).max() < 0.05
        assert np.abs(np.asarray(v_conf["base"]) - np.asarray(sk_conf["base"])).max() < 0.05

        # The policy can pull a full observation through the NQR robot.
        obs = policy.get_observation_reset()
        assert obs["images"] == []
        assert "base" in obs["conf"].value
        obs2 = policy._get_observation(include_images=True)
        assert len(obs2["images"]) == 1
        assert obs2["images"][0].rgb_image.shape == (720, 1280, 3)

        # The belief consumes that observation end to end.
        belief.reset(obs)
        assert belief.spatial_memory.vv.vv_regions is not None
    finally:
        robot.shutdown()
