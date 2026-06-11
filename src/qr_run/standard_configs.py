"""
Standard run configurations for the combined NQR + HPN system.

The HPN configuration-variable overrides and QRSystemConfig settings are
taken from the old repo's working setup in
QR/src/qr_examples/main_lpk_sandbox/main_tlp.py — run problems with these,
not with HPN's defaults (notably `cogman_config.use_sticky_next_step=False`
and the tamp_config MMP/vamp settings; behavior differs wildly without them).
"""

import copy
import math

from qr_problem_defs.problem_def import QRProblem
from qr_run.qr_config import (
    HPNConfig,
    PerceptionConfig,
    QRSystemConfig,
    SpatialMemConfig,
)

spot_overrides = [
    ("robo_config.approachPerpBackoff", 0.025),
]
standard_overrides = [
    ("cogman_config.bind_hierarchical_vars", True),
    ("cogman_config.bind_preimage_vars", False),
    ("cogman_config.handle_HPN_exception_and_retry", True),
    ("cogman_config.max_repeat_executions", 5),
    ("cogman_config.reuse_bindings_during_execution", False),
    ("cogman_config.use_sticky_next_step", False),
    ("csp_config.MAX_DROP_GROUP_SIZE", 2),  # usually 2
    ("csp_config.PICKLE_COSTMAP", False),
    ("plan_config.heuristic_depth_limit", 4),  # was 10
    ("plan_config.max_search_nodes", 30),  # trying this out
    ("plan_config.return_first", 5),
    ("plan_config.try_monotonic", True),
    ("plan_config.use_heuristic", False),  # ('plan_config.use_heuristic', "'H_unmet'"),
    ("robo_config.pause_on_arm_path", False),
    ("robo_config.trajopt.kot.max_control_points", 100),
    ("robo_config.trajopt.kot.num_collision_checks", 50),
    ("tamp_config.always_rehearse", True),
    ("tamp_config.belief_update_from_observation", True),
    ("tamp_config.MAX_POSE_DIST", 0.1),
    ("tamp_config.MAX_POSE_ROT", math.pi / 10),
    ("tamp_config.never_plan_paths", False),
    ("tamp_config.optimistically_ignore_shadows_when_conditioning", True),
    ("tamp_config.use_MMP_for_constance_adjust", True),
    ("tamp_config.use_MMP_for_constance_cp", True),
    ("tamp_config.use_MMP_for_constance_gc", True),
    ("tamp_config.use_MMP_for_graspable", False),
    ("tamp_config.use_MMP_for_grasps", True),
    ("tamp_config.use_MMP_for_vis_collisions", True),
]

in_play_overrides = [
    ("csp_config.PICKLE_NODES", False),
    ("tamp_config.dense_grasps", 1),
    ("tamp_config.MAX_POSES", 5),
    ("tamp_config.MAX_SOFT", 1),
    ("tamp_config.n_smoothing_shortcuts", 250),
    ("tamp_config.n_smoothing_steps", 5),
    ("tamp_config.use_costmap", False),
    ("tamp_config.use_grid_path", True),
    ("tamp_config.use_MMP_for_RRT", True),
    ("tamp_config.always_adjust_poses", False),
    ("tamp_config.use_MMP", 12),  # False is different from 1, don't use 1.
    ("tamp_config.use_safe_pick", False),
    ("tamp_config.voxel_grid_resolution", 0.05),
    ("tamp_config.MAX_COLLISION_DISTANCE_THR", 0.005),
    ("robo_config.rainbow_vamp_collisions", True),
    ("tamp_config.use_vamp_collision_check", ["rainbow", "pandadroid"]),
    ("tamp_config.use_vamp_cspace_path", []),
    ("tamp_config.verify_vamp_collisions", False),
    ("tamp_config.grasp_min_score", -0.5),
    ("tamp_config.finger_overlap_ball_radius", 0.015),
]

overrides = standard_overrides + in_play_overrides

# These defaults are reset for different robots in adjust_config_for_robot
grid_resolution = 0.05
min_z_for_planes = 0.05


def make_sim_config(
    interactive: bool = True,
    debug_level: int = 1,
    write_to_pkl_path=None,
    run_from_pkl_path=None,
) -> QRSystemConfig:
    """The sim_config from main_tlp.py (display flags off; the simulators
    have their own viewers)."""
    return QRSystemConfig(
        terminal_tags=["obm"],
        pause_tags=["execution_fail"] if interactive else (),
        segmentation_method="sim",
        completion_method="projection",
        use_eye_extrusion=False,
        eye_extrusion_distance=0.02,
        policy_module="hpn_btamp",
        hpn_params=HPNConfig(
            run_trajopt=False,
            overrides=list(overrides),
            debug_level=debug_level,
            interactive=interactive,
        ),
        spatial_mem_params=SpatialMemConfig(voxel_grid_resolution=grid_resolution),
        perception_params=PerceptionConfig(min_z_for_planes=min_z_for_planes),
        display_perception=False,
        display_large_objects=False,
        display_obm_state=False,
        write_to_pkl_path=write_to_pkl_path,
        run_from_pkl_path=run_from_pkl_path,
        display_raw_images=False,
        nan_removal_radius=2,
        min_depth_threshold=0.3,  # may want to make this different per robot
        max_depth_threshold=3.0,
    )


def adjust_config_for_robot(
    problem: QRProblem, default_qr_config: QRSystemConfig
) -> QRSystemConfig:
    """Per-robot configuration tweaks (the run_QR wrapper in main_tlp.py)."""
    config = copy.deepcopy(default_qr_config)
    ovr = default_qr_config.hpn_params.overrides.copy()
    if problem.robot in ("panda", "panda_droid"):
        if ("tamp_config.use_costmap", True) in ovr:
            ovr.append(("tamp_config.use_costmap", False))
            ovr.remove(("tamp_config.use_costmap", True))
            print("Disabling costmap for panda robot")
        config.perception_params.min_z_for_planes = -0.1
        import numpy as np

        mask = np.zeros((720, 1280), dtype=bool)
        mask[500:, :] = True  # remove bottom part where hand is
        config.invalid_image_region_mask = mask
        config.spatial_mem_params.voxel_grid_resolution = 0.025
    else:
        config.perception_params.min_z_for_planes = 0.05
        config.spatial_mem_params.voxel_grid_resolution = 0.05

    if problem.robot in ("spot", "panda", "panda_droid"):
        # Robots with only sensor in the only hand
        ovr.append(("tamp_config.use_move_without_sensing", True))
    else:
        ovr.append(("tamp_config.use_move_without_sensing", False))
    config.hpn_params.overrides = ovr
    return config
