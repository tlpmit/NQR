"""
Top-level assembly: virtual robot + belief + policy + main loop.

Close port of the old repo's qr_run/run.py, with NQR's virtual robots and the
ported perception/belief constructions. HPN (planner, TAMP domains,
Roboverse world model) is consumed as an external dependency.
"""

from qr_api.belief_interfaces import BeliefModule
from qr_api.policy_interfaces import PolicyModule
from qr_api.virtual_robot_interfaces import VirtualRobotModule
from qr_main.main_virtual_robot import QRMainVR
from qr_problem_defs.problem_def import QRProblem
from qr_run.qr_config import QRSystemConfig
from qr_run.virtual_robots import get_virtual_robot
from qr_utils.cog_utils import get_static_info


def construct_belief_module(
    problem: QRProblem,
    qr_configuration: QRSystemConfig,
    qr_main: QRMainVR,
    drake_meshcat,
    drake_meshcat_aux,
) -> BeliefModule:
    from qr_belief.dummy_belief_interfaces import (
        DummyAgentMemoryModule,
        DummyBeliefModule,
    )
    from qr_belief.goal.simple_goal_belief_module import SimpleGoalMemoryModule
    from qr_belief.obm.obm_components import (
        BasicBeliefInit,
        BasicMatchScoreFunction,
        BasicMergeFunction,
        BasicObjectTransitionUpdate,
    )
    from qr_belief.obm.obm_module import ObjectBasedDataAssociationFilter
    from qr_belief.robot.simple_robot_belief_module import SimpleRobotMemoryModule
    from qr_belief.spatial.simple_spatial_module import OldHPNSpatialMemoryModule
    from qr_perception.modules.featurizers import (
        ImageCurtailedFeaturizer,
        RGBAverageObjectFeaturizer,
        RGBImageFeaturizer,
    )
    from qr_perception.modules.preprocessors import RGBDDepthFilter
    from qr_perception.modules.table_finder import SurfaceFinder
    from qr_perception.modules.vlm_designation import NullDesignationPerceptionFunction
    from qr_perception.object_based_perception import PercPipelineUncertainSegComplete
    from qr_utils.colors import get_color_similarity

    static_domain_info = get_static_info((problem.domain, problem.belief), problem.goal)

    if not problem.partially_observed:
        return DummyBeliefModule()

    if qr_configuration.segmentation_method == "sim":
        from qr_perception.modules.segmentation_sim import SimSegmenter

        segmentation_module = SimSegmenter()
    else:
        raise ValueError(
            f"Unknown segmentation method: {qr_configuration.segmentation_method}"
        )

    if qr_configuration.completion_method == "projection":
        from qr_perception.modules.simple_completion import ProjectionCompletion

        completion_module = ProjectionCompletion(
            use_eye_extrusion=qr_configuration.use_eye_extrusion,
            eye_extrusion_distance=qr_configuration.eye_extrusion_distance,
            depth_method=qr_configuration.depth_method,
            display=qr_configuration.display_perception,
        )
    else:
        raise ValueError(
            f"Unknown completion method: {qr_configuration.completion_method}"
        )

    object_perc_pipeline = PercPipelineUncertainSegComplete(
        image_preprocessor=RGBDDepthFilter(
            max_depth_threshold=qr_configuration.max_depth_threshold,
            min_depth_threshold=qr_configuration.min_depth_threshold,
            nan_removal_radius=qr_configuration.nan_removal_radius,
            invalid_image_region_mask=qr_configuration.invalid_image_region_mask,
        ),
        segmentation_method=segmentation_module,
        completion_method=completion_module,
        large_object_detection_method=SurfaceFinder(
            min_z_for_planes=qr_configuration.perception_params.min_z_for_planes
        ),
        image_featurizers={"rgb": RGBImageFeaturizer()},
        object_featurizers={
            "rgb_average": RGBAverageObjectFeaturizer(),
            "curtailed": ImageCurtailedFeaturizer(),
        },
        display=qr_configuration.display_perception,
        display_large_objects=qr_configuration.display_large_objects,
        display_small_objects=qr_configuration.display_small_objects,
        display_raw_images=qr_configuration.display_raw_images,
    )

    goal_memory_module = SimpleGoalMemoryModule(problem.goal)
    obm = ObjectBasedDataAssociationFilter(
        goal_memory_module,
        object_perc_pipeline,
        merge_function=BasicMergeFunction(BasicBeliefInit()),
        match_score_function=BasicMatchScoreFunction(
            attribute_similarity_funs={
                "rgb_average": get_color_similarity,
            },
        ),
        new_belief_from_detection=BasicBeliefInit(),
        object_transition_update_function=BasicObjectTransitionUpdate(),
        goal_driven_perception_function=NullDesignationPerceptionFunction(),
        display=qr_configuration.display_obm_state,
    )

    belief_module = BeliefModule(
        qr_main.sensor_modules,
        OldHPNSpatialMemoryModule(
            static_domain_info,
            drake_meshcat,
            drake_meshcat_aux,
            qr_configuration.spatial_mem_params.voxel_grid_resolution,
        ),
        obm,
        DummyAgentMemoryModule(None),
        goal_memory_module,
        SimpleRobotMemoryModule(),
    )
    return belief_module


def construct_policy_module(
    problem: QRProblem,
    qr_configuration: QRSystemConfig,
    belief_module: BeliefModule,
    virtual_robot: VirtualRobotModule,
    drake_meshcat,
) -> PolicyModule:
    if qr_configuration.policy_module == "hpn_btamp":
        from qr_policy.hpn_policy_module import HPNVirtualRobotPolicyModule

        static_domain_info = get_static_info(
            (problem.domain, problem.belief), problem.goal
        )
        policy_module = HPNVirtualRobotPolicyModule(
            belief_module,
            virtual_robot,
            static_domain_info,
            drake_meshcat=drake_meshcat,
            partially_observed=problem.partially_observed,
            run_trajopt=qr_configuration.hpn_params.run_trajopt,
            robot_name=problem.robot,
            qddl_paths=[problem.domain, problem.belief]
            + (problem.assets if problem.assets else []),
            overrides=qr_configuration.hpn_params.overrides,
            debug_tags=qr_configuration.hpn_params.debug_tags,
            debug_level=qr_configuration.hpn_params.debug_level,
            log_level=qr_configuration.hpn_params.log_level,
            interactive=qr_configuration.hpn_params.interactive,
        )
    else:
        raise ValueError(f"Unknown policy module: {qr_configuration.policy_module}")

    return policy_module


def make_belief_meshcats(partially_observed: bool):
    """The Roboverse 'Belief' viewers the spatial memory publishes to."""
    if not partially_observed:
        return None, None, None
    import Roboverse.configuration as robo_config
    from Domains.sim.roboverse_sim import start_display

    start_display(["Belief"], 1)
    drake_meshcat = robo_config.VIEWERS.get("Belief").vis
    drake_meshcat.Delete("/voxel_grid")
    drake_meshcat.SetProperty("/voxel_grid/pointcloud", "visible", False)
    start_display(["Belief_aux0"], 1)
    drake_meshcat_aux0 = robo_config.VIEWERS.get("Belief_aux0").vis
    drake_meshcat_aux0.SetProperty("/voxel_grid/pointcloud", "visible", False)
    start_display(["Belief_aux1"], 1)
    drake_meshcat_aux1 = robo_config.VIEWERS.get("Belief_aux1").vis
    drake_meshcat_aux1.SetProperty("/voxel_grid/pointcloud", "visible", False)
    return drake_meshcat, drake_meshcat_aux0, drake_meshcat_aux1


def run_QR_main(problem: QRProblem, qr_configuration: QRSystemConfig):
    virtual_robot = get_virtual_robot(
        problem.virtual_robot,
        problem.robot,
        problem=problem,
        run_from_pkl_path=qr_configuration.run_from_pkl_path,
    )
    qr_main = QRMainVR(virtual_robot, qr_configuration)

    drake_meshcat, drake_meshcat_aux0, drake_meshcat_aux1 = make_belief_meshcats(
        problem.partially_observed
    )

    belief_module = construct_belief_module(
        problem,
        qr_configuration,
        qr_main,
        drake_meshcat,
        (drake_meshcat_aux0, drake_meshcat_aux1),
    )
    policy_module = construct_policy_module(
        problem, qr_configuration, belief_module, virtual_robot, drake_meshcat
    )

    qr_main.set_belief_module(belief_module)
    qr_main.set_policy_module(policy_module)

    qr_main.main()
