"""
Standard perception-pipeline / OBM constructions.

`make_sim_perception_pipeline` and `make_sim_obm` mirror the wiring of the
old repo's qr_run.run.construct_belief_module for the simulated-perception
configuration: label-image segmentation (SimSegmenter), projection-based
shape completion, plane finding for support surfaces, and RGB featurizers,
feeding the object-based data-association filter (OBM).
"""

from __future__ import annotations

from qr_belief.goal.simple_goal_belief_module import SimpleGoalMemoryModule
from qr_belief.obm.obm_components import (
    BasicBeliefInit,
    BasicMatchScoreFunction,
    BasicMergeFunction,
    BasicObjectTransitionUpdate,
)
from qr_belief.obm.obm_module import ObjectBasedDataAssociationFilter
from qr_perception.modules.featurizers import (
    ImageCurtailedFeaturizer,
    RGBAverageObjectFeaturizer,
    RGBImageFeaturizer,
)
from qr_perception.modules.preprocessors import RGBDDepthFilter
from qr_perception.modules.segmentation_sim import SimSegmenter
from qr_perception.modules.simple_completion import ProjectionCompletion
from qr_perception.modules.table_finder import SurfaceFinder
from qr_perception.modules.vlm_designation import NullDesignationPerceptionFunction
from qr_perception.object_based_perception import PercPipelineUncertainSegComplete
from qr_utils.colors import get_color_similarity


def make_sim_perception_pipeline(
    max_depth_threshold: float | None = None,
    min_depth_threshold: float | None = None,
    nan_removal_radius: int = 2,
    min_z_for_planes: float = 0.025,
    use_eye_extrusion: bool = False,
    display: bool = False,
) -> PercPipelineUncertainSegComplete:
    return PercPipelineUncertainSegComplete(
        image_preprocessor=RGBDDepthFilter(
            max_depth_threshold=max_depth_threshold,
            min_depth_threshold=min_depth_threshold,
            nan_removal_radius=nan_removal_radius,
        ),
        segmentation_method=SimSegmenter(),
        completion_method=ProjectionCompletion(
            use_eye_extrusion=use_eye_extrusion,
            display=display,
        ),
        large_object_detection_method=SurfaceFinder(
            min_z_for_planes=min_z_for_planes
        ),
        image_featurizers={"rgb": RGBImageFeaturizer()},
        object_featurizers={
            "rgb_average": RGBAverageObjectFeaturizer(),
            "curtailed": ImageCurtailedFeaturizer(),
        },
        display=display,
        display_large_objects=False,
        display_small_objects=False,
        display_raw_images=False,
    )


def make_sim_obm(
    goal: str = "",
    perception_pipeline: PercPipelineUncertainSegComplete | None = None,
    display: bool = False,
    **pipeline_kwargs,
) -> ObjectBasedDataAssociationFilter:
    """OBM wired for simulated perception (no VLM)."""
    if perception_pipeline is None:
        perception_pipeline = make_sim_perception_pipeline(**pipeline_kwargs)
    goal_memory_module = SimpleGoalMemoryModule(goal)
    return ObjectBasedDataAssociationFilter(
        goal_memory_module,
        perception_pipeline,
        merge_function=BasicMergeFunction(BasicBeliefInit()),
        match_score_function=BasicMatchScoreFunction(
            attribute_similarity_funs={
                "rgb_average": get_color_similarity,
            },
        ),
        new_belief_from_detection=BasicBeliefInit(),
        object_transition_update_function=BasicObjectTransitionUpdate(),
        goal_driven_perception_function=NullDesignationPerceptionFunction(),
        display=display,
    )
