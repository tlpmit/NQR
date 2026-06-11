from qr_api.belief_interfaces import GoalMemoryModule
from qr_api.perc_interfaces import ObjectBasedGoalDrivenPerceptionFunction
from qr_api.perc_typing import ObjectCentricSceneRepresentation


class NullDesignationPerceptionFunction(ObjectBasedGoalDrivenPerceptionFunction):
    """Goal-driven perception that does nothing — used for simulated
    perception tests where no VLM is available/needed."""

    def forward(
        self, scene: ObjectCentricSceneRepresentation, goal_module: GoalMemoryModule
    ):
        pass


class VLMDesignationPerceptionFunction(ObjectBasedGoalDrivenPerceptionFunction):
    def __init__(self):
        # Lazy import: the VLM clients need API keys and network access.
        from qr_utils.vlm_utils import GeminiClient, GPTClient  # noqa

        self._vlm_client = GeminiClient()  # GPTClient()

    def forward(
        self, scene: ObjectCentricSceneRepresentation, goal_module: GoalMemoryModule
    ):
        designators = goal_module.object_designators_of_interest
        RGB_image = scene.scene_representation.calibrated_rgbds[0].rgb_image
        detections = scene.object_detections  # list of perc_typing.ObjectRepresentation
        masks = [d.segmentation_masks[0][1] for d in detections]

        probs = self._vlm_client.batch_descr_prob(RGB_image, masks, designators)

        for detection, prob_dict in zip(detections, probs):
            for designator, prob in prob_dict.items():
                detection.features[("has_description", designator)] = prob
