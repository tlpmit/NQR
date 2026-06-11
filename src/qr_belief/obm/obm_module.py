from scipy.optimize import linear_sum_assignment

from qr_api.belief_interfaces import GoalMemoryModule, ObjectMemoryModule
from qr_api.obm_interfaces import (
    OBMInitBeliefFunction,
    OBMMatchScoreFunction,
    OBMMergeFunction,
    OBMObjectTransitionUpdateFunction,
)
from qr_api.obm_typing import ObjectBelief, ObjectMemory
from qr_api.perc_interfaces import (
    ObjectBasedGoalDrivenPerceptionFunction,
    ObjectBasedScenePerceptionFunction,
)
from qr_api.perc_typing import ObjectDetection
from qr_api.policy_typing import Action, ActionRV, TransitionPrediction
from qr_api.sensor_typing import Observation
from qr_utils.pcd_utilities import IoM, max_iom
from qr_utils.traceFile import tr

# debug tag: 'obm'


class ObjectBasedDataAssociationFilter(ObjectMemoryModule):
    def __init__(
        self,
        goal_belief: GoalMemoryModule,
        object_perception_function: ObjectBasedScenePerceptionFunction,
        merge_function: OBMMergeFunction,
        match_score_function: OBMMatchScoreFunction,
        new_belief_from_detection: OBMInitBeliefFunction,
        object_transition_update_function: OBMObjectTransitionUpdateFunction,
        goal_driven_perception_function: ObjectBasedGoalDrivenPerceptionFunction,
        new_hypothesis_match_score: float = 0.002,
        existence_confidence_threshold: float = 0.1,
        **kwargs,
    ):
        super().__init__()
        self.memory = ObjectMemory()
        self.goal_belief = goal_belief
        self.object_perception_function = object_perception_function
        self.merge_function = merge_function
        self.match_score_function = match_score_function
        self.new_belief_from_detection = new_belief_from_detection
        self.object_transition_update_function = object_transition_update_function
        self.goal_driven_perception_function = goal_driven_perception_function
        self.new_hypothesis_match_score = new_hypothesis_match_score
        self.existence_confidence_threshold = existence_confidence_threshold
        self.display = kwargs.get("display", False)

    def _reset(self, observation: Observation):
        self.memory = ObjectMemory()
        self.observation_update(observation)

    def _update(
        self,
        action: Action,
        action_rv: ActionRV,
        prediction: TransitionPrediction,
        observation: Observation,
    ):
        self.transition_update(action, action_rv, prediction)
        self.observation_update(observation)
        if observation.get("images", None) or prediction.object_predictions:
            self.memory.print()
            if self.display:
                self.memory.show()

    # May not need first two arguments
    def transition_update(
        self, action: Action, action_rv: ActionRV, prediction: TransitionPrediction
    ):
        for obj_pred in prediction.object_predictions:
            # For now, assume we can do these updates independently
            self.object_transition_update_function(
                self.memory[obj_pred.object_name], obj_pred
            )

    def observation_update(self, observation: Observation):
        if not observation["images"]:
            return
        # For now, just use the first image
        uncertain_scene = self.object_perception_function.forward(observation["images"])
        ml_scene = uncertain_scene.get_ml_object_scene_representation()
        # Operate on the scene to compute properties of interest, contingent on goal
        # Side effects the detections
        self.goal_driven_perception_function(ml_scene, self.goal_belief)
        # For now, just use the most likely object detections in the case of ambiguity
        object_detections = ml_scene.object_detections
        # Dictionary from index of detection to internal name of the object hypothesis
        #  and a matching score
        associations = self.get_associations(object_detections)
        # Update the memory with the associations
        for index, (internal_name, _score) in associations.items():
            detection = object_detections[index]
            hypothesis = self.memory[internal_name]
            self.memory[internal_name] = self.merge_function(hypothesis, detection)
        # Instantiate new hypotheses for unmatched detections
        new_hypotheses = []
        for index, detection in enumerate(object_detections):
            if index not in associations:
                new_hypothesis = self.new_belief_from_detection(detection)
                new_hypotheses.append(new_hypothesis)
        # Adjust hypothesis likelihoods.  Maybe this should be more structured
        self.update_hypothesis_likelihoods(
            observation["images"], object_detections, new_hypotheses, associations
        )
        # Remove hypotheses with low likelihood
        self.memory.delete_low_likelihood_hypotheses(
            self.existence_confidence_threshold
        )
        # Add new hypotheses to the memory
        for new_hypothesis in new_hypotheses:
            self.memory.add_hypothesis(new_hypothesis)

    def get_associations(
        self, object_detections: list[ObjectDetection]
    ) -> dict[int, tuple[str, float]]:
        internal_names = []
        hypotheses = []
        for name, h in self.memory.objects.items():
            # lpk: Do something to handle uncertainty about holding
            if not h.in_hand:
                internal_names.append(name)
                hypotheses.append(h)

        if len(hypotheses) == 0 or len(object_detections) == 0:
            return {}

        score_matrix = self.find_pairwise_scores(hypotheses, object_detections)

        # Don't require a complete matching.  Handle this by allowing
        # observations to be assigned to a hypothesis "new".  Do we have to add
        # a "new" hypothesis for each observation to maintain uniqueness?
        # Score of a "new" match is higher than a terrible actual match

        row_idxs, col_idxs = linear_sum_assignment(score_matrix, maximize=True)

        matching = {}
        for row, col in zip(row_idxs, col_idxs):
            if row < len(internal_names):
                # This is a match to a hypothesis.  If the best match is to
                # the detection itself, omit it from the matching and a new hypoth
                # will be created downstream
                name = internal_names[row]
                match_score = score_matrix[row][col]
                matching[col] = (name, match_score)

        return matching

    def find_pairwise_scores(
        self, hypotheses: list[ObjectBelief], detections: list[ObjectDetection]
    ) -> list[list[float]]:
        """Computes pairwise match score between hypotheses and detections.
            Returns: a 2D array of pairwise match scores

        We treat matching a detection to another detection as infinitely bad,
        and matching a detection to itself as deciding it represents its own
        new hypothesis
        """

        def score(h, d):
            if not isinstance(h, ObjectBelief):
                return self.new_hypothesis_match_score
            return self.match_score_function(h, d)

        def print_score_matrix(tag=("obm", "log")):
            n = len(hypotheses)  # num of real hypotheses
            left = 5
            # Print out the matrix nicely with labels of nominal colors
            tr(tag, " " * left, end=" ")
            for d in detections:
                tr(tag, d.descriptive_string[:5].ljust(5), end=" ")
            tr(tag, "\n")
            for i, row in enumerate(score_matrix[:n]):
                tr(tag, hypotheses[i].name[:5].ljust(5), end=" ")
                for col in row:
                    tr(tag, f"{col:.3f}", end=" ")
                tr(tag, f"      conf = {hypotheses[i].existence_confidence:.3f}")

        score_matrix = [
            [score(h, d) for d in detections] for h in hypotheses + detections
        ]
        print_score_matrix()
        return score_matrix

    # TODO: This is a disgusting wad of code that doesn't belong here
    # modularize it and pass the relevant functions in
    # move visible somewhere else

    def update_hypothesis_likelihoods(
        self,
        images: list[Observation],
        object_detections: list[ObjectDetection],
        new_hypotheses: list[ObjectBelief],
        associations: dict[int, tuple[str, float]],
    ):
        from qr_utils.sensor import Sensor

        # TODO: lpk: gross hack just so I can process some current pkl files
        if not hasattr(images[0], "camera_params") or images[0].camera_params is None:
            images[0].camera_params = (
                60.0,
                720,
                1280,
                0.05,
                3.0,
            )  # FOV, height, width, min_depth, max_depth
            images[0].camera_params = (
                60.0,
                480,
                640,
                0.05,
                3.0,
            )  # FOV, height, width, min_depth, max_depth

        matched_hypotheses = {h for (h, _) in associations.values()}
        sensor_pose = images[0].camera_pose
        named_meshes = {k: h.mesh_worldframe for k, h in self.memory.objects.items()}
        _, _, _, body_obj_counts, body_vis_counts = Sensor.trace_image_raw(
            sensor_pose,
            images[0].camera_intrinsics,
            images[0].camera_params,
            named_meshes,
        )
        MIN_PIXELS_FOR_DETECTION = 20
        MIN_VIS_RATIO_FOR_DETECTON = 0.5
        visible = {}
        for o in named_meshes:
            vis_count = body_vis_counts.get(o, 0) >= MIN_PIXELS_FOR_DETECTION
            visible[o] = (
                vis_count
                and body_vis_counts.get(o, 0) / body_obj_counts.get(o, 1)
                >= MIN_VIS_RATIO_FOR_DETECTON
            )
        tr(("obm", "log"), "Objects predicted to be visible:", visible)
        tr(("obm_detail", "log"), "body vis counts", body_vis_counts)

        tr(("obm", "log"), "Matched objects:", list(matched_hypotheses))
        tr(
            ("obm", "log"),
            "Unmatched objects:",
            [
                h.name
                for h in self.memory.objects.values()
                if h.name not in matched_hypotheses
            ],
        )
        tr(("obm", "log"), "New objects:", [h.name for h in new_hypotheses])
        for new in new_hypotheses:
            hypothesis_is_table = new.get_feature("category", "thing") in (
                "surface",
                "table",
            )
            if hypothesis_is_table:
                continue
            for old_k, old_o in self.memory.objects.items():
                object_is_table = old_o.get_feature("category", "thing") in (
                    "surface",
                    "table",
                )
                if object_is_table:
                    continue
                overlap = IoM(
                    new.mesh_worldframe.vertices, old_o.mesh_worldframe.vertices
                )
                if overlap > 0.2:
                    tr(
                        ("obm", "log"),
                        f"New object {new.name} overlaps with object {old_k}: {overlap}",  # noqa: E501
                    )
                    new.existence_confidence = new.existence_confidence * 0.9
                    tr(
                        ("obm", "log"),
                        f"Decreasing confidence of {new.name} to {new.existence_confidence}",  # noqa: E501
                    )
                    old_o.existence_confidence = old_o.existence_confidence * 0.9
                    tr(
                        ("obm", "log"),
                        f"Decreasing confidence of {old_k} to {old_o.existence_confidence}",  # noqa: E501
                    )

        # Update existence confidence on old hypotheses
        # TODO: make this at least somewhat probabilistically well founded
        good_matched_hypotheses_verts = [
            self.memory.objects[k].mesh_worldframe.vertices for k in matched_hypotheses
        ]
        for k, h in self.memory.objects.items():
            object_is_table = h.get_feature("category", "thing") in ("surface", "table")
            if object_is_table:
                continue
            if k in matched_hypotheses:
                ep = 0.8  # prob obs given exists TODO: make this a function of the
                # match score?
                h.existence_confidence = (
                    h.existence_confidence
                    * ep
                    / (
                        h.existence_confidence * ep
                        + (1 - h.existence_confidence) * (1 - ep)
                    )
                )
            else:
                # Old hypothesis that was not matched and overlaps new or matched
                # hypotheses
                iom_overlap = max_iom(
                    h.mesh_worldframe.vertices, good_matched_hypotheses_verts
                )[0]
                tr(
                    ("obm", "log"),
                    f"Hypoth {k} overlap with other hypoths: {iom_overlap}",
                )
                if iom_overlap > 0.2:
                    tr(
                        ("obm", "log"),
                        f"Hypoth {k} unmatched and overlaps detection or old hypoth. ",
                    )
                    h.existence_confidence = h.existence_confidence * 0.9
                    tr(
                        ("obm", "log"),
                        f"Decreasing confidence of {k} to {h.existence_confidence}",
                    )
                elif visible.get(k, True):
                    # Old hypothesis that was not matched but should have been visible
                    tr(
                        ("obm", "log"),
                        f"Hypoth {k} unmatched and should have been visible",
                    )
                    h.existence_confidence = h.existence_confidence * 0.9
                    tr(
                        ("obm", "log"),
                        f"Decreasing confidence of {k} to {h.existence_confidence}",
                    )
