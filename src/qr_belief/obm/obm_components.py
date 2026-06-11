import numpy as np
import trimesh
from trimesh.transformations import translation_matrix

from qr_api.obm_interfaces import (
    OBMInitBeliefFunction,
    OBMMatchScoreFunction,
    OBMMergeFunction,
    OBMObjectTransitionUpdateFunction,
)
from qr_api.obm_typing import ObjectBelief
from qr_api.perc_typing import ObjectDetection, UncertainObjectMesh
from qr_api.policy_typing import ObjectTransitionPrediction
from qr_utils.misc_utils import gensym
from qr_utils.pcd_utilities import IoM, inflate
from qr_utils.traceFile import tr

# debug tag: 'obm'


class BasicMatchScoreFunction(OBMMatchScoreFunction):
    def __init__(self, attribute_similarity_funs: dict):
        """
        Initialize the match score function with a dictionary of attribute similarity
          functions.
        """
        super().__init__()
        self.attribute_similarity_funs = attribute_similarity_funs

    # lpk: actually using intersection over minimum, because the inflated object becomes
    #  very big when we have substantial position uncertainty, but that shouldn't result
    #  in a low score
    def iou_match_score(
        self, object_state: ObjectBelief, detection: ObjectDetection
    ) -> float:
        mesh1 = detection.mesh
        mesh2 = object_state.mesh_worldframe
        bb1 = mesh1.bounds
        bb2 = mesh2.bounds
        if abs(bb2[0][2] - bb1[1][2]) < 0.001:
            print("thin sheet")
            pass
        delta = object_state.pose_confidence_delta
        iom = IoM(mesh1.vertices, inflate(mesh2.vertices, delta))
        # if not (np.any(bb1[0] >= bb2[1]) or np.any(bb1[1] <= bb2[0])):
        #     # overlap
        #     if iom < 0.5:
        #         print(bb1)
        #         print(bb2)
        #         print('iom', iom)
        #         visualize_pointclouds_colors([mesh1.vertices, mesh2.vertices],
        #                                    [np.array([255,0,0]), np.array([0,255,0])])
        #         input('Low IoM for overlapping objects')
        return iom

    def surface_match_score(
        self, object_state: ObjectBelief, detection: ObjectDetection
    ) -> float:
        mesh1 = detection.mesh
        mesh2 = object_state.mesh_worldframe
        bb1 = mesh1.bounds
        bb2 = mesh2.bounds
        # If the top of the surfaces is close
        if abs(bb1[1, 2] - bb2[1, 2]) < 0.025:
            # The xy boxes overlap
            if not (
                np.any(bb1[0, :2] >= bb2[1, :2]) or np.any(bb1[1, :2] <= bb2[0, :2])
            ):
                # Could do more careful overlap via shapely polygons
                return 1.0
        return 0.0

    # Between 0 and 1 if defined, else None
    def appearance_match_score(
        self, hypothesis: ObjectBelief, detection: ObjectDetection
    ) -> float:
        """Compute cosine similarity of appearance features.
        Return None if we are not computing appearance features"""
        possible_appearance_attrs = ["rgb_average"]
        appearance_attrs = [
            attr
            for attr in possible_appearance_attrs
            if hypothesis.has_feature(attr) and detection.has_feature(attr)
        ]
        return sum(
            self.attribute_similarity_funs[attr](
                hypothesis.get_feature(attr, None), detection.get_feature(attr, None)
            )
            for attr in appearance_attrs
        ) / len(appearance_attrs)

    def __call__(self, hypothesis: ObjectBelief, detection: ObjectDetection) -> float:
        """Computes the matching score between a hypothesis and a detection."""

        # open3d_show([hypothesis.to_o3d(), detection.to_o3d()])

        # Compute 3D IoU
        hypothesis_is_table = hypothesis.get_feature("category", "thing") in (
            "surface",
            "table",
        )
        detection_is_table = detection.get_feature("category", "thing") in (
            "surface",
            "table",
        )
        # Bonus for matching tables because they are thin
        if hypothesis_is_table and detection_is_table:
            iou_score = self.surface_match_score(hypothesis, detection)
            category_score = 1
        else:
            iou_score = self.iou_match_score(hypothesis, detection)
            category_score = int(
                hypothesis.features.get("category", "thing")
                == detection.features.get("category", "thing")
            )

        # Compute similarity of visual features
        appearance_score = self.appearance_match_score(hypothesis, detection)

        # Used to consider the degree to which this detection overlaps other hypotheses,
        #  but detections are better now, so no need

        # TODO: grotesque, but be sure to only match surfaces to surfaces
        if hypothesis_is_table != detection_is_table:
            result = 0.0
        elif appearance_score is not None:
            # Should we sum instead?
            result = iou_score * appearance_score * category_score
        else:
            result = iou_score * category_score
        return result


class BasicMergeFunction(OBMMergeFunction):
    def __init__(
        self,
        new_belief_from_detection: OBMInitBeliefFunction,
        fusion_threshold: float = 0.05,
    ):
        super().__init__()
        self.new_belief_from_detection = new_belief_from_detection
        self.fusion_threshold = fusion_threshold

    def merge_attributes(self, hypothesis: ObjectBelief, detection: ObjectDetection):
        # average feature values
        for attr in hypothesis.features:
            # todo: handle other categories better
            if "average" in attr:
                hypothesis.put_feature(
                    attr,
                    (hypothesis.get_feature(attr) + detection.get_feature(attr)) / 2,
                )
            elif attr == "curtailed":
                hypoth_curtailed = hypothesis.get_feature("curtailed", None)
                detection_curtailed = detection.get_feature("curtailed", None)
                hypothesis.put_feature(
                    "curtailed", hypoth_curtailed and detection_curtailed
                )
            elif (
                type(attr) is tuple
                and attr[0] == "has description"
                and detection.has_feature(attr)
            ):
                # Hmm. Average the probabilities?
                hypoth_prob = hypothesis.get_feature(attr)
                detection_prob = detection.get_feature(attr)
                hypothesis.put_feature(attr, (hypoth_prob + detection_prob) / 2)
            else:
                hypothesis.put_feature(attr, detection.get_feature(attr))

    def center_and_pose(
        self, hypothesis: ObjectBelief, fused_umesh: UncertainObjectMesh
    ):
        umesh_centered, uncenter_tform, center_tform = fused_umesh.centered()
        hypothesis.m4_pose = uncenter_tform
        hypothesis.mesh = umesh_centered

    # Meshes are in world frame
    def simple_fusion(
        self, hypoth_mesh: UncertainObjectMesh, detection_mesh: UncertainObjectMesh
    ) -> UncertainObjectMesh:
        h_mesh = hypoth_mesh.to_trimesh()
        d_mesh = detection_mesh.to_trimesh()
        return self.fuse_meshes_tri(h_mesh, d_mesh)

        # print('fused r and g into b')
        # visualize_pointclouds_colors([h_mesh.vertices, d_mesh.vertices,
        # fused_umesh.vertices],
        #                             [np.array([255,0,0]), np.array([0,255,0]),
        # np.array([0,0,255])])

    def fuse_meshes_tri(
        self, h_mesh: trimesh.Trimesh, d_mesh: trimesh.Trimesh
    ) -> UncertainObjectMesh:
        fused_vertices = np.vstack([h_mesh.vertices, d_mesh.vertices])
        fused_mesh = trimesh.convex.convex_hull(fused_vertices)
        return UncertainObjectMesh(
            fused_mesh.vertices, fused_mesh.faces, np.zeros(len(fused_mesh.vertices))
        )

    # Meshes are in world frame
    def aligned_fusion(
        self, h_mesh: UncertainObjectMesh, d_mesh: UncertainObjectMesh
    ) -> UncertainObjectMesh:
        # We know which vertices are "certain"
        # Not clear how much to trust the predicted shapes vs the observed vertices
        # Maybe focus on maximizing volume of overlap of completed shapes?
        # We know whether some views have been curtailed: maybe use this instead in the
        #  decision to merge?  Ignore for now.
        # Assume initial alignment isn't terrible
        # Which mesh should move?  I guess the old one
        pass

    def merge_surfaces(self, hypothesis: ObjectBelief, detection: ObjectDetection):
        # Move the meshes to align them in z
        h_mesh = hypothesis.mesh_worldframe.to_trimesh()
        d_mesh = detection.mesh.to_trimesh()
        h_z = h_mesh.bounds[1, 2]
        d_z = d_mesh.bounds[1, 2]
        h_area = h_mesh.extents[0] * h_mesh.extents[1]
        d_area = d_mesh.extents[0] * d_mesh.extents[1]
        new_z = (h_area * h_z + d_area * d_z) / (h_area + d_area)
        h_mesh.apply_transform(translation_matrix((0, 0, new_z - h_z)))
        d_mesh.apply_transform(translation_matrix((0, 0, new_z - d_z)))
        # Merge the meshes
        fused_umesh = self.fuse_meshes_tri(h_mesh, d_mesh)
        self.center_and_pose(hypothesis, fused_umesh)
        self.merge_attributes(hypothesis, detection)

    def use_detection(self, hypothesis: ObjectBelief, detection: ObjectDetection):
        new_object = self.new_belief_from_detection(detection)
        hypothesis.mesh = new_object.mesh
        hypothesis.m4_pose = new_object.m4_pose

    def fuse_pcds(
        self, hypothesis: ObjectBelief, detection: ObjectDetection
    ) -> ObjectBelief:
        fused_umesh = self.simple_fusion(hypothesis.mesh_worldframe, detection.mesh)
        self.center_and_pose(hypothesis, fused_umesh)
        self.merge_attributes(hypothesis, detection)

    # TODO:
    # ideally, actually do registration of the whole scene and then fuse
    def fuse_decision(
        self, hypothesis: ObjectBelief, detection: ObjectDetection
    ) -> bool:
        mesh1 = detection.mesh
        mesh2 = hypothesis.mesh_worldframe

        if hypothesis.get_feature("curtailed", False) or detection.get_feature(
            "curtailed", False
        ):
            # If either is curtailed, then fuse, because the overlap test won't be
            # passed
            tr(
                "log",
                f"Fusing curtailed H:{hypothesis.name} D:{detection.descriptive_string}",  # noqa: E501
            )
            return True

        # iou = IoU(mesh1.vertices, mesh2.vertices)
        iom = IoM(mesh1.vertices, mesh2.vertices)

        # TODO: reconsider this approach
        min_iom_for_fusion = 0.75
        fuse = iom > min_iom_for_fusion

        tr("log", f"{iom=} H:{hypothesis.name} D:{detection.descriptive_string}")
        return fuse

    def __call__(
        self, hypothesis: ObjectBelief, detection: ObjectDetection
    ) -> ObjectBelief:
        """
        Merges a hypothesis with a detection.
        """
        """Combine a new detection with this hypothesis in-place.

        Args:
            detection : instance of :class:`ObjectDetection`
        """
        # Fuse the detection point cloud with the hypothsis if the hypothesis
        # uncertainty is relatively low,
        # Otherswise, just use the detection

        hypothesis_is_table = hypothesis.get_feature("category", "thing") in (
            "surface",
            "table",
        )
        detection_is_table = detection.get_feature("category", "thing") in (
            "surface",
            "table",
        )

        if hypothesis_is_table and detection_is_table:
            tr(
                ("log", "obm"),
                f"Merge surfaces: H:{hypothesis.name} D:{detection.descriptive_string}",
            )
            self.merge_surfaces(hypothesis, detection)
        elif self.fuse_decision(hypothesis, detection):
            tr(
                ("log", "obm"),
                f"Fuse: H:{hypothesis.name} D:{detection.descriptive_string}",
            )
            self.fuse_pcds(hypothesis, detection)
        else:
            tr(
                ("log", "obm"),
                f"Use detection: H:{hypothesis.name} D:{detection.descriptive_string}",
            )
            self.use_detection(hypothesis, detection)

        # Combine attributes
        for attr in hypothesis.features:
            if attr == "rgb_average":
                feature_combo_mean(attr, hypothesis, detection)
            elif type(attr) is tuple and attr[0] == "has_description":
                feature_combo_max(attr, hypothesis, detection)
            elif attr == "curtailed":
                feature_combo_and(attr, hypothesis, detection)
            elif attr == "plane_eq":
                feature_combo_mean(attr, hypothesis, detection)
            else:
                print("Copying", attr, "from detection to hypoth")
                feature_combo_new(attr, hypothesis, detection)

        # Clear cached results
        for k in ["meshes", "grasps", "rest_faces"]:
            if k in hypothesis.cached_properties:
                tr("log", f"Clearing cached property {k} of object {hypothesis.name}")
                del hypothesis.cached_properties[k]
        hypothesis.detections.append(detection)

        return hypothesis


class BasicBeliefInit(OBMInitBeliefFunction):
    def __call__(self, detection: ObjectDetection) -> ObjectBelief:
        """
        Initializes a new hypothesis from a detection.
        """
        features = detection.features.copy()
        name = gensym(detection.descriptive_string)
        tr(("obm_detail", "log"), "Making new object hypothesis:", name)

        mesh_centered, uncenter_tform, center_tform = detection.mesh.centered()

        object_state = ObjectBelief(
            name=name,
            m4_pose=uncenter_tform,
            mesh=mesh_centered,
            features=features,
            detections=[detection],
        )

        return object_state


class BasicObjectTransitionUpdate(OBMObjectTransitionUpdateFunction):
    def __call__(
        self, hypothesis: ObjectBelief, prediction: ObjectTransitionPrediction
    ):
        for attr, (new_value, delta) in prediction.attribute_updates.items():
            if attr == "pose":
                # Prediction is that object is resting somehwere
                hypothesis.m4_pose = new_value
                if hypothesis.in_hand:
                    # Putting the object down with this uncertainty
                    hypothesis.in_hand = False
                    hypothesis.pose_confidence_delta = delta
                else:
                    # Added (or decreased!) uncertainty
                    hypothesis.pose_confidence_delta += delta
            elif attr == "in_hand":
                # Prediction is that object is now in the hand
                assert new_value is not False, "Use pose attribute to put object down"
                hypothesis.in_hand = True
            else:
                raise KeyError(f"Unknown attribute '{attr}' in transition prediction.")


def feature_combo_new(feature, hypothesis, detection):
    hypothesis.put_feature(feature, detection.get_feature(feature))


def feature_combo_max(feature, hypothesis, detection):
    hypothesis.put_feature(
        feature, max(detection.get_feature(feature), hypothesis.get_feature(feature))
    )


def feature_combo_mean(feature, hypothesis, detection):
    hypothesis.put_feature(
        feature,
        np.mean(
            [detection.get_feature(feature), hypothesis.get_feature(feature)], axis=0
        ),
    )


def feature_combo_and(feature, hypothesis, detection):
    hypothesis.put_feature(
        feature,
        detection.get_feature(feature) and hypothesis.get_feature(feature),
    )
