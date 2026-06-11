from typing import List, Union

import numpy as np

from qr_api.perc_interfaces import (
    ImageFeaturizerFunction,
    ImagePreprocessingFunction,
    LargeObjectDetectionFunction,
    ObjectFeaturizerFunction,
    UncertainObjectBasedScenePerceptionFunction,
    UncertainSegmentationFunction,
    UncertainShapeCompletionFunction,
)
from qr_api.perc_typing import (
    CalibratedRGBDObservation,
    CalibratedRGBDObservationList,
    ImageMask,
    ObjectDetection,
    SceneRepresentation,
    SegmentationHypothesis,
    UncertainObjectCentricSceneRepresentation,
    UncertainObjectSegmentation,
)
from qr_utils.pcd_utilities import visualize_pointclouds
from qr_utils.traceFile import tr_a

# Later define another perception pipeline that is more monolithic, for scenecomplete


class PercPipelineUncertainSegComplete(UncertainObjectBasedScenePerceptionFunction):
    def __init__(
        self,
        image_preprocessor: ImagePreprocessingFunction,
        segmentation_method: UncertainSegmentationFunction,
        completion_method: UncertainShapeCompletionFunction,
        large_object_detection_method: LargeObjectDetectionFunction = None,
        image_featurizers: dict[str, ImageFeaturizerFunction] = {},
        object_featurizers: dict[str, ObjectFeaturizerFunction] = {},
        **kwargs,
    ):
        super().__init__()
        self.image_preprocessor = image_preprocessor
        self.segmentation_method = segmentation_method
        self.completion_method = completion_method
        self.large_object_detection_method = large_object_detection_method
        self.image_featurizers = image_featurizers
        self.object_featurizers = object_featurizers
        self.display_large_objects = kwargs.get("display_large_objects", True)
        self.display_small_objects = kwargs.get("display_small_objects", False)
        self.display = kwargs.get("display", False)
        self.display_raw_images = kwargs.get("display_raw_images", False)

    def process_object_detection(
        self,
        scene: SceneRepresentation,
        rgbd_observation: CalibratedRGBDObservation,
        object_mask: ImageMask,
    ) -> Union[ObjectDetection, None]:
        if object_mask is None:
            return None
        o = ObjectDetection(scene, ((rgbd_observation, object_mask),))
        for f_name, f_fun in self.object_featurizers.items():
            o.features[f_name] = f_fun(scene, o)
        self.completion_method(o, rgbd_observation)  # assigns mesh
        if o.mesh is None:
            return None
        return o

    # Handles an individual region, for which we have multiple hypotheses
    # Hypotheses are sorted by probability
    def process_uncertain_object_detection(
        self,
        scene: SceneRepresentation,
        rgbd_observation: CalibratedRGBDObservation,
        segmentations: List[tuple[ImageMask, float]],
    ) -> ObjectDetection:
        hypotheses = []
        for mask, prob in segmentations:
            detection = self.process_object_detection(scene, rgbd_observation, mask)
            if detection:
                hypotheses.append(SegmentationHypothesis(detection, prob))
        hypotheses.sort(key=lambda x: x[1], reverse=True)
        return hypotheses

    def forward(
        self, rgbd_observation: CalibratedRGBDObservationList
    ) -> UncertainObjectCentricSceneRepresentation:
        if self.display_raw_images:
            rgbd_observation[0].show_rgb()
            input("continue?")
            rgbd_observation[0].show_depth()
            input("Raw depth: continue?")
            if self.image_preprocessor.invalid_image_region_mask is not None:
                fizz = rgbd_observation[0]
                fizz.depth_image[self.image_preprocessor.invalid_image_region_mask] = (
                    np.nan
                )
                fizz.show_depth()
                input("Masked depth: continue?")
        # Preprocess the images
        rgbd_observation = self.image_preprocessor(rgbd_observation)
        # From here, assume a single image
        rgbd_im = rgbd_observation[0]

        scene = SceneRepresentation.from_calibrated_rgbds([rgbd_im])
        if rgbd_im.point_cloud.max() <= 0.0:
            tr_a("No point cloud in the rgbd image, skipping segmentation")
            return UncertainObjectCentricSceneRepresentation(
                scene, UncertainObjectSegmentation([], [])
            )
        rgb_im = rgbd_im.rgb_image

        if self.display:
            print("Segmenting this rgbd point cloud")
            # plt.imshow(rgb_im)
            # plt.show()
            visualize_pointclouds([rgbd_im.point_cloud], [rgb_im.reshape((-1, 3))])

        for f_name, f_fun in self.image_featurizers.items():
            scene.put_image_feature(f_name, f_fun(rgb_im))

        # Look for large objects, and because there is no segmentation uncertainty,
        # add them directly to the certain segmentation hypotheses
        surfaces = []
        if self.large_object_detection_method:
            large_object_detections = self.large_object_detection_method(scene, rgbd_im)
            for o in large_object_detections:
                for f_name, f_fun in self.object_featurizers.items():
                    o.features[f_name] = f_fun(scene, o)
                if o.get_feature("category") == "surface":
                    surfaces.append(o.get_feature("plane_eq"))
            if self.display_large_objects and large_object_detections:
                large_object_hypothesis = UncertainObjectSegmentation(
                    large_object_detections
                )
                large_object_hypothesis.show("Large objects detected", false_color=True)
            certain_hypotheses = large_object_detections
        else:
            certain_hypotheses = []

        # Segment the image; can take advantage of table plane if we know it; just using the
        # first surface found
        uncertain_segmentation_masks = self.segmentation_method(
            rgbd_im, surfaces[0] if surfaces else None
        )
        if not uncertain_segmentation_masks:
            # input('Uncos failure?')
            segmentation_hypothesis = UncertainObjectSegmentation([], [])
            return UncertainObjectCentricSceneRepresentation(
                scene, segmentation_hypothesis
            )
        # Construct all the individual object hypotheses
        for mask in uncertain_segmentation_masks.certain_segments:
            detection = self.process_object_detection(scene, rgbd_im, mask)
            if detection:
                certain_hypotheses.append(detection)
        uncertain_hypotheses = [
            self.process_uncertain_object_detection(scene, rgbd_im, seg)
            for seg in uncertain_segmentation_masks.uncertain_segments
        ]

        segmentation_hypothesis = UncertainObjectSegmentation(
            certain_hypotheses, uncertain_hypotheses
        )
        if self.display:
            segmentation_hypothesis.show("All objects detected")
        return UncertainObjectCentricSceneRepresentation(scene, segmentation_hypothesis)
