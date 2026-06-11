from typing import Optional

import numpy as np

from qr_api.perc_interfaces import UncertainSegmentationFunction
from qr_api.perc_typing import CalibratedRGBDObservation, UncertainMaskOutput
from qr_utils.traceFile import tr

# debug tag: 'segmentation'


class SimSegmenter(UncertainSegmentationFunction):
    def forward(
        self,
        observation: CalibratedRGBDObservation,
        surface_plane: Optional[np.ndarray] = None,
        display: bool = False,
    ) -> UncertainMaskOutput:

        rgb_image = observation.rgb_image
        label_image = observation.label_image
        point_cloud = observation.point_cloud  # may need to reshape

        if label_image is None:
            rgb_color_vector = rgb_image.reshape(-1, 3)
            unique_colors = np.unique(rgb_color_vector, axis=0)
            obj_masks = [
                np.all(rgb_color_vector == color, axis=1)
                for color in unique_colors
                if not np.all(color == np.zeros(3))
            ]
        else:
            unique_labels = np.unique(label_image)
            obj_masks = [
                label_image.reshape(-1) == label for label in unique_labels if label > 0
            ]

        tr("segmentation", "obj_mask sizes:", [int(np.sum(mask)) for mask in obj_masks])
        pred_masks = filter_masks(observation, obj_masks)

        if display:
            self.visualize_labeled_pcd(point_cloud, pred_masks)

        return UncertainMaskOutput(
            [mask.reshape(rgb_image.shape[:2]) for mask in pred_masks], []
        )


def filter_masks(im, masks: list[np.ndarray], min_size: int = 1000) -> list[np.ndarray]:
    def obj_pcd(mask):
        pcd = im.point_cloud_world_frame[mask]
        # Remove "zero" points from object point cloud
        return pcd[np.logical_not(np.all(pcd == 0.0, axis=1))]

    return [mask for mask in masks if obj_pcd(mask).size > min_size]
