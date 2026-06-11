from qr_api.perc_typing import CalibratedRGBDObservation, CalibratedRGBDObservationList
from qr_api.perc_interfaces import PerceptionFunctionBase
from qr_utils.pcd_utilities import point_cloud_from_depth_image_world_frame

class RGBDPreprocessor(PerceptionFunctionBase):
    def forward(self, input: CalibratedRGBDObservationList) -> CalibratedRGBDObservationList:
        raise NotImplementedError

class RGBDDepthFilter(RGBDPreprocessor):

    def __init__(self, max_depth_threshold = None, 
                 min_depth_threshold = None, nan_removal_radius = 2,
                 invalid_image_region_mask = None):
        self.max_depth_threshold = max_depth_threshold
        self.min_depth_threshold = min_depth_threshold
        self.nan_removal_radius = nan_removal_radius   # pixels
        self.invalid_image_region_mask = invalid_image_region_mask

    def forward(self, input: CalibratedRGBDObservationList) -> CalibratedRGBDObservationList:
        return [self.forward_one(img) for img in input]
    
    def forward_one(self, input: CalibratedRGBDObservation) -> CalibratedRGBDObservation:
        output = CalibratedRGBDObservation(
            input.rgb_image,
            input.depth_image,
            input.camera_intrinsics,
            input.camera_extrinsics,
            input.camera_params,
            input.label_image
        )

        # Add nan values in radius around existing Nans, in order to get rid of "streamers"
        # Then recompute point cloud
        if self.nan_removal_radius > 0:
            import cv2
            import numpy as np
            filtered_depth_image = output.depth_image.copy()
            nan_mask = np.isnan(filtered_depth_image).astype(np.uint8) * 255
            kernel_size = 2 * self.nan_removal_radius + 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
            dilated_nan_mask = cv2.dilate(nan_mask, kernel)
            filtered_depth_image[dilated_nan_mask.astype(bool)] = np.nan
        else:
            filtered_depth_image = input.depth_image

        if self.invalid_image_region_mask is not None:
            filtered_depth_image[self.invalid_image_region_mask] = np.nan
        output.depth_image = filtered_depth_image


        output.point_cloud_world_frame = point_cloud_from_depth_image_world_frame(
            filtered_depth_image, input.camera_intrinsics, input.camera_extrinsics,
            max_depth_threshold=(self.max_depth_threshold if self.max_depth_threshold else
                input.camera_params[-1]),
            min_depth_threshold=(self.min_depth_threshold if self.min_depth_threshold else 0.0))
        return output