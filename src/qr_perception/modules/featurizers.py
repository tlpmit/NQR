import numpy as np
from scipy import ndimage as ndi
from qr_api.perc_interfaces import ObjectFeaturizerFunction, ImageFeaturizerFunction 
from qr_api.perc_typing import ObjectFeature, ObjectDetection, ImageFeature, SceneRepresentation, RGBImage
from qr_utils.traceFile import tr

# debug tag: 'curtailed'


class RGBImageFeaturizer(ImageFeaturizerFunction):
    def forward(self, image: RGBImage) -> ImageFeature:
        return image

class RGBAverageObjectFeaturizer(ObjectFeaturizerFunction):
    def __init__(self, dtype: str = 'float32'):
        self.dtype = dtype

    def forward(self, scene: SceneRepresentation, object: ObjectDetection) -> ObjectFeature:
        rgb_image_feature = scene.get_image_feature('rgb')
        seg_mask = object.segmentation_masks[0][1] # Just using first image
        sum_rgb = np.sum(rgb_image_feature * seg_mask.reshape(rgb_image_feature.shape[:2])[..., None], axis=(0, 1))
        return (sum_rgb / np.sum(seg_mask)).astype(self.dtype)
    

class ImageCurtailedFeaturizer(ObjectFeaturizerFunction):
    def __init__(self, dtype: str = 'bool'):
        self.dtype = dtype

    def forward(self, scene: SceneRepresentation, object: ObjectDetection) -> ObjectFeature:
        # Return True if any mask pixel is on any image boundary
        depth_image = object.segmentation_masks[0][0].depth_image
        imshape = depth_image.shape
        seg_mask = object.segmentation_masks[0][1].reshape(imshape) # Just using first image
        mask_ind = np.where(seg_mask)
        tr('curtailed', np.min(mask_ind[0]), np.max(mask_ind[0]), 
              np.min(mask_ind[1]), np.max(mask_ind[1]))
        if np.any(np.concatenate([seg_mask[0, :], seg_mask[-1, :], seg_mask[:, 0], seg_mask[:, -1]])) \
            .astype(self.dtype):
            return True
        frac = boundary_occlusion_score(depth_image.copy(), seg_mask.copy(), depth_margin=0.01)
        return frac > 0.2
        
    
def boundary_occlusion_score(
    depth: np.ndarray,
    mask: np.ndarray,
    band_px: int = 3,
    depth_margin: float = 0.0,
    invalid_depth_values=(0, np.nan, np.inf),
    min_valid: int = 50
):
    """
    Check if a large percentage of pixels just *outside* a segmentation mask
    have *lower* (closer) depth than the nearest inside-boundary depth.

    Parameters
    ----------
    depth : (H,W) float/uint depth image
        Depth in consistent units (e.g., meters).
    mask : (H,W) bool or {0,1}
        Segmentation mask (True/1 = object).
    band_px : int
        Width (in pixels) of the outside ring around the mask to test.
    depth_margin : float
        Require outside pixel depth < (nearest-inside depth - depth_margin)
        to count as "closer". Use a small positive margin to avoid noise.
    invalid_depth_values : tuple
        Depth values to treat as invalid (ignored), e.g. (0, np.nan).
    min_valid : int
        Minimum number of valid outside-band pixels required to compute a score.
        If fewer, returns None

    Returns
    -------
    frac_closer : float
        Fraction of valid outside-band pixels that are closer than their nearest inside depth by `depth_margin`.
    details : dict
        Extra info: counts, masks, and the band used.
    """
    depth = np.asarray(depth)
    mask = np.asarray(mask).astype(bool)
    H, W = depth.shape

    # 1) Clean invalid depths
    valid_depth = np.isfinite(depth)
    for bad in invalid_depth_values:
        if np.isnan(bad):
            valid_depth &= np.isfinite(depth)
        else:
            valid_depth &= (depth != bad)

    # 2) Inside-boundary: a 1-pixel inner shell (optional, but stabilizes comparisons)
    eroded = ndi.binary_erosion(mask, structure=np.ones((3,3), bool), border_value=False)
    inner_boundary = mask & ~eroded
    inner_boundary &= valid_depth  # need valid depth on the inside boundary

    # If inner boundary has no valid depth, fall back to full mask
    if not inner_boundary.any():
        inner_boundary = mask & valid_depth

    # 3) Outside band: dilate mask and subtract mask to get a ring
    dilated = ndi.binary_dilation(mask, structure=np.ones((3,3), bool), iterations=band_px)
    outside_band = dilated & ~mask
    outside_band &= valid_depth  # only consider valid depths outside

    # 4) For each outside pixel, find the nearest *inside* pixel (by city/euclidean distance)
    # Use distance transform with return_indices=True on the *inside* boundary mask.
    # distance_transform_edt expects zeros where *features* are false; invert logic:
    inv_inner = ~inner_boundary
    _, (iy, ix) = ndi.distance_transform_edt(inv_inner, return_indices=True)

    # Map nearest inside-boundary depth to each pixel
    nearest_inside_depth = depth[iy, ix]  # same shape as depth

    # 5) Compare only on the outside band
    band_idx = np.where(outside_band)
    if band_idx[0].size < min_valid:
        return 0.

    d_out = depth[band_idx]
    d_in_near = nearest_inside_depth[band_idx]

    # valid pairwise comparisons (inside nearest also must be valid)
    valid_pair = np.isfinite(d_out) & np.isfinite(d_in_near)
    d_out = d_out[valid_pair]
    d_in_near = d_in_near[valid_pair]
    if d_out.size < min_valid:
        return 0.

    # 6) "Closer" decision
    closer = d_out < (d_in_near - depth_margin)
    frac_closer = float(np.mean(closer))

    return frac_closer