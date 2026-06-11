from abc import ABC, abstractmethod

from qr_api.belief_interfaces import GoalMemoryModule
from qr_api.perc_typing import (
    CalibratedRGBDObservation,
    CalibratedRGBDObservationList,
    ImageFeature,
    ObjectCentricSceneRepresentation,
    ObjectDetection,
    ObjectDetectionList,
    ObjectFeature,
    UncertainMaskOutput,
    UncertainObjectCentricSceneRepresentation,
)


class PerceptionFunctionBase(ABC):
    def __call__(self, *args, **kwargs):
        return self.forward(*args, **kwargs)

    @abstractmethod
    def forward(self, *args, **kwargs):
        raise NotImplementedError


class ObjectBasedScenePerceptionFunction(PerceptionFunctionBase):
    def forward(
        self, images: CalibratedRGBDObservationList, **kwargs
    ) -> ObjectCentricSceneRepresentation:
        raise NotImplementedError("This method should be implemented in subclasses.")


class ObjectBasedGoalDrivenPerceptionFunction(PerceptionFunctionBase):
    def forward(
        self, scene: ObjectCentricSceneRepresentation, goal_memory: GoalMemoryModule
    ) -> None:
        raise NotImplementedError()


class UncertainObjectBasedScenePerceptionFunction(PerceptionFunctionBase):
    def forward(
        self, images: CalibratedRGBDObservationList, **kwargs
    ) -> UncertainObjectCentricSceneRepresentation:
        raise NotImplementedError("This method should be implemented in subclasses.")


class ImagePreprocessingFunction(PerceptionFunctionBase):
    """A function that preprocesses an image."""

    def forward(self, image: CalibratedRGBDObservation) -> CalibratedRGBDObservation:
        """Preprocess the image."""
        raise NotImplementedError("Subclasses must implement this method.")


class UncertainSegmentationFunction(PerceptionFunctionBase):
    """A function that segments an image."""

    def forward(self, image: CalibratedRGBDObservation) -> UncertainMaskOutput:
        """Segment the image."""
        raise NotImplementedError("Subclasses must implement this method.")


class UncertainShapeCompletionFunction(PerceptionFunctionBase):
    """A function that completes the shape of an object."""

    def forward(self, detection: ObjectDetection):
        """Complete the shape of the object.
        Sets the mesh attribute of the object detection."""
        raise NotImplementedError("Subclasses must implement this method.")


class LargeObjectDetectionFunction(PerceptionFunctionBase):
    """A function that detects large objects in an image, even from partial views."""

    def forward(self, image: CalibratedRGBDObservation) -> ObjectDetectionList:
        """Detect large objects in the image."""
        raise NotImplementedError("Subclasses must implement this method.")


class ImageFeaturizerFunction(PerceptionFunctionBase):
    """Compute any kind of a feature from an entire image"""

    def forward(self, image: CalibratedRGBDObservation) -> ImageFeature:
        raise NotImplementedError("Subclasses must implement this method.")


class ObjectFeaturizerFunction(PerceptionFunctionBase):
    """Compute any kind of a feature from an object (based on its mask)"""

    def forward(self, object_detection: ObjectDetection) -> ObjectFeature:
        raise NotImplementedError("Subclasses must implement this method.")
