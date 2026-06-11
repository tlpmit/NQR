import numpy as np
from dataclasses import dataclass


@dataclass
class Pose(object):
    mat4: np.ndarray

    @property
    def position(self):
        return self.mat4[:3, 3]

    @property
    def rotation_mat3(self):
        return self.mat4[:3, :3]
