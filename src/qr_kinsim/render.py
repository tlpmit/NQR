"""
Camera rendering for the kinematic simulator.

The kinematic world (Pinocchio + Coal) has no renderer, so camera images are
produced by a *render bridge*: a headless MuJoCo scene with the same robot
model and cameras as the MuJoCo dynamics backend, plus MJCF bodies generated
from the kinematic scene's shape specs.  Before every image the bridge is
synced to the kinematic state (robot conf + object poses) and never stepped,
so it acts purely as a renderer.  Images, intrinsics, extrinsics, and label
images therefore match the dynamics backends' formats exactly.
"""

from __future__ import annotations

import os
import tempfile
import xml.etree.ElementTree as ET

import numpy as np


def _shape_to_mjcf(name: str, shape: dict, color, fixed: bool) -> str:
    """Generate a single-body MJCF file for a kinsim shape spec.
    MJCF sizes are half-extents; kinsim/Coal use full sizes."""
    root = ET.Element("mujoco", model=name)
    asset = None
    worldbody = ET.SubElement(root, "worldbody")
    body = ET.SubElement(worldbody, "body", name=name)

    rgba = " ".join(str(float(c)) for c in (tuple(color) + (1.0,))[:4])
    geom_attrs = {"name": name, "rgba": rgba}

    if "box" in shape:
        sx, sy, sz = (float(v) for v in shape["box"])
        geom_attrs |= {"type": "box", "size": f"{sx / 2} {sy / 2} {sz / 2}"}
    elif "sphere" in shape:
        geom_attrs |= {"type": "sphere", "size": str(float(shape["sphere"]))}
    elif "cylinder" in shape:
        r, l = (float(v) for v in shape["cylinder"])
        geom_attrs |= {"type": "cylinder", "size": f"{r} {l / 2}"}
    elif "mesh" in shape:
        asset = ET.SubElement(root, "asset")
        ET.SubElement(asset, "mesh", name=name,
                      file=os.path.abspath(str(shape["mesh"])))
        geom_attrs |= {"type": "mesh", "mesh": name}
    else:
        raise ValueError(f"Unknown shape spec: {shape}")

    ET.SubElement(body, "geom", **geom_attrs)
    return ET.tostring(root, encoding="unicode")


class MujocoRenderBridge:
    """Headless MuJoCo scene mirroring a kinematic world, used as a camera."""

    def __init__(self, robot: str, objects: dict, camera_size=None):
        """
        Args:
            robot: "rby1"/"rainbow" or "spot".
            objects: name → dict(shape=…, color=…, fixed=…, pose=[x, y, z,
                qw, qx, qy, qz]) describing the kinematic scene.
        """
        if robot in ("rby1", "rainbow"):
            from qr_robots.mujoco.rby1.sim import make_sim
        elif robot == "spot":
            from qr_robots.mujoco.spot.sim import make_sim
        else:
            raise ValueError(f"No MuJoCo render model for robot '{robot}'")

        self._tmpdir = tempfile.TemporaryDirectory(prefix="kinsim_render_")
        mj_objects: dict = {}
        for name, spec in objects.items():
            path = os.path.join(self._tmpdir.name, f"{name}.xml")
            with open(path, "w") as f:
                f.write(_shape_to_mjcf(name, spec["shape"], spec["color"],
                                       spec["fixed"]))
            pose = np.asarray(spec["pose"], dtype=float)
            mj_objects[name] = {
                "file": path,
                "fixed": bool(spec["fixed"]),
                # Fixed objects can't be re-posed later; weld them in place.
                "pos": pose[:3].tolist(),
                "quat": pose[3:].tolist(),
            }

        kwargs = {} if camera_size is None else {"camera_size": camera_size}
        self.sim = make_sim(self._tmpdir.name, mj_objects, mode="headless", **kwargs)

    def sync(self, conf: np.ndarray, object_poses: dict[str, np.ndarray]) -> None:
        """Mirror the kinematic state: canonical robot conf and
        object world poses ([x, y, z, qw, qx, qy, qz]) for free objects."""
        self.sim.set_robot_conf(conf)
        for name, pose in object_poses.items():
            self.sim.set_object_pose(name, pose[:3], pose[3:])

    def get_camera_image(self, camera_name: str):
        return self.sim.get_camera_image(camera_name)

    def get_camera_intrinsics(self, camera_name: str) -> np.ndarray:
        return self.sim.get_camera_intrinsics(camera_name)

    def get_camera_extrinsics(self, camera_name: str) -> np.ndarray:
        return self.sim.get_camera_extrinsics(camera_name)

    def close(self) -> None:
        self._tmpdir.cleanup()
