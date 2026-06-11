"""
KinematicWorld — kinematics-only world model built on Pinocchio + Coal,
displayed via Meshcat.

This is the replacement for the HPN/Roboverse kinematic simulator: one robot
(loaded from URDF, modeled as a tree of chains by the caller) plus a set of
rigid objects.  Objects are either free (posed in the world frame), fixed, or
*attached* to a robot frame — attachment models the effect of grasping (and
pushing) by making the object follow the kinematic chain.

No dynamics: setting the configuration is instantaneous, and collision
checking via Coal is the only physical reasoning.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import coal
import numpy as np
import pinocchio as pin


# ── joint q-vector helpers ────────────────────────────────────────────────────
# Revolute/prismatic joints have nq=1; continuous joints are represented as
# (cos, sin) with nq=2; the planar root joint is (x, y, cos, sin) with nq=4.

def get_joint_value(model: pin.Model, q: np.ndarray, joint_name: str) -> float:
    j = model.joints[model.getJointId(joint_name)]
    if j.nq == 1:
        return float(q[j.idx_q])
    if j.nq == 2:
        return math.atan2(q[j.idx_q + 1], q[j.idx_q])
    raise ValueError(f"Joint '{joint_name}' has nq={j.nq}; expected 1 or 2")


def set_joint_value(model: pin.Model, q: np.ndarray, joint_name: str, value: float) -> None:
    j = model.joints[model.getJointId(joint_name)]
    if j.nq == 1:
        q[j.idx_q] = value
    elif j.nq == 2:
        q[j.idx_q] = math.cos(value)
        q[j.idx_q + 1] = math.sin(value)
    else:
        raise ValueError(f"Joint '{joint_name}' has nq={j.nq}; expected 1 or 2")


def _coal_transform(M: pin.SE3) -> coal.Transform3s:
    return coal.Transform3s(M.rotation, M.translation)


def _pose_to_se3(pos, quat_wxyz=None) -> pin.SE3:
    if quat_wxyz is None:
        quat_wxyz = [1.0, 0.0, 0.0, 0.0]
    w, x, y, z = (float(v) for v in quat_wxyz)
    return pin.SE3(pin.Quaternion(w, x, y, z).matrix(), np.asarray(pos, dtype=float))


def _se3_to_pose(M: pin.SE3) -> np.ndarray:
    quat = pin.Quaternion(M.rotation)
    return np.array([
        *M.translation, quat.w, quat.x, quat.y, quat.z,
    ])


# ── scene objects ─────────────────────────────────────────────────────────────

@dataclass
class SceneObject:
    name: str
    geometry: "coal.CollisionGeometry"
    shape: dict                       # original shape spec (for display)
    color: tuple = (0.6, 0.6, 0.6, 1.0)
    fixed: bool = False
    pose: pin.SE3 = field(default_factory=pin.SE3.Identity)
    """World pose for free/fixed objects; ignored while attached."""
    attach_frame: Optional[str] = None
    """Robot frame this object is rigidly attached to (None = not attached)."""
    attach_offset: pin.SE3 = field(default_factory=pin.SE3.Identity)
    """Pose of the object in the attach frame."""


def _pair_collides(g1, t1, g2, t2, margin: float) -> bool:
    """
    Collision test honoring a security margin.  Coal rejects negative margins
    for BVH (mesh) geometry, so for those pairs we collide at margin 0 and
    filter by penetration depth (Coal reports depth as a negative number).
    """
    req = coal.CollisionRequest()
    req.security_margin = margin
    res = coal.CollisionResult()
    try:
        return coal.collide(g1, t1, g2, t2, req, res) > 0
    except ValueError:
        if margin >= 0:
            raise
        req0 = coal.CollisionRequest()
        res0 = coal.CollisionResult()
        if coal.collide(g1, t1, g2, t2, req0, res0) == 0:
            return False
        depths = [c.penetration_depth for c in res0.getContacts()]
        return bool(depths) and min(depths) <= margin


def _make_geometry(shape: dict) -> coal.CollisionGeometry:
    """shape: {"box": [sx, sy, sz]} | {"sphere": r} | {"cylinder": [r, l]}
    | {"mesh": path, "scale": [sx, sy, sz]}"""
    if "box" in shape:
        sx, sy, sz = (float(v) for v in shape["box"])
        return coal.Box(sx, sy, sz)
    if "sphere" in shape:
        return coal.Sphere(float(shape["sphere"]))
    if "cylinder" in shape:
        r, l = (float(v) for v in shape["cylinder"])
        return coal.Cylinder(r, l)
    if "mesh" in shape:
        scale = np.asarray(shape.get("scale", [1.0, 1.0, 1.0]), dtype=float)
        loader = coal.MeshLoader()
        return loader.load(str(shape["mesh"]), scale)
    raise ValueError(f"Unknown shape spec: {shape}")


# ── world ─────────────────────────────────────────────────────────────────────

class KinematicWorld:
    def __init__(
        self,
        urdf_path: str,
        root_joint: Optional[pin.JointModel] = None,
        package_dirs: Optional[list[str]] = None,
        visualize: bool = False,
    ):
        package_dirs = package_dirs or []
        if root_joint is not None:
            self.model = pin.buildModelFromUrdf(str(urdf_path), root_joint)
        else:
            self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.collision_model = pin.buildGeomFromUrdf(
            self.model, str(urdf_path), pin.GeometryType.COLLISION,
            package_dirs=package_dirs,
        )
        self.visual_model = pin.buildGeomFromUrdf(
            self.model, str(urdf_path), pin.GeometryType.VISUAL,
            package_dirs=package_dirs,
        )

        self.data = self.model.createData()
        self.collision_data = self.collision_model.createData()

        self.q = pin.neutral(self.model)
        self.objects: dict[str, SceneObject] = {}

        self.viz = None
        if visualize:
            from pinocchio.visualize import MeshcatVisualizer

            self.viz = MeshcatVisualizer(
                self.model, self.collision_model, self.visual_model
            )
            self.viz.initViewer(open=False)
            self.viz.loadViewerModel()

        self.update()

    # ── kinematics ────────────────────────────────────────────────────────────

    def update(self) -> None:
        """Recompute FK and robot collision-geometry placements for self.q."""
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)
        pin.updateGeometryPlacements(
            self.model, self.data, self.collision_model, self.collision_data
        )

    def set_q(self, q: np.ndarray) -> None:
        self.q = np.asarray(q, dtype=float).copy()
        self.update()

    def get_frame_pose(self, frame_name: str) -> pin.SE3:
        """World pose of a robot frame (link or joint name)."""
        fid = self.model.getFrameId(frame_name)
        if fid >= len(self.model.frames):
            raise ValueError(f"Frame '{frame_name}' not found")
        return self.data.oMf[fid]

    def joint_limits(self, joint_name: str) -> tuple[float, float]:
        j = self.model.joints[self.model.getJointId(joint_name)]
        if j.nq != 1:
            return (-np.pi, np.pi)
        return (
            float(self.model.lowerPositionLimit[j.idx_q]),
            float(self.model.upperPositionLimit[j.idx_q]),
        )

    # ── objects ───────────────────────────────────────────────────────────────

    def add_object(
        self,
        name: str,
        shape: dict,
        pos=(0.0, 0.0, 0.0),
        quat=None,
        color=(0.6, 0.6, 0.6, 1.0),
        fixed: bool = False,
    ) -> SceneObject:
        if name in self.objects:
            raise ValueError(f"Object '{name}' already exists")
        obj = SceneObject(
            name=name,
            geometry=_make_geometry(shape),
            shape=dict(shape),
            color=tuple(color),
            fixed=fixed,
            pose=_pose_to_se3(pos, quat),
        )
        self.objects[name] = obj
        if self.viz is not None:
            self._display_object(obj)
        return obj

    def remove_object(self, name: str) -> None:
        self.objects.pop(name)
        if self.viz is not None:
            self.viz.viewer["objects"][name].delete()

    def get_object_pose(self, name: str) -> pin.SE3:
        obj = self.objects[name]
        if obj.attach_frame is not None:
            return self.get_frame_pose(obj.attach_frame) * obj.attach_offset
        return obj.pose

    def set_object_pose(self, name: str, pos, quat=None) -> None:
        obj = self.objects[name]
        if obj.attach_frame is not None:
            raise ValueError(f"Object '{name}' is attached; detach it first")
        obj.pose = _pose_to_se3(pos, quat)

    # ── attachment (grasping / pushing) ───────────────────────────────────────

    def attach_object(self, name: str, frame_name: str) -> None:
        """Rigidly attach an object to a robot frame, preserving its current
        world pose.  The object then follows the kinematic chain."""
        obj = self.objects[name]
        if obj.fixed:
            raise ValueError(f"Cannot attach fixed object '{name}'")
        world_pose = self.get_object_pose(name)
        X_WF = self.get_frame_pose(frame_name)
        obj.attach_frame = frame_name
        obj.attach_offset = X_WF.inverse() * world_pose

    def detach_object(self, name: str) -> None:
        """Detach an object, freezing it at its current world pose."""
        obj = self.objects[name]
        if obj.attach_frame is None:
            return
        obj.pose = self.get_object_pose(name)
        obj.attach_frame = None
        obj.attach_offset = pin.SE3.Identity()

    def attached_objects(self, frame_name: Optional[str] = None) -> list[str]:
        return [
            n for n, o in self.objects.items()
            if o.attach_frame is not None
            and (frame_name is None or o.attach_frame == frame_name)
        ]

    # ── collision checking ────────────────────────────────────────────────────

    def check_collisions(self, include_self: bool = False,
                         margin: float = -1e-3) -> list[tuple[str, str]]:
        """
        Return the list of colliding pairs at the current configuration:
        robot-link↔object and object↔object pairs (plus robot self-pairs of
        non-adjacent links when include_self=True).

        Objects attached to the robot are not checked against the robot
        (they are in permanent contact with the gripper by construction),
        but are checked against other objects.

        margin: Coal security margin.  The slightly negative default keeps
        resting contact (an object sitting exactly on a table) from being
        reported as a collision.
        """
        collisions: list[tuple[str, str]] = []

        robot_geoms = [
            (go.name, go.geometry, _coal_transform(self.collision_data.oMg[i]))
            for i, go in enumerate(self.collision_model.geometryObjects)
        ]
        object_geoms = {
            name: (obj.geometry, _coal_transform(self.get_object_pose(name)))
            for name, obj in self.objects.items()
        }

        # robot ↔ object
        for name, (geom, tf) in object_geoms.items():
            if self.objects[name].attach_frame is not None:
                continue
            for gname, rgeom, rtf in robot_geoms:
                if _pair_collides(rgeom, rtf, geom, tf, margin):
                    collisions.append((gname, name))

        # object ↔ object
        names = list(object_geoms)
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                a, b = names[i], names[j]
                if self.objects[a].fixed and self.objects[b].fixed:
                    continue
                ga, ta = object_geoms[a]
                gb, tb = object_geoms[b]
                if _pair_collides(ga, ta, gb, tb, margin):
                    collisions.append((a, b))

        if include_self:
            collisions.extend(self._check_self_collisions(margin, robot_geoms))

        return collisions

    def _check_self_collisions(self, margin, robot_geoms) -> list[tuple[str, str]]:
        out = []
        gos = self.collision_model.geometryObjects
        for i in range(len(gos)):
            for j in range(i + 1, len(gos)):
                ji, jj = gos[i].parentJoint, gos[j].parentJoint
                if ji == jj:
                    continue
                # Skip kinematically adjacent links (parent/child joints).
                if (self.model.parents[ji] == jj or self.model.parents[jj] == ji):
                    continue
                if _pair_collides(robot_geoms[i][1], robot_geoms[i][2],
                                  robot_geoms[j][1], robot_geoms[j][2], margin):
                    out.append((gos[i].name, gos[j].name))
        return out

    def object_distance(self, frame_name: str, obj_name: str) -> float:
        """Distance from a robot frame origin to an object's geometry origin
        (cheap proximity test used for grasp attachment)."""
        X_WF = self.get_frame_pose(frame_name)
        X_WO = self.get_object_pose(obj_name)
        return float(np.linalg.norm(X_WF.translation - X_WO.translation))

    # ── display ───────────────────────────────────────────────────────────────

    def _display_object(self, obj: SceneObject) -> None:
        import meshcat.geometry as mg

        node = self.viz.viewer["objects"][obj.name]
        r, g, b = obj.color[:3]
        a = obj.color[3] if len(obj.color) > 3 else 1.0
        material = mg.MeshLambertMaterial(
            color=(int(r * 255) << 16) + (int(g * 255) << 8) + int(b * 255),
            opacity=a,
        )
        shape = obj.shape
        if "box" in shape:
            node.set_object(mg.Box([float(v) for v in shape["box"]]), material)
        elif "sphere" in shape:
            node.set_object(mg.Sphere(float(shape["sphere"])), material)
        elif "cylinder" in shape:
            radius, length = (float(v) for v in shape["cylinder"])
            # Meshcat cylinders are y-axis aligned; URDF/Coal use z.
            node.set_object(mg.Cylinder(length, radius), material)
        elif "mesh" in shape:
            path = str(shape["mesh"])
            ext = path.rsplit(".", 1)[-1].lower()
            if ext == "obj":
                node.set_object(mg.ObjMeshGeometry.from_file(path), material)
            elif ext == "stl":
                node.set_object(mg.StlMeshGeometry.from_file(path), material)
            elif ext == "dae":
                node.set_object(mg.DaeMeshGeometry.from_file(path), material)
            else:
                raise ValueError(f"Unsupported mesh extension: {path}")
        self._update_object_transform(obj)

    def _update_object_transform(self, obj: SceneObject) -> None:
        import meshcat.transformations as mt

        T = self.get_object_pose(obj.name).homogeneous.copy()
        if "cylinder" in obj.shape:
            # Compensate the y-aligned meshcat cylinder (rotate x by 90°).
            T = T @ mt.rotation_matrix(np.pi / 2, [1, 0, 0])
        self.viz.viewer["objects"][obj.name].set_transform(T)

    def update_display(self) -> None:
        if self.viz is None:
            return
        self.viz.display(self.q)
        for obj in self.objects.values():
            self._update_object_transform(obj)

    @property
    def meshcat_url(self) -> Optional[str]:
        if self.viz is None:
            return None
        return self.viz.viewer.url()
