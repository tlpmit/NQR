"""
MujocoRobotSim — generic MuJoCo simulation backend.

Robot-specific layout (chain joint prefixes, gripper actuators, base
handling, cameras) is injected via :class:`MujocoRobotSpec`; see
``qr_robots/mujoco/rby1/sim.py`` and ``qr_robots/mujoco/spot/sim.py`` for the
concrete robots.  The public API matches the shared ZMQ protocol in
``qr_robots.common.zmq_sim``.

Modes
-----
  headless : no window; mujoco.Renderer for camera images; works with plain Python.
  display  : opens an interactive viewer via mujoco.viewer.launch_passive;
             requires ``mjpython`` on macOS.

Object dict format
------------------
  Keys name scene objects.  Values:
    str                              → free-floating body (gets a free joint)
    {"file": str, "fixed": True}     → body welded to the world (no free joint)
    {"file": str, "pos": [...], "quat": [...]} → free body with initial pose
    {"files": [str, ...], ...}       → multiple part files merged into one body
"""

from __future__ import annotations

import os
import time
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Union

import mujoco
import numpy as np

# Contact params for graspable scene objects.  Twice as stiff as the MuJoCo
# defaults (solref="0.02 1", solimp="0.9 0.95 0.001") while staying well above
# the instability threshold of ~2× timestep (0.004 s).
_OBJ_SOLREF: list[float] = [0.01, 1.0]
_OBJ_SOLIMP: list[float] = [0.95, 0.99, 0.001, 0.5, 2.0]

# Camera-frame flip between MuJoCo (-Z forward, +Y up) and OpenCV
# (+Z forward, +Y down) conventions.
_MJ_TO_CV = np.diag([1.0, -1.0, -1.0])


@dataclass
class MujocoRobotSpec:
    """Robot-specific configuration for MujocoRobotSim."""

    name: str
    robot_xml: str
    """Path to the robot MJCF (or preprocessed URDF) file."""

    chain_patterns: dict[str, str]
    """chain name → joint-name prefix, e.g. {"right_arm": "right_arm_"}."""

    gripper_act_prefixes: dict[str, tuple[str, ...]]
    """gripper side → actuator-name prefixes."""

    base_mode: str = "mocap"
    """
    "mocap"  : robot root body is welded to a mocap anchor; base trajectories
               move the anchor kinematically (holonomic, RBY1-style).
    "joints" : the model has actuated planar base joints (listed in
               base_joint_names); base trajectories drive those actuators.
    "fixed"  : no mobile base.
    """

    base_joint_names: tuple[str, ...] = ("base_x", "base_y", "base_rz")
    axle_x_offset: float = 0.0
    """x-offset of the reported base reference point (wheel-axle midpoint)
    relative to the root body frame (mocap mode only)."""

    gripper_opening_conf: bool = True
    """If True, report each gripper in the conf as a single |f1-f2| opening."""


def parse_trajectory(
    trajectory: list, waypoint_dt: float | None
) -> tuple[list[np.ndarray], list[float] | None]:
    """
    Normalize a trajectory argument into (waypoints, durations).

    Accepted forms:
      [(time, config), ...]  — caller owns the timing; durations from deltas
      [config, config, ...]  — durations are waypoint_dt each (None → caller default)
      [q0, q1, ..., qN]      — flat list of scalars: a single waypoint
    """
    if not trajectory:
        return [], []

    def _is_timed_pair(item) -> bool:
        return (
            isinstance(item, (tuple, list))
            and len(item) == 2
            and np.isscalar(item[0])
            and hasattr(item[1], "__len__")
        )

    if all(_is_timed_pair(item) for item in trajectory):
        # Times are arrival times relative to the start of the trajectory.
        times = [float(t) for t, _ in trajectory]
        arrays = [np.asarray(q, dtype=float) for _, q in trajectory]
        durations = []
        prev = 0.0
        for t in times:
            d = t - prev
            durations.append(d if d > 1e-9 else (waypoint_dt or 0.5))
            prev = t
        return arrays, durations

    if not hasattr(trajectory[0], "__len__"):
        arrays = [np.asarray(trajectory, dtype=float)]
    else:
        arrays = [np.asarray(q, dtype=float) for q in trajectory]
    durations = None if waypoint_dt is None else [waypoint_dt] * len(arrays)
    return arrays, durations


def _parse_object_entry(val: Union[str, dict]) -> tuple[list[str], bool, list, list]:
    """Return (filenames, fixed, pos, quat) from a raw objects-dict value."""
    if isinstance(val, str):
        return [val], False, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]
    files = list(val["files"]) if "files" in val else [val["file"]]
    return (
        files,
        bool(val.get("fixed", False)),
        list(val.get("pos", [0.0, 0.0, 0.0])),
        list(val.get("quat", [1.0, 0.0, 0.0, 0.0])),
    )


class MujocoRobotSim:
    """
    MuJoCo simulation with PD-controlled joints and RGB-D cameras.
    Runs headlessly (default) or with an interactive viewer window.
    """

    def __init__(
        self,
        spec: MujocoRobotSpec,
        model_dir: str,
        objects: dict | None = None,
        camera_size: tuple[int, int] = (720, 1280),
        mode: str = "headless",
    ) -> None:
        if mode not in ("headless", "display"):
            raise ValueError(f"mode must be 'headless' or 'display', got '{mode}'")

        self._spec = spec
        self._model_dir = model_dir
        self._objects = objects if objects is not None else {}
        self._camera_size = camera_size
        self._mode = mode
        self._last_sync_time: float = 0.0
        self._sync_period: float = 1 / 30.0  # cap viewer sync at 30 Hz
        self._rt_deadline: float = 0.0       # real-time pacing deadline

        self._viewer = None
        self._renderer = None
        self._build()

        if mode == "display":
            import mujoco.viewer as _mjv

            self._viewer = _mjv.launch_passive(self._model, self._data)

    # ── scene construction ────────────────────────────────────────────────────

    def _load_robot_spec(self) -> mujoco.MjSpec:
        """Load the robot model file as MjSpec.  Subclasses may override to
        preprocess the file (e.g. URDF rewriting, actuator injection)."""
        return mujoco.MjSpec.from_file(os.path.realpath(self._spec.robot_xml))

    def _build(self) -> None:
        spec, self._obj_body_names = self._build_spec()

        h, w = self._camera_size
        spec.visual.global_.offwidth = w
        spec.visual.global_.offheight = h

        self._model = spec.compile()
        self._data = mujoco.MjData(self._model)
        mujoco.mj_forward(self._model, self._data)

        self._robot_qpos_ids, self._robot_qpos_names = self._find_robot_qpos_ids()
        self._gripper_acts = self._find_gripper_acts()
        self._chain_acts = self._find_chain_acts()

        self._base_mocap_id = -1
        if self._spec.base_mode == "mocap":
            bid = mujoco.mj_name2id(
                self._model, mujoco.mjtObj.mjOBJ_BODY, "robot_base_mocap"
            )
            self._base_mocap_id = int(self._model.body_mocapid[bid])

        if self._renderer is not None:
            self._renderer.close()
        self._renderer = mujoco.Renderer(self._model, h, w)

    def _build_spec(self) -> tuple[mujoco.MjSpec, dict[str, str]]:
        spec = self._load_robot_spec()

        if self._spec.base_mode == "mocap":
            # Read the robot root name before adding anything else.
            root_name = spec.worldbody.first_body().name

            # Mocap body: kinematic anchor for the mobile base.
            # Setting data.mocap_pos/quat repositions the robot instantly;
            # the weld keeps the robot root glued to it during physics steps.
            mocap_body = spec.worldbody.add_body()
            mocap_body.name = "robot_base_mocap"
            mocap_body.mocap = True

            eq = spec.add_equality()
            eq.type = mujoco.mjtEq.mjEQ_WELD
            eq.objtype = mujoco.mjtObj.mjOBJ_BODY
            eq.name1 = "robot_base_mocap"
            eq.name2 = root_name

        obj_body_names: dict[str, str] = {}
        self._fixed_objects: set[str] = set()

        for obj_name, obj_val in self._objects.items():
            if obj_name == "robot":
                continue
            obj_files, fixed, pos, quat = _parse_object_entry(obj_val)
            if fixed:
                self._fixed_objects.add(obj_name)
            obj_paths = [
                p if os.path.isabs(p) else os.path.realpath(os.path.join(self._model_dir, p))
                for p in obj_files
            ]
            child, body_name = self._load_object_spec(obj_paths, obj_name, fixed=fixed)
            prefix = f"{obj_name}/"
            obj_body_names[obj_name] = f"{prefix}{body_name}"

            frame = spec.worldbody.add_frame()
            frame.name = f"{obj_name}_attach_frame"
            frame.pos = pos
            frame.quat = quat
            spec.attach(child, frame=frame, prefix=prefix)

        return spec, obj_body_names

    @staticmethod
    def _spec_strip_free_joints(path: str) -> mujoco.MjSpec:
        """Load an XML file as MjSpec with free joints removed from all bodies.

        MjsJoint has no delete() in this version of MuJoCo, so we strip them
        at the XML level via ElementTree and reload from a temp file.
        """
        import tempfile
        import xml.etree.ElementTree as ET

        tree = ET.parse(path)
        xml_root = tree.getroot()
        stripped = False
        for body in xml_root.iter('body'):
            for el in list(body):
                if el.tag == 'freejoint' or (
                    el.tag == 'joint' and el.get('type') == 'free'
                ):
                    body.remove(el)
                    stripped = True
        if not stripped:
            return mujoco.MjSpec.from_file(path)
        # Write to a temp file in the same directory so relative paths and
        # meshdir resolve identically to the original.
        with tempfile.NamedTemporaryFile(
            mode='w', suffix='.xml', delete=False, dir=os.path.dirname(path)
        ) as fh:
            tree.write(fh, encoding='unicode')
            tmp_path = fh.name
        try:
            return mujoco.MjSpec.from_file(tmp_path)
        finally:
            os.unlink(tmp_path)

    @staticmethod
    def _load_object_spec(
        paths: list[str],
        obj_name: str,
        fixed: bool = False,
    ) -> tuple[mujoco.MjSpec, str]:
        child = MujocoRobotSim._spec_strip_free_joints(paths[0])
        root = child.worldbody.first_body()

        if root is None:
            root = child.worldbody.add_body()
            root.name = obj_name
            g = root.add_geom()
            g.type = mujoco.mjtGeom.mjGEOM_BOX
            g.size = [0.05, 0.05, 0.05]

        root.name = obj_name

        if not fixed:
            for g in root.geoms:
                g.solref = _OBJ_SOLREF
                g.solimp = _OBJ_SOLIMP
                g.condim = 6
                g.friction = [3.0, 0.05, 0.01]

        # Attach additional parts as fixed children of the root body.
        for i, path in enumerate(paths[1:], 1):
            part = MujocoRobotSim._spec_strip_free_joints(path)
            part_root = part.worldbody.first_body()
            if part_root is not None and not fixed:
                for g in part_root.geoms:
                    g.solref = _OBJ_SOLREF
                    g.solimp = _OBJ_SOLIMP
                    g.condim = 6
                    g.friction = [3.0, 0.05, 0.01]
            frame = root.add_frame()
            child.attach(part, frame=frame, prefix=f"_p{i}/")

        if not fixed:
            fj = root.add_freejoint()
            fj.name = f"{obj_name}_free"

        return child, root.name

    # ── index helpers ─────────────────────────────────────────────────────────

    def _find_robot_qpos_ids(self) -> tuple[np.ndarray, list[str]]:
        m = self._model
        ids = []
        names = []
        for i in range(m.njnt):
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) or ""
            if "/" in name:   # object joints are prefixed "objname/"
                continue
            if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE:
                continue
            ids.append(m.jnt_qposadr[i])
            names.append(name)
        return np.array(ids, dtype=int), names

    def _find_chain_acts(self) -> dict[str, list[int]]:
        m = self._model
        chain_acts: dict[str, list[tuple[int, int]]] = defaultdict(list)
        gripper_act_ids = {a for acts in self._gripper_acts.values() for a in acts}

        for i in range(m.nu):
            if m.actuator_trntype[i] != mujoco.mjtTrn.mjTRN_JOINT:
                continue
            if i in gripper_act_ids:   # grippers are not chains
                continue
            jid = m.actuator_trnid[i, 0]
            jname = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, jid) or ""
            for chain, prefix in self._spec.chain_patterns.items():
                if jname.startswith(prefix):
                    chain_acts[chain].append((jid, i))
                    break
            else:
                if (self._spec.base_mode == "joints"
                        and jname in self._spec.base_joint_names):
                    chain_acts["base"].append((jid, i))

        return {
            chain: [act_id for _, act_id in sorted(pairs)]
            for chain, pairs in chain_acts.items()
        }

    def _find_gripper_acts(self) -> dict[str, list[int]]:
        m = self._model
        grippers: dict[str, list[int]] = defaultdict(list)

        for i in range(m.nu):
            aname = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or ""
            for side, prefixes in self._spec.gripper_act_prefixes.items():
                if any(aname.startswith(p) for p in prefixes):
                    grippers[side].append(i)
                    break

        return dict(grippers)

    # ── physics step ──────────────────────────────────────────────────────────

    def _mj_step(self) -> None:
        mujoco.mj_step(self._model, self._data)
        if self._viewer is not None:
            # Pace to real time so motions are watchable: headless physics
            # runs as fast as the CPU allows (a 0.5 s trajectory finishes in
            # milliseconds).  The deadline resets after idle gaps between
            # commands so we never try to "catch up".
            now = time.monotonic()
            if now - self._rt_deadline > 0.25:
                self._rt_deadline = now
            self._rt_deadline += self._model.opt.timestep
            if self._rt_deadline > now:
                time.sleep(self._rt_deadline - now)
                now = self._rt_deadline
            if now - self._last_sync_time >= self._sync_period:
                self._viewer.sync()
                self._last_sync_time = now

    # ── base pose helpers ─────────────────────────────────────────────────────

    def _get_base_pose(self) -> np.ndarray:
        """Return [x, y, theta] of the base reference point in the world."""
        if self._spec.base_mode == "mocap":
            pos = self._data.mocap_pos[self._base_mocap_id]
            quat = self._data.mocap_quat[self._base_mocap_id]  # (w, x, y, z)
            theta = 2.0 * np.arctan2(float(quat[3]), float(quat[0]))
            off = self._spec.axle_x_offset
            return np.array([
                float(pos[0]) + off * np.cos(theta),
                float(pos[1]) + off * np.sin(theta),
                theta,
            ])
        if self._spec.base_mode == "joints":
            qs = []
            for jname in self._spec.base_joint_names:
                jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, jname)
                qs.append(float(self._data.qpos[self._model.jnt_qposadr[jid]]))
            return np.array(qs)
        return np.zeros(0)

    # ── public API ────────────────────────────────────────────────────────────

    def get_robot_conf(self) -> np.ndarray:
        """
        Return the full robot configuration.

        mocap base : [x, y, theta] (axle midpoint) followed by all non-free,
                     non-object joint positions in model order.
        joints base: all non-free, non-object joint positions in model order
                     (the base joints appear at their model positions).
        """
        mujoco.mj_kinematics(self._model, self._data)
        joints = self._data.qpos[self._robot_qpos_ids]
        if self._spec.base_mode == "mocap":
            return np.concatenate([self._get_base_pose(), joints])
        return np.array(joints)

    def set_robot_conf(self, conf: np.ndarray) -> None:
        """Inverse of get_robot_conf: teleport the robot to a configuration
        and reset the actuator targets to hold it there."""
        conf = np.asarray(conf, dtype=float)
        if self._spec.base_mode == "mocap":
            x, y, theta = conf[:3]
            off = self._spec.axle_x_offset
            self._data.mocap_pos[self._base_mocap_id] = [
                x - off * np.cos(theta), y - off * np.sin(theta), 0.0
            ]
            c, s = np.cos(theta / 2), np.sin(theta / 2)
            self._data.mocap_quat[self._base_mocap_id] = [c, 0.0, 0.0, s]
            joints = conf[3:]
        else:
            joints = conf
        self._data.qpos[self._robot_qpos_ids] = joints
        # Reset actuator targets so PD controllers hold the new configuration.
        for act_ids in list(self._chain_acts.values()) + list(self._gripper_acts.values()):
            for act_id in act_ids:
                jid = self._model.actuator_trnid[act_id, 0]
                self._data.ctrl[act_id] = self._data.qpos[self._model.jnt_qposadr[jid]]
        mujoco.mj_forward(self._model, self._data)

    def get_world_state(self) -> dict:
        """
        Returns::

            {
              "block": np.array([x, y, z, qw, qx, qy, qz]),   # free objects
              "table": np.array([x, y, z, qw, qx, qy, qz]),   # fixed objects too
              "robot": np.array([q0, ..., qN]),
            }

        Quaternions are in MuJoCo's (w, x, y, z) convention.
        """
        mujoco.mj_kinematics(self._model, self._data)
        state: dict = {}

        for obj_name, body_name in self._obj_body_names.items():
            bid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            pos = self._data.xpos[bid].copy()
            quat = self._data.xquat[bid].copy()
            state[obj_name] = np.concatenate([pos, quat])

        state["robot"] = self.get_robot_conf()
        return state

    def get_camera_intrinsics(self, camera_name: str) -> np.ndarray:
        """Return 3×3 intrinsics matrix matching MuJoCo's actual rendering.

        MuJoCo always places the principal point at the image centre and uses
        square pixels, so cx=W/2, cy=H/2, fx=fy=H/(2·tan(fovy/2)).
        """
        cam_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        if cam_id == -1:
            raise ValueError(f"Camera '{camera_name}' not found in the model")
        fovy_deg = float(self._model.cam_fovy[cam_id])
        H, W = self._camera_size
        f = H / (2.0 * np.tan(np.radians(fovy_deg / 2.0)))
        cx, cy = W / 2.0, H / 2.0
        return np.array([[f, 0.0, cx], [0.0, f, cy], [0.0, 0.0, 1.0]])

    def get_camera_extrinsics(self, camera_name: str) -> np.ndarray:
        """Return the 4×4 world-to-camera transform, OpenCV convention
        (+Z forward, +X right, +Y down)."""
        cam_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        if cam_id == -1:
            raise ValueError(f"Camera '{camera_name}' not found in the model")
        mujoco.mj_kinematics(self._model, self._data)
        mujoco.mj_camlight(self._model, self._data)
        R_wc = self._data.cam_xmat[cam_id].reshape(3, 3) @ _MJ_TO_CV
        t_wc = self._data.cam_xpos[cam_id]
        X_WC = np.eye(4)
        X_WC[:3, :3] = R_wc
        X_WC[:3, 3] = t_wc
        return np.linalg.inv(X_WC)

    def get_camera_image(self, camera_name: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return (rgb uint8 HxWx3, depth float32 HxW, label int HxW).

        Label pixels hold body ids; background, floor, and table bodies are -1.
        """
        cam_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        if cam_id == -1:
            raise ValueError(f"Camera '{camera_name}' not found in the model")

        mujoco.mj_forward(self._model, self._data)

        self._renderer.disable_depth_rendering()
        self._renderer.update_scene(self._data, camera=camera_name)
        rgb = self._renderer.render().copy()

        self._renderer.enable_depth_rendering()
        self._renderer.update_scene(self._data, camera=camera_name)
        depth = self._renderer.render().copy()

        self._renderer.enable_segmentation_rendering()
        self._renderer.update_scene(self._data, camera=camera_name)
        seg = self._renderer.render()  # shape (H, W, 2)
        self._renderer.disable_segmentation_rendering()

        # Cross-backend label convention: positive body ids only for free
        # scene objects; the robot, fixed objects (fixtures/surfaces), and
        # the background are -1.  Surfaces are found geometrically by the
        # perception pipeline, not via labels.
        n_bodies = self._model.nbody
        keep = np.zeros(n_bodies, dtype=bool)
        for obj_name in self._obj_body_names:
            if obj_name in self._fixed_objects:
                continue
            prefix = f"{obj_name}/"
            for i in range(n_bodies):
                if self._model.body(i).name.startswith(prefix):
                    keep[i] = True
        # Channel 0 = geom ID (-1 for background), Channel 1 = object type
        geom_ids = seg[:, :, 0]
        body_ids = np.where(geom_ids >= 0, self._model.geom_bodyid[geom_ids], -1)

        # clip prevents -1 from indexing into keep; the body_ids >= 0 guard
        # ensures those pixels still get -1 in the output
        safe = np.clip(body_ids, 0, n_bodies - 1)
        label = np.where((body_ids >= 0) & keep[safe], body_ids, -1)

        return rgb, depth, label

    def execute_chain_trajectory(
        self,
        chain_name: str,
        trajectory: list,
        waypoint_dt: float | None = None,
    ) -> bool:
        if waypoint_dt is None:
            waypoint_dt = 0.1
        if chain_name == "base" and self._spec.base_mode == "mocap":
            return self.execute_base_trajectory(trajectory, waypoint_dt)
        if chain_name not in self._chain_acts:
            raise ValueError(
                f"Unknown chain '{chain_name}'. Available: {sorted(self._chain_acts)}"
            )

        act_ids = self._chain_acts[chain_name]
        m, d = self._model, self._data
        dt = m.opt.timestep

        arrays, durations = parse_trajectory(trajectory, waypoint_dt)

        q_start = np.array([d.ctrl[act_id] for act_id in act_ids], dtype=float)

        for q_wp, dur in zip(arrays, durations):
            if len(q_wp) != len(act_ids):
                raise ValueError(
                    f"Waypoint length {len(q_wp)} ≠ chain DOF {len(act_ids)} "
                    f"for '{chain_name}'"
                )
            q_end = np.array(
                [float(np.clip(q, *m.actuator_ctrlrange[act_id]))
                 for act_id, q in zip(act_ids, q_wp)],
                dtype=float,
            )
            steps = max(1, round(dur / dt))
            for i in range(steps):
                alpha = (i + 1) / steps
                q_i = q_start + alpha * (q_end - q_start)
                for act_id, q in zip(act_ids, q_i):
                    d.ctrl[act_id] = q
                self._mj_step()
            q_start = q_end

        # Settle: hold the final target until the PD controllers catch up
        # (matches the Drake backend's advance-until-near behavior).
        qaddrs = [m.jnt_qposadr[m.actuator_trnid[a, 0]] for a in act_ids]
        dofadrs = [m.jnt_dofadr[m.actuator_trnid[a, 0]] for a in act_ids]
        max_extra = max(1, round(4.0 / dt))
        for _ in range(max_extra):
            err = np.max(np.abs(d.qpos[qaddrs] - q_start))
            vel = np.max(np.abs(d.qvel[dofadrs]))
            if err < 0.02 and vel < 0.05:
                break
            self._mj_step()

        return True

    def execute_base_trajectory(
        self, trajectory: list, waypoint_dt: float = 0.5
    ) -> bool:
        """
        Drive the robot base through [x, y, theta] waypoints.

        mocap mode : smoothly interpolate the mocap anchor over physics steps
                     so arm joints stay settled via PD control throughout.
        joints mode: drive the actuated base joints like a chain trajectory.
        """
        if waypoint_dt is None:
            waypoint_dt = 0.5
        if self._spec.base_mode == "joints":
            return self.execute_chain_trajectory("base", trajectory, waypoint_dt)
        if self._spec.base_mode != "mocap":
            raise ValueError("This robot has no mobile base")

        waypoints, _ = parse_trajectory(trajectory, waypoint_dt)

        mid = self._base_mocap_id
        dt = self._model.opt.timestep
        off = self._spec.axle_x_offset

        cur_ax, cur_ay, cur_theta = self._get_base_pose()

        for wp in waypoints:
            if len(wp) != 3:
                raise ValueError(
                    f"Base waypoint must be [x, y, theta] (length 3), got {len(wp)}"
                )
            tgt_ax, tgt_ay, tgt_theta = float(wp[0]), float(wp[1]), float(wp[2])

            steps = max(1, round(waypoint_dt / dt))
            for i in range(steps):
                alpha = (i + 1) / steps
                th = cur_theta + alpha * (tgt_theta - cur_theta)
                ax = cur_ax + alpha * (tgt_ax - cur_ax)
                ay = cur_ay + alpha * (tgt_ay - cur_ay)
                # Back-compute base body origin from axle center.
                self._data.mocap_pos[mid] = [ax - off * np.cos(th),
                                             ay - off * np.sin(th),
                                             0.0]
                c, s = np.cos(th / 2), np.sin(th / 2)
                self._data.mocap_quat[mid] = [c, 0.0, 0.0, s]
                self._mj_step()

            cur_ax = tgt_ax
            cur_ay = tgt_ay
            cur_theta = tgt_theta

        return True

    def execute_gripper_command(
        self,
        side: str,
        command: Union[str, float],
        settle_secs: float = 1.0,
    ) -> bool:
        sides = list(self._gripper_acts) if side == "both" else [side]

        for s in sides:
            if s not in self._gripper_acts:
                raise ValueError(f"No gripper actuator found for side '{s}'")
            for act_id in self._gripper_acts[s]:
                self._data.ctrl[act_id] = self._gripper_ctrl(act_id, command)

        steps = max(1, round(settle_secs / self._model.opt.timestep))
        for _ in range(steps):
            self._mj_step()

        return True

    def _gripper_ctrl(self, act_id: int, command: Union[str, float]) -> float:
        m = self._model
        jid = m.actuator_trnid[act_id, 0]
        jlo, jhi = m.jnt_range[jid]
        alo, ahi = m.actuator_ctrlrange[act_id]
        upper_is_zero = abs(jhi) <= abs(jlo)

        if command == "close":
            return float(ahi if upper_is_zero else alo)
        if command == "open":
            return float(alo if upper_is_zero else ahi)
        frac = float(np.clip(command, 0.0, 1.0))
        if upper_is_zero:
            return float(alo + (ahi - alo) * frac)
        return float(ahi - (ahi - alo) * frac)

    def set_object_pose(
        self,
        obj_name: str,
        pos: np.ndarray,
        quat: np.ndarray | None = None,
    ) -> None:
        if obj_name not in self._obj_body_names:
            raise ValueError(f"Unknown object '{obj_name}'")
        body_name = self._obj_body_names[obj_name]
        m = self._model
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_name)

        for i in range(m.njnt):
            if m.jnt_bodyid[i] == bid and m.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE:
                qadr = m.jnt_qposadr[i]
                self._data.qpos[qadr: qadr + 3] = np.asarray(pos, dtype=float)
                q = (
                    np.asarray(quat, dtype=float)
                    if quat is not None
                    else np.array([1.0, 0.0, 0.0, 0.0])
                )
                self._data.qpos[qadr + 3: qadr + 7] = q
                mujoco.mj_forward(m, self._data)
                return

        raise ValueError(
            f"Object '{obj_name}' has no free joint — it may be a fixed object"
        )

    def step(self, n: int = 1) -> None:
        for _ in range(n):
            self._mj_step()

    def point_head_at(
        self,
        target_world_pos: np.ndarray,
        camera_name: str = "head_camera",
        max_iter: int = 200,
        tol: float = 0.01,
        waypoint_dt: float = 0.8,
    ) -> bool:
        """
        Pan/tilt the head chain so the named camera points at a world position.

        Uses finite-difference Jacobian + damped-least-squares IK over the FK
        (no physics steps during search).  Executes the found angles as a
        trajectory so the PD controllers drive the joints smoothly.
        """
        target = np.asarray(target_world_pos, dtype=float)
        cam_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        if cam_id < 0:
            raise ValueError(f"Camera '{camera_name}' not found")

        head_acts = self._chain_acts.get("head", [])
        if not head_acts:
            raise ValueError("No 'head' chain found in this model")

        m, d = self._model, self._data
        dof = len(head_acts)
        qaddrs = [m.jnt_qposadr[m.actuator_trnid[a, 0]] for a in head_acts]
        lo = np.array([m.actuator_ctrlrange[a][0] for a in head_acts])
        hi = np.array([m.actuator_ctrlrange[a][1] for a in head_acts])

        qpos_save = d.qpos.copy()

        def _set_fk(angles: np.ndarray) -> None:
            for qadr, ang in zip(qaddrs, angles):
                d.qpos[qadr] = float(ang)
            mujoco.mj_forward(m, d)

        def _fwd() -> np.ndarray:
            return (-d.cam_xmat[cam_id].reshape(3, 3)[:, 2]).copy()

        angles = np.clip([d.qpos[q] for q in qaddrs], lo, hi)
        eps = 1e-4

        for _ in range(max_iter):
            _set_fk(angles)
            fwd = _fwd()
            to_t = target - d.cam_xpos[cam_id]
            dist = np.linalg.norm(to_t)
            if dist < 1e-6:
                break
            err = to_t / dist - fwd
            if np.linalg.norm(err) < tol:
                break

            # Finite-difference Jacobian  ∂fwd/∂angle_i  (3 × dof)
            J = np.zeros((3, dof))
            for i in range(dof):
                ap = angles.copy()
                ap[i] += eps
                _set_fk(ap)
                J[:, i] = (_fwd() - fwd) / eps
            _set_fk(angles)

            # Damped least squares
            lam = 1e-3
            delta = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(3), err) * 0.5
            angles = np.clip(angles + delta, lo, hi)

        # Restore physics state before executing
        d.qpos[:] = qpos_save
        mujoco.mj_forward(m, d)
        return self.execute_chain_trajectory("head", [angles], waypoint_dt=waypoint_dt)

    def reload(self, file_updates: dict[str, str] | None = None) -> None:
        """
        Optionally write new content to files in model_dir, then rebuild the
        MuJoCo model, data, renderer, and all indices from disk.
        Simulation state is reset to the initial pose.
        """
        if file_updates:
            for rel_path, content in file_updates.items():
                full_path = os.path.join(self._model_dir, rel_path)
                with open(full_path, "w") as fh:
                    fh.write(content)

        self._build()

        if self._viewer is not None:
            self._viewer.close()
            import mujoco.viewer as _mjv

            self._viewer = _mjv.launch_passive(self._model, self._data)

    def idle(self, seconds: float) -> None:
        """Keep the passive viewer in sync while no request is being served."""
        if self._viewer is not None:
            self._viewer.sync()

    def close(self) -> None:
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None

    @property
    def is_running(self) -> bool:
        """False when the display window has been closed by the user."""
        if self._viewer is None:
            return True
        return self._viewer.is_running()

    @property
    def model(self) -> mujoco.MjModel:
        return self._model

    @property
    def data(self) -> mujoco.MjData:
        return self._data

    @property
    def chains(self) -> list[str]:
        chains = list(self._chain_acts.keys())
        if self._spec.base_mode == "mocap":
            chains.append("base")
        return sorted(chains)

    @property
    def object_names(self) -> list[str]:
        return list(self._obj_body_names)
