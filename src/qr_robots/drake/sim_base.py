"""
DrakeRobotSim — generic Drake simulation backend.

Exposes the same API as MujocoRobotSim (see qr_robots.common.zmq_sim for the
shared protocol) so the same ZMQ client works against either backend.
Robot-specific layout is injected via :class:`DrakeRobotSpec`; see
``qr_robots/drake/rby1/sim.py`` and ``qr_robots/drake/spot/sim.py``.

Object dict format matches the MuJoCo backend, but object files must be
URDF/SDF (Drake cannot parse MJCF objects):
    str                              → free-floating body
    {"file": str, "fixed": True}     → body welded to world
    {"file": str, "pos": [...], "quat": [...]} → free body with initial pose
    {"files": [str, ...], ...}       → multiple part files merged into one body
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Callable, Union

import numpy as np

from pydrake.all import (
    CameraInfo,
    Context,
    DiagramBuilder,
    Meshcat,
    MeshcatParams,
    ModelInstanceIndex,
    Parser,
    PiecewisePolynomial,
    Quaternion,
    RgbdSensor,
    RigidTransform,
    RobotDiagram,
    Simulator,
    yaml_load_file,
)
from manipulation.station import LoadScenario, MakeHardwareStation

from qr_robots.drake.desired_state_source import DesiredStateSource
from qr_robots.mujoco.sim_base import _parse_object_entry, parse_trajectory
from qr_utils import GetQRPackageXmls

# Maximum joint velocity (rad/s) enforced by the URDF limits.
# Used to auto-compute waypoint_dt when it is not provided by the caller.
_JOINT_VEL_LIMIT: float = np.pi
_TRAJ_VEL_FRACTION: float = 0.7   # use 70 % of the limit for a comfortable margin
_MIN_WAYPOINT_DT: float = 0.5     # never shorter than this even for tiny moves

# Base velocity limits for auto-timed base trajectories.  The inverse-dynamics
# base controller tracks poorly when asked to move faster than this.
_BASE_LIN_VEL: float = 0.4   # m/s
_BASE_ANG_VEL: float = 0.8   # rad/s


@dataclass
class DrakeRobotSpec:
    """Robot-specific configuration for DrakeRobotSim."""

    name: str

    scenario_files: list[str]
    """Scenario/directives YAML files, merged in order (later override earlier)."""

    groups: list[tuple[str, int]]
    """Station input-port groups: (group_name, num_positions), concatenated in
    this order to form the full Drake position vector."""

    chain_slices: dict[str, slice]
    """chain name → slice of the full Drake position vector."""

    conf_from_q: Callable[[np.ndarray], np.ndarray]
    """Drake position vector → canonical robot conf vector."""

    base_to_drake: Callable[[float, float, float], np.ndarray]
    """Canonical base pose [x, y, theta] → Drake base positions."""

    gripper_positions: Callable[[Union[str, float]], np.ndarray]
    """Gripper command ('open'/'close'/fraction) → gripper joint positions."""

    gripper_sides: tuple[str, ...] = ("right", "left")

    cameras: dict[str, str] = field(default_factory=dict)
    """protocol camera name (e.g. "head_camera") → scenario camera name ("head")."""

    head_chain: str | None = None
    """Chain used by point_head_at, or None if the robot has no head."""

    camera_mount_bodies: dict[str, str] = field(default_factory=dict)
    """scenario camera name → plant body the sensor is mounted on
    (used by point_head_at)."""


class DrakeRobotSim:
    """
    Drake simulation backend.  API mirrors MujocoRobotSim so the same ZMQ
    dispatch table works for both backends.
    """

    _SETTLE_SECS = 1.0   # seconds to advance after building the diagram
    _SIM_DT = 0.05       # seconds per advance increment during motion

    def __init__(
        self,
        spec: DrakeRobotSpec,
        model_dir: str,
        objects: dict | None = None,
        mode: str = "headless",
    ) -> None:
        if mode not in ("headless", "display"):
            raise ValueError(f"mode must be 'headless' or 'display', got '{mode}'")
        self._robot_spec = spec
        self._mode = mode
        self._model_dir = model_dir
        self._objects = objects if objects is not None else {}

        # Track object → ModelInstanceIndex for get_world_state / set_object_pose
        self._obj_instances: dict[str, ModelInstanceIndex] = {}
        # Free-body initial poses deferred until after Initialize()
        self._initial_obj_poses: dict[str, tuple] = {}
        # Per-camera (X_BC, CameraInfo) captured during station construction
        self._camera_info: dict[str, tuple] = {}

        self._meshcat = self._make_meshcat() if mode == "display" else None

        builder = DiagramBuilder()
        station: RobotDiagram = builder.AddNamedSystem(
            "station", self._make_station(self._objects, self._meshcat)
        )
        source = builder.AddNamedSystem(
            "desired_state", DesiredStateSource(spec.groups)
        )
        for group, _n in spec.groups:
            builder.Connect(source.GetOutputPort(f"{group}_desired_state"),
                            station.GetInputPort(f"{group}.desired_state"))

        diagram = builder.Build()
        self._diagram = diagram
        self._station = diagram.GetSubsystemByName("station")
        self._source = diagram.GetSubsystemByName("desired_state")
        self._n_total = self._source.n_total

        self._simulator = Simulator(diagram)
        self._context = self._simulator.get_mutable_context()
        if mode == "display":
            # Pace to real time so motions are watchable in Meshcat.
            self._simulator.set_target_realtime_rate(1.0)
        self._simulator.Initialize()

        # Set desired state to match the plant's default so PD controllers
        # see zero error from the start.
        q0 = self._get_joint_positions()
        self._source.set_hold(self._context, q0)

        # Advance briefly to settle robot physics
        self._simulator.AdvanceTo(self._SETTLE_SECS)
        q0 = self._get_joint_positions()
        self._source.set_hold(self._context, q0)

        # Apply deferred free-body initial poses after settling — Finalize() has
        # now run (floating joints exist) and the settle phase is done, so objects
        # appear at their correct world positions when the server starts serving.
        self._apply_initial_object_poses()

    # ── construction helpers ──────────────────────────────────────────────────

    def _make_meshcat(self) -> Meshcat:
        params = MeshcatParams()
        params.initial_properties = [
            MeshcatParams.PropertyTuple(
                path="/Axes", property="visible", value=False
            ),
        ]
        return Meshcat(params)

    def _make_station(self, objects: dict, meshcat) -> RobotDiagram:
        data: dict = {}
        for path in self._robot_spec.scenario_files:
            data.update(yaml_load_file(filename=str(path)))
        scenario = LoadScenario(data=str(data))
        scenario.visualization.publish_period = 1 / 30.0  # cap Meshcat at 30 Hz

        camera_names = list(getattr(scenario, "cameras", {}).keys())

        # Captured via closures during station construction.
        obj_instances: dict[str, ModelInstanceIndex] = {}
        camera_info: dict[str, tuple] = {}

        def preload(parser: Parser):
            for name, val in objects.items():
                if name == "robot":
                    continue
                file_paths, fixed, pos, quat_wxyz = _parse_object_entry(val)
                resolved = [
                    fp if os.path.isabs(fp)
                    else os.path.realpath(os.path.join(self._model_dir, fp))
                    for fp in file_paths
                ]
                parser.SetAutoRenaming(True)

                # Load primary part and record its instance for state queries.
                instances = parser.AddModels(resolved[0])
                instance = instances[0]
                obj_instances[name] = instance

                plant = parser.plant()
                primary_body = plant.get_body(plant.GetBodyIndices(instance)[0])
                X = RigidTransform(Quaternion(np.asarray(quat_wxyz, float)),
                                   np.asarray(pos, float))
                if fixed:
                    plant.WeldFrames(plant.world_frame(), primary_body.body_frame(), X)
                else:
                    # Floating joint doesn't exist yet (Finalize hasn't run),
                    # so defer pose setting until after simulator.Initialize().
                    self._initial_obj_poses[name] = (instance, X)

                # Load additional parts and weld them rigidly to the primary body
                # (or to world for fixed objects) so the assembly moves as one.
                for extra_path in resolved[1:]:
                    extra_instances = parser.AddModels(extra_path)
                    extra_body = plant.get_body(
                        plant.GetBodyIndices(extra_instances[0])[0]
                    )
                    if fixed:
                        plant.WeldFrames(
                            plant.world_frame(), extra_body.body_frame(), X
                        )
                    else:
                        plant.WeldFrames(
                            primary_body.body_frame(), extra_body.body_frame(),
                            RigidTransform()
                        )

        def prebuild(builder: DiagramBuilder):
            for cam in camera_names:
                sensor: RgbdSensor = builder.GetSubsystemByName(f"rgbd_sensor_{cam}")
                builder.ExportOutput(
                    sensor.body_pose_in_world_output_port(), f"{cam}.X_WB"
                )
                color_core = sensor.default_color_render_camera().core()
                camera_info[cam] = (
                    color_core.sensor_pose_in_camera_body(),  # X_BC
                    color_core.intrinsics(),                  # CameraInfo
                )

        station = MakeHardwareStation(
            scenario, meshcat,
            package_xmls=GetQRPackageXmls(),
            parser_preload_callback=preload,
            prebuild_callback=prebuild,
        )
        self._obj_instances = obj_instances
        self._camera_info = camera_info
        return station

    def _apply_initial_object_poses(self) -> None:
        if not self._initial_obj_poses:
            return
        plant = self._station.plant()
        plant_ctx = plant.GetMyContextFromRoot(self._context)
        for obj_name, (instance, X) in self._initial_obj_poses.items():
            body_indices = plant.GetBodyIndices(instance)
            if body_indices:
                body = plant.get_body(body_indices[0])
                plant.SetFreeBodyPose(plant_ctx, body, X)

    # ── joint state helpers ───────────────────────────────────────────────────

    def _station_ctx(self) -> Context:
        return self._station.GetMyContextFromRoot(self._context)

    def _get_joint_positions(self) -> np.ndarray:
        """Return the full Drake position vector (groups concatenated)."""
        sc = self._station_ctx()
        parts = []
        for group, n in self._robot_spec.groups:
            st = self._station.GetOutputPort(f"{group}.state_estimated").Eval(sc)
            parts.append(st[:n])
        return np.concatenate(parts)

    def _get_desired_positions(self) -> np.ndarray:
        return self._source.get_desired_q(self._context)

    def _drake_camera(self, camera_name: str) -> str:
        cameras = self._robot_spec.cameras
        if camera_name in cameras:
            return cameras[camera_name]
        # Accept the scenario name directly ("head_camera" → "head" fallback).
        short = camera_name.split("_")[0]
        if short in self._camera_info:
            return short
        raise ValueError(
            f"Camera '{camera_name}' not found. Available: {sorted(cameras)}"
        )

    # ── trajectory utilities ──────────────────────────────────────────────────

    def _build_and_set_trajectory(
        self,
        drake_slice: slice,
        arrays: list[np.ndarray],
        durations: list[float],
    ) -> tuple[float, np.ndarray]:
        """
        Build a PiecewisePolynomial for the full robot that interpolates the
        given chain's waypoints.  Returns (t_goal, q_goal).
        """
        t_curr = self._context.get_time()
        q_des = self._get_desired_positions()

        times_traj = [0.0]
        qs_traj = [q_des.copy()]

        for arr, dur in zip(arrays, durations):
            q_next = qs_traj[-1].copy()
            q_next[drake_slice] = arr
            times_traj.append(times_traj[-1] + dur)
            qs_traj.append(q_next)

        ts = np.array(times_traj)
        qs = np.column_stack(qs_traj)
        traj_q = PiecewisePolynomial.FirstOrderHold(ts, qs)
        traj_qdot = traj_q.derivative()

        self._source.set_trajectory(self._context, traj_q, traj_qdot, t_curr)
        return t_curr + ts[-1], qs_traj[-1]

    def _advance_until_near(
        self,
        t_goal: float,
        q_goal: np.ndarray,
        relevant_slice: slice,
        tol: float = 0.05,
        max_extra: float = 30.0,
    ) -> None:
        t_max = t_goal + max_extra
        while True:
            t = self._context.get_time()
            if t >= t_max:
                q_curr = self._get_joint_positions()
                errs = np.abs(q_curr[relevant_slice] - q_goal[relevant_slice])
                bad = np.where(errs >= tol)[0]
                if bad.size:
                    lines = [
                        f"  joint[{relevant_slice.start + i}]: "
                        f"goal={q_goal[relevant_slice.start + i]:.4f}  "
                        f"actual={q_curr[relevant_slice.start + i]:.4f}  "
                        f"err={errs[i]:.4f}"
                        for i in bad
                    ]
                    print(
                        f"[drake_sim] WARNING: timed out after {max_extra:.0f}s extra; "
                        f"{bad.size} joint(s) outside tol={tol}:\n" + "\n".join(lines),
                        flush=True,
                    )
                break
            self._simulator.AdvanceTo(min(t + self._SIM_DT, t_max))
            if self._context.get_time() >= t_goal:
                q_curr = self._get_joint_positions()
                if np.max(np.abs(q_curr[relevant_slice] - q_goal[relevant_slice])) < tol:
                    break

    def _auto_waypoint_dt(
        self,
        drake_slice: slice,
        arrays: list[np.ndarray],
    ) -> list[float]:
        """
        Compute per-segment durations from joint-velocity limits:
        dt = max(|Δq|) / (vel_limit × fraction), clamped to at least
        _MIN_WAYPOINT_DT so tiny moves still give the PD controller time
        to settle.
        """
        q_prev = self._get_desired_positions()[drake_slice]
        durations: list[float] = []
        for arr in arrays:
            max_delta = float(np.max(np.abs(arr - q_prev)))
            dt = max_delta / (_JOINT_VEL_LIMIT * _TRAJ_VEL_FRACTION)
            durations.append(max(dt, _MIN_WAYPOINT_DT))
            q_prev = arr
        return durations

    # ── public API (matches MujocoRobotSim) ───────────────────────────────────

    def get_robot_conf(self) -> np.ndarray:
        """Canonical-format robot configuration."""
        return self._robot_spec.conf_from_q(self._get_joint_positions())

    def get_world_state(self) -> dict:
        plant = self._station.plant()
        plant_ctx = plant.GetMyContextFromRoot(self._context)
        state: dict = {}
        for obj_name, instance in self._obj_instances.items():
            body = plant.get_body(plant.GetBodyIndices(instance)[0])
            X_WB = plant.EvalBodyPoseInWorld(plant_ctx, body)
            pos = X_WB.translation()
            quat = X_WB.rotation().ToQuaternion()
            state[obj_name] = np.array([
                pos[0], pos[1], pos[2],
                quat.w(), quat.x(), quat.y(), quat.z(),
            ])
        state["robot"] = self.get_robot_conf()
        return state

    def get_camera_intrinsics(self, camera_name: str) -> np.ndarray:
        cam = self._drake_camera(camera_name)
        ci: CameraInfo = self._camera_info[cam][1]
        return np.array([
            [ci.focal_x(), 0.0,          ci.center_x()],
            [0.0,          ci.focal_y(), ci.center_y()],
            [0.0,          0.0,          1.0],
        ])

    def get_camera_extrinsics(self, camera_name: str) -> np.ndarray:
        """4×4 world-to-camera transform (Drake camera frames are already in
        the OpenCV convention: +Z forward, +X right, +Y down)."""
        cam = self._drake_camera(camera_name)
        sc = self._station_ctx()
        X_WB = self._station.GetOutputPort(f"{cam}.X_WB").Eval(sc)
        X_BC = self._camera_info[cam][0]
        X_WC = X_WB @ X_BC if X_BC is not None else X_WB
        return np.linalg.inv(X_WC.GetAsMatrix4())

    def get_camera_image(self, camera_name: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return (rgb uint8 HxWx3, depth float32 HxW, label int HxW).

        Cross-backend label convention: positive labels (Drake assigns the
        body index as the render label) only for free scene objects; the
        robot, fixed objects, and background are -1.
        """
        cam = self._drake_camera(camera_name)
        sc = self._station_ctx()
        rgb_raw = self._station.GetOutputPort(f"{cam}.rgb_image").Eval(sc)
        depth_raw = self._station.GetOutputPort(f"{cam}.depth_image").Eval(sc)
        label_raw = self._station.GetOutputPort(f"{cam}.label_image").Eval(sc)

        rgb = rgb_raw.data[:, :, :3].copy()
        depth = depth_raw.data[:, :, 0].copy()
        raw = label_raw.data[:, :, 0].astype(np.int32)   # int16 RenderLabel values

        plant = self._station.plant()
        keep_labels: set[int] = set()
        for name, instance in self._obj_instances.items():
            if name in self._initial_obj_poses:   # free objects only
                keep_labels.update(
                    int(b) for b in plant.GetBodyIndices(instance)
                )
        label = np.where(np.isin(raw, list(keep_labels)), raw, -1)
        return rgb, depth, label

    def execute_chain_trajectory(
        self,
        chain_name: str,
        trajectory: list,
        waypoint_dt: float | None = None,
    ) -> bool:
        """
        Execute a joint-space trajectory for *chain_name*.

        waypoint_dt
            Duration (seconds) for each waypoint segment.  When *None* (the
            default) the duration is computed automatically from the joint
            velocity limits so the commanded velocity never exceeds the URDF
            limit.  Pass an explicit value only when you need a specific speed.
        """
        if chain_name == "base":
            return self.execute_base_trajectory(trajectory, waypoint_dt or 0.5)

        slices = self._robot_spec.chain_slices
        if chain_name not in slices:
            raise ValueError(
                f"Unknown chain '{chain_name}'. Available: {sorted(slices)}"
            )
        drake_slice = slices[chain_name]

        arrays, durations = parse_trajectory(trajectory, waypoint_dt)
        if durations is None:
            durations = self._auto_waypoint_dt(drake_slice, arrays)

        t_goal, q_goal = self._build_and_set_trajectory(drake_slice, arrays, durations)
        self._advance_until_near(t_goal, q_goal, drake_slice)
        # Hold at the GOAL (not the actual position) so the PD controller keeps
        # correcting any residual error even after this call returns.  Using
        # actual_q would cement whatever tracking error remained at timeout.
        self._source.set_hold(self._context, q_goal)
        return True

    def execute_base_trajectory(
        self,
        trajectory: list,
        waypoint_dt: float | None = None,
    ) -> bool:
        waypoints, durations = parse_trajectory(trajectory, waypoint_dt)

        # Convert canonical [x, y, theta] → Drake base positions.
        drake_waypoints = [
            self._robot_spec.base_to_drake(float(wp[0]), float(wp[1]), float(wp[2]))
            for wp in waypoints
        ]
        if durations is None:
            # Auto-time from base velocity limits.
            drake_slice = self._robot_spec.chain_slices["base"]
            prev = self._get_desired_positions()[drake_slice]
            durations = []
            for wp in drake_waypoints:
                lin = float(np.linalg.norm(wp[:2] - prev[:2]))
                ang = float(abs(wp[2] - prev[2]))
                durations.append(
                    max(lin / _BASE_LIN_VEL, ang / _BASE_ANG_VEL, _MIN_WAYPOINT_DT)
                )
                prev = wp

        drake_slice = self._robot_spec.chain_slices["base"]
        t_goal, q_goal = self._build_and_set_trajectory(
            drake_slice, drake_waypoints, durations
        )
        self._advance_until_near(t_goal, q_goal, drake_slice, tol=0.02)
        self._source.set_hold(self._context, q_goal)
        return True

    def execute_gripper_command(
        self,
        side: str,
        command: Union[str, float],
        settle_secs: float = 1.0,
    ) -> bool:
        positions = self._robot_spec.gripper_positions(command)
        sides = list(self._robot_spec.gripper_sides) if side == "both" else [side]
        q_des = self._get_desired_positions()

        for s in sides:
            key = f"{s}_gripper"
            if key not in self._robot_spec.chain_slices:
                raise ValueError(f"No gripper found for side '{s}'")
            q_des[self._robot_spec.chain_slices[key]] = positions

        # Build a short trajectory
        t_curr = self._context.get_time()
        q_start = self._get_desired_positions()
        ts = np.array([0.0, settle_secs])
        qs = np.column_stack([q_start, q_des])
        traj_q = PiecewisePolynomial.FirstOrderHold(ts, qs)
        self._source.set_trajectory(self._context, traj_q, traj_q.derivative(), t_curr)

        self._simulator.AdvanceTo(t_curr + settle_secs)
        self._source.set_hold(self._context, q_des)
        return True

    def step(self, n: int = 1) -> None:
        t = self._context.get_time()
        self._simulator.AdvanceTo(t + n * 0.001)

    def set_object_pose(
        self,
        obj_name: str,
        pos: np.ndarray,
        quat: np.ndarray | None = None,
    ) -> None:
        if obj_name not in self._obj_instances:
            raise ValueError(f"Unknown object '{obj_name}'")
        plant = self._station.plant()
        plant_ctx = plant.GetMyContextFromRoot(self._context)
        instance = self._obj_instances[obj_name]
        body = plant.get_body(plant.GetBodyIndices(instance)[0])
        if quat is None:
            quat = np.array([1.0, 0.0, 0.0, 0.0])
        X = RigidTransform(Quaternion(np.asarray(quat, float)),
                           np.asarray(pos, float))
        plant.SetFreeBodyPose(plant_ctx, body, X)

    def point_head_at(
        self,
        target_world_pos: np.ndarray,
        camera_name: str = "head_camera",
        max_iter: int = 200,
        tol: float = 0.01,
        waypoint_dt: float = 0.8,
    ) -> bool:
        """
        Iterative damped-least-squares IK to point a camera at a target by
        moving the robot's head chain.  Uses finite-difference Jacobian over
        Drake FK — no physics during search.
        """
        head_chain = self._robot_spec.head_chain
        if head_chain is None:
            raise ValueError(f"Robot '{self._robot_spec.name}' has no head chain")
        cam = self._drake_camera(camera_name)

        target = np.asarray(target_world_pos, float)
        plant = self._station.plant()
        plant_ctx = plant.GetMyContextFromRoot(self._context)

        head_instance = plant.GetModelInstanceByName(head_chain)
        head_joints = [
            jidx for jidx in plant.GetJointIndices(head_instance)
            if plant.get_joint(jidx).num_positions() == 1
        ]
        if not head_joints:
            raise ValueError(f"No 1-DOF joints found in model instance '{head_chain}'")

        # Current optical-frame pose, exact from the station: sensor body pose
        # composed with the sensor-in-body offset.
        sc = self._station_ctx()
        X_WB_now = self._station.GetOutputPort(f"{cam}.X_WB").Eval(sc)
        X_BC = self._camera_info[cam][0]
        X_WC_now = X_WB_now @ X_BC if X_BC is not None else X_WB_now

        # Mount body whose frame carries the camera through head FK.
        mount_name = self._robot_spec.camera_mount_bodies.get(cam)
        if mount_name is not None:
            cam_body = plant.GetBodyByName(mount_name)
        else:
            # Fall back to the last body in the head chain (the camera mount).
            cam_body = plant.get_body(plant.GetBodyIndices(head_instance)[-1])
        # Fixed camera-in-mount transform, valid for any head configuration.
        X_W_mount_now = plant.EvalBodyPoseInWorld(plant_ctx, cam_body)
        X_mount_C = X_W_mount_now.inverse() @ X_WC_now

        head_slice = self._robot_spec.chain_slices[head_chain]
        angles = self._get_joint_positions()[head_slice].copy()

        lo = np.array([float(plant.get_joint(jidx).position_lower_limits()[0])
                       for jidx in head_joints])
        hi = np.array([float(plant.get_joint(jidx).position_upper_limits()[0])
                       for jidx in head_joints])

        q_save = plant.GetPositions(plant_ctx).copy()

        def _set_angles(ang: np.ndarray):
            q = plant.GetPositions(plant_ctx).copy()
            for i, jidx in enumerate(head_joints):
                jnt = plant.get_joint(jidx)
                q[jnt.position_start()] = float(ang[i])
            plant.SetPositions(plant_ctx, q)

        def _cam_state() -> tuple[np.ndarray, np.ndarray]:
            X_WC = plant.EvalBodyPoseInWorld(plant_ctx, cam_body) @ X_mount_C
            return X_WC.translation(), X_WC.rotation().col(2)  # pos, +Z forward

        eps = 1e-4
        dof = len(head_joints)
        for _ in range(max_iter):
            _set_angles(angles)
            cam_pos, fwd = _cam_state()
            to_t = target - cam_pos
            dist = np.linalg.norm(to_t)
            if dist < 1e-6:
                break
            err = to_t / dist - fwd
            if np.linalg.norm(err) < tol:
                break

            J = np.zeros((3, dof))
            for i in range(dof):
                ap = angles.copy(); ap[i] += eps
                _set_angles(ap)
                _, fwd_p = _cam_state()
                J[:, i] = (fwd_p - fwd) / eps
            _set_angles(angles)

            lam = 1e-3
            delta = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(3), err) * 0.5
            angles = np.clip(angles + delta, lo, hi)

        # Restore plant state before executing via trajectory
        plant.SetPositions(plant_ctx, q_save)
        return self.execute_chain_trajectory(
            head_chain, [angles.tolist()], waypoint_dt=waypoint_dt
        )

    def reload(self, file_updates: dict[str, str] | None = None) -> None:
        raise NotImplementedError("reload() is not supported by the Drake backend")

    def idle(self, seconds: float) -> None:
        """Advance the simulator while no request is pending so the Meshcat
        visualization stays live in display mode."""
        if self._mode == "display":
            self._simulator.AdvanceTo(self._context.get_time() + seconds)

    def close(self) -> None:
        if self._meshcat is not None:
            self._meshcat.Delete("/drake")

    @property
    def is_running(self) -> bool:
        return True

    @property
    def meshcat(self):
        return self._meshcat
