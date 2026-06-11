"""
KinematicSim — kinematics-only simulation backend (Pinocchio + Coal + Meshcat).

Speaks the same ZMQ protocol as the MuJoCo/Drake backends, so the same
SimClient / virtual robot works against it.  There is no dynamics: trajectory
execution sets configurations directly (animated in display mode), gripper
close/open attaches/detaches the nearest object to model grasping, and
collision checking is available via check_collisions().

Cameras are provided by a render-only MuJoCo bridge (see qr_kinsim.render):
a headless MuJoCo scene with the same robot model and cameras as the MuJoCo
dynamics backend is synced to the kinematic state before each image, so
get_camera_image / get_camera_intrinsics / get_camera_extrinsics return
exactly the same formats as the dynamics backends.  The bridge is built
lazily on the first camera call.

Object dict format (kinematic backend):
    {"shape": {"box": [sx, sy, sz]} | {"sphere": r} | {"cylinder": [r, l]}
              | {"mesh": path},
     "pos": [x, y, z], "quat": [w, x, y, z],
     "fixed": bool, "color": [r, g, b, a]}

Start a server:

    python -m qr_kinsim.sim --robot rby1 \\
        --objects '{"block": {"shape": {"box": [0.08, 0.08, 0.08]}, "pos": [0.65, 0, 0.79]}}' \\
        --mode display --port 5560
"""

from __future__ import annotations

import argparse
import json
import os
import time
from typing import Union

import numpy as np

from qr_kinsim import robots
from qr_kinsim.robots import KinRobotSpec
from qr_kinsim.world import KinematicWorld, _se3_to_pose
from qr_robots.common.zmq_sim import serve
from qr_robots.mujoco.sim_base import parse_trajectory

RBY1_KINSIM_PORT = 5560
SPOT_KINSIM_PORT = 5561


class KinematicSim:
    """Protocol adapter wiring a KinematicWorld to the shared sim protocol."""

    _ANIMATE_HZ = 50.0

    def __init__(
        self,
        robot: str = "rby1",
        model_dir: str = ".",
        objects: dict | None = None,
        mode: str = "headless",
    ):
        if mode not in ("headless", "display"):
            raise ValueError(f"mode must be 'headless' or 'display', got '{mode}'")
        self._mode = mode
        self._model_dir = model_dir

        self.spec: KinRobotSpec = robots.ROBOT_SPECS[robot]()
        self.world: KinematicWorld = robots.make_world(
            self.spec, visualize=(mode == "display")
        )

        for name, val in (objects or {}).items():
            if name == "robot":
                continue
            self._add_object_from_spec(name, val)

        self.world.update_display()
        self._robot = robot
        self._render_bridge = None

    def _add_object_from_spec(self, name: str, val: dict) -> None:
        if not isinstance(val, dict) or "shape" not in val:
            raise ValueError(
                f"Kinematic backend objects need a 'shape' entry; got {val!r} "
                f"for '{name}'"
            )
        shape = dict(val["shape"])
        if "mesh" in shape and not os.path.isabs(str(shape["mesh"])):
            shape["mesh"] = os.path.join(self._model_dir, str(shape["mesh"]))
        self.world.add_object(
            name,
            shape=shape,
            pos=val.get("pos", (0.0, 0.0, 0.0)),
            quat=val.get("quat"),
            color=tuple(val.get("color", (0.6, 0.6, 0.6, 1.0))),
            fixed=bool(val.get("fixed", False)),
        )

    # ── state queries ─────────────────────────────────────────────────────────

    def get_robot_conf(self) -> np.ndarray:
        return robots.conf_from_world(self.spec, self.world)

    def set_robot_conf(self, conf: np.ndarray) -> None:
        robots.set_world_conf(self.spec, self.world, conf)
        self.world.update_display()

    def get_world_state(self) -> dict:
        state = {
            name: _se3_to_pose(self.world.get_object_pose(name))
            for name in self.world.objects
        }
        state["robot"] = self.get_robot_conf()
        return state

    def get_frame_pose(self, frame_name: str) -> np.ndarray:
        return self.world.get_frame_pose(frame_name).homogeneous

    # ── cameras (rendered via the MuJoCo bridge) ──────────────────────────────

    def _renderer(self):
        if self._render_bridge is None:
            from qr_kinsim.render import MujocoRenderBridge

            self._render_bridge = MujocoRenderBridge(
                self._robot,
                {
                    name: {
                        "shape": obj.shape,
                        "color": obj.color,
                        "fixed": obj.fixed,
                        "pose": _se3_to_pose(self.world.get_object_pose(name)),
                    }
                    for name, obj in self.world.objects.items()
                },
            )
        # Mirror the kinematic state (free objects only; fixed ones are welded).
        self._render_bridge.sync(
            self.get_robot_conf(),
            {
                name: _se3_to_pose(self.world.get_object_pose(name))
                for name, obj in self.world.objects.items()
                if not obj.fixed
            },
        )
        return self._render_bridge

    def get_camera_image(self, camera_name: str):
        return self._renderer().get_camera_image(camera_name)

    def get_camera_intrinsics(self, camera_name: str) -> np.ndarray:
        return self._renderer().get_camera_intrinsics(camera_name)

    def get_camera_extrinsics(self, camera_name: str) -> np.ndarray:
        return self._renderer().get_camera_extrinsics(camera_name)

    # ── control ───────────────────────────────────────────────────────────────

    def _chain_conf(self, chain: str) -> np.ndarray:
        if chain == "base":
            return robots.get_base_pose(self.spec, self.world)
        return np.array([
            robots.get_joint_value(self.world.model, self.world.q, j)
            for j in self.spec.joints_for(chain)
        ])

    def _move_chain(self, chain: str, target: np.ndarray, duration: float) -> None:
        """Set a chain configuration, animating in display mode."""
        if self._mode == "display" and duration > 0:
            start = self._chain_conf(chain)
            steps = max(2, int(duration * self._ANIMATE_HZ))
            for i in range(1, steps + 1):
                alpha = i / steps
                robots.set_chain_conf(
                    self.spec, self.world, chain, start + alpha * (target - start)
                )
                self.world.update_display()
                time.sleep(1.0 / self._ANIMATE_HZ)
        else:
            robots.set_chain_conf(self.spec, self.world, chain, target)
            self.world.update_display()

    def execute_chain_trajectory(
        self,
        chain_name: str,
        trajectory: list,
        waypoint_dt: float | None = None,
    ) -> bool:
        arrays, durations = parse_trajectory(trajectory, waypoint_dt)
        if durations is None:
            durations = [0.5] * len(arrays)
        for arr, dur in zip(arrays, durations):
            self._move_chain(chain_name, np.asarray(arr, dtype=float), dur)
        return True

    def execute_base_trajectory(
        self, trajectory: list, waypoint_dt: float | None = None
    ) -> bool:
        return self.execute_chain_trajectory("base", trajectory, waypoint_dt)

    def execute_gripper_command(
        self,
        side: str,
        command: Union[str, float],
        settle_secs: float = 1.0,
    ) -> bool:
        """
        Open/close a gripper.  Closing attaches the nearest free object within
        the gripper's grasp radius (modeling a grasp); opening detaches any
        attached object (releasing it at its current pose).
        """
        sides = list(self.spec.grippers) if side == "both" else [side]
        for s in sides:
            gripper = self.spec.grippers[s]
            if command == "close":
                values = gripper.closed_values
            elif command == "open":
                values = gripper.open_values
            else:
                f = float(np.clip(command, 0.0, 1.0))
                values = [
                    c + f * (o - c)
                    for o, c in zip(gripper.open_values, gripper.closed_values)
                ]
            for j, v in zip(gripper.joints, values):
                robots.set_joint_value(self.world.model, self.world.q, j, float(v))
            self.world.update()

            if command == "close":
                self._try_attach(gripper)
            elif command == "open":
                for name in self.world.attached_objects(gripper.frame):
                    self.world.detach_object(name)

        self.world.update_display()
        return True

    def _try_attach(self, gripper) -> None:
        candidates = [
            (self.world.object_distance(gripper.frame, name), name)
            for name, obj in self.world.objects.items()
            if not obj.fixed and obj.attach_frame is None
        ]
        candidates = [(d, n) for d, n in candidates if d <= gripper.grasp_radius]
        if candidates:
            _, name = min(candidates)
            self.world.attach_object(name, gripper.frame)

    # ── world editing / queries ───────────────────────────────────────────────

    def set_object_pose(self, obj_name: str, pos, quat=None) -> None:
        self.world.set_object_pose(obj_name, pos, quat)
        self.world.update_display()

    def attach_object(self, obj_name: str, frame_name: str) -> None:
        self.world.attach_object(obj_name, frame_name)

    def detach_object(self, obj_name: str) -> None:
        self.world.detach_object(obj_name)

    def check_collisions(self, include_self: bool = False,
                         margin: float = -1e-3) -> list:
        return self.world.check_collisions(include_self=include_self, margin=margin)

    def step(self, n: int = 1) -> None:
        pass   # no dynamics

    def idle(self, seconds: float) -> None:
        pass   # Meshcat updates are pushed eagerly

    def close(self) -> None:
        if self._render_bridge is not None:
            self._render_bridge.close()
            self._render_bridge = None

    @property
    def is_running(self) -> bool:
        return True


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Kinematic (Pinocchio) ZMQ sim server.")
    ap.add_argument("--robot", default="rby1", choices=sorted(robots.ROBOT_SPECS))
    ap.add_argument("--model-dir", default=".",
                    help="Directory that object mesh paths are relative to.")
    ap.add_argument("--objects", default="{}",
                    help="JSON string or path to a JSON file describing scene objects.")
    ap.add_argument("--mode", default="headless", choices=["headless", "display"])
    ap.add_argument("--port", type=int, default=RBY1_KINSIM_PORT)
    args = ap.parse_args()

    if os.path.isfile(args.objects):
        with open(args.objects) as f:
            objects = json.load(f)
    else:
        objects = json.loads(args.objects)

    sim = KinematicSim(args.robot, args.model_dir, objects, mode=args.mode)
    if sim.world.meshcat_url:
        print(f"[kinsim] Meshcat at {sim.world.meshcat_url}", flush=True)
    serve(sim, port=args.port, mode=args.mode, name=f"{args.robot}_kinsim")
