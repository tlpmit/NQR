"""
Shared ZMQ REQ/REP protocol for robot simulation backends.

A *sim* is any object implementing (a subset of) the command methods listed in
``SIM_COMMANDS``.  ``serve()`` exposes it over a ZMQ REP socket using pickled
``{"cmd": str, "args": list, "kwargs": dict}`` requests and
``{"status": "ok"|"error", ...}`` replies.  ``SimClient`` is the matching REQ
client; it is backend-agnostic and works unchanged against MuJoCo, Drake, or
the kinematic (Pinocchio) backend.

Conventions shared by all backends
----------------------------------
- ``get_robot_conf()`` returns the robot configuration vector in the robot's
  canonical format (per-robot; see the robot's ``conf.py``).
- ``get_world_state()`` returns ``{object_name: [x, y, z, qw, qx, qy, qz], ...,
  "robot": conf_vector}``.
- ``get_camera_extrinsics(name)`` returns a 4x4 world-to-camera transform in
  the OpenCV convention (+Z forward, +X right, +Y down).
- Quaternions are (w, x, y, z).
"""

from __future__ import annotations

import pickle
from typing import Union

import numpy as np

# Commands a sim object may implement.  The server only dispatches names in
# this set, so arbitrary attribute access is not exposed on the wire.
SIM_COMMANDS = frozenset({
    "get_robot_conf",
    "get_world_state",
    "get_camera_intrinsics",
    "get_camera_extrinsics",
    "get_camera_image",
    "get_point_cloud",
    "execute_chain_trajectory",
    "execute_base_trajectory",
    "execute_gripper_command",
    "step",
    "set_object_pose",
    "set_robot_conf",
    "point_head_at",
    "attach_object",
    "detach_object",
    "check_collisions",
    "reload",
})


def serve(sim, port: int = 5555, mode: str = "headless", name: str = "sim") -> None:
    """
    Start a ZMQ REP server owning *sim*.  Blocks until a "stop" command is
    received or (in display mode) the viewer is closed.

    The sim may provide two optional hooks used by the serve loop:
    - ``is_running`` (property): False when its display window was closed.
    - ``idle(seconds)``: advance/refresh the backend while no request is
      pending (used to keep viewers live).
    """
    import zmq

    ctx = zmq.Context()
    sock = ctx.socket(zmq.REP)
    sock.bind(f"tcp://*:{port}")

    poller = zmq.Poller()
    poller.register(sock, zmq.POLLIN)
    poll_ms = 16 if mode == "display" else 100

    print(f"[{name}] serving on port {port}  mode={mode}", flush=True)

    try:
        while True:
            if not getattr(sim, "is_running", True):
                print(f"[{name}] viewer closed — shutting down", flush=True)
                break

            events = dict(poller.poll(poll_ms))
            if sock not in events:
                idle = getattr(sim, "idle", None)
                if idle is not None:
                    idle(poll_ms / 1000.0)
                continue

            request = pickle.loads(sock.recv())
            cmd = request.get("cmd", "")

            if cmd == "stop":
                sock.send(pickle.dumps({"status": "ok", "result": None}))
                break

            try:
                if cmd not in SIM_COMMANDS:
                    raise ValueError(f"Unknown command '{cmd}'")
                fn = getattr(sim, cmd, None)
                if fn is None:
                    raise NotImplementedError(
                        f"Backend {type(sim).__name__} does not implement '{cmd}'"
                    )
                result = fn(*request.get("args", []), **request.get("kwargs", {}))
                sock.send(pickle.dumps({"status": "ok", "result": result}))
            except Exception as exc:
                sock.send(pickle.dumps({"status": "error", "error": repr(exc)}))

    finally:
        close = getattr(sim, "close", None)
        if close is not None:
            close()
        sock.close()
        ctx.term()
        print(f"[{name}] server exited", flush=True)


class SimClient:
    """
    ZMQ REQ client for a sim server.

    All methods block until the server responds.  For long trajectories this
    may take several seconds — the server is busy running physics.
    """

    def __init__(
        self, host: str = "localhost", port: int = 5555, timeout_ms: int = 60_000
    ):
        import zmq

        ctx = zmq.Context()
        sock = ctx.socket(zmq.REQ)
        sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
        sock.connect(f"tcp://{host}:{port}")
        self._sock = sock

    def _call(self, cmd: str, *args, **kwargs):
        self._sock.send(
            pickle.dumps({"cmd": cmd, "args": list(args), "kwargs": kwargs})
        )
        response = pickle.loads(self._sock.recv())
        if response["status"] == "error":
            raise RuntimeError(f"[sim error] {response['error']}")
        return response["result"]

    # ── state queries ─────────────────────────────────────────────────────────

    def get_robot_conf(self) -> np.ndarray:
        return self._call("get_robot_conf")

    def get_world_state(self) -> dict:
        return self._call("get_world_state")

    def get_camera_intrinsics(self, camera_name: str) -> np.ndarray:
        return self._call("get_camera_intrinsics", camera_name)

    def get_camera_extrinsics(self, camera_name: str) -> np.ndarray:
        return self._call("get_camera_extrinsics", camera_name)

    def get_camera_image(self, camera_name: str):
        """Returns (rgb uint8 HxWx3, depth float32 HxW, label int HxW)."""
        return self._call("get_camera_image", camera_name)

    # ── control ───────────────────────────────────────────────────────────────

    def execute_chain_trajectory(
        self, chain: str, trajectory: list, waypoint_dt: float | None = None
    ) -> bool:
        return self._call("execute_chain_trajectory", chain, trajectory, waypoint_dt)

    def execute_base_trajectory(
        self, trajectory: list, waypoint_dt: float = 0.5
    ) -> bool:
        return self._call("execute_base_trajectory", trajectory, waypoint_dt)

    def execute_gripper_command(
        self, side: str, command: Union[str, float], settle_secs: float = 1.0
    ) -> bool:
        return self._call("execute_gripper_command", side, command, settle_secs)

    def step(self, n: int = 1) -> None:
        self._call("step", n)

    def point_head_at(
        self,
        target_world_pos: np.ndarray,
        camera_name: str = "head_camera",
        waypoint_dt: float = 0.8,
    ) -> bool:
        return self._call(
            "point_head_at", target_world_pos, camera_name, waypoint_dt=waypoint_dt
        )

    # ── world editing ─────────────────────────────────────────────────────────

    def set_object_pose(
        self, name: str, pos: np.ndarray, quat: np.ndarray | None = None
    ) -> None:
        self._call("set_object_pose", name, pos, quat)

    def set_robot_conf(self, conf: np.ndarray) -> None:
        self._call("set_robot_conf", conf)

    def attach_object(self, obj_name: str, frame_name: str) -> None:
        """Kinematic backends: rigidly attach an object to a robot frame."""
        self._call("attach_object", obj_name, frame_name)

    def detach_object(self, obj_name: str) -> None:
        self._call("detach_object", obj_name)

    def check_collisions(self) -> list:
        """Kinematic backends: list of colliding pair names at the current conf."""
        return self._call("check_collisions")

    def reload(self, file_updates: dict[str, str] | None = None) -> None:
        self._call("reload", file_updates)

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def stop(self) -> None:
        self._call("stop")

    def close(self) -> None:
        self._sock.close()
