"""
RBY1 Drake simulation server.

Speaks the same ZMQ protocol as the MuJoCo backend:

    python -m qr_robots.drake.rby1.sim \\
        --objects '{"block":"block.urdf"}' --mode display --port 5557
"""

from __future__ import annotations

import argparse
import json
import os
from typing import Union

import numpy as np

from qr_robots.common import rby1
from qr_robots.common.zmq_sim import serve
from qr_robots.drake.sim_base import DrakeRobotSim, DrakeRobotSpec
from qr_utils import GetQRAssetsPath

# Gripper finger travel distance (half-opening = 0.05 m per finger)
_FINGER_HALF_RANGE: float = 0.05

# Joint index ranges within the 29-element Drake position vector:
#   [base(3), torso(6), left_arm(7), right_arm(7), head(2), left_gripper(2), right_gripper(2)]
DRAKE_SLICES: dict[str, slice] = dict(
    base=slice(0, 3),
    torso=slice(3, 9),
    left_arm=slice(9, 16),
    right_arm=slice(16, 23),
    head=slice(23, 25),
    left_gripper=slice(25, 27),
    right_gripper=slice(27, 29),
)


def drake_q_to_conf(q: np.ndarray) -> np.ndarray:
    """
    Convert the 29-element Drake position vector to the canonical 31-element
    RBY1 conf:
      [axle_x, axle_y, theta,       # base (3) — axle-midpoint
       0, 0,                         # fake wheels (2)
       torso(6), right_arm(7), right_gripper(2),
       left_arm(7),  left_gripper(2), head(2)]
    """
    bx, by, brz = q[DRAKE_SLICES["base"]]
    axle_x = bx + rby1.AXLE_X_OFFSET * np.cos(brz)
    axle_y = by + rby1.AXLE_X_OFFSET * np.sin(brz)
    return np.concatenate([
        [axle_x, axle_y, brz],           # base (axle midpoint)
        [0.0, 0.0],                       # fake wheels
        q[DRAKE_SLICES["torso"]],
        q[DRAKE_SLICES["right_arm"]],
        q[DRAKE_SLICES["right_gripper"]],
        q[DRAKE_SLICES["left_arm"]],
        q[DRAKE_SLICES["left_gripper"]],
        q[DRAKE_SLICES["head"]],
    ])


def base_to_drake(axle_x: float, axle_y: float, theta: float) -> np.ndarray:
    """Convert canonical axle-midpoint [x, y, theta] to Drake base [bx, by, rz]."""
    bx = axle_x - rby1.AXLE_X_OFFSET * np.cos(theta)
    by = axle_y - rby1.AXLE_X_OFFSET * np.sin(theta)
    return np.array([bx, by, theta])


def gripper_positions(command: Union[str, float]) -> np.ndarray:
    """Return [finger_1, finger_2] positions for open/close/fraction."""
    if command == "close":
        f = 0.0
    elif command == "open":
        f = 1.0
    else:
        f = float(np.clip(command, 0.0, 1.0))
    return np.array([-_FINGER_HALF_RANGE * f, _FINGER_HALF_RANGE * f])


def make_spec() -> DrakeRobotSpec:
    assets = GetQRAssetsPath()
    return DrakeRobotSpec(
        name="rby1",
        scenario_files=[
            str(assets / "rby1_description_drake"
                / "add_rby1_sim_with_holonomic_base_actuators.dmd.yaml"),
            str(assets / "rby1" / "model_drivers.scenario.yaml"),
            str(assets / "rby1" / "cameras.scenario.yaml"),
        ],
        groups=[
            ("base+torso+left_arm+right_arm", 23),
            ("head", 2),
            ("left_gripper", 2),
            ("right_gripper", 2),
        ],
        chain_slices=DRAKE_SLICES,
        conf_from_q=drake_q_to_conf,
        base_to_drake=base_to_drake,
        gripper_positions=gripper_positions,
        gripper_sides=("right", "left"),
        cameras={"head_camera": "head"},
        head_chain="head",
        camera_mount_bodies={"head": "rby1_camera_mount"},
    )


def make_sim(model_dir: str = ".", objects: dict | None = None,
             mode: str = "headless") -> DrakeRobotSim:
    return DrakeRobotSim(make_spec(), model_dir, objects, mode=mode)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="RBY1 Drake ZMQ server.")
    ap.add_argument("--model-dir", default=".",
                    help="Directory that object file paths are relative to.")
    ap.add_argument("--objects", default="{}",
                    help="JSON string or path to a JSON file describing scene objects.")
    ap.add_argument("--mode", default="headless", choices=["headless", "display"])
    ap.add_argument("--port", type=int, default=rby1.DRAKE_PORT)
    args = ap.parse_args()

    if os.path.isfile(args.objects):
        with open(args.objects) as f:
            objects = json.load(f)
    else:
        objects = json.loads(args.objects)

    sim = make_sim(args.model_dir, objects, mode=args.mode)
    if sim.meshcat is not None:
        print(f"[rby1_drake_sim] Meshcat at {sim.meshcat.web_url()}", flush=True)
    serve(sim, port=args.port, mode=args.mode, name="rby1_drake_sim")
