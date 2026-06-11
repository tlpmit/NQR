"""
Spot Drake simulation server.

Speaks the same ZMQ protocol as the other backends:

    python -m qr_robots.drake.spot.sim \\
        --objects '{"block":"block.urdf"}' --mode display --port 5559

The Spot model has a planar floating base (x, y, yaw), a 6-DOF arm, and a
1-DOF gripper; legs are fixed.  The canonical conf is the 10-element Drake
position vector [base(3), arm(6), gripper(1)].
"""

from __future__ import annotations

import argparse
import json
import os
from typing import Union

import numpy as np

from qr_robots.common import spot
from qr_robots.common.zmq_sim import serve
from qr_robots.drake.sim_base import DrakeRobotSim, DrakeRobotSpec
from qr_utils import GetQRAssetsPath

# arm_f1x joint limits: 0.0 = closed, -1.57 = fully open.
_GRIPPER_OPEN_ANGLE = -1.57

DRAKE_SLICES: dict[str, slice] = dict(
    base=slice(0, 3),
    right_arm=slice(3, 9),
    right_gripper=slice(9, 10),
)


def gripper_positions(command: Union[str, float]) -> np.ndarray:
    if command == "close":
        f = 0.0
    elif command == "open":
        f = 1.0
    else:
        f = float(np.clip(command, 0.0, 1.0))
    return np.array([_GRIPPER_OPEN_ANGLE * f])


def make_spec() -> DrakeRobotSpec:
    assets = GetQRAssetsPath()
    return DrakeRobotSpec(
        name="spot",
        scenario_files=[
            str(assets / "spot" / "add_spot_with_arm_and_floating_base_actuators.dmd.yaml"),
            str(assets / "spot" / "model_drivers.scenario.yaml"),
            str(assets / "spot" / "cameras.scenario.yaml"),
        ],
        groups=[("spot", 10)],
        chain_slices=DRAKE_SLICES,
        conf_from_q=lambda q: np.array(q),
        base_to_drake=lambda x, y, theta: np.array([x, y, theta]),
        gripper_positions=gripper_positions,
        gripper_sides=("right",),
        cameras={
            "hand_camera": "hand",
            "back_camera": "back",
            "frontleft_camera": "frontleft",
            "frontright_camera": "frontright",
            "left_camera": "left",
            "right_camera": "right",
        },
        head_chain=None,
    )


def make_sim(model_dir: str = ".", objects: dict | None = None,
             mode: str = "headless") -> DrakeRobotSim:
    return DrakeRobotSim(make_spec(), model_dir, objects, mode=mode)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Spot Drake ZMQ server.")
    ap.add_argument("--model-dir", default=".",
                    help="Directory that object file paths are relative to.")
    ap.add_argument("--objects", default="{}",
                    help="JSON string or path to a JSON file describing scene objects.")
    ap.add_argument("--mode", default="headless", choices=["headless", "display"])
    ap.add_argument("--port", type=int, default=spot.DRAKE_PORT)
    args = ap.parse_args()

    if os.path.isfile(args.objects):
        with open(args.objects) as f:
            objects = json.load(f)
    else:
        objects = json.loads(args.objects)

    sim = make_sim(args.model_dir, objects, mode=args.mode)
    if sim.meshcat is not None:
        print(f"[spot_drake_sim] Meshcat at {sim.meshcat.web_url()}", flush=True)
    serve(sim, port=args.port, mode=args.mode, name="spot_drake_sim")
