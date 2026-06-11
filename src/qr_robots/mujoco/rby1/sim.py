"""
RBY1 MuJoCo simulation server.

Start (macOS display mode requires mjpython):

    mjpython -m qr_robots.mujoco.rby1.sim \\
        --objects '{"table":{"file":"table.xml","fixed":true},"block":"block.xml"}' \\
        --model-dir scene/ --mode display --port 5556
"""

from __future__ import annotations

import argparse
import json
import os

from qr_robots.common import rby1
from qr_robots.common.zmq_sim import serve
from qr_robots.mujoco.sim_base import MujocoRobotSim, MujocoRobotSpec
from qr_utils import GetQRAssetsPath

ROBOT_XML = GetQRAssetsPath() / "rby1a" / "mujoco" / "model_act.xml"

RBY1_SPEC = MujocoRobotSpec(
    name="rby1",
    robot_xml=str(ROBOT_XML),
    chain_patterns={
        "right_arm": "right_arm_",
        "left_arm": "left_arm_",
        "torso": "torso_",
        "head": "head_",
    },
    gripper_act_prefixes={
        "right": ("right_finger",),
        "left": ("left_finger",),
    },
    base_mode="mocap",
    axle_x_offset=rby1.AXLE_X_OFFSET,
)


def make_sim(model_dir: str = ".", objects: dict | None = None,
             mode: str = "headless", **kwargs) -> MujocoRobotSim:
    return MujocoRobotSim(RBY1_SPEC, model_dir, objects, mode=mode, **kwargs)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(
        description="RBY1 MuJoCo ZMQ server.  Start with mjpython on macOS for display mode."
    )
    ap.add_argument("--model-dir", default=".",
                    help="Directory that object file paths are relative to.")
    ap.add_argument("--objects", default="{}",
                    help="JSON string or path to a JSON file describing scene objects.")
    ap.add_argument("--mode", default="headless", choices=["headless", "display"])
    ap.add_argument("--port", type=int, default=rby1.MUJOCO_PORT)
    args = ap.parse_args()

    if os.path.isfile(args.objects):
        with open(args.objects) as f:
            objects = json.load(f)
    else:
        objects = json.loads(args.objects)

    sim = make_sim(args.model_dir, objects, mode=args.mode)
    serve(sim, port=args.port, mode=args.mode, name="rby1_mujoco_sim")
