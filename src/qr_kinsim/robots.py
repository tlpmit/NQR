"""
Robot definitions for the kinematic simulator.

Each robot is modeled as a tree of chains (base, torso, arms, grippers,
head), mirroring the chain layout of the dynamic backends so the canonical
conf vectors are interchangeable across MuJoCo / Drake / kinematic sims.
"""

from __future__ import annotations

import math
import os
import re
import tempfile
from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np
import pinocchio as pin

from qr_kinsim.world import KinematicWorld, get_joint_value, set_joint_value
from qr_robots.common import rby1 as rby1_common
from qr_robots.common import spot as spot_common
from qr_utils import GetQRAssetsPath


@dataclass
class GripperSpec:
    frame: str
    """Robot frame objects get attached to when grasped."""
    joints: list[str]
    open_values: list[float]
    closed_values: list[float]
    grasp_radius: float = 0.25
    """Max distance from the gripper frame for an object to be grasped."""


@dataclass
class KinRobotSpec:
    name: str
    urdf_source: str
    preprocess: Optional[Callable[[str], str]] = None
    """Optional URDF-text rewriter applied before loading."""
    root_joint_factory: Optional[Callable[[], pin.JointModel]] = None
    package_dirs: list[str] = field(default_factory=list)

    base_mode: str = "planar_root"
    """"planar_root": root joint is a planar joint (q = x, y, cos, sin);
    "joints": named base joints exist in the URDF."""
    base_joints: tuple[str, ...] = ()
    axle_x_offset: float = 0.0
    """Canonical base pose reference point offset (see rby1_common)."""

    chains: dict[str, list[str]] = field(default_factory=dict)
    """chain name → ordered joint names (excluding base and grippers)."""

    grippers: dict[str, GripperSpec] = field(default_factory=dict)

    conf_layout: list[tuple[str, int]] = field(default_factory=list)
    """Canonical conf vector layout: ordered (chain_or_special, length).
    "base" is the [x, y, theta] base pose; "zeros" pads with zeros (e.g. the
    RBY1 wheel slots); gripper names map to their joints."""

    def joints_for(self, name: str) -> list[str]:
        if name in self.chains:
            return self.chains[name]
        if name in self.grippers:
            return self.grippers[name].joints
        raise ValueError(f"Unknown chain '{name}'")


def _load_text_via_tempfile(urdf_path: str, text: str) -> str:
    """Write preprocessed URDF text next to the original so relative mesh
    paths resolve; caller must unlink."""
    fd, tmp = tempfile.mkstemp(suffix=".urdf", dir=os.path.dirname(urdf_path))
    with os.fdopen(fd, "w") as f:
        f.write(text)
    return tmp


def make_world(spec: KinRobotSpec, visualize: bool = False) -> KinematicWorld:
    urdf_path = str(spec.urdf_source)
    tmp = None
    if spec.preprocess is not None:
        tmp = _load_text_via_tempfile(urdf_path, spec.preprocess(open(urdf_path).read()))
        urdf_path = tmp
    try:
        root = spec.root_joint_factory() if spec.root_joint_factory else None
        return KinematicWorld(
            urdf_path,
            root_joint=root,
            package_dirs=list(spec.package_dirs),
            visualize=visualize,
        )
    finally:
        if tmp is not None:
            os.unlink(tmp)


# ── canonical conf ↔ pinocchio q ─────────────────────────────────────────────

def get_base_pose(spec: KinRobotSpec, world: KinematicWorld) -> np.ndarray:
    """[x, y, theta] of the canonical base reference point."""
    if spec.base_mode == "planar_root":
        x, y = world.q[0], world.q[1]
        theta = math.atan2(world.q[3], world.q[2])
    else:
        x = get_joint_value(world.model, world.q, spec.base_joints[0])
        y = get_joint_value(world.model, world.q, spec.base_joints[1])
        theta = get_joint_value(world.model, world.q, spec.base_joints[2])
    off = spec.axle_x_offset
    return np.array([x + off * math.cos(theta), y + off * math.sin(theta), theta])


def set_base_pose(spec: KinRobotSpec, world: KinematicWorld, pose) -> None:
    ax, ay, theta = (float(v) for v in pose)
    off = spec.axle_x_offset
    x = ax - off * math.cos(theta)
    y = ay - off * math.sin(theta)
    if spec.base_mode == "planar_root":
        world.q[0] = x
        world.q[1] = y
        world.q[2] = math.cos(theta)
        world.q[3] = math.sin(theta)
    else:
        set_joint_value(world.model, world.q, spec.base_joints[0], x)
        set_joint_value(world.model, world.q, spec.base_joints[1], y)
        set_joint_value(world.model, world.q, spec.base_joints[2], theta)
    world.update()


def conf_from_world(spec: KinRobotSpec, world: KinematicWorld) -> np.ndarray:
    parts: list[np.ndarray] = []
    for name, n in spec.conf_layout:
        if name == "base":
            parts.append(get_base_pose(spec, world))
        elif name == "zeros":
            parts.append(np.zeros(n))
        else:
            joints = spec.joints_for(name)
            parts.append(np.array([
                get_joint_value(world.model, world.q, j) for j in joints
            ]))
    return np.concatenate(parts)


def set_world_conf(spec: KinRobotSpec, world: KinematicWorld, conf: np.ndarray) -> None:
    conf = np.asarray(conf, dtype=float)
    i = 0
    for name, n in spec.conf_layout:
        vals = conf[i: i + n]
        i += n
        if name == "base":
            set_base_pose(spec, world, vals)
        elif name == "zeros":
            continue
        else:
            for j, v in zip(spec.joints_for(name), vals):
                set_joint_value(world.model, world.q, j, float(v))
    world.update()


def set_chain_conf(spec: KinRobotSpec, world: KinematicWorld,
                   chain: str, values) -> None:
    if chain == "base":
        set_base_pose(spec, world, values)
        return
    joints = spec.joints_for(chain)
    values = np.asarray(values, dtype=float)
    if len(values) != len(joints):
        raise ValueError(
            f"Chain '{chain}' has {len(joints)} joints, got {len(values)} values"
        )
    for j, v in zip(joints, values):
        set_joint_value(world.model, world.q, j, float(v))
    world.update()


# ── RBY1 ──────────────────────────────────────────────────────────────────────

def _preprocess_rby1(text: str) -> str:
    # <capsule> collision geometry is a MuJoCo extension; approximate with a
    # cylinder of the same radius/length.
    text = re.sub(
        r"<capsule\b([^>]*?)/>",
        lambda m: "<cylinder" + re.sub(r'\scoltype="[^"]*"', "", m.group(1)) + "/>",
        text,
    )

    # The wheel joints' <limit> elements lack the mandatory effort/velocity.
    def fix_limit(m):
        attrs = m.group(1)
        if "effort" not in attrs:
            attrs += ' effort="1000"'
        if "velocity" not in attrs:
            attrs += ' velocity="1000"'
        return f"<limit{attrs}/>"

    return re.sub(r"<limit\b([^>]*?)/>", fix_limit, text)


def rby1_spec() -> KinRobotSpec:
    assets = GetQRAssetsPath()
    return KinRobotSpec(
        name=rby1_common.ROBOT_NAME,
        urdf_source=str(assets / "rby1a" / "urdf" / "model.urdf"),
        preprocess=_preprocess_rby1,
        root_joint_factory=pin.JointModelPlanar,
        package_dirs=[str(assets / "rby1a" / "urdf")],
        base_mode="planar_root",
        axle_x_offset=rby1_common.AXLE_X_OFFSET,
        chains={
            "torso": [f"torso_{i}" for i in range(6)],
            "right_arm": [f"right_arm_{i}" for i in range(7)],
            "left_arm": [f"left_arm_{i}" for i in range(7)],
            "head": ["head_0", "head_1"],
        },
        grippers={
            "right": GripperSpec(
                frame="ee_right",
                joints=["gripper_finger_r1", "gripper_finger_r2"],
                open_values=[-0.05, 0.05],
                closed_values=[0.0, 0.0],
            ),
            "left": GripperSpec(
                frame="ee_left",
                joints=["gripper_finger_l1", "gripper_finger_l2"],
                open_values=[-0.05, 0.05],
                closed_values=[0.0, 0.0],
            ),
        },
        # Canonical 31-vector (see qr_robots.common.rby1.chain_slices).
        conf_layout=[
            ("base", 3),
            ("zeros", 2),          # wheel slots
            ("torso", 6),
            ("right_arm", 7),
            ("right", 2),           # right gripper fingers
            ("left_arm", 7),
            ("left", 2),            # left gripper fingers
            ("head", 2),
        ],
    )


# ── Spot ──────────────────────────────────────────────────────────────────────

def _preprocess_spot(text: str) -> str:
    text = text.replace(
        "package://qr_assets/", str(GetQRAssetsPath()) + "/"
    )
    text = re.sub(r"<drake:[^>]*?/>", "", text)
    text = re.sub(r"<drake:(\w+)[^>]*?>.*?</drake:\1>", "", text, flags=re.S)
    text = re.sub(r"<transmission\b.*?</transmission>", "", text, flags=re.S)
    return text


def spot_spec() -> KinRobotSpec:
    assets = GetQRAssetsPath()
    return KinRobotSpec(
        name=spot_common.ROBOT_NAME,
        urdf_source=str(assets / "spot" / "spot_with_arm_and_floating_base_actuators.urdf"),
        preprocess=_preprocess_spot,
        root_joint_factory=None,
        package_dirs=[str(assets)],
        base_mode="joints",
        base_joints=("base_x", "base_y", "base_rz"),
        chains={
            "right_arm": list(spot_common.ARM_JOINT_NAMES),
        },
        grippers={
            "right": GripperSpec(
                frame="arm_link_fngr",
                joints=[spot_common.GRIPPER_JOINT_NAME],
                open_values=[-1.57],
                closed_values=[0.0],
            ),
        },
        # Canonical 10-vector (see qr_robots.common.spot.chain_slices).
        conf_layout=[
            ("base", 3),
            ("right_arm", 6),
            ("right", 1),
        ],
    )


ROBOT_SPECS: dict[str, Callable[[], KinRobotSpec]] = {
    "rby1": rby1_spec,
    "rainbow": rby1_spec,
    "spot": spot_spec,
}
