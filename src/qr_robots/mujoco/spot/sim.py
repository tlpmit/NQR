"""
Spot MuJoCo simulation server.

There is no MJCF model for Spot in our assets, so the Drake URDF
(planar floating base + 6-DOF arm + 1-DOF gripper, fixed legs) is rewritten
on the fly into a MuJoCo-loadable URDF and position actuators are injected
programmatically via MjSpec.

    mjpython -m qr_robots.mujoco.spot.sim \\
        --objects '{"block":"block.xml"}' --mode display --port 5558
"""

from __future__ import annotations

import argparse
import json
import os
import re
import tempfile

import mujoco
import numpy as np

from qr_robots.common import spot
from qr_robots.common.zmq_sim import serve
from qr_robots.mujoco.sim_base import MujocoRobotSim, MujocoRobotSpec
from qr_utils import GetQRAssetsPath

ROBOT_URDF = GetQRAssetsPath() / "spot" / "spot_with_arm_and_floating_base_actuators.urdf"

# PD gains for injected position actuators (match the MJCF defaults used for
# the RBY1 model: kp=4000, kv=400; lighter for the gripper).
_ARM_KP, _ARM_KV = 4000.0, 400.0
_BASE_KP, _BASE_KV = 20000.0, 2000.0
_GRIPPER_KP, _GRIPPER_KV = 500.0, 50.0

# Continuous joints have no URDF limits; give base yaw a generous ctrlrange.
_FREE_CTRL_RANGE = (-100.0, 100.0)

SPOT_SPEC = MujocoRobotSpec(
    name="spot",
    robot_xml=str(ROBOT_URDF),
    chain_patterns={
        # The arm joints are arm_sh0 … arm_wr1; arm_f1x is the gripper and is
        # excluded from the chain by matching it as a gripper actuator first.
        "right_arm": "arm_",
    },
    gripper_act_prefixes={
        "right": ("arm_f1x",),
    },
    base_mode="joints",
    base_joint_names=("base_x", "base_y", "base_rz"),
)


_MESH_CACHE_DIR = "mujoco_mesh_cache"


def _parse_mtl_colors(mtl_path: str) -> dict[str, tuple[float, float, float]]:
    """Material name → diffuse (Kd) color from a .mtl file."""
    colors: dict[str, tuple[float, float, float]] = {}
    current = None
    if not os.path.exists(mtl_path):
        return colors
    for line in open(mtl_path):
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "newmtl":
            current = parts[1]
        elif parts[0] == "Kd" and current is not None:
            colors[current] = tuple(float(v) for v in parts[1:4])
    return colors


def _split_obj_by_material(path: str) -> list[tuple[str, tuple[float, float, float]]]:
    """
    Split a multi-material OBJ into one cached single-group OBJ per material,
    returning [(sub_mesh_path, diffuse_rgb), ...].

    MuJoCo's OBJ loader keeps only the first object/material group of a file
    (body.obj silently loses ~6.5k of 33k faces → holes in the model), and it
    ignores .mtl materials.  Splitting fixes both: every face is loaded, and
    each sub-mesh gets its material's color via the URDF.
    """
    cache_dir = os.path.join(
        str(GetQRAssetsPath() / "spot_description_drake"), _MESH_CACHE_DIR
    )
    os.makedirs(cache_dir, exist_ok=True)
    stem = os.path.splitext(os.path.basename(path))[0]

    header: list[str] = []
    buckets: dict[str, list[str]] = {}
    mtl_colors: dict[str, tuple[float, float, float]] = {}
    current = "default"
    for line in open(path):
        key = line.split(" ", 1)[0].strip()
        if key == "mtllib":
            mtl_path = os.path.normpath(
                os.path.join(os.path.dirname(path), line.split(None, 1)[1].strip())
            )
            mtl_colors.update(_parse_mtl_colors(mtl_path))
        elif key == "usemtl":
            current = line.split(None, 1)[1].strip()
        elif key in ("o", "g", "s"):
            continue
        elif key == "f":
            buckets.setdefault(current, []).append(line)
        else:
            header.append(line)

    out: list[tuple[str, tuple[float, float, float]]] = []
    for mtl, faces in buckets.items():
        sub = os.path.join(cache_dir, f"{stem}__{mtl}.obj")
        if not (os.path.exists(sub)
                and os.path.getmtime(sub) >= os.path.getmtime(path)):
            with open(sub, "w") as f:
                f.writelines(header)
                f.writelines(faces)
        out.append((sub, mtl_colors.get(mtl, (0.5, 0.5, 0.5))))
    return out


def _rewrite_urdf_for_mujoco(urdf_path: str) -> str:
    """
    Rewrite the Drake Spot URDF into a MuJoCo-loadable temp URDF:
    - resolve package://qr_assets/ mesh URIs to absolute paths,
    - expand each visual mesh into per-material sub-meshes with their .mtl
      diffuse colors (see _split_obj_by_material),
    - strip <drake:...> extension elements,
    - inject a <mujoco><compiler .../></mujoco> block (mass/inertia bounds for
      the massless dummy base links, discarding visuals keeps collision meshes).
    Returns the temp file path.
    """
    assets_root = str(GetQRAssetsPath())
    text = open(urdf_path).read()

    text = text.replace("package://qr_assets/", f"{assets_root}/")

    # arm_link_hr0's visual mesh is an exact duplicate of arm_link_sh1's
    # (same faces, zero vertex distance) — rendering both z-fights and makes
    # the upper arm flicker.  Keep sh1's copy only.
    duplicate_visuals = {"arm_link_hr0.obj"}

    def expand_visual(m: re.Match) -> str:
        if os.path.basename(m.group("mesh")) in duplicate_visuals:
            return ""
        origin = m.group("origin") or ""
        blocks = []
        for sub, (r, g, b) in _split_obj_by_material(m.group("mesh")):
            mtl = os.path.splitext(os.path.basename(sub))[0]
            blocks.append(
                f"<visual>{origin}<geometry>"
                f'<mesh filename="{sub}"/></geometry>'
                f'<material name="{mtl}"><color rgba="{r} {g} {b} 1"/></material>'
                "</visual>"
            )
        return "\n    ".join(blocks)

    text = re.sub(
        r'<visual>\s*(?P<origin><origin[^>]*/>)?\s*<geometry>\s*'
        r'<mesh filename="(?P<mesh>[^"]*/visual/[^"]*\.obj)"\s*/>'
        r"\s*</geometry>\s*</visual>",
        expand_visual,
        text,
    )

    # Strip drake: extension elements (collision filters, proximity props).
    text = re.sub(r"<drake:[^>]*?/>", "", text)
    text = re.sub(r"<drake:(\w+)[^>]*?>.*?</drake:\1>", "", text, flags=re.S)
    # Strip transmissions: actuators are injected via MjSpec instead.
    text = re.sub(r"<transmission\b.*?</transmission>", "", text, flags=re.S)

    if "<mujoco>" not in text:
        # fusestatic must stay off: fusing the static base_z body silently
        # drops its 0.52 m elevation (and fused URDF specs crash MjSpec.attach).
        mujoco_block = (
            "<mujoco>\n"
            '  <compiler balanceinertia="true" boundmass="0.001" '
            'boundinertia="1e-08" discardvisual="false" strippath="false" '
            'fusestatic="false"/>\n'
            "</mujoco>\n"
        )
        # The <mujoco> extension block must sit inside <robot>.
        text = re.sub(r"(<robot name=[^>]*>)", r"\1\n" + mujoco_block, text, count=1)

    fd, tmp_path = tempfile.mkstemp(suffix=".urdf", prefix="spot_mujoco_")
    with os.fdopen(fd, "w") as fh:
        fh.write(text)
    return tmp_path


class SpotMujocoSim(MujocoRobotSim):
    """MujocoRobotSim with URDF preprocessing and actuator/camera injection."""

    def _load_robot_spec(self) -> mujoco.MjSpec:
        tmp_urdf = _rewrite_urdf_for_mujoco(str(self._spec.robot_xml))
        try:
            spec = mujoco.MjSpec.from_file(tmp_urdf)
        finally:
            os.unlink(tmp_urdf)

        self._add_actuators(spec)
        self._add_cameras(spec)
        self._regroup_geoms(spec)
        self._add_floor(spec)
        return spec

    @staticmethod
    def _add_light(spec: mujoco.MjSpec) -> None:
        """The URDF defines no lights; add a sun-like directional light so
        the scene isn't lit by the headlight alone."""
        light = spec.worldbody.add_light()
        light.name = "sun"
        light.pos = [0.0, 0.0, 4.0]
        light.dir = [0.2, 0.2, -1.0]
        light.castshadow = True

    @staticmethod
    def _regroup_geoms(spec: mujoco.MjSpec) -> None:
        """URDF import puts visual geoms in group 1 and collision geoms in
        group 0 — both rendered by default, so the coarse collision meshes
        draw over the visual ones.  Move them to the standard MJCF
        convention: visuals in group 2, collision in group 3 (hidden)."""
        for geom in spec.geoms:
            geom.group = 3 if geom.contype or geom.conaffinity else 2

    @staticmethod
    def _add_floor(spec: mujoco.MjSpec) -> None:
        """
        The URDF has no ground; add a plane so scene objects don't fall
        forever.  The base height is fixed (base_z is welded at the standing
        height) and the leg/body collision meshes dip below z=0, so floor
        contact would jam the kinematic base.  Contact bitmasks make the
        floor collide with scene objects (contype/conaffinity 1) but not
        with the robot:

            floor : contype=2, conaffinity=1
            robot : contype=4, conaffinity=1
            object: contype=1, conaffinity=1   (MuJoCo defaults)

        robot↔floor and robot↔robot: no contact (the URDF's collision-filter
        groups were Drake extensions; without them the stowed arm jams on the
        legs/body).  robot↔object and floor↔object both collide.
        """
        for geom in spec.geoms:
            if geom.contype:
                geom.contype = 4
                geom.conaffinity = 1

        floor = spec.worldbody.add_geom()
        floor.name = "floor"
        floor.type = mujoco.mjtGeom.mjGEOM_PLANE
        floor.size = [20.0, 20.0, 0.1]
        floor.rgba = [0.35, 0.4, 0.45, 1.0]
        floor.contype = 2
        floor.conaffinity = 1
        SpotMujocoSim._add_light(spec)

    @staticmethod
    def _add_actuators(spec: mujoco.MjSpec) -> None:
        """Add a position actuator for every movable joint.

        Joints also get armature and damping — the URDF base links are
        nearly massless, and stiff PD on massless links makes the
        integration blow up otherwise.
        """
        for joint in spec.joints:
            if joint.type == mujoco.mjtJoint.mjJNT_FREE:
                continue
            if joint.name.startswith("base_"):
                kp, kv = _BASE_KP, _BASE_KV
                joint.armature = 10.0
                joint.damping = [100.0, 0, 0]
            elif joint.name == spot.GRIPPER_JOINT_NAME:
                kp, kv = _GRIPPER_KP, _GRIPPER_KV
                joint.armature = 0.1
                joint.damping = [2.0, 0, 0]
            else:
                kp, kv = _ARM_KP, _ARM_KV
                joint.armature = 0.5
                joint.damping = [10.0, 0, 0]

            act = spec.add_actuator()
            act.name = joint.name
            act.trntype = mujoco.mjtTrn.mjTRN_JOINT
            act.target = joint.name
            # Position servo: gain kp, bias [0, -kp, -kv].
            act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
            act.gainprm[0] = kp
            act.biastype = mujoco.mjtBias.mjBIAS_AFFINE
            act.biasprm[0] = 0.0
            act.biasprm[1] = -kp
            act.biasprm[2] = -kv
            lo, hi = float(joint.range[0]), float(joint.range[1])
            act.ctrllimited = True
            if lo == 0.0 and hi == 0.0:   # unlimited (continuous) joint
                act.ctrlrange = _FREE_CTRL_RANGE
            else:
                act.ctrlrange = (lo, hi)

    @staticmethod
    def _add_cameras(spec: mujoco.MjSpec) -> None:
        """
        Add RGB-D cameras.  The hand camera lives in the gripper jaw, looking
        along the wrist +X axis (MuJoCo cameras look along -Z, so rotate the
        camera frame: +X_wrist = -Z_cam, +Y_cam up).
        """
        wrist = spec.body("arm_link_wr1")
        cam = wrist.add_camera()
        cam.name = "hand_camera"
        cam.pos = [0.16, 0.0, 0.025]
        # Map camera -Z → body +X (forward), camera +Y → body +Z (up):
        # columns of R are camera axes in body frame.
        cam.quat = _quat_from_matrix(np.array([
            [0.0, 0.0, -1.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ]).T)
        cam.fovy = 60.0

        body = spec.body("body")
        front = body.add_camera()
        front.name = "body_camera"
        front.pos = [0.45, 0.0, 0.05]
        front.quat = _quat_from_matrix(np.array([
            [0.0, 0.0, -1.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ]).T)
        front.fovy = 70.0


def _quat_from_matrix(R: np.ndarray) -> list[float]:
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, dtype=float).flatten())
    return q.tolist()


def make_sim(model_dir: str = ".", objects: dict | None = None,
             mode: str = "headless", **kwargs) -> SpotMujocoSim:
    return SpotMujocoSim(SPOT_SPEC, model_dir, objects, mode=mode,
                         camera_size=(480, 640), **kwargs)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(
        description="Spot MuJoCo ZMQ server.  Start with mjpython on macOS for display mode."
    )
    ap.add_argument("--model-dir", default=".",
                    help="Directory that object file paths are relative to.")
    ap.add_argument("--objects", default="{}",
                    help="JSON string or path to a JSON file describing scene objects.")
    ap.add_argument("--mode", default="headless", choices=["headless", "display"])
    ap.add_argument("--port", type=int, default=spot.MUJOCO_PORT)
    args = ap.parse_args()

    if os.path.isfile(args.objects):
        with open(args.objects) as f:
            objects = json.load(f)
    else:
        objects = json.loads(args.objects)

    sim = make_sim(args.model_dir, objects, mode=args.mode)
    serve(sim, port=args.port, mode=args.mode, name="spot_mujoco_sim")
