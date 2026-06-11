"""
Virtual-robot factory for QR problems.

The QDDL world (problem.domain/world/assets) is turned into simulator scene
objects through HPN's Roboverse world model — the same path the old repo's
virtual robots used internally — and the result feeds NQR's ZMQ-backed
virtual robots, which take a plain objects dict.
"""

from __future__ import annotations

import os
import tempfile
import xml.etree.ElementTree as ET
from xml.dom import minidom

import numpy as np
import trimesh

from qr_api.virtual_robot_interfaces import VirtualRobotModule
from qr_problem_defs.problem_def import QRProblem


def build_qddl_world(problem: QRProblem, robot_name: str):
    """Instantiate HPN's kinematic world (Roboverse Physical) from QDDL."""
    from Domains.tamp.state.hpn_qddl_read import extract_world_qddl
    from Domains.tamp.state.world import TampWorld
    from Roboverse.physical_world.physical import Physical
    from Roboverse.skrobot.robot import make_robot

    qddl_paths = [problem.domain, problem.world] + (
        problem.assets if problem.assets is not None else []
    )
    attributes, objects = extract_world_qddl(qddl_paths, robot_override=robot_name)

    robot_att = attributes.get("robot", {})
    robot = make_robot(
        robot_name,
        use_base=robot_att["use_base"],
        use_left=robot_att["use_left"],
        use_right=robot_att["use_right"],
        workspace=robot_att["workspace"],
    )
    world = TampWorld(Physical(robot, "World", use_shadows=False))
    world.phys.initialize_phys(attributes, objects)
    return world


# ── QDDL world → MuJoCo scene objects (port of the old repo's
#    rby1_mujoco_virtual_robot.write_scene_objects) ───────────────────────────

def meshes_to_mujoco_xml(meshes, names, positions, quaternions, output_dir, colors=None):
    """colors: optional list of (r, g, b, a) tuples, values in [0, 1]"""
    os.makedirs(output_dir, exist_ok=True)
    xml_paths = []

    for i, (name, mesh, pos, quat) in enumerate(
        zip(names, meshes, positions, quaternions)
    ):
        mesh.export(os.path.join(output_dir, f"{name}.stl"))

        root = ET.Element("mujoco", model=name)
        ET.SubElement(root, "compiler", meshdir=output_dir, inertiafromgeom="true")

        asset = ET.SubElement(root, "asset")
        ET.SubElement(asset, "mesh", name=name, file=f"{name}.stl")

        worldbody = ET.SubElement(root, "worldbody")
        body = ET.SubElement(
            worldbody,
            "body",
            name=name,
            pos=" ".join(map(str, pos)),
            quat=" ".join(map(str, quat)),
        )

        geom_attrs = {"type": "mesh", "mesh": name}
        if colors is not None:
            geom_attrs["rgba"] = " ".join(map(str, colors[i]))

        ET.SubElement(body, "joint", type="free", name=f"{name}_free", damping="0.01")
        ET.SubElement(body, "geom", name=name, **geom_attrs)

        xml_str = minidom.parseString(ET.tostring(root)).toprettyxml(indent="  ")
        xml_path = os.path.join(output_dir, f"{name}.xml")
        with open(xml_path, "w") as f:
            f.write(xml_str)
        xml_paths.append(xml_path)

    return xml_paths


def write_scene_objects(phys, output_dir):
    def map_color(rgb):
        r, g, b = rgb
        return np.array([r, g, b, 255]) / 255.0

    names = []
    meshes = []
    positions = []
    quaternions = []
    colors = []
    grey = [128, 128, 128]
    for bname, body in phys.get_bodies().items():
        color = map_color(phys.get_body_attr(bname, "rgb_average", default=grey))
        matrix = phys.get_body_trans(bname).T()
        position = matrix[:3, 3]
        quaternion = trimesh.transformations.quaternion_from_matrix(matrix)
        for link in body.shape.link_list:
            if not link.collision_meshes:
                continue
            assert len(link.collision_meshes) == 1
            names.append(link.full_name.replace("::", "__"))
            colors.append(color)
            meshes.append(link.collision_meshes[0])
            positions.append(position)
            quaternions.append(quaternion)
    xml_paths = meshes_to_mujoco_xml(
        meshes, names, positions, quaternions, output_dir, colors=colors
    )
    objects = {
        name: {"file": path, "fixed": ("table" in name or "floor" in name)}
        for name, path in zip(names, xml_paths)
    }
    return objects


def write_scene_objects_kinsim(phys, output_dir):
    """QDDL world → kinematic-sim scene objects: each body's collision mesh
    exported as STL, with world pose, color, and fixed flag (see
    qr_kinsim.sim for the object format)."""
    os.makedirs(output_dir, exist_ok=True)
    grey = [128, 128, 128]
    objects = {}
    for bname, body in phys.get_bodies().items():
        rgb = phys.get_body_attr(bname, "rgb_average", default=grey)
        color = [float(c) / 255.0 for c in rgb] + [1.0]
        matrix = phys.get_body_trans(bname).T()
        position = matrix[:3, 3].tolist()
        quaternion = trimesh.transformations.quaternion_from_matrix(matrix).tolist()
        for link in body.shape.link_list:
            if not link.collision_meshes:
                continue
            assert len(link.collision_meshes) == 1
            name = link.full_name.replace("::", "__")
            mesh_path = os.path.join(output_dir, f"{name}.stl")
            link.collision_meshes[0].export(mesh_path)
            objects[name] = {
                "shape": {"mesh": mesh_path},
                "pos": position,
                "quat": quaternion,
                "color": color,
                "fixed": ("table" in name or "floor" in name),
            }
    return objects


def initial_conf_vector(world) -> np.ndarray:
    """Canonical RBY1 31-vector from the HPN world's initial robot conf, so
    the simulated robot starts where the planner expects it."""
    conf = world.phys.get_conf().value

    def gripper_fingers(opening):
        half = float(np.clip(opening, 0.0, 0.1)) / 2.0
        return [-half, half]

    return np.concatenate([
        np.asarray(conf["base"], dtype=float),
        [0.0, 0.0],   # wheel slots
        np.asarray(conf["torso"], dtype=float),
        np.asarray(conf["right"], dtype=float),
        gripper_fingers(conf["right_gripper"][0]),
        np.asarray(conf["left"], dtype=float),
        gripper_fingers(conf["left_gripper"][0]),
        np.asarray(conf["head"], dtype=float),
    ])


def get_virtual_robot(
    virtual_robot_name: str,
    robot_name: str,
    problem: QRProblem | None = None,
    run_from_pkl_path=None,
    mode: str = "headless",
) -> VirtualRobotModule | None:
    if run_from_pkl_path:
        return None
    if problem is None:
        raise ValueError("Problem must be provided to build the QDDL scene.")

    scene_dir = tempfile.mkdtemp(prefix="qr_scene_")
    world = build_qddl_world(problem, robot_name)

    if virtual_robot_name in ("Ruby_Kinsim", "Rainbow_Kinsim", "Kinsim_Sim"):
        from qr_kinsim.virtual_robot import Rby1KinsimVirtualMoman

        objects = write_scene_objects_kinsim(world.phys, scene_dir)
        robot = Rby1KinsimVirtualMoman(
            objects=objects, model_dir=scene_dir, mode=mode
        )
        robot.client.set_robot_conf(initial_conf_vector(world))
        return robot

    objects = write_scene_objects(world.phys, scene_dir)

    if virtual_robot_name in ("Ruby_Mujoco_Sim", "Rainbow_Mujoco_Sim"):
        from qr_robots.mujoco.rby1.virtual_robot import Rby1MujocoVirtualMoman

        robot = Rby1MujocoVirtualMoman(
            objects=objects, model_dir=scene_dir, mode=mode
        )
        robot.client.set_robot_conf(initial_conf_vector(world))
        return robot
    if virtual_robot_name == "Spot_Mujoco_Sim":
        from qr_robots.mujoco.spot.virtual_robot import SpotMujocoVirtualMoman

        return SpotMujocoVirtualMoman(
            objects=objects, model_dir=scene_dir, mode=mode
        )
    raise ValueError(f"Unknown virtual robot name: {virtual_robot_name}")
