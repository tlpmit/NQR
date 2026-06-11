import os

import numpy as np
import pydrake
import pydrake.math as pym
import trimesh
from lisdf.parsing.qddl import load_qddl
from urdfpy import URDF


def get_static_info(qddl_paths, goal_override=None):
    """
    args:
        qddl_paths: is a list of pathnames; assume first is domain, second is problem, rest are auxiliary
        goal_override: is a string that is a goal expression (optional)
    """
    qddl_domain = qddl_paths[0]
    qddl_problem = qddl_paths[1]
    domain, problem = load_qddl(qddl_domain, qddl_problem)
    if goal_override:
        from lisdf.parsing.qddl import parse_qddl_expression

        # List of parsed QDDL expressions
        goal = parse_qddl_expression(domain, problem, goal_override).arguments
    return (domain, problem, goal if goal_override else None)


def get_problem_feature(static_info, feature_name):
    """Get the value of a zero-ary property out of the problem description
    args:
        static_info : pair of PDDLDomain and PDDLProblem
        reature_name : name of property to find value of
    returns:
        True if feature is present, else False

    """
    return any(prop.predicate.name == feature_name for prop in static_info[1].init)


def get_problem_property(static_info, property_name):
    """Get the value of a unary property out of the problem description
    args:
        static_info : pair of PDDLDomain and PDDLProblem
        property_name : name of property to find value of
    returns:
        value of given property in problem
    raises:
        Exception if no property is declared

    Assumes exactly one predicate of the form (predicate ...)
    """

    def argval(a):
        return a.name if hasattr(a, "name") else a.value

    (_domain, problem, goal) = static_info
    init = problem.init
    for prop in init:
        if prop.predicate.name == property_name:
            return argval(prop.arguments[0])
    raise Exception(f"No {property_name} declared in domain file")


def get_problem_properties(static_info, property_name):
    """Get the value of a unary property out of the problem description
    args:
        static_info : pair of PDDLDomain and PDDLProblem
        property_name : name of property to find value of
    returns:
        value of given property in problem
    raises:
        Exception if no property is declared

    Assumes exactly one predicate of the form (predicate ...)
    """

    def argval(a):
        return a.name if hasattr(a, "name") else a.value

    (_domain, problem, goal) = static_info
    init = problem.init
    return [
        [argval(arg) for arg in prop.arguments]
        for prop in init
        if prop.predicate.name == property_name
    ]


def get_goal(static_info):
    """Extract the goal from a problem definition
    args:
        static_info : pair of PDDLDomain and PDDLProblem
    returns:
        instance of PDDLThing
    """
    (_domain, problem, goal) = static_info
    return goal if goal is not None else problem.conjunctive_goal


def get_object_attrs(static_info):
    """Extract object attributes from problem
    args:
        static_info : pair of PDDLDomain and PDDLProblem
    returns:
        dictionary from object names to dicitonary of attribute to value

    Only works for true properties.
    """
    init = static_info[1].init
    result = {}
    for prop in init:
        if len(prop.arguments) == 1 and hasattr(prop.arguments[0], "name"):
            obj = prop.arguments[0].name
            if obj not in result:
                result[obj] = {}
            result[obj][prop.predicate.name] = True
    return result


def quat_xyzw2wxyz(quat):
    return (quat[3], *(list(quat)[:3]))


def quat_wxyz2xyzw(quat):
    return (*(list(quat)[1:]), quat[0])


def matrix4_from_pos_quat(position, quat_xyzw):
    # drake: wxyz, pybullet: xyzw, ROS2(tf2): xyzw
    quat = np.array(quat_xyzw2wxyz(quat_xyzw))
    quat_orientation = pydrake.common.eigen_geometry.Quaternion(
        quat.reshape(4, 1)
    )  # TODO is the input wxyz?
    pydrake_tform = pydrake.math.RigidTransform(
        quaternion=quat_orientation, p=np.array(position).reshape(3, 1)
    )
    return pydrake_tform.GetAsMatrix4()


def mesh_from_boxgeom(box_size, color=None, tmp_dirname="./tmp") -> trimesh.Trimesh:
    mesh = trimesh.creation.box(extents=box_size, color=color)
    print("exporting urdf")
    # # https://github.com/mikedh/trimesh/blob/556c2b41071f5cc727a2584df193e8967aa1f657/trimesh/exchange/urdf.py
    # trimesh.exchange.urdf.export_urdf(mesh, directory=tmp_dirname, color=color)
    tmp_urdf_path = convexmesh2urdf(mesh, directory=tmp_dirname, color=color)
    print("urdf done")
    import numpy

    numpy.float = numpy.float64
    urdf = URDF.load(tmp_urdf_path)
    return urdf


def convexmesh2urdf(mesh, directory, scale=1.0, color=None, **kwargs):
    """
    Convert a Trimesh object into a URDF package for physics
    simulation. This breaks the mesh into convex pieces and
    writes them to the same directory as the .urdf file.
    Parameters
    ---------
    mesh : trimesh.Trimesh
      Input geometry
    directory : str
      The directory path for the URDF package
    Returns
    ---------
    mesh : Trimesh
      Multi-body mesh containing convex decomposition
    """

    import lxml.etree as et

    # TODO: fix circular import
    from trimesh.exchange.export import export_mesh

    # from ..resources import get
    # Extract the save directory and the file name
    fullpath = os.path.abspath(directory)
    name = os.path.basename(fullpath)
    _, ext = os.path.splitext(name)

    if ext != "":
        raise ValueError("URDF path must be a directory!")

    # Create directory if needed
    if not os.path.exists(fullpath):
        os.mkdir(fullpath)
    elif not os.path.isdir(fullpath):
        raise ValueError("URDF path must be a directory!")

    convex_pieces = [mesh.convex_hull]

    # Get the effective density of the mesh
    effective_density = mesh.volume / sum([m.volume for m in convex_pieces])

    # open an XML tree
    root = et.Element("robot", name="root")

    # Loop through all pieces, adding each as a link
    prev_link_name = None
    for i, piece in enumerate(convex_pieces):
        # Save each nearly convex mesh out to a file
        piece_name = "{}_convex_piece_{}".format(name, i)
        piece_filename = "{}.obj".format(piece_name)
        piece_filepath = os.path.join(fullpath, piece_filename)
        export_mesh(piece, piece_filepath)

        # Set the mass properties of the piece
        piece.center_mass = mesh.center_mass
        piece.density = effective_density * mesh.density

        link_name = "link_{}".format(piece_name)
        geom_name = "{}".format(piece_filename)
        Im = [
            ["{:.2E}".format(y) for y in x]  # NOQA
            for x in piece.moment_inertia
        ]

        # Write the link out to the XML Tree
        link = et.SubElement(root, "link", name=link_name)

        # Inertial information
        inertial = et.SubElement(link, "inertial")
        et.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
        et.SubElement(inertial, "mass", value="{:.2E}".format(piece.mass))
        et.SubElement(
            inertial,
            "inertia",
            ixx=Im[0][0],
            ixy=Im[0][1],
            ixz=Im[0][2],
            iyy=Im[1][1],
            iyz=Im[1][2],
            izz=Im[2][2],
        )
        # Visual Information
        visual = et.SubElement(link, "visual")
        et.SubElement(visual, "origin", xyz="0 0 0", rpy="0 0 0")
        geometry = et.SubElement(visual, "geometry")
        et.SubElement(
            geometry,
            "mesh",
            filename=geom_name,
            scale="{:.4E} {:.4E} {:.4E}".format(scale, scale, scale),
        )
        material = et.SubElement(visual, "material", name="")
        if color is not None:
            et.SubElement(
                material,
                "color",
                rgba="{:.2E} {:.2E} {:.2E} 1".format(color[0], color[1], color[2]),
            )

        # Collision Information
        collision = et.SubElement(link, "collision")
        et.SubElement(collision, "origin", xyz="0 0 0", rpy="0 0 0")
        geometry = et.SubElement(collision, "geometry")
        et.SubElement(
            geometry,
            "mesh",
            filename=geom_name,
            scale="{:.4E} {:.4E} {:.4E}".format(scale, scale, scale),
        )

        # Create rigid joint to previous link
        if prev_link_name is not None:
            joint_name = "{}_joint".format(link_name)
            joint = et.SubElement(root, "joint", name=joint_name, type="fixed")
            et.SubElement(joint, "origin", xyz="0 0 0", rpy="0 0 0")
            et.SubElement(joint, "parent", link=prev_link_name)
            et.SubElement(joint, "child", link=link_name)

        prev_link_name = link_name

    # Write URDF file
    tree = et.ElementTree(root)
    urdf_filename = "{}.urdf".format(name)
    urdf_filepath = os.path.join(fullpath, urdf_filename)
    tree.write(urdf_filepath, pretty_print=True)

    # Write Gazebo config file
    root = et.Element("model")
    model = et.SubElement(root, "name")
    model.text = name
    version = et.SubElement(root, "version")
    version.text = "1.0"
    sdf = et.SubElement(root, "sdf", version="1.4")
    sdf.text = "{}.urdf".format(name)

    author = et.SubElement(root, "author")
    et.SubElement(author, "name").text = "trimesh {}".format(
        trimesh.version.__version__
    )
    et.SubElement(author, "email").text = "blank@blank.blank"

    description = et.SubElement(root, "description")
    description.text = name
    tree = et.ElementTree(root)
    #
    # if tol.strict:
    #     # todo : we don't pass the URDF schema validation
    #     schema = et.XMLSchema(file=get(
    #         'schema/urdf.xsd', as_stream=True))
    #     if not schema.validate(tree):
    #         # actual error isn't raised by validate
    #         log.debug(schema.error_log)

    tree.write(os.path.join(fullpath, "model.config"))
    return urdf_filepath  # np.sum(convex_pieces)


def mesh_from_urdf(urdf_path, return_all=False, scale=1.0) -> trimesh.Trimesh:
    urdf = URDF.load(urdf_path)
    # urdf = load_urdf(urdf_path)
    if return_all:
        # TODO scale
        return urdf
    return urdf


def get_pose_from_value(pose):
    pos, rpy = pose[:3], pose[3:]
    return tuple(pos), quat_wxyz2xyzw(
        pydrake.math.RollPitchYaw(rpy.reshape(3, 1)).ToQuaternion().wxyz().reshape(4)
    )


# a and b are lists of angles
def within_angles(a, b, eps):
    return all(within_angle(ai, bi, eps) for (ai, bi) in zip(a, b))


# a and b are single angles
def within_angle(a, b, eps):
    return abs(put_in_pm_pi(a - b)) < eps


def within_dist(a, b, eps):
    return abs(a - b) < eps


# types is a string with characters 'a' and 'd' (for angle and distance)
# a and be are vectors of same length as string of values
# eps is a vector, as well
def within(types, a, b, eps):
    return all(
        (within_dist(ai, bi, ei) if ti == "d" else within_angle(ai, bi, ei))
        for (ti, ai, bi, ei) in zip(list(types), a, b, eps)
    )


def diff_pose2D(p1, p2):
    (x1, y1, z1) = p1
    (x2, y2, z2) = p2
    return (x1 - x2, y1 - y2, put_in_pm_pi(z1 - z2))


def put_in_pm_pi(z):
    if z > np.pi:
        return put_in_pm_pi(z - 2 * np.pi)
    elif z < -np.pi:
        return put_in_pm_pi(z + 2 * np.pi)
    else:
        return z


def max_joint_dist(q1, q2):
    return max(abs(put_in_pm_pi(q1i - q2j)) for (q1i, q2j) in zip(q1, q2))


def m4_to_drake_rt(m4):
    rot = pym.RotationMatrix(m4[:3, :3])
    trans = m4[:3, 3]
    return pym.RigidTransform(rot, trans)
