import copy
import time

import CogMan.configuration as cogman_config

# HPN imports
import Domains.tamp.configuration as tamp_config
import numpy as np
import RetroPlan.configuration as plan_config
from CogMan.policy import HPN_goal_gen, make_HPN_action_selector
from Domains.btamp.bel_hpn_qddl_read import extract_world_qddl, make_retroplan_bel_goal
from Domains.btamp.constraints import (
    IsSafePickKin,
    IsSafePickPath,
    get_visible_count,
)
from Domains.btamp.fluents import CanPath, FluentConj, Pose6
from Domains.btamp.operators import PICK_TOLERANCE, get_btamp_ops
from Domains.fully_observed_tamp.set_vars import set_config_vars, set_debugging
from Domains.tamp.domain_constraints import movePath
from Domains.tamp.fluents import RConf
from Domains.tamp.heuristic_split import h_split
from Domains.tamp.state.bel_from_QR import QR_bel_to_phys
from Domains.tamp.state.hpn_qddl_read import parse_goal_expression
from qr_api.belief_interfaces import BeliefModule
from qr_api.control_typing import CartesianPoseCommand, JointPositionCommand
from qr_api.policy_interfaces import PolicyModule
from qr_api.policy_typing import (
    ActionRV,
    ObjectTransitionPrediction,
    PlanFailedException,
    PlanFinishedException,
    TransitionPrediction,
)
from qr_api.sensor_typing import Observation
from qr_api.virtual_robot_interfaces import VirtualRobotModule
from qr_main.main_virtual_robot import ActionVR
from qr_utils.cog_utils import (
    get_goal,
    get_problem_feature,
    get_problem_properties,
    get_problem_property,
)
from qr_utils.traceFile import tr, tr_a
from RetroPlan.heuristic import H_unmet, hao
from RetroPlan.planner import CGoal, dummy_helpful_H, plan
from Roboverse.skrobot.region import bbox_from_region_name, make_region_from_name
from Roboverse.skrobot.robot_conf import Conf
from Roboverse.utils.bbox import bboxStr, bboxUnion
from trimesh.transformations import rotation_matrix, translation_matrix
from Utils.hpn_exception import HPN_assert, HPN_RuntimeError
from Utils.tree_drawing import write_coda, write_preamble

from qr_policy.hpn_belief_view import BeliefViewForHPN

try:
    from Domains.btamp.constraints import (
        IsViewConf_sampler_aux as IsViewKin_sampler_aux,
    )
except Exception:
    from Domains.btamp.constraints import IsViewKin_sampler_aux


class HPNAction(ActionVR):
    def pkl_copy(self):
        pkl_action = copy.copy(self)
        pkl_action.policy_module = None
        pkl_action.preimage_and_bindings = None
        return pkl_action


# Return value
class HPNActionRV(ActionRV):
    def __init__(self, success: bool, t):
        self.success = success
        self.final_time = t


# Dummy action : noop
class NOP(HPNAction):
    def __init__(self):
        self.name = "nop"

    def transition_prediction(
        self, belief_module: BeliefModule, action_rv: ActionRV
    ) -> TransitionPrediction:
        return TransitionPrediction(object_predictions=[])  # No change in belief


class Move(HPNAction):
    def __init__(self, policy_module, path_prog):
        self.name = "move"
        self.path_prog = path_prog
        self.policy_module = policy_module

    def fix_gripper_settings(self):
        phys = self.policy_module.planning_bel.phys
        # Fix potential gripper problems
        for hand, thing in phys.get_attached().items():
            gripper_name = f"{hand}_gripper"
            if thing is not None:
                gw = phys.get_conf()[gripper_name]
                self.path_prog.path_exec = [
                    [
                        "move",
                        [
                            (q.set_grip(gripper_name, gw), a, b)
                            for (q, a, b) in self.path_prog.path_exec[0][1]
                        ],
                    ]
                ]

    def execute(
        self, belief_module: BeliefModule, virtual_robot: VirtualRobotModule
    ) -> None:
        assert self.path_prog.op == "move", f"Unexpected path type: {self.path_prog.op}"
        phys = self.policy_module.planning_bel.phys
        self.fix_gripper_settings()
        return execute_path_prog(phys, self.path_prog, virtual_robot)


class MoveWithSensing(Move):
    def __init__(self, policy_module, path_prog):
        self.name = "move_with_sensing"
        self.path_prog = path_prog
        self.policy_module = policy_module

    def execute(
        self, belief_module: BeliefModule, virtual_robot: VirtualRobotModule
    ) -> None:
        self.fix_gripper_settings()
        return move_look_execute(
            self.path_prog, self.policy_module, belief_module, virtual_robot
        )


def execute_path(phys, virtual_robot, path):
    obs_path_prog = movePath(
        phys, path_type="move", path_confs=path, ignore_legal_path=True
    )
    obs_path_prog.planned_move_path = ("move", tuple([(q, None, None) for q in path]))
    obs_path_prog.planned_path_exec = True
    return execute_path_prog(phys, obs_path_prog, virtual_robot)


def execute_path_and_observe(policy_module, belief_module, virtual_robot, path):
    phys = policy_module.planning_bel.phys
    rv = execute_path(phys, virtual_robot, path)
    # Get observation and update the belief
    obs = policy_module._get_observation(include_images=True)
    # do belief update based on obs
    belief_module.update(NOP(), rv, obs)
    policy_module.update(NOP(), rv, obs)
    if not phys.get_conf().near_equal(path[-1], 1e-3):
        input("Did not reach the final conf")
    return rv


def move_look_execute(path_prog, policy_module, belief_module, virtual_robot):
    def check_path(phys, path, tag, ignore_shadows=True):
        for q in path:  # consider skipping some
            s, m = phys.cc.check_all_robot_collision(q, ignore_shadows=ignore_shadows)
            if s or any(not o.startswith("shadow") for o in m):
                input(f"Path {tag} should have been clear {s=} {m=}")

    def construct_box_to_view_and_free_path_prefix():
        check_path(phys, path_prog.get_move_path(), "before")
        # Find shadow collisions if any along the path
        path = path_prog.get_smoothed_path(phys, ignore_shadows=True)
        check_path(phys, path, "after")
        prefix = []
        shadow_bbox = None
        shadow_collisions = 0
        first_colliding_base = None
        for q in path:  # consider skipping some
            s, m = phys.cc.check_all_robot_collision(q)
            for v in m:
                if v.startswith("shadow"):
                    shadow_collisions += 1
                    if first_colliding_base is None:
                        first_colliding_base = q["base"]
                    bbox = bbox_from_region_name(v)
                    if shadow_bbox is None:
                        shadow_bbox = bbox
                    else:
                        shadow_bbox = bboxUnion([shadow_bbox, bbox])
                else:
                    assert None, "Path should have been clear"
            colliion_path_length = (
                np.linalg.norm(q["base"] - first_colliding_base)
                if first_colliding_base is not None
                else 0
            )
            if colliion_path_length > 0.5:
                break
            if not shadow_collisions:
                prefix.append(q)
        return shadow_bbox, prefix

    assert path_prog.op == "move_look", f"Unexpected path type: {path_prog.op}"

    q_initial = path_prog.path_key[0][0]
    q_final = path_prog.path_key[-1][0]
    phys = policy_module.planning_bel.phys

    # See if there's a free path to the final goal
    free_path = phys.cc.cspace_path(q_initial, q_final, ignore_shadows=False)
    if free_path:
        return execute_path(phys, virtual_robot, free_path)

    max_tries = 10
    # We start out with a path to q_final
    for i in range(max_tries):
        shadow_bbox, prefix = construct_box_to_view_and_free_path_prefix()
        if shadow_bbox is None:
            # Our way is clear!
            return execute_path_prog(phys, path_prog, virtual_robot)
        if i == max_tries - 1:
            # Don't plan if we are going to fail
            break

        # Find a conf on the prefix where we could see the collision (via observe)
        path_to_obs_conf = find_path_to_obs_conf(phys, prefix, shadow_bbox, q_final)
        if path_to_obs_conf is None:
            return HPNActionRV(False, time.time())
        rv = execute_path_and_observe(
            policy_module, belief_module, virtual_robot, path_to_obs_conf
        )
        phys = policy_module.planning_bel.phys
        if not rv.success:
            tr_a("move_look_planner: Intermediate path execution failed")
            return HPNActionRV(False, time.time())

        # See if we can go straight there first
        free_path = phys.cc.cspace_path(phys.get_conf(), q_final, ignore_shadows=False)
        if free_path:
            return execute_path(phys, virtual_robot, free_path)

        # Find a new path through shadows to the original goal and repeat.
        new_path = phys.cc.cspace_path(phys.get_conf(), q_final, ignore_shadows=True)
        if new_path is None:
            tr_a(
                "move_look_planner: Could not find a path to the final goal ignoring shadows"
            )
            return HPNActionRV(False, time.time())

        check_path(phys, new_path, "to")

        # New nominal path is new_path; package it up in a path_prog.
        # We already checked the path so no need to do it again - movePath would only do perm_only.
        path_prog = movePath(
            phys, path_type="move_look", path_confs=new_path, ignore_legal_path=True
        )
        path_prog.planned_move_path = (
            "move",
            tuple([(q, None, None) for q in new_path]),
        )
        path_prog.planned_path_exec = True
    tr_a(f"move_look_planner: Could not reach the final goal in {max_tries} tries")
    return HPNActionRV(False, time.time())


def check_path(phys, path):
    for q in path:
        s, m = phys.cc.check_all_robot_collision(q)
        if s:
            phys.draw(q)
            input(f"Perm collision{s}")
        if m:
            if any("shadow_bbox" not in o for o in m):
                phys.draw(q)
                input("Non shadow movable collision", m)


def find_path_to_obs_conf(phys, prefix, shadow_bbox, q_final):
    def rotated_base_conf(conf, point):
        diff_xy = point[:2] - conf["base"][:2]
        angle = np.arctan2(diff_xy[1], diff_xy[0])
        new_base = conf["base"].copy()
        new_base[2] = angle
        return conf.set_chain("base", new_base)

    viewer = phys.get_viewer()
    viewer.add_region(
        q_final.bbox(), color=np.array((0, 0, 250))
    )  # blue : ultimate target conf
    region_name = f"shadow_bbox_{bboxStr(shadow_bbox)}"
    viewer.add_region(
        region_name, color=np.array((250, 150, 250))
    )  # pink region we want to view
    shadow_region = make_region_from_name(region_name)
    if phys.robot.name.lower() in ["rainbow", "pr2", "movohpn"]:
        best_conf_i = None
        best_count = 0
        best_look_conf = None
        n_steps = min(len(prefix), 10)
        step = len(prefix) // n_steps
        indices = range(0, len(prefix), step)
        for step, i in enumerate(indices):
            conf = prefix[i]
            rot_base_conf = rotated_base_conf(conf, shadow_region.centroid())
            # Find one look_conf and compute its count of visible pixels.
            for sensor_name, _, look_conf in IsViewKin_sampler_aux(
                phys, shadow_region, base_conf=rot_base_conf
            ):
                count = get_visible_count(phys, sensor_name, look_conf, shadow_region)
                print("Visible pixel count:", count)
                if count > 100 and phys.cc.cspace_path(
                    prefix[i], look_conf, only_direct=True
                ):
                    best_count = count
                    best_conf_i = i
                    best_look_conf = look_conf
                break
            if best_count > 100 and step > 5:
                break
        if best_conf_i is not None:
            viewer.add_region(
                best_look_conf.bbox(), color=np.array((0, 250, 0))
            )  # green look conf on path
            return phys.robot.interpolate_path(
                prefix[: best_conf_i + 1] + [best_look_conf]
            )
        tr_a("Could not find look_conf for region in path")
    tries = 0
    max_tries = 3
    for sensor_name, _, look_conf in IsViewKin_sampler_aux(phys, shadow_region):
        viewer.add_region(
            look_conf.bbox(), color=np.array((250, 0, 0))
        )  # red random look conf
        tries += 1
        new_path = phys.cc.cspace_path(phys.get_conf(), look_conf, ignore_shadows=False)
        if new_path:
            return new_path
        if tries >= max_tries:
            break
    tr_a("Could not find look_conf for region!")
    pass


ATTEMPTED_PICK_DELTA = 0.04  # How much the object is disturbed by a failed pick
PLACE_DELTA = (
    0.01  # How much uncertainty we have in object pose after a successful place
)
ATTEMPTED_PLACE_DELTA = 0.1  # How much the object is disturbed by a failed place


class PickHand(HPNAction):
    def __init__(self, policy_module, obj: str, hand: str, grasp, path_prog):
        self.name = "pick_hand"
        self.obj = obj
        self.hand = hand
        self.grasp = grasp
        self.path_prog = path_prog
        self.policy_module = policy_module

    def execute(
        self, belief_module: BeliefModule, virtual_robot: VirtualRobotModule
    ) -> None:
        phys = self.policy_module.planning_bel.phys
        return execute_path_prog(phys, self.path_prog, virtual_robot)

    def transition_prediction(
        self, belief_module: BeliefModule, action_rv: ActionRV
    ) -> TransitionPrediction:
        if action_rv.success:
            return TransitionPrediction(
                object_predictions=[
                    ObjectTransitionPrediction(
                        object_name=self.obj,
                        attribute_updates={
                            "in_hand": (self.hand, self.grasp),
                        },
                    )
                ]
            )
        else:
            curr_pose = belief_module.object_memory.memory.objects[self.obj].m4_pose
            tr_a(f"Executing {self.name} failed")
            if not isinstance(curr_pose, np.ndarray):
                curr_pose = curr_pose.value.T()
            return TransitionPrediction(
                object_predictions=[
                    ObjectTransitionPrediction(
                        object_name=self.obj,
                        attribute_updates={
                            "pose": (curr_pose, ATTEMPTED_PICK_DELTA),
                        },
                    )
                ]
            )


class SafePickHand(PickHand):
    def __init__(self, policy_module, obj: str, hand: str, grasp, path_prog):
        super().__init__(policy_module, obj, hand, grasp, path_prog)
        self.name = "safe_pick_hand"

    def execute(
        self, belief_module: BeliefModule, virtual_robot: VirtualRobotModule
    ) -> None:
        # Precond is that we are at a look conf, so we can execute a look and get an observation
        obs = self.policy_module._get_observation(include_images=True)
        # do belief update based on obs
        belief_module.update(NOP(), ActionRV(), obs)
        # Could try to check the belief to see if the object is localized, for efficiency
        # Create a new plannig_bel from the updated belief
        self.policy_module.planning_bel._phys = QR_bel_to_phys(
            belief_module,
            self.policy_module.static_domain_info,
            virtual_robot,
            self.policy_module.planning_bel.phys,
        )
        # Now, make a preimage, and solve to get bindings
        fluents = [
            # CanPath([FluentConj([Ignore([self.obj])]), "Path"], "CanPP"),
            CanPath([FluentConj([]), "Path"], "CanPP"),
            Pose6([self.obj, "OP6Pick", PICK_TOLERANCE], "PoseP"),
            RConf([], "QconfLook"),
        ]
        constraints = [
            IsSafePickKin(
                [
                    self.obj,
                    self.hand,
                    "Grasp",
                    "OP6Pick",
                    "Sensor",
                    "Qconf",
                    "QconfApp",
                    "QconfLook",
                ]
            ),
            IsSafePickPath(
                [
                    self.obj,
                    self.hand,
                    "Grasp",
                    "OP6Pick",
                    "Sensor",
                    "Qconf",
                    "QconfApp",
                    "QconfLook",
                    "Path",
                ]
            ),
        ]
        preimage = CGoal(fluents, constraints)
        bindings = preimage.satisfied(
            self.policy_module.planning_bel, return_bindings=True, lazy=False
        )[2]
        tr_a(f"New SafePickHand bindings in node {preimage.index}")
        if bindings is None:
            tr_a("SafePickHand failed to find bindings for pathprog after observation")
            return HPNActionRV(False, time.time())
        self.grasp = bindings["Grasp"]
        self.path_prog = bindings["Path"]
        # If we get one, execute it, otherwise return failure.
        phys = self.policy_module.planning_bel.phys
        return execute_path_prog(phys, bindings["Path"], virtual_robot)


class PlaceHand(HPNAction):
    def __init__(self, policy_module, obj: str, hand: str, pose, path_prog):
        self.name = "place_hand"
        self.obj = obj
        self.hand = hand
        self.pose = pose
        self.path_prog = path_prog
        self.policy_module = policy_module

    def execute(
        self, belief_module: BeliefModule, virtual_robot: VirtualRobotModule
    ) -> None:
        phys = self.policy_module.planning_bel.phys
        return execute_path_prog(phys, self.path_prog, virtual_robot)

    def transition_prediction(
        self, belief_module: BeliefModule, action_rv: ActionRV
    ) -> TransitionPrediction:
        if action_rv.success:
            return TransitionPrediction(
                object_predictions=[
                    ObjectTransitionPrediction(
                        object_name=self.obj,
                        attribute_updates={"pose": (self.pose.value.T(), PLACE_DELTA)},
                    )
                ]
            )
        else:
            tr_a(f"Executing {self.name} place failed")
            return TransitionPrediction(
                object_predictions=[
                    ObjectTransitionPrediction(
                        object_name=self.obj,
                        attribute_updates={
                            "pose": (self.pose.value.T(), ATTEMPTED_PLACE_DELTA)
                        },
                    )
                ]
            )


class LookRegion(HPNAction):
    def __init__(
        self, policy_module, sensor_name, region_name, view_conf, target_point
    ):
        self.name = "look_region"
        # DEBUG_LOOK
        self.sensor_name = sensor_name
        self.region_name = region_name
        self.view_conf = view_conf
        self.target_point = target_point
        self.policy_module = policy_module

    def execute(
        self, belief_module: BeliefModule, virtual_robot: VirtualRobotModule
    ) -> None:
        # def turn_head(conf, delta_pan, delta_tilt):
        #     eps = np.pi/20
        #     pan, tilt = conf['head']
        #     pan_joint, tilt_joint = phys.robot.joint_lists['head']
        #     new_pan = pan
        #     new_tilt = tilt
        #     min_tilt = 0.0
        #     max_tilt = 0.9
        #     if delta_pan != 0:
        #         new_pan = max(pan_joint.min_angle + eps, min(pan_joint.max_angle - eps, pan + delta_pan))
        #     if delta_tilt != 0.:
        #         new_tilt = max(min_tilt, min(max_tilt, tilt + delta_tilt))
        #     return conf.set_chain('head', np.array([new_pan, new_tilt]))
        # phys = self.policy_module.planning_bel.phys
        # if (phys.robot.name in ('Ruby', 'Rainbow')):
        #     center = self.view_conf
        #     left = turn_head(center, np.pi/4, 0)
        #     right = turn_head(center, -np.pi/4, 0)
        #     up = turn_head(center, 0, -np.pi/6)
        #     down = turn_head(center, 0, np.pi/6)
        #     path_to_left = phys.cc.cspace_path(center, left)
        #     path_to_right = phys.cc.cspace_path(left, right)
        #     path_to_up = phys.cc.cspace_path(right, up)
        #     path_to_down = phys.cc.cspace_path(right, down)
        #     path_to_center = phys.cc.cspace_path(up, center)
        #     execute_path_and_observe(self.policy_module, belief_module, virtual_robot, path_to_left)
        #     execute_path_and_observe(self.policy_module, belief_module, virtual_robot, path_to_right)
        #     execute_path_and_observe(self.policy_module, belief_module, virtual_robot, path_to_up)
        #     execute_path_and_observe(self.policy_module, belief_module, virtual_robot, path_to_down)
        #     execute_path(phys, virtual_robot, path_to_center)
        return HPNActionRV(True, time.time())

    def update_belief_before(self, belief_module, action_rv, observation):
        # TODO: make this nicer
        if self.region_name:
            belief_module.spatial_memory.vv.vv_regions.update_observed_region(
                self.region_name
            )
        phys = self.policy_module.planning_bel.phys
        if hasattr(self, "view_conf") and self.view_conf and self.target_point:
            # Keep track of executed view points so we don't try them again

            # DEBUG_LOOK
            phys.taboo_view_points.set(
                (self.sensor_name, self.view_conf, tuple(self.target_point)), True
            )
            # phys.taboo_view_points.set((self.view_conf, tuple(self.target_point)), True)

            # draw all the taboo view points in cyan
            # DEBUG_LOOK
            # Add for _sensor_name, ...
            for _sensor_name, _conf, view_point in phys.taboo_view_points.cache.keys():
                pt = np.array(view_point)
                box = np.vstack([pt - 0.02, pt + 0.02])
                viewer = phys.get_viewer()
                viewer.add_region(box, color=np.array((0, 250, 250)))
        phys.update_shadows(belief_module.spatial_memory.vv)


class LookObject(LookRegion):
    def __init__(self, policy_module):
        self.name = "look_object"
        self.region_name = None
        self.policy_module = policy_module


class HPNVirtualRobotPolicyModule(PolicyModule):
    def __init__(
        self,
        belief_module: BeliefModule,
        virtual_robot: VirtualRobotModule,
        static_domain_info,
        **kwargs,
    ):
        super().__init__(belief_module, {})
        self._static_domain_info = static_domain_info
        # Information from the problem definition
        default_robot_conf_chains = get_problem_properties(
            static_domain_info, "chain-conf"
        )
        self.workspace = get_problem_property(static_domain_info, "workspace")
        # From arguments
        self.partially_observed = kwargs.get("partially_observed")
        self.run_trajopt = kwargs.get("run_trajopt")
        self.virtual_robot = virtual_robot

        set_config_vars(kwargs.get("overrides"))
        set_debugging(
            kwargs.get("debug_tags", "full"),
            kwargs.get("debug_level", 1),
            kwargs.get("log_level", 7),
            draw_grasps=False,
            draw_grasp_safe=False,
        )
        if not kwargs.get("interactive", True):
            # The debug tags installed above pause (input()) on plan_fail —
            # fine interactively, fatal in headless/automated runs.
            from qaa.utils.traceFile import pause_set

            pause_set([])

        """
        if self.run_trajopt and virtual_robot.robot_name.lower() == 'spot':
            # TODO: make this work with voxel grid as obstacle
            # TODO: Extend to do RBY1
            from qr_robots.drake.spot.station import MakeSpotStation
            from Roboverse.physical_world.drake_state import DrakeState
            meshcat = kwargs.get('drake_meshcat')
            station = MakeSpotStation(meshcat)
            drake_state = DrakeState(station, meshcat)
            self._drake_state = drake_state
            # Exclude collisions that we know cannot happen.
            drake_state.ExcludeCollisionsWithin("spot", "arm")
            drake_state.ExcludeCollisionsBetween("spot", "arm", "spot", "leg")
            drake_state.ExcludeCollisionsBetween("spot", "arm_link_hr0", "spot", "body")
            self._trajopt = TrajectoryOptimizerHpnWrapper(drake_state)
        else:
            self._trajopt = None
            self._drake_state = None
        """

        self.belief_attrs_and_objects = extract_world_qddl(kwargs.get("qddl_paths"))
        robot_name = robot_name = kwargs.get("robot_name", None)
        phys = QR_bel_to_phys(
            None,
            self._static_domain_info,
            virtual_robot,
            robot_name=robot_name,
            known_attrs_and_objects=self.belief_attrs_and_objects,
        )

        phys.robot.set_home_robot_conf(default_robot_conf_chains)
        self.planning_bel = BeliefViewForHPN(belief_module, phys)

        real_robot = virtual_robot.real_robot if virtual_robot else False

        self._cog_man = CogMan(static_domain_info, real_robot=real_robot)

        if real_robot:
            if robot_name in ("rainbow",) and hasattr(virtual_robot, "client"):
                input("Okay to reset robot to nominal configuration?")
                virtual_robot.client.reset_torso_and_arms(phys.get_conf().value)
            if not get_problem_feature(static_domain_info, "use-base"):
                input("Please move table into place")
        elif virtual_robot is not None:   # sim robot; None when replaying from pkl
            if virtual_robot.client:
                print('Initial object poses:')
                for name, pose in virtual_robot.client.get_world_state().items():
                    if name != 'robot':
                        print(name, pose)
            vconf = Conf(virtual_robot.get_robot_conf())
            vconf_dict = vconf.value.copy()
            sk_conf = phys.get_conf()
            skconf_dict = sk_conf.value.copy()
            path = [Conf(vconf_dict.copy())]
            for chain in vconf_dict:
                vconf_dict[chain] = skconf_dict[chain]
                path.append(Conf(vconf_dict))
                vconf_dict = vconf_dict.copy()

            execute_path(
                phys,
                virtual_robot,
                path,
            )

    @property
    def phys(self):
        return self.planning_bel.phys

    @property
    def cog_man(self):
        return self._cog_man

    @property
    def trajopt(self):
        raise NotImplementedError(
            "HPNVirtualRobotPolicyModule does not support trajopt yet"
        )
        # return self._trajopt

    @property
    def drake_state(self):
        raise NotImplementedError(
            "HPNVirtualRobotPolicyModule does not support DrakeState yet"
        )
        # return self._drake_state

    @property
    def static_domain_info(self):
        return self._static_domain_info

    def reset(self, observation: Observation):
        self.update(None, None, observation)

    def update(
        self,
        action: ActionVR,
        action_rv: ActionRV,
        observation: Observation,
        draw: bool = True,
    ):
        # assume all object meshes have changed
        # TODO: mark which objects got updated since the last time we did this and
        # only recompute those

        # Compute a phys from the QR belief
        self.planning_bel._phys = QR_bel_to_phys(
            self.belief_module,
            self.static_domain_info,
            self.virtual_robot,
            self.phys,
            known_attrs_and_objects=self.belief_attrs_and_objects,
        )
        # Summarize shadows and junk as point clouds in phys
        self.phys.update_shadows(self.belief_module.spatial_memory.vv)
        # Clear the satisfiability cache in the planning bel
        self.planning_bel.reset_cache()

        """
        if self.drake_state:
            # Update DrakeState from phys.
            self.drake_state.UpdateFromPhys(self.phys_bel.phys)
            # Associate DrakeState to this phys.
            self.phys_bel.phys.trajopt = self.trajopt
        """

        if tamp_config.cur_MMP:
            tamp_config.cur_MMP.reset_phys(self.phys)

        if draw:
            rgbd = observation["images"][0] if observation["images"] else None
            self.phys.draw(rgbd=rgbd)

    def step(self) -> HPNAction:
        op = self.cog_man.actor(self.planning_bel)
        tr("log", "Policy action", op)
        return self.dispatch_action(op)

    def dispatch_action(self, op) -> HPNAction:
        if op == "fail":
            tr_a("HPN Policy failed to generate an action; quitting.")
            raise PlanFailedException
        elif op == "success":
            tr("terminal", "Yay! Goal achieved.")
            raise PlanFinishedException
        arg_dict = op.arg_dict()
        if op.name == "pick_hand":
            return PickHand(
                self, *[arg_dict[x] for x in ("ObjPick", "Hand", "Grasp", "Path")]
            )
        elif op.name == "safe_pick":
            exec = SafePickHand(
                self, *[arg_dict[x] for x in ("ObjPick", "Hand", "Grasp", "Path")]
            )
            exec.preimage_and_bindings = op.preimage_and_bindings
            return exec
        elif op.name in ("place_hand", "empty_hand"):
            return PlaceHand(
                self, *[arg_dict[x] for x in ("ObjPlace", "Hand", "OP6Place", "Path")]
            )
        elif op.name == "move":
            return Move(self, *[arg_dict[x] for x in ("Path",)])
        elif op.name == "move_with_sensing":
            return MoveWithSensing(self, *[arg_dict[x] for x in ("Path",)])
        elif op.name == "look_region":
            # DEBUG_LOOK
            # Add 'Sensor'
            return LookRegion(
                self, *[arg_dict[x] for x in ("Sensor", "Region", "Qconf", "Point")]
            )
        elif op.name == "look_object":
            return LookObject(self)
        else:
            raise HPN_RuntimeError(f"Unknown operator: {op.name}")

    def get_observation_step(self, action: HPNAction) -> Observation:
        return self._get_observation(
            include_images=action.name in {"look_region", "look_object"}
        )

    def _get_observation(self, include_images=True) -> Observation:
        if include_images:
            camera = list(self.virtual_robot.camera_modules.keys())[0]
            images = [self.virtual_robot.get_camera_image(camera)]
        else:
            images = []
        conf_dict = self.virtual_robot.get_robot_conf()
        if "base" not in conf_dict:
            conf_dict["base"] = self.virtual_robot.get_robot_conf()["base"]
        if False and self.virtual_robot.robot_name.lower() == "ruby":
            # Ruby images are relative to the base
            base_conf = conf_dict["base"]
            input(f"{base_conf=}")
            base_pose = translation_matrix(
                (base_conf[0], base_conf[1], 0.0)
            ) @ rotation_matrix(base_conf[2], (0, 0, 1))
            for image in images:
                image.camera_extrinsics = np.linalg.inv(
                    base_pose @ np.linalg.inv(image.camera_extrinsics)
                )
        return {"images": images, "conf": Conf(conf_dict)}

    def get_observation_reset(self) -> Observation:
        if self.partially_observed:
            conf = Conf(self.virtual_robot.get_robot_conf())
            return {"images": [], "conf": conf}
        else:
            return self.virtual_robot.get_world_state()

    def shutdown(self):
        """
        Clean up resources, if necessary.
        This method can be overridden by subclasses to implement specific cleanup logic.
        """
        # If there are any resources to clean up, do it here
        tr("log", "HPNVirtualRobotPolicyModule shutdown called")
        self.cog_man.shutdown()


def execute_path_prog(phys, path_prog, virtual_robot):
    planned_path_exec = path_prog.planned_path_exec
    if path_prog.op == "move":
        HPN_assert(planned_path_exec, "Path program must have planned_path_exec set")
    smooth = planned_path_exec and tamp_config.smooth_world_paths
    interpolate = planned_path_exec and tamp_config.interpolate_world_paths
    split_traj = path_prog.get_split_trajectory(
        phys,
        exclude_chains=phys.robot.exclude_chains,
        smooth=smooth,
        interpolate=interpolate,
        cartesian=(phys.robot.name == "Spot"),
    )

    if not split_traj:
        return HPNActionRV(True, time.time())

    # This is for error checking
    if path_prog.op == "move":
        moving_chains = set(component[0] for component in split_traj)
        start_conf = path_prog.planned_move_path[1][0][0]
        end_conf = path_prog.planned_move_path[1][-1][0]
        should_move_base = np.any(start_conf["base"] != end_conf["base"])
        if should_move_base and "base" not in moving_chains:
            print("Split trajectory does not include base, but it should!")
            pass
    # End of error checking

    #############  Optional rehearsal in belief ################
    if tamp_config.always_rehearse:
        tr(
            "rehearse", "Rehearsing trajectory in belief"
        )  # Pause here to see the trajectory
        phys.execute_split_trajectory(split_traj, max_steps=20)
        tr("rehearse", "Okay to execute on robot?")
    #############  Execution starts here ################

    if virtual_robot.client:
        print('Object poses:')
        for name, pose in virtual_robot.client.get_world_state().items():
            if name != 'robot':
                print(name, pose)

    (success, t) = execute_split_trajectory(
        phys, virtual_robot, split_traj, phys.robot.exclude_chains
    )

    return HPNActionRV(success, t)


def execute_split_trajectory(
    phys, virtual_robot, split_traj, exclude_chains=[], cartesian_map=None
):
    """
    split_traj is a list of (chain-name, traj) where traj is a sequence of configurations
    of that chain
    """
    tr("exec", "execute_trajectory exclude_chains", exclude_chains)
    for chain, soft, split, _ in split_traj:
        tr("exec", chain, soft)
        for point in split:
            tr("exec", "    ", point)

    if len(split_traj) == 0:
        tr("terminal", "Empty split_traj in execute_trajectory")
    success = True
    for chain, arg, split, times in split_traj:
        # print(f'{chain=} {split=}')
        tr("exec", f"{chain=} len(split)={len(split)}")
        success = True
        if chain in exclude_chains:
            tr("exec", "Excluding", chain)
            continue
        if chain == "base":
            virtual_robot.sync_follow_base_trajectory(
                [JointPositionCommand(duration=2.0, target_position=q) for q in split],
            )
            success = True
            tr("exec", f"base {success=}")
        elif chain == "head":
            virtual_robot.sync_follow_head_trajectory(
                [JointPositionCommand(duration=0.1, target_position=q) for q in split],
            )
            success = True
            tr("exec", f"head {success=}")
        elif chain == "torso":
            if virtual_robot.real_robot:
                input("Continue setting torso joints?")
                pass
            virtual_robot.sync_follow_torso_trajectory(
                [JointPositionCommand(duration=0.1, target_position=q) for q in split],
            )
            success = True
            tr("exec", f"head {success=}")
        elif chain in ("right", "left"):
            if len(split) == 2:
                pass
            virtual_robot.sync_follow_joint_trajectory_stiff(
                [
                    {chain: JointPositionCommand(duration=0.25, target_position=q)}
                    for q in split
                ],
            )
            success = True
            tr("exec", f"{chain} {success=}")
        elif chain in ("right_cart", "left_cart"):
            virtual_robot.sync_follow_cartesian_trajectory_stiff(
                [
                    {
                        chain: CartesianPoseCommand(
                            duration=0.1, target_pose=pose, target_joint_position=q
                        )
                    }
                    for (pose, q) in split
                ],
            )
        elif chain in ("right_gripper", "left_gripper"):
            initial = split[0][0]
            target = split[-1][0]
            if arg:
                assert arg in ("open", "close", "grab", "detach")
                command = "open" if arg in ("open", "detach") else "close"
                virtual_robot.sync_execute_gripper_command(chain, command, split[-1][0])
            elif initial != target:
                max_open = phys.robot.max_finger_opening
                command = "open" if target > 0.75 * max_open else "close"
                virtual_robot.sync_execute_gripper_command(chain, command, target)
            success = True
            tr("exec", f"{chain} {success=}")
        else:
            tr("exec", "ignoring", chain)

    return (success, time.time())


class CogMan:
    def __init__(self, static_info, real_robot: bool = True):
        # Set up debugging files
        file_tag = "HPN"
        self.hpn_file = write_preamble(file_tag)

        # Pick heuristic;
        if plan_config.use_heuristic == "H_unmet":
            h = H_unmet
        elif plan_config.use_heuristic == "hao":
            h = hao(h_split)
        else:
            h = dummy_helpful_H

        # Optional reflex actions
        # max_sequential_action_failures = 4
        reflexes = [  # (lambda b: b.sequential_action_failures > max_sequential_action_failures,
            #  lambda b: 'fail'),
            (lambda b: b.phys.currently_in_collision(complain=True), lambda b: "fail")
        ]

        self.actor = make_HPN_action_selector(
            make_retroplan_bel_goal(parse_goal_expression(get_goal(static_info))),
            lambda s, g, ops_ps: plan(
                s, g, ops_ps[0], ops_ps[1], False, "HPN", useH=h, rescore=False
            ),  # need for fancy belief plan optimization
            get_btamp_ops(),
            lambda _s, _sg, ops, ps: (ops, ps),
            lambda s, ps: HPN_goal_gen(
                s, ps, sticky=cogman_config.use_sticky_next_step
            ),
            self.hpn_file,
            reflexes,
        )

    def shutdown(self):
        """
        Clean up resources, if necessary.
        This method can be overridden by subclasses to implement specific cleanup logic.
        """
        tr("log", "CogMan shutdown called")
        write_coda(self.hpn_file)
        if tamp_config.cur_MMP:
            tamp_config.cur_MMP.cleanup()
