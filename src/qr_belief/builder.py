"""
Standard BeliefModule construction, mirroring the old repo's
qr_run.run.construct_belief_module for the simulated-perception case.

The grid bounds come from explicit workspace/shadow arrays instead of the
QDDL static_domain_info (problem definitions are not ported yet); pass
static_domain_info through to use the QDDL path.
"""

from __future__ import annotations

from qr_api.belief_interfaces import BeliefModule
from qr_belief.dummy_belief_interfaces import DummyAgentMemoryModule
from qr_belief.goal.simple_goal_belief_module import SimpleGoalMemoryModule
from qr_belief.robot.simple_robot_belief_module import SimpleRobotMemoryModule
from qr_belief.spatial.simple_spatial_module import OldHPNSpatialMemoryModule
from qr_perception.pipelines import make_sim_obm, make_sim_perception_pipeline


def make_sim_belief_module(
    goal: str = "",
    sensor_modules: dict | None = None,
    static_domain_info=None,
    workspace=((-2.0, -2.0, 0.0), (2.0, 2.0, 2.0)),
    shadow_extents=(4.0, 4.0, 1.2),
    shadow_pose=(-2.0, -2.0, 0.0, 0.0, 0.0, 0.0),
    voxel_grid_resolution: float = 0.02,
    drake_meshcat=None,
    drake_meshcat_aux=None,
    display: bool = False,
) -> BeliefModule:
    goal_memory_module = SimpleGoalMemoryModule(goal)
    perception = make_sim_perception_pipeline(display=display)
    obm = make_sim_obm(goal=goal, perception_pipeline=perception, display=display)
    # make_sim_obm builds its own goal module; share ours instead.
    obm.goal_belief = goal_memory_module

    spatial = OldHPNSpatialMemoryModule(
        static_domain_info=static_domain_info,
        drake_meshcat=drake_meshcat,
        drake_meshcat_aux=drake_meshcat_aux,
        voxel_grid_resolution=voxel_grid_resolution,
        workspace=workspace,
        shadow_extents=shadow_extents,
        shadow_pose=shadow_pose,
    )

    return BeliefModule(
        sensor_modules if sensor_modules is not None else {},
        spatial,
        obm,
        DummyAgentMemoryModule(None),
        goal_memory_module,
        SimpleRobotMemoryModule(),
    )
