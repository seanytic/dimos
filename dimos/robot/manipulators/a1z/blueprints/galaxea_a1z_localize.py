from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.robot.manipulators.a1z.blueprints.basic import (
    A1Z_REMAPS,
    a1z_coordinator,
)
from dimos.robot.manipulators.a1z.config import make_a1z_model_config
from dimos.robot.manipulators.common.blueprints import planner

# Planner home for arm_joint1 through arm_joint6, in radians.
# The A1Z jaw center is level, about 0.35 m forward and 0.40 m up from base_link.
_HOME_JOINTS_RAD: list[float] = [0.0, 1.23, -1.13, -0.10, 0.0, 0.0]

galaxea_a1z_localize = autoconnect(
    a1z_coordinator(),
    planner(
        model=make_a1z_model_config(
            robot="arm",
            home_joints=_HOME_JOINTS_RAD,
        ),
        gripper_tasks={"manipulator": "arm_gripper"},
        default_speed_scale=0.15,
    ),
    ManipulationSkills.blueprint(),
    McpServer.blueprint(),
).remappings(A1Z_REMAPS)
