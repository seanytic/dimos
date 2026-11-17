from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.robot.manipulators.a1z.blueprints.basic import (
    A1Z_REMAPS,
    a1z_coordinator,
)
from dimos.robot.manipulators.a1z.config import make_a1z_model_config
from dimos.robot.manipulators.common.blueprints import planner

# Existing DimOS default only.
# Replace with your measured custom home.
# Radians, ordered arm_joint1 through arm_joint6.

_HOME_JOINTS_RAD: list[float] = [
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
]

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
