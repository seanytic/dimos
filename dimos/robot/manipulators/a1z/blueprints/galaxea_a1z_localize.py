from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.robot.manipulators.a1z.blueprints.basic import (
    A1Z_REMAPS,
    a1z_coordinator,
)
from dimos.robot.manipulators.a1z.config import make_a1z_model_config
from dimos.robot.manipulators.common.blueprints import eef_twist_task, planner
from dimos.teleop.spacemouse.spacemouse_teleop_module import SpaceMouseTeleopModule

# Planner home for arm_joint1 through arm_joint6, in radians.
# The gripper points 45 degrees down at this working pose.
_HOME_JOINTS_RAD: list[float] = [0.0, 1.23, -1.13, 0.6854, 0.0, 0.0]
_model = make_a1z_model_config(robot="arm", home_joints=_HOME_JOINTS_RAD)

galaxea_a1z_localize = autoconnect(
    a1z_coordinator(
        eef_twist_task(
            robot_model=_model,
            target_frame="gripper_eef_link",
            base_rotation_joint="arm/arm_joint1",
            pitch_rotation_joint="arm/arm_joint4",
            priority=15,
            max_joint_velocity_rad_s=0.5,
            max_command_tracking_error_deg=5.0,
        )
    ),
    planner(
        model=_model,
        gripper_tasks={"manipulator": "arm_gripper"},
        default_speed_scale=0.15,
    ),
    ManipulationSkills.blueprint(),
    McpServer.blueprint(),
).remappings(A1Z_REMAPS)

galaxea_a1z_spacemouse = autoconnect(galaxea_a1z_localize, SpaceMouseTeleopModule.blueprint())
