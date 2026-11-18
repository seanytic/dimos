# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Attach an operator-side SpaceMouse to an already running A1Z."""

import asyncio
from contextlib import closing

from dimos.porcelain.remote_module_source import RemoteModuleSource
from dimos.robot.manipulators.common.topics import EEF_TWIST_TASK_NAME
from dimos.teleop.spacemouse.spacemouse_teleop_module import open_keyboard, open_mouse, poll_mouse


def run() -> None:
    with closing(RemoteModuleSource()) as client:
        coordinator = client.get_module("A1ZCoordinator")
        if (
            not hasattr(coordinator, "list_tasks")
            or EEF_TWIST_TASK_NAME not in coordinator.list_tasks()
        ):
            raise RuntimeError("Start the galaxea-a1z-localize blueprint for SpaceMouse control.")
        twists = coordinator.get_stream("ee_twist_command").transport
        gripper = coordinator.get_stream("gripper_command").transport
        with open_keyboard() as read_toggle, open_mouse() as mouse:
            print(
                f"{mouse.name} connected. Twist right to pitch up, left to pitch down. Ctrl-C to exit."
            )
            asyncio.run(poll_mouse(mouse, twists.publish, gripper.publish, read_toggle))
