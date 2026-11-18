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

"""SpaceMouse input for arm translation, base rotation, and wrist nodding."""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator, Callable, Iterator
from contextlib import contextmanager, suppress
import ctypes.util
import importlib
from itertools import compress
import math
import select
import sys
import termios
import time
import tty
from typing import Any

from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_SPEED = 0.05  # metres per second while the cap is deflected
_BASE_RATE = math.radians(10)
_WRIST_RATE = math.radians(15)
_OPENING = {"LEFT": 1.0, "RIGHT": 0.0}


def _fresh(stamp: float) -> bool:
    return 0 <= time.perf_counter() - stamp <= 0.2


@contextmanager
def open_keyboard() -> Iterator[Callable[[], bool]]:
    """Read s from the controlling terminal, including inside a forkserver worker."""
    try:
        terminal = open("/dev/tty", "rb", buffering=0)
    except OSError as exc:
        raise RuntimeError(
            "SpaceMouse needs a foreground terminal for the s key. "
            "Run galaxea-a1z-spacemouse without --daemon, or attach with "
            "dimos hardware a1z spacemouse in a terminal."
        ) from exc
    with terminal:
        settings = termios.tcgetattr(terminal)

        def read_toggle() -> bool:
            if not select.select([terminal], [], [], 0)[0]:
                return False
            keys = terminal.read(256)
            if not keys:
                raise OSError("SpaceMouse control terminal disconnected")
            return b"s" in keys.lower()

        try:
            tty.setcbreak(terminal.fileno())  # No Enter needed; Ctrl-C still interrupts.
            yield read_toggle
        finally:
            termios.tcsetattr(terminal, termios.TCSADRAIN, settings)


async def poll_mouse(
    mouse: Any,
    publish_twist: Callable[[TwistStamped], None],
    publish_gripper: Callable[[Float32], None],
    read_toggle: Callable[[], bool],
) -> None:
    """Publish one steady motion at a time; the arm controller smooths the joints."""
    previous_buttons: set[str] = set()
    active_axis = 0
    moving = False
    enabled = False
    logger.info("SpaceMouse DISABLED. Tap s in this terminal to toggle control.")
    try:
        while True:
            if read_toggle():
                enabled = not enabled
                logger.info("SpaceMouse ENABLED" if enabled else "SpaceMouse DISABLED")
                if not enabled and moving:
                    publish_twist(TwistStamped(frame_id="base_link"))
                    moving = False
            stamp = -1.0
            state = mouse.read()
            # read() repeats its timestamp when the HID queue is empty. Catch short
            # button clicks in the queue, then use the latest cap position for motion.
            while state.t != stamp:
                stamp = state.t
                pressed = set(compress(mouse.info.button_names, state.buttons))
                if (
                    enabled
                    and len(pressed) == 1
                    and pressed.isdisjoint(previous_buttons)
                    and _fresh(stamp)
                ):
                    opening = _OPENING.get(next(iter(pressed)))
                    if opening is not None:
                        publish_gripper(Float32(data=opening))
                previous_buttons = pressed
                state = mouse.read()

            axes = (state.x, state.y, state.z, state.yaw)
            motion = [0.0] * 4
            if enabled and _fresh(state.t) and all(map(math.isfinite, axes)):
                strongest = max(range(4), key=lambda axis: abs(axes[axis]))
                # Keep the current axis until another is clearly stronger.
                if (
                    abs(axes[active_axis]) <= 0.2
                    or abs(axes[strongest]) > abs(axes[active_axis]) + 0.1
                ):
                    active_axis = strongest
                if abs(axes[active_axis]) > 0.2:
                    motion[active_axis] = math.copysign(1.0, axes[active_axis])
            linear = [motion[0] * _SPEED, 0.0, motion[2] * _SPEED]
            # A1Z maps pitch to joint 4 and yaw to joint 1. Clockwise cap twist
            # has negative ROS yaw, so it decreases joint 4 to nod the gripper up.
            angular = (0.0, motion[3] * _WRIST_RATE, motion[1] * _BASE_RATE)
            if any(linear) or any(angular):
                moving = True
                publish_twist(TwistStamped(frame_id="base_link", linear=linear, angular=angular))
            elif moving:
                publish_twist(TwistStamped(frame_id="base_link"))
                moving = False
            await asyncio.sleep(0.02)
    finally:
        if moving:
            publish_twist(TwistStamped(frame_id="base_link"))


@contextmanager
def open_mouse() -> Iterator[Any]:
    """Open the operator's SpaceMouse with forward, left and up as positive axes."""
    try:
        driver = importlib.import_module("pyspacemouse")
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Install the SpaceMouse extra: uv sync --extra spacemouse --inexact"
        ) from exc
    if sys.platform == "darwin":
        # easyhid 0.0.10 looks for Linux symbols on modern macOS. Expose Homebrew's hidapi.
        library = ctypes.util.find_library("hidapi") or "/opt/homebrew/lib/libhidapi.dylib"
        ctypes.CDLL(library, mode=ctypes.RTLD_GLOBAL)
    with driver.open(axis_convention=driver.AxisConvention.ROS) as mouse:
        yield mouse


class SpaceMouseTeleopModule(Module):
    """Publish arm motion and gripper opening from a USB SpaceMouse."""

    ee_twist_command: Out[TwistStamped]
    gripper_command: Out[Float32]

    async def main(self) -> AsyncIterator[None]:
        with open_keyboard() as read_toggle, open_mouse() as mouse:
            reader = asyncio.create_task(
                poll_mouse(
                    mouse, self.ee_twist_command.publish, self.gripper_command.publish, read_toggle
                )
            )
            try:
                yield
            finally:
                reader.cancel()
                with suppress(asyncio.CancelledError):
                    await reader
