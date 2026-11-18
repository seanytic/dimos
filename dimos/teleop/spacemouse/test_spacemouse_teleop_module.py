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

import asyncio
from contextlib import ExitStack, contextmanager, nullcontext
import math
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from dimos.protocol.rpc.spec import RPCSpec
from dimos.teleop.spacemouse import spacemouse_teleop_module as spacemouse


def report(x=0.0, y=0.0, z=0.0, *, yaw=0.0, buttons=(0, 0), age=0.0):
    return SimpleNamespace(x=x, y=y, z=z, buttons=buttons, age=age, pitch=1.0, yaw=yaw, roll=1.0)


@pytest.fixture
def replay(mocker):
    def run(*frames, names=("LEFT", "RIGHT"), keys=None):
        # Existing motion tests explicitly enable on a neutral sample first.
        if keys is None:
            frames = ([report()], *frames)
            keys = [True]
        toggles = iter(keys)
        clock = mocker.patch.object(spacemouse, "time")
        now = 10.0
        clock.perf_counter.side_effect = lambda: now
        current = SimpleNamespace()
        pending = iter(frames)
        queue = []

        def advance(seconds):
            nonlocal now, queue
            now += seconds
            queue = list(next(pending, []))
            if not queue:
                raise OSError("mouse unplugged")

        def read():
            if queue:
                sample = queue.pop(0)
                # Like PySpaceMouse, return the same mutable object on every read.
                vars(current).update(vars(sample))
                current.t = now - sample.age - 0.001 * len(queue)
            return current

        mouse = mocker.Mock()
        mouse.info.button_names = names
        mouse.read.side_effect = read

        def read_toggle():
            key = next(toggles, False)
            if isinstance(key, Exception):
                raise key
            return key

        mocker.patch.object(spacemouse.asyncio, "sleep", side_effect=advance)
        twists, gripper = mocker.Mock(), mocker.Mock()
        advance(0)
        with pytest.raises(OSError):
            asyncio.run(spacemouse.poll_mouse(mouse, twists.publish, gripper.publish, read_toggle))
        return (
            [call.args[0] for call in twists.publish.call_args_list],
            [call.args[0].data for call in gripper.publish.call_args_list],
        )

    return run


def xyz(v):
    return (v.x, v.y, v.z)


def test_starts_disabled_and_ignores_motion_and_gripper_buttons(replay):
    twists, openings = replay(
        [report(x=1, buttons=(1, 0))], [report(yaw=1, buttons=(0, 1))], keys=[]
    )
    assert twists == openings == []


def test_s_disables_moving_arm_and_blocks_subsequent_buttons(replay):
    twists, openings = replay(
        [report()],
        [report(z=1)],
        [report(z=1, buttons=(0, 1))],
        [report(x=1, buttons=(1, 0))],
        keys=[True, False, True],
    )
    assert [xyz(msg.linear) for msg in twists] == [(0, 0, 0.05), (0, 0, 0)]
    assert openings == []


def test_losing_keyboard_stops_a_moving_arm(replay):
    twists, _ = replay(
        [report()],
        [report(z=1)],
        [report(z=1)],
        keys=[True, False, OSError("keyboard disconnected")],
    )
    assert [xyz(msg.linear) for msg in twists] == [(0, 0, 0.05), (0, 0, 0)]


def test_enabling_accepts_current_motion_and_gripper_input(replay):
    twists, openings = replay([report(z=1, buttons=(0, 1))], keys=[True])
    assert [xyz(msg.linear) for msg in twists] == [(0, 0, 0.05), (0, 0, 0)]
    assert openings == [0.0]


def test_idle_neutral_mouse_can_enable_but_motion_must_be_fresh(replay):
    twists, _ = replay([report(age=10)], [report(z=1, age=1)], [report(z=1)], keys=[True])
    assert [xyz(msg.linear) for msg in twists] == [(0, 0, 0.05), (0, 0, 0)]


def test_s_can_disable_and_reenable_without_delay(replay):
    twists, _ = replay(
        [report()],
        [report(z=1)],
        [report()],
        [report()],
        [report(z=-1)],
        keys=[True, False, True, True, False],
    )
    assert [xyz(msg.linear) for msg in twists] == [
        (0, 0, 0.05),
        (0, 0, 0),
        (0, 0, -0.05),
        (0, 0, 0),
    ]


def test_disabled_button_press_is_not_replayed_after_enabling(replay):
    twists, openings = replay(
        [report(buttons=(0, 1))],
        [report()],
        [report()],
        [report(buttons=(0, 1))],
        keys=[False, True],
    )
    assert twists == []
    assert openings == [0.0]


def test_cap_axes_select_forward_base_rotation_and_lift_then_stop(replay):
    twists, openings = replay(
        [report(x=0.5, y=0.05)], [report(y=-1.0)], [report(z=1.0)], [report()], [report()]
    )
    forward, right, up, hold = [msg.linear for msg in twists]
    assert xyz(forward) == (0.05, 0, 0)
    assert xyz(right) == (0, 0, 0)
    assert xyz(up) == (0, 0, 0.05)
    assert xyz(hold) == (0, 0, 0)
    assert all(msg.frame_id == "base_link" for msg in twists)
    assert [xyz(msg.angular) for msg in twists] == [
        (0, 0, 0),
        (0, 0, -math.radians(10)),
        (0, 0, 0),
        (0, 0, 0),
    ]
    assert openings == []


@pytest.mark.parametrize("direction", [-1, 1])
def test_sideways_rotates_base_and_stops_without_gripper_pose(replay, direction):
    twists, _ = replay([report(y=direction * 0.21)], [report(y=direction)], [report()])

    first, held, stop = twists
    assert all(xyz(msg.linear) == (0, 0, 0) for msg in twists)
    assert xyz(first.angular) == xyz(held.angular) == (0, 0, direction * math.radians(10))
    assert xyz(stop.angular) == (0, 0, 0)


def test_diagonal_input_selects_one_axis_and_unplugging_stops_motion(replay):
    twists, _ = replay([report(x=0.7, y=-1.0, z=0.8)], [report(x=0.7, y=-1.0, z=0.8)])
    moving, stop = twists[-2:]
    assert xyz(moving.linear) == xyz(stop.linear) == xyz(stop.angular) == (0, 0, 0)
    assert xyz(moving.angular) == (0, 0, -math.radians(10))


def test_stale_report_stops_motion_and_cannot_close_gripper(replay):
    twists, openings = replay([report(z=-1)], [report(z=-1, buttons=(0, 1), age=1)])
    moving, stop = twists
    assert xyz(moving.linear) == (0, 0, -0.05)
    assert xyz(stop.linear) == xyz(stop.angular) == (0, 0, 0)
    assert openings == []


def test_right_twist_pitches_up_and_left_twist_pitches_down(replay):
    twists, _ = replay([report(yaw=-0.3)], [report(yaw=-1)], [report(yaw=0.4)], [report()])
    first, held, reversed_twist, stop = twists
    # The A1Z task routes this pitch directly to joint 4; no gripper pose is needed.
    assert first.angular.y == held.angular.y == pytest.approx(-math.radians(15))
    assert reversed_twist.angular.y == pytest.approx(math.radians(15))
    assert all(
        msg.angular.x == msg.angular.z == 0 and xyz(msg.linear) == (0, 0, 0) for msg in twists
    )
    assert xyz(stop.angular) == (0, 0, 0)


def test_neutral_band_produces_no_motion(replay):
    twists, _ = replay(
        [report(x=0.12, yaw=-0.12)],
        [report(x=0.2, y=-0.2, z=0.2, yaw=-0.2)],
        [report()],
    )
    assert twists == []


def test_deflection_changes_keep_speed_constant_and_reversal_is_immediate(replay):
    twists, _ = replay(
        [report(x=0.201)], [report(x=1.0)], [report(x=0.4)], [report(x=-0.4)], [report(x=0.19)]
    )
    assert [xyz(msg.linear) for msg in twists] == [
        (0.05, 0, 0),
        (0.05, 0, 0),
        (0.05, 0, 0),
        (-0.05, 0, 0),
        (0, 0, 0),
    ]


def test_axis_switch_requires_a_clear_advantage_including_twist(replay):
    twists, _ = replay(
        [report(x=0.6, y=0.5)],
        [report(x=0.6, y=0.65)],
        [report(x=0.6, y=0.7)],
        [report(x=0.6, y=0.71)],
        [report(y=0.6, yaw=-0.65)],
        [report(y=0.6, yaw=-0.71)],
        [report(y=0.65, yaw=-0.6)],
        [report(y=0.8, yaw=-0.6)],
        [report()],
    )
    assert [xyz(msg.linear) for msg in twists] == [
        (0.05, 0, 0),
        (0.05, 0, 0),
        (0.05, 0, 0),
        (0, 0, 0),
        (0, 0, 0),
        (0, 0, 0),
        (0, 0, 0),
        (0, 0, 0),
        (0, 0, 0),
    ]
    assert [msg.angular.y for msg in twists] == pytest.approx(
        [0, 0, 0, 0, 0, -math.radians(15), -math.radians(15), 0, 0]
    )
    assert [msg.angular.z for msg in twists] == pytest.approx(
        [0, 0, 0, math.radians(10), math.radians(10), 0, 0, math.radians(10), 0]
    )
    assert all(msg.angular.x == 0 for msg in twists)


def test_centered_axis_can_switch_without_the_extra_advantage(replay):
    twists, _ = replay([report(z=0.5)], [report(z=0.2, y=-0.21)], [report()])

    assert [xyz(msg.linear) for msg in twists] == [(0, 0, 0.05), (0, 0, 0), (0, 0, 0)]
    assert [xyz(msg.angular) for msg in twists] == [
        (0, 0, 0),
        (0, 0, -math.radians(10)),
        (0, 0, 0),
    ]


@pytest.mark.parametrize("value", [math.nan, math.inf])
def test_nonfinite_motion_report_stops_the_arm(replay, value):
    twists, _ = replay([report(z=0.5)], [report(x=value, z=0.5)])

    assert [xyz(msg.linear) for msg in twists] == [(0, 0, 0.05), (0, 0, 0)]
    assert all(xyz(msg.angular) == (0, 0, 0) for msg in twists)


def test_stale_twist_report_stops_joint_rotation(replay):
    twists, _ = replay([report(yaw=-1)], [report(yaw=-1, age=1)])
    moving, stop = twists
    assert xyz(moving.angular) == (0, -math.radians(15), 0)
    assert xyz(moving.linear) == xyz(stop.angular) == xyz(stop.linear) == (0, 0, 0)


def test_quick_clicks_and_held_buttons_only_command_once_per_press(replay):
    twists, openings = replay(
        [report(x=1, buttons=(1, 0)), report()],
        [report(buttons=(0, 1))],
        [report(buttons=(0, 1))],
        [report(), report(buttons=(1, 0)), report()],
    )
    assert openings == [1.0, 0.0, 1.0]
    assert twists == []


def test_unmapped_or_simultaneous_buttons_leave_gripper_alone(replay):
    _, openings = replay(
        [report(buttons=(1, 0, 0))],
        [report(buttons=(0, 1, 1))],
        [report(buttons=(0, 0, 1))],
        [report(buttons=(0, 0, 0))],
        [report(buttons=(0, 1, 0))],
        names=("MENU", "RIGHT", "LEFT"),
    )
    assert openings == [0.0]


class _LocalRPC(MagicMock, RPCSpec):
    """Keep the module lifecycle test off the network."""


def test_module_stop_publishes_zero_before_closing_the_mouse(mocker):
    events = []
    moved = threading.Event()

    def receive(msg):
        events.append(xyz(msg.linear))
        moved.set()

    @contextmanager
    def open_mouse():
        samples = 0

        def read():
            nonlocal samples
            samples += 1
            sample = report(z=0 if samples <= 2 else 1)
            # Two reads per poll have the same timestamp, as when the HID queue is empty.
            sample.t = stamp + (samples - 1) // 2 * 0.001
            return sample

        stamp = time.perf_counter()
        mouse = mocker.Mock()
        mouse.info.button_names = ("LEFT", "RIGHT")
        mouse.read.side_effect = read
        try:
            yield mouse
        finally:
            events.append("closed")

    mocker.patch.object(spacemouse, "open_mouse", open_mouse)
    taps = iter([True])
    mocker.patch.object(
        spacemouse, "open_keyboard", side_effect=lambda: nullcontext(lambda: next(taps, False))
    )
    with ExitStack() as cleanup:
        module = spacemouse.SpaceMouseTeleopModule(rpc_transport=_LocalRPC)
        cleanup.callback(module.stop)
        module.ee_twist_command.subscribe(receive)
        module.start()
        assert moved.wait(timeout=2)
    assert events[0] == (0, 0, 0.05)
    assert events[-2:] == [(0, 0, 0), "closed"]


def test_module_start_reports_an_unavailable_mouse(mocker):
    mocker.patch.object(spacemouse, "open_keyboard", return_value=nullcontext(lambda: False))
    mocker.patch.object(spacemouse, "open_mouse", side_effect=OSError("no mouse"))
    with ExitStack() as cleanup:
        module = spacemouse.SpaceMouseTeleopModule(rpc_transport=_LocalRPC)
        cleanup.callback(module.stop)
        with pytest.raises(OSError, match="no mouse"):
            module.start()
