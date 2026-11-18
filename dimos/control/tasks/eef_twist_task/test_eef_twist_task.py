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

from pathlib import Path

import numpy as np
import pytest
from pytest_mock import MockerFixture
from scipy.spatial.transform import Rotation

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.eef_twist_task.eef_twist_task import (
    EEFTwistTask,
    EEFTwistTaskConfig,
    create_task,
)
from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.common.blueprints import eef_twist_task


def _robot_model() -> RobotModelConfig:
    return RobotModelConfig(
        model=RobotModel.from_file(Path("fake.urdf")),
        joint_names=["arm/joint1", "arm/joint2"],
    )


def _solver(mocker: MockerFixture) -> PinkPoseTargetSolver:
    solver = mocker.Mock(spec=PinkPoseTargetSolver)
    solver.frame_poses.return_value = {
        "tool": PoseStamped(frame_id="world", position=[0.0, 0.0, 0.0])
    }
    solver.step.return_value = JointState(
        name=["arm/joint1", "arm/joint2"],
        position=[0.01, 0.02],
    )
    return solver


def _config(
    *,
    target_frames: tuple[str, ...] = ("tool",),
    command_timeout: float = 0.3,
    base_rotation_joint: str | None = None,
    pitch_rotation_joint: str | None = None,
) -> EEFTwistTaskConfig:
    return EEFTwistTaskConfig(
        joint_names=("arm/joint1", "arm/joint2"),
        robot_model=_robot_model(),
        target_frames=target_frames,
        timeout=0.0,
        command_timeout=command_timeout,
        base_rotation_joint=base_rotation_joint,
        pitch_rotation_joint=pitch_rotation_joint,
    )


def _task(
    mocker: MockerFixture,
    *,
    command_timeout: float = 0.3,
    base_rotation_joint: str | None = None,
    pitch_rotation_joint: str | None = None,
) -> tuple[EEFTwistTask, PinkPoseTargetSolver]:
    solver = _solver(mocker)
    task = EEFTwistTask(
        "eef",
        _config(
            command_timeout=command_timeout,
            base_rotation_joint=base_rotation_joint,
            pitch_rotation_joint=pitch_rotation_joint,
        ),
        solver=solver,
    )
    return task, solver


@pytest.mark.parametrize("target_frames", [(), ("tool", "other")])
def test_eef_twist_config_requires_exactly_one_target_frame(
    target_frames: tuple[str, ...],
) -> None:
    with pytest.raises(ValueError):
        _config(target_frames=target_frames)


def _state(t_now: float = 1.0, *, dt: float = 0.01) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions={"arm/joint1": 0.0, "arm/joint2": 0.0},
        ),
        t_now=t_now,
        dt=dt,
    )


def _twist(x: float = 0.1) -> TwistStamped:
    return TwistStamped(frame_id="tool", linear=[x, 0.0, 0.0], angular=[0.0, 0.0, 0.0])


def test_twist_integrates_target_and_uses_shared_persistent_command(
    mocker: MockerFixture,
) -> None:
    task, solver = _task(mocker)
    assert task.on_ee_twist_command(_twist(), t_now=1.0)

    first = task.compute(_state(1.01))
    second = task.compute(_state(1.02))

    assert first is not None
    assert second is not None
    first_target = solver.step.call_args_list[0].args[0]["tool"]
    second_target = solver.step.call_args_list[1].args[0]["tool"]
    assert first_target.position.x == pytest.approx(0.001)
    assert second_target.position.x == pytest.approx(0.002)


def test_zero_twist_reanchors_target_at_measured_pose(mocker: MockerFixture) -> None:
    task, solver = _task(mocker)
    task.on_ee_twist_command(_twist(), t_now=1.0)
    task.compute(_state(1.01))
    resets = solver.reset.call_count
    task.on_ee_twist_command(_twist(0.0), t_now=1.02)

    task.compute(_state(1.03))

    target = solver.step.call_args.args[0]["tool"]
    assert target.position.x == pytest.approx(0.0)
    assert solver.reset.call_count == resets + 1


def test_twist_back_soon_after_an_unreachable_push_moves_at_once(mocker: MockerFixture) -> None:
    task, solver = _task(mocker)  # the measured pose stays at x=0
    for tick in range(100):
        task.on_ee_twist_command(_twist(1.0), t_now=1.0 + tick * 0.01)
        task.compute(_state(1.0 + tick * 0.01))
    task.on_ee_twist_command(_twist(0.0), t_now=2.0)
    task.on_ee_twist_command(_twist(-1.0), t_now=2.1)  # within command_timeout

    task.compute(_state(2.11))

    target = solver.step.call_args.args[0]["tool"]
    assert target.position.x == pytest.approx(-0.01)


def test_stale_twist_holds_one_tick_then_goes_idle(mocker: MockerFixture) -> None:
    task, solver = _task(mocker)
    task.on_ee_twist_command(_twist(), t_now=1.0)
    task.compute(_state(1.01))

    output = task.compute(_state(1.31))

    assert output is not None
    target = solver.step.call_args.args[0]["tool"]
    assert target.position.x == pytest.approx(0.001)
    assert not task.is_active()


def test_task_is_active_only_while_twists_arrive(mocker: MockerFixture) -> None:
    task, solver = _task(mocker)
    assert not task.is_active()

    task.on_ee_twist_command(_twist(), t_now=1.0)
    assert task.is_active()
    assert task.compute(_state(1.01)) is not None
    assert task.compute(_state(1.2)) is not None
    assert task.is_active()

    task.compute(_state(1.31))
    assert not task.is_active()

    resets = solver.reset.call_count
    task.on_ee_twist_command(_twist(), t_now=2.0)
    assert task.is_active()
    assert solver.reset.call_count == resets + 1
    assert task.compute(_state(2.01)) is not None
    # Re-anchored at the measured pose, not the target held before idling.
    assert solver.frame_poses.call_count == 2
    target = solver.step.call_args.args[0]["tool"]
    assert target.position.x == pytest.approx(0.001)


def test_zero_twist_does_not_keep_task_active(mocker: MockerFixture) -> None:
    task, _ = _task(mocker)
    task.on_ee_twist_command(_twist(), t_now=1.0)
    task.compute(_state(1.01))
    task.on_ee_twist_command(_twist(0.0), t_now=1.25)

    task.compute(_state(1.31))
    assert not task.is_active()

    task.on_ee_twist_command(_twist(0.0), t_now=1.4)
    assert not task.is_active()


def test_zero_timeout_never_goes_idle(mocker: MockerFixture) -> None:
    task, solver = _task(mocker, command_timeout=0.0)
    assert task.is_active()
    assert task.compute(_state(1.0)) is not None

    task.on_ee_twist_command(_twist(), t_now=1.0)
    task.on_ee_twist_command(_twist(0.0), t_now=1.01)
    assert task.compute(_state(1.02)) is not None
    assert task.compute(_state(10.0)) is not None
    assert task.is_active()
    assert solver.frame_poses.call_count == 2  # re-anchored once, when the twist stopped


def test_estop_rejects_input_and_reanchors_after_clear(mocker: MockerFixture) -> None:
    task, solver = _task(mocker)
    task.on_ee_twist_command(_twist(), t_now=1.0)
    task.compute(_state(1.01))

    task.set_estop(True)
    assert not task.on_ee_twist_command(_twist(), t_now=1.02)
    assert task.compute(_state(1.02)) is None

    task.set_estop(False)
    assert task.compute(_state(1.03)) is not None
    assert solver.frame_poses.call_count == 2


def test_preemption_discards_twist_target_and_command_state(mocker: MockerFixture) -> None:
    task, solver = _task(mocker)
    task.on_ee_twist_command(_twist(), t_now=1.0)
    task.compute(_state(1.01))

    task.on_preempted("trajectory", frozenset({"arm/joint1"}))
    task.compute(_state(1.02))

    assert solver.frame_poses.call_count == 2
    assert solver.reset.call_count >= 1


def test_invalid_twist_is_rejected(mocker: MockerFixture) -> None:
    task, _ = _task(mocker)
    invalid = _twist()
    invalid.linear.x = np.nan

    assert not task.on_ee_twist_command(invalid, t_now=1.0)


@pytest.mark.parametrize("rate", [-0.2, 0.2])
@pytest.mark.parametrize(
    "field,joint,axis",
    [("base_rotation_joint", "arm/joint1", 2), ("pitch_rotation_joint", "arm/joint2", 1)],
)
def test_rotation_passes_only_selected_joint_velocity_to_shared_controller(
    mocker: MockerFixture, rate: float, field: str, joint: str, axis: int
) -> None:
    solver = _solver(mocker)
    pose = PoseStamped(position=[0.3, 0, 0.2], orientation=Rotation.from_euler("y", 0.7).as_quat())
    solver.frame_poses.return_value = {"tool": pose}
    mocker.patch("dimos.control.tasks.pose_target_ik.PinkPoseTargetSolver", return_value=solver)
    task = create_task(
        eef_twist_task(robot_model=_robot_model(), target_frame="tool", **{field: joint}),
        {},
    )
    angular = [0.0, 0.0, 0.0]
    angular[axis] = rate
    task.on_ee_twist_command(TwistStamped(angular=angular), t_now=1.0)

    assert task.compute(_state(1.1, dt=0.1)) is not None
    assert task.compute(_state(1.2, dt=0.1)) is not None

    target = solver.step.call_args.args[0]["tool"]
    assert target.position.to_numpy() == pytest.approx(pose.position.to_numpy())
    assert target.orientation.to_rotation_matrix() == pytest.approx(
        pose.orientation.to_rotation_matrix()
    )
    for call in solver.step.call_args_list:
        assert call.kwargs["joint_velocities"] == {joint: rate}
    assert task.claim().joints == frozenset({"arm/joint1", "arm/joint2"})


@pytest.mark.parametrize("angular,axis", [([0.0, 0.0, 0.2], "z"), ([0.0, 0.2, 0.0], "y")])
def test_joint_rotation_is_opt_in(mocker: MockerFixture, angular, axis) -> None:
    task, solver = _task(mocker)
    task.on_ee_twist_command(TwistStamped(angular=angular), t_now=1.0)

    task.compute(_state(1.1, dt=0.1))

    assert "joint_velocities" not in solver.step.call_args.kwargs
    target = solver.step.call_args.args[0]["tool"]
    assert target.orientation.to_rotation_matrix() == pytest.approx(
        Rotation.from_euler(axis, 0.02).as_matrix()
    )


@pytest.mark.parametrize("field", ["base_rotation_joint", "pitch_rotation_joint"])
def test_joint_rotation_requires_a_controlled_joint(mocker: MockerFixture, field) -> None:
    with pytest.raises(ValueError, match=field):
        _task(mocker, **{field: "other/joint"})


@pytest.mark.parametrize(
    "linear,angular", [([0.1, 0.0, 0.0], [0.0, 0.0, 0.2]), ([0.0, 0.0, 0.0], [0.0, 0.2, 0.2])]
)
def test_mixed_twist_keeps_cartesian_control(
    mocker: MockerFixture, linear: list[float], angular: list[float]
) -> None:
    task, solver = _task(
        mocker, base_rotation_joint="arm/joint1", pitch_rotation_joint="arm/joint2"
    )
    task.on_ee_twist_command(TwistStamped(linear=linear, angular=angular), t_now=1.0)

    assert task.compute(_state(1.01)) is not None

    assert "joint_velocities" not in solver.step.call_args.kwargs


def test_switching_between_cartesian_and_base_reanchors_controller(
    mocker: MockerFixture,
) -> None:
    task, solver = _task(mocker, base_rotation_joint="arm/joint1")
    task.on_ee_twist_command(_twist(), t_now=1.0)
    task.compute(_state(1.01))
    resets = solver.reset.call_count
    pose = PoseStamped(position=[0.3, 0.1, 0.2])
    solver.frame_poses.return_value = {"tool": pose}

    task.on_ee_twist_command(TwistStamped(angular=[0.0, 0.0, 0.2]), t_now=1.02)
    assert task.compute(_state(1.03)) is not None

    assert solver.reset.call_count == resets + 1
    assert solver.step.call_args.kwargs["joint_velocities"] == {"arm/joint1": 0.2}
    assert solver.step.call_args.args[0]["tool"].position.to_numpy() == pytest.approx(
        pose.position.to_numpy()
    )
    task.on_ee_twist_command(_twist(), t_now=1.04)
    assert task.compute(_state(1.05)) is not None

    assert solver.reset.call_count == resets + 2
    assert solver.frame_poses.call_count == 3
    assert "joint_velocities" not in solver.step.call_args.kwargs
    assert solver.step.call_args.args[0]["tool"].position.to_numpy() == pytest.approx(
        [0.301, 0.1, 0.2]
    )


def test_switching_between_base_pitch_and_cartesian_discards_previous_motion(mocker):
    task, solver = _task(
        mocker, base_rotation_joint="arm/joint1", pitch_rotation_joint="arm/joint2"
    )
    task.on_ee_twist_command(TwistStamped(angular=[0, 0, 0.2]), t_now=1.0)
    task.compute(_state(1.01))
    resets = solver.reset.call_count

    task.on_ee_twist_command(TwistStamped(angular=[0, -0.2, 0]), t_now=1.02)
    assert task.compute(_state(1.03)) is not None
    assert solver.step.call_args.kwargs["joint_velocities"] == {"arm/joint2": -0.2}
    assert solver.reset.call_count == resets + 1

    task.on_ee_twist_command(TwistStamped(angular=[0, 0, -0.2]), t_now=1.04)
    assert task.compute(_state(1.05)) is not None
    assert solver.step.call_args.kwargs["joint_velocities"] == {"arm/joint1": -0.2}
    assert solver.reset.call_count == resets + 2

    task.on_ee_twist_command(_twist(), t_now=1.06)
    assert task.compute(_state(1.07)) is not None
    assert "joint_velocities" not in solver.step.call_args.kwargs
    assert solver.reset.call_count == resets + 3
    assert solver.frame_poses.call_count == 4


@pytest.fixture(
    params=[
        ("base_rotation_joint", "arm/joint1", [0.0, 0.0, 0.2]),
        ("pitch_rotation_joint", "arm/joint2", [0.0, 0.2, 0.0]),
    ]
)
def joint_rotation(request):
    return request.param


def test_centering_joint_rotation_holds_without_ik_then_releases_joints(
    mocker: MockerFixture, joint_rotation
) -> None:
    field, joint, angular = joint_rotation
    task, solver = _task(mocker, **{field: joint})
    task.on_ee_twist_command(TwistStamped(angular=angular), t_now=1.0)
    task.compute(_state(1.01))
    resets = solver.reset.call_count

    task.on_ee_twist_command(TwistStamped(), t_now=1.21)
    assert task.compute(_state(1.22)) is not None

    assert solver.reset.call_count == resets + 1
    assert solver.step.call_args.kwargs["joint_velocities"] == {joint: 0.0}
    assert task.compute(_state(1.4)) is not None
    assert solver.step.call_args.kwargs["joint_velocities"] == {joint: 0.0}
    assert not task.is_active()
    calls = solver.step.call_count
    assert task.compute(_state(1.41)) is None
    assert solver.step.call_count == calls


def test_joint_rotation_timeout_stops_without_ik(mocker: MockerFixture, joint_rotation) -> None:
    field, joint, angular = joint_rotation
    task, solver = _task(mocker, **{field: joint})
    task.on_ee_twist_command(TwistStamped(angular=angular), t_now=1.0)
    task.compute(_state(1.01))
    resets = solver.reset.call_count

    assert task.compute(_state(1.31)) is not None

    assert solver.reset.call_count == resets + 1
    assert solver.step.call_args.kwargs["joint_velocities"] == {joint: 0.0}
    assert not task.is_active()
    task.on_ee_twist_command(TwistStamped(angular=[-value for value in angular]), t_now=2.0)
    assert task.compute(_state(2.01)) is not None
    assert solver.reset.call_count == resets + 2
    assert solver.step.call_args.kwargs["joint_velocities"] == {joint: -0.2}


def test_joint_rotation_estop_clears_motion_and_preserves_direct_hold(
    mocker: MockerFixture, joint_rotation
) -> None:
    field, joint, angular = joint_rotation
    task, solver = _task(mocker, **{field: joint})
    twist = TwistStamped(angular=angular)
    task.on_ee_twist_command(twist, t_now=1.0)
    task.compute(_state(1.01))

    task.set_estop(True)
    assert not task.on_ee_twist_command(twist, t_now=1.02)
    assert task.compute(_state(1.02)) is None

    task.set_estop(False)
    assert task.compute(_state(1.03)) is not None
    assert solver.step.call_args.kwargs["joint_velocities"] == {joint: 0.0}
    assert solver.frame_poses.call_count == 2
