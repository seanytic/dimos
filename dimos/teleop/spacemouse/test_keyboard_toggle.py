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

import builtins
import errno
import os
import pty
import select
import termios

import pytest

from dimos.teleop.spacemouse import spacemouse_teleop_module as spacemouse


def terminal_settings(fd):
    settings = termios.tcgetattr(fd)
    # macOS sets this kernel-managed retype-pending bit when restoring canonical mode.
    settings[3] &= ~termios.PENDIN
    return settings


@pytest.fixture
def terminal(mocker):
    master, slave = pty.openpty()
    real_open = builtins.open
    opened = []

    def open_terminal(path, *args, **kwargs):
        if path == "/dev/tty":
            stream = real_open(os.ttyname(slave), *args, **kwargs)
            opened.append(stream)
            return stream
        return real_open(path, *args, **kwargs)

    mocker.patch("builtins.open", side_effect=open_terminal)
    try:
        yield master, slave, opened
    finally:
        for stream in opened:
            stream.close()
        os.close(slave)
        os.close(master)


@pytest.mark.parametrize(
    "keys, expected", [(b"s", True), (b"S", True), (b"x", False), (b"sss", True)]
)
def test_keyboard_polls_s_without_enter_and_consumes_buffer(terminal, keys, expected):
    master, slave, opened = terminal
    with spacemouse.open_keyboard() as pressed:
        assert not pressed()
        os.write(master, keys)
        assert select.select([slave], [], [], 1.0)[0] == [slave]

        assert pressed() is expected
        assert not pressed()

    assert opened[0].closed


def test_keyboard_restores_terminal_flags_after_exception(terminal):
    _, slave, opened = terminal
    before = terminal_settings(slave)

    with pytest.raises(ValueError, match="reader stopped"):
        with spacemouse.open_keyboard():
            active = termios.tcgetattr(slave)
            assert not active[3] & (termios.ICANON | termios.ECHO)
            assert active[3] & termios.ISIG == before[3] & termios.ISIG
            raise ValueError("reader stopped")

    assert terminal_settings(slave) == before
    assert opened[0].closed


def test_keyboard_without_controlling_terminal_explains_foreground_requirement(mocker):
    mocker.patch("builtins.open", side_effect=OSError(errno.ENXIO, "no controlling terminal"))

    with pytest.raises(RuntimeError, match="foreground"):
        with spacemouse.open_keyboard():
            pytest.fail("a missing terminal must not enable control")


def test_keyboard_eof_fails_closed_and_restores_terminal(terminal, mocker):
    master, slave, opened = terminal
    before = terminal_settings(slave)
    with spacemouse.open_keyboard() as pressed:
        os.write(master, b"x")
        assert select.select([slave], [], [], 1.0)[0] == [slave]
        mocker.patch.object(opened[0], "read", return_value=b"")

        with pytest.raises(OSError, match="disconnected"):
            pressed()

    assert terminal_settings(slave) == before
    assert opened[0].closed


def test_keyboard_setup_failure_closes_terminal(terminal, mocker):
    _, slave, opened = terminal
    before = terminal_settings(slave)
    mocker.patch.object(spacemouse.tty, "setcbreak", side_effect=termios.error("setup failed"))

    with pytest.raises(termios.error, match="setup failed"):
        with spacemouse.open_keyboard():
            pytest.fail("a failed keyboard setup must not enable control")

    assert terminal_settings(slave) == before
    assert opened[0].closed
