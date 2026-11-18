# Galaxea A1Z

The A1Z integration uses the vendor's 250 Hz position-control loop, the G1Z
gravity model, and the G1Z gripper. The SDK is not published on PyPI, so install
the pinned Git revision until the vendor publishes a compatible release.

## Install from a source checkout

Create a developer environment with the [official installer](/docs/installation/index.md) first. Then install the A1Z-specific SDK and tools below.

The repository setup script shows its complete plan and asks for confirmation
before it changes the checkout environment or installs system packages:

```bash
bin/hardware/a1z/setup
```

The script runs a locked, inexact sync so it preserves other extras in the
checkout. It installs `can-utils` on Ubuntu and `libusb` through Homebrew on
macOS when needed. On other Linux distributions, it prints the missing system
package instead of selecting a package manager for you.

## Add the vendor SDK to a library environment

Follow the [official installation guide](/docs/installation/index.md) and select the `manipulation` extra. In that environment, add the A1Z vendor SDK with the package manager that owns it:

```bash
python -m pip install 'a1z @ git+https://github.com/userguide-galaxea/GALAXEA-A1Z.git@e931ecd0e25ad35df251097ba42921b3d2fa7224'
```

On macOS, also install the userspace USB dependencies:

```bash
python -m pip install 'gs-usb==0.3.1' 'pyusb==1.3.1'
brew install libusb
```

On Linux, install the package that provides `cansend`. Ubuntu calls this package
`can-utils`:

```bash
sudo apt-get install can-utils
```

Check the installation without changing the host:

```bash
dimos hardware a1z doctor --software-only
```

## Linux setup

Connect and power the arm, then configure its CAN interface:

```bash
dimos hardware a1z configure-can
```

The command asks for confirmation, then uses `sudo` to:

1. Load the Linux `gs_usb` kernel driver.
2. Bind the HHS USB-CANFD adapter.
3. Create the stable `a1zcan` SocketCAN interface at 1 Mbit/s.
4. Send a safe probe and verify that the adapter transmitted it.

Do not start the arm unless configuration reports a successful transmission.
Run the full read-only diagnostic afterward:

```bash
dimos hardware a1z doctor
```

After rebooting or reconnecting the HHS adapter, run `configure-can` again.

## macOS setup

macOS uses the HHS adapter directly through libusb because it does not provide
SocketCAN. The doctor verifies the gripper-capable SDK, finds the attached HHS
USB-CANFD adapter, opens it in listen-only mode, and closes it without
transmitting or enabling the arm:

```bash
dimos hardware a1z doctor
```

## Run

The A1Z has no brakes or hardware e-stop button; the PSU switch is the hardware
kill switch. Support the arm and clear its workspace before starting or
stopping dimOS. Disabling the motors makes the arm fall.

Every A1Z blueprint drives the real arm through `A1ZConnection`; there is no
mock arm. Name the CAN interface after the blueprint name:

```bash
dimos run a1z-keyboard-teleop --a1zconnection.can-port a1zcan      # keyboard, in a pygame window
dimos run a1z-teleop --a1zconnection.can-port a1zcan               # Desktop Manipulation app
dimos run teleop-webxr-a1z --a1zconnection.can-port a1zcan         # VR headset, left controller
dimos run a1z-planner-coordinator --a1zconnection.can-port a1zcan  # motion planning
dimos run coordinator-a1z --a1zconnection.can-port a1zcan          # the arm behind a coordinator
```

On Linux, name another verified SocketCAN interface, such as `can0`, if needed.
On macOS the HHS adapter is opened over USB, whatever the name.
`A1ZCONNECTION__CAN_PORT=a1zcan` in the environment works too.

At start, `A1ZConnection` waits for all six arm motors, checks their readings,
holds the arm where it is, then ramps in gravity compensation. When commands stop for 0.1 s,
or the coordinator's `set_estop(True)` stops them, the arm holds where it is.
After starting, and before its motors turn off on stop, it moves to its home:
all joints at 0 rad, folded at rest. Set `--a1zconnection.home '[...]'` (six
angles in rad) for another pose.

### SpaceMouse control

Install the optional Python driver with `uv sync --extra spacemouse --inexact`. On macOS,
install `hidapi` with `brew install hidapi`. On Linux, install `libhidapi-hidraw0` and give
your user access to the SpaceMouse HID device.

Plug the SpaceMouse into the computer running DimOS and launch the A1Z with its input module:

```bash
dimos run galaxea-a1z-spacemouse --a1zconnection.can-port a1zcan
```

`SpaceMouseTeleopModule` publishes `ee_twist_command` (arm motion) and
`gripper_command` (normalized opening). The A1Z coordinator consumes those streams.
If `galaxea-a1z-localize` is already running, you can attach a SpaceMouse from the
operator's computer with `dimos hardware a1z spacemouse` in a second terminal.

SpaceMouse control starts **disabled**, including the gripper buttons. In the
terminal running the SpaceMouse, tap **s** to toggle control on or off. No Enter is
needed. Disabling sends a stop; enabling accepts the current mouse input immediately.
The terminal prints the enabled/disabled state. Tap the key rather than holding it,
since keyboard repeat can toggle it again.
This terminal toggle belongs to the SpaceMouse module and also works in the
attachment command. Run it in the foreground; for a background robot stack, use
`galaxea-a1z-localize --daemon` and the separate foreground attachment command.
The toggle gates SpaceMouse input only; it does not disable other robot commands.

| SpaceMouse Compact input | Gripper action |
| --- | --- |
| Push the cap away / pull it toward you | Move forward / backward |
| Slide the cap left / right | Rotate only the base joint left / right |
| Lift / press the cap | Move up / down |
| Twist right (clockwise from above) / left | Nod up / down using only joint 4 |
| Left button / right button | Open / close |

The strongest cap motion selects one axis at a time, including twist. Deflection
past the 20% dead zone commands a steady 5 cm/s of translation, 10°/s of base
rotation, or 15°/s of wrist rotation.
Another axis must be clearly stronger to switch directions. This keeps small
incidental cap movements from adding a second motion.

Forward/backward and up/down follow straight lines along the arm base axes.
Sideways input rotates joint 1 directly; joints 2–6 hold their angles, so the tip
follows an arc and the jaws turn with the base. The A1Z task's `base_rotation_joint`
setting routes pure `angular.z` commands to joint 1. The `pitch_rotation_joint`
setting routes pure `angular.y` commands to joint 4: right twist decreases its angle
to nod up, and left twist increases it to nod down. Every other joint holds its angle,
so the gripper tip moves in an arc around the wrist. Both rotations bypass IK and
reuse the controller's speed, joint-limit, and tracking-error bounds. SpaceMouse
input no longer needs gripper-pose feedback. The arm controller smooths joint motion.
Centering the cap sends a stop. Button presses also work while moving.
The fully folded startup pose has little room to move backward or down. To start
from the working home pose, run `dimos mcp call go_home` with the cap centered.
This manual control bypasses collision planning. Ctrl-C in the blueprint terminal
stops the whole stack. Ctrl-C in the separate attachment command ends only its input.

## Troubleshooting

- **The interface is UP, but the arm does not respond.** Some Linux `gs_usb`
  drivers create an interface but drop every transmission through the HHS
  adapter. Update to a kernel with the endpoint-discovery fix. On Jetson or
  another pinned-kernel system, follow the
  [Galaxea driver guide](https://galaxea-ai.feishu.cn/docx/XF2ed4pmhoervNxODlfc11Gvnbb)
  and build the driver for the exact running kernel. Do not install a desktop
  kernel or copy a kernel module from another machine.
- **The bus behaves strangely after a crash.** Replug the HHS adapter and rerun
  `dimos hardware a1z configure-can` on Linux or
  `dimos hardware a1z doctor` on macOS.
- **macOS cannot find or open the adapter.** Check that libusb is installed,
  reconnect the adapter, and rerun `dimos hardware a1z doctor`.
- **The gripper remains stiff after shutdown.** dimOS sends the off command
  twice, but a degraded bus can still lose it. Support the arm and turn off
  the PSU.
