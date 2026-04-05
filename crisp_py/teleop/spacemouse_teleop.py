"""SpaceMouse teleoperation input for the UR10e arm.

Uses the ``pyspacemouse`` library to read 3Dconnexion SpaceMouse events.
The SpaceMouse provides 6-DOF analogue input (3 translation + 3 rotation axes)
which maps directly to Cartesian arm control.

Installation::

    pip install pyspacemouse
    # or in pixi:
    pixi add pyspacemouse

Hardware setup (Linux):
    The SpaceMouse appears as a USB HID device.  No kernel driver is needed
    on modern kernels.  If you see a permission error on ``/dev/hidraw*``,
    add a udev rule::

        echo 'SUBSYSTEM=="hidraw", ATTRS{idVendor}=="256f", MODE="0666"' \\
            | sudo tee /etc/udev/rules.d/99-spacemouse.rules
        sudo udevadm control --reload-rules && sudo udevadm trigger

Usage::

    from crisp_py.teleop import SpaceMouseTeleopInput

    with SpaceMouseTeleopInput(pos_scale=0.01, rot_scale=0.05) as sm:
        print("SpaceMouse ready. Push/pull to move the arm.")
        while True:
            cmd = sm.poll()
            if cmd.stop_episode:
                break
            robot.move_cartesian_async(apply_delta(current_pose, cmd))
            time.sleep(0.02)

SpaceMouse button mapping (default):
    Button 0 (left)  →  stop episode (save)
    Button 1 (right) →  open gripper (hold)

    Customise via ``button_stop``, ``button_gripper_open``,
    ``button_gripper_close`` constructor arguments.
"""

import threading
from typing import Optional

import numpy as np

from crisp_py.teleop.teleop_input import TeleopCommand, TeleopInput


class SpaceMouseTeleopInput(TeleopInput):
    """3Dconnexion SpaceMouse teleoperation input.

    Maps the 6-DOF analogue axes to Cartesian position and rotation deltas
    for the arm EE.  Button 0 (left) stops the episode; button 1 (right)
    toggles the gripper.

    Args:
        pos_scale:           Multiplier applied to the raw translation axes
                             to get position deltas [m per poll tick].
                             Tune based on your loop frequency (e.g. 0.01 at
                             20 Hz gives up to 0.2 m/s).
        rot_scale:           Multiplier applied to raw rotation axes [rad per tick].
        deadband:            Axis values below this magnitude are treated as
                             zero (prevents drift from a centred device).
        button_stop:         SpaceMouse button index that triggers stop_episode.
        button_gripper_open: Button index for gripper open (-1 = disabled).
        button_gripper_close: Button index for gripper close (-1 = disabled).
        device_index:        Index of the SpaceMouse device if multiple are
                             connected (0 = first device found).
    """

    def __init__(
        self,
        pos_scale: float = 0.01,
        rot_scale: float = 0.05,
        deadband: float = 0.05,
        button_stop: int = 0,
        button_gripper_open: int = -1,
        button_gripper_close: int = 1,
        device_index: int = 0,
    ) -> None:
        try:
            import pyspacemouse  # noqa: F401
        except ImportError as exc:
            raise ImportError(
                "pyspacemouse is required for SpaceMouseTeleopInput.\n"
                "Install it with:  pip install pyspacemouse\n"
                "or in pixi:      pixi add pyspacemouse"
            ) from exc

        self._pos_scale = pos_scale
        self._rot_scale = rot_scale
        self._deadband = deadband
        self._button_stop = button_stop
        self._button_gripper_open = button_gripper_open
        self._button_gripper_close = button_gripper_close
        self._device_index = device_index

        self._lock = threading.Lock()
        self._pos_acc = np.zeros(3, dtype=np.float64)
        self._rot_acc = np.zeros(3, dtype=np.float64)
        self._gripper: float = 0.0
        self._stop_episode: bool = False
        self._discard_episode: bool = False
        self._quit: bool = False

        self._thread: Optional[threading.Thread] = None
        self._running: bool = False
        self._device = None

    # ------------------------------------------------------------------ #
    #  Lifecycle                                                           #
    # ------------------------------------------------------------------ #

    def open(self) -> None:
        """Open the SpaceMouse device and start the reading thread."""
        if self._running:
            return
        import pyspacemouse
        self._device = pyspacemouse.open(DeviceNumber=self._device_index)
        if self._device is None:
            raise RuntimeError(
                "No SpaceMouse device found. "
                "Check USB connection and /dev/hidraw permissions."
            )
        self._running = True
        self._thread = threading.Thread(
            target=self._read_loop,
            daemon=True,
            name="spacemouse_teleop_reader",
        )
        self._thread.start()

    def close(self) -> None:
        """Stop reading and close the device."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._device is not None:
            try:
                import pyspacemouse
                pyspacemouse.close()
            except Exception:
                pass
            self._device = None

    @property
    def is_open(self) -> bool:
        return self._running

    # ------------------------------------------------------------------ #
    #  Poll                                                                #
    # ------------------------------------------------------------------ #

    def poll(self) -> TeleopCommand:
        """Return and reset the accumulated SpaceMouse command."""
        with self._lock:
            cmd = TeleopCommand(
                pos_delta=self._pos_acc.copy(),
                rot_delta=self._rot_acc.copy(),
                gripper=self._gripper,
                stop_episode=self._stop_episode,
                discard_episode=self._discard_episode,
                quit=self._quit,
            )
            self._pos_acc[:] = 0.0
            self._rot_acc[:] = 0.0
            self._stop_episode = False
            self._discard_episode = False
            self._quit = False

        return cmd

    # ------------------------------------------------------------------ #
    #  Background reader                                                   #
    # ------------------------------------------------------------------ #

    def _dead(self, v: float) -> float:
        return v if abs(v) > self._deadband else 0.0

    def _read_loop(self) -> None:
        """Continuously accumulate SpaceMouse events."""
        import pyspacemouse

        while self._running:
            state = pyspacemouse.read()
            if state is None:
                continue

            # SpaceMouse axes: x=left/right, y=forward/back, z=up/down,
            # roll, pitch, yaw (all in device frame).
            # Map device frame → robot base_link frame deltas:
            #   device +x → robot +Y (rightward in base frame)
            #   device +y → robot +X (forward in base frame)
            #   device +z → robot +Z (upward)
            with self._lock:
                self._pos_acc[0] += self._dead(state.y) * self._pos_scale
                self._pos_acc[1] += self._dead(state.x) * self._pos_scale
                self._pos_acc[2] += self._dead(state.z) * self._pos_scale
                self._rot_acc[0] += self._dead(state.roll)  * self._rot_scale
                self._rot_acc[1] += self._dead(state.pitch) * self._rot_scale
                self._rot_acc[2] += self._dead(state.yaw)   * self._rot_scale

                # Buttons (one-shot on press)
                if self._button_stop >= 0 and len(state.buttons) > self._button_stop:
                    if state.buttons[self._button_stop]:
                        self._stop_episode = True

                if (
                    self._button_gripper_open >= 0
                    and len(state.buttons) > self._button_gripper_open
                    and state.buttons[self._button_gripper_open]
                ):
                    self._gripper = -1.0   # open

                if (
                    self._button_gripper_close >= 0
                    and len(state.buttons) > self._button_gripper_close
                    and state.buttons[self._button_gripper_close]
                ):
                    self._gripper = +1.0   # close
