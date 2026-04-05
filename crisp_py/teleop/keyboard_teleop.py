"""Keyboard-based teleoperation input for the UR10e arm.

Uses ``termios``/``tty`` to put the terminal in raw mode so individual
key-presses are delivered without waiting for Enter.  A background daemon
thread reads key events continuously; the main loop calls ``poll()`` once
per tick to consume the accumulated state.

Key bindings (all lower-case unless noted):

    EE position (pos_scale m per tick):
        w / s   →  +X / -X
        a / d   →  -Y / +Y
        q / e   →  +Z / -Z

    EE rotation (rot_scale rad per tick, applied in EE frame):
        ↑ / ↓   →  pitch +/-
        ← / →   →  yaw +/-
        r / f   →  roll +/-

    Gripper:
        o       →  open  (gripper = -1)
        c       →  close (gripper = +1)

    Episode control:
        Space   →  stop episode (save)
        x       →  discard episode (abort without saving)
        Esc     →  quit program

Usage::

    from crisp_py.teleop import KeyboardTeleopInput

    with KeyboardTeleopInput(pos_scale=0.005, rot_scale=0.02) as kb:
        print(kb.help_text())
        while True:
            cmd = kb.poll()
            if cmd.quit:
                break
            if cmd.stop_episode:
                ...
            robot.move_cartesian_async(apply_delta(current_pose, cmd))
"""

import sys
import termios
import threading
import time
import tty
from typing import Optional

import numpy as np

from crisp_py.teleop.teleop_input import TeleopCommand, TeleopInput


# ANSI escape codes for arrow keys (read as 3-byte sequence: ESC [ X)
_UP    = b'\x1b[A'
_DOWN  = b'\x1b[B'
_RIGHT = b'\x1b[C'
_LEFT  = b'\x1b[D'


class KeyboardTeleopInput(TeleopInput):
    """Non-blocking keyboard input for arm teleoperation.

    Runs a background thread that reads one byte at a time from stdin in raw
    mode.  ``poll()`` collects the accumulated deltas since the last call and
    returns them as a ``TeleopCommand``.

    Args:
        pos_scale:  Position increment per key-press [m].
        rot_scale:  Rotation increment per key-press [rad].
        poll_rate:  How fast the background reader polls stdin [Hz].
                    Higher rates reduce input latency but use more CPU.
    """

    def __init__(
        self,
        pos_scale: float = 0.005,
        rot_scale: float = 0.02,
        poll_rate: float = 200.0,
    ) -> None:
        self._pos_scale = pos_scale
        self._rot_scale = rot_scale
        self._poll_dt = 1.0 / poll_rate

        # Accumulated state — protected by _lock
        self._lock = threading.Lock()
        self._pos_acc = np.zeros(3, dtype=np.float64)
        self._rot_acc = np.zeros(3, dtype=np.float64)
        self._gripper: float = 0.0
        self._stop_episode: bool = False
        self._discard_episode: bool = False
        self._quit: bool = False

        self._thread: Optional[threading.Thread] = None
        self._running: bool = False
        self._old_settings = None

    # ------------------------------------------------------------------ #
    #  Lifecycle                                                           #
    # ------------------------------------------------------------------ #

    def open(self) -> None:
        """Put stdin in raw mode and start the background reader thread."""
        if self._running:
            return
        self._old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        self._running = True
        self._thread = threading.Thread(
            target=self._read_loop,
            daemon=True,
            name="keyboard_teleop_reader",
        )
        self._thread.start()

    def close(self) -> None:
        """Stop the reader and restore terminal settings."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
            self._old_settings = None

    @property
    def is_open(self) -> bool:
        return self._running

    # ------------------------------------------------------------------ #
    #  Poll                                                                #
    # ------------------------------------------------------------------ #

    def poll(self) -> TeleopCommand:
        """Consume and return the accumulated input since the last call."""
        with self._lock:
            cmd = TeleopCommand(
                pos_delta=self._pos_acc.copy(),
                rot_delta=self._rot_acc.copy(),
                gripper=self._gripper,
                stop_episode=self._stop_episode,
                discard_episode=self._discard_episode,
                quit=self._quit,
            )
            # Reset accumulators (one-shot flags reset, gripper holds)
            self._pos_acc[:] = 0.0
            self._rot_acc[:] = 0.0
            self._stop_episode = False
            self._discard_episode = False
            self._quit = False
            # Gripper intentionally NOT reset — holds last command

        return cmd

    # ------------------------------------------------------------------ #
    #  Background reader                                                   #
    # ------------------------------------------------------------------ #

    def _read_loop(self) -> None:
        """Continuously read raw bytes from stdin and update accumulated state."""
        import select

        while self._running:
            # Non-blocking check: is there a byte ready?
            ready, _, _ = select.select([sys.stdin], [], [], self._poll_dt)
            if not ready:
                continue

            try:
                ch = sys.stdin.buffer.read(1)
            except OSError:
                break

            if not ch:
                break

            # Arrow keys are 3-byte sequences: ESC '[' code
            if ch == b'\x1b':
                # Check for escape sequence vs bare Escape key
                ready2, _, _ = select.select([sys.stdin], [], [], 0.02)
                if not ready2:
                    # Bare Escape → quit
                    with self._lock:
                        self._quit = True
                    continue
                try:
                    ch2 = sys.stdin.buffer.read(1)
                    ch3 = sys.stdin.buffer.read(1)
                except OSError:
                    break
                seq = ch + ch2 + ch3
                self._handle_escape(seq)
            else:
                self._handle_char(ch)

    def _handle_char(self, ch: bytes) -> None:
        s = ch.decode(errors="ignore").lower()
        with self._lock:
            if s == 'w':
                self._pos_acc[0] += self._pos_scale    # +X
            elif s == 's':
                self._pos_acc[0] -= self._pos_scale    # -X
            elif s == 'a':
                self._pos_acc[1] -= self._pos_scale    # -Y
            elif s == 'd':
                self._pos_acc[1] += self._pos_scale    # +Y
            elif s == 'q':
                self._pos_acc[2] += self._pos_scale    # +Z
            elif s == 'e':
                self._pos_acc[2] -= self._pos_scale    # -Z
            elif s == 'r':
                self._rot_acc[0] += self._rot_scale    # roll +
            elif s == 'f':
                self._rot_acc[0] -= self._rot_scale    # roll -
            elif s == 'o':
                self._gripper = -1.0                   # open
            elif s == 'c':
                self._gripper = +1.0                   # close
            elif s == ' ':
                self._stop_episode = True              # stop + save
            elif s == 'x':
                self._discard_episode = True           # discard
            elif ch == b'\x03':                        # Ctrl-C
                self._quit = True

    def _handle_escape(self, seq: bytes) -> None:
        with self._lock:
            if seq == _UP:
                self._rot_acc[1] += self._rot_scale    # pitch +
            elif seq == _DOWN:
                self._rot_acc[1] -= self._rot_scale    # pitch -
            elif seq == _LEFT:
                self._rot_acc[2] += self._rot_scale    # yaw +
            elif seq == _RIGHT:
                self._rot_acc[2] -= self._rot_scale    # yaw -

    # ------------------------------------------------------------------ #
    #  Help                                                                #
    # ------------------------------------------------------------------ #

    @staticmethod
    def help_text() -> str:
        """Return a formatted key-binding reference for the user."""
        return (
            "\n"
            "=== Keyboard Teleoperation Controls ===\n"
            "  Position (base_link frame):\n"
            "    W / S        →  +X / -X\n"
            "    A / D        →  -Y / +Y\n"
            "    Q / E        →  +Z / -Z\n"
            "\n"
            "  Rotation (EE frame):\n"
            "    ↑ / ↓        →  pitch + / -\n"
            "    ← / →        →  yaw  + / -\n"
            "    R / F        →  roll + / -\n"
            "\n"
            "  Gripper:\n"
            "    O            →  open\n"
            "    C            →  close\n"
            "\n"
            "  Episode control:\n"
            "    Space        →  stop & save episode\n"
            "    X            →  discard episode\n"
            "    Esc          →  quit program\n"
            "======================================="
        )
