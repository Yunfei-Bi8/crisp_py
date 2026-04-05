"""Teleoperation input device interfaces for arm control.

Provides a unified interface for different input devices that generate
Cartesian deltas for the UR10e end-effector.

Available input devices:

  * :class:`KeyboardTeleopInput` — keyboard (WASD + arrows, always available)
  * :class:`SpaceMouseTeleopInput` — 3Dconnexion SpaceMouse (requires pyspacemouse)

All devices share the same :class:`TeleopCommand` output format and
:class:`TeleopInput` abstract interface.
"""

from crisp_py.teleop.teleop_input import TeleopCommand, TeleopInput, apply_delta
from crisp_py.teleop.keyboard_teleop import KeyboardTeleopInput
from crisp_py.teleop.spacemouse_teleop import SpaceMouseTeleopInput

__all__ = [
    "TeleopCommand",
    "TeleopInput",
    "apply_delta",
    "KeyboardTeleopInput",
    "SpaceMouseTeleopInput",
]
