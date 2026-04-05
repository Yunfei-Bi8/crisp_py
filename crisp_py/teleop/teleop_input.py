"""Abstract interface for teleoperation input devices.

This module defines the common contract that all teleoperation input devices
must fulfil.  Concrete implementations are in sibling modules:

  * ``KeyboardTeleopInput``  — keyboard (WASD + arrows, Linux terminal)
  * ``SpaceMouseTeleopInput`` — 3Dconnexion SpaceMouse (requires pyspacemouse)

Each implementation runs its device-reading loop in a background daemon
thread so that ``poll()`` is always non-blocking.  The main teleoperation
loop calls ``poll()`` once per control tick to get the latest command.

Usage pattern::

    from crisp_py.teleop import KeyboardTeleopInput

    with KeyboardTeleopInput(pos_scale=0.005, rot_scale=0.02) as inp:
        while True:
            cmd = inp.poll()
            if cmd.quit:
                break
            # Apply cmd.pos_delta / cmd.rot_delta to the robot...
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np
from scipy.spatial.transform import Rotation

if TYPE_CHECKING:
    from crisp_py.utils.geometry import Pose


# --------------------------------------------------------------------------- #
#  Command dataclass                                                           #
# --------------------------------------------------------------------------- #

@dataclass
class TeleopCommand:
    """One control tick's worth of teleoperation input.

    Attributes:
        pos_delta:       (3,) EE position increment [m] in base_link frame.
                         Applied by: next_pos = current_pos + pos_delta
        rot_delta:       (3,) EE rotation increment [rad] as axis-angle
                         in the EE frame.  Apply via:
                         ``Rotation.from_rotvec(rot_delta) * current_rot``
        gripper:         Normalised gripper command.
                         -1.0 = fully open, +1.0 = fully close, 0.0 = hold.
        stop_episode:    User pressed "stop" — finish and save this episode.
        discard_episode: User pressed "discard" — abort without saving.
        quit:            User pressed "quit" — exit the whole program.
    """

    pos_delta: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=np.float64)
    )
    rot_delta: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=np.float64)
    )
    gripper: float = 0.0
    stop_episode: bool = False
    discard_episode: bool = False
    quit: bool = False


# --------------------------------------------------------------------------- #
#  Abstract base class                                                         #
# --------------------------------------------------------------------------- #

class TeleopInput(ABC):
    """Abstract base for all teleoperation input devices.

    Subclasses must start their device-reading loop in ``open()`` (or in
    ``__init__``) and expose the latest accumulated state via ``poll()``.

    The context-manager interface is strongly recommended::

        with MyTeleopInput(...) as inp:
            while True:
                cmd = inp.poll()
    """

    # ------------------------------------------------------------------ #
    #  Context manager                                                     #
    # ------------------------------------------------------------------ #

    def __enter__(self) -> "TeleopInput":
        self.open()
        return self

    def __exit__(self, *_) -> None:
        self.close()

    # ------------------------------------------------------------------ #
    #  Abstract API                                                        #
    # ------------------------------------------------------------------ #

    @abstractmethod
    def open(self) -> None:
        """Start the input device (called by ``__enter__``)."""

    @abstractmethod
    def close(self) -> None:
        """Stop the input device and clean up (called by ``__exit__``)."""

    @abstractmethod
    def poll(self) -> TeleopCommand:
        """Return the accumulated command since the last poll() call.

        This method must be non-blocking.  After returning, the accumulated
        state is reset so that button presses are not delivered twice.

        Returns:
            A TeleopCommand containing the current user intent.
        """

    @property
    @abstractmethod
    def is_open(self) -> bool:
        """True if the device is open and reading."""


# --------------------------------------------------------------------------- #
#  Helper: apply a TeleopCommand delta to a Pose                              #
# --------------------------------------------------------------------------- #

def apply_delta(pose: "Pose", cmd: TeleopCommand) -> "Pose":
    """Return a new Pose with position and rotation incremented by ``cmd``.

    Position delta is applied in the **base_link frame** (world-aligned).
    Rotation delta (axis-angle) is applied in the **EE frame** (body-fixed),
    so the device feels intuitive — pushing left on a SpaceMouse rotates the
    gripper around its own axis regardless of the current wrist orientation.

    Args:
        pose: Current end-effector pose.
        cmd:  TeleopCommand from the latest ``poll()`` call.

    Returns:
        New Pose with incremented position and orientation.
    """
    from crisp_py.utils.geometry import Pose  # late import avoids circular dep

    new_pos = pose.position + cmd.pos_delta

    if np.any(cmd.rot_delta != 0.0):
        # Body-fixed rotation: R_new = R_cur * R_delta
        delta_rot = Rotation.from_rotvec(cmd.rot_delta)
        new_rot = pose.orientation * delta_rot
    else:
        new_rot = pose.orientation

    return Pose(position=new_pos, orientation=new_rot)
