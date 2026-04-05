"""Synchronized data collection for robot teleoperation episodes.

Records robot state and multi-camera RGBD frames during teleoperation.
Each episode is saved to a self-contained directory on disk.

Storage layout::

    <output_dir>/
        <episode_id>/
            info.json          # metadata: robot config, camera intrinsics, timing
            state.npz          # robot state arrays (joints, EE pose, gripper, action)
            cam_<name>.npz     # per camera: color (N,H,W,3), depth (N,H,W), stamps

Typical usage::

    from crisp_py.data import DataCollector

    collector = DataCollector(robot, cameras, output_dir="~/data/episodes")
    collector.start_episode()

    robot.switch_to_cartesian_controller()
    try:
        while running:
            command = compute_command(...)
            robot.move_cartesian_async(command)
            collector.record_step(action=command)
            time.sleep(0.02)   # 50 Hz
    finally:
        robot.switch_to_joint_trajectory_controller()

    saved_path = collector.stop_episode()
    print(f"Saved {collector.last_episode_steps} steps to {saved_path}")
"""

import json
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from crisp_py.camera.rgbd_camera import RgbdCamera
    from crisp_py.robot.ridgeback_robot import RidgebackRobot
    from crisp_py.utils.geometry import Pose


# --------------------------------------------------------------------------- #
#  Per-step data container                                                     #
# --------------------------------------------------------------------------- #

@dataclass
class EpisodeStep:
    """All data captured at one time step.

    Attributes:
        wall_time_ns:         Wall-clock time at the moment of recording [ns]
        robot_stamp_ns:       ROS timestamp of the joint state message [ns]
        joint_positions:      (6,) arm joint angles [rad]
        joint_velocities:     (6,) arm joint velocities [rad/s]
        gripper_position:     Robotiq knuckle angle [rad]; NaN if unavailable
        ee_position:          (3,) EE position in base_link [m]
        ee_quaternion:        (4,) EE orientation as [x,y,z,w]
        action_ee_position:   (3,) commanded EE position [m]; zeros if no action
        action_ee_quaternion: (4,) commanded orientation [x,y,z,w]; identity if none
        colors:               List of (H,W,3) uint8 RGB arrays, one per camera
        depths:               List of (H,W) float32 depth arrays [m], one per camera
        color_stamps_ns:      ROS timestamps of each color frame [ns]
        depth_stamps_ns:      ROS timestamps of each depth frame [ns]
    """

    wall_time_ns: int
    robot_stamp_ns: int
    joint_positions: np.ndarray        # (6,)
    joint_velocities: np.ndarray       # (6,)
    gripper_position: float            # scalar, NaN if unknown
    ee_position: np.ndarray            # (3,)
    ee_quaternion: np.ndarray          # (4,) [x,y,z,w]
    action_ee_position: np.ndarray     # (3,)
    action_ee_quaternion: np.ndarray   # (4,) [x,y,z,w]
    colors: List[np.ndarray]           # one per camera
    depths: List[np.ndarray]           # one per camera
    color_stamps_ns: List[int]
    depth_stamps_ns: List[int]


# --------------------------------------------------------------------------- #
#  DataCollector                                                               #
# --------------------------------------------------------------------------- #

class DataCollector:
    """Records synchronized robot state and camera RGBD data during teleoperation.

    Call ``start_episode()`` before the teleoperation loop, ``record_step()``
    inside the loop (once per control tick), and ``stop_episode()`` when done.
    All data is buffered in memory and written to disk atomically on
    ``stop_episode()``.

    Thread safety: ``record_step()`` is protected by an internal lock so it
    can be called from any thread.

    Memory note: At 50 Hz with two 640×480 cameras, one 60-second episode
    uses roughly 1–2 GB of RAM.  For longer sessions, reduce the frame rate
    or use a lower camera resolution.

    Args:
        robot:          Connected ``RidgebackRobot`` instance.
        cameras:        List of ``RgbdCamera`` instances (can be empty).
        output_dir:     Root directory where episode folders are written.
                        Created automatically if it does not exist.
        episode_prefix: Prefix for auto-generated episode folder names
                        (e.g. ``"episode"`` → ``"episode_0000"``).
    """

    def __init__(
        self,
        robot: "RidgebackRobot",
        cameras: List["RgbdCamera"],
        output_dir: "str | Path",
        episode_prefix: str = "episode",
    ) -> None:
        self._robot = robot
        self._cameras = cameras
        self._output_dir = Path(output_dir).expanduser().resolve()
        self._episode_prefix = episode_prefix

        self._lock = threading.Lock()
        self._steps: List[EpisodeStep] = []
        self._episode_id: Optional[str] = None
        self._start_time_utc: Optional[datetime] = None
        self._is_recording: bool = False
        self._last_episode_steps: int = 0

    # ------------------------------------------------------------------ #
    #  Public: episode lifecycle                                           #
    # ------------------------------------------------------------------ #

    def start_episode(self, episode_id: Optional[str] = None) -> str:
        """Begin recording a new episode.

        Args:
            episode_id: Explicit episode identifier, used as the folder name.
                        If None, an ID is generated automatically by scanning
                        the output directory for existing episodes
                        (``episode_0000``, ``episode_0001``, …).

        Returns:
            The episode_id that will be used.

        Raises:
            RuntimeError: If an episode is already in progress.
        """
        with self._lock:
            if self._is_recording:
                raise RuntimeError(
                    "An episode is already in progress. "
                    "Call stop_episode() or discard_episode() first."
                )
            if episode_id is None:
                episode_id = self._next_episode_id()
            self._episode_id = episode_id
            self._steps = []
            self._start_time_utc = datetime.now(timezone.utc)
            self._is_recording = True

        return episode_id

    def record_step(self, action: "Optional[Pose]" = None) -> None:
        """Capture one synchronized step of robot state and camera frames.

        Call this once per control tick, immediately after sending the
        command to the robot.

        Args:
            action: The pose command just sent to the robot.  Stored as
                    ``(action_ee_position, action_ee_quaternion)`` in the
                    episode data for use in supervised imitation learning.
                    Pass None if no command was issued this step.

        Raises:
            RuntimeError: If ``start_episode()`` has not been called.
        """
        if not self._is_recording:
            raise RuntimeError(
                "Not recording. Call start_episode() first."
            )

        wall_ns = time.time_ns()

        # --- Robot state ---
        try:
            joints = self._robot.joint_values          # (6,) — acquires lock
            velocities = self._robot.joint_velocities  # (6,)
            robot_stamp = self._robot.joint_state_stamp_ns
        except RuntimeError:
            joints = np.zeros(6, dtype=np.float64)
            velocities = np.zeros(6, dtype=np.float64)
            robot_stamp = 0

        gripper = self._robot.gripper_position
        gripper_val = float(gripper) if gripper is not None else float("nan")

        try:
            ee_pose = self._robot.end_effector_pose
            ee_pos = ee_pose.position.copy()
            ee_quat = ee_pose.orientation.as_quat()    # [x,y,z,w]
        except RuntimeError:
            ee_pos = np.zeros(3, dtype=np.float64)
            ee_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)

        # --- Action ---
        if action is not None:
            act_pos = action.position.copy()
            act_quat = action.orientation.as_quat()    # [x,y,z,w]
        else:
            act_pos = np.zeros(3, dtype=np.float64)
            act_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)

        # --- Camera frames ---
        colors: List[np.ndarray] = []
        depths: List[np.ndarray] = []
        color_stamps: List[int] = []
        depth_stamps: List[int] = []

        for cam in self._cameras:
            try:
                color, depth, color_stamp = cam.current_rgbd(max_stamp_diff_ms=100.0)
                depth_stamp = cam.depth_stamp_ns
            except RuntimeError:
                # Camera not ready or timestamp mismatch — use latest available
                try:
                    color = cam.current_color
                    color_stamp = cam.color_stamp_ns
                except RuntimeError:
                    intr = cam.intrinsics
                    color = np.zeros((intr.height, intr.width, 3), dtype=np.uint8)
                    color_stamp = 0
                try:
                    depth = cam.current_depth
                    depth_stamp = cam.depth_stamp_ns
                except RuntimeError:
                    intr = cam.intrinsics
                    depth = np.full(
                        (intr.height, intr.width), np.nan, dtype=np.float32
                    )
                    depth_stamp = 0

            colors.append(color)
            depths.append(depth)
            color_stamps.append(color_stamp)
            depth_stamps.append(depth_stamp)

        step = EpisodeStep(
            wall_time_ns=wall_ns,
            robot_stamp_ns=robot_stamp,
            joint_positions=joints,
            joint_velocities=velocities,
            gripper_position=gripper_val,
            ee_position=ee_pos,
            ee_quaternion=ee_quat,
            action_ee_position=act_pos,
            action_ee_quaternion=act_quat,
            colors=colors,
            depths=depths,
            color_stamps_ns=color_stamps,
            depth_stamps_ns=depth_stamps,
        )

        with self._lock:
            self._steps.append(step)

    def stop_episode(self) -> Path:
        """Stop recording and save the episode to disk.

        Stacks all buffered steps into numpy arrays and writes them to the
        episode directory.  The in-memory buffer is cleared after saving.

        Returns:
            Path to the saved episode directory.

        Raises:
            RuntimeError: If no episode is in progress.
        """
        with self._lock:
            if not self._is_recording:
                raise RuntimeError(
                    "No episode in progress. Call start_episode() first."
                )
            steps = list(self._steps)
            episode_id = self._episode_id
            start_time = self._start_time_utc
            self._is_recording = False
            self._steps = []
            self._last_episode_steps = len(steps)

        if not steps:
            raise RuntimeError("Episode contains no recorded steps.")

        episode_dir = self._output_dir / episode_id
        episode_dir.mkdir(parents=True, exist_ok=True)

        self._save_state(steps, episode_dir)
        self._save_cameras(steps, episode_dir)
        self._save_info(steps, episode_id, start_time, episode_dir)

        return episode_dir

    def discard_episode(self) -> None:
        """Discard the current episode without saving.

        Raises:
            RuntimeError: If no episode is in progress.
        """
        with self._lock:
            if not self._is_recording:
                raise RuntimeError("No episode in progress.")
            self._is_recording = False
            self._steps = []
            self._episode_id = None

    # ------------------------------------------------------------------ #
    #  Public: status properties                                           #
    # ------------------------------------------------------------------ #

    @property
    def is_recording(self) -> bool:
        """True if an episode is currently being recorded."""
        return self._is_recording

    @property
    def current_episode_steps(self) -> int:
        """Number of steps recorded so far in the current episode."""
        with self._lock:
            return len(self._steps)

    @property
    def last_episode_steps(self) -> int:
        """Number of steps in the most recently completed episode."""
        return self._last_episode_steps

    # ------------------------------------------------------------------ #
    #  Internal: saving                                                    #
    # ------------------------------------------------------------------ #

    def _save_state(self, steps: List[EpisodeStep], episode_dir: Path) -> None:
        """Stack robot state fields and save to ``state.npz``."""
        np.savez_compressed(
            episode_dir / "state.npz",
            joint_positions=np.stack([s.joint_positions for s in steps]),       # (N,6)
            joint_velocities=np.stack([s.joint_velocities for s in steps]),     # (N,6)
            gripper_position=np.array([s.gripper_position for s in steps]),     # (N,)
            ee_position=np.stack([s.ee_position for s in steps]),               # (N,3)
            ee_quaternion=np.stack([s.ee_quaternion for s in steps]),           # (N,4)
            action_ee_position=np.stack([s.action_ee_position for s in steps]), # (N,3)
            action_ee_quaternion=np.stack([s.action_ee_quaternion for s in steps]), # (N,4)
            robot_stamp_ns=np.array([s.robot_stamp_ns for s in steps], dtype=np.int64),
            wall_time_ns=np.array([s.wall_time_ns for s in steps], dtype=np.int64),
        )

    def _save_cameras(
        self, steps: List[EpisodeStep], episode_dir: Path
    ) -> None:
        """Stack camera frames and save one ``cam_<name>.npz`` per camera."""
        for cam_idx, cam in enumerate(self._cameras):
            cam_name = cam.config.camera_name
            np.savez_compressed(
                episode_dir / f"cam_{cam_name}.npz",
                color=np.stack([s.colors[cam_idx] for s in steps]),          # (N,H,W,3)
                depth=np.stack([s.depths[cam_idx] for s in steps]),           # (N,H,W)
                color_stamp_ns=np.array(
                    [s.color_stamps_ns[cam_idx] for s in steps], dtype=np.int64
                ),
                depth_stamp_ns=np.array(
                    [s.depth_stamps_ns[cam_idx] for s in steps], dtype=np.int64
                ),
            )

    def _save_info(
        self,
        steps: List[EpisodeStep],
        episode_id: str,
        start_time: datetime,
        episode_dir: Path,
    ) -> None:
        """Write episode metadata and camera intrinsics to ``info.json``."""
        cameras_info: Dict = {}
        for cam in self._cameras:
            try:
                intr = cam.intrinsics
                cameras_info[cam.config.camera_name] = {
                    "fx": intr.fx,
                    "fy": intr.fy,
                    "cx": intr.cx,
                    "cy": intr.cy,
                    "width": intr.width,
                    "height": intr.height,
                    "D": intr.D.tolist(),
                    "K": intr.K.tolist(),
                    "color_topic": cam.config.color_topic,
                    "depth_topic": cam.config.depth_topic,
                }
            except RuntimeError:
                cameras_info[cam.config.camera_name] = {"error": "intrinsics not available"}

        cfg = self._robot._cfg
        info = {
            "episode_id": episode_id,
            "start_time_utc": start_time.isoformat(),
            "n_steps": len(steps),
            "duration_s": round(
                (steps[-1].wall_time_ns - steps[0].wall_time_ns) / 1e9, 3
            ) if len(steps) > 1 else 0.0,
            "robot": {
                "arm_joint_names": list(cfg.arm_joint_names),
                "ee_frame": cfg.ee_frame,
                "base_frame": cfg.base_frame,
            },
            "cameras": cameras_info,
        }
        with open(episode_dir / "info.json", "w") as f:
            json.dump(info, f, indent=2)

    # ------------------------------------------------------------------ #
    #  Internal: episode ID generation                                     #
    # ------------------------------------------------------------------ #

    def _next_episode_id(self) -> str:
        """Return the next episode ID by scanning the output directory."""
        self._output_dir.mkdir(parents=True, exist_ok=True)
        existing = sorted(self._output_dir.glob(f"{self._episode_prefix}_*"))
        if not existing:
            return f"{self._episode_prefix}_0000"
        last = existing[-1].name
        try:
            idx = int(last.split("_")[-1]) + 1
        except ValueError:
            idx = len(existing)
        return f"{self._episode_prefix}_{idx:04d}"

    def __repr__(self) -> str:
        status = (
            f"recording (step {self.current_episode_steps})"
            if self._is_recording
            else "idle"
        )
        return (
            f"DataCollector("
            f"output_dir={str(self._output_dir)!r}, "
            f"cameras={len(self._cameras)}, "
            f"status={status!r})"
        )
