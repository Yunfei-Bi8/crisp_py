"""Stage 7 — Human teleoperation data collection.

Records one or more episodes of human-controlled arm motion alongside
synchronized multi-camera RGBD data.  Designed for collecting demonstration
data for imitation learning.

Control modes
-------------
* **Keyboard** (``--input keyboard``, default):
  WASD + arrows move the EE in Cartesian space.  No extra dependencies.

* **SpaceMouse** (``--input spacemouse``):
  6-DOF analogue input from a 3Dconnexion SpaceMouse.
  Requires:  ``pip install pyspacemouse``

Key bindings (keyboard mode)
-----------------------------
  W/S    →  EE +X / -X        (forward/backward)
  A/D    →  EE -Y / +Y        (left/right)
  Q/E    →  EE +Z / -Z        (up/down)
  ↑/↓    →  pitch +/-
  ←/→    →  yaw +/-
  R/F    →  roll +/-
  O/C    →  gripper open/close
  Space  →  stop episode (save)
  X      →  discard episode
  Esc    →  quit program

Data saved per episode
----------------------
  <output_dir>/episode_NNNN/
    info.json      — metadata, camera intrinsics
    state.npz      — joints, EE pose, gripper, commanded actions, timestamps
    cam_<name>.npz — color (N,H,W,3) uint8, depth (N,H,W) float32, stamps

Prerequisites
-------------
  1. Robot bringup (or simulation) is running.
  2. Orbbec driver is running (if cameras are used):
       pixi run ros2 launch tum09_custom orbbec.launch.py
  3. Run ``01_home.py`` first to confirm the arm is at home.

Usage
-----
  cd /home/yunfei/crisp_py

  # Keyboard, real robot, single camera
  pixi run --environment jazzy python3 examples/ridgeback/07_data_collection.py

  # SpaceMouse, two cameras, five episodes
  pixi run --environment jazzy python3 examples/ridgeback/07_data_collection.py \\
      --input spacemouse --namespaces /camera_01 /camera_02 --episodes 5

  # Simulation (no cameras, keyboard)
  pixi run --environment jazzy python3 examples/ridgeback/07_data_collection.py \\
      --sim --namespaces
"""

import argparse
import time

import numpy as np

from crisp_py.camera import RgbdCameraConfig, make_rgbd_cameras
from crisp_py.data import DataCollector
from crisp_py.robot import RidgebackConfig, make_ridgeback_robot, get_ridgeback_urdf_path
from crisp_py.teleop import apply_delta

CLEARPATH_WS = "/home/yunfei/clearpath_remote_ws"
SIM_URDF = f"{CLEARPATH_WS}/src/tum09_ridgeback/tum09_bringup/config/robot.urdf"

# --------------------------------------------------------------------------- #
# Parse arguments
# --------------------------------------------------------------------------- #
parser = argparse.ArgumentParser(
    description="Stage 7: human teleoperation data collection",
    formatter_class=argparse.RawDescriptionHelpFormatter,
)
parser.add_argument("--sim", action="store_true", help="Use simulation config")
parser.add_argument(
    "--input",
    choices=["keyboard", "spacemouse"],
    default="keyboard",
    help="Teleoperation input device (default: keyboard)",
)
parser.add_argument(
    "--namespaces",
    nargs="*",
    default=["/camera"],
    metavar="NS",
    help=(
        "ROS namespaces of Orbbec cameras (default: /camera). "
        "Pass --namespaces with no arguments to disable cameras."
    ),
)
parser.add_argument(
    "--output",
    default="~/data/ridgeback_episodes",
    help="Output directory for episode data (default: ~/data/ridgeback_episodes)",
)
parser.add_argument(
    "--episodes",
    type=int,
    default=1,
    help="Maximum number of episodes to collect (default: 1)",
)
parser.add_argument(
    "--hz",
    type=float,
    default=20.0,
    help="Control loop frequency [Hz] (default: 20)",
)
parser.add_argument(
    "--pos-scale",
    type=float,
    default=0.005,
    help="EE position increment per key-press [m] (keyboard mode, default: 0.005)",
)
parser.add_argument(
    "--rot-scale",
    type=float,
    default=0.02,
    help="EE rotation increment per key-press [rad] (keyboard mode, default: 0.02)",
)
parser.add_argument(
    "--camera-timeout",
    type=float,
    default=15.0,
    help="Seconds to wait for cameras to become ready (default: 15)",
)
args = parser.parse_args()

env_str = "SIMULATION" if args.sim else "REAL ROBOT"
print(f"Mode: {env_str}  |  Input: {args.input}  |  {args.hz:.0f} Hz")
print(f"Cameras: {args.namespaces if args.namespaces else 'none'}")
print(f"Output:  {args.output}")

# --------------------------------------------------------------------------- #
# Connect to robot
# --------------------------------------------------------------------------- #
if args.sim:
    cfg = RidgebackConfig.for_simulation()
    urdf = SIM_URDF
else:
    cfg = RidgebackConfig.for_real_robot()
    urdf = get_ridgeback_urdf_path(CLEARPATH_WS)

print("\nConnecting to robot...")
robot = make_ridgeback_robot(urdf_path=urdf, config=cfg)
robot.wait_until_ready(timeout=15.0)
print("Robot ready.")

# --------------------------------------------------------------------------- #
# Connect to cameras
# --------------------------------------------------------------------------- #
cameras = []
if args.namespaces:
    print(f"\nConnecting to {len(args.namespaces)} camera(s)...")
    cam_configs = [RgbdCameraConfig.for_orbbec(ns) for ns in args.namespaces]
    cameras = make_rgbd_cameras(cam_configs)
    for cam in cameras:
        print(f"  Waiting for {cam.config.camera_name}...")
        cam.wait_until_ready(timeout=args.camera_timeout)
        print(f"  {cam.config.camera_name} ready.")

# --------------------------------------------------------------------------- #
# Build DataCollector
# --------------------------------------------------------------------------- #
collector = DataCollector(
    robot=robot,
    cameras=cameras,
    output_dir=args.output,
)

# --------------------------------------------------------------------------- #
# Build teleoperation input
# --------------------------------------------------------------------------- #
if args.input == "spacemouse":
    from crisp_py.teleop import SpaceMouseTeleopInput
    teleop_input = SpaceMouseTeleopInput(
        pos_scale=args.pos_scale * args.hz,   # scale to per-tick delta
        rot_scale=args.rot_scale * args.hz,
    )
else:
    from crisp_py.teleop import KeyboardTeleopInput
    teleop_input = KeyboardTeleopInput(
        pos_scale=args.pos_scale,
        rot_scale=args.rot_scale,
    )

# --------------------------------------------------------------------------- #
# Home the robot
# --------------------------------------------------------------------------- #
print("\nMoving to home...")
robot.home(duration=8.0)
if not robot.is_homed():
    raise RuntimeError("Failed to reach home. Aborting.")
print("At home.")

# --------------------------------------------------------------------------- #
# Activate CartesianController
# --------------------------------------------------------------------------- #
print("Activating CartesianController for teleoperation...")
if not robot.switch_to_cartesian_controller():
    raise RuntimeError(
        "Failed to activate CartesianController. "
        "Check that crisp_controllers are loaded."
    )
print("CartesianController active.")

# --------------------------------------------------------------------------- #
# Episode collection loop
# --------------------------------------------------------------------------- #
dt = 1.0 / args.hz
episodes_done = 0

with teleop_input:
    if hasattr(teleop_input, "help_text"):
        print(teleop_input.help_text())

    print(
        f"\nReady to collect. Use the input device to control the arm.\n"
        f"Press Space to save an episode, X to discard, Esc to quit.\n"
    )

    # Start with the current EE pose as the Cartesian target
    target_pose = robot.end_effector_pose

    while episodes_done < args.episodes:
        # Start a new episode
        episode_id = collector.start_episode()
        print(f"\n--- Recording episode: {episode_id} ---")

        episode_saved = False
        t_start = time.monotonic()

        while True:
            t_tick = time.monotonic()

            # Read input device
            cmd = teleop_input.poll()

            # Handle episode / program control
            if cmd.quit:
                print("\nQuit requested.")
                if collector.is_recording:
                    collector.discard_episode()
                break

            if cmd.discard_episode:
                print("Episode discarded.")
                collector.discard_episode()
                episode_saved = False
                break

            if cmd.stop_episode:
                saved_path = collector.stop_episode()
                print(
                    f"Episode saved ({collector.last_episode_steps} steps) "
                    f"→ {saved_path}"
                )
                episodes_done += 1
                episode_saved = True
                break

            # Apply delta to get new target pose
            if np.any(cmd.pos_delta != 0.0) or np.any(cmd.rot_delta != 0.0):
                target_pose = apply_delta(target_pose, cmd)

            # Gripper command
            if cmd.gripper < -0.5:
                robot.gripper_open()
            elif cmd.gripper > 0.5:
                robot.gripper_close()

            # Send Cartesian command to robot
            robot.move_cartesian_async(target_pose)

            # Record this step
            collector.record_step(action=target_pose)

            # Status every 2 seconds
            elapsed = time.monotonic() - t_start
            if int(elapsed) % 2 == 0 and int(elapsed * args.hz) % int(args.hz * 2) == 0:
                ee = robot.end_effector_pose
                print(
                    f"  t={elapsed:.0f}s  steps={collector.current_episode_steps}"
                    f"  EE={np.round(ee.position, 3)}"
                )

            # Sleep for remainder of tick
            tick_elapsed = time.monotonic() - t_tick
            sleep_time = dt - tick_elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        if cmd.quit:
            break

        if not episode_saved:
            # Offer to re-record
            print("Episode discarded. Start next episode? (press any key to continue)")

# --------------------------------------------------------------------------- #
# Cleanup
# --------------------------------------------------------------------------- #
print("\nRestoring JointTrajectoryController...")
robot.switch_to_joint_trajectory_controller()
print("JointTrajectoryController restored.")

print(f"\nCollection complete. {episodes_done} episode(s) saved to {args.output}")
