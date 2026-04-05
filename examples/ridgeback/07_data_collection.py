"""Stage 7 — Teleoperation data collection.

Drives the arm along a small circular trajectory while recording:
  - Robot state (joint positions/velocities, EE pose, gripper position)
  - RGBD frames from one or more Orbbec cameras
  - Commanded actions (EE pose targets)

Data is saved per episode to a directory on disk in .npz + .json format:

    <output_dir>/
        episode_0000/
            info.json        — metadata, camera intrinsics, timing
            state.npz        — joints, velocities, EE pose, gripper, actions
            cam_camera.npz   — color (N,H,W,3), depth (N,H,W), timestamps

This script uses the CartesianController for smooth streaming control.
The controller is activated/deactivated programmatically.

Prerequisites:
    1. Robot bringup (or simulation) is running.
    2. Orbbec driver is running:
         pixi run ros2 launch tum09_custom orbbec.launch.py
    3. Run 01_home.py first to confirm the arm is at a safe home pose.

Usage:
    cd /home/yunfei/crisp_py

    # Real robot — single camera, one episode
    pixi run --environment jazzy python3 examples/ridgeback/07_data_collection.py

    # Real robot — two cameras, five episodes, custom output dir
    pixi run --environment jazzy python3 examples/ridgeback/07_data_collection.py \\
        --namespaces /camera_01 /camera_02 \\
        --episodes 5 \\
        --output ~/data/ridgeback_episodes

    # Simulation (no cameras needed, no gripper)
    pixi run --environment jazzy python3 examples/ridgeback/07_data_collection.py \\
        --sim --namespaces
"""

import argparse
import time

import numpy as np

from crisp_py.camera import RgbdCameraConfig, make_rgbd_cameras
from crisp_py.data import DataCollector
from crisp_py.robot import RidgebackConfig, make_ridgeback_robot, get_ridgeback_urdf_path

CLEARPATH_WS = "/home/yunfei/clearpath_remote_ws"
SIM_URDF = f"{CLEARPATH_WS}/src/tum09_ridgeback/tum09_bringup/config/robot.urdf"

# --------------------------------------------------------------------------- #
# Parse arguments
# --------------------------------------------------------------------------- #
parser = argparse.ArgumentParser(description="Stage 7: teleoperation data collection")
parser.add_argument("--sim", action="store_true", help="Use simulation config")
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
    help="Number of episodes to collect (default: 1)",
)
parser.add_argument(
    "--hz",
    type=float,
    default=20.0,
    help="Control loop frequency [Hz] (default: 20)",
)
parser.add_argument(
    "--duration",
    type=float,
    default=10.0,
    help="Duration of each episode [s] (default: 10)",
)
parser.add_argument(
    "--radius",
    type=float,
    default=0.03,
    help="Circle radius for test trajectory [m] (default: 0.03)",
)
parser.add_argument(
    "--camera-timeout",
    type=float,
    default=15.0,
    help="Seconds to wait for cameras to become ready (default: 15)",
)
args = parser.parse_args()

env_str = "SIMULATION" if args.sim else "REAL ROBOT"
print(f"Mode: {env_str}")
print(f"Cameras: {args.namespaces if args.namespaces else 'none'}")
print(f"Output:  {args.output}")
print(f"Episodes: {args.episodes}  |  {args.hz:.0f} Hz  |  {args.duration}s each")

# --------------------------------------------------------------------------- #
# Connect to robot
# --------------------------------------------------------------------------- #
if args.sim:
    cfg = RidgebackConfig.for_simulation()
    urdf = SIM_URDF
else:
    cfg = RidgebackConfig.for_real_robot()
    urdf = get_ridgeback_urdf_path(CLEARPATH_WS)

print("\nLoading URDF and connecting to robot...")
robot = make_ridgeback_robot(urdf_path=urdf, config=cfg)
robot.wait_until_ready(timeout=15.0)
print("Robot ready.")

# --------------------------------------------------------------------------- #
# Connect to cameras (optional)
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
print(f"\nDataCollector: {collector}")

# --------------------------------------------------------------------------- #
# Home the robot
# --------------------------------------------------------------------------- #
print("\nMoving to home...")
robot.home(duration=8.0)
if not robot.is_homed():
    raise RuntimeError("Failed to reach home. Aborting.")
print("At home.")

# --------------------------------------------------------------------------- #
# Episode loop
# --------------------------------------------------------------------------- #
dt = 1.0 / args.hz
circle_freq = 0.2   # one full circle every 5 s

for ep_idx in range(args.episodes):
    print(f"\n{'='*55}")
    print(f"Episode {ep_idx + 1} / {args.episodes}")
    print(f"{'='*55}")

    # Record the current EE pose as the circle centre
    centre_pose = robot.end_effector_pose
    centre_y = centre_pose.position[1]
    centre_z = centre_pose.position[2]
    print(f"Circle centre (EE at home): {np.round(centre_pose.position, 4)}")

    # Activate CartesianController for smooth streaming
    print("Activating CartesianController...")
    if not robot.switch_to_cartesian_controller():
        raise RuntimeError(
            "Failed to activate CartesianController. "
            "Check that crisp_controllers are loaded in the bringup."
        )

    # Start recording
    episode_id = collector.start_episode()
    print(f"Recording episode: {episode_id}")

    n_steps = int(args.duration * args.hz)

    try:
        for step in range(n_steps):
            t = step * dt
            angle = 2.0 * np.pi * circle_freq * t

            # Compute target pose (circle in YZ plane)
            target = centre_pose.copy()
            target.position[1] = centre_y + args.radius * np.cos(angle)
            target.position[2] = centre_z + args.radius * np.sin(angle)

            # Send command to robot
            robot.move_cartesian_async(target)

            # Record this step (state + action + camera frames)
            collector.record_step(action=target)

            # Throttle to loop frequency
            time.sleep(dt)

        print(
            f"Episode done: {collector.current_episode_steps} steps recorded."
        )

    except KeyboardInterrupt:
        print("\nEpisode aborted by user.")
        collector.discard_episode()
        robot.switch_to_joint_trajectory_controller()
        print("JointTrajectoryController restored. Exiting.")
        break

    finally:
        # Always restore joint trajectory controller
        robot.switch_to_joint_trajectory_controller()
        print("JointTrajectoryController restored.")

    # Save episode to disk
    if collector.is_recording:
        saved_path = collector.stop_episode()
        print(f"Saved {collector.last_episode_steps} steps → {saved_path}")

    # Return to home between episodes
    if ep_idx < args.episodes - 1:
        print("Returning to home for next episode...")
        robot.home(duration=6.0)

# --------------------------------------------------------------------------- #
# Summary
# --------------------------------------------------------------------------- #
print(f"\n{'='*55}")
print(f"Collection complete.")
print(f"Output directory: {args.output}")
