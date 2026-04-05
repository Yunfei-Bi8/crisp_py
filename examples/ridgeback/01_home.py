"""Stage 1 — Move to home configuration.

Sends the arm to the predefined home joint configuration:
    [1.57, -1.57, 1.57, -1.57, -1.57, 0.0] rad

This is the simplest motion test.  Run this before any Cartesian or
teleoperation tests to ensure the arm starts from a known, safe pose.

SAFETY:
    Make sure there are no obstacles around the robot before running.
    The arm may make a large movement to reach the home pose.

Usage:
    # Real robot
    cd /home/yunfei/crisp_py
    pixi run --environment jazzy python3 examples/ridgeback/01_home.py

    # Simulation
    pixi run --environment jazzy python3 examples/ridgeback/01_home.py --sim
"""

import argparse

import numpy as np

from crisp_py.robot import RidgebackConfig, make_ridgeback_robot, get_ridgeback_urdf_path

CLEARPATH_WS = "/home/yunfei/clearpath_remote_ws"
SIM_URDF = f"{CLEARPATH_WS}/src/tum09_ridgeback/tum09_bringup/config/robot.urdf"
HOME_DURATION = 8.0   # seconds — increase if robot is far from home

# --------------------------------------------------------------------------- #
# Parse arguments
# --------------------------------------------------------------------------- #
parser = argparse.ArgumentParser(description="Stage 1: move arm to home pose")
parser.add_argument("--sim", action="store_true", help="Use simulation config")
args = parser.parse_args()

if args.sim:
    print("Mode: SIMULATION")
    cfg = RidgebackConfig.for_simulation()
    urdf = SIM_URDF
else:
    print("Mode: REAL ROBOT")
    cfg = RidgebackConfig.for_real_robot()
    urdf = get_ridgeback_urdf_path(CLEARPATH_WS)

# --------------------------------------------------------------------------- #
# Connect
# --------------------------------------------------------------------------- #
print("Loading URDF...")
robot = make_ridgeback_robot(urdf_path=urdf, config=cfg)

print("Waiting for robot...")
robot.wait_until_ready(timeout=15.0)

# --------------------------------------------------------------------------- #
# Show current state
# --------------------------------------------------------------------------- #
print("\n--- Current state ---")
q_before = robot.joint_values
pose_before = robot.end_effector_pose
print(f"Joint values [rad]: {np.round(q_before, 3)}")
print(f"EE position   [m]:  {np.round(pose_before.position, 3)}")
print(f"is_homed:           {robot.is_homed()}")

# --------------------------------------------------------------------------- #
# Move to home
# --------------------------------------------------------------------------- #
home_cfg = robot.home_config
print(f"\nMoving to home in {HOME_DURATION}s ...")
print(f"Target joints [rad]: {[round(v, 3) for v in home_cfg]}")
success = robot.home(duration=HOME_DURATION)

# --------------------------------------------------------------------------- #
# Verify
# --------------------------------------------------------------------------- #
print("\n--- State after homing ---")
q_after = robot.joint_values
pose_after = robot.end_effector_pose
print(f"home() returned:    {success}")
print(f"Joint values [rad]: {np.round(q_after, 3)}")
print(f"EE position   [m]:  {np.round(pose_after.position, 3)}")
print(f"is_homed():         {robot.is_homed()}")

joint_error = np.abs(q_after - np.array(home_cfg))
print(f"Max joint error:    {np.max(joint_error):.4f} rad ({np.degrees(np.max(joint_error)):.2f} deg)")

if not success:
    print("ERROR: home() returned False — trajectory was rejected or failed.")
elif not robot.is_homed():
    print(f"WARNING: is_homed() is False. Max joint error: {np.max(joint_error):.4f} rad.")
else:
    print("Stage 1 complete — arm is at home position.")
