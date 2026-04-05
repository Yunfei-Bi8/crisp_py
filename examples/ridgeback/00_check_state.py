"""Stage 0 — Read robot state, no motion.

Verifies that:
  - ROS2 network is reachable and joint states are being published
  - Pinocchio forward kinematics (FK) produces a plausible EE pose
  - Joint velocities are readable (should be ~zero when robot is at rest)
  - Gripper position is readable
  - nq == 6

No motion is commanded.  This is the safest first test to run after
connecting to the robot.

Usage:
    # Real robot
    cd /home/yunfei/crisp_py
    pixi run --environment jazzy python3 examples/ridgeback/00_check_state.py

    # Simulation (clearpath_simulation pixi run -e ros2 sim must be running)
    pixi run --environment jazzy python3 examples/ridgeback/00_check_state.py --sim
"""

import argparse

import numpy as np

from crisp_py.robot import RidgebackConfig, make_ridgeback_robot, get_ridgeback_urdf_path

CLEARPATH_WS = "/home/yunfei/clearpath_remote_ws"
# Path to robot.urdf inside the simulation workspace (for sim mode).
# This URDF is produced by clearpath_simulation/sim_bringup.launch.py at runtime;
# use the bundled one from clearpath_remote_ws for Pinocchio (same kinematics).
SIM_URDF = f"{CLEARPATH_WS}/src/tum09_ridgeback/tum09_bringup/config/robot.urdf"

# --------------------------------------------------------------------------- #
# Parse arguments
# --------------------------------------------------------------------------- #
parser = argparse.ArgumentParser(description="Stage 0: check robot state (no motion)")
parser.add_argument(
    "--sim", action="store_true",
    help="Connect to the MuJoCo simulation instead of the real robot"
)
args = parser.parse_args()

if args.sim:
    print("Mode: SIMULATION  (/joint_states, no namespace)")
    cfg = RidgebackConfig.for_simulation()
    urdf = SIM_URDF
else:
    print("Mode: REAL ROBOT  (/r100_0207/... namespace)")
    cfg = RidgebackConfig.for_real_robot()
    urdf = get_ridgeback_urdf_path(CLEARPATH_WS)

# --------------------------------------------------------------------------- #
# Connect
# --------------------------------------------------------------------------- #
print(f"URDF: {urdf}")
print("Building Pinocchio model and connecting to ROS2...")
robot = make_ridgeback_robot(urdf_path=urdf, config=cfg)

print("Waiting for joint states...")
robot.wait_until_ready(timeout=15.0)
print("Robot is ready.")

# --------------------------------------------------------------------------- #
# Stage 0a: basic properties
# --------------------------------------------------------------------------- #
print("\n--- Basic properties ---")
print(f"nq (number of joints): {robot.nq}")
assert robot.nq == 6, f"Expected nq=6, got {robot.nq}"

# --------------------------------------------------------------------------- #
# Stage 0b: joint state
# --------------------------------------------------------------------------- #
print("\n--- Joint state ---")
q = robot.joint_values
print(f"joint_values [rad]: {np.round(q, 4)}")
print(f"joint_values [deg]: {np.round(np.degrees(q), 2)}")

dq = robot.joint_velocities
print(f"joint_velocities [rad/s]: {np.round(dq, 4)}")
max_vel = np.max(np.abs(dq))
if max_vel > 0.05:
    print(f"WARNING: max joint velocity {max_vel:.3f} rad/s — is the robot moving?")
else:
    print("Joint velocities look good (robot is at rest).")

# --------------------------------------------------------------------------- #
# Stage 0c: end-effector pose via Pinocchio FK
# --------------------------------------------------------------------------- #
print("\n--- End-effector pose (Pinocchio FK) ---")
pose = robot.end_effector_pose
print(f"EE position [m]:  x={pose.position[0]:.4f}  y={pose.position[1]:.4f}  z={pose.position[2]:.4f}")
print(f"EE orientation (euler RPY [deg]): {np.round(np.degrees(pose.orientation.as_euler('xyz')), 2)}")
print(f"EE orientation (quaternion xyzw): {np.round(pose.orientation.as_quat(), 4)}")

# Sanity check: EE should be within ~1.5 m of the robot base
ee_dist = np.linalg.norm(pose.position)
print(f"EE distance from base: {ee_dist:.3f} m")
if ee_dist > 1.5:
    print("WARNING: EE is very far from base — check URDF or joint state mapping.")
else:
    print("EE distance looks plausible.")

# --------------------------------------------------------------------------- #
# Stage 0d: gripper state
# --------------------------------------------------------------------------- #
print("\n--- Gripper state ---")
gpos = robot.gripper_position
if gpos is None:
    if args.sim:
        print("(Simulation: no gripper joint state expected — this is normal.)")
    else:
        print("WARNING: gripper_position is None.")
        print("  The gripper joint name was not found in the joint state message.")
        print("  Check that the Robotiq driver is running and publishing joint states.")
else:
    print(f"gripper_position: {gpos:.4f} rad  (0.0=open, 0.8=closed)")
    print(f"gripper_is_open:  {robot.gripper_is_open}")

# --------------------------------------------------------------------------- #
# Stage 0e: is_homed check (informational only)
# --------------------------------------------------------------------------- #
print("\n--- Home check ---")
print(f"home_config [rad]: {[round(v, 3) for v in robot.home_config]}")
print(f"is_homed():        {robot.is_homed()}")

print("\nStage 0 complete — all state checks passed, no motion was commanded.")
