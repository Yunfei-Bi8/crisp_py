"""Stage 3 — Cartesian-space motion test.

Tests move_to() (Pinocchio IK + FollowJointTrajectory) by:
  1. Moving to home
  2. Reading the current EE pose via FK
  3. Commanding small Cartesian displacements (+/- 5 cm) along each axis
  4. Verifying that the arm reached the target by comparing FK output
  5. Returning to home after each move

This validates the complete IK pipeline:
    desired Pose -> Pinocchio IK -> joint angles -> FollowJointTrajectory

SAFETY:
    Each move is 5 cm.  Run 01_home.py first.
    Keep clear of the workspace.

Usage:
    # Real robot
    cd /home/yunfei/crisp_py
    pixi run --environment jazzy python3 examples/ridgeback/03_cartesian_motion.py

    # Simulation
    pixi run --environment jazzy python3 examples/ridgeback/03_cartesian_motion.py --sim
"""

import argparse

import numpy as np

from crisp_py.robot import RidgebackConfig, make_ridgeback_robot, get_ridgeback_urdf_path

CLEARPATH_WS = "/home/yunfei/clearpath_remote_ws"
SIM_URDF = f"{CLEARPATH_WS}/src/tum09_ridgeback/tum09_bringup/config/robot.urdf"
DELTA_M = 0.05         # Cartesian displacement per test [m] (5 cm)
MOVE_DURATION = 4.0    # seconds per move
HOME_DURATION = 5.0    # seconds to return to home
POS_TOLERANCE_M = 0.01 # acceptable position error [m] (1 cm)

AXES = [
    (0, "+X"),
    (0, "-X"),
    (1, "+Y"),
    (1, "-Y"),
    (2, "+Z"),
    (2, "-Z"),
]

# --------------------------------------------------------------------------- #
# Parse arguments
# --------------------------------------------------------------------------- #
parser = argparse.ArgumentParser(description="Stage 3: Cartesian motion test")
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
# Connect and home
# --------------------------------------------------------------------------- #
print("Loading URDF...")
robot = make_ridgeback_robot(urdf_path=urdf, config=cfg)

print("Waiting for robot...")
robot.wait_until_ready(timeout=15.0)

print(f"\nMoving to home (duration={HOME_DURATION}s)...")
robot.home(duration=HOME_DURATION)
if not robot.is_homed():
    raise RuntimeError("Failed to reach home. Aborting.")

home_pose = robot.end_effector_pose
print(f"Home EE position [m]: {np.round(home_pose.position, 4)}")

# --------------------------------------------------------------------------- #
# Test each Cartesian axis
# --------------------------------------------------------------------------- #
results = []

for axis_idx, label in AXES:
    print(f"\n--- Cartesian move {label} by {DELTA_M*100:.0f} cm ---")

    sign = 1.0 if label[0] == "+" else -1.0
    target = home_pose.copy()
    target.position[axis_idx] += sign * DELTA_M

    print(f"  Target position [m]: {np.round(target.position, 4)}")
    success = robot.move_to(target, duration=MOVE_DURATION)

    actual_pose = robot.end_effector_pose
    pos_error = np.linalg.norm(actual_pose.position - target.position)
    print(f"  move_to() returned:   {success}")
    print(f"  Actual position [m]:  {np.round(actual_pose.position, 4)}")
    print(f"  Position error  [m]:  {pos_error:.4f}")

    passed = success and pos_error < POS_TOLERANCE_M
    results.append({
        "axis": label,
        "success": success,
        "pos_error_m": pos_error,
        "passed": passed,
    })

    if not passed:
        print(f"  WARNING: test did not pass (error={pos_error:.4f} m, tolerance={POS_TOLERANCE_M} m)")

    # Return to home
    print(f"  Returning to home...")
    robot.home(duration=HOME_DURATION)

# --------------------------------------------------------------------------- #
# Summary
# --------------------------------------------------------------------------- #
print("\n========== Stage 3 Summary ==========")
all_passed = True
for r in results:
    status = "OK" if r["passed"] else "FAIL"
    if not r["passed"]:
        all_passed = False
    print(f"  {r['axis']}  success={r['success']}  pos_error={r['pos_error_m']*100:.2f} cm  [{status}]")

if all_passed:
    print("\nStage 3 complete — Cartesian IK pipeline is working correctly.")
else:
    print("\nStage 3 FAILED — check IK convergence warnings above.")
