"""Stage 2 — Joint-space motion test.

Tests move_joints() by:
  1. Moving to home
  2. Incrementally rotating each joint by a small angle (+/- 0.1 rad ~ 6 deg)
  3. Returning to home after each individual joint move

This verifies that all 6 joints respond correctly to joint-space commands
and that the joint_values property accurately reflects the new configuration
after each motion.

SAFETY:
    Small motion per joint (0.1 rad ~ 6 degrees).
    Run 01_home.py first and make sure the arm is at home before this test.

Usage:
    # Real robot
    cd /home/yunfei/crisp_py
    pixi run --environment jazzy python3 examples/ridgeback/02_joint_motion.py

    # Simulation
    pixi run --environment jazzy python3 examples/ridgeback/02_joint_motion.py --sim
"""

import argparse

import numpy as np

from crisp_py.robot import RidgebackConfig, make_ridgeback_robot, get_ridgeback_urdf_path

CLEARPATH_WS = "/home/yunfei/clearpath_remote_ws"
SIM_URDF = f"{CLEARPATH_WS}/src/tum09_ridgeback/tum09_bringup/config/robot.urdf"
DELTA_RAD = 0.1        # joint displacement per test [rad] (~6 deg)
MOVE_DURATION = 3.0    # seconds per individual move
HOME_DURATION = 4.0    # seconds to return to home

JOINT_LABELS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow",
    "wrist_1",
    "wrist_2",
    "wrist_3",
]

# --------------------------------------------------------------------------- #
# Parse arguments
# --------------------------------------------------------------------------- #
parser = argparse.ArgumentParser(description="Stage 2: joint-space motion test")
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

print(f"\nMoving to home first (duration={HOME_DURATION}s)...")
robot.home(duration=HOME_DURATION)
if not robot.is_homed():
    raise RuntimeError("Failed to reach home position. Aborting test.")
print("At home.")

# --------------------------------------------------------------------------- #
# Test each joint individually
# --------------------------------------------------------------------------- #
results = []

for i, label in enumerate(JOINT_LABELS):
    print(f"\n--- Joint {i} ({label}) ---")

    home_q = list(robot.home_config)

    # Move joint i by +DELTA_RAD
    target_q = home_q.copy()
    target_q[i] += DELTA_RAD
    print(f"  Moving joint {i} by +{np.degrees(DELTA_RAD):.1f} deg ...")
    success = robot.move_joints(target_q, duration=MOVE_DURATION)

    q_after = robot.joint_values
    actual_delta = q_after[i] - home_q[i]
    error = abs(actual_delta - DELTA_RAD)
    print(f"  Commanded delta: +{np.degrees(DELTA_RAD):.2f} deg")
    print(f"  Actual delta:    +{np.degrees(actual_delta):.2f} deg")
    print(f"  Error:            {np.degrees(error):.3f} deg")
    results.append({"joint": label, "success": success, "error_deg": np.degrees(error)})

    # Return to home
    robot.home(duration=HOME_DURATION)

# --------------------------------------------------------------------------- #
# Summary
# --------------------------------------------------------------------------- #
print("\n========== Stage 2 Summary ==========")
all_passed = True
for r in results:
    status = "OK" if r["success"] and r["error_deg"] < 2.0 else "FAIL"
    if status == "FAIL":
        all_passed = False
    print(f"  {r['joint']:20s}  success={r['success']}  error={r['error_deg']:.3f} deg  [{status}]")

if all_passed:
    print("\nStage 2 complete — all joint motions passed.")
else:
    print("\nStage 2 FAILED — check warnings above.")
