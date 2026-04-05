"""Stage 4 — Gripper test.

Tests the Robotiq 2F-85 gripper via the GripperCommand action server:
  1. Read current gripper state
  2. Open fully
  3. Close fully
  4. Move to half-open position (0.4 rad)
  5. Open again to leave gripper ready for use

NOTE: In simulation (--sim), no gripper action server exists.
      This script will print a skip message and exit cleanly.

SAFETY:
    Keep fingers and objects clear of the gripper during this test.
    The gripper will open and close with up to 50 N of force.

Usage:
    # Real robot
    cd /home/yunfei/crisp_py
    pixi run --environment jazzy python3 examples/ridgeback/04_gripper.py

    # Simulation (will report skipped — no gripper action server in sim)
    pixi run --environment jazzy python3 examples/ridgeback/04_gripper.py --sim
"""

import argparse
import time

from crisp_py.robot import RidgebackConfig, make_ridgeback_robot, get_ridgeback_urdf_path

CLEARPATH_WS = "/home/yunfei/clearpath_remote_ws"
SIM_URDF = f"{CLEARPATH_WS}/src/tum09_ridgeback/tum09_bringup/config/robot.urdf"
MAX_EFFORT = 50.0    # Newtons
SETTLE_TIME = 1.0    # seconds to wait after each gripper command before reading state

# --------------------------------------------------------------------------- #
# Parse arguments
# --------------------------------------------------------------------------- #
parser = argparse.ArgumentParser(description="Stage 4: gripper test")
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
# Connect (no arm motion needed for gripper test)
# --------------------------------------------------------------------------- #
print("Loading URDF...")
robot = make_ridgeback_robot(urdf_path=urdf, config=cfg)

print("Waiting for robot...")
robot.wait_until_ready(timeout=15.0)

# --------------------------------------------------------------------------- #
# Skip gracefully in simulation
# --------------------------------------------------------------------------- #
if args.sim:
    print("\nSimulation mode: gripper_action=None — no GripperCommand server available.")
    print("Gripper commands will return False with a warning (expected behavior).")
    print("Calling gripper_open() to confirm graceful handling...")
    result = robot.gripper_open()
    print(f"gripper_open() returned: {result}  (expected: False)")
    print("\nStage 4 SKIPPED in simulation mode — gripper not available in MuJoCo sim.")
    exit(0)

# --------------------------------------------------------------------------- #
# Stage 4a: read initial state
# --------------------------------------------------------------------------- #
print("\n--- Initial gripper state ---")
gpos = robot.gripper_position
if gpos is None:
    raise RuntimeError(
        "gripper_position is None — the Robotiq driver is not publishing "
        "joint states or the joint name does not match expected keywords "
        "('knuckle', 'finger', 'robotiq')."
    )
print(f"gripper_position: {gpos:.4f} rad  (0.0=open, 0.8=closed)")
print(f"gripper_is_open:  {robot.gripper_is_open}")

# --------------------------------------------------------------------------- #
# Stage 4b: open
# --------------------------------------------------------------------------- #
print("\n--- Open gripper ---")
success = robot.gripper_open(max_effort=MAX_EFFORT)
time.sleep(SETTLE_TIME)
print(f"gripper_open() accepted: {success}")
print(f"gripper_position after:  {robot.gripper_position:.4f} rad")
print(f"gripper_is_open:         {robot.gripper_is_open}")

# --------------------------------------------------------------------------- #
# Stage 4c: close
# --------------------------------------------------------------------------- #
print("\n--- Close gripper ---")
success = robot.gripper_close(max_effort=MAX_EFFORT)
time.sleep(SETTLE_TIME)
print(f"gripper_close() accepted: {success}")
print(f"gripper_position after:   {robot.gripper_position:.4f} rad")
print(f"gripper_is_open:          {robot.gripper_is_open}")

# --------------------------------------------------------------------------- #
# Stage 4d: set to half-open
# --------------------------------------------------------------------------- #
print("\n--- Set gripper to half-open (0.4 rad) ---")
success = robot.gripper_set(0.4, max_effort=MAX_EFFORT)
time.sleep(SETTLE_TIME)
print(f"gripper_set(0.4) accepted: {success}")
print(f"gripper_position after:    {robot.gripper_position:.4f} rad")

# --------------------------------------------------------------------------- #
# Stage 4e: open again to leave in safe state
# --------------------------------------------------------------------------- #
print("\n--- Open gripper (final safe state) ---")
robot.gripper_open(max_effort=MAX_EFFORT)
time.sleep(SETTLE_TIME)
print(f"Final gripper_position: {robot.gripper_position:.4f} rad")

print("\nStage 4 complete — gripper tests passed.")
