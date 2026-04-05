"""Stage 5 — Streaming teleoperation loop test.

Tests two streaming control modes by driving the EE along a small circular
trajectory in the YZ plane.

Control modes
-------------
* **JointTrajectory mode** (default):
  Calls ``move_to_async()`` at 10 Hz.  Client-side Pinocchio IK → short
  FollowJointTrajectory goals sent to ``arm_0_joint_trajectory_controller``.

* **CartesianController mode** (``--cartesian`` flag):
  Calls ``move_cartesian_async()`` at 50 Hz.  PoseStamped published directly
  to the CRISP CartesianController; no client-side IK.  The controller is
  activated programmatically via ``robot.switch_to_cartesian_controller()``
  and restored afterward.  This is the recommended mode for smooth real-time
  teleoperation.

SAFETY:
    The arm will trace a 3 cm radius circle.  Keep clear of the workspace.
    Run 01_home.py first and confirm the arm is at home.

Usage:
    cd /home/yunfei/crisp_py

    # JointTrajectory mode (default)
    pixi run --environment jazzy python3 examples/ridgeback/05_teleoperation_loop.py

    # CartesianController mode (recommended for teleoperation)
    pixi run --environment jazzy python3 examples/ridgeback/05_teleoperation_loop.py --cartesian

    # Simulation — JointTrajectory mode
    pixi run --environment jazzy python3 examples/ridgeback/05_teleoperation_loop.py --sim

    # Simulation — CartesianController mode
    pixi run --environment jazzy python3 examples/ridgeback/05_teleoperation_loop.py --sim --cartesian
"""

import argparse
import time

import numpy as np

from crisp_py.robot import RidgebackConfig, make_ridgeback_robot, get_ridgeback_urdf_path

CLEARPATH_WS = "/home/yunfei/clearpath_remote_ws"
SIM_URDF = f"{CLEARPATH_WS}/src/tum09_ridgeback/tum09_bringup/config/robot.urdf"

# --------------------------------------------------------------------------- #
# Parse arguments
# --------------------------------------------------------------------------- #
parser = argparse.ArgumentParser(description="Stage 5: streaming teleoperation loop")
parser.add_argument("--sim", action="store_true", help="Use simulation config")
parser.add_argument(
    "--cartesian",
    action="store_true",
    help=(
        "Use CartesianController mode (move_cartesian_async). "
        "Recommended for smooth teleoperation. Requires CRISP cartesian_controller "
        "to be loaded (it will be activated/deactivated automatically)."
    ),
)
parser.add_argument(
    "--hz",
    type=float,
    default=None,
    help="Override control loop frequency [Hz]. Default: 50 for cartesian, 10 for joint.",
)
parser.add_argument(
    "--duration",
    type=float,
    default=10.0,
    help="Total test duration [s] (default: 10)",
)
parser.add_argument(
    "--radius",
    type=float,
    default=0.03,
    help="Circle radius [m] (default: 0.03 = 3 cm)",
)
args = parser.parse_args()

# Choose defaults based on control mode
if args.hz is not None:
    LOOP_HZ = args.hz
else:
    LOOP_HZ = 50.0 if args.cartesian else 10.0

LOOP_DURATION = args.duration
CIRCLE_RADIUS = args.radius
CIRCLE_FREQ = 0.2       # one full circle every 5 s
TRAJ_HORIZON = 0.15     # only used in JointTrajectory mode
HOME_DURATION = 6.0

mode_str = "CartesianController" if args.cartesian else "JointTrajectory"
env_str = "SIMULATION" if args.sim else "REAL ROBOT"
print(f"Mode: {env_str}  |  Control: {mode_str}  |  {LOOP_HZ:.0f} Hz  |  {LOOP_DURATION}s")

# --------------------------------------------------------------------------- #
# Connect and home
# --------------------------------------------------------------------------- #
if args.sim:
    cfg = RidgebackConfig.for_simulation()
    urdf = SIM_URDF
else:
    cfg = RidgebackConfig.for_real_robot()
    urdf = get_ridgeback_urdf_path(CLEARPATH_WS)

print("Loading URDF and connecting...")
robot = make_ridgeback_robot(urdf_path=urdf, config=cfg)
robot.wait_until_ready(timeout=15.0)
print("Robot ready.")

print(f"\nMoving to home (duration={HOME_DURATION}s)...")
robot.home(duration=HOME_DURATION)
if not robot.is_homed():
    raise RuntimeError("Failed to reach home. Aborting.")
print("At home.")

# Record the home EE pose as the circle centre
centre_pose = robot.end_effector_pose
centre_y = centre_pose.position[1]
centre_z = centre_pose.position[2]
print(f"Circle centre (home EE): {np.round(centre_pose.position, 4)}")
print(f"Circle radius: {CIRCLE_RADIUS*100:.0f} cm  |  freq: {CIRCLE_FREQ} Hz")

# --------------------------------------------------------------------------- #
# Switch to CartesianController if requested
# --------------------------------------------------------------------------- #
if args.cartesian:
    print("\nActivating CartesianController...")
    if not robot.switch_to_cartesian_controller():
        raise RuntimeError(
            "Failed to activate CartesianController. "
            "Check that crisp_controllers are loaded in the bringup."
        )
    print("CartesianController active.")

# --------------------------------------------------------------------------- #
# Teleoperation loop
# --------------------------------------------------------------------------- #
print(f"\nStarting {LOOP_DURATION}s loop at {LOOP_HZ:.0f} Hz...")
print("Press Ctrl+C to abort early.\n")

dt = 1.0 / LOOP_HZ
n_steps = int(LOOP_DURATION * LOOP_HZ)
step_times = []
ee_positions = []

t_start = time.monotonic()

try:
    for step in range(n_steps):
        t_loop_start = time.monotonic()
        t = step * dt

        # Desired EE position on a circle in the YZ plane
        angle = 2.0 * np.pi * CIRCLE_FREQ * t
        target = centre_pose.copy()
        target.position[1] = centre_y + CIRCLE_RADIUS * np.cos(angle)
        target.position[2] = centre_z + CIRCLE_RADIUS * np.sin(angle)

        # Send command (non-blocking in both modes)
        if args.cartesian:
            robot.move_cartesian_async(target)
        else:
            robot.move_to_async(target, duration=TRAJ_HORIZON)

        # Record FK-computed EE position for tracking analysis
        actual = robot.end_effector_pose
        ee_positions.append(actual.position.copy())

        # Sleep for remainder of loop period
        elapsed = time.monotonic() - t_loop_start
        sleep_time = dt - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

        step_times.append(time.monotonic() - t_loop_start)

        log_every = max(1, int(LOOP_HZ))
        if (step + 1) % log_every == 0:
            print(
                f"  t={t+dt:.1f}s  EE={np.round(actual.position, 3)}  "
                f"loop={step_times[-1]*1000:.1f}ms"
            )

except KeyboardInterrupt:
    print("\nAborted by user.")

finally:
    # Always restore JointTrajectory controller so the arm is safe to command
    if args.cartesian:
        print("\nRestoring JointTrajectoryController...")
        robot.switch_to_joint_trajectory_controller()
        print("JointTrajectoryController active.")

total_time = time.monotonic() - t_start
print(f"\nLoop finished in {total_time:.2f}s ({len(step_times)} steps)")

# --------------------------------------------------------------------------- #
# Analysis
# --------------------------------------------------------------------------- #
if len(ee_positions) > 0:
    ee_positions = np.array(ee_positions)
    n = len(step_times)
    mean_loop_ms = np.mean(step_times) * 1000
    max_loop_ms = np.max(step_times) * 1000

    angles = np.array([2.0 * np.pi * CIRCLE_FREQ * i * dt for i in range(n)])
    target_y = centre_y + CIRCLE_RADIUS * np.cos(angles)
    target_z = centre_z + CIRCLE_RADIUS * np.sin(angles)
    tracking_error = np.sqrt(
        (ee_positions[:, 1] - target_y) ** 2 +
        (ee_positions[:, 2] - target_z) ** 2
    )

    print("\n========== Stage 5 Analysis ==========")
    print(f"Control mode :  {mode_str}")
    print(f"Loop timing  :  mean={mean_loop_ms:.1f}ms  max={max_loop_ms:.1f}ms  target={dt*1000:.0f}ms")
    print(f"Tracking err :  mean={np.mean(tracking_error)*100:.2f}cm  max={np.max(tracking_error)*100:.2f}cm")

    if max_loop_ms > dt * 1000 * 1.5:
        print("WARNING: loop ran slower than target frequency.")
    else:
        print("Loop timing OK.")

# --------------------------------------------------------------------------- #
# Return to home
# --------------------------------------------------------------------------- #
print(f"\nReturning to home (duration={HOME_DURATION}s)...")
robot.home(duration=HOME_DURATION)
print(f"is_homed: {robot.is_homed()}")

print(f"\nStage 5 PASSED — {mode_str} teleoperation loop test finished.")
