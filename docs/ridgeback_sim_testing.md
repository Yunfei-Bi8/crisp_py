# Ridgeback Robot Simulation Testing Guide

Step-by-step tests to verify that the `RidgebackRobot` class and the full data
collection pipeline work correctly in the MuJoCo simulation environment before
running on the real robot.

**Covers:**
- ROS2 connection and joint state reading
- `home()` and joint trajectory control
- Forward kinematics via Pinocchio (`end_effector_pose`)
- Cartesian control via `move_cartesian_async()`
- `DataCollector` robot-state recording
- Full recording pipeline with `KeyboardRecordingManager` and LeRobot Dataset

---

## Prerequisites

| Requirement | Details |
|-------------|---------|
| MuJoCo simulation | `clearpath_simulation` repo, `pixi run --environment ros2 sim` |
| URDF | `/home/yunfei/clearpath_remote_ws/src/tum09_ridgeback/tum09_bringup/config/robot.urdf` |
| Python environment | `jazzy-lerobot` feature in `crisp_gym` pixi workspace |
| Modified source files | See [Key Bug Fixes](#key-bug-fixes) below |

---

## Key Bug Fixes Applied

Before running tests, make sure the following fixes are in place.

### 1. Cartesian controller topic (`ridgeback_robot.py`)

`CartesianController` subscribes to the **root-level** `/target_pose`, not
`/cartesian_controller/target_pose`.

```python
# RidgebackConfig.for_simulation() — correct value:
cartesian_target_topic="/target_pose"
```

Verify with:
```bash
ros2 node info /cartesian_controller   # must show /target_pose under Subscribers
```

### 2. Controller switching is idempotent (`ridgeback_controller_switcher.py`)

`switch_to_cartesian()` and `switch_to_joint_trajectory()` now query
`list_controllers` first.  If the target controller is already active they
return `True` immediately instead of issuing a STRICT switch that would fail.

This is important after a program crash, which leaves the controller in an
unknown state.

### 3. Keyboard input uses `sys.stdin` raw mode (`recording_manager.py`)

`pynput` relies on X11 global event capture, which Wayland's security model
blocks.  `KeyboardRecordingManager` now reads single characters from `stdin`
via `termios` raw mode in a background thread — works in any terminal
(SSH, Wayland, VSCode integrated terminal).

### 4. `home()` always preceded by JTC restore (`08_ridgeback_spacemouse_record.py`)

`switch_to_joint_trajectory_controller()` is called before `home()` so that a
previous crash leaving `cartesian_controller` active does not prevent homing.

### 5. Unlimited episode mode (`RecordingManager` + script)

`num_episodes = 0` means record indefinitely; the user presses `q` to quit.
`_set_to_wait()` checks `unlimited = config.num_episodes <= 0` and never
auto-transitions to `"exit"` in that case.

---

## Setup

### Terminal 1 — Start MuJoCo simulation (keep running for all tests)

```bash
cd /home/yunfei/clearpath_simulation
pixi run --environment ros2 sim
```

Wait until you see:
```
[joint_trajectory_controller]: Controller active
[cartesian_controller]: Controller active
```

### Terminal 2 — Open the test environment

```bash
cd /home/yunfei/crisp_gym
pixi shell --environment jazzy-lerobot
```

All tests below are run inside this shell.

---

## Test 1 — ROS2 Connection + Joint State

**What is tested:**
Initialises `rclpy`, builds a `RidgebackRobot` instance with the simulation
config, waits until the first `/joint_states` message arrives, then reads the
6 arm joint angles, velocities, and the ROS timestamp.  This is the
prerequisite for every subsequent test.

```python
python3 - << 'EOF'
import rclpy
from crisp_py.robot import RidgebackConfig, make_ridgeback_robot

URDF = "/home/yunfei/clearpath_remote_ws/src/tum09_ridgeback/tum09_bringup/config/robot.urdf"

rclpy.init()
cfg = RidgebackConfig.for_simulation()
robot = make_ridgeback_robot(urdf_path=URDF, config=cfg)
robot.wait_until_ready(timeout=15.0)

print("=== Test 1: ROS2 Connection + Joint State ===")
print(f"Robot ready: True")
print(f"Number of joints (nq):         {robot.nq}")
print(f"Joint values      [rad]:       {robot.joint_values}")
print(f"Joint velocities  [rad/s]:     {robot.joint_velocities}")
print(f"Joint state stamp [ns]:        {robot.joint_state_stamp_ns}")
print("Test 1 PASSED")

rclpy.shutdown()
EOF
```

**Expected output:**
```
=== Test 1: ROS2 Connection + Joint State ===
Robot ready: True
Number of joints (nq):         6
Joint values      [rad]:       [ 0.    -1.571  1.571 -1.571 -1.571  0.   ]
Joint velocities  [rad/s]:     [0. 0. 0. 0. 0. 0.]
Joint state stamp [ns]:        1775XXXXXXXXX
Test 1 PASSED
```

---

## Test 2 — `home()` + Joint Trajectory Controller

**What is tested:**
Calls `switch_to_joint_trajectory_controller()` (idempotent — safe even if
already active after a previous crash) then sends the robot to its predefined
home configuration via `FollowJointTrajectory`.  Verifies that `is_homed()`
returns `True` and that the maximum joint error is below 0.05 rad.

The test first moves the arm to a non-home position so that the return motion
is clearly visible in the MuJoCo viewer.  If the robot is already at home
when `home()` is called, there is no displacement and the viewer shows no
movement — this is expected, not a bug.

```python
python3 - << 'EOF'
import time
import numpy as np
import rclpy
from crisp_py.robot import RidgebackConfig, make_ridgeback_robot

URDF = "/home/yunfei/clearpath_remote_ws/src/tum09_ridgeback/tum09_bringup/config/robot.urdf"

rclpy.init()
cfg = RidgebackConfig.for_simulation()
robot = make_ridgeback_robot(urdf_path=URDF, config=cfg)
robot.wait_until_ready(timeout=15.0)

print("=== Test 2: home() + Joint Trajectory Controller ===")

# Idempotent: safe to call even if JTC is already active
print("Ensuring JointTrajectoryController is active...")
robot.switch_to_joint_trajectory_controller()

# Move to a clearly different pose first so the return-to-home motion
# is visible in the MuJoCo viewer window.
non_home = [0.0, -0.5, 0.5, -0.5, -1.57, 0.0]
print(f"Moving to non-home position {non_home} (5 s) — watch the MuJoCo viewer...")
robot.move_joints(non_home, duration=5.0)
time.sleep(1.0)
print(f"Non-home joints: {robot.joint_values}")

print("Moving BACK to home position (5 s) — watch the MuJoCo viewer...")
robot.home(duration=5.0)
time.sleep(1.0)

homed = robot.is_homed()
home_cfg = np.array(robot.home_config)
current  = robot.joint_values
error    = np.abs(current - home_cfg)

print(f"Is homed:              {homed}")
print(f"Home config   [rad]:   {home_cfg}")
print(f"Current joints [rad]:  {current}")
print(f"Max joint error [rad]: {error.max():.4f}  (threshold: 0.05)")

if homed and error.max() < 0.05:
    print("Test 2 PASSED")
else:
    print("Test 2 FAILED")

rclpy.shutdown()
EOF
```

**Expected output:**
```
=== Test 2: home() + Joint Trajectory Controller ===
Ensuring JointTrajectoryController is active...
[INFO] JointTrajectoryController ('joint_trajectory_controller') is already active — no switch needed.
Moving to non-home position [0.0, -0.5, 0.5, -0.5, -1.57, 0.0] (5 s) — watch the MuJoCo viewer...
Non-home joints: [ 0.001 -0.500  0.500 -0.500 -1.570  0.000]
Moving BACK to home position (5 s) — watch the MuJoCo viewer...
Is homed:              True
Home config   [rad]:   [ 0.    -1.571  1.571 -1.571 -1.571  0.   ]
Current joints [rad]:  [ 0.001 -1.570  1.571 -1.571 -1.570  0.000]
Max joint error [rad]: 0.0012  (threshold: 0.05)
Test 2 PASSED
```

> **Note on MuJoCo viewer:** The passive viewer tracks `/joint_states` at 60 FPS.
> It only shows movement when the robot's joint angles actually change.
> If the robot is already at the target position, `home()` completes with zero
> displacement and the viewer appears static — this is correct behaviour.

---

## Test 3 — End-Effector Pose (Forward Kinematics)

**What is tested:**
Reads the current joint angles and computes the end-effector pose in
`base_link` frame using Pinocchio FK.  Checks that the returned position has
shape `(3,)` and that the quaternion norm is 1.0 (unit quaternion).  This is
the basis for all Cartesian-space operations.

```python
python3 - << 'EOF'
import time
import numpy as np
import rclpy
from crisp_py.robot import RidgebackConfig, make_ridgeback_robot

URDF = "/home/yunfei/clearpath_remote_ws/src/tum09_ridgeback/tum09_bringup/config/robot.urdf"

rclpy.init()
cfg = RidgebackConfig.for_simulation()
robot = make_ridgeback_robot(urdf_path=URDF, config=cfg)
robot.wait_until_ready(timeout=15.0)

robot.switch_to_joint_trajectory_controller()
robot.home(duration=8.0)
time.sleep(1.0)

print("=== Test 3: End-Effector Pose (Forward Kinematics via Pinocchio) ===")
pose = robot.end_effector_pose

pos  = pose.position
rpy  = pose.orientation.as_euler("xyz")
quat = pose.orientation.as_quat()   # [x, y, z, w]

print(f"EE position   (x, y, z)   [m]:    {pos}")
print(f"EE euler      (r, p, y)   [rad]:  {rpy}")
print(f"EE quaternion (x, y, z, w):       {quat}")
print(f"Position shape:  {pos.shape}   (expected: (3,))")
print(f"Quaternion norm: {np.linalg.norm(quat):.6f}  (expected: 1.0)")

ok = pos.shape == (3,) and abs(np.linalg.norm(quat) - 1.0) < 1e-4
print(f"Test 3 {'PASSED' if ok else 'FAILED'}")

rclpy.shutdown()
EOF
```

**Expected output:**
```
=== Test 3: End-Effector Pose (Forward Kinematics via Pinocchio) ===
EE position   (x, y, z)   [m]:    [0.xxx  0.xxx  0.xxx]
EE euler      (r, p, y)   [rad]:  [0.xxx  0.xxx  0.xxx]
EE quaternion (x, y, z, w):       [0.xxx  0.xxx  0.xxx  0.xxx]
Position shape:  (3,)   (expected: (3,))
Quaternion norm: 1.000000  (expected: 1.0)
Test 3 PASSED
```

---

## Test 4 — Cartesian Controller (`move_cartesian_async`)

**What is tested:**
Switches to `CartesianController`, sends a target pose that is 5 cm above the
current end-effector position, streams the command at 20 Hz for 3 seconds, and
verifies that the robot actually moved within 1 cm of the target.

This test validates the full loop:
`move_cartesian_async()` → `/target_pose` topic → `CartesianController` →
EMA filter → MuJoCo hardware interface (with gravity compensation) → joint
torques → robot motion.

```python
python3 - << 'EOF'
import time
import copy
import numpy as np
import rclpy
from crisp_py.robot import RidgebackConfig, make_ridgeback_robot

URDF = "/home/yunfei/clearpath_remote_ws/src/tum09_ridgeback/tum09_bringup/config/robot.urdf"

rclpy.init()
cfg = RidgebackConfig.for_simulation()
robot = make_ridgeback_robot(urdf_path=URDF, config=cfg)
robot.wait_until_ready(timeout=15.0)

print("=== Test 4: CartesianController (move_cartesian_async) ===")

# Start from a known home position
robot.switch_to_joint_trajectory_controller()
robot.home(duration=8.0)
time.sleep(1.0)

initial_pose = robot.end_effector_pose
initial_z    = initial_pose.position[2]
print(f"Initial EE z: {initial_z:.4f} m")

# Switch to CartesianController (idempotent — safe if already active)
print("Switching to CartesianController...")
ok = robot.switch_to_cartesian_controller()
print(f"Switch successful: {ok}")

# Target: +5 cm in Z
target = copy.deepcopy(initial_pose)
target.position[2] += 0.05

print("Streaming target pose at 20 Hz for 3 seconds (z + 0.05 m)...")
for _ in range(60):
    robot.move_cartesian_async(target)
    time.sleep(0.05)

final_z       = robot.end_effector_pose.position[2]
z_displacement = final_z - initial_z
print(f"Final EE z:     {final_z:.4f} m")
print(f"Z displacement: {z_displacement:.4f} m  (expected ~0.05, threshold ±0.01)")

# Restore JointTrajectoryController
robot.switch_to_joint_trajectory_controller()
print("Restored JointTrajectoryController.")

if abs(z_displacement - 0.05) < 0.01:
    print("Test 4 PASSED")
else:
    print(f"Test 4 FAILED — displacement {z_displacement:.4f} m != 0.05 m")

rclpy.shutdown()
EOF
```

**Expected output:**
```
=== Test 4: CartesianController (move_cartesian_async) ===
Initial EE z: 0.xxxx m
Switching to CartesianController...
Switch successful: True
Streaming target pose at 20 Hz for 3 seconds (z + 0.05 m)...
Final EE z:     0.xxxx m
Z displacement: 0.0498 m  (expected ~0.05, threshold ±0.01)
Restored JointTrajectoryController.
Test 4 PASSED
```

---

## Test 5 — `DataCollector` (Robot State Recording)

**What is tested:**
Runs the full `DataCollector` lifecycle — `start_episode()`, 40 calls to
`record_step()`, `stop_episode()` — and validates the saved `state.npz` file.
Checks array shapes, dtypes, step count, and `info.json` metadata.

In simulation mode there are no cameras, so only `state.npz` and `info.json`
are written (no `cam_*.npz`).

```python
python3 - << 'EOF'
import time
import json
import numpy as np
import rclpy
from crisp_py.robot import RidgebackConfig, make_ridgeback_robot
from crisp_py.data import DataCollector

URDF       = "/home/yunfei/clearpath_remote_ws/src/tum09_ridgeback/tum09_bringup/config/robot.urdf"
OUTPUT_DIR = "/tmp/test_ridgeback_depth"
N_STEPS    = 40

rclpy.init()
cfg = RidgebackConfig.for_simulation()
robot = make_ridgeback_robot(urdf_path=URDF, config=cfg)
robot.wait_until_ready(timeout=15.0)

print("=== Test 5: DataCollector (robot state recording) ===")
robot.switch_to_joint_trajectory_controller()
robot.home(duration=8.0)
time.sleep(1.0)

collector = DataCollector(
    robot=robot,
    cameras=[],           # no cameras in simulation
    output_dir=OUTPUT_DIR,
)
print(f"Collector: {collector}")

episode_id = collector.start_episode()
print(f"Recording episode: {episode_id}  ({N_STEPS} steps at ~20 Hz)")

for i in range(N_STEPS):
    pose = robot.end_effector_pose
    collector.record_step(action=pose)
    if i % 10 == 0:
        print(f"  Step {i+1:3d}/{N_STEPS} — EE z: {pose.position[2]:.4f} m  "
              f"| buffered: {collector.current_episode_steps}")
    time.sleep(0.05)

saved_path  = collector.stop_episode()
steps_saved = collector.last_episode_steps
print(f"\nSaved to:     {saved_path}")
print(f"Steps saved:  {steps_saved}  (expected {N_STEPS})")

# Verify state.npz
state = np.load(saved_path / "state.npz")
print(f"\n--- state.npz contents ---")
expected_shapes = {
    "joint_positions":      (N_STEPS, 6),
    "joint_velocities":     (N_STEPS, 6),
    "gripper_position":     (N_STEPS,),
    "ee_position":          (N_STEPS, 3),
    "ee_quaternion":        (N_STEPS, 4),
    "action_ee_position":   (N_STEPS, 3),
    "action_ee_quaternion": (N_STEPS, 4),
    "robot_stamp_ns":       (N_STEPS,),
    "wall_time_ns":         (N_STEPS,),
}
all_ok = True
for key, expected in expected_shapes.items():
    actual = state[key].shape
    status = "OK" if actual == expected else f"FAIL (got {actual})"
    print(f"  {key:<26} shape={actual}  {status}")
    if actual != expected:
        all_ok = False

# Verify info.json
with open(saved_path / "info.json") as f:
    info = json.load(f)
print(f"\n--- info.json ---")
print(f"  n_steps:    {info['n_steps']}")
print(f"  duration_s: {info['duration_s']}")
print(f"  ee_frame:   {info['robot']['ee_frame']}")

ok = all_ok and steps_saved == N_STEPS and (saved_path / "state.npz").exists()
print(f"\nTest 5 {'PASSED' if ok else 'FAILED'}")

rclpy.shutdown()
EOF
```

**Expected output:**
```
=== Test 5: DataCollector (robot state recording) ===
Recording episode: episode_0000  (40 steps at ~20 Hz)
  Step   1/40 — EE z: 0.xxxx m  | buffered: 1
  Step  11/40 — EE z: 0.xxxx m  | buffered: 11
  Step  21/40 — EE z: 0.xxxx m  | buffered: 21
  Step  31/40 — EE z: 0.xxxx m  | buffered: 31

Saved to:     /tmp/test_ridgeback_depth/episode_0000
Steps saved:  40  (expected 40)

--- state.npz contents ---
  joint_positions           shape=(40, 6)  OK
  joint_velocities          shape=(40, 6)  OK
  gripper_position          shape=(40,)    OK
  ee_position               shape=(40, 3)  OK
  ee_quaternion             shape=(40, 4)  OK
  action_ee_position        shape=(40, 3)  OK
  action_ee_quaternion      shape=(40, 4)  OK
  robot_stamp_ns            shape=(40,)    OK
  wall_time_ns              shape=(40,)    OK

--- info.json ---
  n_steps:    40
  duration_s: 1.95
  ee_frame:   tool0

Test 5 PASSED
```

---

## Test 6 — Full Recording Pipeline

**What is tested:**
End-to-end test of the complete data collection pipeline:

```
Keyboard (stdin raw mode)
  → KeyboardRecordingManager state machine
    → RecordingManager.record_episode() control loop (20 Hz)
      → move_cartesian_async()         — robot command
      → DataCollector.record_step()    — depth/state npz
      → LeRobot dataset writer process — parquet + video
```

Two data stores are written in parallel and aligned by episode index.

**Storage locations:**
- LeRobot dataset: `$HF_LEROBOT_HOME/test_sim/` (default: `~/.cache/huggingface/lerobot/test_sim/`)
- Depth npz: `~/data/ridgeback_depth/`

### Run the script

```bash
# Remove any leftover dataset from a previous run (repo_id collision raises an error)
rm -rf ~/.cache/huggingface/lerobot/test_sim 2>/dev/null || true

python3 examples/08_ridgeback_spacemouse_record.py \
    --repo-id test_sim \
    --sim \
    --fps 20
# --num-episodes is omitted → defaults to 0 (unlimited); press q to quit
```

### Keyboard controls

| Key | State required | Effect |
|-----|---------------|--------|
| `r` | `is_waiting`  | Start recording |
| `r` | `recording`   | Pause recording |
| `s` | `paused`      | Save episode → next episode |
| `d` | `paused`      | Discard episode → next episode |
| `q` | `is_waiting` or `paused` | Exit program |

### Keyboard operation sequence

```
"Press 'r' to start recording." appears

  Press r   → recording starts
  (wait a few seconds)
  Press r   → recording pauses
  Press s   → episode saved, next episode begins

  (repeat for as many episodes as needed)

  Press q   → exit (only works in is_waiting or paused state)
```

### Expected output

```
============================================================
  Mode:      SIMULATION
  Repo ID:   test_sim
  Task:      pick the object
  Episodes:  unlimited (press q to quit)
  Frequency: 20 Hz
  Camera:    none (sim)
============================================================

Connecting to robot...
Robot ready.
LeRobot RecordingManager ready.
[INFO] JointTrajectoryController already active — no switch needed.

Moving to home...
At home.
Activating CartesianController...
CartesianController active.

╭──────────────────────────────────────────╮
│ Keys for recording:                      │
│ <r> To start/stop Recording.             │
│ <s> To Save the current recorded episode.│
│ <d> to Delete the current episode.       │
│ <q> To Quit the recording.               │
╰──────────────────────────────────────────╯

--- Episode 1 / ? ---
    Depth episode ID: episode_0000
    Press 'r' to start recording.
    ...
Saved — LeRobot episode 000000 | Depth: .../episode_0000 (N steps)

--- Episode 2 / ? ---
    ...
Saved — LeRobot episode 000001 | Depth: .../episode_0001 (N steps)

Restoring JointTrajectoryController...
Done.

Collection complete.
  LeRobot dataset : $HF_LEROBOT_HOME/test_sim/
  Depth npz       : ~/data/ridgeback_depth
```

> **Note:** A harmless `Exception in thread Thread-1 (_spin_node)` may appear
> after the program exits.  This is a known rclpy shutdown race condition and
> does not affect saved data.

### Verify saved data (run after Test 6)

```python
python3 - << 'EOF'
import os
import numpy as np
from pathlib import Path

print("=== Verifying saved data ===")

# LeRobot dataset
lerobot_home = Path(
    os.environ.get("HF_LEROBOT_HOME", "~/.cache/huggingface/lerobot")
).expanduser()
dataset_dir = lerobot_home / "test_sim"
print(f"\nLeRobot dataset: {dataset_dir}")
for f in sorted(dataset_dir.rglob("*"))[:25]:
    if f.is_file():
        size_kb = f.stat().st_size // 1024
        print(f"  {str(f.relative_to(dataset_dir)):<55}  {size_kb:>6} KB")

# Depth npz
depth_dir = Path("~/data/ridgeback_depth").expanduser()
print(f"\nDepth npz root: {depth_dir}")
for ep_dir in sorted(depth_dir.iterdir()):
    if not ep_dir.is_dir():
        continue
    state_file = ep_dir / "state.npz"
    if state_file.exists():
        data = np.load(state_file)
        n = data["ee_position"].shape[0]
        print(f"  {ep_dir.name}/state.npz — {n} steps | "
              f"ee_position shape: {data['ee_position'].shape}")

print("\nVerification complete.")
EOF
```

---

## Test Summary

| Test | Functionality verified | Key code paths |
|------|----------------------|----------------|
| 1 | ROS2 init, `/joint_states` subscriber, `joint_values`, `joint_velocities` | `RidgebackRobot.__init__`, `_cb_joint_state` |
| 2 | `JointTrajectoryController`, `home()`, idempotent controller switch | `switch_to_joint_trajectory_controller`, `home()` |
| 3 | Pinocchio FK, `end_effector_pose`, unit quaternion | `_fk()`, `end_effector_pose` property |
| 4 | `/target_pose` publisher, EMA filter, actual robot displacement | `move_cartesian_async`, `switch_to_cartesian_controller` |
| 5 | `DataCollector` lifecycle, `state.npz` format, `info.json` metadata | `record_step`, `stop_episode`, `_save_state` |
| 6 | Full pipeline: keyboard → state machine → writer process → dataset | `KeyboardRecordingManager`, `record_episode` |

All six tests must pass before running the real-robot data collection workflow.

---

## Troubleshooting

### `controller_manager rejected switch` (STRICT switch error)

Happens when `cartesian_controller` was left active by a previous crash.
The idempotent `switch_to_cartesian_controller()` / `switch_to_joint_trajectory_controller()`
methods (added in `ridgeback_controller_switcher.py`) handle this automatically.
If it still occurs, restart the MuJoCo simulation (Terminal 1).

### Keyboard `r` key has no effect

Symptom: UI appears but pressing `r` does nothing.
Cause: `pynput` uses X11 global capture, which Wayland blocks.
Fix: already applied in `recording_manager.py` — `KeyboardRecordingManager`
now uses `sys.stdin` + `termios` raw mode.  Make sure you are running the
script in an interactive terminal (not piped stdin).

### `Z displacement: 0.0` in Test 4

Cause: `move_cartesian_async()` was publishing to the wrong topic
(`/cartesian_controller/target_pose` instead of `/target_pose`).
Fix: already applied in `RidgebackConfig.for_simulation()`:
```python
cartesian_target_topic="/target_pose"
```
Diagnose with:
```bash
ros2 topic info /target_pose                      # Subscription count should be 1
ros2 node info /cartesian_controller              # Subscribers: /target_pose
```

### `AttributeError: 'RidgebackRobot' object has no attribute 'joint_positions'`

The correct attribute name is `joint_values` (not `joint_positions`).
`joint_positions` does not exist on `RidgebackRobot`.

### `FileExistsError: The repo_id already exists`

Remove the existing dataset before re-running Test 6:
```bash
rm -rf ~/.cache/huggingface/lerobot/test_sim
```
