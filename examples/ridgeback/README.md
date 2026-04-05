# Ridgeback Robot Test Scripts

Staged hardware tests for `RidgebackRobot` on the Clearpath Ridgeback + UR10e platform.

Run scripts in order. Each stage depends on the previous one passing.

## Prerequisites

1. Robot is powered on and bringup is running  
   **OR** the MuJoCo simulation is running (`clearpath_simulation`)
2. Dev machine is connected to the robot network (Ethernet or WiFi)
3. URDF has been generated (run once in clearpath_remote_ws):
   ```bash
   cd /home/yunfei/clearpath_remote_ws
   pixi run ros2 run clearpath_generator_common generate_description \
     -s src/tum09_ridgeback/tum09_bringup/config
   ```

## Configuration

`RidgebackConfig` controls which ROS2 topics and action names are used.
Two factory presets are available:

| Preset | Topics | Use case |
|--------|--------|----------|
| `RidgebackConfig.for_real_robot()` | `/r100_0207/manipulators/...` | TUM09 Clearpath bringup |
| `RidgebackConfig.for_simulation()` | `/joint_states` (no namespace) | clearpath_simulation MuJoCo |

All scripts accept a `--sim` flag to switch to the simulation preset automatically.

## How to run

```bash
cd /home/yunfei/crisp_py

# Real robot
pixi run --environment jazzy python3 examples/ridgeback/<script>.py

# Simulation
pixi run --environment jazzy python3 examples/ridgeback/<script>.py --sim
```

## Scripts

| Script | Motion? | What it tests |
|--------|---------|---------------|
| `00_check_state.py` | No | ROS2 connectivity, FK, joint/gripper state reading |
| `01_home.py` | Yes (large) | `home()`, `is_homed()`, `move_joints()` |
| `02_joint_motion.py` | Yes (small, 6 deg) | Each joint individually via `move_joints()` |
| `03_cartesian_motion.py` | Yes (5 cm) | `move_to()` along each Cartesian axis, IK pipeline |
| `04_gripper.py` | Gripper only | `gripper_open/close/set()` — skipped automatically in `--sim` |
| `05_teleoperation_loop.py` | Yes (3 cm circle) | `move_to_async()` at 10 Hz (default) **or** `move_cartesian_async()` at 50 Hz (`--cartesian`); controller switched automatically |
| `06_rgbd_cameras.py` | No | RGBD cameras: colour, depth, intrinsics, timestamp sync |
| `07_data_collection.py` | Yes (3 cm circle) | Full pipeline: CartesianController teleoperation + multi-camera RGBD recording + episode saving |

## Data collection (07)

`07_data_collection.py` is the full data collection pipeline.  Each episode is
saved to a self-contained directory:

```
<output_dir>/
    episode_0000/
        info.json          # metadata, camera intrinsics, timing
        state.npz          # joints, EE pose, gripper, commanded actions
        cam_camera.npz     # color (N,H,W,3), depth (N,H,W), timestamps
```

Load an episode in Python:

```python
import numpy as np, json, pathlib

ep = pathlib.Path("~/data/ridgeback_episodes/episode_0000").expanduser()
info   = json.loads((ep / "info.json").read_text())
state  = np.load(ep / "state.npz")
cam    = np.load(ep / "cam_camera.npz")

print(state["joint_positions"].shape)   # (N, 6)
print(cam["color"].shape)               # (N, H, W, 3)  uint8 RGB
print(cam["depth"].shape)               # (N, H, W)     float32 metres
```

## Camera scripts (06+)

Before running camera scripts, start the Orbbec driver:

```bash
# Single camera (default namespace /camera)
pixi run ros2 launch tum09_custom orbbec.launch.py

# Two cameras
pixi run ros2 launch tum09_custom orbbec_multi.launch.py usb_port1:=2-1 usb_port2:=2-3
```

Then in a second terminal:

```bash
# Single camera
pixi run --environment jazzy python3 examples/ridgeback/06_rgbd_cameras.py

# Two cameras
pixi run --environment jazzy python3 examples/ridgeback/06_rgbd_cameras.py \
    --namespaces /camera_01 /camera_02
```
