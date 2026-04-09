# Ridgeback Hardware Data Collection — RGBD + Robot

Complete step-by-step guide for collecting RGBD demonstration data on the
TUM09 Clearpath Ridgeback + UR10e robot using Orbbec Femto Bolt cameras and
a 3Dconnexion SpaceMouse.

Covers single-camera and dual-camera setups. All data (robot state + colour +
depth) is saved to disk via `DataCollector` in crisp_py.

---

## Hardware Requirements

- Clearpath Ridgeback with UR10e arm and Robotiq 2F-85 gripper
- 1 or 2 Orbbec Femto Bolt RGBD cameras connected via USB 3.0
- 3Dconnexion SpaceMouse connected via USB
- Developer laptop connected to the robot network (Ethernet `192.168.131.x`
  or WiFi `192.168.50.x`)

---

## Workspaces Overview

| Workspace | Purpose |
|---|---|
| `~/clearpath_remote_ws` | ROS2 camera drivers, robot launch files |
| `~/crisp_py` | Data collection scripts, robot API |

Both use `pixi` for environment management.

---

## Phase 1 — Physical Robot Startup

> Do this once at the beginning of each session.

### 1.1 Power on the Ridgeback base

1. Press the **power button** at the base of the robot
2. Wait until the LED changes from solid red → flashing red

### 1.2 Release E-Stops

Perform in this exact order:

1. Pull out / disable all E-stop buttons on the Ridgeback base
2. Take the **Autec remote E-stop**
3. Pull out / disable the E-stop button on the remote
4. **Hold the Start button** on the remote until the LED blinks green fast
5. **Press Start once** → LED blinks green slowly → remote E-stop is now armed
6. Press the **E-stop Reset button** at the back of the Ridgeback base

**Expected state:** solid red light at the back, white light at the front → robot ready.

### 1.3 Power on the UR10e arm

1. Press the **"UR Power"** button on the plate above the touchpad
2. Wait for the touchpad to fully boot
3. On the touchpad, power on the arm and press **Play** to run the saved
   External Control program

### 1.4 Verify network connectivity

On the developer laptop, open a terminal and check:

```bash
ping 192.168.131.40   # robot base IP
```

---

## Phase 2 — Environment Setup (Developer Laptop)

### 2.1 Set ROS2 environment (one-time per session)

`crisp_py` uses `personal_ros_env.sh` to match the RMW and domain of
`clearpath_remote_ws`. This file already exists at
`~/crisp_py/scripts/personal_ros_env.sh`:

```bash
# Content of personal_ros_env.sh — already set correctly:
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

This is sourced automatically when you run `pixi run --environment jazzy`
from within `~/crisp_py`.

### 2.2 Verify ROS2 connectivity

**Terminal A** — confirm robot topics are visible:

```bash
cd ~/clearpath_remote_ws
pixi run ros2 topic list | grep joint_states
```

Expected output:
```
/r100_0207/platform/joint_states
```

If nothing appears, check that the laptop is on the correct network
(`192.168.131.x` or `192.168.50.x`).

---

## Phase 3 — Camera Setup

### 3.1 Identify USB port(s)

**Terminal A:**

```bash
cd ~/clearpath_remote_ws
pixi run ros2 run orbbec_camera list_devices_node
```

Example output for two cameras:
```
Found Orbbec device Femto Bolt, usb port 2-1,   serial CL83263009G
Found Orbbec device Femto Bolt, usb port 4-1,   serial CL8326300D2
```

Note down the `usb port` values — you will need them below.

### 3.2 (Dual camera only) Increase USB buffer

Two Femto Bolt cameras at full resolution exhaust Linux's default USB
filesystem buffer. Run this before launching dual cameras — **required after
every reboot**:

```bash
sudo sh -c 'echo 256 > /sys/module/usbcore/parameters/usbfs_memory_mb'
```

To make it permanent across reboots, add to `/etc/rc.local`:
```bash
echo 256 > /sys/module/usbcore/parameters/usbfs_memory_mb
```

### 3.3 Launch the Orbbec driver

**Terminal B** — keep this terminal running for the entire session.

**Single camera:**
```bash
cd ~/clearpath_remote_ws
pixi run ros2 launch tum09_custom orbbec.launch.py
```

Topics published under `/camera/`:
```
/camera/color/image_raw       # RGB 1280×720 @30 Hz
/camera/depth/image_raw       # depth 640×576 @30 Hz (aligned to colour)
/camera/color/camera_info     # intrinsics
/camera/depth/color/points    # RGBD point cloud
```

**Dual cameras** (replace port values with those from Step 3.1):
```bash
cd ~/clearpath_remote_ws
pixi run ros2 launch tum09_custom orbbec_multi.launch.py usb_port1:=2-1 usb_port2:=4-1
```

Topics published under `/camera_01/` and `/camera_02/`:
```
/camera_01/color/image_raw    /camera_02/color/image_raw
/camera_01/depth/image_raw    /camera_02/depth/image_raw
/camera_01/color/camera_info  /camera_02/color/camera_info
```

Wait until you see both cameras report:
```
[camera_01]: Color Frame - Width: 1280 Height: 720 fps: 30 Format: MJPG
[camera_01]: Depth Frame - Width: 640 Height: 576 fps: 30 Format: Y16
[camera_01]: Initialize device cost XXXX ms
[camera_02]: Color Frame - Width: 1280 Height: 720 fps: 30 Format: MJPG
[camera_02]: Depth Frame - Width: 640 Height: 576 fps: 30 Format: Y16
[camera_02]: Initialize device cost XXXX ms
```

### 3.4 Verify camera streams (optional but recommended)

**Terminal C** — quick visual check:

```bash
cd ~/clearpath_remote_ws

# Single camera — opens a window with colour (left) and depth (right)
pixi run python src/tum09_ridgeback/tum09_custom/scripts/test_orbbec.py

# Dual cameras — opens Rerun viewer with 2×2 grid
pixi run python src/tum09_ridgeback/tum09_custom/scripts/visualize_orbbec_multi.py
```

Wave your hand in front of the cameras to confirm both colour and depth
respond in real time. Press `Q` to close.

---

## Phase 4 — Robot Checks (crisp_py)

> Run from `~/crisp_py` using the `jazzy` environment.
> All scripts below can be run from the same terminal.

**Terminal D:**

```bash
cd ~/crisp_py
```

### 4.1 Check robot state (no motion)

```bash
pixi run --environment jazzy python3 examples/ridgeback/00_check_state.py
```

Confirms: ROS2 connectivity, joint state reading, FK, gripper state.
Expected: prints current joint angles, EE pose, and gripper position with no errors.

### 4.2 Verify cameras via crisp_py

**Single camera:**
```bash
pixi run --environment jazzy python3 examples/ridgeback/06_rgbd_cameras.py
```

**Dual cameras:**
```bash
pixi run --environment jazzy python3 examples/ridgeback/06_rgbd_cameras.py \
    --namespaces /camera_01 /camera_02
```

Expected output per camera:
```
Camera: camera_01
  Intrinsics:
    Resolution : 1280 x 720  (W x H)
    Focal length : fx=...  fy=...
  Color frame:
    Shape  : (720, 1280, 3)  (H x W x 3, uint8 RGB)
  Depth frame:
    Shape  : (576, 640)  (H x W, float32 metres)
    Valid pixels : 85.x%
    Depth range  : [0.3xx, 2.xxx] m
  Timestamp alignment:
    Difference  : x.xx ms
    Sync OK  (< 50 ms)
Stage 6 PASSED — all N camera(s) operational.
```

### 4.3 Home the robot

> **Stand next to the robot and be ready to press E-stop.**

```bash
pixi run --environment jazzy python3 examples/ridgeback/01_home.py
```

The arm moves to the home configuration. Confirm motion is smooth and
there are no unexpected collisions before proceeding.

---

## Phase 5 — SpaceMouse Setup

Plug in the SpaceMouse before running the data collection script.

### 5.1 Check device permissions

```bash
ls /dev/hidraw*
```

If you get `Permission denied` when running the collection script, add a
udev rule (one-time setup):

```bash
echo 'SUBSYSTEM=="hidraw", ATTRS{idVendor}=="256f", MODE="0666"' \
    | sudo tee /etc/udev/rules.d/99-spacemouse.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 5.2 SpaceMouse axis convention

The axis mapping in crisp_py matches the clearpath `spacemouse_ros2` convention:

| SpaceMouse motion | End-effector motion |
|---|---|
| Push forward (away from you) | −X (away in base frame) |
| Pull backward (toward you) | +X |
| Push left | +Y |
| Push right | −Y |
| Push up | +Z (up) |
| Push down | −Z (down) |
| Rotate (roll/pitch/yaw) | EE rotation, all axes negated vs. raw device |
| Left button | Open gripper |
| Right button | Close gripper |

---

## Phase 6 — Data Collection

**Terminal D** (same `~/crisp_py` terminal):

### 6.1 Single camera

```bash
pixi run --environment jazzy python3 examples/ridgeback/07_data_collection.py \
    --input spacemouse \
    --namespaces /camera \
    --output ~/data/ridgeback_episodes \
    --episodes 5 \
    --hz 20
```

### 6.2 Dual cameras

```bash
pixi run --environment jazzy python3 examples/ridgeback/07_data_collection.py \
    --input spacemouse \
    --namespaces /camera_01 /camera_02 \
    --output ~/data/ridgeback_episodes \
    --episodes 5 \
    --hz 20
```

### 6.3 Episode workflow

The script will:
1. Connect to the robot and cameras
2. Move the arm to home position
3. Switch to `CartesianController` (required for SpaceMouse control)
4. Print `--- Recording episode: 0 ---` and wait for input

**During each episode:**

| Action | Result |
|---|---|
| Move SpaceMouse | EE moves in Cartesian space |
| Right button | Gripper closes |
| Left button | Gripper opens |
| **Space** | Save episode → start next |
| **X** | Discard episode → start next |
| **Esc** | Quit (restores JointTrajectoryController) |

**Terminal output during recording:**
```
--- Recording episode: 0 ---
  t=2s  steps=40  EE=[0.512, -0.130, 0.421]
  t=4s  steps=80  EE=[0.524, -0.118, 0.435]
Episode saved (120 steps) → /home/yunfei/data/ridgeback_episodes/episode_0000
--- Recording episode: 1 ---
```

---

## Phase 7 — Verify Saved Data

After collection, inspect the saved files:

```bash
ls ~/data/ridgeback_episodes/episode_0000/
# info.json   state.npz   cam_camera.npz          (single camera)
# info.json   state.npz   cam_camera_01.npz   cam_camera_02.npz   (dual cameras)
```

**Quick verification in Python:**

```python
import numpy as np, json, pathlib

ep = pathlib.Path("~/data/ridgeback_episodes/episode_0000").expanduser()

# Metadata
info = json.loads((ep / "info.json").read_text())
print(f"Steps: {info['n_steps']},  Duration: {info['duration_s']:.1f}s")
print(f"Cameras: {list(info['cameras'].keys())}")

# Robot state
state = np.load(ep / "state.npz")
print(f"joint_positions : {state['joint_positions'].shape}")   # (N, 6)
print(f"ee_position     : {state['ee_position'].shape}")       # (N, 3)
print(f"gripper_position: {state['gripper_position'].shape}")  # (N,)

# Camera data (single camera example)
cam = np.load(ep / "cam_camera.npz")         # or cam_camera_01.npz
print(f"color : {cam['color'].shape}")        # (N, H, W, 3)  uint8 RGB
print(f"depth : {cam['depth'].shape}")        # (N, H, W)     float32 metres
print(f"depth range: [{np.nanmin(cam['depth']):.3f}, {np.nanmax(cam['depth']):.3f}] m")
```

---

## Terminal Summary

| Terminal | Directory | Command | Keep running? |
|---|---|---|---|
| A | `clearpath_remote_ws` | Verify topics / list devices | Close after check |
| **B** | `clearpath_remote_ws` | `ros2 launch tum09_custom orbbec[_multi].launch.py` | **Yes — entire session** |
| C | `clearpath_remote_ws` | Visual verification (`test_orbbec.py` / `visualize_orbbec_multi.py`) | Close after check |
| **D** | `crisp_py` | All `examples/ridgeback/` scripts including `07_data_collection.py` | **Yes — during collection** |

---

## Storage Layout

```
~/data/ridgeback_episodes/
    episode_0000/
        info.json           # metadata: n_steps, duration, camera intrinsics
        state.npz           # robot state arrays:
                            #   joint_positions    (N, 6)  float64 rad
                            #   joint_velocities   (N, 6)  float64 rad/s
                            #   gripper_position   (N,)    float64 rad
                            #   ee_position        (N, 3)  float64 m
                            #   ee_quaternion      (N, 4)  float64 [x,y,z,w]
                            #   action_ee_position (N, 3)  float64 m
                            #   action_ee_quaternion(N,4)  float64 [x,y,z,w]
                            #   robot_stamp_ns     (N,)    int64   ns
                            #   wall_time_ns       (N,)    int64   ns
        cam_camera.npz      # single camera:
                            #   color              (N, H, W, 3)  uint8
                            #   depth              (N, H, W)     float32 m
                            #   color_stamp_ns     (N,)          int64
                            #   depth_stamp_ns     (N,)          int64
        cam_camera_01.npz   # dual camera — camera 01 (same fields as above)
        cam_camera_02.npz   # dual camera — camera 02 (same fields as above)
    episode_0001/
        ...
```

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `ros2 topic list` empty | Wrong network or RMW mismatch | Check `ROS_DOMAIN_ID=0` and `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` in both workspaces |
| `list_devices_node` finds no cameras | USB not recognised | Run `lsusb \| grep -i orbbec`; check cable and port |
| Camera driver starts but 0 Hz in crisp_py | RMW mismatch between workspaces | Ensure `~/crisp_py/scripts/personal_ros_env.sh` sets `ROS_DOMAIN_ID=0` and `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` |
| Second camera fails with `UVC_ERROR_NO_MEM` | USB buffer too small | Run `sudo sh -c 'echo 256 > /sys/module/usbcore/parameters/usbfs_memory_mb'` |
| `Stage 6 FAILED — depth all invalid` | `depth_registration` off | Restart orbbec driver; `depth_registration:=true` is default in `orbbec.launch.py` |
| SpaceMouse `permission denied` | Missing udev rule | Add the udev rule in Phase 5.1 |
| `Failed to activate CartesianController` | Controller not loaded on robot | Check `clearpath-manipulators.service` is running on the robot and CRISP controllers are loaded |
| Robot does not move after E-stop release | Remote E-stop not armed | Repeat the E-stop procedure in Phase 1.2 in the correct order |
