"""Stage 6 — RGBD camera check (no motion).

Tests that one or more Orbbec Femto Bolt cameras are reachable and streaming:
  1. Wait for colour, depth, and camera_info on each camera
  2. Print intrinsics (K matrix: focal lengths and principal point)
  3. Report frame resolution and depth statistics
  4. Verify colour/depth timestamp alignment (hardware sync check)

No robot motion is commanded.

Prerequisites:
    Start the Orbbec driver(s) before running this script:

    # Single camera:
    pixi run ros2 launch tum09_custom orbbec.launch.py

    # Two cameras:
    pixi run ros2 launch tum09_custom orbbec_multi.launch.py \
        usb_port1:=2-1 usb_port2:=2-3

Usage:
    cd /home/yunfei/crisp_py

    # Single camera at default namespace /camera
    pixi run --environment jazzy python3 examples/ridgeback/06_rgbd_cameras.py

    # Two cameras
    pixi run --environment jazzy python3 examples/ridgeback/06_rgbd_cameras.py \
        --namespaces /camera_01 /camera_02

    # Three cameras
    pixi run --environment jazzy python3 examples/ridgeback/06_rgbd_cameras.py \
        --namespaces /camera_01 /camera_02 /camera_03
"""

import argparse

import numpy as np

from crisp_py.camera import RgbdCameraConfig, make_rgbd_cameras

# --------------------------------------------------------------------------- #
# Parse arguments
# --------------------------------------------------------------------------- #
parser = argparse.ArgumentParser(description="Stage 6: RGBD camera check")
parser.add_argument(
    "--namespaces",
    nargs="+",
    default=["/camera"],
    metavar="NS",
    help="ROS namespaces of the Orbbec cameras (default: /camera)",
)
parser.add_argument(
    "--timeout",
    type=float,
    default=15.0,
    help="Seconds to wait for each camera to become ready (default: 15)",
)
args = parser.parse_args()

print(f"Camera namespaces: {args.namespaces}")

# --------------------------------------------------------------------------- #
# Create cameras (all share one ROS2 node)
# --------------------------------------------------------------------------- #
configs = [RgbdCameraConfig.for_orbbec(ns) for ns in args.namespaces]
cameras = make_rgbd_cameras(configs)

print(f"\nWaiting for {len(cameras)} camera(s) to become ready...")
for cam in cameras:
    print(f"  Waiting for {cam.config.camera_name} ...")
    cam.wait_until_ready(timeout=args.timeout)
    print(f"  {cam.config.camera_name} ready.")

# --------------------------------------------------------------------------- #
# Inspect each camera
# --------------------------------------------------------------------------- #
all_ok = True

for cam in cameras:
    print(f"\n{'='*55}")
    print(f"Camera: {cam.config.camera_name}")
    print(f"{'='*55}")

    # --- Intrinsics ---
    intr = cam.intrinsics
    print(f"\n  Intrinsics:")
    print(f"    Resolution : {intr.width} x {intr.height}  (W x H)")
    print(f"    Focal length : fx={intr.fx:.2f}  fy={intr.fy:.2f}  [px]")
    print(f"    Principal pt : cx={intr.cx:.2f}  cy={intr.cy:.2f}  [px]")
    print(f"    Distortion D : {np.round(intr.D, 5)}")
    print(f"    K matrix:")
    for row in intr.K:
        print(f"      {np.round(row, 4)}")

    # --- Color frame ---
    color = cam.current_color
    print(f"\n  Color frame:")
    print(f"    Shape  : {color.shape}  (H x W x 3, uint8 RGB)")
    print(f"    Range  : [{color.min()}, {color.max()}]")

    # --- Depth frame ---
    depth = cam.current_depth          # float32, metres, NaN where invalid
    depth_mm = cam.current_depth_mm    # uint16, mm, 0 where invalid
    valid_mask = ~np.isnan(depth)
    valid_pct = valid_mask.mean() * 100
    print(f"\n  Depth frame:")
    print(f"    Shape  : {depth.shape}  (H x W, float32 metres)")
    print(f"    Valid pixels : {valid_pct:.1f}%  (non-NaN)")
    if valid_pct > 0:
        d_valid = depth[valid_mask]
        print(f"    Depth range  : [{d_valid.min():.3f}, {d_valid.max():.3f}] m")
        print(f"    Depth mean   : {d_valid.mean():.3f} m")
    else:
        print("    WARNING: all depth pixels are invalid (0 mm)!")
        all_ok = False

    # --- Timestamp alignment ---
    print(f"\n  Timestamp alignment:")
    color_t = cam.color_stamp_ns
    depth_t = cam.depth_stamp_ns
    diff_ms = abs(color_t - depth_t) / 1e6
    print(f"    Color stamp : {color_t} ns")
    print(f"    Depth stamp : {depth_t} ns")
    print(f"    Difference  : {diff_ms:.2f} ms")
    if diff_ms < 50.0:
        print("    Sync OK  (< 50 ms)  — hardware sync is working")
    else:
        print("    WARNING: large timestamp gap — check depth_registration setting!")
        all_ok = False

    # --- Synchronized grab ---
    try:
        c, d, stamp = cam.current_rgbd(max_stamp_diff_ms=50.0)
        print(f"\n  current_rgbd() OK  (stamp={stamp} ns)")
    except RuntimeError as e:
        print(f"\n  current_rgbd() FAILED: {e}")
        all_ok = False

    # --- Deprojection sanity check ---
    cloud = cam.deproject(depth)
    finite = np.isfinite(cloud).all(axis=-1)
    print(f"\n  Deprojection (3D point cloud):")
    print(f"    Shape  : {cloud.shape}  (H x W x 3, float32 metres)")
    print(f"    Finite : {finite.sum()} / {finite.size} points")

# --------------------------------------------------------------------------- #
# Summary
# --------------------------------------------------------------------------- #
print(f"\n{'='*55}")
if all_ok:
    print(f"Stage 6 PASSED — all {len(cameras)} camera(s) operational.")
else:
    print("Stage 6 FAILED — check warnings above.")
