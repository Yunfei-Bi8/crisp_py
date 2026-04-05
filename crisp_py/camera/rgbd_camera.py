"""RGBD camera interface for depth cameras publishing raw Image streams.

Designed for cameras that publish separate color and depth topics as
``sensor_msgs/Image`` (uncompressed), such as the Orbbec Femto Bolt used
on the TUM09 Ridgeback.  Unlike the base ``Camera`` class, this class does
NOT assume CompressedImage — it subscribes to the raw image topics directly.

Key features:
  - Reads color (RGB, uint8), depth (float32, metres), and camera intrinsics
  - Timestamps allow callers to detect whether color and depth arrived together
  - Flexible: one instance per physical camera, any number of cameras
  - Follows the crisp_py Camera API style (is_ready, wait_until_ready, …)

Typical usage (single Orbbec camera)::

    from crisp_py.camera import RgbdCamera, RgbdCameraConfig

    cfg = RgbdCameraConfig.for_orbbec("/camera_01")
    cam = RgbdCamera(config=cfg)
    cam.wait_until_ready()

    color = cam.current_color          # (H, W, 3) uint8 RGB
    depth = cam.current_depth          # (H, W) float32 metres
    K     = cam.intrinsics.K           # (3, 3) calibration matrix

Multiple cameras sharing one ROS2 node::

    import rclpy
    rclpy.init()
    node = rclpy.create_node("data_collector")

    cams = [
        RgbdCamera(config=RgbdCameraConfig.for_orbbec(ns), node=node, spin_node=False)
        for ns in ["/camera_01", "/camera_02", "/camera_03"]
    ]

    # Spin the shared node once in a background thread
    import threading
    import rclpy.executors
    exec_ = rclpy.executors.MultiThreadedExecutor(num_threads=6)
    exec_.add_node(node)
    threading.Thread(target=exec_.spin, daemon=True).start()

    for cam in cams:
        cam.wait_until_ready()
"""

import threading
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np
import rclpy
import rclpy.executors
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from sensor_msgs.msg import CameraInfo, Image


# --------------------------------------------------------------------------- #
#  Camera intrinsics                                                           #
# --------------------------------------------------------------------------- #

@dataclass
class CameraIntrinsics:
    """Pinhole camera intrinsic parameters.

    Populated from the ``sensor_msgs/CameraInfo`` message published by the
    camera driver.  All values are in pixels except ``depth_scale``.

    Attributes:
        K:      (3, 3) float64 intrinsic matrix  [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
        D:      Distortion coefficients (length varies by model; often 5 or 8)
        width:  Image width  [px]
        height: Image height [px]

    Derived properties:
        fx, fy: Focal lengths (pixels)
        cx, cy: Principal point (pixels)
    """

    K: np.ndarray          # shape (3, 3)
    D: np.ndarray          # shape (N,)
    width: int
    height: int

    @property
    def fx(self) -> float:
        """Horizontal focal length [px]."""
        return float(self.K[0, 0])

    @property
    def fy(self) -> float:
        """Vertical focal length [px]."""
        return float(self.K[1, 1])

    @property
    def cx(self) -> float:
        """Principal point x (column) [px]."""
        return float(self.K[0, 2])

    @property
    def cy(self) -> float:
        """Principal point y (row) [px]."""
        return float(self.K[1, 2])

    def deproject(self, depth: np.ndarray) -> np.ndarray:
        """Back-project a depth image to a 3-D point cloud.

        Uses the standard pinhole model::

            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy
            Z = depth[v, u]

        Pixels with depth == 0 (no measurement) produce NaN points.

        Args:
            depth: (H, W) float32 depth image in **metres**.
                   Zero values are treated as invalid (→ NaN).

        Returns:
            (H, W, 3) float32 array of 3-D points in the camera frame
            (x right, y down, z forward).  Invalid pixels are NaN.
        """
        h, w = depth.shape
        # Build pixel-coordinate grids
        u = np.arange(w, dtype=np.float32)
        v = np.arange(h, dtype=np.float32)
        uu, vv = np.meshgrid(u, v)

        valid = depth > 0.0
        Z = np.where(valid, depth, np.nan).astype(np.float32)
        X = (uu - self.cx) * Z / self.fx
        Y = (vv - self.cy) * Z / self.fy

        return np.stack([X, Y, Z], axis=-1)   # (H, W, 3)

    def __repr__(self) -> str:
        return (
            f"CameraIntrinsics(fx={self.fx:.2f}, fy={self.fy:.2f}, "
            f"cx={self.cx:.2f}, cy={self.cy:.2f}, "
            f"size={self.width}×{self.height})"
        )


# --------------------------------------------------------------------------- #
#  Configuration                                                               #
# --------------------------------------------------------------------------- #

@dataclass
class RgbdCameraConfig:
    """Topic names and parameters for one RGBD camera.

    Each physical camera needs its own ``RgbdCameraConfig`` instance.
    Use ``RgbdCameraConfig.for_orbbec(namespace)`` for the Orbbec Femto Bolt
    cameras on the TUM09 Ridgeback.

    Attributes:
        color_topic:      Full topic name for the color stream (``sensor_msgs/Image``)
        depth_topic:      Full topic name for the depth stream (``sensor_msgs/Image``,
                          uint16 millimetres from the Orbbec driver)
        color_info_topic: Full topic name for colour camera calibration
                          (``sensor_msgs/CameraInfo``)
        camera_name:      Human-readable identifier used in log messages.
        depth_scale:      Multiplier to convert raw depth integer values to metres:
                          ``depth_m = raw_uint16 * depth_scale``.
                          Orbbec publishes uint16 millimetres → ``depth_scale = 1/1000``.
        max_delay:        Staleness threshold [s] used by the readiness check.
    """

    color_topic: str
    depth_topic: str
    color_info_topic: str
    camera_name: str = "camera"
    depth_scale: float = 1.0 / 1000.0   # Orbbec uint16 mm → metres
    max_delay: float = 1.0

    @classmethod
    def for_orbbec(cls, namespace: str) -> "RgbdCameraConfig":
        """Build a config for an Orbbec Femto Bolt under the given ROS namespace.

        Orbbec cameras launched via ``orbbec.launch.py`` or
        ``orbbec_multi.launch.py`` publish under::

            /<namespace>/color/image_raw    — RGB Image
            /<namespace>/depth/image_raw    — uint16 mm depth Image
            /<namespace>/color/camera_info  — calibration

        Args:
            namespace: ROS topic namespace, e.g. ``"/camera_01"``.
                       Leading slash is optional.

        Returns:
            RgbdCameraConfig configured for this Orbbec camera.

        Example::

            cfg1 = RgbdCameraConfig.for_orbbec("/camera_01")
            cfg2 = RgbdCameraConfig.for_orbbec("/camera_02")
        """
        ns = namespace if namespace.startswith("/") else f"/{namespace}"
        name = ns.strip("/").replace("/", "_")
        return cls(
            color_topic=f"{ns}/color/image_raw",
            depth_topic=f"{ns}/depth/image_raw",
            color_info_topic=f"{ns}/color/camera_info",
            camera_name=name,
        )


# --------------------------------------------------------------------------- #
#  RgbdCamera                                                                  #
# --------------------------------------------------------------------------- #

class RgbdCamera:
    """High-level interface for a single depth camera publishing raw Image streams.

    Subscribes to three ROS2 topics (colour, depth, camera_info) and provides
    synchronised access to the latest frames and calibration data.

    Colour is stored as ``(H, W, 3)`` uint8 in **RGB** order.
    Depth is stored as ``(H, W)`` float32 in **metres**.
    Raw depth (uint16, mm as published by the Orbbec driver) is also retained
    as ``current_depth_mm`` for callers that need the original values.

    ``current_rgbd()`` returns ``(color, depth)`` only when both frames share
    the same ROS timestamp — use this for temporally aligned data collection.
    If the camera driver hardware-synchronises colour and depth (as the Orbbec
    does with ``depth_registration: true``), colour and depth will always have
    matching stamps and this method will always succeed.

    Thread safety: all internal state is protected by a ``threading.Lock``.
    """

    THREADS_REQUIRED = 3

    def __init__(
        self,
        config: RgbdCameraConfig,
        node: Optional[Node] = None,
        spin_node: bool = True,
        name: str = "rgbd_camera",
    ) -> None:
        """Initialise the RGBD camera subscriber.

        Args:
            config:     Camera configuration (topics, depth scale, …).
            node:       ROS2 node to reuse.  Pass the same node shared with
                        other cameras or with ``RidgebackRobot`` to minimise
                        the number of nodes.  A new node is created when None.
            spin_node:  Whether to spin the node in a background thread.
                        Set to False if you manage spinning externally.
            name:       Node name used only when ``node`` is None.
        """
        self.config = config

        if not rclpy.ok():
            rclpy.init()
        self.node: Node = node if node is not None else rclpy.create_node(name)

        self._bridge = CvBridge()
        self._lock = threading.Lock()

        # Latest frames and their ROS timestamps (nanoseconds)
        self._color: Optional[np.ndarray] = None        # (H, W, 3) uint8 RGB
        self._depth_m: Optional[np.ndarray] = None      # (H, W) float32 metres
        self._depth_mm: Optional[np.ndarray] = None     # (H, W) uint16 mm
        self._intrinsics: Optional[CameraIntrinsics] = None

        self._color_stamp_ns: int = 0
        self._depth_stamp_ns: int = 0

        _cb = ReentrantCallbackGroup()

        # Color subscriber (raw Image, not CompressedImage)
        self.node.create_subscription(
            Image,
            config.color_topic,
            self._cb_color,
            qos_profile_sensor_data,
            callback_group=_cb,
        )

        # Depth subscriber
        self.node.create_subscription(
            Image,
            config.depth_topic,
            self._cb_depth,
            qos_profile_sensor_data,
            callback_group=_cb,
        )

        # Camera info subscriber (calibration — arrives once then rarely changes)
        self.node.create_subscription(
            CameraInfo,
            config.color_info_topic,
            self._cb_camera_info,
            qos_profile_system_default,
            callback_group=_cb,
        )

        if spin_node:
            threading.Thread(target=self._spin_node, daemon=True).start()

    # ------------------------------------------------------------------ #
    #  Internal: spinning                                                  #
    # ------------------------------------------------------------------ #

    def _spin_node(self) -> None:
        """Spin the ROS2 node in a background MultiThreadedExecutor."""
        executor = rclpy.executors.MultiThreadedExecutor(
            num_threads=self.THREADS_REQUIRED
        )
        executor.add_node(self.node)
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    # ------------------------------------------------------------------ #
    #  Internal: callbacks                                                 #
    # ------------------------------------------------------------------ #

    def _cb_color(self, msg: Image) -> None:
        """Decode and store the latest colour frame."""
        try:
            # imgmsg_to_cv2 returns BGR for bgr8; request rgb8 directly
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as exc:
            self.node.get_logger().warn(
                f"[{self.config.camera_name}] Color decode failed: {exc}"
            )
            return
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        with self._lock:
            self._color = frame.copy()
            self._color_stamp_ns = stamp_ns

    def _cb_depth(self, msg: Image) -> None:
        """Decode and store the latest depth frame.

        The Orbbec driver publishes depth as ``uint16`` (encoding ``16UC1``),
        with values in millimetres.  We store both the raw uint16 array and a
        float32 version converted to metres using ``config.depth_scale``.
        """
        try:
            frame_mm = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.node.get_logger().warn(
                f"[{self.config.camera_name}] Depth decode failed: {exc}"
            )
            return
        frame_m = frame_mm.astype(np.float32) * self.config.depth_scale
        # Zero depth means "no measurement" — replace with NaN in float version
        frame_m[frame_mm == 0] = np.nan

        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        with self._lock:
            self._depth_mm = frame_mm.copy()
            self._depth_m = frame_m
            self._depth_stamp_ns = stamp_ns

    def _cb_camera_info(self, msg: CameraInfo) -> None:
        """Parse and cache camera calibration from the CameraInfo message.

        ``msg.k`` is the row-major 3×3 intrinsic matrix stored as a 9-element
        flat array.  ``msg.d`` holds the distortion coefficients.
        """
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64)
        with self._lock:
            self._intrinsics = CameraIntrinsics(
                K=K, D=D, width=msg.width, height=msg.height
            )

    # ------------------------------------------------------------------ #
    #  Public: readiness                                                   #
    # ------------------------------------------------------------------ #

    def is_ready(self) -> bool:
        """Return True when colour, depth, and intrinsics are all available."""
        with self._lock:
            return (
                self._color is not None
                and self._depth_m is not None
                and self._intrinsics is not None
            )

    def wait_until_ready(
        self, timeout: float = 15.0, check_frequency: float = 10.0
    ) -> None:
        """Block until colour, depth, and intrinsics have all been received.

        Args:
            timeout:         Maximum wait time [s].
            check_frequency: Polling rate [Hz].

        Raises:
            TimeoutError: If any stream is still missing after ``timeout`` seconds.
        """
        rate = self.node.create_rate(check_frequency)
        elapsed = 0.0
        while not self.is_ready():
            rate.sleep()
            elapsed += 1.0 / check_frequency
            if elapsed >= timeout:
                with self._lock:
                    missing = []
                    if self._color is None:
                        missing.append(f"color ({self.config.color_topic})")
                    if self._depth_m is None:
                        missing.append(f"depth ({self.config.depth_topic})")
                    if self._intrinsics is None:
                        missing.append(f"camera_info ({self.config.color_info_topic})")
                raise TimeoutError(
                    f"[{self.config.camera_name}] Timed out waiting for: "
                    + ", ".join(missing)
                )

    # ------------------------------------------------------------------ #
    #  Public: colour                                                      #
    # ------------------------------------------------------------------ #

    @property
    def current_color(self) -> np.ndarray:
        """Latest colour frame as an ``(H, W, 3)`` uint8 array in RGB order.

        Raises:
            RuntimeError: If no colour frame has been received yet.
        """
        with self._lock:
            if self._color is None:
                raise RuntimeError(
                    f"[{self.config.camera_name}] No colour frame received yet. "
                    "Call wait_until_ready() first."
                )
            return self._color.copy()

    @property
    def color_stamp_ns(self) -> int:
        """ROS timestamp of the latest colour frame [nanoseconds]."""
        with self._lock:
            return self._color_stamp_ns

    # ------------------------------------------------------------------ #
    #  Public: depth                                                       #
    # ------------------------------------------------------------------ #

    @property
    def current_depth(self) -> np.ndarray:
        """Latest depth frame as an ``(H, W)`` float32 array in **metres**.

        Pixels with no depth measurement are ``NaN``.

        Raises:
            RuntimeError: If no depth frame has been received yet.
        """
        with self._lock:
            if self._depth_m is None:
                raise RuntimeError(
                    f"[{self.config.camera_name}] No depth frame received yet. "
                    "Call wait_until_ready() first."
                )
            return self._depth_m.copy()

    @property
    def current_depth_mm(self) -> np.ndarray:
        """Latest depth frame as an ``(H, W)`` uint16 array in **millimetres**.

        This is the raw value as published by the Orbbec driver.
        Zero means no measurement.

        Raises:
            RuntimeError: If no depth frame has been received yet.
        """
        with self._lock:
            if self._depth_mm is None:
                raise RuntimeError(
                    f"[{self.config.camera_name}] No depth frame received yet. "
                    "Call wait_until_ready() first."
                )
            return self._depth_mm.copy()

    @property
    def depth_stamp_ns(self) -> int:
        """ROS timestamp of the latest depth frame [nanoseconds]."""
        with self._lock:
            return self._depth_stamp_ns

    # ------------------------------------------------------------------ #
    #  Public: intrinsics                                                  #
    # ------------------------------------------------------------------ #

    @property
    def intrinsics(self) -> CameraIntrinsics:
        """Camera calibration (intrinsic matrix K and distortion D).

        Populated from the first ``CameraInfo`` message received.  Does not
        change during normal operation.

        Raises:
            RuntimeError: If no ``CameraInfo`` has been received yet.
        """
        with self._lock:
            if self._intrinsics is None:
                raise RuntimeError(
                    f"[{self.config.camera_name}] No CameraInfo received yet. "
                    "Call wait_until_ready() first."
                )
            intr = self._intrinsics
            return CameraIntrinsics(
                K=intr.K.copy(),
                D=intr.D.copy(),
                width=intr.width,
                height=intr.height,
            )

    # ------------------------------------------------------------------ #
    #  Public: synchronised grab                                           #
    # ------------------------------------------------------------------ #

    def current_rgbd(
        self, max_stamp_diff_ms: float = 50.0
    ) -> Tuple[np.ndarray, np.ndarray, int]:
        """Return the latest colour and depth frames with a shared timestamp.

        When the Orbbec driver runs with ``depth_registration: true``, colour
        and depth are hardware-synchronised and will always share the same
        ROS timestamp.  This method raises ``RuntimeError`` if the timestamps
        differ by more than ``max_stamp_diff_ms`` ms — which would indicate
        that the driver is *not* in hardware-sync mode.

        Args:
            max_stamp_diff_ms: Maximum acceptable time difference between the
                               colour and depth timestamps [milliseconds].
                               Default 50 ms = ½ frame at 10 Hz.

        Returns:
            (color, depth, stamp_ns):
                - color:    ``(H, W, 3)`` uint8 RGB
                - depth:    ``(H, W)`` float32 metres (NaN where invalid)
                - stamp_ns: colour-frame timestamp [nanoseconds]

        Raises:
            RuntimeError: If either stream is not ready, or if the timestamps
                          differ by more than ``max_stamp_diff_ms``.
        """
        with self._lock:
            if self._color is None or self._depth_m is None:
                raise RuntimeError(
                    f"[{self.config.camera_name}] Not ready. "
                    "Call wait_until_ready() first."
                )
            diff_ms = abs(self._color_stamp_ns - self._depth_stamp_ns) / 1e6
            if diff_ms > max_stamp_diff_ms:
                raise RuntimeError(
                    f"[{self.config.camera_name}] Colour/depth timestamps differ "
                    f"by {diff_ms:.1f} ms (threshold: {max_stamp_diff_ms} ms). "
                    "Check that depth_registration is enabled in the camera launch."
                )
            return self._color.copy(), self._depth_m.copy(), self._color_stamp_ns

    # ------------------------------------------------------------------ #
    #  Public: convenience                                                 #
    # ------------------------------------------------------------------ #

    def deproject(self, depth: Optional[np.ndarray] = None) -> np.ndarray:
        """Back-project the depth image to a 3-D point cloud.

        A convenience wrapper around ``intrinsics.deproject()``.

        Args:
            depth: ``(H, W)`` float32 depth in metres.  Uses
                   ``current_depth`` when None.

        Returns:
            ``(H, W, 3)`` float32 point cloud in the camera frame.
        """
        if depth is None:
            depth = self.current_depth
        return self.intrinsics.deproject(depth)

    def __repr__(self) -> str:
        ready = self.is_ready()
        return (
            f"RgbdCamera(name={self.config.camera_name!r}, "
            f"ready={ready}, "
            f"color={self.config.color_topic!r}, "
            f"depth={self.config.depth_topic!r})"
        )


# --------------------------------------------------------------------------- #
#  Factory helpers                                                             #
# --------------------------------------------------------------------------- #

def make_rgbd_camera(
    config: RgbdCameraConfig,
    node: Optional[Node] = None,
    spin_node: bool = True,
    name: str = "rgbd_camera",
) -> RgbdCamera:
    """Factory function to create a single RgbdCamera.

    Mirrors the ``make_camera()`` pattern in crisp_py.

    Args:
        config:    Camera configuration (use ``RgbdCameraConfig.for_orbbec()``).
        node:      ROS2 node to reuse.
        spin_node: Whether to spin the node in a background thread.
        name:      Node name (used only when ``node`` is None).

    Returns:
        Fully initialised ``RgbdCamera`` instance.
    """
    return RgbdCamera(config=config, node=node, spin_node=spin_node, name=name)


def make_rgbd_cameras(
    configs: list,
    node: Optional[Node] = None,
    spin_node: bool = True,
    shared_node_name: str = "rgbd_cameras",
) -> list:
    """Create multiple RgbdCamera instances that share one ROS2 node.

    Sharing a node reduces overhead when subscribing to many cameras
    simultaneously.  A single ``MultiThreadedExecutor`` (spun in one
    background thread) handles all subscriptions.

    Args:
        configs:          List of ``RgbdCameraConfig`` instances, one per
                          physical camera.
        node:             Existing ROS2 node to reuse.  A new shared node is
                          created when None.
        spin_node:        Whether to spin the shared node in a background thread.
        shared_node_name: Name for the auto-created node (used only when
                          ``node`` is None).

    Returns:
        List of ``RgbdCamera`` instances in the same order as ``configs``.

    Example::

        cameras = make_rgbd_cameras([
            RgbdCameraConfig.for_orbbec("/camera_01"),
            RgbdCameraConfig.for_orbbec("/camera_02"),
            RgbdCameraConfig.for_orbbec("/camera_03"),
        ])
        for cam in cameras:
            cam.wait_until_ready()
    """
    if not rclpy.ok():
        rclpy.init()

    shared_node: Node = node if node is not None else rclpy.create_node(shared_node_name)

    cameras = [
        RgbdCamera(config=cfg, node=shared_node, spin_node=False)
        for cfg in configs
    ]

    if spin_node:
        n_threads = max(3 * len(cameras), 4)   # at least 3 threads per camera
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=n_threads)
        executor.add_node(shared_node)
        threading.Thread(
            target=lambda: _spin_executor(executor),
            daemon=True,
            name="rgbd_cameras_spin",
        ).start()

    return cameras


def _spin_executor(executor: rclpy.executors.MultiThreadedExecutor) -> None:
    """Spin helper for the shared executor background thread."""
    while rclpy.ok():
        executor.spin_once(timeout_sec=0.1)
