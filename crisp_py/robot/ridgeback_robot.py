"""High-level interface for the Clearpath Ridgeback mobile manipulation platform.

This module provides RidgebackRobot, a self-contained class that mirrors the
crisp_py Robot API but targets the Ridgeback's UR10e arm and Robotiq 2F-85
gripper.

Two control modes are supported, selectable at runtime:

* **JointTrajectory mode** (default) — all robots and simulation.
  Uses ``FollowJointTrajectory`` action server.  Pinocchio IK runs client-side.
  Methods: ``move_joints()``, ``move_to()``, ``move_to_async()``.

* **CartesianController mode** — real robot only (requires the CRISP
  ``cartesian_controller`` to be active via ``ros2 control switch_controllers``).
  Sends ``PoseStamped`` directly to the controller; impedance control runs
  on the robot side, no client-side IK needed.
  Method: ``move_cartesian_async()``.

Requires:
    pinocchio  -- install via:  pixi add pinocchio  (inside the jazzy env)
"""

import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import numpy as np
import rclpy
import rclpy.executors
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

try:
    import pinocchio as pin
except ImportError as exc:
    raise ImportError(
        "pinocchio is required for RidgebackRobot but is not installed.\n"
        "Install it with:  conda install -c conda-forge pinocchio\n"
        "or add it to your pixi.toml:  pinocchio = '*'"
    ) from exc

from crisp_py.robot.ridgeback_controller_switcher import RidgebackControllerSwitcher
from crisp_py.utils.geometry import Pose


# --------------------------------------------------------------------------- #
#  Configuration                                                               #
# --------------------------------------------------------------------------- #

_DEFAULT_ARM_JOINTS: List[str] = [
    "arm_0_shoulder_pan_joint",
    "arm_0_shoulder_lift_joint",
    "arm_0_elbow_joint",
    "arm_0_wrist_1_joint",
    "arm_0_wrist_2_joint",
    "arm_0_wrist_3_joint",
]

_DEFAULT_HOME_CONFIG: List[float] = [1.57, -1.57, 1.57, -1.57, -1.57, 0.0]


@dataclass
class RidgebackConfig:
    """All tuneable parameters for one RidgebackRobot instance.

    Two factory methods cover the standard environments:

    * ``RidgebackConfig.for_real_robot()`` — TUM09 r100-0207 Clearpath bringup.
    * ``RidgebackConfig.for_simulation()`` — clearpath_simulation MuJoCo stack.

    Individual fields can be overridden after construction::

        cfg = RidgebackConfig.for_real_robot()
        cfg.ik_max_iter = 4000
    """

    # ------------------------------------------------------------------ #
    #  ROS2 topic / action names                                          #
    # ------------------------------------------------------------------ #

    joint_states_topic: str = "/r100_0207/platform/joint_states"
    """Topic on which combined platform joint states are published."""

    arm_action: str = (
        "/r100_0207/manipulators"
        "/arm_0_joint_trajectory_controller"
        "/follow_joint_trajectory"
    )
    """FollowJointTrajectory action server for the UR10e arm."""

    gripper_action: Optional[str] = (
        "/r100_0207/manipulators/arm_0_gripper_controller/gripper_cmd"
    )
    """GripperCommand action server.  Set to None when unavailable (e.g. sim)."""

    cartesian_target_topic: Optional[str] = (
        "/r100_0207/manipulators/cartesian_controller/target_pose"
    )
    """PoseStamped topic consumed by the CRISP CartesianController.
    Set to None when the CartesianController is not loaded.
    The controller must be activated via ``ros2 control switch_controllers``
    before calling ``move_cartesian_async()``.
    """

    # ------------------------------------------------------------------ #
    #  Kinematic configuration                                            #
    # ------------------------------------------------------------------ #

    arm_joint_names: List[str] = field(
        default_factory=lambda: list(_DEFAULT_ARM_JOINTS)
    )
    """UR10e joint names as published by the bringup (arm_0_ prefix)."""

    ee_frame: str = "arm_0_tool0"
    """TF frame name of the UR10e tool flange in the URDF."""

    base_frame: str = "arm_0_base_link"
    """Base frame used in PoseStamped headers for the CartesianController."""

    # ------------------------------------------------------------------ #
    #  Default poses                                                      #
    # ------------------------------------------------------------------ #

    home_config: List[float] = field(
        default_factory=lambda: list(_DEFAULT_HOME_CONFIG)
    )
    """Home joint configuration [rad].  Matches 'home' keyframe in MJCF."""

    gripper_open_rad: float = 0.0
    """Robotiq 2F-85 knuckle angle for fully open [rad]."""

    gripper_closed_rad: float = 0.8
    """Robotiq 2F-85 knuckle angle for fully closed [rad]."""

    # ------------------------------------------------------------------ #
    #  Controller manager (for programmatic controller switching)        #
    # ------------------------------------------------------------------ #

    controller_manager_topic: Optional[str] = (
        "/r100_0207/manipulators/controller_manager"
    )
    """Full namespace of the ROS2 controller_manager node.
    Used to call the ``switch_controller`` service programmatically.
    Set to None to disable controller switching (CLI only).
    """

    cartesian_controller_name: str = "cartesian_controller"
    """Name of the CRISP CartesianController as registered in ros2_control."""

    joint_trajectory_controller_name: str = "arm_0_joint_trajectory_controller"
    """Name of the FollowJointTrajectory controller as registered in ros2_control."""

    # ------------------------------------------------------------------ #
    #  Timeouts                                                           #
    # ------------------------------------------------------------------ #

    action_server_timeout: float = 10.0
    """Seconds to wait for an action server before raising TimeoutError."""

    # ------------------------------------------------------------------ #
    #  Pinocchio IK tuning                                                #
    # ------------------------------------------------------------------ #

    ik_max_iter: int = 2000
    """Maximum Levenberg-Marquardt iterations."""

    ik_eps: float = 1e-5
    """Convergence threshold on the 6-D error norm."""

    ik_dt: float = 0.5
    """Step size for the damped Jacobian update."""

    ik_damp: float = 1e-6
    """Levenberg-Marquardt damping coefficient."""

    # ------------------------------------------------------------------ #
    #  Factory methods                                                    #
    # ------------------------------------------------------------------ #

    @classmethod
    def for_real_robot(cls) -> "RidgebackConfig":
        """Configuration for the TUM09 r100-0207 Clearpath bringup.

        Topics are namespaced under ``/r100_0207/manipulators/`` as defined
        in ``robot.yaml``.  The CartesianController topic is included; the
        controller must still be activated via ``ros2 control switch_controllers``
        before ``move_cartesian_async()`` can be used.
        """
        return cls()

    @classmethod
    def for_simulation(cls) -> "RidgebackConfig":
        """Configuration for the clearpath_simulation MuJoCo ros2_control stack.

        Uses un-namespaced topic names as launched by
        ``sim_ros2/sim_bringup.launch.py``.  The gripper action server does
        not exist in simulation (``gripper_action=None``).  The CartesianController
        is available in simulation with no namespace prefix.
        """
        return cls(
            joint_states_topic="/joint_states",
            arm_action="/joint_trajectory_controller/follow_joint_trajectory",
            gripper_action=None,
            cartesian_target_topic="/cartesian_controller/target_pose",
            controller_manager_topic="/controller_manager",
            joint_trajectory_controller_name="joint_trajectory_controller",
        )


# --------------------------------------------------------------------------- #
#  Main class                                                                  #
# --------------------------------------------------------------------------- #

class RidgebackRobot:
    """High-level interface for the Clearpath Ridgeback mobile manipulation platform.

    Controls the UR10e arm and Robotiq 2F-85 gripper.  Two motion modes:

    **JointTrajectory mode** (always available)::

        robot.move_joints([q1, q2, q3, q4, q5, q6], duration=5.0)  # joint space
        robot.move_to(pose, duration=3.0)                           # Cartesian, blocking
        robot.move_to_async(pose, duration=0.15)                    # Cartesian, streaming

    **CartesianController mode** (real robot and simulation, requires CRISP controller
    to be active via ``ros2 control switch_controllers``)::

        robot.move_cartesian_async(pose)   # publish PoseStamped, 10-50 Hz

    Thread safety: all joint-state reads are protected by an internal lock.
    Pinocchio FK and IK each create their own Data object, so concurrent FK
    queries (e.g. from a logging thread) are safe.
    """

    THREADS_REQUIRED = 4

    # ------------------------------------------------------------------ #
    #  Initialisation                                                     #
    # ------------------------------------------------------------------ #

    def __init__(
        self,
        urdf_path: str,
        config: Optional[RidgebackConfig] = None,
        node: Optional[Node] = None,
        spin_node: bool = True,
        name: str = "ridgeback_robot",
    ) -> None:
        """Initialise the RidgebackRobot interface.

        Args:
            urdf_path:  Absolute path to the robot URDF file.
            config:     Environment configuration.  Defaults to
                        ``RidgebackConfig.for_real_robot()`` when None.
            node:       Existing ROS2 node to reuse (e.g. shared with cameras).
            spin_node:  Whether to spin the node in a background thread.
            name:       Node name (used only when ``node`` is None).
        """
        self._cfg: RidgebackConfig = config if config is not None else RidgebackConfig()

        if not rclpy.ok():
            rclpy.init()

        self.node: Node = node if node is not None else rclpy.create_node(name)

        # --- Pinocchio arm model (Data objects are created per-call, not shared) ---
        self._arm_model = self._build_arm_model(str(urdf_path))
        self._ee_id: int = self._arm_model.getFrameId(self._cfg.ee_frame)

        # --- Joint state (protected by _js_lock) ---
        self._js_lock = threading.Lock()
        self._current_joint: Optional[np.ndarray] = None           # (6,) rad
        self._current_joint_velocity: Optional[np.ndarray] = None  # (6,) rad/s
        self._gripper_position: Optional[float] = None             # knuckle rad
        self._joint_state_stamp_ns: int = 0                        # ROS timestamp ns

        # --- Current trajectory goal handle (for cancel_motion) ---
        self._goal_handle_lock = threading.Lock()
        self._current_goal_handle = None

        # --- Action clients ---
        _cb = ReentrantCallbackGroup()
        self._arm_action_client = ActionClient(
            self.node, FollowJointTrajectory, self._cfg.arm_action, callback_group=_cb
        )

        if self._cfg.gripper_action is not None:
            self._gripper_action_client: Optional[ActionClient] = ActionClient(
                self.node, GripperCommand, self._cfg.gripper_action, callback_group=_cb
            )
        else:
            self._gripper_action_client = None

        # --- CartesianController publisher ---
        if self._cfg.cartesian_target_topic is not None:
            self._cartesian_pub = self.node.create_publisher(
                PoseStamped,
                self._cfg.cartesian_target_topic,
                10,
            )
        else:
            self._cartesian_pub = None

        # --- Controller switcher (delegates to RidgebackControllerSwitcher) ---
        self._ctrl_switcher = RidgebackControllerSwitcher(
            node=self.node,
            controller_manager_topic=self._cfg.controller_manager_topic,
            cartesian_controller_name=self._cfg.cartesian_controller_name,
            joint_trajectory_controller_name=self._cfg.joint_trajectory_controller_name,
            service_timeout=self._cfg.action_server_timeout,
        )

        # --- Joint state subscriber ---
        self.node.create_subscription(
            JointState,
            self._cfg.joint_states_topic,
            self._cb_joint_state,
            qos_profile_sensor_data,
            callback_group=ReentrantCallbackGroup(),
        )

        if spin_node:
            threading.Thread(target=self._spin_node, daemon=True).start()

    # ------------------------------------------------------------------ #
    #  Internal: node spinning                                            #
    # ------------------------------------------------------------------ #

    def _spin_node(self) -> None:
        executor = rclpy.executors.MultiThreadedExecutor(
            num_threads=self.THREADS_REQUIRED
        )
        executor.add_node(self.node)
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    # ------------------------------------------------------------------ #
    #  Internal: Pinocchio model                                          #
    # ------------------------------------------------------------------ #

    def _build_arm_model(self, urdf_path: str) -> "pin.Model":
        """Build a reduced Pinocchio model with only the 6 arm joints.

        All non-arm joints (wheels, fingers, sensor mounts) are locked at
        their neutral positions.  Returns the model only — each FK/IK call
        creates its own Data object so concurrent calls are safe.
        """
        if not Path(urdf_path).exists():
            raise FileNotFoundError(
                f"URDF file not found: {urdf_path}\n"
                "Generate it from xacro:\n"
                "  xacro robot.urdf.xacro > robot.urdf"
            )

        full_model = pin.buildModelFromUrdf(urdf_path)
        arm_joint_set = set(self._cfg.arm_joint_names)
        joints_to_lock = [
            full_model.getJointId(name)
            for name in full_model.names[1:]   # skip 'universe'
            if name not in arm_joint_set
        ]
        q0 = pin.neutral(full_model)
        return pin.buildReducedModel(full_model, joints_to_lock, q0)

    def _q_from_joint_list(self, joint_positions: List[float]) -> np.ndarray:
        """Pack 6 arm joint values into a Pinocchio configuration vector."""
        q = pin.neutral(self._arm_model)
        for name, value in zip(self._cfg.arm_joint_names, joint_positions):
            jid = self._arm_model.getJointId(name)
            q[self._arm_model.joints[jid].idx_q] = value
        return q

    def _joint_list_from_q(self, q: np.ndarray) -> List[float]:
        """Extract the 6 arm joint values from a Pinocchio configuration vector."""
        return [
            float(q[self._arm_model.joints[self._arm_model.getJointId(n)].idx_q])
            for n in self._cfg.arm_joint_names
        ]

    # ------------------------------------------------------------------ #
    #  Internal: FK and IK (each call owns its Pinocchio Data object)     #
    # ------------------------------------------------------------------ #

    def _fk(self, joint_positions: List[float]) -> "pin.SE3":
        """Forward kinematics → EE pose (Pinocchio SE3, base_link frame).

        Creates a fresh Data object per call so this method is safe to call
        from multiple threads concurrently (e.g. a logging thread and the
        teleoperation loop).
        """
        q = self._q_from_joint_list(joint_positions)
        data = self._arm_model.createData()
        pin.forwardKinematics(self._arm_model, data, q)
        pin.updateFramePlacements(self._arm_model, data)
        return data.oMf[self._ee_id].copy()

    def _solve_ik(
        self, target_se3: "pin.SE3", q_init: np.ndarray
    ) -> tuple:
        """Numerical IK — damped Jacobian pseudo-inverse (Levenberg-Marquardt).

        Creates one Data object per IK call (allocated once, reused over all
        iterations) so concurrent IK solves are also safe.

        Returns:
            (q_solution, converged)
        """
        data = self._arm_model.createData()   # one allocation per IK call
        q = q_init.copy()

        for _ in range(self._cfg.ik_max_iter):
            pin.forwardKinematics(self._arm_model, data, q)
            pin.updateFramePlacements(self._arm_model, data)

            oMee = data.oMf[self._ee_id]
            err = pin.log6(oMee.inverse() * target_se3).vector
            if np.linalg.norm(err) < self._cfg.ik_eps:
                return q, True

            pin.computeJointJacobians(self._arm_model, data, q)
            J = pin.getFrameJacobian(
                self._arm_model, data, self._ee_id, pin.ReferenceFrame.LOCAL,
            )
            JJT = J @ J.T + self._cfg.ik_damp * np.eye(6)
            dq = J.T @ np.linalg.solve(JJT, err)
            q = pin.integrate(self._arm_model, q, self._cfg.ik_dt * dq)
            q = np.clip(
                q,
                self._arm_model.lowerPositionLimit,
                self._arm_model.upperPositionLimit,
            )

        return q, False

    @staticmethod
    def _pose_to_se3(pose: Pose) -> "pin.SE3":
        return pin.SE3(pose.orientation.as_matrix(), pose.position.copy())

    @staticmethod
    def _se3_to_pose(se3: "pin.SE3") -> Pose:
        return Pose(
            position=se3.translation.copy(),
            orientation=Rotation.from_matrix(se3.rotation.copy()),
        )

    # ------------------------------------------------------------------ #
    #  Internal: ROS2 callbacks                                           #
    # ------------------------------------------------------------------ #

    def _cb_joint_state(self, msg: JointState) -> None:
        """Store the latest arm joint state and its ROS timestamp.

        Protected by ``_js_lock`` so reads from any thread are consistent.
        Gripper position is detected by keyword matching on joint names
        ('knuckle', 'finger', 'robotiq').
        """
        pos_map: dict = dict(zip(msg.name, msg.position))
        vel_map: dict = (
            dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        )
        arm_joint_set = set(self._cfg.arm_joint_names)

        # ROS timestamp of this message (nanoseconds)
        stamp_ns = (
            msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        )

        with self._js_lock:
            if all(j in pos_map for j in self._cfg.arm_joint_names):
                self._current_joint = np.array(
                    [pos_map[j] for j in self._cfg.arm_joint_names],
                    dtype=np.float64,
                )
                self._current_joint_velocity = np.array(
                    [vel_map.get(j, 0.0) for j in self._cfg.arm_joint_names],
                    dtype=np.float64,
                )
                self._joint_state_stamp_ns = stamp_ns

            # Gripper — best-effort keyword match
            _kws = ("knuckle", "finger", "robotiq")
            for name, pos in pos_map.items():
                if name not in arm_joint_set and any(
                    kw in name.lower() for kw in _kws
                ):
                    self._gripper_position = pos
                    break

    # ------------------------------------------------------------------ #
    #  Internal: action helpers                                           #
    # ------------------------------------------------------------------ #

    def _wait_for_arm_server(self) -> None:
        """Wait for the arm action server, raising TimeoutError on timeout."""
        if not self._arm_action_client.wait_for_server(
            timeout_sec=self._cfg.action_server_timeout
        ):
            raise TimeoutError(
                f"Arm action server '{self._cfg.arm_action}' not available "
                f"after {self._cfg.action_server_timeout}s. "
                "Is the controller running?"
            )

    def _send_joint_trajectory(
        self,
        joint_positions: List[float],
        duration: float,
        blocking: bool = True,
    ) -> bool:
        """Send a FollowJointTrajectory goal to the arm action server.

        Stores the accepted goal handle in ``_current_goal_handle`` so that
        ``cancel_motion()`` can cancel it later.

        Args:
            joint_positions: Target angles for the 6 arm joints [rad].
            duration:        Time allocated for the motion [s].
            blocking:        If True, wait until the trajectory completes.
                             If False, return as soon as the goal is accepted
                             (suitable for streaming teleoperation).

        Returns:
            True if the goal was accepted (and, when blocking, completed
            without error).
        """
        self._wait_for_arm_server()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self._cfg.arm_joint_names
        point = JointTrajectoryPoint()
        point.positions = list(joint_positions)
        point.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration % 1) * 1e9),
        )
        goal.trajectory.points = [point]

        done = threading.Event()
        result: dict = {}

        def _on_goal_response(future):
            handle = future.result()
            if not handle.accepted:
                result["accepted"] = False
                done.set()
                return
            result["accepted"] = True
            # Store so cancel_motion() can reference it
            with self._goal_handle_lock:
                self._current_goal_handle = handle
            if blocking:
                res_future = handle.get_result_async()
                res_future.add_done_callback(_on_result)
            else:
                done.set()

        def _on_result(future):
            result["error_code"] = future.result().result.error_code
            with self._goal_handle_lock:
                self._current_goal_handle = None
            done.set()

        send_future = self._arm_action_client.send_goal_async(goal)
        send_future.add_done_callback(_on_goal_response)
        done.wait()

        if not result.get("accepted", False):
            self.node.get_logger().warn(
                "Arm trajectory goal was rejected by the action server."
            )
            return False
        if blocking:
            return result.get("error_code", -1) == 0
        return True

    def _send_gripper_command(
        self,
        position: float,
        max_effort: float = 50.0,
        blocking: bool = True,
    ) -> bool:
        """Send a GripperCommand goal to the Robotiq action server.

        Returns False immediately (with a warning) if no gripper action is
        configured (simulation).
        """
        if self._gripper_action_client is None:
            self.node.get_logger().warn(
                "Gripper command ignored: gripper_action=None in RidgebackConfig."
            )
            return False

        if not self._gripper_action_client.wait_for_server(
            timeout_sec=self._cfg.action_server_timeout
        ):
            raise TimeoutError(
                f"Gripper action server '{self._cfg.gripper_action}' not available "
                f"after {self._cfg.action_server_timeout}s."
            )

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)

        done = threading.Event()
        result: dict = {}

        def _on_goal_response(future):
            handle = future.result()
            result["accepted"] = handle.accepted
            if not handle.accepted:
                done.set()
                return
            if blocking:
                res_future = handle.get_result_async()
                res_future.add_done_callback(lambda _: done.set())
            else:
                done.set()

        send_future = self._gripper_action_client.send_goal_async(goal)
        send_future.add_done_callback(_on_goal_response)
        done.wait()
        return result.get("accepted", False)

    # ------------------------------------------------------------------ #
    #  Public: readiness                                                  #
    # ------------------------------------------------------------------ #

    def is_ready(self) -> bool:
        """Return True if at least one joint state message has been received."""
        with self._js_lock:
            return self._current_joint is not None

    def wait_until_ready(
        self, timeout: float = 10.0, check_frequency: float = 10.0
    ) -> None:
        """Block until arm joint states are being received.

        Raises:
            TimeoutError: If no joint states arrive within ``timeout`` seconds.
        """
        rate = self.node.create_rate(check_frequency)
        elapsed = 0.0
        while not self.is_ready():
            rate.sleep()
            elapsed += 1.0 / check_frequency
            if elapsed >= timeout:
                raise TimeoutError(
                    f"Timed out waiting for joint states on "
                    f"'{self._cfg.joint_states_topic}'.\n"
                    "Is the bringup (or simulation) running and the ROS2 "
                    "network reachable?"
                )

    # ------------------------------------------------------------------ #
    #  Public: state properties                                           #
    # ------------------------------------------------------------------ #

    @property
    def nq(self) -> int:
        """Number of arm joints (always 6 for the UR10e)."""
        return len(self._cfg.arm_joint_names)

    @property
    def joint_values(self) -> np.ndarray:
        """Current 6 arm joint positions [rad], shape (6,).

        Raises:
            RuntimeError: If no joint states have been received yet.
        """
        with self._js_lock:
            if self._current_joint is None:
                raise RuntimeError(
                    "No joint states received yet. Call wait_until_ready() first."
                )
            return self._current_joint.copy()

    @property
    def joint_velocities(self) -> np.ndarray:
        """Current 6 arm joint velocities [rad/s], shape (6,).

        Raises:
            RuntimeError: If no joint states have been received yet.
        """
        with self._js_lock:
            if self._current_joint_velocity is None:
                raise RuntimeError(
                    "No joint states received yet. Call wait_until_ready() first."
                )
            return self._current_joint_velocity.copy()

    @property
    def joint_state_stamp_ns(self) -> int:
        """ROS timestamp of the latest joint state message [nanoseconds].

        Useful for synchronising robot state with camera frames during data
        collection.  Zero until the first joint state arrives.
        """
        with self._js_lock:
            return self._joint_state_stamp_ns

    @property
    def end_effector_pose(self) -> Pose:
        """Current EE pose in the robot base_link frame (Pinocchio FK).

        Thread-safe: reads joint_values under the lock, then runs FK with
        a local Data object.

        Raises:
            RuntimeError: If no joint states have been received yet.
        """
        return self._se3_to_pose(self._fk(self.joint_values.tolist()))

    @property
    def home_config(self) -> List[float]:
        """Home joint configuration [rad] from the active RidgebackConfig."""
        return list(self._cfg.home_config)

    def is_homed(self) -> bool:
        """Return True if the arm is at the home configuration (±0.01 rad)."""
        return np.allclose(self.joint_values, self._cfg.home_config, atol=1e-2)

    @property
    def gripper_position(self) -> Optional[float]:
        """Current Robotiq 2F-85 knuckle angle [rad], or None if unknown.

        Range: 0.0 = fully open, 0.8 = fully closed.
        """
        with self._js_lock:
            return self._gripper_position

    @property
    def gripper_is_open(self) -> bool:
        """True if the gripper is closer to open than to closed.

        Raises:
            RuntimeError: If gripper joint state is not available.
        """
        pos = self.gripper_position   # acquires _js_lock internally
        if pos is None:
            raise RuntimeError(
                "Gripper state not available. "
                "Check that the gripper joint name matches 'knuckle', 'finger', "
                "or 'robotiq'."
            )
        return pos < self._cfg.gripper_closed_rad / 2.0

    # ------------------------------------------------------------------ #
    #  Public: arm motion — JointTrajectory mode                          #
    # ------------------------------------------------------------------ #

    def move_joints(
        self, joint_positions: List[float], duration: float = 5.0
    ) -> bool:
        """Move to a joint configuration (blocking, JointTrajectory mode).

        Args:
            joint_positions: Target angles for the 6 joints [rad].
            duration:        Time budget [s].

        Returns:
            True if the trajectory completed without errors.
        """
        assert len(joint_positions) == 6, (
            f"Expected 6 joint positions, got {len(joint_positions)}."
        )
        return self._send_joint_trajectory(joint_positions, duration, blocking=True)

    def home(self, duration: float = 8.0) -> bool:
        """Move the arm to the home configuration (blocking).

        Args:
            duration: Time budget [s].

        Returns:
            True if the trajectory completed without errors.
        """
        return self.move_joints(self._cfg.home_config, duration=duration)

    def move_to(self, pose: Pose, duration: float = 5.0) -> bool:
        """Move the EE to a Cartesian pose (blocking, JointTrajectory mode).

        Solves Pinocchio IK client-side, then sends a FollowJointTrajectory
        goal.  Blocks until the motion completes.

        Args:
            pose:     Target EE pose in the base_link frame.
            duration: Time budget [s].

        Returns:
            True if the trajectory completed without errors.
        """
        q_init = self._q_from_joint_list(self.joint_values.tolist())
        q_sol, converged = self._solve_ik(self._pose_to_se3(pose), q_init)
        if not converged:
            self.node.get_logger().warn(
                "Pinocchio IK did not converge. Executing best solution."
            )
        return self._send_joint_trajectory(
            self._joint_list_from_q(q_sol), duration, blocking=True
        )

    def move_to_async(self, pose: Pose, duration: float = 0.15) -> None:
        """Stream a short trajectory toward a Cartesian pose (non-blocking).

        Intended for teleoperation at 10-20 Hz via the JointTrajectory
        controller.  Each call solves IK and dispatches a goal immediately,
        returning before the motion completes.  The UR controller preempts
        the previous goal when the new one arrives.

        Typical loop::

            while running:
                robot.move_to_async(desired_pose, duration=0.15)
                time.sleep(0.05)   # ~20 Hz

        Args:
            pose:     Target EE pose in the base_link frame.
            duration: Trajectory horizon [s].  Keep 0.1-0.2 s for responsive
                      teleoperation.
        """
        q_init = self._q_from_joint_list(self.joint_values.tolist())
        q_sol, _ = self._solve_ik(self._pose_to_se3(pose), q_init)
        self._send_joint_trajectory(
            self._joint_list_from_q(q_sol), duration, blocking=False
        )

    def cancel_motion(self, wait: bool = False) -> bool:
        """Cancel the currently executing arm trajectory.

        Sends a cancel request to the UR action server for the last accepted
        goal.  The robot will decelerate to a stop (UR firmware behaviour).

        Args:
            wait: If True, block until the cancel acknowledgement arrives
                  (up to 2 s).  If False, fire-and-forget.

        Returns:
            True if a goal was active and the cancel was sent.
            False if there was no active goal.
        """
        with self._goal_handle_lock:
            handle = self._current_goal_handle
            self._current_goal_handle = None

        if handle is None:
            return False

        cancel_future = handle.cancel_goal_async()
        if wait:
            done = threading.Event()
            cancel_future.add_done_callback(lambda _: done.set())
            done.wait(timeout=2.0)
        return True

    # ------------------------------------------------------------------ #
    #  Public: controller switching                                       #
    # ------------------------------------------------------------------ #

    def switch_to_cartesian_controller(self) -> bool:
        """Deactivate the JointTrajectory controller and activate the CartesianController.

        Required before calling ``move_cartesian_async()``.  After switching,
        the arm is controlled by the CRISP impedance CartesianController; the
        FollowJointTrajectory action server is no longer available until you
        call ``switch_to_joint_trajectory_controller()`` again.

        Returns:
            True if the switch was accepted successfully.

        Raises:
            RuntimeError: If ``controller_manager_topic`` is None in config.
            TimeoutError: If the service is unreachable.

        Example::

            robot.switch_to_cartesian_controller()
            while running:
                robot.move_cartesian_async(target_pose)
                time.sleep(0.02)   # 50 Hz
            robot.switch_to_joint_trajectory_controller()
        """
        return self._ctrl_switcher.switch_to_cartesian()

    def switch_to_joint_trajectory_controller(self) -> bool:
        """Deactivate the CartesianController and restore the JointTrajectory controller.

        Call this after a CartesianController teleoperation session to restore
        joint-space control.

        Returns:
            True if the switch was accepted successfully.

        Raises:
            RuntimeError: If ``controller_manager_topic`` is None in config.
            TimeoutError: If the service is unreachable.
        """
        return self._ctrl_switcher.switch_to_joint_trajectory()

    # ------------------------------------------------------------------ #
    #  Public: arm motion — CartesianController mode                      #
    # ------------------------------------------------------------------ #

    def move_cartesian_async(self, pose: Pose) -> None:
        """Publish a Cartesian target to the CRISP CartesianController.

        This is the recommended method for smooth teleoperation.  Unlike
        ``move_to_async()``, there is no client-side IK: the pose is sent
        directly to the impedance controller running on the robot, which
        tracks it with a spring-damper law.

        **Prerequisites:**
          1. The CartesianController must be active.  Use the programmatic API::

                robot.switch_to_cartesian_controller()

             or the CLI equivalent::

                CM=/r100_0207/manipulators/controller_manager
                ros2 control switch_controllers \\
                    --activate cartesian_controller \\
                    --deactivate arm_0_joint_trajectory_controller -c $CM

          2. ``config.cartesian_target_topic`` must not be None.

        **Teleoperation pattern (50 Hz recommended):**

            robot.switch_to_cartesian_controller()
            while running:
                robot.move_cartesian_async(desired_pose)
                time.sleep(0.02)
            robot.switch_to_joint_trajectory_controller()

        The controller stiffness and error-clip values are set in
        ``crisp_controllers.yaml``.  Start conservatively (k_pos ≈ 200 N/m)
        and increase only after verifying stable behaviour.

        Args:
            pose: Target EE pose in the base_link frame (same coordinate
                  frame as ``end_effector_pose``).

        Raises:
            RuntimeError: If ``cartesian_target_topic`` is None in the
                          current configuration.
        """
        if self._cartesian_pub is None:
            raise RuntimeError(
                "CartesianController mode is not configured. "
                "Set cartesian_target_topic in RidgebackConfig."
            )
        msg = pose.to_ros_msg(
            frame_id=self._cfg.base_frame,
            stamp=self.node.get_clock().now().to_msg(),
        )
        self._cartesian_pub.publish(msg)

    # ------------------------------------------------------------------ #
    #  Public: gripper                                                    #
    # ------------------------------------------------------------------ #

    def gripper_open(self, max_effort: float = 50.0) -> bool:
        """Open the Robotiq 2F-85 gripper fully (blocking).

        Returns False if no gripper action is configured (simulation).
        """
        return self._send_gripper_command(
            self._cfg.gripper_open_rad, max_effort=max_effort, blocking=True
        )

    def gripper_close(self, max_effort: float = 50.0) -> bool:
        """Close the Robotiq 2F-85 gripper fully (blocking).

        Returns False if no gripper action is configured (simulation).
        """
        return self._send_gripper_command(
            self._cfg.gripper_closed_rad, max_effort=max_effort, blocking=True
        )

    def gripper_set(self, position: float, max_effort: float = 50.0) -> bool:
        """Set the gripper to a specific knuckle angle (blocking).

        Args:
            position:   Target knuckle angle [rad].  0.0 = open, 0.8 = closed.
            max_effort: Maximum gripping force [N].

        Raises:
            ValueError: If position is outside [gripper_open_rad, gripper_closed_rad].
        """
        lo, hi = self._cfg.gripper_open_rad, self._cfg.gripper_closed_rad
        if not lo <= position <= hi:
            raise ValueError(
                f"Gripper position {position:.3f} rad is outside [{lo}, {hi}]."
            )
        return self._send_gripper_command(position, max_effort=max_effort, blocking=True)

    # ------------------------------------------------------------------ #
    #  Public: lifecycle                                                  #
    # ------------------------------------------------------------------ #

    def shutdown(self) -> None:
        """Shut down the ROS2 node."""
        if rclpy.ok():
            rclpy.shutdown()


# --------------------------------------------------------------------------- #
#  Module-level helpers                                                        #
# --------------------------------------------------------------------------- #

def get_ridgeback_urdf_path(clearpath_ws: "str | Path") -> str:
    """Return the path to robot.urdf, generating it from xacro if necessary.

    Args:
        clearpath_ws: Root of the clearpath_remote_ws directory.

    Returns:
        Absolute path string to the ready-to-use ``robot.urdf`` file.

    Raises:
        FileNotFoundError: If the xacro source does not exist.
    """
    config_dir = (
        Path(clearpath_ws)
        / "src" / "tum09_ridgeback" / "tum09_bringup" / "config"
    )
    xacro_path = config_dir / "robot.urdf.xacro"
    urdf_path = config_dir / "robot.urdf"

    if not xacro_path.exists():
        raise FileNotFoundError(
            f"Xacro source not found: {xacro_path}\n"
            "Run the Clearpath description generator first:\n"
            "  ros2 run clearpath_generator_common generate_description "
            f"  -s {config_dir}/robot.yaml -o {config_dir}"
        )

    if not urdf_path.exists():
        print(f"robot.urdf not found — generating from {xacro_path.name} ...")
        try:
            import xacro
        except ImportError as exc:
            raise ImportError(
                "The 'xacro' package is not importable.  Run inside the "
                "clearpath pixi environment."
            ) from exc
        doc = xacro.process_file(str(xacro_path))
        urdf_path.write_text(doc.toxml())
        print(f"Written: {urdf_path}")

    return str(urdf_path)


def make_ridgeback_robot(
    urdf_path: str,
    config: Optional[RidgebackConfig] = None,
    node: Optional[Node] = None,
    spin_node: bool = True,
    name: str = "ridgeback_robot",
) -> RidgebackRobot:
    """Factory function to create a RidgebackRobot.

    Args:
        urdf_path:  Absolute path to the Ridgeback URDF file.
        config:     Environment configuration.  Defaults to
                    ``RidgebackConfig.for_real_robot()``.
                    Pass ``RidgebackConfig.for_simulation()`` for the sim.
        node:       Existing ROS2 node to reuse.
        spin_node:  Whether to spin the node in a background thread.
        name:       Node name (used only when ``node`` is None).

    Returns:
        Fully initialised ``RidgebackRobot`` instance.
    """
    return RidgebackRobot(
        urdf_path=urdf_path,
        config=config,
        node=node,
        spin_node=spin_node,
        name=name,
    )
