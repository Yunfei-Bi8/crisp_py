"""Ridgeback-specific ROS2 controller switching helper.

Wraps the ``controller_manager/switch_controller`` service for the Clearpath
Ridgeback platform, which runs its controller_manager under a non-standard
namespace (``/r100_0207/manipulators/controller_manager``).

Two high-level transitions are provided, matching the Ridgeback's two
supported arm control modes:

  * ``switch_to_cartesian()``      — activate CRISP CartesianController
  * ``switch_to_joint_trajectory()``  — restore FollowJointTrajectory controller
"""

import threading
from typing import List, Optional

from controller_manager_msgs.srv import SwitchController
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


class RidgebackControllerSwitcher:
    """Manages ros2_control controller transitions for the Ridgeback arm.

    Uses asynchronous service calls with ``threading.Event`` to avoid
    busy-waits.  All methods are safe to call from any thread as long as
    the ROS2 node is being spun by a ``MultiThreadedExecutor``.

    Args:
        node:                       ROS2 node that owns the service client.
        controller_manager_topic:   Full absolute path to the
            controller_manager node (e.g.
            ``"/r100_0207/manipulators/controller_manager"``).
            If None, controller switching is disabled and calls raise
            ``RuntimeError``.
        cartesian_controller_name:  Name of the CRISP CartesianController as
            registered in ros2_control (e.g. ``"cartesian_controller"``).
        joint_trajectory_controller_name: Name of the FollowJointTrajectory
            controller (e.g. ``"arm_0_joint_trajectory_controller"``).
        service_timeout:            Seconds to wait for the switch_controller
            service to become available.
    """

    def __init__(
        self,
        node: Node,
        controller_manager_topic: Optional[str],
        cartesian_controller_name: str,
        joint_trajectory_controller_name: str,
        service_timeout: float = 5.0,
    ) -> None:
        self._node = node
        self._cartesian_name = cartesian_controller_name
        self._jtc_name = joint_trajectory_controller_name
        self._service_timeout = service_timeout

        if controller_manager_topic is not None:
            ns = controller_manager_topic.rstrip("/")
            self._client = node.create_client(
                SwitchController,
                f"{ns}/switch_controller",
                callback_group=ReentrantCallbackGroup(),
            )
        else:
            self._client = None

    # ------------------------------------------------------------------ #
    #  Internal                                                            #
    # ------------------------------------------------------------------ #

    def _call_switch(
        self,
        activate: List[str],
        deactivate: List[str],
        strictness: int = SwitchController.Request.STRICT,
    ) -> bool:
        """Call switch_controller and block until the response arrives.

        Uses threading.Event — the calling thread sleeps while the executor
        thread handles the response callback.

        Returns:
            True if the controller_manager accepted the switch.

        Raises:
            RuntimeError: If no controller_manager_topic was configured.
            TimeoutError: If the service is not available or the call times out.
        """
        if self._client is None:
            raise RuntimeError(
                "Controller switching is not configured. "
                "Set controller_manager_topic in RidgebackConfig."
            )
        if not self._client.wait_for_service(timeout_sec=self._service_timeout):
            raise TimeoutError(
                f"switch_controller service not available after "
                f"{self._service_timeout}s. Is the controller_manager running?"
            )

        req = SwitchController.Request()
        req.activate_controllers = list(activate)
        req.deactivate_controllers = list(deactivate)
        req.strictness = strictness
        req.activate_asap = True

        done = threading.Event()
        ok_box: List[bool] = [False]

        def _cb(future):
            ok_box[0] = future.result().ok
            done.set()

        self._client.call_async(req).add_done_callback(_cb)
        if not done.wait(timeout=self._service_timeout + 1.0):
            raise TimeoutError(
                "switch_controller service call timed out "
                f"after {self._service_timeout + 1.0:.0f}s."
            )
        return ok_box[0]

    # ------------------------------------------------------------------ #
    #  Public: transitions                                                 #
    # ------------------------------------------------------------------ #

    def switch_to_cartesian(self) -> bool:
        """Deactivate JointTrajectoryController and activate CartesianController.

        Returns:
            True if the controller_manager accepted the request.
        """
        ok = self._call_switch(
            activate=[self._cartesian_name],
            deactivate=[self._jtc_name],
        )
        if ok:
            self._node.get_logger().info(
                f"Switched to CartesianController ('{self._cartesian_name}')."
            )
        else:
            self._node.get_logger().error(
                f"controller_manager rejected switch to '{self._cartesian_name}'."
            )
        return ok

    def switch_to_joint_trajectory(self) -> bool:
        """Deactivate CartesianController and restore JointTrajectoryController.

        Returns:
            True if the controller_manager accepted the request.
        """
        ok = self._call_switch(
            activate=[self._jtc_name],
            deactivate=[self._cartesian_name],
        )
        if ok:
            self._node.get_logger().info(
                f"Restored JointTrajectoryController ('{self._jtc_name}')."
            )
        else:
            self._node.get_logger().error(
                f"controller_manager rejected switch to '{self._jtc_name}'."
            )
        return ok
