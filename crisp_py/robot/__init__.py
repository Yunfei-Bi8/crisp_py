"""Initialize the robot module."""

from crisp_py.robot.robot import (  # noqa: F401
    Robot,
    list_robot_configs,
    make_robot,
)
from crisp_py.robot.robot_config import (  # noqa: F401
    FrankaConfig,
    IiwaConfig,
    KinovaConfig,
    RobotConfig,
    SO101Config,
    make_robot_config,
)
from crisp_py.robot.ridgeback_robot import (  # noqa: F401
    RidgebackConfig,
    RidgebackRobot,
    make_ridgeback_robot,
    get_ridgeback_urdf_path,
)
from crisp_py.robot.ridgeback_controller_switcher import (  # noqa: F401
    RidgebackControllerSwitcher,
)
from crisp_py.utils.geometry import Pose  # noqa: F401

__all__ = [
    "Robot",
    "make_robot",
    "list_robot_configs",
    "RobotConfig",
    "FrankaConfig",
    "IiwaConfig",
    "KinovaConfig",
    "SO101Config",
    "make_robot_config",
    "RidgebackConfig",
    "RidgebackRobot",
    "RidgebackControllerSwitcher",
    "make_ridgeback_robot",
    "get_ridgeback_urdf_path",
    "Pose",
]
