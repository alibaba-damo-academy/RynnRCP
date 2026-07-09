"""Base robot controller abstractions.

Robot packages put hardware-specific logic behind this interface. RynnRCP calls
the methods named in robot_integration.yaml; methods exposed as Observations
return protocol value objects, and Action methods receive protocol value objects.
"""

from __future__ import annotations

import logging
from typing import Any, Dict, List, Sequence


class BaseRobotController:
    """Base helpers and overridable protocol method templates for robot controllers.

    The server config and ``robot_integration.yaml`` are the external RCP
    protocol contract. They decide which controller methods are called for
    lifecycle hooks, Observations, Actions, and health checks.

    Methods below are common RCP protocol method templates. Robot packages may
    override only the methods referenced by their integration config. Unused
    protocol methods can inherit the default ``NotImplementedError``. Health is
    the same: ``get_health`` is only meaningful when a component's health source
    references it.

    RCP protocol metadata is defined by the server config and robot integration
    config, not by this class. Runtime protocol objects must not be derived from
    controller-side metadata helpers.
    """

    def __init__(self, logger: logging.Logger | None = None) -> None:
        self.logger = logger or logging.getLogger(self.__class__.__name__)

    def get_joint_positions(self) -> Dict[str, List[float]]:
        """Return a protocol ``joint_state`` observation value."""
        raise NotImplementedError("get_joint_positions is not implemented")

    def set_joint_positions(self, value: Dict[str, Any]) -> Any:
        """Handle a protocol ``joint_position`` action value."""
        raise NotImplementedError("set_joint_positions is not implemented")

    def get_ee_pose(self) -> Dict[str, List[float]]:
        """Return a protocol ``ee_pose`` observation value."""
        raise NotImplementedError("get_ee_pose is not implemented")

    def set_ee_pose(self, pose: Sequence[float] | Dict[str, Any]) -> Any:
        """Handle a protocol ``ee_pose`` action value."""
        raise NotImplementedError("set_ee_pose is not implemented")

    def get_health(self) -> Dict[str, Any]:
        """Return protocol health payload for configured health checks."""
        return {"errors": [], "warnings": []}
