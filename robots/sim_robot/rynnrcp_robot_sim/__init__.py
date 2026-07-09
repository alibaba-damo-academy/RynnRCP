"""RynnRCP Simulation robot integration package.

Provides SimRobotController and SimCamera for connecting to Isaac Sim
via ZeroMQ (JointManager and FrameManager servers).
Supports any robot type (SO101, RM75, etc.) through configuration.
"""

from .controller import SimRobotController
from .sim_camera import SimCamera

__all__ = ["SimRobotController", "SimCamera"]
