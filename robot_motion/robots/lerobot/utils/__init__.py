"""
LeRobot v0 Utilities Package.

This package contains utility modules for robot control:
- lcm_handler: LCM communication handling
- policy_interpolator: Trajectory interpolation utilities
- joint_plotter: offline joint position plotting
- interpolator_plotter: offline interpolator position plotting
- lekiwi_velocity_mapping: LeKiwi chassis velocity to wheel speed mapping
"""

from .lcm_handler import LCMHandler
from .policy_interpolator import PolicyInterpolator, lerp
from .joint_plotter import JointPlotter
from .interpolator_plotter import InterpolatorPlotter
from .lekiwi_velocity_mapping import chassis_ikvelocity

__all__ = [
    "LCMHandler",
    "PolicyInterpolator",
    "lerp",
    "JointPlotter",
    "InterpolatorPlotter",
    "chassis_ikvelocity",
]

__version__ = "0.1.0"
