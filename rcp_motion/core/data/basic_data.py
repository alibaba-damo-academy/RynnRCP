"""
Common utilities and shared components for the robot motion project.
"""

from typing import List
from dataclasses import dataclass, field


@dataclass
class Pose:
    """3D pose: position + orientation"""

    pos: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # [x, y, z]
    quat: List[float] = field(
        default_factory=lambda: [0.0, 0.0, 0.0, 1.0]
    )  # [x, y, z, w]

    def __post_init__(self):
        if len(self.pos) != 3:
            raise ValueError("Pose.pos must be length 3")
        if len(self.quat) != 4:
            raise ValueError("Pose.quat must be length 4")


@dataclass
class Wrench:
    """Force and torque"""

    force: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # [fx, fy, fz]
    torque: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # [tx, ty, tz]

    def __post_init__(self):
        if len(self.force) != 3:
            raise ValueError("Wrench.force must be length 3")
        if len(self.torque) != 3:
            raise ValueError("Wrench.torque must be length 3")


@dataclass
class Twist:
    """Spatial velocity: linear + angular"""

    linear: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    angular: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

    def __post_init__(self):
        if len(self.linear) != 3:
            raise ValueError("Twist.linear must be length 3")
        if len(self.angular) != 3:
            raise ValueError("Twist.angular must be length 3")
