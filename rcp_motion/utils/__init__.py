"""
RCP Motion Utilities Package.

This package contains consolidated utility modules for robot control:
- policy_interpolator: Trajectory interpolation utilities (PolicyInterpolator, lerp)
- lcm_handler: LCM communication handling
- joint_plotter: Offline joint position plotting
- interpolator_plotter: Offline interpolator position plotting
- ik_solver: Inverse kinematics solver
- lekiwi_velocity_mapping: LeKiwi chassis velocity to wheel speed mapping
- rt_trajgen: Real-time trajectory generation (Ruckig-based)
- simulation_command_gen: Simulation command generation
"""

# rcp_motion/utils/__init__.py
__version__ = "0.2.0"

__all__ = [
    "PolicyInterpolator",
    "lerp",
    "LCMHandler",
    "JointPlotter",
    "InterpolatorPlotter",
    "IKSolver",
    "chassis_ikvelocity",
    "RealtimeTrajGenRuckig",
    "SimCommandGenerator",
]


def __getattr__(name: str):
    if name in ("PolicyInterpolator", "lerp"):
        from .policy_interpolator import PolicyInterpolator, lerp

        return {"PolicyInterpolator": PolicyInterpolator, "lerp": lerp}[name]

    if name == "LCMHandler":
        from .lcm_handler import LCMHandler

        return LCMHandler

    if name == "JointPlotter":
        from .joint_plotter import JointPlotter

        return JointPlotter

    if name == "InterpolatorPlotter":
        from .interpolator_plotter import InterpolatorPlotter

        return InterpolatorPlotter

    if name == "IKSolver":
        from .ik_solver import IKSolver

        return IKSolver

    if name == "chassis_ikvelocity":
        from .lekiwi_velocity_mapping import chassis_ikvelocity

        return chassis_ikvelocity

    if name == "RealtimeTrajGenRuckig":
        from .rt_trajgen import RealtimeTrajGenRuckig

        return RealtimeTrajGenRuckig

    if name == "SimCommandGenerator":
        from .simulation_command_gen import SimCommandGenerator

        return SimCommandGenerator

    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
