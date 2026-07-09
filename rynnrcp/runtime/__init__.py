"""Runtime orchestration primitives."""

from rynnrcp.config.runner_config import RunnerInputSpec, RunnerOutputSpec
from .runner_manager import (
    RunnerManager,
    setup_runner_manager_process,
)
from .scheduler import Component, Scheduler
from .tool_bus import ToolBus, ToolInfo

__all__ = [
    "RunnerInputSpec",
    "RunnerManager",
    "RunnerOutputSpec",
    "setup_runner_manager_process",
    "Runtime",
    "Component",
    "Scheduler",
    "ToolBus",
    "ToolInfo",
]


def __getattr__(name: str):
    if name == "Runtime":
        from .runtime import Runtime

        return Runtime
    raise AttributeError(name)
