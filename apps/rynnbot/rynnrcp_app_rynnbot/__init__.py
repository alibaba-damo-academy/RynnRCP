"""RynnBot app package."""

from __future__ import annotations

from typing import Any

__all__ = ["RynnBotApp"]


def __getattr__(name: str) -> Any:
    if name == "RynnBotApp":
        from .rynnbot_app import RynnBotApp

        return RynnBotApp
    raise AttributeError(name)
