"""LeKiwi robot package for RynnRCP."""

__all__ = ["LeKiwiController"]


def __getattr__(name: str):
    if name == "LeKiwiController":
        from .controller import LeKiwiController

        return LeKiwiController
    raise AttributeError(name)
