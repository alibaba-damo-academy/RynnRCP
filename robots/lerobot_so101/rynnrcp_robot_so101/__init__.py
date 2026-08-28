"""SO101 robot package for RynnRCP."""

__all__ = ["SO101Controller", "SO101BimanualController"]


def __getattr__(name: str):
    if name == "SO101Controller":
        from .controller import SO101Controller
        return SO101Controller
    elif name == "SO101BimanualController":
        from .controller import SO101BimanualController
        return SO101BimanualController
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
