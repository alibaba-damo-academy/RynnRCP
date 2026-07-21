"""SO101 robot package for RynnRCP."""

__all__ = ["SO101Controller", "SO101BimanualController"]


def __getattr__(name: str):
    if name in __all__:
        from .controller import SO101BimanualController, SO101Controller

        return {
            "SO101Controller": SO101Controller,
            "SO101BimanualController": SO101BimanualController,
        }[name]
    raise AttributeError(name)
