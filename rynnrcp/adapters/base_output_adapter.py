"""
Base output adapter.

Defines the abstract interface for converting a protocol action step into
publishable messages.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Dict, List, Tuple


class BaseOutputAdapter(ABC):
    """Abstract base for output adapters.

    Subclasses implement :meth:`build_step_output` to convert one action step
    into a list of frame groups, where each frame group is a list of
    ``(connector_type, params, msg, interval, step)`` tuples.

    - *interval*: seconds to wait before publishing this frame
    - *step*: number of frames this message represents; output adapters
      should emit one frame per message.
    """

    @abstractmethod
    def build_step_output(
        self,
        step_output: Any,
        outputs: List[Dict[str, Any]],
        fps: float,
    ) -> List[List[Tuple[str, Dict, Any, float, int]]]:
        """Build publishable messages from one action step.

        Args:
            step_output: Dict keyed by protocol action name.
            outputs: Output configuration dicts from the server config.
            fps: Publishing frequency in Hz.

        Returns:
            List of frame groups. Each group is a list of tuples
            ``(connector_type, params, msg, interval, step)``.
        """
        ...
