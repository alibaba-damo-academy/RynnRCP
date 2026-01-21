# rcp_core/common/middleware/lcm_adapter.py

"""
LCM protocol adapter.
~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.middleware.lcm_adapter.LCMAdapter`, an
implementation of :class:`~rcp_core.common.middleware.base_protocol_adapter.BaseProtocolAdapter`
for the LCM transport.

Key behaviors:
- lazily guards import/availability of the ``lcm`` Python package and raises at init if missing
- maintains an internal background thread that continuously services LCM traffic via
  ``handle_timeout``
- :meth:`LCMAdapter.pub` publishes by encoding the message and sending it on ``params.topic``
- :meth:`LCMAdapter.sub` subscribes to ``params.topic`` with a configured ``params.msg_type``,
  decodes incoming bytes into the concrete message class, and forwards it to the provided callback
- :meth:`LCMAdapter.stop` stops the background loop
"""

import threading
import time
from typing import Any, Callable, Dict

from .base_protocol_adapter import BaseProtocolAdapter
from ..utils.get_message_class import get_message_class
from rcp_core.common.utils.logger import server_logger

logger = server_logger()

try:
    import lcm

    LCM_AVAILABLE = True
except ImportError:
    LCM_AVAILABLE = False


class LCMAdapter(BaseProtocolAdapter):
    """LCM protocol adapter."""

    def __init__(self):
        if not LCM_AVAILABLE:
            raise RuntimeError("LCM is not available, cannot initialize LCMAdapter")

        self.lcm_instance = lcm.LCM()

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        """LCM spin loop."""
        while self._running:
            try:
                self.lcm_instance.handle_timeout(10)
            except Exception as e:
                logger.error(f"[LCMAdapter] LCM loop error: {e}")
                time.sleep(0.01)

    def pub(self, params: Dict[str, Any], msg: Any) -> None:
        """Publish an LCM message on the given topic."""
        topic = params.get("topic")
        if not topic:
            raise ValueError("[LCMAdapter.pub] params.topic not configured")
        self.lcm_instance.publish(topic, msg.encode())

    def sub(self, params: Dict[str, Any], callback: Callable) -> None:
        """Subscribe to an LCM topic with the given message type and callback."""
        topic = params.get("topic")
        msg_type_str = params.get("msg_type")
        if not topic or not msg_type_str:
            raise ValueError(
                "[LCMAdapter.sub] params.topic or params.msg_type not configured"
            )

        msg_type = get_message_class(msg_type_str)

        def _internal_handler(channel, data):
            decoded_msg = msg_type.decode(data)
            callback(decoded_msg)

        self.lcm_instance.subscribe(topic, _internal_handler)

        logger.info(
            f"[LCMAdapter] Subscribed to LCM channel: topic={topic}, msg_type={msg_type_str}"
        )

    def stop(self) -> None:
        """stop the LCM adapter loop and clean up."""
        self._running = False
        logger.info("[LCMAdapter] LCM stopped")
