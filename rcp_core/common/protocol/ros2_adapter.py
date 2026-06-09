# rcp_core/common/middleware/ros2_adapter.py

"""
ROS 2 protocol adapter.
~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~rcp_core.common.middleware.ros2_adapter.ROS2Adapter`, an
implementation of :class:`~rcp_core.common.middleware.base_protocol_adapter.BaseProtocolAdapter`
that provides unified publish/subscribe operations on ROS 2 topics.

Key behaviors:
- Guards ROS 2 availability (imports ``rclpy``) and raises at initialization if ROS 2 is
  not installed.
- Creates an *independent* ``rclpy.context.Context`` and calls ``rclpy.init`` with it to
  avoid conflicts with other ROS 2 code in the same process.
- Creates a node and spins it on a background :class:`~rclpy.executors.MultiThreadedExecutor`
  thread.
- Reuses publishers per topic (cached in ``self.publishers``) to avoid recreating them
  on every publish call.
- Uses a default QoS profile (depth=10, reliable, keep-last) for both publishers and
  subscribers.
- :meth:`ROS2Adapter.pub` publishes to ``params.topic``.
- :meth:`ROS2Adapter.sub` subscribes to ``params.topic`` using the message class resolved
  from ``params.msg_type``.
- :meth:`ROS2Adapter.stop` destroys the node and shuts down the independent ROS 2 context.
"""

import threading
from typing import Any, Callable, Dict

from .base_protocol_adapter import BaseProtocolAdapter
from ..utils.get_message_class import get_message_class
from rcp_core.common.utils.logger import server_logger

logger = server_logger()

try:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from rclpy.context import Context

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class ROS2Adapter(BaseProtocolAdapter):
    """ROS2 protocol adapter: manages ROS2 node, publishers, and subscribers."""

    def __init__(self, node_name: str = "ros2_node"):
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS2 is not available, cannot initialize ROS2Adapter")

        # Independent context to avoid conflicts with external rclpy.init
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node(node_name, context=self.context)

        # Multi-threaded executor
        self.executor = MultiThreadedExecutor(num_threads=4, context=self.context)
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()

        logger.info(
            f"[ROS2Adapter] ROS2 node '{node_name}' has been started (independent Context)"
        )

        # Reuse publishers to avoid recreating them on each pub
        self.publishers: Dict[str, Any] = {}
        self._publisher_lock = threading.Lock()

    def _get_publisher(self, topic: str, msg: Any):
        """Get or create a publisher for the given topic and message type."""
        if topic not in self.publishers:
            with self._publisher_lock:
                if topic not in self.publishers:
                    qos = QoSProfile(
                        depth=10,
                        reliability=ReliabilityPolicy.RELIABLE,
                        history=HistoryPolicy.KEEP_LAST,
                    )
                    msg_type = type(msg)
                    self.publishers[topic] = self.node.create_publisher(
                        msg_type, topic, qos
                    )
                    logger.info(f"[ROS2Adapter] Created ROS2 publisher: {topic}")
        return self.publishers[topic]

    def pub(self, params: Dict[str, Any], msg: Any) -> None:
        """Publish a ROS2 message on the given topic."""
        topic = params.get("topic")
        if not topic:
            raise ValueError("[ROS2Adapter.pub] params.topic is not configured")
        publisher = self._get_publisher(topic, msg)
        publisher.publish(msg)

    def sub(self, params: Dict[str, Any], callback: Callable) -> None:
        """Subscribe to a ROS2 topic with the given message type and callback."""
        topic = params.get("topic")
        msg_type_str = params.get("msg_type")
        if not topic or not msg_type_str:
            raise ValueError(
                "[ROS2Adapter.sub] params.topic or params.msg_type is not configured"
            )

        msg_type = get_message_class(msg_type_str)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.node.create_subscription(msg_type, topic, callback, qos)
        logger.info(f"[ROS2Adapter] Subscribed to ROS2 topic: {topic} ({msg_type_str})")

    def stop(self) -> None:
        """Stop the ROS2 node and its independent context."""
        try:
            if getattr(self, "node", None) is not None:
                self.node.destroy_node()
            if getattr(self, "context", None) is not None and self.context.ok():
                rclpy.shutdown(context=self.context)
        except Exception as e:
            logger.warning(f"[WARN] Error shutting down ROS2: {e}")

        logger.info("[ROS2Adapter] Stop complete")
