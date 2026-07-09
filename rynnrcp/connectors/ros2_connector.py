"""
ROS 2 connector.

Ported from RynnRCP ROS2Adapter. Guards ``rclpy`` import; raises at init
if not available.  Creates an independent Context + MultiThreadedExecutor
and pools publishers by topic/message type/QoS.
"""

from __future__ import annotations

import json
import logging
import threading
from typing import Any, Callable, Dict, Tuple

from rynnrcp.utils.imports import get_message_class

from .base import BaseConnector

logger = logging.getLogger(__name__)
_THREAD_JOIN_TIMEOUT_S = 1.0

try:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.context import Context

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class ROS2Connector(BaseConnector):
    """ROS 2 protocol connector."""

    def __init__(
        self,
        node_name: str = "rynnrcp_ros2",
        qos: Dict[str, Any] | None = None,
        num_threads: int = 4,
    ) -> None:
        if not ROS2_AVAILABLE:
            raise RuntimeError(
                "rclpy is not installed – cannot create ROS2Connector"
            )

        self._default_qos = dict(qos or {})

        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node(node_name, context=self.context)

        self.executor = MultiThreadedExecutor(num_threads=int(num_threads), context=self.context)
        self.executor.add_node(self.node)
        self._executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._executor_thread.start()

        self.publishers: Dict[Tuple[str, type, tuple[tuple[str, str], ...]], Any] = {}
        self._pub_lock = threading.Lock()

        logger.info("ROS2Connector node '%s' started", node_name)

    # ── QoS ──────────────────────────────────────────────────────────

    @staticmethod
    def _qos_config(default_qos: Dict[str, Any], params: Dict[str, Any]) -> Dict[str, Any]:
        qos = dict(default_qos or {})
        qos.update(dict((params or {}).get("qos") or {}))
        return qos

    @staticmethod
    def _qos_key(qos: Dict[str, Any]) -> tuple[tuple[str, str], ...]:
        return tuple(sorted((str(k), str(v)) for k, v in qos.items()))

    @staticmethod
    def _build_qos(qos: Dict[str, Any]) -> QoSProfile:
        reliability = qos.get("reliability", "reliable")
        if reliability in (ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT):
            reliability_policy = reliability
        else:
            reliability_name = str(reliability).lower()
            if reliability_name == "best_effort":
                reliability_policy = ReliabilityPolicy.BEST_EFFORT
            elif reliability_name == "reliable":
                reliability_policy = ReliabilityPolicy.RELIABLE
            else:
                raise ValueError("qos.reliability must be 'reliable' or 'best_effort'")

        history = qos.get("history", "keep_last")
        if history in (HistoryPolicy.KEEP_LAST, HistoryPolicy.KEEP_ALL):
            history_policy = history
        else:
            history_name = str(history).lower()
            if history_name == "keep_all":
                history_policy = HistoryPolicy.KEEP_ALL
            elif history_name == "keep_last":
                history_policy = HistoryPolicy.KEEP_LAST
            else:
                raise ValueError("qos.history must be 'keep_last' or 'keep_all'")

        durability = qos.get("durability", "volatile")
        if durability in (DurabilityPolicy.VOLATILE, DurabilityPolicy.TRANSIENT_LOCAL):
            durability_policy = durability
        else:
            durability_name = str(durability).lower()
            if durability_name == "transient_local":
                durability_policy = DurabilityPolicy.TRANSIENT_LOCAL
            elif durability_name == "volatile":
                durability_policy = DurabilityPolicy.VOLATILE
            else:
                raise ValueError("qos.durability must be 'volatile' or 'transient_local'")

        return QoSProfile(
            depth=int(qos.get("depth", 10)),
            reliability=reliability_policy,
            history=history_policy,
            durability=durability_policy,
        )

    # ── pub ──────────────────────────────────────────────────────────

    def _get_publisher(self, topic: str, msg: Any, params: Dict[str, Any]):
        qos_config = self._qos_config(self._default_qos, params)
        key = (topic, type(msg), self._qos_key(qos_config))
        if key not in self.publishers:
            with self._pub_lock:
                if key not in self.publishers:
                    self.publishers[key] = self.node.create_publisher(
                        type(msg),
                        topic,
                        self._build_qos(qos_config),
                    )
        return self.publishers[key]

    def pub(self, params: Dict[str, Any], msg: Any) -> None:
        topic = params.get("topic")
        if not topic:
            raise ValueError("params.topic is required")
        self._get_publisher(topic, msg, params).publish(msg)

    # ── sub ──────────────────────────────────────────────────────────

    def sub(self, params: Dict[str, Any], callback: Callable) -> None:
        topic = params.get("topic")
        msg_type_str = params.get("msg_type") or params.get("type")
        if not topic or not msg_type_str:
            raise ValueError("params.topic and params.msg_type are required")

        msg_type = get_message_class(msg_type_str)
        qos = self._build_qos(self._qos_config(self._default_qos, params))
        sub_callback = callback
        if params.get("payload_mode") == "protocol_json":
            sub_callback = lambda msg: callback(_decode_protocol_json_message(msg))
        self.node.create_subscription(msg_type, topic, sub_callback, qos)
        logger.info("ROS2Connector subscribed to %s (%s)", topic, msg_type_str)

    # ── stop ─────────────────────────────────────────────────────────

    def stop(self) -> None:
        try:
            if getattr(self, "executor", None) is not None:
                try:
                    self.executor.shutdown()
                except Exception as exc:
                    logger.debug("ROS2 executor shutdown ignored: %s", exc, exc_info=True)
            if getattr(self, "node", None) is not None:
                self.node.destroy_node()
            if getattr(self, "context", None) is not None and self.context.ok():
                rclpy.shutdown(context=self.context)
            if getattr(self, "_executor_thread", None) is not None:
                self._executor_thread.join(timeout=_THREAD_JOIN_TIMEOUT_S)
                if self._executor_thread.is_alive():
                    logger.warning(
                        "ROS2Connector executor thread did not stop within %.1fs",
                        _THREAD_JOIN_TIMEOUT_S,
                    )
        except Exception as e:
            logger.warning("Error shutting down ROS2: %s", e, exc_info=True)
        logger.info("ROS2Connector stopped")


def _decode_protocol_json_message(msg: Any) -> Dict[str, Any]:
    data = getattr(msg, "data", msg)
    if isinstance(data, bytes):
        data = data.decode("utf-8")
    if isinstance(data, str):
        decoded = json.loads(data)
    elif isinstance(data, dict):
        decoded = data
    else:
        raise TypeError("ROS2 protocol_json messages must provide string, bytes, or dict data")
    if not isinstance(decoded, dict):
        raise TypeError("ROS2 protocol_json payload must decode to a JSON object")
    return decoded
