# comm_plugin/rynnbot_plugin/mqtt_client.py

"""
MQTT transport client for the Rynnbot plugin.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module provides :class:`~comm_plugin.rynnbot_plugin.mqtt_client.RynnMqttClient`,
a thin wrapper around ``paho-mqtt`` that connects using credentials fetched from
:class:`~comm_plugin.rynnbot_plugin.auth.RynnAuthClient`.

The client derives the MQTT username/password from the auth response, connects to
the broker, subscribes to the Rynnbot control topics (acquire/release), and forwards
incoming messages to an optional user callback in a normalized dict form.

Lifecycle:
    Call :meth:`RynnMqttClient.start` to authenticate and connect, use
    :meth:`RynnMqttClient.publish` to send messages, and :meth:`RynnMqttClient.close`
    to disconnect and stop the network loop thread.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional
import base64
import hmac
import threading
import time

import paho.mqtt.client as mqtt
from paho.mqtt.client import CallbackAPIVersion

from .auth import RynnAuthClient
from rcp_core.common.utils.logger import server_logger

logger = server_logger()


class RynnMqttClient:
    def __init__(
        self,
        auth: RynnAuthClient,
        endpoint_mqtt: str,
        mqtt_port: int = 1883,
        on_message_callback=None,
    ) -> None:
        """Initialize the MQTT client wrapper."""
        self.auth = auth
        self.endpoint_mqtt = endpoint_mqtt
        self.mqtt_port = mqtt_port

        self.product_key = auth.product_key
        self.device_name = auth.device_name

        self._client: Optional[mqtt.Client] = None
        self._connected = threading.Event()

        self._mqtt_host = ""
        self._client_id = ""
        self._device_ak = ""
        self._device_sk = ""
        self._instance_id = ""

        self._client_username = ""
        self._client_password = ""
        self._topic_prefix = ""
        self._sub_topics: List[str] = []

        self._on_message_callback = on_message_callback

    def set_message_callback(self, cb) -> None:
        """Set the callback function for handling received MQTT messages."""
        self._on_message_callback = cb

    def _init_from_config(self, cfg: Dict[str, Any]) -> None:
        """Initialize MQTT connection info from the auth service configuration."""
        self._mqtt_host = cfg["mqtt_host"]
        self._client_id = cfg["client_id"]
        self._device_ak = cfg["device_ak"]
        self._device_sk = cfg["device_sk"]
        self._instance_id = cfg["instance_id"]

        mac = hmac.new(
            self._device_sk.encode("utf-8"),
            self._client_id.encode("utf-8"),
            digestmod="sha1",
        )
        self._client_password = base64.b64encode(mac.digest()).decode("utf-8")
        self._client_username = (
            f"DeviceCredential|{self._device_ak}|{self._instance_id}"
        )

        self._topic_prefix = f"sys/{self.product_key}/{self.device_name}/"
        self._sub_topics = [
            self._topic_prefix + "rrpc/request/+/acquire_device",
            self._topic_prefix + "rrpc/request/+/release_device",
        ]

    def _on_connect(
        self, client, userdata, flags, reason_code, properties=None
    ) -> None:
        """Callback for successful/failed MQTT connection."""
        if reason_code == 0:
            logger.info("[MQTT] Connected")
            for t in self._sub_topics:
                logger.info(f"[MQTT] Subscribing: {t}")
                client.subscribe(t, qos=1)
            self._connected.set()
        else:
            logger.error(f"[MQTT] Connect failed, reason_code={reason_code}")
            self._connected.clear()

    def _on_disconnect(self, client, userdata, reason_code, properties=None) -> None:
        """Callback when MQTT connection is disconnected."""
        logger.info(f"[MQTT] Disconnected, reason_code={reason_code}")
        self._connected.clear()

    def _on_message(self, client, userdata, msg: mqtt.MQTTMessage) -> None:
        """Callback when an MQTT message is received."""
        topic = msg.topic
        payload = msg.payload.decode("utf-8", errors="ignore")
        logger.info(f"[MQTT] Message: {topic} {payload[:200]}")

        if self._on_message_callback:
            self._on_message_callback(
                {"channel": "mqtt", "topic": topic, "payload": payload, "raw": msg}
            )

    def start(self, keepalive: int = 30) -> None:
        """Start the MQTT client and establish a connection."""
        cfg_auth = self.auth.get_mqtt_config(self.endpoint_mqtt, self.mqtt_port)
        self._init_from_config(cfg_auth)

        self._client = mqtt.Client(
            client_id=self._client_id,
            clean_session=True,
            protocol=mqtt.MQTTv311,
            # callback_api_version=CallbackAPIVersion.VERSION2,
        )
        self._client.username_pw_set(self._client_username, self._client_password)
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message
        self._client.reconnect_delay_set(min_delay=2, max_delay=365 * 24 * 3600)

        logger.info(f"[MQTT] Connecting to {self._mqtt_host}:{self.mqtt_port}")
        self._client.connect(self._mqtt_host, self.mqtt_port, keepalive=keepalive)
        self._client.loop_start()

        while not self._connected.is_set():
            logger.info("[MQTT] Waiting for connection...")
            time.sleep(1)

    def publish(self, topic: str, payload: str, qos: int = 1) -> None:
        """Publish an MQTT message."""
        if not self._client:
            logger.warning("[MQTT] client not ready, drop message")
            return
        self._client.publish(topic, payload, qos=qos)

    def close(self) -> None:
        """Close the MQTT client connection and stop its loop."""
        if not self._client:
            return
        logger.info("[MQTT] Closing...")
        self._client.disconnect()
        self._client.loop_stop()
        self._client = None
        self._connected.clear()
        logger.info("[MQTT] Closed")
