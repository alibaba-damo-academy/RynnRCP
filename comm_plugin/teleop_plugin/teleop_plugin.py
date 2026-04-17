# comm_plugin/teleop_plugin/teleop_plugin.py

"""
Teleoperation plugin for RynnRCP.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`TeleopPlugin`, a communication plugin that enables
bilateral teleoperation between a leader arm and a follower arm over LAN (UDP).

Roles
-----
- ``leader``:
    - Reads own arm state via ``get_state`` and sends it to the follower **only when
      both teleop is enabled and the peer connection is established**.
    - Receives follower state and images via UDP.
    - If ``run_action_chunk`` is registered (i.e. ``action_server.outputs`` is configured),
      applies the received follower state back to the leader arm as force feedback.
    - Teleop can be started/stopped dynamically via :meth:`start_teleop` / :meth:`stop_teleop`.
- ``follower``:
    - Receives leader state via UDP and executes it via ``run_action_chunk``.
    - Sends own state back to the leader at ``control_hz`` **only when connected**.
    - Sends camera images (if ``get_image`` is available) at ``image_hz`` **only when connected**.
    - Sends recording status and buffer keys **only when connected**.

Connection detection
--------------------
Both roles send a heartbeat (``MSG_HEARTBEAT``) to the peer at 1 Hz.
Each side monitors incoming heartbeats and sets ``_peer_connected = True`` when
heartbeats arrive within the ``_HEARTBEAT_TIMEOUT`` window (default 5 s), and
resets it to ``False`` when the peer goes silent.  All outgoing data streams
are gated on this flag, so if only one side is running no data is ever sent.

Transport
---------
UDP sockets over LAN with simple packet fragmentation for large payloads (images).
Both state and image payloads are serialized with MsgPack.

Default ports
-------------
- Leader listens on ``leader_port`` (default: 9101).
- Follower listens on ``follower_port`` (default: 9102).
Both sides configure both ports; each side binds to its own and sends to the other.

Config format (YAML)
--------------------
Keys may be placed at the top level or nested under a ``teleop`` key::

    teleop:
      role: leader            # or follower
      remote_host: 192.168.1.100
      leader_port: 9101       # optional, default 9101
      follower_port: 9102     # optional, default 9102
      control_hz: 30          # optional, default 30
      image_hz: 10            # optional, default 10
"""

from __future__ import annotations

import json
import os
import socket
import struct
import threading
import time
import yaml
import msgpack
from typing import Any, Dict, List, Optional, Tuple
from concurrent.futures import ThreadPoolExecutor

from ..base_plugin.base import RcpPlugin
from rcp_core import RcpCore
from rcp_core.common.utils.logger import server_logger

logger = server_logger()

# --------------------------------------------------------------------------- #
# UDP packet protocol
# --------------------------------------------------------------------------- #
# Header layout (11 bytes):
#   magic(2B) + reserved(1B) + msg_id(4B) + frag_idx(2B) + total_frags(2B)
# The msg_type byte in the header is no longer used for routing; it is fixed
# to 0.  The actual message type is carried inside the MsgPack envelope (see
# encode_msg / decode_msg below).
_MAGIC = b"\xAA\xBB"
_HEADER_FMT = "!2sBIHH"
_HEADER_SIZE = struct.calcsize(_HEADER_FMT)   # 11
_MAX_FRAG_PAYLOAD = 8192                       # max bytes carried per UDP datagram
                                                # (macOS net.inet.udp.maxdgram defaults to 9216;
                                                #  8192 + 11-byte header = 8203, safely below)

# Message type identifiers — aligned with PackageType in RobotServerTransportPacket.proto.
# Add new types here without touching any other code except _dispatch.
# Format: 1XX=core data, 2XX=record control, 3XX=data mgmt, 4XX=playback control,
#         5XX=status/heartbeat, 6XX=playback status, 7XX=data mgmt response

# 100-199: Core data messages
MSG_IMAGE_DATA      = 100  # image bundle     {camera_key: bytes, ...}
MSG_STATE_DATA      = 101  # state dict       {"observation.state": [float, ...], ...}
MSG_ACTION_DATA     = 102  # action chunk     {"action": [[float, ...], ...]}
MSG_REQ_IMAGE       = 103  # request image    (no payload)
MSG_REQ_STATE       = 104  # request state    (no payload)
MSG_ACTION_FINISH   = 105  # action finished  (no payload)

# 200-299: Data collection control messages (leader → follower)
MSG_START_RECORD    = 200  # start recording  {"keys": [...], "task_prompt": str, ...}
MSG_STOP_RECORD     = 201  # stop recording   (no payload)
MSG_EXPORT_DATA     = 202  # export data      {"data_coll_id": str}
MSG_DISCARD_RECORD  = 203  # discard episode  {"round_number": int}

# 300-399: Data management messages (leader → follower)
MSG_DATA_MGMT_SCAN   = 300  # scan records     {"request_id": str}
MSG_DATA_MGMT_DELETE = 301  # delete record    {"request_id": str, "path": str} or {"request_id": str, "data_coll_id": str}
MSG_DATA_MGMT_EXPORT = 302  # export specific  {"request_id": str, "episode_paths": [...], "zip_name": str}
MSG_DATA_MGMT_ENCODE = 303  # encode episodes  {"request_id": str, "episode_paths": [...]}

# 400-499: Playback control messages (leader → follower)
MSG_START_PLAYBACK   = 400  # start playback   {"episode_path": str, "request_id": str}
MSG_STOP_PLAYBACK    = 401  # stop playback    {"request_id": str}

# 500-599: Status/heartbeat messages (follower → leader)
MSG_RECORD_STATUS    = 500  # recording status {"recording": bool, "frames_written": int, ...}
MSG_HEARTBEAT        = 501  # heartbeat        {"timestamp": float}
MSG_EXPORT_RESULT    = 502  # export result    {"success": bool, "zip_path": str, "exported_count": int, "message": str}
MSG_EXPORT_PROGRESS  = 503  # export progress  {"progress": float, "total": float, "message": str}
MSG_EXPORT_STARTED   = 504  # export started   {"data_coll_id": str}
MSG_BUFFER_KEYS      = 505  # available buffer keys {"keys": [str, ...]}

# 600-699: Playback status messages (follower → leader)
MSG_PLAYBACK_STATUS  = 600  # playback status  {"request_id": str, "status": str, "progress": int, "message": str, "frame_idx": int, "total_frames": int}

# 700-799: Data management response messages (follower → leader)
MSG_DATA_MGMT_RESULT = 700  # scan/delete/export/encode result {"request_id": str, "success": bool, "data": {...}, "message": str}


def _pack_header(msg_id: int, frag_idx: int, total_frags: int) -> bytes:
    # reserved byte is always 0
    return struct.pack(_HEADER_FMT, _MAGIC, 0, msg_id, frag_idx, total_frags)


def _unpack_header(data: bytes) -> Optional[Tuple[int, int, int]]:
    """Return ``(msg_id, frag_idx, total_frags)`` or ``None`` if invalid."""
    if len(data) < _HEADER_SIZE:
        return None
    magic, _reserved, msg_id, frag_idx, total_frags = struct.unpack(
        _HEADER_FMT, data[:_HEADER_SIZE]
    )
    if magic != _MAGIC:
        return None
    return msg_id, frag_idx, total_frags


def _send_udp(
    sock: socket.socket,
    addr: Tuple[str, int],
    payload: bytes,
    msg_id: int,
) -> None:
    """Fragment *payload* and send over UDP."""
    size = len(payload)
    total = max(1, (size + _MAX_FRAG_PAYLOAD - 1) // _MAX_FRAG_PAYLOAD)
    for i in range(total):
        chunk = payload[i * _MAX_FRAG_PAYLOAD : (i + 1) * _MAX_FRAG_PAYLOAD]
        sock.sendto(_pack_header(msg_id, i, total) + chunk, addr)


# --------------------------------------------------------------------------- #
# Unified MsgPack envelope
# --------------------------------------------------------------------------- #
# Every message is serialized as:
#   msgpack({"t": <msg_type: int>, "d": <data: any>})
#
# To add a new message type:
#   1. Define MSG_XXX = <int> above.
#   2. Call self._send(MSG_XXX, your_data) in a worker loop.
#   3. Handle it in _dispatch with an elif block.
# No changes needed to encode_msg / decode_msg.

def encode_msg(msg_type: int, data: Any) -> bytes:
    """Wrap *data* in a typed envelope and serialize to MsgPack bytes."""
    return msgpack.packb({"t": msg_type, "d": data}, use_bin_type=True)


def decode_msg(raw: bytes) -> Optional[Tuple[int, Any]]:
    """Deserialize a MsgPack envelope; return ``(msg_type, data)`` or ``None``."""
    try:
        env = msgpack.unpackb(raw, raw=False)
        return env["t"], env["d"]
    except Exception as exc:
        logger.error(f"[TeleopPlugin] decode_msg failed: {exc}")
        return None


# --------------------------------------------------------------------------- #
# Fragment reassembler
# --------------------------------------------------------------------------- #

class _Assembler:
    """Reassemble fragmented UDP messages from multiple datagrams."""

    def __init__(self, timeout_s: float = 5.0) -> None:
        self._buf: Dict[int, Dict[int, bytes]] = {}
        self._total: Dict[int, int] = {}
        self._ts: Dict[int, float] = {}
        self._timeout = timeout_s
        self._lock = threading.Lock()

    def feed(
        self,
        msg_id: int,
        frag_idx: int,
        total_frags: int,
        payload: bytes,
    ) -> Optional[bytes]:
        """
        Feed one fragment.

        :returns: Complete reassembled payload once all fragments have arrived,
                  otherwise ``None``.
        """
        with self._lock:
            now = time.monotonic()
            # Expire stale incomplete messages
            stale = [mid for mid, t in self._ts.items() if now - t > self._timeout]
            for mid in stale:
                self._buf.pop(mid, None)
                self._total.pop(mid, None)
                self._ts.pop(mid, None)

            if msg_id not in self._buf:
                self._buf[msg_id] = {}
                self._total[msg_id] = total_frags
                self._ts[msg_id] = now

            self._buf[msg_id][frag_idx] = payload

            if len(self._buf[msg_id]) == self._total[msg_id]:
                complete = b"".join(
                    self._buf[msg_id][i] for i in range(self._total[msg_id])
                )
                self._buf.pop(msg_id)
                self._total.pop(msg_id)
                self._ts.pop(msg_id)
                return complete

        return None


# --------------------------------------------------------------------------- #
# TeleopPlugin
# --------------------------------------------------------------------------- #

class TeleopPlugin(RcpPlugin):
    """
    Teleoperation plugin for RynnRCP.

    Refer to the module docstring for a full description of roles, transport,
    and configuration format.
    """

    _DEFAULT_LEADER_PORT = 9101
    _DEFAULT_FOLLOWER_PORT = 9102
    _DEFAULT_CONTROL_HZ = 30.0
    _DEFAULT_IMAGE_HZ = 10.0

    def __init__(
        self,
        config_file: str,
        role: str,
        enable_web_ui: bool = False,
        web_port: int = 5000,
        open_browser: bool = True,
    ) -> None:
        """
        Initialize TeleopPlugin from a YAML config file.

        :param config_file: Path to the teleop config YAML.
        :param role: ``'leader'`` or ``'follower'``. Typically passed from the
            launch script command-line argument rather than stored in the config.
        :param enable_web_ui: If True and role is 'leader', start a web UI server
            for browser-based control.
        :param web_port: Port for the web UI server (default: 5000).
        :param open_browser: Whether to automatically open a browser when web UI starts.
        :raises ValueError: If ``role`` is invalid or required hosts are missing.
        """
        cfg = self._load_config(config_file)

        self.role: str = str(role).lower()
        if self.role not in ("leader", "follower"):
            raise ValueError(
                f"[TeleopPlugin] role must be 'leader' or 'follower', got: {self.role!r}"
            )

        leader_host: str = str(cfg.get("leader_host", ""))
        follower_host: str = str(cfg.get("follower_host", ""))
        if not leader_host:
            raise ValueError("[TeleopPlugin] leader_host must be specified in config.")
        if not follower_host:
            raise ValueError("[TeleopPlugin] follower_host must be specified in config.")

        # Each side sends to the OTHER side's host
        self.remote_host: str = follower_host if self.role == "leader" else leader_host

        self.leader_port: int = int(cfg.get("leader_port", self._DEFAULT_LEADER_PORT))
        self.follower_port: int = int(cfg.get("follower_port", self._DEFAULT_FOLLOWER_PORT))
        self.control_hz: float = float(cfg.get("control_hz", self._DEFAULT_CONTROL_HZ))
        self.image_hz: float = float(cfg.get("image_hz", self._DEFAULT_IMAGE_HZ))

        self.core: Optional[RcpCore] = None
        self._stop_event = threading.Event()
        self._sock: Optional[socket.socket] = None
        self._remote_addr: Optional[Tuple[str, int]] = None
        self._mid_counter = 0
        self._mid_lock = threading.Lock()

        # Cache for data received from the follower (leader side only)
        self._latest_follower_state: Optional[Dict[str, Any]] = None
        self._latest_follower_images: Optional[Dict[str, bytes]] = None
        self._follower_state_lock = threading.Lock()
        self._follower_images_lock = threading.Lock()

        # Cache for leader's own state (leader side only, updated in _state_send_loop)
        self._latest_leader_state: Optional[Dict[str, Any]] = None
        self._leader_state_lock = threading.Lock()

        # Leader-only: teleop enabled flag (controls whether leader sends state to follower).
        # Follower always runs; leader can start/stop teleop dynamically.
        self._teleop_enabled = False if self.role == "leader" else True

        # Data collection state (leader side: tracking follower's recording status)
        self._record_status_lock = threading.Lock()
        self._latest_record_status: Dict[str, Any] = {
            "recording": False,
            "frames_written": 0,
            "episode_dir": None,
            "round_number": 0,
        }
        # Leader-side: available buffer keys from follower
        self._available_buffer_keys_lock = threading.Lock()
        self._available_buffer_keys: List[str] = []
        # Leader-side: current round number for sequential data collection IDs
        self._current_round = 0
        # Generate data_coll_id once at startup (format: teleop_<timestamp_seconds>)
        import time
        self._data_coll_id = f"teleop_dataset_{int(time.time())}"

        # Web UI settings (leader only)
        self._enable_web_ui = enable_web_ui and (self.role == "leader")
        self._web_port = web_port
        self._open_browser = open_browser
        self._web_ui: Optional[Any] = None  # Lazy import to avoid circular dependency

        # Heartbeat tracking (bidirectional: both sides send and receive heartbeats)
        # _last_peer_heartbeat_time: last time a heartbeat was received from the remote peer
        self._last_heartbeat_time: Optional[float] = None   # kept for backward compat
        self._last_peer_heartbeat_time: Optional[float] = None
        self._heartbeat_thread: Optional[threading.Thread] = None
        self._stop_heartbeat = threading.Event()

        # Peer connection state: True only when the remote peer is reachable
        # (i.e. heartbeats are being received within _HEARTBEAT_TIMEOUT seconds).
        # All outgoing data streams are gated on this flag.
        self._peer_connected: bool = False
        self._peer_connected_lock = threading.Lock()
        self._HEARTBEAT_TIMEOUT: float = 5.0  # seconds without heartbeat → disconnected

        # Export result tracking (follower sends result after async export)
        self._export_result: Optional[Dict[str, Any]] = None
        self._export_result_time: Optional[float] = None
        self._export_in_progress: bool = False
        self._export_started_time: Optional[float] = None  # time when follower confirmed export started
        self._export_progress: Optional[Dict[str, Any]] = None  # latest progress snapshot

        # Data management results tracking (follower sends results after scan/delete/export)
        self._data_mgmt_results_lock = threading.Lock()
        self._data_mgmt_results: Dict[str, Dict[str, Any]] = {}  # request_id -> {data, timestamp}

        # Playback status tracking (follower sends status during playback)
        self._playback_status_lock = threading.Lock()
        self._playback_status: Dict[str, Any] = {}  # latest playback status
        self._playback_in_progress: bool = False

        # Thread pool for async action execution (follower side only)
        # This prevents blocking the receive loop when executing actions
        self._action_executor: Optional[ThreadPoolExecutor] = None
        if self.role == "follower":
            self._action_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="teleop-action-")

        # Connection monitor thread (shared by both roles)
        self._connection_monitor_thread: Optional[threading.Thread] = None

    @staticmethod
    def _load_config(path: str) -> Dict[str, Any]:
        with open(path, "r", encoding="utf-8") as f:
            raw = yaml.safe_load(f) or {}
        # Support both top-level keys and keys nested under 'teleop'
        return raw.get("teleop", raw)

    # ------------------------------------------------------------------ #
    # RcpPlugin interface
    # ------------------------------------------------------------------ #

    def bind_rcp_core(self, core: RcpCore) -> None:
        """Attach an :class:`RcpCore` instance so the plugin can call bus tools."""
        self.core = core

    def start(self) -> None:
        """
        Open the UDP socket, start worker threads, and block until :meth:`stop` is called.

        Leader threads:
          - ``teleop-state-send``: loop at *control_hz*, call ``get_state``, send to follower
            (only when teleop is enabled).
          - ``teleop-recv``: receive follower state / images; optionally apply force feedback.

        Follower threads:
          - ``teleop-recv``: receive leader state, execute via ``run_action_chunk``.
          - ``teleop-state-send``: loop at *control_hz*, call ``get_state``, send to leader.
          - ``teleop-image-send``: loop at *image_hz*, call ``get_image``, send to leader
            (only started if ``get_image`` tool is registered).
        """
        if self.core is None:
            raise RuntimeError(
                "[TeleopPlugin] bind_rcp_core() must be called before start()."
            )

        local_port = self.leader_port if self.role == "leader" else self.follower_port
        remote_port = self.follower_port if self.role == "leader" else self.leader_port
        self._remote_addr = (self.remote_host, remote_port)

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", local_port))
        self._sock.settimeout(1.0)

        logger.info(
            f"[TeleopPlugin] role={self.role}, "
            f"listen=0.0.0.0:{local_port}, send→{self._remote_addr}, "
            f"control_hz={self.control_hz}, image_hz={self.image_hz}"
        )

        threads = []
        # Both roles: receive loop
        threads.append(threading.Thread(
            target=self._recv_loop, daemon=True, name="teleop-recv"
        ))
        # Both roles: state send loop
        threads.append(threading.Thread(
            target=self._state_send_loop, daemon=True, name="teleop-state-send"
        ))
        # Both roles: send heartbeat so the peer can detect connection
        self._stop_heartbeat.clear()
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_send_loop, daemon=True, name="teleop-heartbeat"
        )
        self._heartbeat_thread.start()
        # Both roles: monitor peer heartbeat to set/clear _peer_connected
        self._connection_monitor_thread = threading.Thread(
            target=self._connection_monitor_loop, daemon=True, name="teleop-conn-monitor"
        )
        self._connection_monitor_thread.start()

        if self.role == "leader":
            if self._enable_web_ui:
                logger.info(
                    f"[TeleopPlugin][leader] Starting Web UI on port {self._web_port}..."
                )
            else:
                logger.info(
                    "[TeleopPlugin][leader] Teleop disabled by default. "
                    "Call start_teleop() to begin streaming."
                )
        else:
            if self._has_tool("get_image"):
                threads.append(threading.Thread(
                    target=self._image_send_loop, daemon=True, name="teleop-image-send"
                ))
            else:
                logger.info(
                    "[TeleopPlugin][follower] get_image not available, "
                    "skipping image streaming."
                )
            # Follower: periodically send recording status to leader
            threads.append(threading.Thread(
                target=self._record_status_send_loop, daemon=True, name="teleop-record-status"
            ))
            # Follower: periodically send available buffer keys to leader
            threads.append(threading.Thread(
                target=self._buffer_keys_send_loop, daemon=True, name="teleop-buffer-keys"
            ))

        for t in threads:
            t.start()

        # Start Web UI if enabled (leader only)
        if self._enable_web_ui:
            from .web_ui import TeleopWebUI

            self._web_ui = TeleopWebUI(
                plugin=self,
                port=self._web_port,
                open_browser=self._open_browser,
                quit_callback=self.stop,
            )
            self._web_ui.start()

        self._stop_event.wait()

    def stop(self) -> None:
        """Signal all worker threads to stop and close the socket."""
        self._stop_event.set()
        self._stop_heartbeat.set()
        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            self._heartbeat_thread.join(timeout=1.0)
        if self._connection_monitor_thread and self._connection_monitor_thread.is_alive():
            self._connection_monitor_thread.join(timeout=1.0)
        # Shutdown action executor (follower side)
        if self._action_executor:
            self._action_executor.shutdown(wait=False)
        if self._web_ui:
            self._web_ui.stop()
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
        logger.info("[TeleopPlugin] Stopped.")

    # ------------------------------------------------------------------ #
    # Teleop control (leader only)
    # ------------------------------------------------------------------ #

    def start_teleop(self) -> None:
        """
        Enable teleop streaming (leader only).

        Once called, the leader will start sending its joint state to the follower.
        """
        if self.role != "leader":
            logger.warning("[TeleopPlugin] start_teleop() is only valid for leader role.")
            return
        if self._teleop_enabled:
            logger.info("[TeleopPlugin][leader] Teleop already enabled.")
            return
        self._teleop_enabled = True
        logger.info("[TeleopPlugin][leader] Teleop STARTED.")

    def stop_teleop(self) -> None:
        """
        Disable teleop streaming (leader only).

        Once called, the leader will stop sending its joint state to the follower.
        """
        if self.role != "leader":
            logger.warning("[TeleopPlugin] stop_teleop() is only valid for leader role.")
            return
        if not self._teleop_enabled:
            logger.info("[TeleopPlugin][leader] Teleop already disabled.")
            return
        self._teleop_enabled = False
        logger.info("[TeleopPlugin][leader] Teleop STOPPED.")

    @property
    def teleop_enabled(self) -> bool:
        """Return whether teleop streaming is currently enabled."""
        return self._teleop_enabled

    @property
    def peer_connected(self) -> bool:
        """Return whether the remote peer is currently connected (heartbeat alive)."""
        with self._peer_connected_lock:
            return self._peer_connected

    # ------------------------------------------------------------------ #
    # Playback control (leader only)
    # ------------------------------------------------------------------ #

    def start_playback(self, episode_path: str, request_id: str = "") -> bool:
        """
        Send START_PLAYBACK command to follower to begin episode playback.

        The follower will:
        1. Load the episode data (supports both encoded and raw formats)
        2. Pre-move arm to the first frame position (linear interpolation)
        3. Wait 2 seconds
        4. Playback actions according to original timestamps

        :param episode_path: Full path to the episode directory.
        :param request_id: Optional request ID for tracking playback status.
        :returns: True if command was sent successfully.
        """
        if self.role != "leader":
            logger.warning("[TeleopPlugin] start_playback() is only valid for leader role.")
            return False
        if not self._peer_connected:
            logger.warning("[TeleopPlugin] Cannot start playback: peer not connected.")
            return False
        if self._teleop_enabled:
            logger.warning("[TeleopPlugin] Cannot start playback while teleop is enabled. Please stop teleop first.")
            return False

        with self._playback_status_lock:
            if self._playback_in_progress:
                logger.warning("[TeleopPlugin] Playback already in progress.")
                return False
            self._playback_in_progress = True
            self._playback_status = {"status": "starting", "progress": 0, "message": "Starting playback..."}

        cmd = {
            "episode_path": episode_path,
            "request_id": request_id or f"pb_{int(time.time())}",
        }
        self._send(MSG_START_PLAYBACK, cmd)
        logger.info(f"[TeleopPlugin][leader] Playback START command sent: {episode_path}")
        return True

    def stop_playback(self, request_id: str = "") -> bool:
        """
        Send STOP_PLAYBACK command to follower to stop episode playback.

        :param request_id: Optional request ID for tracking.
        :returns: True if command was sent successfully.
        """
        if self.role != "leader":
            logger.warning("[TeleopPlugin] stop_playback() is only valid for leader role.")
            return False

        cmd = {"request_id": request_id}
        self._send(MSG_STOP_PLAYBACK, cmd)
        logger.info("[TeleopPlugin][leader] Playback STOP command sent.")

        with self._playback_status_lock:
            self._playback_in_progress = False

        return True

    def get_playback_status(self) -> Dict[str, Any]:
        """Get current playback status (leader only)."""
        with self._playback_status_lock:
            return dict(self._playback_status)

    @property
    def playback_in_progress(self) -> bool:
        """Return whether playback is currently in progress."""
        with self._playback_status_lock:
            return self._playback_in_progress

    # ------------------------------------------------------------------ #
    # Data collection control (leader only)
    # ------------------------------------------------------------------ #

    # Default keys for data collection
    DEFAULT_RECORD_KEYS = [
        "observation.state",
        "observation.images.front",
        "observation.images.wrist",
        "action",
    ]

    def start_recording(
        self,
        task_prompt: str = "teleop_demo",
        task_description: str = "Teleoperation data collection",
        fps: float = 30.0,
        data_coll_id: str = "teleop",
        keys: Optional[list] = None,
    ) -> bool:
        """
        Send START_RECORD command to follower to begin data collection.

        :param task_prompt: Task name for organizing episodes.
        :param task_description: Human-readable description.
        :param fps: Recording frame rate.
        :param data_coll_id: Collection ID for grouping episodes.
        :param keys: List of data keys to record. Defaults to DEFAULT_RECORD_KEYS if not specified.
            Common keys: "observation.state", "observation.images.front", 
            "observation.images.wrist", "action", "observation.effort", etc.
        :returns: True if command was sent successfully.
        """
        if self.role != "leader":
            logger.warning("[TeleopPlugin] start_recording() is only valid for leader role.")
            return False
        if not self._teleop_enabled:
            logger.warning("[TeleopPlugin] Cannot start recording: teleop is not enabled.")
            return False

        with self._record_status_lock:
            if self._latest_record_status.get("recording"):
                logger.warning("[TeleopPlugin] Recording already in progress.")
                return False

        self._current_round += 1
        # Use startup-generated data_coll_id if not explicitly provided
        if not data_coll_id:
            data_coll_id = self._data_coll_id
        # Use provided keys or default keys
        record_keys = keys if keys is not None else self.DEFAULT_RECORD_KEYS
        cmd = {
            "keys": record_keys,
            "task_prompt": task_prompt,
            "task_description": task_description,
            "fps": fps,
            "round_number": self._current_round,
            "data_coll_id": data_coll_id,
        }
        logger.info(f"[TeleopPlugin][leader] Calling start_recording with cmd: {cmd}")
        self._send(MSG_START_RECORD, cmd)
        logger.info(f"[TeleopPlugin][leader] Sent START_RECORD: round={self._current_round}")
        return True

    def stop_recording(self) -> Dict[str, Any]:
        """
        Send STOP_RECORD command to follower to stop data collection.

        :returns: Dict with recording result info (frames_written, etc.)
        """
        if self.role != "leader":
            logger.warning("[TeleopPlugin] stop_recording() is only valid for leader role.")
            return {"frames_written": 0}

        logger.info("[TeleopPlugin][leader] Calling stop_recording...")
        self._send(MSG_STOP_RECORD, {})
        logger.info("[TeleopPlugin][leader] Sent STOP_RECORD.")
        
        # Wait for follower to send final record status (max 2 seconds)
        frames_written = 0
        episode_dir = None
        for _ in range(20):  # 20 * 0.1s = 2s max
            time.sleep(0.1)
            with self._record_status_lock:
                status = self._latest_record_status
                # Check if we received the final status (recording=False after being True)
                if not status.get("recording") and status.get("frames_written", 0) > 0:
                    frames_written = status.get("frames_written", 0)
                    episode_dir = status.get("episode_dir")
                    break
        
        # Fallback to cached values if not received
        if frames_written == 0:
            with self._record_status_lock:
                frames_written = self._latest_record_status.get("frames_written", 0)
                episode_dir = self._latest_record_status.get("episode_dir")
        
        logger.info(f"[TeleopPlugin][leader] stop_recording result: frames_written={frames_written}, episode_dir={episode_dir}")
        
        # If no valid frames, automatically rollback round_number
        if frames_written == 0 and self._current_round > 0:
            logger.info(
                f"[TeleopPlugin][leader] No valid frames recorded, rolling back round from "
                f"{self._current_round} to {self._current_round - 1}"
            )
            self._current_round -= 1
        
        return {
            "frames_written": frames_written,
            "episode_dir": episode_dir,
        }

    def discard_last_recording(self) -> None:
        """
        Discard the last recording by deleting it from disk and decrementing round number.

        Sends a DISCARD_RECORD command to the follower to physically remove the
        episode directory, then rolls back the round counter so the next recording
        reuses the same episode number.
        """
        if self.role != "leader":
            return
        if self._current_round > 0:
            logger.info(
                f"[TeleopPlugin][leader] Discarding round {self._current_round}, "
                f"sending delete command to follower."
            )
            # Ask follower to delete the episode directory
            self._send(MSG_DISCARD_RECORD, {"round_number": self._current_round})
            self._current_round -= 1

    def export_data(self, data_coll_id: str = "") -> Dict[str, Any]:
        """
        Send EXPORT_DATA command to follower to export collected data.

        :param data_coll_id: Collection ID to export.
        :returns: Dict with export result info.
        """
        if self.role != "leader":
            logger.warning("[TeleopPlugin] export_data() is only valid for leader role.")
            return {"success": False, "message": "Only leader can export data"}

        if self._current_round == 0:
            return {"success": False, "message": "没有可导出的数据，请先采集数据"}

        # Use stored data_coll_id if not explicitly provided
        if not data_coll_id:
            data_coll_id = self._data_coll_id

        # Reset export state and mark as in progress
        self._export_in_progress = True
        self._export_result = None
        self._export_result_time = None
        self._export_started_time = None  # reset; will be set when follower acks

        self._send(MSG_EXPORT_DATA, {"data_coll_id": data_coll_id})
        logger.info(f"[TeleopPlugin][leader] Sent EXPORT_DATA: data_coll_id={data_coll_id}")

        return {
            "success": True,
            "in_progress": True,
            "message": "导出已开始，请等待...",
        }

    @property
    def record_status(self) -> Dict[str, Any]:
        """Return the latest recording status received from follower."""
        with self._record_status_lock:
            return dict(self._latest_record_status)

    @property
    def available_buffer_keys(self) -> List[str]:
        """Return the available buffer keys received from follower."""
        with self._available_buffer_keys_lock:
            return list(self._available_buffer_keys)

    # ------------------------------------------------------------------ #
    # Internal helpers
    # ------------------------------------------------------------------ #

    def _has_tool(self, name: str) -> bool:
        return self.core is not None and name in self.core.tool_list()

    def _next_mid(self) -> int:
        with self._mid_lock:
            self._mid_counter = (self._mid_counter + 1) & 0xFFFFFFFF
            return self._mid_counter

    def _send(self, msg_type: int, data: Any) -> None:
        if self._sock is None or self._remote_addr is None:
            return
        try:
            _send_udp(self._sock, self._remote_addr, encode_msg(msg_type, data), self._next_mid())
        except Exception as exc:
            logger.error(f"[TeleopPlugin] UDP send error: {exc}")

    # ------------------------------------------------------------------ #
    # Worker threads
    # ------------------------------------------------------------------ #

    def _state_send_loop(self) -> None:
        """
        Shared by both roles: periodically call ``get_state`` and send the result.

        - Leader sends its own joint state to the follower (which executes it).
          Only sends when teleop is enabled (controlled by start_teleop / stop_teleop)
          AND the peer is connected.
        - Follower sends its own joint state to the leader (for monitoring / force feedback)
          only when the peer is connected.
        """
        interval = 1.0 / self.control_hz
        while not self._stop_event.is_set():
            t0 = time.monotonic()
            try:
                if self._has_tool("get_state"):
                    res = self.core.tool_call("get_state")
                    if res.get("success") and res.get("result"):
                        if self.role == "leader":
                            with self._leader_state_lock:
                                self._latest_leader_state = res["result"]
                            # Only send to follower when both teleop and peer connection are active
                            if self._teleop_enabled and self._peer_connected:
                                self._send(MSG_STATE_DATA, res["result"])
                        else:
                            # Follower: only send state when peer (leader) is connected
                            if self._peer_connected:
                                self._send(MSG_STATE_DATA, res["result"])
            except Exception as exc:
                logger.error(
                    f"[TeleopPlugin][{self.role}] state_send error: {exc}",
                    exc_info=True,
                )
            rem = interval - (time.monotonic() - t0)
            if rem > 0:
                time.sleep(rem)

    def _image_send_loop(self) -> None:
        """
        Follower only: periodically call ``get_image`` and send the encoded bundle.
        Only sends when the peer (leader) is connected.
        """
        interval = 1.0 / self.image_hz
        while not self._stop_event.is_set():
            t0 = time.monotonic()
            try:
                if self._peer_connected:
                    res = self.core.tool_call("get_image")
                    if res.get("success") and res.get("result"):
                        self._send(MSG_IMAGE_DATA, res["result"])
            except Exception as exc:
                logger.error(
                    f"[TeleopPlugin][follower] image_send error: {exc}",
                    exc_info=True,
                )
            rem = interval - (time.monotonic() - t0)
            if rem > 0:
                time.sleep(rem)

    def _record_status_send_loop(self) -> None:
        """
        Follower only: periodically send recording status to leader.
        Only sends when the peer (leader) is connected.
        """
        interval = 1.0  # Send status every 1 second
        while not self._stop_event.is_set():
            t0 = time.monotonic()
            try:
                if self._peer_connected:
                    if self._has_tool("get_record_info"):
                        res = self.core.tool_call("get_record_info")
                        if res.get("success"):
                            self._send(MSG_RECORD_STATUS, res.get("result", {}))
                    else:
                        # Fallback: send basic status indicating no active recording
                        self._send(MSG_RECORD_STATUS, {
                            "recording": False,
                            "frames_written": 0,
                        })
            except Exception as exc:
                logger.error(
                    f"[TeleopPlugin][follower] record_status_send error: {exc}",
                    exc_info=True,
                )
            rem = interval - (time.monotonic() - t0)
            if rem > 0:
                time.sleep(rem)

    def _buffer_keys_send_loop(self) -> None:
        """
        Follower only: periodically send available buffer keys to leader.
        Only sends when the peer (leader) is connected.
        """
        interval = 2.0  # Send keys every 2 seconds
        while not self._stop_event.is_set():
            t0 = time.monotonic()
            try:
                if self._peer_connected:
                    all_keys = set()

                    # Try to get buffer from core's data_server or other servers
                    if self.core and hasattr(self.core, 'data_server'):
                        ds = self.core.data_server
                        if hasattr(ds, 'get_buffer_global'):
                            all_buf = ds.get_buffer_global()
                            for server_name, server_buf in all_buf.items():
                                if isinstance(server_buf, dict):
                                    all_keys.update(server_buf.keys())

                    if all_keys:
                        self._send(MSG_BUFFER_KEYS, {"keys": sorted(list(all_keys))})
            except Exception as exc:
                logger.debug(f"[TeleopPlugin][follower] buffer_keys_send error: {exc}")
            rem = interval - (time.monotonic() - t0)
            if rem > 0:
                time.sleep(rem)

    def _heartbeat_send_loop(self) -> None:
        """Both roles: send heartbeat to the peer every 1 second so the peer can detect connection."""
        interval = 1.0  # 1 Hz heartbeat
        while not self._stop_heartbeat.is_set():
            t0 = time.monotonic()
            try:
                self._send(MSG_HEARTBEAT, {"timestamp": time.time(), "role": self.role})
            except Exception as exc:
                logger.error(f"[TeleopPlugin][{self.role}] heartbeat send error: {exc}")
            rem = interval - (time.monotonic() - t0)
            if rem > 0:
                time.sleep(rem)

    def _connection_monitor_loop(self) -> None:
        """
        Both roles: monitor peer heartbeat and update ``_peer_connected``.

        Sets ``_peer_connected = True`` when heartbeats arrive within the timeout
        window, and resets it to ``False`` when the peer goes silent.
        Logs connection/disconnection events.
        """
        check_interval = 1.0  # check every second
        while not self._stop_event.is_set():
            time.sleep(check_interval)
            now = time.time()
            with self._peer_connected_lock:
                was_connected = self._peer_connected
                if self._last_peer_heartbeat_time is None:
                    currently_connected = False
                else:
                    currently_connected = (
                        now - self._last_peer_heartbeat_time < self._HEARTBEAT_TIMEOUT
                    )
                self._peer_connected = currently_connected
                # Also keep legacy field in sync (leader side used _last_heartbeat_time)
                if self.role == "leader":
                    self._last_heartbeat_time = self._last_peer_heartbeat_time

            if currently_connected and not was_connected:
                logger.info(
                    f"[TeleopPlugin][{self.role}] Peer connected — data exchange enabled."
                )
            elif not currently_connected and was_connected:
                logger.warning(
                    f"[TeleopPlugin][{self.role}] Peer disconnected — data exchange paused."
                )

    def _recv_loop(self) -> None:
        """
        Shared receive loop.  Reassembles fragmented UDP packets and dispatches
        completed messages to :meth:`_dispatch`.
        """
        assembler = _Assembler()
        while not self._stop_event.is_set():
            try:
                data, _ = self._sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break

            hdr = _unpack_header(data)
            if hdr is None:
                continue
            msg_id, frag_idx, total_frags = hdr
            complete = assembler.feed(msg_id, frag_idx, total_frags, data[_HEADER_SIZE:])
            if complete is not None:
                parsed = decode_msg(complete)
                if parsed is not None:
                    self._dispatch(parsed[0], parsed[1])

    def _dispatch(self, msg_type: int, data: Any) -> None:
        """
        Dispatch a fully reassembled and decoded message.

        Leader receives:
          - ``MSG_STATE_DATA``: follower's joint state; optionally apply as force feedback.
          - ``MSG_IMAGE_DATA``: follower's camera images; store for external access.
          - ``MSG_RECORD_STATUS``: follower's recording status.

        Follower receives:
          - ``MSG_STATE_DATA``: leader's joint state; execute via ``run_action_chunk``.
          - ``MSG_START_RECORD``: start data collection.
          - ``MSG_STOP_RECORD``: stop data collection.
          - ``MSG_EXPORT_DATA``: export collected data.
        """
        if self.role == "leader":
            if msg_type == MSG_STATE_DATA:
                state = data
                with self._follower_state_lock:
                    self._latest_follower_state = state
                # Force feedback: apply follower's actual positions to leader arm
                # (only when action_server.outputs is configured on the leader side)
                if self._has_tool("run_action_chunk"):
                    positions = state.get("observation.state")
                    if positions is not None:
                        try:
                            self.core.tool_call(
                                "run_action_chunk",
                                action_chunk={"action": [list(positions)]},
                                fps=0,  # fps=0 disables internal sleep; timing controlled externally
                                post_delay_s=0.0,
                                # Avoid double-execution via _action_watch_loop
                                _skip_buffer_write=True,
                            )
                        except Exception as exc:
                            logger.error(
                                f"[TeleopPlugin][leader] force feedback error: {exc}"
                            )

            elif msg_type == MSG_IMAGE_DATA:
                with self._follower_images_lock:
                    self._latest_follower_images = data

            elif msg_type == MSG_RECORD_STATUS:
                with self._record_status_lock:
                    # Merge leader's own round counter into the status received from follower
                    self._latest_record_status = {**data, "round_number": self._current_round}
                logger.debug(f"[TeleopPlugin][leader] Record status: {data}")

            elif msg_type == MSG_HEARTBEAT:
                self._last_peer_heartbeat_time = time.time()
                self._last_heartbeat_time = self._last_peer_heartbeat_time  # backward compat
                logger.debug(f"[TeleopPlugin][leader] Heartbeat received: {data}")

            elif msg_type == MSG_EXPORT_RESULT:
                # Cache export result and notify Web UI
                self._export_result = data
                self._export_result_time = time.time()
                self._export_progress = None  # clear progress once result arrives
                self._export_started_time = None
                logger.info(f"[TeleopPlugin][leader] Export result received: {data}")

            elif msg_type == MSG_EXPORT_PROGRESS:
                self._export_progress = data
                logger.debug(
                    f"[TeleopPlugin][leader] Export progress: "
                    f"{data.get('progress', 0):.1f}% - {data.get('message', '')}"
                )

            elif msg_type == MSG_EXPORT_STARTED:
                self._export_started_time = time.time()
                logger.info(f"[TeleopPlugin][leader] Export started by follower: {data}")

            elif msg_type == MSG_BUFFER_KEYS:
                # Update available buffer keys from follower
                keys = data.get("keys", [])
                with self._available_buffer_keys_lock:
                    self._available_buffer_keys = keys
                logger.debug(f"[TeleopPlugin][leader] Received buffer keys: {keys}")

            elif msg_type == MSG_DATA_MGMT_RESULT:
                # Store data management result for Web UI to retrieve
                request_id = data.get("request_id")
                if request_id:
                    with self._data_mgmt_results_lock:
                        self._data_mgmt_results[request_id] = {
                            "data": data,
                            "timestamp": time.time(),
                        }
                logger.info(f"[TeleopPlugin][leader] Data mgmt result: {data}")

            elif msg_type == MSG_PLAYBACK_STATUS:
                # Update playback status from follower
                with self._playback_status_lock:
                    self._playback_status = {
                        "status": data.get("status", "unknown"),
                        "progress": data.get("progress", 0),
                        "message": data.get("message", ""),
                        "frame_idx": data.get("frame_idx", 0),
                        "total_frames": data.get("total_frames", 0),
                        "request_id": data.get("request_id", ""),
                        "timestamp": time.time(),
                    }
                    # Update in_progress flag based on status
                    if data.get("status") in ("completed", "stopped", "error"):
                        self._playback_in_progress = False
                logger.debug(f"[TeleopPlugin][leader] Playback status: {data}")

        elif self.role == "follower":
            if msg_type == MSG_STATE_DATA:
                state = data
                positions = state.get("observation.state")
                if positions is None:
                    logger.warning(
                        f"[TeleopPlugin][follower] received state missing "
                        f"'observation.state', keys={list(state.keys())}"
                    )
                    return
                if self._has_tool("run_action_chunk"):
                    # Submit action execution to thread pool to avoid blocking recv loop
                    def _execute_action(pos):
                        try:
                            call_args = {
                                "action_chunk": {"action": [list(pos)]},
                                "fps": 0,  # fps=0 disables internal sleep; timing controlled by leader's send rate
                                "post_delay_s": 0.0,
                                "_skip_buffer_write": False,
                            }
                            res = self.core.tool_call("run_action_chunk", **call_args)
                            # Log first call or failures
                            if not hasattr(self, '_run_action_logged'):
                                logger.info(f"[TeleopPlugin][follower] First run_action_chunk call: {call_args}")
                                logger.info(f"[TeleopPlugin][follower] run_action_chunk result: {res}")
                                self._run_action_logged = True
                            elif not res.get("success"):
                                logger.error(f"[TeleopPlugin][follower] run_action_chunk failed: {res}")
                        except Exception as exc:
                            logger.error(f"[TeleopPlugin][follower] run_action_chunk error: {exc}")

                    if self._action_executor:
                        self._action_executor.submit(_execute_action, positions)
                    else:
                        _execute_action(positions)

            elif msg_type == MSG_START_RECORD:
                self._handle_start_record(data)

            elif msg_type == MSG_STOP_RECORD:
                self._handle_stop_record()

            elif msg_type == MSG_DISCARD_RECORD:
                self._handle_discard_record(data)

            elif msg_type == MSG_EXPORT_DATA:
                self._handle_export_data(data)

            elif msg_type == MSG_DATA_MGMT_SCAN:
                self._handle_data_mgmt_scan(data)

            elif msg_type == MSG_DATA_MGMT_DELETE:
                self._handle_data_mgmt_delete(data)

            elif msg_type == MSG_DATA_MGMT_EXPORT:
                self._handle_data_mgmt_export(data)

            elif msg_type == MSG_DATA_MGMT_ENCODE:
                self._handle_data_mgmt_encode(data)

            elif msg_type == MSG_START_PLAYBACK:
                self._handle_start_playback(data)

            elif msg_type == MSG_STOP_PLAYBACK:
                self._handle_stop_playback(data)

            elif msg_type == MSG_HEARTBEAT:
                # Follower receives heartbeat from leader — update peer timestamp
                self._last_peer_heartbeat_time = time.time()
                logger.debug(f"[TeleopPlugin][follower] Heartbeat received: {data}")

    # ------------------------------------------------------------------ #
    # Follower-side data collection handlers
    # ------------------------------------------------------------------ #

    def _handle_start_record(self, cmd: Dict[str, Any]) -> None:
        """Handle START_RECORD command from leader."""
        if not self._has_tool("start_data_collection"):
            logger.error("[TeleopPlugin][follower] start_data_collection tool not available.")
            return

        try:
            call_args = {
                "keys": cmd.get("keys", []),
                "task_description": cmd.get("task_description", ""),
                "task_prompt": cmd.get("task_prompt", "teleop"),
                "fps": cmd.get("fps", 30.0),
                "round_number": cmd.get("round_number", 1),
                "data_coll_id": cmd.get("data_coll_id"),
            }
            logger.info(f"[TeleopPlugin][follower] Calling start_data_collection with: {call_args}")
            res = self.core.tool_call("start_data_collection", **call_args)
            logger.info(f"[TeleopPlugin][follower] start_data_collection result: {res}")
            if res.get("success"):
                ep_dir = res['result'].get('episode_dir') or res['result'].get('target_dir')
                self._last_episode_dir = ep_dir
                logger.info(
                    f"[TeleopPlugin][follower] Recording started: "
                    f"episode_dir={ep_dir}"
                )
            else:
                logger.error(f"[TeleopPlugin][follower] start_data_collection failed: {res}")
        except Exception as exc:
            logger.error(f"[TeleopPlugin][follower] start_data_collection error: {exc}")

    def _handle_stop_record(self) -> None:
        """Handle STOP_RECORD command from leader."""
        if not self._has_tool("stop_data_collection"):
            logger.error("[TeleopPlugin][follower] stop_data_collection tool not available.")
            return

        try:
            logger.info("[TeleopPlugin][follower] Calling stop_data_collection")
            res = self.core.tool_call("stop_data_collection")
            logger.info(f"[TeleopPlugin][follower] stop_data_collection result: {res}")
            if res.get("success"):
                result = res.get("result", {})
                frames_written = result.get("frames_written", 0)
                logger.info(
                    f"[TeleopPlugin][follower] Recording stopped: "
                    f"frames={frames_written}"
                )
                # Send final record status back to leader
                status_msg = {
                    "recording": False,
                    "frames_written": frames_written,
                    "episode_dir": result.get("episode_dir") or result.get("target_dir"),
                    "round_number": result.get("round_number", 0),
                }
                self._send(MSG_RECORD_STATUS, status_msg)
            else:
                logger.error(f"[TeleopPlugin][follower] stop_data_collection failed: {res}")
        except Exception as exc:
            logger.error(f"[TeleopPlugin][follower] stop_data_collection error: {exc}")

    def _handle_discard_record(self, cmd: Dict[str, Any]) -> None:
        """Handle DISCARD_RECORD command from leader - delete the episode directory.

        Safety checks:
        1. Cannot discard while recording is in progress
        2. Uses provided episode_dir from command if available (reconstruct from round_number as fallback)
        3. Validates the directory contains episode_meta.json before deletion
        """
        import shutil as _shutil
        round_number = cmd.get("round_number")
        if round_number is None:
            logger.warning("[TeleopPlugin][follower] DISCARD_RECORD missing round_number, ignoring")
            return

        # Safety check: cannot discard while recording
        with self._record_status_lock:
            if self._latest_record_status.get("recording", False):
                logger.error(
                    f"[TeleopPlugin][follower] DISCARD_RECORD round={round_number}: "
                    "cannot discard while recording is in progress"
                )
                return

        # Get episode directory: prefer explicit path from command, fallback to last known
        ep_dir = cmd.get("episode_dir") or getattr(self, "_last_episode_dir", None)

        if not ep_dir or not os.path.isdir(ep_dir):
            logger.warning(
                f"[TeleopPlugin][follower] DISCARD_RECORD round={round_number}: "
                f"episode dir not found or unknown (ep_dir={ep_dir})"
            )
            return

        # Safety check: validate this is actually an episode directory
        if not os.path.exists(os.path.join(ep_dir, "episode_meta.json")):
            logger.error(
                f"[TeleopPlugin][follower] DISCARD_RECORD: refusing to delete {ep_dir} "
                "(no episode_meta.json found, not a valid episode directory)"
            )
            return

        # Additional safety: verify the round_number matches the directory
        try:
            ep_name = os.path.basename(ep_dir)
            if ep_name.startswith("episode_"):
                ep_round = int(ep_name.split("_")[1])
                if ep_round != round_number:
                    logger.warning(
                        f"[TeleopPlugin][follower] DISCARD_RECORD: round_number mismatch "
                        f"(cmd={round_number}, dir={ep_round}), proceeding with caution"
                    )
        except (ValueError, IndexError):
            pass  # Non-standard episode name format, proceed anyway

        try:
            _shutil.rmtree(ep_dir)
            logger.info(f"[TeleopPlugin][follower] Discarded episode dir: {ep_dir}")
            # Clear last_episode_dir only if it matches the deleted directory
            if getattr(self, "_last_episode_dir", None) == ep_dir:
                self._last_episode_dir = None
        except Exception as e:
            logger.error(f"[TeleopPlugin][follower] Failed to delete episode dir {ep_dir}: {e}")

    def _handle_export_data(self, cmd: Dict[str, Any]) -> None:
        """Handle EXPORT_DATA command from leader - run export in background thread."""
        if not self._has_tool("export_task_episodes"):
            logger.error("[TeleopPlugin][follower] export_task_episodes tool not available.")
            self._send(MSG_EXPORT_RESULT, {
                "success": False,
                "zip_path": None,
                "exported_count": 0,
                "message": "export_task_episodes tool not available"
            })
            return

        data_coll_id = cmd.get("data_coll_id", "teleop")

        # Progress adapter: forwards ProgressTracker callbacks over UDP to the leader
        plugin_ref = self  # closure

        class _TeleopProgressAdapter:
            """Synchronous progress callback that sends MSG_EXPORT_PROGRESS to leader."""

            _MIN_DELTA_PCT: float = 1.0
            _MIN_INTERVAL_S: float = 0.5

            def __init__(self) -> None:
                self._last_progress: float = -1.0
                self._last_time: float = 0.0

            def __call__(self, current: float, total: float, message: str) -> None:
                import time as _time
                now = _time.monotonic()
                if (abs(current - self._last_progress) < self._MIN_DELTA_PCT and
                        (now - self._last_time) < self._MIN_INTERVAL_S):
                    return  # throttle
                self._last_progress = current
                self._last_time = now
                try:
                    plugin_ref._send(MSG_EXPORT_PROGRESS, {
                        "progress": current,
                        "total": total,
                        "message": message,
                    })
                except Exception:
                    pass

        progress_adapter = _TeleopProgressAdapter()

        # Notify leader that export has started before spawning background thread
        self._send(MSG_EXPORT_STARTED, {"data_coll_id": data_coll_id})
        logger.info(f"[TeleopPlugin][follower] Sent MSG_EXPORT_STARTED for {data_coll_id}")

        def do_export():
            try:
                logger.info(f"[TeleopPlugin][follower] Starting export for data_coll_id={data_coll_id}")
                res = self.core.tool_call(
                    "export_task_episodes",
                    data_coll_id=data_coll_id,
                    _progress_callback=progress_adapter,
                )
                logger.info(f"[TeleopPlugin][follower] export_task_episodes result: {res}")

                if res.get("success"):
                    result_data = res.get("result", {})
                    zip_path = result_data.get("zip_path")
                    exported = result_data.get("exported", [])
                    failed = result_data.get("failed", [])
                    exported_count = len(exported)

                    msg = f"导出完成: {exported_count} 条数据"
                    if failed:
                        msg += f", {len(failed)} 条失败"
                    if zip_path:
                        msg += f", 文件: {zip_path}"

                    self._send(MSG_EXPORT_RESULT, {
                        "success": True,
                        "zip_path": zip_path,
                        "exported_count": exported_count,
                        "message": msg,
                        "failed_count": len(failed),
                    })
                    logger.info(f"[TeleopPlugin][follower] Data exported: zip={zip_path}")
                else:
                    self._send(MSG_EXPORT_RESULT, {
                        "success": False,
                        "zip_path": None,
                        "exported_count": 0,
                        "message": res.get("message", "导出失败")
                    })
                    logger.error(f"[TeleopPlugin][follower] export_task_episodes failed: {res}")
            except Exception as exc:
                logger.error(f"[TeleopPlugin][follower] export_task_episodes error: {exc}")
                self._send(MSG_EXPORT_RESULT, {
                    "success": False,
                    "zip_path": None,
                    "exported_count": 0,
                    "message": f"导出异常: {exc}"
                })

        # Run export in background thread to avoid blocking
        threading.Thread(target=do_export, daemon=True, name="teleop-export").start()

    # ------------------------------------------------------------------ #
    # Follower-side data management handlers
    # ------------------------------------------------------------------ #

    def _handle_data_mgmt_scan(self, cmd: Dict[str, Any]) -> None:
        """Handle DATA_MGMT_SCAN command from leader."""
        request_id = cmd.get("request_id", "")
        logger.info(f"[TeleopPlugin][follower] Data mgmt scan request: {request_id}")

        try:
            result = self.scan_local_records()
            # scan_local_records on follower returns the data directly (not {success, result, message})
            self._send(MSG_DATA_MGMT_RESULT, {
                "request_id": request_id,
                "success": True,
                "data": result,  # This is the scan result dict
                "message": "Scan completed",
            })
        except Exception as e:
            logger.error(f"[TeleopPlugin][follower] Data mgmt scan error: {e}")
            self._send(MSG_DATA_MGMT_RESULT, {
                "request_id": request_id,
                "success": False,
                "data": {},
                "message": str(e),
            })

    def _handle_data_mgmt_delete(self, cmd: Dict[str, Any]) -> None:
        """Handle DATA_MGMT_DELETE command from leader."""
        request_id = cmd.get("request_id", "")
        path = cmd.get("path")
        data_coll_id = cmd.get("data_coll_id")

        logger.info(f"[TeleopPlugin][follower] Data mgmt delete request: {request_id}, path={path}, coll={data_coll_id}")

        try:
            if path:
                result = self.delete_episode(path)
            elif data_coll_id:
                result = self.delete_data_collection(data_coll_id)
            else:
                result = {"success": False, "message": "No path or data_coll_id specified"}

            self._send(MSG_DATA_MGMT_RESULT, {
                "request_id": request_id,
                "success": result.get("success", False),
                "data": result,  # delete returns {success, message} format
                "message": result.get("message", ""),
            })
        except Exception as e:
            logger.error(f"[TeleopPlugin][follower] Data mgmt delete error: {e}")
            self._send(MSG_DATA_MGMT_RESULT, {
                "request_id": request_id,
                "success": False,
                "data": {},
                "message": str(e),
            })

    def _handle_data_mgmt_export(self, cmd: Dict[str, Any]) -> None:
        """Handle DATA_MGMT_EXPORT command from leader."""
        request_id = cmd.get("request_id", "")
        episode_paths = cmd.get("episode_paths", [])
        zip_name = cmd.get("zip_name")

        logger.info(f"[TeleopPlugin][follower] Data mgmt export request: {request_id}, paths={len(episode_paths)}")

        def do_export():
            try:
                result = self.export_episodes(episode_paths, zip_name)
                self._send(MSG_DATA_MGMT_RESULT, {
                    "request_id": request_id,
                    "success": result.get("success", False),
                    "data": result.get("result", {}),
                    "message": result.get("message", ""),
                })
            except Exception as e:
                logger.error(f"[TeleopPlugin][follower] Data mgmt export error: {e}")
                self._send(MSG_DATA_MGMT_RESULT, {
                    "request_id": request_id,
                    "success": False,
                    "data": {},
                    "message": str(e),
                })

        # Run export in background thread
        threading.Thread(target=do_export, daemon=True, name="teleop-data-mgmt-export").start()

    def _handle_data_mgmt_encode(self, cmd: Dict[str, Any]) -> None:
        """Handle DATA_MGMT_ENCODE command from leader."""
        request_id = cmd.get("request_id", "")
        episode_paths = cmd.get("episode_paths", [])

        logger.info(f"[TeleopPlugin][follower] Data mgmt encode request: {request_id}, paths={len(episode_paths)}")

        # Progress adapter: forwards encode progress to leader
        plugin_ref = self  # closure

        class _EncodeProgressAdapter:
            """Synchronous progress callback that sends encode progress to leader."""

            _MIN_DELTA_PCT: float = 1.0
            _MIN_INTERVAL_S: float = 0.5

            def __init__(self) -> None:
                self._last_progress: float = -1.0
                self._last_time: float = 0.0

            def __call__(self, current: float, total: float, message: str) -> None:
                import time as _time
                now = _time.monotonic()
                if (abs(current - self._last_progress) < self._MIN_DELTA_PCT and
                        (now - self._last_time) < self._MIN_INTERVAL_S):
                    return  # throttle
                self._last_progress = current
                self._last_time = now
                try:
                    plugin_ref._send(MSG_EXPORT_PROGRESS, {
                        "progress": current,
                        "total": total,
                        "message": message,
                    })
                except Exception:
                    pass

        progress_adapter = _EncodeProgressAdapter()

        def do_encode():
            try:
                result = self.encode_episodes(
                    episode_paths,
                    _progress_callback=progress_adapter,
                )
                self._send(MSG_DATA_MGMT_RESULT, {
                    "request_id": request_id,
                    "success": result.get("success", False),
                    "data": result.get("result", {}),
                    "message": result.get("message", ""),
                })
            except Exception as e:
                logger.error(f"[TeleopPlugin][follower] Data mgmt encode error: {e}")
                self._send(MSG_DATA_MGMT_RESULT, {
                    "request_id": request_id,
                    "success": False,
                    "data": {},
                    "message": str(e),
                })

        # Run encode in background thread
        threading.Thread(target=do_encode, daemon=True, name="teleop-data-mgmt-encode").start()

    # ------------------------------------------------------------------ #
    # Follower-side playback handlers
    # ------------------------------------------------------------------ #

    def _handle_start_playback(self, cmd: Dict[str, Any]) -> None:
        """Handle START_PLAYBACK command from leader."""
        episode_path = cmd.get("episode_path", "")
        request_id = cmd.get("request_id", "")
        logger.info(f"[TeleopPlugin][follower] Playback start request: {request_id}, path={episode_path}")

        # Check if playback is already running
        if getattr(self, '_playback_thread', None) and self._playback_thread.is_alive():
            logger.warning("[TeleopPlugin][follower] Playback already in progress, ignoring new request.")
            self._send_playback_status(request_id, "error", 0, "Playback already in progress", 0, 0)
            return

        # Start playback in background thread
        self._playback_stop_event = threading.Event()
        self._playback_thread = threading.Thread(
            target=self._playback_loop,
            args=(episode_path, request_id),
            daemon=True,
            name="teleop-playback"
        )
        self._playback_thread.start()

    def _handle_stop_playback(self, cmd: Dict[str, Any]) -> None:
        """Handle STOP_PLAYBACK command from leader."""
        request_id = cmd.get("request_id", "")
        logger.info(f"[TeleopPlugin][follower] Playback stop request: {request_id}")

        if hasattr(self, '_playback_stop_event'):
            self._playback_stop_event.set()
            self._send_playback_status(request_id, "stopped", 0, "Playback stopped by user", 0, 0)

    def _send_playback_status(self, request_id: str, status: str, progress: int, message: str, frame_idx: int, total_frames: int) -> None:
        """Send playback status to leader."""
        try:
            self._send(MSG_PLAYBACK_STATUS, {
                "request_id": request_id,
                "status": status,
                "progress": progress,
                "message": message,
                "frame_idx": frame_idx,
                "total_frames": total_frames,
            })
        except Exception as e:
            logger.error(f"[TeleopPlugin][follower] Failed to send playback status: {e}")

    def _playback_loop(self, episode_path: str, request_id: str) -> None:
        """
        Main playback loop running in background thread.
        
        Steps:
        1. Load episode data
        2. Pre-move to first frame position (linear interpolation)
        3. Wait 2 seconds
        4. Playback actions according to original timestamps
        """
        import numpy as np

        # Step 1: Load episode data
        self._send_playback_status(request_id, "loading", 0, "Loading episode data...", 0, 0)
        
        if not self._has_tool("load_episode_for_playback"):
            logger.error("[TeleopPlugin][follower] load_episode_for_playback tool not available.")
            self._send_playback_status(request_id, "error", 0, "Playback tool not available", 0, 0)
            return

        try:
            res = self.core.tool_call("load_episode_for_playback", episode_path=episode_path)
            if not res.get("success"):
                logger.error(f"[TeleopPlugin][follower] Failed to load episode: {res}")
                self._send_playback_status(request_id, "error", 0, f"Failed to load episode: {res.get('message', '')}", 0, 0)
                return

            result = res.get("result", {})
            timestamps = result.get("timestamps", [])
            actions = result.get("actions", [])
            first_state = result.get("first_state")
            total_frames = result.get("total_frames", 0)
            
            if not timestamps or not actions:
                logger.error("[TeleopPlugin][follower] Episode data is empty.")
                self._send_playback_status(request_id, "error", 0, "Episode data is empty", 0, 0)
                return

            logger.info(f"[TeleopPlugin][follower] Episode loaded: {total_frames} frames, format={result.get('format')}")

        except Exception as e:
            logger.exception(f"[TeleopPlugin][follower] Error loading episode: {e}")
            self._send_playback_status(request_id, "error", 0, f"Error loading episode: {e}", 0, 0)
            return

        # Step 2: Pre-move to first frame position
        if first_state and self._has_tool("run_action_chunk"):
            self._send_playback_status(request_id, "pre_move", 0, "Moving to start position...", 0, total_frames)
            
            try:
                # Get current position
                current_pos = None
                if self._has_tool("get_state"):
                    state_res = self.core.tool_call("get_state")
                    if state_res.get("success"):
                        current_pos = state_res.get("result", {}).get("observation.state")
                
                if current_pos is not None:
                    # Linear interpolation pre-move (3 seconds, 30Hz)
                    duration = 3.0
                    steps = int(duration * 30)
                    current_pos = np.array(current_pos)
                    target_pos = np.array(first_state)
                    
                    for i in range(steps):
                        if self._playback_stop_event.is_set():
                            self._send_playback_status(request_id, "stopped", 0, "Playback stopped during pre-move", 0, total_frames)
                            return
                        
                        t = i / steps
                        interp_pos = current_pos * (1 - t) + target_pos * t
                        
                        self.core.tool_call(
                            "run_action_chunk",
                            action_chunk={"action": [interp_pos.tolist()]},
                            fps=0,
                            post_delay_s=0.0,
                            _skip_buffer_write=True,
                        )
                        
                        # Progress: 0-50% for pre-move
                        progress = int((i / steps) * 50)
                        if i % 10 == 0:  # Update every 10 frames to reduce network traffic
                            self._send_playback_status(request_id, "pre_move", progress, f"Moving to start position: {progress*2}%", 0, total_frames)
                        
                        time.sleep(1.0 / 30)
                else:
                    # Direct move if cannot get current position
                    self.core.tool_call(
                        "run_action_chunk",
                        action_chunk={"action": [list(first_state)]},
                        fps=0,
                        post_delay_s=0.0,
                        _skip_buffer_write=True,
                    )
                    
            except Exception as e:
                logger.error(f"[TeleopPlugin][follower] Pre-move error: {e}")
                self._send_playback_status(request_id, "error", 0, f"Pre-move error: {e}", 0, total_frames)
                return

        # Step 3: Wait 2 seconds
        self._send_playback_status(request_id, "waiting", 50, "Ready to start in 2.0s", 0, total_frames)
        for i in range(20):
            if self._playback_stop_event.is_set():
                self._send_playback_status(request_id, "stopped", 50, "Playback stopped during wait", 0, total_frames)
                return
            # Countdown progress: 50-60%
            progress = 50 + int((i / 20) * 10)
            remaining = 2.0 - i * 0.1
            if i % 5 == 0:
                self._send_playback_status(request_id, "waiting", progress, f"Ready to start in {remaining:.1f}s", 0, total_frames)
            time.sleep(0.1)

        # Step 4: Playback actions according to timestamps
        self._send_playback_status(request_id, "playing", 60, "Starting playback...", 0, total_frames)
        
        t0 = timestamps[0]
        playback_start_time = time.monotonic()
        
        for i in range(len(timestamps)):
            if self._playback_stop_event.is_set():
                self._send_playback_status(request_id, "stopped", 60 + int((i / total_frames) * 40), "Playback stopped", i, total_frames)
                return

            # Calculate target time for this frame
            target_time = timestamps[i] - t0
            
            # Wait until target time
            while True:
                elapsed = time.monotonic() - playback_start_time
                if elapsed >= target_time:
                    break
                time.sleep(0.001)  # 1ms sleep to avoid busy waiting

            # Execute action
            try:
                action = actions[i]
                self.core.tool_call(
                    "run_action_chunk",
                    action_chunk={"action": [list(action)]},
                    fps=0,
                    post_delay_s=0.0,
                    _skip_buffer_write=True,
                )
                
                # Update progress: 60-100%
                progress = 60 + int((i / total_frames) * 40)
                if i % 10 == 0 or i == total_frames - 1:
                    self._send_playback_status(request_id, "playing", progress, f"Playing: frame {i+1}/{total_frames}", i+1, total_frames)
                    
            except Exception as e:
                logger.error(f"[TeleopPlugin][follower] Playback execution error at frame {i}: {e}")
                self._send_playback_status(request_id, "error", progress, f"Execution error at frame {i}: {e}", i, total_frames)
                return

        # Playback completed
        self._send_playback_status(request_id, "completed", 100, f"Playback completed: {total_frames} frames", total_frames, total_frames)
        logger.info(f"[TeleopPlugin][follower] Playback completed: {request_id}")

    # ------------------------------------------------------------------ #
    # Public data access (for application scripts on the leader side)
    # ------------------------------------------------------------------ #

    @property
    def latest_follower_state(self) -> Optional[Dict[str, Any]]:
        """Latest state dict received from the follower arm (leader side only)."""
        with self._follower_state_lock:
            return self._latest_follower_state

    @property
    def latest_leader_state(self) -> Optional[Dict[str, Any]]:
        """Latest state dict read from the leader arm itself (leader side only)."""
        with self._leader_state_lock:
            return self._latest_leader_state

    @property
    def latest_follower_images(self) -> Optional[Dict[str, bytes]]:
        """Latest image dict (key → JPEG bytes) received from the follower arm (leader side only)."""
        with self._follower_images_lock:
            return self._latest_follower_images

    # ------------------------------------------------------------------ #
    # Data Management Module (leader side)
    # ------------------------------------------------------------------ #

    def _send_data_mgmt_command(self, msg_type: int, data: Dict[str, Any], timeout: float = 30.0) -> Dict[str, Any]:
        """
        Send a data management command to follower and wait for result.
        Only used when running as leader.
        """
        if self.role != "leader":
            return {"success": False, "message": "Only leader can send data mgmt commands"}

        request_id = f"mgmt_{int(time.time() * 1000)}_{id(data) % 10000}"
        data["request_id"] = request_id

        # Clear any old result for this request
        with self._data_mgmt_results_lock:
            self._data_mgmt_results.pop(request_id, None)

        # Send command to follower
        self._send(msg_type, data)
        logger.info(f"[TeleopPlugin][leader] Sent data mgmt command: {msg_type}, request_id={request_id}")

        # Wait for result
        start_time = time.time()
        while time.time() - start_time < timeout:
            with self._data_mgmt_results_lock:
                if request_id in self._data_mgmt_results:
                    result = self._data_mgmt_results.pop(request_id)
                    follower_msg = result.get("data", {})
                    # Follower message format: {request_id, success, data, message}
                    # where 'data' contains the actual result payload
                    return {
                        "success": follower_msg.get("success", False),
                        "result": follower_msg.get("data", {}),
                        "message": follower_msg.get("message", ""),
                    }
            time.sleep(0.1)

        return {"success": False, "message": f"Timeout waiting for result (>{timeout}s)"}

    def scan_local_records(self) -> Dict[str, Any]:
        """
        Scan all local data collection directories and return their status.
        On leader: sends command to follower.
        On follower: scans local filesystem.

        Returns:
            Dict with structure:
            {
                "export_dir": str,  # path to export directory
                "exported_zips": [{"name": str, "path": str, "size": int}, ...],
                "data_collections": [
                    {
                        "data_coll_id": str,
                        "path": str,
                        "task_prompts": [
                            {
                                "task_prompt": str,
                                "path": str,
                                "episodes": [
                                    {
                                        "episode_name": str,
                                        "path": str,
                                        "status": "complete" | "incomplete" | "exported",
                                        "frames": int,
                                        "has_video": bool,
                                        "has_metadata": bool,
                                    }, ...
                                ]
                            }, ...
                        ]
                    }, ...
                ]
            }
        """
        # If leader, send command to follower
        if self.role == "leader":
            result = self._send_data_mgmt_command(MSG_DATA_MGMT_SCAN, {})
            if result.get("success"):
                return result.get("result", {})
            return {"export_dir": "", "exported_zips": [], "data_collections": [], "error": result.get("message")}

        # Follower: scan local filesystem
        root_dir = os.path.expanduser("~/.cache/RynnRCPData/data_coll")
        export_dir = os.path.join(root_dir, "export_dir")

        def get_dir_size(path: str) -> int:
            """Calculate total size of a directory in bytes."""
            total = 0
            try:
                for dirpath, dirnames, filenames in os.walk(path):
                    for f in filenames:
                        fp = os.path.join(dirpath, f)
                        if os.path.exists(fp):
                            total += os.path.getsize(fp)
            except Exception as e:
                logger.warning(f"[TeleopPlugin] get_dir_size error for {path}: {e}")
            return total

        result = {
            "export_dir": export_dir,
            "exported_zips": [],
            "data_collections": [],
            "total_size_bytes": 0,
            "total_size_formatted": "0 B",
        }

        # Scan exported zips
        if os.path.isdir(export_dir):
            for fn in os.listdir(export_dir):
                if fn.endswith(".zip"):
                    fp = os.path.join(export_dir, fn)
                    try:
                        st = os.stat(fp)
                        result["exported_zips"].append({
                            "name": fn,
                            "path": fp,
                            "size": st.st_size,
                            "mtime": st.st_mtime,
                        })
                    except Exception:
                        pass
            result["exported_zips"].sort(key=lambda x: x["mtime"], reverse=True)

        # Scan data collections
        if not os.path.isdir(root_dir):
            logger.info(f"[TeleopPlugin] root_dir not found: {root_dir}")
            return result

        logger.info(f"[TeleopPlugin] Scanning data collections in: {root_dir}")

        for data_coll_name in os.listdir(root_dir):
            if data_coll_name == "export_dir":
                continue
            data_coll_path = os.path.join(root_dir, data_coll_name)
            if not os.path.isdir(data_coll_path):
                continue

            logger.info(f"[TeleopPlugin] Found data collection: {data_coll_name}")

            data_coll_entry = {
                "data_coll_id": data_coll_name,
                "path": data_coll_path,
                "task_prompts": [],
            }

            # Scan task prompts
            for task_name in os.listdir(data_coll_path):
                task_path = os.path.join(data_coll_path, task_name)
                if not os.path.isdir(task_path):
                    continue

                task_entry = {
                    "task_prompt": task_name,
                    "path": task_path,
                    "episodes": [],
                }

                logger.info(f"[TeleopPlugin]  Scanning task: {task_name}")

                # Scan episodes
                for ep_name in os.listdir(task_path):
                    if not ep_name.startswith("episode_"):
                        continue
                    ep_path = os.path.join(task_path, ep_name)
                    if not os.path.isdir(ep_path):
                        continue

                    # Determine episode status
                    # State 1 (raw): has streams/ + episode_meta.json -> can encode or export
                    # State 2 (encoded): has metadata.json + timeseries.parquet + .mp4 videos, no streams/ -> can only export
                    # Invalid: any other format -> recommend delete
                    has_meta = os.path.exists(os.path.join(ep_path, "metadata.json"))
                    has_parquet = os.path.exists(os.path.join(ep_path, "timeseries.parquet"))
                    has_video = any(f.endswith(".mp4") for f in os.listdir(ep_path) if os.path.isfile(os.path.join(ep_path, f)))
                    has_streams = os.path.isdir(os.path.join(ep_path, "streams"))
                    has_ep_meta = os.path.exists(os.path.join(ep_path, "episode_meta.json"))

                    # State 2: Fully encoded (no streams, has all required files)
                    if not has_streams and has_meta and has_parquet and has_video:
                        status = "complete"
                    # State 1: Raw data with streams
                    elif has_streams and has_ep_meta:
                        status = "incomplete"
                    # Invalid format
                    else:
                        status = "invalid"

                    # Count frames if episode_meta.json exists
                    frames = 0
                    try:
                        if has_ep_meta:
                            with open(os.path.join(ep_path, "episode_meta.json"), "r") as f:
                                ep_meta = json.load(f)
                                frames = ep_meta.get("frames_written", 0)
                    except Exception as e:
                        logger.warning(f"[TeleopPlugin] Failed to read frames from {ep_path}: {e}")

                    task_entry["episodes"].append({
                        "episode_name": ep_name,
                        "path": ep_path,
                        "status": status,
                        "frames": frames,
                        "has_video": has_video,
                        "has_metadata": has_meta,
                    })
                    logger.info(f"[TeleopPlugin]    Episode {ep_name}: status={status}, frames={frames}")

                if task_entry["episodes"]:
                    task_entry["episodes"].sort(key=lambda x: x["episode_name"])
                    data_coll_entry["task_prompts"].append(task_entry)
                    logger.info(f"[TeleopPlugin]  Task {task_name}: {len(task_entry['episodes'])} episodes")
                else:
                    logger.info(f"[TeleopPlugin]  Task {task_name}: no episodes found")

            if data_coll_entry["task_prompts"]:
                data_coll_entry["task_prompts"].sort(key=lambda x: x["task_prompt"])
                # Calculate data collection size
                data_coll_entry["size_bytes"] = get_dir_size(data_coll_path)
                data_coll_entry["size_formatted"] = self._format_bytes(data_coll_entry["size_bytes"])
                logger.info(f"[TeleopPlugin] Data collection {data_coll_name}: {len(data_coll_entry['task_prompts'])} tasks, size={data_coll_entry['size_formatted']}")
                result["data_collections"].append(data_coll_entry)
            else:
                logger.info(f"[TeleopPlugin] Data collection {data_coll_name}: no tasks found")

        result["data_collections"].sort(key=lambda x: x["data_coll_id"], reverse=True)

        # Calculate total size
        total_size = sum(dc.get("size_bytes", 0) for dc in result["data_collections"])
        total_size += sum(z.get("size", 0) for z in result["exported_zips"])
        result["total_size_bytes"] = total_size
        result["total_size_formatted"] = self._format_bytes(total_size)

        logger.info(f"[TeleopPlugin] Scan complete: {len(result['data_collections'])} collections, {len(result['exported_zips'])} zips, total={result['total_size_formatted']}")

        return result

    def _format_bytes(self, bytes: int) -> str:
        """Format bytes to human readable string."""
        if bytes == 0:
            return "0 B"
        k = 1024
        sizes = ["B", "KB", "MB", "GB", "TB"]
        i = int(bytes.bit_length() // 10)  # log2(bytes) / log2(1024)
        if i >= len(sizes):
            i = len(sizes) - 1
        return f"{bytes / (k ** i):.2f} {sizes[i]}"

    def delete_episode(self, episode_path: str) -> Dict[str, Any]:
        """
        Delete a specific episode directory.
        On leader: sends command to follower.
        On follower: deletes from local filesystem.

        Args:
            episode_path: Full path to the episode directory.

        Returns:
            Result dict with success status.
        """
        # If leader, send command to follower
        if self.role == "leader":
            return self._send_data_mgmt_command(MSG_DATA_MGMT_DELETE, {"path": episode_path})

        # Follower: delete from local filesystem
        import shutil
        try:
            if not os.path.isdir(episode_path):
                return {"success": False, "message": "Episode directory not found"}
            shutil.rmtree(episode_path)
            logger.info(f"[TeleopPlugin] deleted episode: {episode_path}")
            return {"success": True, "message": "Episode deleted"}
        except Exception as e:
            logger.error(f"[TeleopPlugin] failed to delete episode {episode_path}: {e}")
            return {"success": False, "message": str(e)}

    def delete_data_collection(self, data_coll_id: str) -> Dict[str, Any]:
        """
        Delete an entire data collection directory.
        On leader: sends command to follower.
        On follower: deletes from local filesystem.

        Args:
            data_coll_id: The data collection ID.

        Returns:
            Result dict with success status.
        """
        # If leader, send command to follower
        if self.role == "leader":
            return self._send_data_mgmt_command(MSG_DATA_MGMT_DELETE, {"data_coll_id": data_coll_id})

        # Follower: delete from local filesystem
        import shutil
        root_dir = os.path.expanduser("~/.cache/RynnRCPData/data_coll")
        data_coll_path = os.path.join(root_dir, data_coll_id)
        try:
            if not os.path.isdir(data_coll_path):
                return {"success": False, "message": "Data collection not found"}
            shutil.rmtree(data_coll_path)
            logger.info(f"[TeleopPlugin] deleted data collection: {data_coll_path}")
            return {"success": True, "message": "Data collection deleted"}
        except Exception as e:
            logger.error(f"[TeleopPlugin] failed to delete data collection {data_coll_id}: {e}")
            return {"success": False, "message": str(e)}

    def export_episodes(
        self,
        episode_paths: List[str],
        zip_name: Optional[str] = None,
        _progress_callback: Optional[callable] = None,
    ) -> Dict[str, Any]:
        """
        Export specific episodes.
        On leader: sends command to follower.
        On follower: uses data_server tool.

        Args:
            episode_paths: List of episode directory paths to export.
            zip_name: Optional name for the output zip file.
            _progress_callback: Optional progress callback (current, total, message) -> None.
                               Only used on follower side.

        Returns:
            Result dict from the export operation.
        """
        # If leader, send command to follower
        if self.role == "leader":
            return self._send_data_mgmt_command(
                MSG_DATA_MGMT_EXPORT,
                {"episode_paths": episode_paths, "zip_name": zip_name},
                timeout=120.0  # Export may take longer
            )

        # Follower: use data_server tool
        if not self.core:
            return {"success": False, "message": "Core not initialized"}

        try:
            res = self.core.tool_call(
                "export_specific_episodes",
                episode_paths=episode_paths,
                zip_name=zip_name,
                _progress_callback=_progress_callback,
            )
            return res
        except Exception as e:
            logger.error(f"[TeleopPlugin] export_episodes failed: {e}")
            return {"success": False, "message": str(e)}

    def encode_episodes(
        self,
        episode_paths: List[str],
        _progress_callback: Optional[callable] = None,
    ) -> Dict[str, Any]:
        """
        Encode (export in-place) specific episodes without creating ZIP.
        On leader: sends command to follower.
        On follower: uses data_server tool.

        Args:
            episode_paths: List of episode directory paths to encode.
            _progress_callback: Optional progress callback (current, total, message) -> None.
                               Only used on follower side.

        Returns:
            Result dict from the encode operation.
        """
        # If leader, send command to follower
        if self.role == "leader":
            return self._send_data_mgmt_command(
                MSG_DATA_MGMT_ENCODE,
                {"episode_paths": episode_paths},
                timeout=120.0  # Encode may take longer
            )

        # Follower: use data_server tool
        if not self.core:
            return {"success": False, "message": "Core not initialized"}

        try:
            res = self.core.tool_call(
                "encode_episodes",
                episode_paths=episode_paths,
                _progress_callback=_progress_callback,
            )
            return res
        except Exception as e:
            logger.error(f"[TeleopPlugin] encode_episodes failed: {e}")
            return {"success": False, "message": str(e)}
