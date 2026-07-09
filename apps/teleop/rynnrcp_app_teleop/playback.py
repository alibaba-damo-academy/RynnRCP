"""Teleop collection playback support."""

from __future__ import annotations

import logging
from pathlib import Path
import shutil
import threading
from typing import Any, Callable, Dict, List, Optional

import msgpack

from rynnrcp.interface.protocol_client import RcpProtocolClient
from rynnrcp.utils import safe_name

from .recording_manager import TeleopRecordingManager
from .utils import as_float_list


class TeleopPlaybackManager:
    def __init__(
        self,
        *,
        config: Dict[str, Any],
        records: TeleopRecordingManager,
        get_protocol_client: Callable[[], RcpProtocolClient | None],
        get_observation_name: Callable[[], str],
        get_action_name: Callable[[], str],
        send_status: Callable[[Dict[str, Any]], None],
        logger: logging.Logger,
    ) -> None:
        self._config = config
        self._records = records
        self._get_protocol_client = get_protocol_client
        self._get_observation_name = get_observation_name
        self._get_action_name = get_action_name
        self._send_status_payload = send_status
        self._logger = logger
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self.status: Optional[Dict[str, Any]] = None
        self.in_progress = False

    def mark_requested(self) -> None:
        self.in_progress = True

    def start(self, episode_path: str, *, cleanup_path: str | None = None) -> None:
        if self._thread and self._thread.is_alive():
            self._send_status("error", message="playback already running")
            self._cleanup_path(cleanup_path)
            return
        if not episode_path:
            self._send_status("error", message="episode_path required")
            self._cleanup_path(cleanup_path)
            return
        try:
            episode_dir = self._records.safe_record_path(str(episode_path))
        except Exception as exc:
            self._send_status("error", message=str(exc))
            self._cleanup_path(cleanup_path)
            return
        return_position = self._read_current_joint_positions()
        if not return_position:
            self._send_status("error", message="current state is empty; cannot preserve return position")
            self._cleanup_path(cleanup_path)
            return
        self._stop_event.clear()
        self.in_progress = True
        self._thread = threading.Thread(
            target=self._loop,
            args=(episode_dir, return_position, cleanup_path),
            daemon=True,
            name="teleop-playback",
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._send_status("returning", message="stop requested; returning to start position")
            return
        self.in_progress = False
        self._send_status("stopped", progress=0.0, message="stopped")

    def shutdown(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3.0)
        self._thread = None

    def set_remote_status(self, status: Dict[str, Any]) -> None:
        self.status = status
        if status.get("status") in ("completed", "stopped", "error"):
            self.in_progress = False

    def _loop(self, episode_dir: str, return_position: List[float], cleanup_path: str | None = None) -> None:
        try:
            samples = self._load_samples(episode_dir)
            if not samples:
                self._send_status("error", message="no protocol action stream found")
                return
            total = len(samples)
            self._send_status("loading", progress=0.0, total=total, episode_path=episode_dir)
            control_hz = int(self._config.get("control_hz", 30))
            if bool(self._config.get("playback_move_to_start", True)):
                if not self._move_to_position(
                    samples[0][1],
                    control_hz,
                    total,
                    status="pre_move",
                    error_context="move_to_start",
                ):
                    if self._stop_event.is_set():
                        self._return_after_stop(return_position, control_hz, total, progress=0.0)
                    return
            fallback_interval = 1.0 / max(float(control_hz), 1.0)
            previous_ts = samples[0][0]
            for index, (timestamp, action_value) in enumerate(samples):
                if self._stop_event.is_set():
                    self._return_after_stop(return_position, control_hz, total, progress=index / max(total, 1))
                    return
                if index > 0:
                    delay = max(0.0, min(float(timestamp - previous_ts), 2.0))
                    if delay == 0.0:
                        delay = fallback_interval
                    if self._stop_event.wait(delay):
                        self._return_after_stop(return_position, control_hz, total, progress=index / max(total, 1))
                        return
                previous_ts = timestamp
                self._send_status(
                    "playing",
                    progress=(index + 1) / max(total, 1),
                    current_frame=index + 1,
                    total=total,
                )
                if not self._send_action_batch([action_value], control_hz, "playback"):
                    return
            if not self._move_to_position(
                return_position,
                control_hz,
                total,
                status="returning",
                error_context="return_to_start_position",
                interruptible=False,
            ):
                return
            self.in_progress = False
            if self._stop_event.is_set():
                self._send_status(
                    "stopped",
                    progress=1.0,
                    total=total,
                    message="returned_to_start_position",
                )
            else:
                self._send_status("completed", progress=1.0, total=total)
        except Exception as exc:
            self._logger.exception("playback failed")
            self.in_progress = False
            self._send_status("error", message=str(exc))
        finally:
            if cleanup_path:
                self._cleanup_path(cleanup_path)

    @staticmethod
    def _cleanup_path(path: str | None) -> None:
        if path:
            shutil.rmtree(path, ignore_errors=True)

    def _move_to_position(
        self,
        target_position: Any,
        control_hz: int,
        total_frames: int,
        *,
        status: str,
        error_context: str,
        interruptible: bool = True,
    ) -> bool:
        client = self._get_protocol_client()
        if client is None:
            self._send_status("error", message=f"target protocol client is required for {error_context}")
            return False
        current = self._read_current_joint_positions()
        target = _joint_positions(target_position)
        if not current or not target:
            self._send_status("error", message=f"current state or target position is empty for {error_context}")
            return False
        if len(current) != len(target):
            self._send_status(
                "error",
                message=f"state/action dimension mismatch: state={len(current)} action={len(target)}",
            )
            return False

        max_step = float(self._config.get("playback_move_max_step", 0.03))
        min_steps = int(self._config.get("playback_move_min_steps", 10))
        max_steps = int(self._config.get("playback_move_max_steps", 180))
        move_hz = int(self._config.get("playback_move_hz", min(control_hz, 20)))
        max_delta = max(abs(dst - src) for src, dst in zip(current, target))
        steps = max(min_steps, int(max_delta / max(max_step, 1e-6)) + 1)
        steps = min(max(steps, 1), max_steps)
        self._send_status(
            status,
            progress=0.0,
            total=total_frames,
            move_steps=steps,
            max_delta=max_delta,
        )

        chunk = []
        for step in range(1, steps + 1):
            if interruptible and self._stop_event.is_set():
                self._send_status("stopped", progress=0.0, total=total_frames)
                return False
            alpha = step / steps
            chunk.append([src + (dst - src) * alpha for src, dst in zip(current, target)])

        batch_size = max(1, int(self._config.get("playback_rpc_batch_size", 5)))
        for start in range(0, len(chunk), batch_size):
            if interruptible and self._stop_event.is_set():
                self._send_status("stopped", progress=0.0, total=total_frames)
                return False
            if not self._send_action_batch(chunk[start:start + batch_size], move_hz, error_context):
                return False
        if interruptible and self._stop_event.is_set():
            self._send_status("stopped", progress=0.0, total=total_frames)
            return False
        settle_s = float(self._config.get("playback_settle_s", 0.5))
        settle_status = "waiting" if interruptible else status
        self._send_status(settle_status, progress=0.0, total=total_frames, settle_s=settle_s)
        if settle_s > 0 and interruptible and self._stop_event.wait(settle_s):
            self._send_status("stopped", progress=0.0, total=total_frames)
            return False
        if settle_s > 0 and not interruptible:
            threading.Event().wait(settle_s)
        return True

    def _return_after_stop(
        self,
        return_position: List[float],
        control_hz: int,
        total_frames: int,
        *,
        progress: float,
    ) -> None:
        if self._move_to_position(
            return_position,
            control_hz,
            total_frames,
            status="returning",
            error_context="return_after_stop",
            interruptible=False,
        ):
            self.in_progress = False
            self._send_status(
                "stopped",
                progress=1.0,
                total=total_frames,
                message="returned_to_start_position",
            )

    def _read_current_joint_positions(self) -> List[float]:
        client = self._get_protocol_client()
        observation_name = self._get_observation_name()
        if client is None or not observation_name:
            return []
        response = client.get_observations([observation_name])
        if not response.ok or not isinstance(response.payload, dict):
            return []
        observations = response.payload.get("observations")
        if not isinstance(observations, list):
            return []
        for item in observations:
            if isinstance(item, dict) and item.get("name") == observation_name:
                return _joint_positions(item.get("value"))
        return []

    def _send_action_batch(self, actions: List[Any], fps: int, context: str) -> bool:
        client = self._get_protocol_client()
        action_name = self._get_action_name()
        if client is None or not action_name:
            self._send_status("error", message=f"{context}: target protocol client is not bound")
            return False
        try:
            result = client.run_action_chunk(
                action_name,
                [_action_value(item) for item in actions],
                frame_rate=fps,
            )
        except Exception as exc:
            self._logger.warning("playback %s run_action_chunk failed: %s", context, exc)
            self._send_status("error", message=f"{context}: {exc}")
            return False
        if not result.ok:
            self._send_status("error", message=result.message or f"{context} failed")
            return False
        return True

    def _load_samples(self, episode_dir: str) -> List[tuple[float, Any]]:
        action_name = self._get_action_name()
        if not action_name:
            raise RuntimeError("target action name is not selected")
        path = Path(episode_dir) / "streams" / safe_name(action_name) / "samples.msgpack"
        if not path.exists():
            raise FileNotFoundError(f"collection action stream not found: {path}")
        loaded: List[tuple[float, Any]] = []
        with path.open("rb") as fh:
            unpacker = msgpack.Unpacker(fh, raw=False)
            for sample in unpacker:
                if isinstance(sample, dict) and "timestamp" in sample and "value" in sample:
                    loaded.append((float(sample["timestamp"]), sample["value"]))
        return loaded

    def _send_status(self, status: str, **data: Any) -> None:
        payload = {"status": status, **data}
        self.status = payload
        if status in ("completed", "stopped", "error"):
            self.in_progress = False
        self._send_status_payload(payload)


def _joint_positions(value: Any) -> List[float]:
    if isinstance(value, dict):
        return as_float_list(value.get("joint_positions"))
    return as_float_list(value)


def _action_value(value: Any) -> Any:
    if isinstance(value, dict):
        return value
    return {"joint_positions": as_float_list(value)}
