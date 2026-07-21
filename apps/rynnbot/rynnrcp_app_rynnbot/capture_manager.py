"""RynnBot capture lifecycle manager."""

from __future__ import annotations

import json
import logging
import os
import shutil
import time
import threading
from typing import Any, Callable, Dict, List, Optional

from rynnrcp.utils import safe_name
from rynnrcp_app_common.resource_transfer import download_collection_resource

logger = logging.getLogger(__name__)

UPLOAD_FAILED = 4021


class RynnBotCaptureManager:
    def __init__(
        self,
        *,
        config: Dict[str, Any],
        raw_capture_root: Any,
        server_client_provider: Callable[[], Any],
        mqtt_send: Callable[[str, str, int], None],
        parse_json_or_reply_error: Callable[[str, str, str], Optional[dict]],
        make_base_response: Callable[[dict], dict],
        start_ws_state_stream: Callable[[], None],
        stop_ws_state_stream: Callable[[], None],
        post_upload_event: Callable[[str, Dict[str, Any], int], None],
        post_occupancy_error: Callable[..., None],
        occupied_id_provider: Callable[[], Any],
        clear_occupancy: Callable[[], None],
        executor: Any,
        cloud_to_protocol_key: Callable[[str], str] | None = None,
        protocol_to_cloud_mapping: Callable[[List[str]], Dict[str, str]] | None = None,
    ) -> None:
        self._config = config
        self._raw_capture_root = raw_capture_root
        self._server_client_provider = server_client_provider
        self._mqtt_send = mqtt_send
        self._parse_json_or_reply_error = parse_json_or_reply_error
        self._make_base_response = make_base_response
        self._start_ws_state_stream = start_ws_state_stream
        self._stop_ws_state_stream = stop_ws_state_stream
        self._post_upload_event = post_upload_event
        self._post_occupancy_error = post_occupancy_error
        self._occupied_id_provider = occupied_id_provider
        self._clear_occupancy = clear_occupancy
        self._tele_exec = executor
        self._cloud_to_protocol_key = cloud_to_protocol_key or (lambda key: key)
        self._protocol_to_cloud_mapping = protocol_to_cloud_mapping or (lambda names: {name: name for name in names})
        self._data_coll_lock = threading.Lock()
        self._data_coll = {
            "task_id": None,
            "sub_task_id": None,
            "round_number": None,
            "episode_number": None,
            "data_coll_id": None,
            "recording": False,
            "last_episode_dir": None,
            "fps": None,
        }
        self._skill_record_lock = threading.Lock()
        self._skill_record = {
            "record_id": None,
            "recording": False,
            "start_time": None,
            "max_duration_s": float(config["max_capture_duration_s"]),
        }
        self._server_resource_by_capture_dir: Dict[str, str] = {}
        self._stopped_collection_lock = threading.Lock()
        self._stopped_collections: Dict[str, Dict[str, Dict[str, Any]]] = {}

    @property
    def _server_client(self) -> Any:
        return self._server_client_provider()

    @staticmethod
    def payload_upload_type(payload: str) -> Optional[str]:
        try:
            request_json = json.loads(payload)
        except Exception:
            return None
        params = request_json.get("params") if isinstance(request_json, dict) else None
        if not isinstance(params, dict):
            return None
        upload_type = params.get("upload_type")
        return str(upload_type) if upload_type is not None else None

    def start_collection_from_mqtt(self, topic: str, payload: str, kind: str) -> None:
        send_topic = topic.replace("/request/", "/response/")
        request_json = self._parse_json_or_reply_error(payload, send_topic, kind)
        if request_json is None:
            return
        resp = self._make_base_response(request_json)
        try:
            if self._server_client is None:
                raise RuntimeError("RCP protocol client is not available")
            params = request_json.get("params") or {}
            capture_request = self._capture_request_from_params(kind, params)
            capture_id = capture_request["capture_id"]
            names = self._protocol_names_from_cloud(
                list(params.get("names") or params.get("keys") or params.get("task_keys") or self.default_capture_names())
            )
            fps = float(params.get("fps") or params.get("rate") or 30.0)
            max_duration_s = self._max_capture_duration_from_params(params)
            logger.info(
                "[RynnBot][%s:start] capture_id=%s collection_id=%s episode_id=%s names=%s fps=%s max_duration_s=%s",
                kind,
                capture_id,
                capture_request["collection_id"],
                capture_request["episode_id"],
                names,
                fps,
                max_duration_s,
            )
            result = self._server_client.start_collection(
                names,
                collection_id=str(capture_request["collection_id"]),
                episode_id=str(capture_request["episode_id"]),
                task_prompt=str(capture_request["metadata"].get("task_prompt") or capture_id),
                task_description=str(capture_request["metadata"].get("task_description") or kind),
                frame_rate=fps,
                max_duration=max_duration_s,
                metadata=capture_request["metadata"],
            )
            if not result.ok:
                raise RuntimeError(result.message or "start_collection failed")
            capture = result.payload if isinstance(result.payload, dict) else {}
            resource = capture.get("collection_resource") if isinstance(capture.get("collection_resource"), dict) else {}
            logger.info(
                "[RynnBot][%s:start] started capture_id=%s resource_id=%s",
                kind,
                capture_id,
                resource.get("resource_id"),
            )
            state_fps = self._state_stream_fps_from_params(params) if kind == "tele_data_coll" else fps
            self._set_capture_state(kind, capture_id, True, None, state_fps, capture_request)
            self._forget_stopped_collection(
                kind,
                str(capture_request["collection_id"]),
                str(capture_request["episode_id"]),
            )
            if kind == "tele_data_coll":
                self._start_ws_state_stream()
            if kind == "skill_record":
                resp["result"].update(
                    {
                        "capture_id": capture_id,
                        "collection_resource": capture.get("collection_resource") or {},
                        "data_coll_id": capture_request.get("data_coll_id"),
                        "round_number": capture_request.get("cloud_round_number"),
                        "max_duration_s": max_duration_s,
                    }
                )
        except Exception as exc:
            resp["error"] = {"code": -1, "message": str(exc)}
        self._mqtt_send(send_topic, json.dumps(resp, ensure_ascii=False))

    def _capture_request_from_params(self, kind: str, params: Dict[str, Any]) -> Dict[str, Any]:
        data_coll_id = self._data_collection_id_from_params(params)
        record_id = self._first_param(params, "record_id", "capture_id", "data_coll_id", "id")
        cloud_round_number = self._first_param(params, "round_number", "round", "round_id", "sub_task_id", "episode_id")
        episode_number = self._first_param(params, "episode_number", "episode_id") or cloud_round_number
        explicit_capture_id = self._first_param(params, "capture_id")

        if kind == "skill_record":
            capture_id = explicit_capture_id or record_id or f"{kind}_{int(time.time())}"
        else:
            data_coll_id = data_coll_id or explicit_capture_id or f"{kind}_{int(time.time())}"
            if explicit_capture_id:
                capture_id = explicit_capture_id
            elif episode_number is not None:
                capture_id = self._episode_capture_id(episode_number)
            else:
                capture_id = data_coll_id

        task_prompt = self._first_param(params, "task_prompt", "prompt", "task_name", "task", "instruction")
        task_description = self._first_param(params, "task_description", "task_desc", "description")
        collection_id = str(data_coll_id or record_id or kind)
        episode_id = str(capture_id)
        metadata = {
            "source": "rynnbot",
            "kind": kind,
            "data_coll_id": data_coll_id,
            "record_id": record_id,
            "episode_number": self._coerce_int_or_original(episode_number),
            "cloud_round_number": self._coerce_int_or_original(cloud_round_number),
            "task_prompt": task_prompt,
            "task_description": task_description,
            "request_params": params,
        }
        return {
            "capture_id": str(capture_id),
            "collection_id": collection_id,
            "episode_id": episode_id,
            "data_coll_id": data_coll_id,
            "record_id": record_id,
            "episode_number": metadata["episode_number"],
            "cloud_round_number": metadata["cloud_round_number"],
            "metadata": metadata,
        }

    @staticmethod
    def _first_param(params: Dict[str, Any], *keys: str) -> Any:
        for key in keys:
            value = params.get(key)
            if value is not None and value != "":
                return value
        return None

    def _data_collection_id_from_params(self, params: Dict[str, Any]) -> Any:
        data_coll_id = self._first_param(params, "data_coll_id")
        if data_coll_id is not None:
            return data_coll_id
        task_id = self._first_param(params, "task_id")
        sub_task_id = self._first_param(params, "sub_task_id")
        if task_id is not None and sub_task_id is not None:
            return f"{task_id}/{sub_task_id}"
        return self._first_param(params, "task_id", "record_id", "id")

    def _record_ids_from_params(self, params: Dict[str, Any]) -> List[str]:
        value = self._first_param(params, "record_ids", "recordIdList", "record_id", "recordId")
        if value is None:
            return []
        if isinstance(value, str):
            try:
                parsed = json.loads(value)
            except Exception:
                parsed = value
            value = parsed
        if isinstance(value, (list, tuple, set)):
            return [str(item) for item in value if item is not None and item != ""]
        return [str(value)]

    @staticmethod
    def _episode_capture_id(episode_number: Any) -> str:
        try:
            return f"episode_{int(episode_number):06d}"
        except (TypeError, ValueError):
            return f"episode_{safe_name(str(episode_number))}"

    @staticmethod
    def _coerce_int_or_original(value: Any) -> Any:
        if value is None or value == "":
            return None
        try:
            return int(value)
        except (TypeError, ValueError):
            return value

    def _max_capture_duration_from_params(self, params: Dict[str, Any]) -> float:
        for key in ("max_duration_s", "max_duration_sec", "max_duration", "timeout_s"):
            value = params.get(key)
            if value is not None and value != "":
                duration = float(value)
                if duration <= 0:
                    raise ValueError(f"{key} must be greater than 0")
                return duration
        expected_steps = params.get("expected_steps")
        fps = params.get("fps") or params.get("rate")
        if expected_steps is not None and fps:
            duration = float(expected_steps) / max(float(fps), 1.0)
            return max(1.0, duration)
        return float(self._config["max_capture_duration_s"])

    @staticmethod
    def _state_stream_fps_from_params(params: Dict[str, Any]) -> float:
        value = params.get("state_stream_fps") or params.get("state_fps") or 5.0
        return max(0.1, float(value))

    def _protocol_names_from_cloud(self, names: List[Any]) -> List[str]:
        result: List[str] = []
        seen: set[str] = set()
        for name in names:
            mapped = str(self._cloud_to_protocol_key(str(name)))
            if mapped and mapped not in seen:
                result.append(mapped)
                seen.add(mapped)
        return result

    def _encode_keys_from_params(self, params: Dict[str, Any]) -> Optional[List[str]]:
        keys = params.get("keys")
        if keys is None:
            return None
        if isinstance(keys, str):
            try:
                parsed = json.loads(keys)
            except Exception:
                parsed = keys
            keys = parsed
        if isinstance(keys, (list, tuple, set)):
            return self._protocol_names_from_cloud(list(keys))
        return self._protocol_names_from_cloud([keys])

    def stop_collection_from_mqtt(self, topic: str, payload: str, kind: str) -> None:
        send_topic = topic.replace("/request/", "/response/")
        request_json = self._parse_json_or_reply_error(payload, send_topic, kind)
        if request_json is None:
            return
        resp = self._make_base_response(request_json)
        try:
            if self._server_client is None:
                raise RuntimeError("RCP protocol client is not available")
            stopped = self._latest_stopped_collection(kind)
            if not self.capture_state(kind).get("recording") and stopped is not None:
                status = dict(stopped["status"])
                logger.info(
                    "[RynnBot][%s:stop] duplicate stop_round acknowledged collection_id=%s episode_id=%s",
                    kind,
                    status.get("collection_id"),
                    status.get("episode_id"),
                )
            else:
                result = self._server_client.stop_collection()
                if not result.ok:
                    raise RuntimeError(result.message or "stop_collection failed")
                status = result.payload if isinstance(result.payload, dict) else {}
                self._finalize_stopped_collection(
                    kind,
                    status,
                    trigger="stop_round",
                    defer_download=True,
                )
            if kind == "skill_record":
                resp["result"].update(status)
        except Exception as exc:
            resp["error"] = {"code": -1, "message": str(exc)}
        self._mqtt_send(send_topic, json.dumps(resp, ensure_ascii=False))

    def encode_capture_from_mqtt(self, topic: str, payload: str, kind: str) -> None:
        send_topic = topic.replace("/request/", "/response/")
        request_json = self._parse_json_or_reply_error(payload, send_topic, kind)
        if request_json is None:
            return
        resp = self._make_base_response(request_json)
        params = dict(request_json.get("params") or {})
        validation_error = self._upload_validation_error(kind, params)
        if validation_error:
            resp["error"] = {"code": -1, "message": validation_error}
            self._mqtt_send(send_topic, json.dumps(resp, ensure_ascii=False))
            return
        self._mqtt_send(send_topic, json.dumps(resp, ensure_ascii=False))

        future = self._tele_exec.submit(self._encode_capture_upload_job, kind, params)

        def _done(done_future) -> None:
            try:
                done_future.result()
            except Exception as exc:
                logger.error("[RynnBot][%s:upload] background job failed: %s", kind, exc, exc_info=True)
                self._handle_upload_failure(exc)

        future.add_done_callback(_done)

    def _encode_capture_upload_job(self, kind: str, params: Dict[str, Any]) -> Dict[str, Any]:
        from rynnrcp_app_common.recording import encode_raw_capture_subprocess
        from rynnrcp_app_common.recording_package import package_encoded_captures

        return self._encode_and_upload_capture(
            kind,
            params,
            encode_raw_capture_subprocess,
            package_encoded_captures,
        )

    def _upload_validation_error(self, kind: str, params: Dict[str, Any]) -> str:
        explicit_dirs = params.get("capture_dirs") or params.get("episode_dirs")
        if explicit_dirs or params.get("capture_dir"):
            return ""
        if kind == "skill_record":
            if not self._record_ids_from_params(params):
                return "record_ids required"
            return ""
        if self._first_param(params, "data_coll_id"):
            return ""
        if self._first_param(params, "task_id") and self._first_param(params, "sub_task_id"):
            return ""
        return "missing task_id/sub_task_id"

    def _encode_and_upload_capture(
        self,
        kind: str,
        params: Dict[str, Any],
        encode_raw_capture_subprocess,
        package_encoded_captures,
    ) -> Dict[str, Any]:
        self._materialize_stopped_collections(kind, params)
        capture_dirs = self._capture_dirs_for_upload(kind, params)
        if not capture_dirs:
            raise RuntimeError("capture_dir or data_coll_id is required")
        include_original_videos = kind == "skill_record"
        encoded_captures = []
        for capture_dir in capture_dirs:
            logger.info("[RynnBot][ENCODE] start kind=%s capture_dir=%s", kind, capture_dir)
            output_dir = params.get("output_dir")
            if output_dir and len(capture_dirs) > 1:
                output_dir = os.path.join(
                    os.path.abspath(os.path.expanduser(str(output_dir))),
                    safe_name(os.path.basename(capture_dir)),
                )
            encoded = encode_raw_capture_subprocess(
                capture_dir,
                output_dir=output_dir,
                keys=self._encode_keys_from_params(params),
                fps=params.get("fps"),
                include_original_videos=include_original_videos,
                key_mapping=self._protocol_to_cloud_mapping(self.default_capture_names()),
                video_backend=params.get("video_backend") or self._config.get("video_backend", "auto"),
                video_encoder=params.get("video_encoder") or self._config.get("video_encoder"),
                timeout_s=params.get("timeout_s"),
            )
            diagnostics = _encoded_capture_diagnostics(encoded)
            logger.info(
                "[RynnBot][ENCODE] finished kind=%s frames=%s output_dir=%s diagnostics=%s",
                kind,
                encoded.get("total_frames"),
                encoded.get("output_dir"),
                diagnostics,
            )
            encoded_captures.append(encoded)
        package_base_dir = self._package_base_dir_for_upload(capture_dirs)
        package_dir = params.get("package_dir") or os.path.join(
            package_base_dir,
            "packages",
        )
        package = package_encoded_captures(
            encoded_captures,
            package_dir=package_dir,
            zip_name=params.get("zip_name") or self._default_package_zip_name(kind, capture_dirs, params),
            include_original_videos=include_original_videos,
        )
        logger.info(
            "[RynnBot][PACKAGE] finished kind=%s zip=%s bytes=%s",
            kind,
            package.get("zip_path"),
            package.get("size_bytes"),
        )
        from .oss_manager import upload_package_if_requested

        upload = upload_package_if_requested(
            package["zip_path"],
            params=params,
            kind=kind,
        )
        if upload is not None:
            if _coerce_bool(params.get("delete_uploaded_package", self._config.get("delete_uploaded_package", True))):
                try:
                    os.remove(package["zip_path"])
                    logger.info("[RynnBot][PACKAGE] deleted uploaded package: %s", package["zip_path"])
                except Exception as exc:
                    logger.warning("[RynnBot][PACKAGE] failed to delete uploaded package: %s", exc, exc_info=True)
            else:
                logger.info("[RynnBot][PACKAGE] kept uploaded package for inspection: %s", package["zip_path"])
            if _coerce_bool(
                params.get("delete_uploaded_raw_captures", self._config.get("delete_uploaded_raw_captures", True))
            ):
                self._delete_uploaded_raw_captures(capture_dirs)
            else:
                logger.info("[RynnBot][CLEANUP] kept local raw captures for inspection: %s", capture_dirs)
            self._forget_uploaded_stopped_collections(kind, capture_dirs)
        result = {
            "capture_dir": capture_dirs[0] if len(capture_dirs) == 1 else package_base_dir,
            "capture_dirs": capture_dirs,
            "encoded": encoded_captures[0] if len(encoded_captures) == 1 else encoded_captures,
            "encoded_captures": encoded_captures,
            "package": package,
            "zip_path": package["zip_path"],
            "upload": upload,
        }
        if _coerce_bool(params.get("post_upload_event", True)):
            self._post_upload_event(
                "occupancy_complete",
                {"occupancy_id": str(self._occupied_id_provider() or "")},
                qos=1,
            )
        return result

    def _capture_dirs_for_upload(self, kind: str, params: Dict[str, Any]) -> List[str]:
        explicit_dirs = params.get("capture_dirs") or params.get("episode_dirs")
        if isinstance(explicit_dirs, (list, tuple)):
            return self._normalize_capture_dirs(explicit_dirs)

        capture_dir = params.get("capture_dir")
        if capture_dir:
            return self._expand_capture_dir(str(capture_dir))

        if kind == "skill_record":
            record_ids = self._record_ids_from_params(params)
            if record_ids:
                root = self._capture_root(kind)
                return self._normalize_capture_dirs(
                    [os.path.join(root, safe_name(record_id)) for record_id in record_ids]
                )

        data_coll_id = self._data_collection_id_from_params(params)
        if kind != "skill_record" and data_coll_id:
            collection_dir = os.path.join(self._capture_root(kind), safe_name(str(data_coll_id)))
            dirs = self._expand_capture_dir(collection_dir)
            if dirs:
                return dirs

        last_episode_dir = self.capture_state(kind).get("last_episode_dir")
        if last_episode_dir:
            return self._expand_capture_dir(str(last_episode_dir))
        return []

    def _normalize_capture_dirs(self, capture_dirs: List[Any]) -> List[str]:
        result: List[str] = []
        seen: set[str] = set()
        for item in capture_dirs:
            if item is None or item == "":
                continue
            for capture_dir in self._expand_capture_dir(str(item)):
                if capture_dir not in seen:
                    result.append(capture_dir)
                    seen.add(capture_dir)
        return result

    def _expand_capture_dir(self, path: str) -> List[str]:
        root = os.path.abspath(os.path.expanduser(path))
        if _is_capture_dir(root):
            return [root]
        capture_dirs: List[str] = []
        if not os.path.isdir(root):
            return capture_dirs
        for dirpath, dirnames, filenames in os.walk(root):
            if "capture_meta.json" in filenames or "collection_meta.json" in filenames:
                capture_dirs.append(os.path.abspath(dirpath))
                dirnames[:] = []
        capture_dirs.sort(key=self._capture_sort_key)
        return capture_dirs

    @staticmethod
    def _capture_sort_key(capture_dir: str) -> tuple[int, str]:
        name = os.path.basename(os.path.abspath(capture_dir))
        if name.startswith("episode_"):
            try:
                return (int(name.split("_", 1)[1]), name)
            except (IndexError, ValueError):
                return (0, name)
        return (0, name)

    @staticmethod
    def _package_base_dir_for_upload(capture_dirs: List[str]) -> str:
        if len(capture_dirs) > 1:
            return os.path.abspath(os.path.commonpath(capture_dirs))
        return os.path.abspath(os.path.expanduser(capture_dirs[0]))

    def _default_package_zip_name(self, kind: str, capture_dirs: List[str], params: Dict[str, Any]) -> str:
        data_coll_id = self._data_collection_id_from_params(params)
        if kind != "skill_record" and data_coll_id:
            return f"{safe_name(str(data_coll_id))}.zip"
        if len(capture_dirs) > 1:
            capture_id = os.path.basename(self._package_base_dir_for_upload(capture_dirs))
            return f"{kind}_{capture_id}.zip"
        capture_id = os.path.basename(os.path.abspath(os.path.expanduser(capture_dirs[0])))
        return f"{kind}_{capture_id}.zip"

    def _capture_root(self, kind: str) -> str:
        return os.path.abspath(os.path.expanduser(str(self._raw_capture_root / safe_name(kind))))

    @staticmethod
    def _capture_dir_summary(capture_dir: str) -> Dict[str, Any]:
        total_bytes = 0
        file_count = 0
        for dirpath, _dirnames, filenames in os.walk(capture_dir):
            for filename in filenames:
                file_count += 1
                path = os.path.join(dirpath, filename)
                try:
                    total_bytes += os.path.getsize(path)
                except OSError:
                    pass

        streams_dir = os.path.join(capture_dir, "streams")
        streams = []
        if os.path.isdir(streams_dir):
            streams = sorted(
                name
                for name in os.listdir(streams_dir)
                if os.path.isdir(os.path.join(streams_dir, name))
            )
        return {"bytes": total_bytes, "files": file_count, "streams": streams}

    def _remember_server_collection_resource(self, capture_dir: str, status: Dict[str, Any]) -> None:
        resource = status.get("collection_resource")
        if not isinstance(resource, dict):
            return
        resource_id = str(resource.get("resource_id") or "")
        if not resource_id:
            return
        self._server_resource_by_capture_dir[os.path.abspath(capture_dir)] = resource_id

    @staticmethod
    def _stopped_collection_key(collection_id: Any, episode_id: Any) -> str:
        return f"{collection_id or ''}\0{episode_id or ''}"

    def _remember_stopped_collection(
        self,
        kind: str,
        status: Dict[str, Any],
        capture_dir: Optional[str] = None,
    ) -> None:
        key = self._stopped_collection_key(status.get("collection_id"), status.get("episode_id"))
        with self._stopped_collection_lock:
            records = self._stopped_collections.setdefault(kind, {})
            records[key] = {"status": dict(status), "capture_dir": capture_dir}

    def _forget_stopped_collection(self, kind: str, collection_id: str, episode_id: str) -> None:
        key = self._stopped_collection_key(collection_id, episode_id)
        with self._stopped_collection_lock:
            records = self._stopped_collections.get(kind)
            if records is not None:
                records.pop(key, None)

    def _latest_stopped_collection(self, kind: str) -> Optional[Dict[str, Any]]:
        with self._stopped_collection_lock:
            records = self._stopped_collections.get(kind) or {}
            if not records:
                return None
            latest_key = next(reversed(records))
            return dict(records[latest_key])

    def _stopped_collections_for_upload(self, kind: str, params: Dict[str, Any]) -> List[Dict[str, Any]]:
        if params.get("capture_dir") or params.get("capture_dirs") or params.get("episode_dirs"):
            return []
        data_coll_id = self._data_collection_id_from_params(params)
        record_ids = set(self._record_ids_from_params(params)) if kind == "skill_record" else set()
        with self._stopped_collection_lock:
            records = [dict(record) for record in (self._stopped_collections.get(kind) or {}).values()]
        if data_coll_id is not None:
            records = [
                record
                for record in records
                if str(record["status"].get("collection_id") or "") == str(data_coll_id)
            ]
        elif record_ids:
            records = [
                record
                for record in records
                if str(record["status"].get("episode_id") or "") in record_ids
            ]
        return records

    def _forget_uploaded_stopped_collections(self, kind: str, capture_dirs: List[str]) -> None:
        uploaded_dirs = {os.path.abspath(path) for path in capture_dirs}
        with self._stopped_collection_lock:
            records = self._stopped_collections.get(kind)
            if not records:
                return
            for key, record in list(records.items()):
                capture_dir = record.get("capture_dir")
                if capture_dir and os.path.abspath(str(capture_dir)) in uploaded_dirs:
                    records.pop(key, None)

    def _materialize_stopped_collections(self, kind: str, params: Dict[str, Any]) -> List[str]:
        capture_dirs: List[str] = []
        for record in self._stopped_collections_for_upload(kind, params):
            status = record["status"]
            capture_dir = record.get("capture_dir")
            if not capture_dir or not os.path.isdir(str(capture_dir)):
                capture_dir = self._download_collection_resource(kind, status)
                self._remember_stopped_collection(kind, status, capture_dir)
                self._remember_server_collection_resource(capture_dir, status)
                self._set_capture_state(
                    kind,
                    status.get("episode_id") or status.get("collection_id"),
                    False,
                    capture_dir,
                    None,
                )
                summary = self._capture_dir_summary(capture_dir)
                logger.info(
                    "[RynnBot][%s:upload] downloaded raw capture capture_dir=%s bytes=%s files=%s streams=%s",
                    kind,
                    capture_dir,
                    summary["bytes"],
                    summary["files"],
                    summary["streams"],
                )
            capture_dirs.append(str(capture_dir))
        return capture_dirs

    def _delete_uploaded_raw_captures(self, capture_dirs: List[str]) -> None:
        self._delete_server_collections(capture_dirs)
        self._delete_local_capture_dirs(capture_dirs)

    def _delete_server_collections(self, capture_dirs: List[str]) -> None:
        client = self._server_client
        if client is None:
            return
        resource_ids: List[str] = []
        for capture_dir in capture_dirs:
            resource_id = self._server_resource_by_capture_dir.pop(os.path.abspath(capture_dir), "")
            if resource_id and resource_id not in resource_ids:
                resource_ids.append(resource_id)
        for resource_id in resource_ids:
            try:
                result = client.delete_collection(resource_id, timeout_ms=30000)
                if not result.ok:
                    logger.warning("[RynnBot][CLEANUP] server collection delete failed: %s", result.message)
                else:
                    logger.info("[RynnBot][CLEANUP] deleted server collection resource: %s", resource_id)
            except Exception as exc:
                logger.warning("[RynnBot][CLEANUP] server collection delete error: %s", exc, exc_info=True)

    def _delete_local_capture_dirs(self, capture_dirs: List[str]) -> None:
        raw_root = os.path.abspath(os.path.expanduser(str(self._raw_capture_root)))
        for capture_dir in capture_dirs:
            path = os.path.abspath(os.path.expanduser(str(capture_dir)))
            if not os.path.isdir(path):
                continue
            try:
                if os.path.commonpath([raw_root, path]) != raw_root:
                    logger.warning("[RynnBot][CLEANUP] skip raw capture outside app root: %s", path)
                    continue
            except ValueError:
                logger.warning("[RynnBot][CLEANUP] skip raw capture outside app root: %s", path)
                continue
            shutil.rmtree(path)
            logger.info("[RynnBot][CLEANUP] deleted local raw capture: %s", path)
            self._delete_empty_capture_parents(os.path.dirname(path), raw_root)

    def _delete_empty_capture_parents(self, start_dir: str, raw_root: str) -> None:
        current = os.path.abspath(os.path.expanduser(start_dir))
        raw_root = os.path.abspath(os.path.expanduser(raw_root))
        while current != raw_root:
            try:
                if os.path.commonpath([raw_root, current]) != raw_root:
                    return
                os.rmdir(current)
                logger.info("[RynnBot][CLEANUP] deleted empty capture directory: %s", current)
            except OSError:
                return
            current = os.path.dirname(current)

    def _download_collection_resource(self, kind: str, status: Dict[str, Any]) -> str:
        if self._server_client is None:
            raise RuntimeError("RCP protocol client is not available")
        return download_collection_resource(
            self._server_client,
            status,
            self._capture_root(kind),
            timeout_ms=30000,
        )

    def _handle_upload_failure(self, exc: Exception) -> None:
        try:
            self._post_occupancy_error(
                error_code=UPLOAD_FAILED,
                error_msg=str(exc),
                qos=1,
            )
        except Exception as post_exc:
            logger.warning("[RynnBot] occupancy_error post failed: %s", post_exc, exc_info=True)
        self._clear_capture_state_after_upload_failure()
        self._clear_occupancy()

    def _clear_capture_state_after_upload_failure(self) -> None:
        with self._data_coll_lock:
            self._data_coll.update(
                {
                    "task_id": None,
                    "sub_task_id": None,
                    "round_number": None,
                    "episode_number": None,
                    "data_coll_id": None,
                    "recording": False,
                }
            )
        with self._skill_record_lock:
            self._skill_record.update(
                {
                    "record_id": None,
                    "recording": False,
                    "start_time": None,
                }
            )

    def force_stop_collection_on_release(self) -> List[Dict[str, Any]]:
        active_kinds = self._active_capture_kinds()
        if not active_kinds:
            return []

        cleanup: List[Dict[str, Any]] = []
        logger.warning("[RynnBot] release_device: force stopping active collections: %s", active_kinds)
        active_kind = active_kinds[0]
        try:
            if self._server_client is None:
                raise RuntimeError("RCP protocol client is not available")
            result = self._server_client.stop_collection()
            if not result.ok:
                raise RuntimeError(result.message or "stop_collection failed")
            status = result.payload if isinstance(result.payload, dict) else {}
            capture_dir = self._finalize_stopped_collection(active_kind, status, trigger="release_device")
            cleanup.append(
                {
                    "kind": active_kind,
                    "result": status,
                    "capture_dir": capture_dir,
                }
            )
        except Exception as exc:
            cleanup.append({"kind": active_kind, "error": str(exc)})
            logger.warning("[RynnBot] release_device: force stop collection failed: %s", exc, exc_info=True)

        self._clear_capture_state_on_release()
        self._stop_ws_state_stream()
        return cleanup

    def _finalize_stopped_collection(
        self,
        kind: str,
        status: Dict[str, Any],
        *,
        trigger: str,
        defer_download: bool = False,
    ) -> str:
        stopped_id = status.get("episode_id") or status.get("collection_id")
        resource = status.get("collection_resource") if isinstance(status.get("collection_resource"), dict) else {}
        logger.info(
            "[RynnBot][%s:stop] trigger=%s collection_id=%s episode_id=%s resource_id=%s "
            "duration_s=%.3f counts=%s stream_stats=%s",
            kind,
            trigger,
            status.get("collection_id"),
            status.get("episode_id"),
            resource.get("resource_id"),
            float(status.get("duration") or status.get("duration_s") or 0.0),
            status.get("per_name_counts") or status.get("counts") or {},
            status.get("stream_stats") or {},
        )

        # Mark the stream stopped before resource transfer so a download error
        # cannot leave state streaming alive after the Server already stopped.
        self._set_capture_state(kind, stopped_id, False, None, None)
        if kind == "tele_data_coll":
            self._stop_ws_state_stream()

        if defer_download:
            self._remember_stopped_collection(kind, status)
            logger.info(
                "[RynnBot][%s:stop] raw capture transfer deferred until upload collection_id=%s episode_id=%s",
                kind,
                status.get("collection_id"),
                status.get("episode_id"),
            )
            return ""

        capture_dir = self._download_collection_resource(kind, status)
        self._remember_stopped_collection(kind, status, capture_dir)
        self._set_capture_state(kind, stopped_id, False, capture_dir, None)
        self._remember_server_collection_resource(capture_dir, status)
        summary = self._capture_dir_summary(capture_dir)
        logger.info(
            "[RynnBot][%s:stop] trigger=%s saved raw capture capture_dir=%s bytes=%s files=%s streams=%s",
            kind,
            trigger,
            capture_dir,
            summary["bytes"],
            summary["files"],
            summary["streams"],
        )
        return capture_dir

    def _active_capture_kinds(self) -> List[str]:
        active = []
        with self._data_coll_lock:
            if self._data_coll.get("recording"):
                active.append("tele_data_coll")
        with self._skill_record_lock:
            if self._skill_record.get("recording"):
                active.append("skill_record")
        return active

    def _clear_capture_state_on_release(self) -> None:
        with self._data_coll_lock:
            self._data_coll.update(
                {
                    "task_id": None,
                    "sub_task_id": None,
                    "round_number": None,
                    "episode_number": None,
                    "data_coll_id": None,
                    "recording": False,
                    "fps": None,
                }
            )
        with self._skill_record_lock:
            self._skill_record.update(
                {
                    "record_id": None,
                    "recording": False,
                    "start_time": None,
                }
            )

    def default_capture_names(self) -> List[str]:
        client = self._server_client
        if client is None:
            return []
        names: List[str] = []
        observations = client.list_observations()
        if observations.ok and isinstance(observations.payload, dict):
            names.extend(
                str(item["name"])
                for item in observations.payload.get("observations", [])
                if isinstance(item, dict) and item.get("name")
            )
        actions = client.list_actions()
        if actions.ok and isinstance(actions.payload, dict):
            names.extend(
                str(item["name"])
                for item in actions.payload.get("actions", [])
                if isinstance(item, dict) and item.get("name")
            )
        return names

    def capture_state(self, kind: str) -> Dict[str, Any]:
        lock = self._skill_record_lock if kind == "skill_record" else self._data_coll_lock
        state = self._skill_record if kind == "skill_record" else self._data_coll
        with lock:
            return dict(state)

    def _set_capture_state(
        self,
        kind: str,
        capture_id: Any,
        recording: bool,
        capture_dir: Any,
        fps: Any,
        request: Optional[Dict[str, Any]] = None,
    ) -> None:
        lock = self._skill_record_lock if kind == "skill_record" else self._data_coll_lock
        state = self._skill_record if kind == "skill_record" else self._data_coll
        request = request or {}
        with lock:
            if kind == "skill_record":
                if capture_id is not None:
                    state["record_id"] = capture_id
                state["recording"] = recording
                state["last_episode_dir"] = capture_dir
                if recording:
                    state["start_time"] = time.time()
                    metadata = request.get("metadata") if isinstance(request.get("metadata"), dict) else {}
                    params = metadata.get("request_params") if isinstance(metadata.get("request_params"), dict) else {}
                    try:
                        state["max_duration_s"] = self._max_capture_duration_from_params(params)
                    except Exception:
                        pass
            else:
                if request.get("data_coll_id") is not None:
                    state["data_coll_id"] = request.get("data_coll_id")
                elif capture_id is not None:
                    state["data_coll_id"] = capture_id
                if request.get("episode_number") is not None:
                    state["episode_number"] = request.get("episode_number")
                if request.get("cloud_round_number") is not None:
                    state["round_number"] = request.get("cloud_round_number")
                metadata = request.get("metadata") if isinstance(request.get("metadata"), dict) else {}
                params = metadata.get("request_params") if isinstance(metadata.get("request_params"), dict) else {}
                if params.get("task_id") is not None:
                    state["task_id"] = params.get("task_id")
                if params.get("sub_task_id") is not None:
                    state["sub_task_id"] = params.get("sub_task_id")
                state["recording"] = recording
                state["last_episode_dir"] = capture_dir
                if fps is not None:
                    state["fps"] = fps

    def check_skill_record_timeout(self) -> None:
        with self._skill_record_lock:
            if not self._skill_record.get("recording"):
                return
            start_time = self._skill_record.get("start_time")
            max_duration_s = float(self._skill_record.get("max_duration_s") or self._config["max_capture_duration_s"])
            if not start_time or time.time() - float(start_time) <= max_duration_s:
                return

        logger.warning("[RynnBot][skill_record] recording timeout reached, auto-stopping")
        try:
            if self._server_client is not None:
                self._server_client.stop_collection()
        except Exception as exc:
            logger.warning("[RynnBot][skill_record] auto-stop failed: %s", exc, exc_info=True)

        with self._skill_record_lock:
            self._skill_record.update(
                {
                    "recording": False,
                    "record_id": None,
                    "start_time": None,
                }
            )


def _encoded_capture_diagnostics(encoded: Dict[str, Any]) -> Dict[str, Any]:
    metadata_path = encoded.get("metadata_path")
    if not metadata_path:
        output_dir = encoded.get("output_dir")
        if output_dir:
            metadata_path = os.path.join(str(output_dir), "metadata.json")
    if not metadata_path or not os.path.isfile(str(metadata_path)):
        return {}
    try:
        with open(str(metadata_path), "r", encoding="utf-8") as f:
            metadata = json.load(f)
    except (OSError, TypeError, ValueError):
        return {}
    if not isinstance(metadata, dict):
        return {}
    alignment = metadata.get("alignment") if isinstance(metadata.get("alignment"), dict) else {}
    return {
        "fps": metadata.get("fps"),
        "timestamp_policy": metadata.get("timestamp_policy"),
        "total_frames": metadata.get("total_frames"),
        "alignment_policy": metadata.get("alignment_policy"),
        "recording_duration_s": alignment.get("recording_duration_s"),
        "frames_recorded_by_key": alignment.get("frames_recorded_by_key") or {},
        "recording_duration_by_key": alignment.get("recording_duration_by_key") or {},
        "reused_sample_rate_by_key": alignment.get("reused_sample_rate_by_key") or {},
    }


def _is_capture_dir(path: str) -> bool:
    return os.path.isfile(os.path.join(path, "collection_meta.json")) or os.path.isfile(
        os.path.join(path, "capture_meta.json")
    )


def _coerce_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return False
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"", "0", "false", "no", "off", "none", "null"}:
            return False
        if normalized in {"1", "true", "yes", "on"}:
            return True
    return bool(value)
