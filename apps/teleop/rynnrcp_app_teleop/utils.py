"""Small shared helpers for the teleop plugin."""

from __future__ import annotations

import base64
import os
from typing import Any, List

_CV2_MODULES: tuple[Any, Any] | None = None


def resize_jpeg_long_edge(data: bytes, max_long_edge: int) -> bytes:
    if max_long_edge <= 0:
        return data
    width, height = _jpeg_size(data)
    if width > 0 and height > 0 and max(width, height) <= max_long_edge:
        return data
    try:
        cv2, np = _cv2_modules()

        arr = np.frombuffer(data, dtype=np.uint8)
        image = cv2.imdecode(arr, _jpeg_decode_flag(cv2, data, max_long_edge))
        if image is None:
            return data
        height, width = image.shape[:2]
        long_edge = max(width, height)
        if long_edge <= max_long_edge:
            return data
        scale = float(max_long_edge) / float(long_edge)
        resized = cv2.resize(
            image,
            (max(1, int(width * scale)), max(1, int(height * scale))),
            interpolation=cv2.INTER_AREA,
        )
        ok, encoded = cv2.imencode(".jpg", resized, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if not ok:
            return data
        return encoded.tobytes()
    except Exception:
        return data


def _cv2_modules() -> tuple[Any, Any]:
    global _CV2_MODULES
    if _CV2_MODULES is None:
        import cv2
        import numpy as np

        try:
            cv2.setNumThreads(1)
        except Exception:
            pass
        _CV2_MODULES = (cv2, np)
    return _CV2_MODULES


def _jpeg_decode_flag(cv2_module: Any, data: bytes, max_long_edge: int) -> int:
    width, height = _jpeg_size(data)
    if width <= 0 or height <= 0:
        return cv2_module.IMREAD_COLOR
    long_edge = max(width, height)
    if long_edge <= max_long_edge:
        return cv2_module.IMREAD_COLOR
    if long_edge / 8.0 >= max_long_edge:
        return cv2_module.IMREAD_REDUCED_COLOR_8
    if long_edge / 4.0 >= max_long_edge:
        return cv2_module.IMREAD_REDUCED_COLOR_4
    if long_edge / 2.0 >= max_long_edge:
        return cv2_module.IMREAD_REDUCED_COLOR_2
    return cv2_module.IMREAD_COLOR


def _jpeg_size(data: bytes) -> tuple[int, int]:
    if len(data) < 4 or data[:2] != b"\xff\xd8":
        return (0, 0)
    i = 2
    size = len(data)
    while i + 9 < size:
        while i < size and data[i] == 0xFF:
            i += 1
        if i >= size:
            break
        marker = data[i]
        i += 1
        if marker in (0xD8, 0xD9) or 0xD0 <= marker <= 0xD7:
            continue
        if i + 2 > size:
            break
        segment_length = int.from_bytes(data[i:i + 2], "big")
        if segment_length < 2 or i + segment_length > size:
            break
        if marker in {
            0xC0,
            0xC1,
            0xC2,
            0xC3,
            0xC5,
            0xC6,
            0xC7,
            0xC9,
            0xCA,
            0xCB,
            0xCD,
            0xCE,
            0xCF,
        }:
            height = int.from_bytes(data[i + 3:i + 5], "big")
            width = int.from_bytes(data[i + 5:i + 7], "big")
            return (width, height)
        i += segment_length
    return (0, 0)


def payload_size(data: Any) -> int:
    if isinstance(data, (bytes, bytearray, memoryview)):
        return len(data)
    if isinstance(data, dict):
        return sum(payload_size(value) for value in data.values())
    if isinstance(data, (list, tuple)):
        return sum(payload_size(value) for value in data)
    if isinstance(data, (int, float)):
        return 8
    if isinstance(data, str):
        return len(data.encode("utf-8"))
    return 0


def directory_size(path: str) -> int:
    total = 0
    for dirpath, _, filenames in os.walk(path):
        for name in filenames:
            try:
                total += os.path.getsize(os.path.join(dirpath, name))
            except OSError:
                pass
    return total


def format_bytes(size: int) -> str:
    value = float(size)
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if value < 1024.0 or unit == "TB":
            return f"{value:.1f} {unit}" if unit != "B" else f"{int(value)} B"
        value /= 1024.0
    return f"{int(size)} B"


def as_float_list(value: Any) -> List[float]:
    if value is None:
        return []
    if hasattr(value, "tolist"):
        value = value.tolist()
    if not isinstance(value, (list, tuple)):
        return []
    try:
        return [float(item) for item in value]
    except (TypeError, ValueError):
        return []


def json_safe(value: Any) -> Any:
    if isinstance(value, bytes):
        return {"encoding": "base64", "data": base64.b64encode(value).decode("ascii")}
    if isinstance(value, bytearray):
        return json_safe(bytes(value))
    if isinstance(value, memoryview):
        return json_safe(value.tobytes())
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value
