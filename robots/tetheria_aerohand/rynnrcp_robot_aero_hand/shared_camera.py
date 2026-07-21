"""Order-independent local camera ownership and frame sharing for Aero Hand."""

from __future__ import annotations

import errno
import threading
import time
from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import urlopen


Frame = tuple[bytes, int, int]


def fetch_shared_frame(url: str, timeout_s: float = 0.2) -> Frame:
    try:
        with urlopen(url, timeout=max(0.05, float(timeout_s))) as response:
            if response.status != 200:
                raise RuntimeError("shared camera has not produced a frame")
            data = response.read()
            width = int(response.headers.get("X-Image-Width") or 0)
            height = int(response.headers.get("X-Image-Height") or 0)
    except (HTTPError, URLError, TimeoutError, OSError, ValueError) as exc:
        raise RuntimeError("shared camera is not available") from exc
    if not data or width <= 0 or height <= 0:
        raise RuntimeError("shared camera returned an invalid frame")
    return data, width, height


def try_start_frame_server(
    port: int,
    get_frame: Callable[[], Frame],
) -> tuple[ThreadingHTTPServer, threading.Thread] | None:
    try:
        server = _FrameServer(("127.0.0.1", int(port)), _FrameHandler, get_frame)
    except OSError as exc:
        if exc.errno not in {errno.EADDRINUSE, 10048}:
            raise
        return None
    thread = threading.Thread(
        target=server.serve_forever,
        name=f"aero-hand-frame-server-{port}",
        daemon=True,
    )
    thread.start()
    return server, thread


def stop_frame_server(
    server: ThreadingHTTPServer | None,
    thread: threading.Thread | None,
) -> None:
    if server is not None:
        server.shutdown()
        server.server_close()
    if thread is not None and thread is not threading.current_thread():
        thread.join(timeout=2.0)


class AeroHandSharedCamera:
    """Own the webcam when first, otherwise consume the current owner's frames."""

    def __init__(
        self,
        name: str,
        device_id: str | int = 0,
        url: str = "http://127.0.0.1:28412/frame",
        frame_server_port: int = 28412,
        width: int = 640,
        height: int = 360,
        encoding: str = "jpg",
        fps: float = 30.0,
    ) -> None:
        self.name = str(name)
        self.device_id = int(device_id)
        self.url = str(url)
        self.frame_server_port = int(frame_server_port)
        self.width = int(width)
        self.height = int(height)
        self.encoding = str(encoding)
        self.fps = float(fps)
        self._running = False
        self._capture: Any | None = None
        self._server: ThreadingHTTPServer | None = None
        self._server_thread: threading.Thread | None = None
        self._latest_frame: Frame | None = None
        self._lock = threading.Lock()
        self._next_owner_attempt_at = 0.0

    def start(self) -> None:
        self._try_become_owner(raise_on_camera_error=True)
        self._running = True

    def stop(self) -> None:
        self._running = False
        if self._capture is not None:
            self._capture.release()
            self._capture = None
        stop_frame_server(self._server, self._server_thread)
        self._server = None
        self._server_thread = None

    def read(self) -> tuple[bool, int, int, str, bytes | None]:
        if not self._running:
            return False, 0, 0, self.encoding, None
        try:
            if self._capture is not None:
                frame = self._read_owned_frame()
            else:
                try:
                    frame = fetch_shared_frame(
                        self.url,
                        timeout_s=max(0.1, 3.0 / self.fps),
                    )
                except RuntimeError:
                    if not self._try_become_owner():
                        return False, 0, 0, self.encoding, None
                    frame = self._read_owned_frame()
        except RuntimeError:
            return False, 0, 0, self.encoding, None
        data, width, height = frame
        return True, width, height, self.encoding, data

    def _read_owned_frame(self) -> Frame:
        import cv2

        ok, image = self._capture.read()
        if not ok or image is None:
            raise RuntimeError("Could not read frame from camera")
        height, width = image.shape[:2]
        encoded, jpeg = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if not encoded:
            raise RuntimeError("Could not encode camera frame")
        frame = (jpeg.tobytes(), int(width), int(height))
        with self._lock:
            self._latest_frame = frame
        return frame

    def _try_become_owner(self, *, raise_on_camera_error: bool = False) -> bool:
        if self._capture is not None:
            return True
        now = time.monotonic()
        if now < self._next_owner_attempt_at:
            return False
        self._next_owner_attempt_at = now + 1.0
        owned = try_start_frame_server(self.frame_server_port, self._get_latest_frame)
        if owned is None:
            return False
        self._server, self._server_thread = owned
        try:
            import cv2

            capture = cv2.VideoCapture(self.device_id)
            if not capture.isOpened():
                capture.release()
                raise RuntimeError(f"Could not open camera index {self.device_id}")
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            capture.set(cv2.CAP_PROP_FPS, self.fps)
            self._capture = capture
            return True
        except Exception:
            stop_frame_server(self._server, self._server_thread)
            self._server = None
            self._server_thread = None
            if raise_on_camera_error:
                raise
            return False

    def _get_latest_frame(self) -> Frame:
        with self._lock:
            if self._latest_frame is None:
                raise RuntimeError("camera has not produced a frame")
            return self._latest_frame


class _FrameServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        address: tuple[str, int],
        handler: type[BaseHTTPRequestHandler],
        get_frame: Callable[[], Frame],
    ) -> None:
        self.get_frame = get_frame
        super().__init__(address, handler)


class _FrameHandler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        if self.path != "/frame":
            self.send_error(404)
            return
        try:
            data, width, height = self.server.get_frame()  # type: ignore[attr-defined]
        except RuntimeError:
            self.send_response(204)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("X-Image-Width", str(width))
        self.send_header("X-Image-Height", str(height))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, format: str, *args: Any) -> None:
        return
