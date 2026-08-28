"""
Tests for USBCamera sensor adapter.
"""

import sys
import types
import unittest
from unittest.mock import MagicMock, PropertyMock, call, patch

# ---------------------------------------------------------------------------
# Ensure rynnrcp package root is importable
# ---------------------------------------------------------------------------
import os

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from rynnkit.cameras.base import BaseCamera, SensorState


# ===========================================================================
# Test helpers — mock cv2 / numpy
# ===========================================================================

def _make_mock_cv2():
    """Return a mock cv2 module with essential constants and classes."""
    mock_cv2 = MagicMock()
    mock_cv2.CAP_V4L2 = 200
    mock_cv2.CAP_GSTREAMER = 1800
    mock_cv2.CAP_AVFOUNDATION = 1200
    mock_cv2.CAP_DSHOW = 700
    mock_cv2.CAP_ANY = 0
    mock_cv2.CAP_PROP_FRAME_WIDTH = 3
    mock_cv2.CAP_PROP_FRAME_HEIGHT = 4
    mock_cv2.CAP_PROP_FPS = 5
    mock_cv2.CAP_PROP_FOURCC = 6
    mock_cv2.CAP_PROP_CONVERT_RGB = 16
    mock_cv2.CAP_PROP_BUFFERSIZE = 38
    mock_cv2.ROTATE_90_CLOCKWISE = 0
    mock_cv2.ROTATE_180 = 1
    mock_cv2.ROTATE_90_COUNTERCLOCKWISE = 2
    mock_cv2.COLOR_BGR2RGB = 4
    mock_cv2.VideoWriter_fourcc = MagicMock(return_value=1196444237)
    return mock_cv2


def _make_mock_numpy():
    """Return a lightweight mock numpy with ndarray-like behaviour."""
    import numpy as np
    return np


# ===========================================================================
# Tests
# ===========================================================================

class TestUSBCameraInit(unittest.TestCase):
    """Constructor and parameter validation."""

    def test_init_stores_params(self):
        from rynnkit.cameras.usb_camera import USBCamera
        cam = USBCamera(
            name="front", device_id=0, width=1280, height=720,
            encoding="rgb8", fps=60.0, brand="Logitech", rotate=90,
            capture_backend="opencv", capture_fourcc="yuyv",
        )
        self.assertEqual(cam.name, "front")
        self.assertEqual(cam.device_id, 0)
        self.assertEqual(cam.width, 1280)
        self.assertEqual(cam.height, 720)
        self.assertEqual(cam.encoding, "rgb8")
        self.assertAlmostEqual(cam.frequency_hz, 60.0)
        self.assertEqual(cam.brand, "Logitech")
        self.assertEqual(cam.rotate, 90)
        self.assertEqual(cam.capture_backend, "opencv")
        self.assertEqual(cam.capture_fourcc, "YUYV")
        self.assertEqual(cam.state, SensorState.IDLE)

    def test_default_params(self):
        from rynnkit.cameras.usb_camera import USBCamera
        cam = USBCamera(name="cam0", device_id=0)
        self.assertEqual(cam.width, 640)
        self.assertEqual(cam.height, 480)
        self.assertEqual(cam.encoding, "bgr8")
        self.assertAlmostEqual(cam.frequency_hz, 30.0)
        self.assertEqual(cam.brand, "Unknown")
        self.assertEqual(cam.rotate, 0)
        self.assertEqual(cam.capture_fourcc, "MJPG")

    def test_capture_fourcc_requires_four_characters(self):
        from rynnkit.cameras.usb_camera import USBCamera

        with self.assertRaisesRegex(ValueError, "four-character code"):
            USBCamera(name="cam0", device_id=0, capture_fourcc="YUY")

    def test_is_base_camera_subclass(self):
        from rynnkit.cameras.usb_camera import USBCamera
        cam = USBCamera(name="cam0", device_id=0)
        self.assertIsInstance(cam, BaseCamera)

    def test_get_info_includes_camera_fields(self):
        from rynnkit.cameras.usb_camera import USBCamera
        cam = USBCamera(name="cam0", device_id="/dev/video0", brand="Realsense")
        info = cam.get_info()
        self.assertEqual(info["name"], "cam0")
        self.assertEqual(info["device_id"], "/dev/video0")
        self.assertEqual(info["brand"], "Realsense")
        self.assertIn("width", info)
        self.assertIn("encoding", info)


class TestUSBCameraNoCV2(unittest.TestCase):
    """Behaviour when cv2 is not installed."""

    def test_start_raises_without_cv2(self):
        from rynnkit.cameras import usb_camera as mod
        original = mod._HAS_CV2
        try:
            mod._HAS_CV2 = False
            cam = mod.USBCamera(name="cam0", device_id=0)
            with self.assertRaises(RuntimeError) as ctx:
                cam.start()
            self.assertIn("cv2", str(ctx.exception))
        finally:
            mod._HAS_CV2 = original


class TestBackendSelection(unittest.TestCase):
    """_pick_cv_backend() platform branches."""

    def _assert_backend(self, sysname: str, attribute: str) -> None:
        from rynnkit.cameras import usb_camera as mod
        mock_cv2 = _make_mock_cv2()
        with patch.object(mod, "cv2", mock_cv2), \
                patch.object(mod, "_stdlib_platform") as mp:
            mp.system.return_value = sysname
            self.assertEqual(mod._pick_cv_backend(), getattr(mock_cv2, attribute))

    def test_linux_backend(self):
        self._assert_backend("Linux", "CAP_V4L2")

    def test_darwin_backend(self):
        self._assert_backend("Darwin", "CAP_AVFOUNDATION")

    def test_windows_backend(self):
        self._assert_backend("Windows", "CAP_DSHOW")

    def test_other_backend(self):
        self._assert_backend("FreeBSD", "CAP_ANY")


class TestMacOSUniqueID(unittest.TestCase):
    """_is_probably_macos_uniqueid() identification."""

    def test_hex_id(self):
        from rynnkit.cameras.usb_camera import _is_probably_macos_uniqueid
        self.assertTrue(_is_probably_macos_uniqueid("0x213000010bb2b08"))

    def test_uuid(self):
        from rynnkit.cameras.usb_camera import _is_probably_macos_uniqueid
        self.assertTrue(_is_probably_macos_uniqueid(
            "1FD4B3A2-236E-492B-8CE5-255DD288CE50"
        ))

    def test_avf_prefix(self):
        from rynnkit.cameras.usb_camera import _is_probably_macos_uniqueid
        self.assertTrue(_is_probably_macos_uniqueid("avf:0x213000010bb2b08"))

    def test_integer_string_is_not_uniqueid(self):
        from rynnkit.cameras.usb_camera import _is_probably_macos_uniqueid
        self.assertFalse(_is_probably_macos_uniqueid("0"))
        self.assertFalse(_is_probably_macos_uniqueid("42"))

    def test_device_path_is_not_uniqueid(self):
        from rynnkit.cameras.usb_camera import _is_probably_macos_uniqueid
        self.assertFalse(_is_probably_macos_uniqueid("/dev/video0"))


class TestUSBCameraWithMockCV2(unittest.TestCase):
    """Full lifecycle with mocked cv2."""

    def setUp(self):
        import numpy as _np
        self.np = _np

        self.mock_cv2 = _make_mock_cv2()

        # Mock VideoCapture
        self.mock_cap = MagicMock()
        self.mock_cap.isOpened.return_value = True
        # Return a fake 480x640 BGR frame
        self.fake_frame = self.np.zeros((480, 640, 3), dtype=self.np.uint8)
        self.fake_frame[0, 0] = [10, 20, 30]
        self.mock_cap.read.return_value = (True, self.fake_frame.copy())
        self.mock_cv2.VideoCapture.return_value = self.mock_cap

        # imencode for jpeg/png
        encoded = self.np.frombuffer(b"\xff\xd8fake_jpeg_data", dtype=self.np.uint8)
        self.mock_cv2.imencode.return_value = (True, encoded)

        # cvtColor for rgb8
        self.mock_cv2.cvtColor.side_effect = lambda frame, code: frame[:, :, ::-1].copy()

        # rotate
        self.mock_cv2.rotate.side_effect = lambda frame, code: self.np.rot90(frame).copy()

    def _import_usb_camera(self):
        """Import usb_camera with mocked cv2."""
        import rynnkit.cameras.usb_camera as mod
        mod.cv2 = self.mock_cv2
        mod.np = self.np
        mod._HAS_CV2 = True
        # Patch _stdlib_platform to return 'Windows' (avoids shadowing issues)
        mock_plat = MagicMock()
        mock_plat.system.return_value = "Windows"
        mod._stdlib_platform = mock_plat
        return mod

    def test_start_stop_lifecycle(self):
        mod = self._import_usb_camera()
        cam = mod.USBCamera(name="cam0", device_id=0)
        cam.start()
        self.assertEqual(cam.state, SensorState.RUNNING)
        self.assertTrue(cam._running)

        cam.stop()
        self.assertEqual(cam.state, SensorState.STOPPED)
        self.assertFalse(cam._running)
        self.mock_cap.release.assert_called_once()

    def test_read_frame_bgr8(self):
        mod = self._import_usb_camera()
        cam = mod.USBCamera(name="cam0", device_id=0, encoding="bgr8")
        cam.start()
        success, w, h, enc, image = cam.read_frame()
        self.assertTrue(success)
        self.assertEqual(w, 640)
        self.assertEqual(h, 480)
        self.assertEqual(enc, "bgr8")
        self.assertEqual(image.shape, (480, 640, 3))
        cam.stop()

    def test_read_frame_rgb8(self):
        mod = self._import_usb_camera()
        cam = mod.USBCamera(name="cam0", device_id=0, encoding="rgb8")
        cam.start()
        success, w, h, enc, image = cam.read_frame()
        self.assertTrue(success)
        self.assertEqual(enc, "rgb8")
        self.mock_cv2.cvtColor.assert_called_once()
        cam.stop()

    def test_read_frame_jpeg(self):
        mod = self._import_usb_camera()
        cam = mod.USBCamera(name="cam0", device_id=0, encoding="jpeg")
        cam.start()
        success, w, h, enc, image = cam.read_frame()
        self.assertTrue(success)
        self.assertEqual(enc, "jpeg")
        self.assertIsInstance(image, bytes)
        cam.stop()

    def test_read_frame_unsupported_encoding(self):
        mod = self._import_usb_camera()
        cam = mod.USBCamera(name="cam0", device_id=0, encoding="yuv422")
        cam.start()
        with self.assertRaises(ValueError):
            cam.read_frame()
        cam.stop()

    def test_read_returns_image_tuple(self):
        """read() should return the RynnRCP image tuple."""
        mod = self._import_usb_camera()
        cam = mod.USBCamera(name="cam0", device_id=0, encoding="bgr8")
        cam.start()
        success, w, h, enc, image = cam.read()
        self.assertTrue(success)
        self.assertEqual(w, 640)
        self.assertEqual(h, 480)
        self.assertEqual(enc, "bgr8")
        self.assertEqual(image.shape[:2], (480, 640))
        cam.stop()

    def test_read_frame_with_rotation(self):
        mod = self._import_usb_camera()
        cam = mod.USBCamera(name="cam0", device_id=0, rotate=180)
        cam.start()
        success, w, h, enc, image = cam.read_frame()
        self.assertTrue(success)
        self.mock_cv2.rotate.assert_called_once()
        cam.stop()

    def test_read_frame_failure(self):
        mod = self._import_usb_camera()
        self.mock_cap.read.return_value = (False, None)
        cam = mod.USBCamera(name="cam0", device_id=0)
        cam.start()
        success, w, h, enc, image = cam.read_frame()
        self.assertFalse(success)
        self.assertIsNone(image)
        cam.stop()

    def test_start_device_open_failure(self):
        mod = self._import_usb_camera()
        self.mock_cap.isOpened.return_value = False
        cam = mod.USBCamera(name="cam0", device_id=99)
        with self.assertRaises(RuntimeError) as ctx:
            cam.start()
        self.assertIn("Unable to open", str(ctx.exception))

    def test_read_before_start_raises(self):
        mod = self._import_usb_camera()
        cam = mod.USBCamera(name="cam0", device_id=0)
        with self.assertRaises(RuntimeError):
            cam.read_frame()

    def test_string_device_id_parsed_as_int(self):
        mod = self._import_usb_camera()
        cam = mod.USBCamera(name="cam0", device_id="2")
        cam.start()
        # VideoCapture should have been called with int 2
        call_args = self.mock_cv2.VideoCapture.call_args
        self.assertEqual(call_args[0][0], 2)
        cam.stop()

    def test_string_device_path_kept_as_string(self):
        mod = self._import_usb_camera()
        cam = mod.USBCamera(name="cam0", device_id="/dev/video0")
        cam.start()
        call_args = self.mock_cv2.VideoCapture.call_args
        self.assertEqual(call_args[0][0], "/dev/video0")
        cam.stop()

    def test_linux_native_compressed_jpeg_passthrough(self):
        mod = self._import_usb_camera()
        mock_plat = MagicMock()
        mock_plat.system.return_value = "Linux"
        mod._stdlib_platform = mock_plat

        native = self.np.frombuffer(b"\xff\xd8native_jpeg_data\xff\xd9", dtype=self.np.uint8).reshape(1, -1)
        self.mock_cap.read.return_value = (True, native)
        cam = mod.USBCamera(
            name="cam0",
            device_id=0,
            width=640,
            height=360,
            encoding="jpg",
            native_compressed=True,
            capture_backend="opencv",
        )
        cam.start()
        success, w, h, enc, image = cam.read_frame()
        self.assertTrue(success)
        self.assertEqual((w, h, enc), (640, 360, "jpg"))
        self.assertEqual(image, b"\xff\xd8native_jpeg_data\xff\xd9")
        self.mock_cv2.imencode.assert_not_called()
        self.mock_cap.set.assert_any_call(self.mock_cv2.CAP_PROP_CONVERT_RGB, 0)
        cam.stop()

    def test_linux_opencv_uses_configured_capture_fourcc(self):
        mod = self._import_usb_camera()
        mock_plat = MagicMock()
        mock_plat.system.return_value = "Linux"
        mod._stdlib_platform = mock_plat

        cam = mod.USBCamera(
            name="cam0",
            device_id=0,
            capture_backend="opencv",
            capture_fourcc="YUYV",
        )
        cam.start()

        self.mock_cv2.VideoWriter_fourcc.assert_called_with(*"YUYV")
        self.mock_cap.set.assert_any_call(self.mock_cv2.CAP_PROP_FOURCC, 1196444237)
        cam.stop()

    def test_linux_auto_uses_gstreamer_when_available(self):
        mod = self._import_usb_camera()
        mock_plat = MagicMock()
        mock_plat.system.return_value = "Linux"
        mod._stdlib_platform = mock_plat
        mod._has_gstreamer = MagicMock(return_value=True)
        native_cap = MagicMock()
        native_cap.prime.return_value = True

        with patch.object(mod, "_GStreamerJpegCapture", return_value=native_cap) as capture_cls:
            cam = mod.USBCamera(
                name="cam0",
                device_id=0,
                width=640,
                height=360,
                encoding="jpg",
                native_compressed=True,
            )
            cam.start()

        capture_cls.assert_called_once_with(
            device_path="/dev/video0",
            width=640,
            height=360,
            fps=30.0,
        )
        self.assertEqual(cam._capture_backend_active, "gstreamer")
        self.assertTrue(cam._native_compressed_active)
        cam.stop()
        native_cap.release.assert_called_once()

    def test_linux_auto_falls_back_to_v4l2_when_gstreamer_open_fails(self):
        mod = self._import_usb_camera()
        mock_plat = MagicMock()
        mock_plat.system.return_value = "Linux"
        mod._stdlib_platform = mock_plat
        mod._has_gstreamer = MagicMock(return_value=True)

        native_cap = MagicMock()
        native_cap.prime.return_value = False
        native_cap.error_detail.return_value = ""
        gst_cap = MagicMock()
        gst_cap.isOpened.return_value = False
        v4l2_cap = MagicMock()
        v4l2_cap.isOpened.return_value = True
        self.mock_cv2.VideoCapture.side_effect = [gst_cap, v4l2_cap]

        with patch.object(mod, "_GStreamerJpegCapture", return_value=native_cap):
            cam = mod.USBCamera(
                name="cam0",
                device_id=0,
                encoding="jpg",
                native_compressed=True,
            )
            cam.start()

        self.assertEqual(self.mock_cv2.VideoCapture.call_args_list[0][0][1], self.mock_cv2.CAP_GSTREAMER)
        self.assertEqual(self.mock_cv2.VideoCapture.call_args_list[1][0], (0, self.mock_cv2.CAP_V4L2))
        self.assertEqual(cam._capture_backend_active, "opencv")
        self.assertFalse(cam._native_compressed_active)
        v4l2_cap.set.assert_any_call(self.mock_cv2.CAP_PROP_FOURCC, 1196444237)
        self.assertNotIn(
            call(self.mock_cv2.CAP_PROP_CONVERT_RGB, 0),
            v4l2_cap.set.call_args_list,
        )
        native_cap.release.assert_called_once()
        gst_cap.release.assert_called_once()
        cam.stop()

    def test_linux_auto_without_gstreamer_uses_decoded_opencv(self):
        mod = self._import_usb_camera()
        mock_plat = MagicMock()
        mock_plat.system.return_value = "Linux"
        mod._stdlib_platform = mock_plat
        mod._has_gstreamer = MagicMock(return_value=False)

        cam = mod.USBCamera(
            name="cam0",
            device_id=0,
            encoding="jpg",
            native_compressed=True,
        )
        cam.start()

        self.assertEqual(cam._capture_backend_active, "opencv")
        self.assertFalse(cam._native_compressed_active)
        self.assertNotIn(
            call(self.mock_cv2.CAP_PROP_CONVERT_RGB, 0),
            self.mock_cap.set.call_args_list,
        )
        cam.stop()

    def test_explicit_gstreamer_requires_linux(self):
        mod = self._import_usb_camera()
        mock_plat = MagicMock()
        mock_plat.system.return_value = "Darwin"
        mod._stdlib_platform = mock_plat

        cam = mod.USBCamera(name="cam0", device_id=0, capture_backend="gstreamer")

        with self.assertRaisesRegex(RuntimeError, "only supported on Linux"):
            cam.start()
        self.mock_cv2.VideoCapture.assert_not_called()

    def test_explicit_gstreamer_requires_executable(self):
        mod = self._import_usb_camera()
        mock_plat = MagicMock()
        mock_plat.system.return_value = "Linux"
        mod._stdlib_platform = mock_plat
        mod._has_gstreamer = MagicMock(return_value=False)

        cam = mod.USBCamera(name="cam0", device_id=0, capture_backend="gstreamer")

        with self.assertRaisesRegex(RuntimeError, "requires gst-launch-1.0"):
            cam.start()
        self.mock_cv2.VideoCapture.assert_not_called()

    def test_auto_falls_back_when_gstreamer_process_cannot_launch(self):
        mod = self._import_usb_camera()
        mock_plat = MagicMock()
        mock_plat.system.return_value = "Linux"
        mod._stdlib_platform = mock_plat
        mod._has_gstreamer = MagicMock(return_value=True)

        with patch.object(mod, "_GStreamerJpegCapture", side_effect=OSError("cannot execute")):
            cam = mod.USBCamera(
                name="cam0",
                device_id=0,
                encoding="jpg",
                native_compressed=True,
            )
            cam.start()

        self.assertEqual(cam._capture_backend_active, "opencv")
        self.assertFalse(cam._native_compressed_active)
        cam.stop()

    def test_incomplete_native_jpeg_is_dropped_instead_of_reencoded(self):
        mod = self._import_usb_camera()
        mock_plat = MagicMock()
        mock_plat.system.return_value = "Linux"
        mod._stdlib_platform = mock_plat

        incomplete = self.np.frombuffer(b"not-a-jpeg", dtype=self.np.uint8).reshape(1, -1)
        self.mock_cap.read.return_value = (True, incomplete)
        cam = mod.USBCamera(
            name="cam0",
            device_id=0,
            encoding="jpg",
            native_compressed=True,
            capture_backend="opencv",
        )
        cam.start()

        success, width, height, encoding, image = cam.read_frame()

        self.assertFalse(success)
        self.assertEqual((width, height, encoding, image), (0, 0, "jpg", None))
        self.mock_cv2.imencode.assert_not_called()
        cam.stop()


class TestGStreamerNativeHelpers(unittest.TestCase):
    def test_native_command_uses_jpegparse_and_stdout_sink(self):
        from rynnkit.cameras.usb_camera import _build_gstreamer_native_command

        command = _build_gstreamer_native_command(
            device_path="/dev/video3",
            width=640,
            height=360,
            fps=30,
        )

        self.assertIn("jpegparse", command)
        self.assertIn("fdsink", command)
        self.assertIn("fd=1", command)
        self.assertIn("image/jpeg,width=640,height=360,framerate=30/1", command)

    def test_pop_complete_jpeg_discards_prefix_and_keeps_next_frame(self):
        from rynnkit.cameras.usb_camera import _pop_complete_jpeg

        buffer = bytearray(
            b"prefix"
            b"\xff\xd8first\xff\xd9"
            b"\xff\xd8second\xff\xd9"
        )

        self.assertEqual(_pop_complete_jpeg(buffer), b"\xff\xd8first\xff\xd9")
        self.assertEqual(_pop_complete_jpeg(buffer), b"\xff\xd8second\xff\xd9")
        self.assertEqual(buffer, bytearray())

    def test_extract_native_jpeg_trims_wrapping_bytes(self):
        from rynnkit.cameras.usb_camera import _extract_native_jpeg_bytes

        frame = b"wrapper\xff\xd8jpeg-data\xff\xd9trailer"

        self.assertEqual(_extract_native_jpeg_bytes(frame), b"\xff\xd8jpeg-data\xff\xd9")


if __name__ == "__main__":
    unittest.main()
