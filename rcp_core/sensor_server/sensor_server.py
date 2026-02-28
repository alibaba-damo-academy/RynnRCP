# rcp_core/sensor_server/sensor_server.py

"""
Sensor server (image retrieval from buffer).
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module defines :class:`~python.sensor_server.sensor_server.SensorServer`, a
:class:`~rcp_core.common.server.base_server.BaseServer` subclass that serves camera
frames from the shared buffer as encoded image bytes.

Core idea
---------
Upstream input adapters push structured image dictionaries into the GlobalBuffer under
keys like ``observation.images.<camera_name>``. This server:
- snapshots the current per-server buffer
- synchronizes frames across requested cameras using timestamp alignment
- converts each structured image dict into encoded bytes (JPEG/PNG, optional resize)
- exposes the functionality via tools registered on :class:`~rcp_core.common.bus.rcp_bus.RcpBus`

Image conversion
----------------
- Uses :class:`~rcp_core.common.utils.image_converter.ImageConverter` with a PIL backend
  (:class:`~rcp_core.common.utils.image_backend_pil.ImageBackendPIL`) to decode/encode/resize.

Tools
-----
1) ``get_image(image_opts=None)``
   - If ``image_opts`` is omitted/empty, it automatically selects all keys in the buffer
     beginning with ``"observation.images."`` and defaults each to ``encoding="jpeg"``.
   - Otherwise, uses the provided per-key options:

     ``{ "<image_key>": {"encoding": "jpeg|png|...", "width": int, "height": int}, ... }``

   - Synchronizes selected keys via :func:`~rcp_core.common.utils.sync_frames.sync_by_trigger_time`.
   - Converts each aligned image to encoded bytes and returns:

     ``{"success": bool, "message": str, "result": {"<image_key>": bytes, ...}}``

2) ``get_image_info()``
   - Inspects the latest buffered entry for each ``observation.images.*`` key and returns
     basic metadata:
     - for raw images: uses ``height``, ``width``, and ``encoding`` fields directly
     - for compressed images: attempts to open the bytes with PIL to derive ``(width, height)``
   - Returns the same bus result envelope.

Bus registration
----------------
:meth:`SensorServer.bind_bus` registers both tools with input/output schema metadata for
introspection and external RPC-style calls.
"""

from typing import Any, Dict, Tuple, Optional

from ..common.server.base_server import BaseServer
from ..common.utils.image_converter import ImageConverter
from ..common.utils.image_backend_pil import ImageBackendPIL
from ..common.bus.rcp_bus import RcpBus
from ..common.utils.sync_frames import sync_by_trigger_time
from rcp_core.common.utils.logger import server_logger

logger = server_logger()


class SensorServer(BaseServer):
    """Sensor server that reads image messages from the buffer and returns encoded image bytes."""

    def __init__(self, config: Dict[str, Any]):
        """Initialize the sensor server and create the image converter."""
        super().__init__(config, "sensor_server")
        # self.image_converter = ImageConverter()
        self.image_converter = ImageConverter(backend=ImageBackendPIL())

        self._image_key_to_brand: Dict[str, str] = self._parse_image_brands_from_config(
            self.server_config
        )

    @staticmethod
    def _parse_image_brands_from_config(config: Dict[str, Any]) -> Dict[str, str]:
        """
        Parse server config to build mapping:
          out_key (e.g. observation.images.front) -> brand (e.g. usb_camera)
        """
        m: Dict[str, str] = {}
        inputs = config.get("inputs", []) or []
        for inp in inputs:
            params = inp.get("params", {}) or {}
            out_key = params.get("out_key")
            init_args = params.get("init_args", {}) or {}
            brand = init_args.get("brand")
            if out_key and brand:
                m[out_key] = brand

        return m

    def get_image(
        self, image_opts: Optional[Dict[str, Dict[str, Any]]] = None
    ) -> Dict[str, bytes]:
        """
        Read image messages from the buffer and convert them to encoded bytes.

        Behavior:
        - If image_opts is provided, only the specified keys are synchronized and returned.
        - If image_opts is None or an empty dict, all keys starting with
          'observation.images.' are used, encoded as JPEG with no resizing.

        Args:
            image_opts: Optional dict specifying per-key options, e.g.:
                {
                  "observation.images.cam0": {
                      "encoding": "png",
                      "width": 320,
                      "height": 240
                  },
                  ...
                }

        Returns:
            Dict wrapped by bus.make_result:
                {
                  "success": bool,
                  "message": str,
                  "result": {
                      "<image_key>": bytes  # encoded image data
                  }
                }
        """
        res: Dict[str, bytes] = {}

        # 1. Get buffer snapshot: { key: Deque[(ts, value)] }
        snap = self.get_buffer()

        # 2. If no image_opts given, collect all observation.images.* keys
        if not image_opts:
            image_opts = {}
            for key, q in snap.items():
                # q is Deque[(ts, value)]
                if key.startswith("observation.images.") and q and q[-1][1] is not None:
                    # Default options: JPEG encoding, no resize
                    image_opts[key] = {
                        "encoding": "jpeg",
                    }

        if not image_opts:
            return self.bus.make_result(
                success=False,
                result={},
                message="no image keys available in buffer",
            )

        # 3. Use image_opts keys for synchronization
        out_keys = list(image_opts.keys())

        aligned = sync_by_trigger_time(
            buffers=snap,
            out_keys=out_keys,
        )

        if aligned is None:
            msg = (
                "[SensorServer] get_image: sync_by_trigger_time failed, "
                "no aligned frame for given image_opts"
            )
            logger.error(msg)
            return self.bus.make_result(
                success=False,
                result={},
                message="sync failed: no aligned frame",
            )

        # Convert to dict for easier lookup: key -> (ts, value)
        aligned_dict: Dict[str, Tuple[float, Any]] = {
            k: (ts, v) for k, ts, v in aligned
        }

        # 4. For each requested key, convert aligned raw message to encoded image
        for key, opt in image_opts.items():
            pair = aligned_dict.get(key)
            if not pair:
                logger.warning(
                    f"[SensorServer] get_image: no aligned msg for key '{key}', skip"
                )
                continue

            ts, raw_msg = pair
            if raw_msg is None:
                logger.warning(
                    f"[SensorServer] get_image: aligned msg for key '{key}' is None, skip"
                )
                continue

            encoding = opt.get("encoding", "jpeg").lower()
            width = opt.get("width")
            height = opt.get("height")

            try:
                res[key] = self.image_converter.msg_to_img(
                    raw_msg, img_type=encoding, width=width, height=height
                )
            except Exception as e:
                logger.error(
                    f"[SensorServer] conversion failed, key='{key}', ts={ts}, "
                    f"encoding={encoding}, size=({width}, {height}): {e}"
                )

        success = bool(res)
        return self.bus.make_result(
            success=success,
            result=res,
            message="OK" if success else "no image converted",
        )

    def get_image_info(self) -> Dict[str, Any]:
        """
        Retrieve encoding, width, height, and key information for each camera.

        Returns:
            Dict wrapped by bus.make_result:
                {
                    "success": bool,
                    "message": str,
                    "result": {
                        "<image_key>": {
                            "encoding": str,
                            "width": Optional[int],
                            "height": Optional[int]
                        }
                    }
                }
        """
        res: Dict[str, Dict[str, Any]] = {}

        # Retrieve buffer snapshot
        snap = self.get_buffer()

        for key, q in snap.items():
            if key.startswith("observation.images.") and q and q[-1][1] is not None:
                img_msg = q[-1][1]
                brand = self._image_key_to_brand.get(key)

                if isinstance(img_msg, dict) and "type" in img_msg:
                    img_type = img_msg["type"]
                    if img_type == "image":
                        height = img_msg.get("height")
                        width = img_msg.get("width")
                        encoding = img_msg.get("encoding", "unknown")

                        res[key] = {
                            "format": encoding,
                            "width": width,
                            "height": height,
                            "brand": brand,
                        }
                    elif img_type == "compressed":
                        compressed_data = img_msg.get("data")
                        img_format = img_msg.get("format", "unknown").lower()
                        if compressed_data:
                            try:
                                from PIL import Image
                                import io

                                # Open the compressed image from bytes
                                image = Image.open(io.BytesIO(compressed_data))
                                width, height = image.size

                                res[key] = {
                                    "format": img_format,
                                    "width": width,
                                    "height": height,
                                    "brand": brand,
                                }
                            except Exception as e:
                                logger.error(
                                    f"[SensorServer] Failed to process compressed image for key '{key}': {e}"
                                )
                                continue

        success = bool(res)
        return self.bus.make_result(
            success=success,
            result=res,
            message="OK" if success else "no image keys available",
        )

    def bind_bus(self, bus: RcpBus):
        """Register get_image on the bus for external image requests."""
        super().bind_bus(bus)
        bus.add_tool(
            "get_image",
            self.get_image,
            input_schema={
                "image_opts": {
                    "observation.images.<camera_name>": {
                        "encoding": "str  # 'jpeg' or 'png' (default: jpeg)",
                        "width": "int  # optional resize width",
                        "height": "int  # optional resize height",
                    }
                }
            },
            output_schema={
                "success": "bool",
                "message": "str",
                "result": {
                    "observation.images.<camera_name>": "bytes  # encoded image bytes (jpeg/png)",
                },
            },
            description=(
                "Get encoded images from internal buffer for specified camera keys and options; "
                "if no options are provided, return all 'observation.images.*' keys."
            ),
        )
        bus.add_tool(
            "get_image_info",
            self.get_image_info,
            input_schema=None,
            output_schema={
                "success": "bool",
                "message": "str",
                "result": {
                    "observation.images.<camera_name>": {
                        "encoding": "str  # 'jpeg'/'png'",
                        "width": "int | None  # image width",
                        "height": "int | None  # image height",
                        "brand": "str | None  # camera brand",
                    },
                },
            },
            description="Get per-camera image metadata for keys 'observation.images.*'.",
        )
