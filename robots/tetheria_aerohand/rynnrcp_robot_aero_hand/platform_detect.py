"""Runtime platform detection for Aero Hand gesture-inference backends.

Picks the best acceleration backend for the device the code runs on, so
deployments no longer need to hard-code ``AERO_HAND_INFERENCE_BACKEND``:

- Rockchip RK3588 / RK3566 (NPU via RKNN)  -> ``rga_rknn_zero``
- Moore Threads E300 / M-series (MTNN NPU) -> ``mediapipe_lite_mtnn_zero``
- NVIDIA Jetson (TensorRT / CUDA)          -> ``mediapipe_lite_jetson``
- anything else                            -> ``mediapipe_lite`` (CPU baseline)

Detection reads ``/proc/device-tree/compatible``, PCI vendor IDs and the
vendor runtime libraries. ``setup_aero_hand.sh`` has an install-time variant
of these checks; the two deliberately differ in fallback signals (this one
also distinguishes RK3588 from RK3566).

``AERO_HAND_PLATFORM`` overrides detection for debugging / mixed setups.
Results are cached per process.
"""
from __future__ import annotations

import functools
import glob
import os
import platform

PLATFORM_GENERIC = "generic"
PLATFORM_RK3588 = "rk3588"
PLATFORM_RK3566 = "rk3566"
PLATFORM_MTNN = "mtnn_e300"
PLATFORM_JETSON = "jetson"

ALL_PLATFORMS = (
    PLATFORM_RK3588,
    PLATFORM_RK3566,
    PLATFORM_MTNN,
    PLATFORM_JETSON,
    PLATFORM_GENERIC,
)

_PLATFORM_ENV = "AERO_HAND_PLATFORM"
# Moore Threads PCI vendor ID (lspci: vendor 1ed5).
_MTHREADS_PCI_VENDOR = "0x1ed5"

# Backend registered by each accelerator module (see accelerators/*.py).
_RECOMMENDED_BACKENDS: dict[str, tuple[str, str | None]] = {
    PLATFORM_RK3588: (
        "rga_rknn_zero",
        "rynnrcp_robot_aero_hand.accelerators.rga_rknn_zero",
    ),
    PLATFORM_RK3566: (
        "rga_rknn_zero",
        "rynnrcp_robot_aero_hand.accelerators.rga_rknn_zero",
    ),
    PLATFORM_MTNN: (
        "mediapipe_lite_mtnn_zero",
        "rynnrcp_robot_aero_hand.accelerators.mtnn_zero",
    ),
    PLATFORM_JETSON: (
        "mediapipe_lite_jetson",
        "rynnrcp_robot_aero_hand.accelerators.mediapipe_lite_jetson",
    ),
    PLATFORM_GENERIC: ("mediapipe_lite", None),
}


def _device_tree_compatible() -> str:
    try:
        with open("/proc/device-tree/compatible", "rb") as fh:
            return fh.read().decode("utf-8", "replace").lower().replace("\x00", " ")
    except OSError:
        return ""


def _has_mthreads_gpu() -> bool:
    for vendor_path in glob.glob("/sys/bus/pci/devices/*/vendor"):
        try:
            with open(vendor_path, encoding="ascii") as fh:
                if fh.read().strip().lower() == _MTHREADS_PCI_VENDOR:
                    return True
        except OSError:
            continue
    return False


def _runtime_lib_installed(name: str) -> bool:
    for pattern in (f"/usr/lib/{name}*", f"/usr/local/lib/{name}*",
                    f"/usr/lib/aarch64-linux-gnu/{name}*"):
        if glob.glob(pattern):
            return True
    return False


@functools.lru_cache(maxsize=1)
def detect_platform() -> str:
    """Return the accelerator platform this process is running on."""

    override = os.environ.get(_PLATFORM_ENV, "").strip().lower()
    if override:
        if override not in ALL_PLATFORMS:
            raise ValueError(
                f"{_PLATFORM_ENV}={override!r} is not one of {ALL_PLATFORMS}"
            )
        return override

    if platform.system() != "Linux":
        return PLATFORM_GENERIC

    compatible = _device_tree_compatible()

    if (os.path.isfile("/etc/nv_tegra_release")
            or "nvidia,tegra" in compatible):
        return PLATFORM_JETSON

    if _has_mthreads_gpu() or _runtime_lib_installed("libmtnnrt.so"):
        return PLATFORM_MTNN

    if "rk3588" in compatible:
        return PLATFORM_RK3588
    if "rk3566" in compatible or "rk3568" in compatible:
        return PLATFORM_RK3566
    # Rockchip boards whose compatible string we do not list explicitly:
    # the rk3566 fp16 RKNN models are the conservative default.
    if "rockchip" in compatible or _runtime_lib_installed("librknnrt.so"):
        return PLATFORM_RK3566

    return PLATFORM_GENERIC


def recommended_backend(detected: str | None = None) -> tuple[str, str | None]:
    """Best ``(backend_name, module_name)`` for a platform (default: detect)."""

    plat = detected or detect_platform()
    return _RECOMMENDED_BACKENDS.get(plat, _RECOMMENDED_BACKENDS[PLATFORM_GENERIC])
