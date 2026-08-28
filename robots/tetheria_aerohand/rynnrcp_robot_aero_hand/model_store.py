"""Model store: fetch Aero Hand inference weights from the Rynn model zoo.

Weights do not live in git anymore. They are distributed by
``https://gitee.com/alibaba-damo-academy/rynnmodelzoo`` (branch ``master``,
layout ``models/<platform>/<file>``) and materialized under the repo-root
``models/`` tree, which is exactly where the accelerator backends look:

  models/rk3566/         RKNN fp16 models (RK3566; also fallback on RK3588)
  models/rk3588/         RKNN fp16 models for RK3588
  models/mtnn/           MTNN models for the Moore Threads E300 NPU
  models/jetson_orin_nx/ prebuilt TensorRT engines (trt10.4.0 / sm87)
  models/onnx/           ONNX source models (Jetson runtime-build path)

Every file is size + sha256 pinned in :data:`MANIFEST` below. ``ensure_models``
is idempotent: existing verified files are skipped, downloads stream to a
``.tmp`` file and are renamed into place only after verification.

Gitee's ``raw`` endpoint rejects files over ~1 MB with HTTP 403, so the
fetcher falls back to the v5 contents API (base64, capped at 10 MB) and
finally to a shallow ``git clone`` of the model zoo.

Env overrides:

  AERO_HAND_MODELZOO_BASE   alternate mirror base URL (default: gitee raw)
  AERO_HAND_MODELS_DIR      weights root (default: <repo root>/models)
"""
from __future__ import annotations

import base64
import hashlib
import json
import logging
import os
import shutil
import subprocess
import tempfile
import urllib.request
from typing import Callable

from .platform_detect import (
    PLATFORM_GENERIC,
    PLATFORM_JETSON,
    PLATFORM_MTNN,
    PLATFORM_RK3566,
    PLATFORM_RK3588,
)

logger = logging.getLogger(__name__)

MODELZOO_REPO = "alibaba-damo-academy/rynnmodelzoo"
MODELZOO_BRANCH = "master"
DEFAULT_MODELZOO_BASE = (
    f"https://gitee.com/{MODELZOO_REPO}/raw/{MODELZOO_BRANCH}"
)
_API_CONTENTS_URL = (
    f"https://gitee.com/api/v5/repos/{MODELZOO_REPO}/contents"
)
_CLONE_URL = f"https://gitee.com/{MODELZOO_REPO}.git"
_BASE_ENV = "AERO_HAND_MODELZOO_BASE"
_MODELS_DIR_ENV = "AERO_HAND_MODELS_DIR"
_HTTP_TIMEOUT_S = 60.0

# sha256 / size pinned from the model zoo at 2026-08-24 (master @ 75bd3dd).
# entry: (relative name inside the platform dir, sha256, size_bytes)
MANIFEST: dict[str, tuple[tuple[str, str, int], ...]] = {
    "rk3566": (
        ("palm_detection_full_fp16_rknn2.3.0.rknn",
         "f182d408e46b2c2abc061b1132853e231f6fa51fdf4832d1f4f268520cfc28e4",
         2634020),
        ("palm_detection_lite_fp16_rknn2.3.0.rknn",
         "50b9fa8058154289719404e994242b0404513fe5533e260300724766902a3f60",
         2241764),
        ("hand_landmark_full_fp16_rknn2.3.0.rknn",
         "ea866ef9e6a5bc5cf94ff27ee95e7b0754582f2ba9fea9b1dcdbeaf234b1a7b4",
         3648755),
        ("hand_landmark_lite_fp16_rknn2.3.0.rknn",
         "f9c00d190bbbaf6459b47ca5aeb175ffb1aa6d134a4421a921a9cf36754370ab",
         2039987),
    ),
    "rk3588": (
        ("palm_detection_full_fp16_rknn2.3.0.rknn",
         "4079b57fbe8ce517b1c81c2033c3e648ce51926cb439edc8654d0e9cd1042c71",
         3235172),
        ("hand_landmark_full_fp16_rknn2.3.0.rknn",
         "b71dfd422357563e06ffe35c98200fc03bd7eb525ddafa3ac18d04d8e54fe221",
         3968755),
    ),
    "mtnn": (
        ("palm_detection.mtnn",
         "f0f47900ab9618ab9e425a75732acd230108d85a11b15105778bd479ae9c806f",
         2240031),
        ("hand_landmark.mtnn",
         "61e13595d1f277a31747494fa22c07ad5705084484bba534e46a02159719f72c",
         3246630),
    ),
    "jetson_orin_nx": (
        ("palm_detection_192_nchw_trt10.4.0_sm87.engine",
         "97cebc63fa0f089ab954cae814ac524ae4be1ef3546815187bbd7c47c65b9e7e",
         2978436),
        ("hand_landmark_224_trt10.4.0_sm87.engine",
         "ee5d52679382a65155734b80745279cbfe12588e88b80e48bba147bd3d427fb7",
         5923124),
    ),
    "onnx": (
        ("palm_detection_192.onnx",
         "6ff52897daad25b662bebde41c02b33c4771f6b08a102dfe3bbd101f0ffced0a",
         4589466),
        ("palm_detection_192_nchw.onnx",
         "0cc6716f002d862732df8d9a93e76bf3d21e0135d21f306467e6a78a50870020",
         4589522),
        ("hand_landmark_224.onnx",
         "89c8728102d1b9b09a25e279fdb7fc08ee3774779fb543f36b63206fbd3b74f2",
         10948267),
    ),
}

# Platform id (platform_detect) -> model zoo subdir(s) to materialize.
_PLATFORM_DIRS: dict[str, tuple[str, ...]] = {
    PLATFORM_RK3566: ("rk3566",),
    PLATFORM_RK3588: ("rk3588",),
    PLATFORM_MTNN: ("mtnn",),
    PLATFORM_JETSON: ("onnx", "jetson_orin_nx"),
    PLATFORM_GENERIC: (),
}


def repo_root() -> str:
    """Repo root for the default ``models/`` location.

    model_store.py lives at ``<repo>/robots/tetheria_aerohand/
    rynnrcp_robot_aero_hand/``, i.e. four levels below the repo root.
    """

    return os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))


def models_root() -> str:
    return os.environ.get(
        _MODELS_DIR_ENV, os.path.join(repo_root(), "models")
    )


def _modelzoo_base() -> str:
    return os.environ.get(_BASE_ENV, DEFAULT_MODELZOO_BASE).rstrip("/")


def resolve_model_dir(platform: str, root: str | None = None) -> str:
    """Directory holding the model files a backend needs for ``platform``.

    RK3588 prefers its dedicated exports but transparently falls back to the
    rk3566 models, which are accuracy-verified on Rockchip NPU in general.
    """

    base = root or models_root()
    if platform == PLATFORM_RK3588:
        preferred = os.path.join(base, "rk3588")
        if os.path.isdir(preferred) or not os.path.isdir(os.path.join(base, "rk3566")):
            return preferred
        return os.path.join(base, "rk3566")
    if platform == PLATFORM_MTNN:
        return os.path.join(base, "mtnn")
    if platform == PLATFORM_JETSON:
        return os.path.join(base, "onnx")
    return os.path.join(base, "rk3566")


def _verify(path: str, sha256: str, size: int) -> bool:
    if not os.path.isfile(path):
        return False
    if os.path.getsize(path) != size:
        return False
    digest = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest() == sha256


def _http_get(url: str) -> bytes:
    request = urllib.request.Request(
        url, headers={"User-Agent": "rynnrcp-aero-hand-model-store"}
    )
    with urllib.request.urlopen(request, timeout=_HTTP_TIMEOUT_S) as resp:
        return resp.read()


def _stream_to(url: str, tmp_path: str) -> None:
    request = urllib.request.Request(
        url, headers={"User-Agent": "rynnrcp-aero-hand-model-store"}
    )
    with urllib.request.urlopen(request, timeout=_HTTP_TIMEOUT_S) as resp, \
            open(tmp_path, "wb") as fh:
        shutil.copyfileobj(resp, fh)


def _fetch_via_api(remote_rel: str, tmp_path: str) -> None:
    """Gitee v5 contents API (base64 payload, works up to 10 MB)."""

    payload = json.loads(
        _http_get(f"{_API_CONTENTS_URL}/{remote_rel}").decode("utf-8")
    )
    with open(tmp_path, "wb") as fh:
        fh.write(base64.b64decode(payload["content"]))


def _fetch_via_clone(remote_rel: str, tmp_path: str) -> None:
    """Last resort: shallow-clone the model zoo (handles files > 10 MB)."""

    if shutil.which("git") is None:
        raise RuntimeError("git executable not found")
    workdir = tempfile.mkdtemp(prefix="rynnmodelzoo_")
    try:
        subprocess.run(
            ["git", "clone", "--depth", "1", "--branch", MODELZOO_BRANCH,
             _CLONE_URL, workdir],
            check=True, capture_output=True, timeout=900,
        )
        src = os.path.join(workdir, remote_rel)
        if not os.path.isfile(src):
            raise RuntimeError(f"{remote_rel} missing in the model zoo clone")
        shutil.copyfile(src, tmp_path)
    finally:
        shutil.rmtree(workdir, ignore_errors=True)


def fetch_file(remote_rel: str, dest_path: str, sha256: str, size: int,
               *, force: bool = False) -> None:
    """Materialize and verify one model file; raises RuntimeError on failure."""

    if not force and _verify(dest_path, sha256, size):
        return

    os.makedirs(os.path.dirname(dest_path), exist_ok=True)
    tmp_path = dest_path + ".tmp"
    base = _modelzoo_base()
    attempts: list[tuple[str, Callable[[], None]]] = [
        ("http", lambda: _stream_to(f"{base}/{remote_rel}", tmp_path)),
    ]
    if base == DEFAULT_MODELZOO_BASE:
        attempts.append(
            ("gitee api", lambda: _fetch_via_api(remote_rel, tmp_path)))
        attempts.append(
            ("git clone", lambda: _fetch_via_clone(remote_rel, tmp_path)))

    failures: list[str] = []
    for strategy, attempt in attempts:
        logger.info("aero-hand: fetching %s via %s (%d bytes)",
                    remote_rel, strategy, size)
        try:
            attempt()
            if _verify(tmp_path, sha256, size):
                os.replace(tmp_path, dest_path)
                logger.info("aero-hand model ready: %s", dest_path)
                return
            failures.append(f"{strategy}: checksum mismatch")
        except Exception as exc:  # noqa: BLE001 - try the next strategy
            failures.append(f"{strategy}: {exc}")
        finally:
            if os.path.exists(tmp_path):
                try:
                    os.remove(tmp_path)
                except OSError:
                    pass
    raise RuntimeError(
        "failed to fetch aero-hand model "
        f"{remote_rel} -> {dest_path}: " + "; ".join(failures) + "\n"
        "manual fallback: python robots/tetheria_aerohand/scripts/"
        "fetch_aero_hand_models.py --platform all"
    )


def ensure_subdir(subdir: str, root: str | None = None, *,
                  force: bool = False) -> list[str]:
    """Download every file of one zoo subdir (e.g. ``rk3566``, ``onnx``)."""

    entries = MANIFEST.get(subdir)
    if not entries:
        raise KeyError(f"unknown model zoo subdir: {subdir!r}")
    base = root or models_root()
    logger.info("aero-hand: ensuring %d model(s) under %s",
                len(entries), os.path.join(base, subdir))
    fetched: list[str] = []
    for name, sha256, size in entries:
        dest = os.path.join(base, subdir, name)
        fetch_file(f"models/{subdir}/{name}", dest, sha256, size,
                   force=force)
        fetched.append(dest)
    return fetched


def ensure_backend_files(paths: tuple | list) -> None:
    """Fetch the zoo subdir matching ``paths`` unless all files exist.

    Backends call this right before their model-existence check, so a fresh
    device downloads weights on first use instead of failing. Paths pointing
    outside a known zoo subdir (custom ``AERO_HAND_*_DIR`` overrides) only get
    a warning -- the caller's FileNotFoundError then carries the real path.
    """

    if all(os.path.isfile(path) for path in paths):
        return
    subdir = os.path.basename(os.path.dirname(paths[0]))
    if subdir not in MANIFEST:
        logger.warning(
            "aero-hand: %s is not a model-zoo subdir; place the weights "
            "manually", os.path.dirname(paths[0]),
        )
        return
    ensure_subdir(subdir)


def ensure_models(platform: str, root: str | None = None, *,
                  force: bool = False) -> list[str]:
    """Download every model the given platform needs; returns local paths."""

    fetched: list[str] = []
    for subdir in _PLATFORM_DIRS.get(platform, ()):
        fetched.extend(ensure_subdir(subdir, root, force=force))
    return fetched
