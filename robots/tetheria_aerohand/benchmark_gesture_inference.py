#!/usr/bin/env python3
"""Benchmark Aero Hand gesture models with recorded single- and dual-hand clips."""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import statistics
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable


@dataclass(frozen=True)
class Candidate:
    name: str
    engine: str
    input_width: int
    model_complexity: int = 1
    backend: str = ""
    backend_module: str | None = None
    threads: int | None = None


@dataclass
class Result:
    profile: str
    candidate: str
    engine: str
    backend: str
    backend_module: str | None
    inference_threads: int | None
    model_bytes: int
    input_size: str
    frames: int
    complete_frames: int
    complete_detection_rate_percent: float
    mean_detected_hands: float
    latency_mean_ms: float
    latency_p50_ms: float
    latency_p95_ms: float
    latency_max_ms: float
    process_cpu_percent: float
    shape_mae_vs_current: float | None


Detection = dict[str, list[float]]


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "录制同一摄像头的单手和双手动态样本，对比当前 MediaPipe Tasks full "
            "模型与官方 legacy full/lite 模型。脚本只访问摄像头，不连接串口。"
        )
    )
    parser.add_argument("--camera", type=int, default=0, help="摄像头编号，默认 0")
    parser.add_argument("--width", type=int, default=640, help="录制宽度，默认 640")
    parser.add_argument("--height", type=int, default=480, help="录制高度，默认 480")
    parser.add_argument("--camera-fps", type=float, default=30.0, help="请求摄像头帧率，默认 30")
    parser.add_argument("--sample-fps", type=float, default=15.0, help="基准采样帧率，默认 15")
    parser.add_argument("--seconds", type=float, default=8.0, help="单手、双手各录制秒数，默认 8")
    parser.add_argument(
        "--threads",
        type=int,
        default=1,
        help="待验证后端的推理线程数，默认 1",
    )
    parser.add_argument(
        "--baseline-threads",
        type=int,
        default=1,
        help="mediapipe_lite 基线线程数，默认并建议保持 1",
    )
    parser.add_argument(
        "--backend",
        default="mediapipe_lite",
        help="要验证的注册后端名称，默认 mediapipe_lite",
    )
    parser.add_argument(
        "--backend-module",
        default="",
        help="注册优化后端的 Python 模块；benchmark、configure 和 server 使用相同值",
    )
    parser.add_argument(
        "--candidate",
        action="append",
        default=[],
        metavar="NAME=MODULE@THREADS",
        help=(
            "增加一个同帧候选后端，可重复使用；"
            "例如 rknn_int8=rynnrcp_robot_aero_hand.accelerators.rknn_int8@1"
        ),
    )
    parser.add_argument(
        "--baseline-only",
        action="store_true",
        help="只对比 640x480 的 mediapipe_lite 基线与指定优化后端",
    )
    parser.add_argument(
        "--profile",
        choices=("single", "dual", "both"),
        default="both",
        help="测试 single、dual 或两者；默认 both",
    )
    dataset_group = parser.add_mutually_exclusive_group()
    dataset_group.add_argument(
        "--record-dataset",
        type=Path,
        default=None,
        help="录制并保存固定帧数据集到指定目录",
    )
    dataset_group.add_argument(
        "--dataset",
        type=Path,
        default=None,
        help="读取已录制的数据集并跳过摄像头录制",
    )
    parser.add_argument(
        "--record-only",
        action="store_true",
        help="保存数据集后退出；需与 --record-dataset 一起使用",
    )
    parser.add_argument(
        "--task-model",
        type=Path,
        default=None,
        help="当前 hand_landmarker.task；默认使用 Aero Hand 内置模型",
    )
    parser.add_argument(
        "--json",
        type=Path,
        default=Path("aero_hand_gesture_benchmark.json"),
        help="结果 JSON，默认当前目录下 aero_hand_gesture_benchmark.json",
    )
    parser.add_argument(
        "--non-interactive",
        action="store_true",
        help="跳过回车确认，适合摄像头前已持续展示单手和双手的自动化环境",
    )
    parser.add_argument(
        "--no-preview",
        action="store_true",
        help="录制时不显示原始摄像头预览；预览本身不参与推理计时",
    )
    return parser


def _parse_candidate_spec(value: str) -> Candidate:
    name, separator, target = str(value).strip().partition("=")
    module, thread_separator, threads_text = target.rpartition("@")
    name = name.strip().lower()
    module = module.strip()
    if not separator or not thread_separator or not name or not module:
        raise ValueError(
            "candidate must use NAME=MODULE@THREADS, "
            "for example rknn_int8=accelerators.rknn_int8@1"
        )
    threads = int(threads_text)
    if threads <= 0:
        raise ValueError("candidate threads must be positive")
    return Candidate(
        name=f"{name}-640-t{threads}",
        engine="legacy",
        input_width=640,
        model_complexity=0,
        backend=name,
        backend_module=module,
        threads=threads,
    )


def _percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, round((len(ordered) - 1) * fraction)))
    return ordered[index]


def _resize_and_mirror(cv2: Any, image: Any, width: int) -> Any:
    height, source_width = image.shape[:2]
    if source_width != width:
        target_height = max(1, round(height * width / source_width))
        image = cv2.resize(image, (width, target_height))
    return cv2.flip(image, 1)


def _shape_vector(landmarks: Any) -> list[float]:
    points = [
        (float(landmark.x), float(landmark.y), float(landmark.z))
        for landmark in landmarks
    ]
    palm_scale = max(
        1e-6,
        sum((points[0][axis] - points[9][axis]) ** 2 for axis in range(3)) ** 0.5,
    )
    return [
        (
            sum((points[left][axis] - points[right][axis]) ** 2 for axis in range(3))
            ** 0.5
        )
        / palm_scale
        for left in range(len(points))
        for right in range(left + 1, len(points))
    ]


def _unique_side(target: Detection, label: str, index: int) -> str:
    side = str(label or f"hand_{index}").strip().lower()
    if side not in target:
        return side
    return f"{side}_{index}"


def _task_runner(mp: Any, model: Path, num_hands: int) -> tuple[Callable[[Any], Detection], Callable[[], None]]:
    from mediapipe.tasks import python
    from mediapipe.tasks.python import vision

    options = vision.HandLandmarkerOptions(
        base_options=python.BaseOptions(model_asset_path=str(model)),
        running_mode=vision.RunningMode.VIDEO,
        num_hands=num_hands,
    )
    landmarker = vision.HandLandmarker.create_from_options(options)
    timestamp_ms = 0

    def run(rgb: Any) -> Detection:
        nonlocal timestamp_ms
        timestamp_ms += 67
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        result = landmarker.detect_for_video(image, timestamp_ms)
        detections: Detection = {}
        for index, world in enumerate(result.hand_world_landmarks):
            label = (
                result.handedness[index][0].category_name
                if index < len(result.handedness) and result.handedness[index]
                else f"hand_{index}"
            )
            detections[_unique_side(detections, label, index)] = _shape_vector(world)
        return detections

    return run, landmarker.close


def _legacy_runner(
    mp: Any,
    *,
    num_hands: int,
    model_complexity: int,
    num_threads: int,
    backend_name: str,
    backend_module: str | None,
) -> tuple[Callable[[Any], Detection], Callable[[], None], int]:
    if model_complexity == 0:
        from rynnrcp_robot_aero_hand.gesture_inference import create_gesture_backend

        hands = create_gesture_backend(
            backend_name,
            module_name=backend_module,
            mediapipe=mp,
            max_num_hands=num_hands,
            num_threads=num_threads,
        )
        model_bytes = int(getattr(hands, "model_bytes", 0))
    else:
        hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=num_hands,
            model_complexity=model_complexity,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        model_bytes = _legacy_model_bytes(mp, model_complexity)

    def run(rgb: Any) -> Detection:
        result = hands.process(rgb)
        worlds = result.multi_hand_world_landmarks or []
        handedness = result.multi_handedness or []
        detections: Detection = {}
        for index, world in enumerate(worlds):
            label = (
                handedness[index].classification[0].label
                if index < len(handedness) and handedness[index].classification
                else f"hand_{index}"
            )
            detections[_unique_side(detections, label, index)] = _shape_vector(world.landmark)
        return detections

    return run, hands.close, model_bytes


def _legacy_model_bytes(mp: Any, complexity: int) -> int:
    package = Path(mp.__file__).resolve().parent
    variant = "lite" if complexity == 0 else "full"
    paths = (
        package / "modules" / "palm_detection" / f"palm_detection_{variant}.tflite",
        package / "modules" / "hand_landmark" / f"hand_landmark_{variant}.tflite",
    )
    return sum(path.stat().st_size for path in paths if path.is_file())


def _record_clip(
    *,
    cv2: Any,
    capture: Any,
    label: str,
    instructions: str,
    seconds: float,
    sample_fps: float,
    interactive: bool,
    preview: bool,
) -> list[Any]:
    print(f"\n[{label}录制]")
    print(instructions)
    if interactive:
        input("准备好后按回车开始录制...")
    print(f"开始录制 {seconds:.1f} 秒，请持续活动手指并缓慢转动手腕。")

    frames: list[Any] = []
    started = time.monotonic()
    deadline = started + seconds
    next_sample_at = started
    while time.monotonic() < deadline:
        ok, frame = capture.read()
        if not ok or frame is None:
            raise RuntimeError(f"{label}录制期间摄像头没有返回画面")
        now = time.monotonic()
        if now >= next_sample_at:
            frames.append(frame.copy())
            next_sample_at = now + 1.0 / sample_fps
        if preview:
            mirrored = cv2.flip(frame, 1)
            cv2.imshow("Aero Hand Benchmark - raw recording preview", mirrored)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                raise KeyboardInterrupt
    print(f"{label}录制完成：{len(frames)} 帧。")
    return frames


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _save_dataset(
    *,
    np: Any,
    directory: Path,
    clips: dict[str, list[Any]],
    camera: dict[str, Any],
) -> Path:
    directory = directory.expanduser().resolve()
    directory.mkdir(parents=True, exist_ok=True)
    manifest_path = directory / "manifest.json"
    targets = [manifest_path] + [
        directory / f"{profile}.npz" for profile in clips
    ]
    existing = [path for path in targets if path.exists()]
    if existing:
        names = ", ".join(path.name for path in existing)
        raise FileExistsError(
            f"dataset target already exists in {directory}: {names}"
        )

    profiles: dict[str, Any] = {}
    for profile, frames in clips.items():
        if not frames:
            raise ValueError(f"dataset profile has no frames: {profile}")
        path = directory / f"{profile}.npz"
        np.savez_compressed(path, frames=np.stack(frames, axis=0))
        profiles[profile] = {
            "file": path.name,
            "frames": len(frames),
            "sha256": _sha256(path),
        }
    manifest = {
        "format": "aero_hand_gesture_frames_v1",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "recording_platform": platform.platform(),
        "camera": camera,
        "profiles": profiles,
    }
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return directory


def _load_dataset(
    *,
    np: Any,
    directory: Path,
    profiles: tuple[str, ...],
) -> tuple[dict[str, Any], dict[str, Any]]:
    directory = directory.expanduser().resolve()
    manifest_path = directory / "manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError(f"dataset manifest not found: {manifest_path}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("format") != "aero_hand_gesture_frames_v1":
        raise ValueError(f"unsupported dataset format: {manifest.get('format')!r}")

    clips: dict[str, Any] = {}
    manifest_profiles = manifest.get("profiles") or {}
    for profile in profiles:
        metadata = manifest_profiles.get(profile)
        if not isinstance(metadata, dict):
            raise ValueError(f"dataset does not contain profile: {profile}")
        path = directory / str(metadata.get("file") or "")
        if not path.is_file():
            raise FileNotFoundError(f"dataset frame file not found: {path}")
        actual_checksum = _sha256(path)
        if actual_checksum != metadata.get("sha256"):
            raise ValueError(f"dataset checksum mismatch: {path}")
        with np.load(path, allow_pickle=False) as archive:
            frames = archive["frames"]
        if frames.ndim != 4 or frames.shape[-1] != 3:
            raise ValueError(f"invalid frame array in dataset: {path}")
        clips[profile] = frames
    return clips, manifest


def _match_mae(baseline: Detection, candidate: Detection, expected_hands: int) -> list[float]:
    if expected_hands == 1:
        if not baseline or not candidate:
            return []
        left = next(iter(baseline.values()))
        right = next(iter(candidate.values()))
        return [statistics.fmean(abs(a - b) for a, b in zip(left, right))]

    errors: list[float] = []
    for side in ("left", "right"):
        if side not in baseline or side not in candidate:
            continue
        errors.append(
            statistics.fmean(
                abs(a - b) for a, b in zip(baseline[side], candidate[side])
            )
        )
    return errors


def _run_candidate(
    *,
    cv2: Any,
    mp: Any,
    frames: list[Any],
    profile: str,
    candidate: Candidate,
    task_model: Path,
    baseline: list[Detection] | None,
) -> tuple[Result, list[Detection]]:
    expected_hands = 1 if profile == "single" else 2
    prepared = [
        cv2.cvtColor(
            _resize_and_mirror(cv2, frame, candidate.input_width),
            cv2.COLOR_BGR2RGB,
        )
        for frame in frames
    ]
    if candidate.engine == "task":
        run, close = _task_runner(mp, task_model, expected_hands)
        model_bytes = task_model.stat().st_size
        candidate_threads = None
    else:
        runtime_threads = candidate.threads or 1
        candidate_threads = runtime_threads if candidate.model_complexity == 0 else None
        run, close, model_bytes = _legacy_runner(
            mp,
            num_hands=expected_hands,
            model_complexity=candidate.model_complexity,
            num_threads=runtime_threads,
            backend_name=candidate.backend or "mediapipe_lite",
            backend_module=candidate.backend_module,
        )

    detections: list[Detection] = []
    latencies: list[float] = []
    started_wall = time.monotonic()
    started_cpu = time.process_time()
    try:
        for frame in prepared:
            started = time.perf_counter()
            detections.append(run(frame))
            latencies.append((time.perf_counter() - started) * 1000.0)
    finally:
        close()
    wall_time = time.monotonic() - started_wall
    cpu_time = time.process_time() - started_cpu

    complete_frames = sum(len(value) >= expected_hands for value in detections)
    errors = (
        [
            error
            for baseline_frame, candidate_frame in zip(baseline, detections)
            for error in _match_mae(baseline_frame, candidate_frame, expected_hands)
        ]
        if baseline is not None
        else []
    )
    height, width = prepared[0].shape[:2]
    return (
        Result(
            profile=profile,
            candidate=candidate.name,
            engine=candidate.engine,
            backend=candidate.backend or candidate.engine,
            backend_module=candidate.backend_module,
            inference_threads=(
                candidate_threads if candidate.engine == "legacy" else None
            ),
            model_bytes=model_bytes,
            input_size=f"{width}x{height}",
            frames=len(frames),
            complete_frames=complete_frames,
            complete_detection_rate_percent=complete_frames / len(frames) * 100.0,
            mean_detected_hands=statistics.fmean(len(value) for value in detections),
            latency_mean_ms=statistics.fmean(latencies),
            latency_p50_ms=_percentile(latencies, 0.50),
            latency_p95_ms=_percentile(latencies, 0.95),
            latency_max_ms=max(latencies),
            process_cpu_percent=cpu_time / wall_time * 100.0,
            shape_mae_vs_current=statistics.fmean(errors) if errors else None,
        ),
        detections,
    )


def _print_results(profile: str, results: list[Result]) -> None:
    title = "单手" if profile == "single" else "双手"
    print(f"\n[{title}结果]")
    headers = ("方案", "线程", "输入", "平均/P95延迟", "完整检出率", "平均手数", "手型MAE", "CPU")
    rows = [headers]
    for result in results:
        mae = (
            "基准"
            if result.shape_mae_vs_current == 0.0
            else (
                f"{result.shape_mae_vs_current:.5f}"
                if result.shape_mae_vs_current is not None
                else "无法比较"
            )
        )
        rows.append(
            (
                result.candidate,
                str(result.inference_threads or "-"),
                result.input_size,
                f"{result.latency_mean_ms:.1f}/{result.latency_p95_ms:.1f}ms",
                f"{result.complete_detection_rate_percent:.1f}%",
                f"{result.mean_detected_hands:.2f}",
                mae,
                f"{result.process_cpu_percent:.1f}%",
            )
        )
    widths = [max(len(row[index]) for row in rows) for index in range(len(headers))]
    for row_index, row in enumerate(rows):
        print("  ".join(value.ljust(widths[index]) for index, value in enumerate(row)))
        if row_index == 0:
            print("  ".join("-" * width for width in widths))


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.width <= 0 or args.height <= 0:
        raise SystemExit("--width 和 --height 必须大于 0")
    if args.camera_fps <= 0 or args.sample_fps <= 0 or args.seconds <= 0:
        raise SystemExit("--camera-fps、--sample-fps 和 --seconds 必须大于 0")
    if args.threads <= 0 or args.baseline_threads <= 0:
        raise SystemExit("--threads 和 --baseline-threads 必须大于 0")
    if args.record_only and args.record_dataset is None:
        raise SystemExit("--record-only 需要与 --record-dataset 一起使用")
    try:
        extra_candidates = [_parse_candidate_spec(value) for value in args.candidate]
    except (TypeError, ValueError) as exc:
        raise SystemExit(f"--candidate 格式错误：{exc}") from exc

    import cv2
    import numpy as np

    print("Aero Hand 单手/双手模型基准")
    print("安全范围：脚本只访问摄像头与模型，不连接串口，也不发送灵巧手动作。")
    requested_profiles = (
        ("single", "dual") if args.profile == "both" else (args.profile,)
    )
    dataset_manifest: dict[str, Any] | None = None
    if args.dataset is not None:
        try:
            clips, dataset_manifest = _load_dataset(
                np=np,
                directory=args.dataset,
                profiles=requested_profiles,
            )
        except (FileNotFoundError, KeyError, TypeError, ValueError) as exc:
            raise SystemExit(f"数据集读取失败：{exc}") from exc
        camera_info = dict(dataset_manifest.get("camera") or {})
        print(f"使用固定数据集：{args.dataset.expanduser().resolve()}")
        print(
            "数据集帧数："
            + "，".join(
                f"{profile}={len(clips[profile])}" for profile in requested_profiles
            )
        )
    else:
        print("流程：录制固定单手/双手帧，再对相同帧运行全部候选方案。")
        capture = cv2.VideoCapture(args.camera)
        if not capture.isOpened():
            capture.release()
            raise SystemExit(
                f"无法打开 Camera {args.camera}；请确认终端已获得摄像头权限"
            )
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        capture.set(cv2.CAP_PROP_FPS, args.camera_fps)
        actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        reported_fps = float(capture.get(cv2.CAP_PROP_FPS))
        camera_info = {
            "index": args.camera,
            "width": actual_width,
            "height": actual_height,
            "reported_fps": reported_fps,
            "sample_fps": args.sample_fps,
            "seconds_per_profile": args.seconds,
        }
        print(
            f"摄像头实际格式：{actual_width}x{actual_height}，"
            f"报告帧率 {reported_fps:.2f} FPS"
        )
        try:
            clips = {}
            if "single" in requested_profiles:
                clips["single"] = _record_clip(
                    cv2=cv2,
                    capture=capture,
                    label="单手",
                    instructions="画面中保留一只手；覆盖张开、握拳、逐指弯曲和手腕转动。",
                    seconds=args.seconds,
                    sample_fps=args.sample_fps,
                    interactive=not args.non_interactive,
                    preview=not args.no_preview,
                )
            if "dual" in requested_profiles:
                clips["dual"] = _record_clip(
                    cv2=cv2,
                    capture=capture,
                    label="双手",
                    instructions="两只手同时完整进入画面；分别活动手指，并做少量交叉和遮挡。",
                    seconds=args.seconds,
                    sample_fps=args.sample_fps,
                    interactive=not args.non_interactive,
                    preview=not args.no_preview,
                )
        finally:
            capture.release()
            if not args.no_preview:
                cv2.destroyAllWindows()
        if args.record_dataset is not None:
            try:
                saved_directory = _save_dataset(
                    np=np,
                    directory=args.record_dataset,
                    clips=clips,
                    camera=camera_info,
                )
            except (FileExistsError, OSError, TypeError, ValueError) as exc:
                raise SystemExit(f"数据集保存失败：{exc}") from exc
            dataset_manifest = json.loads(
                (saved_directory / "manifest.json").read_text(encoding="utf-8")
            )
            print(f"固定数据集已保存：{saved_directory}")
            if args.record_only:
                print("录制完成。后续使用 --dataset 重放这批帧。")
                return 0

    import mediapipe as mp
    from rynnrcp_robot_aero_hand.gesture_inference import (
        available_gesture_backends,
        load_gesture_backend_module,
    )

    if not hasattr(mp, "solutions"):
        raise SystemExit(
            "完整 full/lite 对比需要 mediapipe==0.10.18；"
            "请使用 setup_aero_hand.sh 创建的兼容环境或指定 Python 3.10 环境。"
        )
    load_gesture_backend_module(args.backend_module or None)
    for candidate in extra_candidates:
        load_gesture_backend_module(candidate.backend_module)
    optimized_backend = str(args.backend).strip().lower()
    if not optimized_backend:
        raise SystemExit("--backend 不能为空")
    requested_backends = [optimized_backend] + [
        candidate.backend for candidate in extra_candidates
    ]
    missing_backends = sorted(
        set(requested_backends) - set(available_gesture_backends())
    )
    if missing_backends:
        available = ", ".join(available_gesture_backends())
        raise SystemExit(
            f"后端 {', '.join(missing_backends)!r} 未注册；当前可用后端：{available}"
        )
    if args.task_model is None:
        from rynnrcp_robot_aero_hand.vision_master import DEFAULT_MODEL_PATH

        task_model = Path(DEFAULT_MODEL_PATH).expanduser().resolve()
    else:
        task_model = args.task_model.expanduser().resolve()
    if not args.baseline_only and not task_model.is_file():
        raise SystemExit(
            f"当前 Tasks 模型不存在：{task_model}\n"
            "hand_landmarker.task 不再随仓库分发；请用 --task-model 指定本地副本，"
            "或加 --baseline-only 跳过 Tasks 对比。"
        )

    optimized_candidate = Candidate(
        f"{optimized_backend}-640-t{args.threads}",
        "legacy",
        640,
        0,
        optimized_backend,
        args.backend_module or None,
        args.threads,
    )
    baseline_candidate = Candidate(
        "mediapipe-lite-640-baseline",
        "legacy",
        640,
        0,
        "mediapipe_lite",
        None,
        args.baseline_threads,
    )
    selected_candidates = (
        []
        if (
            optimized_backend == "mediapipe_lite"
            and args.threads == args.baseline_threads
            and not args.backend_module
        )
        else [optimized_candidate]
    )
    unique_candidates: list[Candidate] = []
    candidate_keys: set[tuple[str, str | None, int | None]] = set()
    for candidate in selected_candidates + extra_candidates:
        key = (candidate.backend, candidate.backend_module, candidate.threads)
        if key in candidate_keys:
            continue
        candidate_keys.add(key)
        unique_candidates.append(candidate)
    selected_candidates = unique_candidates
    candidates = (
        [baseline_candidate] + selected_candidates
        if args.baseline_only
        else [
            Candidate("task-full-640", "task", 640),
            Candidate("task-full-320", "task", 320),
            Candidate("legacy-full-640", "legacy", 640, 1),
            baseline_candidate,
            Candidate(
                "mediapipe-lite-320",
                "legacy",
                320,
                0,
                "mediapipe_lite",
                None,
                args.threads,
            ),
            Candidate(
                "mediapipe-lite-192",
                "legacy",
                192,
                0,
                "mediapipe_lite",
                None,
                args.threads,
            ),
        ]
        + selected_candidates
    )

    all_results: list[Result] = []
    for profile in requested_profiles:
        print(f"\n开始分析{'单手' if profile == 'single' else '双手'}样本...")
        profile_results: list[Result] = []
        baseline: list[Detection] | None = None
        for candidate in candidates:
            print(f"  测试 {candidate.name}")
            result, detections = _run_candidate(
                cv2=cv2,
                mp=mp,
                frames=clips[profile],
                profile=profile,
                candidate=candidate,
                task_model=task_model,
                baseline=baseline,
            )
            if baseline is None:
                baseline = detections
                result.shape_mae_vs_current = 0.0
            profile_results.append(result)
            all_results.append(result)
        _print_results(profile, profile_results)

    payload = {
        "environment": {
            "platform": platform.platform(),
            "machine": platform.machine(),
            "python": platform.python_version(),
            "mediapipe": mp.__version__,
            "opencv": cv2.__version__,
            "baseline_backend": "mediapipe_lite",
            "baseline_threads": args.baseline_threads,
            "optimized_backend": optimized_backend,
            "optimized_threads": args.threads,
            "optimized_backend_module": args.backend_module or None,
            "extra_candidates": [
                {
                    "backend": candidate.backend,
                    "module": candidate.backend_module,
                    "threads": candidate.threads,
                }
                for candidate in extra_candidates
            ],
        },
        "dataset": {
            "path": (
                str(args.dataset.expanduser().resolve())
                if args.dataset is not None
                else (
                    str(args.record_dataset.expanduser().resolve())
                    if args.record_dataset is not None
                    else None
                )
            ),
            "manifest": dataset_manifest,
        },
        "camera": camera_info,
        "results": [asdict(result) for result in all_results],
    }
    args.json.expanduser().resolve().write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"\n完整结果已保存：{args.json.expanduser().resolve()}")
    print("请把终端表格或 JSON 文件发给我，我会据此选择最终模型和参数。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
