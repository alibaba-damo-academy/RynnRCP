# Aero Hand RKNN 加速部署与验证

## 默认方案

Aero Hand 使用同一套 `auto` 后端选择逻辑：

- Rockchip Linux：优先使用 C++ `rga_rknn_zero_fp16` 和 Full FP16 模型；C++
  模块不可用时回退到 Python `mediapipe_lite_rknn_fp16` 和 Lite FP16 模型。
  两条路径的输入均保持相机原始 640×480。
- Mac、x86 和其他 ARM：`mediapipe_lite`，不安装 RKNN，也不改变现有 Lite 行为。
- RKNN 初始化失败：记录一次原因并回退到 `mediapipe_lite`。

configure 和真实 master server 都走这套逻辑，不需要维护两份实现。YAML 继续写可移植基线：

```yaml
inference_backend: mediapipe_lite
inference_backend_module: ''
inference_threads: 1
prefer_platform_acceleration: true
```

RK 平台会自动把上述配置解析为双 FP16 后端；其他平台仍按 YAML 运行。

## 目录说明

| 路径 | 内容 |
|---|---|
| `accelerators/rknn_models.py` | 按后端和当前 Rockchip SoC 选择 FP16 模型 |
| `accelerators/mediapipe_lite_rknn.py` | Python Lite FP16 RKNN 后端 |
| `accelerators/rga_rknn_zero.py` | C++ Full FP16 后端的 Python 包装 |
| `accelerators/_native/` | setup 下载或编译的 `.so` 产物，不与 Python 源码混放 |
| `models/palm_detection_full_fp16.rknn` | RK3566 C++ Full FP16 掌检测模型 |
| `models/hand_landmark_full_fp16.rknn` | RK3566 C++ Full FP16 手部关键点模型 |
| `models/palm_detection_lite_fp16.rknn` | RK3566 Python Lite FP16 掌检测模型 |
| `models/hand_landmark_lite_fp16.rknn` | RK3566 Python Lite FP16 手部关键点模型 |
| `src/` | RGA + RKNN C++ 后端源码 |

当前无后缀模型面向 RK3566。其他 SoC 使用带平台后缀的模型；C++ 后端需要 Full
模型，Python 后端需要 Lite 模型。例如 RK3588 的 C++ 模型命名为：

```text
palm_detection_full_fp16_rk3588.rknn
hand_landmark_full_fp16_rk3588.rknn
```

## RK 设备安装

在 `robots/tetheria_aerohand` 执行：

```bash
bash setup_aero_hand.sh
source venv/bin/activate
```

首次 setup 会：

1. 从清华 PyPI 镜像下载 RKNN Lite 2.3.2 Python wheel，并按 PyPI 公布值校验。
2. 下载并校验 `librknnrt.so` 2.3.2。
3. 如果系统 `/usr/lib/librknnrt.so` 低于 2.3.0，则请求一次 `sudo` 并安装兼容版本。
4. 如果系统已有 RGA 开发文件，再编译可选 C++ 后端；缺少 RGA 不影响默认双 FP16 后端。

完成后不需要手工 bind mount 或 `unshare`。

离线安装可指定本地文件：

```bash
AERO_HAND_RKNN_WHEEL_PATH=/path/to/rknn_toolkit_lite2-2.3.2-cp310-cp310-manylinux_2_17_aarch64.manylinux2014_aarch64.whl \
AERO_HAND_RKNNRT_PATH=/path/to/librknnrt.so \
bash setup_aero_hand.sh
```

网络受限时也可分别设置 `AERO_HAND_RKNN_WHEEL_URL` 和
`AERO_HAND_RKNNRT_URL`。无论使用官方地址、镜像还是本地文件，setup 都会校验固定
SHA-256。

跳过所有 RKNN 安装：

```bash
AERO_HAND_SKIP_NPU=1 bash setup_aero_hand.sh
```

## 验证默认后端

```bash
python - <<'PY'
import mediapipe as mp
from rynnrcp_robot_aero_hand.gesture_inference import (
    _is_rockchip_platform,
    create_gesture_backend,
)

print("Rockchip:", _is_rockchip_platform())
backend = create_gesture_backend(
    "auto",
    mediapipe=mp,
    max_num_hands=1,
    num_threads=1,
)
print("Backend:", backend.name)
backend.close()
PY
```

具备已编译 C++ 模块的 RK3566 应输出：

```text
Rockchip: True
Backend: rga_rknn_zero_fp16
```

C++ 模块不可用时输出 `mediapipe_lite_rknn_fp16`。当前 SoC 缺少专用模型时，错误会
列出预期文件名和 RKNN Toolkit2 的 `target_platform`。

然后分别测试 configure、单手 master server 和双手 master server：

```bash
rynnrcp-aero-hand-configure
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_hand_master_server.yaml
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_hand_master_server.yaml
```

## 显式选择 FP16 后端

仅在定位 C++ 与 Python 路径差异时显式覆盖：

```bash
# Python RKNN 双 FP16
AERO_HAND_INFERENCE_BACKEND=mediapipe_lite_rknn_fp16 \
AERO_HAND_INFERENCE_BACKEND_MODULE=rynnrcp_robot_aero_hand.accelerators.mediapipe_lite_rknn \
rynnrcp-aero-hand-configure

# C++ RGA + RKNN 双 FP16
AERO_HAND_INFERENCE_BACKEND=rga_rknn_zero_fp16 \
AERO_HAND_INFERENCE_BACKEND_MODULE=rynnrcp_robot_aero_hand.accelerators.rga_rknn_zero \
rynnrcp-aero-hand-configure
```

测试时使用同一段单手、双手视频或完全相同的动作，记录识别 FPS、平均/P95 推理耗时、
CPU、检出率、左右手一致率和关键点抖动。默认双 FP16 只有在真实动作的稳定性满足要求后
才算验收通过，单看空跑吞吐不能代替精度验收。

## 已知边界

- 当前官方 RKNN Lite wheel 固定为 Python 3.10、Linux aarch64。
- 双 FP16 后端固定接收 640×480，不使用 YAML 中面向 CPU Lite 的
  `inference_width` 降采样。
- 可选 C++ 后端的构建和内部实现见 [`src/README.zh-CN.md`](./src/README.zh-CN.md)。
