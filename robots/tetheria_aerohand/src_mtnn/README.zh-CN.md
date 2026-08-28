# aero_hand_mtnn：MTNN 零拷贝手势管线（C++ / pybind11）

后端名 `mediapipe_lite_mtnn_zero`。用 C++ 复刻 MediaPipe `hand_landmark_tracking` 级联，
推理走 **Moore Threads M1000 NPU** 的 MTNN 零拷贝接口，pybind11 暴露给 Python。
目标平台：E300 / M1000（INT8 50TOPS）。

```
外部帧(HxW, RGB/BGR)
  ├─ palm：letterbox 192 + ÷255 归一化 + HWC→NCHW 直写 palm 零拷贝输入
  │        → mtnn_inference → SSD 2016 锚点解码 + weighted-NMS → 旋转方形 ROI
  ├─ hand：仿射裁剪 224（任意角）+ ÷255 → 直写 hand 零拷贝输入
  │        → mtnn_inference → 关键点逆仿射投回原图
  └─ 跟踪：下一帧 ROI 来自上帧关键点；presence<0.5 才重跑 palm（同官方图）
```

## 模型

模型来源链：MediaPipe 官方 `*_full.tflite` → ONNX（palm 用 tf2onnx、hand 用 tflite2onnx）
→ **MTC int16 量化**（`quant_type: int16` + `quantizer: dynamic_fixed_point` + 真实手部图
校准，归一化 ÷255）。int16 相对 MediaPipe 原生**无精度损失**（端到端 91.0% / 85.5%，
与 TFLite 基准完全一致）。模型准备见 `../scripts/prepare_mtnn_models.sh`。

| 文件 | 输入 | 输出 |
|---|---|---|
| `models/palm_detection.mtnn` | [1,3,192,192] float32 (÷255) | reg[1,2016,18] + scores[1,2016,1] |
| `models/hand_landmark.mtnn` | [1,3,224,224] float32 (÷255) | lm[1,63] + score[1,1] + handed[1,1] + world[1,63] |

预处理约定（与 MediaPipe 官方一致）：
- palm 阈值 0.5、NMS IoU 0.3、ROI 由 wrist→middle 转正 ×2.6 shift −0.5
- hand 224 裁剪，landmark 输出为裁剪图像素坐标，presence 阈值 0.5

## 构建

```bash
cd robots/tetheria_aerohand/src_mtnn
./build.sh            # 默认用 active venv / python3（需已 pip install pybind11 cmake）
```

产物输出到 `../rynnrcp_robot_aero_hand/accelerators/`：
- `aero_hand_mtnn.cpython-310-aarch64-linux-gnu.so`（pybind 模块，RPATH=$ORIGIN）

依赖：系统 `libmtnnrt.so`（`/usr/lib`）、NPU 头文件（`/usr/include/npu/include`）。
这些由 Moore Threads SDK / 设备镜像提供。预处理（letterbox/仿射裁剪）为纯 C++ 实现，
不依赖 OpenCV。

## 使用

```python
from rynnrcp_robot_aero_hand.accelerators import aero_hand_mtnn
p = aero_hand_mtnn.AeroHandPipeline(palm_mtnn, hand_mtnn, num_hands=1)
hands = p.process(frame_hwc_uint8, format)   # format: 0=RGB888, 1=BGR888
p.timings()        # 各阶段累计耗时
p.reset_tracking() # 清跟踪 ROI
p.release()
```

## 性能（E300 / M1000，int16）

| 场景 | avg | p95 |
|---|---|---|
| 丢跟踪帧（palm+hand） | ~7.1 ms | ~7.5 ms |
| 稳态跟踪帧（只 hand） | ~4.2 ms | ~4.6 ms |

稳态主路径 ~4.2ms，轻松满足 30Hz 实时手势跟踪。
