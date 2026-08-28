# aero_hand_rga：RGA + RKNN 零拷贝手势管线（C++ / pybind11）

后端名 `rga_rknn_zero`。用 C++ 复刻 MediaPipe `hand_landmark_tracking` 级联，
图像处理走 **RGA 2D 硬件**、推理输入走 **RKNN 零拷贝固定地址**，pybind11 暴露给 Python。

```
外部帧指针(640x480, RGB/BGR)
  ├─ ptr→RGA handle 缓存（importbuffer_virtualaddr 仅首次，命中零开销）
  │        被 MMU 拒绝时退 DMA32 暂存（一次 memcpy），再不行退 CPU letterbox
  ├─ palm：RGA improcess 一次完成 缩放+补边(letterbox 192)+色转 → 直写 palm 零拷贝输入
  │        → rknn_run → SSD 2016 锚点解码 + weighted-NMS → 旋转方形 ROI
  ├─ hand：CPU 双线性仿射（任意角，RGA 只支持 90° 倍数）→ 直写 hand 零拷贝输入
  │        → rknn_run → 关键点逆仿射投回原图
  └─ 跟踪：下一帧 ROI 来自上帧关键点；presence<0.5 才重跑 palm（同官方图）
```

## 构建

```bash
cd robots/tetheria_aerohand/src
./build.sh            # 默认用仓库 venv 的 python（需已 pip install pybind11 cmake）
```

产物输出到 `../rynnrcp_robot_aero_hand/accelerators/`：
- `aero_hand_rga.cpython-310-aarch64-linux-gnu.so`（pybind 模块，RPATH=$ORIGIN）
- `librknnrt.so`（构建时找到的 ≥2.3.x 副本，随模块通过 $ORIGIN 解析——**不需要替换系统库，也不需要 bind-mount**）

依赖：系统 librga（`/usr/lib/aarch64-linux-gnu/librga.so.2` + `/usr/include/rga`，
Orange Pi 镜像自带）；`src/include/rknn_api.h`（v2.3.2）；librknnrt.so ≥ 2.3.x
（不入 git；查找顺序：仓库根 `models/rknn_runtime/`（模型仓库分发）→
`src/lib/` → `/usr/lib/`，见 CMakeLists.txt）。

## 使用

```python
from rynnrcp_robot_aero_hand.accelerators import aero_hand_rga
p = aero_hand_rga.AeroHandPipeline(palm_rknn, hand_rknn, num_hands=1)
hands = p.process(frame_hwc_uint8, format)   # format: 0=RGB888, 1=BGR888
hands = p.process_ptr(addr, w, h, stride_bytes, format)  # 上游内存池直传指针
p.stats()          # ptr 缓存 imports/hits/entries
p.clear_cache()    # 上游内存池重建时必须调用（防悬垂指针）
```

作为手势后端（benchmark / configure / server 通用；生产配置，默认加载包内
full fp16 模型，可用 `AERO_HAND_RKNN_PALM/_HAND` 覆盖）：

```yaml
inference_backend: rga_rknn_zero
inference_backend_module: rynnrcp_robot_aero_hand.accelerators.rga_rknn_zero
inference_threads: 2   # clone 流水线：full fp16 49.5->85.9fps，结果滞后 1 帧
```

## 设计要点

1. **零拷贝**：palm/hand 输入用 `rknn_create_mem` + `rknn_set_io_mem` 绑定固定地址；
   palm 输入的 fd 在初始化时 `importbuffer_fd` 一次包成 RGA 目的 handle；输出同样
   固定地址（float32），推理后直接读，无 `rknn_outputs_get` 拷贝。
2. **ptr→handle 缓存 + DMA32 暂存兜底**：上游（VideoCapture 内存池）地址可数；
   `importbuffer_virtualaddr` 只在首见地址时执行，之后全命中。import 失败有两种原因：
   - **驱动并发 import 配额**（RK3566/librga1.9.3 实测 ≈50 个 900KB 缓冲），故 FIFO
     上限设 48，失败时逐出最旧条目重试；
   - **RGA2 只能寻址低 4GB**（`dmesg` 报 `RGA_MMU unsupported memory larger than 4G!`）。
     Linux 从高位给用户匿名内存分页，所以 >4GB 板子上采集缓冲会被**永久拒绝**；改用
     `wrapbuffer_virtualaddr` 也无效——`RGA_BLIT` 会以 `Invalid argument` 拒绝同一批页
     （已实测）。此类地址记入拒绝表，不再每帧重试那个必然失败的 ioctl
     （否则每帧白付 ≈0.37ms）。

   拒绝后并不直接放弃硬件，而是走**三级候选链**：直接 import（零拷贝，最优）→
   **DMA32 暂存缓冲**→ CPU letterbox。暂存缓冲从 `/dev/dma_heap/system-dma32`
   分配 dma-buf，其页必然落在 4GB 以下，因而 `importbuffer_fd` 恒可映射；代价是一次
   memcpy。实测 memcpy 0.38ms + 硬件 blit 0.76ms = 1.33ms，优于 CPU letterbox 的
   2.93ms，故 palm letterbox 启用暂存；hand 分块路径**不**启用——该路径 RGA 实测并不快于
   CPU warp（2.489 vs 2.510ms），多一次拷贝反而净亏，只在 palm 已把该帧暂存好时顺带复用。
   堆不存在或映射失败时暂存路径**永久自禁**，静默退到 CPU。
   可观测计数：`stats().refusals` / `.staged` / `.fallbacks`（均不丢帧）。
   实测预处理占比小、NPU 是瓶颈，整链收益约 0.7ms/帧（23.4 vs 24.1ms）。
3. **缓存一致性教训（重要，已踩两次）**：RGA/NPU 与 CPU **不缓存一致**，凡"CPU 写、设备读"
   或反向的缓冲都必须显式做 cache 维护。
   - 绑定给 rknn 的 DMA 输入缓冲**不能用 CPU 写后再由 RGA 写**——CPU 脏 cache 行会在
     `rknn_run` 的 cache 同步时回写覆盖 RGA 结果（表现为首帧检不出）。补边清零必须用
     设备侧 `imfill`。hand 输入是纯 CPU 写路径，则无此问题。
   - DMA32 暂存缓冲是"CPU memcpy 写、RGA 读"，且 handle **只 import 一次**长期复用，
     librga 不会每帧替你做维护，因此 memcpy 必须用 `DMA_BUF_IOCTL_SYNC`
     （`SYNC_START|WRITE` … `SYNC_END|WRITE`）括起来，把 cache 维护交给 exporter。
     漏掉时的表现极隐蔽：**只有首帧**读到部分陈旧数据，关键点与稳态差 0.0154（归一化），
     之后逐帧逐位一致——而原始 CPU 路径是逐位确定的，正是这个 A/B 对照把它暴露出来。
     代价约 0.05ms，可忽略。
4. **RGA 限制与两段式裁剪**：RGA 仅支持 90° 倍数旋转，hand 任意角旋转采样在 CPU。
   直接在 640×480 大图上旋转采样是**访存受限**的（对角线扫描全打 DRAM，标量 5.8ms、
   定点 2.5ms 到顶）；改为**两段式**：RGA 先把 ROI 的 AABB 预缩放进 256² 小 tile
   （硬件完成 DRAM 聚集+色转，0.6ms），CPU 再在 cache 常驻的 tile 内做 NEON 旋转
   采样（1.6ms）——合计 2.2ms，CPU 占用减半。
5. **palm 输出 int8 直采 + 贪心反量化**（同官方 YOLO demo 后处理套路）：输出绑定
   为**标准布局 int8**（不请求 float），阈值在 int8 域完成——score>0.5 ≡ logit>0 ≡
   `q > zp`（scale>0）——仅对过阈锚点（通常 <20 个）反量化 18 个回归值；输出写量
   145KB→36KB。注意：该模型的 **NATIVE 输出是 NC1HWC2 打包**，不可直接线性读；
   实测 `palm.npu_run` 内残留的 ≈1.5ms CPU 是 runtime 的 **NC1HWC2→线性布局重排**
   （int8/float 两种输出都需要），而非反量化本身；彻底消除需自行解 NC1HWC2，
   收益有限未做。只影响丢跟踪的重检帧。

## 上游接入（BGR 直传 + letterbox 规整，重要）

`vision_master` 对声明 `accepts_bgr=True` 且实现 `process_bgr()` 的后端（即本后端）
自动切换到硬件预处理路径，与默认路径的对比：

| | 默认后端 | accepts_bgr 后端（本方案） |
|---|---|---|
| 色转 | CPU `cvtColor(BGR→RGB)`，每帧 ≈2ms | **不做**，BGR 直传，RGA 在 letterbox/tile 时顺带完成 |
| 尺寸规整 | 仅宽>640 时等比缩宽（16:9 相机→640×360） | **letterbox 到固定 640×480**（`letterbox_frame`，等比+居中补零） |
| 后端入口 | `process(rgb)` | `process_bgr(bgr)` |

**为什么必须 letterbox 到 640×480 而不是简单 resize**：
1. **RGA 对齐要求**：RGB888 的 wstride 需满足对齐（640 满足；而 16:9 等比缩出的
   640×360、或任意相机尺寸的行宽不可控）；固定 640×480 保证几何/对齐永远合法。
2. **ptr→handle 缓存稳定**：RGA import 时按 size 包裹；尺寸恒定才能保证缓存命中与
   handle 复用（否则换相机/换分辨率就是新尺寸、新 import）。
3. **不引入形变**：letterbox 等比+补零，不改变手的长宽比（简单拉伸 resize 会损害
   palm 精度：实测拉伸与 letterbox 两种喂法在同一帧上的检测框 IoU 仅 ~0.63、
   框中心差 ~22px，即预处理几何会实质改变 palm 输出）。

**行为注意（非 4:3 相机）**：letterbox 后后端输出的归一化坐标是相对 640×480 画布
（含补边）的；预览叠加绘制用的是原始 flip 帧，两坐标系在非 4:3 相机下会有补边
偏移（world 坐标/关节重定向不受影响）。部署相机为 640×480 时两路径完全等价、
零开销（直接透传）。验证：640×480 下 BGR 直传与 RGB 路径输出**比特级一致**；
1280×720 模拟相机经 letterbox 后检出正常。回退：切回默认后端即自动恢复旧路径。

## 多线程吞吐池（num_threads，默认 1）

后端顶层内置工作池（`rga_rknn_zero.py`），用 benchmark/YAML 的 `threads` 直接控制：
- **threads=1（生产默认）**：同步透传，行为与单实例完全一致。
- **threads=N>1**：N 个流水线实例轮转接帧；实例由 C++ `clone()` 创建，
  基于 **`rknn_dup_context` 共享模型权重**（实测克隆仅 +2.3MB RSS，输出与主
  实例比特级一致），io 内存/RGA handle/跟踪状态各自独立。一个实例等 NPU 时
  另一个跑 CPU 段，用重叠换吞吐；**代价是结果滞后 N−1 帧**（流水线延迟），
  且场景切换时两个实例各自重检（连续视频中影响很小）。

稳态吞吐实测（同帧重复，含 Python 胶水层）。**下表为 int8 模型时代的数据**，
int8 变体已在 2026-07-28 的模型矩阵评测后移除，
当前默认为 full fp16，实测见后文：

| threads | 吞吐（int8，历史） | CPU | 备注 |
|---|---|---|---|
| 1 | 112 fps (8.9ms/帧) | 43.8% | 无滞后 |
| 2 | **181.7 fps (5.5ms/帧)** | 93.3% | 滞后1帧；已压到 **NPU 串行极限**（hand 4.4+tile 0.6≈5ms），再加线程无收益 |

## 实测（单手 120 帧 640×480，RK3566）

`rga_rknn_zero` / `mediapipe_lite_rknn` 两列为 **int8 时代**数据；当前 full fp16
默认值见下一节。CPU 基线一列与 fp16 复测一致，可直接对照。

| 后端 | 平均/P95 延迟 | CPU |
|---|---|---|
| mediapipe_lite（CPU 基线 t1） | 125.1 / 289.8 ms | 102.0% |
| mediapipe_lite t4 | 51.8 / 111.9 ms | 333.3% |
| mediapipe_lite_rknn（Python 级联+NPU，int8） | 16.2 / 31.5 ms | 68.8% |
| **rga_rknn_zero t1（int8）** | **10.8 / 20.5 ms** | **46.9%** |
| **rga_rknn_zero t2（流水线，int8）** | **7.7 ms**（吞吐间隔） | 79.1% |

**稳态跟踪性能（int8，真实部署主路径）：149.5 fps，CPU 36.4%（2.44ms/帧）**。
逐阶段（wall/CPU，`timings()` 实测）：hand NPU 4.35/0.43、RGA tile 0.62/0.29、
tile 内 NEON 采样 1.56/1.56、pybind 边界 0.13；重检帧另加 palm（letterbox 0.93 +
NPU 10.5，内含 runtime 输出 float 转换 ≈1.6ms CPU）。
对比 Python 版同阶段：rknnlite 调用每次多 2～3ms（胶水+输入输出拷贝）、cv2
letterbox 2.37ms vs RGA 0.93ms、numpy 解码 2.8ms vs C++ 0.02ms。

### 当前默认（full fp16）实测

RK3566 / librknnrt 2.3.2 / 驱动 0.9.6，NPU devfreq 停在 600MHz（上限 900MHz）。
**fp16 推理本身比 int8 慢 2.5～4 倍**，这是上面两张 int8 表与本节的差距来源。

| 后端 | 平均延迟 | 吞吐 | CPU |
|---|---|---|---|
| mediapipe_lite（CPU 基线 t1） | 120.8 / 314.8 ms | 8.3 fps | 102.1% |
| mediapipe_lite_rknn | 26.1 ms | 38.3 fps | 59.8% |
| **rga_rknn_zero t1** | **18.9 / 19.6 ms** | **52.9 fps** | **45.3%** |
| **rga_rknn_zero t2（流水线）** | **10.8 ms**（吞吐间隔） | 92.8 fps | 98.1% |

逐阶段（wall，full fp16）：hand NPU 16.3、palm NPU 25.8（仅重检帧）、
RGA letterbox 1.03（CPU 回退 2.92）、RGA tile 0.79、hand 仿射裁剪 2.5。
NPU 段已占单帧 85% 以上，预处理优化的上限因此很有限。

精度验收（vs mediapipe_lite，与固化方案同阈值，两段式裁剪后复测）：
稳态均差 8.34px、p95 19.05px、handedness 100%、检出率 82%≈基线 83% —— **PASS**。
vs Python 孪生后端（同模型同逻辑，纯预处理差异）：稳态均差 **2.28px**（p95 5.55）、
world MAE 0.0028、handedness 100% —— RGA 与 cv2 预处理等效。

稳定性与性能分解（120s 连续压测，评测脚本已随交付清理，结论如下）：
- 延迟零漂移，RSS 封顶，fd 无泄漏；init/release ×10 无泄漏；300 个不同地址冲击下
  entries 封顶 48、帧帧连续（fallback 兜底）。
- 稳态跟踪帧（仅 hand）≈ 8ms（≈125fps，CPU 49.5%）；重检测帧（palm+hand）≈ 19ms。
- BGR(fmt=1) 与 RGB(fmt=0) 输出比特级一致（RGA 色转正确）；process 与 process_ptr 一致。

验收方式：用项目自带的 `benchmark_gesture_inference.py`（录制数据集后回放，含手型
MAE/检出率/延迟/CPU）+ `rynnrcp-aero-hand-configure` 真机联调；验收命令见交付
文档 §7。本后端自带 librknnrt，无需 bind-mount。

## 已知限制

- `clear_cache()` 语义靠调用方保证：上游释放/重建帧缓冲池后必须调用，否则 RGA 持有
  悬垂映射。
- RGA 驱动 import 配额有限（同机其它进程占用 RGA 也会挤占）；已用逐出重试+CPU
  兜底保连续性，监控 `stats().fallbacks` 即可发现配额压力。
- 输入需 3 通道紧凑行（stride=3×W 字节对齐 RGA 要求；640/192/224 均满足）。
- BGR 直传路径（`format=1`，RGA 顺带色转）已实现；生产接入需在 vision_master 侧跳过
  cvtColor（后端已声明 `accepts_bgr=True`），当前 RGB 契约下不影响使用。
