# RynnBot 模型调试指南

模型调试 App 用于连接 HTTP 模型推理服务，查看 RCP Server 的实时
Observation，并按设定的总步数循环完成推理和 Action 执行。它适用于返回
单帧或多帧 Action 的模型。

```text
最新 Observation → 模型推理 → 执行 Action chunk → 更新 Observation → 下一轮
```

## 使用前提

本指南假定机器人环境已经完成以下准备：

- 已安装 RynnBot App，可以在机器人 Python 环境中执行
  `rynnrcp-model-debug`。
- 已完成机器人 Server 配置。SO101 从臂使用
  `robots/lerobot_so101/rynnrcp_robot_so101/config/so101_follower_server.yaml`。
- 机器人、相机和急停设备已准备就绪。

## 启动

在 RynnRCP 仓库根目录和 SO101 机器人 Python 环境中执行：

```bash
rynnrcp-model-debug \
  --server-config robots/lerobot_so101/rynnrcp_robot_so101/config/so101_follower_server.yaml
```

终端会打印页面地址：

```text
Model Debug UI: http://127.0.0.1:8093/
```

页面打开后按以下顺序操作：

1. **连接机器人**：确认 Server 已连接，机器人、关节和相机状态正常。
2. **配置推理**：点选模型需要的 Observation，填写推理服务、凭据和 Prompt。
3. **开始推理并查看结果**：设置总执行步数、目标 Action 和默认帧率，
   准备好真机急停后开始运行；在同一区域查看实时 Observation、累计进度
   和 Action 历史，需要结束时点击“停止推理”。

页面根据 Server 返回的 Observation 生成多级标签，例如
`observation` / `front` / `image`。推理请求只携带用户点选的
Observation；状态和图像预览仍展示 Server 的全部实时数据。

每轮推理都会读取最新 Observation。模型返回的 Action chunk 执行完成后，
页面继续下一轮；最后一轮只执行达到总步数所需的 frames。点击“停止推理”
时，页面停止循环并请求 Server 停止当前 Action。正在进行的云端请求返回后
保持停止状态。

Server 提供 `home` 或 `go_home` Action 时，“停止推理”右侧会显示
“回到 Home”按钮。点击并确认后，页面先停止当前推理和 Action，再执行
机器人提供的 Home Action。

## Protobuf 服务配置

RynnBot Protobuf 请求使用以下模型字段：

- `observation.state`
- `observation.images.front`
- `observation.images.wrist`
- `prompt`

页面会把用户点选的 RCP 状态和图像转换为这些字段，并把基础 URL
自动补成 `/protobuf`。

PAI-EAS 服务填写：

| 页面字段 | 填写内容 |
| --- | --- |
| 请求协议 | `Protobuf（RynnBot）` |
| 推理 URL | PAI-EAS 的完整 `/api/predict/<service>` 地址 |
| API Key | EAS Token |
| 鉴权 Header | `Authorization` |
| Key 前缀 | 留空 |

其他 Protobuf 服务填写：

| 页面字段 | 填写内容 |
| --- | --- |
| 请求协议 | `Protobuf（RynnBot）` |
| 推理 URL | 服务提供方给出的 HTTPS 推理地址和端口 |
| API Key | 按服务要求填写 |
| 鉴权 Header | 按服务要求填写 |
| Key 前缀 | 按服务要求填写 |

SO101 推理输入选择六维关节状态，以及模型需要的 `front`、`wrist`
图像。页面使用 RynnBot 内置 Protobuf 编解码：状态使用 float32 原始
数据，图像使用 JPEG。

## 推理请求格式

JSON 协议使用 HTTP POST 发送 UTF-8 JSON：

```json
{
  "prompt": "Pick up the red block",
  "observations": {
    "observation.robot.joint_state": {
      "joint_positions": [0.0, -1.2, 1.4, 0.6, -0.3, 0.0]
    },
    "observation.front.image": {
      "width": 640,
      "height": 360,
      "encoding": "jpg",
      "image": {
        "encoding": "base64",
        "data": "<base64 image>"
      }
    }
  }
}
```

二进制 Observation 会统一转换成：

```json
{
  "encoding": "base64",
  "data": "<base64 data>"
}
```

API Key 为空时不发送鉴权 Header。需要 Bearer 鉴权时，在页面填写：

```text
Authorization: Bearer <API Key>
```

推理服务使用其他 Header 时，在页面修改“鉴权 Header”和“Key 前缀”。

## 推理响应格式

推荐直接返回 RCP Action chunk：

```json
{
  "name": "action.robot.joint_position",
  "frame_rate": 30,
  "frames": [
    {"joint_positions": [0.01, -1.19, 1.39, 0.6, -0.3, 0.0]},
    {"joint_positions": [0.02, -1.18, 1.38, 0.6, -0.3, 0.0]}
  ]
}
```

也可以只返回二维 Action 数组：

```json
{
  "action": [
    [0.01, -1.19, 1.39, 0.6, -0.3, 0.0],
    [0.02, -1.18, 1.38, 0.6, -0.3, 0.0]
  ]
}
```

App 会把二维数组转换为 `joint_positions` frames，并使用页面选择的 Action 名和帧率。

## 页面功能

- Server 配置：使用指定配置启动和停止 RynnRCP Server。
- Observation 状态：每个状态 Observation 使用一张独立图表，以 10 Hz
  读取并绘制最近 120 个采样点的数值曲线。
- 实时图像：通过独立二进制接口连续刷新，避免在页面预览链路中使用
  Base64 JSON；实际帧率取决于相机和 Server 响应速度。
- 推理配置：填写 URL、Key、鉴权方式、Prompt、总执行步数和超时。
- 运行控制：开始后自动循环推理和执行，并可随时停止。
- Home：根据 Server Manifest 自动显示回到 Home 按钮。
- Action 历史：每轮 Action 追加到曲线，滚动显示最近 120 step；原始数据
  区显示最新一轮 Action。

## 查看报错

运行失败时，页面会显示失败轮次、处理阶段、累计 step 和底层原因。例如：

```text
第 2 轮推理失败（已执行 20/100 step）：
请求模型服务失败：POST https://example.test/protobuf 返回 HTTP 401：Unauthorized
```

模型请求错误会包含目标 URL、HTTP 状态或连接/超时原因；Protobuf 错误会
区分 Observation 编码和 Action 解码；Action 错误会显示目标 Action、当前
chunk step 数和 Server 返回原因。API Key 保持在请求 Header 中，不写入报错。

API Key 仅用于当前页面发起请求。关闭页面前先停止 Server；终止 App
进程时也会停止它启动的 Server。
