# RynnBot App

RynnBot App 用于连接 RynnBot 云端平台。它负责云端协议和 RCP 协议之间的边界转换。

它不直接连接机器人硬件。机器人能力来自已经启动的 RCP Server；云端设备凭据、鉴权、
MQTT 和 OSS 等平台配置来自 RynnBot App 配置文件。

## 启动

```bash
rynnrcp-rynnbot-app --config <rynnbot-app.yaml> --server-config <server-config.yaml>
```

SO101 示例：

```bash
rynnrcp-rynnbot-app \
  --config rynnrcp_robot_so101/config/so101_rynnbot_app.yaml \
  --server-config rynnrcp_robot_so101/config/so101_follower_server.yaml
```

运行 RynnBot App 前，请先启动对应 RCP Server。

- `--config`：RynnBot App 配置，包含云端地址、设备凭据和平台相关参数。
- `--server-config`：RCP Server 配置，用来读取 `manifest.robot_id` 并连接目标 Server。

可以用 `-h` 查看命令参数：

```bash
rynnrcp-rynnbot-app -h
```

## 职责

- 向云端上报设备属性。
- 接收云端下发的动作、状态请求、图像请求和技能执行请求。
- 将云端使用的 RynnBot key 映射为 RCP key，再调用 Server。
- 将端侧 RCP 返回值映射回云端需要的格式。
- 处理云端数据采集、技能回流、OSS 上传等平台相关流程。

## 边界

App 与 Server 之间使用 RCP 协议 key；App 与云端之间使用 RynnBot 平台 key。
图像 key 映射应按协议对象名动态生成，不写死相机数量。

RynnBot App 只在云端边界做格式转换：

- 云端下发 RynnBot key，App 转成 RCP key 后调用 Server。
- Server 返回 RCP key，App 转回 RynnBot key 后上报云端。
- Server 端采集的是 RCP 原始数据；需要上传云端时，App 负责编码、打包和上传。

## 配置文件

SO101 的 RynnBot App 配置示例位于：

```text
robots/so101/rynnrcp_robot_so101/config/so101_rynnbot_app.yaml
```

常见字段包括云端地址、设备凭据、MQTT 连接参数、设备属性上报间隔、最大采集时长、
视频编码配置和上传后清理策略。字段说明见：
[`../../docs/RCP配置文件说明.md`](../../docs/RCP配置文件说明.md)。
