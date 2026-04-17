# SO101接入乐云平台最小接入示例
本目录提供 SO101 机械臂接入 RynnRCP 的最小可运行工程，包括：

- 配置文件：
  - config/so101_config.yaml：RcpCore 配置（Action/Sensor/DeviceMonitor 的 inputs/outputs）
  - config/rynnbot_config.yaml：乐云平台设备信息（三元组/URL）
  - config/mcp_config.yaml：MCP server 配置
- 端侧启动示例：
  - run_rcp_so101.py：启动 RcpCore + RynnBot + McpPlugin
    - RynnBot：连接乐云平台（MQTT/WS）
    - McpPlugin：启动 MCP HTTP 服务，对外暴露 tools 调用入口
- 配置向导脚本：
  - configure_so101.py：交互式配置（支持中文/英文），可自动完成相机配置、从臂/主臂串口配置、从臂/主臂标定及乐云平台三元组填写
  - configure_so101_web.py：Web 配置工具，提供图形化界面，支持相机实时预览、串口自动检测、标定实时日志流等功能

## 目录结构

```text
robots/so101/
├── config/
│   ├── rynnbot_config.yaml         # 乐云设备三元组/URL
│   ├── so101_config.yaml           # RcpCore 配置：Action/Sensor/DeviceMonitor（从臂）
│   ├── so101_leader_config.yaml    # RcpCore 配置（主臂，遥操用）
│   ├── teleop_config.yaml          # TeleopPlugin 网络配置（IP/端口/频率）
│   └── mcp_config.yaml             # MCP Server 配置
├── configure_so101.py              # 交互式配置脚本（中/英文）
├── configure_so101_web.py          # Web 配置工具（图形化界面）
├── configure_so101_web_usage.md    # Web 配置工具使用说明
├── run_rcp_so101.py                # 启动示例：RcpCore + RynnBot(plugin)
├── run_teleop_leader.py            # 启动主臂 + Web UI
├── run_teleop_follower.py          # 启动从臂
├── README.md                       # 使用文档（英文）
└── README_cn.md                    # 使用文档（中文）
```


## 前置条件

1. 已完成项目安装（Python 环境就绪，依赖已安装，Python环境已激活）
2. 已连接 SO101 机械臂控制器（串口方式）
3. 已连接相机（USB 摄像头，建议两路：front/wrist）
4. 具备乐云平台设备信息三元组：
   - product_key
   - device_name
   - device_secret
   - http_url（平台接入 URL，维持默认值即可）
5. 如需启用 MCP，确认端口可访问（示例 8000）

## 通过环境变量配置
本示例支持通过环境变量为插件提供配置。当检测到任意相关环境变量被设置时，插件将优先使用环境变量。

### RynnBot 环境变量（连接乐云平台）
必填（三元组）：
- `RYNNBOT_PRODUCT_KEY`
- `RYNNBOT_DEVICE_NAME`
- `RYNNBOT_DEVICE_SECRET`

可选：
- `RYNNBOT_HTTP_URL`（不设置则默认 `https://robot-access.damo-academy.com`）

示例：

```bash
export RYNNBOT_PRODUCT_KEY="put_product_key_here"
export RYNNBOT_DEVICE_NAME="put_device_name_here"
export RYNNBOT_DEVICE_SECRET="put_device_secret_here"
# 可选
export RYNNBOT_HTTP_URL="https://robot-access.damo-academy.com"
```

### MCP 环境变量（启动 MCP 服务）

必填：
- `MCP_SERVER_NAME`
- `MCP_HOST`
- `MCP_PORT`

可选：
- `MCP_PATH（默认 /mcp）`
- `MCP_TRANSPORT（默认 streamable-http）`

示例：
```bash
export MCP_SERVER_NAME="rcp_server"
export MCP_HOST="0.0.0.0"
export MCP_PORT="8000"
# 可选
export MCP_PATH="/mcp"
export MCP_TRANSPORT="streamable-http"
```
注意：当你使用环境变量配置时，可以不提供对应的 `rynnbot_config.yaml` / `mcp_config.yaml`；当设置了环境变量时，将优先使用环境变量。


## 快速开始
进入目录并运行配置脚本：
```bash
cd RynnRCP/robots/so101
python configure_so101.py
```
脚本启动后会先让你选择语言（中文/英文），然后进入菜单：
1. 设备设置：填写乐云平台信息，保存到 `config/rynnbot_config.yaml`
2. 相机设置：通过插拔检测相机，写入 `config/so101_config.yaml`
3. 从臂串口设置：通过插拔检测从臂串口，修改串口权限，写入 `rcp_motion/robots/so101/configs/so101.yaml` → `robot.port`
4. 从臂标定：执行从臂标定程序，输出写入 `~/.cache/huggingface/lerobot/calibration/robots/so101_follower`
5. 一键配置全部：按 1→2→3→4→6→7 顺序执行
6. 主臂串口设置：通过插拔检测主臂串口，修改串口权限，写入 `rcp_motion/robots/so101/configs/so101.yaml` → `teleoperate.port`
7. 主臂标定：执行主臂标定程序，输出写入 `~/.cache/huggingface/lerobot/calibration/teleoperators/so101_leader`

> 标定方法参考：[SO101 标定方法](../../docs/so101_calibrate.md)


## 启动端侧服务

完成配置后，在本目录启动：

```bash
python run_rcp_so101.py
```
该脚本会读取：
- ./config/so101_config.yaml
- ./config/rynnbot_config.yaml
- ./config/mcp_config.yaml

并启动端侧服务节点，连接乐云平台（RynnBot）, 同时启动 MCP 服务（McpPlugin）。


## 遥操控制（主臂 / 从臂）

SO101 支持通过 `TeleopPlugin` 在主臂（操作员）和从臂（机器人）之间进行双边遥操控制，通过局域网 UDP 通信。

### 连接机制

双方每秒互相发送心跳包（1 Hz）。  
**只有双方都运行后**，在 5 秒内检测到心跳，数据交换才会开始——如果只有一方运行，不会发送任何业务数据。

### 配置文件：`config/teleop_config.yaml`

```yaml
teleop:
  leader_host: 127.0.0.1    # 运行主臂的机器 IP
  follower_host: 127.0.0.1  # 运行从臂的机器 IP

  leader_port: 9101
  follower_port: 9102

  control_hz: 30            # 关节状态发送频率（Hz）
  image_hz: 10              # 摄像头图像发送频率（Hz）
```

> 若主臂和从臂部署在不同机器上，将 `127.0.0.1` 替换为对应的局域网 IP。

### 启动主臂（含 Web UI）

```bash
cd RynnRCP/robots/so101
python run_teleop_leader.py
```

读取 `config/teleop_config.yaml` 和 `config/so101_leader_config.yaml`，启动主臂并在浏览器中打开控制面板：

```
http://127.0.0.1:5000
```

Web UI 支持：
- 开始 / 停止遥操数据流
- 开始 / 停止数据录制
- 导出或丢弃已录制的 episode

### 启动从臂

```bash
cd RynnRCP/robots/so101
python run_teleop_follower.py
```

读取 `config/teleop_config.yaml` 和 `config/so101_config.yaml`，启动从臂，连接建立后开始接收主臂关节指令并执行。

### 典型操作流程

1. 先启动从臂（或与主臂同时启动）。
2. 启动主臂——Web UI 会自动打开。
3. 等待约 5 秒，直到连接指示器显示**已连接**。
4. 点击 **Start Teleop** 开始向从臂发送主臂关节状态。
5. 点击 **Start Recording** 开始录制 episode；完成后点击 **Stop Recording**。
6. 点击 **Export** 将所有已录制的 episode 打包为 ZIP 文件。
