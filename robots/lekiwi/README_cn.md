# Lekiwi接入乐云平台最小接入示例
本目录提供 lekiwi 机械臂接入 RynnRCP 的最小可运行工程，包括：

- 配置文件：
  - config/lekiwi_config.yaml：RcpCore 配置（Action/Sensor/DeviceMonitor 的 inputs/outputs）
  - config/rynnbot_config.yaml：乐云平台设备信息（三元组/URL）
  - config/mcp_config.yaml：MCP server 配置
- 端侧启动示例：
  - run_rcp_lekiwi.py：启动 RcpCore + RynnBot + McpPlugin
    - RynnBot：连接乐云平台（MQTT/WS）
    - McpPlugin：启动 MCP HTTP 服务，对外暴露 tools 调用入口
- 配置向导脚本：
  - configure_lekiwi.py：交互式配置（支持中文/英文），可自动完成相机、串口、标定及平台三元组填写以及机械臂标定

## 目录结构

```text
robots/lekiwi/
├── config/
│   ├── rynnbot_config.yaml     # 乐云设备三元组/URL
│   └── lekiwi_config.yaml       # RcpCore 配置：Action/Sensor/DeviceMonitor
│   └── mcp_config.yaml         # MCP Server 配置
├── configure_lekiwi.py          # 交互式配置脚本（中/英文）
├── run_rcp_lekiwi.py            # 启动示例：RcpCore + RynnBot(plugin)
├── README.md                   # 使用文档（英文）
└── README_cn.md                # 使用文档（中文）
```


## 前置条件

1. 已完成项目安装（Python 环境就绪，依赖已安装，Python环境已激活）
2. 已连接 lekiwi 机械臂控制器（串口方式）
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
cd RynnRCP/robots/lekiwi
python configure_lekiwi.py
```
脚本启动后会先让你选择语言（中文/英文），然后进入菜单：
1. 设备设置：填写乐云平台信息，保存到 config/rynnbot_config.yaml
2. 相机设置：通过插拔检测相机，写入 config/lekiwi_config.yaml
3. 机器人串口设置：通过插拔检测串口，并修改串口权限，同时写入 rcp_motion 的底层配置文件
4. 机械臂标定：执行机械臂标定程序，写入标定配置文件
5. 一键配置全部：按 1→2→3→4 顺序执行

> 标定方法参考：[lekiwi 标定方法](../../docs/lekiwi_calibrate.md)


## 启动端侧服务

完成配置后，在本目录启动：

```bash
python run_rcp_lekiwi.py
```
该脚本会读取：
- ./config/lekiwi_config.yaml
- ./config/rynnbot_config.yaml
- ./config/mcp_config.yaml

并启动端侧服务节点，连接乐云平台（RynnBot）, 同时启动 MCP 服务（McpPlugin）。
