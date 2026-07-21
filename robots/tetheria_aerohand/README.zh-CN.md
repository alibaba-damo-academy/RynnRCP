# Aero Hand

[English](README.md)

Aero Hand 的 RynnRCP 接入包，支持单手 7 自由度、双手 14 自由度、摄像头手势控制真机进行本地 Teleop 和数采，以及摄像手势控制端通过 RynnBot 控制仿真目标。

## 先判断需要填写哪端的云端凭据

判断方法：**为本次启动 RynnBot App 的真实端填写一套设备凭据。** 仿真目标由仿真环境提供，云端手势控制场景填写摄像手势控制端。

| 使用方式 | 需要填写的 Product Key / Device Name / Device Secret |
| --- | --- |
| 配置页面“开始手势遥操” | 0 套 |
| 本地 Teleop 和本地数采 | 0 套 |
| 真机本体接入 RynnBot | 1 套：真机执行端 |
| 摄像手势控制端通过 RynnBot 控制仿真目标 | 1 套：摄像手势控制端 |

“摄像手势控制端”是读取摄像头、识别手势并输出关节动作的 RCP Server。使用配置页面的“手势摄像头与采集画面”选择摄像头编号。

## 前置条件

- Python 3
- Aero Hand 已连接，串口可访问
- 摄像手势控制端需要可访问的摄像头；首次启动时允许 Python 使用摄像头

## 安装

先进入 Aero Hand 目录，再安装并激活环境：

```bash
cd robots/tetheria_aerohand
bash setup_aero_hand.sh
source venv/bin/activate
rynnrcp-aero-hand-configure
```

安装脚本会同时安装 RynnBot、MCP、Protocol Debug 和 Teleop App。

后续每个终端都先进入 Aero Hand 目录并激活虚拟环境：

```bash
cd robots/tetheria_aerohand
source venv/bin/activate
```

## 启动真机从端

先选择一个真机构型启动 Server。单手：

```bash
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
```

双手：

```bash
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml
```

每个 Server 启动后都会打印 `Debug UI` 地址。需要查看状态、Action、相机图像或实时曲线时，打开该地址；端口被占用时使用 Server 终端打印的新地址。

## Protocol Debug

Protocol Debug 使用指定配置启动或连接对应的真机 Server。

### 单手 Protocol Debug

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
```

### 双手 Protocol Debug

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml
```

## MCP

MCP App 连接已经启动的真机 Server，每种构型使用两个终端。

### 单手 MCP

```bash
# 终端 1
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml

# 终端 2
rynnrcp-mcp-app --server-config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
```

### 双手 MCP

```bash
# 终端 1
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml

# 终端 2
rynnrcp-mcp-app --server-config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml
```

## 摄像手势遥操与数采

先运行 `rynnrcp-aero-hand-configure`，在页面选择单手或双手，再扫描、预览并保存摄像头。这里的摄像头选择同时用于手势识别和 `observation.images.front`。单手模式自动跟随画面中的任意一只手；双手模式同时识别左右手，并固定按 `left 7 + right 7` 输出 14 维状态。

### 本地单手遥操与数采

打开三个终端，并在每个终端中进入 `robots/tetheria_aerohand`、激活同一个虚拟环境。按下面顺序启动：

终端 1，启动单手摄像手势控制端：

```bash
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_hand_master_server.yaml
```

终端 2，启动单手真机从端：

```bash
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
```

终端 3，启动 Teleop：

```bash
rynnrcp-teleop-app
```

在 Teleop 页面绑定：

```text
observation.robot.joint_state -> action.robot.joint_position
```

点击“开始遥操”后即可控制真机，随后可在同一页面开始和停止数采。

### 本地双手遥操与数采

双手同样使用三个终端。终端 1：

```bash
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_hand_master_server.yaml
```

终端 2：

```bash
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml
```

终端 3：

```bash
rynnrcp-teleop-app
```

Teleop 中仍绑定 `observation.robot.joint_state -> action.robot.joint_position`。双手摄像手势控制端会等待左右手都识别成功后发布首个完整状态；某只手短暂离开画面时保持该手最后一个有效姿态。

只有一个摄像头时，摄像手势控制端和真机执行端会共用同机画面，因此两端启动顺序不影响使用。真机执行端继续对外提供 `observation.front.image`。

### 接入 RynnBot

根据本次用途选择一组配置：

1. **真机执行端**：让 Aero Hand 本体接入 RynnBot，上传状态并接收云端动作。
2. **摄像手势控制端**：让摄像手势服务接入 RynnBot，读取摄像头并控制仿真目标。

每套凭据都包含 Product Key、Device Name 和 Device Secret。HTTP URL 通常保持默认，App ID 由配置工具自动生成。本地 Teleop 和本地数采可直接使用本地 Server 配置。

配置页会把凭据写入当前机器的对应 YAML。公开或分享配置文件时保留 `YOUR_*` 占位值。

真机执行端接入 RynnBot 时，选择单手或双手构型，并在两个终端中分别启动对应的 Server 和 RynnBot App。

#### 单手真机执行端接入 RynnBot

```bash
# 终端 1
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml

# 终端 2
rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_single_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
```

#### 双手真机执行端接入 RynnBot

```bash
# 终端 1
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml

# 终端 2
rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_dual_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml
```

#### 单手摄像手势控制端控制仿真目标

使用两个终端。先启动控制端 Server，并等待终端打印 Server 已就绪和 `Debug UI` 地址：

```bash
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_hand_master_server.yaml
```

再启动摄像手势控制端 RynnBot App：

```bash
rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_single_hand_master_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_single_hand_master_server.yaml
```

#### 双手摄像手势控制端控制仿真目标

同样使用两个终端。先启动双手控制端：

```bash
rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_hand_master_server.yaml
```

Server 就绪后再启动：

```bash
rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_dual_hand_master_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_dual_hand_master_server.yaml
```

如果 App 报 `robot_id ... was not found`，说明对应 Server 尚未就绪或启动了另一种构型。确认 Server 终端仍在运行并已打印就绪信息，再重新启动 App。

需要先验证手势控制时，在配置页“真机调试”中点击“开始手势遥操”。页面会显示带手部骨架的实时识别画面；停止后再执行 homing 或调试姿态。

## 能力

- 读取关节状态
- 控制关节位置
- 单手和双手独立配置
- 摄像手势控制端及 RynnBot 云端仿真控制

云端凭据填写在本机配置中。硬件配置、标定和排查方法见
[调试参考](DEBUGGING.zh-CN.md)。

## 配置文件

| 构型 | 真机 Server | 摄像手势控制端 Server | 真机 RynnBot App | 手势控制端 RynnBot App |
| --- | --- | --- | --- | --- |
| 单手 | `aero_hand_single_server.yaml` | `aero_hand_single_hand_master_server.yaml` | `aero_hand_single_rynnbot_app.yaml` | `aero_hand_single_hand_master_rynnbot_app.yaml` |
| 双手 | `aero_hand_dual_server.yaml` | `aero_hand_dual_hand_master_server.yaml` | `aero_hand_dual_rynnbot_app.yaml` | `aero_hand_dual_hand_master_rynnbot_app.yaml` |
