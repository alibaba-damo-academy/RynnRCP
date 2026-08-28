# Sim Robot

[English](README.md)

RynnRCP 的 Isaac Sim 接入包，通过 ZeroMQ 支持 SO101 单臂/双臂、RM75、Franka R3、Aero Hand 单手/双手等仿真构型。

## 前置条件

- Isaac Sim 环境可用
- 仿真侧 ZeroMQ 服务已启动

## 安装

先进入 Sim Robot 目录，再安装并激活环境：

```bash
cd robots/sim_robot
bash setup_sim.sh
source venv/bin/activate
```

安装脚本会安装 RynnBot、MCP、Protocol Debug 和 Teleop App。

## 启动

```bash
bash start_rcp.sh --config so101
bash start_rcp.sh --config lerobot_so101_dual_sim_v1
bash start_rcp.sh --config lerobot_so101_dual_sim_v2
bash start_rcp.sh --config aero_hand_sim_v1
bash start_rcp.sh --config aero_hand_dual_sim_v1
bash start_rcp.sh --config aero_hand_dual_sim_v2
bash start_rcp.sh --config rm75_rmg24_sim_v1
bash start_rcp.sh --config rm75_rmg24_sim_v2
bash start_rcp.sh --config franka_r3_sim_v1
bash start_rcp.sh --config franka_r3_sim_v2
```

仅保留以上 10 个 server 配置：9 个精确构型 + 原始 `so101` 构型。Server 启动后，可按需启动 App：

```bash
rynnrcp-protocol-debug --config rynnrcp_robot_sim/config/sim_server_so101.yaml
rynnrcp-mcp-app --server-config rynnrcp_robot_sim/config/sim_server_so101.yaml
rynnrcp-rynnbot-app --config rynnrcp_robot_sim/config/sim_rynnbot_app.yaml --server-config rynnrcp_robot_sim/config/sim_server_so101.yaml
rynnrcp-teleop-app
```

每个 Server 启动后都会打印 `Debug UI` 地址。需要查看状态、Action、相机图像或实时曲线时，手动在浏览器打开该地址；端口被占用时，使用对应 Server 终端打印的新地址。

## 能力

- `aero_hand_sim_v1`：7DoF 单手 compact 向量 + front 1 相机，action/state 字段名为 thumb/index/middle/ring/pinky compact joints
- `aero_hand_dual_sim_v1`：14DoF compact 合并向量 + front 1 相机，action/state 字段名为 left_* + right_* tendon/actuator
- `aero_hand_dual_sim_v2`：14DoF compact 合并向量 + front/top/left/right 4 相机，action/state 不拆左右 component
- `rm75_rmg24_sim_v1` / `rm75_rmg24_sim_v2`：8DoF + 5 相机
- `franka_r3_sim_v1` / `franka_r3_sim_v2`：9DoF + 5 相机
- `lerobot_so101_dual_sim_v1` / `lerobot_so101_dual_sim_v2`：12DoF 合并向量（左6+右6），RCP/乐云侧只有一个 `robot` component；底层内部才连接 `left_robot`/`right_robot`
- `so101`：原始 SO101 单臂构型，6DoF + front/top/left/right/wrist 5 相机
- 仿真关节状态与控制
- 多相机图像
- 多机器人构型配置

ZeroMQ 数据流、云端配置和新增构型方法见[调试参考](DEBUGGING.zh-CN.md)。
