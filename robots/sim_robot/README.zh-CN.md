# Sim Robot (Isaac Sim)

RynnRCP 通用仿真机器人集成包，通过 ZeroMQ 连接 Isaac Sim。
支持多种机器人构型（SO101、RM75 等），通过配置文件区分。

## 安装

```bash
cd robots/sim_robot
bash setup_sim.sh
```

## 启动

```bash
# SO101 (6 DOF, 2 cameras)
bash start_rcp.sh --config so101

# RM75 (9 DOF, 5 cameras)
bash start_rcp.sh --config rm75
```

前置条件：仿真已在 Docker 容器内运行。

## 配置

### 云平台凭据

编辑 `rynnrcp_robot_sim/config/sim_rynnbot_app.yaml`：

```yaml
app:
  product_key: <your_product_key>
  device_name: <your_device_name>
  device_secret: <your_device_secret>
```

支持环境变量覆盖：
```bash
RYNNBOT_PRODUCT_KEY=xxx RYNNBOT_DEVICE_NAME=yyy RYNNBOT_DEVICE_SECRET=zzz bash start_rcp.sh --config so101
```

完整环境变量列表：

| 环境变量 | 说明 | 默认值 |
|---------|------|--------|
| `RYNNBOT_PRODUCT_KEY` | 云平台 product_key | yaml 中的值 |
| `RYNNBOT_DEVICE_NAME` | 云平台 device_name | yaml 中的值 |
| `RYNNBOT_DEVICE_SECRET` | 云平台 device_secret | yaml 中的值 |
| `RYNNBOT_HTTP_URL` | 云平台接入地址 | https://robot-access.damo-academy.com |
| `PORT` | 仿真基础端口 | 8080 |

### 添加新机器人构型

1. 创建 `config/robot_integration_<name>.yaml`（定义相机列表和 DOF）
2. 创建 `config/sim_server_<name>.yaml`（引用 integration 并设置 components）
3. 启动：`bash start_rcp.sh --config <name>`

## 数据流

```
键盘/云端 → WSS tunnel → RynnBot App → UDP:8085 → 仿真 IK
  → action_bridge 轮询 get_joint_command
    → gRPC run_action_chunk → RCP Server (数据采集)
      → ZMQ set_joint_positions → 仿真机器人运动
```

## 文件说明

```
robots/sim_robot/
├── setup_sim.sh
├── start_rcp.sh                    # bash start_rcp.sh --config <robot>
├── action_bridge.py
├── pyproject.toml
├── README.zh-CN.md
└── rynnrcp_robot_sim/
    ├── __init__.py
    ├── controller.py               # SimRobotController(n_dof=N)
    ├── sim_camera.py               # SimCamera (JPEG)
    ├── zmq_clients.py              # JointClient + FrameClient
    └── config/
        ├── robot_integration_so101.yaml  # SO101: 6DOF, front+wrist
        ├── robot_integration_rm75.yaml   # RM75: 9DOF, 5 cameras
        ├── sim_server_so101.yaml
        ├── sim_server_rm75.yaml
        └── sim_rynnbot_app.yaml
```
