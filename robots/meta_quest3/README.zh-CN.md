# Meta Quest 3 主臂

这个目录把 Meta Quest 3 手柄转换成目标机械臂的关节位置。目标构型由 YAML 和 URDF 决定，控制器、IK 和 Web 仿真代码保持通用。

- 带夹爪单臂：`[机械臂关节, 夹爪]`
- 纯机械臂单臂：`[机械臂关节]`
- 双臂：`[左臂关节, 可选左夹爪, 右臂关节, 可选右夹爪]`
- 带夹爪构型中，`0` 为闭合，`1` 为打开
- 控制频率：60 Hz

[English](README.md)

## 可用构型

### Web 仿真

以下配置可直接传给 `rynnrcp-meta-quest3-sim --config`：

| 目标机械臂 | 配置文件 | 模式 | 控制维度 |
| --- | --- | --- | ---: |
| Franka FR3 | `franka_fr3_right.yaml` | 单右臂 | 8 |
| UR5e | `ur5e_right.yaml` | 单右臂 | 6 |
| Piper | `piper_right.yaml` | 单右臂 | 6 |
| RealMan RM75 | `rm75_right.yaml` | 单右臂 | 7 |
| SO101 Follower | `so101_follower_right.yaml` | 单右臂 | 6 |
| Rizon 4s | `rizon4s_right.yaml` | 单右臂 | 7 |
| ECO65 | `eco65_right.yaml` | 单右臂 | 6 |
| Damiao 6DoF | `dm6dof_right.yaml` | 单右臂 | 6 |
| OpenArm | `openarm_right.yaml` | 单右臂 | 7 |
| 双 Franka FR3 | `franka_fr3_dual.yaml` | 双臂 | 16 |

配置文件都位于：

```text
rynnrcp_robot_meta_quest3/config/sim/
```

所有已列出的构型都会加载随包提供的低精度 STL 模型。它们采用与 FR3
相同的轻量碰撞外形思路，保留各 link 的整体轮廓和连接关系，适合 Web
调试，不用于精细外观展示或碰撞计算。页面根据 URDF 的 `visual`（缺失时使用
`collision`）把每个 STL 挂到对应 link；只有资源缺失时才退回杆状运动学骨架。

每个仿真配置的 `home_joint_positions` 是 Cartesian 调试参考姿态。配置会优先
选择工作空间中部、远离奇异点的姿态，让 XYZ 和旋转滑块能够向正负方向运动。
真实机器人的启动姿态由对应目标机器人配置确定。

### RynnRCP 主臂服务

| 构型 | Server 配置 | 输出 |
| --- | --- | --- |
| Quest 右手柄 → 单 FR3 | `meta_quest3_franka_fr3_right_server.yaml` | 7 个关节 + 1 个夹爪 |
| Quest 双手柄 → 双 FR3 | `meta_quest3_franka_fr3_dual_server.yaml` | 左 7+1 + 右 7+1 |

Server 配置位于 `rynnrcp_robot_meta_quest3/config/`，通过 `rynnrcp-meta-quest3-server --config <配置文件> --source-ip <Quest IP>` 启动。当前单台 FR3 真机使用 `meta_quest3_franka_fr3_right_server.yaml`。

## 快速开始：本地 Web 仿真

仿真服务使用 Python 3.10+ 和浏览器，可运行在 macOS、Linux，以及 Windows WSL。以下 Bash 命令适用于这些环境。

在仓库根目录执行：

```bash
cd robots/meta_quest3
bash setup_meta_quest3.sh
source venv/bin/activate

rynnrcp-meta-quest3-sim \
  --config rynnrcp_robot_meta_quest3/config/sim/franka_fr3_right.yaml
```

浏览器打开 `http://127.0.0.1:8765`。页面根据配置加载 URDF、Home、关节范围、工作空间和运动限制。

接入 Quest 数据时增加 `--quest`：

```bash
rynnrcp-meta-quest3-sim \
  --config rynnrcp_robot_meta_quest3/config/sim/rm75_right.yaml \
  --quest
```

双臂仿真同样由配置决定：

```bash
rynnrcp-meta-quest3-sim \
  --config rynnrcp_robot_meta_quest3/config/sim/franka_fr3_dual.yaml \
  --quest
```

## 使用 Quest 手柄

Franka 右臂和双臂配置监听 UDP `0.0.0.0:8888`。启动主臂 Server 时通过 `--source-ip` 指定 Quest 3 地址，例如 `172.16.1.29`；下次地址变化时直接修改启动参数。

发送端通过 UDP 逐包发送 JSON 对象。单臂模式读取 `rightController`，双臂
模式同时读取左右手柄：

```json
{
  "leftController": {
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    "timestamp": 1.0,
    "input": {
      "primaryButton": false,
      "gripPressed": false,
      "trigger": 0.0
    }
  },
  "rightController": {
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    "timestamp": 1.0,
    "input": {
      "primaryButton": false,
      "gripPressed": false,
      "trigger": 0.0
    }
  }
}
```

`position` 和 `rotation` 是发送端提供的手柄绝对位姿，`rotation` 使用 xyzw
四元数。`gripPressed` 是运动离合器，`trigger` 范围为 `[0, 1]`，
`primaryButton` 用于标定。`timestamp` 可选；填写时，每个手柄应保持递增。

### 单臂

1. 将右手柄保持在准备开始遥操的中立位置。
2. 按一次 A。
3. 按住右侧 Grip 才发布关节目标并移动机械臂；松开后停止发布，机械臂保持当前位置。
4. 使用 trigger 控制夹爪。
5. 再次按下标定按钮会平滑回到配置中的 Home，并重新标定。

### 双臂

1. 双手保持在各自的中立位置。
2. 同时按下 X 和 A。
3. 左右 Grip 分别控制对应机械臂；至少按住一侧时才发布合并向量，未按住的一侧保持最后目标。
4. 左右 trigger 分别控制对应夹爪。

双臂对外是一条向量，每个已配置的夹爪紧跟在对应机械臂关节之后：

```text
[
  left joint positions,
  optional left gripper,
  right joint positions,
  optional right gripper
]
```

## 坐标和标定

手柄移动先转换到统一的 `rcp_cartesian_v1` 坐标：

```text
+X：向前
+Y：向左
+Z：向上
位置单位：米
姿态：xyzw 四元数
```

标定记录当前手柄位姿。之后使用“当前手柄位姿相对标定位姿的变化量”，叠加到配置中的机器人 Home 末端位姿，再通过 URDF IK 得到绝对关节目标。

首次接入新的 Quest 发送程序时，依次向前、向左、向上移动手柄，确认 X、Y、Z 分别增加。方向映射由以下配置控制：

- `coordinate_basis`：平移坐标映射
- `rotation_basis`：旋转坐标映射
- `rotation_component_signs`：绕标准 X、Y、Z 轴的旋转方向
- `translation_scale`：手柄位移缩放

## 控制真实 FR3

在连接 FR3 的 Ubuntu 主机执行：

```bash
cd robots/franka_fr3
bash setup_franka_fr3.sh
source venv/bin/activate
```

这个安装脚本把 Franka、Meta Quest 3、RynnRCP Server 和 Teleop 安装到同一个 `robots/franka_fr3/venv`。

分别启动三个进程：

```bash
# 终端 1：真实 FR3
rynnrcp-server \
  --config rynnrcp_robot_franka_fr3/config/franka_fr3_server.yaml

# 终端 2：Quest 右手柄关节目标
rynnrcp-meta-quest3-server \
  --config rynnrcp_robot_meta_quest3/config/meta_quest3_franka_fr3_right_server.yaml \
  --source-ip 172.16.1.29

# 终端 3：遥操和采集
rynnrcp-teleop-app
```

在 Teleop 中选择：

- 主臂：`meta_quest3_franka_fr3_right`
- 从臂：`franka_fr3`
- 映射：`observation.robot.joint_state` → `action.robot.joint_position`

启动前清空机械臂工作空间并保持急停可用。右手柄按 A 标定后，按住右侧 Grip 才会向 FR3 发布 8 维关节目标；松开 Grip 会停止发布，FR3 在目标流超时后平滑停止并保持当前位置。再次按住时，当前手柄位姿会成为新的离合参考，不会沿用松开期间的手柄位移。

FR3 单臂数据固定为：

```text
[joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7, gripper]
```

Home、工作空间和速度限制位于 `meta_quest3_franka_fr3_right_server.yaml`。当前工作空间相对 Home 为：

- X：`[-0.18, 0.40] m`
- Y：`[-0.50, 0.50] m`
- Z：`[-0.50, 0.00] m`

## 增加构型

### 只增加 Web 仿真

1. 把 URDF 放入 `rynnrcp_robot_meta_quest3/models/`。
2. 把 URDF 实际引用的 STL 放入
   `rynnrcp_robot_meta_quest3/models/assets/<target_model>/`。文件名匹配不区分
   大小写；Web 仿真优先读取 `visual`，没有可用 STL 时读取 `collision`。
3. 复制一个 `config/sim/*_right.yaml`。
4. 修改 `target_urdf`、`target_dof`、`base_link`、`tip_link` 和 `home_joint_positions`。
5. 使用 `rynnrcp-meta-quest3-sim --config <配置文件>` 验证。

单臂控制维度为 `target_dof + int(has_gripper)`。

双臂配置在 `components.robot.arms.left` 和 `right` 中分别描述两条运动链。两侧可以使用不同的 URDF 和关节数，总维度为：

```text
left target_dof + int(left has_gripper)
+ right target_dof + int(right has_gripper)
```

### 增加 RynnRCP 主臂服务

单臂复制 `meta_quest3_franka_fr3_right_server.yaml`，双臂复制 `meta_quest3_franka_fr3_dual_server.yaml`。设置：

- `manifest.robot_id`、`manifest.robot_name`
- `target_model`、`target_urdf`
- `target_dof`、`control_dof`
- `base_link`、`tip_link`
- `home_joint_positions`
- `workspace_delta_limits_m`
- 坐标映射和运动限制

单臂共用 `MetaQuest3UrdfJointController`，双臂共用 `MetaQuest3DualUrdfJointController`。两个 Integration 文件都会从选中的 Server 配置读取 `control_dof` 和 `joint_order`。接入新的真实机械臂时，新增一份 Server YAML，按每条机械臂的关节数和夹爪填写控制维度；共享的 Integration 和控制代码会按该配置发布数据。

Grip 逻辑位于这两个通用控制类中。没有按下 Grip 时，主臂 observation 会等待新的手柄数据，不会生成空值或新的通道样本，因此 Teleop 没有 action 可以转发。接入新构型时复用相同控制类并增加配置文件即可获得一致的离合行为。

## 主要配置字段

| 字段 | 用途 |
| --- | --- |
| `target_urdf` | 目标机械臂 URDF |
| `target_dof` | base-to-tip 活动关节数量 |
| `control_dof` | 对外发布的向量维度，包含所有启用的夹爪 |
| `joint_order` | 对外发布的关节和夹爪顺序 |
| `has_gripper` | 控制向量和 Web 界面是否包含夹爪 |
| `home_includes_gripper` | 将 Home 最后一维作为归一化夹爪 Home |
| `native_gripper_joint` | 直接驱动指定的 URDF 夹爪关节，不叠加通用夹爪模型 |
| `joint_names` | 可选的对外机械臂关节名称 |
| `control_dof` | 根据关节数和 `has_gripper` 得到的对外数据维度 |
| `base_link` / `tip_link` | IK 运动链起点和终点 |
| `home_joint_positions` | 机械臂 Home 关节；启用 `has_gripper` 时最后一维为夹爪 Home |
| `workspace_delta_limits_m` | 末端相对 Home 的 XYZ 范围 |
| `max_joint_velocity_rad_s` | 关节目标最大速度 |
| `max_joint_acceleration_rad_s2` | 关节目标最大加速度 |
| `max_target_translation_m_s` | 末端平移跟踪速度 |
| `max_target_rotation_rad_s` | 末端旋转跟踪速度 |

## 状态与排障

- `meta_quest3.waiting`：检查 Quest 目标地址、UDP 端口和启动参数 `--source-ip`。
- `meta_quest3.calibration_required`：单臂按 A，双臂同时按 X+A。
- `meta_quest3.stale`：检查无线网络和 Quest 发送程序。
- `meta_quest3.ik_not_converged`：将手柄移回上一个可达姿态，并减小移动或旋转幅度。
- `meta_quest3.left_ik_not_converged` / `right_ik_not_converged`：对应一侧 IK 未收敛。
- `controller_state.calibration_count`：标定成功次数。
- `controller_state.packet_age_s`：最近 UDP 数据包的时间。
